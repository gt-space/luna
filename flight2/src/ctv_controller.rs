//! CTV time-scheduled LQR controller.
//!
//! Rust port of the CTV tuning variant merged from `CTV-Controller-Tuning`
//! (`CONTROLLER = "lqr_ctv_tuning"`): `subsystems/gnc/LQR_CTV_Tuning.slx`
//! reading the tables designed by `src/gnc/ctvTuningGains.m`.
//!
//! The scheduling law is:
//!
//!     u = unom - K1 * dx - K2
//!
//! # Reference trajectory CSV
//!
//! Full form:
//!
//! ```text
//! t,x,y,z,vx,vy,vz,p,q,r,q0,q1,q2,q3,thrust,tvc_pitch,tvc_yaw,rcs
//! ```
//!
//! Position and velocity are NED (z positive down), body rates in rad/s,
//! quaternion scalar-first, thrust in N, TVC angles in rad and roll torque
//! in N m. Every reference quaternion must have a nonzero norm; a table that
//! does not is rejected with an error rather than panicking.
//!
//! # Gain CSV files
//!
//! Headerless, one row per reference time:
//!
//! - k1flat: 52 values per row, four input rows by 13 states.
//! - k2grid: 4 values per row.
//!
//! Do not negate k2grid on input. The CTV scheduling law subtracts K2.

use std::{fs, io, path::Path};

use nalgebra::{
    Const, Dyn, Matrix, MatrixXx1, MatrixXx4, OMatrix, SMatrix, VecStorage,
};
use serde::{Deserialize, Serialize};

use crate::control::Controller;
use common::comm::ctv::{ControlState, ControlVector};

const QUATERNION_EPSILON: f64 = 1.0e-12;

/// 13-state reference table:
/// position (3), velocity (3), body rates (3), quaternion (4).
pub type CtvXRef =
    Matrix<f64, Dyn, Const<13>, VecStorage<f64, Dyn, Const<13>>>;

/// 52-column flattened K1 table.
pub type CtvK1Flat =
    Matrix<f64, Dyn, Const<52>, VecStorage<f64, Dyn, Const<52>>>;

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct CtvLqrController {
    n: usize,
    tgrid: MatrixXx1<f64>,
    xref: CtvXRef,
    k1flat: CtvK1Flat,
    k2grid: MatrixXx4<f64>,
    unom: MatrixXx4<f64>,
}

impl CtvLqrController {
    pub fn new(
        tgrid: MatrixXx1<f64>,
        mut xref: CtvXRef,
        k1flat: CtvK1Flat,
        k2grid: MatrixXx4<f64>,
        unom: MatrixXx4<f64>,
    ) -> Self {
        let n = tgrid.nrows();

        assert!(
            n >= 2,
            "the gain schedule needs at least two grid points"
        );

        for i in 0..n {
            assert!(
                tgrid[i].is_finite(),
                "tgrid must contain only finite values"
            );
        }

        for i in 1..n {
            assert!(
                tgrid[i] > tgrid[i - 1],
                "tgrid must be strictly increasing"
            );
        }

        assert_eq!(n, xref.nrows(), "inconsistent row dimension");
        assert_eq!(n, k1flat.nrows(), "inconsistent row dimension");
        assert_eq!(n, k2grid.nrows(), "inconsistent row dimension");
        assert_eq!(n, unom.nrows(), "inconsistent row dimension");

        assert!(
            xref.iter().all(|v| v.is_finite()),
            "xref must contain only finite values"
        );

        assert!(
            k1flat.iter().all(|v| v.is_finite()),
            "k1flat must contain only finite values"
        );

        assert!(
            k2grid.iter().all(|v| v.is_finite()),
            "k2grid must contain only finite values"
        );

        assert!(
            unom.iter().all(|v| v.is_finite()),
            "unom must contain only finite values"
        );

        // Normalize reference quaternions and keep consecutive samples
        // on the same quaternion hemisphere.
        //
        // q and -q represent the same physical attitude. Aligning their
        // signs prevents interpolation from passing through the zero vector.
        for i in 0..n {
            let mut q = nalgebra::Vector4::new(
                xref[(i, 9)],
                xref[(i, 10)],
                xref[(i, 11)],
                xref[(i, 12)],
            );

            let norm = q.norm();

            assert!(
                norm.is_finite() && norm > QUATERNION_EPSILON,
                "reference quaternion must have a nonzero finite norm"
            );

            q /= norm;

            if i > 0 {
                let previous = nalgebra::Vector4::new(
                    xref[(i - 1, 9)],
                    xref[(i - 1, 10)],
                    xref[(i - 1, 11)],
                    xref[(i - 1, 12)],
                );

                if q.dot(&previous) < 0.0 {
                    q *= -1.0;
                }
            }

            for j in 0..4 {
                xref[(i, 9 + j)] = q[j];
            }
        }

        CtvLqrController {
            n,
            tgrid,
            xref,
            k1flat,
            k2grid,
            unom,
        }
    }

    pub fn from_trajectory_csv(
        trajectory: impl AsRef<Path>,
        k1flat: impl AsRef<Path>,
        k2grid: impl AsRef<Path>,
    ) -> io::Result<Self> {
        let trajectory = trajectory.as_ref();
        let name = trajectory.display().to_string();

        let (tgrid, xref, unom) =
            parse_trajectory(&read_text(trajectory)?, &name)?;

        Self::with_gain_files(
            tgrid,
            xref,
            unom,
            k1flat.as_ref(),
            k2grid.as_ref(),
        )
    }

    /// Loads the gain set that the hopper repo's `export_plant_gains.m`
    /// writes into `dir` (`tgrid`, `xref`, `k1flat`, `k2grid` and `unom`, as
    /// headerless `.csv` files), after `gain_manifest::verify` has checked
    /// `gains.json` against `policy`: the controller name, the K1 layout, the
    /// files' SHA-256 hashes and the hopper commit.
    ///
    /// Prefer this over `from_trajectory_csv` for anything that flies: the
    /// loose files of the other loaders say nothing about what they are.
    pub fn from_gain_dir(
        dir: impl AsRef<Path>,
        policy: &crate::gain_manifest::Policy,
    ) -> io::Result<Self> {
        let dir = dir.as_ref();

        crate::gain_manifest::verify(dir, policy).map_err(invalid)?;

        let table = |name: &str| dir.join(format!("{name}.csv"));

        let tgrid = read_matrix::<1>(&table("tgrid"))?;
        let xref = read_matrix::<13>(&table("xref"))?;
        let k1flat = read_matrix::<52>(&table("k1flat"))?;
        let k2grid = read_matrix::<4>(&table("k2grid"))?;
        let unom = read_matrix::<4>(&table("unom"))?;

        let n = tgrid.nrows();

        for (name, rows) in [
            ("xref", xref.nrows()),
            ("k1flat", k1flat.nrows()),
            ("k2grid", k2grid.nrows()),
            ("unom", unom.nrows()),
        ] {
            if rows != n {
                return Err(invalid(format!(
                    "{}: {rows} rows for a {n}-row tgrid",
                    table(name).display()
                )));
            }
        }

        check_time_grid(&tgrid, &table("tgrid").display().to_string())?;
        check_quaternions(&xref, &table("xref").display().to_string())?;

        Ok(Self::new(tgrid, xref, k1flat, k2grid, unom))
    }

    fn with_gain_files(
        tgrid: MatrixXx1<f64>,
        xref: CtvXRef,
        unom: MatrixXx4<f64>,
        k1flat_path: &Path,
        k2grid_path: &Path,
    ) -> io::Result<Self> {
        let n = tgrid.nrows();

        let k1flat = read_matrix::<52>(k1flat_path)?;
        let k2grid = read_matrix::<4>(k2grid_path)?;

        if k1flat.nrows() != n {
            return Err(invalid(format!(
                "{}: {} rows for a {n}-row trajectory",
                k1flat_path.display(),
                k1flat.nrows()
            )));
        }

        if k2grid.nrows() != n {
            return Err(invalid(format!(
                "{}: {} rows for a {n}-row trajectory",
                k2grid_path.display(),
                k2grid.nrows()
            )));
        }

        Ok(Self::new(tgrid, xref, k1flat, k2grid, unom))
    }

    /// The last time of the schedule, s.
    pub fn schedule_end(&self) -> f64 {
        self.tgrid[self.n - 1]
    }

    /// The reference state at schedule time `t`: position NED (3), velocity
    /// NED (3), body rates (3) and the interpolated, normalized attitude (4).
    /// Before the schedule it is the first row and after it the last.
    pub fn reference_state(&self, t: f64) -> SMatrix<f64, 13, 1> {
        let (idx, w) = self.interval(t);

        let mut x = (1.0 - w) * self.xref.row(idx).transpose()
            + w * self.xref.row(idx + 1).transpose();

        let norm = x.rows_range(9..13).norm();

        if norm > QUATERNION_EPSILON {
            let q = x.rows_range(9..13).clone_owned() / norm;

            x.rows_range_mut(9..13).copy_from(&q);
        }

        SMatrix::<f64, 13, 1>::from_iterator(x.iter().copied())
    }

    /// For an abort to land: the schedule time on the descending branch at
    /// which the reference is `altitude_m` above the pad, or the time of the
    /// apex if the vehicle is higher than the schedule ever goes. Taking the
    /// reference from there keeps it continuous in height with the vehicle.
    ///
    /// `None` if the schedule never leaves the pad (no apex to descend from).
    pub fn descent_time_at_altitude(&self, altitude_m: f64) -> Option<f64> {
        // Altitude is -z (NED down). The apex is the highest row.
        let altitude = |i: usize| -self.xref[(i, 2)];

        let apex = (0..self.n).max_by(|&a, &b| {
            altitude(a)
                .partial_cmp(&altitude(b))
                .unwrap_or(std::cmp::Ordering::Equal)
        })?;

        if altitude(apex) <= 0.0 {
            return None;
        }

        // Take the last row of the apex plateau, so a hover is not re-flown
        let mut last_top = apex;

        while last_top + 1 < self.n && altitude(last_top + 1) >= altitude(apex) - 1e-9 {
            last_top += 1;
        }

        if altitude_m >= altitude(last_top) {
            return Some(self.tgrid[last_top]);
        }

        for i in last_top + 1..self.n {
            if altitude(i) <= altitude_m {
                let (a0, a1) = (altitude(i - 1), altitude(i));
                let w = if (a0 - a1).abs() < 1e-12 {
                    1.0
                } else {
                    (a0 - altitude_m) / (a0 - a1)
                };

                return Some(self.tgrid[i - 1] + w * (self.tgrid[i] - self.tgrid[i - 1]));
            }
        }

        Some(self.schedule_end())
    }

    /// Returns the 0-based interpolation interval and weight.
    ///
    /// The schedule holds its first row before the trajectory and its
    /// last row after the trajectory.
    fn interval(&self, t: f64) -> (usize, f64) {
        if t <= self.tgrid[0] {
            return (0, 0.0);
        }

        if t >= self.tgrid[self.n - 1] {
            return (self.n - 2, 1.0);
        }

        let (mut lo, mut hi) = (0usize, self.n - 1);

        while hi - lo > 1 {
            let mid = (lo + hi) / 2;

            if self.tgrid[mid] <= t {
                lo = mid;
            } else {
                hi = mid;
            }
        }

        let w =
            (t - self.tgrid[lo]) / (self.tgrid[lo + 1] - self.tgrid[lo]);

        (lo, w)
    }
}

/// Why a control step could not produce a command.
///
/// These are runtime faults, so they are reported to the caller instead of
/// panicking inside the control loop. The caller decides what to do, which
/// should be to take the abort path.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum StepError {
    /// A position, velocity, rate or attitude component is NaN or infinite.
    NonFiniteState,
    /// The attitude quaternion has no usable norm.
    InvalidAttitude,
    /// The interpolated reference quaternion has no usable norm.
    InvalidReference,
}

impl std::fmt::Display for StepError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(match self {
            StepError::NonFiniteState => {
                "runtime state contains non-finite values"
            }
            StepError::InvalidAttitude => {
                "runtime attitude quaternion is invalid"
            }
            StepError::InvalidReference => {
                "interpolated reference quaternion is invalid"
            }
        })
    }
}

impl std::error::Error for StepError {}

/// What `Controller::step` returns when `try_step` fails.
///
/// Every channel is NaN on purpose. The engine-controller link rejects any
/// non-finite demand, the FC stops resending, and the EC safes itself when
/// its demand timeout expires. A zero or held command would instead look like
/// a valid demand. Callers that can handle the error should use `try_step`.
pub const INVALID_COMMAND: ControlVector = ControlVector {
    thrust: f64::NAN,
    tvc_pitch: f64::NAN,
    tvc_yaw: f64::NAN,
    rcs_torque: f64::NAN,
};

impl CtvLqrController {
    /// One control step, reporting a bad state as an error.
    pub fn try_step(
        &self,
        state: ControlState,
    ) -> Result<ControlVector, StepError> {
        let x = state.to_matrix();

        if !x.iter().all(|v| v.is_finite()) {
            return Err(StepError::NonFiniteState);
        }

        let (idx, w) = self.interval(state.time.as_secs_f64());

        // Interpolate the reference state.
        let mut xref_t =
            (1.0 - w) * self.xref.row(idx).transpose()
            + w * self.xref.row(idx + 1).transpose();

        // Interpolate the flattened K1 table.
        let kflat =
            (1.0 - w) * self.k1flat.row(idx)
            + w * self.k1flat.row(idx + 1);

        // Explicitly reconstruct K1 as a 4x13 matrix.
        //
        // Flattening convention:
        // flat[input * 13 + state] = K1(input, state)
        //
        // This matches MATLAB's:
        // reshape(K1(:,:,i).', 1, 52)
        let mut flat_values = [0.0_f64; 52];

        for i in 0..52 {
            flat_values[i] = kflat[i];
        }

        let k1 = unpack_k1_row(&flat_values);

        // Interpolate K2 and nominal control.
        let k2 =
            (1.0 - w) * self.k2grid.row(idx).transpose()
            + w * self.k2grid.row(idx + 1).transpose();

        let unom =
            (1.0 - w) * self.unom.row(idx).transpose()
            + w * self.unom.row(idx + 1).transpose();

        // Normalize the interpolated reference quaternion.
        let mut qref = xref_t.rows_range(9..13).clone_owned();

        let qref_norm = qref.norm();

        if !(qref_norm.is_finite() && qref_norm > QUATERNION_EPSILON) {
            return Err(StepError::InvalidReference);
        }

        qref /= qref_norm;

        xref_t.rows_range_mut(9..13).copy_from(&qref);

        // Normalize the actual quaternion.
        let mut q = x.rows_range(9..13).clone_owned();

        let q_norm = q.norm();

        if !(q_norm.is_finite() && q_norm > QUATERNION_EPSILON) {
            return Err(StepError::InvalidAttitude);
        }

        q /= q_norm;

        // Ensure both quaternions use the same hemisphere.
        if q.dot(&qref) < 0.0 {
            q *= -1.0;
        }

        let mut x_used = x.clone_owned();
        x_used.rows_range_mut(9..13).copy_from(&q);

        // Tracking error.
        let dx = x_used - xref_t;

        // CTV scheduling law:
        // u = unom - K1 * dx - K2
        let u = unom - k1 * dx - k2;

        Ok(ControlVector::from_matrix(u))
    }
}

impl Controller for CtvLqrController {
    /// Returns `INVALID_COMMAND` if the state cannot be used. See `try_step`.
    fn step(&mut self, state: ControlState) -> ControlVector {
        self.try_step(state).unwrap_or(INVALID_COMMAND)
    }
}

/// Reconstructs a 4x13 K1 matrix from input-major flattened data.
///
/// Each input contributes 13 consecutive state gains.
fn unpack_k1_row(flat: &[f64; 52]) -> SMatrix<f64, 4, 13> {
    let mut k1 = SMatrix::<f64, 4, 13>::zeros();

    for input in 0..4 {
        for state in 0..13 {
            k1[(input, state)] = flat[input * 13 + state];
        }
    }

    k1
}

const TRAJECTORY_COLUMNS: [&str; 18] = [
    "t",
    "x",
    "y",
    "z",
    "vx",
    "vy",
    "vz",
    "p",
    "q",
    "r",
    "q0",
    "q1",
    "q2",
    "q3",
    "thrust",
    "tvc_pitch",
    "tvc_yaw",
    "rcs",
];

struct Table {
    name: String,
    header: Vec<String>,
    rows: Vec<Vec<f64>>,
}

impl Table {
    fn parse(text: &str, name: &str) -> io::Result<Self> {
        let mut lines = text
            .lines()
            .enumerate()
            .filter(|(_, line)| !line.trim().is_empty());

        let (header_line, header_text) = lines
            .next()
            .ok_or_else(|| invalid(format!("{name}: empty file")))?;

        let header: Vec<String> = header_text
            .trim_start_matches('\u{feff}')
            .trim()
            .split(',')
            .map(|column| column.trim().to_ascii_lowercase())
            .collect();

        if let Some((column, _)) =
            header.iter().enumerate().find(|(_, name)| name.is_empty())
        {
            return Err(invalid(format!(
                "{name} line {}: empty column name (column {})",
                header_line + 1,
                column + 1
            )));
        }

        for (column, column_name) in header.iter().enumerate() {
            if header[..column]
                .iter()
                .any(|previous| previous == column_name)
            {
                return Err(invalid(format!(
                    "{name} line {}: duplicate column {column_name:?}",
                    header_line + 1
                )));
            }
        }

        let mut rows = Vec::new();

        for (line, text) in lines {
            let fields: Vec<&str> = text.split(',').collect();

            if fields.len() != header.len() {
                return Err(invalid(format!(
                    "{name} line {}: expected {} values, got {}",
                    line + 1,
                    header.len(),
                    fields.len()
                )));
            }

            let mut row = Vec::with_capacity(fields.len());

            for field in fields {
                row.push(parse_number(field, name, line + 1)?);
            }

            rows.push(row);
        }

        Ok(Table {
            name: name.to_string(),
            header,
            rows,
        })
    }

    fn column(&self, name: &str) -> io::Result<usize> {
        self.header
            .iter()
            .position(|column| column.as_str() == name)
            .ok_or_else(|| {
                invalid(format!("{}: missing column {name:?}", self.name))
            })
    }
}

fn parse_trajectory(
    text: &str,
    name: &str,
) -> io::Result<(MatrixXx1<f64>, CtvXRef, MatrixXx4<f64>)> {
    let table = Table::parse(text, name)?;

    let mut column = [0usize; TRAJECTORY_COLUMNS.len()];

    for (k, name) in TRAJECTORY_COLUMNS.iter().enumerate() {
        column[k] = table.column(*name)?;
    }

    let n = table.rows.len();

    if n < 2 {
        return Err(invalid(format!(
            "{}: at least two rows are needed",
            table.name
        )));
    }

    let mut tgrid = MatrixXx1::zeros(n);
    let mut xref = CtvXRef::zeros(n);
    let mut unom = MatrixXx4::zeros(n);

    for (i, row) in table.rows.iter().enumerate() {
        tgrid[i] = row[column[0]];

        for state in 0..13 {
            xref[(i, state)] = row[column[1 + state]];
        }

        for input in 0..4 {
            unom[(i, input)] = row[column[14 + input]];
        }
    }

    check_time_grid(&tgrid, &table.name)?;
    check_quaternions(&xref, &table.name)?;

    Ok((tgrid, xref, unom))
}

fn check_time_grid(
    tgrid: &MatrixXx1<f64>,
    name: &str,
) -> io::Result<()> {
    for i in 0..tgrid.nrows() {
        if !tgrid[i].is_finite() {
            return Err(invalid(format!(
                "{name}: time at row {} is not finite",
                i + 1
            )));
        }

        if i > 0 && tgrid[i] <= tgrid[i - 1] {
            return Err(invalid(format!(
                "{name}: t must be strictly increasing \
                 (row {}: {} then {})",
                i + 1,
                tgrid[i - 1],
                tgrid[i]
            )));
        }
    }

    Ok(())
}

/// Rejects reference attitudes that could not be normalized.
///
/// `new` asserts on a degenerate or non-finite quaternion, but a CSV file is
/// untrusted input, so tables read from disk are checked here and reported as
/// errors instead of panicking.
fn check_quaternions(xref: &CtvXRef, name: &str) -> io::Result<()> {
    for i in 0..xref.nrows() {
        let mut norm_squared = 0.0;

        for j in 9..13 {
            let value = xref[(i, j)];

            norm_squared += value * value;
        }

        let norm = norm_squared.sqrt();

        if !norm.is_finite() || norm <= QUATERNION_EPSILON {
            return Err(invalid(format!(
                "{name}: reference quaternion at row {} has norm {norm}, \
                 which cannot be normalized",
                i + 1
            )));
        }
    }

    Ok(())
}

fn parse_matrix(
    text: &str,
    name: &str,
    columns: usize,
) -> io::Result<(usize, Vec<f64>)> {
    let mut data = Vec::new();
    let mut rows = 0;

    for (line, text) in text
        .lines()
        .enumerate()
        .filter(|(_, line)| !line.trim().is_empty())
    {
        let fields: Vec<&str> = text.split(',').collect();

        if fields.len() != columns {
            return Err(invalid(format!(
                "{name} line {}: expected {columns} values, got {}",
                line + 1,
                fields.len()
            )));
        }

        for field in fields {
            data.push(parse_number(field, name, line + 1)?);
        }

        rows += 1;
    }

    if rows < 2 {
        return Err(invalid(format!(
            "{name}: at least two rows are needed"
        )));
    }

    Ok((rows, data))
}

fn read_matrix<const C: usize>(
    path: &Path,
) -> io::Result<OMatrix<f64, Dyn, Const<C>>> {
    let name = path.display().to_string();

    let (rows, data) = parse_matrix(&read_text(path)?, &name, C)?;

    Ok(OMatrix::<f64, Dyn, Const<C>>::from_row_slice_generic(
        Dyn(rows),
        Const::<C>,
        &data,
    ))
}

fn read_text(path: &Path) -> io::Result<String> {
    fs::read_to_string(path).map_err(|error| {
        io::Error::new(
            error.kind(),
            format!("{}: {error}", path.display()),
        )
    })
}

fn parse_number(
    text: &str,
    name: &str,
    line: usize,
) -> io::Result<f64> {
    let text = text.trim();

    let value: f64 = text.parse().map_err(|error| {
        invalid(format!("{name} line {line}: {text:?}: {error}"))
    })?;

    if !value.is_finite() {
        return Err(invalid(format!(
            "{name} line {line}: {text:?} is not finite"
        )));
    }

    Ok(value)
}

fn invalid(message: String) -> io::Error {
    io::Error::new(io::ErrorKind::InvalidData, message)
}

#[cfg(test)]
mod tests {
    use std::time::Duration;

    use nalgebra::{Quaternion, Vector3};

    use super::*;

    fn tables() -> (
        MatrixXx1<f64>,
        CtvXRef,
        CtvK1Flat,
        MatrixXx4<f64>,
        MatrixXx4<f64>,
    ) {
        let tgrid = MatrixXx1::from_column_slice(&[0.0, 1.0]);

        let mut xref = CtvXRef::zeros(2);

        xref[(0, 9)] = 1.0;
        xref[(1, 9)] = 1.0;
        xref[(1, 2)] = -2.0;

        let mut k1flat = CtvK1Flat::zeros(2);

        // K1(0, 2) = 3.
        k1flat[(0, 2)] = 3.0;
        k1flat[(1, 2)] = 3.0;

        // K1(3, 6) = 5.
        k1flat[(0, 13 * 3 + 6)] = 5.0;
        k1flat[(1, 13 * 3 + 6)] = 5.0;

        let k2grid = MatrixXx4::from_row_slice(&[
            0.5, 0.0, 0.0, 0.1,
            1.5, 0.0, 0.0, 0.3,
        ]);

        let unom = MatrixXx4::from_row_slice(&[
            20.0, 0.0, 0.0, 0.0,
            30.0, 0.0, 0.0, 0.0,
        ]);

        (tgrid, xref, k1flat, k2grid, unom)
    }

    fn controller() -> CtvLqrController {
        let (tgrid, xref, k1flat, k2grid, unom) = tables();

        CtvLqrController::new(
            tgrid,
            xref,
            k1flat,
            k2grid,
            unom,
        )
    }

    fn state(
        t: f64,
        down: f64,
        roll_rate: f64,
        q: Quaternion<f64>,
    ) -> ControlState {
        ControlState {
            time: Duration::from_secs_f64(t),
            position: Vector3::new(0.0, 0.0, down),
            velocity: Vector3::zeros(),
            body_rate: Vector3::new(roll_rate, 0.0, 0.0),
            attitude: q,
        }
    }

    #[test]
    fn a_non_finite_state_is_an_error_not_a_panic() {
        let c = controller();
        let upright = Quaternion::identity();

        for bad in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
            let mut s = state(0.5, 0.0, 0.0, upright);
            s.position.x = bad;
            assert_eq!(c.try_step(s), Err(StepError::NonFiniteState));

            let mut s = state(0.5, 0.0, 0.0, upright);
            s.velocity.z = bad;
            assert_eq!(c.try_step(s), Err(StepError::NonFiniteState));

            let s = state(0.5, 0.0, bad, upright);
            assert_eq!(c.try_step(s), Err(StepError::NonFiniteState));

            let s = state(0.5, 0.0, 0.0, Quaternion::new(1.0, bad, 0.0, 0.0));
            assert_eq!(c.try_step(s), Err(StepError::NonFiniteState));
        }
    }

    #[test]
    fn a_zero_attitude_quaternion_is_an_error_not_a_panic() {
        let c = controller();
        let zero = Quaternion::new(0.0, 0.0, 0.0, 0.0);

        assert_eq!(
            c.try_step(state(0.5, 0.0, 0.0, zero)),
            Err(StepError::InvalidAttitude)
        );
    }

    #[test]
    fn step_returns_an_all_nan_command_for_an_unusable_state() {
        let mut c = controller();
        let u = c.step(state(0.5, f64::NAN, 0.0, Quaternion::identity()));

        assert!(u.thrust.is_nan());
        assert!(u.tvc_pitch.is_nan());
        assert!(u.tvc_yaw.is_nan());
        assert!(u.rcs_torque.is_nan());
    }

    #[test]
    fn try_step_matches_step_for_a_usable_state() {
        let mut c = controller();
        let s = state(0.5, 0.3, 0.1, Quaternion::identity());

        assert_eq!(c.try_step(s), Ok(c.step(s)));
    }

    #[test]
    fn k1_flattening_uses_input_major_state_minor_order() {
        let mut flat = [0.0_f64; 52];

        for i in 0..52 {
            flat[i] = i as f64;
        }

        let k1 = unpack_k1_row(&flat);

        assert_eq!(k1[(0, 0)], 0.0);
        assert_eq!(k1[(0, 12)], 12.0);

        assert_eq!(k1[(1, 0)], 13.0);
        assert_eq!(k1[(1, 12)], 25.0);

        assert_eq!(k1[(2, 0)], 26.0);
        assert_eq!(k1[(3, 12)], 51.0);
    }

    #[test]
    fn matches_the_sim_law_with_k2_subtracted() {
        let u = controller().step(state(
            0.5,
            0.0,
            0.1,
            Quaternion::identity(),
        ));

        assert!((u.thrust - 21.0).abs() < 1e-12, "{u:?}");
        assert!((u.rcs_torque - -0.7).abs() < 1e-12, "{u:?}");

        assert_eq!((u.tvc_pitch, u.tvc_yaw), (0.0, 0.0));
    }

    #[test]
    fn flips_the_quaternion_to_the_reference_hemisphere() {
        let mut c = controller();

        let same = c.step(state(
            0.5,
            -1.0,
            0.0,
            Quaternion::identity(),
        ));

        let flipped = c.step(state(
            0.5,
            -1.0,
            0.0,
            -Quaternion::identity(),
        ));

        assert_eq!(same, flipped);
    }

    #[test]
    fn holds_the_ends_of_the_schedule() {
        let mut c = controller();

        let before = c.step(state(
            0.0,
            0.0,
            0.0,
            Quaternion::identity(),
        ));

        let after = c.step(state(
            5.0,
            -2.0,
            0.0,
            Quaternion::identity(),
        ));

        assert!((before.thrust - (20.0 - 0.5)).abs() < 1e-12);
        assert!((after.thrust - (30.0 - 1.5)).abs() < 1e-12);
    }

    #[test]
    fn normalizes_reference_quaternions() {
        let (tgrid, mut xref, k1flat, k2grid, unom) = tables();

        // Deliberately provide a non-unit quaternion.
        xref[(0, 9)] = 2.0;
        xref[(1, 9)] = 2.0;

        let c = CtvLqrController::new(
            tgrid,
            xref,
            k1flat,
            k2grid,
            unom,
        );

        let q_norm = (
            c.xref[(0, 9)].powi(2)
            + c.xref[(0, 10)].powi(2)
            + c.xref[(0, 11)].powi(2)
            + c.xref[(0, 12)].powi(2)
        )
        .sqrt();

        assert!((q_norm - 1.0).abs() < 1e-12);
    }

    const TRAJECTORY_CSV: &str = "\
t,x,y,z,vx,vy,vz,p,q,r,q0,q1,q2,q3,thrust,tvc_pitch,tvc_yaw,rcs
0,0,0,0,0,0,0,0,0,0,1,0,0,0,20,0,0,0
1,0,0,-2,0,0,0,0,0,0,1,0,0,0,30,0,0,0
";

    #[test]
    fn loads_a_full_reference_trajectory_from_csv() {
        let (tgrid, xref, unom) =
            parse_trajectory(TRAJECTORY_CSV, "trajectory.csv").unwrap();

        let (_, _, k1flat, k2grid, _) = tables();

        let mut c = CtvLqrController::new(
            tgrid,
            xref,
            k1flat,
            k2grid,
            unom,
        );

        let u = c.step(state(
            0.5,
            0.0,
            0.1,
            Quaternion::identity(),
        ));

        assert!((u.thrust - 21.0).abs() < 1e-12, "{u:?}");
        assert!((u.rcs_torque - -0.7).abs() < 1e-12, "{u:?}");
    }

    #[test]
    fn rejects_a_schedule_that_does_not_move_forward() {
        let bad = TRAJECTORY_CSV.replace("\n1,", "\n0,");

        assert!(parse_trajectory(&bad, "bad.csv").is_err());
    }

    #[test]
    fn rejects_a_csv_that_does_not_match_its_header() {
        let bad = TRAJECTORY_CSV.replace("tvc_yaw,", "");

        assert!(parse_trajectory(&bad, "bad.csv").is_err());
    }

    #[test]
    fn interpolates_every_table_at_the_midpoint() {
        let u = controller().step(state(
            0.5,
            -1.0,
            0.0,
            Quaternion::identity(),
        ));

        assert!((u.thrust - 24.0).abs() < 1e-12, "{u:?}");
        assert!((u.rcs_torque + 0.2).abs() < 1e-12, "{u:?}");
        assert_eq!((u.tvc_pitch, u.tvc_yaw), (0.0, 0.0));
    }

    #[test]
    fn returns_each_grid_row_at_its_own_time() {
        let start = controller().step(state(
            0.0,
            0.0,
            0.0,
            Quaternion::identity(),
        ));
        let end = controller().step(state(
            1.0,
            -2.0,
            0.0,
            Quaternion::identity(),
        ));

        assert!((start.thrust - 19.5).abs() < 1e-12, "{start:?}");
        assert!((start.rcs_torque + 0.1).abs() < 1e-12, "{start:?}");
        assert!((end.thrust - 28.5).abs() < 1e-12, "{end:?}");
        assert!((end.rcs_torque + 0.3).abs() < 1e-12, "{end:?}");
    }

    #[test]
    fn normalizes_an_interpolated_reference_quaternion() {
        let (tgrid, mut xref, mut k1flat, k2grid, unom) = tables();

        // A half turn about z leaves the midpoint quaternion non-unit.
        xref[(1, 9)] = 0.0;
        xref[(1, 12)] = 1.0;

        // Thrust sees the attitude, so a missing normalization shows up as
        // this gain acting on the 0.207 error of the raw midpoint.
        for row in 0..2 {
            k1flat[(row, 9)] = 2.0;
            k1flat[(row, 12)] = 2.0;
        }

        let mut c =
            CtvLqrController::new(tgrid, xref, k1flat, k2grid, unom);

        let half = std::f64::consts::FRAC_1_SQRT_2;

        let u = c.step(state(
            0.5,
            -1.0,
            0.0,
            Quaternion::new(half, 0.0, 0.0, half),
        ));

        // The error is zero only if the interpolated reference is unit
        // length, so the command is the nominal control minus K2.
        assert!((u.thrust - 24.0).abs() < 1e-12, "{u:?}");
        assert!((u.rcs_torque + 0.2).abs() < 1e-12, "{u:?}");
    }

    #[test]
    fn normalizes_the_measured_attitude() {
        let (tgrid, xref, mut k1flat, k2grid, unom) = tables();

        // Thrust sees q0, so skipping the normalization would turn the 4.0
        // attitude into a 2 * (4 - 1) error on the command.
        for row in 0..2 {
            k1flat[(row, 9)] = 2.0;
        }

        let mut c =
            CtvLqrController::new(tgrid, xref, k1flat, k2grid, unom);

        let unit = c.step(state(
            0.5,
            -1.0,
            0.0,
            Quaternion::identity(),
        ));

        let scaled = c.step(state(
            0.5,
            -1.0,
            0.0,
            Quaternion::new(4.0, 0.0, 0.0, 0.0),
        ));

        assert!((unit.thrust - 24.0).abs() < 1e-12, "{unit:?}");
        assert_eq!(unit, scaled);
    }

    #[test]
    fn aligns_consecutive_reference_quaternions() {
        let (tgrid, mut xref, k1flat, k2grid, unom) = tables();

        // q and -q are the same attitude; the second row is the flipped copy.
        xref[(1, 9)] = -1.0;

        let c =
            CtvLqrController::new(tgrid, xref, k1flat, k2grid, unom);

        assert_eq!(c.xref[(1, 9)], 1.0);
    }

    #[test]
    fn applies_all_four_k1_rows_in_input_major_order() {
        let tgrid = MatrixXx1::from_column_slice(&[0.0, 1.0]);

        let mut xref = CtvXRef::zeros(2);

        xref[(0, 9)] = 1.0;
        xref[(1, 9)] = 1.0;

        let mut k1flat = CtvK1Flat::zeros(2);
        let mut k2grid = MatrixXx4::zeros(2);
        let mut unom = MatrixXx4::zeros(2);

        for row in 0..2 {
            for input in 0..4 {
                for state_index in 0..13 {
                    let gain = (input * 13 + state_index + 1) as f64;
                    k1flat[(row, input * 13 + state_index)] = gain;
                }

                k2grid[(row, input)] = (input + 1) as f64;
                unom[(row, input)] = 10.0 * (input + 1) as f64;
            }
        }

        let mut c =
            CtvLqrController::new(tgrid, xref, k1flat, k2grid, unom);

        let dx = [
            1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 0.0, 0.0, 0.0,
            0.0,
        ];

        let u = c.step(ControlState {
            time: Duration::from_secs_f64(0.5),
            position: Vector3::new(1.0, 2.0, 3.0),
            velocity: Vector3::new(4.0, 5.0, 6.0),
            body_rate: Vector3::new(7.0, 8.0, 9.0),
            attitude: Quaternion::identity(),
        });

        let got = [u.thrust, u.tvc_pitch, u.tvc_yaw, u.rcs_torque];

        for input in 0..4 {
            let mut expected =
                10.0 * (input + 1) as f64 - (input + 1) as f64;

            for (state_index, error) in dx.iter().enumerate() {
                let gain = (input * 13 + state_index + 1) as f64;
                expected -= gain * error;
            }

            assert!(
                (got[input] - expected).abs() < 1e-9,
                "input {input}: {} vs {expected}",
                got[input]
            );
        }
    }

    #[test]
    fn interpolates_inside_a_long_schedule() {
        let n = 100;

        let mut tgrid = MatrixXx1::zeros(n);
        let mut xref = CtvXRef::zeros(n);
        let mut k1flat = CtvK1Flat::zeros(n);

        for i in 0..n {
            tgrid[i] = i as f64 / 10.0;
            xref[(i, 2)] = -(i as f64);
            xref[(i, 9)] = 1.0;
            k1flat[(i, 2)] = 1.0;
        }

        let mut c = CtvLqrController::new(
            tgrid,
            xref,
            k1flat,
            MatrixXx4::zeros(n),
            MatrixXx4::zeros(n),
        );

        // 4.35 s sits halfway between the rows at 4.3 s and 4.4 s.
        let u = c.step(state(
            4.35,
            0.0,
            0.0,
            Quaternion::identity(),
        ));

        assert!((u.thrust + 43.5).abs() < 1e-9, "{u:?}");
    }

    #[test]
    #[should_panic(expected = "at least two grid points")]
    fn rejects_a_single_grid_point() {
        CtvLqrController::new(
            MatrixXx1::from_column_slice(&[0.0]),
            CtvXRef::zeros(1),
            CtvK1Flat::zeros(1),
            MatrixXx4::zeros(1),
            MatrixXx4::zeros(1),
        );
    }

    #[test]
    #[should_panic(expected = "strictly increasing")]
    fn rejects_a_time_grid_that_stands_still() {
        let (_, xref, k1flat, k2grid, unom) = tables();

        let tgrid = MatrixXx1::from_column_slice(&[0.0, 0.0]);

        CtvLqrController::new(tgrid, xref, k1flat, k2grid, unom);
    }

    #[test]
    #[should_panic(expected = "inconsistent row dimension")]
    fn rejects_tables_with_different_row_counts() {
        let (tgrid, _, k1flat, k2grid, unom) = tables();

        CtvLqrController::new(
            tgrid,
            CtvXRef::zeros(3),
            k1flat,
            k2grid,
            unom,
        );
    }

    #[test]
    #[should_panic(expected = "k1flat must contain only finite values")]
    fn rejects_non_finite_gains() {
        let (tgrid, xref, mut k1flat, k2grid, unom) = tables();

        k1flat[(0, 0)] = f64::NAN;

        CtvLqrController::new(tgrid, xref, k1flat, k2grid, unom);
    }

    #[test]
    #[should_panic(expected = "nonzero finite norm")]
    fn rejects_a_zero_reference_quaternion() {
        let (tgrid, mut xref, k1flat, k2grid, unom) = tables();

        for j in 9..13 {
            xref[(0, j)] = 0.0;
        }

        CtvLqrController::new(tgrid, xref, k1flat, k2grid, unom);
    }

    #[test]
    fn rejects_a_csv_with_a_degenerate_attitude() {
        // `new` panics on this input; the CSV loader reports an error instead.
        let zeroed = TRAJECTORY_CSV
            .replace(",1,0,0,0,20,", ",0,0,0,0,20,")
            .replace(",1,0,0,0,30,", ",0,0,0,0,30,");

        let error = parse_trajectory(&zeroed, "bad.csv").unwrap_err();

        assert!(
            error.to_string().contains("cannot be normalized"),
            "{error}"
        );
    }

    #[test]
    fn reads_trajectory_headers_case_insensitively() {
        let csv = TRAJECTORY_CSV.to_ascii_uppercase();

        let (tgrid, xref, unom) =
            parse_trajectory(&csv, "trajectory.csv").unwrap();

        assert_eq!(tgrid.nrows(), 2);
        assert_eq!(xref[(1, 2)], -2.0);
        assert_eq!(unom[(1, 0)], 30.0);
    }

    #[test]
    fn ignores_a_byte_order_mark() {
        let csv = format!("\u{feff}{}", TRAJECTORY_CSV.trim_start());

        assert!(parse_trajectory(&csv, "trajectory.csv").is_ok());
    }

    #[test]
    fn rejects_broken_trajectory_tables() {
        let missing_column = TRAJECTORY_CSV.replace("rcs", "roll");
        assert!(parse_trajectory(&missing_column, "bad.csv").is_err());

        let duplicate_column = TRAJECTORY_CSV.replace("vx,", "x,");
        assert!(parse_trajectory(&duplicate_column, "bad.csv").is_err());

        let missing_field = TRAJECTORY_CSV.replace(",20,0,0,0", ",20,0,0");
        assert!(parse_trajectory(&missing_field, "bad.csv").is_err());

        let not_a_number =
            TRAJECTORY_CSV.replace("1,0,0,-2", "1,0,0,-2x");
        assert!(parse_trajectory(&not_a_number, "bad.csv").is_err());

        let infinite = TRAJECTORY_CSV.replace("1,0,0,-2", "1,0,0,-inf");
        assert!(parse_trajectory(&infinite, "bad.csv").is_err());
    }

    #[test]
    fn rejects_empty_and_header_only_trajectories() {
        assert!(parse_trajectory("", "bad.csv").is_err());

        let header = TRAJECTORY_CSV.trim_start().lines().next().unwrap();

        assert!(parse_trajectory(header, "bad.csv").is_err());
    }

    #[test]
    fn drives_the_tvc_channels_from_k1_rows_one_and_two() {
        let (tgrid, xref, mut k1flat, k2grid, mut unom) = tables();

        for row in 0..2 {
            k1flat[(row, 13)] = 2.0;
            k1flat[(row, 26)] = -4.0;
            unom[(row, 1)] = 0.1;
            unom[(row, 2)] = -0.2;
        }

        let mut c =
            CtvLqrController::new(tgrid, xref, k1flat, k2grid, unom);

        let u = c.step(ControlState {
            time: Duration::from_secs_f64(0.5),
            position: Vector3::new(1.0, 0.0, -1.0),
            velocity: Vector3::zeros(),
            body_rate: Vector3::zeros(),
            attitude: Quaternion::identity(),
        });

        assert!((u.thrust - 24.0).abs() < 1e-12, "{u:?}");
        assert!((u.tvc_pitch + 1.9).abs() < 1e-12, "{u:?}");
        assert!((u.tvc_yaw - 3.8).abs() < 1e-12, "{u:?}");
    }

    #[test]
    fn repeated_steps_return_the_same_command() {
        let mut c = controller();

        let first = c.step(state(
            0.3,
            0.25,
            0.05,
            Quaternion::identity(),
        ));

        let second = c.step(state(
            0.3,
            0.25,
            0.05,
            Quaternion::identity(),
        ));

        assert_eq!(first, second);
    }
}