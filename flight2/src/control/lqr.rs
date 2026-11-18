use common::comm::ctv::{ControlState, ControlVector};
use nalgebra::{Const, Dyn, Matrix, MatrixXx1, MatrixXx4, VecStorage};
use serde::{Deserialize, Serialize};

use super::Controller;

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct LqrController {
    n: usize,
    tgrid: MatrixXx1<f64>,
    xref: Matrix<f64, Dyn, Const<13>, VecStorage<f64, Dyn, Const<13>>>,
    k1flat: Matrix<f64, Dyn, Const<52>, VecStorage<f64, Dyn, Const<52>>>,
    k2grid: MatrixXx4<f64>,
    unom: MatrixXx4<f64>,
}

impl LqrController {
    pub fn new(
        tgrid: MatrixXx1<f64>,
        xref: Matrix<f64, Dyn, Const<13>, VecStorage<f64, Dyn, Const<13>>>,
        k1flat: Matrix<f64, Dyn, Const<52>, VecStorage<f64, Dyn, Const<52>>>,
        k2grid: MatrixXx4<f64>,
        unom: MatrixXx4<f64>,
    ) -> Self {
        let n = tgrid.nrows();
        assert!(n == xref.nrows(), "inconsistent row dimension");
        assert!(n == k1flat.nrows(), "inconsistent row dimension");
        assert!(n == k2grid.nrows(), "inconsistent row dimension");
        assert!(n == unom.nrows(), "inconsistent row dimension");

        LqrController {
            n,
            tgrid,
            xref,
            k1flat,
            k2grid,
            unom,
        }
    }
}

impl Controller for LqrController {
    fn step(&mut self, state: ControlState) -> ControlVector {
        let x = state.to_matrix();

        let t = state.time.as_secs_f64();

        // Clamp time and compute interval index (0-based)
        let (idx, w) = if t <= self.tgrid[0] {
            (0, 0.0)
        } else if t >= self.tgrid[self.n - 1] {
            (self.n - 2, 1.0)
        } else {
            let mut idx = 0;
            for k in 0..(self.n - 1) {
                if self.tgrid[k] <= t && t < self.tgrid[k + 1] {
                    idx = k;
                    break;
                }
            }
            let (t1, t2) = (self.tgrid[idx], self.tgrid[idx + 1]);
            let w = (t - t1) / (t2 - t1);
            (idx, w)
        };

        // Interpolate reference state (13x1)
        let xref_t =
            (1.0 - w) * self.xref.row(idx).transpose() + w * self.xref.row(idx + 1).transpose();
        // Interpolate K1flat then reshape to 4x13
        // MATLAB: reshape(Kflat, 13, 4)' → col-major fill into 13x4, then transpose to
        // 4x13
        let kflat = (1.0 - w) * self.k1flat.row(idx) + w * self.k1flat.row(idx + 1);
        let k1 = kflat.reshape_generic(Const::<13>, Const::<4>).transpose();
        // Interpolate K2 (4x1)
        let k2 =
            (1.0 - w) * self.k2grid.row(idx).transpose() + w * self.k2grid.row(idx + 1).transpose();
        // Interpolate nominal control (4x1)
        let unom =
            (1.0 - w) * self.unom.row(idx).transpose() + w * self.unom.row(idx + 1).transpose();

        // Quaternion sign convention: flip q if dot(q, qref) < 0
        let mut q = x.rows_range(9..13).clone_owned();
        let qref = xref_t.rows_range(9..13);
        if q.dot(&qref) < 0.0 {
            q *= -1.0;
        }

        let mut x_used = x.clone_owned();
        x_used.rows_range_mut(9..13).copy_from(&q);

        // Control law: u = u_nom - K1 * dx - K2, the same as the hopper
        // sim's LQR_Gain_Scheduler (Gain_Selector) that the gains come from
        let dx = x_used - xref_t;
        let u = unom - k1 * dx - k2;

        ControlVector::from_matrix(u)
    }
}

#[cfg(test)]
mod tests {
    use std::time::Duration;

    use nalgebra::{Quaternion, Vector3};

    use super::*;

    /// Two grid points, 0 s and 1 s, with every table linear in time
    fn controller() -> LqrController {
        let tgrid = MatrixXx1::from_column_slice(&[0.0, 1.0]);
        let mut xref = Matrix::<f64, Dyn, Const<13>, VecStorage<f64, Dyn, Const<13>>>::zeros(2);
        xref[(0, 9)] = 1.0; // identity quaternion at both times
        xref[(1, 9)] = 1.0;
        xref[(1, 2)] = -2.0; // climb to 2 m up (NED down = -2) by t = 1
        let mut k1flat = Matrix::<f64, Dyn, Const<52>, VecStorage<f64, Dyn, Const<52>>>::zeros(2);
        // K1(0, 2) = 3 (thrust row, down position) at both times: row-major,
        // as MATLAB's reshape(Kflat, 13, 4)' reads it
        k1flat[(0, 2)] = 3.0;
        k1flat[(1, 2)] = 3.0;
        // K1(3, 6) = 5 (RCS row, roll rate)
        k1flat[(0, 13 * 3 + 6)] = 5.0;
        k1flat[(1, 13 * 3 + 6)] = 5.0;
        let k2grid = MatrixXx4::from_row_slice(&[0.5, 0.0, 0.0, 0.1, 1.5, 0.0, 0.0, 0.3]);
        let unom = MatrixXx4::from_row_slice(&[20.0, 0.0, 0.0, 0.0, 30.0, 0.0, 0.0, 0.0]);
        LqrController::new(tgrid, xref, k1flat, k2grid, unom)
    }

    fn state(t: f64, down: f64, roll_rate: f64, q: Quaternion<f64>) -> ControlState {
        ControlState {
            time: Duration::from_secs_f64(t),
            position: Vector3::new(0.0, 0.0, down),
            velocity: Vector3::zeros(),
            body_rate: Vector3::new(roll_rate, 0.0, 0.0),
            attitude: q,
        }
    }

    #[test]
    fn matches_the_sim_law_with_k2_subtracted() {
        // t = 0.5: xref down = -1, K1(0,2) = 3, K2 = [1, 0, 0, 0.2], unom = 25
        let u = controller().step(state(0.5, 0.0, 0.1, Quaternion::identity()));
        // thrust: 25 - 3 * (0 - (-1)) - 1 = 21
        assert!((u.thrust - 21.0).abs() < 1e-12, "{u:?}");
        // RCS: 0 - 5 * 0.1 - 0.2 = -0.7
        assert!((u.rcs_torque - -0.7).abs() < 1e-12, "{u:?}");
        assert_eq!((u.tvc_pitch, u.tvc_yaw), (0.0, 0.0));
    }

    #[test]
    fn flips_the_quaternion_to_the_reference_hemisphere() {
        let mut c = controller();
        let same = c.step(state(0.5, -1.0, 0.0, Quaternion::identity()));
        let flipped = c.step(state(0.5, -1.0, 0.0, -Quaternion::identity()));
        assert_eq!(same, flipped);
    }

    #[test]
    fn holds_the_ends_of_the_schedule() {
        let mut c = controller();
        let before = c.step(state(0.0, 0.0, 0.0, Quaternion::identity()));
        let after = c.step(state(5.0, -2.0, 0.0, Quaternion::identity()));
        assert!((before.thrust - (20.0 - 0.5)).abs() < 1e-12);
        assert!((after.thrust - (30.0 - 1.5)).abs() < 1e-12);
    }
}
