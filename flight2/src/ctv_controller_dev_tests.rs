//! Development tests for the CTV LQR controller.
//!
//! These drive the controller the way the flight computer will: reference and
//! gain tables come from CSV files on disk and the controller is stepped in a
//! loop. The maths itself is covered by the unit tests in
//! `flight2/src/ctv_controller.rs`.

use std::{
    fs,
    path::{Path, PathBuf},
    time::Duration,
};

use common::comm::ctv::ControlState;
use serde_json::Value;
use nalgebra::{Quaternion, Vector3};
use tempfile::tempdir;

use crate::{control::Controller, ctv_controller::CtvLqrController};

const TRAJECTORY: &str = "\
t,x,y,z,vx,vy,vz,p,q,r,q0,q1,q2,q3,thrust,tvc_pitch,tvc_yaw,rcs
0,0,0,0,0,0,0,0,0,0,1,0,0,0,20,0,0,0
2,0,0,-4,0,0,0,0,0,0,1,0,0,0,40,0,0,0
";

const K2: &str = "\
1,0.1,0.2,0.3
3,0.3,0.4,0.5
";

/// A headerless 52-column K1 table with `entries` set in every row.
///
/// K1 is stored input-major, so entry `input * 13 + state` is the gain from
/// that state to that input.
fn k1_table(rows: usize, entries: &[(usize, f64)]) -> String {
    let mut text = String::new();

    for _ in 0..rows {
        let mut fields = vec![0.0_f64; 52];

        for (index, value) in entries {
            fields[*index] = *value;
        }

        let row: Vec<String> =
            fields.iter().map(|value| value.to_string()).collect();

        text.push_str(&row.join(","));
        text.push('\n');
    }

    text
}

fn write_csv(dir: &Path, name: &str, contents: &str) -> PathBuf {
    let path = dir.join(name);

    fs::write(&path, contents).unwrap();

    path
}

fn state(t: f64, down: f64) -> ControlState {
    ControlState {
        time: Duration::from_secs_f64(t),
        position: Vector3::new(0.0, 0.0, down),
        velocity: Vector3::zeros(),
        body_rate: Vector3::zeros(),
        attitude: Quaternion::identity(),
    }
}

fn load(dir: &Path, trajectory: &str, k1: &str, k2: &str) -> CtvLqrController {
    let trajectory = write_csv(dir, "trajectory.csv", trajectory);
    let k1 = write_csv(dir, "k1flat.csv", k1);
    let k2 = write_csv(dir, "k2grid.csv", k2);

    CtvLqrController::from_trajectory_csv(trajectory, k1, k2).unwrap()
}

#[test]
fn loads_a_trajectory_and_gain_files_from_disk() {
    let dir = tempdir().unwrap();

    // Thrust <- down position, TVC pitch <- x, RCS <- roll rate.
    let k1 = k1_table(2, &[(2, 2.0), (13, -1.0), (45, 4.0)]);

    let mut c = load(dir.path(), TRAJECTORY, &k1, K2);

    let quarter = c.step(state(0.5, 0.0));
    let three_quarters = c.step(state(1.5, 0.0));

    assert!((quarter.thrust - 21.5).abs() < 1e-12, "{quarter:?}");
    assert!((quarter.tvc_pitch + 0.15).abs() < 1e-12, "{quarter:?}");
    assert!((quarter.tvc_yaw + 0.25).abs() < 1e-12, "{quarter:?}");
    assert!((quarter.rcs_torque + 0.35).abs() < 1e-12, "{quarter:?}");

    assert!((three_quarters.thrust - 26.5).abs() < 1e-12);
    assert!((three_quarters.tvc_pitch + 0.25).abs() < 1e-12);
    assert!((three_quarters.tvc_yaw + 0.35).abs() < 1e-12);
    assert!((three_quarters.rcs_torque + 0.45).abs() < 1e-12);
}

#[test]
fn holds_the_last_row_past_the_schedule() {
    let dir = tempdir().unwrap();

    let k1 = k1_table(2, &[(2, 2.0)]);

    let mut c = load(dir.path(), TRAJECTORY, &k1, K2);

    let at_end = c.step(state(2.0, -4.0));
    let past_end = c.step(state(30.0, -4.0));

    assert!((past_end.thrust - 37.0).abs() < 1e-12, "{past_end:?}");
    assert_eq!(at_end, past_end);
}

#[test]
fn loads_reference_files_with_windows_line_endings() {
    let dir = tempdir().unwrap();

    let csv = TRAJECTORY.replace('\n', "\r\n");
    let k1 = k1_table(2, &[(2, 2.0)]);

    let mut c = load(dir.path(), &csv, &k1, K2);

    let u = c.step(state(0.5, 0.0));

    assert!((u.thrust - 21.5).abs() < 1e-12, "{u:?}");
}

#[test]
fn reports_gain_tables_with_the_wrong_row_count() {
    let dir = tempdir().unwrap();

    let trajectory = write_csv(dir.path(), "trajectory.csv", TRAJECTORY);
    let k1 = write_csv(dir.path(), "k1flat.csv", &k1_table(3, &[]));
    let k2 = write_csv(dir.path(), "k2grid.csv", K2);

    let error = CtvLqrController::from_trajectory_csv(trajectory, k1, k2)
        .unwrap_err();

    assert!(
        error.to_string().contains("3 rows for a 2-row trajectory"),
        "{error}"
    );
}

#[test]
fn reports_a_gain_row_with_the_wrong_number_of_values() {
    let dir = tempdir().unwrap();

    let trajectory = write_csv(dir.path(), "trajectory.csv", TRAJECTORY);
    let k1 = write_csv(dir.path(), "k1flat.csv", "0,0,0\n0,0,0\n");
    let k2 = write_csv(dir.path(), "k2grid.csv", K2);

    let error = CtvLqrController::from_trajectory_csv(trajectory, k1, k2)
        .unwrap_err();

    assert!(
        error.to_string().contains("expected 52 values, got 3"),
        "{error}"
    );
}

#[test]
fn reports_a_bad_attitude_instead_of_panicking() {
    let dir = tempdir().unwrap();

    let zeroed = TRAJECTORY
        .replace(",1,0,0,0,20,", ",0,0,0,0,20,")
        .replace(",1,0,0,0,40,", ",0,0,0,0,40,");

    let trajectory = write_csv(dir.path(), "trajectory.csv", &zeroed);
    let k1 = write_csv(dir.path(), "k1flat.csv", &k1_table(2, &[]));
    let k2 = write_csv(dir.path(), "k2grid.csv", K2);

    let error = CtvLqrController::from_trajectory_csv(trajectory, k1, k2)
        .unwrap_err();

    assert!(
        error.to_string().contains("cannot be normalized"),
        "{error}"
    );
}

#[test]
fn reports_a_missing_reference_file() {
    let dir = tempdir().unwrap();

    let missing = dir.path().join("does_not_exist.csv");
    let k1 = write_csv(dir.path(), "k1flat.csv", &k1_table(2, &[]));
    let k2 = write_csv(dir.path(), "k2grid.csv", K2);

    let error =
        CtvLqrController::from_trajectory_csv(&missing, k1, k2).unwrap_err();

    assert!(error.to_string().contains("does_not_exist.csv"), "{error}");
}

#[test]
fn round_trips_the_schedule_through_json() {
    let dir = tempdir().unwrap();

    let k1 = k1_table(2, &[(2, 2.0), (13, -1.0)]);

    let mut original = load(dir.path(), TRAJECTORY, &k1, K2);

    let json = serde_json::to_string(&original).unwrap();
    let mut restored: CtvLqrController = serde_json::from_str(&json).unwrap();

    let expected = original.step(state(0.75, 0.1));
    let actual = restored.step(state(0.75, 0.1));

    assert_eq!(expected, actual);
}

#[test]
fn holds_altitude_in_a_toy_closed_loop() {
    let dir = tempdir().unwrap();

    // Four seconds of holding still, trimmed to the hover thrust.
    let trajectory = "\
t,x,y,z,vx,vy,vz,p,q,r,q0,q1,q2,q3,thrust,tvc_pitch,tvc_yaw,rcs
0,0,0,0,0,0,0,0,0,0,1,0,0,0,25.5,0,0,0
4,0,0,0,0,0,0,0,0,0,1,0,0,0,25.5,0,0,0
";

    // Thrust rises when the vehicle sinks, with velocity damping.
    let k1 = k1_table(2, &[(2, -2.0), (5, -1.0)]);

    let mut c = load(dir.path(), trajectory, &k1, "0,0,0,0\n0,0,0,0\n");

    let dt = 0.005;
    let mut down = 0.0_f64;
    let mut speed = 0.0_f64;

    for k in 0..800 {
        let u = c.step(ControlState {
            time: Duration::from_secs_f64(k as f64 * dt),
            position: Vector3::new(0.0, 0.0, down),
            velocity: Vector3::new(0.0, 0.0, speed),
            body_rate: Vector3::zeros(),
            attitude: Quaternion::identity(),
        });

        assert!(u.thrust.is_finite(), "thrust at step {k}");

        // 25.5 N holds a 2.6 kg vehicle, so the command stays near hover.
        assert!(
            (20.0..32.0).contains(&u.thrust),
            "thrust {} at step {k}",
            u.thrust
        );

        speed += (9.81 - u.thrust / 2.6) * dt;
        down += speed * dt;
    }

    assert!(down.abs() < 0.05, "drifted {down} m");
}

/// The Rust controller against the Simulink controller's own output.
///
/// Hopper's `export_controller_vectors.m` runs the model's
/// `LQR_Gain_Scheduler/Gain_Selector` on the exported gain tables for a few
/// thousand states (the nominal flight, every grid point, dispersed states with
/// negated quaternions, and times outside the schedule) and writes the result
/// as `controller_vectors.csv` (`t,x1..x13,u1..u4`) next to the gains.
///
/// Set `CTV_GOLDEN_DIR` to that folder, for example
/// `<hopper>/sims/hopper_6dof_sizer/fsw_bridge/build/gains/ctv`, and run with
/// `--ignored`. A `-dirty` export needs `CTV_GOLDEN_ALLOW_DIRTY=1`; flight
/// builds must use a clean one.
#[test]
#[ignore = "needs hopper's export: set CTV_GOLDEN_DIR to its build/gains/ctv folder"]
fn matches_the_simulink_golden_vectors() {
    use crate::gain_manifest::Policy;

    let dir = PathBuf::from(
        std::env::var_os("CTV_GOLDEN_DIR").expect("CTV_GOLDEN_DIR is not set"),
    );
    let policy = Policy {
        controller: "lqr_ctv_tuning",
        plant_commit: None,
        allow_dirty: std::env::var_os("CTV_GOLDEN_ALLOW_DIRTY").is_some(),
    };

    let controller = CtvLqrController::from_gain_dir(&dir, &policy).unwrap();

    let json = |name: &str| -> Value {
        serde_json::from_str(&fs::read_to_string(dir.join(name)).unwrap()).unwrap()
    };

    // The vectors have to come from these very gains
    let meta = json("controller_vectors.json");
    let gains = json("gains.json");

    assert_eq!(meta["controller"], "lqr_ctv_tuning");
    assert_eq!(meta["gains_sha256"], gains["sha256"], "vectors are for other gains");

    // Times before the schedule hold its first row, which is only the same as
    // t = 0 if the schedule starts at 0
    let tgrid = fs::read_to_string(dir.join("tgrid.csv")).unwrap();
    assert_eq!(tgrid.lines().next().unwrap().trim().parse::<f64>().unwrap(), 0.0);

    let text = fs::read_to_string(dir.join("controller_vectors.csv")).unwrap();
    let mut rows = 0;
    let mut worst = 0.0_f64;
    let mut non_unit = 0;
    let mut failures = Vec::new();

    for (index, line) in text.lines().enumerate().skip(1) {
        let v: Vec<f64> = line.split(',').map(|f| f.trim().parse().unwrap()).collect();

        assert_eq!(v.len(), 18, "line {}", index + 1);

        let state = ControlState {
            time: Duration::from_secs_f64(v[0].max(0.0)),
            position: Vector3::new(v[1], v[2], v[3]),
            velocity: Vector3::new(v[4], v[5], v[6]),
            body_rate: Vector3::new(v[7], v[8], v[9]),
            attitude: Quaternion::new(v[10], v[11], v[12], v[13]),
        };
        let got = controller.try_step(state).unwrap();
        let got = [got.thrust, got.tvc_pitch, got.tvc_yaw, got.rcs_torque];

        // Simulink uses the measured quaternion as it is. The Rust controller
        // normalizes it, so the two only agree for a unit quaternion, which is
        // what the estimator produces. A deliberately non-unit one is checked
        // against a loose bound instead: it must not move the command by more
        // than 5 % of max(1, |u|).
        let norm = (v[10] * v[10] + v[11] * v[11] + v[12] * v[12] + v[13] * v[13]).sqrt();
        let unit = (norm - 1.0).abs() <= 1e-9;

        if !unit {
            non_unit += 1;
        }

        for (channel, (got, want)) in got.iter().zip(&v[14..18]).enumerate() {
            let error = (got - want).abs();
            let scale = want.abs().max(1.0);
            let limit = if unit { 1e-9 * scale } else { 0.05 * scale };

            if unit {
                worst = worst.max(error / limit);
            }

            if !(error <= limit) {
                failures.push(format!(
                    "line {} u{}: got {got}, want {want} (|q| = {norm})",
                    index + 1,
                    channel + 1
                ));
            }
        }

        rows += 1;
    }

    println!(
        "{rows} vectors ({non_unit} with a non-unit quaternion, checked loosely);          worst error on the rest is {worst:.3e} of the allowed"
    );

    assert_eq!(rows, meta["rows"].as_u64().unwrap() as usize);
    assert!(
        failures.is_empty(),
        "{} of {} components differ:
{}",
        failures.len(),
        rows * 4,
        failures.iter().take(20).cloned().collect::<Vec<_>>().join("
")
    );
}
