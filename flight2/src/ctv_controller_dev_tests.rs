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
use nalgebra::{Quaternion, Vector3};
use tempfile::tempdir;

use crate::{control::Controller, ctv_controller::CtvLqrController};

const TRAJECTORY: &str = "\
t,x,y,z,vx,vy,vz,p,q,r,q0,q1,q2,q3,thrust,tvc_pitch,tvc_yaw,rcs
0,0,0,0,0,0,0,0,0,0,1,0,0,0,20,0,0,0
2,0,0,-4,0,0,0,0,0,0,1,0,0,0,40,0,0,0
";

const HOP: &str = "\
t,z,vz,az,thrust,mass
0,0,0,0,20,2.6
1,2,2,0,30,2.6
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
fn loads_a_hop_schedule_from_disk() {
    let dir = tempdir().unwrap();

    // Thrust <- down position, thrust <- down velocity.
    let k1 = k1_table(2, &[(2, 3.0), (5, 7.0)]);

    let hop = write_csv(dir.path(), "hop.csv", HOP);
    let k1 = write_csv(dir.path(), "k1flat.csv", &k1);
    let k2 = write_csv(dir.path(), "k2grid.csv", "0,0,0,0\n0,0,0,0\n");

    let mut c = CtvLqrController::from_hop_csv(hop, k1, k2).unwrap();

    let half = std::f64::consts::FRAC_1_SQRT_2;

    let on_reference = ControlState {
        time: Duration::from_secs_f64(0.5),
        position: Vector3::new(0.0, 0.0, -1.0),
        velocity: Vector3::new(0.0, 0.0, -1.0),
        body_rate: Vector3::zeros(),
        attitude: Quaternion::new(half, 0.0, half, 0.0),
    };

    assert!((c.step(on_reference).thrust - 25.0).abs() < 1e-12);

    // Two metres below the reference, still moving with it.
    let low = ControlState {
        time: Duration::from_secs_f64(0.5),
        position: Vector3::new(0.0, 0.0, 1.0),
        velocity: Vector3::new(0.0, 0.0, -1.0),
        body_rate: Vector3::zeros(),
        attitude: Quaternion::new(half, 0.0, half, 0.0),
    };

    assert!((c.step(low).thrust - 19.0).abs() < 1e-12);
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
