//! The Sensors block's outputs come through `Plant::sensors`, and a plant
//! stepped one 1 ms step at a time agrees with one stepped by control periods.
//!
//! Needs a plant built with sensor outports (hopper's build_plant_c from the
//! sensor-outport commit on). Fails without the plant, unless
//! HOPPER_PLANT_ALLOW_SKIP=1.

use hopper_plant::{layout, Error, Plant, HAS_SENSORS, STEP};

mod common;

/// Only one `Plant` can exist at a time, so the tests in this file take turns.
static ONE_AT_A_TIME: std::sync::Mutex<()> = std::sync::Mutex::new(());

#[test]
fn the_sensors_read_the_pad_and_follow_the_plant() {
    let _turn = ONE_AT_A_TIME.lock().unwrap_or_else(|e| e.into_inner());
    let mut plant = match Plant::new() {
        Ok(p) => p,
        Err(Error::NotBuilt) => {
            common::cannot_run("built without the plant (HOPPER_PLANT_CODE unset)");
            return;
        }
        Err(e) => panic!("{e}"),
    };

    if !HAS_SENSORS {
        common::cannot_run("this plant has no sensor outports");
        return;
    }

    let at_rest = plant.sensors().unwrap().expect("a plant with sensors reads them");

    // On the pad: one g up along body x (up on the pad), no rotation, and the
    // barometer at the pad's altitude
    assert!(
        (at_rest.meas[layout::ACCEL.start] - 9.8).abs() < 0.5,
        "{:?}",
        &at_rest.meas[layout::ACCEL]
    );
    assert!(at_rest.valid[layout::S_IMU] && at_rest.valid[layout::S_BARO] && at_rest.valid[layout::S_GPS]);
    assert!(at_rest.meas[layout::BARO].abs() < 1.0, "{}", at_rest.meas[layout::BARO]);

    // Stepped one millisecond at a time with thrust, the plant climbs, the
    // barometer follows, and the IMU takes new samples
    plant.set_command(&[30.0, 0.0, 0.0, 0.0]);

    let mut stamps = std::collections::BTreeSet::new();
    let mut last = at_rest;

    for _ in 0..1000 {
        plant.step().unwrap();
        last = plant.sensors().unwrap().unwrap();
        stamps.insert(last.stamp[layout::S_IMU].to_bits());
    }

    assert!((plant.time() - 1000.0 * STEP).abs() < 1e-9);
    assert!(stamps.len() > 100, "{} distinct IMU sample times in a second", stamps.len());
    let state = plant.state().unwrap();

    assert!(
        last.meas[layout::BARO] > 0.4,
        "climbing: baro {} m, truth {} m up, accel {:?}",
        last.meas[layout::BARO],
        -state[2],
        &last.meas[layout::ACCEL]
    );
}

#[test]
fn single_steps_are_the_same_flight_as_control_periods() {
    let _turn = ONE_AT_A_TIME.lock().unwrap_or_else(|e| e.into_inner());
    let command = [30.0, 0.001, -0.002, 0.0];

    let mut by_period = match Plant::new() {
        Ok(p) => p,
        Err(Error::NotBuilt) => {
            common::cannot_run("built without the plant (HOPPER_PLANT_CODE unset)");
            return;
        }
        Err(e) => panic!("{e}"),
    };

    for _ in 0..50 {
        by_period.run_period(&command).unwrap();
    }

    let want = by_period.state().unwrap();

    drop(by_period);

    let mut by_step = Plant::new().unwrap();

    by_step.set_command(&command);

    for _ in 0..(50 * by_step.steps_per_period()) {
        by_step.step().unwrap();
    }

    assert_eq!(by_step.state().unwrap(), want);
}
