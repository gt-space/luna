//! A plant built with a ground (`build_plant_c('Ground', 'strut')`): what the state
//! and the sensors read with the vehicle at rest on its legs. Prints; it asserts
//! only what holds for any plant.
//!
//!   HOPPER_PLANT_CODE=<hopper>/.../plant_strut cargo test -p hopper_plant \
//!       --test ground_rest -- --nocapture

use hopper_plant::{layout, Error, Plant, HAS_SENSORS};

mod common;

#[test]
fn what_the_state_and_sensors_read_at_rest_on_the_ground() {
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

    // thrust far below the weight: the vehicle settles on its legs
    plant.set_command(&[0.0, 0.0, 0.0, 0.0]);

    for _ in 0..1500 {
        plant.step().unwrap();
    }

    let state = plant.state().unwrap();
    let s = plant.sensors().unwrap().unwrap();

    println!("state: pos {:?} vel {:?}", &state[0..3], &state[3..6]);
    println!("accel {:?}", &s.meas[layout::ACCEL]);
    println!("baro height {}", s.meas[layout::BARO]);
    println!("lidar ranges {:?}", &s.meas[layout::LIDAR]);
    println!("gps pos {:?}", &s.meas[layout::GPS_POS]);

    assert!(state.iter().all(|v| v.is_finite()));
}
