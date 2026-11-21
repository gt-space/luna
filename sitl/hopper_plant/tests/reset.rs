//! `Plant::reset` starts over exactly as `Plant::new` did: time back to 0,
//! the touchdown flag cleared, and the same commands give the same flight.
//! The generated code's own initialize() doesn't clear the model's time or
//! stop flag, so this guards the shim's full reset.
//!
//! Needs the plant (HOPPER_PLANT_CODE); skipped in a stub build.

use hopper_plant::{Error, Plant, State};

/// Flies `periods` control periods of a fixed command, returning every state
fn fly(plant: &mut Plant, u: [f64; 4], periods: usize) -> Vec<State> {
    (0..periods)
        .map(|_| {
            let x = plant.state().unwrap();
            plant.run_period(&u).unwrap();
            x
        })
        .collect()
}

#[test]
fn reset_starts_over_exactly() {
    let mut plant = match Plant::new() {
        Ok(p) => p,
        Err(Error::NotBuilt) => {
            eprintln!("built without the plant; skipping");
            return;
        }
        Err(e) => panic!("{e}"),
    };

    // No thrust: the vehicle falls and the harness's touchdown stop fires
    let mut periods = 0;
    while !plant.touchdown() && periods < 1500 {
        plant.state().unwrap();
        plant.run_period(&[0.0; 4]).unwrap();
        periods += 1;
    }
    assert!(plant.touchdown(), "no touchdown in {periods} periods");

    plant.reset().unwrap();
    assert_eq!(plant.time(), 0.0, "time starts over");
    assert!(!plant.touchdown(), "touchdown flag cleared");

    // The same 3 s of commands after a fresh start and after a reset
    let u = [30.0, 0.002, -0.001, 0.0005];
    let first = fly(&mut plant, u, 300);
    plant.reset().unwrap();
    let second = fly(&mut plant, u, 300);
    assert_eq!(first, second, "a reset run differs from the first one");
    assert!(
        first.last().unwrap()[2] < -0.1,
        "the vehicle should have climbed"
    );
}
