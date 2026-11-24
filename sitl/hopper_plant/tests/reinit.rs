//! A second `Plant::new` in the same process starts from t = 0 exactly like
//! the first one. The generated code's initialize() doesn't clear the model's
//! time, states or touchdown flag, so this guards `Plant::new` against
//! carrying a previous run over.
//!
//! Needs the plant (HOPPER_PLANT_CODE). In a stub build it fails, unless
//! HOPPER_PLANT_ALLOW_SKIP=1 says to skip it. Kept in its own test binary
//! because only one `Plant` can exist at a time.

use hopper_plant::{Error, Plant, State};

mod common;

/// Flies until touchdown (or `max` periods), returning every state and the
/// period count
fn fly(plant: &mut Plant, u: [f64; 4], max: usize) -> (Vec<State>, usize) {
    let mut states = Vec::new();
    while !plant.touchdown() && states.len() < max {
        states.push(plant.state().unwrap());
        plant.run_period(&u).unwrap();
    }
    let n = states.len();
    (states, n)
}

#[test]
fn a_second_plant_flies_the_same_as_the_first() {
    let u = [0.0; 4];

    let mut first = match Plant::new() {
        Ok(p) => p,
        Err(Error::NotBuilt) => {
            common::cannot_run("built without the plant (HOPPER_PLANT_CODE unset)");
            return;
        }
        Err(e) => panic!("{e}"),
    };
    assert_eq!(first.time(), 0.0);
    let (a, periods) = fly(&mut first, u, 1500);
    assert!(first.touchdown(), "no touchdown in {periods} periods");
    drop(first);

    let mut second = Plant::new().unwrap();
    assert_eq!(second.time(), 0.0, "the second plant starts at t = 0");
    assert!(!second.touchdown(), "the touchdown flag starts cleared");
    let (b, _) = fly(&mut second, u, 1500);
    assert_eq!(a, b, "the second plant's flight differs from the first");
}
