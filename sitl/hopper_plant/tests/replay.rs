//! The Rust wrapper steps the plant exactly like plant_replay.c and the
//! Simulink replay in verify_plant_c.m.
//!
//! Needs a verify_plant_c run whose plant this crate was built from:
//!   HOPPER_PLANT_CODE=<hopper fsw_bridge>/build/verify/plant
//!   HOPPER_PLANT_REPLAY=<hopper fsw_bridge>/build/verify   (commands.csv,
//! states_c.csv) Without either the test fails, unless
//! HOPPER_PLANT_ALLOW_SKIP=1 says to skip it.

use std::{env, fs, path::PathBuf};

use hopper_plant::{Error, Plant};

mod common;

fn rows(path: PathBuf) -> Vec<Vec<f64>> {
    fs::read_to_string(&path)
        .unwrap_or_else(|e| panic!("{}: {e}", path.display()))
        .lines()
        .filter(|l| !l.trim().is_empty())
        .map(|l| l.split(',').map(|v| v.trim().parse().unwrap()).collect())
        .collect()
}

#[test]
fn matches_the_c_replay() {
    let Some(dir) = env::var_os("HOPPER_PLANT_REPLAY").map(PathBuf::from) else {
        common::cannot_run("HOPPER_PLANT_REPLAY is not set");
        return;
    };
    let commands = rows(dir.join("commands.csv"));
    let states = rows(dir.join("states_c.csv"));
    let mut plant = match Plant::new() {
        Ok(p) => p,
        Err(Error::NotBuilt) => {
            common::cannot_run("built without the plant (HOPPER_PLANT_CODE unset)");
            return;
        }
        Err(e) => panic!("{e}"),
    };
    let mut compared = 0;
    for (k, (cmd, want)) in commands.iter().zip(&states).enumerate() {
        let x = plant.state().unwrap();
        assert_eq!(plant.time(), want[0], "time at command {k}");
        for i in 0..13 {
            assert_eq!(
                x[i],
                want[1 + i],
                "state {i} at command {k} (t = {})",
                want[0]
            );
        }
        compared += 1;
        if want[14] != 0.0 {
            break; // the C replay stopped at touchdown
        }
        plant.run_period(&[cmd[1], cmd[2], cmd[3], cmd[4]]).unwrap();
    }
    assert!(compared > 100, "only {compared} commands compared");
    eprintln!("{compared} commands bit-identical to plant_replay.c");
}
