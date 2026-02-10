// src/main.rs
mod dashboard;
mod fusion;
mod sensors;

use fusion::FusionState;
use tokio::join;
use tokio::task;
use tokio::time::{Duration, sleep};

use crate::dashboard::print_dashboard;

#[tokio::main]
async fn main() {
    loop {
        // Spawn async sensor tasks
        let engine_temp_task = task::spawn(sensors::async_temp_sensor("Engine Temp", "K"));
        let lox_temp_task = task::spawn(sensors::async_temp_sensor("LOX Temp", "K"));
        let fuel_temp_task = task::spawn(sensors::async_temp_sensor("Fuel Temp", "K"));

        let lox_pressure_task =
            task::spawn(sensors::async_pressure_sensor("LOX Tank Pressure", "psi"));
        let fuel_pressure_task =
            task::spawn(sensors::async_pressure_sensor("Fuel Tank Pressure", "psi"));
        let pneumatics_task =
            task::spawn(sensors::async_pressure_sensor("Pneumatics Pressure", "psi"));

        let valve_task = task::spawn(sensors::async_valve_sensor("Main Valve"));

        // Add a timing task that enforces a 500 ms delay
        let timing_task = task::spawn(async {
            sleep(Duration::from_millis(500)).await;
        });

        // Collect results into FusionState
        let (
            engine_temp,
            lox_temp,
            fuel_temp,
            lox_pressure,
            fuel_pressure,
            pneumatics,
            valve_state,
            _,
        ) = join!(
            engine_temp_task,
            lox_temp_task,
            fuel_temp_task,
            lox_pressure_task,
            fuel_pressure_task,
            pneumatics_task,
            valve_task,
            timing_task // ensures loop always lasts at least 500 ms
        );

        let state = FusionState {
            engine_temp: engine_temp.unwrap(),
            lox_temp: lox_temp.unwrap(),
            fuel_temp: fuel_temp.unwrap(),
            lox_pressure: lox_pressure.unwrap(),
            fuel_pressure: fuel_pressure.unwrap(),
            pneumatics: pneumatics.unwrap(),
            valve: valve_state.unwrap(),
        };
        print_dashboard(&state);
    }
}
