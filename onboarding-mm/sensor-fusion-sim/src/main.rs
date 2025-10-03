// src/main.rs
mod sensors;
mod fusion;
mod dashboard;

use tokio::join;
use tokio::task;
use tokio::time::{sleep, Duration};
use fusion::FusionState;

#[tokio::main]
async fn main() {
    loop {
        let engine_temp_task = task::spawn(sensors::async_temp_sensor("Engine Temp", "K"));
        let lox_temp_task = task::spawn(sensors::async_temp_sensor("LOX Temp", "K"));
        let fuel_temp_task = task::spawn(sensors::async_temp_sensor("Fuel Temp", "K"));

        let lox_pressure_task = task::spawn(sensors::async_pressure_sensor("LOX Tank Pressure", "psi"));
        let fuel_pressure_task = task::spawn(sensors::async_pressure_sensor("Fuel Tank Pressure", "psi"));
        let pneumatics_task = task::spawn(sensors::async_pressure_sensor("Pneumatics Pressure", "psi"));

        let valve_task = task::spawn(sensors::async_valve_sensor("Main Valve"));
        let timing_task = task::spawn(async {
            sleep(Duration::from_millis(500)).await;
        });
        let (engine_temp, lox_temp, fuel_temp, lox_pressure, fuel_pressure, pneumatics, valve_state, _) =
            join!(
                engine_temp_task,
                lox_temp_task,
                fuel_temp_task,
                lox_pressure_task,
                fuel_pressure_task,
                pneumatics_task,
                valve_task,
                timing_task
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
        dashboard::print_dashboard(&state);
    }
}