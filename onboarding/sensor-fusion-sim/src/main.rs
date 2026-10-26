mod dashboard;
mod fusion;
mod sensors;

use fusion::FusionState;
use tokio::join;
use tokio::task;
use tokio::time::{Duration, sleep};

#[tokio::main]
async fn main() -> Result<(), tokio::task::JoinError> {
    loop {
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

        let timing_task = task::spawn(async {
            sleep(Duration::from_millis(500)).await;
        });

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
            timing_task
        );

        let state = FusionState {
            engine_temp: engine_temp?,
            lox_temp: lox_temp?,
            fuel_temp: fuel_temp?,
            lox_pressure: lox_pressure?,
            fuel_pressure: fuel_pressure?,
            pneumatics: pneumatics?,
            valve: valve_state?,
        };

        dashboard::print_dashboard(&state);
    }
}
