
#[allow(unused_variables)]

mod sensors;
mod fusion;
mod dashboard;

use tokio::join;
use tokio::task;
use tokio::time::{sleep, Duration};
use fusion::FusionState;

// What's this for? This is an attribute macro telling the compiler that main will use tokio's async runtime.
#[tokio::main] // A link to learn more about macros is later in the section.
async fn main() {
    loop {
        // Spawn async sensor tasks
        let engine_temp_task = task::spawn(sensors::async_temp_sensor("Engine Temp", "K"));
        let lox_temp_task = task::spawn(sensors::async_temp_sensor("LOX Temp", "K"));
        let fuel_temp_task = task::spawn(sensors::async_temp_sensor("Fuel Temp", "K"));

        let lox_pressure_task = task::spawn(sensors::async_pressure_sensor("LOX Tank Pressure", "psi"));
        let fuel_pressure_task = task::spawn(sensors::async_pressure_sensor("Fuel Tank Pressure", "psi"));
        let pneumatics_task = task::spawn(sensors::async_pressure_sensor("Pneumatics Pressure", "psi"));

        let valve_task = task::spawn(sensors::async_valve_sensor("Main Valve"));

        // Add a timing task that enforces a 500 ms delay
        let timing_task = task::spawn(async {
            sleep(Duration::from_millis(500)).await;
        });

        // Await all results together
        let (engine_temp, lox_temp, fuel_temp, lox_pressure, fuel_pressure, pneumatics, valve_state) =
            join!(
                engine_temp_task,
                lox_temp_task,
                fuel_temp_task,
                lox_pressure_task,
                fuel_pressure_task,
                pneumatics_task,
                valve_task
            );

        // Build a FusionState struct
        let state = FusionState {
            engine_temp: engine_temp.unwrap(),
            lox_temp: lox_temp.unwrap(),
            fuel_temp: fuel_temp.unwrap(),
            lox_pressure: lox_pressure.unwrap(),
            fuel_pressure: fuel_pressure.unwrap(),
            pneumatics: pneumatics.unwrap(),
            valve: valve_state.unwrap(),
        };


        // Print the dashboard view
        dashboard::print_dashboard(&state);
    }
}


