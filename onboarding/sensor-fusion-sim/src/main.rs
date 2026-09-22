// mod sensors;
// mod fusion;
// mod dashboard;

// use sensors::Sensor;

// fn main() {
//     //----------------------------------------------
//     //Part 2: Rust Basics
//     //----------------------------------------------
//     let engine_temp = 5;
//     let mut lox_temp = 5;
//     println!("{}", engine_temp);
//     println!("{}", lox_temp);

//     let pressure = 5;
//     let valve_voltage = 5;
//     let active = 5;
//     println!("{}", pressure);
//     println!("{}", valve_voltage);
//     println!("{}", active);

//     println!("{}", average_pressure(4.0, 6.0));

//     let sensor_name_1 = "Sensor Name 1".to_string();
//     take_ownership(sensor_name_1);

//     let sensor_name_2 = "Sensor Name 2".to_string();
//     borrow(&sensor_name_2);

//     let num = 60.0;
//     let closure = |num : f64| if num >= 60.0 {println!("Nominal")} else {println!("Low")};
//     closure(num);

//     let lox_temp = sensors::SensorReading {
//         name: "lox_temp".to_string(),
//         value: 90.0,
//         unit: "K".to_string(),
//     };

//     println!("{}: {}{}", lox_temp.name, lox_temp.value, lox_temp.unit);

//     let valve = sensors::ValveState::Closed;

//     sensors::print_valve_state(&valve);

//     let values = [5.0, 100.0];

//     for value in values {
//         match sensors::safe_pressure(value) {
//             Ok(val) => println!("Voltage: {}", val),
//             Err(e) => println!("Error: {}", e),
//         }
//     }

//     let pressure_sensor = sensors::PressureSensor {
//         name: "Pressure Sensor".to_string(),
//         pressure: 40.0,
//     };

//     println!("Pressure Sensor Reading: {}", pressure_sensor.read());

//     let mut engine_temp_readings = Vec::new();

//     engine_temp_readings.push(300.0);
//     engine_temp_readings.push(310.5);
//     engine_temp_readings.push(295.2);

//     for reading in engine_temp_readings {
//         println!("{}", reading);
//     }

//     let active = true;

//     match sensors::maybe_valve(active) {
//         Some(val) => println!("Valve Reading: {}", val),
//         None => println!("Sensor is not active"),
//     }

//     let lox_temp_sensor = sensors::new_temp_sensor("Lox temp sensor".to_string(), 400.0);
//     sensors::print_reading(&lox_temp_sensor);

//     //----------------------------------------------
//     // Part 2: Last step
//     //----------------------------------------------

//     // 1) Create and print a temperature sensor
//     let lox = sensors::new_temp_sensor("LOX Temp".to_string(), 90.0);
//     sensors::print_reading(&lox);

//     // 2) Valve state and match printing
//     let valve = sensors::ValveState::Closed;
//     sensors::print_valve_state(&valve);

//     // 3) safe_pressure result handling
//     match sensors::safe_pressure(5.0) {
//         Ok(v) => println!("safe_pressure(5.0) -> Ok({})", v),
//         Err(e) => println!("safe_pressure(5.0) -> Err({})", e),
//     }
//     match sensors::safe_pressure(-1.0) {
//         Ok(v) => println!("safe_pressure(-1.0) -> Ok({})", v),
//         Err(e) => println!("safe_pressure(-1.0) -> Err({})", e),
//     }

//     // 4) Trait implementation: PressureSensor
//     let p = sensors::PressureSensor {
//         name: "Fuel Tank".to_string(),
//         pressure: 101.3,
//     };
//     println!("PressureSensor.read() -> {}", p.read());

//     // 5) Option example
//     match sensors::maybe_sensor(true) {
//         Some(v) => println!("maybe_sensor(true) -> Some({})", v),
//         None => println!("maybe_sensor(true) -> None"),
//     }
//     match sensors::maybe_sensor(false) {
//         Some(v) => println!("maybe_sensor(false) -> Some({})", v),
//         None => println!("maybe_sensor(false) -> None"),
//     }

//     // 6) Generic identity function tests
//     println!("identity(42) -> {:?}", sensors::identity(42));
//     println!("identity(3.14) -> {:?}", sensors::identity(3.14));

//     // 7) Confirm placeholder modules are visible
//     fusion::placeholder();
//     dashboard::placeholder();
// }

// fn take_ownership(sensor_name: String) {
//     println!("{}", sensor_name);
// }

// fn borrow(sensor_name: &String) {
//     println!("{}", sensor_name);
// }

// fn average_pressure(a: f64, b: f64) -> f64{
//     (a + b) / 2.0
// }

//----------------------------------------------
// Part 3
//----------------------------------------------

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

        crate::dashboard::print_dashboard(&state);
    }
}