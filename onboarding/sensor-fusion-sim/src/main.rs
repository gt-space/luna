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

        // Collect results into FusionState
        let (engine_temp, lox_temp, fuel_temp, lox_pressure, fuel_pressure, pneumatics, valve_state, _) =
            join!(
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

        // Print the dashboard view
        dashboard::print_dashboard(&state);
    }
}


//Main.rs for fusion - Kept for personal records
//--------------------------------------------------------------------------------------
// use fusion::FusionState;
// mod sensors;
// mod fusion;
// mod dashboard;

// use tokio::join;
// use tokio::task;

// // What's this for? This is an attribute macro telling the compiler that main will use tokio's async runtime.
// #[tokio::main] // A link to learn more about macros is later in the section.
// async fn main() {
//     // Spawn async sensor tasks
//     let engine_temp_task = task::spawn(sensors::async_temp_sensor("Engine Temp", "K"));
//     let lox_temp_task = task::spawn(sensors::async_temp_sensor("LOX Temp", "K"));
//     let fuel_temp_task = task::spawn(sensors::async_temp_sensor("Fuel Temp", "K"));

//     let lox_pressure_task = task::spawn(sensors::async_pressure_sensor("LOX Tank Pressure", "psi"));
//     let fuel_pressure_task = task::spawn(sensors::async_pressure_sensor("Fuel Tank Pressure", "psi"));
//     let pneumatics_task = task::spawn(sensors::async_pressure_sensor("Pneumatics Pressure", "psi"));

//     let valve_task = task::spawn(sensors::async_valve_sensor("Main Valve"));

//     // Await all results together
//     let (engine_temp, lox_temp, fuel_temp, lox_pressure, fuel_pressure, pneumatics, valve_state) =
//         join!(
//             engine_temp_task,
//             lox_temp_task,
//             fuel_temp_task,
//             lox_pressure_task,
//             fuel_pressure_task,
//             pneumatics_task,
//             valve_task
//         );

//     // Print results
//     // sensors::print_reading(&engine_temp.unwrap());
//     // sensors::print_reading(&lox_temp.unwrap());
//     // sensors::print_reading(&fuel_temp.unwrap());
//     // sensors::print_reading(&lox_pressure.unwrap());
//     // sensors::print_reading(&fuel_pressure.unwrap());
//     // sensors::print_reading(&pneumatics.unwrap());
//     // sensors::print_valve_state(&valve_state.unwrap());

//     // Build a FusionState struct
//     let state = FusionState {
//         engine_temp: engine_temp.unwrap(),
//         lox_temp: lox_temp.unwrap(),
//         fuel_temp: fuel_temp.unwrap(),
//         lox_pressure: lox_pressure.unwrap(),
//         fuel_pressure: fuel_pressure.unwrap(),
//         pneumatics: pneumatics.unwrap(),
//         valve: valve_state.unwrap(),
//     };

//     // Print everything in a fused way
//     println!("--- Fusion State ---");
//     sensors::print_reading(&state.engine_temp);
//     sensors::print_reading(&state.lox_temp);
//     sensors::print_reading(&state.fuel_temp);
//     sensors::print_reading(&state.lox_pressure);
//     sensors::print_reading(&state.fuel_pressure);
//     sensors::print_reading(&state.pneumatics);
//     sensors::print_valve_state(&state.valve);

//     // Safety check
//     if state.check_safety() {
//         println!("✅ All systems nominal. Safe to proceed.");
//     } else {
//         println!("⚠️ Warning: Unsafe conditions detected!");
//     }
// }

//V1 of Main.rs
//-------------------------------------------------------------------------------------------------------

// // src/main.rs
// mod sensors; // bring the sensors module into scope
// mod fusion;
// mod dashboard;
// use crate::sensors::Sensor;

// fn main() {
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



//Learning stage main.rs
//---------------------------------------------------------------------------------------



// mod sensors;

// fn main() {
//     println!("Hello, world!");
//     let engine_temp = 100.0;
//     let lox_temp = 150.8;
//     println!("Engine temp: {}", engine_temp);
//     println!("Lox temp: {}", lox_temp);

//     let pressure: f64 = 101.3;
//     let valve_voltage: u8 = 12;         //The maxmimum value for a u8 is 255
//     let active: bool = true;

//     println!("Pressure: {}", pressure);
//     println!("Valve Voltage: {}", valve_voltage);
//     println!("Is active: {}", active);

//     //Function variable type testing;
//     let result = average_pressure(125.7, 50.2);
//     println!("Average pressure: {}", result);

//     //Ownership vs borrowing of an element
//     let sensor = "Engine Temp Sensor".to_string();
//     take_ownership(sensor);
//     let borrow_sensor = "Lox Temp Sensor".to_string();
//     borrow_element(&borrow_sensor);

//     let test_value: i32 = 75;
//     //0..3 is a range - as in all values 0,1,2,3 will be passed

//     check_pressure(test_value);

//     //Learn about implementing a defined struct
//     // let lox_temp = SensorReading {
//     //     name: "lox_temp".to_string(),
//     //     value: 90.00,
//     //     unit: "K".to_string(),
//     // };
//     // println!("{} : {}{}", lox_temp.name, lox_temp.value, lox_temp.unit);

//     //Implemeting enums (I moved away ValveState and SensorReading so this code won't work.)
//     // let valve = ValveState::Open;

//     // match valve {
//     //     ValveState::Open => println!("Vale is OPEN"),
//     //     ValveState::Closed => println!("Valve is CLOSED"),
//     // }

//     //Error Handling
//     match safe_pressure(5.0) {
//         Err(e) => println!("Error!: {}", e),
//         Ok(v) => println!("Pressure: {}", v),
//     }
//     match safe_pressure(100.0) {
//         Err(e) => println!("Error!: {}", e),
//         Ok(v) => println!("Pressure: {}", v),
//     }    

//     //Traits and Generics General
//     //Traits are like interfaces: they define behavior that multiple types can share.
//     trait SensorDemo{
//         fn read(&self) -> f64;
//     }

//     struct TempSensor{
//         value: f64,
//     }

//     impl SensorDemo for TempSensor {    //Here impl serves as the implements keyword
//         fn read(&self) -> f64{      //I think this is a read function that can return a read sensor value
//             self.value
//         }
//     }

//     //Traits and Generics Checkpoint
//     trait Sensor{
//         fn read(&self) -> f64;
//     }
//     struct PressureSensor{
//         value: f64,
//     }
//     impl Sensor for PressureSensor{
//         fn read(&self) -> f64{
//             self.value
//         }
//     }
//     let my_pressure_sense = PressureSensor{value: 124.0};
//     println!("Pressure Sensor Reading: {}", my_pressure_sense.read());

//     let mut engine_readings = Vec::new();
//     engine_readings.push(300.0);
//     engine_readings.push(310.5);
//     engine_readings.push(295.2);

//     for reading in engine_readings {
//         println!("Reading: {}", reading);
//     }

//     //Options
//     match maybe_valve(true) {
//         Some(v) => println!("Reading: {}", v),
//         None => println!("Sensor is off."),
//     }

//     //Modularity
//     let LOX_temp_sensor = sensors::new_temp_sensor("LOX temp".to_string(), 100.0);
//     sensors::print_reading(&LOX_temp_sensor);

// }

// fn take_ownership(sensor: String) {
//     println!("Ownership taken: {}", sensor);
// }

// fn borrow_element(sensor: &String) {
//     println!("Element borrowed: {}", sensor);
// }

// fn average_pressure(a: f64, b: f64) -> f64 {
//     (a+b)/2.0
// }


// //Control Flow
// fn check_pressure(pressure: i32) {
//     if pressure > 60 {
//         println!("Nominal");
//     } else {
//         println!("Low");
//     }
// }

// //Error Handling 
// fn safe_pressure(value: f64) -> Result<f64, String> {
//     if value < 0.00 {
//         Err("Negative pressure!".to_string())
//     } else {
//         Ok(value)
//     }
// }

// //Standard Library Tools - Option
// fn maybe_valve(active: bool) -> Option<u8> {
//     if(active) {
//         Some(12)
//     } else {
//         None
//     }
// }