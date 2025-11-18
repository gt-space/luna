mod dashboard;
mod fusion;
mod sensors; //bring in the sensors module into scope
use crate::sensors::Sensor;
use fusion::FusionState;

use tokio::join;
use tokio::task;
use tokio::time::{Duration, sleep};

#[tokio::main]
async fn main() {
    loop {
        let engine_temp_task = task::spawn(sensors::async_temp_sensor("Engine Temp", "K"));
        let lox_temp_task = task::spawn(sensors::async_temp_sensor("LOX Temp:", "K"));
        let fuel_temp_task = task::spawn(sensors::async_temp_sensor("Fuel Temp:", "K"));

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

        // Await all results together
        let (
            engine_temp,
            lox_temp,
            fuel_temp,
            lox_pressure,
            fuel_pressure,
            pneumatics,
            valve_state,
            timine_task,
        ) = tokio::join!(
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

    // Spawn async sensor tasks

    // //Print results
    // sensors::print_reading(&engine_temp.unwrap());
    // sensors::print_reading(&lox_temp.unwrap());
    // sensors::print_reading(&fuel_temp.unwrap());
    // sensors::print_reading(&lox_pressure.unwrap());
    // sensors::print_reading(&fuel_pressure.unwrap());
    // sensors::print_reading(&pneumatics.unwrap());
    // sensors::print_valve_state(&valve_state.unwrap());

    // // Print everything in a fused way
    // println!("--- Fusion State ---");
    // sensors::print_reading(&state.engine_temp);
    // sensors::print_reading(&state.lox_temp);
    // sensors::print_reading(&state.fuel_temp);
    // sensors::print_reading(&state.lox_pressure);
    // sensors::print_reading(&state.fuel_pressure);
    // sensors::print_reading(&state.pneumatics);
    // sensors::print_valve_state(&state.valve);

    // // Safety check
    // if state.check_safety() {
    //     println!("✅ All systems nominal. Safe to proceed.");
    // } else {
    //     println!("⚠️ Warning: Unsafe conditions detected!");
    // }

    // ln!("safe_pressure(-1.0) -> Ok({})", v),
    //     Err(e) => pr// 1) Create and print a new temperature sensor
    // let lox = sensors::new_temp_sensor("LOX Temp".to_string(), 90.0);

    // sensors::print_reading(&lox);

    // // 2) Valve state and match printing
    // let valve = sensors::ValveState::Closed;
    // sensors::print_valve_state(&valve);

    // // 3) safe_pressure result handling
    // match sensors::safe_pressure(5.0) {
    //     Ok(v) => println!("safe_pressure(5.0) -> Ok({})", v),
    //     Err(e) => println!("safe_pressure(5.0) -> Err({})", e),
    // }
    // match sensors::safe_pressure(-1.0) {
    //     Ok(v) => printintln!("safe_pressure(-1.0) -> Err({})", e),
    // }

    // // 4) Trait implementation: PressureSensor
    // let p = sensors::PressureSensor {
    //     name: "Fuel Tank".to_string(),
    //     pressure: 101.3,
    // };
    // println!("PressureSensor.read() -> {}", p.read());

    // // 5) Option example
    // match sensors::maybe_sensor(true) {
    //     Some(v) => println!("maybe_sensor(true) -> Some({})", v),
    //     None => println!("maybe_sensor(true) -> None"),
    // }
    // match sensors::maybe_sensor(false) {
    //     Some(v) => println!("maybe_sensor(false) -> Some({})", v),
    //     None => println!("maybe_sensor(false) -> None"),
    // }

    // // 6) Generic identity function tests
    // println!("identity(42) -> {:?}", sensors::identity(42));
    // println!("identity(3.14) -> {:?}", sensors::identity(3.14));

    // // 7) Confirm placeholder modules are visible
    // fusion::placeholder();
    // dashboard::placeholder();

    // let engine_temp = 5;
    // let mut lox_temp = 10;
    // println!("engine temp: {}\nlox temp: {}", engine_temp, lox_temp);

    // let pressure: f32 = 101.3;
    // let valve_voltage: u8 = 12;
    // let active: bool = true;
    // println!("{}", pressure);
    // println!("{}", valve_voltage);
    // println!("{}", active);

    // fn average_pressure(a: f64, b: f64) -> f64 {
    //     (a+b)/2.0
    // }
    // let result = average_pressure(2.0, 3.0);
    // println!("{}", result);

    // let name = "sensorname".to_string();
    // let name2 = "sensorname2".to_string();
    // take_ownership(name);
    // borrow(&name2);
    // fn take_ownership(sname: String) {
    //     println!("ownership taken: {}", sname);
    // }
    // fn borrow(sname2: &String) {
    //     println!("just borrowed!: {}", sname2);
    // }
    // println!("{}", name2);

    // fn random(num: u8) {
    //     if num > 60 {
    //         println!("Normal");
    //     } else {
    //         println!("Low");
    //     }
    // }
    // random(61);

    // let engine_temp = SensorReading {
    //     name: "Engine Temp".to_string(),
    //     value: 300.5,
    //     unit: "K".to_string(),
    // };
    // println!("{}: {}{}", engine_temp.name, engine_temp.value, engine_temp.unit);

    // let valve = ValveState::Open;
    // match valve {
    //     ValveState::Open => println!("Valve is OPEN"),
    //     ValveState::Closed => println!("Valve is CLOSED")
    // }

    //access struct's field by: [name of struct instance].[field name]

    // struct SensorReading {
    //     name: String,
    //     value: f64,
    //     unit: String,
    // }
    // let lox_temp = SensorReading {
    //     name: "LOX Temperature".to_string(),
    //     value: 90.0,
    //     unit: "K".to_string(),
    // };
    // enum ValveState {
    //     Open,
    //     Closed,
    // }
    // let valve = ValveState::Closed;
    // match valve {
    //     ValveState::Open => println!("OPEN"),
    //     ValveState::Closed => println!("CLOSED"),
    // }
    // println!("{}: {}{}", lox_temp.name, lox_temp.value, lox_temp.unit);

    // fn read_voltage(v: f64) -> Result<f64, String> {
    //     if v < 0.0 {
    //         Err("Invalid voltage".to_string())
    //     } else {
    //         Ok(v)
    //     }
    // }
    // match read_voltage(-5.0) {
    //     Ok(val) => println!("Voltage: {}", val),
    //     Err(e) => println!("Error: {}", e),
    // }

    // fn safe_pressure(value: f64) -> Result<f64, String>{
    //     if value < 0.0 {
    //         Err("Negative pressure!".to_string())
    //     } else {
    //         Ok(value)
    //     }
    // }
    // match safe_pressure(5.0) {
    //     Ok(v) => println!("Pressure: {}", v),
    //     Err(e) => println!("Error: {}", e),
    // }

    // struct PressureSensor {
    //     value: f64,
    // }
    // impl Sensor for PressureSensor {
    //     fn read(&self) -> f64 { //self refers to the instance of pressure sensor
    //         self.value
    //     }
    // }
    // let p = PressureSensor { value: 300.0};
    // println!("Sensor reading: {}", p.read()); //self doesn't need to be passed in, the function will reference p

    // let mut readings = Vec::new();
    // readings.push(300.0);
    // readings.push(310.5);
    // readings.push(295.2);
    // for r in readings {
    //     println!("reading: {}", r);
    // }

    // match maybe_valve(true) {
    //     Some(v) => println!("Reading: {}", v),
    //     None => println!("Sensor if off"),
    // }
}
