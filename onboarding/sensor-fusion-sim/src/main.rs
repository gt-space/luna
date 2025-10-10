// fn average_pressure(a: f64 , b: f64) -> f64 {
//   (a + b) / 2.0
// }

// fn take_ownership(name: String) {
//   print!("(OWNED) sensor = {}\n", name);
// }

// fn borrow_name(name: &String) {
//   print!("(BORROWED) sensor = {}\n", name);
// }

// fn check_pressure (n: i32) {
//   if n > 60 {
//     print!("Nominal")
//   } else {
//     print!("Low")
//   }
// }

// struct SensorReading {
//   name: String,
//   value: f64,
//   unit: String,
// }

// enum ValveState {
//     Open,
//     Closed,
// }

// fn safe_pressure(value: f64) -> Result<f64, &'static str> {
//   if value < 0.0 {
//       Err("Negative pressure!")
//   } else {
//       Ok(value)
//   }
// }

// trait Sensor {
//   fn read(&self) -> f64;
// }

// struct PressureSensor {
//   value: f64,
// }

// impl Sensor for PressureSensor {
//   fn read(&self) -> f64 {
//       self.value
//   }
// }

// fn maybe_valve(active: bool) -> Option<u8> {
//   if active {
//       Some(12) // value exists
//   } else {
//       None // no value
//   }
// }

// fn main() {
// SENSORS AND AVERAGE
  // let engine_temp = 250;
  // let mut lox_temp = 150;
  // print!("Fuel Temp: {} \nLox Temp: {}", lox_temp, engine_temp);

  // let pressure = 10;
  // let valve_voltage = 20;
  // let active = 0;
  // print!("\nPressure: {} \nValve Voltage: {} \nActive Status: {}\n", pressure, valve_voltage, active);



  //print!("Average Pressure: {}\n",  average_pressure(101.64, 99.67))

// BORROWING AND OWNERSHIP
  // let pressure_sensor_name = "Pressure Sensor".to_string();
  // take_ownership(pressure_sensor_name);

  // let temp_sensor_name = "Temp Sensor".to_string();
  // borrow_name(&temp_sensor_name);

  // print!("after lending, temp_sensor_name is still here and can be used: {}", temp_sensor_name);

// IF ELSE
  // let test_value = 75;
  // check_pressure(test_value);

// STRUCT AND ENUM
  // let lox_temp = SensorReading {
  //   name: "Lox Temp".to_string(),
  //   value: 90.0,
  //   unit: "K".to_string(),
  // };
  // println!("{} = {} {}", lox_temp.name, lox_temp.value, lox_temp.unit);

  // let valve = ValveState::Closed;

  // match valve {
  //   ValveState::Open => print!("Valve is OPEN"),
  //   ValveState::Closed => println!("Valve is CLOSED"),
  // }

// ERROR HANDLING
  // match safe_pressure(5.0) {
  //   Ok(val) => println!("Pressure: {}", val),
  //   Err(e) => println!("Error: {}", e),
  // }

  // match safe_pressure(100.0) {
  //   Ok(val) => println!("Pressure: {}", val),
  //   Err(e) => println!("Error: {}", e),
  // }

 // TRAITS AND GENERICS
  // let t = PressureSensor { value: 300.0 };
  // println!("Sensor reading: {}", t.read());

// LIBRARY TOOLS 
  // let mut readings = Vec::new();
  //   readings.push(300.0);
  //   readings.push(310.5);
  //   readings.push(295.2);

  //   for r in readings {
  //       println!("Reading: {}", r);
  //   }

  //   match maybe_valve(true) {
  //     Some(v) => println!("Reading: {}", v),
  //     None => println!("Sensor is off"),
  // }

  // let lox = new_temp_sensor("LOX Temp".to_string(), 90.0);
  //   print_reading(&lox);
  //}




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


