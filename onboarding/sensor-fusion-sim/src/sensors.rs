//--------------------------
//Part 2
//--------------------------
pub struct SensorReading {
    pub name: String,
    pub value: f64,
    pub unit: String,
}

pub enum ValveState {
    Open,
    Closed,
}

pub fn new_temp_sensor(name: String, value: f64) -> SensorReading {
    SensorReading {
        name,
        value,
        unit: "K".to_string(),
    }
}

pub fn print_reading(sensor: &SensorReading) {
    println!("{}: {:.2}{}", sensor.name, sensor.value, sensor.unit)
}

pub fn print_valve_state(state: &ValveState) {
    match state {
        ValveState::Open => println!("Valve is OPEN"),
        ValveState::Closed => println!("Valve is CLOSED"),
    }
}

pub trait Sensor {
    fn read(&self) -> f64;
}

pub struct PressureSensor {
    pub name : String,
    pub pressure : f64,
}

impl Sensor for PressureSensor {
    fn read(&self) -> f64{
        self.pressure
    }
}

pub fn safe_pressure(value : f64) -> Result<f64, String> {
    if value < 0.0 {
        Ok(value)
    }
    else {
        Err("Negative pressure!".to_string())
    }
}

pub fn maybe_sensor(active: bool) -> Option<f64> {
    if active {
        Some(300.0)
    } else {
        None
    }
}

pub fn maybe_valve(active: bool) -> Option<u8>{
    if active {
        Some(12)
    }
    else {
        None
    }
}

pub fn identity<T>(x: T) -> T {
    x
}

//--------------------------
//Part 3
//--------------------------

use tokio::time::{sleep, Duration};
use rand::{Rng, SeedableRng}; // Thread-safe random number generation
use rand::rngs::StdRng;

pub async fn async_temp_sensor(name: &str, unit: &str) -> SensorReading {
    let mut rng = StdRng::from_entropy(); // random number generator
    let value: f64 = rng.gen_range(80.0..320.0); // pick random temp between 80 and 120
    sleep(Duration::from_millis(500)).await; // wait half a second
    SensorReading {
        name: name.to_string(),
        value,
	        unit: unit.to_string(),
    }
}

pub async fn async_pressure_sensor(name: &str, unit: &str) -> SensorReading {
    let mut rng = StdRng::from_entropy();
    let value: f64 = rng.gen_range(90.0..110.0); // random pressure
    sleep(Duration::from_millis(400)).await; // simulate delay
    SensorReading {
        name: name.to_string(),
        value,
        unit: unit.to_string(),
    }
}

// Simulate async valve sensor
pub async fn async_valve_sensor(_name: &str) -> ValveState {
    let mut rng = StdRng::from_entropy();
    let state = if rng.gen_bool(0.5) { // generate true with 0.5 probability
        ValveState::Open
    } else {
        ValveState::Closed
    };
    sleep(Duration::from_millis(300)).await; // simulate delay
    state
}