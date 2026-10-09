use rand::rngs::StdRng;
use rand::{Rng, SeedableRng}; // Thread-safe random number generation
use tokio::time::{Duration, sleep};

pub struct SensorReading {
    pub name: String,
    pub value: f64,
    pub unit: String,
}

pub enum ValveState {
    Open,
    Closed,
}

pub fn print_reading(reading: &SensorReading) {
    println!("{}: {:.2} {}", reading.name, reading.value, reading.unit);
}

pub fn print_valve_state(state: &ValveState) {
    let state = match state {
        ValveState::Open => "Open",
        ValveState::Closed => "Closed",
    };
    println!("Main Valve: {state}");
}

// Simulate async temperature sensor
pub async fn async_temp_sensor(name: &str, unit: &str) -> SensorReading {
    let mut rng = StdRng::from_entropy(); // random number generator
    let value: f64 = rng.gen_range(80.0..320.0); // pick a random temperature between 80 and 320
    sleep(Duration::from_millis(500)).await; // wait half a second
    SensorReading {
        name: name.to_string(),
        value,
        unit: unit.to_string(),
    }
}
// Simulate async pressure sensor
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
    let state = if rng.gen_bool(0.5) {
        // generate true with 0.5 probability
        ValveState::Open
    } else {
        ValveState::Closed
    };
    sleep(Duration::from_millis(300)).await; // simulate delay
    state
}
