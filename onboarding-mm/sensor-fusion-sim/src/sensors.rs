// src/sensors.rs
use tokio::time::{sleep, Duration};
use rand::{Rng, SeedableRng};
use rand::rngs::StdRng;    
    
    pub struct SensorReading {
        name: String,
        pub(crate) value: f64,
        unit: String,
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
    println!("{}: {}{}", sensor.name, sensor.value, sensor.unit);
}

pub fn print_valve_state(state: &ValveState) {
    match state {
        ValveState::Open => println!("Valve is OPEN"),
        ValveState::Closed => println!("Valve is CLOSED"),
    }
}

    pub(crate) trait Sensor {
        fn read(&self) -> f64;
    }

    pub struct PressureSensor {
        pub name: String,
        pub pressure: f64,
    }

    impl Sensor for PressureSensor {
        fn read(&self) -> f64 {
            self.pressure
        }
    }

pub fn safe_pressure(v: f64) -> Result<f64, String> {
    if v < 0.0 {
        Err("Negative pressure!".to_string())
    } else {
        Ok(v)
    }
}

 pub(crate) fn maybe_sensor(active: bool) -> Option<u8> {
        if active {
            Some(12)
        } else {
            None
        }
    }

    pub fn identity<T>(x: T) -> T {
        x
    }   

pub async fn async_temp_sensor(name: &str, unit: &str) -> SensorReading {
    let mut rng = StdRng::from_entropy(); 
    let value: f64 = rng.gen_range(80.0..320.0);
    sleep(Duration::from_millis(500)).await; 
    SensorReading {
        name: name.to_string(),
        value,
        unit: unit.to_string(),
    }
}

pub async fn async_pressure_sensor(name: &str, unit: &str) -> SensorReading {
    let mut rng = StdRng::from_entropy();
    let value: f64 = rng.gen_range(90.0..110.0); 
    sleep(Duration::from_millis(400)).await; 
    SensorReading {
        name: name.to_string(),
        value,
        unit: unit.to_string(),
    }
}


pub async fn async_valve_sensor(_name: &str) -> ValveState {
    let mut rng = StdRng::from_entropy();
    let state = if rng.gen_bool(0.5) {
        ValveState::Open
    } else {
        ValveState::Closed
    };
    sleep(Duration::from_millis(300)).await; 
    state
}