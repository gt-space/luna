// src/fusion.rs
use crate::sensors::{SensorReading, ValveState};

/// FusionState combines all the sensor readings into one snapshot.
pub struct FusionState {
    pub engine_temp: SensorReading,
    pub lox_temp: SensorReading,
    pub fuel_temp: SensorReading,
    pub lox_pressure: SensorReading,
    pub fuel_pressure: SensorReading,
    pub pneumatics: SensorReading,
    pub valve: ValveState,
}

impl FusionState {
    /// Simple function to check safety
    pub fn check_safety(&self) -> bool {
        let temps_ok = self.engine_temp.value < 150.0
            && self.lox_temp.value < 150.0
            && self.fuel_temp.value < 150.0;

        let pressures_ok = self.lox_pressure.value > 50.0
            && self.fuel_pressure.value > 50.0
            && self.pneumatics.value > 50.0;

        let valve_ok = matches!(self.valve, ValveState::Open);

        temps_ok && pressures_ok && valve_ok
    }
}