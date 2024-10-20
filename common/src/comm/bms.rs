use core::fmt;

use postcard::experimental::max_size::MaxSize;
use serde::{Serialize, Deserialize};

use super::{sam::Unit, Measurement, VehicleState};

/// Represents the current state of a device on the BMS.
#[derive(Deserialize, Serialize, Clone, MaxSize, Debug, PartialEq)]
pub enum Device {
  /// The state of the Battery Bus.
  BatteryBus {
    /// The voltage of the Battery Bus.
    voltage: f64,

    /// The current of the Battery Bus.
    current: f64,
  },

  /// The state of the Umbilical Bus.
  UmbilicalBus{
    /// The voltage of the Umbilical Bus.
    voltage: f64,

    /// The current of the Umbilical Bus.
    current: f64,
  },

  /// The state of the Charger.
  Charger {
    /// The current of the Charger.
    current: f64,
  },

  /// The state of the Sam Power Bus.
  SamPowerBus {
    /// The voltage of the Sam Power Bus.
    voltage: f64,

    /// The voltage of the Sam Power Bus.
    current: f64,
  },

  /// The state of the 5v Rail.
  FiveVoltRail {
    /// The voltage of the 5v Rail.
    voltage: f64,

    /// The current of the 5v Rail.
    current: f64,
  },

  /// The state of the Estop.
  Estop {
    /// 3.3v if the Estop is disabled, 0.0v if it isn't
    voltage: f64
  },

  /// The state of the RBFTag
  RBFTag {
    /// 3.3v if the RBFTag is disabled, 0.0v if it isn't
    voltage: f64
  },
}

impl fmt::Display for Device {
  fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
    match self {
      Self::BatteryBus { .. } => write!(f, "BatteryBus"),
      Self::Charger { .. } => write!(f, "Charger"),
      Self::Estop { .. } => write!(f, "Estop"),
      Self::FiveVoltRail { .. } => write!(f, "5VRail"),
      Self::RBFTag { .. } => write!(f, "RBFTag"),
      Self::UmbilicalBus { .. } => write!(f, "UmbilicalBus"),
      Self::SamPowerBus { .. } => write!(f, "SamPowerBus"),
    }
  }
}

/// A single data point with a timestamp and channel, no units.
#[derive(Clone, Debug, Deserialize, MaxSize, PartialEq, Serialize)]
pub struct DataPoint {
  /// The state of some device on the BMS.
  pub device: Device,

  /// The timestamp of when this data was collected
  pub timestamp: f64,
}

/// Defines how some data coming into the flight computer should be processed
pub trait Ingestible {
  /// Using the data from self, update the vehicle_state
  fn ingest(&self, vehicle_state: &mut VehicleState);
}


impl Ingestible for DataPoint {
  fn ingest(&self, vehicle_state: &mut VehicleState) {
    // voltage
    match self.device {
      Device::BatteryBus { voltage, .. } |
      Device::UmbilicalBus { voltage, .. } |
      Device::SamPowerBus { voltage, .. } |
      Device::FiveVoltRail { voltage, .. } |
      Device::Estop { voltage } |
      Device::RBFTag { voltage } => {
        vehicle_state.sensor_readings.insert(
          format!("{}_V", self.device), 
          Measurement { value: voltage, unit: Unit::Volts }
        );
      }
      Device::Charger { .. } => {}
    }

    // current
    match self.device {
      Device::BatteryBus { current, ..  } |
      Device::UmbilicalBus { current, ..  } |
      Device::SamPowerBus { current, ..  } |
      Device::FiveVoltRail { current, ..  } |
      Device::Charger { current } => {
        vehicle_state.sensor_readings.insert(
          format!("{}_I", self.device), 
          Measurement { value: current, unit: Unit::Amps }
        );
      }
      Device::Estop { .. } | Device::RBFTag { .. } => {}
    }
  }
}

/// Represents a command intended for the BMS
#[derive(Clone, Debug, Deserialize, PartialEq, Serialize)]
pub enum Command {
  /// If charging should be enabled 
  Charge(bool),
  /// if the Battery Load Switch should be enabled
  BatteryLoadSwitch(bool),
  /// if the Battery Load Switch should be enabled
  SamLoadSwitch(bool),
  /// If the Estop should be reset
  ResetEstop
}

impl fmt::Display for Command {
  fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
      match self {
        Self::Charge(value) => write!(f, "Set Charge to {}", value),
        Self::BatteryLoadSwitch(value) => write!(f, "Set Battery Load Switch to {}", value),
        Self::SamLoadSwitch(value) => write!(f, "Set Sam Load Switch to {}", value),
        Self::ResetEstop => write!(f, "Reset Estop")
      }
  }
}