use std::fmt;
use postcard::experimental::max_size::MaxSize;
use serde::{Serialize, Deserialize};
use super::{flight::Ingestible, VehicleState};

type Current = f64;
type Voltage = f64;

/// Describes the state of some power bus
#[derive(Copy, Clone, MaxSize, Debug, Deserialize, PartialEq, Serialize)]
pub struct Bus {
  pub voltage: Voltage,
  pub current: Current,
}

// impl Default for Bus {
//   fn default() -> Self {
//       Bus {
//         voltage: -999.0,
//         current: -999.0
//       }
//   }
// }

/// Describes the state of some power rail
pub type Rail = Bus;

/// Represents the state of BMS as a whole
#[derive(MaxSize, Debug, Deserialize, PartialEq, Serialize, Clone, Copy)]
pub struct Bms {
  pub battery_bus: Bus,
  pub umbilical_bus: Bus,
  pub sam_power_bus: Bus,
  pub five_volt_rail: Rail,
  pub charger: Current,
  pub e_stop: Voltage,
  pub rbf_tag: Voltage
}

/*
If there is an error in the data collection process, the default error values are
not replaced. If valid data is received the default values are replaced
 */
// impl Default for Bms {
//   fn default() -> Self {
//       Bms {
//         battery_bus: Bus::default(),
//         umbilical_bus: Bus::default(),
//         sam_power_bus: Bus::default(),
//         five_volt_rail: Rail::default(),
//         charger: -999.0,
//         e_stop: -999.0,
//         rbf_tag: -999.0
//       }
//   }
// }

/// Represents the current state of a device on the BMS.
/*#[derive(Deserialize, Serialize, Clone, MaxSize, Debug, PartialEq)]
pub enum Device {
  /// The state of the Battery Bus.
  BatteryBus(Bus),

  /// The state of the Umbilical Bus.
  UmbilicalBus(Bus),

  /// The state of the Sam Power Bus.
  SamPowerBus(Bus),

  /// The state of the 5v Rail.
  FiveVoltRail(Rail),

  /// The state of the Charger.
  Charger(Current),

  /// The state of the Estop.
  Estop(Voltage),

  /// The state of the RBFTag
  RBFTag(Voltage),
}*/

/// A single data point with a timestamp and channel, no units.
#[derive(Clone, Copy, Debug, Deserialize, MaxSize, PartialEq, Serialize)]
pub struct DataPoint {
  /// The state of the BMS.
  pub state: Bms,

  /// The timestamp of when this data was collected
  pub timestamp: f64,
}


impl Ingestible for DataPoint {
  fn ingest(&self, vehicle_state: &mut VehicleState) {
    vehicle_state.bms = self.state;
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