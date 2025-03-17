use super::{flight::Ingestible, VehicleState};
use postcard::experimental::max_size::MaxSize;
use serde::{Deserialize, Serialize};
use std::fmt;

type Current = f64;
type Voltage = f64;

/// Describes the state of some power bus
#[derive(
  Copy, Clone, Default, MaxSize, Debug, Deserialize, PartialEq, Serialize,
)]
pub struct Bus {
  pub voltage: Voltage,
  pub current: Current,
}

/// Describes the state of some power rail
pub type Rail = Bus;

/// Represents the state of the DBMS as a whole
#[derive(
  MaxSize, Debug, Default, Deserialize, PartialEq, Serialize, Clone, Copy,
)]
pub struct Dbms {
  pub battery_bus: Bus,
  pub five_volt_rail: Rail,
  pub rbf_tag: Voltage,
}

/// Represents the current state of a device on the DBMS.
/*#[derive(Deserialize, Serialize, Clone, MaxSize, Debug, PartialEq)]
pub enum Device {
  /// The state of the Battery Bus.
  BatteryBus(Bus),

  /// The state of the 5v Rail.
  FiveVoltRail(Rail),

  /// The state of the RBFTag
  RBFTag(Voltage),
}*/

/// A single data point with a timestamp and channel, no units.
#[derive(Clone, Copy, Debug, Deserialize, MaxSize, PartialEq, Serialize)]
pub struct DataPoint {
  /// The state of the DBMS.
  pub state: Dbms,

  /// The timestamp of when this data was collected
  pub timestamp: f64,
}

impl Ingestible for DataPoint {
  fn ingest(&self, vehicle_state: &mut VehicleState) {
    vehicle_state.dbms = self.state;
  }
}

/// Represents a command intended for the DBMS
#[derive(Clone, Debug, Deserialize, PartialEq, Serialize)]
pub enum Command {
  /// if Load Switch 1 should be enabled
  LoadSwitch1(bool),
  /// if Load Switch 2 should be enabled
  LoadSwitch2(bool),
  /// if Load Switch 3 should be enabled
  LoadSwitch3(bool),
  /// if Load Switch 4 should be enabled
  LoadSwitch4(bool),
}

impl fmt::Display for Command {
  fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
    match self {
      Self::LoadSwitch1(value) => {
        write!(f, "Set Load Switch 1 to {}", value)
      }
      Self::LoadSwitch2(value) => {
        write!(f, "Set Load Switch 2 to {}", value)
      }
      Self::LoadSwitch3(value) => {
        write!(f, "Set Load Switch 3 to {}", value)
      }
      Self::LoadSwitch4(value) => {
        write!(f, "Set Load Switch 4 to {}", value)
      }
    }
  }
}
