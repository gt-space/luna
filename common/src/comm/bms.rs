use super::{flight::Ingestible, VehicleState};
use postcard::experimental::max_size::MaxSize;
use serde::{Deserialize, Serialize};
use std::fmt;

type Current = f64;
type Voltage = f64;

/// Describes the state of some power bus
#[derive(Copy, Clone, Default, MaxSize, Debug, Deserialize, PartialEq, Serialize, rkyv::Archive, rkyv::Serialize, rkyv::Deserialize)]
#[archive_attr(derive(bytecheck::CheckBytes))]
pub struct Bus {
  /// The voltage of the bus
  pub voltage: Voltage,
  /// The current of the bus
  pub current: Current,
}

/// Describes the state of some power rail
pub type Rail = Bus;

/// Represents the state of BMS as a whole
#[derive(
  MaxSize, Debug, Default, Deserialize, PartialEq, Serialize, Clone, Copy, rkyv::Archive, rkyv::Serialize, rkyv::Deserialize
)]
#[archive_attr(derive(bytecheck::CheckBytes))]
pub struct Bms {
  /// Batteru bus data
  pub battery_bus: Bus,
  /// Umbilical bus data
  pub umbilical_bus: Bus,
  /// Sam power bus data
  pub sam_power_bus: Bus,
  /// Five volt rail data
  pub five_volt_rail: Rail,
  /// Charger data
  pub charger: Current,
  /// Chassis voltage data
  pub chassis: Voltage,
  /// Estop voltage data
  pub e_stop: Voltage,
  /// RBF tag voltage data
  pub rbf_tag: Voltage,
}

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
  /// if the Sam Load Switch should be enabled
  SamLoadSwitch(bool),
  /// If the Estop reset sequence should be run
  ResetEstop,
}

impl fmt::Display for Command {
  fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
    match self {
      Self::Charge(value) => write!(f, "Set Charge to {}", value),
      Self::BatteryLoadSwitch(value) => {
        write!(f, "Set Battery Load Switch to {}", value)
      }
      Self::SamLoadSwitch(value) => {
        write!(f, "Set Sam Load Switch to {}", value)
      }
      Self::ResetEstop => write!(f, "Reset Estop"),
    }
  }
}
