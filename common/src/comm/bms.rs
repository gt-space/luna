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
  /// voltage data
  pub voltage: Voltage,
  /// current data
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
  /// battery bus data
  pub battery_bus: Bus,
  /// umbilical bus data
  pub umbilical_bus: Bus,
  /// sam power bus data
  pub sam_power_bus: Bus,
  /// ethernet load switch bus data
  pub ethernet_bus: Bus,
  /// tel load switch bus data
  pub tel_bus: Bus,
  /// flight computer board bus data
  pub fcb_bus: Bus,
  /// 5v rail data
  pub five_volt_rail: Rail,
  /// charger data
  pub charger: Current,
  /// chassis data
  pub chassis: Voltage,
  /// estop data
  pub e_stop: Voltage,
  /// rbf tag data
  pub rbf_tag: Voltage,
  /// reco load switch 1 data
  pub reco_load_switch_1: Voltage,
  /// reco load switch 2 data
  pub reco_load_switch_2: Voltage,  
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
  /// if the TEL load switch should be enabled
  TelLoadSwitch(bool),
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
      Self::TelLoadSwitch(value) => write!(f, "Set Tel Load Switch to {}", value),
    }
  }
}
