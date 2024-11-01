use std::fmt;
use postcard::experimental::max_size::MaxSize;
use serde::{Serialize, Deserialize};
use super::{flight::Ingestible, VehicleState};

#[derive(Copy, Clone, MaxSize, Debug, Default, Deserialize, PartialEq, Serialize)]
pub struct Bus {
  voltage: f64,
  current: f64,
}

pub type Rail = Bus;
type Current = f64;
type Voltage = f64;

#[derive(MaxSize, Debug, Default, Deserialize, PartialEq, Serialize, Clone)]
pub struct Bms {
  battery_bus: Bus,
  umbilical_bus: Bus,
  sam_power_bus: Bus,
  five_volt_rail: Rail,
  charger: Current,
  e_stop: Voltage,
  rbf_tag: Voltage
}

/// Represents the current state of a device on the BMS.
#[derive(Deserialize, Serialize, Clone, MaxSize, Debug, PartialEq)]
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
}

/// A single data point with a timestamp and channel, no units.
#[derive(Clone, Debug, Deserialize, MaxSize, PartialEq, Serialize)]
pub struct DataPoint {
  /// The state of some device on the BMS.
  pub device: Device,

  /// The timestamp of when this data was collected
  pub timestamp: f64,
}


impl Ingestible for DataPoint {
  fn ingest(&self, vehicle_state: &mut VehicleState) {
    match self.device {
      Device::BatteryBus(bus) => vehicle_state.bms.battery_bus = bus,
      Device::UmbilicalBus(bus) => vehicle_state.bms.umbilical_bus = bus,
      Device::SamPowerBus(bus) => vehicle_state.bms.sam_power_bus = bus,
      Device::FiveVoltRail(rail) => vehicle_state.bms.five_volt_rail = rail,
      Device::Charger(current) => vehicle_state.bms.charger = current,
      Device::Estop(voltage) => vehicle_state.bms.e_stop = voltage,
      Device::RBFTag(voltage) => vehicle_state.bms.rbf_tag = voltage
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