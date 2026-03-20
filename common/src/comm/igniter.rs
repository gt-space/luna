use super::{
  Bus,
  Rail,
  Current,
  Voltage,
};
use postcard::experimental::max_size::MaxSize;
use serde::{Deserialize, Serialize};
use std::fmt;

/// Represents the state of Igniter as a whole
#[derive(
  MaxSize, Debug, Default, Deserialize, PartialEq, Serialize, Clone, Copy, 
  rkyv::Archive, rkyv::Serialize, rkyv::Deserialize
)]
#[archive_attr(derive(bytecheck::CheckBytes))]
pub struct Igniter {
  /// 5v0 rail
  pub p5v0_rail: Rail,
  /// Configurable rail that can be configured to use different voltages
  pub config_rail: Rail,
  /// 24v0 rail
  pub p24v0_rail: Rail,
  /// Constant Voltage (channels 0-2) and Constant Current (channels 3-5) buses.
  pub channels: [Bus; 6],
  /// Continuity readings across all 6 channels
  pub continuity: [Current; 6],
  /// Constant current fault status across all 6 channels
  pub cc_faults: [u8; 6],
  /// Igniter RBF
  pub rbf: u8,
}

/// A single data point with a timestamp and channel, no units.
#[derive(Clone, Copy, Debug, Deserialize, MaxSize, PartialEq, Serialize)]
pub struct DataPoint {
  /// The state of the Igniter.
  pub state: Igniter,

  /// The timestamp of when this data was collected
  pub timestamp: f64,
}

/// Represents a command intended for the Igniter
#[derive(Clone, Debug, Deserialize, PartialEq, Serialize)]
pub enum Command {
  /// Arms all igniter channels
  ArmIgniter, 
  /// Disarms all igniter channels
  DisarmIgniter,
  /// Enables the specified channel
  EnableIgniter(u8),
  /// Disables the specified channel
  DisableIgniter(u8),
  /// Enables continuity current 
  EnableContinuityCurrent,
  /// Disables continuity current
  DisableContinuityCurrent,
}

impl fmt::Display for Command {
  fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
    match self {
      Self::ArmIgniter => {
        write!(f, "Arming Igniter")
      },
      Self::DisarmIgniter => {
        write!(f, "Disarming Igniter")
      },
      Self::EnableIgniter(channel) => {
        write!(f, "Enabling Igniter channel {}", channel)
      },
      Self::DisableIgniter(channel) => {
        write!(f, "Disabling Igniter channel {}", channel)
      },
      Self::EnableContinuityCurrent => {
        write!(f, "Enabling Continuity Current")
      },
      Self::DisableContinuityCurrent => {
        write!(f, "Disabling Continuity Current")
      },
    }
  }
}
