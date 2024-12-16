use crate::comm::bms;
use postcard::experimental::max_size::MaxSize;
use serde::{Deserialize, Serialize};
use std::{borrow::Cow, fmt, str::FromStr};

#[cfg(feature = "rusqlite")]
use rusqlite::{
  types::{
    FromSql,
    FromSqlError,
    FromSqlResult,
    ToSqlOutput,
    Value as SqlValue,
    ValueRef as SqlValueRef,
  },
  ToSql,
};

/// Every unit needed to be passed around in communications, mainly for sensor
/// readings.
#[derive(
  Clone, Copy, Debug, Deserialize, Eq, Hash, MaxSize, PartialEq, Serialize,
)]
#[serde(rename_all = "snake_case")]
pub enum Unit {
  /// Current, in amperes.
  Amps,

  /// Pressure, in pounds per square inch.
  Psi,

  /// Temperature, in Kelvin.
  Kelvin,

  /// Force, in pounds.
  Pounds,

  /// Electric potential, in volts.
  Volts,
}

/// Represents all possible channel types that may be used in a `NodeMapping`.
#[derive(
  Clone, Copy, Debug, Deserialize, Eq, Hash, MaxSize, PartialEq, Serialize,
)]
#[serde(rename_all = "snake_case")]
pub enum ChannelType {
  /// A pressure transducer, formerly known as CurrentLoop, which measures the
  /// pressure of a fluid.
  CurrentLoop,

  /// The voltage present on a pin connected to a valve.
  ValveVoltage,

  /// The current flowing through a pin connected to a valve.
  ValveCurrent,

  /// The voltage on the power rail of the board.
  RailVoltage,

  /// The current flowing through the power rail of the board.
  RailCurrent,

  /// The signal from a load cell, carried by a differential pair.
  DifferentialSignal,

  /// The channel of a resistance thermometer, measuring temperature.
  Rtd,

  /// The channel of a thermocouple, measuring temperature.
  Tc,
}

impl ChannelType {
  /// Gets the associated unit for the given channel type.
  pub fn unit(&self) -> Unit {
    match self {
      Self::CurrentLoop => Unit::Psi,
      Self::ValveVoltage => Unit::Volts,
      Self::ValveCurrent => Unit::Amps,
      Self::RailVoltage => Unit::Volts,
      Self::RailCurrent => Unit::Amps,
      Self::DifferentialSignal => Unit::Pounds,
      Self::Rtd => Unit::Kelvin,
      Self::Tc => Unit::Kelvin,
    }
  }
}

impl fmt::Display for ChannelType {
  fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
    match self {
      Self::CurrentLoop => write!(f, "current_loop"),
      Self::DifferentialSignal => write!(f, "differential_signal"),
      Self::RailCurrent => write!(f, "rail_current"),
      Self::RailVoltage => write!(f, "rail_voltage"),
      Self::Rtd => write!(f, "rtd"),
      Self::Tc => write!(f, "tc"),
      Self::ValveCurrent => write!(f, "valve_current"),
      Self::ValveVoltage => write!(f, "valve_voltage"),
    }
  }
}

impl FromStr for ChannelType {
  type Err = ();

  fn from_str(string: &str) -> Result<Self, Self::Err> {
    match string {
      "current_loop" => Ok(ChannelType::CurrentLoop),
      "differential_signal" => Ok(ChannelType::DifferentialSignal),
      "rail_current" => Ok(ChannelType::RailCurrent),
      "rail_voltage" => Ok(ChannelType::RailVoltage),
      "rtd" => Ok(ChannelType::Rtd),
      "tc" => Ok(ChannelType::Tc),
      "valve_current" => Ok(ChannelType::ValveCurrent),
      "valve_voltage" => Ok(ChannelType::ValveVoltage),
      _ => Err(()),
    }
  }
}

#[cfg(feature = "rusqlite")]
impl ToSql for ChannelType {
  fn to_sql(&self) -> rusqlite::Result<ToSqlOutput<'_>> {
    Ok(ToSqlOutput::Owned(SqlValue::Text(self.to_string())))
  }
}

#[cfg(feature = "rusqlite")]
impl FromSql for ChannelType {
  fn column_result(value: SqlValueRef<'_>) -> FromSqlResult<Self> {
    if let SqlValueRef::Text(text) = value {
      let Ok(string) = std::str::from_utf8(text) else {
        return Err(FromSqlError::InvalidType);
      };

      if let Ok(channel_type) = ChannelType::from_str(string) {
        Ok(channel_type)
      } else {
        Err(FromSqlError::InvalidType)
      }
    } else {
      Err(FromSqlError::InvalidType)
    }
  }
}

/// A control message send from the flight computer to a SAM board.
#[derive(Clone, Debug, Deserialize, Eq, MaxSize, PartialEq, Serialize)]
pub enum SamControlMessage {
  /// Instructs the board to actuate a valve.
  ActuateValve {
    /// The channel that the valve is connected to.
    channel: u32,

    /// Set to `true` for powered and `false` for unpowered.
    ///
    /// The actual state of the valve depends on whether it is normally open or
    /// normally closed.
    powered: bool,
  },
  /// Instructs the board to set an LED.
  SetLed {
    /// The channel that the LED is wired to.
    channel: u32,

    /// Set to `true` to turn off and `false` to turn off.
    on: bool,
  },
}

/// A single data point with a timestamp and channel, no units.
#[derive(Clone, Debug, Deserialize, MaxSize, PartialEq, Serialize)]
pub struct DataPoint {
  /// The raw float value of the measurement, no units.
  pub value: f64,

  /// The exact UNIX timestamp of when this single data point was recorded.
  pub timestamp: f64,

  /// The channel that the data point was recorded from.
  pub channel: u32,

  /// The channel
  pub channel_type: ChannelType,
}
