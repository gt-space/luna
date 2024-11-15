use super::sam::ChannelType;
use crate::ToPrettyString;
use postcard::experimental::max_size::MaxSize;
use serde::{Deserialize, Serialize};
use std::{fmt, str::FromStr};

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

/// The state or commanded state of a valve.
#[derive(
  Clone, Copy, Debug, Deserialize, Eq, Hash, MaxSize, PartialEq, Serialize,
)]
#[serde(rename_all = "snake_case")]
pub enum ValveState {
  /// Undetermined state, whether because the valve is unmapped or has not been
  /// commanded yet.
  Undetermined,

  /// Valve disconnected.
  Disconnected,

  /// Valve open.
  Open,

  /// Valve closed.
  Closed,

  /// Fault in valve.
  Fault,
}

impl fmt::Display for ValveState {
  fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
    write!(
      f,
      "{}",
      match self {
        Self::Undetermined => "undetermined",
        Self::Disconnected => "disconnected",
        Self::Open => "open",
        Self::Closed => "closed",
        Self::Fault => "fault",
      }
    )
  }
}

impl ToPrettyString for ValveState {
  /// Converts the valve state into a colored string ready to be displayed on
  /// the interface.
  fn to_pretty_string(&self) -> String {
    match self {
      Self::Undetermined => "\x1b[38;5;248mundetermined\x1b[0m",
      Self::Disconnected => "\x1b[33mdisconnected\x1b[0m",
      Self::Open => "\x1b[32mopen\x1b[0m",
      Self::Closed => "\x1b[31mclosed\x1b[0m",
      Self::Fault => "\x1b[34mfault\x1b[0m",
    }
    .to_owned()
  }
}

#[cfg(feature = "rusqlite")]
impl rusqlite::ToSql for ValveState {
  fn to_sql(&self) -> rusqlite::Result<rusqlite::types::ToSqlOutput<'_>> {
    Ok(rusqlite::types::ToSqlOutput::Owned(
      rusqlite::types::Value::Text(self.to_string()),
    ))
  }
}

/// Stores the estimated actual valve state as well as the software-commanded
/// state.
#[derive(
  Clone, Debug, Deserialize, Eq, Hash, MaxSize, PartialEq, Serialize,
)]
pub struct CompositeValveState {
  /// Commanded state of the valve, according to software.
  pub commanded: ValveState,

  /// Actual state of the valve, determined using voltage and current
  /// measurements.
  pub actual: ValveState,
}

/// Represents all possible sensor types that may be used in a `NodeMapping`.
#[derive(
  Clone, Copy, Debug, Deserialize, Eq, Hash, MaxSize, PartialEq, Serialize,
)]
#[serde(rename_all = "snake_case")]
pub enum SensorType {
  /// Load cell, measuring force.
  LoadCell,

  /// Pressure transducer, which measures the pressure of a fluid.
  Pt,

  /// Current of the power tail of the board.
  RailCurrent,

  /// Voltage of the power rail of the board.
  RailVoltage,

  /// Resistance thermometer, measuring temperature.
  Rtd,

  /// Thermocouple, measuring temperature.
  Tc,

  /// Valve, which can be actuated and read with voltage and current.
  Valve,
}

impl SensorType {
  /// Returns the channel types associated with this sensor type.
  pub fn channel_types(self) -> &'static [ChannelType] {
    match self {
      Self::LoadCell => &[ChannelType::DifferentialSignal],
      Self::Pt => &[ChannelType::CurrentLoop],
      Self::RailCurrent => &[ChannelType::RailCurrent],
      Self::RailVoltage => &[ChannelType::RailVoltage],
      Self::Rtd => &[ChannelType::Rtd],
      Self::Tc => &[ChannelType::Tc],
      Self::Valve => &[ChannelType::ValveVoltage, ChannelType::ValveCurrent],
    }
  }
}

impl fmt::Display for SensorType {
  fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
    match self {
      Self::LoadCell => write!(f, "load_cell"),
      Self::Pt => write!(f, "pt"),
      Self::RailCurrent => write!(f, "rail_current"),
      Self::RailVoltage => write!(f, "rail_voltage"),
      Self::Rtd => write!(f, "rtd"),
      Self::Tc => write!(f, "tc"),
      Self::Valve => write!(f, "valve"),
    }
  }
}

impl FromStr for SensorType {
  type Err = ();

  fn from_str(string: &str) -> Result<Self, Self::Err> {
    match string {
      "load_cell" => Ok(Self::LoadCell),
      "pt" => Ok(Self::Pt),
      "rail_current" => Ok(Self::RailCurrent),
      "rail_voltage" => Ok(Self::RailVoltage),
      "rtd" => Ok(Self::Rtd),
      "tc" => Ok(Self::Tc),
      "valve" => Ok(Self::Valve),
      _ => Err(()),
    }
  }
}

#[cfg(feature = "rusqlite")]
impl ToSql for SensorType {
  fn to_sql(&self) -> rusqlite::Result<ToSqlOutput<'_>> {
    Ok(ToSqlOutput::Owned(SqlValue::Text(self.to_string())))
  }
}

#[cfg(feature = "rusqlite")]
impl FromSql for SensorType {
  fn column_result(value: SqlValueRef<'_>) -> FromSqlResult<Self> {
    if let SqlValueRef::Text(text) = value {
      let Ok(string) = std::str::from_utf8(text) else {
        return Err(FromSqlError::InvalidType);
      };

      if let Ok(sensor_type) = SensorType::from_str(string) {
        Ok(sensor_type)
      } else {
        Err(FromSqlError::InvalidType)
      }
    } else {
      Err(FromSqlError::InvalidType)
    }
  }
}
