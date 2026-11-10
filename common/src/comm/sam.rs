use std::{fmt, str::FromStr};

use compaq::compress_identity_impl;
use postcard::experimental::max_size::MaxSize;
#[cfg(feature = "rusqlite")]
use rusqlite::{
    types::{
        FromSql, FromSqlError, FromSqlResult, ToSqlOutput, Value as SqlValue,
        ValueRef as SqlValueRef,
    },
    ToSql,
};
use serde::{Deserialize, Serialize};

use crate::comm::ValveAction;

/// Wire representation for revisioned abort stages, preserving millisecond delays.
/// Older commands retain their existing seconds-based encoding.
#[derive(Clone, Copy, Debug, Deserialize, Eq, PartialEq, Serialize)]
pub struct ValveActionMillis {
    /// SAM valve channel.
    pub channel_num: u32,
    /// Whether to power the valve at its abort deadline.
    pub powered: bool,
    /// Delay from abort command receipt in milliseconds.
    pub timer_ms: u64,
}

impl TryFrom<&ValveAction> for ValveActionMillis {
    type Error = std::num::TryFromIntError;

    fn try_from(action: &ValveAction) -> Result<Self, Self::Error> {
        Ok(Self {
            channel_num: action.channel_num,
            powered: action.powered,
            timer_ms: action.timer.as_millis().try_into()?,
        })
    }
}

impl From<ValveActionMillis> for ValveAction {
    fn from(action: ValveActionMillis) -> Self {
        Self {
            channel_num: action.channel_num,
            powered: action.powered,
            timer: std::time::Duration::from_millis(action.timer_ms),
        }
    }
}

/// Every unit needed to be passed around in communications, mainly for sensor
/// readings.
#[derive(
    Clone,
    Copy,
    Debug,
    Default,
    Deserialize,
    Eq,
    Hash,
    MaxSize,
    PartialEq,
    Serialize,
    rkyv::Archive,
    rkyv::Serialize,
    rkyv::Deserialize,
)]
#[serde(rename_all = "snake_case")]
#[archive_attr(derive(bytecheck::CheckBytes))]
pub enum Unit {
    /// Unknown or not-yet-restored unit metadata.
    #[default]
    Unknown,

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
compress_identity_impl!(Unit);

/// Represents all possible channel types that may be used in a `NodeMapping`.
#[derive(Clone, Copy, Debug, Deserialize, Eq, Hash, MaxSize, PartialEq, Serialize)]
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
#[derive(Clone, Debug, Deserialize, Eq, PartialEq, Serialize)]
pub enum SamControlMessage {
    /// Instructs the board to actuate a valve.
    ActuateValve {
        /// The channel that the valve is connected to.
        channel: u32,

        /// Set to `true` for powered and `false` for unpowered.
        ///
        /// The actual state of the valve depends on whether it is normally open
        /// or normally closed.
        powered: bool,
    },
    /// Instructs the board to save these valve states in case of an abort
    AbortStageValveStates {
        /// States that a board will remember and go to in case of an abort.
        /// (channel_num, powered) pairs
        valve_states: Vec<ValveAction>,
    },
    /// Tells a board to abort.
    Abort,
    /// Clears messages that we have stored for an abort stage
    ClearStoredAbortStage {},
    /// Clears the abort status. This is used to tell a SAM that we are
    /// no longer in an abort state, which allows for future aborts to be
    /// performed.
    ClearAbortStatus,
    /// Toggles the camera enable/disable pin
    CameraEnable(bool), // true for enable, false for disable
    /// Toggles the launch lug enable/disable pin
    LaunchLugArm(bool), // true for enable, false for disable
    /// Toggles the launch lug detonate pin
    LaunchLugDetonate(bool), /* true for enable, false for disable
                              * No more LED command it takes up valuable
                              * space in code memory */
    /// Store a configuration without changing the active one.
    PrepareAbortStage {
        /// Monotonic transaction revision.
        revision: u128,
        /// Legacy seconds-encoded valve actions.
        valve_states: Vec<ValveAction>,
    },
    /// Activate a previously prepared revision.
    ActivateAbortStage {
        /// Revision to activate.
        revision: u128,
    },
    /// Clear stored configuration only if newer than every stage command seen.
    ClearStoredAbortStageV2 {
        /// Revision ordering this clear against stage commands.
        revision: u128,
    },
    /// Prepare without losing subsecond valve delays.
    PrepareAbortStageMillis {
        /// Monotonic transaction revision.
        revision: u128,
        /// Millisecond-encoded valve actions.
        valve_states: Vec<ValveActionMillis>,
    },
}

/// Result of a revisioned abort-stage command.
#[derive(Clone, Copy, Debug, Deserialize, Eq, MaxSize, PartialEq, Serialize)]
pub enum AbortStageAck {
    /// Configuration stored without activation.
    Prepared,
    /// Configuration activated.
    Activated,
    /// Configuration rejected.
    Rejected,
}

/// A single sensor data point for a SAM.
#[derive(Clone, Debug, Deserialize, MaxSize, PartialEq, Serialize)]
pub struct SensorDataPoint {
    /// The raw float value of the measurement, no units.
    pub value: f64,

    /// The exact UNIX timestamp of when this single data point was recorded.
    pub timestamp: f64,

    /// The channel that the data point was recorded from.
    pub channel: u32,

    /// The channel
    pub channel_type: ChannelType,
}

/// One telemetry sample sent from a SAM board to the flight computer.
#[derive(Clone, Debug, Deserialize, MaxSize, PartialEq, Serialize)]
pub enum SamDataPoint {
    /// A sensor sample.
    Sensor(SensorDataPoint),

    /// The RBF state on a flight SAM.
    /// RBF only exists on flight SAMs.
    Rbf {
        /// RBF value
        value: u8,
    },
    /// Repeated in telemetry so a lost UDP packet cannot hide an acknowledgment.
    AbortStageAck {
        /// Revision acknowledged by the SAM.
        revision: u128,
        /// Outcome for this revision.
        phase: AbortStageAck,
    },
}

#[cfg(test)]
mod abort_stage_protocol_tests {
    use super::*;
    use std::time::Duration;

    #[test]
    fn legacy_and_revisioned_clear_round_trip_without_reindexing() {
        for (command, tag) in [
            (SamControlMessage::ClearStoredAbortStage {}, 3),
            (SamControlMessage::ClearStoredAbortStageV2 { revision: u128::MAX }, 10),
        ] {
            let bytes = postcard::to_allocvec(&command).unwrap();
            assert_eq!(bytes[0], tag);
            assert_eq!(postcard::from_bytes::<SamControlMessage>(&bytes).unwrap(), command);
        }
    }

    #[test]
    fn revisioned_stage_preserves_millisecond_delays_on_the_wire() {
        let delays = [0, 200, 500, 1000, 2000, 5000];
        let actions: Vec<_> = delays
            .into_iter()
            .enumerate()
            .map(|(index, milliseconds)| ValveAction {
                channel_num: index as u32 + 1,
                powered: true,
                timer: Duration::from_millis(milliseconds),
            })
            .collect();
        let command = SamControlMessage::PrepareAbortStageMillis {
            revision: 10,
            valve_states: actions
                .iter()
                .map(ValveActionMillis::try_from)
                .collect::<Result<Vec<_>, _>>()
                .unwrap(),
        };
        let bytes = postcard::to_allocvec(&command).unwrap();
        assert_eq!(bytes[0], 11);
        let decoded = postcard::from_bytes::<SamControlMessage>(&bytes).unwrap();
        let SamControlMessage::PrepareAbortStageMillis { valve_states, .. } = decoded else {
            panic!("wrong command variant");
        };
        let restored: Vec<ValveAction> = valve_states.into_iter().map(Into::into).collect();
        assert_eq!(restored, actions);
    }
}
