use super::{bms::Rail, flight::Ingestible, VehicleState};
use postcard::experimental::max_size::MaxSize;
use serde::{Deserialize, Serialize};
use std::fmt;

type Celsius = f64;
type Bar = f64;

/// Represents a vector
#[derive(
  Deserialize, Serialize, Clone, Copy, MaxSize, Debug, PartialEq, Default,
)]
pub struct Vector {
  pub x: f64,
  pub y: f64,
  pub z: f64,
}

/// in units of Gs
type Accelerometer = Vector;

/// in units of degrees/second
type Gyroscope = Vector;

/// in units of Gauss
type Magnetometer = Vector;

/// Represents the state of the IMU
#[derive(
  Deserialize, Serialize, Clone, Copy, MaxSize, Debug, PartialEq, Default,
)]
pub struct Imu {
  pub accelerometer: Accelerometer,
  pub gyroscope: Gyroscope,
}

/// Represents the state of the Barometer
#[derive(
  Deserialize, Serialize, Clone, Copy, MaxSize, Debug, PartialEq, Default,
)]
pub struct Barometer {
  pub temperature: Celsius,
  pub pressure: Bar,
}

/// Represents the state of AHRS as a whole
#[derive(
  Clone, Copy, MaxSize, Debug, Default, Deserialize, PartialEq, Serialize,
)]
pub struct Ahrs {
  pub rail_3_3_v: Rail,
  pub rail_5_v: Rail,
  pub imu: Imu,
  pub magnetometer: Magnetometer,
  pub barometer: Barometer,
}

/// A single data point with a timestamp and channel, no units.
#[derive(Clone, Copy, Debug, Deserialize, MaxSize, PartialEq, Serialize)]
pub struct DataPoint {
  /// The state of some device on the BMS.
  pub state: Ahrs,

  /// The timestamp of when this data was collected
  pub timestamp: f64,
}

/// Describes how a datapoint from an AHRS board should be interpreted.
impl Ingestible for DataPoint {
  fn ingest(&self, vehicle_state: &mut VehicleState) {
    vehicle_state.ahrs = self.state;
  }
}

/// Represents a command intended for AHRS from the FC
#[derive(Clone, Debug, Deserialize, PartialEq, Serialize)]
pub enum Command {
  /// True if the camera should be enabled, False otherwise.
  CameraEnable(bool),
}

impl fmt::Display for Command {
  fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
    match self {
      Self::CameraEnable(enabled) => {
        write!(f, "Set CameraEnable to {}", enabled)
      }
    }
  }
}
