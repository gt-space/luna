use super::{bms::Rail, flight::Ingestible, VehicleState};
use postcard::experimental::max_size::MaxSize;
use serde::{Deserialize, Serialize};
use std::fmt;

type Celsius = f64;
type Pascals = f64;

/// Represents a vector
#[derive(
  Deserialize,
  Serialize,
  Clone,
  Copy,
  MaxSize,
  Debug,
  PartialEq,
  Default,
  rkyv::Archive,
  rkyv::Serialize,
  rkyv::Deserialize,
)]
#[archive_attr(derive(bytecheck::CheckBytes))]
pub struct Vector {
  /// X value
  pub x: f64,
  /// Y value
  pub y: f64,
  /// Z value
  pub z: f64,
}

/// in units of meters/second
type Accelerometer = Vector;

/// in units of degrees/second
type Gyroscope = Vector;

/// in units of Gauss
type Magnetometer = Vector;

/// Represents the state of the IMU
#[derive(
  Deserialize,
  Serialize,
  Clone,
  Copy,
  MaxSize,
  Debug,
  PartialEq,
  Default,
  rkyv::Archive,
  rkyv::Serialize,
  rkyv::Deserialize,
)]
#[archive_attr(derive(bytecheck::CheckBytes))]
pub struct Imu {
  /// Accelerometer (x, y, z) data
  pub accelerometer: Accelerometer,
  /// Gyroscope (rx, ry, rz) data
  pub gyroscope: Gyroscope,
}

/// Represents the state of the Barometer
#[derive(
  Deserialize,
  Serialize,
  Clone,
  Copy,
  MaxSize,
  Debug,
  PartialEq,
  Default,
  rkyv::Archive,
  rkyv::Serialize,
  rkyv::Deserialize,
)]
#[archive_attr(derive(bytecheck::CheckBytes))]
pub struct Barometer {
  /// Temperature data
  pub temperature: Celsius,
  /// Pressure data
  pub pressure: Pascals,
}

/// Represents the state of AHRS as a whole
#[derive(
  Clone,
  Copy,
  MaxSize,
  Debug,
  Default,
  Deserialize,
  PartialEq,
  Serialize,
  rkyv::Archive,
  rkyv::Serialize,
  rkyv::Deserialize,
)]
#[archive_attr(derive(bytecheck::CheckBytes))]
pub struct Ahrs {
  /// 3V3 rail
  pub rail_3v3: Rail,
  /// 5V rail
  pub rail_5v: Rail,
  /// IMU data
  pub imu: Imu,
  /// Barometer data
  pub barometer: Barometer,
  /// Magnetometer data
  pub magnetometer: Magnetometer,
}

/// A single data point with a timestamp and channel, no units.
#[derive(
  Clone,
  Copy,
  Debug,
  Deserialize,
  MaxSize,
  PartialEq,
  Serialize,
  rkyv::Archive,
  rkyv::Serialize,
  rkyv::Deserialize,
)]
#[archive_attr(derive(bytecheck::CheckBytes))]
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
