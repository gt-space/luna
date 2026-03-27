use super::bms::Rail;
use postcard::experimental::max_size::MaxSize;
use serde::{Deserialize, Serialize};

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

/// in units of meters/second^2 (acceleration)
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

/// Represents the state of the flight computer's onboard sensors.
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
pub struct FcSensors {
  /// 3V3 rail
  pub rail_3v3: Rail,
  /// 5V rail
  pub rail_5v: Rail,
  /// IMU data
  pub imu: Imu,
  /// Magnetometer data
  pub magnetometer: Magnetometer,
  /// Barometer data
  pub barometer: Barometer,
}
