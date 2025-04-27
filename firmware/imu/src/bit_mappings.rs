use bitflags::*;
use std::{
  clone,
  error,
  fmt::{self, Binary},
  io,
};

pub type DriverResult<T> = std::result::Result<T, ImuDriverError>;

#[derive(Debug)]
pub struct InvalidDataError {
  reason: &'static str,
}

impl InvalidDataError {
  pub fn new(reason: &'static str) -> InvalidDataError {
    InvalidDataError { reason }
  }
}

impl fmt::Display for InvalidDataError {
  fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
    write!(f, "Invalid Data Received : {}", self.reason)
  }
}

#[derive(Debug)]
pub enum ImuDriverError {
  /// Error is from inside the IMU
  ImuError(DiagnosticStats),
  /// Error is from imu communication (ex. SPI)
  IOError(io::Error),
  /// Invalid Data received
  InvalidDataError(InvalidDataError),
}

impl From<io::Error> for ImuDriverError {
  fn from(err: io::Error) -> Self {
    ImuDriverError::IOError(err)
  }
}

impl From<DiagnosticStats> for ImuDriverError {
  fn from(stats: DiagnosticStats) -> Self {
    ImuDriverError::ImuError(stats)
  }
}

impl From<InvalidDataError> for ImuDriverError {
  fn from(err: InvalidDataError) -> Self {
    ImuDriverError::InvalidDataError(err)
  }
}

bitflags! {
  #[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
  pub struct DiagnosticStats : u16 {
    const DATA_PATH_OVERRUN = 1 << 1;
    const FLASH_MEMORY_UPDATE_FAILURE = 1 << 2;
    const SPI_COMMUNICATION = 1 << 3;
    const STANDBY_MODE = 1 << 4;
    const SENSOR_FAILURE = 1 << 5;
    const MEMORY_FAILURE = 1 << 6;
    const CLOCK_ERROR = 1 << 7;
    const GYRO_1_FAILURE = 1 << 8;
    const GYRO_2_FAILURE = 1 << 9;
    const ACCELEROMETER_FAILURE = 1 << 10;
  }
}

impl From<u16> for DiagnosticStats {
  fn from(num: u16) -> DiagnosticStats {
    DiagnosticStats::from_bits_truncate(num)
  }
}

impl fmt::Display for ImuDriverError {
  fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
    match self {
      ImuDriverError::ImuError(err) => {
        write!(f, "IMU ERROR - ")?;
        err.fmt(f)
      }
      ImuDriverError::IOError(err) => {
        write!(f, "IO ERROR - ")?;
        err.fmt(f)
      }
      ImuDriverError::InvalidDataError(err) => {
        write!(f, "Invalid Data Error - ")?;
        err.fmt(f)
      }
    }
  }
}

pub enum BurstSelectOptions {
  GyroBurst,
  DeltaBurst,
}
