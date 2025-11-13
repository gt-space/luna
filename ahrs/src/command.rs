use crate::pins::GPIO_CONTROLLERS;
use common::comm::{
  ahrs::Command,
  gpio::{PinMode, PinValue},
};
use imu::{bit_mappings::ImuDriverError, AdisIMUDriver};
use lis2mdl::LIS2MDL;
use spidev::Spidev;
use std::fmt;

const IMU_CS_PIN_LOC: [usize; 2] = [0, 11];
const IMU_DR_PIN_LOC: [usize; 2] = [2, 17];
const IMU_NRESET_PIN_LOC: [usize; 2] = [2, 25];
const IMU_SPI: &str = "/dev/spidev0.0";

const BAR_CS_PIN_LOC: [usize; 2] = [0, 12];

const MAG_CS_PIN_LOC: [usize; 2] = [0, 13];
const MAG_SPI: &str = "/dev/spidev1.1";

pub const RAIL_5V: (&str, &str) = ( r"/sys/bus/iio/devices/iio:device0/in_voltage0_raw", r"/sys/bus/iio/devices/iio:device0/in_voltage1_raw");
pub const RAIL_3V3: (&str, &str) = (
    r"/sys/bus/iio/devices/iio:device0/in_voltage3_raw",
    r"/sys/bus/iio/devices/iio:device0/in_voltage2_raw",

);


pub struct Drivers {
  pub imu: AdisIMUDriver,
  pub magnetometer: LIS2MDL,
}

#[derive(Debug)]
pub enum DriverError {
  Io(std::io::Error),
  Imu(ImuDriverError),
  ImuSetDecimationRateFailed, // TODO: upstream into ImuDriverError
  ImuValidationFailed,        // TODO: upstream into ImuDriverError
  Magnetometer(lis2mdl::Error),
}

impl fmt::Display for DriverError {
  fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
    match self {
      DriverError::Io(e) => write!(f, "IO error: {e}"),
      DriverError::Imu(e) => write!(f, "IMU driver error: {e}"),
      DriverError::ImuSetDecimationRateFailed => {
        write!(f, "Failed to set IMU decimation rate")
      }
      DriverError::ImuValidationFailed => {
        write!(f, "Failed to validate IMU Prod ID")
      }
      DriverError::Magnetometer(e) => {
        write!(f, "Magnetometer driver error: {e}")
      }
    }
  }
}

impl std::error::Error for DriverError {}

impl From<std::io::Error> for DriverError {
  fn from(value: std::io::Error) -> Self {
    Self::Io(value)
  }
}

impl From<ImuDriverError> for DriverError {
  fn from(value: ImuDriverError) -> Self {
    Self::Imu(value)
  }
}

impl From<lis2mdl::Error> for DriverError {
  fn from(value: lis2mdl::Error) -> Self {
    Self::Magnetometer(value)
  }
}

pub fn init_gpio() {
  let mut imu_cs =
    GPIO_CONTROLLERS[IMU_CS_PIN_LOC[0]].get_pin(IMU_CS_PIN_LOC[1]);
  imu_cs.mode(PinMode::Output);
  imu_cs.digital_write(PinValue::High);

  let mut bar_cs =
    GPIO_CONTROLLERS[BAR_CS_PIN_LOC[0]].get_pin(BAR_CS_PIN_LOC[1]);
  bar_cs.mode(PinMode::Output);
  bar_cs.digital_write(PinValue::High);

  let mut mag_cs =
    GPIO_CONTROLLERS[MAG_CS_PIN_LOC[0]].get_pin(MAG_CS_PIN_LOC[1]);
  mag_cs.mode(PinMode::Output);
  mag_cs.digital_write(PinValue::High);
}

pub fn init_drivers() -> Result<Drivers, DriverError> {
  let imu = {
    let imu_spi = Spidev::open(IMU_SPI)?;

    let mut imu_cs =
      GPIO_CONTROLLERS[IMU_CS_PIN_LOC[0]].get_pin(IMU_CS_PIN_LOC[1]);
    imu_cs.mode(PinMode::Output);
    let mut imu_dr =
      GPIO_CONTROLLERS[IMU_DR_PIN_LOC[0]].get_pin(IMU_DR_PIN_LOC[1]);
    imu_dr.mode(PinMode::Input);
    let mut imu_nreset =
      GPIO_CONTROLLERS[IMU_NRESET_PIN_LOC[0]].get_pin(IMU_NRESET_PIN_LOC[1]);
    imu_nreset.mode(PinMode::Output);

    let mut imu =
      AdisIMUDriver::initialize(imu_spi, imu_dr, imu_nreset, imu_cs)?;

    imu
      .write_dec_rate(8)
      .map_err(|_| DriverError::ImuSetDecimationRateFailed)?;

    if !imu.validate() {
      return Err(DriverError::ImuValidationFailed);
    }

    imu
  };

  let magnetometer = {
    let mut mag_cs =
      GPIO_CONTROLLERS[MAG_CS_PIN_LOC[0]].get_pin(MAG_CS_PIN_LOC[1]);
    mag_cs.mode(PinMode::Output);

    LIS2MDL::new(MAG_SPI, Some(mag_cs))?
  };

  Ok(Drivers { imu, magnetometer })
}

pub fn execute(command: Command) {
  match command {
    Command::CameraEnable(enabled) => todo!(),
  }
}
