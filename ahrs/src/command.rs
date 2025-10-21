use common::comm::{
  ahrs::Command,
  gpio::{Gpio, PinMode},
};
use imu::{bit_mappings::ImuDriverError, AdisIMUDriver};
use once_cell::sync::Lazy;
use spidev::Spidev;
use std::fmt;

const IMU_CS_PIN_LOC: [usize; 2] = [0, 11];
const IMU_DR_PIN_LOC: [usize; 2] = [2, 17];
const IMU_NRESET_PIN_LOC: [usize; 2] = [2, 25];

pub static GPIO_CONTROLLERS: Lazy<Vec<Gpio>> =
  Lazy::new(|| (0..=3).map(Gpio::open_controller).collect());

pub struct Drivers {
  pub imu: AdisIMUDriver,
}

#[derive(Debug)]
pub enum DriverError {
  Io(std::io::Error),
  Imu(ImuDriverError),
}

impl fmt::Display for DriverError {
  fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
    match self {
      DriverError::Io(e) => write!(f, "IO error: {e}"),
      DriverError::Imu(e) => write!(f, "IMU driver error: {e}"),
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

pub fn init_gpio() {}

pub fn init_drivers() -> Result<Drivers, DriverError> {
  init_gpio();

  let spi = Spidev::open("/dev/spidev0.0")?;

  let mut imu_cs =
    GPIO_CONTROLLERS[IMU_CS_PIN_LOC[0]].get_pin(IMU_CS_PIN_LOC[1]);
  imu_cs.mode(PinMode::Output);
  let mut imu_dr =
    GPIO_CONTROLLERS[IMU_DR_PIN_LOC[0]].get_pin(IMU_DR_PIN_LOC[1]);
  imu_dr.mode(PinMode::Input);
  let mut imu_nreset =
    GPIO_CONTROLLERS[IMU_NRESET_PIN_LOC[0]].get_pin(IMU_NRESET_PIN_LOC[1]);
  imu_nreset.mode(PinMode::Output);

  let imu = AdisIMUDriver::initialize(spi, imu_dr, imu_nreset, imu_cs)?;

  Ok(Drivers { imu })
}

pub fn execute(command: Command) {}
