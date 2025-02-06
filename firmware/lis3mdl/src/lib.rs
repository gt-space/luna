//! Firmware driver for the LIS3MDL magnetometer.

#![warn(missing_docs)]

use common::comm::gpio::{Pin, PinMode, PinValue};
use spidev::{SpiModeFlags, Spidev, SpidevOptions, SpidevTransfer};
use std::{fmt, io, thread, time::Duration};

/// Error originating from the LIS3MDL magnetometer.
#[derive(Debug)]
pub enum Error {
  /// Indicates that the device ID returned was not the correct value.
  DeviceIdUnexpected(u8),

  /// Serial peripheral interface error.
  SPI(io::Error),
}

impl fmt::Display for Error {
  fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
    match self {
      Error::DeviceIdUnexpected(id) => write!(f, "unexpected device ID: {id}"),
      Error::SPI(error) => write!(f, "SPI error: {error}"),
    }
  }
}

impl std::error::Error for Error {}

impl From<io::Error> for Error {
  fn from(err: io::Error) -> Self {
    Error::SPI(err)
  }
}

/// Result type encapsulating an error from the LIS3MDL.
pub type Result<T> = std::result::Result<T, Error>;

/// Magnetometer measurement data.
#[derive(Clone, Copy, Debug, Default)]
pub struct MagnetometerData {
  /// The x-component of the magnetic field, in gauss.
  pub x: f32,

  /// The y-component of the magnetic field, in gauss.
  pub y: f32,

  /// The z-component of the magnetic field, in gauss.
  pub z: f32,
}

impl fmt::Display for MagnetometerData {
  fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
    write!(
      f, "Magnetic Field: X: {:.4} gauss, Y: {:.4} gauss, Z: {:.4} gauss",
      self.x,
      self.y,
      self.z,
    )
  }
}

#[allow(unused)]
mod registers {
  pub const OFFSET_X_REG_L_M: u8 = 0x05;
  pub const OFFSET_X_REG_H_M: u8 = 0x06;
  pub const OFFSET_Y_REG_L_M: u8 = 0x07;
  pub const OFFSET_Y_REG_H_M: u8 = 0x08;
  pub const OFFSET_Z_REG_L_M: u8 = 0x09;
  pub const OFFSET_Z_REG_H_M: u8 = 0x0A;
  pub const WHO_AM_I: u8 = 0x0F;
  pub const CTRL_REG1: u8 = 0x20;
  pub const CTRL_REG2: u8 = 0x21;
  pub const CTRL_REG3: u8 = 0x22;
  pub const CTRL_REG4: u8 = 0x23;
  pub const CTRL_REG5: u8 = 0x24;
  pub const STATUS_REG: u8 = 0x27;
  pub const OUT_X_L: u8 = 0x28;
  pub const OUT_X_H: u8 = 0x29;
  pub const OUT_Y_L: u8 = 0x2A;
  pub const OUT_Y_H: u8 = 0x2B;
  pub const OUT_Z_L: u8 = 0x2C;
  pub const OUT_Z_H: u8 = 0x2D;
  pub const TEMP_OUT_L: u8 = 0x2E;
  pub const TEMP_OUT_H: u8 = 0x2F;
  pub const INT_CFG: u8 = 0x30;
  pub const INT_SRC: u8 = 0x31;
  pub const INT_THS_L: u8 = 0x32;
  pub const INT_THS_H: u8 = 0x33;
}

const DEV_ID: u8 = 0x3D;

/// Controls a hardware LIS3MDL magnetometer device over SPI.
pub struct LIS3MDL {
  /// The SPI bus connected to the LIS3MDL.
  spi: Spidev,

  /// The data ready GPIO pin.
  drdy: Pin,

  /// An optional GPIO chip select pin.
  cs: Option<Pin>,
}

impl LIS3MDL {
  /// Constructs a new magnetometer device with the specified SPI bus, chip
  /// select, and data ready pins.
  pub fn new(bus: &str, mut cs: Option<Pin>, mut drdy: Pin) -> Result<Self> {
    // Configure the SPI bus.
    let mut spi = Spidev::open(bus)
      .map_err(Error::SPI)?;

    let options = SpidevOptions::new()
      .bits_per_word(8)
      .max_speed_hz(500000)
      .mode(SpiModeFlags::SPI_MODE_3)
      .lsb_first(false)
      .build();

    spi.configure(&options)
      .map_err(Error::SPI)?;

    // Configure the chip select.
    if let Some(cs) = &mut cs {
      cs.mode(PinMode::Output);
      cs.digital_write(PinValue::High);
    }

    // Configure the data ready pin.
    drdy.mode(PinMode::Input);

    let mut driver = LIS3MDL {
      spi,
      drdy,
      cs,
    };

    // Initialize sensor CTRL registers
    driver.init()?;

    // Verify device id
    let who_am_i = driver.read_register(registers::WHO_AM_I)?;

    if who_am_i != DEV_ID {
      return Err(Error::DeviceIdUnexpected(who_am_i));
    }

    Ok(driver)
  }

  /// Pulls the chip select GPIO low, if applicable.
  fn select(&mut self) {
    if let Some(cs) = &mut self.cs {
      cs.digital_write(PinValue::Low);
    }
  }

  /// Pulls the chip select GPIO high, if applicable.
  fn deselect(&mut self) {
    if let Some(cs) = &mut self.cs {
      cs.digital_write(PinValue::High);
    }
  }

  /// Performs a transfer on the configured SPI bus.
  fn transfer(&mut self, transfer: &mut SpidevTransfer) -> Result<()> {
    self.select();
    self.spi.transfer(transfer)?;
    self.deselect();

    Ok(())
  }

  /// Reads a single registers.
  fn read_register(&mut self, register: u8) -> Result<u8> {
    // RW = 1, MS = 0.
    let tx = [(1 << 7) | register, 0];
    let mut rx = [0x00; 2];
    self.transfer(&mut SpidevTransfer::read_write(&tx, &mut rx))?;

    Ok(rx[1])
  }

  /// Writes to a single register.
  fn write_register(&mut self, register: u8, value: u8) -> Result<()> {
    // RW = 0, MS = 0.
    let tx = [register, value];
    self.transfer(&mut SpidevTransfer::write(&tx))?;

    Ok(())
  }

  /// Initializes configuration registers.
  fn init(&mut self) -> Result<()> {
    self.write_register(registers::CTRL_REG2, 0x04)?;
    thread::sleep(Duration::from_millis(50));

    self.write_register(registers::CTRL_REG1, 0x7c)?;
    self.write_register(registers::CTRL_REG4, 0x0c)?;
    self.write_register(registers::CTRL_REG3, 0x00)?;
    self.write_register(registers::CTRL_REG5, 0x00)?;
    thread::sleep(Duration::from_millis(50));

    Ok(())
  }

  /// Reads and returns magnetic field data.
  pub fn read(&mut self) -> Result<MagnetometerData> {
    while self.drdy.digital_read() == PinValue::Low {}

    let x_l = self.read_register(registers::OUT_X_L)?;
    let x_h = self.read_register(registers::OUT_X_H)?;
    let y_l = self.read_register(registers::OUT_Y_L)?;
    let y_h = self.read_register(registers::OUT_Y_H)?;
    let z_l = self.read_register(registers::OUT_Z_H)?;
    let z_h = self.read_register(registers::OUT_Z_L)?;

    let x = ((x_h as i16) << 8) | (x_l as i16);
    let y = ((y_h as i16) << 8) | (y_l as i16);
    let z = ((z_h as i16) << 8) | (z_l as i16);

    let scale = 4.0 / 32767.0 * 100.0;

    Ok(MagnetometerData {
      x: x as f32 * scale,
      y: y as f32 * scale,
      z: z as f32 * scale,
    })
  }
}
