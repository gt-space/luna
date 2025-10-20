//! Firmware driver for the LIS2MDL magnetometer.

#![warn(missing_docs)]

use common::comm::gpio::{Pin, PinMode, PinValue};
use spidev::{SpiModeFlags, Spidev, SpidevOptions, SpidevTransfer};
use std::{fmt, io, thread, time::Duration};

/// Error originating from the LIS2MDL magnetometer.
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

/// Result type encapsulating an error from the LIS2MDL.
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
      f,
      "Magnetic Field: X: {:.4} gauss, Y: {:.4} gauss, Z: {:.4} gauss",
      self.x, self.y, self.z,
    )
  }
}

#[allow(unused)]
mod registers {
  pub const OFFSET_X_REG_L: u8 = 0x45;
  pub const OFFSET_X_REG_H: u8 = 0x46;
  pub const OFFSET_Y_REG_L: u8 = 0x47;
  pub const OFFSET_Y_REG_H: u8 = 0x48;
  pub const OFFSET_Z_REG_L: u8 = 0x49;
  pub const OFFSET_Z_REG_H: u8 = 0x4a;

  pub const WHO_AM_I: u8 = 0x4f;

  pub const CFG_REG_A: u8 = 0x60;
  pub const CFG_REG_B: u8 = 0x61;
  pub const CFG_REG_C: u8 = 0x62;

  pub const INT_CRTL_REG: u8 = 0x63;
  pub const INT_SOURCE_REG: u8 = 0x64;
  pub const INT_THS_L_REG: u8 = 0x65;
  pub const INT_THS_H_REG: u8 = 0x66;

  pub const STATUS_REG: u8 = 0x67;

  pub const OUTX_L: u8 = 0x68;
  pub const OUTX_H: u8 = 0x69;
  pub const OUTY_L: u8 = 0x6a;
  pub const OUTY_H: u8 = 0x6b;
  pub const OUTZ_L: u8 = 0x6c;
  pub const OUTZ_H: u8 = 0x6d;

  pub const TEMP_OUT_L_REG: u8 = 0x6e;
  pub const TEMP_OUT_H_REG: u8 = 0x6f;
}

const DEV_ID: u8 = 0b01000000;

/// Controls a hardware LIS2MDL magnetometer device over SPI.
pub struct LIS2MDL {
  /// The SPI bus connected to the LIS2MDL.
  spi: Spidev,

  /// An optional GPIO chip select pin.
  cs: Option<Pin>,
}

impl LIS2MDL {
  /// Constructs a new magnetometer device with the specified SPI bus, chip
  /// select, and data ready pins.
  pub fn new(bus: &str, mut cs: Option<Pin>) -> Result<Self> {
    // Configure the SPI bus.
    println!("Initializing SPI");
    let mut spi = Spidev::open(bus).map_err(Error::SPI)?;

    let options = SpidevOptions::new()
      .bits_per_word(8)
      .max_speed_hz(500000)
      .mode(SpiModeFlags::SPI_MODE_3)
      .lsb_first(false)
      .build();

    println!("Configuring SPI");
    spi.configure(&options).map_err(Error::SPI)?;

    // Configure the chip select.
    if let Some(cs) = &mut cs {
      cs.mode(PinMode::Output);
      cs.digital_write(PinValue::High);
    }

    let mut driver = LIS2MDL { spi, cs };

    println!("Initializing driver");
    // Initialize sensor CTRL registers
    driver.init()?;

    loop {
      println!("Reading WHO_AM_I");
      // Verify device id
      let who_am_i = driver.read_register(registers::WHO_AM_I)?;

      if who_am_i != DEV_ID {
        // return Err(Error::DeviceIdUnexpected(who_am_i));
        println!("received: {who_am_i}");
      }

      std::thread::sleep(Duration::from_millis(100));
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
    // RW = 1
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
    // Enable temperature compensation
    self.write_register(registers::CFG_REG_A, 0x80)?;
    thread::sleep(Duration::from_millis(50));

    // Enable mag, data-ready interrupt
    self.write_register(registers::CFG_REG_C, 0x01)?;
    thread::sleep(Duration::from_millis(50));

    Ok(())
  }

  /// Reads and returns magnetic field data.
  pub fn read(&mut self) -> Result<MagnetometerData> {
    let x_l = self.read_register(registers::OUTX_L)?;
    let x_h = self.read_register(registers::OUTX_H)?;
    let y_l = self.read_register(registers::OUTY_L)?;
    let y_h = self.read_register(registers::OUTY_H)?;
    let z_l = self.read_register(registers::OUTZ_L)?;
    let z_h = self.read_register(registers::OUTZ_H)?;

    let x = ((x_h as i16) << 8) | (x_l as i16);
    let y = ((y_h as i16) << 8) | (y_l as i16);
    let z = ((z_h as i16) << 8) | (z_l as i16);

    let scale: f32 = 1.5 * 1e-3; // converts from mgauss to gauss

    Ok(MagnetometerData {
      x: x as f32 * scale,
      y: y as f32 * scale,
      z: z as f32 * scale,
    })
  }
}
