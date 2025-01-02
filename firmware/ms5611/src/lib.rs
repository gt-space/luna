//! Firmware for the MS5611-01BA03 barometric pressure sensor.

#![warn(missing_docs)]

#[cfg(feature = "checkout")]
pub mod checkout;

use log::warn;
use spidev::{SpiModeFlags, Spidev, SpidevOptions, SpidevTransfer};
use std::{
  fmt::{self, Display, Formatter},
  io,
  thread,
  time::{Duration, Instant},
};

/// A error related to interacting with the MS5611-01BA03 barometer.
#[derive(Debug)]
pub enum Error {
  /// Indicates that the most recently dispatched conversion command has failed.
  ///
  /// This may have been caused by a premature read, but it could also be a
  /// malfunction in the device.
  ConversionFailed,

  /// Indicates that the OSR passed as an argument is invalid.
  /// Valid OSRs are 256, 512, 1024, 2048, and 4096.
  OSRInvalid(u16),

  /// Indicates that the PROM address passed as an argument is invalid.
  /// Valid PROM addresses range from 0 to 7.
  PROMAddressInvalid(u8),

  /// Indicates that the PROM CRC check failed, signalling that the calibration
  /// data has been corrupted.
  PROMValidationFailed(PROM),

  /// Indicates that a SPI I/O error has occurred.
  SPI(io::Error),
}

impl Display for Error {
  fn fmt(&self, f: &mut Formatter) -> fmt::Result {
    match self {
      Self::ConversionFailed => {
        write!(f, "ADC conversion failed")
      },
      Self::OSRInvalid(_osr) => {
        write!(f, "OSR must be one of [256, 512, 1024, 2048, 4096]")
      },
      Self::PROMAddressInvalid(address) => {
        write!(f, "invalid PROM address: {address}")
      },
      Self::PROMValidationFailed(prom) => {
        write!(f, "PROM validation failed: {prom:#?}")
      },
      Self::SPI(error) => {
        write!(f, "SPI error: {error}")
      },
    }
  }
}

impl std::error::Error for Error {
  fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
    match self {
      Self::SPI(error) => Some(error),
      _ => None,
    }
  }
}

impl From<io::Error> for Error {
  fn from(error: io::Error) -> Self {
    Error::SPI(error)
  }
}

/// A result wrapper for errors from the MS5611-01BA03 barometer.
pub type Result<T> = std::result::Result<T, Error>;

/// An embedded, 128-bit, read-only memory module containing information about
/// the sensor.
#[derive(Clone, Debug, Default, Eq, Hash, PartialEq)]
pub struct PROM {
  /// Reserved for the manufacturer.
  pub factory_data: u16,

  /// Pressure sensitivity.
  pub sens_t1: u16,

  /// Pressure offset.
  pub off_t1: u16,

  /// Temperature coefficient of pressure sensitivity.
  pub tcs: u16,

  /// Temperature coefficient of pressure offset.
  pub tco: u16,

  /// Reference temperature.
  pub t_ref: u16,

  /// Temperature coefficient of the temperature.
  pub tempsens: u16,

  /// A 4-bit cyclic rundundancy check code used to validate the PROM contents.
  ///
  /// The CRC is only the last 4 bits of this field. The other bits are not
  /// guaranteed to be zeroed.
  ///
  /// TODO: Implement validation of the PROM using this CRC.
  pub crc: u16,
}

/// A data channel that can be read from the MS5611.
#[derive(Clone, Copy, Debug, Eq, Hash, PartialEq)]
pub enum Channel {
  /// The pressure channel, corresponding to D1.
  Pressure,

  /// The temperature channel, corresponding to D2.
  Temperature,
}

/// A single pressure and temperature reading from an MS5611-01BA03 barometer.
pub struct Reading {
  /// A barometer's measured pressure, in millibar.
  pub pressure: f64,

  /// A barometer's measured temperature, in degrees Celsius.
  pub temperature: f64,
}

/// Controls a physical MS5611-01BA03 barometric pressure meter over SPI.
#[derive(Debug)]
pub struct MS5611 {
  /// The underlying SPI device corresponding to the barometer.
  spi: Spidev,

  /// The instant at which the last conversion started.
  conversion_start: Option<Instant>,

  /// The channel that was last converted.
  ///
  /// The value read from this channel will be the next read. If this is
  /// temperature, then it should be stored.
  last_converted: Option<Channel>,

  /// Data from the barometer's embedded ROM holding calibration coefficients.
  prom: PROM,

  /// The over-sampling rate.
  ///
  /// This field controls the number of times that the signal is sampled over
  /// during conversion. Higher values correlate with reduced noise.
  osr: u16,

  /// The offset calculated in a temperature reading to adjust pressure.
  offset: i64,

  /// The sensitivity calculated in a temperature reading to adjust pressure.
  sensitivity: i64,
}

impl MS5611 {
  /// Constructs a new `MS5611` given the path to the device file corresponding
  /// to the barometer's SPI bus.
  pub fn new(bus: &str, osr: u16) -> Result<Self> {
    let mut spi = Spidev::open(bus)?;

    // SPI options specified by the datasheet.
    let options = SpidevOptions::new()
      .bits_per_word(8)
      .max_speed_hz(20_000_000)
      .lsb_first(false)
      .mode(SpiModeFlags::SPI_MODE_0)
      .build();

    spi.configure(&options)?;

    let mut barometer = MS5611 {
      spi,
      osr,
      conversion_start: None,
      last_converted: None,

      // This PROM object will be immediately overwritten.
      prom: PROM::default(),

      // Default offset and sensitivity to zero.
      //
      // This may produce undesirable effects if pressure is read before
      // temperature. Otherwise, these values will be overwritten.
      offset: 0,
      sensitivity: 0,
    };

    // The datasheet specifies that the barometer should be reset to guarantee
    // that the PROM is loaded into an internal register.
    barometer.reset()?;
    barometer.prom = barometer.read_prom()?;

    Ok(barometer)
  }

  /// Returns the maximum conversion time for the set OSR, specified by the
  /// datasheet.
  pub fn conversion_time(&self) -> Duration {
    match self.osr {
      256 => Duration::from_micros(600),
      512 => Duration::from_micros(1170),
      1024 => Duration::from_micros(2280),
      2048 => Duration::from_micros(4540),
      4096 => Duration::from_micros(9040),
      _ => unreachable!(),
    }
  }

  /// Returns the over-sampling rate.
  pub fn osr(&self) -> u16 {
    self.osr
  }

  /// Sets the over-sampling rate, checking that it is valid.
  pub fn set_osr(&mut self, osr: u16) -> Result<()> {
    if ![256, 512, 1024, 2048, 4096].contains(&self.osr) {
      return Err(Error::OSRInvalid(osr));
    }

    self.osr = osr;
    Ok(())
  }

  /// Resets the barometer to its default state.
  pub fn reset(&self) -> Result<()> {
    let mut transfer = SpidevTransfer::write(&[0x1e]);
    self.spi.transfer(&mut transfer)?;
    Ok(())
  }

  /// Reads the ROM on MS5611's internal barometer containing calibration data.
  pub fn read_prom(&self) -> Result<PROM> {
    Ok(PROM {
      factory_data: self.read_prom_address(0)?,
      sens_t1: self.read_prom_address(1)?,
      off_t1: self.read_prom_address(2)?,
      tcs: self.read_prom_address(3)?,
      tco: self.read_prom_address(4)?,
      t_ref: self.read_prom_address(5)?,
      tempsens: self.read_prom_address(6)?,
      crc: self.read_prom_address(7)?,
    })
  }

  /// Reads an individual value from the MS5611's embedded ROM.
  fn read_prom_address(&self, address: u8) -> Result<u16> {
    if address > 0b111 {
      return Err(Error::PROMAddressInvalid(address));
    }

    // Due to the address range restriction, this command translates into a byte
    // 0xA0 - 0xAE.
    let tx = [0xA0 | address << 1, 0x00, 0x00];
    let mut rx = [0x00; 3];
    let mut transfer = SpidevTransfer::read_write(&tx, &mut rx);
    self.spi.transfer(&mut transfer)?;

    // The response comes in as a 16-bit big-endian integer.
    // Since this is split into two bytes by spidev, it must be recombined.
    //
    // The first byte of the response can be discarded, as it corresponds to the
    // data on MISO during the transmission of the command on MOSI.
    let value = (rx[1] as u16) << 8 | (rx[2] as u16);
    Ok(value)
  }

  /// Performs a conversion followed by an ADC read on the given data channel.
  pub fn convert(&mut self, channel: Channel) -> Result<()> {
    let d = match channel {
      Channel::Pressure => 0,
      Channel::Temperature => 1,
    };

    let osr_bits = (7 - self.osr.leading_zeros()) as u8;
    let tx = [0x40 | d << 4 | osr_bits << 1];

    let mut transfer = SpidevTransfer::write(&tx);
    self.spi.transfer(&mut transfer)?;

    self.last_converted = Some(channel);
    Ok(())
  }

  /// Reads the raw value that has been prepared by a conversion command.
  fn read_raw(&mut self) -> Result<u32> {
    let Some(conversion_start) = self.conversion_start else {
      return Err(Error::ConversionFailed);
    };

    // The remaining wait until the end of the last conversion.
    let wait = self.conversion_time() - (Instant::now() - conversion_start);

    // If not enough time has passed since the start of the conversion to
    // guarantee that it's complete, then wait the remaining time.
    if !wait.is_zero() {
      thread::sleep(wait);
    }

    let tx = [0x00; 4];
    let mut rx = [0x00; 4];

    let mut transfer = SpidevTransfer::read_write(&tx, &mut rx);
    self.spi.transfer(&mut transfer)?;

    // Reset the last conversion channel.
    //
    // This generates the correct behavior for a premature read.
    self.last_converted = None;

    // See comment in `read_prom_address`.
    let value = (rx[1] as u32) << 16 | (rx[2] as u32) << 8 | (rx[3] as u32);

    // As specified in the datasheet, a zero value indicates that the read
    // occurred before the conversion was finished. This invalidates both the
    // read and the conversion.
    if value == 0 {
      return Err(Error::ConversionFailed);
    }

    Ok(value)
  }

  /// Reads the barometer's pressure, in millibar.
  pub fn read_pressure(&mut self) -> Result<f64> {
    // If the last conversion was not a pressure conversion, then a pressure
    // conversion is necessary before the read.
    if self.last_converted != Some(Channel::Pressure) {
      if let Some(channel) = self.last_converted {
        warn!("wasted conversion on {channel:?}");
      }

      self.convert(Channel::Pressure)?;
    }

    // The raw digital pressure value.
    let d1 = self.read_raw()? as i64;

    // Temperature-compensated pressure.
    // (10 - 1200 mbar with 0.01 mbar resolution)
    let p = (((d1 * self.sensitivity) >> 21) - self.offset) >> 15;

    // Convert to floating-point millibar.
    Ok((p as f64) * 0.01)
  }

  /// Reads the barometer's temperature, in degrees Celsius.
  ///
  /// **NOTE**: If the proper channel conversion has not been initiated before
  /// calling this method, then the conversion will be performed in the method,
  /// which may take up to 9 ms.
  pub fn read_temperature(&mut self) -> Result<f64> {
    // If the last conversion was not a temperature conversion, then a
    // temperature conversion is necessary before the read.
    if self.last_converted != Some(Channel::Temperature) {
      if let Some(channel) = self.last_converted {
        warn!("wasted conversion on {channel:?}");
      }

      self.convert(Channel::Temperature)?;
    }

    // The raw digital temperature value.
    let d2 = self.read_raw()? as i64;

    // Difference between the actual and reference temperature.
    let dt = d2 - ((self.prom.t_ref as i64) << 8);

    // Actual temperature.
    // (-40 - 85 C with 0.01 C resolution)
    let mut temp = 2000 + (dt * (self.prom.tempsens as i64)) >> 23;

    // Pressure offset at actual temperature.
    let mut off = ((self.prom.off_t1 as i64) << 16)
      + ((self.prom.tco as i64 * dt) >> 7);

    // Sensitivity at actual temperature.
    let mut sens = ((self.prom.sens_t1 as i64) << 15)
      + ((self.prom.tcs as i64 * dt) >> 8);

    // The datasheet calls for additional compensations for second-order effects
    // at temperatures of less than 20 C and further at less than -15 C.
    //
    // While these may be uncommon, branch prediction will effectively eliminate
    // these checks if they are infrequently used.
    if temp < 2000 {
      // Normalize the temperature because it is used frequently in later
      // calculations.
      let norm = temp - 2000;
      let norm2 = norm * norm;

      let t2 = (dt * dt) >> 31;
      let mut off2 = (5 * norm2) >> 1;
      let mut sens2 = (5 * norm2) >> 2;

      if temp < -1500 {
        let norm = temp + 1500;
        let norm2 = norm * norm;

        off2 += 7 * norm2;
        sens2 += (11 * norm2) >> 1;
      }

      // Apply offsets due to second-order effects.
      temp -= t2;
      off -= off2;
      sens -= sens2;
    }

    self.offset = off;
    self.sensitivity = sens;

    // Convert to floating-point Celsius.
    Ok((temp as f64) * 0.01)
  }
}
