use common::comm::gpio::{Gpio, Pin, PinMode::*, PinValue::*};
use spidev::{SpiModeFlags, Spidev, SpidevOptions, SpidevTransfer};
use std::{fmt, io::Error};

const DEBUG_INTERNALS: bool = false;

/// An abstraction layer around the internal pins of the device
/// used to improve syntax of the actual driver
pub struct DriverInternals {
  spi: Spidev,

  data_ready: Pin,

  nreset: Pin,

  nchip_select: Pin,
}

impl DriverInternals {
  pub fn initialize(
    mut spi: Spidev,
    data_ready: Pin,
    nreset: Pin,
    nchip_select: Pin,
  ) -> Result<DriverInternals, Error> {
    // Configure spi
    let options = SpidevOptions::new()
      .bits_per_word(16) // still page 17
      .max_speed_hz(500_000) // 2Mhz max as given on bottom left of page 17, BUT 1Mhz is max for burst
      // read.
      .mode(SpiModeFlags::SPI_MODE_3) // As given on bottom left of page 17
      .lsb_first(false) // page 17, we are in MSB mode
      .build();

    spi.configure(&options)?;

    // Create internal structure
    let mut internals = DriverInternals {
      spi,
      data_ready,
      nreset,
      nchip_select,
    };

    if !(DEBUG_INTERNALS) {
      // Configure pins
      internals.nchip_select.mode(Output);
      internals.nreset.mode(Output);
      internals.data_ready.mode(Input);
    }

    // Set pins to their defaults
    internals.disable_chip_select();
    internals.disable_reset();

    // Return
    Ok(internals)
  }

  pub fn enable_chip_select(&mut self) {
    if !(DEBUG_INTERNALS) {
      self.nchip_select.digital_write(Low);
    } else {
      println!("  !CHIP_SELECT LOW");
    }
  }

  pub fn disable_chip_select(&mut self) {
    if !(DEBUG_INTERNALS) {
      self.nchip_select.digital_write(High);
    } else {
      println!("  !CHIP_SELECT HIGH");
    }
  }

  pub fn enable_reset(&mut self) {
    if !(DEBUG_INTERNALS) {
      self.nreset.digital_write(Low);
    } else {
      println!("  !RESET LOW");
    }
  }

  pub fn disable_reset(&mut self) {
    if !(DEBUG_INTERNALS) {
      self.nreset.digital_write(High);
    } else {
      println!("  !RESET HIGH");
    }
  }

  pub fn check_data_ready(&mut self) -> bool {
    if !(DEBUG_INTERNALS) {
      return self.data_ready.digital_read() == High;
    } else {
      println!("  CHECKED DATA READY (ASSUMED HIGH)");
      return true;
    }
  }

  fn debug_buffer_display(buf: &[u8]) -> String {
    let mut output: String = String::with_capacity(buf.len() * 2);
    output.push('[');
    output.push(' ');
    if buf.len() % 2 == 1 {
      panic!("Why is buffer length not a multiple of 2?")
    }
    for (index, byte) in buf.iter().enumerate() {
      if (index % 2 == 0) {
        output
          .push_str(format!("{:02x}{:02x} ", buf[index + 1], byte).as_str());
      } else {
        continue;
      }
    }
    output.push(']');
    return output;
  }

  /// Write the bytes in tx_buf to the spi device (MOSI) and reads the output
  /// of the device (MISO) at the same time
  ///
  /// Useful for commands that require both read and write (such as sending
  /// a command to tell the spi device to read register, and then recording it's
  /// response)
  pub fn spi_transfer(
    &mut self,
    tx_buf: &[u8],
    rx_buf: &mut [u8],
  ) -> Result<(), Error> {
    self.enable_chip_select();
    if !DEBUG_INTERNALS {
      let mut transfer = SpidevTransfer::read_write(tx_buf, rx_buf);
      self.spi.transfer(&mut transfer)?;
    } else {
      println!(
        "DOING TRANSFER : \nSend :\n  {}\nReceive :\n  {}",
        Self::debug_buffer_display(tx_buf),
        Self::debug_buffer_display(rx_buf)
      );
    }
    self.disable_chip_select();
    Ok(())
  }
  /// Write the bytes in tx_buf to the spi device (on MOSI)
  ///
  /// There is notable delay between spi calls, so one cannot chain these
  /// together for spi calls
  pub fn spi_write(&mut self, tx_buf: &[u8]) -> Result<(), Error> {
    self.enable_chip_select();
    if !DEBUG_INTERNALS {
      let mut transfer = SpidevTransfer::write(tx_buf);
      self.spi.transfer(&mut transfer)?;
    } else {
      println!(
        "DOING WRITE : \nSend :\n  {}",
        Self::debug_buffer_display(tx_buf),
      );
    }
    self.disable_chip_select();
    Ok(())
  }
}
