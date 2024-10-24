use spidev::{Spidev, SpidevOptions, SpidevTransfer, SpiModeFlags};
use common::comm::gpio::{Pin, PinMode::*, PinValue::*, Gpio};
use std::{fmt, io::Error};

const DEBUG_INTERNALS : bool = false;

pub struct DriverInternals<'a> {
  spi : Spidev,
  
  data_ready : Pin<'a>,
  
  nchip_select : Pin<'a>,

  interrupt_pin: Option<Pin<'a>>
}

impl<'a> DriverInternals<'a> {
  pub fn initialize(mut spi : Spidev, data_ready : Pin<'a>, nchip_select : Pin<'a>, interrupt_pin : Pin<'a>) -> Result<DriverInternals<'a>, Error> {
		let options = SpidevOptions::new()
    .bits_per_word(16)
    .max_speed_hz(10000000)
    .mode(SpiModeFlags::SPI_MODE_0)
    .lsb_first(false)
    .build();
  
		spi.configure(&options)?;

    // Create internal structure
    let mut internals = DriverInternals {
      spi,
      data_ready,
      nchip_select,
      interrupt_pin
    };

    if !(DEBUG_INTERNALS){
      // Configure pins
      internals.nchip_select.mode(Output);
      internals.data_ready.mode(Input);
    }

    // Set pins to their defaults
    internals.disable_chip_select();

    // Return
    Ok(internals)
  }

  pub fn enable_chip_select(&mut self) {
    if !(DEBUG_INTERNALS){
      self.nchip_select.digital_write(Low);
    } else {
      println!("  !CHIP_SELECT LOW");
    }
  }

  pub fn disable_chip_select(&mut self) {
    if !(DEBUG_INTERNALS){
      self.nchip_select.digital_write(High);
    } else {
      println!("  !CHIP_SELECT HIGH");
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

  fn debug_buffer_display(buf : &[u8]) -> String {
    let mut output : String = String::with_capacity(buf.len() * 2);
    output.push('[');
    output.push(' ');
    if buf.len() % 2 == 1 {
      panic!("Why is buffer length not a multiple of 2?")
    }
    for (index, byte) in buf.iter().enumerate() {
      if (index % 2 == 0) {
        output.push_str(format!("{:02x}{:02x} ", buf[index + 1], byte).as_str());
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
  pub fn spi_transfer(&mut self, tx_buf : &[u8], rx_buf : &mut [u8]) -> Result<(), Error> {
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
  pub fn spi_write(&mut self, tx_buf : &[u8]) -> Result<(), Error> {
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

  pub fn is_interrupt_triggered(&self) -> bool {
    if let Some(ref pin) = self.interrupt_pin {
        return pin.read() == PinValue::High;
    }
    false
}

}
