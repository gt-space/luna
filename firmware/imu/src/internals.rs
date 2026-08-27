use std::io::Error;

use common::comm::gpio::{GpioPin, PinValue::*};
use spidev::{Spidev, SpidevTransfer};

pub const DEBUG_INTERNALS: bool = false;

/// An abstraction layer around the internal pins of the device
/// used to improve syntax of the actual driver
pub struct DriverInternals {
    pub spi: Spidev,

    pub data_ready: Box<dyn GpioPin>,

    pub nreset: Box<dyn GpioPin>,

    pub nchip_select: Box<dyn GpioPin>,
}

impl DriverInternals {
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

    #[expect(
        unused,
        reason = "DriverInternals::data_ready is currently not used, but might \
                be used in the future. This function can be useful in the \
                event that it is used."
    )]
    pub fn check_data_ready(&mut self) -> bool {
        if !(DEBUG_INTERNALS) {
            self.data_ready.digital_read() == High
        } else {
            println!("  CHECKED DATA READY (ASSUMED HIGH)");
            true
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
            if index % 2 == 0 {
                output.push_str(format!("{:02x}{:02x} ", buf[index + 1], byte).as_str());
            } else {
                continue;
            }
        }
        output.push(']');
        output
    }

    /// Write the bytes in tx_buf to the spi device (MOSI) and reads the output
    /// of the device (MISO) at the same time
    ///
    /// Useful for commands that require both read and write (such as sending
    /// a command to tell the spi device to read register, and then recording
    /// it's response)
    pub fn spi_transfer(&mut self, tx_buf: &[u8], rx_buf: &mut [u8]) -> Result<(), Error> {
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
