//! Firmware driver for the AD8403 four-channel digital potentiometer.
//!
//! Datasheet: <https://www.analog.com/media/en/technical-documentation/data-sheets/AD8400_8402_8403.pdf>
//!
//! Each of the four RDACs is a 256-position potentiometer. A write is a 10-bit
//! serial word, two address bits followed by eight data bits, clocked in MSB
//! first in SPI mode 0. The part keeps the last 10 bits it receives before CS
//! rises, so a normal 2-byte transfer works: the 6 leading bits are ignored.
//!
//! The AD8403 cannot be read back, and its wipers power up at random
//! positions. This driver therefore writes every channel during construction
//! and keeps a copy of the last code written to each one.
//!
//! The SHDN and RS pins must be held high. If SHDN is low, terminal A opens and
//! the wiper connects to B. If RS is low, every wiper is forced to midscale.
//! Neither shows up in software, because the part has no readback.

#![warn(missing_docs)]

use std::{fmt, io};

use common::comm::gpio::{GpioPin, Pin, PinMode, PinValue};
use spidev::{SpiModeFlags, Spidev, SpidevOptions, SpidevTransfer};

/// Number of potentiometers (RDAC1 through RDAC4) in one AD8403.
pub const CHANNELS: usize = 4;

/// Wiper code for the midpoint of the resistor, which the RS pin also selects.
pub const MIDSCALE: u8 = 0x80;

/// SPI clock used for the AD8403. The datasheet allows up to 10 MHz.
const SPI_SPEED_HZ: u32 = 1_000_000;

/// Error originating from the AD8403 digital potentiometer.
#[derive(Debug)]
pub enum Error {
    /// Indicates that the channel passed as an argument is invalid.
    /// Valid channels range from 0 (RDAC1) to 3 (RDAC4).
    ChannelInvalid(u8),

    /// Serial peripheral interface error.
    SPI(io::Error),
}

impl fmt::Display for Error {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Error::ChannelInvalid(channel) => write!(f, "invalid channel: {channel}"),
            Error::SPI(error) => write!(f, "SPI error: {error}"),
        }
    }
}

impl std::error::Error for Error {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            Error::SPI(error) => Some(error),
            _ => None,
        }
    }
}

impl From<io::Error> for Error {
    fn from(err: io::Error) -> Self {
        Error::SPI(err)
    }
}

/// Result type encapsulating an error from the AD8403.
pub type Result<T> = std::result::Result<T, Error>;

/// Builds the 2-byte SPI frame that sets `channel` to `code`.
///
/// The first byte carries the address bits A1 and A0 in its lowest two bits,
/// and the second byte is the wiper code D7..D0.
fn frame(channel: u8, code: u8) -> Result<[u8; 2]> {
    if channel as usize >= CHANNELS {
        return Err(Error::ChannelInvalid(channel));
    }

    Ok([channel, code])
}

/// Controls a hardware AD8403 digital potentiometer over SPI.
pub struct AD8403 {
    /// The SPI bus connected to the AD8403.
    spi: Spidev,

    /// An optional GPIO chip select pin.
    cs: Option<Box<dyn GpioPin>>,

    /// The last code written to each channel, since the AD8403 has no
    /// readback.
    wipers: [u8; CHANNELS],
}

impl AD8403 {
    /// This function is only used for BeagleBone-backed pins.
    /// For Raspberry Pi, use the new_with_gpio_pin function.
    pub fn new(bus: &str, mut cs: Option<Pin>) -> Result<Self> {
        // Configure the chip select.
        if let Some(cs) = &mut cs {
            cs.mode(PinMode::Output);
            cs.digital_write(PinValue::High);
        }

        let cs_boxed = cs.map(|pin| Box::new(pin) as Box<dyn GpioPin>);

        // Delegate to the generic constructor.
        Self::new_with_gpio_pin(bus, cs_boxed)
    }

    /// Constructs a new digital potentiometer with the specified SPI bus and
    /// generic chip select pin implementing `GpioPin`.
    ///
    /// Pass `None` for `cs` when the device tree gives the kernel the chip
    /// select line, for example `dtoverlay=spi5-1cs` on the HITL test board,
    /// which uses GPIO12.
    ///
    /// Every channel is set to [`MIDSCALE`], because the wipers power up at
    /// random positions.
    pub fn new_with_gpio_pin(bus: &str, cs: Option<Box<dyn GpioPin>>) -> Result<Self> {
        // Configure the SPI bus.
        let mut spi = Spidev::open(bus).map_err(Error::SPI)?;

        let options = SpidevOptions::new()
            .bits_per_word(8)
            .max_speed_hz(SPI_SPEED_HZ)
            .mode(SpiModeFlags::SPI_MODE_0)
            .lsb_first(false)
            .build();

        spi.configure(&options).map_err(Error::SPI)?;

        let mut driver = AD8403 {
            spi,
            cs,
            wipers: [MIDSCALE; CHANNELS],
        };

        // Put every wiper in a known position.
        driver.set_all(MIDSCALE)?;

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
        let result = self.spi.transfer(transfer);
        // Deselect even if the transfer failed, so CS is not left asserted.
        self.deselect();
        result?;

        Ok(())
    }

    /// Sets the wiper of `channel` (0 for RDAC1 through 3 for RDAC4) to
    /// `code`, where 0 puts the wiper next to terminal B and 255 next to
    /// terminal A.
    pub fn set_wiper(&mut self, channel: u8, code: u8) -> Result<()> {
        let tx = frame(channel, code)?;
        self.transfer(&mut SpidevTransfer::write(&tx))?;
        self.wipers[channel as usize] = code;

        Ok(())
    }

    /// Sets every channel's wiper to `code`.
    pub fn set_all(&mut self, code: u8) -> Result<()> {
        for channel in 0..CHANNELS as u8 {
            self.set_wiper(channel, code)?;
        }

        Ok(())
    }

    /// Returns the last code written to `channel`.
    ///
    /// The AD8403 has no readback, so this is the driver's own copy, not a
    /// value read from the device.
    pub fn wiper(&self, channel: u8) -> Result<u8> {
        self.wipers
            .get(channel as usize)
            .copied()
            .ok_or(Error::ChannelInvalid(channel))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn frame_puts_address_then_code() {
        assert_eq!(frame(0, 0x00).unwrap(), [0x00, 0x00]);
        assert_eq!(frame(2, MIDSCALE).unwrap(), [0x02, 0x80]);
        assert_eq!(frame(3, 0xFF).unwrap(), [0x03, 0xFF]);
    }

    #[test]
    fn frame_rejects_invalid_channel() {
        assert!(matches!(frame(4, 0x00), Err(Error::ChannelInvalid(4))));
        assert!(matches!(
            frame(0xFF, 0x00),
            Err(Error::ChannelInvalid(0xFF))
        ));
    }
}
