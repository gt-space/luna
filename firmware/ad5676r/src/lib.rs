//! Firmware driver for the AD5676R octal 16-bit digital-to-analog converter.
//!
//! Datasheet: <https://www.analog.com/media/en/technical-documentation/data-sheets/AD5672R_5676R.pdf>
//!
//! Every write is a 24-bit frame, clocked in MSB first on the falling edge of
//! SCLK: a 4-bit command, a 4-bit channel address, then a 16-bit data word.
//!
//! Each channel has two registers. The input register holds a staged value,
//! and the DAC register drives the output pin. [`AD5676R::set`] writes both at
//! once. [`AD5676R::stage`] writes only the input register, and
//! [`AD5676R::latch`] then moves every staged channel to its output with a
//! single command, so they change together.
//!
//! Staging needs the LDAC pin high. The constructor drives the LDAC pin high if
//! one is given, and sets the LDAC mask so every channel ignores LDAC
//! transitions. The datasheet says the mask is ignored when LDAC is held low,
//! so on a board where LDAC is pulled low and not driven, staged codes reach
//! the outputs right away.
//!
//! The output voltage is a fraction of a reference voltage. The chip has an
//! internal 2.5 V reference, which is on at power-up. The driver can switch
//! it off instead, so an external reference wired to the VREF pin sets the
//! scale. With the external option and nothing driving VREF, the outputs are
//! meaningless. See [`Reference::External`] for what the external source has
//! to tolerate.
//!
//! The SDO line is not connected on the HITL test board, so the outputs cannot
//! be read back. This driver keeps its own copy of the last code written to
//! each channel.
//!
//! The RESET pin must be held high. A falling edge on RESET sends every output
//! back to its power-on value, and while RESET is low the outputs cannot be
//! updated. Software has no way to see either, so [`AD5676R::output`] can
//! report a code the pin is not actually driving.

#![warn(missing_docs)]

use std::{fmt, io, thread, time::Duration};

use common::comm::gpio::{GpioPin, Pin, PinMode, PinValue};
use spidev::{SpiModeFlags, Spidev, SpidevOptions, SpidevTransfer};

/// Number of DAC channels (VOUT0 through VOUT7) in one AD5676R.
pub const CHANNELS: usize = 8;

/// Voltage of the internal reference, which is enabled at power-up.
pub const INTERNAL_VREF: f64 = 2.5;

/// SPI clock used for the AD5676R. The datasheet allows up to 50 MHz.
const SPI_SPEED_HZ: u32 = 1_000_000;

/// Data word the software reset command requires before it will execute.
const SOFTWARE_RESET_KEY: u16 = 0x1234;

/// Time to wait after a software reset before the next write. The datasheet
/// gives no settling time for the software reset command. The nearest
/// figures are 90 ns for the RESET pin and 5.5 us to leave power-down, so
/// 1 ms is a wide margin.
const RESET_DELAY: Duration = Duration::from_millis(1);

/// Highest external reference voltage accepted: the chip's maximum supply.
const EXTERNAL_VREF_MAX: f64 = 5.5;

/// Reference enable bit (DB0) of the reference setup register. Setting it
/// switches the internal reference off.
const REFERENCE_INTERNAL_OFF: u16 = 1 << 0;

/// Error originating from the AD5676R DAC.
#[derive(Debug)]
pub enum Error {
    /// Indicates that the channel passed as an argument is invalid.
    /// Valid channels range from 0 (VOUT0) to 7 (VOUT7).
    ChannelInvalid(u8),

    /// Indicates that the requested voltage is outside the output span.
    VoltageOutOfRange(f64),

    /// Indicates that the external reference voltage is not a usable
    /// voltage. Holds the value that was given.
    ReferenceInvalid(f64),

    /// Serial peripheral interface error.
    SPI(io::Error),
}

impl fmt::Display for Error {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Error::ChannelInvalid(channel) => write!(f, "invalid channel: {channel}"),
            Error::VoltageOutOfRange(volts) => write!(f, "voltage out of range: {volts} V"),
            Error::ReferenceInvalid(volts) => write!(f, "invalid external reference: {volts} V"),
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

/// Result type encapsulating an error from the AD5676R.
pub type Result<T> = std::result::Result<T, Error>;

/// The reference voltage that sets the output scale.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Reference {
    /// The internal 2.5 V reference, which is on at power-up.
    Internal,

    /// An external reference wired to the VREF pin. Holds its voltage in
    /// volts. It must not be above the chip's supply, or above half the
    /// supply with [`Span::TwiceVref`].
    ///
    /// The driver switches the internal reference off, but the internal
    /// reference drives the VREF pin from power-up until the driver is
    /// constructed, and possibly for about 1 ms after every reset. Connect
    /// the external source through a series resistor or a current limit, or
    /// only after the driver is constructed, and do not switch back to
    /// [`Reference::Internal`] while the source is still driving the pin.
    External(f64),
}

impl Reference {
    /// The reference voltage in volts.
    pub fn volts(self) -> f64 {
        match self {
            Reference::Internal => INTERNAL_VREF,
            Reference::External(volts) => volts,
        }
    }

    /// Returns an error when the external voltage is not a usable voltage.
    fn validate(self) -> Result<()> {
        match self {
            Reference::External(volts) if !(volts > 0.0 && volts <= EXTERNAL_VREF_MAX) => {
                Err(Error::ReferenceInvalid(volts))
            }
            _ => Ok(()),
        }
    }

    /// Data word for the reference setup command that selects this reference.
    fn setup_word(self) -> u16 {
        match self {
            Reference::Internal => 0,
            Reference::External(_) => REFERENCE_INTERNAL_OFF,
        }
    }
}

/// Output range of every channel, set by the GAIN pin on the TSSOP package.
///
/// The LFCSP and WLCSP packages have no GAIN pin and use a gain bit instead.
/// This driver never sets that bit, so it stays 0 and only [`Span::Vref`] is
/// correct on those packages.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Span {
    /// GAIN tied to GND: 0 V up to VREF, which is 2.5 V with the internal
    /// reference. The HITL test board uses this span.
    Vref,

    /// GAIN tied to VLOGIC: 0 V up to twice VREF, which is 5 V with the
    /// internal reference.
    TwiceVref,
}

impl Span {
    /// Voltage that a full 16-bit code (65536) would produce with a
    /// reference of `vref` volts. The largest real code, 65535, is one step
    /// below it.
    pub fn full_scale(self, vref: f64) -> f64 {
        match self {
            Span::Vref => vref,
            Span::TwiceVref => 2.0 * vref,
        }
    }
}

/// Commands from Table 10 of the datasheet that this driver uses.
#[derive(Clone, Copy, Debug)]
enum Command {
    /// Write to the input register of one channel (depends on LDAC).
    WriteInput = 0b0001,

    /// Copy input registers to DAC registers for the channels set in data
    /// bits D7 to D0.
    UpdateFromInput = 0b0010,

    /// Write to the input and DAC registers of one channel.
    WriteAndUpdate = 0b0011,

    /// Load the LDAC mask register from data bits D7 to D0.
    LdacMask = 0b0101,

    /// Software reset to the power-on state (needs the 0x1234 key).
    SoftwareReset = 0b0110,

    /// Write the internal reference and gain setup register.
    ReferenceSetup = 0b0111,

    /// Write the same code to the input and DAC registers of every channel.
    WriteAndUpdateAll = 0b1011,
}

/// Builds the 3-byte SPI frame for `command`, `address` and `data`.
///
/// The first byte carries the command in its upper four bits and the address
/// in its lower four bits. The next two bytes are the data word, high byte
/// first.
fn frame(command: Command, address: u8, data: u16) -> [u8; 3] {
    let [high, low] = data.to_be_bytes();
    [((command as u8) << 4) | (address & 0x0F), high, low]
}

/// Returns an error unless `channel` is between 0 and 7.
fn check_channel(channel: u8) -> Result<()> {
    if channel as usize >= CHANNELS {
        return Err(Error::ChannelInvalid(channel));
    }

    Ok(())
}

/// Converts `volts` to the nearest code for a span whose full scale is
/// `full_scale` volts, or returns an error when it is outside the span.
fn volts_to_code(volts: f64, full_scale: f64) -> Result<u16> {
    if !(0.0..=full_scale).contains(&volts) {
        return Err(Error::VoltageOutOfRange(volts));
    }

    // VOUT = full_scale * code / 65536, so the top of the span rounds to
    // 65536 and is clamped to the largest code the chip accepts.
    let code = (volts / full_scale * 65536.0).round();
    Ok(code.min(u16::MAX as f64) as u16)
}

/// Converts `code` to volts for a span whose full scale is `full_scale`
/// volts.
fn code_to_volts(code: u16, full_scale: f64) -> f64 {
    full_scale * code as f64 / 65536.0
}

/// Builds the D7 to D0 channel mask for every staged channel.
fn staged_mask(staged: &[Option<u16>; CHANNELS]) -> u16 {
    staged
        .iter()
        .enumerate()
        .filter(|(_, code)| code.is_some())
        .fold(0, |mask, (channel, _)| mask | (1 << channel))
}

/// Controls a hardware AD5676R DAC over SPI.
pub struct AD5676R {
    /// The SPI bus connected to the AD5676R.
    spi: Spidev,

    /// An optional GPIO chip select pin.
    cs: Option<Box<dyn GpioPin>>,

    /// An optional GPIO pin driving the active-low LDAC input, held high.
    ldac: Option<Box<dyn GpioPin>>,

    /// The output range set by the GAIN pin.
    span: Span,

    /// The reference that sets the output scale.
    reference: Reference,

    /// The last code sent to each channel's output, since the outputs cannot
    /// be read back.
    outputs: [u16; CHANNELS],

    /// Codes written with `stage` that have not been latched yet.
    staged: [Option<u16>; CHANNELS],
}

impl AD5676R {
    /// This function is only used for BeagleBone-backed pins.
    /// For Raspberry Pi, use the new_with_gpio_pins function.
    pub fn new(
        bus: &str,
        mut cs: Option<Pin>,
        mut ldac: Option<Pin>,
        span: Span,
        reference: Reference,
    ) -> Result<Self> {
        // Configure the chip select.
        if let Some(cs) = &mut cs {
            cs.mode(PinMode::Output);
            cs.digital_write(PinValue::High);
        }

        // Configure LDAC as an output so it can be held high.
        if let Some(ldac) = &mut ldac {
            ldac.mode(PinMode::Output);
            ldac.digital_write(PinValue::High);
        }

        let cs_boxed = cs.map(|pin| Box::new(pin) as Box<dyn GpioPin>);
        let ldac_boxed = ldac.map(|pin| Box::new(pin) as Box<dyn GpioPin>);

        // Delegate to the generic constructor.
        Self::new_with_gpio_pins(bus, cs_boxed, ldac_boxed, span, reference)
    }

    /// Constructs a new DAC with the specified SPI bus, generic chip select
    /// and LDAC pins implementing `GpioPin`, the span set by the GAIN pin, and
    /// the reference to use.
    /// Pins must already be configured as outputs, and `span` must match the
    /// GAIN pin.
    ///
    /// Pass `None` for `cs` when the device tree gives the kernel the chip
    /// select line, for example `dtoverlay=spi0-1cs` on the HITL test board,
    /// which uses GPIO8 and leaves GPIO7 free for LDAC.
    ///
    /// Pass `Some` for `ldac` unless LDAC is tied high on the board. On the
    /// HITL test board LDAC is pulled low, so with `None`, [`AD5676R::stage`]
    /// changes the output right away and [`AD5676R::output`] keeps reporting
    /// the old code until [`AD5676R::latch`].
    ///
    /// The constructor resets the chip, so every channel starts at 0 V, and
    /// selects `reference`. It returns [`Error::ReferenceInvalid`] when an
    /// external reference voltage is not between 0 V and the chip's maximum
    /// supply.
    pub fn new_with_gpio_pins(
        bus: &str,
        cs: Option<Box<dyn GpioPin>>,
        ldac: Option<Box<dyn GpioPin>>,
        span: Span,
        reference: Reference,
    ) -> Result<Self> {
        reference.validate()?;

        // Configure the SPI bus. Data is sampled on the falling edge of SCLK.
        let mut spi = Spidev::open(bus).map_err(Error::SPI)?;

        let options = SpidevOptions::new()
            .bits_per_word(8)
            .max_speed_hz(SPI_SPEED_HZ)
            .mode(SpiModeFlags::SPI_MODE_1)
            .lsb_first(false)
            .build();

        spi.configure(&options).map_err(Error::SPI)?;

        let mut driver = AD5676R {
            spi,
            cs,
            ldac,
            span,
            reference,
            outputs: [0; CHANNELS],
            staged: [None; CHANNELS],
        };

        // Put every channel in a known state.
        driver.reset()?;

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

    /// Sends one 24-bit frame.
    fn send(&mut self, command: Command, address: u8, data: u16) -> Result<()> {
        let tx = frame(command, address, data);
        self.transfer(&mut SpidevTransfer::write(&tx))
    }

    /// Resets the chip to a known state: LDAC driven high (if an LDAC pin was
    /// given), a software reset, the reference this driver was built with,
    /// every output at 0 V, and the LDAC mask set so every channel ignores
    /// the LDAC pin.
    ///
    /// The power-on value depends on the RSTSEL pin (zero scale or midscale),
    /// so the outputs are written to zero explicitly after the reset.
    ///
    /// If this returns an error, the cached outputs and the chip's reference
    /// selection are both unknown. A later successful `reset` restores both;
    /// `set_all` restores only the outputs.
    pub fn reset(&mut self) -> Result<()> {
        // Hold LDAC high so staged writes stay in the input registers.
        if let Some(ldac) = &mut self.ldac {
            ldac.digital_write(PinValue::High);
        }

        self.send(Command::SoftwareReset, 0, SOFTWARE_RESET_KEY)?;
        // Give the chip time to finish resetting before the next write.
        thread::sleep(RESET_DELAY);
        // The reset loads the power-on code into every input register, so
        // nothing is staged anymore.
        self.staged = [None; CHANNELS];

        // Select the reference after every reset, so the driver does not
        // depend on what the reset does to the reference setup register.
        self.send(Command::ReferenceSetup, 0, self.reference.setup_word())?;

        // Startup level: every channel starts at 0 V. This is a choice made
        // here, not a property of the chip. Change the code below (and
        // `outputs`) if a board needs to start somewhere else, such as full
        // scale.
        //
        // This command writes the input and DAC registers whatever the LDAC
        // state, so it can go before the mask.
        self.send(Command::WriteAndUpdateAll, 0, 0)?;
        self.outputs = [0; CHANNELS];

        self.send(Command::LdacMask, 0, 0x00FF)?;

        Ok(())
    }

    /// Changes the reference that sets the output scale.
    ///
    /// The codes in the chip do not change, so every output voltage moves
    /// with the new reference. See [`Reference::External`] before switching
    /// while an external source is connected.
    pub fn set_reference(&mut self, reference: Reference) -> Result<()> {
        reference.validate()?;
        self.send(Command::ReferenceSetup, 0, reference.setup_word())?;
        self.reference = reference;

        Ok(())
    }

    /// Sets the output of `channel` (0 for VOUT0 through 7 for VOUT7) to
    /// `code` right away. 0 is 0 V and 65535 is one step below full scale.
    pub fn set(&mut self, channel: u8, code: u16) -> Result<()> {
        check_channel(channel)?;
        self.send(Command::WriteAndUpdate, channel, code)?;

        // This command writes the input register too, so nothing is staged.
        self.outputs[channel as usize] = code;
        self.staged[channel as usize] = None;

        Ok(())
    }

    /// Sets every channel's output to `code` with one command.
    pub fn set_all(&mut self, code: u16) -> Result<()> {
        self.send(Command::WriteAndUpdateAll, 0, code)?;

        self.outputs = [code; CHANNELS];
        self.staged = [None; CHANNELS];

        Ok(())
    }

    /// Sets the output of `channel` to the code nearest `volts`.
    pub fn set_volts(&mut self, channel: u8, volts: f64) -> Result<()> {
        let code = self.volts_to_code(volts)?;
        self.set(channel, code)
    }

    /// Loads `code` into the input register of `channel` without changing
    /// its output, as long as LDAC is held high (see the crate docs). Call
    /// [`AD5676R::latch`] to apply every staged channel at once.
    pub fn stage(&mut self, channel: u8, code: u16) -> Result<()> {
        check_channel(channel)?;
        self.send(Command::WriteInput, channel, code)?;
        self.staged[channel as usize] = Some(code);

        Ok(())
    }

    /// Moves every staged channel to its output with one command, so they all
    /// change together. Does nothing when no channel is staged.
    pub fn latch(&mut self) -> Result<()> {
        let mask = staged_mask(&self.staged);
        if mask == 0 {
            return Ok(());
        }

        self.send(Command::UpdateFromInput, 0, mask)?;

        for (output, staged) in self.outputs.iter_mut().zip(self.staged.iter_mut()) {
            if let Some(code) = staged.take() {
                *output = code;
            }
        }

        Ok(())
    }

    /// Returns the last code sent to the output of `channel`.
    ///
    /// The SDO line is not connected on the HITL test board, so this is the
    /// driver's own copy, not a value read from the device.
    pub fn output(&self, channel: u8) -> Result<u16> {
        check_channel(channel)?;
        Ok(self.outputs[channel as usize])
    }

    /// Voltage that a full 16-bit code would produce with this DAC's span and
    /// reference.
    fn full_scale(&self) -> f64 {
        self.span.full_scale(self.reference.volts())
    }

    /// Converts `volts` to the nearest code for this DAC's span and reference.
    pub fn volts_to_code(&self, volts: f64) -> Result<u16> {
        volts_to_code(volts, self.full_scale())
    }

    /// Converts `code` to the voltage it produces with this DAC's span and
    /// reference.
    pub fn code_to_volts(&self, code: u16) -> f64 {
        code_to_volts(code, self.full_scale())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn frame_puts_command_then_address_then_data() {
        assert_eq!(
            frame(Command::WriteAndUpdate, 3, 0xABCD),
            [0x33, 0xAB, 0xCD]
        );
        assert_eq!(frame(Command::WriteInput, 7, 0x8000), [0x17, 0x80, 0x00]);
        assert_eq!(frame(Command::WriteAndUpdateAll, 0, 0), [0xB0, 0x00, 0x00]);
    }

    #[test]
    fn reset_and_mask_frames_match_the_datasheet() {
        assert_eq!(
            frame(Command::SoftwareReset, 0, SOFTWARE_RESET_KEY),
            [0x60, 0x12, 0x34]
        );
        assert_eq!(frame(Command::LdacMask, 0, 0x00FF), [0x50, 0x00, 0xFF]);
    }

    #[test]
    fn check_channel_rejects_invalid_channels() {
        assert!(check_channel(0).is_ok());
        assert!(check_channel(7).is_ok());
        assert!(matches!(check_channel(8), Err(Error::ChannelInvalid(8))));
        assert!(matches!(
            check_channel(0xFF),
            Err(Error::ChannelInvalid(0xFF))
        ));
    }

    #[test]
    fn volts_round_trip_on_the_vref_span() {
        let full_scale = Span::Vref.full_scale(INTERNAL_VREF);
        assert_eq!(volts_to_code(0.0, full_scale).unwrap(), 0);
        assert_eq!(volts_to_code(1.25, full_scale).unwrap(), 0x8000);
        assert_eq!(volts_to_code(2.5, full_scale).unwrap(), u16::MAX);
        assert_eq!(code_to_volts(0x8000, full_scale), 1.25);
    }

    #[test]
    fn volts_outside_the_span_are_rejected() {
        let full_scale = Span::Vref.full_scale(INTERNAL_VREF);
        assert!(matches!(
            volts_to_code(-0.1, full_scale),
            Err(Error::VoltageOutOfRange(_))
        ));
        assert!(matches!(
            volts_to_code(2.6, full_scale),
            Err(Error::VoltageOutOfRange(_))
        ));
        assert!(volts_to_code(f64::NAN, full_scale).is_err());
        assert!(volts_to_code(5.0, Span::TwiceVref.full_scale(INTERNAL_VREF)).is_ok());
    }

    #[test]
    fn reference_setup_frames_match_the_datasheet() {
        let internal = Reference::Internal.setup_word();
        let external = Reference::External(3.0).setup_word();
        assert_eq!(
            frame(Command::ReferenceSetup, 0, internal),
            [0x70, 0x00, 0x00]
        );
        assert_eq!(
            frame(Command::ReferenceSetup, 0, external),
            [0x70, 0x00, 0x01]
        );
    }

    #[test]
    fn external_reference_sets_the_scale() {
        let full_scale = Span::Vref.full_scale(Reference::External(3.0).volts());
        assert_eq!(volts_to_code(1.5, full_scale).unwrap(), 0x8000);
        assert_eq!(code_to_volts(0x8000, full_scale), 1.5);
        assert!(volts_to_code(2.9, full_scale).is_ok());
        assert_eq!(Span::TwiceVref.full_scale(2.0), 4.0);
    }

    #[test]
    fn external_reference_must_be_a_usable_voltage() {
        assert!(Reference::Internal.validate().is_ok());
        assert!(Reference::External(3.0).validate().is_ok());
        assert!(Reference::External(0.0).validate().is_err());
        assert!(Reference::External(-2.5).validate().is_err());
        assert!(Reference::External(6.0).validate().is_err());
        assert!(Reference::External(f64::NAN).validate().is_err());
    }

    #[test]
    fn staged_mask_sets_one_bit_per_staged_channel() {
        let mut staged = [None; CHANNELS];
        assert_eq!(staged_mask(&staged), 0);

        staged[0] = Some(1);
        staged[5] = Some(2);
        staged[7] = Some(3);
        assert_eq!(staged_mask(&staged), 0b1010_0001);
    }
}
