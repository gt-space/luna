//! Firmware driver for the ADS1261 24-bit analog-to-digital converter.
//!
//! Datasheet: <https://www.ti.com/lit/ds/symlink/ads1261.pdf>
//!
//! The ADS1261 measures the voltage between two inputs picked by its input
//! multiplexer, one pair at a time. Every command is two bytes, sent in SPI
//! mode 1, and the chip answers on the bytes that follow.
//!
//! This driver runs the chip in pulse-conversion mode. Each read selects the
//! input pair, sends a START command, polls the STATUS register until the
//! DRDY bit says the conversion is done, and then reads the 24-bit result.
//! That needs neither the START pin nor the DRDY pin, which matters on the
//! HITL adapter board: START is tied low and DRDY is not connected.
//!
//! The PGA is bypassed, so inputs can be measured all the way down to ground.
//! On the HITL adapter board AINCOM is tied to ground, so a single-ended read
//! is `read_volts(Input::AinN, Input::Aincom)`.
//!
//! The CRC and STATUS-byte options stay off, and the calibration registers
//! are left at their reset values.

#![warn(missing_docs)]

use std::{
    fmt, io, thread,
    time::{Duration, Instant},
};

use common::comm::gpio::{GpioPin, Pin, PinMode, PinValue};
use spidev::{SpiModeFlags, Spidev, SpidevOptions, SpidevTransfer};

/// Voltage of the internal reference.
pub const INTERNAL_VREF: f64 = 2.5;

/// SPI clock used for the ADS1261. The datasheet allows about 10 MHz.
const SPI_SPEED_HZ: u32 = 1_000_000;

/// Upper four bits of the ID register for an ADS1261.
const DEVICE_ID: u8 = 0b1000;

/// Time to wait after a RESET command before talking to the chip again.
const RESET_DELAY: Duration = Duration::from_millis(5);

/// Time the internal reference needs to settle after it is switched on, with
/// the 10 uF capacitor on REFOUT.
const REFERENCE_SETTLE: Duration = Duration::from_millis(100);

/// Highest analog supply voltage the chip allows.
const AVDD_MAX: f64 = 5.25;

/// Time to wait between polls of the DRDY bit.
const POLL_INTERVAL: Duration = Duration::from_micros(200);

/// Extra time allowed on top of the expected conversion time. It also covers
/// the fixed processing overhead and the conversion-start delay.
const TIMEOUT_MARGIN: Duration = Duration::from_millis(50);

/// DRDY bit of the STATUS register: conversion data is new since the last
/// read.
const STATUS_DRDY: u8 = 1 << 2;

/// RESET bit of the STATUS register: the chip has reset since the bit was
/// last cleared.
const STATUS_RESET: u8 = 1 << 0;

/// CONVRT bit of the MODE1 register: pulse (one-shot) conversions.
const MODE1_PULSE: u8 = 1 << 4;

/// Reset value of the DELAY field of MODE1: a 50 us conversion-start delay.
const MODE1_DELAY_50US: u8 = 0b0001;

/// BYPASS bit of the PGA register.
const PGA_BYPASS: u8 = 1 << 7;

/// Error originating from the ADS1261 ADC.
#[derive(Debug)]
pub enum Error {
    /// Indicates that the ID register did not identify an ADS1261. Holds the
    /// value that was read.
    DeviceIdUnexpected(u8),

    /// Indicates that a register did not read back the value written to it.
    RegisterMismatch {
        /// Address of the register.
        address: u8,
        /// Value that was written.
        wrote: u8,
        /// Value that was read back.
        read: u8,
    },

    /// Indicates that the FIR filter was requested at a data rate where the
    /// chip does not offer it. It is only offered at 2.5, 5, 10 and 20 SPS.
    FilterInvalid,

    /// Indicates that the supply voltage given for the AVDD reference is not
    /// a real supply voltage. Holds the value that was given.
    ReferenceInvalid(f64),

    /// Indicates that the chip reset on its own since the driver configured
    /// it, so its settings are gone. Call `reset` to configure it again.
    DeviceReset,

    /// Indicates that a conversion did not finish in the expected time.
    ConversionTimeout,

    /// Serial peripheral interface error.
    SPI(io::Error),
}

impl fmt::Display for Error {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Error::DeviceIdUnexpected(id) => write!(f, "unexpected device id: {id:#04X}"),
            Error::RegisterMismatch {
                address,
                wrote,
                read,
            } => write!(
                f,
                "register {address:#04X} read back {read:#04X} after writing {wrote:#04X}"
            ),
            Error::FilterInvalid => {
                write!(f, "the FIR filter is only offered at 2.5, 5, 10 and 20 SPS")
            }
            Error::ReferenceInvalid(volts) => write!(f, "invalid AVDD reference: {volts} V"),
            Error::DeviceReset => write!(f, "the device reset and lost its configuration"),
            Error::ConversionTimeout => write!(f, "conversion timed out"),
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

/// Result type encapsulating an error from the ADS1261.
pub type Result<T> = std::result::Result<T, Error>;

/// Commands from Table 16 of the datasheet that this driver uses.
#[derive(Clone, Copy, Debug)]
enum Command {
    /// Reset the chip and its registers.
    Reset = 0x06,

    /// Start a conversion.
    Start = 0x08,

    /// Read the latest conversion data.
    ReadData = 0x12,

    /// Read one register. The register address is added to this value.
    ReadRegister = 0x20,

    /// Write one register. The register address is added to this value.
    WriteRegister = 0x40,
}

/// Registers from Table 29 of the datasheet that this driver uses.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Register {
    /// Device and revision ID.
    Id = 0x00,

    /// Status flags, including DRDY.
    Status = 0x01,

    /// Data rate and digital filter.
    Mode0 = 0x02,

    /// Conversion mode and start delay.
    Mode1 = 0x03,

    /// Reference selection.
    Ref = 0x06,

    /// PGA gain and bypass.
    Pga = 0x10,

    /// Input multiplexer.
    InpMux = 0x11,
}

/// A selection of the input multiplexer, for either side of a measurement.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Input {
    /// The AINCOM pin, tied to ground on the HITL adapter board.
    Aincom = 0b0000,
    /// Analog input 0.
    Ain0 = 0b0001,
    /// Analog input 1.
    Ain1 = 0b0010,
    /// Analog input 2.
    Ain2 = 0b0011,
    /// Analog input 3.
    Ain3 = 0b0100,
    /// Analog input 4.
    Ain4 = 0b0101,
    /// Analog input 5.
    Ain5 = 0b0110,
    /// Analog input 6.
    Ain6 = 0b0111,
    /// Analog input 7.
    Ain7 = 0b1000,
    /// Analog input 8.
    Ain8 = 0b1001,
    /// Analog input 9.
    Ain9 = 0b1010,
}

/// The reference voltage that conversions are measured against.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Reference {
    /// The internal 2.5 V reference. It needs a 10 uF capacitor on REFOUT,
    /// which the HITL adapter board has.
    Internal,

    /// The analog supply (AVDD to AVSS), which is the chip's reset default.
    /// Holds the supply voltage in volts, 5.0 on the HITL adapter board.
    /// Readings are only as accurate as that number.
    Avdd(f64),
}

impl Reference {
    /// The reference voltage in volts.
    pub fn volts(self) -> f64 {
        match self {
            Reference::Internal => INTERNAL_VREF,
            Reference::Avdd(volts) => volts,
        }
    }

    /// Returns an error when the AVDD voltage is not a real supply voltage.
    fn validate(self) -> Result<()> {
        match self {
            Reference::Avdd(volts) if !(volts > 0.0 && volts <= AVDD_MAX) => {
                Err(Error::ReferenceInvalid(volts))
            }
            _ => Ok(()),
        }
    }

    /// Value of the REF register that selects this reference.
    fn register_value(self) -> u8 {
        match self {
            // REFENB set, positive and negative muxes on the internal reference.
            Reference::Internal => 0b0001_0000,
            // Internal reference off, muxes on AVDD and AVSS.
            Reference::Avdd(_) => 0b0000_0101,
        }
    }
}

/// Conversions per second. Rates of 14400 SPS and up always use the sinc5
/// filter. 40000 SPS is left out: it switches the internal oscillator to
/// 10.24 MHz, and the HITL board has no use for it.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum DataRate {
    /// 2.5 conversions per second.
    Sps2_5 = 0b00000,
    /// 5 conversions per second.
    Sps5 = 0b00001,
    /// 10 conversions per second.
    Sps10 = 0b00010,
    /// 16.6 conversions per second.
    Sps16_6 = 0b00011,
    /// 20 conversions per second, the reset default.
    Sps20 = 0b00100,
    /// 50 conversions per second.
    Sps50 = 0b00101,
    /// 60 conversions per second.
    Sps60 = 0b00110,
    /// 100 conversions per second.
    Sps100 = 0b00111,
    /// 400 conversions per second.
    Sps400 = 0b01000,
    /// 1200 conversions per second.
    Sps1200 = 0b01001,
    /// 2400 conversions per second.
    Sps2400 = 0b01010,
    /// 4800 conversions per second.
    Sps4800 = 0b01011,
    /// 7200 conversions per second.
    Sps7200 = 0b01100,
    /// 14400 conversions per second.
    Sps14400 = 0b01101,
    /// 19200 conversions per second.
    Sps19200 = 0b01110,
    /// 25600 conversions per second.
    Sps25600 = 0b01111,
}

impl DataRate {
    /// The rate in conversions per second.
    pub fn sps(self) -> f64 {
        match self {
            DataRate::Sps2_5 => 2.5,
            DataRate::Sps5 => 5.0,
            DataRate::Sps10 => 10.0,
            DataRate::Sps16_6 => 16.6,
            DataRate::Sps20 => 20.0,
            DataRate::Sps50 => 50.0,
            DataRate::Sps60 => 60.0,
            DataRate::Sps100 => 100.0,
            DataRate::Sps400 => 400.0,
            DataRate::Sps1200 => 1200.0,
            DataRate::Sps2400 => 2400.0,
            DataRate::Sps4800 => 4800.0,
            DataRate::Sps7200 => 7200.0,
            DataRate::Sps14400 => 14400.0,
            DataRate::Sps19200 => 19200.0,
            DataRate::Sps25600 => 25600.0,
        }
    }
}

/// The digital filter. Higher sinc orders reject more noise but take more
/// conversion periods to settle.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Filter {
    /// Sinc1: settles in one conversion period.
    Sinc1 = 0b000,
    /// Sinc2: settles in two conversion periods.
    Sinc2 = 0b001,
    /// Sinc3: settles in three conversion periods.
    Sinc3 = 0b010,
    /// Sinc4: settles in four conversion periods.
    Sinc4 = 0b011,
    /// FIR, the reset default: rejects 50 Hz and 60 Hz together. Only
    /// offered at 2.5, 5, 10 and 20 SPS.
    Fir = 0b100,
}

/// Builds the 2-byte frame for a command that carries no data.
fn command_frame(command: Command) -> [u8; 2] {
    [command as u8, 0x00]
}

/// Builds the 3-byte frame that reads `register`. The register data comes
/// back on the third byte.
fn read_register_frame(register: Register) -> [u8; 3] {
    [Command::ReadRegister as u8 + register as u8, 0x00, 0x00]
}

/// Builds the 2-byte frame that writes `value` to `register`.
fn write_register_frame(register: Register, value: u8) -> [u8; 2] {
    [Command::WriteRegister as u8 + register as u8, value]
}

/// Builds the INPMUX register value for a positive and negative input.
fn inpmux_value(positive: Input, negative: Input) -> u8 {
    ((positive as u8) << 4) | negative as u8
}

/// Builds the MODE0 register value, or returns an error when the FIR filter
/// is requested at a rate other than 2.5, 5, 10 or 20 SPS.
fn mode0_value(rate: DataRate, filter: Filter) -> Result<u8> {
    let fir_rate = matches!(
        rate,
        DataRate::Sps2_5 | DataRate::Sps5 | DataRate::Sps10 | DataRate::Sps20
    );
    if filter == Filter::Fir && !fir_rate {
        return Err(Error::FilterInvalid);
    }

    Ok(((rate as u8) << 3) | filter as u8)
}

/// Converts the three conversion data bytes, MSB first, to a signed code.
fn code_from_bytes(bytes: [u8; 3]) -> i32 {
    // Put the 24 bits at the top of an i32, then shift back down so the sign
    // bit is extended.
    i32::from_be_bytes([bytes[0], bytes[1], bytes[2], 0]) >> 8
}

/// Converts `code` to volts for a reference of `vref` volts at a gain of 1.
fn code_to_volts(code: i32, vref: f64) -> f64 {
    code as f64 * vref / 8_388_608.0
}

/// How long to wait for one conversion at `rate` before giving up.
///
/// Sinc4, the slowest filter, takes four conversion periods plus about 0.4 ms
/// of fixed overhead (datasheet Table 8). Five periods covers the slow rates,
/// and the fixed margin covers the overhead that dominates at the fast rates.
fn conversion_timeout(rate: DataRate) -> Duration {
    Duration::from_secs_f64(5.0 / rate.sps()) + TIMEOUT_MARGIN
}

/// Controls a hardware ADS1261 ADC over SPI.
pub struct ADS1261 {
    /// The SPI bus connected to the ADS1261.
    spi: Spidev,

    /// An optional GPIO chip select pin.
    cs: Option<Box<dyn GpioPin>>,

    /// The reference that conversions are measured against.
    reference: Reference,

    /// The data rate, which sets how long a conversion is given to finish.
    rate: DataRate,
}

impl ADS1261 {
    /// This function is only used for BeagleBone-backed pins.
    /// For Raspberry Pi, use the new_with_gpio_pin function.
    pub fn new(bus: &str, mut cs: Option<Pin>, reference: Reference) -> Result<Self> {
        // Configure the chip select.
        if let Some(cs) = &mut cs {
            cs.mode(PinMode::Output);
            cs.digital_write(PinValue::High);
        }

        let cs_boxed = cs.map(|pin| Box::new(pin) as Box<dyn GpioPin>);

        // Delegate to the generic constructor.
        Self::new_with_gpio_pin(bus, cs_boxed, reference)
    }

    /// Constructs a new ADC with the specified SPI bus, generic chip select
    /// pin implementing `GpioPin`, and reference.
    ///
    /// Pass `None` for `cs` when the device tree gives the kernel the chip
    /// select line, for example `dtoverlay=spi1-1cs` on the HITL adapter
    /// board, which uses GPIO18.
    ///
    /// The constructor resets the chip, checks its ID, and leaves it at
    /// 20 SPS with the FIR filter. With the internal reference it also waits
    /// 100 ms for the reference to settle.
    pub fn new_with_gpio_pin(
        bus: &str,
        cs: Option<Box<dyn GpioPin>>,
        reference: Reference,
    ) -> Result<Self> {
        reference.validate()?;

        // Configure the SPI bus. The ADS1261 uses SPI mode 1.
        let mut spi = Spidev::open(bus).map_err(Error::SPI)?;

        let options = SpidevOptions::new()
            .bits_per_word(8)
            .max_speed_hz(SPI_SPEED_HZ)
            .mode(SpiModeFlags::SPI_MODE_1)
            .lsb_first(false)
            .build();

        spi.configure(&options).map_err(Error::SPI)?;

        let mut driver = ADS1261 {
            spi,
            cs,
            reference,
            rate: DataRate::Sps20,
        };

        // Put the chip in a known state.
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

    /// Sends a command that carries no data.
    fn command(&mut self, command: Command) -> Result<()> {
        let tx = command_frame(command);
        self.transfer(&mut SpidevTransfer::write(&tx))
    }

    /// Reads one register.
    fn read_register(&mut self, register: Register) -> Result<u8> {
        let tx = read_register_frame(register);
        let mut rx = [0; 3];
        self.transfer(&mut SpidevTransfer::read_write(&tx, &mut rx))?;

        Ok(rx[2])
    }

    /// Writes one register and reads it back to check the write took.
    fn write_register(&mut self, register: Register, value: u8) -> Result<()> {
        let tx = write_register_frame(register, value);
        self.transfer(&mut SpidevTransfer::write(&tx))?;

        let read = self.read_register(register)?;
        if read != value {
            return Err(Error::RegisterMismatch {
                address: register as u8,
                wrote: value,
                read,
            });
        }

        Ok(())
    }

    /// Reads the latest conversion data as a signed 24-bit code.
    fn read_data(&mut self) -> Result<i32> {
        // Two command bytes, then the three data bytes.
        let tx = [Command::ReadData as u8, 0x00, 0x00, 0x00, 0x00];
        let mut rx = [0; 5];
        self.transfer(&mut SpidevTransfer::read_write(&tx, &mut rx))?;

        Ok(code_from_bytes([rx[2], rx[3], rx[4]]))
    }

    /// Resets the chip to a known state: a RESET command, an ID check, pulse
    /// conversions, the PGA bypassed, and the reference this driver was
    /// built with. The data rate and filter return to 20 SPS and FIR.
    ///
    /// The RESET command switches the internal reference off, so when this
    /// driver uses it, this waits 100 ms for it to settle again.
    pub fn reset(&mut self) -> Result<()> {
        self.command(Command::Reset)?;
        thread::sleep(RESET_DELAY);
        self.rate = DataRate::Sps20;

        let id = self.read_register(Register::Id)?;
        if id >> 4 != DEVICE_ID {
            return Err(Error::DeviceIdUnexpected(id));
        }

        self.write_register(Register::Mode1, MODE1_PULSE | MODE1_DELAY_50US)?;
        self.write_register(Register::Pga, PGA_BYPASS)?;
        self.write_register(Register::Ref, self.reference.register_value())?;
        if self.reference == Reference::Internal {
            thread::sleep(REFERENCE_SETTLE);
        }

        // Clear the RESET flag, so a later reset can be told apart from this
        // one. The other STATUS bits are live, so this write is not read back.
        let tx = write_register_frame(Register::Status, 0x00);
        self.transfer(&mut SpidevTransfer::write(&tx))?;

        Ok(())
    }

    /// Sets the data rate and digital filter. Returns an error when the FIR
    /// filter is requested at a rate other than 2.5, 5, 10 or 20 SPS.
    pub fn set_rate(&mut self, rate: DataRate, filter: Filter) -> Result<()> {
        let value = mode0_value(rate, filter)?;
        self.write_register(Register::Mode0, value)?;
        self.rate = rate;

        Ok(())
    }

    /// Changes the reference that conversions are measured against. Switching
    /// to the internal reference waits 100 ms for it to settle.
    pub fn set_reference(&mut self, reference: Reference) -> Result<()> {
        reference.validate()?;

        let switching_on = reference == Reference::Internal && self.reference != reference;
        self.write_register(Register::Ref, reference.register_value())?;
        self.reference = reference;
        if switching_on {
            thread::sleep(REFERENCE_SETTLE);
        }

        Ok(())
    }

    /// Runs one conversion of `positive` minus `negative` and returns the
    /// signed 24-bit code. Full scale is plus or minus the reference voltage,
    /// so a code of 8388607 or -8388608 means the input is at or beyond it.
    ///
    /// Returns [`Error::DeviceReset`] if the chip has reset on its own.
    pub fn read_code(&mut self, positive: Input, negative: Input) -> Result<i32> {
        self.write_register(Register::InpMux, inpmux_value(positive, negative))?;

        // Reading data clears the DRDY bit, so a result left over from an
        // earlier conversion cannot be mistaken for this one.
        let status = self.read_register(Register::Status)?;
        if status & STATUS_RESET != 0 {
            return Err(Error::DeviceReset);
        }
        if status & STATUS_DRDY != 0 {
            self.read_data()?;
        }

        self.command(Command::Start)?;

        let deadline = Instant::now() + conversion_timeout(self.rate);
        loop {
            let status = self.read_register(Register::Status)?;
            if status & STATUS_RESET != 0 {
                return Err(Error::DeviceReset);
            }
            if status & STATUS_DRDY != 0 {
                break;
            }
            if Instant::now() >= deadline {
                return Err(Error::ConversionTimeout);
            }

            thread::sleep(POLL_INTERVAL);
        }

        self.read_data()
    }

    /// Runs one conversion of `positive` minus `negative` and returns volts.
    pub fn read_volts(&mut self, positive: Input, negative: Input) -> Result<f64> {
        let code = self.read_code(positive, negative)?;
        Ok(self.code_to_volts(code))
    }

    /// Converts `code` to volts for this ADC's reference.
    pub fn code_to_volts(&self, code: i32) -> f64 {
        code_to_volts(code, self.reference.volts())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn command_frames_match_the_datasheet() {
        assert_eq!(command_frame(Command::Reset), [0x06, 0x00]);
        assert_eq!(command_frame(Command::Start), [0x08, 0x00]);
    }

    #[test]
    fn register_frames_add_the_address_to_the_opcode() {
        assert_eq!(read_register_frame(Register::Status), [0x21, 0x00, 0x00]);
        assert_eq!(read_register_frame(Register::InpMux), [0x31, 0x00, 0x00]);
        assert_eq!(write_register_frame(Register::Mode0, 0x24), [0x42, 0x24]);
        assert_eq!(
            write_register_frame(Register::Pga, PGA_BYPASS),
            [0x50, 0x80]
        );
    }

    #[test]
    fn inpmux_puts_positive_in_the_upper_nibble() {
        assert_eq!(inpmux_value(Input::Ain0, Input::Aincom), 0x10);
        assert_eq!(inpmux_value(Input::Ain4, Input::Aincom), 0x50);
        assert_eq!(inpmux_value(Input::Ain9, Input::Ain8), 0xA9);
    }

    #[test]
    fn mode0_packs_rate_and_filter() {
        // The reset value: 20 SPS with the FIR filter.
        assert_eq!(mode0_value(DataRate::Sps20, Filter::Fir).unwrap(), 0x24);
        assert_eq!(mode0_value(DataRate::Sps1200, Filter::Sinc4).unwrap(), 0x4B);
    }

    #[test]
    fn mode0_rejects_fir_at_unsupported_rates() {
        assert!(matches!(
            mode0_value(DataRate::Sps50, Filter::Fir),
            Err(Error::FilterInvalid)
        ));
        assert!(matches!(
            mode0_value(DataRate::Sps16_6, Filter::Fir),
            Err(Error::FilterInvalid)
        ));
        assert!(mode0_value(DataRate::Sps2_5, Filter::Fir).is_ok());
        assert!(mode0_value(DataRate::Sps10, Filter::Fir).is_ok());
    }

    #[test]
    fn avdd_reference_must_be_a_real_supply_voltage() {
        assert!(Reference::Internal.validate().is_ok());
        assert!(Reference::Avdd(5.0).validate().is_ok());
        assert!(Reference::Avdd(0.0).validate().is_err());
        assert!(Reference::Avdd(-1.0).validate().is_err());
        assert!(Reference::Avdd(6.0).validate().is_err());
        assert!(Reference::Avdd(f64::NAN).validate().is_err());
    }

    #[test]
    fn reference_register_values_match_the_datasheet() {
        assert_eq!(Reference::Internal.register_value(), 0x10);
        // The reset value: AVDD and AVSS.
        assert_eq!(Reference::Avdd(5.0).register_value(), 0x05);
    }

    #[test]
    fn codes_are_sign_extended() {
        assert_eq!(code_from_bytes([0x00, 0x00, 0x00]), 0);
        assert_eq!(code_from_bytes([0x00, 0x00, 0x01]), 1);
        assert_eq!(code_from_bytes([0x7F, 0xFF, 0xFF]), 8_388_607);
        assert_eq!(code_from_bytes([0xFF, 0xFF, 0xFF]), -1);
        assert_eq!(code_from_bytes([0x80, 0x00, 0x00]), -8_388_608);
    }

    #[test]
    fn codes_convert_to_volts() {
        assert_eq!(code_to_volts(0, 2.5), 0.0);
        assert_eq!(code_to_volts(4_194_304, 2.5), 1.25);
        assert_eq!(code_to_volts(-8_388_608, 2.5), -2.5);
        assert_eq!(code_to_volts(4_194_304, 5.0), 2.5);
    }

    #[test]
    fn timeout_covers_the_slowest_filter() {
        // Sinc4 at 2.5 SPS takes 1.6 s.
        assert!(conversion_timeout(DataRate::Sps2_5) > Duration::from_millis(1600));
        assert!(conversion_timeout(DataRate::Sps20) > Duration::from_millis(200));
    }
}
