//! Firmware driver for the TMP112 digital temperature sensor.
//!
//! Datasheet: <https://www.ti.com/lit/ds/symlink/tmp112.pdf>
//!
//! The TMP112 measures its own temperature and reports it over I2C. It has
//! four 16-bit registers: the temperature, a configuration register, and a
//! low and a high limit. To read or write one, the first byte sent is a
//! pointer that names the register, and the two data bytes follow, high byte
//! first.
//!
//! The sensor converts on its own, four times a second by default, so a read
//! just returns the latest result. One step of the result is 0.0625 degrees
//! Celsius.
//!
//! The sensor compares every result against the two limits. Once the
//! temperature reaches the high limit an alert turns on, and it stays on
//! until the temperature falls below the low limit. The alert shows on the
//! ALERT pin and in a status bit, which [`TMP112::alert`] reads, so the pin
//! does not have to be wired to anything.
//!
//! This driver keeps the sensor in its 12-bit mode, which holds -128 to
//! +127.9375 degrees Celsius (the part itself is rated for -55 to +150), and
//! in comparator mode with an active-low ALERT pin.

#![warn(missing_docs)]

use std::fmt;

use rppal::i2c::I2c;

/// Degrees Celsius per step of the temperature and limit registers.
pub const DEGREES_PER_STEP: f64 = 0.0625;

/// Lowest temperature this driver accepts for a limit, in degrees Celsius:
/// the sensor's minimum rated temperature. The 12-bit format itself goes down
/// to -128.
pub const TEMPERATURE_MIN: f64 = -55.0;

/// Highest temperature the 12-bit format can hold, in degrees Celsius.
pub const TEMPERATURE_MAX: f64 = 127.9375;

/// First configuration byte this driver writes: continuous conversions,
/// comparator mode, active-low alert, and one fault to trigger an alert.
const CONFIG_BYTE_1: u8 = 0x60;

/// Bits of the first configuration byte that read back what was written. The
/// one-shot and resolution bits do not.
const CONFIG_MASK_1: u8 = 0x1F;

/// Bits of the second configuration byte that read back what was written: the
/// conversion rate and the extended-mode bit. The alert bit does not.
const CONFIG_MASK_2: u8 = 0xD0;

/// AL bit of the second configuration byte. With an active-low alert it
/// reads 0 while the alert is on.
const CONFIG_ALERT: u8 = 1 << 5;

/// Error originating from the TMP112 temperature sensor.
#[derive(Debug)]
pub enum Error {
    /// Indicates that the configuration register did not read back the value
    /// written to it, which usually means a different chip is answering at
    /// this address. An absent sensor shows up as [`Error::I2C`] instead.
    ConfigMismatch {
        /// The two bytes that were written.
        wrote: [u8; 2],
        /// The two bytes that were read back.
        read: [u8; 2],
    },

    /// Indicates that a temperature limit is outside -55 to +127.9375
    /// degrees Celsius.
    TemperatureOutOfRange(f64),

    /// Indicates that the low limit is above the high limit.
    LimitsInvalid,

    /// I2C bus error.
    I2C(rppal::i2c::Error),
}

impl fmt::Display for Error {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Error::ConfigMismatch { wrote, read } => write!(
                f,
                "configuration read back {read:02X?} after writing {wrote:02X?}"
            ),
            Error::TemperatureOutOfRange(celsius) => {
                write!(f, "temperature out of range: {celsius} C")
            }
            Error::LimitsInvalid => write!(f, "the low limit is above the high limit"),
            Error::I2C(error) => write!(f, "I2C error: {error}"),
        }
    }
}

impl std::error::Error for Error {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            Error::I2C(error) => Some(error),
            _ => None,
        }
    }
}

impl From<rppal::i2c::Error> for Error {
    fn from(err: rppal::i2c::Error) -> Self {
        Error::I2C(err)
    }
}

/// Result type encapsulating an error from the TMP112.
pub type Result<T> = std::result::Result<T, Error>;

/// The I2C address, which is set by what the ADD0 pin is wired to.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Address {
    /// ADD0 tied to ground: 0x48. The HITL test board is drawn this way.
    Gnd = 0x48,
    /// ADD0 tied to V+: 0x49.
    VPlus = 0x49,
    /// ADD0 tied to SDA: 0x4A.
    Sda = 0x4A,
    /// ADD0 tied to SCL: 0x4B.
    Scl = 0x4B,
}

/// The four registers, by the pointer value that selects them.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Register {
    /// The latest temperature result. Read only.
    Temperature = 0b00,
    /// The configuration register.
    Configuration = 0b01,
    /// The low temperature limit.
    TLow = 0b10,
    /// The high temperature limit.
    THigh = 0b11,
}

/// How often the sensor converts on its own.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ConversionRate {
    /// One conversion every four seconds.
    Hz0_25 = 0b00,
    /// One conversion per second.
    Hz1 = 0b01,
    /// Four conversions per second, the power-up default.
    Hz4 = 0b10,
    /// Eight conversions per second.
    Hz8 = 0b11,
}

/// Builds the two configuration bytes for `rate`.
fn config_bytes(rate: ConversionRate) -> [u8; 2] {
    [CONFIG_BYTE_1, (rate as u8) << 6]
}

/// Converts the two temperature register bytes, high byte first, to degrees
/// Celsius.
///
/// The lowest bit of the second byte says which format the bytes are in: 0
/// for the 12-bit format and 1 for the 13-bit extended format.
fn celsius_from_bytes(bytes: [u8; 2]) -> f64 {
    let raw = i16::from_be_bytes(bytes);
    // The result is left-justified, so shifting right drops the unused low
    // bits and keeps the sign.
    let steps = if bytes[1] & 1 == 0 {
        raw >> 4
    } else {
        raw >> 3
    };

    steps as f64 * DEGREES_PER_STEP
}

/// Converts degrees Celsius to the two bytes of a limit register in the
/// 12-bit format, or returns an error when the temperature is out of range.
fn bytes_from_celsius(celsius: f64) -> Result<[u8; 2]> {
    if !(TEMPERATURE_MIN..=TEMPERATURE_MAX).contains(&celsius) {
        return Err(Error::TemperatureOutOfRange(celsius));
    }

    let steps = (celsius / DEGREES_PER_STEP).round() as i16;
    Ok((steps << 4).to_be_bytes())
}

/// Controls a hardware TMP112 temperature sensor over I2C.
pub struct TMP112 {
    /// The I2C bus, already pointed at the sensor's address.
    i2c: I2c,
}

impl TMP112 {
    /// Constructs a new temperature sensor on I2C bus number `i2c_bus` at
    /// `address`.
    ///
    /// On the HITL test board with the CM5 adapter, the TMP112 is on bus 1
    /// (GPIO2 and GPIO3). ADD0 is drawn tied to ground, which is
    /// [`Address::Gnd`], but a note on the schematic says 0x49, so check with
    /// `i2cdetect -y 1` if the constructor fails.
    ///
    /// The constructor writes the configuration register and reads it back,
    /// which also confirms a TMP112 is answering. It leaves the sensor
    /// converting four times a second.
    pub fn new(i2c_bus: u8, address: Address) -> Result<Self> {
        let mut i2c = I2c::with_bus(i2c_bus)?;
        i2c.set_slave_address(address as u16)?;

        let mut driver = TMP112 { i2c };
        driver.set_rate(ConversionRate::Hz4)?;

        Ok(driver)
    }

    /// Reads the two bytes of `register`.
    fn read_register(&mut self, register: Register) -> Result<[u8; 2]> {
        let mut bytes = [0; 2];
        self.i2c.write_read(&[register as u8], &mut bytes)?;

        Ok(bytes)
    }

    /// Writes the two bytes of `register`.
    fn write_register(&mut self, register: Register, bytes: [u8; 2]) -> Result<()> {
        self.i2c.write(&[register as u8, bytes[0], bytes[1]])?;

        Ok(())
    }

    /// Sets how often the sensor converts, and puts the rest of the
    /// configuration back to this driver's defaults: continuous conversions,
    /// the 12-bit format, comparator mode, and an active-low alert.
    pub fn set_rate(&mut self, rate: ConversionRate) -> Result<()> {
        let wrote = config_bytes(rate);
        self.write_register(Register::Configuration, wrote)?;

        // Compare only the bits that read back what was written.
        let read = self.read_register(Register::Configuration)?;
        if read[0] & CONFIG_MASK_1 != wrote[0] & CONFIG_MASK_1
            || read[1] & CONFIG_MASK_2 != wrote[1] & CONFIG_MASK_2
        {
            return Err(Error::ConfigMismatch { wrote, read });
        }

        Ok(())
    }

    /// Returns the latest temperature in degrees Celsius.
    ///
    /// The sensor converts on its own, so this is the most recent result, up
    /// to one conversion period old. It reads 0 until the first conversion
    /// finishes, about 10 ms after power-up.
    pub fn read_celsius(&mut self) -> Result<f64> {
        let bytes = self.read_register(Register::Temperature)?;
        Ok(celsius_from_bytes(bytes))
    }

    /// Sets the alert limits in degrees Celsius. The alert turns on when the
    /// temperature reaches `high`, and off again when it falls below `low`.
    ///
    /// Limits are rounded to the nearest 0.0625 degrees, so [`TMP112::limits`]
    /// may return a slightly different value. The sensor compares the limits
    /// on every conversion, so [`TMP112::alert`] can lag a new limit by up to
    /// one conversion period.
    pub fn set_limits(&mut self, low: f64, high: f64) -> Result<()> {
        let low_bytes = bytes_from_celsius(low)?;
        let high_bytes = bytes_from_celsius(high)?;
        if low > high {
            return Err(Error::LimitsInvalid);
        }

        self.write_register(Register::TLow, low_bytes)?;
        self.write_register(Register::THigh, high_bytes)?;

        Ok(())
    }

    /// Returns the alert limits as (low, high) in degrees Celsius, read from
    /// the sensor.
    pub fn limits(&mut self) -> Result<(f64, f64)> {
        let low = celsius_from_bytes(self.read_register(Register::TLow)?);
        let high = celsius_from_bytes(self.read_register(Register::THigh)?);

        Ok((low, high))
    }

    /// Returns whether the alert is on, read from the sensor's status bit.
    pub fn alert(&mut self) -> Result<bool> {
        let config = self.read_register(Register::Configuration)?;
        Ok(config[1] & CONFIG_ALERT == 0)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn bytes_convert_to_celsius_like_the_datasheet_table() {
        assert_eq!(celsius_from_bytes([0x7F, 0xF0]), 127.9375);
        assert_eq!(celsius_from_bytes([0x64, 0x00]), 100.0);
        assert_eq!(celsius_from_bytes([0x19, 0x00]), 25.0);
        assert_eq!(celsius_from_bytes([0x00, 0x10]), 0.0625);
        assert_eq!(celsius_from_bytes([0x00, 0x00]), 0.0);
    }

    #[test]
    fn negative_temperatures_keep_their_sign() {
        assert_eq!(celsius_from_bytes([0xFF, 0xF0]), -0.0625);
        assert_eq!(celsius_from_bytes([0xE7, 0x00]), -25.0);
        assert_eq!(celsius_from_bytes([0xC9, 0x00]), -55.0);
    }

    #[test]
    fn extended_format_is_detected_from_the_low_bit() {
        // 150 C is 0x0960 in the 13-bit format, left-justified with the
        // format bit set.
        assert_eq!(celsius_from_bytes([0x4B, 0x01]), 150.0);
        assert_eq!(celsius_from_bytes([0x0C, 0x81]), 25.0);
    }

    #[test]
    fn celsius_converts_to_limit_bytes() {
        assert_eq!(bytes_from_celsius(75.0).unwrap(), [0x4B, 0x00]);
        assert_eq!(bytes_from_celsius(80.0).unwrap(), [0x50, 0x00]);
        assert_eq!(bytes_from_celsius(-25.0).unwrap(), [0xE7, 0x00]);
        assert_eq!(bytes_from_celsius(0.0).unwrap(), [0x00, 0x00]);
    }

    #[test]
    fn limits_round_trip() {
        for celsius in [-55.0, -0.0625, 25.0, 127.9375] {
            let bytes = bytes_from_celsius(celsius).unwrap();
            assert_eq!(celsius_from_bytes(bytes), celsius);
        }
    }

    #[test]
    fn out_of_range_limits_are_rejected() {
        assert!(matches!(
            bytes_from_celsius(128.0),
            Err(Error::TemperatureOutOfRange(_))
        ));
        assert!(matches!(
            bytes_from_celsius(-55.1),
            Err(Error::TemperatureOutOfRange(_))
        ));
        assert!(bytes_from_celsius(f64::NAN).is_err());
    }

    #[test]
    fn config_bytes_hold_the_rate_in_the_top_two_bits() {
        // The power-up value is 0x60A0; the alert bit is read only, so this
        // driver writes it as 0.
        assert_eq!(config_bytes(ConversionRate::Hz4), [0x60, 0x80]);
        assert_eq!(config_bytes(ConversionRate::Hz0_25), [0x60, 0x00]);
        assert_eq!(config_bytes(ConversionRate::Hz8), [0x60, 0xC0]);
    }

    #[test]
    fn addresses_match_the_add0_table() {
        assert_eq!(Address::Gnd as u16, 0x48);
        assert_eq!(Address::VPlus as u16, 0x49);
        assert_eq!(Address::Sda as u16, 0x4A);
        assert_eq!(Address::Scl as u16, 0x4B);
    }
}
