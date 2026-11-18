//! Linux SPI driver for the ST ASM330LHBG1 accelerometer and gyroscope.
//!
//! The driver uses the chip select assigned to the spidev device. Its output
//! stays in sensor coordinates so the caller can apply the mounting
//! orientation.

use std::{
    fmt, io,
    path::Path,
    thread,
    time::{Duration, Instant},
};

use spidev::{SpiModeFlags, Spidev, SpidevOptions, SpidevTransfer};

// ST datasheet DS14080, register descriptions and mechanical characteristics.
const WHO_AM_I: u8 = 0x0F;
const DEVICE_ID: u8 = 0x6B;
const CTRL1_XL: u8 = 0x10;
const CTRL2_G: u8 = 0x11;
const CTRL3_C: u8 = 0x12;
const CTRL4_C: u8 = 0x13;
const CTRL9_XL: u8 = 0x18;
const STATUS_REG: u8 = 0x1E;
const OUTX_L_G: u8 = 0x22;

const READ: u8 = 1 << 7;
const SW_RESET: u8 = 1;
const BLOCK_DATA_UPDATE: u8 = 1 << 6;
const AUTO_INCREMENT: u8 = 1 << 2;
const I2C_DISABLE: u8 = 1 << 2;
const I3C_DISABLE: u8 = 1 << 1;
const ODR_208_HZ: u8 = 0b0101 << 4;
const ACCEL_16_G: u8 = 0b01 << 2;
const GYRO_2000_DPS: u8 = 0b11 << 2;
const ACCEL_READY: u8 = 1;
const GYRO_READY: u8 = 1 << 1;
const TEMP_READY: u8 = 1 << 2;

const ACCEL_MPS2_PER_COUNT: f64 = 0.488e-3 * 9.80665;
const GYRO_DPS_PER_COUNT: f64 = 70e-3;
const RESET_TIMEOUT: Duration = Duration::from_millis(100);
const RESET_POLL_INTERVAL: Duration = Duration::from_millis(1);
const TURN_ON_TIME: Duration = Duration::from_millis(35);

/// A six-axis reading in the sensor's X, Y, Z order.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Sample {
    pub acceleration_mps2: [f64; 3],
    pub angular_rate_dps: [f64; 3],
}

#[derive(Debug)]
pub enum Error {
    Spi(io::Error),
    UnexpectedDeviceId(u8),
    ResetTimeout,
    ConfigurationMismatch {
        register: u8,
        expected: u8,
        actual: u8,
    },
    InvalidStatus(u8),
}

impl fmt::Display for Error {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Spi(err) => write!(f, "SPI I/O failed: {err}"),
            Self::UnexpectedDeviceId(id) => {
                write!(f, "expected WHO_AM_I {DEVICE_ID:#04x}, got {id:#04x}")
            }
            Self::ResetTimeout => write!(
                f,
                "software reset did not complete within {RESET_TIMEOUT:?}"
            ),
            Self::ConfigurationMismatch {
                register,
                expected,
                actual,
            } => write!(
                f,
                "register {register:#04x}: wrote {expected:#04x}, read {actual:#04x}"
            ),
            Self::InvalidStatus(status) => write!(f, "invalid sensor status {status:#04x}"),
        }
    }
}

impl std::error::Error for Error {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            Self::Spi(err) => Some(err),
            _ => None,
        }
    }
}

impl From<io::Error> for Error {
    fn from(err: io::Error) -> Self {
        Self::Spi(err)
    }
}

pub struct Asm330lhbg1 {
    spi: Spidev,
}

impl Asm330lhbg1 {
    /// Open and configure the IMU at 208 Hz, ±16 g, and ±2000 deg/s.
    pub fn open(path: impl AsRef<Path>) -> Result<Self, Error> {
        let mut spi = Spidev::open(path)?;
        spi.configure(
            &SpidevOptions::new()
                .bits_per_word(8)
                .max_speed_hz(1_000_000)
                .lsb_first(false)
                .mode(SpiModeFlags::SPI_MODE_3)
                .build(),
        )?;
        initialize(&mut spi)?;
        thread::sleep(TURN_ON_TIME);
        Ok(Self { spi })
    }

    /// Read the latest six-axis sample when both sensors have new data.
    pub fn try_read_sample(&mut self) -> Result<Option<Sample>, Error> {
        read_sample(&mut self.spi)
    }
}

// Each exchange keeps chip select asserted across the command and data bytes.
trait SpiExchange {
    fn exchange(&mut self, tx: &[u8], rx: &mut [u8]) -> io::Result<()>;
}

impl SpiExchange for Spidev {
    fn exchange(&mut self, tx: &[u8], rx: &mut [u8]) -> io::Result<()> {
        self.transfer(&mut SpidevTransfer::read_write(tx, rx))
    }
}

fn read<const N: usize>(spi: &mut impl SpiExchange, register: u8) -> Result<[u8; N], Error> {
    let mut tx = [0; N];
    let mut rx = [0; N];
    tx[0] = register | READ;
    spi.exchange(&tx, &mut rx)?;
    Ok(rx)
}

fn read_register(spi: &mut impl SpiExchange, register: u8) -> Result<u8, Error> {
    Ok(read::<2>(spi, register)?[1])
}

fn write_register(spi: &mut impl SpiExchange, register: u8, value: u8) -> Result<(), Error> {
    spi.exchange(&[register, value], &mut [0; 2])?;
    Ok(())
}

fn initialize(spi: &mut impl SpiExchange) -> Result<(), Error> {
    let id = read_register(spi, WHO_AM_I)?;
    if id != DEVICE_ID {
        return Err(Error::UnexpectedDeviceId(id));
    }

    write_register(spi, CTRL3_C, SW_RESET)?;
    let reset_started = Instant::now();
    while read_register(spi, CTRL3_C)? & SW_RESET != 0 {
        if reset_started.elapsed() >= RESET_TIMEOUT {
            return Err(Error::ResetTimeout);
        }
        thread::sleep(RESET_POLL_INTERVAL);
    }

    let ctrl9_xl = read_register(spi, CTRL9_XL)? | I3C_DISABLE;
    let config = [
        (CTRL9_XL, ctrl9_xl),
        (CTRL4_C, I2C_DISABLE),
        (CTRL3_C, BLOCK_DATA_UPDATE | AUTO_INCREMENT),
        (CTRL1_XL, ODR_208_HZ | ACCEL_16_G),
        (CTRL2_G, ODR_208_HZ | GYRO_2000_DPS),
    ];
    for (register, expected) in config {
        write_register(spi, register, expected)?;
        let actual = read_register(spi, register)?;
        if actual != expected {
            return Err(Error::ConfigurationMismatch {
                register,
                expected,
                actual,
            });
        }
    }
    Ok(())
}

fn read_sample(spi: &mut impl SpiExchange) -> Result<Option<Sample>, Error> {
    let status = read_register(spi, STATUS_REG)?;
    let ready = ACCEL_READY | GYRO_READY;
    // Bit 3 reports a boot CRC failure. Bits 4–7 must read zero.
    if status & !(ready | TEMP_READY) != 0 {
        return Err(Error::InvalidStatus(status));
    }
    if status & ready != ready {
        return Ok(None);
    }

    // Ignore the byte received during the address phase. Gyro precedes accel.
    let data = read::<13>(spi, OUTX_L_G)?;
    let axis = |index| f64::from(i16::from_le_bytes([data[index], data[index + 1]]));
    Ok(Some(Sample {
        angular_rate_dps: [axis(1), axis(3), axis(5)].map(|v| v * GYRO_DPS_PER_COUNT),
        acceleration_mps2: [axis(7), axis(9), axis(11)].map(|v| v * ACCEL_MPS2_PER_COUNT),
    }))
}

#[cfg(test)]
mod tests {
    use std::collections::VecDeque;

    use super::*;

    struct ScriptedSpi(VecDeque<(&'static [u8], &'static [u8])>);

    impl ScriptedSpi {
        fn new<const N: usize>(steps: [(&'static [u8], &'static [u8]); N]) -> Self {
            Self(steps.into())
        }
    }

    impl SpiExchange for ScriptedSpi {
        fn exchange(&mut self, tx: &[u8], rx: &mut [u8]) -> io::Result<()> {
            let (expected, response) = self.0.pop_front().expect("unexpected SPI exchange");
            assert_eq!(tx, expected);
            rx.copy_from_slice(response);
            Ok(())
        }
    }

    #[test]
    fn verifies_identity_before_writing() {
        let mut spi = ScriptedSpi::new([(&[0x8F, 0], &[0, 0xFF])]);
        assert!(matches!(
            initialize(&mut spi),
            Err(Error::UnexpectedDeviceId(0xFF))
        ));
        assert!(spi.0.is_empty());
    }

    #[test]
    fn resets_and_verifies_each_configuration_write() {
        let mut spi = ScriptedSpi::new([
            (&[0x8F, 0], &[0, 0x6B]),
            (&[0x12, 0x01], &[0, 0]),
            (&[0x92, 0], &[0, 0x01]),
            (&[0x92, 0], &[0, 0x04]),
            (&[0x98, 0], &[0, 0xE0]),
            (&[0x18, 0xE2], &[0, 0]),
            (&[0x98, 0], &[0, 0xE2]),
            (&[0x13, 0x04], &[0, 0]),
            (&[0x93, 0], &[0, 0x04]),
            (&[0x12, 0x44], &[0, 0]),
            (&[0x92, 0], &[0, 0x44]),
            (&[0x10, 0x54], &[0, 0]),
            (&[0x90, 0], &[0, 0x54]),
            (&[0x11, 0x5C], &[0, 0]),
            (&[0x91, 0], &[0, 0x5C]),
        ]);
        initialize(&mut spi).unwrap();
        assert!(spi.0.is_empty());
    }

    #[test]
    fn rejects_configuration_readback_mismatch() {
        let mut spi = ScriptedSpi::new([
            (&[0x8F, 0], &[0, 0x6B]),
            (&[0x12, 0x01], &[0, 0]),
            (&[0x92, 0], &[0, 0x04]),
            (&[0x98, 0], &[0, 0xE0]),
            (&[0x18, 0xE2], &[0, 0]),
            (&[0x98, 0], &[0, 0xE0]),
        ]);
        assert!(matches!(
            initialize(&mut spi),
            Err(Error::ConfigurationMismatch {
                register: 0x18,
                expected: 0xE2,
                actual: 0xE0
            })
        ));
        assert!(spi.0.is_empty());
    }

    #[test]
    fn times_out_if_reset_stays_asserted() {
        struct StuckReset;

        impl SpiExchange for StuckReset {
            fn exchange(&mut self, tx: &[u8], rx: &mut [u8]) -> io::Result<()> {
                rx[1] = match tx {
                    [0x8F, 0] => 0x6B,
                    [0x12, 0x01] => 0,
                    [0x92, 0] => 0x01,
                    _ => panic!("unexpected SPI exchange {tx:?}"),
                };
                Ok(())
            }
        }

        assert!(matches!(
            initialize(&mut StuckReset),
            Err(Error::ResetTimeout)
        ));
    }

    #[test]
    fn waits_for_both_sensors_and_rejects_invalid_status() {
        let mut spi = ScriptedSpi::new([
            (&[0x9E, 0], &[0, 0x00]),
            (&[0x9E, 0], &[0, 0x01]),
            (&[0x9E, 0], &[0, 0x02]),
            (&[0x9E, 0], &[0, 0x0B]),
            (&[0x9E, 0], &[0, 0xFF]),
        ]);
        for _ in 0..3 {
            assert_eq!(read_sample(&mut spi).unwrap(), None);
        }
        for status in [0x0B, 0xFF] {
            assert!(
                matches!(read_sample(&mut spi), Err(Error::InvalidStatus(value)) if value == status)
            );
        }
        assert!(spi.0.is_empty());
    }

    #[test]
    fn reads_and_scales_signed_axes_in_one_burst() {
        let mut spi = ScriptedSpi::new([
            (&[0x9E, 0], &[0, 0x07]),
            (
                &[0xA2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                &[
                    0xFF, 0, 0x80, 0, 1, 0xFF, 0x7F, 0xE8, 0x03, 0x18, 0xFC, 0, 0,
                ],
            ),
        ]);
        let sample = read_sample(&mut spi).unwrap().unwrap();
        for (actual, expected) in sample
            .angular_rate_dps
            .into_iter()
            .zip([-2293.76, 17.92, 2293.69])
        {
            assert!((actual - expected).abs() < 1e-9);
        }
        for (actual, expected) in sample
            .acceleration_mps2
            .into_iter()
            .zip([4.7856452, -4.7856452, 0.0])
        {
            assert!((actual - expected).abs() < 1e-9);
        }
        assert!(spi.0.is_empty());
    }
}
