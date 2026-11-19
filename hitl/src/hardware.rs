//! The test board's chips, behind one interface.
//!
//! [`Board`] holds the real drivers. [`Mock`] keeps the same state in memory
//! so the service can run and be tested without a board.

use std::{
    error::Error,
    sync::{Arc, Mutex, OnceLock},
};

use ad5676r::{Reference as DacReference, Span, AD5676R};
use ad8403::AD8403;
use ads1261::{Input, Reference as AdcReference, ADS1261};
use common::comm::gpio::{GpioPin, PinMode, RpiGpioController};
use jeflog::{pass, warn};
use tmp112::{Address, TMP112};

/// Result type for hardware calls. Any driver's error fits.
pub type Result<T> = std::result::Result<T, Box<dyn Error>>;

/// Number of AD5676R DAC channels.
pub const DAC_CHANNELS: usize = ad5676r::CHANNELS;

/// Number of AD8403 digipot channels.
pub const DIGIPOT_CHANNELS: usize = ad8403::CHANNELS;

/// Digipot code for the safe state: midscale.
pub const DIGIPOT_SAFE: u8 = ad8403::MIDSCALE;

/// Everything the service needs from the board.
pub trait Hardware: Send {
    /// Drives one DAC channel to a voltage.
    fn set_dac_volts(&mut self, channel: u8, volts: f64) -> Result<()>;

    /// Sets one digipot channel's wiper code.
    fn set_digipot(&mut self, channel: u8, code: u8) -> Result<()>;

    /// Reads the voltage between two ADC inputs. `None` is AINCOM.
    fn read_adc_volts(&mut self, positive: u8, negative: Option<u8>) -> Result<f64>;

    /// Reads the temperature sensor, in degrees Celsius.
    fn read_temperature(&mut self) -> Result<f64>;

    /// Puts every output in its safe state: DAC at 0 V, digipot at midscale.
    fn safe(&mut self) -> Result<()>;
}

/// How the real chips are wired to the CM5 on the HITL adapter board.
#[derive(Clone, Copy, Debug)]
pub struct BoardConfig {
    /// Reference for the AD5676R DAC.
    pub dac_reference: DacReference,

    /// I2C address of the TMP112. The schematic draws ADD0 to ground (0x48),
    /// but a note on it says 0x49, so this is configurable until bring-up.
    pub tmp_address: Address,
}

impl Default for BoardConfig {
    fn default() -> Self {
        Self {
            dac_reference: DacReference::Internal,
            tmp_address: Address::Gnd,
        }
    }
}

/// AD5676R: SPI0, kernel chip select on GPIO8 (`dtoverlay=spi0-1cs`).
const DAC_SPI: &str = "/dev/spidev0.0";

/// BCM pin wired to the AD5676R's LDAC.
const DAC_LDAC_PIN: u8 = 7;

/// AD8403: SPI5, kernel chip select on GPIO12 (`dtoverlay=spi5-1cs`).
const DIGIPOT_SPI: &str = "/dev/spidev5.0";

/// ADS1261: SPI1, kernel chip select on GPIO18 (`dtoverlay=spi1-1cs`).
const ADC_SPI: &str = "/dev/spidev1.0";

/// TMP112: I2C bus 1.
const TMP_I2C_BUS: u8 = 1;

/// The Raspberry Pi GPIO controller, opened once. Pins borrow it, so it has to
/// live for the whole program.
fn gpio_controller() -> std::result::Result<&'static RpiGpioController, String> {
    static CONTROLLER: OnceLock<std::result::Result<RpiGpioController, String>> = OnceLock::new();
    CONTROLLER
        .get_or_init(|| RpiGpioController::open_controller().map_err(|e| e.to_string()))
        .as_ref()
        .map_err(Clone::clone)
}

/// A chip that either started or failed to, with the reason.
type Chip<T> = std::result::Result<T, String>;

/// The real test board.
///
/// Each chip is opened on its own. A chip that fails to start is logged and
/// left out: commands for it fail with the reason, and the others keep
/// working. During bring-up this means one bad chip does not stop the rest.
pub struct Board {
    dac: Chip<AD5676R>,
    digipot: Chip<AD8403>,
    adc: Chip<ADS1261>,
    thermometer: Chip<TMP112>,
}

impl Board {
    /// Opens every chip. The DAC starts with every channel at 0 V; the
    /// digipot is then set to midscale so both start in the safe state.
    pub fn open(config: BoardConfig) -> Self {
        let dac = open_dac(config.dac_reference);
        let digipot = AD8403::new_with_gpio_pin(DIGIPOT_SPI, None)
            .and_then(|mut digipot| digipot.set_all(DIGIPOT_SAFE).map(|()| digipot))
            .map_err(|e| e.to_string());
        let adc = ADS1261::new_with_gpio_pin(ADC_SPI, None, AdcReference::Internal)
            .map_err(|e| e.to_string());
        let thermometer = TMP112::new(TMP_I2C_BUS, config.tmp_address).map_err(|e| e.to_string());

        report("AD5676R DAC", &dac);
        report("AD8403 digipot", &digipot);
        report("ADS1261 ADC", &adc);
        report("TMP112 temperature sensor", &thermometer);

        Self {
            dac,
            digipot,
            adc,
            thermometer,
        }
    }
}

fn open_dac(reference: DacReference) -> Chip<AD5676R> {
    let mut ldac = gpio_controller()?.get_pin(DAC_LDAC_PIN);
    ldac.mode(PinMode::Output);
    AD5676R::new_with_gpio_pins(DAC_SPI, None, Some(Box::new(ldac)), Span::Vref, reference)
        .map_err(|e| e.to_string())
}

fn report<T>(name: &str, chip: &Chip<T>) {
    match chip {
        Ok(_) => pass!("{name} ready."),
        Err(e) => warn!("{name} not available: {e}"),
    }
}

/// Borrows a chip, or fails with why it is not there.
fn chip<'a, T>(chip: &'a mut Chip<T>, name: &str) -> Result<&'a mut T> {
    chip.as_mut()
        .map_err(|e| format!("{name} not available: {e}").into())
}

/// Maps 0 to 9 to AIN0 to AIN9.
pub fn adc_input(n: u8) -> Option<Input> {
    Some(match n {
        0 => Input::Ain0,
        1 => Input::Ain1,
        2 => Input::Ain2,
        3 => Input::Ain3,
        4 => Input::Ain4,
        5 => Input::Ain5,
        6 => Input::Ain6,
        7 => Input::Ain7,
        8 => Input::Ain8,
        9 => Input::Ain9,
        _ => return None,
    })
}

/// Maps an optional input number to an ADC input, `None` being AINCOM.
fn adc_inputs(positive: u8, negative: Option<u8>) -> Result<(Input, Input)> {
    let input = |n| adc_input(n).ok_or_else(|| format!("invalid ADC input: {n}"));
    let negative = match negative {
        Some(n) => input(n)?,
        None => Input::Aincom,
    };
    Ok((input(positive)?, negative))
}

impl Hardware for Board {
    fn set_dac_volts(&mut self, channel: u8, volts: f64) -> Result<()> {
        Ok(chip(&mut self.dac, "AD5676R DAC")?.set_volts(channel, volts)?)
    }

    fn set_digipot(&mut self, channel: u8, code: u8) -> Result<()> {
        Ok(chip(&mut self.digipot, "AD8403 digipot")?.set_wiper(channel, code)?)
    }

    fn read_adc_volts(&mut self, positive: u8, negative: Option<u8>) -> Result<f64> {
        let (positive, negative) = adc_inputs(positive, negative)?;
        Ok(chip(&mut self.adc, "ADS1261 ADC")?.read_volts(positive, negative)?)
    }

    fn read_temperature(&mut self) -> Result<f64> {
        Ok(chip(&mut self.thermometer, "TMP112 temperature sensor")?.read_celsius()?)
    }

    fn safe(&mut self) -> Result<()> {
        // Try both, even if one fails, and report the first failure.
        let dac = chip(&mut self.dac, "AD5676R DAC").and_then(|dac| Ok(dac.set_all(0)?));
        let digipot = chip(&mut self.digipot, "AD8403 digipot")
            .and_then(|digipot| Ok(digipot.set_all(DIGIPOT_SAFE)?));
        dac.and(digipot)
    }
}

/// State a [`Mock`] keeps, shared so a test can look at it.
#[derive(Clone, Debug, PartialEq)]
pub struct MockState {
    /// Last voltage set on each DAC channel.
    pub dac_volts: [f64; DAC_CHANNELS],

    /// Last code set on each digipot channel.
    pub digipot_codes: [u8; DIGIPOT_CHANNELS],

    /// How many times the board was put in its safe state.
    pub safe_count: u32,
}

impl Default for MockState {
    fn default() -> Self {
        Self {
            dac_volts: [0.0; DAC_CHANNELS],
            digipot_codes: [DIGIPOT_SAFE; DIGIPOT_CHANNELS],
            safe_count: 0,
        }
    }
}

/// A board in memory, for tests and for running the service off the board.
///
/// It checks channels and voltages like the drivers do. ADC reads return
/// 0 V and the temperature is always 25 °C.
#[derive(Clone, Debug, Default)]
pub struct Mock {
    /// The mock's state. Clones of the mock share it.
    pub state: Arc<Mutex<MockState>>,
}

impl Mock {
    /// DAC full scale for the mock: the internal reference.
    pub const DAC_FULL_SCALE: f64 = ad5676r::INTERNAL_VREF;

    /// Temperature the mock always reads.
    pub const TEMPERATURE: f64 = 25.0;

    fn state(&self) -> std::sync::MutexGuard<'_, MockState> {
        // A test that panicked while holding the lock still left valid state.
        self.state.lock().unwrap_or_else(|e| e.into_inner())
    }
}

impl Hardware for Mock {
    fn set_dac_volts(&mut self, channel: u8, volts: f64) -> Result<()> {
        if channel as usize >= DAC_CHANNELS {
            return Err(format!("invalid DAC channel: {channel}").into());
        }
        if !(0.0..=Self::DAC_FULL_SCALE).contains(&volts) {
            return Err(format!("voltage out of range: {volts} V").into());
        }
        self.state().dac_volts[channel as usize] = volts;
        Ok(())
    }

    fn set_digipot(&mut self, channel: u8, code: u8) -> Result<()> {
        if channel as usize >= DIGIPOT_CHANNELS {
            return Err(format!("invalid digipot channel: {channel}").into());
        }
        self.state().digipot_codes[channel as usize] = code;
        Ok(())
    }

    fn read_adc_volts(&mut self, positive: u8, negative: Option<u8>) -> Result<f64> {
        adc_inputs(positive, negative)?;
        Ok(0.0)
    }

    fn read_temperature(&mut self) -> Result<f64> {
        Ok(Self::TEMPERATURE)
    }

    fn safe(&mut self) -> Result<()> {
        let mut state = self.state();
        state.dac_volts = [0.0; DAC_CHANNELS];
        state.digipot_codes = [DIGIPOT_SAFE; DIGIPOT_CHANNELS];
        state.safe_count += 1;
        Ok(())
    }
}
