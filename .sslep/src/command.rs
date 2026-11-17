
use std::sync::{LazyLock, Mutex};

use common::comm::{
    gpio::{GpioPin, PinMode, PinValue},
    sslep::SslepControlMessage,
};

use crate::pins::{
    GPIO_CONTROLLER,
    ESC_A_ENABLE,
    ESC_B_ENABLE,
    MAINS_ENABLE,
    PRECHARGE_ENABLE,
    PWM0,
    PWM1,
};

#[derive(Debug)]
pub enum CommandError {
    InvalidChannel(u8),
    InvalidDutyCycle(f32),
    EnableNotVerified(&'static str),
    PwmNotImplemented,
    GpioLockPoisoned,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CommandOutcome {
    Continue,
    AbortRequested,
}

// Holds the GPIO pins controlled by SSLEP.
struct OutputPins {
    precharge: Box<dyn GpioPin>,
    mains: Box<dyn GpioPin>,
    esc_a: Box<dyn GpioPin>,
    esc_b: Box<dyn GpioPin>,
    pwm0: Box<dyn GpioPin>,
    pwm1: Box<dyn GpioPin>,
}

// Create a GPIO output initialized LOW.
// Assumes LOW is inactive; this must be verified
// against the SSLEP schematic.
fn output_low(pin_num: u8) -> Box<dyn GpioPin> {
    let mut pin = GPIO_CONTROLLER.get_pin(pin_num);

    pin.mode(PinMode::Output);
    pin.digital_write(PinValue::Low);

    Box::new(pin)
}

impl OutputPins {
    fn new() -> Self {
        Self {
            precharge: output_low(PRECHARGE_ENABLE.pin_num),
            mains: output_low(MAINS_ENABLE.pin_num),
            esc_a: output_low(ESC_A_ENABLE.pin_num),
            esc_b: output_low(ESC_B_ENABLE.pin_num),
            pwm0: output_low(PWM0.pin_num),
            pwm1: output_low(PWM1.pin_num),
        }
    }

    // Request inactive outputs.
    // Not a verified power-down sequence.
    fn disable_all(&mut self) {
        self.pwm0.digital_write(PinValue::Low);
        self.pwm1.digital_write(PinValue::Low);

        self.esc_a.digital_write(PinValue::Low);
        self.esc_b.digital_write(PinValue::Low);

        self.mains.digital_write(PinValue::Low);
        self.precharge.digital_write(PinValue::Low);
    }
}

// Share GPIO outputs across commands.
// Mutex prevents simultaneous mutable access.
static OUTPUT_PINS: LazyLock<Mutex<OutputPins>> =
    LazyLock::new(|| Mutex::new(OutputPins::new()));

fn with_pins<T>(
    f: impl FnOnce(&mut OutputPins) -> T,
) -> Result<T, CommandError> {
    let mut pins = OUTPUT_PINS
        .lock()
        .map_err(|_| CommandError::GpioLockPoisoned)?;

    Ok(f(&mut pins))
}

// Initialize the GPIO outputs.
pub fn init_gpio() -> Result<(), CommandError> {
    safe_shutdown()
}

// Request the defined inactive output states.
pub fn safe_shutdown() -> Result<(), CommandError> {
    with_pins(|pins| pins.disable_all())
}

// Control precharge (GPIO23).
pub fn set_precharge(
    enabled: bool,
) -> Result<(), CommandError> {
    if enabled {
        return Err(CommandError::EnableNotVerified(
            "precharge",
        ));
    }

    with_pins(|pins| {
        pins.precharge.digital_write(PinValue::Low);
    })
}

// Control mains (GPIO24).
pub fn set_mains(
    enabled: bool,
) -> Result<(), CommandError> {
    if enabled {
        return Err(CommandError::EnableNotVerified(
            "mains",
        ));
    }

    with_pins(|pins| {
        pins.mains.digital_write(PinValue::Low);
    })
}

// Control ESC enable pins.
// Channel 0 = ESC A (GPIO25)
// Channel 1 = ESC B (GPIO27)
pub fn set_esc_enable(
    channel: u8,
    enabled: bool,
) -> Result<(), CommandError> {
    if channel > 1 {
        return Err(CommandError::InvalidChannel(channel));
    }

    if enabled {
        return Err(CommandError::EnableNotVerified(
            "ESC enable",
        ));
    }

    with_pins(|pins| {
        match channel {
            0 => pins.esc_a.digital_write(PinValue::Low),
            1 => pins.esc_b.digital_write(PinValue::Low),
            _ => unreachable!(),
        }
    })
}

// Control PWM outputs.
// Channel 0 = PWM0 (GPIO12)
// Channel 1 = PWM1 (GPIO13)
//
// Active PWM requires the actual ESC protocol.
pub fn set_pwm(
    channel: u8,
    duty_cycle: f32,
) -> Result<(), CommandError> {
    if channel > 1 {
        return Err(CommandError::InvalidChannel(channel));
    }

    if !duty_cycle.is_finite()
        || !(0.0..=1.0).contains(&duty_cycle)
    {
        return Err(CommandError::InvalidDutyCycle(
            duty_cycle,
        ));
    }

    if duty_cycle != 0.0 {
        return Err(CommandError::PwmNotImplemented);
    }

    // GPIO LOW is not necessarily an ESC neutral signal.
    with_pins(|pins| {
        match channel {
            0 => pins.pwm0.digital_write(PinValue::Low),
            1 => pins.pwm1.digital_write(PinValue::Low),
            _ => unreachable!(),
        }
    })
}

// Execute a command received from the Flight Computer.
pub fn execute(
    command: SslepControlMessage,
) -> Result<CommandOutcome, CommandError> {
    match command {
        SslepControlMessage::SetPrecharge { enabled } => {
            set_precharge(enabled)?;
        }

        SslepControlMessage::SetMains { enabled } => {
            set_mains(enabled)?;
        }

        SslepControlMessage::SetEscEnable {
            channel,
            enabled,
        } => {
            set_esc_enable(channel, enabled)?;
        }

        SslepControlMessage::SetPwm {
            channel,
            duty_cycle,
        } => {
            set_pwm(channel, duty_cycle)?;
        }

        SslepControlMessage::Abort => {
            safe_shutdown()?;
            return Ok(CommandOutcome::AbortRequested);
        }
    }

    Ok(CommandOutcome::Continue)
}
