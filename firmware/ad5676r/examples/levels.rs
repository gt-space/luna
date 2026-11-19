//! Standalone example: steps one AD5676R channel through a few voltages so the
//! output can be checked with a multimeter, then stages two channels and
//! latches them together.
//!
//! On the HITL test board with the CM5 adapter, the AD5676R is on SPI0 with
//! the kernel driving chip select on GPIO8 (`dtoverlay=spi0-1cs`, which also
//! frees GPIO7 for LDAC). GAIN is tied to GND, so the span is 0 V to the
//! reference.
//!
//! RESET must be held high. On a board without a pull-up on RESET, the pin can
//! float: a falling edge clears every output to 0 V (RSTSEL is tied to GND),
//! the outputs cannot be updated while it is low, and the chip does not
//! initialize if it is low at power-up. An output that sits at 0 V or drops to
//! 0 V while the driver reports success points to that pin, not the driver.
//!
//! Pass a voltage as the second argument to use an external reference wired
//! to VREF instead of the internal 2.5 V one. The levels then scale with it.
//!
//! Run on the CM5:
//! `cargo run -p ad5676r --example levels -- [channel] [external_vref]`

use std::{env, sync::OnceLock, thread, time::Duration};

use ad5676r::{Reference, Result, Span, AD5676R, CHANNELS};
use common::comm::gpio::{GpioPin, PinMode, RpiGpioController};

const SPI_PATH: &str = "/dev/spidev0.0";

/// BCM pin wired to the AD5676R's active-low LDAC input on the CM5 adapter.
const LDAC_PIN: u8 = 7;

/// Time to hold each level so a meter can settle.
const STEP_DELAY: Duration = Duration::from_secs(1);

/// The Raspberry Pi GPIO controller, opened once.
fn gpio_controller() -> &'static RpiGpioController {
    static CONTROLLER: OnceLock<RpiGpioController> = OnceLock::new();
    CONTROLLER.get_or_init(|| {
        RpiGpioController::open_controller().expect("Failed to open Raspberry Pi GPIO controller")
    })
}

fn main() -> Result<()> {
    let channel = env::args()
        .nth(1)
        .map(|arg| arg.parse::<u8>().expect("channel must be 0-7"))
        .unwrap_or(0);
    assert!((channel as usize) < CHANNELS, "channel must be 0-7");

    let reference = match env::args().nth(2) {
        Some(arg) => Reference::External(arg.parse().expect("external_vref must be a voltage")),
        None => Reference::Internal,
    };
    let vref = reference.volts();

    let mut ldac = gpio_controller().get_pin(LDAC_PIN);
    ldac.mode(PinMode::Output);

    // spi0-1cs gives the kernel GPIO8 as chip select, so no CS pin is passed.
    let mut dac =
        AD5676R::new_with_gpio_pins(SPI_PATH, None, Some(Box::new(ldac)), Span::Vref, reference)?;
    println!("AD5676R on {SPI_PATH}: {vref} V reference, all channels at 0 V");

    // Six levels from 0 V to full scale.
    for step in 0..=5 {
        // Scale by the fraction, so the last step is exactly the reference.
        let volts = vref * (step as f64 / 5.0);
        dac.set_volts(channel, volts)?;
        println!(
            "channel {channel} -> {volts:.3} V (code {})",
            dac.output(channel)?
        );
        thread::sleep(STEP_DELAY);
    }

    // Stage the same level on two channels, then apply both with one command.
    let other = (channel + 1) % CHANNELS as u8;
    let half = vref / 2.0;
    let code = dac.volts_to_code(half)?;
    dac.stage(channel, code)?;
    dac.stage(other, code)?;
    println!("staged {half:.3} V on channels {channel} and {other}; outputs unchanged");
    thread::sleep(STEP_DELAY);

    dac.latch()?;
    println!("latched: channels {channel} and {other} now at {half:.3} V");
    thread::sleep(STEP_DELAY);

    dac.set_all(0)?;
    println!("all channels back to 0 V");

    Ok(())
}
