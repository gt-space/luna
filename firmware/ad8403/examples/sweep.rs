//! Standalone example: steps one AD8403 channel through its range so the wiper
//! can be checked with a multimeter (W to B resistance).
//!
//! On the HITL test board with the CM5 adapter, the AD8403 is on SPI5 with the
//! kernel driving chip select on GPIO12 (`dtoverlay=spi5-1cs`).
//!
//! SHDN and RS must be held high for the sweep to show anything. On the
//! current HITL board revision they are not connected, so a reading that never
//! changes points to those pins, not the driver.
//!
//! Run on the CM5: `cargo run -p ad8403 --example sweep -- [channel]`

use std::{env, thread, time::Duration};

use ad8403::{Result, AD8403, CHANNELS, MIDSCALE};

const SPI_PATH: &str = "/dev/spidev5.0";

/// Time to hold each code so a meter can settle.
const STEP_DELAY: Duration = Duration::from_millis(500);

fn main() -> Result<()> {
    let channel = env::args()
        .nth(1)
        .map(|arg| arg.parse::<u8>().expect("channel must be 0-3"))
        .unwrap_or(0);
    assert!((channel as usize) < CHANNELS, "channel must be 0-3");

    // spi5-1cs gives the kernel GPIO12 as chip select, so no GPIO is passed.
    let mut pot = AD8403::new_with_gpio_pin(SPI_PATH, None)?;
    println!("AD8403 on {SPI_PATH}: all channels set to {MIDSCALE:#04X}");

    for code in (0..=u8::MAX).step_by(16).chain([u8::MAX]) {
        pot.set_wiper(channel, code)?;
        println!("channel {channel} -> code {code:#04X} ({code})");
        thread::sleep(STEP_DELAY);
    }

    pot.set_wiper(channel, MIDSCALE)?;
    println!("channel {channel} back to midscale");

    Ok(())
}
