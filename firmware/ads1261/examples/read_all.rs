//! Standalone example: reads every wired ADS1261 input on the HITL adapter
//! board once a second and prints the voltages.
//!
//! On the HITL test board with the CM5 adapter, the ADS1261 is on SPI1 with
//! the kernel driving chip select on GPIO18 (`dtoverlay=spi1-1cs`). AINCOM is
//! tied to ground, so every input is read against it.
//!
//! The 3V3 and 5V0 rails reach the ADC through divide-by-two resistor pairs,
//! so those two readings are doubled. With the internal 2.5 V reference the
//! 5V0 reading sits at full scale, and it clips if the rail is above 5.0 V.
//!
//! Run on the CM5: `cargo run -p ads1261 --example read_all`. It runs until
//! Ctrl-C.

use std::{thread, time::Duration};

use ads1261::{Input, Reference, Result, ADS1261};

const SPI_PATH: &str = "/dev/spidev1.0";

/// Each wired input: its name, its pin, and the factor that undoes its
/// resistor divider.
const INPUTS: [(&str, Input, f64); 7] = [
    ("Temp", Input::Ain0, 1.0),
    ("DET_VSNS", Input::Ain1, 1.0),
    ("DET_ISNS", Input::Ain2, 1.0),
    ("RTD_VOUT", Input::Ain3, 1.0),
    ("VDAC_REF", Input::Ain4, 1.0),
    ("3V3", Input::Ain5, 2.0),
    ("5V0", Input::Ain6, 2.0),
];

fn main() -> Result<()> {
    // spi1-1cs gives the kernel GPIO18 as chip select, so no GPIO is passed.
    let mut adc = ADS1261::new_with_gpio_pin(SPI_PATH, None, Reference::Internal)?;
    println!("ADS1261 on {SPI_PATH}: internal 2.5 V reference, 20 SPS");

    loop {
        for (name, input, scale) in INPUTS {
            let code = adc.read_code(input, Input::Aincom)?;
            let volts = adc.code_to_volts(code) * scale;
            // A code at either end of the range means the input is at or
            // beyond the reference, so the real voltage is unknown.
            let clipped = code == 8_388_607 || code == -8_388_608;
            let note = if clipped { " (clipped)" } else { "" };
            println!("{name:>9}: {volts:.4} V{note}");
        }

        println!();
        thread::sleep(Duration::from_secs(1));
    }
}
