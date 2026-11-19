//! Standalone example: prints the TMP112's temperature and alert state once a
//! second.
//!
//! On the HITL test board with the CM5 adapter, the TMP112 is on I2C bus 1
//! (GPIO2 and GPIO3). ADD0 is drawn tied to ground, which is address 0x48,
//! but a note on the schematic says 0x49. Pass `49` to try the other one.
//!
//! The alert limits are set 1 and 2 degrees Celsius above the starting
//! temperature, so warming the sensor with a finger turns the alert on.
//!
//! Run on the CM5: `cargo run -p tmp112 --example read_temp -- [48|49]`. It
//! runs until Ctrl-C.

use std::{env, process, thread, time::Duration};

use tmp112::{Address, Result, TEMPERATURE_MAX, TMP112};

const I2C_BUS: u8 = 1;

fn main() -> Result<()> {
    let arg = env::args().nth(1);
    let address = match arg.as_deref().map(|arg| arg.trim_start_matches("0x")) {
        None | Some("48") => Address::Gnd,
        Some("49") => Address::VPlus,
        Some(other) => {
            eprintln!("usage: read_temp [48|49] (got {other})");
            process::exit(2);
        }
    };

    let mut sensor = TMP112::new(I2C_BUS, address)?;

    // Wait for a conversion to finish, then set the limits just above the
    // starting temperature.
    thread::sleep(Duration::from_millis(300));
    let start = sensor.read_celsius()?;
    let high = (start + 2.0).min(TEMPERATURE_MAX);
    let low = (start + 1.0).min(high);
    sensor.set_limits(low, high)?;
    println!(
        "TMP112 on bus {I2C_BUS} at {:#04X}: alert on at {high:.1} C, off below {low:.1} C",
        address as u16
    );

    loop {
        let celsius = sensor.read_celsius()?;
        let alert = if sensor.alert()? { " (alert)" } else { "" };
        println!("{celsius:.4} C{alert}");

        thread::sleep(Duration::from_secs(1));
    }
}
