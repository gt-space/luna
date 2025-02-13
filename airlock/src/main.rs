//! Airlock is a tool used for isolated firmware and device debugging.

#![warn(missing_docs)]

use checkout::Checkout;
use ms5611::MS5611;
use std::{env, process::exit};

fn main() {
  let args = env::args()
    .collect::<Vec<String>>();

  // The firmware name is given as the first argument.
  let Some(firmware) = args.get(1) else {
    eprintln!("\x1b[31;1merror:\x1b[0m no firmware specified");
    exit(1);
  };

  // Create different devices depending on the firmware specified.
  let mut device: Box<dyn Checkout> = match firmware.as_str() {
    // MS5611 barometer firmware configuration.
    "ms5611" => {
      let Some(bus_path) = args.get(2) else {
        eprintln!("\x1b[31;1merror:\x1b[0m bus path not supplied");
        exit(1);
      };

      let Ok(device) = MS5611::new(bus_path, 256) else {
        eprintln!("\x1b[31;1merror:\x1b[0m failed to open device");
        exit(1);
      };

      Box::new(device)
    },
    _ => {
      eprintln!("\x1b[31;1merror:\x1b[0m unrecognized firmware");
      exit(1);
    },
  };

  // Interact with the device and print diagnostic information.
  println!("[ {} ]\n", device.name());
  device.interface().print_details();
  device.print_checkout();
}
