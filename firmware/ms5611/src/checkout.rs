//! Components that can be called from Airlock to debug the MS5611 or its
//! firmware.

use checkout::{Checkout, Interface};
use crate::Channel;
use super::MS5611;

impl Checkout for MS5611 {
  fn name(&self) -> &'static str {
    "MS5611-01BA03"
  }

  fn interface(&self) -> Interface {
    Interface::SPI(&self.spi)
  }

  #[allow(unused)]
  fn print_checkout(&mut self) {
    // PROM contents.
    println!("PROM:");
    println!("  PROM[0] (reserved) : {:#04x}", self.prom.factory_data);
    println!("  PROM[1] (SENS_T1)  : {}", self.prom.sens_t1);
    println!("  PROM[2] (OFF_T1)   : {}", self.prom.off_t1);
    println!("  PROM[3] (TCS)      : {}", self.prom.tcs);
    println!("  PROM[4] (TCO)      : {}", self.prom.tco);
    println!("  PROM[5] (T_REF)    : {}", self.prom.t_ref);
    println!("  PROM[6] (TEMPSENS) : {}", self.prom.tempsens);
    println!("  PROM[7] (CRC)      : {:#04x}", self.prom.crc);
    println!();

    // D1 and D2 conversions check.
    println!("Conversions (OSR = 256):");
    print!("  D1: ");

    self.set_osr(256);
    self.convert(Channel::Pressure);

    if let Ok(d1) = self.read_raw() {
      println!("{d1}");
    } else {
      println!("\x1b[31;1mfailed\x1b[0m");
    }

    print!("  D2: ");

    self.convert(Channel::Temperature);

    if let Ok(d2) = self.read_raw() {
      println!("{d2}");
    } else {
      println!("\x1b[31;1mfailed\x1b[0m");
    }

    println!();

    // Readings check with all valid OSRs.
    println!("Readings:");

    for osr in [256, 512, 1024, 2048, 4096] {
      println!("  OSR = {osr}:");
      print!("    Temperature: ");

      if let Ok(temperature) = self.read_temperature() {
        println!("{temperature}");
      } else {
        println!("\x1b[31;1mfailed\x1b[0m");
      }

      print!("    Pressure: ");

      if let Ok(pressure) = self.read_pressure() {
        println!("{pressure}");
      } else {
        println!("\x1b[31;1mfailed\x1b[0m");
      }
    }

    println!();

    // Reset check.
    print!("Reset: ");

    if self.reset().is_ok() {
      println!("\x1b[32;1mok\x1b[0m");
    } else {
      println!("\x1b[31;1mfailed\x1b[0m");
    }

    println!();
  }
}

