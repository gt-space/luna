use common::comm::gpio::{Gpio, Pin, PinMode::{Input, Output}};
use std::{collections::HashMap, process::Command};
use hostname;
use crate::{SAM_INFO, SamVersion};
use once_cell::sync::Lazy;

pub static GPIO_CONTROLLERS: Lazy<Vec<Gpio>> = Lazy::new(|| open_controllers());
pub static VALVE_PINS: Lazy<HashMap<u8, GpioInfo>> = Lazy::new(|| get_valve_sel_mappings());

pub struct GpioInfo {
  pub controller: usize,
  pub pin_num: usize
}

pub fn open_controllers() -> Vec<Gpio> {
  (0..=3).map(Gpio::open_controller).collect()
}

pub fn get_valve_sel_mappings() -> HashMap<int, GpioInfo> {
  let mut map = HashMap::new();

  match SAM_INFO.version {
    SamVersion::Rev3 => {
      map.insert(1, GpioInfo { controller: 0, pin_num: 8 });
      map.insert(2, GpioInfo { controller: 2, pin_num: 16 });
      map.insert(3, GpioInfo { controller: 2, pin_num: 17 });
      map.insert(4, GpioInfo { controller: 2, pin_num: 25 });
      map.insert(5, GpioInfo { controller: 2, pin_num: 1 });
      map.insert(6, GpioInfo { controller: 1, pin_num: 14 });
    },

    SamVersion::Rev4Ground => {
      map.insert(1, GpioInfo { controller: 1, pin_num: 0 });
      map.insert(2, GpioInfo { controller: 1, pin_num: 4 });
      map.insert(3, GpioInfo { controller: 1, pin_num: 14 });
      map.insert(4, GpioInfo { controller: 1, pin_num: 15 });
      map.insert(5, GpioInfo { controller: 0, pin_num: 15 });
      map.insert(6, GpioInfo { controller: 1, pin_num: 17 });
    },

    SamVersion::Rev4Flight => {
      map.insert(1, GpioInfo { controller: 2, pin_num: 16 });
      map.insert(2, GpioInfo { controller: 1, pin_num: 16 });
      map.insert(3, GpioInfo { controller: 2, pin_num: 13 });
      map.insert(4, GpioInfo { controller: 1, pin_num: 17 });
      map.insert(5, GpioInfo { controller: 3, pin_num: 19 });
      map.insert(6, GpioInfo { controller: 2, pin_num: 8 });
    }
  };

  map
}

pub fn config_pins() {
  if SAM_INFO.version == SamVersion::Rev3 {
    // P8 GPIO
    config_pin("p8.10", "gpio");
    config_pin("p8.11", "gpio");
    config_pin("p8.12", "gpio");
    config_pin("p8.13", "gpio");
    config_pin("p8.14", "gpio");
    config_pin("p8.15", "gpio");
    config_pin("p8.16", "gpio");
    config_pin("p8.17", "gpio");
    config_pin("p8.18", "gpio");
    config_pin("p8.19", "gpio");
    config_pin("p8.20", "gpio");
    config_pin("p8.21", "gpio");
    config_pin("p8.22", "gpio");
    config_pin("p8.23", "gpio");
    config_pin("p8.24", "gpio");
    config_pin("p8.25", "gpio");
    config_pin("p8.26", "gpio");
    config_pin("p8.27", "gpio");
    config_pin("p8.28", "gpio");
    config_pin("p8.29", "gpio");
    config_pin("p8.30", "gpio");
    config_pin("p8.31", "gpio");
    config_pin("p8.41", "gpio");
    config_pin("p8.42", "gpio");
    config_pin("p8.43", "gpio");
    config_pin("p8.44", "gpio");
    config_pin("p8.45", "gpio");
    config_pin("p8.46", "gpio");

    // P9 GPIO
    config_pin("p9.11", "gpio");
    config_pin("p9.12", "gpio");
    config_pin("p9.13", "gpio");
    config_pin("p9.14", "gpio");
    config_pin("p9.15", "gpio");
    config_pin("p9.16", "gpio");
    config_pin("p9.17", "gpio");
    config_pin("p9.18", "gpio");
    config_pin("p9.19", "gpio");
    config_pin("p9.20", "gpio");
    config_pin("p9.21", "gpio");
    config_pin("p9.22", "gpio");
    config_pin("p9.23", "gpio");
    config_pin("p9.24", "gpio");
    config_pin("p9.25", "gpio");
    config_pin("p9.26", "gpio");
    config_pin("p9.27", "gpio");
    config_pin("p9.28", "gpio");
    config_pin("p9.29", "gpio");
    config_pin("p9.30", "gpio");
    config_pin("p9.31", "gpio");
    config_pin("p9.41", "gpio");
    config_pin("p9.42", "gpio");

    // SPI
    config_pin("p9_17", "spi_cs");
    config_pin("p9_18", "spi");
    config_pin("p9_21", "spi");
    config_pin("p9_22", "spi_sclk");

  } else if SAM_INFO.version == SamVersion::Rev4Ground {
    // P8 GPIO
    config_pin("p8.7", "gpio");
    config_pin("p8.8", "gpio");
    config_pin("p8.9", "gpio");
    config_pin("p8.10", "gpio");
    config_pin("p8.11", "gpio");
    config_pin("p8.12", "gpio");
    config_pin("p8.13", "gpio");
    config_pin("p8.14", "gpio");
    config_pin("p8.15", "gpio");
    config_pin("p8.16", "gpio");
    config_pin("p8.19", "gpio");
    config_pin("p8.21", "gpio");
    config_pin("p8.23", "gpio");
    config_pin("p8.25", "gpio");

    // P9 GPIO
    config_pin("p9.11", "gpio");
    config_pin("p9.12", "gpio");
    config_pin("p9.13", "gpio");
    config_pin("p9.14", "gpio");
    config_pin("p9.15", "gpio");
    config_pin("p9.16", "gpio");
    config_pin("p9.23", "gpio");
    config_pin("p9.24", "gpio");
    config_pin("p9.25", "gpio");
    config_pin("p9.26", "gpio");
    config_pin("p9.27", "gpio");
    config_pin("p9.28", "gpio"); // somehow works even tho its SPI

    // SPI 0 (slow)
    config_pin("p9_18", "spi");
    config_pin("p9_21", "spi");
    config_pin("p9_22", "spi_sclk");

    // SPI 1 (fast)
    config_pin("p9_19", "spi_cs");
    config_pin("p9_29", "spi");
    config_pin("p9_30", "spi");
    config_pin("p9_31", "spi_sclk");

  } else if SAM_INFO.version == SamVersion::Rev4Flight {
    // P8 GPIO
    config_pin("p8.7", "gpio");
    config_pin("p8.8", "gpio");
    config_pin("p8.9", "gpio");
    config_pin("p8.10", "gpio");
    config_pin("p8.11", "gpio");
    config_pin("p8.12", "gpio");
    config_pin("p8.13", "gpio");
    config_pin("p8.14", "gpio");
    config_pin("p8.15", "gpio");
    config_pin("p8.16", "gpio");
    config_pin("p8.17", "gpio");
    config_pin("p8.18", "gpio");
    config_pin("p8.19", "gpio");
    config_pin("p8.21", "gpio");
    config_pin("p8.36", "gpio");
    config_pin("p8.37", "gpio");
    config_pin("p8.38", "gpio");
    config_pin("p8.39", "gpio");
    config_pin("p8.40", "gpio");
    config_pin("p8.41", "gpio");
    config_pin("p8.42", "gpio");
    config_pin("p8.43", "gpio");
    config_pin("p8.44", "gpio");
    config_pin("p8.45", "gpio");
    config_pin("p8.46", "gpio");

    // P9 GPIO
    config_pin("p9.11", "gpio");
    config_pin("p9.12", "gpio");
    config_pin("p9.14", "gpio");
    config_pin("p9.15", "gpio");
    config_pin("p9.16", "gpio");
    config_pin("p9.23", "gpio");
    config_pin("p9.24", "gpio");
    config_pin("p9.25", "gpio");
    config_pin("p9.26", "gpio");
    config_pin("p9.27", "gpio");

    // SPI 0 (slow)
    config_pin("p9_17", "spi_cs");
    config_pin("p9_18", "spi");
    config_pin("p9_21", "spi");
    config_pin("p9_22", "spi_sclk");

    // SPI 1 (fast)
    config_pin("p9_19", "spi_cs");
    config_pin("p9_28", "spi_cs");
    config_pin("p9_29", "spi");
    config_pin("p9_30", "spi");
    config_pin("p9_31", "spi_sclk");
  }
}

fn config_pin(pin: &str, mode: &str) {
  match Command::new("dash")
    .args(["config-pin.sh", pin, mode])
    .output() {
      Ok(result) => {
        if result.status.success() {
          println!("Configured {pin} as {mode}");
        } else {
          println!("Configuration did not work for {pin} -> {mode}");
        }
      },

      Err(e) => {
        eprintln!("Failed to execute config-pin for {pin} -> {mode}, Error: {e}");
      }
    }
}
