use common::comm::gpio::{Gpio, Pin, PinMode::{Input, Output}};
use std::{collections::HashMap, pin};
use hostname;
use crate::{SAM_INFO, SamVersion};
use once_cell::sync::Lazy;

pub static GPIO_CONTROLLERS: Lazy<Vec<Gpio>> = Lazy::new(|| open_controllers());
pub static VALVE_PINS: Lazy<HashMap<u8, GpioInfo>> = Lazy::new(|| get_valve_sel_mappings());

pub struct GpioInfo {
  controller: u8,
  pin: u8
}

pub fn open_controllers() -> Vec<Gpio> {
  (0..=3).map(Gpio::open_controller).collect()
}

pub fn get_valve_sel_mappings() -> HashMap<int, GpioInfo> {
  let mut map = HashMap::new();

  match SAM_INFO.version {
    SamVersion::Rev3 => {
      map.insert(1, GpioInfo(0, 8));
      map.insert(2, GpioInfo(2, 16));
    },

    SamVersion::Rev4Ground => {

    },

    SamVersion::Rev4Flight => {

    }
  };

  map
}

pub fn get_cs_mappings() -> HashMap<int, Pin> {

}

pub fn get_drdy_mappings() -> 

pub fn config_pins() {
  if SAM_INFO.version == SamVersion::Rev3 {

  } else if SAM_INFO.version == SamVersion::Rev4Ground {

  } else if SAM_INFO.version == SamVersion::Rev4Flight {

  }
}

fn config_pin(pin: &str, mode: &str) {
  Command::new("dash")
    .args(["config-pin.sh", pin, mode])
    .output()
    .expect("failed to configure pin");
}
