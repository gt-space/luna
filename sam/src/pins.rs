use common::comm::gpio::{Gpio, Pin, PinMode::{Input, Output}};
use common::
use std::{collections::HashMap, pin};
use hostname;
use crate::{SAM_INFO, SamVersion};
use once_cell::sync::Lazy;

pub static GPIO_CONTROLLERS: Lazy<Vec<Gpio>> = Lazy::new(|| open_controllers());
pub static VALVE_PINS: Lazy<HashMap<u8, GpioInfo>> = Lazy::new(|| get_valve_sel_mappings());

pub struct GpioInfo {
  pub controller: usize,
  pub pin: usize
}

pub fn open_controllers() -> Vec<Gpio> {
  (0..=3).map(Gpio::open_controller).collect()
}

pub fn get_valve_sel_mappings() -> HashMap<int, GpioInfo> {
  let mut map = HashMap::new();

  match SAM_INFO.version {
    SamVersion::Rev3 => {
      map.insert(1, GpioInfo { controller: 0, pin: 8 });
      map.insert(2, GpioInfo { controller: 2, pin: 16 });
      map.insert(3, GpioInfo { controller: 2, pin: 17 });
      map.insert(4, GpioInfo { controller: 2, pin: 25 });
      map.insert(5, GpioInfo { controller: 2, pin: 1 });
      map.insert(6, GpioInfo { controller: 1, pin: 14 });
    },

    SamVersion::Rev4Ground => {
      map.insert(1, GpioInfo { controller: 1, pin: 0 });
      map.insert(2, GpioInfo { controller: 1, pin: 4 });
      map.insert(3, GpioInfo { controller: 1, pin: 14 });
      map.insert(4, GpioInfo { controller: 1, pin: 15 });
      map.insert(5, GpioInfo { controller: 0, pin: 15 });
      map.insert(6, GpioInfo { controller: 1, pin: 17 });
    },

    SamVersion::Rev4Flight => {
      map.insert(1, GpioInfo { controller: 2, pin: 16 });
      map.insert(2, GpioInfo { controller: 1, pin: 16 });
      map.insert(3, GpioInfo { controller: 2, pin: 13 });
      map.insert(4, GpioInfo { controller: 1, pin: 17 });
      map.insert(5, GpioInfo { controller: 3, pin: 19 });
      map.insert(6, GpioInfo { controller: 2, pin: 8 });
    }
  };

  map
}

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
