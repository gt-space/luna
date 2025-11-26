use pyo3::prelude::*;
use pyo3::wrap_pyfunction;
// use common::comm::gpio::{Gpio, Pin, PinMode, PinValue};
#[cfg(any(test, feature = "test_mode"))]
use crate::mocks::MockController as Controller;
use once_cell::sync::Lazy;
use std::sync::Mutex;
#[cfg(not(any(test, feature = "test_mode")))]
use common::comm::gpio::Gpio as Controller;

use common::comm::{
  gpio::{
    PinMode::Output,
    PinValue::{High, Low},
  },
  sam::SamControlMessage,
};

use crate::pins::GPIO_CONTROLLERS;
use crate::pins::{SPI_INFO, VALVE_CURRENT_PINS, VALVE_PINS};
use crate::version::{SamVersion, SAM_VERSION};

#[cfg(any(test, feature = "test_mode"))]
static VALVE_STATES: Lazy<Mutex<[bool; 6]>> = Lazy::new(|| Mutex::new([false; 6]));

pub fn execute(command: SamControlMessage) {
  match command {
    SamControlMessage::ActuateValve { channel, powered } => {
      actuate_valve(channel, powered);
    }
  }
}

pub fn safe_valves() {
  for i in 1..7 {
    actuate_valve(i, false); // turn off all valves
  }
}

pub fn init_gpio() {
  // disable all chip selects
  for spi_info in SPI_INFO.values() {
    if let Some(cs_info) = &spi_info.cs {
      let mut cs_pin =
        GPIO_CONTROLLERS[cs_info.controller].get_pin(cs_info.pin_num);
      cs_pin.mode(Output);
      // chip select is active low so make it high to disable
      cs_pin.digital_write(High);
    }
  }

  // handles CS for cold junction IC on rev3 (not an ADC)
  if *SAM_VERSION == SamVersion::Rev3 {
    let mut cs_tc_cjc1 = GPIO_CONTROLLERS[2].get_pin(23);
    cs_tc_cjc1.mode(Output);
    cs_tc_cjc1.digital_write(High); // chip select is active low

    let mut cs_tc_cjc2 = GPIO_CONTROLLERS[0].get_pin(7);
    cs_tc_cjc2.mode(Output);
    cs_tc_cjc2.digital_write(High); // chip select is active low
  }

  // turn off all valves
  safe_valves();
  // initally measure valve currents on valves 1, 3, and 5 for rev4
  reset_valve_current_sel_pins();
}

pub fn reset_valve_current_sel_pins() {
  // handle the pins that choose which valve the current feedback is from
  if *SAM_VERSION != SamVersion::Rev3 {
    for gpio_info in VALVE_CURRENT_PINS.values() {
      let mut pin =
        GPIO_CONTROLLERS[gpio_info.controller].get_pin(gpio_info.pin_num);
      pin.mode(Output); // like so incredibly redundant
      pin.digital_write(Low); // start on valves 1, 3, 5
    }
  }
}

fn actuate_valve(channel: u32, powered: bool) {
  if !(1..=6).contains(&channel) {
    panic!("Invalid valve channel number")
  }

  let gpio_info = VALVE_PINS.get(&channel).unwrap();
  let mut pin =
    GPIO_CONTROLLERS[gpio_info.controller].get_pin(gpio_info.pin_num);
  pin.mode(Output);

  match powered {
    true => {
      println!("Powering Valve {}", channel);
      pin.digital_write(High);
    }

    false => {
      println!("Unpowering Valve {}", channel);
      pin.digital_write(Low);
    }
  }
  #[cfg(any(test, feature = "test_mode"))]
    {
        VALVE_STATES.lock().unwrap()[(channel - 1) as usize] = powered;
    }
}

#[pyfunction]
pub fn get_num_valves() -> usize {
    VALVE_CURRENT_PINS.len()
}
#[pyfunction]
#[cfg(any(test, feature = "test_mode"))]
pub fn get_valve_state(channel: u32) -> bool {
    VALVE_STATES.lock().unwrap()[(channel - 1) as usize]
}