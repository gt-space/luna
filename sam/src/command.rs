use common::comm::{gpio::{Gpio, Pin, PinMode::Output, PinValue::{High, Low}}, sam::SamControlMessage, ADCKind::{self, Sam, SamRev4}, SamADC, SamRev4ADC};
use std::{thread, time::Duration};
use std::collections::HashMap;
use once_cell::sync::Lazy;

use crate::pins::{GPIO_CONTROLLERS, VALVE_PINS, SPI_INFO, GpioInfo};

pub fn init_gpio() {
  // disable all chip selects
  // turn off all valves
  // put valve current sense gpios into low state to sense valves 1, 3, and 5

  for spi_info in SPI_INFO.values() {
    if let Some(cs_info) = &spi_info.cs {
      let mut cs_pin = GPIO_CONTROLLERS[cs_info.controller].get_pin(cs_info.pin_num);
      cs_pin.mode(Output);
      // chip select is active low so make it high to disable
      cs_pin.digital_write(High);
    }
  }

  actuate_valve(1, false);
  actuate_valve(2, false);
  actuate_valve(3, false);
  actuate_valve(4, false);
  actuate_valve(5, false);
  actuate_valve(6, false);
}

pub fn execute(command: SamControlMessage) {
  match command {
    SamControlMessage::ActuateValve { channel, powered } => {
      actuate_valve(channel, powered);
    },

    SamControlMessage::Abort => {
      init_gpio();
    }
  }
}

fn actuate_valve(channel: u32, powered: bool) {
  if (channel < 1 || channel > 6) {
    panic!("Invalid valve channel number")
  }

  let info = VALVE_PINS.get(&(channel as usize)).unwrap();
  let mut pin = GPIO_CONTROLLERS[info.controller as usize].get_pin(info.pin_num as usize);
  pin.mode(Output);

  match powered {
    true => {
      pin.digital_write(High);
    },

    false => {
      pin.digital_write(Low);
    }
  }
}