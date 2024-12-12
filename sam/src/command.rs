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
    if let Some(cs_info) = spi_info.cs {
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

fn execute(command: SamControlMessage, gpio_controllers: Vec<Arc<Gpio>>) {
  match command {
    SamControlMessage::SetLed { channel, on } => match on {
      true => match channel {
        0 => {
          let mut file: File = std::fs::OpenOptions::new()
            .write(true)
            .truncate(true)
            .open("/sys/class/leds/beaglebone:green:usr0/brightness")
            .unwrap();
          file.write_all(b"1").expect("Failed to write");
        }
        1 => {
          let mut file: File = std::fs::OpenOptions::new()
            .write(true)
            .truncate(true)
            .open("/sys/class/leds/beaglebone:green:usr1/brightness")
            .unwrap();
          file.write_all(b"1").expect("Failed to write");
        }
        2 => {
          let mut file: File = std::fs::OpenOptions::new()
            .write(true)
            .truncate(true)
            .open("/sys/class/leds/beaglebone:green:usr2/brightness")
            .unwrap();
          file.write_all(b"1").expect("Failed to write");
        }
        3 => {
          let mut file: File = std::fs::OpenOptions::new()
            .write(true)
            .truncate(true)
            .open("/sys/class/leds/beaglebone:green:usr3/brightness")
            .unwrap();
          file.write_all(b"1").expect("Failed to write");
        }
        _ => println!("Error"),
      },
      false => match channel {
        0 => {
          let mut file: File = std::fs::OpenOptions::new()
            .write(true)
            .truncate(true)
            .open("/sys/class/leds/beaglebone:green:usr0/brightness")
            .unwrap();
          file.write_all(b"0").expect("Failed to write");
        }
        1 => {
          let mut file: File = std::fs::OpenOptions::new()
            .write(true)
            .truncate(true)
            .open("/sys/class/leds/beaglebone:green:usr1/brightness")
            .unwrap();
          file.write_all(b"0").expect("Failed to write");
        }
        2 => {
          let mut file: File = std::fs::OpenOptions::new()
            .write(true)
            .truncate(true)
            .open("/sys/class/leds/beaglebone:green:usr2/brightness")
            .unwrap();
          file.write_all(b"0").expect("Failed to write");
        }
        3 => {
          let mut file: File = std::fs::OpenOptions::new()
            .write(true)
            .truncate(true)
            .open("/sys/class/leds/beaglebone:green:usr3/brightness")
            .unwrap();
          file.write_all(b"0").expect("Failed to write");
        }
        _ => println!("Error"),
      },
    },

    SamControlMessage::ActuateValve { channel, powered } => {
      actuate_valve(channel, powered);
    },
  }
}

fn actuate_valve(channel: u8, powered: bool) {
  if (channel < 1 || channel > 6) {
    fail!("Invalid valve channel number")
  }

  let info = VALVE_PINS.get(&channel).unwrap();
  let mut pin = GPIO_CONTROLLERS[info.controller].get_pin(info.pin_num);
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