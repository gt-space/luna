use common::comm::{gpio::{ControlModuleRegister, Gpio, Pin, PinMode::Output, PinValue::{High, Low}}, sam::SamControlMessage};
use std::{thread, time::Duration};
use std::collections::HashMap;
use once_cell::sync::Lazy;

use crate::pins::{GPIO_CONTROLLERS, VALVE_PINS, VALVE_CURRENT_PINS, SPI_INFO, GpioInfo};
use crate::{SamVersion, SAM_VERSION};

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

/* So the Beaglebone is really annoying and the config-pin script is not
available to be used on every pin. Some pins have GPIO as available modes
but are initially used during boot in the LCD and GPMC modes. While I could have
attempted to modify this and make them available through device tree overlays
I instead go through /dev/mem access to directly modify the registers that
control the pins, very similar to how we use /dev/mem to toggle GPIOs and read
their states.
 */
pub fn fix_gpio() {
  // handle pesky boot pins to become GPIOs
  match *SAM_VERSION {
    SamVersion::Rev3 => {
      // I cause different types of problems!
    },

    SamVersion::Rev4Ground => {
      ControlModuleRegister::conf_gpmc_ad0.change_pin_mode(7);
      ControlModuleRegister::conf_gpmc_ad0.disable_pull_resistor();

      ControlModuleRegister::conf_gpmc_ad4.change_pin_mode(7);
      ControlModuleRegister::conf_gpmc_ad4.disable_pull_resistor();
    },

    SamVersion::Rev4Flight => {
      ControlModuleRegister::conf_lcd_data2.change_pin_mode(7);
      ControlModuleRegister::conf_lcd_data2.disable_pull_resistor();
    }
  }
}

pub fn init_gpio() {
  // disable all chip selects
  for spi_info in SPI_INFO.values() {
    if let Some(cs_info) = &spi_info.cs {
      let mut cs_pin = GPIO_CONTROLLERS[cs_info.controller].get_pin(cs_info.pin_num);
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
  actuate_valve(1, false);
  actuate_valve(2, false);
  actuate_valve(3, false);
  actuate_valve(4, false);
  actuate_valve(5, false);
  actuate_valve(6, false);

  // handle the pins that choose which valve the current feedback is from
  if *SAM_VERSION != SamVersion::Rev3 {
    for gpio_info in VALVE_CURRENT_PINS.values() {
      let mut pin = GPIO_CONTROLLERS[gpio_info.controller].get_pin(gpio_info.pin_num);
      pin.mode(Output); // like so incredibly redundant
      pin.digital_write(Low); // start on valves 1, 3, 5
    }
  }
}

fn actuate_valve(channel: u32, powered: bool) {
  if (channel < 1 || channel > 6) {
    panic!("Invalid valve channel number")
  }

  let gpio_info = VALVE_PINS.get(&channel).unwrap();
  let mut pin = GPIO_CONTROLLERS[gpio_info.controller].get_pin(gpio_info.pin_num);
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