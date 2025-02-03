use crate::pins::{GPIO_CONTROLLERS, SPI_INFO};
use common::comm::bms::Command;
use common::comm::gpio::{
  PinMode::Output,
  PinValue::{High, Low},
};
use std::{thread, time::Duration};

// controller = floor(GPIO#/32)
// pin = remainder

pub fn init_gpio() {
  // set battery enable low
  // set sam enable low (disable)
  // set charge enable low (disable)
  // safe estop
  disable_battery_power();
  disable_sam_power();
  disable_charger();
  estop_init();

  for spi_info in SPI_INFO.values() {
    if let Some(cs_info) = &spi_info.cs {
      let mut cs_pin =
        GPIO_CONTROLLERS[cs_info.controller].get_pin(cs_info.pin_num);
      cs_pin.mode(Output);
      // chip select is active low
      cs_pin.digital_write(High);
    }
  }

  // disable reco chip select
  let mut reco_cs_pin = GPIO_CONTROLLERS[1].get_pin(16);
  reco_cs_pin.mode(Output);
  // chip select is active low
  reco_cs_pin.digital_write(High);
}

pub fn enable_battery_power() {
  // P8 GPIO 36 Pin 69
  let mut pin = GPIO_CONTROLLERS[1].get_pin(4);
  pin.mode(Output);
  pin.digital_write(High);
}

pub fn disable_battery_power() {
  // P8 GPIO 36 Pin 69
  let mut pin = GPIO_CONTROLLERS[1].get_pin(4);
  pin.mode(Output);
  pin.digital_write(Low);
}

pub fn enable_sam_power() {
  // P8 GPIO 22 Pin 65
  let mut pin = GPIO_CONTROLLERS[0].get_pin(22);
  pin.mode(Output);
  pin.digital_write(High);
}

pub fn disable_sam_power() {
  // P8 GPIO 22 Pin 65
  let mut pin = GPIO_CONTROLLERS[0].get_pin(22);
  pin.mode(Output);
  pin.digital_write(Low);
}

pub fn enable_charger() {
  let mut pin = GPIO_CONTROLLERS[2].get_pin(25);
  pin.mode(Output);
  pin.digital_write(High);
}

pub fn disable_charger() {
  let mut pin = GPIO_CONTROLLERS[2].get_pin(25);
  pin.mode(Output);
  pin.digital_write(Low);
}

// The delays are made from the BMS hardware team for safing the system
pub fn estop_init() {
  let mut pin = GPIO_CONTROLLERS[2].get_pin(1);
  pin.mode(Output);
  pin.digital_write(High);
  thread::sleep(Duration::from_millis(5));
  pin.digital_write(Low);
  thread::sleep(Duration::from_millis(5));
  pin.digital_write(High);
}

// need to confirm that pin actually needs to be toggled and for how long
// is estop_init all that is necessary?
pub fn estop_reset() {
  // P8 GPIO 65 Pin 64
  let mut pin = GPIO_CONTROLLERS[2].get_pin(1);
  pin.mode(Output);
  pin.digital_write(High);
  thread::sleep(Duration::from_millis(5));
  pin.digital_write(Low);
}

// not a command that can be currently sent from FC
pub fn set_estop_low() {
  let mut pin = GPIO_CONTROLLERS[2].get_pin(1);
  pin.mode(Output);
  pin.digital_write(Low);
}

// not a command that can be currently sent from FC
pub fn reco_enable(channel: u32) {
  match channel {
    1 => {
      // P8 GPIO 68 Pin 56
      let mut pin = GPIO_CONTROLLERS[2].get_pin(4);
      pin.mode(Output);
      pin.digital_write(High);
    }
    2 => {
      // P8 GPIO 67 Pin 54
      let mut pin = GPIO_CONTROLLERS[2].get_pin(3);
      pin.mode(Output);
      pin.digital_write(High);
    }
    3 => {
      // P8 GPIO 66 Pin 53
      let mut pin = GPIO_CONTROLLERS[2].get_pin(2);
      pin.mode(Output);
      pin.digital_write(High);
    }
    _ => println!("Error"),
  }
}

pub fn execute(command: Command) {
  match command {
    Command::Charge(x) => {
      if x {
        enable_charger();
      } else {
        disable_charger();
      }
    }

    Command::BatteryLoadSwitch(x) => {
      if x {
        enable_battery_power();
      } else {
        disable_battery_power();
      }
    }

    Command::SamLoadSwitch(x) => {
      if x {
        enable_sam_power();
      } else {
        disable_sam_power();
      }
    }

    Command::ResetEstop => {
      // explore what actually needs to happen here
      estop_init();
    }
  }
}
