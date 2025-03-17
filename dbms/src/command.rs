use crate::pins::{GPIO_CONTROLLERS, SPI_INFO};
use common::comm::dbms::Command;
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
  disable_ls1_power();
  disable_ls2_power();
  disable_ls3_power();
  disable_ls4_power();

  for spi_info in SPI_INFO.values() {
    if let Some(cs_info) = &spi_info.cs {
      let mut cs_pin =
        GPIO_CONTROLLERS[cs_info.controller].get_pin(cs_info.pin_num);
      cs_pin.mode(Output);
      // chip select is active low
      cs_pin.digital_write(High);
    }
  }
}

pub fn enable_ls1_power() {
  println!("Enabling LS1 power");
  // P9 GPIO 15 Pin 24
  let mut pin = GPIO_CONTROLLERS[0].get_pin(15);
  pin.mode(Output);
  pin.digital_write(High);
}

pub fn disable_ls1_power() {
  println!("Disabling LS1 power");
  // P9 GPIO 15 Pin 24
  let mut pin = GPIO_CONTROLLERS[0].get_pin(15);
  pin.mode(Output);
  pin.digital_write(Low);
}

pub fn enable_ls2_power() {
  println!("Enabling LS2 power");
  // P9 GPIO 117 Pin 25
  let mut pin = GPIO_CONTROLLERS[3].get_pin(21);
  pin.mode(Output);
  pin.digital_write(High);
}

pub fn disable_ls2_power() {
  println!("Disabling LS2 power");
  // P9 GPIO 117 Pin 25
  let mut pin = GPIO_CONTROLLERS[3].get_pin(21);
  pin.mode(Output);
  pin.digital_write(Low);
}

pub fn enable_ls3_power() {
  println!("Enabling LS3 power");
  // P9 GPIO 14 Pin 26
  let mut pin = GPIO_CONTROLLERS[0].get_pin(14);
  pin.mode(Output);
  pin.digital_write(High);
}

pub fn disable_ls3_power() {
  println!("Disabling LS3 power");
  // P9 GPIO 14 Pin 26
  let mut pin = GPIO_CONTROLLERS[0].get_pin(14);
  pin.mode(Output);
  pin.digital_write(Low);
}

pub fn enable_ls4_power() {
  println!("Enabling LS4 power");
  // P9 GPIO 115 Pin 27
  let mut pin = GPIO_CONTROLLERS[3].get_pin(19);
  pin.mode(Output);
  pin.digital_write(High);
}

pub fn disable_ls4_power() {
  println!("Disabling LS4 power");
  // P9 GPIO 115 Pin 27
  let mut pin = GPIO_CONTROLLERS[3].get_pin(19);
  pin.mode(Output);
  pin.digital_write(Low);
}

pub fn execute(command: Command) {
  match command {
    Command::LoadSwitch1(x) => {
      if x {
        enable_ls1_power();
      } else {
        disable_ls1_power();
      }
    }

    Command::LoadSwitch2(x) => {
      if x {
        enable_ls2_power();
      } else {
        disable_ls2_power();
      }
    }

    Command::LoadSwitch3(x) => {
      if x {
        enable_ls3_power();
      } else {
        disable_ls3_power();
      }
    }

    Command::LoadSwitch4(x) => {
      if x {
        enable_ls4_power();
      } else {
        disable_ls4_power();
      }
    }
  }
}
