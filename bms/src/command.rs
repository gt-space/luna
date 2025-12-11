use crate::{pins::{GPIO_CONTROLLERS, SPI_INFO}, BMS_VERSION, BmsVersion};
use common::comm::{bms::Command, gpio::PinValue};
use common::comm::gpio::{
  PinMode::{Input, Output},
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

  if *BMS_VERSION == BmsVersion::Rev16Bit {
    // disable reco chip select
    let mut reco_cs_pin = GPIO_CONTROLLERS[1].get_pin(16);
    reco_cs_pin.mode(Output);
    // chip select is active low
    reco_cs_pin.digital_write(High);
  }
}

pub fn enable_battery_power() {
  println!("Enabling Battery power");
  if *BMS_VERSION == BmsVersion::Rev16Bit {
    // P8 GPIO 36 Pin 69
    let mut pin = GPIO_CONTROLLERS[1].get_pin(4);
    pin.mode(Output);
    pin.digital_write(High);
  } else if *BMS_VERSION == BmsVersion::Rev24Bit {
    // P9 GPIO 11 Pin 11
    let mut pin = GPIO_CONTROLLERS[2].get_pin(1);
    pin.mode(Output);
    pin.digital_write(High);
  }
}

pub fn disable_battery_power() {
  println!("Disabling Battery power");
  if *BMS_VERSION == BmsVersion::Rev16Bit {
    // P8 GPIO 36 Pin 69
    let mut pin = GPIO_CONTROLLERS[1].get_pin(4);
    pin.mode(Output);
    pin.digital_write(Low);
  } else if *BMS_VERSION == BmsVersion::Rev24Bit {
    // P8 GPIO 65 Pin 64
    let mut pin = GPIO_CONTROLLERS[2].get_pin(1);
    pin.mode(Output);
    pin.digital_write(Low);
  }
}

pub fn enable_sam_power() {
  println!("Enabling SAM power");
  if *BMS_VERSION == BmsVersion::Rev16Bit {
    // P8 GPIO 22 Pin 65
    let mut pin = GPIO_CONTROLLERS[0].get_pin(22);
    pin.mode(Output);
    pin.digital_write(High);
  } else if *BMS_VERSION == BmsVersion::Rev24Bit {
    // P8 GPIO 46 Pin 62
    let mut pin = GPIO_CONTROLLERS[1].get_pin(14);
    pin.mode(Output);
    pin.digital_write(High);
  }
}

pub fn disable_sam_power() {
  println!("Disabling SAM power");
  if *BMS_VERSION == BmsVersion::Rev16Bit {
    // P8 GPIO 22 Pin 65
    let mut pin = GPIO_CONTROLLERS[0].get_pin(22);
    pin.mode(Output);
    pin.digital_write(Low);
  } else if *BMS_VERSION == BmsVersion::Rev24Bit {
    // P8 GPIO 46 Pin 62
    let mut pin = GPIO_CONTROLLERS[1].get_pin(14);
    pin.mode(Output);
    pin.digital_write(Low);
  }
}

pub fn enable_charger() {
  println!("Enabling charger");
  if *BMS_VERSION == BmsVersion::Rev16Bit {
    // P8 GPIO 89 Pin 76
    let mut pin = GPIO_CONTROLLERS[2].get_pin(25);
    pin.mode(Output);
    pin.digital_write(High);
  } else if *BMS_VERSION == BmsVersion::Rev24Bit {
    // P8 GPIO 61 Pin 72
    let mut pin = GPIO_CONTROLLERS[1].get_pin(29);
    pin.mode(Output);
    pin.digital_write(High);
  }
}

pub fn disable_charger() {
  println!("Disabling charger");
  if *BMS_VERSION == BmsVersion::Rev16Bit {
    // P8 GPIO 89 Pin 76
    let mut pin = GPIO_CONTROLLERS[2].get_pin(25);
    pin.mode(Output);
    pin.digital_write(Low);
  } else if *BMS_VERSION == BmsVersion::Rev24Bit {
    // P8 GPIO 61 Pin 72
    let mut pin = GPIO_CONTROLLERS[1].get_pin(29);
    pin.mode(Output);
    pin.digital_write(Low);
  }
}

// The delays are made from the BMS hardware team for safing the system
pub fn estop_init() {
  println!("Running Estop Init Sequence");
  estop_reset();
}

// need to confirm that pin actually needs to be toggled and for how long
// is estop_init all that is necessary?
pub fn estop_reset() {
  if *BMS_VERSION == BmsVersion::Rev16Bit {
    // P8 GPIO 65 Pin 64
    let mut pin = GPIO_CONTROLLERS[2].get_pin(1);
    pin.mode(Output);
    pin.digital_write(Low);
    thread::sleep(Duration::from_millis(5));
    pin.digital_write(High);
    thread::sleep(Duration::from_millis(5));
    pin.digital_write(Low); 
  } else if *BMS_VERSION == BmsVersion::Rev24Bit {
    // P8 GPIO 47 Pin 61
    let mut pin = GPIO_CONTROLLERS[1].get_pin(15);
    pin.mode(Output);
    pin.digital_write(Low);
    thread::sleep(Duration::from_millis(5));
    pin.digital_write(High);
    thread::sleep(Duration::from_millis(5));
    pin.digital_write(Low);
  }
}

// not a command that can be currently sent from FC
pub fn set_estop_low() {
  if *BMS_VERSION == BmsVersion::Rev16Bit {
    let mut pin = GPIO_CONTROLLERS[2].get_pin(1);
    pin.mode(Output);
    pin.digital_write(Low);
  } else if *BMS_VERSION == BmsVersion::Rev24Bit {
    let mut pin = GPIO_CONTROLLERS[1].get_pin(15);
    pin.mode(Output);
    pin.digital_write(Low);
  }
}

pub fn read_estop() -> PinValue {
  // pin number for both BmsVersion::Rev16Bit and BmsVersion::Rev24Bit
  let mut pin = GPIO_CONTROLLERS[2].get_pin(1); // P8 Pin 55 GPIO 69
  pin.mode(Input);
  pin.digital_read()
}

// not a command that can be currently sent from FC
pub fn reco_enable(channel: u32) {
  if *BMS_VERSION == BmsVersion::Rev16Bit {
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
}

pub fn read_rbf_tag() -> PinValue {
  let mut pin = GPIO_CONTROLLERS[2].get_pin(1); // pin number for BmsVersion::Rev16Bit
  
  if *BMS_VERSION == BmsVersion::Rev24Bit {
    pin = GPIO_CONTROLLERS[2].get_pin(5);
  } 

  pin.mode(Input);
  pin.digital_read()
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
