use std::char;

use common::comm::{Gpio, PinMode::Output, PinValue::{Low, High}, SamControlMessage};

// controller = floor(GPIO#/32)
// pin = remainder

// channel = 10 : powered = True
pub fn enable_battery_power(gpio_controllers: &[Gpio]) {
  // GPIO 61, Pin 72
  let mut pin = gpio_controllers[1].get_pin(29);
  pin.mode(Output);
  pin.digital_write(High);
}

// channel = 10 : powered = False
pub fn disable_battery_power(gpio_controllers: &[Gpio]) {
  // GPIO 61, Pin 72
  let mut pin = gpio_controllers[1].get_pin(29);
  pin.mode(Output);
  pin.digital_write(Low);
}

pub fn disable_rbftag(gpio_controllers: &[Gpio]) {
  // GPIO 66, Pin 53
  let mut pin = gpio_controllers[2].get_pin(2);
  pin.mode(Output);
  pin.digital_write(Low);
}

pub fn enable_rbftag(gpio_controllers: &[Gpio]) {
  // GPIO 66, Pin 53
  let mut pin = gpio_controllers[2].get_pin(2);
  pin.mode(Output);
  pin.digital_write(High);
}

pub fn reco_disable(channel: u8, gpio_controllers: &[Gpio]) {
  match channel {
    1 => {
      // P8 GPIO 46 Pin 62
      let mut pin = gpio_controllers[1].get_pin(14);
      pin.mode(Output);
      pin.digital_write(Low);
    }
    2 => {
      // P8 GPIO 65 Pin 64
      let mut pin = gpio_controllers[2].get_pin(1);
      pin.mode(Output);
      pin.digital_write(Low);
    }
    3 => {
      // P8 GPIO 67 Pin 54
      let mut pin = gpio_controllers[1].get_pin(22);
      pin.mode(Output);
      pin.digital_write(Low);
    },
    4 => {
      // P8 GPIO 68 Pin 56
      let mut pin = gpio_controllers[1].get_pin(24);
      pin.mode(Output);
      pin.digital_write(Low);
    }
    _ => println!("Error"),
  }
}

pub fn execute(gpio_controllers: &[Gpio], command: SamControlMessage) {
  match command {
    SamControlMessage::ActuateValve{channel, powered} => {
      match channel {
        20 => {
          if powered {
            enable_battery_power(gpio_controllers);
          } else {
            disable_battery_power(gpio_controllers);
          }
        },
        _ => {
          eprintln!("Unrecognized Channel: {channel}");
        }
      };
    },
    _ => {
      eprintln!("Unrecognized Command: {command:#?}");
    }
  };
}

// HOW TO ACTIVATE BMS COMMANDS:
// Mapppings settings:
// Text ID (channel) = battey_power (20)
// SensorType = Valve
// Computer = Flight
// NormallyClosed = False
// Board ID = bms-01
// HOW TO SET BMS PROPERTIES
// Open Valve = True
// Close Valve = False