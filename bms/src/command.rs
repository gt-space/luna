use common::comm::{gpio::{Gpio, PinMode::Output, PinValue::{Low, High}}, bms::Command};
use std::{thread, time::Duration};

// controller = floor(GPIO#/32)
// pin = remainder

// channel = 10 : powered = True
pub fn enable_battery_power(gpio_controllers: &[Gpio]) {
  // P8 GPOI 36 Pin 69
  let mut pin = gpio_controllers[1].get_pin(4);
  pin.mode(Output);
  pin.digital_write(High);
}

// channel = 10 : powered = False
pub fn disable_battery_power(gpio_controllers: &[Gpio]) {
  // P8 GPOI 36 Pin 69
  let mut pin = gpio_controllers[1].get_pin(4);
  pin.mode(Output);
  pin.digital_write(Low);
}

// channel = 11 : powered = True
pub fn enable_sam_power(gpio_controllers: &[Gpio]) {
  // P8 GPIO22 Pin 65
  let mut pin = gpio_controllers[0].get_pin(22);
  pin.mode(Output);
  pin.digital_write(High);
}

// channel = 11 : powered = False
pub fn disable_sam_power(gpio_controllers: &[Gpio]) {
  // P8 GPIO22 Pin 65
  let mut pin = gpio_controllers[0].get_pin(22);
  pin.mode(Output);
  pin.digital_write(Low);
}

// channel = 12 : powered = True
pub fn enable_charger(gpio_controllers: &[Gpio]) {
  let mut pin = gpio_controllers[2].get_pin(25);
  pin.mode(Output);
  pin.digital_write(High);
}

// channel = 12 : powered = False
pub fn disable_charger(gpio_controllers: &[Gpio]) {
  let mut pin = gpio_controllers[2].get_pin(25);
  pin.mode(Output);
  pin.digital_write(Low);
}

// can be included in normal execution code
pub fn estop_init(gpio_controllers: &[Gpio]) {
  let mut pin = gpio_controllers[2].get_pin(1);
  pin.mode(Output);
  pin.digital_write(High);
  thread::sleep(Duration::from_millis(5));
  pin.digital_write(Low);
  thread::sleep(Duration::from_millis(5));
  pin.digital_write(High);
}

// not needed rn
pub fn estop_reset(gpio_controllers: &[Gpio]) {
  // P8 GPIO 65 Pin 64
  let mut pin = gpio_controllers[2].get_pin(1);
  pin.mode(Output);
  pin.digital_write(High);
}

// not needed rn
pub fn set_estop_low(gpio_controllers: &[Gpio]) {
  let mut pin = gpio_controllers[2].get_pin(1);
  pin.mode(Output);
  pin.digital_write(Low);
}


// no need to implement now
pub fn reco_enable(channel: u32, gpio_controllers: &[Gpio]) {
  match channel {
    1 => {
      // P8 GPIO 68 Pin 56
      let mut pin = gpio_controllers[2].get_pin(4);
      pin.mode(Output);
      pin.digital_write(High);
    }
    2 => {
      // P8 GPIO 67 Pin 54
      let mut pin = gpio_controllers[2].get_pin(3);
      pin.mode(Output);
      pin.digital_write(High);
    }
    3 => {
      // P8 GPIO 66 Pin 53
      let mut pin = gpio_controllers[2].get_pin(2);
      pin.mode(Output);
      pin.digital_write(High);
    }
    _ => println!("Error"),
  }
}

pub fn execute(gpio_controllers: &[Gpio], command: Command) {
  match command {
    Command::Charge(x) => {
      if x {
        enable_charger(gpio_controllers);
      } else {
        disable_charger(gpio_controllers);
      }
    },

    Command::BatteryLoadSwitch(x) => {
      if x {
        enable_battery_power(gpio_controllers);
      } else {
        disable_battery_power(gpio_controllers);
      }
    },

    Command::SamLoadSwitch(x) => {
      if x {
        enable_battery_power(gpio_controllers);
      } else {
        disable_battery_power(gpio_controllers);
      }
    },

    Command::ResetEstop => {
      estop_reset(gpio_controllers);
    }
  }
}


// DEPRECATED!
// HOW TO ACTIVATE BMS COMMANDS:
// Mapppings settings:
// Text ID (channel) = battey_power (20), sam_power (21), charger (22)
// SensorType = Valve
// Computer = Flight
// NormallyClosed = False
// Board ID = bms-01
// HOW TO SET BMS PROPERTIES
// Open Valve = True
// Close Valve = False