use crate::gpio::{
  self, Gpio, PinMode::Output, PinValue::{High, Low}
};
use core::time;
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
  thread::sleep(time::Duration::from_millis(5));
  pin.digital_write(Low);
  thread::sleep(time::Duration::from_millis(5));
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

pub fn execute(gpio_controllers: &[Gpio], command: SamControlMessage) {
  match command {
    SamControlMessage::ActuateValve(channel, powered) => {
      match channel {
        10 => {
          if powered {
            enable_battery_power(gpio_controllers);
          } else {
            disable_battery_power(gpio_controllers);
          }
        },
        11 => {
          if powered {
            enable_sam_power(gpio_controllers);
          } else {
            disable_sam_power(gpio_controllers);
          }
        },
        12 => {
          if powered {
            enable_charger(gpio_controllers);
          } else {
            disable_charger(gpio_controllers);
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

// Normally closed must be False for the mappings
// ValveState::Closed == False
// ValveState::Open == True
// Text ID = battey_power, sam_power, charger
// SensorType = Valve
// Computer = Flight
// NormallyClosed = False