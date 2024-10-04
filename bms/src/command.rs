use crate::gpio::{
  self, Gpio, PinMode::Output, PinValue::{High, Low}
};

// controller = floor(GPIO#/32)
// pin = remainder

pub fn enable_battery_power(gpio_controllers: &[Gpio]) {
  // P8 GPOI 36 Pin 69
  let mut pin = gpio_controllers[1].get_pin(4);
  pin.mode(Output);
  pin.digital_write(High);
}

pub fn disable_battery_power(gpio_controllers: &[Gpio]) {
  // P8 GPOI 36 Pin 69
  let mut pin = gpio_controllers[1].get_pin(4);
  pin.mode(Output);
  pin.digital_write(Low);
}

pub fn enable_sam_power(gpio_controllers: &[Gpio]) {
  // P8 GPIO22 Pin 65
  let mut pin = gpio_controllers[0].get_pin(22);
  pin.mode(Output);
  pin.digital_write(High);
}

pub fn disable_sam_power(gpio_controllers: &[Gpio]) {
  // P8 GPIO22 Pin 65
  let mut pin = gpio_controllers[0].get_pin(22);
  pin.mode(Output);
  pin.digital_write(Low);
}

pub fn enable_charger(gpio_controllers: &[Gpio]) {
  let mut pin = gpio_controllers[2].get_pin(25);
  pin.mode(Output);
  pin.digital_write(High);
}

pub fn disable_charger(gpio_controllers: &[Gpio]) {
  let mut pin = gpio_controllers[2].get_pin(25);
  pin.mode(Output);
  pin.digital_write(Low);
}

pub fn estop_reset(gpio_controllers: &[Gpio]) {
  // P8 GPIO 65 Pin 64
  let mut pin = gpio_controllers[2].get_pin(1);
  pin.mode(Output);
  pin.digital_write(High);
}

pub fn set_estop_low(gpio_controllers: &[Gpio]) {
  let mut pin = gpio_controllers[2].get_pin(1);
  pin.mode(Output);
  pin.digital_write(Low);
}

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