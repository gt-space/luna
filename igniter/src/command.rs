use crate::{
  pins::{GPIO_CONTROLLER, SPI_INFO, IGNITER_CHANNEL_ENABLE_PINS, CC_FAULT_PINS}, 
};
use common::comm::{
  igniter::Command, 
  gpio::{
    GpioPin, 
    PinMode::{Input, Output}, 
    PinValue::{High, Low}, 
    PinValue,
  },
};

/// Initializes the GPIO pins for the igniter. Currently, only forces the chip 
/// selects high as they are active low. By default, pins are set to input mode 
/// (unless otherwise specified in the device tree) so there is no risk of them 
/// driving something random.
pub fn init_gpio() {
  for spi_info in SPI_INFO.values() {
    if let Some(cs_pin_num) = spi_info.cs {
      let mut cs_pin = GPIO_CONTROLLER.get_pin(cs_pin_num);
      cs_pin.mode(Output);
      // chip select is active low
      cs_pin.digital_write(High);
    }
  }
}

/// Executes a command received from FC.
pub fn execute(command: Command) {
  match command {
    Command::ArmIgniter => {
      arming_igniter(true);
    }
    Command::DisarmIgniter => {
      arming_igniter(false);
    }
    Command::EnableIgniter(channel) => {
      enabling_igniter(true, channel);
    }
    Command::DisableIgniter(channel) => {
      enabling_igniter(false, channel);
    }
    Command::EnableContinuityCurrent => {
      enabling_continuity_current(true);
    }
    Command::DisableContinuityCurrent => {
      enabling_continuity_current(false);
    }
  }
}

/// Arms / disarms the igniter by setting the arming pin high / low
pub fn arming_igniter(should_arm: bool) {
  println!("{} Igniter", if should_arm { "Arming" } else { "Disarming" });
  let mut pin = GPIO_CONTROLLER.get_pin(27);
  pin.mode(Output);
  pin.digital_write(if should_arm { High } else { Low });
}

/// Enables / disables the specified channel on the igniter
pub fn enabling_igniter(should_enable: bool, channel: u8) {
  println!("{} Igniter channel {}", if should_enable { "Enabling" } 
    else { "Disabling" }, channel);

  let enable_pin = IGNITER_CHANNEL_ENABLE_PINS.get(&channel).unwrap();
  let mut pin = GPIO_CONTROLLER.get_pin(*enable_pin);
  pin.mode(Output);
  pin.digital_write(if should_enable { High } else { Low });
}

/// Enables / disables the continuity current on all igniter channels
pub fn enabling_continuity_current(should_enable: bool) {
  println!("{} Continuity current", if should_enable { "Enabling" } 
    else { "Disabling" });
  let mut pin = GPIO_CONTROLLER.get_pin(25);
  pin.mode(Output);
  pin.digital_write(if should_enable { High } else { Low });
}

/// Returns the cc fault pin values for the corresponding igniter device (A or B)
pub fn read_cc_fault() -> [f64; 3] {
  let mut data = [0.0; 3];
  for (channel, pin) in CC_FAULT_PINS.iter() {
    let mut pin = GPIO_CONTROLLER.get_pin(*pin);
    pin.mode(Input);
    data[*channel as usize] = pin.digital_read() as i64 as f64;
  }
  
  data
}