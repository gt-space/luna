use common::comm::gpio::{Gpio, Pin, PinMode::{Input, Output}};
use once_cell::sync::Lazy;

pub static GPIO_CONTROLLERS: Lazy<Vec<Gpio>> = Lazy::new(|| open_controllers());

pub fn open_controllers() -> Vec<Gpio> {
  (0..=3).map(Gpio::open_controller).collect()
}
