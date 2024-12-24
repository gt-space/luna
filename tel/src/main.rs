pub mod pins;

use pins::GPIO_CONTROLLERS;
use sx1280::state::{SX1280, StandbyData, SleepData, FsData, RxDutyCycleData, TxData};

fn main() {
  let mut driver = SX1280::new(
    "/dev/spidev0.0", 
    GPIO_CONTROLLERS[1].get_pin(0), 
    GPIO_CONTROLLERS[1].get_pin(1), 
    GPIO_CONTROLLERS[1].get_pin(2), 
    GPIO_CONTROLLERS[1].get_pin(3), 
    GPIO_CONTROLLERS[1].get_pin(4), 
    GPIO_CONTROLLERS[1].get_pin(5)
  );

  driver.enable_auto_fs();

  let mut driver = driver.set_fs();
}