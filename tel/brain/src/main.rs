use rppal::gpio::Gpio;
use std::{thread, time::Duration};

const V1_PIN: u8 = 3;
const V2_PIN: u8 = 2;
const PERIOD: Duration = Duration::from_secs(1);

fn main() -> Result<(), Box<dyn std::error::Error>> {
  let gpio = Gpio::new()?;
  let mut v1 = gpio.get(V1_PIN)?.into_output();
  let mut v2 = gpio.get(V2_PIN)?.into_output();

  let mut v1_active = true;

  loop {
    if v1_active {
      v2.set_low();
      v1.set_high();
      println!("\x1b[32mV1 ACTIVE\x1b[0m : \x1b[31mV2 IDLE\x1b[0m");
    } else {
      v1.set_low();
      v2.set_high();
      println!("\x1b[31mV1 IDLE  \x1b[0m : \x1b[32mV2 ACTIVE\x1b[0m");
    }

    thread::sleep(PERIOD);
    v1_active = !v1_active;
  }
}
