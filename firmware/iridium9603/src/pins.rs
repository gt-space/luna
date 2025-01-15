use std::thread;
use std::time::Duration;
use rppal::gpio::Gpio;

const TX_PIN: u8 = 14;
const RX_PIN: u8 = 15;

fn main() -> Result<(), Box<dyn std::error::Error>> {
  let mut pin = Gpio::new()?.get(TX_PIN)?into.output();

  println("Connected");

  loop {
    pin.set_high();
    thread::sleep(Duration::from_millis(500));

    pin.set_low();
    thread::sleep(Duration::from_millis(500));
  }
}