// This file adapted from the example in the rust-spidev README:
//     https://github.com/rust-embedded/rust-spidev

extern crate spidev;

use std::io;
// use std::io::prelude::*;
use spidev::{SpiModeFlags, Spidev, SpidevOptions, SpidevTransfer};
use std::thread;
use std::time::Duration;
pub mod gpio;

fn create_spi() -> io::Result<Spidev> {
  let mut spi = Spidev::open("/dev/spidev0.0")?;
  let options = SpidevOptions::new()
    .bits_per_word(8)
    .max_speed_hz(1_000_000)
    .mode(SpiModeFlags::SPI_MODE_0)
    .build();
  spi.configure(&options)?;
  Ok(spi)
}

/// perform half duplex operations using Read and Write traits
// fn half_duplex(spi: &mut Spidev) -> io::Result<()> {
//     let mut rx_buf = [0_u8; 10];
//     spi.write(&[0x01, 0x02, 0x03])?;
//     spi.read(&mut rx_buf)?;
//     println!("{:?}", rx_buf);
//     Ok(())
// }

/// Perform full duplex operations using Ioctl
fn full_duplex(
  spi: &mut Spidev,
  tx_buf: &[u8],
  rx_buf: &mut [u8],
) -> io::Result<()> {
  // "write" transfers are also reads at the same time with
  // the read having the same length as the write
  {
    let mut transfer = SpidevTransfer::read_write(tx_buf, rx_buf);
    spi.transfer(&mut transfer)?;
  }
  println!("{:?}", rx_buf);
  Ok(())
}

fn main() {
  let mut spi = create_spi().unwrap();

  gpio::set_output("49");
  gpio::set_high("49"); // 3V3-RX

  gpio::set_output("77");
  gpio::set_low("77"); // HF-NRESET
  thread::sleep(Duration::from_millis(100));
  gpio::set_high("77");

  gpio::set_output("81"); // HF-CS
  gpio::set_high("81");
  gpio::set_output("44"); // GPS-CS
  gpio::set_high("44");
  gpio::set_output("86"); // LF-CS

  let tx_buf = [0x1D, 0x08, 0xAC, 0x00, 0x00, 0x00];
  let mut rx_buf = [0x00; 6];

  gpio::set_low("86");
  thread::sleep(Duration::from_micros(200));
  println!("{:?}", full_duplex(&mut spi, &tx_buf, &mut rx_buf).unwrap());
  gpio::set_high("86");
  thread::sleep(Duration::from_millis(100));

  let tx_buf_wr = [0x0D, 0x08, 0xAC, 0x95];
  let mut rx_buf_wr = [0x00; 4];
  gpio::set_low("86");
  thread::sleep(Duration::from_micros(200));
  println!(
    "{:?}",
    full_duplex(&mut spi, &tx_buf_wr, &mut rx_buf_wr).unwrap()
  );
  gpio::set_high("86");
  thread::sleep(Duration::from_millis(100));
  loop {
    gpio::set_low("86");
    thread::sleep(Duration::from_micros(200));
    println!("{:?}", full_duplex(&mut spi, &tx_buf, &mut rx_buf).unwrap());
    gpio::set_high("86");
    thread::sleep(Duration::from_millis(100));
  }
}

// HF
// gpio::set_output("49");
// gpio::set_high("49"); // 3V3-RX

// gpio::set_output("9");
// gpio::set_low("9"); // HF-NRESET
// thread::sleep(Duration::from_millis(100));
// gpio::set_high("9");

// gpio::set_output("86"); // LF-CS
// gpio::set_high("86");
// gpio::set_output("44"); // GPS-CS
// gpio::set_high("44");
// gpio::set_output("81"); // HF-CS

// let mut tx_buf = [0x19, 0x08, 0x91, 0x00, 0x00, 0x00];
// let mut rx_buf = [0; 6];

// gpio::set_low("81");
// thread::sleep(Duration::from_micros(200));
// println!("{:?}", full_duplex(&mut spi, &mut tx_buf, &mut rx_buf).unwrap());
// gpio::set_high("81");
// thread::sleep(Duration::from_millis(100));

// let mut tx_buf_wr = [0x18, 0x08, 0x91, 0x35];
// let mut rx_buf_wr = [0; 4];
// gpio::set_low("81");
// thread::sleep(Duration::from_micros(200));
// println!("{:?}", full_duplex(&mut spi, &mut tx_buf_wr, &mut
// rx_buf_wr).unwrap()); gpio::set_high("81");
// thread::sleep(Duration::from_millis(100));
// loop {
//     gpio::set_low("81");
//     thread::sleep(Duration::from_micros(200));
//     println!("{:?}", full_duplex(&mut spi, &mut tx_buf, &mut
// rx_buf).unwrap());     gpio::set_high("81");
//     thread::sleep(Duration::from_millis(100));
// }

// GPS
// gpio::set_output("15");
// gpio::set_high("15"); // 3V3-GPS

// gpio::set_output("45");
// gpio::set_low("45"); // GPS-NRESET
// thread::sleep(Duration::from_millis(100));
// gpio::set_high("45");

// gpio::set_output("86"); // LF-CS
// gpio::set_high("86");
// gpio::set_output("81"); // HF-CS
// gpio::set_high("81");
// gpio::set_output("44"); // GPS-CS

// let mut tx_buf = [0xFD, 0xFE];
// let mut rx_buf = [0; 2];

// loop {
//     gpio::set_low("44");
//     thread::sleep(Duration::from_micros(200));
//     println!("{:?}", full_duplex(&mut spi, &mut tx_buf, &mut
// rx_buf).unwrap());     gpio::set_high("44");
//     thread::sleep(Duration::from_millis(100));
// }
