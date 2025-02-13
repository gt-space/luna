//! Provides definitions for implementations used by Airlock to display firmware
//! and device checkout information.
//!
//! Although it would be best for these definitions to reside in the `airlock`
//! crate, doing so would create a cyclic dependency between Airlock and the
//! firmware crates, which is disallowed by Cargo.

#![warn(missing_docs)]

use spidev::Spidev;
use std::os::fd::AsRawFd;

/// The interface variant that a firmware device is using.
pub enum Interface<'a> {
  /// Serial peripheral interface.
  SPI(&'a Spidev),
}

fn print_spi_details(spi: &Spidev) {
  use spidev::spidevioctl::{
    get_bits_per_word,
    get_lsb_first,
    get_max_speed_hz,
    get_mode,
  };

  let failed = "\x1b[31;1mfailed\x1b[0m";

  println!("Interface (SPI):");

  // The spidevioctl functions require a raw file descriptor.
  let spi_fd = spi.as_raw_fd();

  print!("  Bits per word: ");

  if let Ok(bits) = get_bits_per_word(spi_fd) {
    println!("{bits}");
  } else {
    println!("{failed}");
  }

  print!("  LSB first: ");

  if let Ok(lsb_first) = get_lsb_first(spi_fd) {
    println!("{lsb_first}");
  } else {
    println!("{failed}");
  }

  print!("  Max Speed: ");

  // Display Hz, kHz, or MHz depending on magnitude.
  if let Ok(hz) = get_max_speed_hz(spi_fd) {
    if hz < 1_000 {
      println!("{} Hz", hz)
    } else if hz < 1_000_000 {
      println!("{} kHz", hz / 1_000)
    } else {
      println!("{} MHz", hz / 1_000_000)
    }
  } else {
    println!("{failed}");
  }

  print!("  Mode: ");

  if let Ok(mode) = get_mode(spi_fd) {
    println!("{mode}");
  } else {
    println!("{failed}");
  }

  println!();
}

impl Interface<'_> {
  /// Interrogates the underlying device and prints its details.
  pub fn print_details(&self) {
    match self {
      Self::SPI(device) => print_spi_details(device),
    }
  }
}

/// Enables the Airlock tool to perform a full checkout of a firmware device.
pub trait Checkout {
  /// Fetches the active interface of the device.
  fn interface<'a>(&'a self) -> Interface<'a>;

  /// Returns the specific device name that the firmware controls.
  ///
  /// This method should always return the same static string per trait
  /// implementation. The only reason why `self` is a parameter is to enable
  /// storing dynamic trait objects with virtual method tables.
  fn name(&self) -> &'static str;

  /// Performs a full checkout / validation of the firmware device and prints
  /// the results.
  ///
  /// This method is intended to be used by Airlock.
  fn print_checkout(&mut self);
}
