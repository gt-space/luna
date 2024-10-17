extern crate spidev;
use std::io;
use std::io::prelude::*;
use spidev::{Spidev, SpidevOptions, SpidevTransfer, SpiModeFlags};
use common::comm::gpio::{Pin, PinMode, PinValue, Gpio};

mod driver;
mod internals;
pub use driver::*;