//! Engine controller (EC) logic for the CTV.
//!
//! This crate holds every *decision* the EC makes, with no hardware access,
//! so all of it can be unit-tested on a laptop with `cargo test`. The
//! hardware layer (timers, UARTs, Ethernet, ADC on the STM32H745) only moves
//! bytes and pin states in and out of these types. See `README.md`.
//!
//! - [`protocol`]: messages between the flight computer and the EC
//! - [`supervisor`]: link watchdog and the overall arm/safe logic
//! - [`rcs`]: roll fans M1-M4
//! - [`propeller`]: main propeller ESCs
//! - [`pa12`]: TVC actuators
//! - [`config`]: every tunable or assumed number

#![cfg_attr(not(test), no_std)]

pub mod config;
pub mod pa12;
pub mod propeller;
pub mod protocol;
pub mod rcs;
pub mod supervisor;
