//! The service that runs on the HITL test board's CM5, and the client the
//! HITL worker on the Meerkat uses to talk to it.
//!
//! The worker sends commands (set a DAC channel, read an ADC input, and so
//! on) over UDP. The service runs each one against the board's chips and
//! answers. The messages are in [`common::comm::hitl`].
//!
//! - [`hardware`] wraps the chip drivers behind one trait, with a mock for
//!   tests and for running off the board.
//! - [`service`] is the CM5 side: the request loop and the heartbeat watchdog.
//! - [`client`] is the Meerkat side.

pub mod client;
pub mod hardware;
pub mod service;
