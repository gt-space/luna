#![warn(missing_docs)]
#![warn(clippy::correctness)]

//! Servo is the library/binary hybrid written for the Yellow Jacket Space
//! Program's control server.

/// Components related to interacting with the terminal and developer display.
pub mod interface;

/// Components related to the server, including route functions, forwarding,
/// flight communication, and the interface.
pub mod server;

/// Everything related to the Servo command line tool.
pub mod tool;
