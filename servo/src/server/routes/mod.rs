/// Route functions requiring admin privilages for execution.
pub mod admin;

/// Route functions related to operator commands.
pub mod command;

/// Route functions for fetching and manipulating data about the flight
/// computer.
pub mod data;

/// Route functions for getting and setting node mappings.
pub mod mappings;

/// Route functions for setting and sending sequences.
pub mod sequence;

/// Route functions for setting and deleting triggers.
pub mod trigger;

pub use admin::*;
pub use command::*;
pub use data::*;
pub use mappings::*;
pub use sequence::*;
pub use trigger::*;
