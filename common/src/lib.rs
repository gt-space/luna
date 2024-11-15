#![warn(missing_docs)]

//! Common consists of the common shared types between different parts of the
//! YJSP software stack. More specifically, the types sent across the network
//! between the flight computer, control server, GUI, and SAM boards are all
//! stored here.

/// All structs and definitions related to communication between different
/// subsystems.
pub mod comm;

/// All components necessary to run Python sequences.
#[cfg(feature = "sequences")]
pub mod sequence;

/// Trait providing a method to create a pretty, terminal-friendly
/// representation of the underlying.
pub trait ToPrettyString {
  /// Provides a representation of the underlying which is preferable when
  /// displaying to the console but not as a raw string.
  ///
  /// ANSI codes such as color codes, for example, can be used in a "pretty
  /// string" but would be atypical in a `fmt::Display` implementation.
  fn to_pretty_string(&self) -> String;
}
