use spidev::Spidev;
use crate::gpio::{
  Gpio,
  Pin,
  PinMode::{Input, Output},
  PinValue::{High, Low},
};

#[derive(Debug, PartialEq, Eq, Hash, Clone, Copy)]
pub enum ADCKind {
  VBatUmbCharge,
  SamAnd5V,
}