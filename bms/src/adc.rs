use crate::adc::{ADCWrapper, ADCKind};
use crate::{command, adc::ADCKind};
use spidev::{SpiModeFlags, Spidev, SpidevOptions};
use ads114s06::ADC;

use common::comm::{
  Gpio,
  Pin,
  PinMode::{Input, Output},
  PinValue::{High, Low},
};
use common::comm::ADCKind::{VBatUmbCharge, SamAnd5V};