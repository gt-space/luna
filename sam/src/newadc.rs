use std::time::{Instant, Duration};
use common::comm::{bms::{Bms, DataPoint}, gpio::PinValue::Low, ADCKind::{Sam, SamRev3, SamRev4}, SamADC, SamRev3ADC, SamRev4ADC};
use ads114s06::ADC;

pub fn init_adcs(adcs: &mut Vec<ADC>) {
  for (i, adc) in adcs.iter_mut().enumerate() {

  }
}

pub fn poll_adcs(adcs: &mut Vec<ADC>) {
  
}