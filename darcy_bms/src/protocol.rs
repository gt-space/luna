use std::collections::HashMap;
use crate::command;
use common::comm::ADCKind;

use common::comm::{
  Gpio,
  Pin,
  PinMode::{Input, Output},
  PinValue::High,
};

pub fn init_gpio(gpio_controllers: &[Gpio]) {
  // set battery enable low
  // set reco enables low
  command::disable_battery_power(gpio_controllers);
  command::reco_disable(1, gpio_controllers);
  command::reco_disable(2, gpio_controllers);
  command::reco_disable(3, gpio_controllers);
  command::reco_disable(4, gpio_controllers);

  for chip_select_pin in get_cs_mappings(gpio_controllers).values_mut() {
    chip_select_pin.digital_write(High); // active low
  }
}

pub fn get_cs_mappings(gpio_controllers: &[Gpio]) -> HashMap<ADCKind, Pin> {
  let mut vbat_umb_charge_chip_select: Pin = gpio_controllers[0].get_pin(30);
  vbat_umb_charge_chip_select.mode(Output);
  let mut sam_and_5v_chip_select: Pin = gpio_controllers[0].get_pin(31);
  sam_and_5v_chip_select.mode(Output);

  HashMap::from([(ADCKind::VBatUmbCharge, vbat_umb_charge_chip_select),
  (ADCKind::SamAnd5V, sam_and_5v_chip_select)])
}

// pub fn get_drdy_mappings(gpio_controllers: &[Gpio]) -> HashMap<ADCKind, Pin> {
//   let mut vbat_umb_charge_drdy: Pin = gpio_controllers[1].get_pin(28);
//   vbat_umb_charge_drdy.mode(Input);
//   let mut sam_and_5v_drdy: Pin = gpio_controllers[1].get_pin(18);
//   sam_and_5v_drdy.mode(Input);

//   HashMap::from([(ADCKind::VBatUmbCharge, vbat_umb_charge_drdy), 
//   (ADCKind::SamAnd5V, sam_and_5v_drdy)])
// }