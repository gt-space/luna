use std::collections::HashMap;
use crate::command;
use spidev::{SpiModeFlags, Spidev, SpidevOptions};
use ads114s06::ADC;
use common::comm::ADCKind;

use common::comm::{
  Gpio,
  Pin,
  PinMode::{Input, Output},
  PinValue::{High, Low},
};
use common::comm::ADCKind::{VBatUmbCharge, SamAnd5V};

impl State {
  pub fn next(self, data: &mut Data, gpio_controllers: &[Gpio]) -> State {
    match self {
      State::InitGpio => {
        init_gpio(gpio_controllers);
        let cs_mappings = get_cs_mappings(gpio_controllers);
        let drdy_mappings = get_drdy_mappings(gpio_controllers);
        let spi0 = create_spi("/dev/spidev0.0").unwrap();

        let adc1: ADC = ADC::new(
          spi0,
          drdy_mappings.get(&ADCKind::VBatUmbCharge).unwrap(),
          cs_mappings.get(&ADCKind::VBatUmbCharge).unwrap(),
          VBatUmbCharge
        );

        let adc2: ADC = ADC::new(
          spi0,
          drdy_mappings.get(&ADC::SamAnd5V).unwrap(),
          cs_mappings.get(&ADCKind::SamAnd5V).unwrap(),
          SamAnd5V
        );

        let adcs = vec![adc1, adc2];

        State::EstablishFlightComputerConnection
      },

      State::EstablishFlightComputerConnection => {
        State::InitAdcs
      },

      State::InitAdcs => {
        State::PollAdcs
      },

      State::PollAdcs => {

        data.data_points.clear();
        for i in 0..6 {
          for adc in data.adcs {
            let reached_max_vbat_umb_charge = 
            adc.cs_pin.digital_write(Low); // active Low
            adc.cs_pin.digital_write(High); // active Low
          }
        }

        State::PollAdcs
      }
    }
  }
}