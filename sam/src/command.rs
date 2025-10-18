use common::comm::{
  gpio::{
    PinMode::Output,
    PinValue::{High, Low},
  },
  sam::SamControlMessage,
};

use crate::{pins::{GPIO_CONTROLLERS, SPI_INFO, VALVE_CURRENT_PINS, VALVE_PINS}, state::ConnectData};
use crate::{SamVersion, SAM_VERSION};
use std::time::{Duration, Instant};

pub fn execute(command: SamControlMessage, prvnt_channel: &mut u32) {
  match command {
    SamControlMessage::ActuateValve { channel, powered } => {
      actuate_valve(channel, powered);
    },
    SamControlMessage::SafeValves {  } => {
      safe_valves(0); // pass in 0 since we don't do any special prvnt related thing here 
    },
    SamControlMessage::PRVNTSafing { channel } => {
      *prvnt_channel = channel;
    },
  }
}

pub fn check_prvnt_abort(data: &mut ConnectData) {
  if !data.abort_info.opened_prvnt && Instant::now().duration_since(data.abort_info.last_heard_from_fc) > Duration::from_secs(60 * 10) {
    actuate_valve(data.abort_info.prvnt_channel, false);
    data.abort_info.opened_prvnt = true;
  }
}

pub fn safe_valves(prvnt_channel: u32) {
  for i in 1..7 {
    // ensure we aren't actuating the prvnt channel (if a board doesn't have prvnt on it, prvnt_channel will be 0 so it won't be checked here)
    if i != prvnt_channel {
      actuate_valve(i, false); // turn off all valves
    }
  }
}

pub fn init_gpio() {
  // disable all chip selects
  for spi_info in SPI_INFO.values() {
    if let Some(cs_info) = &spi_info.cs {
      let mut cs_pin =
        GPIO_CONTROLLERS[cs_info.controller].get_pin(cs_info.pin_num);
      cs_pin.mode(Output);
      // chip select is active low so make it high to disable
      cs_pin.digital_write(High);
    }
  }

  // handles CS for cold junction IC on rev3 (not an ADC)
  if *SAM_VERSION == SamVersion::Rev3 {
    let mut cs_tc_cjc1 = GPIO_CONTROLLERS[2].get_pin(23);
    cs_tc_cjc1.mode(Output);
    cs_tc_cjc1.digital_write(High); // chip select is active low

    let mut cs_tc_cjc2 = GPIO_CONTROLLERS[0].get_pin(7);
    cs_tc_cjc2.mode(Output);
    cs_tc_cjc2.digital_write(High); // chip select is active low
  }

  // turn off all valves
  safe_valves(0);
  // initally measure valve currents on valves 1, 3, and 5 for rev4
  reset_valve_current_sel_pins();
}

pub fn reset_valve_current_sel_pins() {
  // handle the pins that choose which valve the current feedback is from
  if *SAM_VERSION != SamVersion::Rev3 {
    for gpio_info in VALVE_CURRENT_PINS.values() {
      let mut pin =
        GPIO_CONTROLLERS[gpio_info.controller].get_pin(gpio_info.pin_num);
      pin.mode(Output); // like so incredibly redundant
      pin.digital_write(Low); // start on valves 1, 3, 5
    }
  }
}

fn actuate_valve(channel: u32, powered: bool) {
  if !(1..=6).contains(&channel) {
    panic!("Invalid valve channel number")
  }

  let gpio_info = VALVE_PINS.get(&channel).unwrap();
  let mut pin =
    GPIO_CONTROLLERS[gpio_info.controller].get_pin(gpio_info.pin_num);
  pin.mode(Output);

  match powered {
    true => {
      println!("Powering Valve {}", channel);
      pin.digital_write(High);
    }

    false => {
      println!("Unpowering Valve {}", channel);
      pin.digital_write(Low);
    }
  }
}
