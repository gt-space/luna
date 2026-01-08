use common::comm::{
  gpio::{
    PinMode::Output,
    PinValue::{High, Low},
  },
  sam::SamControlMessage,
  ValveAction,
};

use crate::{pins::{GPIO_CONTROLLERS, SPI_INFO, VALVE_CURRENT_PINS, VALVE_PINS}, state::{AbortInfo}};
use crate::{SamVersion, SAM_VERSION};
use std::{time::{Instant}};
use crate::{communication::HEARTBEAT_TIME_LIMIT};

pub fn execute(command: SamControlMessage, abort_info: &mut AbortInfo, abort_valve_states: &mut Vec<(ValveAction, bool)>) {
  match command {
    SamControlMessage::ActuateValve { channel, powered } => {
      actuate_valve(channel, powered);
    },
    SamControlMessage::AbortStageValveStates { valve_states } => {
      store_abort_valve_states(&valve_states, abort_valve_states, &mut abort_info.all_valves_aborted, &mut abort_info.received_abort);
    },
    SamControlMessage::Abort { use_stage_timers } => {
      abort_info.time_aborted = Some(Instant::now()); // do this before so timer instantly starts, also to prevent reading stale timer
      safe_valves(abort_valve_states, &abort_info.time_aborted, &mut abort_info.all_valves_aborted, use_stage_timers);
      abort_info.received_abort = true; 
    },
    SamControlMessage::ClearStoredAbortStage {  } => {
      *abort_valve_states = Vec::<(ValveAction, bool)>::new();
    }
    SamControlMessage::CameraEnable(should_enable) => {
      toggle_camera_enable(should_enable);
    },
    SamControlMessage::LaunchLugArm(should_enable) => {
      toggle_launch_lug_arm(should_enable);
    },
    SamControlMessage::LaunchLugDetonate(should_enable) => {
      toggle_launch_lug_detonate(should_enable);
    },
  }
}

// stores the sent over desired valve states
fn store_abort_valve_states(desired_valve_states: &Vec<ValveAction>, stored_valve_states: &mut Vec<(ValveAction, bool)>, all_valves_aborted: &mut bool, received_abort: &mut bool) {
  for desired_valve_state in desired_valve_states {
    (*stored_valve_states).push((*desired_valve_state, false));
  }
  *all_valves_aborted = false;
  *received_abort = false;
}

// Calls safe_valves under the hood, exists primarily for naming convention logic 
pub fn check_valve_abort_timers(abort_valve_states: &mut Vec<(ValveAction, bool)>, all_valves_aborted: &mut bool, time_aborted: &Option<Instant>) {
  safe_valves(abort_valve_states, time_aborted, all_valves_aborted, true);
}

// safe the valves by going to safe states (if abort stage is set) or depowering valves
pub fn safe_valves(abort_valve_states: &mut Vec<(ValveAction, bool)>, time_aborted: &Option<Instant>, all_valves_aborted: &mut bool, use_stage_timers: bool) {
  let mut non_aborted_valve_exists = false;
  // check if an abort stage has been set (indirectly) by seeing if we have predefined abort valve states
  if use_stage_timers && !*all_valves_aborted {
    for (valve_info, aborted) in abort_valve_states {

      // abort the valve if we want an instant abort OR if our timer is up and we haven't aborted yet
      if !use_stage_timers || (!*aborted && (Instant::now().duration_since(time_aborted.unwrap()) + HEARTBEAT_TIME_LIMIT) > valve_info.timer)  {
        actuate_valve(valve_info.channel_num, valve_info.powered);

        // mark this valve as aborted 
        *aborted = true;
      }

      if !*aborted {
        non_aborted_valve_exists = true;
      } 
    }
  } else { 
    // we can assume that no abort stage has been set, therefore we just depower. 
    // also enter this case in a servo to fc (mc to pad) disconnect
    for i in 1..7 {
      actuate_valve(i, false); // turn off all valves
    }
  }

  *all_valves_aborted = !non_aborted_valve_exists;
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
  safe_valves(&mut Vec::new(), &None, &mut false, false);
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

fn toggle_camera_enable(should_enable: bool) {
  // Same pin number for SAMRev4Flight and SAMRev4FlightV2
  let mut pin = GPIO_CONTROLLERS[0].get_pin(5); // GPIO_5, P9. 
  pin.mode(Output);
  pin.digital_write(if should_enable { High } else { Low });
}

fn toggle_launch_lug_arm(should_enable: bool) {
  let mut pin = GPIO_CONTROLLERS[1].get_pin(30); // GPIO_62, P8. for og fsam
  
  // fsams rev4 v2 have different pin numbers
  if *SAM_VERSION == SamVersion::Rev4FlightV2 {
    pin = GPIO_CONTROLLERS[2].get_pin(4); // GPIO_68, P8. for rev4 flight v2
  }
  pin.mode(Output);
  pin.digital_write(if should_enable { High } else { Low });
}

fn toggle_launch_lug_detonate(should_enable: bool) {
  let mut pin = GPIO_CONTROLLERS[0].get_pin(22); // GPIO_22, P8. for og fsam

  // fsams rev4 v2 have different pin numbers
  if *SAM_VERSION == SamVersion::Rev4FlightV2 {
    pin = GPIO_CONTROLLERS[2].get_pin(5); // GPIO_69, P8. for rev4 flight v2
  }
  pin.mode(Output);
  pin.digital_write(if should_enable { High } else { Low });
}