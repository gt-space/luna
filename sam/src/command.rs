use std::time::Instant;

use common::comm::{
    gpio::{
        PinMode::{Input, Output},
        PinValue,
        PinValue::{High, Low},
    },
    sam::SamControlMessage,
    ValveAction,
};

use crate::{
    abort_timing::{advance_valves, fallback_channels},
    communication::HEARTBEAT_TIME_LIMIT,
    pins::{GPIO_CONTROLLERS, SPI_INFO, VALVE_CURRENT_PINS, VALVE_PINS},
    stage_transition::valid_valves,
    state::AbortInfo,
    SamVersion, SAM_VERSION,
};

pub fn execute(
    command: SamControlMessage,
    abort_info: &mut AbortInfo,
    abort_valve_states: &mut Vec<(ValveAction, bool)>,
) {
    match command {
        SamControlMessage::ActuateValve { channel, powered } => {
            actuate_valve(channel, powered);
        }
        SamControlMessage::AbortStageValveStates { valve_states } => {
            if !abort_info.stage_transition.legacy_sync_allowed() { return; }
            if !valid_valves(&valve_states) { return; }
            abort_info.stage_transition.cancel();
            // clear the abort valve states
            *abort_valve_states = Vec::<(ValveAction, bool)>::new();

            // store the new abort valve states
            store_abort_valve_states(
                &valve_states,
                abort_valve_states,
                &mut abort_info.all_valves_aborted,
                &mut abort_info.received_abort,
            );
        }
        SamControlMessage::AbortStageValveStatesMillis { valve_states } => {
            if !abort_info.stage_transition.legacy_sync_allowed() { return; }
            let valves: Vec<ValveAction> = valve_states.into_iter().map(Into::into).collect();
            if !valid_valves(&valves) { return; }
            abort_info.stage_transition.cancel();
            abort_valve_states.clear();
            store_abort_valve_states(
                &valves,
                abort_valve_states,
                &mut abort_info.all_valves_aborted,
                &mut abort_info.received_abort,
            );
        }
        SamControlMessage::Abort => {
            abort_info.stage_transition.abort();
            abort_info.heartbeat_loss_abort = false;
            abort_info.time_aborted = Some(Instant::now()); // do this before so
                                                            // timer instantly
                                                            // starts, also to
                                                            // prevent reading
                                                            // stale
                                                            // timer
            safe_valves(
                abort_valve_states,
                &abort_info.time_aborted,
                abort_info.heartbeat_loss_abort,
                &mut abort_info.all_valves_aborted,
            );
            abort_info.received_abort = true;
        }
        SamControlMessage::ClearAbortStatus => {
            abort_info.received_abort = false;
            abort_info.all_valves_aborted = false;
            abort_info.time_aborted = None;
            abort_info.heartbeat_loss_abort = false;
            clear_abort_stage_valve_states(abort_valve_states);
        }
        SamControlMessage::ClearStoredAbortStage {} => {
            if !abort_info.stage_transition.clear_legacy() { return; }
            *abort_valve_states = Vec::<(ValveAction, bool)>::new();
        }
        SamControlMessage::ClearStoredAbortStageV2 { revision } => {
            if abort_info.stage_transition.clear(revision) {
                *abort_valve_states = Vec::new();
            }
        }
        SamControlMessage::PrepareAbortStage { revision, valve_states } => {
            if valid_valves(&valve_states) {
                abort_info.stage_transition.prepare(revision, valve_states);
            } else {
                abort_info.stage_transition.reject_prepare(revision);
            }
        }
        SamControlMessage::PrepareAbortStageMillis { revision, valve_states } => {
            let valves: Vec<ValveAction> = valve_states.into_iter().map(Into::into).collect();
            if valid_valves(&valves) {
                abort_info.stage_transition.prepare(revision, valves);
            } else {
                abort_info.stage_transition.reject_prepare(revision);
            }
        }
        SamControlMessage::ActivateAbortStage { revision } => {
            if let Some(valves) = abort_info.stage_transition.activate(revision, abort_info.received_abort) {
                *abort_valve_states = Vec::new();
                store_abort_valve_states(&valves, abort_valve_states, &mut abort_info.all_valves_aborted, &mut abort_info.received_abort);
            }
        }
        SamControlMessage::CameraEnable(should_enable) => {
            toggle_camera_enable(should_enable);
        }
        SamControlMessage::LaunchLugArm(should_enable) => {
            toggle_launch_lug_arm(should_enable);
        }
        SamControlMessage::LaunchLugDetonate(should_enable) => {
            toggle_launch_lug_detonate(should_enable);
        }
    }
}

/// Clears the abort stage valve states by setting all aborted flags for valves
/// to false.
fn clear_abort_stage_valve_states(abort_valve_states: &mut Vec<(ValveAction, bool)>) {
    for (_, aborted) in abort_valve_states {
        *aborted = false;
    }
}

// stores the sent over desired valve states
fn store_abort_valve_states(
    desired_valve_states: &Vec<ValveAction>,
    stored_valve_states: &mut Vec<(ValveAction, bool)>,
    all_valves_aborted: &mut bool,
    received_abort: &mut bool,
) {
    for desired_valve_state in desired_valve_states {
        (*stored_valve_states).push((*desired_valve_state, false));
    }
    *all_valves_aborted = false;
    *received_abort = false;
}

// Calls safe_valves under the hood, exists primarily for naming convention
// logic
pub fn check_valve_abort_timers(
    abort_valve_states: &mut Vec<(ValveAction, bool)>,
    all_valves_aborted: &mut bool,
    time_aborted: &Option<Instant>,
    heartbeat_loss_abort: bool,
) {
    safe_valves(abort_valve_states, time_aborted, heartbeat_loss_abort, all_valves_aborted);
}

// safe the valves by going to safe states (if abort stage is set) or depowering
// valves
pub fn safe_valves(
    abort_valve_states: &mut Vec<(ValveAction, bool)>,
    time_aborted: &Option<Instant>,
    heartbeat_loss_abort: bool,
    all_valves_aborted: &mut bool,
) {
    let mut non_aborted_valve_exists = false;

    // if we have not aborted all valves, valves still need to be safed
    if !*all_valves_aborted {
        // check if an abort stage has been set by seeing if we have any
        // predefined abort valve states
        if !abort_valve_states.is_empty() {
            non_aborted_valve_exists = !advance_valves(
                abort_valve_states,
                || Instant::now().duration_since(time_aborted.unwrap()),
                if heartbeat_loss_abort { HEARTBEAT_TIME_LIMIT } else { std::time::Duration::ZERO },
                |valve| valve.timer,
                |valve| actuate_valve(valve.channel_num, valve.powered),
            );
        } else {
            // we can assume that no abort stage has been set, therefore we just
            // depower all valves
            for i in fallback_channels() {
                actuate_valve(i, false);
            }
        }

        // if no non-aborted valves exist, we have aborted all valves
        *all_valves_aborted = !non_aborted_valve_exists;
    }
}

pub fn init_gpio() {
    // disable all chip selects
    for spi_info in SPI_INFO.values() {
        if let Some(cs_info) = &spi_info.cs {
            let mut cs_pin = GPIO_CONTROLLERS[cs_info.controller].get_pin(cs_info.pin_num);
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
    safe_valves(&mut Vec::new(), &None, false, &mut false);
    // initally measure valve currents on valves 1, 3, and 5 for rev4
    reset_valve_current_sel_pins();
}

pub fn reset_valve_current_sel_pins() {
    // handle the pins that choose which valve the current feedback is from
    if *SAM_VERSION != SamVersion::Rev3 {
        for gpio_info in VALVE_CURRENT_PINS.values() {
            let mut pin = GPIO_CONTROLLERS[gpio_info.controller].get_pin(gpio_info.pin_num);
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
    let mut pin = GPIO_CONTROLLERS[gpio_info.controller].get_pin(gpio_info.pin_num);
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

/// Reads the RBF tag from the SAM. Only supported on flight SAMs.
pub fn read_rbf() -> Option<PinValue> {
    match *SAM_VERSION {
        SamVersion::Rev4Flight | SamVersion::Rev4FlightV2 => {
            let mut pin = GPIO_CONTROLLERS[0].get_pin(31); // GPIO_31, P9.
            pin.mode(Input);
            Some(pin.digital_read())
        }
        _ => None,
    }
}

#[cfg(test)]
mod abort_stage_command_tests {
    use super::*;
    use common::comm::sam::ValveActionMillis;
    use crate::stage_transition::{AckPhase, StageTransition};

    #[test]
    fn malformed_prepare_rejects_and_valid_prepare_preserves_milliseconds() {
        let mut info = AbortInfo {
            received_abort: false,
            all_valves_aborted: false,
            last_heard_from_fc: Instant::now(),
            time_aborted: None,
            heartbeat_loss_abort: false,
            stage_transition: StageTransition::default(),
        };
        let mut valves = Vec::new();
        execute(SamControlMessage::PrepareAbortStageMillis {
            revision: 1,
            valve_states: vec![ValveActionMillis { channel_num: 7, powered: true, timer_ms: 200 }],
        }, &mut info, &mut valves);
        assert_eq!(info.stage_transition.acknowledgment, Some((1, AckPhase::Rejected)));
        execute(SamControlMessage::ActivateAbortStage { revision: 1 }, &mut info, &mut valves);
        assert!(valves.is_empty());

        execute(SamControlMessage::PrepareAbortStageMillis {
            revision: 2,
            valve_states: vec![ValveActionMillis { channel_num: 1, powered: true, timer_ms: 200 }],
        }, &mut info, &mut valves);
        assert_eq!(info.stage_transition.acknowledgment, Some((2, AckPhase::Prepared)));
        execute(SamControlMessage::ActivateAbortStage { revision: 2 }, &mut info, &mut valves);
        assert_eq!(info.stage_transition.acknowledgment, Some((2, AckPhase::Activated)));
        assert_eq!(valves[0].0.timer, std::time::Duration::from_millis(200));
    }
}
