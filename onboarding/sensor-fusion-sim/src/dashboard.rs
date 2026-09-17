// src/dashboard.rs
use crate::fusion::FusionState;
use crate::sensors;

/// Print the fused state nicely.
pub fn print_dashboard(state: &FusionState) {
    let blue = "\x1b[34m";
    let green = "\x1b[32m";
    let yellow = "\x1b[33m";
    let reset = "\x1b[0m";

    println!("==============================");
    println!("{}       Rocket Dashboard       {}", blue, reset);
    println!("==============================");
    sensors::print_reading(&state.engine_temp);
    sensors::print_reading(&state.lox_temp);
    sensors::print_reading(&state.fuel_temp);
    sensors::print_reading(&state.lox_pressure);
    sensors::print_reading(&state.fuel_pressure);
    sensors::print_reading(&state.pneumatics);
    sensors::print_valve_state(&state.valve);

    if state.check_safety() {
        println!("{}[+]{} All systems nominal.", green, reset);
    } else {
        println!("{}[!]{} Warning: Unsafe conditions!", yellow, reset);
    }

    println!("==============================\n");
}