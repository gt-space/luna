use crate::fusion::FusionState;
use crate::sensors;

/// Print the fused state nicely.
pub fn print_dashboard(state: &FusionState) {
    println!("==============================");
    println!("       🚀 Rocket Dashboard     ");
    println!("==============================");
    sensors::print_reading(&state.engine_temp);
    sensors::print_reading(&state.lox_temp);
    sensors::print_reading(&state.fuel_temp);
    sensors::print_reading(&state.lox_pressure);
    sensors::print_reading(&state.fuel_pressure);
    sensors::print_reading(&state.pneumatics);
    sensors::print_valve_state(&state.valve);

    if state.check_safety() {
        println!("✅ All systems nominal.");
    } else {
        println!("⚠️ Warning: Unsafe conditions!");
    }

    println!("==============================\n");
}
