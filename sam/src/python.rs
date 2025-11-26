//A File for configuring/adding the python wrappers for the unit tests
use pyo3::prelude::*;
use pyo3::wrap_pyfunction;
// use crate::command::get_gpio_controllers_len;
// use crate::command::get_gpio_controller;
// use common::comm::gpio::{Gpio, Pin, PinMode, PinValue};
use crate::command::get_num_valves;
use crate::command::get_valve_state;
use crate::command;
#[cfg(feature = "test_mode")]
#[pymodule]
pub fn sam_command(_py: Python, m: &PyModule) -> PyResult<()> {
    m.add_function(wrap_pyfunction!(execute_valve_py, m)?)?;
    m.add_function(wrap_pyfunction!(safe_valves_py, m)?)?;
    m.add_function(wrap_pyfunction!(init_gpio_py, m)?)?;
    m.add_function(wrap_pyfunction!(reset_valve_current_sel_pins_py, m)?)?;
    // m.add_class::<PyGpio>()?;
    // m.add_function(wrap_pyfunction!(get_gpio_controller, m)?)?;
    m.add_function(wrap_pyfunction!(get_valve_state, m)?)?;
    m.add_function(wrap_pyfunction!(get_num_valves, m)?)?;
    Ok(())
}
#[cfg(feature = "test_mode")]
#[pyfunction]
fn execute_valve_py(channel: u32, powered: bool) {
    let command = common::comm::sam::SamControlMessage::ActuateValve {
        channel,
        powered,
    };
    command::execute(command);
}
#[cfg(feature = "test_mode")]
#[pyfunction]
fn safe_valves_py() {
    command::safe_valves();
}
#[cfg(feature = "test_mode")]
#[pyfunction]
fn init_gpio_py() {
    command::init_gpio();
}
#[cfg(feature = "test_mode")]
#[pyfunction]
fn reset_valve_current_sel_pins_py() {
    command::reset_valve_current_sel_pins();
}

// use pyo3::prelude::*;
// use crate::command;

// /// Python-exposed submodule for SAM commands
// #[pymodule]
// pub fn sam_command(_py: Python, m: &PyModule) -> PyResult<()> {
//     // Each function defined here gets exported to Python
//     #[pyfn(m)]
//     fn execute_valve(_py: Python, channel: u32, powered: bool) {
//         let command = common::comm::sam::SamControlMessage::ActuateValve { channel, powered };
//         command::execute(command);
//     }

//     #[pyfn(m)]
//     fn safe_valves() {
//         command::safe_valves();
//     }

//     #[pyfn(m)]
//     fn init_gpio() {
//         command::init_gpio();
//     }

//     #[pyfn(m)]
//     fn reset_valve_current_sel_pins() {
//         command::reset_valve_current_sel_pins();
//     }

//     Ok(())
// }
