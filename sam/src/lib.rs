// #[cfg(feature = "python")]
// mod python;
#[cfg(any(test, feature = "test_mode"))]

mod mocks;

#[cfg(feature = "python")]
mod python;
// pub mod gpio;
pub mod command;
pub mod pins;
pub mod version;
pub mod communication;
use once_cell::sync::OnceCell;

pub static FC_ADDR: OnceCell<String> = OnceCell::new();

#[cfg(feature = "python")]
use pyo3::prelude::*;

#[cfg(feature = "python")]
#[pymodule]
fn sam(py: Python, m: &PyModule) -> PyResult<()> {
    // Create submodule "sam_command"
    let sam_command_mod = PyModule::new(py, "sam_command")?;
    python::sam_command(py, sam_command_mod)?;
    m.add_submodule(sam_command_mod)?;
    Ok(())
}
