use crate::comm::{sam::Unit, Measurement};

use pyo3::{pyclass, IntoPy, PyObject, Python};

macro_rules! create_unit {
  ($name:ident, $abbrev:literal) => {
    /// A unit struct representing a continuous physical property.
    #[pyo3::pyclass]
    #[derive(Clone, Copy, Debug, PartialEq, PartialOrd)]
    pub struct $name {
      /// The contained raw value without its unit.
      pub raw: f64,
    }

    #[pyo3::pymethods]
    impl $name {
      /// Constructs a new instance of the unit type using the given raw value.
      #[new]
      pub fn new(raw: f64) -> Self {
        $name { raw }
      }

      fn __add__(&self, other: &Self) -> Self {
        $name {
          raw: self.raw + other.raw,
        }
      }

      fn __sub__(&self, other: &Self) -> Self {
        $name {
          raw: self.raw - other.raw,
        }
      }

      fn __mul__(&self, other: f64) -> Self {
        $name {
          raw: self.raw * other,
        }
      }

      fn __rmul__(&self, other: f64) -> Self {
        self.__mul__(other)
      }

      fn __truediv__(&self, other: f64) -> Self {
        $name {
          raw: self.raw / other,
        }
      }

      fn __iadd__(&mut self, other: &Self) {
        self.raw += other.raw
      }

      fn __isub__(&mut self, other: &Self) {
        self.raw -= other.raw
      }

      fn __imul__(&mut self, other: f64) {
        self.raw *= other
      }

      fn __itruediv__(&mut self, other: f64) {
        self.raw /= other
      }

      fn __richcmp__(&self, other: &Self, op: pyclass::CompareOp) -> bool {
        op.matches(self.raw.total_cmp(&other.raw))
      }

      fn __repr__(&self) -> String {
        format!(concat!("{} ", $abbrev), self.raw)
      }
    }
  };
}

create_unit!(Current, "A");
create_unit!(Duration, "s");
create_unit!(ElectricPotential, "V");
create_unit!(Force, "lbf");
create_unit!(Pressure, "psi");
create_unit!(Temperature, "K");

impl From<Duration> for std::time::Duration {
  fn from(value: Duration) -> Self {
    std::time::Duration::from_secs_f64(value.raw)
  }
}

impl From<std::time::Duration> for Duration {
  fn from(value: std::time::Duration) -> Self {
    Duration {
      raw: value.as_secs_f64(),
    }
  }
}

impl IntoPy<PyObject> for Measurement {
  fn into_py(self, py: Python<'_>) -> PyObject {
    match self.unit {
      Unit::Amps => Current::new(self.value).into_py(py),
      Unit::Kelvin => Temperature::new(self.value).into_py(py),
      Unit::Pounds => Force::new(self.value).into_py(py),
      Unit::Psi => Pressure::new(self.value).into_py(py),
      Unit::Volts => ElectricPotential::new(self.value).into_py(py),
    }
  }
}
