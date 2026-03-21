use common::comm::ctv::{ControlState, ControlVector};
use nalgebra::{Const, Dyn, Matrix, MatrixXx1, MatrixXx4, VecStorage};
use serde::{Deserialize, Serialize};

use super::Controller;

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct LqrController {
  n: usize,
  tgrid: MatrixXx1<f64>,
  xref: Matrix<f64, Dyn, Const<13>, VecStorage<f64, Dyn, Const<13>>>,
  k1flat: Matrix<f64, Dyn, Const<52>, VecStorage<f64, Dyn, Const<52>>>,
  k2grid: MatrixXx4<f64>,
  unom: MatrixXx4<f64>,
}

impl LqrController {
  pub fn new(
    tgrid: MatrixXx1<f64>,
    xref: Matrix<f64, Dyn, Const<13>, VecStorage<f64, Dyn, Const<13>>>,
    k1flat: Matrix<f64, Dyn, Const<52>, VecStorage<f64, Dyn, Const<52>>>,
    k2grid: MatrixXx4<f64>,
    unom: MatrixXx4<f64>,
  ) -> Self {
    let n = tgrid.nrows();
    assert!(n == xref.nrows(), "inconsistent row dimension");
    assert!(n == k1flat.nrows(), "inconsistent row dimension");
    assert!(n == k2grid.nrows(), "inconsistent row dimension");
    assert!(n == unom.nrows(), "inconsistent row dimension");

    LqrController {
      n,
      tgrid,
      xref,
      k1flat,
      k2grid,
      unom,
    }
  }
}

impl Controller for LqrController {
  fn step(&mut self, state: ControlState) -> ControlVector {
    let x = state.to_matrix();

    let t = state.time.as_secs_f64();

    // Clamp time and compute interval index (0-based)
    let (idx, w) = if t <= self.tgrid[0] {
      (0, 0.0)
    } else if t >= self.tgrid[self.n - 1] {
      (self.n - 2, 1.0)
    } else {
      let mut idx = 0;
      for k in 0..(self.n - 1) {
        if self.tgrid[k] <= t && t < self.tgrid[k + 1] {
          idx = k;
          break;
        }
      }
      let (t1, t2) = (self.tgrid[idx], self.tgrid[idx + 1]);
      let w = (t - t1) / (t2 - t1);
      (idx, w)
    };

    // Interpolate reference state (13x1)
    let xref_t = (1.0 - w) * self.xref.row(idx).transpose()
      + w * self.xref.row(idx + 1).transpose();
    // Interpolate K1flat then reshape to 4x13
    // MATLAB: reshape(Kflat, 13, 4)' → col-major fill into 13x4, then transpose to 4x13
    let kflat = (1.0 - w) * self.k1flat.row(idx) + w * self.k1flat.row(idx + 1);
    let k1 = kflat.reshape_generic(Const::<13>, Const::<4>).transpose();
    // Interpolate K2 (4x1)
    let k2 = (1.0 - w) * self.k2grid.row(idx).transpose()
      + w * self.k2grid.row(idx + 1).transpose();
    // Interpolate nominal control (4x1)
    let unom = (1.0 - w) * self.unom.row(idx).transpose()
      + w * self.unom.row(idx + 1).transpose();

    // Quaternion sign convention: flip q if dot(q, qref) < 0
    let mut q = x.rows_range(9..13).clone_owned();
    let qref = xref_t.rows_range(9..13);
    if q.dot(&qref) < 0.0 {
      q *= -1.0;
    }

    let mut x_used = x.clone_owned();
    x_used.rows_range_mut(9..13).copy_from(&q);

    // Control law: u = u_nom - K1 * dx + K2
    let dx = x_used - xref_t;
    let u = unom - k1 * dx + k2;

    ControlVector::from_matrix(u)
  }
}
