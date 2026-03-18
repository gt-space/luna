use std::time::Duration;

use nalgebra::SMatrix;
pub use nalgebra::{geometry::Quaternion, Vector3};
use rkyv::Archive;
use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ControlState {
  pub time: Duration,
  pub position: Vector3<f64>,
  pub velocity: Vector3<f64>,
  pub body_rate: Vector3<f64>,
  pub attitude: Quaternion<f64>,
}

impl ControlState {
  pub fn to_matrix(&self) -> SMatrix<f64, 13, 1> {
    SMatrix::from_column_slice(&[
      self.position.x,
      self.position.y,
      self.position.z,
      self.velocity.x,
      self.velocity.y,
      self.velocity.z,
      self.body_rate.x,
      self.body_rate.y,
      self.body_rate.z,
      // TODO: check this is the right order
      self.attitude.w,
      self.attitude.i,
      self.attitude.j,
      self.attitude.k,
    ])
  }
}

#[derive(
  Debug,
  Clone,
  Copy,
  PartialEq,
  Serialize,
  Deserialize,
  Archive,
  rkyv::Serialize,
  rkyv::Deserialize,
)]
pub struct ControlVector {
  pub thrust: f64,
  pub tvc_pitch: f64,
  pub tvc_yaw: f64,
  pub rcs_torque: f64,
}

impl ControlVector {
  pub fn from_matrix(mat: SMatrix<f64, 4, 1>) -> Self {
    ControlVector {
      thrust: mat[0],
      tvc_pitch: mat[1],
      tvc_yaw: mat[2],
      rcs_torque: mat[3],
    }
  }
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum CtvControlMessage {
  Control(ControlVector),
  Abort,
}
