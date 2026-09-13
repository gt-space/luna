use std::time::Duration;

use nalgebra::SMatrix;
pub use nalgebra::{geometry::Quaternion, Vector3};
use rkyv::Archive;
use serde::{Deserialize, Serialize};

/// Full control state of the CTV, used as input to the control loop.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ControlState {
    /// Time since the start of the control loop.
    pub time: Duration,
    /// Position in the local frame.
    pub position: Vector3<f64>,
    /// Velocity in the local frame.
    pub velocity: Vector3<f64>,
    /// Angular rate in the body frame.
    pub body_rate: Vector3<f64>,
    /// Attitude quaternion.
    pub attitude: Quaternion<f64>,
}

impl ControlState {
    /// Converts this state into a 13x1 matrix suitable for the control loop.
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

/// A control vector sent to the CTV to command thrust, TVC, and RCS.
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
    /// Commanded thrust.
    pub thrust: f64,
    /// Commanded TVC pitch angle.
    pub tvc_pitch: f64,
    /// Commanded TVC yaw angle.
    pub tvc_yaw: f64,
    /// Commanded RCS torque.
    pub rcs_torque: f64,
}

impl ControlVector {
    /// Constructs a `ControlVector` from a 4x1 matrix.
    pub fn from_matrix(mat: SMatrix<f64, 4, 1>) -> Self {
        ControlVector {
            thrust: mat[0],
            tvc_pitch: mat[1],
            tvc_yaw: mat[2],
            rcs_torque: mat[3],
        }
    }
}

/// A control message sent to the CTV.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum CtvControlMessage {
    /// Commands the CTV with a control vector.
    Control(ControlVector),
    /// Instructs the CTV to abort.
    Abort,
}
