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

/// Estimated CTV state for full-rate DAQ.
#[derive(
    Debug, Clone, PartialEq, Serialize, Deserialize, Archive, rkyv::Serialize, rkyv::Deserialize,
)]
#[archive_attr(derive(bytecheck::CheckBytes))]
pub struct CtvEstimate {
    /// Position [x, y, z] in meters.
    pub position_m: [f64; 3],
    /// Velocity [x, y, z] in meters per second.
    pub velocity_m_s: [f64; 3],
    /// Body rates [x, y, z] in radians per second.
    pub body_rates_rad_s: [f64; 3],
    /// Attitude quaternion [w, x, y, z].
    pub attitude_wxyz: [f64; 4],
    /// Covariance in the 13-state order of ControlState::to_matrix, when available.
    pub covariance: Option<[[f64; 13]; 13]>,
}

impl CtvEstimate {
    /// Copies a control state into the DAQ shape without inventing covariance.
    pub fn from_control_state(state: &ControlState) -> Self {
        Self {
            position_m: [state.position.x, state.position.y, state.position.z],
            velocity_m_s: [state.velocity.x, state.velocity.y, state.velocity.z],
            body_rates_rad_s: [state.body_rate.x, state.body_rate.y, state.body_rate.z],
            attitude_wxyz: [
                state.attitude.w,
                state.attitude.i,
                state.attitude.j,
                state.attitude.k,
            ],
            covariance: None,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CtvVoltageChannel {
    /// TVC actuator voltage.
    Tvc(usize),
    /// RCS fan voltage.
    RcsFan(usize),
    /// Main propeller voltage.
    MainPropeller(usize),
}

/// CTV DAQ fields that are not already present in BMS, FC sensors, GPS, or RECO.
#[derive(
    Debug,
    Clone,
    Default,
    PartialEq,
    Serialize,
    Deserialize,
    Archive,
    rkyv::Serialize,
    rkyv::Deserialize,
)]
#[archive_attr(derive(bytecheck::CheckBytes))]
pub struct CtvTelemetry {
    /// Measured TVC actuator voltages, channels 1 and 2, in volts.
    pub tvc_voltages_v: [Option<f64>; 2],
    /// Measured RCS fan voltages, channels 1 through 4, in volts.
    pub rcs_fan_voltages_v: [Option<f64>; 4],
    /// Measured main propeller voltages, channels 1 and 2, in volts.
    pub main_propeller_voltages_v: [Option<f64>; 2],
    /// LIDAR distances, sensors 1 through 3, in meters.
    pub lidar_distances_m: [Option<f64>; 3],
    /// Altitude calculated from barometric pressure, in meters.
    pub barometric_altitude_m: Option<f64>,
    /// Heading calculated from magnetometer data, in radians.
    pub magnetic_heading_rad: Option<f64>,
    /// Most recent state-estimator output.
    pub estimate: Option<CtvEstimate>,
    /// Commanded throttle fraction, from 0 to 1.
    pub commanded_throttle: Option<f64>,
    /// Commanded RCS fan speed setpoints, channels 1 through 4. Units depend on the fan interface.
    pub commanded_rcs_speeds: [Option<f64>; 4],
    /// Commanded TVC angles, actuators 1 and 2, in radians.
    pub commanded_tvc_angles_rad: [Option<f64>; 2],
    /// Seconds elapsed since CtvStartFlight.
    pub mission_elapsed_s: Option<f64>,
    /// Intentionally injected faults; None means the status is unavailable.
    pub injected_faults: Option<Vec<String>>,
    /// Active fault configuration identifier, if known.
    pub fault_config: Option<String>,
    /// Active mission configuration identifier, if known.
    pub mission_config: Option<String>,
}

impl CtvTelemetry {
    /// Updates one measured voltage. Returns false for an invalid channel index.
    pub fn record_voltage(&mut self, channel: CtvVoltageChannel, voltage_v: f64) -> bool {
        let slot = match channel {
            CtvVoltageChannel::Tvc(index) => self.tvc_voltages_v.get_mut(index),
            CtvVoltageChannel::RcsFan(index) => self.rcs_fan_voltages_v.get_mut(index),
            CtvVoltageChannel::MainPropeller(index) => {
                self.main_propeller_voltages_v.get_mut(index)
            }
        };
        if let Some(slot) = slot {
            *slot = Some(voltage_v);
            true
        } else {
            false
        }
    }

    /// Updates one LIDAR distance.
    pub fn record_lidar_distance(&mut self, index: usize, distance_m: f64) -> bool {
        if let Some(slot) = self.lidar_distances_m.get_mut(index) {
            *slot = Some(distance_m);
            true
        } else {
            false
        }
    }
}

#[cfg(test)]
mod tests {
    use super::{CtvTelemetry, CtvVoltageChannel};
    use crate::comm::VehicleState;

    #[test]
    fn ctv_readings_round_trip_on_full_state_but_not_radio() {
        let mut state = VehicleState::default();
        assert!(state.ctv.record_voltage(CtvVoltageChannel::RcsFan(3), 18.5));
        assert!(state.ctv.record_lidar_distance(2, 1.75));
        assert!(!state.ctv.record_voltage(CtvVoltageChannel::RcsFan(4), 9.0));
        assert!(!state.ctv.record_lidar_distance(3, 9.0));
        state.ctv.mission_elapsed_s = Some(2.5);
        state.bms_valid = true;

        let full = postcard::to_allocvec(&state).unwrap();
        let decoded: VehicleState = postcard::from_bytes(&full).unwrap();
        assert_eq!(decoded.ctv, state.ctv);
        assert!(decoded.bms_valid);

        let mut ctv_radio = [0u8; 512];
        let ctv_size = state.compress_compaq(&mut ctv_radio).unwrap();
        let mut baseline_radio = [0u8; 512];
        let baseline_size = VehicleState::default()
            .compress_compaq(&mut baseline_radio)
            .unwrap();
        assert_eq!(&ctv_radio[..ctv_size], &baseline_radio[..baseline_size]);
        assert_eq!(CtvTelemetry::default().lidar_distances_m, [None; 3]);
    }
}
