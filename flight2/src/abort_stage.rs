//! CTV fault detection and latched abort control policy.
//!
//! This module is deliberately independent of the flight-computer transport:
//! callers provide current fault observations and forward the returned control
//! vector to the CTV controller.

use common::comm::ctv::ControlVector;

/// A fault that requires the CTV to enter its abort state.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CTVAbortFault {
    /// Flight control communications have exceeded their allowed timeout.
    CommunicationsLost,
    /// Battery voltage fell below the configured minimum.
    BatteryUndervoltage,
    /// A required power rail or power-good signal failed.
    PowerUnavailable,
}

/// Current inputs used by [`CTVAbortStage`] to detect a vehicle fault.
#[derive(Debug, Clone, Copy)]
pub struct CTVAbortInputs {
    /// Whether the flight-control link is currently healthy.
    pub communications_healthy: bool,
    /// Measured battery voltage, when available.
    pub battery_voltage: Option<f64>,
    /// Minimum acceptable battery voltage.
    pub minimum_battery_voltage: f64,
    /// Whether required power rails are available.
    pub power_available: bool,
}

/// Latched CTV abort state and the commands used to make the actuators safe.
///
/// On abort, this policy cuts engine thrust, commands neutral TVC, and removes
/// RCS torque. Upright recovery requires an attitude controller and actuator
/// feedback; those interfaces are not present in the flight2 CTV command path
/// yet, so the safe vector does not claim to actively reorient the vehicle.
#[derive(Debug, Clone)]
pub struct CTVAbortStage {
    fault: Option<CTVAbortFault>,
}

impl Default for CTVAbortStage {
    fn default() -> Self {
        Self::new()
    }
}

impl CTVAbortStage {
    /// Creates an inactive abort stage.
    pub const fn new() -> Self {
        Self { fault: None }
    }

    /// Evaluates fault inputs and latches the first detected fault.
    pub fn update(&mut self, inputs: CTVAbortInputs) -> Option<CTVAbortFault> {
        if self.fault.is_none() {
            self.fault = if !inputs.communications_healthy {
                Some(CTVAbortFault::CommunicationsLost)
            } else if !inputs.power_available {
                Some(CTVAbortFault::PowerUnavailable)
            } else if inputs
                .battery_voltage
                .is_some_and(|voltage| voltage < inputs.minimum_battery_voltage)
            {
                Some(CTVAbortFault::BatteryUndervoltage)
            } else {
                None
            };
        }
        self.fault
    }

    /// Returns the latched abort cause, if any.
    pub const fn fault(&self) -> Option<CTVAbortFault> {
        self.fault
    }

    /// Whether this stage has entered abort.
    pub const fn is_aborted(&self) -> bool {
        self.fault.is_some()
    }

    /// Safe actuator vector: engine off, TVC neutral, and RCS torque off.
    pub const fn safe_control_vector(&self) -> Option<ControlVector> {
        if self.is_aborted() {
            Some(ControlVector {
                thrust: 0.0,
                tvc_pitch: 0.0,
                tvc_yaw: 0.0,
                rcs_torque: 0.0,
            })
        } else {
            None
        }
    }

    /// Clears the latched fault after an explicit operator/system reset.
    pub fn clear(&mut self) {
        self.fault = None;
    }
}
