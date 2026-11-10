//! Turns a `ControlVector` (thrust, gimbal angles, roll torque) into the
//! actuator demands the engine controller takes.
//!
//! PLACEHOLDER: nothing here is calibrated. The thrust-to-throttle and
//! torque-to-duty maps need the ESC and RCS fan characterization, the TVC
//! geometry needs the CAD values, and which actuator is A1 needs the wiring.
//! This exists so `CtvControl` commands reach the EC in the right shape; don't
//! fly on these numbers.

use common::comm::ctv::{ActuatorDemand, ControlVector};

use crate::tvc_actuator::{AxisGeometry, TvcActuators};

/// Converts control vectors into actuator demands.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) struct Allocation {
    /// Gimbal geometry. Pitch drives actuator A1, yaw drives A2.
    tvc: TvcActuators,
    /// Thrust at full throttle, in N. Throttle is taken as linear in thrust.
    full_throttle_thrust_n: f64,
    /// Roll torque from one fan pair at full duty, in N·m.
    full_duty_roll_torque_nm: f64,
}

/// Made-up geometry with a 26.9 mm stroke centered on the gimbal's zero angle.
const PLACEHOLDER_AXIS: AxisGeometry = AxisGeometry {
    r: 0.05,
    d: 0.12,
    centered_length: 0.13,
    retracted_length: 0.1165,
    stroke: 0.0269,
    inverted: false,
};

impl Allocation {
    /// Placeholder numbers until the characterization and CAD values exist.
    pub(crate) const PLACEHOLDER: Allocation = Allocation {
        tvc: TvcActuators {
            pitch: PLACEHOLDER_AXIS,
            yaw: PLACEHOLDER_AXIS,
        },
        full_throttle_thrust_n: 100.0,
        full_duty_roll_torque_nm: 0.1,
    };

    /// The hopper sim's CTV numbers (`vehicles/ctv/ctv_config.m`), so the
    /// sim-plant bench test commands the EC in the same scale the plant
    /// flies: 7.875 lbf EDF static thrust at full throttle, and the sim's
    /// roll authority (16 N placeholder prop thrust on a 4.25 in arm). Both
    /// are still ASSUMPTIONs there, and the TVC geometry is the placeholder.
    #[cfg_attr(not(feature = "sim-plant"), allow(dead_code))]
    pub(crate) const CTV_SIM: Allocation = Allocation {
        tvc: TvcActuators {
            pitch: PLACEHOLDER_AXIS,
            yaw: PLACEHOLDER_AXIS,
        },
        full_throttle_thrust_n: 7.875 * 4.448_22,
        full_duty_roll_torque_nm: 16.0 * 4.25 * 0.0254,
    };

    /// Outputs at rest: zero throttle (the ESC's minimum), gimbal centered,
    /// fans off.
    #[cfg_attr(not(feature = "sim-plant"), allow(dead_code))]
    pub(crate) fn safe(&self) -> ActuatorDemand {
        self.demand(&ControlVector {
            thrust: 0.0,
            tvc_pitch: 0.0,
            tvc_yaw: 0.0,
            rcs_torque: 0.0,
        })
    }

    /// The gimbal actuators' extension at zero angle and their stroke, mm
    #[cfg_attr(not(feature = "sim-plant"), allow(dead_code))]
    pub(crate) fn tvc_center_and_stroke_mm(&self) -> ([f64; 2], [f64; 2]) {
        let axes = [self.tvc.pitch, self.tvc.yaw];
        (
            axes.map(|a| a.angle_to_extension_mm(0.0)),
            axes.map(|a| a.stroke * 1000.0),
        )
    }

    /// Actuator demands for `control`. Positive roll torque runs fans M1 and
    /// M3, negative runs M2 and M4. NaN in stays NaN out, so the link rejects
    /// it instead of sending it.
    pub(crate) fn demand(&self, control: &ControlVector) -> ActuatorDemand {
        let tvc = self.tvc.command(control.tvc_pitch, control.tvc_yaw);
        let throttle = (control.thrust / self.full_throttle_thrust_n).clamp(0.0, 1.0);
        let duty = (control.rcs_torque.abs() / self.full_duty_roll_torque_nm).clamp(0.0, 1.0);
        let rcs_duty = if control.rcs_torque >= 0.0 {
            [duty, 0.0, duty, 0.0]
        } else {
            [0.0, duty, 0.0, duty]
        };

        ActuatorDemand {
            tvc_extension_mm: [tvc.pitch_mm, tvc.yaw_mm],
            rcs_duty,
            main_throttle: throttle,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn control(thrust: f64, tvc_pitch: f64, tvc_yaw: f64, rcs_torque: f64) -> ControlVector {
        ControlVector {
            thrust,
            tvc_pitch,
            tvc_yaw,
            rcs_torque,
        }
    }

    #[test]
    fn centered_gimbal_and_no_torque() {
        let demand = Allocation::PLACEHOLDER.demand(&control(50.0, 0.0, 0.0, 0.0));
        let center = PLACEHOLDER_AXIS.angle_to_extension_mm(0.0);

        assert_eq!(demand.tvc_extension_mm, [center, center]);
        assert_eq!(demand.rcs_duty, [0.0; 4]);
        assert!((demand.main_throttle - 0.5).abs() < 1e-12);
    }

    #[test]
    fn roll_torque_picks_the_fan_pair() {
        let right = Allocation::PLACEHOLDER.demand(&control(0.0, 0.0, 0.0, 0.05));
        let left = Allocation::PLACEHOLDER.demand(&control(0.0, 0.0, 0.0, -0.05));

        assert_eq!(right.rcs_duty, [0.5, 0.0, 0.5, 0.0]);
        assert_eq!(left.rcs_duty, [0.0, 0.5, 0.0, 0.5]);
    }

    #[test]
    fn outputs_are_clamped_to_their_range() {
        let demand = Allocation::PLACEHOLDER.demand(&control(1000.0, 1.0, -1.0, 5.0));

        assert_eq!(demand.main_throttle, 1.0);
        assert_eq!(demand.rcs_duty, [1.0, 0.0, 1.0, 0.0]);
        assert!((demand.tvc_extension_mm[0] - 26.9).abs() < 1e-9);
        assert_eq!(demand.tvc_extension_mm[1], 0.0);

        let negative = Allocation::PLACEHOLDER.demand(&control(-10.0, 0.0, 0.0, 0.0));
        assert_eq!(negative.main_throttle, 0.0);
    }

    #[test]
    fn nan_stays_nan() {
        let demand = Allocation::PLACEHOLDER.demand(&control(f64::NAN, 0.0, 0.0, 0.0));
        assert!(demand.main_throttle.is_nan());
    }
}
