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
    /// The gimbal's limit on each axis, in rad. The plant's gimbal saturates
    /// there, and the controller's gains were designed for it; a command past
    /// it used to go to the actuator as far as its stroke allows, about twice
    /// as far, which the plant would never have received.
    max_tvc_angle_rad: f64,
}

/// Made-up geometry with a 26.9 mm stroke centered on the gimbal's zero angle.
pub(crate) const PLACEHOLDER_AXIS: AxisGeometry = AxisGeometry {
    r: 0.05,
    d: 0.12,
    centered_length: 0.13,
    retracted_length: 0.1165,
    stroke: 0.0269,
    inverted: false,
};

/// The CTV's gimbal limit in the hopper sim: 8 degrees.
pub(crate) const DEFAULT_MAX_TVC_ANGLE_RAD: f64 = 8.0 * std::f64::consts::PI / 180.0;

impl Allocation {
    /// Placeholder numbers until the characterization and CAD values exist.
    pub(crate) const PLACEHOLDER: Allocation = Allocation {
        tvc: TvcActuators {
            pitch: PLACEHOLDER_AXIS,
            yaw: PLACEHOLDER_AXIS,
        },
        full_throttle_thrust_n: 100.0,
        full_duty_roll_torque_nm: 0.1,
        max_tvc_angle_rad: DEFAULT_MAX_TVC_ANGLE_RAD,
    };

    /// An allocation from measured or configured values: gimbal geometry,
    /// thrust at full throttle (N; the ESC taken as linear) and roll torque
    /// from one fan pair at full duty (N·m). The sim-plant bench test builds
    /// it from its config file (`bench_config.rs`).
    #[cfg_attr(not(feature = "sim-plant"), allow(dead_code))]
    pub(crate) fn new(
        tvc: TvcActuators,
        full_throttle_thrust_n: f64,
        full_duty_roll_torque_nm: f64,
    ) -> Allocation {
        Allocation {
            tvc,
            full_throttle_thrust_n,
            full_duty_roll_torque_nm,
            max_tvc_angle_rad: DEFAULT_MAX_TVC_ANGLE_RAD,
        }
    }

    /// The gimbal angle each axis is held to, degrees.
    pub(crate) fn max_tvc_angle_deg(&self) -> f64 {
        self.max_tvc_angle_rad.to_degrees()
    }

    /// The same allocation with the gimbal limited to `degrees` on each axis.
    #[cfg_attr(not(feature = "sim-plant"), allow(dead_code))]
    pub(crate) fn with_tvc_limit_deg(mut self, degrees: f64) -> Allocation {
        self.max_tvc_angle_rad = degrees.to_radians();

        self
    }

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

    /// The control vector that `demand` would turn into `demand`: the inverse
    /// of `demand` for anything it can produce. It is for feeding a model the
    /// command the actuators actually got, after caps changed it. The fan pairs
    /// are averaged, so a demand that drives both pairs gives their net torque.
    /// NaN in stays NaN out.
    #[cfg_attr(not(feature = "sim-plant"), allow(dead_code))]
    pub(crate) fn command_for(&self, demand: &ActuatorDemand) -> ControlVector {
        let [m1, m2, m3, m4] = demand.rcs_duty;
        let net_duty = (m1 + m3) / 2.0 - (m2 + m4) / 2.0;

        ControlVector {
            thrust: demand.main_throttle * self.full_throttle_thrust_n,
            tvc_pitch: self
                .tvc
                .pitch
                .extension_mm_to_angle(demand.tvc_extension_mm[0]),
            tvc_yaw: self
                .tvc
                .yaw
                .extension_mm_to_angle(demand.tvc_extension_mm[1]),
            rcs_torque: net_duty * self.full_duty_roll_torque_nm,
        }
    }

    /// Actuator demands for `control`. Positive roll torque runs fans M1 and
    /// M3, negative runs M2 and M4. NaN in stays NaN out, so the link rejects
    /// it instead of sending it.
    pub(crate) fn demand(&self, control: &ControlVector) -> ActuatorDemand {
        // NaN stays NaN through clamp, so the link still rejects it
        let limit = self.max_tvc_angle_rad;
        let tvc = self
            .tvc
            .command(control.tvc_pitch.clamp(-limit, limit), control.tvc_yaw.clamp(-limit, limit));
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

        // the gimbal goes as far as its limit, not as far as the stroke allows
        let at_limit = Allocation::PLACEHOLDER.demand(&control(0.0, DEFAULT_MAX_TVC_ANGLE_RAD, -DEFAULT_MAX_TVC_ANGLE_RAD, 0.0));

        assert_eq!(demand.tvc_extension_mm, at_limit.tvc_extension_mm);
        assert!(demand.tvc_extension_mm[0] < 26.0 && demand.tvc_extension_mm[1] > 1.0);

        let negative = Allocation::PLACEHOLDER.demand(&control(-10.0, 0.0, 0.0, 0.0));
        assert_eq!(negative.main_throttle, 0.0);
    }

    #[test]
    fn command_for_inverts_demand_inside_the_limits() {
        let a = Allocation::PLACEHOLDER;

        for (thrust, pitch, yaw, torque) in [
            (50.0, 0.0, 0.0, 0.0),
            (73.0, 0.05, -0.03, 0.04),
            (10.0, -0.08, 0.08, -0.07),
        ] {
            let u = control(thrust, pitch, yaw, torque);
            let back = a.command_for(&a.demand(&u));

            assert!((back.thrust - u.thrust).abs() < 1e-9, "{back:?}");
            assert!((back.tvc_pitch - u.tvc_pitch).abs() < 1e-9, "{back:?}");
            assert!((back.tvc_yaw - u.tvc_yaw).abs() < 1e-9, "{back:?}");
            assert!((back.rcs_torque - u.rcs_torque).abs() < 1e-9, "{back:?}");
        }
    }

    #[test]
    fn command_for_reports_what_the_actuators_got_not_what_was_asked() {
        let a = Allocation::PLACEHOLDER;
        let mut demand = a.demand(&control(80.0, 0.0, 0.0, 0.0));

        // A cap of 70 % throttle, as the bench config applies
        demand.main_throttle = 0.7;

        let got = a.command_for(&demand);

        assert!((got.thrust - 70.0).abs() < 1e-9, "{got:?}");
    }

    #[test]
    fn command_for_nan_stays_nan() {
        let a = Allocation::PLACEHOLDER;
        let demand = a.demand(&control(f64::NAN, f64::NAN, 0.0, f64::NAN));
        let back = a.command_for(&demand);

        assert!(back.thrust.is_nan());
        assert!(back.tvc_pitch.is_nan());
        assert!(back.rcs_torque.is_nan());
    }

    #[test]
    fn the_gimbal_limit_is_per_axis_and_symmetric_and_can_be_set() {
        let a = Allocation::PLACEHOLDER.with_tvc_limit_deg(4.0);
        let four = 4f64.to_radians();
        let inside = a.demand(&control(0.0, 0.5 * four, -0.5 * four, 0.0));
        let past = a.demand(&control(0.0, 3.0 * four, -3.0 * four, 0.0));
        let limit = a.demand(&control(0.0, four, -four, 0.0));

        assert_eq!(past.tvc_extension_mm, limit.tvc_extension_mm);
        assert_ne!(inside.tvc_extension_mm, limit.tvc_extension_mm);

        // and the other way round
        let flipped = a.demand(&control(0.0, -3.0 * four, 3.0 * four, 0.0));
        let mirrored = a.demand(&control(0.0, -four, four, 0.0));

        assert_eq!(flipped.tvc_extension_mm, mirrored.tvc_extension_mm);
    }

    #[test]
    fn nan_stays_nan() {
        let demand = Allocation::PLACEHOLDER.demand(&control(f64::NAN, 0.0, 0.0, 0.0));
        assert!(demand.main_throttle.is_nan());
    }
}
