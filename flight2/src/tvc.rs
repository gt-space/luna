use common::comm::ctv::ControlState;
use nalgebra::{UnitQuaternion, Vector3};

#[derive(Debug, Clone, Copy)]
pub struct TvcConfig {
    pub kp: f64,
    pub kd: f64,
    pub max_angle: f64,
    pub max_rate: f64,
}

impl Default for TvcConfig {
    fn default() -> Self {
        Self {
            kp: 0.5,
            kd: 0.1,
            max_angle: 10f64.to_radians(),
            max_rate: 60f64.to_radians(),
        }
    }
}

//#[derive(Debug, Clone, Copy, PartialEq, Default)]
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct TvcCommand {
    pub pitch: f64,
    pub yaw: f64,
}

pub struct TvcController {
    config: TvcConfig,
    target: UnitQuaternion<f64>,
    last_cmd: TvcCommand,
}

impl TvcController {
    pub fn new(config: TvcConfig) -> Self {
        Self {
            config,
            target: UnitQuaternion::identity(),
            last_cmd: TvcCommand::default(),
        }
    }

    pub fn set_target(&mut self, target: UnitQuaternion<f64>) {
        self.target = target;
    }

    pub fn reset(&mut self) {
        self.last_cmd = TvcCommand::default();
    }

    pub fn step(&mut self, state: &ControlState, dt: f64) -> TvcCommand {
        assert!(dt.is_finite() && dt >= 0.0, "dt must be finite and nonnegative");

        let current = UnitQuaternion::from_quaternion(state.attitude);
        let err = attitude_error(&current, &self.target);
        let rate = state.body_rate;

        // Angles in radians, body rates in rad/s.
        // Assumes x = roll/thrust axis, y = pitch, z = yaw.
        // Gimbal-to-torque signs must be verified for your vehicle.
        let raw = TvcCommand {
            pitch: self.config.kp * err.y - self.config.kd * rate.y,
            yaw: self.config.kp * err.z - self.config.kd * rate.z,
        };

        let cmd = TvcCommand {
            pitch: self.limit(raw.pitch, self.last_cmd.pitch, dt),
            yaw: self.limit(raw.yaw, self.last_cmd.yaw, dt),
        };

        self.last_cmd = cmd;
        cmd
    }

    fn limit(&self, desired: f64, last: f64, dt: f64) -> f64 {
        let max_step = self.config.max_rate * dt;

        desired
            .clamp(-self.config.max_angle, self.config.max_angle)
            .clamp(last - max_step, last + max_step)
    }
}

// Body-frame error assumes attitude maps body coordinates to world coordinates.
fn attitude_error(
    current: &UnitQuaternion<f64>,
    target: &UnitQuaternion<f64>,
) -> Vector3<f64> {
    let mut q_err = current.inverse() * target;

    if q_err.w < 0.0 {
        q_err = UnitQuaternion::new_unchecked(-q_err.into_inner());
    }

    q_err.scaled_axis()
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::time::Duration;

    fn state(
        attitude: UnitQuaternion<f64>,
        body_rate: Vector3<f64>,
    ) -> ControlState {
        ControlState {
            time: Duration::ZERO,
            position: Vector3::zeros(),
            velocity: Vector3::zeros(),
            body_rate,
            attitude: attitude.into_inner(),
        }
    }

    #[test]
    fn upright_and_still_gives_zero() {
        let mut tvc = TvcController::new(TvcConfig::default());
        let s = state(UnitQuaternion::identity(), Vector3::zeros());

        let cmd = tvc.step(&s, 0.01);

        assert_eq!(cmd, TvcCommand::default());
    }

        #[test]
    fn matching_target_gives_zero() {
        let mut tvc = TvcController::new(TvcConfig::default());
        let target = UnitQuaternion::from_axis_angle(
            &Vector3::y_axis(),
            15f64.to_radians(),
        );

        tvc.set_target(target);
        let cmd = tvc.step(&state(target, Vector3::zeros()), 0.01);

        assert!(cmd.pitch.abs() < 1e-12);
        assert!(cmd.yaw.abs() < 1e-12);
    }

    #[test]
    fn reset_clears_previous_command() {
        let mut tvc = TvcController::new(TvcConfig::default());
        let tilted = UnitQuaternion::from_axis_angle(
            &Vector3::y_axis(),
            45f64.to_radians(),
        );

        let cmd = tvc.step(&state(tilted, Vector3::zeros()), 0.01);
        assert!(cmd.pitch.abs() > 0.0);

        tvc.reset();

        let cmd = tvc.step(
            &state(UnitQuaternion::identity(), Vector3::zeros()),
            0.01,
        );
        assert_eq!(cmd, TvcCommand::default());
    }
    
    #[test]
    fn big_tilt_saturates_at_gimbal_limit() {
        let cfg = TvcConfig::default();
        let mut tvc = TvcController::new(cfg);
        let tilted =
            UnitQuaternion::from_axis_angle(&Vector3::y_axis(), 45f64.to_radians());
        let s = state(tilted, Vector3::zeros());

        let mut cmd = TvcCommand::default();
        for _ in 0..100 {
            cmd = tvc.step(&s, 0.01);
        }

        assert!((cmd.pitch + cfg.max_angle).abs() < 1e-9);
        assert_eq!(cmd.yaw, 0.0);
    }

    #[test]
    fn slew_rate_is_limited() {
        let cfg = TvcConfig::default();
        let mut tvc = TvcController::new(cfg);
        let tilted =
            UnitQuaternion::from_axis_angle(&Vector3::y_axis(), 45f64.to_radians());
        let s = state(tilted, Vector3::zeros());

        let cmd = tvc.step(&s, 0.01);

        assert!((cmd.pitch + cfg.max_rate * 0.01).abs() < 1e-12);
        assert_eq!(cmd.yaw, 0.0);
    }

    #[test]
    fn rate_adds_damping() {
        let mut tvc = TvcController::new(TvcConfig::default());
        let s = state(
            UnitQuaternion::identity(),
            Vector3::new(0.0, 1.0, 0.0),
        );

        let cmd = tvc.step(&s, 0.01);

        assert!(cmd.pitch < 0.0);
        assert_eq!(cmd.yaw, 0.0);
    }
}