//! From the navigation estimate to the controller's frame.
//!
//! The controller (`CtvLqrController`) and the gain schedule it follows are
//! designed in a frame that is fixed by the pad, not by the compass:
//!
//! - position is zero on the pad where the vehicle stands, NED, and the
//!   point it measures is the vehicle's reference point, not the IMU;
//! - the attitude on the pad is the schedule's first reference attitude, the
//!   vertical one, `(sqrt2/2, 0, sqrt2/2, 0)` (body x up), at the schedule's
//!   heading, and
//! - velocity is NED in that same frame.
//!
//! The navigation filter instead gives position of the IMU from wherever its
//! origin is (the ground at the pad, from the barometer and GPS), and an
//! attitude whose heading comes from the magnetometer, so it is a different
//! compass heading on every pad. Flying the schedule from that heading would
//! ask for a roll to the reference heading as soon as the controller started.
//!
//! [`NavFrame::tare`] is called once on the pad, at arm. It records the pad
//! pose from a healthy estimate; [`NavFrame::control_state`] then removes the
//! pad position, the heading offset and the IMU lever arm from every later
//! estimate. Only the heading is re-referenced: the tilt of the vehicle on the
//! pad is kept, and arming is refused if it is more than a few degrees.
//!
//! Hopper's counterpart is `nav_state_to_sim.m` with `nav_frames.m`, whose nav
//! filter has z down the long axis and so needs a fixed rotation as well. This
//! filter's body frame is already the sim's (body x up, body-to-NED quaternion,
//! scalar first), so only the pad tare is needed, and the CG offset is the
//! lever arm below.

use std::fmt;

use common::comm::ctv::ControlState;
use nalgebra::{Quaternion, UnitQuaternion, Vector3};

use crate::navigate::NavState;

/// The largest tilt from vertical, in degrees, at which the pad pose can be
/// taken. A leaning pad or a bad leveling would otherwise be flown out.
pub const MAX_TARE_TILT_DEG: f64 = 3.0;

/// The vertical reference attitude of the CTV schedule, body to NED, scalar
/// first: body x up.
pub fn vertical_attitude() -> UnitQuaternion<f64> {
    let h = std::f64::consts::FRAC_1_SQRT_2;

    UnitQuaternion::from_quaternion(Quaternion::new(h, 0.0, h, 0.0))
}

/// Why the pad pose could not be taken.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum FrameError {
    /// The estimate has a non-finite component or no usable attitude.
    InvalidEstimate,
    /// The vehicle is tilted more than [`MAX_TARE_TILT_DEG`] from vertical.
    NotUpright { tilt_deg: f64 },
}

impl fmt::Display for FrameError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            FrameError::InvalidEstimate => {
                write!(f, "the estimate has non-finite values or no attitude")
            }
            FrameError::NotUpright { tilt_deg } => write!(
                f,
                "the vehicle is tilted {tilt_deg:.1} deg from vertical (limit {MAX_TARE_TILT_DEG})"
            ),
        }
    }
}

impl std::error::Error for FrameError {}

/// The pad pose, and the offset of the controller's reference point from the
/// IMU. Create it with [`NavFrame::tare`].
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct NavFrame {
    /// Rotation about the NED vertical from the estimate's heading to the
    /// schedule's heading.
    heading: UnitQuaternion<f64>,
    /// The reference point's position in the estimate's NED frame on the pad.
    pad_position: Vector3<f64>,
    /// The reference point relative to the IMU, in body axes, m.
    lever_body: Vector3<f64>,
}

/// The tilt of the body x axis from vertical, in degrees, for a body-to-NED
/// attitude.
pub fn tilt_deg(attitude: &UnitQuaternion<f64>) -> f64 {
    // body x in NED; up is -z
    let x_ned = attitude * Vector3::x();

    (-x_ned.z).clamp(-1.0, 1.0).acos().to_degrees()
}

/// The heading of a body-to-NED attitude: the direction of the body y axis in
/// the horizontal plane, measured from north toward east, in radians. None
/// when y is nearly vertical, which only happens far from upright.
fn heading(attitude: &UnitQuaternion<f64>) -> Option<f64> {
    let y_ned = attitude * Vector3::y();

    (y_ned.x.hypot(y_ned.y) > 0.5).then(|| y_ned.y.atan2(y_ned.x))
}

/// A rotation about the NED vertical (z, down) by `angle` radians.
fn about_down(angle: f64) -> UnitQuaternion<f64> {
    UnitQuaternion::from_axis_angle(&Vector3::z_axis(), angle)
}

fn finite(estimate: &NavState) -> bool {
    estimate.position.iter().all(|v| v.is_finite())
        && estimate.velocity.iter().all(|v| v.is_finite())
        && estimate.body_rate.iter().all(|v| v.is_finite())
        && [
            estimate.attitude.w,
            estimate.attitude.i,
            estimate.attitude.j,
            estimate.attitude.k,
        ]
        .iter()
        .all(|v| v.is_finite())
        && estimate.attitude.norm() > 1e-6
}

impl NavFrame {
    /// Takes the pad pose from `estimate`, the vehicle standing on the pad.
    /// `lever_body` is the controller's reference point relative to the IMU in
    /// body axes (zero until the mount is measured).
    pub fn tare(estimate: &NavState, lever_body: Vector3<f64>) -> Result<Self, FrameError> {
        if !finite(estimate) || !lever_body.iter().all(|v| v.is_finite()) {
            return Err(FrameError::InvalidEstimate);
        }

        let attitude = UnitQuaternion::from_quaternion(estimate.attitude);
        let tilt = tilt_deg(&attitude);

        if tilt > MAX_TARE_TILT_DEG {
            return Err(FrameError::NotUpright { tilt_deg: tilt });
        }

        let (Some(est), Some(reference)) = (heading(&attitude), heading(&vertical_attitude()))
        else {
            return Err(FrameError::InvalidEstimate);
        };

        Ok(NavFrame {
            heading: about_down(reference - est),
            pad_position: estimate.position + attitude * lever_body,
            lever_body,
        })
    }

    /// The controller's input for `estimate`, in the pad frame.
    pub fn control_state(&self, estimate: &NavState) -> ControlState {
        let attitude = UnitQuaternion::from_quaternion(estimate.attitude);
        let reference_point = estimate.position + attitude * self.lever_body;
        // The estimate's velocity is the IMU's. The reference point moves with
        // the vehicle's rotation about the IMU too: v + (rate x lever), in NED.
        let reference_velocity =
            estimate.velocity + attitude * estimate.body_rate.cross(&self.lever_body);

        ControlState {
            time: estimate.time,
            position: self.heading * (reference_point - self.pad_position),
            velocity: self.heading * reference_velocity,
            body_rate: estimate.body_rate,
            attitude: *(self.heading * attitude).quaternion(),
        }
    }
}

#[cfg(test)]
mod tests {
    use std::time::Duration;

    use super::*;

    fn estimate(position: Vector3<f64>, attitude: UnitQuaternion<f64>) -> NavState {
        NavState {
            time: Duration::from_millis(1234),
            position,
            velocity: Vector3::new(0.1, -0.2, -0.3),
            attitude: *attitude.quaternion(),
            body_rate: Vector3::new(0.01, 0.02, 0.03),
            position_std: Vector3::repeat(0.1),
            velocity_std: Vector3::repeat(0.1),
            attitude_std: Vector3::repeat(0.01),
            body_rate_std: Vector3::repeat(0.01),
        }
    }

    /// The vertical attitude turned about the vertical by `psi`.
    fn upright_at(psi: f64) -> UnitQuaternion<f64> {
        about_down(psi) * vertical_attitude()
    }

    fn close(a: &Vector3<f64>, b: &Vector3<f64>) -> bool {
        (a - b).norm() < 1e-9
    }

    #[test]
    fn the_vertical_attitude_is_body_x_up() {
        let up = vertical_attitude() * Vector3::x();

        assert!(close(&up, &Vector3::new(0.0, 0.0, -1.0)), "{up:?}");
        assert!(tilt_deg(&vertical_attitude()) < 1e-9);
    }

    #[test]
    fn on_the_pad_the_control_state_is_the_schedules_start() {
        let pad = estimate(Vector3::new(12.0, -7.0, -1.3), upright_at(0.8));
        let frame = NavFrame::tare(&pad, Vector3::zeros()).unwrap();
        let state = frame.control_state(&pad);

        assert!(close(&state.position, &Vector3::zeros()), "{state:?}");

        let q = vertical_attitude();

        assert!(
            (state.attitude.coords - q.quaternion().coords).norm() < 1e-9
                || (state.attitude.coords + q.quaternion().coords).norm() < 1e-9,
            "{state:?}"
        );
    }

    #[test]
    fn any_compass_heading_gives_the_same_control_state() {
        // The same motion, seen by two filters that disagree on north.
        let motion = |psi: f64, origin: Vector3<f64>| {
            let rot = about_down(psi);
            let pad = estimate(origin, upright_at(psi));
            let frame = NavFrame::tare(&pad, Vector3::zeros()).unwrap();

            // 2 m up, 0.3 m east of the pad in the schedule's frame, tilted a
            // little about the body y axis.
            let truth_attitude =
                UnitQuaternion::from_axis_angle(&Vector3::z_axis(), 0.05) * vertical_attitude();
            let mut seen = estimate(
                origin + rot * Vector3::new(0.0, 0.3, -2.0),
                rot * truth_attitude,
            );

            seen.velocity = rot * Vector3::new(0.0, 0.0, -1.0);

            frame.control_state(&seen)
        };

        let a = motion(0.0, Vector3::new(0.0, 0.0, 0.0));
        let b = motion(2.4, Vector3::new(100.0, -50.0, -3.0));

        assert!(close(&a.position, &b.position), "{a:?} {b:?}");
        assert!(close(&a.velocity, &b.velocity), "{a:?} {b:?}");
        assert!(close(&a.position, &Vector3::new(0.0, 0.3, -2.0)), "{a:?}");

        let dot = a.attitude.coords.dot(&b.attitude.coords).abs();

        assert!((dot - 1.0).abs() < 1e-9, "{dot}");
    }

    #[test]
    fn body_rates_pass_through() {
        let pad = estimate(Vector3::zeros(), upright_at(1.0));
        let frame = NavFrame::tare(&pad, Vector3::zeros()).unwrap();

        assert_eq!(frame.control_state(&pad).body_rate, pad.body_rate);
    }

    #[test]
    fn the_lever_arm_moves_the_position_to_the_reference_point() {
        // Reference point 0.3 m below the IMU along body x (x is up).
        let lever = Vector3::new(-0.3, 0.0, 0.0);
        let pad = estimate(Vector3::new(0.0, 0.0, -0.8), upright_at(0.0));
        let frame = NavFrame::tare(&pad, lever).unwrap();

        // Standing on the pad the reference point is the pad position: zero.
        assert!(close(
            &frame.control_state(&pad).position,
            &Vector3::zeros()
        ));

        // Climbing 1 m, upright, moves the reference point 1 m.
        let up = estimate(Vector3::new(0.0, 0.0, -1.8), upright_at(0.0));

        assert!(close(
            &frame.control_state(&up).position,
            &Vector3::new(0.0, 0.0, -1.0)
        ));

        // Tilting 10 deg about north, with the IMU where it was, swings the
        // reference point (0.3 m below the IMU along the tilted body x) to
        // the side and up by 0.3 (1 - cos) .
        let a = 10f64.to_radians();
        let tilt = UnitQuaternion::from_axis_angle(&Vector3::x_axis(), a) * vertical_attitude();
        let tilted = estimate(Vector3::new(0.0, 0.0, -0.8), tilt);
        let expected = Vector3::new(0.0, -0.3 * a.sin(), 0.3 * (a.cos() - 1.0));

        assert!(close(&frame.control_state(&tilted).position, &expected));
    }

    #[test]
    fn the_velocity_is_the_reference_points_not_the_imus() {
        // Reference point 0.3 m below the IMU; the vehicle pitches and rolls
        // while the IMU moves
        let lever = Vector3::new(-0.3, 0.0, 0.0);
        let rate = Vector3::new(0.1, 0.5, -0.4);
        let tilt = UnitQuaternion::from_axis_angle(&Vector3::y_axis(), 0.2) * vertical_attitude();
        let at_start = NavState {
            body_rate: rate,
            ..estimate(Vector3::new(0.0, 0.0, -0.8), tilt)
        };
        let frame = NavFrame::tare(
            &estimate(Vector3::new(0.0, 0.0, -0.8), upright_at(0.0)),
            lever,
        )
        .unwrap();

        // Step the IMU along its velocity and the attitude by the body rate, a
        // little either side: the reference point's velocity is how its position
        // changes (a central difference, so the step's own error is far below the
        // tolerance)
        let dt = 1e-4;
        let at = |s: f64| NavState {
            position: at_start.position + at_start.velocity * (s * dt),
            attitude: *(UnitQuaternion::from_quaternion(at_start.attitude)
                * UnitQuaternion::from_scaled_axis(rate * (s * dt)))
            .quaternion(),
            ..at_start
        };
        let moved =
            frame.control_state(&at(1.0)).position - frame.control_state(&at(-1.0)).position;
        let velocity = frame.control_state(&at_start).velocity;

        assert!(
            close(&(moved / (2.0 * dt)), &velocity),
            "{:?} {velocity:?}",
            moved / (2.0 * dt)
        );
        // and it is not the IMU's
        assert!(!close(&velocity, &at_start.velocity));
    }

    #[test]
    fn refuses_a_tilted_vehicle() {
        // 10 deg about north tips body x off vertical; about down it would
        // only turn the heading
        let tilted = UnitQuaternion::from_axis_angle(&Vector3::x_axis(), 10f64.to_radians())
            * vertical_attitude();
        let error =
            NavFrame::tare(&estimate(Vector3::zeros(), tilted), Vector3::zeros()).unwrap_err();

        assert!(matches!(error, FrameError::NotUpright { .. }), "{error:?}");
    }

    #[test]
    fn refuses_a_non_finite_estimate() {
        let mut bad = estimate(Vector3::zeros(), upright_at(0.0));

        bad.position.x = f64::NAN;

        assert_eq!(
            NavFrame::tare(&bad, Vector3::zeros()),
            Err(FrameError::InvalidEstimate)
        );

        let mut zero = estimate(Vector3::zeros(), upright_at(0.0));

        zero.attitude = Quaternion::new(0.0, 0.0, 0.0, 0.0);

        assert_eq!(
            NavFrame::tare(&zero, Vector3::zeros()),
            Err(FrameError::InvalidEstimate)
        );
    }
}
