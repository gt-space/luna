//! Converts TVC gimbal angles into PA-12-T actuator extensions.
//!
//! The FC sends each actuator's rod extension in millimeters to the engine
//! controller, which converts it to PA-12 goal position counts with its own
//! calibration. This module handles the geometry between a gimbal angle and
//! that extension.
//!
//! Each axis is driven by one linear actuator forming a triangle with the
//! gimbal pivot (see "Actuator Angle Rate Derivation" in Notion):
//!
//! ```text
//! L(θ)² = r² + d² - 2·r·d·cos(φ₀ + θ)
//! ```
//!
//! where `L` is the pin-to-pin actuator length, `r` is the pivot to actuator
//! gimbal mount distance, `d` is the pivot to actuator structural mount
//! distance, and `φ₀` is the angle between them with the gimbal centered.
//! Pitch and yaw are treated as independent axes.

/// Fixed geometry for one gimbal axis. All lengths are in meters, measured
/// pin-to-pin from CAD.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct AxisGeometry {
    /// Distance from the gimbal pivot to the actuator's gimbal-side mount.
    pub r: f64,
    /// Distance from the gimbal pivot to the actuator's structure-side mount.
    pub d: f64,
    /// Actuator length when the gimbal is centered (θ = 0).
    pub centered_length: f64,
    /// Actuator length when fully retracted (zero extension).
    pub retracted_length: f64,
    /// Usable rod travel. The 1.06" PA-12-T has 26.9 mm with factory stroke
    /// limits.
    pub stroke: f64,
    /// Set if a positive gimbal angle retracts the actuator instead of
    /// extending it.
    pub inverted: bool,
}

impl AxisGeometry {
    /// φ₀, the pivot angle when the gimbal is centered.
    fn centered_phi(&self) -> f64 {
        let Self {
            r,
            d,
            centered_length: l0,
            ..
        } = *self;

        ((r * r + d * d - l0 * l0) / (2.0 * r * d)).acos()
    }

    fn sign(&self) -> f64 {
        if self.inverted {
            -1.0
        } else {
            1.0
        }
    }

    /// Actuator length in meters for a gimbal angle in radians.
    pub fn angle_to_length(&self, angle: f64) -> f64 {
        let Self { r, d, .. } = *self;
        let phi = self.centered_phi() + self.sign() * angle;

        (r * r + d * d - 2.0 * r * d * phi.cos()).sqrt()
    }

    /// Gimbal angle in radians for an actuator length in meters.
    pub fn length_to_angle(&self, length: f64) -> f64 {
        let Self { r, d, .. } = *self;
        let cos_phi = (r * r + d * d - length * length) / (2.0 * r * d);
        let phi = cos_phi.clamp(-1.0, 1.0).acos();

        self.sign() * (phi - self.centered_phi())
    }

    /// Rod extension in millimeters for a gimbal angle in radians, clamped to
    /// the stroke.
    pub fn angle_to_extension_mm(&self, angle: f64) -> f64 {
        let extension = self.angle_to_length(angle) - self.retracted_length;

        extension.clamp(0.0, self.stroke) * 1000.0
    }

    /// Gimbal angle in radians for a rod extension in millimeters, e.g. a
    /// measured position reported back by the engine controller.
    pub fn extension_mm_to_angle(&self, extension_mm: f64) -> f64 {
        self.length_to_angle(self.retracted_length + extension_mm / 1000.0)
    }

    /// Whether every angle in `[-max_angle, max_angle]` fits inside the
    /// stroke. Check this at startup so commands are never silently clamped.
    pub fn fits(&self, max_angle: f64) -> bool {
        [-max_angle, max_angle].iter().all(|&angle| {
            let extension = self.angle_to_length(angle) - self.retracted_length;
            (0.0..=self.stroke).contains(&extension)
        })
    }

    /// Fastest gimbal rate in rad/s the actuator can produce at `angle`,
    /// given its linear speed in m/s (from the datasheet speed vs. load
    /// curve).
    pub fn max_gimbal_rate(&self, angle: f64, actuator_speed: f64) -> f64 {
        let Self { r, d, .. } = *self;
        let phi = self.centered_phi() + self.sign() * angle;
        let dl_dtheta = r * d * phi.sin() / self.angle_to_length(angle);

        actuator_speed / dl_dtheta.abs()
    }
}

/// Rod extensions for both TVC actuators, in millimeters.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ActuatorCommand {
    pub pitch_mm: f64,
    pub yaw_mm: f64,
}

/// Geometry for both gimbal axes.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct TvcActuators {
    pub pitch: AxisGeometry,
    pub yaw: AxisGeometry,
}

impl TvcActuators {
    /// Converts gimbal angles in radians into actuator extensions.
    pub fn command(&self, pitch: f64, yaw: f64) -> ActuatorCommand {
        ActuatorCommand {
            pitch_mm: self.pitch.angle_to_extension_mm(pitch),
            yaw_mm: self.yaw.angle_to_extension_mm(yaw),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Made-up geometry with a 26.9 mm stroke centered on the gimbal's zero
    /// angle. Replace with CAD values before use.
    fn example() -> AxisGeometry {
        AxisGeometry {
            r: 0.05,
            d: 0.12,
            centered_length: 0.13,
            retracted_length: 0.1165,
            stroke: 0.0269,
            inverted: false,
        }
    }

    #[test]
    fn centered_gimbal_gives_centered_length() {
        let axis = example();

        assert!((axis.angle_to_length(0.0) - 0.13).abs() < 1e-12);
        assert!(axis.length_to_angle(0.13).abs() < 1e-12);
        assert!((axis.angle_to_extension_mm(0.0) - 13.5).abs() < 1e-9);
    }

    #[test]
    fn positive_angle_extends_actuator() {
        let axis = example();

        assert!(axis.angle_to_length(5f64.to_radians()) > 0.13);
        assert!(axis.angle_to_length(-5f64.to_radians()) < 0.13);
    }

    #[test]
    fn inverted_flips_direction() {
        let mut axis = example();
        axis.inverted = true;

        assert!(axis.angle_to_length(5f64.to_radians()) < 0.13);
        assert!((axis.length_to_angle(axis.angle_to_length(0.1)) - 0.1).abs() < 1e-12);
    }

    #[test]
    fn angle_round_trips_through_extension() {
        let axis = example();

        for degrees in [-10.0, -3.5, 0.0, 2.0, 10.0] {
            let angle = f64::to_radians(degrees);
            let back = axis.extension_mm_to_angle(axis.angle_to_extension_mm(angle));

            assert!(
                (back - angle).abs() < 1e-12,
                "{degrees}° came back as {back}"
            );
        }
    }

    #[test]
    fn extension_clamps_to_stroke() {
        let axis = example();

        assert_eq!(axis.angle_to_extension_mm(-60f64.to_radians()), 0.0);
        assert!((axis.angle_to_extension_mm(60f64.to_radians()) - 26.9).abs() < 1e-9);
    }

    #[test]
    fn fits_checks_angle_range_against_stroke() {
        let axis = example();

        assert!(axis.fits(10f64.to_radians()));
        assert!(!axis.fits(30f64.to_radians()));
    }

    #[test]
    fn gimbal_rate_matches_small_angle_derivation() {
        let axis = example();
        let AxisGeometry { r, d, .. } = axis;
        let phi0 = axis.centered_phi();
        // θ̇ = k₀·L̇ with k₀ = L₀ / (r·d·sin φ₀)
        let k0 = 0.13 / (r * d * phi0.sin());

        assert!((axis.max_gimbal_rate(0.0, 0.036) - k0 * 0.036).abs() < 1e-9);
    }

    #[test]
    fn command_converts_both_axes() {
        let tvc = TvcActuators {
            pitch: example(),
            yaw: example(),
        };

        let cmd = tvc.command(0.0, 5f64.to_radians());

        assert_eq!(cmd.pitch_mm, tvc.pitch.angle_to_extension_mm(0.0));
        assert!(cmd.yaw_mm > cmd.pitch_mm);
    }
}
