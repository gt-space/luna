//! Every value the sim-plant bench test may need to change at the bench,
//! read from one JSON file at startup (`--sim-config`), so nothing needs a
//! rebuild: the caps on what reaches the EC, the EDF and RCS fan thrusts,
//! the ESC throttle map and the PA-12 mounting geometry.
//!
//! Any field left out keeps its default; unknown fields and out-of-range
//! values are rejected with the field's name. Defaults are the hopper sim's
//! CTV numbers (`vehicles/ctv/ctv_config.m`) and placeholder PA-12 geometry,
//! all to be tuned at the bench.
//!
//! ```json
//! {
//!   "caps": { "throttle": 0.7, "fan_duty": 0.7, "tvc_travel": 0.7 },
//!   "edf":  { "full_thrust_n": 35.03 },
//!   "rcs":  { "fan_thrust_n": 16.0, "arm_m": 0.10795 },
//!   "tvc":  {
//!     "max_angle_deg": 8.0,
//!     "pitch": { "r_m": 0.05, "d_m": 0.12, "centered_length_m": 0.13,
//!                "retracted_length_m": 0.1165, "stroke_m": 0.0269, "inverted": false },
//!     "yaw":   { "...": "same fields" }
//!   }
//! }
//! ```

use std::{fs, path::Path};

use common::comm::ctv::ActuatorDemand;
use serde::Deserialize;

use crate::{
    ctv_allocation::{Allocation, PLACEHOLDER_AXIS},
    tvc_actuator::{AxisGeometry, TvcActuators},
};

/// Which caps acted on a demand (bits)
pub(crate) const CAP_THROTTLE: u8 = 1;
pub(crate) const CAP_FAN: u8 = 2;
pub(crate) const CAP_TVC: u8 = 4;

/// Everything adjustable at the bench
#[derive(Debug, Clone, Copy, PartialEq, Deserialize, Default)]
#[serde(default, deny_unknown_fields)]
pub(crate) struct BenchConfig {
    pub caps: Caps,
    pub edf: Edf,
    pub rcs: Rcs,
    pub tvc: Tvc,
}

/// Limits on what reaches the EC. The plant never sees them: it flies the
/// controller's own command, so the flight stays nominal.
#[derive(Debug, Clone, Copy, PartialEq, Deserialize)]
#[serde(default, deny_unknown_fields)]
pub(crate) struct Caps {
    /// Max main throttle, 0..1 of full; `null` = uncapped
    pub throttle: Option<f64>,
    /// Max duty per RCS fan, 0..1
    pub fan_duty: f64,
    /// Max gimbal actuator travel from center, as a fraction of the stroke
    /// left on that side, 0..1. ASSUMPTION: what "70%" means for the TVC.
    pub tvc_travel: f64,
}

impl Default for Caps {
    /// 70% on every actuator until more testing (bench guardrail)
    fn default() -> Self {
        Caps {
            throttle: Some(0.7),
            fan_duty: 0.7,
            tvc_travel: 0.7,
        }
    }
}

/// The EDF and its ESC. The ESC is taken as linear: thrust = throttle x
/// full thrust.
#[derive(Debug, Clone, Copy, PartialEq, Deserialize)]
#[serde(default, deny_unknown_fields)]
pub(crate) struct Edf {
    /// Thrust at full throttle, N
    pub full_thrust_n: f64,
}

impl Default for Edf {
    /// 7.875 lbf static thrust, as in the sim
    fn default() -> Self {
        Edf {
            full_thrust_n: 7.875 * 4.448_22,
        }
    }
}

/// The roll fans. One fan pair at full duty gives `fan_thrust_n * arm_m` of
/// roll torque, the same roll authority the plant has.
#[derive(Debug, Clone, Copy, PartialEq, Deserialize)]
#[serde(default, deny_unknown_fields)]
pub(crate) struct Rcs {
    /// Thrust per fan at full duty, N
    pub fan_thrust_n: f64,
    /// Fan thrust line to the roll axis, m
    pub arm_m: f64,
}

impl Default for Rcs {
    /// The sim's placeholders: 16 N on a 4.25 in arm
    fn default() -> Self {
        Rcs {
            fan_thrust_n: 16.0,
            arm_m: 4.25 * 0.0254,
        }
    }
}

/// The gimbal and its two PA-12 actuators (pitch drives A1, yaw A2)
#[derive(Debug, Clone, Copy, PartialEq, Deserialize)]
#[serde(default, deny_unknown_fields)]
pub(crate) struct Tvc {
    /// Gimbal limit the geometry is checked against, deg
    pub max_angle_deg: f64,
    pub pitch: Axis,
    pub yaw: Axis,
}

impl Default for Tvc {
    fn default() -> Self {
        Tvc {
            max_angle_deg: 8.0,
            pitch: Axis::default(),
            yaw: Axis::default(),
        }
    }
}

/// One PA-12 mount, pin to pin (see tvc_actuator.rs)
#[derive(Debug, Clone, Copy, PartialEq, Deserialize)]
#[serde(default, deny_unknown_fields)]
pub(crate) struct Axis {
    pub r_m: f64,
    pub d_m: f64,
    pub centered_length_m: f64,
    pub retracted_length_m: f64,
    pub stroke_m: f64,
    pub inverted: bool,
}

impl Default for Axis {
    /// Placeholder geometry: 26.9 mm stroke centered on the gimbal's zero
    fn default() -> Self {
        Axis::from(PLACEHOLDER_AXIS)
    }
}

impl From<AxisGeometry> for Axis {
    fn from(g: AxisGeometry) -> Self {
        Axis {
            r_m: g.r,
            d_m: g.d,
            centered_length_m: g.centered_length,
            retracted_length_m: g.retracted_length,
            stroke_m: g.stroke,
            inverted: g.inverted,
        }
    }
}

impl Axis {
    fn geometry(&self) -> AxisGeometry {
        AxisGeometry {
            r: self.r_m,
            d: self.d_m,
            centered_length: self.centered_length_m,
            retracted_length: self.retracted_length_m,
            stroke: self.stroke_m,
            inverted: self.inverted,
        }
    }

    fn check(&self, name: &str) -> Result<(), String> {
        for (field, v) in [
            ("r_m", self.r_m),
            ("d_m", self.d_m),
            ("centered_length_m", self.centered_length_m),
            ("retracted_length_m", self.retracted_length_m),
            ("stroke_m", self.stroke_m),
        ] {
            if !(v.is_finite() && v > 0.0) {
                return Err(format!("tvc.{name}.{field} must be > 0 (got {v})"));
            }
        }
        let (lo, hi) = (
            self.retracted_length_m,
            self.retracted_length_m + self.stroke_m,
        );
        if !(lo..=hi).contains(&self.centered_length_m) {
            return Err(format!(
                "tvc.{name}.centered_length_m ({}) must be within the stroke, {lo}..{hi} m",
                self.centered_length_m
            ));
        }
        let (r, d, l) = (self.r_m, self.d_m, self.centered_length_m);
        if !(l < r + d && r < l + d && d < l + r) {
            return Err(format!(
                "tvc.{name}: r_m, d_m and centered_length_m ({r}, {d}, {l}) don't make a triangle"
            ));
        }
        Ok(())
    }
}

impl BenchConfig {
    /// The file's values over the defaults, checked; the defaults alone
    /// without a file.
    pub(crate) fn load(path: Option<&Path>) -> Result<BenchConfig, String> {
        let config = match path {
            None => BenchConfig::default(),
            Some(p) => {
                let text = fs::read_to_string(p).map_err(|e| format!("{}: {e}", p.display()))?;
                serde_json::from_str(&text).map_err(|e| format!("{}: {e}", p.display()))?
            }
        };
        config.check()?;
        Ok(config)
    }

    pub(crate) fn check(&self) -> Result<(), String> {
        let fraction = |field: &str, v: f64| {
            if (0.0..=1.0).contains(&v) {
                Ok(())
            } else {
                Err(format!("{field} must be within 0..1 (got {v})"))
            }
        };
        if let Some(t) = self.caps.throttle {
            fraction("caps.throttle", t)?;
        }
        fraction("caps.fan_duty", self.caps.fan_duty)?;
        fraction("caps.tvc_travel", self.caps.tvc_travel)?;
        for (field, v) in [
            ("edf.full_thrust_n", self.edf.full_thrust_n),
            ("rcs.fan_thrust_n", self.rcs.fan_thrust_n),
            ("rcs.arm_m", self.rcs.arm_m),
        ] {
            if !(v.is_finite() && v > 0.0) {
                return Err(format!("{field} must be > 0 (got {v})"));
            }
        }
        let max = self.tvc.max_angle_deg;
        if !(max.is_finite() && max > 0.0 && max < 45.0) {
            return Err(format!(
                "tvc.max_angle_deg must be within 0..45 (got {max})"
            ));
        }
        self.tvc.pitch.check("pitch")?;
        self.tvc.yaw.check("yaw")?;
        Ok(())
    }

    /// Thrust/torque/angle -> actuator demand with these values
    pub(crate) fn allocation(&self) -> Allocation {
        Allocation::new(
            TvcActuators {
                pitch: self.tvc.pitch.geometry(),
                yaw: self.tvc.yaw.geometry(),
            },
            self.edf.full_thrust_n,
            self.rcs.fan_thrust_n * self.rcs.arm_m,
        )
    }

    /// Axes whose stroke can't reach the gimbal limit (commands there get
    /// clamped by the geometry before any cap)
    pub(crate) fn tvc_short_axes(&self) -> Vec<&'static str> {
        let max = self.tvc.max_angle_deg.to_radians();
        [("pitch", self.tvc.pitch), ("yaw", self.tvc.yaw)]
            .into_iter()
            .filter(|(_, a)| !a.geometry().fits(max))
            .map(|(n, _)| n)
            .collect()
    }

    /// The loaded values, for the startup printout
    pub(crate) fn describe(&self) -> Vec<String> {
        let throttle = match self.caps.throttle {
            None => "uncapped".to_string(),
            Some(t) => format!("{:.0}% of full", t * 100.0),
        };
        let axis = |a: &Axis| {
            format!(
                "r {:.4} d {:.4} centered {:.4} retracted {:.4} stroke {:.4} m{}",
                a.r_m,
                a.d_m,
                a.centered_length_m,
                a.retracted_length_m,
                a.stroke_m,
                if a.inverted { ", inverted" } else { "" }
            )
        };
        vec![
            format!(
                "caps: throttle {throttle}; fans {:.0}% duty; TVC {:.0}% of travel from center (ASSUMPTION)",
                self.caps.fan_duty * 100.0,
                self.caps.tvc_travel * 100.0
            ),
            format!(
                "EDF: {:.2} N at full throttle (ESC linear)",
                self.edf.full_thrust_n
            ),
            format!(
                "RCS: {:.2} N per fan on a {:.4} m arm = {:.3} N m per fan pair at full duty",
                self.rcs.fan_thrust_n,
                self.rcs.arm_m,
                self.rcs.fan_thrust_n * self.rcs.arm_m
            ),
            format!(
                "TVC: limit {:.1} deg; pitch (A1) {}",
                self.tvc.max_angle_deg,
                axis(&self.tvc.pitch)
            ),
            format!("     yaw (A2) {}", axis(&self.tvc.yaw)),
        ]
    }
}

impl Caps {
    /// The demand the EC gets, and which caps acted
    pub(crate) fn apply(
        &self,
        d: &ActuatorDemand,
        center_mm: [f64; 2],
        stroke_mm: [f64; 2],
    ) -> (ActuatorDemand, u8) {
        let mut out = *d;
        let mut acted = 0;
        if let Some(max) = self.throttle {
            if out.main_throttle > max {
                out.main_throttle = max;
                acted |= CAP_THROTTLE;
            }
        }
        for duty in &mut out.rcs_duty {
            if *duty > self.fan_duty {
                *duty = self.fan_duty;
                acted |= CAP_FAN;
            }
        }
        for i in 0..2 {
            let lo = center_mm[i] - self.tvc_travel * center_mm[i];
            let hi = center_mm[i] + self.tvc_travel * (stroke_mm[i] - center_mm[i]);
            let e = out.tvc_extension_mm[i];
            if e < lo || e > hi {
                out.tvc_extension_mm[i] = e.clamp(lo, hi);
                acted |= CAP_TVC;
            }
        }
        (out, acted)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn demand(tvc: [f64; 2], fans: [f64; 4], throttle: f64) -> ActuatorDemand {
        ActuatorDemand {
            tvc_extension_mm: tvc,
            rcs_duty: fans,
            main_throttle: throttle,
        }
    }

    fn parse(json: &str) -> Result<BenchConfig, String> {
        let config: BenchConfig = serde_json::from_str(json).map_err(|e| e.to_string())?;
        config.check()?;
        Ok(config)
    }

    #[test]
    fn defaults_cap_everything_at_70_percent() {
        let config = BenchConfig::load(None).unwrap();
        assert_eq!(config.caps.throttle, Some(0.7));
        let (out, acted) =
            config
                .caps
                .apply(&demand([13.45; 2], [0.0; 4], 0.75), [13.45; 2], [26.9; 2]);
        assert_eq!(out.main_throttle, 0.7);
        assert_eq!(acted, CAP_THROTTLE);
        assert!(config.describe()[0].contains("throttle 70% of full"));
    }

    #[test]
    fn fans_and_tvc_capped_at_70_percent() {
        let caps = Caps::default();
        // center 10 mm on a 26.9 mm stroke: 7 mm below, 11.83 mm above allowed
        let (out, acted) = caps.apply(
            &demand([0.0, 26.9], [0.9, 0.0, 0.9, 0.5], 0.5),
            [10.0; 2],
            [26.9; 2],
        );
        assert_eq!(out.rcs_duty, [0.7, 0.0, 0.7, 0.5]);
        assert!((out.tvc_extension_mm[0] - 3.0).abs() < 1e-12);
        assert!((out.tvc_extension_mm[1] - (10.0 + 0.7 * 16.9)).abs() < 1e-12);
        assert_eq!(acted, CAP_FAN | CAP_TVC);
    }

    #[test]
    fn partial_file_keeps_the_other_defaults() {
        let config =
            parse(r#"{"caps": {"throttle": null}, "edf": {"full_thrust_n": 30.0}}"#).unwrap();
        assert_eq!(config.caps.throttle, None);
        assert_eq!(config.caps.fan_duty, 0.7);
        assert_eq!(config.edf.full_thrust_n, 30.0);
        assert_eq!(config.rcs, Rcs::default());
        assert!(config.describe()[0].contains("throttle uncapped"));
    }

    #[test]
    fn values_reach_the_allocation() {
        let config = parse(
            r#"{"edf": {"full_thrust_n": 20.0}, "rcs": {"fan_thrust_n": 1.0, "arm_m": 0.1}}"#,
        )
        .unwrap();
        let alloc = config.allocation();
        let d = alloc.demand(&common::comm::ctv::ControlVector {
            thrust: 10.0,
            tvc_pitch: 0.0,
            tvc_yaw: 0.0,
            rcs_torque: 0.05,
        });
        assert!((d.main_throttle - 0.5).abs() < 1e-12);
        assert!((d.rcs_duty[0] - 0.5).abs() < 1e-12);
    }

    #[test]
    fn bad_values_are_rejected_by_name() {
        let cases = [
            (r#"{"caps": {"fan_duty": 1.5}}"#, "caps.fan_duty"),
            (r#"{"caps": {"throttle": -0.1}}"#, "caps.throttle"),
            (r#"{"edf": {"full_thrust_n": 0}}"#, "edf.full_thrust_n"),
            (r#"{"rcs": {"arm_m": -1}}"#, "rcs.arm_m"),
            (r#"{"tvc": {"max_angle_deg": 60}}"#, "tvc.max_angle_deg"),
            (
                r#"{"tvc": {"pitch": {"centered_length_m": 0.2}}}"#,
                "tvc.pitch.centered_length_m",
            ),
            (r#"{"tvc": {"yaw": {"r_m": 0.5}}}"#, "tvc.yaw"),
        ];
        for (json, field) in cases {
            let e = parse(json).unwrap_err();
            assert!(e.contains(field), "{json}: {e}");
        }
    }

    #[test]
    fn unknown_fields_are_rejected() {
        assert!(parse(r#"{"capz": {}}"#).is_err());
        assert!(parse(r#"{"caps": {"thrust": 0.5}}"#).is_err());
    }

    #[test]
    fn missing_file_is_an_error() {
        let dir = tempfile::tempdir().unwrap();
        assert!(BenchConfig::load(Some(&dir.path().join("nope.json"))).is_err());
    }
}
