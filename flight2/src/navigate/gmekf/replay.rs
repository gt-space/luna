//! Replays hopper's recorded CTV flight (`fsw_bridge/build/nav_replay/ctv`) through
//! this GMEKF, and scores it against the truth and against the estimate the
//! MATLAB filter gave for the same sensor readings.
//!
//! Needs the files, so it only runs on request:
//!
//! ```text
//! HOPPER_NAV_REPLAY=<folder> cargo test -p ctv-flight-computer --release \
//!     gmekf::replay -- --ignored --nocapture
//! ```

use std::{path::PathBuf, time::Duration};

use nalgebra::{Matrix3, Quaternion, UnitQuaternion, Vector3};
use serde_json::Value;

use super::{Gmekf, GmekfConfig, LidarMount};
use crate::navigate::{Estimator, ImuSample, Measurement, NavState, Timestamped};

/// The recorded flight.
struct Replay {
    /// Column index by name.
    columns: std::collections::HashMap<String, usize>,
    rows: Vec<Vec<f64>>,
    params: Value,
}

impl Replay {
    fn load() -> Option<Replay> {
        let dir = PathBuf::from(std::env::var("HOPPER_NAV_REPLAY").ok()?);
        let text = std::fs::read_to_string(dir.join("nav_replay.csv")).ok()?;
        let mut lines = text.lines();
        let columns = lines
            .next()?
            .split(',')
            .enumerate()
            .map(|(i, name)| (name.trim().to_string(), i))
            .collect();
        let rows = lines
            .map(|line| line.split(',').map(|v| v.parse().unwrap()).collect())
            .collect();
        let params =
            serde_json::from_str(&std::fs::read_to_string(dir.join("nav_replay.json")).ok()?)
                .ok()?;

        Some(Replay {
            columns,
            rows,
            params,
        })
    }

    fn at(&self, row: usize, name: &str) -> f64 {
        self.rows[row][self.columns[name]]
    }

    fn vec3(&self, row: usize, prefix: &str, first: usize) -> Vector3<f64> {
        Vector3::new(
            self.at(row, &format!("{prefix}{first}")),
            self.at(row, &format!("{prefix}{}", first + 1)),
            self.at(row, &format!("{prefix}{}", first + 2)),
        )
    }

    /// A 13-state (truth or `x_nav`) as a state.
    fn state(&self, row: usize, prefix: &str) -> NavState {
        let q = |i| self.at(row, &format!("{prefix}{i}"));

        NavState {
            time: Duration::from_secs_f64(self.at(row, "t")),
            position: self.vec3(row, prefix, 1),
            velocity: self.vec3(row, prefix, 4),
            body_rate: self.vec3(row, prefix, 7),
            attitude: Quaternion::new(q(10), q(11), q(12), q(13)),
            position_std: Vector3::repeat(0.1),
            velocity_std: Vector3::repeat(0.1),
            attitude_std: Vector3::repeat(0.01),
            body_rate_std: Vector3::repeat(0.01),
        }
    }

    fn config(&self) -> GmekfConfig {
        let sensors = &self.params["sensor_params"];
        let f = |v: &Value| v.as_f64().unwrap();
        let v3 = |v: &Value| Vector3::new(f(&v[0]), f(&v[1]), f(&v[2]));
        let antenna = &sensors["gps"]["antenna"];

        GmekfConfig {
            gravity: f(&self.params["nav_g"]),
            inertia: Matrix3::from_diagonal(&Vector3::new(0.0123, 0.105, 0.105)),
            accel_std: v3(&sensors["imu"]["accel_sigma"]),
            gyro_std: v3(&sensors["imu"]["gyro_sigma"]),
            baro_std: f(&sensors["baro"]["sigma"]),
            gps_position_std: v3(&sensors["gps"]["sigma"]),
            gps_velocity_std: v3(&sensors["gps"]["vel_sigma"]),
            // hopper gives the antenna from the base; the filter wants it from the COM
            gps_lever_arm: Vector3::new(
                f(&antenna[0][0]) - f(&self.params["frames"]["heightCOM"]),
                f(&antenna[1][0]),
                f(&antenna[2][0]),
            ),
            mag_field_ned: Some(v3(&sensors["mag"]["field_ned"]) / 100.0),
            mag_std: f(&sensors["mag"]["sigma"]) / 100.0,
            accel_bias_psd: Vector3::repeat(1.8e-10),
            gyro_bias_psd: Vector3::repeat(1.54e-11),
            ..GmekfConfig::default()
        }
    }

    /// The four LiDAR beams, from hopper's geometry.
    fn lidar_mounts(&self) -> Vec<LidarMount> {
        let geom = &self.params["geom"];
        let f = |v: &Value| v.as_f64().unwrap();
        let height_com = f(&self.params["frames"]["heightCOM"]);

        (0..4)
            .map(|beam| {
                let column = |key: &str| {
                    Vector3::new(
                        f(&geom[key][0][beam]),
                        f(&geom[key][1][beam]),
                        f(&geom[key][2][beam]),
                    )
                };

                LidarMount {
                    beam_dir_body: column("lidar_dir"),
                    // hopper gives the mount from the base: the filter wants it from the COM
                    position_body: column("lidar_mount")
                        - Vector3::new(self.at(0, "cg_x"), self.at(0, "cg_y"), self.at(0, "cg_z")),
                    height_imu: height_com,
                }
            })
            .collect()
    }

    /// Runs the filter over the flight; the state at every `every`-th row.
    fn run(&self, config: GmekfConfig, every: usize) -> Vec<NavState> {
        let mut filter = Gmekf::new(config);
        let mut initial = self.state(0, "truth_");
        let mut stamps = [f64::NAN; 9];
        let mut gps_origin: Option<Vector3<f64>> = None;
        let mut out = Vec::new();

        initial.position = Vector3::zeros();
        initial.velocity = Vector3::zeros();
        initial.body_rate = Vector3::zeros();
        initial.position_std = Vector3::repeat(0.05);
        initial.velocity_std = Vector3::repeat(0.01);
        initial.attitude_std = Vector3::repeat(0.01);
        initial.body_rate_std = Vector3::repeat(0.005);
        filter.initialize(&initial).unwrap();
        // the first stamps are the filter's start
        let first = |name: &str| self.at(0, name);
        for (i, stamp) in stamps.iter_mut().enumerate() {
            *stamp = first(&format!("stamp_{}", i + 1));
        }

        for row in 1..self.rows.len() {
            let fresh = |sensor: usize, stamps: &mut [f64; 9]| {
                let stamp = self.at(row, &format!("stamp_{}", sensor + 1));
                let new =
                    stamp != stamps[sensor] && self.at(row, &format!("valid_{}", sensor + 1)) > 0.5;

                stamps[sensor] = stamp;
                new.then(|| Duration::from_secs_f64(stamp))
            };
            let meas = |i: usize| self.at(row, &format!("meas_{}", i + 1));

            if let Some(time) = fresh(0, &mut stamps) {
                let imu = ImuSample {
                    accel: Vector3::new(meas(0), meas(1), meas(2)),
                    gyro: Vector3::new(meas(3), meas(4), meas(5)),
                };

                filter.predict(&Timestamped::new(time, imu)).unwrap();
            }
            if let Some(time) = fresh(1, &mut stamps) {
                let _ = filter.update_baro_height(time, meas(6));
            }
            for beam in 0..4 {
                if let Some(time) = fresh(2 + beam, &mut stamps) {
                    let measurement = Measurement::Lidar {
                        index: beam,
                        range: meas(7 + beam),
                    };

                    let _ = filter.update(&Timestamped::new(time, measurement));
                }
            }
            if let Some(time) = fresh(6, &mut stamps) {
                let fix = Vector3::new(meas(11), meas(12), meas(13));
                // the GPS origin is set from the first fix, as the flight code does: the
                // recording's is the ground, the filter's is under the vehicle at the pad
                let origin = *gps_origin.get_or_insert_with(|| {
                    let state = filter.nav_state().expect("the filter is initialized");
                    let attitude = UnitQuaternion::from_quaternion(state.attitude);

                    fix - (state.position + attitude * filter.config().gps_lever_arm)
                });
                let _ = filter.update_gps(
                    time,
                    fix - origin,
                    Some(Vector3::new(meas(20), meas(21), meas(22))),
                );
            }
            if let Some(time) = fresh(7, &mut stamps) {
                let field = Vector3::new(meas(17), meas(18), meas(19)) / 100.0;

                let _ = filter.update(&Timestamped::new(time, Measurement::Magnetometer { field }));
            }

            if row % every == 0 {
                out.push(filter.nav_state().unwrap());
            }
        }

        out
    }
}

/// Root-mean-square error of an estimate against the truth over t = 1 to 15 s:
/// position (mean removed, because the filter's position is the IMU's), velocity,
/// attitude angle and body rate, each per axis.
fn score(replay: &Replay, every: usize, estimate: impl Fn(usize) -> NavState) -> String {
    let (mut dp, mut dv, mut dr) = (Vec::new(), Vec::new(), Vec::new());
    let mut att = Vec::new();

    for row in (every..replay.rows.len()).step_by(every) {
        let t = replay.at(row, "t");

        if !(1.0..15.0).contains(&t) {
            continue;
        }

        let e = estimate(row / every - 1);
        let truth = replay.state(row, "truth_");
        let q = |n: &NavState| UnitQuaternion::from_quaternion(n.attitude);

        dp.push(e.position - truth.position);
        dv.push(e.velocity - truth.velocity);
        dr.push(e.body_rate - truth.body_rate);
        att.push((q(&truth).inverse() * q(&e)).scaled_axis());
    }

    let n = dp.len() as f64;
    let mean = |v: &[Vector3<f64>]| v.iter().sum::<Vector3<f64>>() / n;
    let rms = |v: &[Vector3<f64>]| {
        (v.iter().map(|x| x.component_mul(x)).sum::<Vector3<f64>>() / n).map(f64::sqrt)
    };
    let scatter = |v: &[Vector3<f64>]| {
        let m = mean(v);
        (v.iter()
            .map(|x| (x - m).component_mul(&(x - m)))
            .sum::<Vector3<f64>>()
            / n)
            .map(f64::sqrt)
    };
    let (p, v, r, a) = (
        scatter(&dp),
        rms(&dv),
        rms(&dr),
        scatter(&att).map(f64::to_degrees),
    );
    let a_mean = mean(&att).map(f64::to_degrees);
    let r_mean = mean(&dr);
    let truth_rate: Vec<Vector3<f64>> = (every..replay.rows.len())
        .step_by(every)
        .filter(|row| (1.0..15.0).contains(&replay.at(*row, "t")))
        .map(|row| replay.state(row, "truth_").body_rate)
        .collect();
    let t_rms = rms(&truth_rate);

    format!(
        "pos scatter [{:.3} {:.3} {:.3}] m | vel rms [{:.3} {:.3} {:.3}] m/s | att scatter [{:.2} {:.2} {:.2}] deg, mean [{:.2} {:.2} {:.2}] | rate rms [{:.4} {:.4} {:.4}] (mean [{:.4} {:.4} {:.4}], truth rms [{:.4} {:.4} {:.4}])",
        p.x, p.y, p.z, v.x, v.y, v.z, a.x, a.y, a.z, a_mean.x, a_mean.y, a_mean.z, r.x, r.y, r.z, r_mean.x, r_mean.y, r_mean.z, t_rms.x, t_rms.y, t_rms.z
    )
}

#[test]
#[ignore = "needs hopper's recorded flight: set HOPPER_NAV_REPLAY"]
fn replays_hoppers_flight() {
    let Some(replay) = Replay::load() else {
        panic!("set HOPPER_NAV_REPLAY to the nav_replay/ctv folder");
    };
    let every = 10;
    let hopper: Vec<NavState> = (every..replay.rows.len())
        .step_by(every)
        .map(|row| replay.state(row, "x_nav_"))
        .collect();

    println!("hopper's x_nav : {}", score(&replay, every, |i| hopper[i]));

    let base = replay.config();
    let configs: Vec<(&str, GmekfConfig)> = vec![
        ("luna, as nav.json", base.clone()),
        (
            "luna + lidar",
            GmekfConfig {
                lidar_mounts: replay.lidar_mounts(),
                ..base.clone()
            },
        ),
        (
            "luna + lidar to 6 m",
            GmekfConfig {
                lidar_mounts: replay.lidar_mounts(),
                lidar_max_range: 6.0,
                ..base.clone()
            },
        ),
    ];

    for (name, config) in configs {
        let states = replay.run(config, every);

        println!("{name:<36}: {}", score(&replay, every, |i| states[i]));
    }
}
