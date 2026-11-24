//! Sensor models for the software-in-the-loop run: what the IMU, barometer,
//! magnetometer and GPS would read for the plant's truth state.
//!
//! The plant gives truth (position, velocity, body rates, attitude), and
//! nothing a sensor would measure. [`SensorModel`] turns each truth state into
//! the samples the flight computer's own drivers would hand to the navigation
//! filter, at realistic rates and with seeded noise, so the GMEKF in the loop
//! has to do the work the vehicle's sensors give it: level on the pad, find
//! heading from the magnetometer, wait for a GPS fix, and track a flight.
//!
//! It is a stand-in. Missing on purpose: IMU bias and scale error, vibration,
//! magnetometer hard and soft iron, GPS multipath and time-correlated error,
//! barometer lag, LiDAR (its mounts are not measured), and any sensor fault.
//! Fault injection belongs in the bridge that sends these.

use std::f64::consts::PI;

use common::comm::GpsState;
use nalgebra::{Quaternion, UnitQuaternion, Vector3};
use serde::{Deserialize, Serialize};

use crate::sil::State;

/// One sample as it goes from the bridge to the flight computer.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub enum SilSample {
    /// Specific force (m/s^2, body axes) and angular rate (deg/s, body axes).
    Imu { accel: [f64; 3], gyro_dps: [f64; 3] },
    /// Static pressure (Pa) and temperature (deg C).
    Baro {
        pressure_pa: f64,
        temperature_c: f64,
    },
    /// Magnetic field in body axes, Gauss.
    Mag { gauss: [f64; 3] },
    /// A GPS solution.
    Gps(GpsState),
    /// One LiDAR's range along its beam, m. `id` is 1 to 4.
    Lidar { id: u8, range_m: f64 },
    /// The plant's true state (position NED, velocity NED, body rates,
    /// quaternion), for running the flight logic on truth instead of the
    /// navigation filter's estimate. A flight has no such thing; this is for
    /// telling a navigation problem from the rest of the loop.
    Truth { x: [f64; 13] },
}

#[cfg(test)]
impl SilSample {
    fn encode(&self) -> Vec<u8> {
        postcard::to_allocvec(self).expect("a sample always serializes")
    }

    fn decode(bytes: &[u8]) -> Option<SilSample> {
        postcard::from_bytes(bytes).ok()
    }
}

/// A sample with the time it was taken, bridge time in seconds. The flight
/// computer places it on its own clock from that time, not from when the
/// datagram arrived, so a late or bunched-up datagram does not change when the
/// filter thinks the sample was read, as a driver's `read_at` does not.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Stamped {
    pub t_s: f64,
    pub sample: SilSample,
}

impl Stamped {
    /// The wire form.
    pub fn encode(&self) -> Vec<u8> {
        postcard::to_allocvec(self).expect("a sample always serializes")
    }

    pub fn decode(bytes: &[u8]) -> Option<Stamped> {
        postcard::from_bytes(bytes).ok()
    }
}

/// How the sensors behave. The defaults follow the CTV's parts and the
/// noise the navigation filter assumes (`GmekfConfig::default`), a little
/// below it.
#[derive(Debug, Clone, PartialEq)]
pub struct SensorConfig {
    pub imu_hz: f64,
    pub baro_hz: f64,
    pub mag_hz: f64,
    pub gps_hz: f64,
    /// The GPS has no fix until this long after the start, s.
    pub gps_fix_after_s: f64,
    /// Accelerometer noise per sample, m/s^2.
    pub accel_std: f64,
    /// Gyro noise per sample, deg/s.
    pub gyro_std_dps: f64,
    /// Barometer noise as an altitude, m.
    pub baro_alt_std_m: f64,
    /// Magnetometer noise per axis, Gauss.
    pub mag_std_gauss: f64,
    /// GPS position noise per axis (N, E, D), m. The receiver measured on the
    /// CTV (static, no RTK).
    pub gps_position_std: [f64; 3],
    pub gps_velocity_std: f64,
    /// Where the GPS antenna is on the vehicle, body axes, m. The GPS reports
    /// the antenna's position, not the vehicle's: a filter that is told this
    /// lever arm expects it, and a model that leaves it out puts the filter's
    /// origin half a meter off.
    pub gps_antenna_body: [f64; 3],
    /// Added to the state's position to get the position the receiver is
    /// measured against, NED, m. Zero when the state is the vehicle's own
    /// position. Hopper's harness reports the controller's position as the CG's
    /// plus a fixed offset (`sim_pos_offset`, not rotated with the vehicle), and
    /// its GPS is the CG's real position plus the antenna's lever: a pad model
    /// that leaves the offset out sits that far from the plant's GPS, and a
    /// filter whose GPS origin was set on the pad then rejects every fix in flight.
    pub gps_position_offset: [f64; 3],
    pub pad_latitude_deg: f64,
    pub pad_longitude_deg: f64,
    /// Pad altitude above sea level, m.
    pub pad_altitude_m: f64,
    pub gravity: f64,
    /// Earth's field at the pad in NED, Gauss (Atlanta).
    pub mag_field_ned: [f64; 3],
    pub temperature_c: f64,
    pub seed: u64,
}

impl Default for SensorConfig {
    fn default() -> Self {
        SensorConfig {
            imu_hz: 200.0,
            baro_hz: 50.0,
            mag_hz: 10.0,
            gps_hz: 10.0,
            gps_fix_after_s: 6.0,
            accel_std: 0.05,
            gyro_std_dps: 0.03,
            baro_alt_std_m: 0.3,
            mag_std_gauss: 0.003,
            gps_position_std: [1.5, 1.6, 4.43],
            gps_velocity_std: 0.1,
            gps_antenna_body: [0.0; 3],
            gps_position_offset: [0.0; 3],
            pad_latitude_deg: 33.7756,
            pad_longitude_deg: -84.3963,
            pad_altitude_m: 320.0,
            gravity: 9.795,
            mag_field_ned: crate::shadow_nav::ShadowNavConfig::default().mag_field_ned,
            temperature_c: 20.0,
            seed: 1,
        }
    }
}

/// Meters per degree of latitude, near enough for a few meters of flight.
const M_PER_DEG: f64 = 111_320.0;

/// A small seeded generator: xorshift64* with Box-Muller for the noise.
#[derive(Debug, Clone, PartialEq)]
pub struct Rng {
    state: u64,
    spare: Option<f64>,
}

impl Rng {
    pub fn new(seed: u64) -> Self {
        Rng {
            // never zero
            state: seed.wrapping_mul(0x9E37_79B9_7F4A_7C15) | 1,
            spare: None,
        }
    }

    fn next_u64(&mut self) -> u64 {
        let mut x = self.state;

        x ^= x >> 12;
        x ^= x << 25;
        x ^= x >> 27;
        self.state = x;

        x.wrapping_mul(0x2545_F491_4F6C_DD1D)
    }

    /// Uniform in (0, 1).
    fn uniform(&mut self) -> f64 {
        ((self.next_u64() >> 11) as f64 + 0.5) / (1u64 << 53) as f64
    }

    /// Standard normal.
    pub fn gauss(&mut self) -> f64 {
        if let Some(v) = self.spare.take() {
            return v;
        }

        let (u, v) = (self.uniform(), self.uniform());
        let r = (-2.0 * u.ln()).sqrt();

        self.spare = Some(r * (2.0 * PI * v).sin());

        r * (2.0 * PI * v).cos()
    }
}

/// Truth in, samples out.
#[derive(Debug, Clone)]
pub struct SensorModel {
    config: SensorConfig,
    rng: Rng,
    previous: Option<(f64, Vector3<f64>)>,
    next_imu: f64,
    next_baro: f64,
    next_mag: f64,
    next_gps: f64,
    fixes: i64,
}

impl SensorModel {
    pub fn new(config: SensorConfig) -> Self {
        let rng = Rng::new(config.seed);

        SensorModel {
            config,
            rng,
            previous: None,
            next_imu: 0.0,
            next_baro: 0.0,
            next_mag: 0.0,
            next_gps: 0.0,
            fixes: 0,
        }
    }

    /// A model that starts at time `t` instead of 0: its first samples are due
    /// then, not as a burst to catch up from 0.
    #[cfg_attr(not(feature = "sim-plant"), allow(dead_code))]
    pub fn starting_at(config: SensorConfig, t: f64) -> Self {
        let mut model = SensorModel::new(config);

        model.next_imu = t;
        model.next_baro = t;
        model.next_mag = t;
        model.next_gps = t;
        model
    }

    /// The samples due at plant time `t` for the truth state `x`. Call it on
    /// every plant step: the acceleration the IMU feels comes from how the
    /// velocity changed since the last call.
    pub fn step(&mut self, t: f64, x: &State) -> Vec<SilSample> {
        let velocity = Vector3::new(x[3], x[4], x[5]);
        let acceleration = match self.previous {
            Some((t0, v0)) if t > t0 => (velocity - v0) / (t - t0),
            _ => Vector3::zeros(),
        };

        self.previous = Some((t, velocity));

        let attitude = UnitQuaternion::from_quaternion(Quaternion::new(x[9], x[10], x[11], x[12]));
        let to_body = attitude.inverse();
        let mut out = Vec::new();

        if t >= self.next_imu {
            self.next_imu += 1.0 / self.config.imu_hz;

            let gravity = Vector3::new(0.0, 0.0, self.config.gravity);
            let force = to_body * (acceleration - gravity);
            let noise = |rng: &mut Rng, std: f64| rng.gauss() * std;
            let c = &self.config;
            let (a, g) = (c.accel_std, c.gyro_std_dps);
            let rates = [x[6], x[7], x[8]].map(f64::to_degrees);

            out.push(SilSample::Imu {
                accel: [
                    force.x + noise(&mut self.rng, a),
                    force.y + noise(&mut self.rng, a),
                    force.z + noise(&mut self.rng, a),
                ],
                gyro_dps: [
                    rates[0] + noise(&mut self.rng, g),
                    rates[1] + noise(&mut self.rng, g),
                    rates[2] + noise(&mut self.rng, g),
                ],
            });
        }

        if t >= self.next_baro {
            self.next_baro += 1.0 / self.config.baro_hz;

            let altitude =
                self.config.pad_altitude_m - x[2] + self.rng.gauss() * self.config.baro_alt_std_m;

            out.push(SilSample::Baro {
                pressure_pa: pressure_at(altitude),
                temperature_c: self.config.temperature_c,
            });
        }

        if t >= self.next_mag {
            self.next_mag += 1.0 / self.config.mag_hz;

            let field = to_body * Vector3::from(self.config.mag_field_ned);
            let s = self.config.mag_std_gauss;

            out.push(SilSample::Mag {
                gauss: [
                    field.x + self.rng.gauss() * s,
                    field.y + self.rng.gauss() * s,
                    field.z + self.rng.gauss() * s,
                ],
            });
        }

        if t >= self.next_gps {
            self.next_gps += 1.0 / self.config.gps_hz;

            if t >= self.config.gps_fix_after_s {
                self.fixes += 1;

                let [sn, se, sd] = self.config.gps_position_std;
                let c = self.config.clone();
                // the antenna, not the vehicle's reference point
                let antenna = attitude * Vector3::from(c.gps_antenna_body);
                let offset = c.gps_position_offset;
                let north = x[0] + offset[0] + antenna.x + self.rng.gauss() * sn;
                let east = x[1] + offset[1] + antenna.y + self.rng.gauss() * se;
                let down = x[2] + offset[2] + antenna.z + self.rng.gauss() * sd;
                let v = c.gps_velocity_std;
                let latitude = c.pad_latitude_deg + north / M_PER_DEG;

                out.push(SilSample::Gps(GpsState {
                    latitude_deg: latitude,
                    longitude_deg: c.pad_longitude_deg
                        + east / (M_PER_DEG * latitude.to_radians().cos()),
                    altitude_m: c.pad_altitude_m - down,
                    north_mps: x[3] + self.rng.gauss() * v,
                    east_mps: x[4] + self.rng.gauss() * v,
                    down_mps: x[5] + self.rng.gauss() * v,
                    // distinct per fix, so the filter does not take one twice
                    timestamp_unix_ms: Some(1_700_000_000_000 + self.fixes * 100),
                    has_fix: true,
                    num_satellites: 12,
                }));
            }
        }

        out
    }
}

/// Static pressure at `altitude` above sea level, Pa, in the standard
/// atmosphere.
pub fn pressure_at(altitude: f64) -> f64 {
    101_325.0 * (1.0 - 2.255_77e-5 * altitude).powf(5.255_88)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn upright() -> State {
        let h = std::f64::consts::FRAC_1_SQRT_2;

        [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, h, 0.0, h, 0.0]
    }

    fn quiet() -> SensorConfig {
        SensorConfig {
            accel_std: 0.0,
            gyro_std_dps: 0.0,
            baro_alt_std_m: 0.0,
            mag_std_gauss: 0.0,
            gps_position_std: [0.0; 3],
            gps_velocity_std: 0.0,
            ..SensorConfig::default()
        }
    }

    #[test]
    fn at_rest_the_imu_reads_one_g_up_along_body_x() {
        let mut m = SensorModel::new(quiet());
        let out = m.step(0.0, &upright());

        let Some(SilSample::Imu { accel, gyro_dps }) = out
            .iter()
            .find(|s| matches!(s, SilSample::Imu { .. }))
            .cloned()
        else {
            panic!("no IMU sample at t = 0");
        };

        assert!((accel[0] - 9.795).abs() < 1e-9, "{accel:?}");
        assert!(accel[1].abs() < 1e-9 && accel[2].abs() < 1e-9, "{accel:?}");
        assert_eq!(gyro_dps, [0.0; 3]);
    }

    #[test]
    fn in_free_fall_the_accelerometer_reads_nothing() {
        let mut m = SensorModel::new(quiet());
        let mut x = upright();

        m.step(0.0, &x);

        // falling: velocity down grows by g
        x[5] = 9.795 * 0.005;

        let out = m.step(0.005, &x);
        let Some(SilSample::Imu { accel, .. }) =
            out.into_iter().find(|s| matches!(s, SilSample::Imu { .. }))
        else {
            panic!("no IMU sample");
        };

        assert!(accel.iter().all(|a| a.abs() < 1e-6), "{accel:?}");
    }

    #[test]
    fn climbing_against_gravity_adds_to_the_force() {
        let mut m = SensorModel::new(quiet());
        let mut x = upright();

        m.step(0.0, &x);

        // accelerating up at 2 m/s^2: velocity down becomes more negative
        x[5] = -2.0 * 0.005;

        let Some(SilSample::Imu { accel, .. }) = m
            .step(0.005, &x)
            .into_iter()
            .find(|s| matches!(s, SilSample::Imu { .. }))
        else {
            panic!("no IMU sample");
        };

        assert!((accel[0] - (9.795 + 2.0)).abs() < 1e-6, "{accel:?}");
    }

    #[test]
    fn the_rates_come_out_in_degrees_per_second() {
        let mut m = SensorModel::new(quiet());
        let mut x = upright();

        x[6] = 0.1;
        x[8] = -0.2;

        let Some(SilSample::Imu { gyro_dps, .. }) = m
            .step(0.0, &x)
            .into_iter()
            .find(|s| matches!(s, SilSample::Imu { .. }))
        else {
            panic!();
        };

        assert!((gyro_dps[0] - 0.1f64.to_degrees()).abs() < 1e-9);
        assert!((gyro_dps[2] + 0.2f64.to_degrees()).abs() < 1e-9);
    }

    #[test]
    fn the_magnetometer_sees_the_earths_field_in_body_axes() {
        let mut m = SensorModel::new(quiet());
        let out = m.step(0.0, &upright());
        let Some(SilSample::Mag { gauss }) =
            out.into_iter().find(|s| matches!(s, SilSample::Mag { .. }))
        else {
            panic!();
        };
        let field = SensorConfig::default().mag_field_ned;
        let norm = |v: [f64; 3]| (v[0] * v[0] + v[1] * v[1] + v[2] * v[2]).sqrt();

        // a rotation keeps the length, and puts the downward field on body -x
        assert!((norm(gauss) - norm(field)).abs() < 1e-9);
        assert!((gauss[0] + field[2]).abs() < 1e-9, "{gauss:?} {field:?}");
    }

    #[test]
    fn the_pressure_falls_with_height() {
        let mut m = SensorModel::new(quiet());
        let pressure = |m: &mut SensorModel, t: f64, z: f64| {
            let mut x = upright();

            x[2] = z;

            m.step(t, &x).into_iter().find_map(|s| match s {
                SilSample::Baro { pressure_pa, .. } => Some(pressure_pa),
                _ => None,
            })
        };

        let low = pressure(&mut m, 0.0, 0.0).unwrap();
        let high = pressure(&mut m, 0.5, -10.0).unwrap();

        // about 12 Pa per meter near the ground
        assert!(low - high > 100.0 && low - high < 140.0, "{low} {high}");
        assert!((low - pressure_at(320.0)).abs() < 1e-6);
    }

    #[test]
    fn the_gps_has_no_fix_until_its_time_and_then_reports_the_position() {
        let mut m = SensorModel::new(quiet());
        let gps_at = |m: &mut SensorModel, t: f64, x: &State| {
            m.step(t, x).into_iter().find_map(|s| match s {
                SilSample::Gps(g) => Some(g),
                _ => None,
            })
        };

        assert!(gps_at(&mut m, 0.0, &upright()).is_none());
        assert!(gps_at(&mut m, 5.9, &upright()).is_none());

        let mut x = upright();

        x[0] = 111.32; // 111 m north is a thousandth of a degree
        x[2] = -4.5;

        let fix = gps_at(&mut m, 6.0, &x).expect("a fix at 6 s");

        assert!(fix.has_fix && fix.num_satellites >= 6);
        assert!((fix.latitude_deg - (33.7756 + 0.001)).abs() < 1e-9);
        assert!((fix.altitude_m - 324.5).abs() < 1e-9);

        let next = gps_at(&mut m, 6.1, &x).expect("another 0.1 s later");

        assert_ne!(fix.timestamp_unix_ms, next.timestamp_unix_ms);
    }

    #[test]
    fn the_gps_reports_the_antenna_not_the_vehicle() {
        // antenna 0.55 m up the vehicle axis (body x, up on the pad)
        let mut m = SensorModel::new(SensorConfig {
            gps_antenna_body: [0.55, 0.0, 0.1],
            gps_fix_after_s: 0.0,
            ..quiet()
        });
        let fix = m
            .step(0.0, &upright())
            .into_iter()
            .find_map(|s| match s {
                SilSample::Gps(g) => Some(g),
                _ => None,
            })
            .unwrap();

        // 0.55 m above the pad (the altitude is the pad's plus that)
        assert!(
            (fix.altitude_m - (320.0 + 0.55)).abs() < 1e-9,
            "{}",
            fix.altitude_m
        );
        // body z is horizontal when body x is up; with this attitude it is north or south
        assert!(((fix.latitude_deg - 33.7756) * 111_320.0).abs() - 0.1 < 1e-6);
    }

    #[test]
    fn the_gps_position_offset_moves_the_fix() {
        // the harness's CG offset: the receiver is measured 0.35 m above the state's
        // position, whatever the antenna is
        let mut m = SensorModel::new(SensorConfig {
            gps_position_offset: [0.0, 0.0, -0.35],
            gps_fix_after_s: 0.0,
            ..quiet()
        });
        let fix = m
            .step(0.0, &upright())
            .into_iter()
            .find_map(|s| match s {
                SilSample::Gps(g) => Some(g),
                _ => None,
            })
            .unwrap();

        assert!(
            (fix.altitude_m - (320.0 + 0.35)).abs() < 1e-9,
            "{}",
            fix.altitude_m
        );
    }

    #[test]
    fn rates_are_the_configured_ones() {
        let mut m = SensorModel::new(quiet());
        let x = upright();
        let (mut imu, mut baro, mut mag, mut gps) = (0, 0, 0, 0);

        // 10 s of 1 kHz stepping; the GPS has a fix from 6 s on
        for k in 0..=10_000 {
            for s in m.step(f64::from(k) * 1e-3, &x) {
                match s {
                    SilSample::Imu { .. } => imu += 1,
                    SilSample::Baro { .. } => baro += 1,
                    SilSample::Mag { .. } => mag += 1,
                    SilSample::Gps(_) => gps += 1,
                    SilSample::Lidar { .. } | SilSample::Truth { .. } => {}
                }
            }
        }

        assert!((2000..=2001).contains(&imu), "{imu}");
        assert!((500..=501).contains(&baro), "{baro}");
        assert!((100..=101).contains(&mag), "{mag}");
        assert!((40..=41).contains(&gps), "{gps}");
    }

    #[test]
    fn the_noise_is_seeded_and_about_the_right_size() {
        let run = |seed| {
            let mut m = SensorModel::new(SensorConfig {
                seed,
                ..SensorConfig::default()
            });
            let x = upright();
            let mut samples = Vec::new();

            for k in 0..20_000 {
                for s in m.step(f64::from(k) * 5e-3, &x) {
                    if let SilSample::Imu { accel, .. } = s {
                        samples.push(accel[1]);
                    }
                }
            }

            samples
        };
        let a = run(7);
        let b = run(7);
        let c = run(8);

        assert_eq!(a, b);
        assert_ne!(a, c);

        let mean = a.iter().sum::<f64>() / a.len() as f64;
        let var = a.iter().map(|v| (v - mean).powi(2)).sum::<f64>() / a.len() as f64;

        assert!(mean.abs() < 0.005, "{mean}");
        assert!((var.sqrt() - 0.05).abs() < 0.005, "{}", var.sqrt());
    }

    #[test]
    fn a_stamped_sample_survives_the_wire() {
        let sample = Stamped {
            t_s: 12.345,
            sample: SilSample::Mag {
                gauss: [0.1, 0.2, 0.3],
            },
        };

        assert_eq!(Stamped::decode(&sample.encode()), Some(sample));
        assert_eq!(Stamped::decode(&[0xFF, 0xFF]), None);
    }

    #[test]
    fn a_sample_survives_the_wire() {
        for sample in [
            SilSample::Imu {
                accel: [1.0, 2.0, 3.0],
                gyro_dps: [0.1, 0.2, 0.3],
            },
            SilSample::Baro {
                pressure_pa: 97_000.0,
                temperature_c: 20.0,
            },
            SilSample::Mag {
                gauss: [0.1, 0.2, 0.3],
            },
        ] {
            assert_eq!(SilSample::decode(&sample.encode()), Some(sample));
        }

        assert_eq!(SilSample::decode(&[0xFF, 0xFF, 0xFF]), None);
    }
}
