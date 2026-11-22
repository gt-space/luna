//! GMEKF tests.
//!
//! `testdata/*.json` come from `testdata/make_reference.m`, which runs the
//! MATLAB filter functions themselves (hopper `rev_CTV_gmekf`, ed077f9):
//! - `unit_cases.json`: random inputs and outputs of each function.
//! - `sequence.json`: 4 s of a simulated tethered vehicle at 208 Hz through
//!   `gmekf_predict` (driven by the gyro) + `gmekf_correct_ctv`, with barometer
//!   and GPS at about 21 Hz, four LiDAR beams at about 10 Hz, the magnetometer
//!   at about 10 Hz, and outliers in the barometer, LiDAR and magnetometer.

use std::{sync::OnceLock, time::Duration};

use nalgebra::{Matrix3, Quaternion, SMatrix, SVector, Vector3, Vector4};
use serde_json::Value;

use super::{math::*, Gmekf, GmekfConfig, LidarMount, DIAG_LIDAR, LIDAR_GATE};
use crate::navigate::{
    kalman::G0,
    testing::{self, Scenario, SensorNoise},
    Estimator, ImuSample, Measurement, NavState, Timestamped,
};

// ---------------------------------------------------------------------------
// Reference data

fn unit_cases() -> &'static Value {
    static V: OnceLock<Value> = OnceLock::new();
    V.get_or_init(|| serde_json::from_str(include_str!("testdata/unit_cases.json")).unwrap())
}

fn sequence() -> &'static Value {
    static V: OnceLock<Value> = OnceLock::new();
    V.get_or_init(|| serde_json::from_str(include_str!("testdata/sequence.json")).unwrap())
}

/// A numeric field, flat in MATLAB's column-major order.
fn nums(v: &Value, key: &str) -> Vec<f64> {
    v[key]
        .as_array()
        .unwrap_or_else(|| panic!("missing field {key}"))
        .iter()
        .map(|n| n.as_f64().unwrap())
        .collect()
}

fn num(v: &Value, key: &str) -> f64 {
    let n = nums(v, key);
    assert_eq!(n.len(), 1, "{key} is not a scalar");
    n[0]
}

fn mat<const R: usize, const C: usize>(v: &Value, key: &str) -> SMatrix<f64, R, C> {
    let n = nums(v, key);
    assert_eq!(
        n.len(),
        R * C,
        "{key} has {} values, expected {R}x{C}",
        n.len()
    );
    SMatrix::from_column_slice(&n)
}

fn vec<const R: usize>(v: &Value, key: &str) -> SVector<f64, R> {
    mat::<R, 1>(v, key)
}

fn cases<'a>(group: &'a Value, name: &str) -> &'a Vec<Value> {
    group[name].as_array().unwrap()
}

/// Asserts `|actual - expected| <= tol * (1 + max|expected|)` elementwise.
fn assert_close<const R: usize, const C: usize>(
    actual: &SMatrix<f64, R, C>,
    expected: &SMatrix<f64, R, C>,
    tol: f64,
    what: &str,
) {
    let scale = 1.0 + expected.amax();
    let err = (actual - expected).amax();
    assert!(
        err <= tol * scale,
        "{what}: max difference {err:e} (scale {scale:e}, tolerance {tol:e})"
    );
}

fn dynamics(case: &Value) -> Dynamics {
    Dynamics::new(mat(case, "J"), num(case, "g")).unwrap()
}

const TIGHT: f64 = 1e-12;

// ---------------------------------------------------------------------------
// Each function against MATLAB

#[test]
fn quaternion_helpers_match_matlab() {
    for c in cases(unit_cases(), "quaternion") {
        let (q1, q2): (Vector4<f64>, Vector4<f64>) = (vec(c, "q1"), vec(c, "q2"));
        assert_close(&qmult(&q1, &q2), &vec(c, "product"), TIGHT, "qmult");
        assert_close(
            &quaternion_to_rotation(&q1),
            &mat(c, "rotation"),
            TIGHT,
            "rotation",
        );
        assert_close(&psi(&q1), &mat(c, "psi"), TIGHT, "Psi_helper");
    }
}

#[test]
fn rotation_matches_nalgebra_attitude_convention() {
    // The GMEKF quaternion is luna's ControlState::attitude: the same
    // components give the same body-to-NED matrix as nalgebra.
    for c in cases(unit_cases(), "quaternion") {
        let q: Vector4<f64> = vec(c, "q1");
        let unit =
            nalgebra::UnitQuaternion::from_quaternion(Quaternion::new(q[0], q[1], q[2], q[3]));
        assert_close(
            &quaternion_to_rotation(&q),
            &unit.to_rotation_matrix().into_inner(),
            TIGHT,
            "rotation vs nalgebra",
        );
    }
}

#[test]
fn propagate_state_matches_matlab() {
    for c in cases(unit_cases(), "propagate_state") {
        let xn: StateVector = vec(c, "xn");
        let a_ib = quaternion_to_rotation(&xn.fixed_rows::<4>(QUAT).into_owned());
        let out = propagate_state(
            &xn,
            &a_ib,
            &vec(c, "accel"),
            &vec(c, "gyro"),
            num(c, "validity") != 0.0,
            &dynamics(c),
            num(c, "dt"),
        );
        assert_close(&out, &vec(c, "xnp1"), TIGHT, "propagate_state");
    }
}

#[test]
fn error_state_jacobian_matches_matlab() {
    for c in cases(unit_cases(), "error_state_jacobian") {
        let half = HalfStep {
            a_ib: mat(c, "A_IB"),
            a_ib05: mat(c, "A_IB05"),
            a_i: vec(c, "a_I"),
            w_i: vec(c, "w_I"),
        };
        let (f, g) = error_state_jacobian(
            &vec(c, "x"),
            &half,
            num(c, "dt"),
            num(c, "validity") != 0.0,
            &dynamics_j(c),
        );
        assert_close(&f, &mat(c, "F"), TIGHT, "F");
        assert_close(&g, &mat(c, "G"), TIGHT, "G");
    }
}

fn dynamics_j(case: &Value) -> Dynamics {
    Dynamics::new(mat(case, "J"), 9.81).unwrap()
}

#[test]
fn propagate_covariance_matches_matlab() {
    for c in cases(unit_cases(), "propagate_covariance") {
        let out = propagate_covariance(
            &mat(c, "P"),
            &mat(c, "F"),
            &mat(c, "G"),
            &mat(c, "Qc"),
            num(c, "dt"),
        );
        assert_close(&out, &mat(c, "Pnp1"), TIGHT, "propagate_covariance");
    }
}

#[test]
fn gmekf_predict_matches_matlab() {
    for c in cases(unit_cases(), "gmekf_predict") {
        let (x, p) = gmekf_predict(
            &vec(c, "xn"),
            &mat(c, "Pn"),
            &vec(c, "accel"),
            &vec(c, "gyro"),
            num(c, "validity") != 0.0,
            &dynamics(c),
            num(c, "dt"),
            &mat(c, "Qc"),
        );
        assert_close(&x, &vec(c, "xnp1"), TIGHT, "gmekf_predict x");
        assert_close(&p, &mat(c, "Pnp1"), TIGHT, "gmekf_predict P");
    }
}

fn kalman_case<const M: usize>(c: &Value) {
    let k = kalman_measurement_update::<M>(
        &mat(c, "P"),
        &vec(c, "innovation"),
        &mat(c, "H"),
        &mat(c, "R"),
        num(c, "gate"),
    );
    assert_eq!(k.accepted, num(c, "accepted") != 0.0, "accepted");
    let nis = num(c, "nis");
    assert!(
        (k.nis - nis).abs() <= 1e-9 * (1.0 + nis.abs()),
        "nis {} vs {nis}",
        k.nis
    );
    assert_close(&k.dx, &vec(c, "dx"), 1e-10, "dx");
    assert_close(&k.p, &mat(c, "P_post"), 1e-10, "P_post");
}

#[test]
fn kalman_measurement_update_matches_matlab() {
    let mut rejected = 0;
    for c in cases(unit_cases(), "kalman_measurement_update") {
        rejected += (num(c, "accepted") == 0.0) as usize;
        match num(c, "m") as usize {
            1 => kalman_case::<1>(c),
            2 => kalman_case::<2>(c),
            3 => kalman_case::<3>(c),
            4 => kalman_case::<4>(c),
            5 => kalman_case::<5>(c),
            6 => kalman_case::<6>(c),
            m => panic!("unexpected measurement size {m}"),
        }
    }
    assert!(rejected > 0, "reference data has no gated-out case");
}

#[test]
fn state_injection_and_reset_match_matlab() {
    for c in cases(unit_cases(), "state_injection") {
        let (x, dx): (StateVector, ErrorVector) = (vec(c, "x"), vec(c, "dx"));
        assert_close(
            &state_injection(&x, &dx),
            &vec(c, "x_post"),
            TIGHT,
            "state_injection",
        );
        assert_close(
            &compute_error_reset_jacobian(&dx),
            &mat(c, "reset_jacobian"),
            TIGHT,
            "reset jacobian",
        );
    }
}

#[test]
fn sensor_models_match_matlab() {
    for c in cases(unit_cases(), "sensor_models") {
        let x: StateVector = vec(c, "x");

        let (r, h) = baro_model(&x, num(c, "baro_height"));
        assert_close(&r, &vec(c, "baro_innovation"), TIGHT, "baro innovation");
        assert_close(&h, &mat(c, "baro_H"), TIGHT, "baro H");

        let (r, h) = gyro_model(&x, &vec(c, "gyro_angvel"));
        assert_close(&r, &vec(c, "gyro_innovation"), TIGHT, "gyro innovation");
        assert_close(&h, &mat(c, "gyro_H"), TIGHT, "gyro H");

        let (r, h) = gps_model(
            &x,
            &vec(c, "gps_pos"),
            &vec(c, "gps_vel"),
            &vec(c, "gps_r_b"),
        );
        assert_close(&r, &vec(c, "gps_innovation"), TIGHT, "gps innovation");
        assert_close(&h, &mat(c, "gps_H"), TIGHT, "gps H");

        let (r, h) = mag_model(&x, &vec(c, "mag_field_body"), &vec(c, "mag_field_ned"));
        assert_close(&r, &vec(c, "mag_innovation"), TIGHT, "mag innovation");
        assert_close(&h, &mat(c, "mag_H"), TIGHT, "mag H");

        let beam: Vector3<f64> = vec(c, "lidar_beam");
        let sensor: Vector3<f64> = vec(c, "lidar_sensor");
        let height_com = num(c, "lidar_heightCOM");
        let (r, h) = lidar_model(&x, num(c, "lidar_range"), &beam, &sensor, height_com).unwrap();
        assert_close(&r, &vec(c, "lidar_innovation"), TIGHT, "LiDAR innovation");
        assert_close(&h, &mat(c, "lidar_H"), TIGHT, "LiDAR H");
    }
}

#[test]
fn lidar_model_matches_forward_geometry_and_rejects_unusable_beams() {
    let mut x = StateVector::zeros();
    x[POS + 2] = -2.0;
    x.fixed_rows_mut::<4>(QUAT)
        .copy_from(&Vector4::new(1.0, 0.0, 0.0, 0.0));

    let beam = Vector3::new(0.2, 0.1, 0.97);
    let sensor = Vector3::new(0.1, -0.2, 0.3);
    let height_com = 0.5;
    let measured_range = 1.7;
    let (innovation, h) = lidar_model(&x, measured_range, &beam, &sensor, height_com).unwrap();

    let n = Vector3::z();
    let d = n.dot(&beam);
    let predicted_range = -(n.dot(&sensor) + x[POS + 2] - height_com) / d;
    let u = sensor + predicted_range * beam;
    let expected_attitude = Vector3::z().cross(&u) / d;
    assert!((innovation[0] - (predicted_range - measured_range)).abs() < 1e-12);
    assert!((h[(0, E_POS + 2)] + 1.0 / d).abs() < 1e-12);
    assert_close(
        &h.fixed_view::<1, 3>(0, E_ATT).into_owned(),
        &expected_attitude.transpose(),
        TIGHT,
        "LiDAR attitude Jacobian",
    );
    assert_eq!(h[(0, E_POS)], 0.0);
    assert_eq!(h[(0, E_POS + 1)], 0.0);
    assert_eq!(h.fixed_view::<1, 6>(0, E_VEL).amax(), 0.0);
    assert_eq!(h.fixed_view::<1, 7>(0, E_ACCEL_BIAS).amax(), 0.0);

    assert!(lidar_model(&x, measured_range, &Vector3::x(), &sensor, height_com).is_none());

    let mut below_ground = x;
    below_ground[POS + 2] = 3.0;
    assert!(lidar_model(
        &below_ground,
        measured_range,
        &Vector3::z(),
        &sensor,
        height_com
    )
    .is_none());
}

#[test]
fn lidar_covariance_matches_brick_histogram_calculations() {
    let cases = [
        (0.25, 0.0006248388252148995),
        (0.50, 0.0006835653151205994),
        (1.00, 0.0004522540148365515),
        (2.00, 0.0003407199007033577),
        (3.50, 0.0006514607715648194),
    ];
    for (range, expected) in cases {
        let actual = lidar_covariance(range).unwrap();
        assert!((actual - expected).abs() < 1e-12);
    }
}

#[test]
fn lidar_covariance_rejects_unusable_ranges() {
    assert_eq!(lidar_covariance(0.199), None);
    assert_eq!(lidar_covariance(4.001), None);
    assert_eq!(lidar_covariance(f64::NAN), None);
    assert_eq!(lidar_covariance(f64::INFINITY), None);
    assert!(lidar_covariance(0.2).is_some());
    assert!(lidar_covariance(4.0).is_some());
}

// ---------------------------------------------------------------------------
// Whole sequence against MATLAB

struct Step {
    time: f64,
    accel: Vector3<f64>,
    gyro: Vector3<f64>,
    baro: Option<f64>,
    gps: Option<(Vector3<f64>, Vector3<f64>)>,
    mag: Option<Vector3<f64>>,
    lidar: [Option<f64>; 4],
    x: StateVector,
    p_diag: ErrorVector,
    diagnostics: [f64; 16],
}

struct Sequence {
    config: Value,
    steps: Vec<Step>,
    /// (step index, full covariance) at every `full_P_every` steps.
    full_p: Vec<(usize, Covariance)>,
}

fn load_sequence() -> Sequence {
    let seq = sequence();
    let st = &seq["steps"];
    let col = |key: &str, rows: usize| {
        let n = nums(st, key);
        move |k: usize| n[k * rows..(k + 1) * rows].to_vec()
    };
    let (t, accel, gyro) = (nums(st, "t"), col("accel", 3), col("gyro", 3));
    let (baro_new, baro_h) = (nums(st, "baro_new"), nums(st, "baro_height"));
    let (gps_new, gps_pos, gps_vel) = (nums(st, "gps_new"), col("gps_pos", 3), col("gps_vel", 3));
    let (mag_new, mag_field) = (nums(st, "mag_new"), col("mag_field", 3));
    let (lidar_new, lidar_range) = (col("lidar_new", 4), col("lidar_range", 4));
    let (x, p_diag, diag) = (col("x", 20), col("P_diag", 19), col("diagnostics", 16));

    let steps = (0..t.len())
        .map(|k| Step {
            time: t[k],
            accel: Vector3::from_column_slice(&accel(k)),
            gyro: Vector3::from_column_slice(&gyro(k)),
            baro: (baro_new[k] != 0.0).then_some(baro_h[k]),
            gps: (gps_new[k] != 0.0).then(|| {
                (
                    Vector3::from_column_slice(&gps_pos(k)),
                    Vector3::from_column_slice(&gps_vel(k)),
                )
            }),
            mag: (mag_new[k] != 0.0).then(|| Vector3::from_column_slice(&mag_field(k))),
            lidar: std::array::from_fn(|i| (lidar_new(k)[i] != 0.0).then_some(lidar_range(k)[i])),
            x: StateVector::from_column_slice(&x(k)),
            p_diag: ErrorVector::from_column_slice(&p_diag(k)),
            diagnostics: diag(k).try_into().unwrap(),
        })
        .collect();

    let full_steps = nums(st, "full_P_step");
    let full = nums(st, "full_P");
    let full_p = full_steps
        .iter()
        .enumerate()
        .map(|(i, &k)| {
            (
                k as usize - 1,
                Covariance::from_column_slice(&full[i * 361..(i + 1) * 361]),
            )
        })
        .collect();

    Sequence {
        config: seq["config"].clone(),
        steps,
        full_p,
    }
}

/// The MATLAB loop of `make_reference.m`, written with the core functions:
/// `gmekf_predict`, then `gmekf_correct_ctv`'s barometer, GPS,
/// LiDAR (1-4), then magnetometer order.
#[test]
fn core_sequence_matches_matlab() {
    let seq = load_sequence();
    let c = &seq.config;
    let dyn_ = Dynamics::new(mat(c, "J"), num(c, "g")).unwrap();
    let (qc, dt): (ProcessNoise, f64) = (mat(c, "Qc"), num(c, "dt"));
    let r_b: Vector3<f64> = vec(c, "r_b");
    let r_baro: SMatrix<f64, 1, 1> = mat(c, "R_baro");
    let mag_field: Vector3<f64> = vec(c, "mag_field");
    let r_mag = Matrix3::identity() * num(c, "R_mag");
    let lidar_beams: SMatrix<f64, 3, 4> = mat(c, "lidar_beams");
    let lidar_positions: SMatrix<f64, 3, 4> = mat(c, "lidar_positions");
    let height_com = num(c, "lidar_heightCOM");
    let mut r_gps = SMatrix::<f64, 6, 6>::zeros();
    r_gps
        .fixed_view_mut::<3, 3>(0, 0)
        .copy_from(&mat::<3, 3>(c, "pcov"));
    r_gps
        .fixed_view_mut::<3, 3>(3, 3)
        .copy_from(&mat::<3, 3>(c, "vcov"));

    let (mut x, mut p): (StateVector, Covariance) = (vec(c, "x0"), mat(c, "P0"));
    let mut full = seq.full_p.iter().peekable();
    let mut worst = (0.0_f64, 0.0_f64);
    for (k, s) in seq.steps.iter().enumerate() {
        (x, p) = gmekf_predict(&x, &p, &s.accel, &s.gyro, true, &dyn_, dt, &qc);
        let mut diagnostics = [-1.0; 16];
        let mut correct = |slot: usize, out: Corrected, x: &mut StateVector, p: &mut Covariance| {
            diagnostics[slot] = if out.accepted { 1.0 } else { 0.0 };
            diagnostics[slot + 8] = out.nis;
            *x = out.x;
            *p = out.p;
        };
        if let Some(h) = s.baro {
            let out = update(&x, &p, Some(baro_model(&x, h)), &r_baro, BARO_GATE);
            correct(0, out, &mut x, &mut p);
        }
        if let Some((pos, vel)) = s.gps {
            let out = update(
                &x,
                &p,
                Some(gps_model(&x, &pos, &vel, &r_b)),
                &r_gps,
                GPS_GATE,
            );
            correct(2, out, &mut x, &mut p);
        }
        for index in 0..4 {
            if let Some(range) = s.lidar[index] {
                let Some(variance) = lidar_covariance(range) else {
                    continue;
                };
                let r = SMatrix::<f64, 1, 1>::new(variance);
                let beam = lidar_beams.column(index).into_owned();
                let sensor = lidar_positions.column(index).into_owned();
                let out = update(
                    &x,
                    &p,
                    lidar_model(&x, range, &beam, &sensor, height_com),
                    &r,
                    LIDAR_GATE,
                );
                correct(3 + index, out, &mut x, &mut p);
            }
        }
        if let Some(m) = s.mag {
            let out = update(
                &x,
                &p,
                Some(mag_model(&x, &m, &mag_field)),
                &r_mag,
                MAG_GATE,
            );
            correct(7, out, &mut x, &mut p);
        }

        let what = format!("step {k}");
        assert_close(&x, &s.x, 1e-10, &format!("{what} x"));
        assert_close(
            &p.diagonal(),
            &s.p_diag,
            1e-10,
            &format!("{what} P diagonal"),
        );
        for i in 0..8 {
            assert_eq!(diagnostics[i], s.diagnostics[i], "{what} accepted flag {i}");
            let (a, b) = (diagnostics[i + 8], s.diagnostics[i + 8]);
            assert!(
                (a - b).abs() <= 1e-8 * (1.0 + b.abs()),
                "{what} NIS {i}: {a} vs {b}"
            );
        }
        if let Some((_, full_p)) = full.next_if(|(i, _)| *i == k) {
            assert_close(&p, full_p, 1e-10, &format!("{what} full P"));
        }
        worst.0 = worst.0.max((x - s.x).amax());
        worst.1 = worst.1.max((p.diagonal() - s.p_diag).amax());
    }
    assert!(
        full.next().is_none(),
        "not every full covariance was checked"
    );
    let mag_flags: Vec<f64> = seq.steps.iter().map(|s| s.diagnostics[7]).collect();
    assert!(mag_flags.contains(&1.0), "no magnetometer update accepted");
    assert!(
        mag_flags.contains(&0.0),
        "the magnetometer outlier was not gated"
    );
    for index in 0..4 {
        let flags: Vec<f64> = seq
            .steps
            .iter()
            .map(|step| step.diagnostics[3 + index])
            .collect();
        assert!(
            flags.contains(&1.0),
            "LiDAR {} had no accepted update",
            index + 1
        );
    }
    assert!(
        seq.steps.iter().any(|step| step.diagnostics[6] == 0.0),
        "the LiDAR 4 outlier was not gated"
    );
    println!(
        "worst difference: x {:e}, P diagonal {:e}",
        worst.0, worst.1
    );
}

/// The [`Gmekf`] wrapper configured like the MATLAB sequence.
fn sequence_filter(c: &Value) -> (Gmekf, NavState) {
    let qc: ProcessNoise = mat(c, "Qc");
    let q = qc.diagonal();
    let pcov: Matrix3<f64> = mat(c, "pcov");
    let vcov: Matrix3<f64> = mat(c, "vcov");
    let p0: Covariance = mat(c, "P0");
    let lidar_beams: SMatrix<f64, 3, 4> = mat(c, "lidar_beams");
    let lidar_positions: SMatrix<f64, 3, 4> = mat(c, "lidar_positions");
    let config = GmekfConfig {
        gravity: num(c, "g"),
        inertia: mat(c, "J"),
        accel_std: q.fixed_rows::<3>(0).map(f64::sqrt),
        gyro_std: q.fixed_rows::<3>(3).map(f64::sqrt),
        accel_bias_psd: q.fixed_rows::<3>(6).into_owned(),
        gyro_bias_psd: q.fixed_rows::<3>(9).into_owned(),
        baro_bias_psd: q[12],
        accel_bias_std: p0[(12, 12)].sqrt(),
        gyro_bias_std: p0[(15, 15)].sqrt(),
        baro_bias_std: p0[(18, 18)].sqrt(),
        baro_std: num(c, "R_baro").sqrt(),
        gps_position_std: pcov.diagonal().map(f64::sqrt),
        gps_velocity_std: vcov.diagonal().map(f64::sqrt),
        gps_lever_arm: vec(c, "r_b"),
        mag_field_ned: Some(vec(c, "mag_field")),
        mag_std: num(c, "R_mag").sqrt(),
        lidar_mounts: (0..4)
            .map(|index| LidarMount {
                beam_dir_body: lidar_beams.column(index).into_owned(),
                position_body: lidar_positions.column(index).into_owned(),
                height_com: num(c, "lidar_heightCOM"),
            })
            .collect(),
        ..GmekfConfig::default()
    };

    let x0: StateVector = vec(c, "x0");
    let std3 =
        |o: usize| Vector3::new(p0[(o, o)], p0[(o + 1, o + 1)], p0[(o + 2, o + 2)]).map(f64::sqrt);
    let initial = NavState {
        time: Duration::ZERO,
        position: x0.fixed_rows::<3>(POS).into_owned(),
        velocity: x0.fixed_rows::<3>(VEL).into_owned(),
        attitude: Quaternion::new(x0[QUAT], x0[QUAT + 1], x0[QUAT + 2], x0[QUAT + 3]),
        body_rate: x0.fixed_rows::<3>(RATE).into_owned(),
        position_std: std3(E_POS),
        velocity_std: std3(E_VEL),
        attitude_std: std3(E_ATT),
        body_rate_std: std3(E_RATE),
    };
    (Gmekf::new(config), initial)
}

#[test]
fn lidar_wrapper_checks_mounts_ranges_indices_and_repeat_times() {
    let seq = load_sequence();
    let (mut filter, initial) = sequence_filter(&seq.config);
    filter.initialize(&initial).unwrap();

    assert!(matches!(
        filter.update_lidar(Duration::ZERO, 4, 1.0),
        Ok(crate::navigate::UpdateOutcome::Rejected(
            crate::navigate::Rejection::InvalidReading
        ))
    ));
    assert!(matches!(
        filter.update_lidar(Duration::ZERO, 0, 0.19),
        Ok(crate::navigate::UpdateOutcome::Rejected(
            crate::navigate::Rejection::InvalidReading
        ))
    ));
    for range in [4.001, f64::NAN, f64::INFINITY] {
        assert!(matches!(
            filter.update_lidar(Duration::ZERO, 0, range),
            Ok(crate::navigate::UpdateOutcome::Rejected(
                crate::navigate::Rejection::InvalidReading
            ))
        ));
    }

    filter.update_lidar(Duration::ZERO, 0, 1.0).unwrap();
    assert!(matches!(
        filter.update_lidar(Duration::ZERO, 0, 1.0),
        Ok(crate::navigate::UpdateOutcome::Rejected(
            crate::navigate::Rejection::OutOfTimeWindow
        ))
    ));

    let mut unconfigured = Gmekf::new(GmekfConfig::default());
    unconfigured.initialize(&initial).unwrap();
    assert_eq!(
        unconfigured.update_lidar(Duration::ZERO, 0, 1.0).unwrap(),
        crate::navigate::UpdateOutcome::NotUsed
    );
}

fn lidar_height_test_filter() -> (Gmekf, NavState) {
    let config = GmekfConfig {
        lidar_mounts: vec![LidarMount {
            beam_dir_body: Vector3::z(),
            position_body: Vector3::zeros(),
            height_com: 0.0,
        }],
        ..GmekfConfig::default()
    };
    let mut initial = NavState::unknown(Duration::ZERO);
    initial.position.z = -2.0;
    initial.position_std = Vector3::repeat(0.5);
    initial.velocity_std = Vector3::repeat(0.1);
    initial.body_rate_std = Vector3::repeat(0.01);
    initial.attitude_std = Vector3::repeat(0.01);
    (Gmekf::new(config), initial)
}

#[test]
fn lidar_measurement_corrects_height_and_reduces_uncertainty() {
    let (mut filter, initial) = lidar_height_test_filter();
    filter.initialize(&initial).unwrap();
    let (before_x, before_p) = filter.estimate().unwrap();

    // At identity attitude, a downward beam sees 2 m from z = -2 m.
    // The 1.5 m reading should move the estimated down position toward -1.5 m.
    let outcome = filter.update_lidar(Duration::ZERO, 0, 1.5).unwrap();
    let (after_x, after_p) = filter.estimate().unwrap();
    assert!(matches!(
        outcome,
        crate::navigate::UpdateOutcome::Applied { .. }
    ));
    assert!(
        after_x[POS + 2] > before_x[POS + 2],
        "shorter measured range should move the NED down estimate toward the ground"
    );
    assert!(
        (after_x[POS + 2] + 1.5).abs() < 0.03,
        "estimated down position {} should approach -1.5 m",
        after_x[POS + 2]
    );
    assert!(after_p[(E_POS + 2, E_POS + 2)] < before_p[(E_POS + 2, E_POS + 2)]);

    let diagnostics = filter.diagnostics().unwrap();
    assert_eq!(diagnostics[DIAG_LIDAR], 1.0);
    assert!(diagnostics[DIAG_LIDAR + 8].is_finite());
}

#[test]
fn lidar_in_range_outlier_is_gated_without_changing_estimate() {
    let (mut filter, initial) = lidar_height_test_filter();
    filter.initialize(&initial).unwrap();
    let before = filter.estimate().unwrap();

    // 3.9 m is a valid sensor range, but is inconsistent with the 2 m prediction.
    let outcome = filter.update_lidar(Duration::ZERO, 0, 3.9).unwrap();
    let nis = match outcome {
        crate::navigate::UpdateOutcome::Rejected(crate::navigate::Rejection::Outlier { nis }) => {
            nis
        }
        other => panic!("expected a gated outlier, got {other:?}"),
    };
    assert!(nis > LIDAR_GATE);
    assert_eq!(filter.estimate().unwrap(), before);

    let diagnostics = filter.diagnostics().unwrap();
    assert_eq!(diagnostics[DIAG_LIDAR], 0.0);
    assert!(diagnostics[DIAG_LIDAR + 8] > LIDAR_GATE);
}

/// The same sequence through the `Estimator` interface, as the flight loop
/// drives it. Times are `Duration`s (whole nanoseconds), so each step's dt
/// differs from MATLAB's by up to 1 ns; the tolerance allows for that.
#[test]
fn estimator_sequence_matches_matlab() {
    let seq = load_sequence();
    let (mut filter, initial) = sequence_filter(&seq.config);
    filter.initialize(&initial).unwrap();

    let mut worst = 0.0_f64;
    for (k, s) in seq.steps.iter().enumerate() {
        let time = Duration::from_secs_f64(s.time);
        let imu = ImuSample {
            accel: s.accel,
            gyro: s.gyro,
        };
        filter.predict(&Timestamped::new(time, imu)).unwrap();
        if let Some(h) = s.baro {
            filter.update_baro_height(time, h).unwrap();
        }
        if let Some((pos, vel)) = s.gps {
            filter.update_gps(time, pos, Some(vel)).unwrap();
        }
        for index in 0..4 {
            if let Some(range) = s.lidar[index] {
                filter
                    .update(&Timestamped::new(time, Measurement::Lidar { index, range }))
                    .unwrap();
            }
        }
        if let Some(field) = s.mag {
            let m = Measurement::Magnetometer { field };
            filter.update(&Timestamped::new(time, m)).unwrap();
        }

        let (x, p) = filter.estimate().unwrap();
        let what = format!("step {k}");
        assert_close(&x, &s.x, 1e-6, &format!("{what} x"));
        assert_close(
            &p.diagonal(),
            &s.p_diag,
            1e-6,
            &format!("{what} P diagonal"),
        );
        let diagnostics = filter.diagnostics().unwrap();
        assert_eq!(
            diagnostics[..8],
            s.diagnostics[..8],
            "{what} accepted flags"
        );
        worst = worst.max((x - s.x).amax());

        let nav = filter.nav_state().unwrap();
        assert_eq!(nav.time, time);
        assert_eq!(nav.position, x.fixed_rows::<3>(POS).into_owned());
    }
    for index in 0..4 {
        assert!(
            seq.steps
                .iter()
                .any(|step| step.diagnostics[3 + index] == 1.0),
            "LiDAR {} had no accepted update through Estimator",
            index + 1
        );
    }
    assert!(
        seq.steps.iter().any(|step| step.diagnostics[6] == 0.0),
        "LiDAR 4 outlier was not gated through Estimator"
    );
    println!("worst state difference {worst:e}");
}

#[test]
fn estimator_estimates_the_simulated_truth() {
    // Sanity check on the same data: after 4 s the estimate is close to the
    // truth that generated the sensors, and the barometer bias (0.4 m) was
    // found within 3 sigma of the filter's own uncertainty.
    let seq = load_sequence();
    let truth = nums(&sequence()["steps"], "truth");
    let last = seq.steps.len() - 1;
    let x = &seq.steps[last].x;
    let true_pos = Vector3::from_column_slice(&truth[last * 13..last * 13 + 3]);
    let pos_err = (x.fixed_rows::<3>(POS) - true_pos).norm();
    assert!(pos_err < 0.2, "position error {pos_err} m");
    let baro_std = seq.steps[last].p_diag[E_BARO_BIAS].sqrt();
    assert!(
        (x[BARO_BIAS] - 0.4).abs() < 3.0 * baro_std,
        "baro bias {} (std {baro_std})",
        x[BARO_BIAS]
    );
}

// ---------------------------------------------------------------------------
// Shared estimator checks (navigate::testing)

/// A GMEKF tuned for the noise of a shared test scenario.
fn matched_filter(noise: &SensorNoise, _scenario: &Scenario) -> Gmekf {
    Gmekf::new(GmekfConfig {
        gravity: G0,
        // Per-sample IMU noise. Keep a floor so perfect sensors still give a
        // well-conditioned filter.
        accel_std: Vector3::repeat(noise.accel_std.max(1e-3)),
        gyro_std: Vector3::repeat(noise.gyro_std.max(1e-4)),
        baro_std: noise.baro_altitude_std().max(0.01),
        gps_position_std: Vector3::repeat(noise.gps_position_std.max(0.01)),
        gps_velocity_std: Vector3::repeat(noise.gps_velocity_std.max(0.01)),
        accel_bias_std: 0.01,
        gyro_bias_std: 1e-3,
        baro_bias_std: 0.01,
        ..GmekfConfig::default()
    })
}

#[test]
fn follows_time_rules() {
    let scenario = Scenario::stationary(1.14, 1.0);
    let mut filter = matched_filter(&scenario.noise, &scenario);
    testing::check_time_rules(&mut filter, &scenario);
}

#[test]
fn starts_from_initial_state() {
    let scenario = Scenario::stationary(1.14, 1.0);
    let mut filter = matched_filter(&scenario.noise, &scenario);
    testing::check_initialization(&mut filter, &scenario);
}

#[test]
fn stays_put_on_the_pad() {
    let scenario = Scenario::stationary(1.14, 10.0).with_noise(SensorNoise::typical());
    let mut filter = matched_filter(&scenario.noise, &scenario);
    testing::check_stationary(&mut filter, &scenario, 0.5, 0.1);
}

#[test]
fn converges_from_a_wrong_start() {
    let scenario = Scenario::stationary(1.14, 10.0);
    let mut filter = matched_filter(&scenario.noise, &scenario);
    let error = testing::check_convergence(&mut filter, &scenario, 1.0, 0.5);
    assert!(error < 0.05, "position error {error} m after 10 s");
}

#[test]
fn tracks_a_hop() {
    let scenario = Scenario::hop(1.14, 5.0, 6.0).with_noise(SensorNoise::typical());
    let mut filter = matched_filter(&scenario.noise, &scenario);
    let rms = testing::check_tracking(&mut filter, &scenario, 4.0);
    assert!(rms.position < 0.5, "RMS position error {} m", rms.position);
}

#[test]
fn rejects_non_finite_imu_samples_without_changing_the_estimate() {
    let scenario = Scenario::stationary(1.14, 1.0);
    let mut filter = matched_filter(&scenario.noise, &scenario);
    filter
        .initialize(&scenario.initial_state(0.5, 0.2))
        .unwrap();
    let before = filter.estimate();
    let bad = ImuSample {
        accel: Vector3::new(f64::NAN, 0.0, 0.0),
        gyro: Vector3::zeros(),
    };
    let time = Duration::from_millis(5);
    assert_eq!(
        filter.predict(&Timestamped::new(time, bad)),
        Err(crate::navigate::NavError::InvalidSample { time })
    );
    assert_eq!(filter.estimate(), before);
}

#[test]
fn ignores_repeated_measurements() {
    let scenario = Scenario::stationary(1.14, 1.0);
    let mut filter = matched_filter(&scenario.noise, &scenario);
    filter
        .initialize(&scenario.initial_state(0.5, 0.2))
        .unwrap();
    let t = Duration::from_millis(5);
    let imu = ImuSample {
        accel: Vector3::new(G0, 0.0, 0.0),
        gyro: Vector3::zeros(),
    };
    filter.predict(&Timestamped::new(t, imu)).unwrap();
    assert!(matches!(
        filter.update_baro_height(t, 1.14).unwrap(),
        crate::navigate::UpdateOutcome::Applied { .. }
    ));
    // The same reading again (a held sample) must not be fused twice.
    assert_eq!(
        filter.update_baro_height(t, 1.14).unwrap(),
        crate::navigate::UpdateOutcome::Rejected(crate::navigate::Rejection::OutOfTimeWindow)
    );
}

// ---------------------------------------------------------------------------
// Timing

/// Time per filter step (predict + gyro, plus a barometer and a GPS update
/// every 10 steps, as on the CTV). Run on the target with
/// `cargo test --release gmekf::tests::step_time -- --ignored --nocapture`.
#[test]
#[ignore]
fn step_time() {
    let seq = load_sequence();
    let (mut filter, initial) = sequence_filter(&seq.config);
    let rounds = 20;
    let mut samples = Vec::with_capacity(rounds * seq.steps.len());
    for round in 0..rounds {
        let offset = round as f64 * 10.0;
        let mut init = initial;
        init.time = Duration::from_secs_f64(offset);
        filter.initialize(&init).unwrap();
        for s in &seq.steps {
            let time = Duration::from_secs_f64(offset + s.time);
            let start = std::time::Instant::now();
            filter
                .predict(&Timestamped::new(
                    time,
                    ImuSample {
                        accel: s.accel,
                        gyro: s.gyro,
                    },
                ))
                .unwrap();
            if let Some(h) = s.baro {
                filter.update_baro_height(time, h).unwrap();
            }
            if let Some((pos, vel)) = s.gps {
                filter.update_gps(time, pos, Some(vel)).unwrap();
            }
            std::hint::black_box(filter.nav_state());
            samples.push(start.elapsed());
        }
    }
    samples.sort();
    let mean = samples.iter().sum::<Duration>() / samples.len() as u32;
    let p99 = samples[samples.len() * 99 / 100];
    let max = *samples.last().unwrap();
    println!(
        "GMEKF step over {} steps: mean {:?}, p99 {:?}, max {:?} (budget at 500 Hz: 2 ms)",
        samples.len(),
        mean,
        p99,
        max
    );
    assert!(
        p99 < Duration::from_millis(2),
        "p99 step time {p99:?} over the 500 Hz budget"
    );
}
