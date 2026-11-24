//! The GMEKF equations, one function per function of the MATLAB filter
//! (hopper `subsystems/navigation/Src_filter`, branch `ctv-gmekf-integration`,
//! commit ef5c7fb: the gyro drives the rotation, no torque input). The
//! MATLAB name is given on each function. Indices in comments are MATLAB's
//! (1-based); the code is 0-based.
//!
//! Nominal state `x` (20): position NED (1:3), velocity NED (4:6), body rate
//! (7:9), attitude quaternion scalar first (10:13), accelerometer bias
//! (14:16), gyro bias (17:19), barometer bias (20). Biases are in body axes.
//!
//! Error state (19): the same, with a 3-element attitude error (10:12) in
//! inertial axes, and accelerometer and gyro bias errors in inertial axes
//! (13:15, 16:18), barometer bias error (19). Errors are estimate minus
//! truth, so corrections are subtracted.
//!
//! [`quaternion_to_rotation`] gives the body-to-NED rotation, the same
//! matrix as `nalgebra::UnitQuaternion::to_rotation_matrix` for the same
//! components, so the quaternion matches luna's `ControlState::attitude`.

use nalgebra::{Cholesky, Matrix3, Matrix4x3, SMatrix, SVector, Vector3, Vector4};

/// Nominal state (20).
pub type StateVector = SVector<f64, 20>;
/// Error state (19).
pub type ErrorVector = SVector<f64, 19>;
/// Error covariance (19 x 19).
pub type Covariance = SMatrix<f64, 19, 19>;
/// Process noise (13 x 13): per-sample accelerometer covariance (3) and
/// gyro covariance (3), body axes; then the continuous-time random-walk
/// PSDs of the accelerometer bias (3), gyro bias (3) and barometer bias (1).
pub type ProcessNoise = SMatrix<f64, 13, 13>;
/// Process noise Jacobian (19 x 13).
pub type NoiseJacobian = SMatrix<f64, 19, 13>;
/// Measurement Jacobian for an `M`-dimensional measurement.
pub type MeasurementJacobian<const M: usize> = SMatrix<f64, M, 19>;

// 0-based offsets into the nominal state.
pub const POS: usize = 0;
pub const VEL: usize = 3;
pub const RATE: usize = 6;
pub const QUAT: usize = 9;
pub const ACCEL_BIAS: usize = 13;
pub const GYRO_BIAS: usize = 16;
pub const BARO_BIAS: usize = 19;

// 0-based offsets into the error state.
pub const E_POS: usize = 0;
pub const E_VEL: usize = 3;
pub const E_RATE: usize = 6;
pub const E_ATT: usize = 9;
pub const E_ACCEL_BIAS: usize = 12;
pub const E_GYRO_BIAS: usize = 15;
pub const E_BARO_BIAS: usize = 18;

// `kalman_measurement_update` gates, chi-square 99 %, from
// `gmekf_correct_ctv.m`.
pub const BARO_GATE: f64 = 6.635; // 1 DOF
pub const GYRO_GATE: f64 = 11.345; // 3 DOF
pub const GPS_GATE: f64 = 16.812; // 6 DOF
pub const GPS_3DOF_GATE: f64 = 11.345; // 3 DOF, position or velocity alone
pub const MAG_GATE: f64 = 11.345; // 3 DOF

/// `skew.m`: `skew(v) * u == v.cross(u)`.
pub fn skew(v: &Vector3<f64>) -> Matrix3<f64> {
    Matrix3::new(0.0, -v.z, v.y, v.z, 0.0, -v.x, -v.y, v.x, 0.0)
}

/// `qmult.m`: Hamilton product of scalar-first quaternions.
pub fn qmult(q1: &Vector4<f64>, q2: &Vector4<f64>) -> Vector4<f64> {
    let (s1, v1) = (q1[0], q1.fixed_rows::<3>(1).into_owned());
    let (s2, v2) = (q2[0], q2.fixed_rows::<3>(1).into_owned());
    let v = v1 * s2 + v2 * s1 + v1.cross(&v2);
    Vector4::new(s1 * s2 - v1.dot(&v2), v.x, v.y, v.z)
}

/// `quaternion_to_rotation.m`: body-to-NED rotation of a scalar-first
/// quaternion.
pub fn quaternion_to_rotation(q: &Vector4<f64>) -> Matrix3<f64> {
    let (w, x, y, z) = (q[0], q[1], q[2], q[3]);
    Matrix3::new(
        1.0 - 2.0 * (y * y + z * z),
        2.0 * (x * y + w * z),
        2.0 * (x * z - w * y),
        2.0 * (x * y - w * z),
        1.0 - 2.0 * (x * x + z * z),
        2.0 * (y * z + w * x),
        2.0 * (x * z + w * y),
        2.0 * (y * z - w * x),
        1.0 - 2.0 * (x * x + y * y),
    )
    .transpose()
}

/// `Psi_helper.m`: maps an inertial attitude error to a quaternion
/// increment, `[0; e] * q`.
pub fn psi(q: &Vector4<f64>) -> Matrix4x3<f64> {
    let (w, x, y, z) = (q[0], q[1], q[2], q[3]);
    Matrix4x3::new(-x, -y, -z, w, z, -y, -z, w, x, y, -x, w)
}

fn quat(x: &StateVector) -> Vector4<f64> {
    x.fixed_rows::<4>(QUAT).into_owned()
}

fn vec3(x: &StateVector, offset: usize) -> Vector3<f64> {
    x.fixed_rows::<3>(offset).into_owned()
}

/// Constants of the rigid-body model used by the prediction.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Dynamics {
    /// Inertia tensor in body axes, kg m^2.
    pub inertia: Matrix3<f64>,
    /// Its inverse.
    pub inertia_inv: Matrix3<f64>,
    /// Gravity, m/s^2, pointing down.
    pub gravity: f64,
}

impl Dynamics {
    /// Returns `None` if `inertia` is not invertible.
    pub fn new(inertia: Matrix3<f64>, gravity: f64) -> Option<Self> {
        Some(Dynamics {
            inertia,
            inertia_inv: inertia.try_inverse()?,
            gravity,
        })
    }

    fn g_vec(&self) -> Vector3<f64> {
        Vector3::new(0.0, 0.0, self.gravity)
    }
}

/// `propagate_state.m`: propagates the nominal state by `dt` s.
///
/// `a_ib` is the body-to-NED rotation at the start of the step. `accel` is
/// the measured specific force and `gyro` the measured rate (body, m/s^2 and
/// rad/s). With a valid sample the rate becomes the gyro minus the gyro bias
/// estimate. With `validity` false the IMU is not used: position is
/// dead-reckoned from the velocity and the rate follows Euler's equations
/// with zero torque.
#[allow(clippy::too_many_arguments)]
pub fn propagate_state(
    xn: &StateVector,
    a_ib: &Matrix3<f64>,
    accel: &Vector3<f64>,
    gyro: &Vector3<f64>,
    validity: bool,
    dynamics: &Dynamics,
    dt: f64,
) -> StateVector {
    let posn = vec3(xn, POS);
    let veln = vec3(xn, VEL);
    let angveln = vec3(xn, RATE);
    let qn = quat(xn);
    let abn = vec3(xn, ACCEL_BIAS);
    let gbn = vec3(xn, GYRO_BIAS);

    let (posnp1, velnp1, angvelnp1) = if validity {
        let accel_est = a_ib * (accel - abn) + dynamics.g_vec();
        (
            posn + veln * dt + accel_est / 2.0 * dt * dt,
            veln + accel_est * dt,
            gyro - gbn,
        )
    } else {
        let j = &dynamics.inertia;
        let angaccel = -(dynamics.inertia_inv * angveln.cross(&(j * angveln)));
        (posn + veln * dt, veln, angveln + angaccel * dt)
    };
    let angvelavg = 0.5 * (angveln + angvelnp1);

    // Quaternion by the exponential of the average rate.
    let theta = angvelavg * dt;
    let ang = theta.norm();
    let dq = if ang < 1e-8 {
        Vector4::new(1.0, 0.5 * theta.x, 0.5 * theta.y, 0.5 * theta.z)
    } else {
        let s = (ang / 2.0).sin() / ang;
        Vector4::new((ang / 2.0).cos(), s * theta.x, s * theta.y, s * theta.z)
    };
    let qnp1 = qmult(&qn, &dq).normalize();

    let mut xnp1 = *xn;
    xnp1.fixed_rows_mut::<3>(POS).copy_from(&posnp1);
    xnp1.fixed_rows_mut::<3>(VEL).copy_from(&velnp1);
    xnp1.fixed_rows_mut::<3>(RATE).copy_from(&angvelnp1);
    xnp1.fixed_rows_mut::<4>(QUAT).copy_from(&qnp1);
    xnp1
}

/// Rotations, force and rate that `error_state_jacobian.m` uses.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct HalfStep {
    /// Body-to-NED rotation at the start of the step.
    pub a_ib: Matrix3<f64>,
    /// Body-to-NED rotation at the half step.
    pub a_ib05: Matrix3<f64>,
    /// Inertial specific force, m/s^2 (start-of-step attitude).
    pub a_i: Vector3<f64>,
    /// Inertial angular rate, mean over the step, rad/s.
    pub w_i: Vector3<f64>,
}

/// `error_state_jacobian.m`: the discrete error-state transition `F` and
/// noise Jacobian `G` of one IMU step of `dt` s.
pub fn error_state_jacobian(
    x: &StateVector,
    half: &HalfStep,
    dt: f64,
    validity: bool,
    dynamics: &Dynamics,
) -> (Covariance, NoiseJacobian) {
    let mut f = Covariance::identity();
    let mut g = NoiseJacobian::zeros();
    let c = half.a_ib05;
    let i3 = Matrix3::identity();
    let set = |m: &mut Covariance, r: usize, col: usize, v: Matrix3<f64>| {
        m.fixed_view_mut::<3, 3>(r, col).copy_from(&v);
    };

    set(&mut f, E_POS, E_VEL, i3 * dt);
    if validity {
        let sa = skew(&half.a_i);
        set(&mut f, E_POS, E_ATT, -0.5 * sa * (dt * dt));
        set(&mut f, E_POS, E_ACCEL_BIAS, -0.5 * i3 * (dt * dt));
        set(&mut f, E_VEL, E_ATT, -sa * dt);
        set(&mut f, E_VEL, E_ACCEL_BIAS, -i3 * dt);
        // The rate is the bias-corrected gyro sample: it no longer depends on
        // the previous rate, only on the (inertial-aligned) gyro bias error.
        set(&mut f, E_RATE, E_RATE, Matrix3::zeros());
        set(&mut f, E_RATE, E_GYRO_BIAS, -half.a_ib.transpose());
        // Attitude uses the mean of the old and new rates.
        set(&mut f, E_ATT, E_RATE, 0.5 * c * dt);
        set(&mut f, E_ATT, E_GYRO_BIAS, -0.5 * i3 * dt);

        // Per-sample accelerometer and gyro noise. The acceleration is taken
        // with the start-of-step attitude in `propagate_state`, so its noise
        // enters through that attitude too.
        let c0 = half.a_ib;
        g.fixed_view_mut::<3, 3>(E_POS, 0)
            .copy_from(&(0.5 * c0 * (dt * dt)));
        g.fixed_view_mut::<3, 3>(E_VEL, 0).copy_from(&(c0 * dt));
        g.fixed_view_mut::<3, 3>(E_RATE, 3).copy_from(&i3);
        g.fixed_view_mut::<3, 3>(E_ATT, 3)
            .copy_from(&(0.5 * c * dt));
    } else {
        let angvel = vec3(x, RATE);
        let j = dynamics.inertia;
        let m = dynamics.inertia_inv * (skew(&(j * angvel)) - skew(&angvel) * j);
        set(&mut f, E_RATE, E_RATE, i3 + m * dt);
        set(&mut f, E_ATT, E_RATE, c * (i3 + 0.5 * m * dt) * dt);
    }

    // Bias errors are inertial-aligned while the biases are body-frame.
    let w = i3 + skew(&half.w_i) * dt;
    set(&mut f, E_ACCEL_BIAS, E_ACCEL_BIAS, w);
    set(&mut f, E_GYRO_BIAS, E_GYRO_BIAS, w);

    // Random walks: continuous-time PSDs scaled to one step.
    let sq = dt.sqrt();
    g.fixed_view_mut::<3, 3>(E_ACCEL_BIAS, 6)
        .copy_from(&(c * sq));
    g.fixed_view_mut::<3, 3>(E_GYRO_BIAS, 9)
        .copy_from(&(c * sq));
    g[(E_BARO_BIAS, 12)] = sq;
    (f, g)
}

/// `propagate_covariance.m`: `F P F' + G Q G'` with the discrete `F` and
/// `G` of [`error_state_jacobian`].
pub fn propagate_covariance(
    pn: &Covariance,
    f05: &Covariance,
    g05: &NoiseJacobian,
    qc: &ProcessNoise,
    dt: f64,
) -> Covariance {
    debug_assert!(dt >= 0.0);
    let p = f05 * pn * f05.transpose() + g05 * qc * g05.transpose();
    0.5 * (p + p.transpose())
}

/// `gmekf_predict.m`: propagates the state and covariance by `dt` s with
/// one IMU sample.
#[allow(clippy::too_many_arguments)]
pub fn gmekf_predict(
    xn: &StateVector,
    pn: &Covariance,
    accel: &Vector3<f64>,
    gyro: &Vector3<f64>,
    validity: bool,
    dynamics: &Dynamics,
    dt: f64,
    qc: &ProcessNoise,
) -> (StateVector, Covariance) {
    let a_ib = quaternion_to_rotation(&quat(xn));
    let xnp1 = propagate_state(xn, &a_ib, accel, gyro, validity, dynamics, dt);
    let xnp05 = propagate_state(xn, &a_ib, accel, gyro, validity, dynamics, dt * 0.5);

    let a_ib05 = quaternion_to_rotation(&quat(&xnp05));
    // Specific force with the start-of-step attitude, as `propagate_state`
    // applies it (hopper's ef5c7fb; the half-step attitude before)
    let a_i = if validity {
        a_ib * (accel - vec3(&xnp05, ACCEL_BIAS))
    } else {
        dynamics.g_vec() // assume unaccelerated
    };
    let w_i = a_ib05 * (0.5 * (vec3(xn, RATE) + vec3(&xnp1, RATE)));

    let half = HalfStep {
        a_ib,
        a_ib05,
        a_i,
        w_i,
    };
    let (f05, g05) = error_state_jacobian(&xnp05, &half, dt, validity, dynamics);
    let pnp1 = propagate_covariance(pn, &f05, &g05, qc, dt);
    (xnp1, pnp1)
}

/// Result of [`kalman_measurement_update`].
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct KalmanUpdate {
    /// Error-state correction (zero if not accepted).
    pub dx: ErrorVector,
    /// Conditioned covariance (the prior if not accepted).
    pub p: Covariance,
    pub accepted: bool,
    /// Normalized innovation squared; -1 if the innovation covariance was
    /// not positive definite.
    pub nis: f64,
}

/// `kalman_measurement_update.m`: gated error-state update, Joseph form.
pub fn kalman_measurement_update<const M: usize>(
    p: &Covariance,
    innovation: &SVector<f64, M>,
    h: &MeasurementJacobian<M>,
    r: &SMatrix<f64, M, M>,
    gate: f64,
) -> KalmanUpdate {
    let rejected = |nis| KalmanUpdate {
        dx: ErrorVector::zeros(),
        p: *p,
        accepted: false,
        nis,
    };

    let hp = h * p;
    let s = hp * h.transpose() + r;
    let s = 0.5 * (s + s.transpose());
    let Some(chol) = Cholesky::new(s) else {
        return rejected(-1.0);
    };

    let whitened = chol
        .l()
        .solve_lower_triangular(innovation)
        .expect("Cholesky factor has a positive diagonal");
    let nis = whitened.norm_squared();
    if nis > gate {
        return rejected(nis);
    }

    // K = P H' S^-1, so K' = S^-1 H P (P and S are symmetric).
    let k = chol.solve(&hp).transpose();
    let dx = k * innovation;
    let i_kh = Covariance::identity() - k * h;
    let post = i_kh * p * i_kh.transpose() + k * r * k.transpose();
    KalmanUpdate {
        dx,
        p: 0.5 * (post + post.transpose()),
        accepted: true,
        nis,
    }
}

/// `state_injection.m`: applies an error correction to the nominal state.
pub fn state_injection(x: &StateVector, dx: &ErrorVector) -> StateVector {
    let e3 = |offset: usize| dx.fixed_rows::<3>(offset).into_owned();

    let q = quat(x);
    let q_post = (q - 0.5 * psi(&q) * e3(E_ATT)).normalize();
    let c_post_t = quaternion_to_rotation(&q_post).transpose();

    let mut out = *x;
    out.fixed_rows_mut::<3>(POS)
        .copy_from(&(vec3(x, POS) - e3(E_POS)));
    out.fixed_rows_mut::<3>(VEL)
        .copy_from(&(vec3(x, VEL) - e3(E_VEL)));
    out.fixed_rows_mut::<3>(RATE)
        .copy_from(&(vec3(x, RATE) - e3(E_RATE)));
    out.fixed_rows_mut::<4>(QUAT).copy_from(&q_post);
    out.fixed_rows_mut::<3>(ACCEL_BIAS)
        .copy_from(&(vec3(x, ACCEL_BIAS) - c_post_t * e3(E_ACCEL_BIAS)));
    out.fixed_rows_mut::<3>(GYRO_BIAS)
        .copy_from(&(vec3(x, GYRO_BIAS) - c_post_t * e3(E_GYRO_BIAS)));
    out[BARO_BIAS] = x[BARO_BIAS] - dx[E_BARO_BIAS];
    out
}

/// `compute_error_reset_jacobian.m`: first-order covariance reset after an
/// injection.
pub fn compute_error_reset_jacobian(dx: &ErrorVector) -> Covariance {
    let e3 = |offset: usize| dx.fixed_rows::<3>(offset).into_owned();
    let mut j = Covariance::identity();
    j.fixed_view_mut::<3, 3>(E_ATT, E_ATT)
        .copy_from(&(Matrix3::identity() - 0.5 * skew(&e3(E_ATT))));
    j.fixed_view_mut::<3, 3>(E_ACCEL_BIAS, E_ATT)
        .copy_from(&(-skew(&e3(E_ACCEL_BIAS))));
    j.fixed_view_mut::<3, 3>(E_GYRO_BIAS, E_ATT)
        .copy_from(&(-skew(&e3(E_GYRO_BIAS))));
    j
}

/// Result of [`update`].
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Corrected {
    pub x: StateVector,
    pub p: Covariance,
    pub accepted: bool,
    /// -1 when not computed.
    pub nis: f64,
}

/// `update.m`: one full measurement correction. `model` is the sensor
/// model's `(innovation, H)`, or `None` if it found the reading unusable.
pub fn update<const M: usize>(
    x: &StateVector,
    p: &Covariance,
    model: Option<(SVector<f64, M>, MeasurementJacobian<M>)>,
    r: &SMatrix<f64, M, M>,
    gate: f64,
) -> Corrected {
    let unchanged = |nis| Corrected {
        x: *x,
        p: *p,
        accepted: false,
        nis,
    };
    let Some((innovation, h)) = model else {
        return unchanged(-1.0);
    };

    let k = kalman_measurement_update(p, &innovation, &h, r, gate);
    if !k.accepted {
        return unchanged(k.nis);
    }

    let x_post = state_injection(x, &k.dx);
    let reset = compute_error_reset_jacobian(&k.dx);
    let p_post = reset * k.p * reset.transpose();
    Corrected {
        x: x_post,
        p: 0.5 * (p_post + p_post.transpose()),
        accepted: true,
        nis: k.nis,
    }
}

/// `baro_update_model.m`: barometric height above the pad, m.
pub fn baro_model(x: &StateVector, height: f64) -> (SVector<f64, 1>, MeasurementJacobian<1>) {
    let predicted = -x[POS + 2] + x[BARO_BIAS];
    let mut h = MeasurementJacobian::<1>::zeros();
    h[(0, E_POS + 2)] = -1.0;
    h[(0, E_BARO_BIAS)] = 1.0;
    (SVector::<f64, 1>::new(predicted - height), h)
}

/// `gyro_update_model.m`: body angular rate, rad/s.
pub fn gyro_model(
    x: &StateVector,
    angvel: &Vector3<f64>,
) -> (Vector3<f64>, MeasurementJacobian<3>) {
    let predicted = vec3(x, RATE) + vec3(x, GYRO_BIAS);
    let c_bi = quaternion_to_rotation(&quat(x)).transpose();
    let mut h = MeasurementJacobian::<3>::zeros();
    h.fixed_view_mut::<3, 3>(0, E_RATE)
        .copy_from(&Matrix3::identity());
    h.fixed_view_mut::<3, 3>(0, E_GYRO_BIAS).copy_from(&c_bi);
    (predicted - angvel, h)
}

/// `gps_update_model.m`: antenna position (NED, m) and velocity (NED, m/s)
/// with the antenna at `r_b` (body, m) from the IMU.
pub fn gps_model(
    x: &StateVector,
    pos: &Vector3<f64>,
    vel: &Vector3<f64>,
    r_b: &Vector3<f64>,
) -> (SVector<f64, 6>, MeasurementJacobian<6>) {
    let c_ib = quaternion_to_rotation(&quat(x));
    let lever = c_ib * r_b;
    let rot_vel = c_ib * vec3(x, RATE).cross(r_b);

    let predicted_pos = vec3(x, POS) + lever;
    let predicted_vel = vec3(x, VEL) + rot_vel;
    let mut innovation = SVector::<f64, 6>::zeros();
    innovation
        .fixed_rows_mut::<3>(0)
        .copy_from(&(predicted_pos - pos));
    innovation
        .fixed_rows_mut::<3>(3)
        .copy_from(&(predicted_vel - vel));

    let i3 = Matrix3::identity();
    let mut h = MeasurementJacobian::<6>::zeros();
    h.fixed_view_mut::<3, 3>(0, E_POS).copy_from(&i3);
    h.fixed_view_mut::<3, 3>(0, E_ATT)
        .copy_from(&(-skew(&lever)));
    h.fixed_view_mut::<3, 3>(3, E_VEL).copy_from(&i3);
    h.fixed_view_mut::<3, 3>(3, E_RATE)
        .copy_from(&(-c_ib * skew(r_b)));
    h.fixed_view_mut::<3, 3>(3, E_ATT)
        .copy_from(&(-skew(&rot_vel)));
    (innovation, h)
}

/// `mag_update_model.m`: magnetic field in body axes, for a field
/// `field_ned` in NED (any unit, the same for both).
pub fn mag_model(
    x: &StateVector,
    field_body: &Vector3<f64>,
    field_ned: &Vector3<f64>,
) -> (Vector3<f64>, MeasurementJacobian<3>) {
    let c_bi = quaternion_to_rotation(&quat(x)).transpose();
    let mut h = MeasurementJacobian::<3>::zeros();
    h.fixed_view_mut::<3, 3>(0, E_ATT)
        .copy_from(&(c_bi * skew(field_ned)));
    (c_bi * field_ned - field_body, h)
}

/// Returns the calibrated LiDAR measurement variance for a range, in m^2.
///
/// The calibration uses the nearest nominal-range band and is valid from
/// 0.2 m through 4.0 m.
pub fn lidar_covariance(range: f64) -> Option<f64> {
    if !range.is_finite() || !(0.2..=4.0).contains(&range) {
        return None;
    }

    const BANDS: [(f64, f64); 5] = [
        (0.25, 0.0006248388252148995),
        (0.50, 0.0006835653151205994),
        (1.00, 0.0004522540148365515),
        (2.00, 0.0003407199007033577),
        (3.00, 0.0006514607715648194),
    ];

    BANDS
        .iter()
        .copied()
        .min_by(|(a, _), (b, _)| {
            (range - a)
                .abs()
                .partial_cmp(&(range - b).abs())
                .expect("finite range and finite calibration bands")
        })
        .map(|(_, covariance)| covariance)
}

/// `lidar_update_model.m`: single-beam range in the navigation frame.
///
/// `beam_dir_body` is the beam direction in body axes, `r_body` is the
/// sensor position relative to the filter's origin (the IMU) in body axes, and
/// `height` is the height of that origin above the ground. Returns `None` when the beam is nearly
/// parallel to the ground or the predicted intersection is not in front of
/// the sensor.
pub fn lidar_model(
    x: &StateVector,
    range: f64,
    beam_dir_body: &Vector3<f64>,
    r_body: &Vector3<f64>,
    height: f64,
) -> Option<(SVector<f64, 1>, MeasurementJacobian<1>)> {
    if !range.is_finite()
        || !height.is_finite()
        || !beam_dir_body.iter().all(|v| v.is_finite())
        || !r_body.iter().all(|v| v.is_finite())
    {
        return None;
    }

    let c_ib = quaternion_to_rotation(&quat(x));
    let n = Vector3::new(c_ib[(2, 0)], c_ib[(2, 1)], c_ib[(2, 2)]);
    let h = x[POS + 2] - height;
    let d = n.dot(beam_dir_body);
    if !d.is_finite() || d.abs() < 1e-3 {
        return None;
    }

    let predicted_range = -(n.dot(r_body) + h) / d;
    if !predicted_range.is_finite() || predicted_range <= 0.0 {
        return None;
    }

    let u = c_ib * (r_body + predicted_range * beam_dir_body);
    let mut jacobian = MeasurementJacobian::<1>::zeros();
    jacobian[(0, E_POS + 2)] = -1.0 / d;
    jacobian
        .fixed_view_mut::<1, 3>(0, E_ATT)
        .copy_from(&(Vector3::z().cross(&u) / d).transpose());

    Some((SVector::<f64, 1>::new(predicted_range - range), jacobian))
}
