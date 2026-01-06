#include "ekf.h"

/**
 * @brief Compute the time derivative of the attitude quaternion.
 *
 * This function computes the continuous-time derivative of the attitude
 * quaternion using the MEKF formulation. The quaternion derivative is given by:
 *
 * \f[
 *   \dot{\mathbf{q}} = \frac{1}{2}\,\boldsymbol{\omega}_q \otimes \mathbf{q}
 * \f]
 *
 * where:
 * - \f$\mathbf{q}\f$ is the current attitude quaternion,
 * - \f$\boldsymbol{\omega}_q = [0\ \omega_x\ \omega_y\ \omega_z]^T\f$
 *   is the angular rate expressed as a pure quaternion,
 * - \f$\otimes\f$ denotes quaternion multiplication.
 *
 * This formulation arises naturally in MEKF, where the attitude error
 * is represented multiplicatively and the nominal quaternion is
 * propagated using continuous-time kinematics.
 *
 * The quaternion derivative is orthogonal to the quaternion itself:
 *
 * \f[
 *   \mathbf{q}^T \dot{\mathbf{q}} = 0
 * \f]
 *
 * which preserves the quaternion norm in continuous time in the absence
 * of numerical integration error.
 *
 * @param[in]  q         Current attitude quaternion (4×1), ordered as
 *                      \f$[q_0\ q_1\ q_2\ q_3]^T\f$.
 * @param[in]  what      Measured or estimated body angular rate
 *                      \f$[\omega_x\ \omega_y\ \omega_z]^T\f$ in rad/s.
 * @param[out] qdot      Time derivative of the quaternion (4×1).
 * @param[out] qDotBuff  User-provided buffer backing @p qdot
 *                      (size = 4 floats).
 *
 * @note Quaternion normalization is not performed in this function.
 *       In practice, the propagated quaternion should be periodically
 *       renormalized after numerical integration.
 *
 * @note Quaternion multiplication is performed using
 *       arm_quaternion_product_single_f32().
 */
void compute_qdot(arm_matrix_instance_f32* q, arm_matrix_instance_f32* what, arm_matrix_instance_f32* qdot, float32_t qDotBuff[4]) {

	float32_t wQuatBuff[4] = {0, what->pData[0], what->pData[1], what->pData[2]};
	arm_quaternion_product_single_f32(q->pData, wQuatBuff, qDotBuff);
	arm_scale_f32(qDotBuff, 0.5f, qDotBuff, 4);
	arm_mat_init_f32(qdot, 4, 1, qDotBuff);

}

/**
 * @brief Compute the time derivative of the geodetic position (LLA).
 *
 * This function computes the continuous-time rate of change of the
 * geodetic latitude, longitude, and altitude (LLA) state using
 * local tangent plane (NED) velocities and the WGS-84 reference
 * ellipsoid.
 *
 * The kinematic relations are derived from the geometry of the
 * ellipsoid of revolution and are given by:
 *
 * \f[
 *   \dot{\phi}   = \frac{v_n}{R_\phi + h}
 * \f]
 * \f[
 *   \dot{\lambda} = \frac{v_e}{(R_\lambda + h)\cos\phi}
 * \f]
 * \f[
 *   \dot{h} = -v_d
 * \f]
 *
 * where:
 * - \f$\phi\f$ is the geodetic latitude,
 * - \f$\lambda\f$ is the geodetic longitude,
 * - \f$h\f$ is the geodetic altitude above the WGS-84 ellipsoid,
 * - \f$v_n, v_e, v_d\f$ are the NED velocities,
 * - \f$R_\phi\f$ is the meridional radius of curvature,
 * - \f$R_\lambda\f$ is the transverse (prime vertical) radius of curvature.
 *
 * Longitude does not appear explicitly as an input because the Earth
 * is modeled as an ellipsoid of revolution, which is rotationally
 * symmetric about the north–south axis.
 *
 * Latitude and longitude rates are returned in degrees per second
 * to remain consistent with the state representation.
 *
 * @param[in]  phi         Geodetic latitude (degrees).
 * @param[in]  h           Geodetic altitude above the WGS-84 ellipsoid (m).
 * @param[in]  vn          North velocity in the local NED frame (m/s).
 * @param[in]  ve          East velocity in the local NED frame (m/s).
 * @param[in]  vd          Down velocity in the local NED frame (m/s).
 * @param[out] llaDot      Time derivative of the LLA state (3×1):
 *                         \f$[\dot{\phi}\ \dot{\lambda}\ \dot{h}]^T\f$.
 * @param[out] llaDotBuff  User-provided buffer backing @p llaDot
 *                         (size = 3 floats).
 *
 * @note Internally, latitude is converted to radians for computation,
 *       but the output rates for latitude and longitude are expressed
 *       in degrees per second.
 */
void compute_lla_dot(float32_t phi, float32_t h, float32_t vn, float32_t ve, float32_t vd, arm_matrix_instance_f32* llaDot, float32_t llaDotBuff[3]) {
	float32_t computeRadiiResult[4];
	compute_radii(phi, computeRadiiResult);
	float32_t phiRad = deg2rad(phi);

	// radii of curvature of the circles tangent to the current location of the rocket
	// R_phi is the radius of the tangent circle lying in the meridional plane that also contains the rocket
	// R_lamb is the radius of the tangent circle lying in the plane of constant latitude that also contains the rocket
	float32_t R_phi = computeRadiiResult[0];
	float32_t R_lamb = computeRadiiResult[1];

	// linearization since we know v_tangent = r * theta_dot, simply rearrange to find theta_dot
	float32_t phidot = vn / (R_phi + h);
	float32_t lambadot = ve / ((R_lamb + h) * arm_cos_f32(phiRad));

	// convert back to degrees
	llaDotBuff[0] = rad2deg(phidot);
	llaDotBuff[1] = rad2deg(lambadot);
	llaDotBuff[2] = -vd;

	arm_mat_init_f32(llaDot, 3, 1, llaDotBuff);
}

/**
 * @brief Compute the time derivative of the NED velocity in the local tangent plane (LTP) frame.
 *
 * This function calculates the acceleration vector in the North-East-Down
 * (NED) coordinate frame due to gravity, Coriolis effects, centrifugal
 * forces, and measured acceleration. The computed derivative represents
 * \f$\dot{\mathbf{v}} = [\dot{v}_n, \dot{v}_e, \dot{v}_d]^T\f$ in
 * meters per second squared (m/s²).
 *
 * @param[in]  phi        Geodetic latitude (degrees).
 * @param[in]  h          Altitude above WGS-84 reference ellipsoid (meters).
 * @param[in]  vn         North velocity in LTP NED frame (m/s).
 * @param[in]  ve         East velocity in LTP NED frame (m/s).
 * @param[in]  vd         Down velocity in LTP NED frame (m/s).
 * @param[in]  ahat_n     Estimated specific force in the NED frame
 *                        \f$[a_n, a_e, a_d]^T\f$ (m/s²).
 * @param[in]  we         Earth rotation rate (rad/s).
 * @param[out] vdot       Time derivative of NED velocity (3×1).
 * @param[out] vdotBuff   User-provided buffer backing @p vdot
 *                        (size = 3 floats).
 */
void compute_vdot(float32_t phi, float32_t h, float32_t vn, float32_t ve, float32_t vd, float32_t ahat_n[3], float32_t we, arm_matrix_instance_f32* vdot, float32_t vdotBuff[3]) {
	float32_t an = ahat_n[0];
	float32_t ae = ahat_n[1];
	float32_t ad = ahat_n[2];

	float32_t computeRadiiResult[4];
	compute_radii(phi, computeRadiiResult);

	float32_t R_phi = computeRadiiResult[0];
	float32_t R_lamb = computeRadiiResult[1];

	// Compute gravity - Eqn 7.69c
	// This is an approximation accounting for flattening and centrifugal effects
	float32_t sin_phi = arm_sind_f32(phi);
	float32_t cos_phi = arm_cosd_f32(phi);
	float32_t sin_phi_sq = sin_phi * sin_phi;
	float32_t sin_2phi = arm_sind_f32(2.0f * phi);
	float32_t sin_2phi_sq = sin_2phi * sin_2phi;

	float32_t ghat = 9.780327f * (1.0f + 5.3024e-3f * sin_phi_sq - 5.8e-6f * sin_2phi_sq) - (3.0877e-6f - 4.4e-9f * sin_phi_sq) * h + 7.2e-14f * h * h;

	float32_t R_phi_h = R_phi + h;
	float32_t R_lamb_h = R_lamb + h;

	// Calculate vdot
	// Since the LTP coordinate frame is non-inertial, the influence of fictitious forces must be included
	// These include gravity and the Coriolis force (LTP is defined relative to ECEF not ECI, and ECEF is itself non-inertial)
	// Additionally, the LTP coordinate frame moves relative to ECEF if the NED velocity is nonzero
	float32_t vndot = -(ve / (R_lamb_h * cos_phi) + 2.0f * we) * ve * sin_phi + (vn * vd) / R_phi_h + an;

	float32_t vedot = (ve / (R_lamb_h * cos_phi) + 2.0f * we) * vn * sin_phi + (ve * vd) / R_lamb_h + 2.0f * we * vd * cos_phi + ae;

	float32_t vddot = -ve * ve / R_lamb_h - vn * vn / R_phi_h - 2.0f * we * ve * cos_phi + ghat + ad;

	vdotBuff[0] = vndot;
	vdotBuff[1] = vedot;
	vdotBuff[2] = vddot;

	// printf("A Hat: [%f, %f, %f]\n", an, ae, ad);
	// printf("V Dot: [%f, %f, %f] m/s^2 (in compute_vdot)\n", vndot, vedot, vddot);

	arm_mat_init_f32(vdot, 3, 1, vdotBuff);
}

/**
 * @brief Compute the continuous-time covariance time derivative \f$\dot{\mathbf{P}}\f$.
 *
 * This function computes the time rate of change of the state covariance
 * matrix \f$\mathbf{P}\f$ for a nonlinear Kalman filter by evaluating the
 * continuous-time Riccati equation:
 *
 * \f[
 *   \dot{\mathbf{P}} =
 *   \mathbf{F}\mathbf{P} +
 *   \mathbf{P}\mathbf{F}^T +
 *   \mathbf{G}\mathbf{Q}\mathbf{G}^T
 * \f]
 *
 * where:
 * - \f$\mathbf{F}\f$ is the state Jacobian (system dynamics matrix),
 * - \f$\mathbf{G}\f$ is the process noise input matrix,
 * - \f$\mathbf{Q}\f$ is the continuous-time process noise covariance.
 *
 * Unlike a linear discrete-time Kalman Filter, where the covariance is
 * updated directly, this nonlinear EKF propagates \f$\mathbf{P}\f$ forward
 * in time by first computing \f$\dot{\mathbf{P}}\f$ and then numerically
 * integrating it.
 *
 * This implementation:
 * - Computes \f$\mathbf{F}\f$ and \f$\mathbf{G}\f$ from the current state
 *   and sensor measurements
 * - Forms all matrix products explicitly using CMSIS-DSP routines
 * - Produces a 21×21 covariance derivative matrix suitable for time integration
 *
 * Matrix dimensions:
 * - State covariance \f$\mathbf{P}\f$: 21×21
 * - State Jacobian \f$\mathbf{F}\f$: 21×21
 * - Noise input matrix \f$\mathbf{G}\f$: 21×12
 * - Process noise covariance \f$\mathbf{Q}\f$: 12×12
 *
 * @param[in]  q         Attitude quaternion.
 * @param[in]  sf_a      Accelerometer scale factor error states.
 * @param[in]  sf_g      Gyroscope scale factor error states.
 * @param[in]  bias_g    Gyroscope bias states.
 * @param[in]  bias_a    Accelerometer bias states.
 * @param[in]  a_meas    Measured acceleration (m/s^2).
 * @param[in]  w_meas    Measured angular rate (rad/s).
 * @param[in]  P         Current state covariance matrix (21×21).
 * @param[in]  Q         Continuous-time process noise covariance (12×12).
 * @param[in]  phi       Geodetic latitude [rad].
 * @param[in]  h         Altitude above reference ellipsoid [m].
 * @param[in]  vn        North velocity [m/s].
 * @param[in]  ve        East velocity [m/s].
 * @param[in]  vd        Down velocity [m/s].
 * @param[in]  we        Earth rotation rate (rad/s).
 * @param[out] Pdot      Time derivative of the covariance matrix (21×21).
 * @param[out] PdotBuff  User-provided buffer backing @p Pdot
 *                       (size = 21×21 floats).
 */
void compute_Pdot(arm_matrix_instance_f32* q, arm_matrix_instance_f32* sf_a, arm_matrix_instance_f32* sf_g,
		  	  	  arm_matrix_instance_f32* bias_g, arm_matrix_instance_f32* bias_a, arm_matrix_instance_f32* a_meas,
				  arm_matrix_instance_f32* w_meas, arm_matrix_instance_f32* P, arm_matrix_instance_f32* Q,
				  float32_t phi, float32_t h, float32_t vn, float32_t ve, float32_t vd, float32_t we,
				  arm_matrix_instance_f32* Pdot, float32_t PdotBuff[21*21]) {
	// F (21 x 21) / G (21 x 12) / P (21 x 21) / Q (12 x 12) / FP (21 x 21) / PF' (21 x 21)

	arm_matrix_instance_f32 F, G;
	float32_t FBuff[21*21], GBuff[21*12]; // G is 21x12

	compute_F(q, sf_a, sf_g, bias_g, bias_a, phi, h, vn, ve, vd, a_meas, w_meas, we, &F, FBuff);
	compute_G(sf_g, sf_a, q, &G, GBuff);

	arm_matrix_instance_f32 FTrans, GTrans;
	float32_t FTransBuff[21*21], GTransBuff[12*21]; // G' is 12x21

	arm_mat_init_f32(&FTrans, 21, 21, FTransBuff);
	arm_mat_init_f32(&GTrans, 12, 21, GTransBuff);

	arm_mat_trans_f32(&F, &FTrans);    // FTrans = F'
	arm_mat_trans_f32(&G, &GTrans);    // GTrans = G'

	arm_matrix_instance_f32 FP, PF, GQ, GQGT;
	float32_t FPBuff[21*21], PFBuff[21*21], GQBuff[21*12], GQGTBuff[21*21];

	arm_mat_init_f32(&FP, 21, 21, FPBuff);
	arm_mat_init_f32(&PF, 21, 21, PFBuff);
	arm_mat_init_f32(&GQ, 21, 12, GQBuff);
	arm_mat_init_f32(&GQGT, 21, 21, GQGTBuff);
	arm_mat_init_f32(Pdot, 21, 21, PdotBuff);

	arm_mat_mult_f32(&F, P, &FP);          // FP = F * P
	arm_mat_mult_f32(P, &FTrans, &PF);     // PF = P * F'
	arm_mat_mult_f32(&G, Q, &GQ);          // GQ = G * Q
	arm_mat_mult_f32(&GQ, &GTrans, &GQGT);// term3 = G * Q * G'

	arm_mat_add_f32(&FP, &PF, &FP);        // FP = F*P + P*F'
	arm_mat_add_f32(&FP, &GQGT, Pdot);    // Pdot = F*P + P*F' + G*Q*G'
}

/**
 * @brief Propagate the EKF state and covariance using first-order (Euler) integration.
 *
 * This function performs the EKF time update (prediction step) by integrating
 * the continuous-time state and covariance dynamics over a single timestep dt.
 *
 * The state vector is assumed to have the form:
 *
 *   x = [ q(4x1)
 *         p(3x1)
 *         v(3x1)
 *         other states (12x1) ]
 *
 * where:
 *   - q is the attitude quaternion
 *   - p is position
 *   - v is velocity
 *   - remaining states (e.g., sensor biases, scale factors) are modeled as
 *     random constants (zero dynamics)
 *
 * The continuous-time state derivative is assembled as:
 *
 *   xDot = [ qdot
 *            pdot
 *            vdot
 *            0_(12x1) ]
 *
 * State propagation is performed using forward explicit Euler integration:
 *
 *   xMinus = x + dt * xDot
 *
 * The covariance is propogated using explicit Euler integration similarily:
 *
 *   Pminus = P + dt * Pdot
 *
 * @param[in]  x           Current state estimate (22x1)
 * @param[in]  P           Current state covariance (21x21)
 * @param[in]  qdot        Quaternion derivative (4x1)
 * @param[in]  pdot        Position derivative (3x1)
 * @param[in]  vdot        Velocity derivative (3x1)
 * @param[in]  Pdot        Covariance time derivative (21x21)
 * @param[in]  dt          Integration timestep [s]
 * @param[out] xMinus      Propagated (a priori) state estimate (22x1)
 * @param[out] Pminus      Propagated (a priori) covariance (21x21)
 * @param[out] xMinusBuff  Backing buffer for xMinus (length 22)
 * @param[out] PMinusBuff  Backing buffer for Pminus (length 21*21)
 */
void integrate(arm_matrix_instance_f32* x, arm_matrix_instance_f32* P, arm_matrix_instance_f32* qdot,
			   arm_matrix_instance_f32* pdot, arm_matrix_instance_f32* vdot, arm_matrix_instance_f32* Pdot,
			   float32_t dt, arm_matrix_instance_f32* xMinus, arm_matrix_instance_f32* Pminus,
			   float32_t xMinusBuff[22], float32_t PMinusBuff[21*21]) {


	// Assemble xDot = [qdot; pdot; vdot; zeros(12,1)] ---
	arm_matrix_instance_f32 xDot;
	float32_t xDotBuff[22] = {0}; // initialize to zero
	arm_mat_init_f32(&xDot, 22, 1, xDotBuff);

	// Copy qdot (4x1)
	for (uint8_t i = 0; i < 4; i++) {
		xDotBuff[i] = qdot->pData[i];
	}
	// Copy pdot (3x1)
	for (uint8_t i = 0; i < 3; i++) {
		xDotBuff[4 + i] = pdot->pData[i];
	}
	// Copy vdot (3x1)
	for (uint8_t i = 0; i < 3; i++) {
		xDotBuff[7 + i] = vdot->pData[i];
	}

	// Compute xMinus = x + dt * xDot ---
	arm_matrix_instance_f32 xDotScaled;
	float32_t xDotScaledBuff[22] = {0};
	arm_mat_init_f32(&xDotScaled, 22, 1, xDotScaledBuff);
	arm_mat_scale_f32(&xDot, dt, &xDotScaled);

	arm_mat_init_f32(xMinus, 22, 1, xMinusBuff);
	arm_mat_add_f32(x, &xDotScaled, xMinus);

	// Normalize quaternion (first 4 elements) ---
	arm_quaternion_normalize_f32(xMinus->pData, xMinus->pData, 1);  // in-place

	// Compute Pminus = P + dt * Pdot ---
	arm_matrix_instance_f32 PdotScaled;
	float32_t PDotScaledBuff[21*21] = {0};
	arm_mat_init_f32(&PdotScaled, P->numRows, P->numCols, PMinusBuff); // reuse buffer
	arm_mat_scale_f32(Pdot, dt, &PdotScaled);

	arm_mat_init_f32(Pminus, P->numRows, P->numCols, PMinusBuff);
	arm_mat_add_f32(P, &PdotScaled, Pminus);
}

/**
 * @brief Propagate the EKF state and covariance forward in time.
 *
 * This function performs the time-update (prediction) step of an Extended
 * Kalman Filter for an inertial navigation system. It computes the time
 * derivatives of the system states (attitude, position, velocity) and the
 * covariance matrix, then integrates them forward using first-order
 * Euler integration.
 *
 * The state vector includes:
 * - Attitude represented as a quaternion
 * - Position (latitude, longitude/altitude)
 * - Velocity in the navigation frame
 * - Gyroscope and accelerometer biases
 * - Gyroscope and accelerometer scale factors
 *
 * The covariance propagation accounts for sensor noise and process noise
 * provided via the process noise covariance matrix @p Q.
 *
 * @param[in]  xMinus      Pointer to the prior state vector.
 * @param[in]  PMinus      Pointer to the prior state covariance matrix.
 * @param[in]  what        Estimated body frame angular rate (rad/s).
 * @param[in]  aHatN       Estimated acceleration in the body frame (m/s^2).
 * @param[in]  wMeas       Measured angular rate from the gyroscope (rad/s).
 * @param[in]  aMeas       Measured acceleration from the accelerometer (m/s^2).
 * @param[in]  Q           Process noise covariance matrix.
 * @param[in]  dt          Propagation time step (seconds).
 * @param[in]  we          Earth rotation rate (rad/s).
 *
 * @param[out] xPlus       Pointer to the propagated (predicted) state vector.
 * @param[out] PPlus       Pointer to the propagated state covariance matrix.
 * @param[out] xPlusBuff   User-provided buffer backing @p xPlus (size = 22).
 * @param[out] PPlusBuff   User-provided buffer backing @p PPlus (size = 21×21).
 */

void propogate(arm_matrix_instance_f32* xMinus, arm_matrix_instance_f32* PMinus, arm_matrix_instance_f32* what,
			   arm_matrix_instance_f32* aHatN, arm_matrix_instance_f32* wMeas, arm_matrix_instance_f32* aMeas,
			   arm_matrix_instance_f32* Q, float32_t dt, float32_t we,
			   arm_matrix_instance_f32* xPlus, arm_matrix_instance_f32* PPlus, float32_t xPlusBuff[22],
			   float32_t PPlusBuff[21*21]) {

	arm_matrix_instance_f32 q, gBias, aBias, g_sf, a_sf;
	float32_t quatBuff[4], gBiasBuff[3], aBiasBuff[3], gSFBias[3], aSFBias[3];

	getStateQuaternion(xMinus, &q, quatBuff);
	getStateGBias(xMinus, &gBias, gBiasBuff);
	getStateABias(xMinus, &aBias, aBiasBuff);
	getStateGSF(xMinus, &g_sf, gSFBias);
	getStateASF(xMinus, &a_sf, aSFBias);

	arm_matrix_instance_f32 qdot, pdot, vdot, Pdot;
	float32_t qDotBuff[4], pDotBuff[3], vDotBuff[3], PdotBuff[21*21];

	float32_t phi = xMinus->pData[4];
	float32_t h = xMinus->pData[6];
	float32_t vn = xMinus->pData[7];
	float32_t ve = xMinus->pData[8];
	float32_t vd = xMinus->pData[9];

	// Computes the time derivative of our quaternion, position, velocity,
	// and covariance.

	compute_qdot(&q, what, &qdot, qDotBuff); // Comput
	compute_lla_dot(phi, h, vn, ve, vd, &pdot, pDotBuff);
	compute_vdot(phi, h, vn, ve, vd, aHatN->pData, we, &vdot, vDotBuff);
	compute_Pdot(&q, &a_sf, &g_sf, &gBias, &aBias, aMeas, wMeas, PMinus, Q,
				 phi, h, vn, ve, vd, we, &Pdot, PdotBuff);
				 
	//	printf("VDot:\n");
	//	printMatrix(&vdot);
	//	printf("Acceleration Measurement:\n");
	//	printMatrix(aMeas);
	//	printf("\n\n");

	// Integrate the states and covariance via Euler integration to determine the new state.
	// For example, given x0, the previous state, and the dxdt which is time derivative of our
	// states we can calculate the next state, x1.
	// x1 = dxdt * dt + x0

	integrate(xMinus, PMinus, &qdot, &pdot, &vdot, &Pdot, dt,
			  xPlus, PPlus, xPlusBuff, PPlusBuff);
}

