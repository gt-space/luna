#include "ekf.h"

/**
 * @brief Jacobian of dpdot (time rate of change of lla position) wrt lla position
 * 
 * @param[in] phi 			Latitude
 * @param[in] h   			Altitude
 * @param[in] vn  			North Velocity
 * @param[out] dpdot_dp 	Time Derivative of Position wrt Position
 * @param[out] dpDotBuff	Backing array for dpdot_dp
 */

void compute_dpdot_dp(float32_t phi, float32_t h, float32_t vn, float32_t ve, arm_matrix_instance_f32* dpdot_dp, float32_t dpDotBuff[9]) {

    float32_t computeRadiiResult[4];
    compute_radii(phi, computeRadiiResult);

	// radii of curvature of the circles tangent to the current location of the rocket
	// R_phi is the radius of the tangent circle lying in the meridional plane that also contains the rocket
	// R_lamb is the radius of the tangent circle lying in the plane of constant latitude that also contains the rocket
    float32_t R_phi = computeRadiiResult[0], R_lamb = computeRadiiResult[1];
    float32_t dR_phi_dphi = computeRadiiResult[2], dR_lamb_dphi = computeRadiiResult[3];

    float32_t square_phi  = (R_phi  + h) * (R_phi  + h);
    float32_t square_lamb = (R_lamb + h) * (R_lamb + h);

	float32_t sin_phi = arm_sind_f32(phi);
	float32_t cos_phi = arm_cosd_f32(phi);
	float32_t tan_phi = sin_phi / tan_phi;

	// Eqn 7.80a terms
    float32_t m11 = -vn / square_phi * dR_phi_dphi;
    float32_t m13 = rad2deg(-vn / square_phi);
    float32_t m21 = -(ve * arm_secd_f32(phi)) / square_lamb * dR_lamb_dphi
                    + (ve * arm_secd_f32(phi) * tan_phi) / (R_lamb + h);
    float32_t m23 = rad2deg(-ve * arm_secd_f32(phi) / square_lamb);

	// Assemble finl matrix
    dpDotBuff[0] = m11; dpDotBuff[1] = 0;    dpDotBuff[2] = m13;
    dpDotBuff[3] = m21; dpDotBuff[4] = 0;    dpDotBuff[5] = m23;
    dpDotBuff[6] = 0;   dpDotBuff[7] = 0;    dpDotBuff[8] = 0;

    arm_mat_init_f32(dpdot_dp, 3, 3, dpDotBuff);
}

/**
 * @brief Jacobian of dpdot (time rate of change of lla position) wrt ned velocity
 * 
 * @param phi 		Latitude
 * @param h			Altitude
 * @param dpdot_dv  Time Derivative of Position wrt Velocity
 * @param dpDotBuff	Backing array for dpdot_dv
 */
void compute_dpdot_dv(float32_t phi, float32_t h, arm_matrix_instance_f32* dpdot_dv, float32_t dpDotBuff[9]) {

	float32_t computeRadiiResult[4];
	compute_radii(phi, computeRadiiResult);

	float32_t R_phi = computeRadiiResult[0];
	float32_t R_lamb = computeRadiiResult[1];

	float32_t m11 = rad2deg(1.0f / (R_phi + h));
	float32_t m22 = rad2deg(arm_secd_f32(phi) / (R_lamb + h));

	dpDotBuff[0] = m11; dpDotBuff[1] = 0; 	dpDotBuff[2] = 0;
	dpDotBuff[3] = 0;	dpDotBuff[4] = m22; dpDotBuff[5] = 0;
	dpDotBuff[6] = 0;	dpDotBuff[7] = 0;	dpDotBuff[8] = -1;

	arm_mat_init_f32(dpdot_dv, 3, 3, dpDotBuff);
}

// used to calculate F
// 

/**
 * @brief Jacobian of dvdot (NED acceleration) wrt lla position
 * 
 * @param[in] phi			Latitude (deg)
 * @param[in] h				Altitude (m)
 * @param[in] vn			North Velocity (m/s)
 * @param[in] ve			East Velocity (m/s)
 * @param[in] vd			Down Velocity (m/s)
 * @param[in] we			Earth Rotation Rate (rad/s)
 * @param[out] dvdot_dp 	NED Acceleration wrt Position 
 * @param[out] dvDotBuff	Backing array for dvdot_dp
 */
void compute_dvdot_dp(float32_t phi, float32_t h, float32_t vn, float32_t ve, float32_t vd,
					  float32_t we,  arm_matrix_instance_f32* dvdot_dp, float32_t dvdotBuff[9]) {
    // Compute radii and derivatives
    float32_t computeRadiiResult[4];
    compute_radii(phi, computeRadiiResult);

    float32_t R_phi = computeRadiiResult[0], R_lamb = computeRadiiResult[1];
    float32_t dR_phi_dphi = computeRadiiResult[2], dR_lamb_dphi = computeRadiiResult[3];

    // Compute gravity derivatives
    float32_t gDgResult[3];

    float32_t phiRad = deg2rad(phi);
    compute_g_dg2(phiRad, h, gDgResult);
    float32_t dg_dphi = gDgResult[1];
    float32_t dg_dh = gDgResult[2];

    // Precompute frequently used terms
    float32_t sinphi = arm_sin_f32(phiRad);
    float32_t cosphi = arm_cos_f32(phiRad);
    float32_t secphi = 1.0f / cosphi;
    float32_t tanphi = sinphi / cosphi;
    float32_t secphi2 = secphi * secphi;

    float32_t Rphi_h  = R_phi  + h;
    float32_t Rlamb_h = R_lamb + h;
    float32_t Rphi_h2  = Rphi_h  * Rphi_h;
    float32_t Rlamb_h2 = Rlamb_h * Rlamb_h;

    // Compute matrix elements
    float32_t Y11 = -(ve*ve*secphi2)/(Rlamb_h)
                    + (ve*ve*tanphi)/(Rlamb_h2) * dR_lamb_dphi
                    - 2.0f * we * ve * cosphi
                    - (vn*vd)/(Rphi_h2) * dR_phi_dphi;

    float32_t Y13 = (ve*ve*tanphi)/(Rlamb_h2) - (vn*vd)/(Rphi_h2);

    float32_t Y21 = (ve*vn*secphi2)/(Rlamb_h)
                    - (ve*vn*tanphi)/(Rlamb_h2) * dR_lamb_dphi
                    + 2.0f * we * vn * cosphi
                    - (ve*vd)/(Rlamb_h2) * dR_lamb_dphi
                    - 2.0f * we * vd * sinphi;

    float32_t Y23 = -ve * ((vn*tanphi + vd) / Rlamb_h2);

    float32_t Y31 = (ve*ve)/(Rlamb_h2) * dR_lamb_dphi
                    + (vn*vn)/(Rphi_h2) * dR_phi_dphi
                    + 2.0f * we * ve * sinphi
                    + dg_dphi;

    float32_t Y33 = (ve*ve)/(Rlamb_h2) + (vn*vn)/(Rphi_h2) + dg_dh;

    // Fill CMSIS-DSP buffer (row-major order)
    dvdotBuff[0] = Y11; dvdotBuff[1] = 0.0f; dvdotBuff[2] = Y13;
    dvdotBuff[3] = Y21; dvdotBuff[4] = 0.0f; dvdotBuff[5] = Y23;
    dvdotBuff[6] = Y31; dvdotBuff[7] = 0.0f; dvdotBuff[8] = Y33;

    // Initialize CMSIS-DSP matrix
    arm_mat_init_f32(dvdot_dp, 3, 3, dvdotBuff);
}

/**
 * @brief Jacobian of dvdot (NED acceleration) wrt NED velocity
 * 
 * @param[in] phi			Latitude (deg)
 * @param[in] h				Altitude (m)
 * @param[in] vn			North Velocity (m/s)
 * @param[in] ve			East Velocity (m/s)
 * @param[in] vd			Down Velocity (m/s)
 * @param[in] we			Earth Sidereal Rotation (rad/s)
 * @param[out] dvdot_dv		NED Acceleration wrt NED velocity
 * @param[out] dvDotBuff	Backing array for dvdot_dv
 */
void compute_dvdot_dv(float32_t phi, float32_t h, float32_t vn, float32_t ve, float32_t vd,
					  float32_t we, arm_matrix_instance_f32* dvdot_dv, float32_t dvdotBuff[9]) {
    // Compute radii
    float32_t computeRadiiResult[4];
    compute_radii(phi, computeRadiiResult);

    float32_t R_phi = computeRadiiResult[0], R_lamb = computeRadiiResult[1];

    // Precompute frequently used terms
    float32_t sinphi = arm_sind_f32(phi);
    float32_t cosphi = arm_cosd_f32(phi);
	float32_t tanphi = sinphi / cosphi;

    float32_t Rphi_h  = R_phi  + h;
    float32_t Rlamb_h = R_lamb + h;

    // Compute matrix elements
    float32_t Z11 = vd / Rphi_h;
    float32_t Z12 = (-2.0f * ve * tanphi) / Rlamb_h + 2.0f * we * sinphi;
    float32_t Z13 = vn / Rphi_h;

    float32_t Z21 = (ve * tanphi) / Rlamb_h + 2.0f * we * sinphi;
    float32_t Z22 = (vd + vn * tanphi) / Rlamb_h;
    float32_t Z23 = ve / Rlamb_h + 2.0f * we * cosphi;

    float32_t Z31 = (-2.0f * vn) / Rphi_h;
    float32_t Z32 = (-2.0f * ve) / Rlamb_h - 2.0f * we * cosphi;

    // Fill CMSIS-DSP buffer (row-major order)
    dvdotBuff[0] = Z11; dvdotBuff[1] = Z12; dvdotBuff[2] = Z13;
    dvdotBuff[3] = Z21; dvdotBuff[4] = Z22; dvdotBuff[5] = Z23;
    dvdotBuff[6] = Z31; dvdotBuff[7] = Z32; dvdotBuff[8] = 0.0f;

    // Initialize CMSIS-DSP matrix
    arm_mat_init_f32(dvdot_dv, 3, 3, dvdotBuff);
}

/**
 * @brief Jacobian of body frame angular velocity wrt lla position
 * 
 * @param[in] phi			Latitude (deg)
 * @param[in] h				Altitude (m)
 * @param[in] ve			East Velocity (m/s)
 * @param[in] vn			North Velocity (m/s)
 * @param[in] we			Earth Rotational Speed (rad/s)
 * @param[out] dwdp			Body Frame Angular Velocity wrt Position
 * @param[out] dwdpBuffer	Backing buffer for dwdp
 */
void compute_dwdp(float32_t phi, float32_t h, float32_t ve, float32_t vn, float32_t we,
				  arm_matrix_instance_f32* dwdp, float32_t dwdpBuffer[9]) {

	float32_t computeRadiiResult[4];
	compute_radii(phi, computeRadiiResult);

    float32_t R_phi = computeRadiiResult[0], R_lamb = computeRadiiResult[1];
    float32_t dR_phi_dphi = computeRadiiResult[2], dR_lamb_dphi = computeRadiiResult[3];

    float32_t sin_phi = arm_sind_f32(phi);
    float32_t cos_phi = arm_cosd_f32(phi);
    float32_t tan_phi = arm_tand_f32(phi);
    float32_t sec_phi = arm_secd_f32(phi);
    float32_t sec_phi2 = sec_phi * sec_phi;

    float32_t RLh = R_lamb + h;
    float32_t RPh = R_phi + h;

    float32_t m11 = -we * sin_phi - ve / (RLh * RLh) * dR_lamb_dphi;
    float32_t m13 = -ve / (RLh * RLh);
    float32_t m21 =  vn / (RPh * RPh) * dR_phi_dphi;
    float32_t m23 =  vn / (RPh * RPh);
    float32_t m31 = -we * cos_phi
                    - (ve * sec_phi2) / RLh
                    + (ve * tan_phi / (RLh * RLh)) * dR_lamb_dphi;
    float32_t m33 = (ve * tan_phi) / (RLh * RLh);

    dwdpBuffer[0] = m11; dwdpBuffer[1] = 0.0f; dwdpBuffer[2] = m13;
    dwdpBuffer[3] = m21; dwdpBuffer[4] = 0.0f; dwdpBuffer[5] = m23;
    dwdpBuffer[6] = m31; dwdpBuffer[7] = 0.0f; dwdpBuffer[8] = m33;

    arm_mat_init_f32(dwdp, 3, 3, dwdpBuffer);
}

/**
 * @brief Compute the Jacobian of body-frame angular velocity with respect to NED velocity.
 *
 * This function computes the partial derivative
 * \f[
 *     \frac{\partial \boldsymbol{\omega}}{\partial \mathbf{v}_{n}}
 * \f]
 * which appears in the continuous-time state transition matrix (F-matrix)
 * for an inertial navigation system formulated in the NED frame.
 *
 * The Jacobian accounts for Earth curvature effects through the meridian
 * and transverse radii of curvature and depends on latitude and altitude.
 *
 * The resulting matrix has the form:
 * \f[
 * \frac{\partial \boldsymbol{\omega}}{\partial \mathbf{v}_{n}} =
 * \begin{bmatrix}
 *   0 & \frac{1}{R_\lambda + h} & 0 \\
 *  -\frac{1}{R_\phi + h} & 0 & 0 \\
 *   0 & -\frac{\tan\phi}{R_\lambda + h} & 0
 * \end{bmatrix}
 * \f]
 *
 * where:
 * - \f$\phi\f$ is geodetic latitude
 * - \f$h\f$ is altitude above the reference ellipsoid
 * - \f$R_\phi\f$ is the meridian radius of curvature
 * - \f$R_\lambda\f$ is the transverse radius of curvature
 *
 * @param[in]  phi          Geodetic latitude (degrees).
 * @param[in]  h            Altitude above the reference ellipsoid (meters).
 * @param[out] dwdv         Pointer to a 3×3 CMSIS-DSP matrix instance that will
 *                          contain the Jacobian \f$\partial \boldsymbol{\omega} / \partial \mathbf{v}_n\f$.
 * @param[out] dwdvBuffer   User-provided buffer backing @p dwdv (size = 9 floats).
 *
 * @note Trigonometric functions are evaluated in degrees using
 *       arm_sind_f32() and arm_cosd_f32().
 *
 * @note The output matrix is initialized in row-major order.
 *
 * @warning No singularity protection is performed for
 *          \f$\cos\phi \rightarrow 0\f$ (near the poles).
 */

void compute_dwdv(float32_t phi, float32_t h, arm_matrix_instance_f32* dwdv, float32_t dwdvBuffer[9]) {
	float32_t computeRadiiResult[4];
	compute_radii(phi, computeRadiiResult);

	float32_t R_phi = computeRadiiResult[0];
	float32_t R_lamb = computeRadiiResult[1];

	float32_t sin_phi = arm_sind_f32(phi);
	float32_t cos_phi = arm_cosd_f32(phi);
	float32_t tan_phi = sin_phi / cos_phi;

	float32_t m12 = 1 / (R_lamb + h);
	float32_t m21 = -1 / (R_phi + h);
	float32_t m32 = -tan_phi / (R_lamb + h);

	dwdvBuffer[0] = 0.0f; dwdvBuffer[1] = m12; dwdvBuffer[2] = 0.0f;
	dwdvBuffer[3] = m21; dwdvBuffer[4] = 0.0f; dwdvBuffer[5] = 0.0f;
	dwdvBuffer[6] = 0.0f; dwdvBuffer[7] = m32; dwdvBuffer[8] = 0.0f;

	arm_mat_init_f32(dwdv, 3, 3, dwdvBuffer);
}

/**
 * @brief Compute the continuous-time system dynamics matrix F.
 *
 * This function constructs the continuous-time state Jacobian
 * \f[
 *     \mathbf{F} = \frac{\partial \dot{\mathbf{x}}}{\partial \mathbf{x}}
 * \f]
 * such that
 * \f[
 *     \dot{\mathbf{x}} = \mathbf{F}\,\mathbf{x}.
 * \f]
 *
 * The matrix F is not used directly for state propagation; instead, it is
 * required for covariance propagation in the EKF time-update:
 * \f[
 *     \dot{\mathbf{P}} = \mathbf{F}\mathbf{P} + \mathbf{P}\mathbf{F}^\top + \mathbf{Q}.
 * \f]
 *
 * The full matrix is assembled as a 21×21 block matrix from analytically
 * derived sub-Jacobians corresponding to attitude, position, velocity,
 * sensor biases, and scale factors.
 *
 * ---
 * State vector ordering (21 states):
 * \f[
 * \mathbf{x} =
 * \begin{bmatrix}
 *   \boldsymbol{\theta} &
 *   \mathbf{p} &
 *   \mathbf{v} &
 *   \mathbf{s}_g &
 *   \mathbf{b}_a &
 *   \mathbf{s}_a
 * \end{bmatrix}^\top
 * \f]
 *
 * where:
 * - \f$\boldsymbol{\theta}\f$ : attitude error (3)
 * - \f$\mathbf{p}\f$          : position (latitude, longitude, altitude) (3)
 * - \f$\mathbf{v}\f$          : velocity in NED frame (3)
 * - \f$\mathbf{s}_g\f$        : gyroscope scale factors (3)
 * - \f$\mathbf{b}_g\f$        : gyroscope biases (3)
 * - \f$\mathbf{s}_a\f$        : accelerometer scale factors (3)
 *
 * ---
 * Block structure:
 *
 * \f[
 * \mathbf{F} =
 * \begin{bmatrix}
 *   \mathbf{F}_{11} & \mathbf{F}_{12} & \mathbf{F}_{13} & \mathbf{F}_{14} & \mathbf{0} & \mathbf{F}_{16} \\
 *   \mathbf{0}      & \mathbf{F}_{22} & \mathbf{F}_{23} & \mathbf{0}      & \mathbf{0} & \mathbf{0}      \\
 *   \mathbf{F}_{31} & \mathbf{F}_{32} & \mathbf{F}_{33} & \mathbf{0}      & \mathbf{F}_{35} & \mathbf{F}_{37} \\
 *   \mathbf{0}      & \mathbf{0}      & \mathbf{0}      & \mathbf{0}      & \mathbf{0}      & \mathbf{0}
 * \end{bmatrix}
 * \f]
 *
 * Each sub-block is computed using dedicated helper functions that account
 * for Earth curvature, transport rate, Coriolis effects, and sensor models.
 *
 * ---
 * @param[in]  q        Current attitude quaternion (body-to-navigation).
 * @param[in]  sf_a     Accelerometer scale factor vector.
 * @param[in]  sf_g     Gyroscope scale factor vector.
 * @param[in]  bias_g  Gyroscope bias vector.
 * @param[in]  bias_a  Accelerometer bias vector.
 * @param[in]  phi     Geodetic latitude (degrees).
 * @param[in]  h       Altitude above reference ellipsoid (meters).
 * @param[in]  vn      North velocity component (m/s).
 * @param[in]  ve      East velocity component (m/s).
 * @param[in]  vd      Down velocity component (m/s).
 * @param[in]  a_meas  Measured acceleration (m/s).
 * @param[in]  w_meas  Measured angular rate (rad/s).
 * @param[in]  we      Earth rotation rate (rad/s).
 *
 * @param[out] F       Pointer to the resulting 21×21 system dynamics matrix.
 * @param[out] FBuff   User-provided buffer backing @p F (size = 21×21).
 *
 */

void compute_F(arm_matrix_instance_f32* q, arm_matrix_instance_f32* sf_a, arm_matrix_instance_f32* sf_g,
			  arm_matrix_instance_f32* bias_g, arm_matrix_instance_f32* bias_a, float32_t phi, float32_t h,
			  float32_t vn, float32_t ve, float32_t vd, arm_matrix_instance_f32* a_meas, arm_matrix_instance_f32* w_meas,
			  float32_t we, arm_matrix_instance_f32* F, float32_t FBuff[21*21]) {

	float32_t dnbBuff[9], dbnBuff[9], offsetResultData[3], F11Data[9];
	arm_matrix_instance_f32 D_nb, D_bn, F11, dwdp;

	arm_mat_init_f32(&D_bn, 3, 3, dbnBuff);
	arm_mat_init_f32(&D_nb, 3, 3, dnbBuff);

	quaternion2DCM(q, &D_nb, dnbBuff);
	arm_mat_trans_f32(&D_nb, &D_bn);

//	arm_offset_f32(sf_g->pData, 1.0f, offsetResultData, 3);
//
//	for (uint8_t i = 0; i < sizeof(offsetResultData) / sizeof(float32_t); i++) {
//		offsetResultData[i] = -1 / offsetResultData[i];
//	}
//
//	float32_t wMeasBiasGDiff[3];
//	arm_sub_f32(w_meas->pData, bias_g->pData, wMeasBiasGDiff, 3);
//	arm_mult_f32(offsetResultData, wMeasBiasGDiff, wMeasBiasGDiff, 3);

	for (uint8_t i = 0; i < 3; i++) {
		offsetResultData[i] = -1 / (1 + sf_g->pData[i]) * (w_meas->pData[i] - bias_g->pData[i]);
	}

	arm_mat_init_f32(&F11, 3, 3, F11Data);
	arm_mat_skew_f32(&(arm_matrix_instance_f32){3, 1, offsetResultData}, &F11, F11Data);

	// compute_dwdp(float32_t phi, float32_t h, float32_t ve, float32_t vn, float32_t we, arm_matrix_instance_f32* dwdp, float32_t dwdpBuffer[9])

	float32_t dwdpBuff[9];
	compute_dwdp(phi, h, ve, vn, we, &dwdp, dwdpBuff);

	arm_matrix_instance_f32 F12;
	float32_t F12Data[9];
	arm_mat_init_f32(&F12, 3, 3, F12Data);
	arm_mat_scale_f32(&D_bn, -1.0f, &D_bn);
	arm_mat_mult_f32(&D_bn, &dwdp, &F12);

	arm_matrix_instance_f32 dwdv, F13;
	float32_t dwdvBuff[9], F13Data[9];
	compute_dwdv(phi, h, &dwdv, dwdvBuff);
	arm_mat_init_f32(&F13, 3, 3, F13Data);
	arm_mat_mult_f32(&D_bn, &dwdv, &F13);

	arm_matrix_instance_f32 F14, Omega, Bg, F16;
	float32_t F14Data[9], OmegaData[9], inverseSFG[3], BgData[9], F16Data[9];

	arm_mat_init_f32(&F16, 3, 3, F16Data);

	for (uint8_t i = 0; i < sizeof(inverseSFG) / sizeof(float32_t); i++) {
		inverseSFG[i] = -1 / (sf_g->pData[i] + 1);
	}

	arm_mat_get_diag_f32(&(arm_matrix_instance_f32){3, 1, inverseSFG}, &F14, F14Data);
	arm_mat_get_diag_f32(w_meas, &Omega, OmegaData);
	arm_mat_get_diag_f32(bias_g, &Bg, BgData);

	arm_mat_sub_f32(&Omega, &Bg, &F16);
	arm_mat_scale_f32(&F16, -1.0f, &F16);

	arm_matrix_instance_f32 F22, F23;
	float32_t dpdot_dpData[9], dpdot_dvData[9];
	compute_dpdot_dp(phi, h, vn, ve, &F22, dpdot_dpData);
	compute_dpdot_dv(phi, h, &F23, dpdot_dvData);

	arm_matrix_instance_f32 ahat_n, ahatB, ahatBSkew;
	float32_t ahat_nData[9], ahatBData[9], ahatBSkewVar[9];
	arm_mat_init_f32(&ahatB, 3, 1, ahatBData);
	arm_mat_init_f32(&ahatBSkew, 3, 3, ahatBSkewVar);

	compute_ahat(q, sf_a, bias_a, a_meas, &ahat_n, ahat_nData);
	arm_mat_scale_f32(&D_bn, -1.0f, &D_bn);
	arm_mat_mult_f32(&D_bn, &ahat_n, &ahatB);

	arm_matrix_instance_f32 F31, F32, F33;
	float32_t dvdot_dpData[9], dvdot_dvData[9], F31Data[9], F32Data[9], F33Data[9];
	arm_mat_init_f32(&F31, 3, 3, F31Data);
	arm_mat_init_f32(&F32, 3, 3, F32Data);
	arm_mat_init_f32(&F33, 3, 3, F33Data);

	arm_mat_skew_f32(&ahatB, &ahatBSkew, ahatBSkewVar);
	arm_mat_scale_f32(&D_nb, -1.0f, &D_nb);
	arm_mat_mult_f32(&D_nb, &ahatBSkew, &F31);

	compute_dvdot_dp(phi, h, vn, ve, vd, we, &F32, dvdot_dpData);
	compute_dvdot_dv(phi, h, vn, ve, vd, we, &F33, dvdot_dvData);

	arm_matrix_instance_f32 F35, F37, measDiff;
	float32_t F35Data[9], F37Data[9], measDiffBuff[9];
	arm_mat_init_f32(&F35, 3, 3, F35Data);
	arm_mat_init_f32(&F37, 3, 3, F37Data);

	float32_t inverseSFa[9] = {(1.0f / (1 + sf_a->pData[0])), 0, 0, 0, (1.0f / (1 + sf_a->pData[1])), 0, 0, 0, (1.0f / (1 + sf_a->pData[2]))};
	float32_t tempBuff[3];

	arm_mat_mult_f32(&D_nb, &(arm_matrix_instance_f32){3, 3, inverseSFa}, &F35);
	arm_mat_sub_f32(a_meas, bias_a, &(arm_matrix_instance_f32){3, 1, tempBuff});
	arm_mat_get_diag_f32(&(arm_matrix_instance_f32){3, 1, tempBuff}, &measDiff, measDiffBuff);
	arm_mat_mult_f32(&D_nb, &measDiff, &F37);

	memset(FBuff, 0, 21 * 21 * sizeof(float32_t));
	arm_mat_init_f32(F, 21, 21, FBuff);

	arm_mat_place_f32(&F11, F, 0, 0);
	arm_mat_place_f32(&F12, F, 0, 3);
	arm_mat_place_f32(&F13, F, 0, 6);
	arm_mat_place_f32(&F14, F, 0, 9);
	arm_mat_place_f32(&F16, F, 0, 15);

	// Row block 2 (rows 3–5)
	arm_mat_place_f32(&F22, F, 3, 3);
	arm_mat_place_f32(&F23, F, 3, 6);

	// Row block 3 (rows 6–8)
	arm_mat_place_f32(&F31, F, 6, 0);
	arm_mat_place_f32(&F32, F, 6, 3);
	arm_mat_place_f32(&F33, F, 6, 6);
	arm_mat_place_f32(&F35, F, 6, 12);
	arm_mat_place_f32(&F37, F, 6, 18);
}

/**
 * @brief Compute the continuous-time process noise mapping matrix G.
 *
 * This function constructs the continuous-time noise influence matrix
 * \f[
 *     \mathbf{G}
 * \f]
 * which maps the continuous-time white noise vector into the state
 * derivative equations:
 * \f[
 *     \dot{\mathbf{x}} = \mathbf{F}\mathbf{x} + \mathbf{G}\mathbf{w}
 * \f]
 *
 * The matrix G is required to propagate the covariance matrix through
 * continuous-time process noise:
 * \f[
 *     \dot{\mathbf{P}} =
 *     \mathbf{F}\mathbf{P} + \mathbf{P}\mathbf{F}^T +
 *     \mathbf{G}\mathbf{Q}_c\mathbf{G}^T
 * \f]
 *
 * where \f$\mathbf{Q}_c\f$ is the continuous-time process noise covariance.
 *
 * The resulting G matrix has dimensions 21×12 and reflects how gyroscope
 * noise, accelerometer noise, and sensor bias random walks drive the
 * system states.
 *
 * The assumed noise vector ordering is:
 * \f[
 * \mathbf{w} =
 * \begin{bmatrix}
 * \mathbf{n}_g &
 * \mathbf{n}_{b_g} &
 * \mathbf{n}_a &
 * \mathbf{n}_{b_a}
 * \end{bmatrix}^T
 * \f]
 * where each block is 3-dimensional.
 *
 * @param[in]  sf_g     Gyroscope scale factor vector.
 * @param[in]  sf_a     Accelerometer scale factor vector.
 * @param[in]  q        Attitude quaternion (body → navigation).
 *
 * @param[out] G        Pointer to the resulting 21×12 noise mapping matrix.
 * @param[out] GBuff    User-provided buffer backing @p G
 *                      (size = 21×12 floats)
 */

void compute_G(arm_matrix_instance_f32* sf_g, arm_matrix_instance_f32* sf_a, arm_matrix_instance_f32* q,
			   arm_matrix_instance_f32* G, float32_t GBuff[21*12]) {

	arm_matrix_instance_f32 D_bn, G11, diagSFa, G33, eye3;
	float32_t DbnBuff[9], G11Buff[9], diagSFaBuff[9], G33Buff[9], eye3Buff[9];

	quaternion2DCM(q, &D_bn, DbnBuff);

	float32_t sfGInv[3];
	for (int i = 0; i < 3; i++) {
		sfGInv[i] = -1 / (1 + sf_g->pData[i]);
	}

	float32_t sfAInv[3];
	for (int i = 0; i < 3; i++) {
		sfAInv[i] = 1 / (1 + sf_a->pData[i]);
	}

	arm_mat_get_diag_f32(&(arm_matrix_instance_f32){3, 1, sfGInv}, &G11, G11Buff);
	arm_mat_get_diag_f32(&(arm_matrix_instance_f32){3, 1, sfAInv}, &diagSFa, diagSFaBuff);

	arm_mat_init_f32(&G33, 3, 3, G33Buff);
	arm_mat_scale_f32(&D_bn, -1, &D_bn);
	arm_mat_mult_f32(&D_bn, &diagSFa, &G33);

	arm_mat_eye_f32(&eye3, eye3Buff, 3);

    memset(GBuff, 0, 21 * 12 * sizeof(float32_t));
    arm_mat_init_f32(G, 21, 12, GBuff);

	// G11 -> (0, 0)
    arm_mat_place_f32(&G11, G, 0, 0);

    // G33 → (12,6)
    arm_mat_place_f32(&G33, G, 6, 6);

    // eye3 → (9, 3)
    arm_mat_place_f32(&eye3, G, 9, 3);

    // eye3 → (12, 9)
    arm_mat_place_f32(&eye3, G, 15, 9);
}
