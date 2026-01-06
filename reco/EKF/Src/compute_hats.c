#include "ekf.h"

// because the body frame is non-inertial, there is some extra angular velocity caused by its transport
void compute_wn(float32_t phi, float32_t h, float32_t vn, float32_t ve, arm_matrix_instance_f32* wn, float32_t we, float32_t buffer[3]) {

	float32_t computeRadiiVec[4];
	compute_radii(phi, computeRadiiVec);

	// radii of curvature of the circles tangent to the current location of the rocket
	// R_phi is the radius of the tangent circle lying in the meridional plane that also contains the rocket
	// R_lamb is the radius of the tangent circle lying in the plane of constant latitude that also contains the rocket
	float32_t R_phi = computeRadiiVec[0];
	float32_t R_lamb =  computeRadiiVec[1];

	arm_matrix_instance_f32 vec1;
	float32_t term1[3] = {we * arm_cosd_f32(phi), 0, -we * arm_sind_f32(phi)};
	arm_mat_init_f32(&vec1, 3, 1, term1);

	arm_matrix_instance_f32 vec2;
	float32_t term2[3] = {ve / (R_lamb + h), -vn / (R_phi + h), -(ve * arm_tand_f32(phi)) / (R_lamb + h)};
	arm_mat_init_f32(&vec2, 3, 1, term2);

	arm_mat_init_f32(wn, 3, 1, buffer);
	arm_add_f32(vec1.pData, vec2.pData, buffer, 3);
}

// Compensates the angular velocity given directly by the IMU
// The goal is to find an accurate body frame angular velocity in rad/s
// The direct measurement given by the IMU is not reliable due to the influence of gyro bias and gyro scale factors
// Additionally, because the body frame is a non-inertial frame, we must compensate for this
void compute_what(arm_matrix_instance_f32* q, arm_matrix_instance_f32* bias_g, arm_matrix_instance_f32* sf_g,
				  float32_t phi, float32_t h, float32_t vn, float32_t ve, float32_t we, arm_matrix_instance_f32* w_meas,
				  arm_matrix_instance_f32* what, float32_t whatBuffer[3]) {

	arm_matrix_instance_f32 D_bn, D_bnT, wn, wnBody;
	float32_t D_bn_buff[9], wnBuff[3], productBuff[3], D_bnTData[9], sf_g_temp[3], w_corrected[3], wnBodyBuff[3];

	memcpy(sf_g_temp, sf_g->pData, 3*sizeof(float32_t));
	arm_offset_f32(sf_g_temp, 1, sf_g_temp, 3);

	// gyro measurement model is as follows:
	// w_raw = scale factor * w_corrected + w_bias
	// where w_raw is the value taken directly from the gyro
	// we have w_raw and want to find w_corrected
	// therefore we must first subtract w_bias from the raw IMU data, w_raw
	// then we divide by scale factor
	for (uint8_t i = 0; i < 3; i++) {
		w_corrected[i] = (w_meas->pData[i] - bias_g->pData[i]) / sf_g_temp[i];
	}

	// because the body frame is non-inertial, there is some extra angular velocity caused by its transport
	// we need to get rid of this by subtracting it from the measured angular velocity
	// compute the transport rate caused by the non-inertial nature of the rocket body frame
	compute_wn(phi, h, vn, ve, &wn, we, wnBuff);

	// we need to now subtract this from the measured angular velocity
	// however we calculated the transport rate in an outside frame
	// in order to subtract it from the body frame angular velocity, we need a body frame transport rate
	// in order to find this, we need the transformation matrix between the outside frame to the body frame
	
	// first we find the DCM that will take us from body frame to outside frame
	quaternion2DCM(q, &D_bn, D_bn_buff);

	// we want the opposite of this (outside frame to body frame), so we actually need to take the inverse of this DCM
	// inverting a matrix is computationally expensive and may run into numerical issues
	// luckily, a DCM is always invertible, and its inverse is just its transpose
	arm_mat_init_f32(&D_bnT, 3, 3, D_bnTData);
	arm_mat_trans_f32(&D_bn, &D_bnT);

	// multiply the outside frame transport rate angular velocity vector by the new transformation matrix
	arm_mat_init_f32(&wnBody, 3, 1, wnBodyBuff);
	arm_mat_mult_f32(&D_bnT, &wn, &wnBody);

	// subtract the body frame transport rate angular velocity vector from the corrected measured angular velocity vector
	arm_mat_init_f32(what, 3, 1, whatBuffer);
	arm_sub_f32(w_corrected, wnBody.pData, what->pData, 3);
}

// Compensates the linear acceleration given directly by the IMU
// The goal is to find an accurate body frame acceleration in m/s^2
// The direct measurement given by the IMU is not reliable due to the influence of accelerometer bias and accelerometer scale factors
void compute_ahat(arm_matrix_instance_f32* q, arm_matrix_instance_f32* sf_a, arm_matrix_instance_f32* bias_a, arm_matrix_instance_f32* a_meas, arm_matrix_instance_f32* ahat_n, float32_t ahatBuff[3]) {
	arm_matrix_instance_f32 D_bn, aBody;
	float32_t D_bn_buff[9], result[3], resultData1[3];

	quaternion2DCM(q, &D_bn, D_bn_buff);

	arm_offset_f32(sf_a->pData, 1.0f, resultData1, 3);

	// accelerometer measurement model is as follows:
	// a_raw = scale factor * a_body + a_bias
	// where a_raw is the value taken directly from the accelerometer
	// we have a_raw and want to find a_body
	// therefore we must first subtract a_bias from the raw IMU data, a_raw
	// then we divide by scale factor
	arm_mat_init_f32(&aBody, 3, 1, result);
	for (uint8_t i = 0; i < 3; i++) {
	    aBody.pData[i] = (a_meas->pData[i] - bias_a->pData[i]) / resultData1[i];
	}

	arm_mat_init_f32(ahat_n, 3, 1, ahatBuff);

	// however because we measure velocity in LTP coordinate frame NED (north east down) velocities, we must do a coordinate transformation
	// our acceleration is in body frame, but we want NED (outside frame) acceleration
	// so we calculated a DCM earlier (using quaternion2DCM) to go from body frame to outside frame
	// we just multiply a_body by this DCM we calculated in order to get the NED frame acceleration
	arm_mat_mult_f32(&D_bn, &aBody, ahat_n);
}
