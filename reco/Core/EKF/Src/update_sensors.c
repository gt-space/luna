#include "ekf.h"

static float32_t eye6Buff[6*6] = {1,0,0,0,0,0,
								  0,1,0,0,0,0,
								  0,0,1,0,0,0,
								  0,0,0,1,0,0,
								  0,0,0,0,1,0,
								  0,0,0,0,0,1};

static float32_t eye21Buff[21*21] = {1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
									 0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
									 0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
									 0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
									 0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
									 0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
									 0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
									 0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,
									 0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,
									 0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,
									 0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,
									 0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,
									 0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,
									 0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,
									 0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,
									 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,
									 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,
									 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,
									 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,
									 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,
									 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1};

const arm_matrix_instance_f32 eye6 = {.numCols=6, .numRows=6, .pData = eye6Buff};
const arm_matrix_instance_f32 eye21 = {.numCols=21, .numRows=21, .pData = eye21Buff};

// we call this if GPS measurement comes in

// we call this if GPS measurement comes in
void update_GPS(const arm_matrix_instance_f32* x_minus, const arm_matrix_instance_f32* P_minus, const arm_matrix_instance_f32* H,
				const arm_matrix_instance_f32* R, const arm_matrix_instance_f32* lla_meas, arm_matrix_instance_f32* x_plus,
				arm_matrix_instance_f32* P_plus, float32_t xPlusData[22*1], float32_t P_plus_data[21*21] PERF_ARG) {

	PERF_START(1);
	
	// STEP 2: KALMAN GAIN
	float32_t trace_sub = P_minus->pData[3 * 21 + 3] + P_minus->pData[4 * 21 + 4] + P_minus->pData[5 * 21 + 5];
	float32_t beta = (trace_sub > 1000.0f) ? 0.25f : 0.0f;
	float32_t sqrt_beta = sqrtf(1.0f + beta);
	
	arm_matrix_instance_f32 HT_mat;
	float32_t HT_data[63];
	arm_mat_init_f32(&HT_mat, 21, 3, HT_data);
	arm_mat_trans_f32(H, &HT_mat);

	// Compute S = (1+beta)*H*P_minus*H' + R
	arm_matrix_instance_f32 HP_mat;
	float32_t HP_data[63];
	arm_mat_init_f32(&HP_mat, 3, 21, HP_data);
	arm_mat_mult_f32(H, P_minus, &HP_mat);

	arm_matrix_instance_f32 W_temp_mat;
	float32_t W_temp_data[9];
	arm_mat_init_f32(&W_temp_mat, 3, 3, W_temp_data);
	arm_mat_mult_f32(&HP_mat, &HT_mat, &W_temp_mat);

	float32_t W_data[9];
	for (int i = 0; i < 9; i++) {
		W_data[i] = (1.0f + beta) * W_temp_data[i] + R->pData[i];
	}

	// Compute PHT = P_minus*H'
	arm_matrix_instance_f32 PHT_mat;
	float32_t PHT_data[63];
	arm_mat_init_f32(&PHT_mat, 21, 3, PHT_data);
	arm_mat_mult_f32(P_minus, &HT_mat, &PHT_mat);

	// Compute K = P_minus*H'/ S
	arm_matrix_instance_f32 S_mat;
	arm_mat_init_f32(&S_mat, 3, 3, W_data);

	arm_matrix_instance_f32 K_mat;
	float32_t K_data[63];
	arm_mat_init_f32(&K_mat, 21, 3, K_data);

	float64_t KDoubleData[21*3], SMatDoubleData[3*3], PHTMatDoubleData[21*3];
	arm_matrix_instance_f64 KDouble, SMatDouble, PHTMatDouble;

	arm_mat_init_f64(&SMatDouble, 3, 3, SMatDoubleData);
	arm_mat_init_f64(&PHTMatDouble, 21, 3, PHTMatDoubleData);
	copyMatrixDouble(&S_mat, &SMatDouble);
	copyMatrixDouble(&PHT_mat, &PHTMatDouble);

	PERF_START(2);
	arm_mat_linsolve_right_f64(&PHTMatDouble, &SMatDouble, &KDouble, KDoubleData);
	PERF_END(PERF_LINSOLVE_GPS, 2);

	arm_mat_init_f32(&K_mat, 21, 3, K_data);
	copyMatrixFloat(&KDouble, &K_mat);

	// STEP 3: MEASUREMENT UPDATE
	float32_t residual_data[3];
	arm_sub_f32(lla_meas->pData, &x_minus->pData[4], residual_data, 3);

	arm_matrix_instance_f32 residual_vec, Delta_x_vec;
	float32_t Delta_x_data[21];
	arm_mat_init_f32(&residual_vec, 3, 1, residual_data);
	arm_mat_init_f32(&Delta_x_vec, 21, 1, Delta_x_data);
	arm_mat_mult_f32(&K_mat, &residual_vec, &Delta_x_vec);

	arm_mat_init_f32(x_plus, 22, 1, xPlusData);
	memcpy(&x_plus->pData[0], &x_minus->pData[0], 4 * sizeof(float32_t));
	arm_add_f32(&x_minus->pData[4],  &Delta_x_vec.pData[3],  &x_plus->pData[4],  3);
	arm_add_f32(&x_minus->pData[7],  &Delta_x_vec.pData[6],  &x_plus->pData[7],  3);
	arm_add_f32(&x_minus->pData[10], &Delta_x_vec.pData[9],  &x_plus->pData[10], 3);
	arm_add_f32(&x_minus->pData[13], &Delta_x_vec.pData[12], &x_plus->pData[13], 3);
	arm_add_f32(&x_minus->pData[16], &Delta_x_vec.pData[15], &x_plus->pData[16], 3);
	arm_add_f32(&x_minus->pData[19], &Delta_x_vec.pData[18], &x_plus->pData[19], 3);

	// PosteriorM = [ SigmaMinus * (I - KH)^T ; V * K^T ]
	float32_t KH_data[441];
	arm_matrix_instance_f32 KH_mat;
	arm_mat_init_f32(&KH_mat, 21, 21, KH_data);
	arm_mat_mult_f32(&K_mat, H, &KH_mat);

	float32_t I_KH_data[441];
	arm_matrix_instance_f32 I_KH_mat;
	arm_mat_init_f32(&I_KH_mat, 21, 21, I_KH_data);
	arm_mat_sub_f32(&eye21, &KH_mat, &I_KH_mat);

	float32_t I_KH_T_data[441];
	arm_matrix_instance_f32 I_KH_T_mat;
	arm_mat_init_f32(&I_KH_T_mat, 21, 21, I_KH_T_data);
	arm_mat_trans_f32(&I_KH_mat, &I_KH_T_mat);

	float32_t Sigma_IKHT_data[441];
	arm_matrix_instance_f32 Sigma_IKHT_mat;
	arm_mat_init_f32(&Sigma_IKHT_mat, 21, 21, Sigma_IKHT_data);
	arm_mat_mult_f32(P_minus, &I_KH_T_mat, &Sigma_IKHT_mat);
	
	float32_t V_data[9] = {0};
	// R is diagonal, V is sqrt(R)
	for(int i=0; i<3; i++) V_data[i*3 + i] = sqrtf(R->pData[i*3 + i]);
	arm_matrix_instance_f32 V_mat;
	arm_mat_init_f32(&V_mat, 3, 3, V_data);

	float32_t KT_data[63];
	arm_matrix_instance_f32 KT_mat;
	arm_mat_init_f32(&KT_mat, 3, 21, KT_data);
	arm_mat_trans_f32(&K_mat, &KT_mat);
	
	float32_t VKT_data[63];
	arm_matrix_instance_f32 VKT_mat;
	arm_mat_init_f32(&VKT_mat, 3, 21, VKT_data);
	arm_mat_mult_f32(&V_mat, &KT_mat, &VKT_mat);

	float32_t PosteriorM_data[24*21] = {0};
	arm_matrix_instance_f32 PosteriorM;
	arm_mat_init_f32(&PosteriorM, 24, 21, PosteriorM_data);
	
	for(int r=0; r<21; r++) {
		for(int c=0; c<21; c++) {
			PosteriorM_data[r*21 + c] = Sigma_IKHT_data[r*21 + c];
		}
	}
	for(int r=0; r<3; r++) {
		for(int c=0; c<21; c++) {
			PosteriorM_data[(r+21)*21 + c] = VKT_data[r*21 + c];
		}
	}
	
	float32_t R_posterior_data[24*21];
	float32_t Q_posterior_data[24*24];
	float32_t tau_posterior[21];
	arm_matrix_instance_f32 R_posterior, Q_posterior;
	arm_mat_init_f32(&R_posterior, 24, 21, R_posterior_data);
	arm_mat_init_f32(&Q_posterior, 24, 24, Q_posterior_data);
	
	arm_mat_qr_f32(&PosteriorM, 0.0f, &R_posterior, &Q_posterior, tau_posterior, KH_data, I_KH_data);

	arm_mat_init_f32(P_plus, 21, 21, P_plus_data);
	for(int r=0; r<21; r++) {
		for(int c=0; c<21; c++) {
			P_plus_data[r*21 + c] = R_posterior_data[r*21 + c];
		}
	}

	PERF_END(PERF_GPS, 1);
}



// - x_minus: (22 x 1)
// - P_minus: (21 x 21)
// - Pq_minus: (6 x 6)
// - Hq: (3 x 6)
// - Rq: (3 x 3)
// - R: (3 x 3)
// - magI: (3 x 1)
// - mag_meas: (3 x 1)
// we are not using this for vespula because of tel

void update_mag(const arm_matrix_instance_f32* x_minus, const arm_matrix_instance_f32* P_minus, const arm_matrix_instance_f32* Rq,
				const arm_matrix_instance_f32* magI, const arm_matrix_instance_f32* mag_meas, arm_matrix_instance_f32* x_plus,
				arm_matrix_instance_f32* P_plus, float32_t x_plus_buff[22*1], float32_t P_plus_buff[21*21] PERF_ARG) {

	PERF_START(1);

	float32_t qBuff[4], n2b[3*3], b2n[3*3], magBBuff[3*1], magBSkewBuff[3*3];
	arm_matrix_instance_f32 q, DCMn2b, DCMb2n, magB, magBSkew, H;

	arm_mat_init_f32(&DCMb2n, 3, 3, b2n);

	getStateQuaternion(x_minus, &q, qBuff);
	quaternion2DCM(&q, &DCMn2b, n2b PERF_PASS);
	arm_mat_trans_f32(&DCMn2b, &DCMb2n);

	arm_mat_init_f32(&magB, 3, 1, magBBuff);
	arm_mat_mult_f32(&DCMb2n, magI, &magB);
	arm_mat_skew_f32(&magB, &magBSkew, magBSkewBuff);

	float32_t HBuff[3*21] = {0};
	arm_mat_init_f32(&H, 3, 21, HBuff);
	arm_mat_place_f32(&magBSkew, &H, 0, 0);

	arm_matrix_instance_f32 K, A, B;
	float32_t KBuff[21*3], HT[21*3], PMinusHT[21*3],
			  HPMinus[3*21], HPMinusHT[3*3], HPMinusHTR[3*3];

	arm_mat_trans_f32(&H, &(arm_matrix_instance_f32){21, 3, HT});

	arm_mat_mult_f32(P_minus, &(arm_matrix_instance_f32){21, 3, HT}, &(arm_matrix_instance_f32){21, 3, PMinusHT});

	arm_mat_mult_f32(&H, P_minus, &(arm_matrix_instance_f32){3, 21, HPMinus});

	arm_mat_mult_f32(&(arm_matrix_instance_f32){3, 21, HPMinus},
					 &(arm_matrix_instance_f32){21, 3, HT},
					 &(arm_matrix_instance_f32){3, 3, HPMinusHT});

	arm_mat_add_f32(&(arm_matrix_instance_f32){3, 3, HPMinusHT},
					Rq,
					&(arm_matrix_instance_f32){3, 3, HPMinusHTR});

	arm_mat_init_f32(&A, 3, 3, HPMinusHTR);
	arm_mat_init_f32(&B, 21, 3, PMinusHT);
	arm_mat_init_f32(&K, 21, 3, KBuff);

	arm_matrix_instance_f64 PHTMatDouble, SMatDouble, KDouble;
	float64_t PHTMatDoubleData[21*3], SMatDoubleData[3*3], KDoubleData[21*3];

	arm_mat_init_f64(&SMatDouble, 3, 3, SMatDoubleData);
	arm_mat_init_f64(&PHTMatDouble, 21, 3, PHTMatDoubleData);

	copyMatrixDouble(&A, &SMatDouble);
	copyMatrixDouble(&B, &PHTMatDouble);

	PERF_START(2);
	arm_mat_linsolve_right_f64(&PHTMatDouble, &SMatDouble, &KDouble, KDoubleData);
	PERF_END(PERF_LINSOLVE_MAG, 2);

	copyMatrixFloat(&KDouble, &K);

	arm_matrix_instance_f32 deltaX, innov;
	float32_t innovData[3], deltaXData[21];

	arm_mat_init_f32(&deltaX, 21, 1, deltaXData);
	arm_mat_init_f32(&innov, 3, 1, innovData);

	arm_sub_f32(mag_meas->pData, magB.pData, innov.pData, 3);
	arm_mat_mult_f32(&K, &innov, &deltaX);

	arm_matrix_instance_f32 deltaQ, quatXi;
	float32_t quatXiData[4*3], deltaQData[4];

	arm_mat_init_f32(&deltaQ, 4, 1, deltaQData);
	arm_quaternion_calculate_Xi(&q, &quatXi, quatXiData);
	arm_mat_mult_f32(&quatXi, &(arm_matrix_instance_f32){4, 1, deltaX.pData}, &deltaQ);
	arm_mat_scale_f32(&deltaQ, 0.5f, &deltaQ);

	arm_mat_init_f32(x_plus, 22, 1, x_plus_buff);

	arm_add_f32(q.pData, deltaQ.pData, x_plus->pData, 4);
	arm_quaternion_normalize_f32(x_plus->pData, x_plus->pData, 1);
	arm_add_f32(&x_minus->pData[4], &deltaX.pData[3], &x_plus->pData[4], 18);

	// PosteriorM = [ SigmaMinus * (I - KH)^T ; V * K^T ]	
	float32_t KH[21*21], I_KH[21*21], I_KHT[21*21], Sigma_IKHT[21*21];
	
	arm_mat_mult_f32(&K, &H, &(arm_matrix_instance_f32){21, 21, KH});
	arm_mat_sub_f32(&eye21, &(arm_matrix_instance_f32){21, 21, KH}, &(arm_matrix_instance_f32){21, 21, I_KH});
	arm_mat_trans_f32(&(arm_matrix_instance_f32){21, 21, I_KH}, &(arm_matrix_instance_f32){21, 21, I_KHT});
	arm_mat_mult_f32(P_minus, &(arm_matrix_instance_f32){21, 21, I_KHT}, &(arm_matrix_instance_f32){21, 21, Sigma_IKHT});

	// V for mag (Rq is 3x3)
	float32_t V_data[9] = {0};
	for(int i=0; i<3; i++) V_data[i*3 + i] = sqrtf(Rq->pData[i*3 + i]);
	arm_matrix_instance_f32 V_mat;
	arm_mat_init_f32(&V_mat, 3, 3, V_data);

	float32_t KT_data[63], VKT_data[63];
	arm_mat_trans_f32(&K, &(arm_matrix_instance_f32){3, 21, KT_data});
	arm_mat_mult_f32(&V_mat, &(arm_matrix_instance_f32){3, 21, KT_data}, &(arm_matrix_instance_f32){3, 21, VKT_data});

	float32_t PosteriorM_data[24*21] = {0};
	arm_matrix_instance_f32 PosteriorM;
	arm_mat_init_f32(&PosteriorM, 24, 21, PosteriorM_data);
	
	for(int r=0; r<21; r++) {
		for(int c=0; c<21; c++) {
			PosteriorM_data[r*21 + c] = Sigma_IKHT[r*21 + c];
		}
	}
	for(int r=0; r<3; r++) {
		for(int c=0; c<21; c++) {
			PosteriorM_data[(r+21)*21 + c] = VKT_data[r*21 + c];
		}
	}
	
	float32_t R_posterior_data[24*21], Q_posterior_data[24*24], tau_posterior[21];
	arm_matrix_instance_f32 R_posterior, Q_posterior;
	arm_mat_init_f32(&R_posterior, 24, 21, R_posterior_data);
	arm_mat_init_f32(&Q_posterior, 24, 24, Q_posterior_data);
	
	arm_mat_qr_f32(&PosteriorM, 0.0f, &R_posterior, &Q_posterior, tau_posterior, KH, I_KHT); // reuse buffers

	arm_mat_init_f32(P_plus, 21, 21, P_plus_buff);
	for(int r=0; r<21; r++) {
		for(int c=0; c<21; c++) {
			P_plus_buff[r*21 + c] = R_posterior_data[r*21 + c];
		}
	}

	PERF_END(PERF_MAG, 1);
}


// we call this if a barometer measurement comes in
void update_baro(const arm_matrix_instance_f32* xMinus, const arm_matrix_instance_f32* PMinus, const float32_t pressMeas,
				 const float32_t Rb, arm_matrix_instance_f32* xPlus, arm_matrix_instance_f32* Pplus,
				 float32_t xPlusData[22*1], float32_t pPlusData[21*21] PERF_ARG) {

	// Hb = Hb_func(x_minus);
    // K = P_minus*Hb'/(Hb*P_minus*Hb' + Rb);

	PERF_START(1);

	arm_matrix_instance_f32 Hb, K;
	float32_t HbTData[21*1], KData[21*1], temp1[1*21], temp2, temp3[21*1];
	float32_t HbData[1*21] = {0};

	arm_mat_init_f32(&K, 21, 1, KData);

	// Hb
	arm_mat_init_f32(&Hb, 1, 21, HbData);

	PERF_START(2);
	HbData[5] = filter_dP_dH(xMinus->pData[6]);
	PERF_END(PERF_FILTER_DP_DH, 2);

	// HbT
	arm_mat_trans_f32(&Hb, &(arm_matrix_instance_f32){21, 1, HbTData});

	// Hb*PMinus
	arm_mat_mult_f32(&Hb, PMinus, &(arm_matrix_instance_f32){1, 21, temp1});

	// Hb*PMinus*Hb'
	arm_mat_mult_f32(&(arm_matrix_instance_f32){1, 21, temp1},
					 &(arm_matrix_instance_f32){21, 1, HbTData},
					 &(arm_matrix_instance_f32){1, 1, &temp2});

	//  Hb*PMinus*Hb' + Rb
	temp2 += Rb;

	// PMinus*Hb'
	arm_mat_mult_f32(PMinus,
					 &(arm_matrix_instance_f32){21, 1, HbTData},
					 &(arm_matrix_instance_f32){21, 1, temp3});

	arm_mat_init_f32(&K, 21, 1, KData);
	arm_mat_scale_f32(&(arm_matrix_instance_f32){21, 1, temp3}, 1 / temp2, &K);
	float32_t hbFunc = filter_P(xMinus->pData[6]);

	arm_matrix_instance_f32 deltaX;
	float32_t deltaXData[21];

	arm_mat_init_f32(&deltaX, K.numRows, K.numCols, deltaXData);
	arm_mat_scale_f32(&K, (pressMeas - hbFunc), &deltaX);

	//  p_plus = x_minus(5:7) + Delta_x(4:6);
	//  v_plus = x_minus(8:10) + Delta_x(7:9);
	//  bg_plus = x_minus(11:13);
	//  ba_plus = x_minus(14:16) + Delta_x(13:15);
	//  kg_plus = x_minus(17:19) + Delta_x(16:18);
	//  ka_plus = x_minus(20:22) + Delta_x(19:21);
	//  x_plus = [q_plus; p_plus; v_plus; bg_plus; ba_plus; kg_plus; ka_plus];

	arm_mat_init_f32(xPlus, 22, 1, xPlusData);

	memcpy(&xPlus->pData[0], &xMinus->pData[0], 4 * sizeof(float32_t));

	arm_add_f32(&xMinus->pData[4],  &deltaX.pData[3],  &xPlus->pData[4],  3);
	arm_add_f32(&xMinus->pData[7],  &deltaX.pData[6],  &xPlus->pData[7],  3);
	arm_add_f32(&xMinus->pData[10], &deltaX.pData[9],  &xPlus->pData[10], 3);
	arm_add_f32(&xMinus->pData[13], &deltaX.pData[12], &xPlus->pData[13], 3);
	arm_add_f32(&xMinus->pData[16], &deltaX.pData[15], &xPlus->pData[16], 3);
	arm_add_f32(&xMinus->pData[19], &deltaX.pData[18], &xPlus->pData[19], 3);

//	// Quaternion Update
//	memcpy(&xPlusData[0], &xMinus->pData[0], 4*sizeof(float32_t));
//
//	// Position Update
//	arm_add_f32(&xMinus->pData[4], &deltaX.pData[3], &xPlusData[4], 3);
//
//	// Velocity Update
//	arm_add_f32(&xMinus->pData[7], &deltaX.pData[6], &xPlusData[7], 3);
//
//	// Gyro Bias Updated
//	arm_add_f32(&xMinus->pData[10], &deltaX.pData[9], 3);
//	//memcpy(&xPlusData[10], &xMinus->pData[10], 3 * sizeof(float32_t));
//
//	// Bias Accel Update
//	arm_add_f32(&xMinus->pData[13], &deltaX.pData[12], &xPlusData[13], 3);
//
//	// Scale Factor Gyro Update
//	//memcpy(&xPlusData[16], &xMinus->pData[16], 3 * sizeof(float32_t));
//	arm_add_f32(&xMinus->pData[15], &deltaX.pData[9], 3);
//
//	// Scale Factor Accel Update
//	arm_add_f32(&xMinus->pData[19], &deltaX.pData[18], &xPlusData[19], 3);

    // P_plus = (eye(21,21) - K*Hb)*P_minus* (eye(21,21)-K * Hb)' + K*Rb*K';

	float32_t temp3Buff[21*21], temp4Buff[21*21], temp5Buff[21*21],
			  temp6Buff[21*21], KT[1*21], temp7Buff[21*21], temp8Buff[21*21],
			  KRb[21*1];

	// K*Hb
	PERF_START(3);
	arm_mat_mult_f32(&K,
					 &Hb,
					 &(arm_matrix_instance_f32){21, 21, temp3Buff});

	// eye(21,21)-K*Hb
	arm_mat_sub_f32(&eye21,
					&(arm_matrix_instance_f32){21, 21, temp3Buff},
					&(arm_matrix_instance_f32){21, 21, temp4Buff});

	// (eye(21,21)-K*Hb)'
	arm_mat_trans_f32(&(arm_matrix_instance_f32){21, 21, temp4Buff},
					  &(arm_matrix_instance_f32){21, 21, temp6Buff});

	// (eye(21,21) - K*Hb)*P_minus
	arm_mat_mult_f32(&(arm_matrix_instance_f32){21, 21, temp4Buff},
					 PMinus,
					 &(arm_matrix_instance_f32){21, 21, temp5Buff});

	arm_mat_mult_f32(&(arm_matrix_instance_f32){21, 21, temp5Buff},
					 &(arm_matrix_instance_f32){21, 21, temp6Buff},
					 &(arm_matrix_instance_f32){21, 21, temp7Buff});

	arm_mat_scale_f32(&K, Rb, &(arm_matrix_instance_f32){21, 1, KRb});

	arm_mat_trans_f32(&K, &(arm_matrix_instance_f32){1, 21, KT});

	arm_mat_mult_f32(&(arm_matrix_instance_f32){21, 1, KRb},
					 &(arm_matrix_instance_f32){1, 21, KT},
					 &(arm_matrix_instance_f32){21, 21, temp8Buff});

	arm_mat_add_f32(&(arm_matrix_instance_f32){21, 21, temp7Buff},
			        &(arm_matrix_instance_f32){21, 21, temp8Buff},
					&(arm_matrix_instance_f32){21, 21, pPlusData});

	arm_mat_init_f32(Pplus, 21, 21, pPlusData);
	PERF_END(PERF_KALMAN_COVARIANCE_UPDATE, 3);

	PERF_END(PERF_BARO, 1);
}

// same as update_baro excepts requires less cycles by using sparse matrices

// same as update_baro excepts requires less cycles by using sparse matrices
void update_baro_new(const arm_matrix_instance_f32* xMinus, const arm_matrix_instance_f32* PMinus, const float32_t pressMeas,
				 	 const float32_t Rb, arm_matrix_instance_f32* xPlus, arm_matrix_instance_f32* Pplus,
					 float32_t xPlusData[22*1], float32_t pPlusData[21*21] PERF_ARG) {

	PERF_START(1);

	float32_t KData[21];
	float32_t temp1[21], temp2, temp3[21];

	PERF_START(2);
	float32_t h5 = filter_dP_dH(xMinus->pData[6]);
	PERF_END(PERF_FILTER_DP_DH, 2);

	for (uint8_t col = 0; col < 21; col++) {
		temp1[col] = h5 * PMinus->pData[5 * 21 + col];
	}

	temp2 = temp1[5] * h5 + Rb;

	float32_t invS = 1.0f / temp2;
	for (uint8_t row = 0; row < 21; row++) {
		KData[row] = temp1[row] * invS;
	}

	float32_t hbFunc = filter_P(xMinus->pData[6]);
	float32_t innovation = pressMeas - hbFunc;

	float32_t deltaXData[21];
	for (uint8_t i = 0; i < 21; i++) {
		deltaXData[i] = KData[i] * innovation;
	}

	arm_mat_init_f32(xPlus, 22, 1, xPlusData);

	for (uint8_t i = 0; i < 4; i++) {
		xPlus->pData[i] = xMinus->pData[i];
	}

	arm_add_f32(&xMinus->pData[4],  &deltaXData[3],  &xPlus->pData[4],  3);
	arm_add_f32(&xMinus->pData[7],  &deltaXData[6],  &xPlus->pData[7],  3);
	arm_add_f32(&xMinus->pData[10], &deltaXData[9],  &xPlus->pData[10], 3);
	arm_add_f32(&xMinus->pData[13], &deltaXData[12], &xPlus->pData[13], 3);
	arm_add_f32(&xMinus->pData[16], &deltaXData[15], &xPlus->pData[16], 3);
	arm_add_f32(&xMinus->pData[19], &deltaXData[18], &xPlus->pData[19], 3);

	// PosteriorM = [ SigmaMinus * (I - K*Hb)' ; V * K' ]
	// Hb is 1x21, with only element 5 non-zero (h5).
	// K is 21x1.
	// So K*Hb is 21x21, column 5 is K * h5
	float32_t temp4Buff[21*21];
	memcpy(temp4Buff, eye21.pData, sizeof(temp4Buff));
	for (uint8_t row = 0; row < 21; row++) {
		temp4Buff[row * 21 + 5] -= KData[row] * h5;
	}

	float32_t temp5Buff[21*21];
	arm_mat_trans_f32(&(arm_matrix_instance_f32){21, 21, temp4Buff}, &(arm_matrix_instance_f32){21, 21, temp5Buff});

	float32_t Sigma_IKHT[21*21];
	arm_mat_mult_f32(PMinus, &(arm_matrix_instance_f32){21, 21, temp5Buff}, &(arm_matrix_instance_f32){21, 21, Sigma_IKHT});

	float32_t V = sqrtf(Rb);
	float32_t VKT_data[21]; // V * K^T is 1x21
	for(int i=0; i<21; i++) VKT_data[i] = V * KData[i];

	float32_t PosteriorM_data[22*21] = {0}; // 22 x 21
	arm_matrix_instance_f32 PosteriorM;
	arm_mat_init_f32(&PosteriorM, 22, 21, PosteriorM_data);
	
	for(int r=0; r<21; r++) {
		for(int c=0; c<21; c++) {
			PosteriorM_data[r*21 + c] = Sigma_IKHT[r*21 + c];
		}
	}
	for(int c=0; c<21; c++) {
		PosteriorM_data[21*21 + c] = VKT_data[c];
	}

	float32_t R_posterior_data[22*21], Q_posterior_data[22*22], tau_posterior[21];
	arm_matrix_instance_f32 R_posterior, Q_posterior;
	arm_mat_init_f32(&R_posterior, 22, 21, R_posterior_data);
	arm_mat_init_f32(&Q_posterior, 22, 22, Q_posterior_data);
	
	arm_mat_qr_f32(&PosteriorM, 0.0f, &R_posterior, &Q_posterior, tau_posterior, temp4Buff, temp5Buff);

	arm_mat_init_f32(Pplus, 21, 21, pPlusData);
	for(int r=0; r<21; r++) {
		for(int c=0; c<21; c++) {
			pPlusData[r*21 + c] = R_posterior_data[r*21 + c];
		}
	}

	PERF_END(PERF_BARO, 1);
}




