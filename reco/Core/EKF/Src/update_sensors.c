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

	(void)H; // H_gps is [0_{3x3}, I_{3x3}, 0_{3x15}]; sparsity is exploited directly
	PERF_START(1);
	
	// 1. Exploit Sparsity of H_gps (3x21, Identity at cols 3..5) & Upper-Triangular P_minus (SigmaMinus)
	//    SigmaMinus * H^T is simply columns 3..5 of SigmaMinus, which are non-zero ONLY in rows 0..5!
	float32_t trace_sub = 0.0f;
	for (int r = 0; r < 6; r++) {
		for (int c = 3; c < 6; c++) {
			float32_t s_rc = P_minus->pData[r * 21 + c];
			trace_sub += s_rc * s_rc;
		}
	}
	float32_t beta = (trace_sub > 1000.0f) ? 0.25f : 0.0f;
	float32_t sqrt_beta = sqrtf(1.0f + beta);

	// 2. Innovation QR Sequence: M_innov = [ sqrt(1+beta) * SigmaMinus * H^T ; V_gps ] (24 x 3)
	//    QR decomposition of M_innov yields the 3x3 upper-triangular Cholesky factor PiHat of S = PiHat^T * PiHat.
	CREATE_MAT_F32(M_innov, 24, 3);
	for (int r = 0; r < 6; r++) {
		for (int c = 0; c < 3; c++) {
			M_innovData[r * 3 + c] = sqrt_beta * P_minus->pData[r * 21 + (3 + c)];
		}
	}
	float32_t v_gps[3];
	for (int i = 0; i < 3; i++) {
		v_gps[i] = sqrtf(R->pData[i * 3 + i]);
		M_innovData[(21 + i) * 3 + i] = v_gps[i];
	}

	CREATE_MAT_F32(R_innov, 24, 3);
	CREATE_MAT_F32(Q_innov, 24, 24);
	float32_t tau_innov[3], qr_tmpA[24], qr_tmpB[24];
	arm_mat_qr_f32(&M_innov, 0.0f, &R_innov, &Q_innov, tau_innov, qr_tmpA, qr_tmpB);

	// Extract 3x3 upper-triangular PiHat and lower-triangular PiHat^T in f64
	CREATE_MAT_F64(PiHatDouble, 3, 3);
	CREATE_MAT_F64(PiHatTDouble, 3, 3);
	for (int r = 0; r < 3; r++) {
		for (int c = r; c < 3; c++) {
			float64_t val = (float64_t)R_innovData[r * 3 + c];
			PiHatDoubleData[r * 3 + c] = val;
			PiHatTDoubleData[c * 3 + r] = val;
		}
	}

	// 3. Cross-covariance transposed: B = (P^- * H^T)^T = (SigmaMinus * H^T)^T * SigmaMinus (3 x 21)
	CREATE_MAT_F64(BDouble, 3, 21);
	for (int i = 0; i < 3; i++) {
		for (int c = 0; c < 21; c++) {
			float64_t sum = 0.0;
			int r_max = (c < 5) ? c : 5;
			for (int r = 0; r <= r_max; r++) {
				sum += (float64_t)P_minus->pData[r * 21 + (3 + i)] * (float64_t)P_minus->pData[r * 21 + c];
			}
			BDoubleData[i * 21 + c] = sum;
		}
	}

	// 4. Two-Step Triangular Cholesky Solve for Kalman Gain (No general linsolve needed):
	//    PiHat^T * (PiHat * K^T) = B
	//    - Inner call (Lower Triangular): solve (PiHat^T) * Y = B
	//    - Outer call (Upper Triangular): solve PiHat * K^T = Y
	CREATE_MAT_F64(YDouble, 3, 21);
	CREATE_MAT_F64(KTDouble, 3, 21);

	PERF_START(2);
	arm_mat_solve_lower_triangular_f64(&PiHatTDouble, &BDouble, &YDouble);
	arm_mat_solve_upper_triangular_f64(&PiHatDouble, &YDouble, &KTDouble);
	PERF_END(PERF_LINSOLVE_GPS, 2);

	CREATE_MAT_F32(KT_mat, 3, 21);
	CREATE_MAT_F32(K_mat, 21, 3);
	copyMatrixFloat(&KTDouble, &KT_mat);
	arm_mat_trans_f32(&KT_mat, &K_mat);

	// 5. Measurement Residual and State Update
	float32_t residual_data[3];
	arm_sub_f32(lla_meas->pData, &x_minus->pData[4], residual_data, 3);

	arm_matrix_instance_f32 residual_vec;
	arm_mat_init_f32(&residual_vec, 3, 1, residual_data);
	CREATE_MAT_F32(Delta_x_vec, 21, 1);
	arm_mat_mult_f32(&K_mat, &residual_vec, &Delta_x_vec);

	arm_mat_init_f32(x_plus, 22, 1, xPlusData);
	memcpy(&x_plus->pData[0], &x_minus->pData[0], 4 * sizeof(float32_t));
	arm_add_f32(&x_minus->pData[4], &Delta_x_vecData[3], &x_plus->pData[4], 18);

	// 6. Posterior Square-Root Covariance via Sparse Rank-3 Update & Stacked Block QR:
	//    PosteriorM = [ SigmaMinus * (I - K*H)^T ; V_gps * K^T ] (24 x 21)
	//    Since SigmaMinus * H^T is non-zero only in rows 0..5, only rows 0..5 of SigmaMinus change!
	CREATE_MAT_F32(PosteriorM, 24, 21);
	memcpy(PosteriorMData, P_minus->pData, 21 * 21 * sizeof(float32_t));

	for (int r = 0; r < 6; r++) {
		float32_t s_r3 = P_minus->pData[r * 21 + 3];
		float32_t s_r4 = P_minus->pData[r * 21 + 4];
		float32_t s_r5 = P_minus->pData[r * 21 + 5];
		for (int c = 0; c < 21; c++) {
			PosteriorMData[r * 21 + c] -= (s_r3 * KT_matData[0 * 21 + c] +
			                               s_r4 * KT_matData[1 * 21 + c] +
			                               s_r5 * KT_matData[2 * 21 + c]);
		}
	}
	for (int r = 0; r < 3; r++) {
		float32_t vr = v_gps[r];
		for (int c = 0; c < 21; c++) {
			PosteriorMData[(21 + r) * 21 + c] = vr * KT_matData[r * 21 + c];
		}
	}
	
	CREATE_MAT_F32(R_posterior, 24, 21);
	CREATE_MAT_F32(Q_posterior, 24, 24);
	float32_t tau_posterior[21];
	
	arm_mat_qr_f32(&PosteriorM, 0.0f, &R_posterior, &Q_posterior, tau_posterior, qr_tmpA, qr_tmpB);

	arm_mat_init_f32(P_plus, 21, 21, P_plus_data);
	for (int r = 0; r < 21; r++) {
		for (int c = 0; c < r; c++) {
			P_plus_data[r * 21 + c] = 0.0f;
		}
		for (int c = r; c < 21; c++) {
			P_plus_data[r * 21 + c] = R_posteriorData[r * 21 + c];
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
	arm_matrix_instance_f32 q, DCMn2b, DCMb2n, magB, magBSkew;

	arm_mat_init_f32(&DCMb2n, 3, 3, b2n);

	getStateQuaternion(x_minus, &q, qBuff);
	quaternion2DCM(&q, &DCMn2b, n2b PERF_PASS);
	arm_mat_trans_f32(&DCMn2b, &DCMb2n);

	arm_mat_init_f32(&magB, 3, 1, magBBuff);
	arm_mat_mult_f32(&DCMb2n, magI, &magB);
	arm_mat_skew_f32(&magB, &magBSkew, magBSkewBuff);

	// 1. Exploit Upper-Triangular & Block Sparsity of H_mag = [magBSkew, 0_{3x18}]
	//    Since P_minus (SigmaMinus) is upper triangular, only its top-left 3x3 block multiplies magBSkew^T.
	//    Thus SigmaMinus * H^T has non-zero entries ONLY in rows 0..2!
	float32_t Sigma_HT_top3[3 * 3] = {0};
	for (int r = 0; r < 3; r++) {
		for (int c = 0; c < 3; c++) {
			float32_t sum = 0.0f;
			for (int k = r; k < 3; k++) {
				sum += P_minus->pData[r * 21 + k] * magBSkewBuff[c * 3 + k];
			}
			Sigma_HT_top3[r * 3 + c] = sum;
		}
	}

	// 2. Innovation QR Sequence: M_innov = [ SigmaMinus * H^T ; V_mag ] (24 x 3)
	CREATE_MAT_F32(M_innov, 24, 3);
	for (int r = 0; r < 3; r++) {
		for (int c = 0; c < 3; c++) {
			M_innovData[r * 3 + c] = Sigma_HT_top3[r * 3 + c];
		}
	}
	float32_t v_mag[3];
	for (int i = 0; i < 3; i++) {
		// RqInitData is initialized directly with standard deviations (0.005f) in compute_initial_consts.c
		v_mag[i] = Rq->pData[i * 3 + i];
		M_innovData[(21 + i) * 3 + i] = v_mag[i];
	}

	CREATE_MAT_F32(R_innov, 24, 3);
	CREATE_MAT_F32(Q_innov, 24, 24);
	float32_t tau_innov[3], qr_tmpA[24], qr_tmpB[24];
	arm_mat_qr_f32(&M_innov, 0.0f, &R_innov, &Q_innov, tau_innov, qr_tmpA, qr_tmpB);

	CREATE_MAT_F64(PiHatDouble, 3, 3);
	CREATE_MAT_F64(PiHatTDouble, 3, 3);
	for (int r = 0; r < 3; r++) {
		for (int c = r; c < 3; c++) {
			float64_t val = (float64_t)R_innovData[r * 3 + c];
			PiHatDoubleData[r * 3 + c] = val;
			PiHatTDoubleData[c * 3 + r] = val;
		}
	}

	// 3. Cross-covariance transposed: B = (SigmaMinus * H^T)^T * SigmaMinus (3 x 21)
	CREATE_MAT_F64(BDouble, 3, 21);
	for (int i = 0; i < 3; i++) {
		for (int c = 0; c < 21; c++) {
			float64_t sum = 0.0;
			int r_max = (c < 2) ? c : 2;
			for (int r = 0; r <= r_max; r++) {
				sum += (float64_t)Sigma_HT_top3[r * 3 + i] * (float64_t)P_minus->pData[r * 21 + c];
			}
			BDoubleData[i * 21 + c] = sum;
		}
	}

	// 4. Two-Step Triangular Cholesky Solve for Kalman Gain:
	//    - Inner call (Lower Triangular): solve (PiHat^T) * Y = B
	//    - Outer call (Upper Triangular): solve PiHat * K^T = Y
	CREATE_MAT_F64(YDouble, 3, 21);
	CREATE_MAT_F64(KTDouble, 3, 21);

	PERF_START(2);
	arm_mat_solve_lower_triangular_f64(&PiHatTDouble, &BDouble, &YDouble);
	arm_mat_solve_upper_triangular_f64(&PiHatDouble, &YDouble, &KTDouble);
	PERF_END(PERF_LINSOLVE_MAG, 2);

	CREATE_MAT_F32(KT_mat, 3, 21);
	CREATE_MAT_F32(K, 21, 3);
	copyMatrixFloat(&KTDouble, &KT_mat);
	arm_mat_trans_f32(&KT_mat, &K);

	CREATE_MAT_F32(deltaX, 21, 1);
	CREATE_MAT_F32(innov, 3, 1);

	arm_sub_f32(mag_meas->pData, magB.pData, innov.pData, 3);
	arm_mat_mult_f32(&K, &innov, &deltaX);

	arm_matrix_instance_f32 deltaQ, quatXi;
	float32_t quatXiData[4*3], deltaQData[4];

	arm_mat_init_f32(&deltaQ, 4, 1, deltaQData);
	arm_quaternion_calculate_Xi(&q, &quatXi, quatXiData);
	arm_mat_mult_f32(&quatXi, &(arm_matrix_instance_f32){3, 1, deltaX.pData}, &deltaQ);
	arm_mat_scale_f32(&deltaQ, 0.5f, &deltaQ);

	arm_mat_init_f32(x_plus, 22, 1, x_plus_buff);

	arm_add_f32(q.pData, deltaQ.pData, x_plus->pData, 4);
	arm_quaternion_normalize_f32(x_plus->pData, x_plus->pData, 1);
	arm_add_f32(&x_minus->pData[4], &deltaX.pData[3], &x_plus->pData[4], 18);

	// 5. PosteriorM = [ SigmaMinus * (I - KH)^T ; V_mag * K^T ] (24 x 21)
	//    Since SigmaMinus * H^T is non-zero only in rows 0..2, only rows 0..2 of SigmaMinus change!
	CREATE_MAT_F32(PosteriorM, 24, 21);
	memcpy(PosteriorMData, P_minus->pData, 21 * 21 * sizeof(float32_t));

	for (int r = 0; r < 3; r++) {
		float32_t sh0 = Sigma_HT_top3[r * 3 + 0];
		float32_t sh1 = Sigma_HT_top3[r * 3 + 1];
		float32_t sh2 = Sigma_HT_top3[r * 3 + 2];
		for (int c = 0; c < 21; c++) {
			PosteriorMData[r * 21 + c] -= (sh0 * KT_matData[0 * 21 + c] +
			                               sh1 * KT_matData[1 * 21 + c] +
			                               sh2 * KT_matData[2 * 21 + c]);
		}
	}
	for (int r = 0; r < 3; r++) {
		float32_t vr = v_mag[r];
		for (int c = 0; c < 21; c++) {
			PosteriorMData[(21 + r) * 21 + c] = vr * KT_matData[r * 21 + c];
		}
	}
	
	CREATE_MAT_F32(R_posterior, 24, 21);
	CREATE_MAT_F32(Q_posterior, 24, 24);
	float32_t tau_posterior[21];
	
	arm_mat_qr_f32(&PosteriorM, 0.0f, &R_posterior, &Q_posterior, tau_posterior, qr_tmpA, qr_tmpB);

	arm_mat_init_f32(P_plus, 21, 21, P_plus_buff);
	for (int r = 0; r < 21; r++) {
		for (int c = 0; c < r; c++) {
			P_plus_buff[r * 21 + c] = 0.0f;
		}
		for (int c = r; c < 21; c++) {
			P_plus_buff[r * 21 + c] = R_posteriorData[r * 21 + c];
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

// same as update_baro except requires less cycles by exploiting upper-triangular & 1D sparsity
void update_baro_new(const arm_matrix_instance_f32* xMinus, const arm_matrix_instance_f32* PMinus, const float32_t pressMeas,
				 	 const float32_t Rb, arm_matrix_instance_f32* xPlus, arm_matrix_instance_f32* Pplus,
					 float32_t xPlusData[22*1], float32_t pPlusData[21*21] PERF_ARG) {

	PERF_START(1);

	float32_t KData[21];
	float32_t PHT[21];

	PERF_START(2);
	float32_t h5 = filter_dP_dH(xMinus->pData[6]);
	PERF_END(PERF_FILTER_DP_DH, 2);

	// 1. Exploit Upper-Triangular PMinus (SigmaMinus) & 1D Sparsity of Hb (only index 5 is h5):
	//    Column 5 of SigmaMinus is non-zero only in rows 0..5.
	//    Cross-covariance P^- * Hb^T = h5 * SigmaMinus(0..5, :)^T * SigmaMinus(0..5, 5)
	for (uint8_t col = 0; col < 21; col++) {
		float32_t sum = 0.0f;
		uint8_t r_max = (col < 5) ? col : 5;
		for (uint8_t r = 0; r <= r_max; r++) {
			sum += PMinus->pData[r * 21 + 5] * PMinus->pData[r * 21 + col];
		}
		PHT[col] = h5 * sum;
	}

	// Scalar innovation covariance S = Hb * P^- * Hb^T + Rb = h5 * PHT[5] + Rb
	float32_t S = PHT[5] * h5 + Rb;
	float32_t invS = 1.0f / S;
	for (uint8_t row = 0; row < 21; row++) {
		KData[row] = PHT[row] * invS;
	}

	float32_t hbFunc = filter_P(xMinus->pData[6]);
	float32_t innovation = pressMeas - hbFunc;

	float32_t deltaXData[21];
	for (uint8_t i = 0; i < 21; i++) {
		deltaXData[i] = KData[i] * innovation;
	}

	arm_mat_init_f32(xPlus, 22, 1, xPlusData);
	memcpy(xPlus->pData, xMinus->pData, 4 * sizeof(float32_t));
	arm_add_f32(&xMinus->pData[4], &deltaXData[3], &xPlus->pData[4], 18);

	// 2. Posterior Square-Root Covariance via Sparse Rank-1 Update & Stacked Block QR:
	//    PosteriorM = [ SigmaMinus * (I - K*Hb)^T ; sqrt(Rb) * K^T ] (22 x 21)
	//    Since SigmaMinus * Hb^T = h5 * SigmaMinus(:, 5) is non-zero ONLY in rows 0..5,
	//    only rows 0..5 of SigmaMinus are modified by the rank-1 update!
	CREATE_MAT_F32(PosteriorM, 22, 21);
	memcpy(PosteriorMData, PMinus->pData, 21 * 21 * sizeof(float32_t));

	for (uint8_t r = 0; r <= 5; r++) {
		float32_t sh_r = h5 * PMinus->pData[r * 21 + 5];
		for (uint8_t c = 0; c < 21; c++) {
			PosteriorMData[r * 21 + c] -= sh_r * KData[c];
		}
	}

	float32_t V = sqrtf(Rb);
	for (uint8_t c = 0; c < 21; c++) {
		PosteriorMData[21 * 21 + c] = V * KData[c];
	}

	CREATE_MAT_F32(R_posterior, 22, 21);
	CREATE_MAT_F32(Q_posterior, 22, 22);
	float32_t tau_posterior[21], qr_tmpA[22], qr_tmpB[22];
	
	arm_mat_qr_f32(&PosteriorM, 0.0f, &R_posterior, &Q_posterior, tau_posterior, qr_tmpA, qr_tmpB);

	arm_mat_init_f32(Pplus, 21, 21, pPlusData);
	for (uint8_t r = 0; r < 21; r++) {
		for (uint8_t c = 0; c < r; c++) {
			pPlusData[r * 21 + c] = 0.0f;
		}
		for (uint8_t c = r; c < 21; c++) {
			pPlusData[r * 21 + c] = R_posteriorData[r * 21 + c];
		}
	}

	PERF_END(PERF_BARO, 1);
}




