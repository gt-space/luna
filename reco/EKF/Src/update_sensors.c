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
void update_GPS(const arm_matrix_instance_f32* x_minus, const arm_matrix_instance_f32* P_minus, const arm_matrix_instance_f32* H,
				const arm_matrix_instance_f32* R, const arm_matrix_instance_f32* lla_meas, arm_matrix_instance_f32* x_plus,
				arm_matrix_instance_f32* P_plus, float32_t xPlusData[22*1], float32_t P_plus_data[21*21]) {
	// STEP 2: KALMAN GAIN - Adaptive underweighting based on position uncertainty
	float32_t trace_sub = P_minus->pData[3 * 21 + 3] + P_minus->pData[4 * 21 + 4] + P_minus->pData[5 * 21 + 5];
	float32_t beta = (trace_sub > 1000.0f) ? 0.25f : 0.0f;

	// Compute W = (1+beta)*H*P_minus*H' + R
	arm_matrix_instance_f32 HP_mat;
	float32_t HP_data[63];
	arm_mat_init_f32(&HP_mat, 3, 21, HP_data);
	arm_mat_mult_f32(H, P_minus, &HP_mat);

	arm_matrix_instance_f32 HT_mat;
	float32_t HT_data[63], W_temp_data[9];
	arm_mat_init_f32(&HT_mat, 21, 3, HT_data);
	arm_mat_trans_f32(H, &HT_mat);

	arm_matrix_instance_f32 W_temp_mat;
	arm_mat_init_f32(&W_temp_mat, 3, 3, W_temp_data);
	arm_mat_mult_f32(&HP_mat, &HT_mat, &W_temp_mat);

	float32_t W_data[9];
	for (int i = 0; i < 9; i++)
	{
		W_data[i] = (1.0f + beta) * W_temp_data[i] + R->pData[i];
	}

	// Compute PH = P_minus*H'
	arm_matrix_instance_f32 PH_mat;
	float32_t PH_data[63];
	arm_mat_init_f32(&PH_mat, 21, 3, PH_data);
	arm_mat_mult_f32(P_minus, &HT_mat, &PH_mat);

	// Compute K = P_minus*H'/ W;  using library function
	arm_matrix_instance_f32 W_mat;
	arm_mat_init_f32(&W_mat, 3, 3, W_data);

	arm_matrix_instance_f32 K_mat;
	float32_t K_data[63];
	arm_mat_init_f32(&K_mat, 21, 3, K_data);

	float64_t KDoubleData[21*3], WMatDoubleData[3*3], PHMatDoubleData[21*3];
	arm_matrix_instance_f64 KDouble, WMatDouble, PHMatDouble;

	// Start converting terms to f64 for minimal numerical inaccuracies in linsolve()
	arm_mat_init_f64(&WMatDouble, 3, 3, WMatDoubleData);
	arm_mat_init_f64(&PHMatDouble, 21, 3, PHMatDoubleData);

	copyMatrixDouble(&W_mat, &WMatDouble);
	copyMatrixDouble(&PH_mat, &PHMatDouble);

	arm_mat_linsolve_right_f64(&WMatDouble, &PHMatDouble, &KDouble, KDoubleData);

	arm_mat_init_f32(&K_mat, 21, 3, K_data);
	copyMatrixFloat(&KDouble, &K_mat);

	// STEP 3: MEASUREMENT UPDATE - Compute measurement residual
	float32_t residual_data[3];
	arm_sub_f32(lla_meas->pData, &x_minus->pData[4], residual_data, 3);

	// Compute Delta_x = K * residual
	arm_matrix_instance_f32 residual_vec, Delta_x_vec;
	float32_t Delta_x_data[21];
	arm_mat_init_f32(&residual_vec, 3, 1, residual_data);
	arm_mat_init_f32(&Delta_x_vec, 21, 1, Delta_x_data);
	arm_mat_mult_f32(&K_mat, &residual_vec, &Delta_x_vec);

	arm_mat_init_f32(x_plus, 22, 1, xPlusData);

	// Update state components
	// Quaternion: unchanged
	for (int i = 0; i < 4; i++)
	{
		x_plus->pData[i] = x_minus->pData[i];
	}

	// Position: p_plus = x_minus(5:7) + Delta_x(4:6)
	for (int i = 0; i < 3; i++)
	{
		x_plus->pData[4 + i] = x_minus->pData[4 + i] + Delta_x_data[3 + i];
	}

	// Velocity: v_plus = x_minus(8:10) + Delta_x(7:9)
	for (int i = 0; i < 3; i++)
	{
		x_plus->pData[7 + i] = x_minus->pData[7 + i] + Delta_x_data[6 + i];
	}

	// printf("Velocity [%f, %f, %f] m/s (Update GPS)\n", Delta_x_data[6], Delta_x_data[7], Delta_x_data[8]);

	// Gyro bias: CHANGE
	for (int i = 0; i < 3; i++)
	{
		x_plus->pData[10 + i] = x_minus->pData[10 + i] + Delta_x_data[9 + i];
	}

	// Accel bias: ba_plus = x_minus(14:16) + Delta_x(13:15)
	for (int i = 0; i < 3; i++)
	{
		x_plus->pData[13 + i] = x_minus->pData[13 + i] + Delta_x_data[12 + i];
	}

	// Gyro scale factor: kg_plus = x_minus(17:19) + Delta_x(16:18) CHANGE
	for (int i = 0; i < 3; i++)
	{
		x_plus->pData[16 + i] = x_minus->pData[16 + i] + Delta_x_data[15 + i];
	}

	// Accel scale factor: ka_plus = x_minus(20:22) + Delta_x(19:21)
	for (int i = 0; i < 3; i++)
	{
		x_plus->pData[19 + i] = x_minus->pData[19 + i] + Delta_x_data[18 + i];
	}

	// Compute P_plus = (I - K*H)*P_minus*(I - K*H)' + K*R*K'
	// Compute KH = K*H
	arm_matrix_instance_f32 KH_mat;
	float32_t KH_data[441];
	arm_mat_init_f32(&KH_mat, 21, 21, KH_data);
	arm_mat_mult_f32(&K_mat, H, &KH_mat);


	// Compute I_KH = I - K*H
	arm_matrix_instance_f32 I_KH_mat;
	float32_t I_KH_data[441];
	arm_mat_init_f32(&I_KH_mat, 21, 21, I_KH_data);
	arm_mat_sub_f32(&eye21, &KH_mat, &I_KH_mat);

	// Compute I_KH_T (transpose of I_KH)
	arm_matrix_instance_f32 I_KH_T_mat;
	float32_t I_KH_T_data[441];
	arm_mat_init_f32(&I_KH_T_mat, 21, 21, I_KH_T_data);
	arm_mat_trans_f32(&I_KH_mat, &I_KH_T_mat);

	// Compute temp1 = I_KH * P_minus
	arm_matrix_instance_f32 temp1_mat;
	float32_t temp1_data[441];
	arm_mat_init_f32(&temp1_mat, 21, 21, temp1_data);
	arm_mat_mult_f32(&I_KH_mat, P_minus, &temp1_mat);

	// Compute temp2 = (I_KH * P_minus) * I_KH_T
	arm_matrix_instance_f32 temp2_mat;
	float32_t temp2_data[441];
	arm_mat_init_f32(&temp2_mat, 21, 21, temp2_data);
	arm_mat_mult_f32(&temp1_mat, &I_KH_T_mat, &temp2_mat);

	// Compute K*R
	arm_matrix_instance_f32 KR_mat;
	float32_t KR_data[63];
	arm_mat_init_f32(&KR_mat, 21, 3, KR_data);
	arm_mat_mult_f32(&K_mat, R, &KR_mat);

	// Compute K_T = K'
	arm_matrix_instance_f32 KT_mat;
	float32_t KT_data[63];
	arm_mat_init_f32(&KT_mat, 3, 21, KT_data);
	arm_mat_trans_f32(&K_mat, &KT_mat);

	// Compute KRK = KR * K_T
	arm_matrix_instance_f32 KRK_mat;
	float32_t KRK_data[441];
	arm_mat_init_f32(&KRK_mat, 21, 21, KRK_data);
	arm_mat_mult_f32(&KR_mat, &KT_mat, &KRK_mat);

	// Sum: P_plus = temp2 + KRK
	arm_mat_init_f32(P_plus, 21, 21, P_plus_data);
	arm_mat_add_f32(&temp2_mat, &KRK_mat, P_plus);
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
void update_mag(const arm_matrix_instance_f32* x_minus, const arm_matrix_instance_f32* P_minus, const arm_matrix_instance_f32* R,
				const arm_matrix_instance_f32* magI, const arm_matrix_instance_f32* mag_meas, arm_matrix_instance_f32* x_plus,
				arm_matrix_instance_f32* P_plus, float32_t x_plus_buff[22*1], float32_t P_plus_buff[21*21]) {

	/*
	q_minus = x_minus(0:4);
	DCM_n2b = q2DCM(q_minus)';
	magB = DCM_n2b * magI;
	H = [skew(magB) zeros(3, 18)]
	*/

	float32_t qBuff[4], n2b[3*3], b2n[3*3], magBBuff[3*1],
			  magBSkewBuff[3*3];

	arm_matrix_instance_f32 q, DCMn2b, DCMb2n, magB, magBSkew, H;

	arm_mat_init_f32(&DCMb2n, 3, 3, b2n);

	getStateQuaternion(x_minus, &q, qBuff);
	quaternion2DCM(&q, &DCMn2b, n2b);
	arm_mat_trans_f32(&DCMn2b, &DCMb2n);

	arm_mat_init_f32(&magB, 3, 1, magBBuff);
	arm_mat_mult_f32(&DCMb2n, magI, &magB);
	arm_mat_skew_f32(&magB, &magBSkew, magBSkewBuff);

	float32_t HBuff[3*21] = {0};
	arm_mat_init_f32(&H, 3, 21, HBuff);
	arm_mat_place_f32(&magBSkew, &H, 0, 0);

	// K = P_minus*H'/(H*P_minus*H' + R);
	// Can save one mult instruction here

	arm_matrix_instance_f32 K, A, B;

	float32_t KBuff[21*3], HT[21*3], PMinusHT[21*3],
			  HPMinus[3*21], HPMinusHT[3*3], HPMinusHTR[3*3];

	arm_mat_trans_f32(&H, &(arm_matrix_instance_f32){21, 3, HT});

	// B
	arm_mat_mult_f32(P_minus,
					 &(arm_matrix_instance_f32){21, 3, HT},
					 &(arm_matrix_instance_f32){21, 3, PMinusHT});

	// A
	arm_mat_mult_f32(&H,
					 P_minus,
					 &(arm_matrix_instance_f32){3, 21, HPMinus});

	arm_mat_mult_f32(&(arm_matrix_instance_f32){3, 21, HPMinus},
					 &(arm_matrix_instance_f32){21, 3, HT},
					 &(arm_matrix_instance_f32){3, 3, HPMinusHT});

	arm_mat_add_f32(&(arm_matrix_instance_f32){3, 3, HPMinusHT},
					R,
					&(arm_matrix_instance_f32){3, 3, HPMinusHTR});

	arm_mat_init_f32(&A, 3, 3, HPMinusHTR);
	arm_mat_init_f32(&B, 21, 3, PMinusHT);
	arm_mat_init_f32(&K, 21, 3, KBuff);

	arm_matrix_instance_f64 ADouble, BDouble, KDouble;
	float64_t ADoubleData[3*3], BDoubleData[21*3], KDoubleData[21*3];

	arm_mat_init_f64(&ADouble, 3, 3, ADoubleData);
	arm_mat_init_f64(&BDouble, 21, 3, BDoubleData);

	copyMatrixDouble(&A, &ADouble);
	copyMatrixDouble(&B, &BDouble);

	arm_mat_linsolve_right_f64(&ADouble,
							   &BDouble,
							   &KDouble,
							   KDoubleData);

	copyMatrixFloat(&KDouble, &K);

	// Trash magB and use it as a normal buffer (not done but can be to save init costs)
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

	// P_plus = (eye(21,21) - K*H)*P_minus* (eye(21,21)-K * H)' + K*R*K';

	float32_t KH[21*21], eye21KH[21*21], eye21KHPMinus[21*21], eye21KHT[21*21], part1Result[21*21],
			  KR[21*3], KT[3*21], part2Result[21*21];


	arm_mat_mult_f32(&K, &H, &(arm_matrix_instance_f32){21, 21, KH});

	arm_mat_sub_f32(&eye21,
					&(arm_matrix_instance_f32){21, 21, KH},
					&(arm_matrix_instance_f32){21, 21, eye21KH});

	arm_mat_mult_f32(&(arm_matrix_instance_f32){21, 21, eye21KH},
					 P_minus,
					 &(arm_matrix_instance_f32){21, 21, eye21KHPMinus});

	arm_mat_trans_f32(&(arm_matrix_instance_f32){21, 21, eye21KH},
					  &(arm_matrix_instance_f32){21, 21, eye21KHT});

	arm_mat_mult_f32(&(arm_matrix_instance_f32){21, 21, eye21KHPMinus},
					 &(arm_matrix_instance_f32){21, 21, eye21KHT},
					 &(arm_matrix_instance_f32){21, 21, part1Result});

	arm_mat_mult_f32(&K, R, &(arm_matrix_instance_f32){21, 3, KR});

	arm_mat_trans_f32(&K, &(arm_matrix_instance_f32){3, 21, KT});

	arm_mat_mult_f32(&(arm_matrix_instance_f32){21, 3, KR},
					 &(arm_matrix_instance_f32){3, 21, KT},
					 &(arm_matrix_instance_f32){21, 21, part2Result});

	arm_mat_init_f32(P_plus, 21, 21, P_plus_buff);

	arm_mat_add_f32(&(arm_matrix_instance_f32){21, 21, part1Result},
					&(arm_matrix_instance_f32){21, 21, part2Result},
					P_plus);
}

// we call this if a barometer measurement comes in
void update_baro(const arm_matrix_instance_f32* xMinus, const arm_matrix_instance_f32* PMinus, const float32_t pressMeas,
				 const float32_t Rb, arm_matrix_instance_f32* xPlus, arm_matrix_instance_f32* Pplus,
				 float32_t xPlusData[22*1], float32_t pPlusData[21*21]) {

	// Hb = Hb_func(x_minus);
    // K = P_minus*Hb'/(Hb*P_minus*Hb' + Rb);

	arm_matrix_instance_f32 Hb, K;
	float32_t HbTData[21*1], KData[21*1], temp1[1*21], temp2, temp3[21*1];
	float32_t HbData[1*21] = {0};

	arm_mat_init_f32(&K, 21, 1, KData);

	// Hb
	arm_mat_init_f32(&Hb, 1, 21, HbData);
	HbData[5] = filter_dP_dH(xMinus->pData[6]);

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

	// Quaternion Update
	memcpy(&xPlusData[0], &xMinus->pData[0], 4*sizeof(float32_t));

	// Position Update
	arm_add_f32(&xMinus->pData[4], &deltaX.pData[3], &xPlusData[4], 3);

	// Velocity Update
	arm_add_f32(&xMinus->pData[7], &deltaX.pData[6], &xPlusData[7], 3);

	// Gyro Bias Updated
	memcpy(&xPlusData[10], &xMinus->pData[10], 3 * sizeof(float32_t));

	// Bias Accel Update
	arm_add_f32(&xMinus->pData[13], &deltaX.pData[12], &xPlusData[13], 3);

	// Scale Factor Gyro Update
	memcpy(&xPlusData[16], &xMinus->pData[16], 3 * sizeof(float32_t));

	// Scale Factor Accel Update
	arm_add_f32(&xMinus->pData[19], &deltaX.pData[18], &xPlusData[19], 3);

    // P_plus = (eye(21,21) - K*Hb)*P_minus* (eye(21,21)-K * Hb)' + K*Rb*K';

	float32_t temp3Buff[21*21], temp4Buff[21*21], temp5Buff[21*21],
			  temp6Buff[21*21], KT[1*21], temp7Buff[21*21], temp8Buff[21*21],
			  KRb[21*1];

	// K*Hb
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
}



