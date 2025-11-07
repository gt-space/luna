#include "Inc/update_sensors.h"

void update_GPS(float32_t *x_plus, float32_t *P_plus, float32_t *Pq_plus, float32_t *x_minus, float32_t *P_minus, float32_t *Pq_minus, float32_t *H, float32_t *R, float32_t *lla_meas)
{
	// STEP 2: KALMAN GAIN - Adaptive underweighting based on position uncertainty
	float32_t trace_sub = P_minus[3 * 21 + 3] + P_minus[4 * 21 + 4] + P_minus[5 * 21 + 5];
	float32_t beta = (trace_sub > 1000.0f) ? 0.25f : 0.0f;

	// Compute W = (1+beta)*H*P_minus*H' + R
	arm_matrix_instance_f32 H_mat, P_minus_mat, HPT_mat;
	float32_t HP_data[63];
	arm_mat_init_f32(&H_mat, 3, 21, H);
	arm_mat_init_f32(&P_minus_mat, 21, 21, P_minus);
	arm_mat_init_f32(&HPT_mat, 3, 21, HP_data);
	arm_mat_mult_f32(&H_mat, &P_minus_mat, &HPT_mat);

	arm_matrix_instance_f32 HT_mat;
	float32_t HT_data[63], W_temp_data[9];
	arm_mat_init_f32(&HT_mat, 21, 3, HT_data);
	arm_mat_trans_f32(&H_mat, &HT_mat);

	arm_matrix_instance_f32 W_temp_mat;
	arm_mat_init_f32(&W_temp_mat, 3, 3, W_temp_data);
	arm_mat_mult_f32(&HPT_mat, &HT_mat, &W_temp_mat);

	float32_t W_data[9];
	for (int i = 0; i < 9; i++)
	{
		W_data[i] = (1.0f + beta) * W_temp_data[i] + R[i];
	}

	// Compute PH = P_minus*H'
	arm_matrix_instance_f32 PH_mat;
	float32_t PH_data[63];
	arm_mat_init_f32(&PH_mat, 21, 3, PH_data);
	arm_mat_mult_f32(&P_minus_mat, &HT_mat, &PH_mat);

	// Compute K = PH * W^(-1) using library function
	arm_matrix_instance_f32 W_mat, W_inv_mat;
	float32_t W_inv[9];
	arm_mat_init_f32(&W_mat, 3, 3, W_data);
	arm_mat_init_f32(&W_inv_mat, 3, 3, W_inv);
	arm_mat_inverse_f32(&W_mat, &W_inv_mat);

	arm_matrix_instance_f32 K_mat;
	float32_t K_data[63];
	arm_mat_init_f32(&K_mat, 21, 3, K_data);
	arm_mat_mult_f32(&PH_mat, &W_inv_mat, &K_mat);

	// STEP 3: MEASUREMENT UPDATE - Compute measurement residual
	float32_t residual_data[3];
	for (int i = 0; i < 3; i++)
	{
		residual_data[i] = lla_meas[i] - x_minus[4 + i];
	}

	// Compute Delta_x = K * residual
	arm_matrix_instance_f32 residual_vec, Delta_x_vec;
	float32_t Delta_x_data[21];
	arm_mat_init_f32(&residual_vec, 3, 1, residual_data);
	arm_mat_init_f32(&Delta_x_vec, 21, 1, Delta_x_data);
	arm_mat_mult_f32(&K_mat, &residual_vec, &Delta_x_vec);

	// Update state components
	// Quaternion: unchanged
	for (int i = 0; i < 4; i++)
	{
		x_plus[i] = x_minus[i];
	}

	// Position: p_plus = x_minus(5:7) + Delta_x(4:6)
	for (int i = 0; i < 3; i++)
	{
		x_plus[4 + i] = x_minus[4 + i] + Delta_x_data[3 + i];
	}

	// Velocity: v_plus = x_minus(8:10) + Delta_x(7:9)
	for (int i = 0; i < 3; i++)
	{
		x_plus[7 + i] = x_minus[7 + i] + Delta_x_data[6 + i];
	}

	// Gyro bias: unchanged
	for (int i = 0; i < 3; i++)
	{
		x_plus[10 + i] = x_minus[10 + i];
	}

	// Accel bias: ba_plus = x_minus(14:16) + Delta_x(13:15)
	for (int i = 0; i < 3; i++)
	{
		x_plus[13 + i] = x_minus[13 + i] + Delta_x_data[12 + i];
	}

	// Gyro scale factor: kg_plus = x_minus(17:19) + Delta_x(16:18)
	for (int i = 0; i < 3; i++)
	{
		x_plus[16 + i] = x_minus[16 + i] + Delta_x_data[15 + i];
	}

	// Accel scale factor: ka_plus = x_minus(20:22) + Delta_x(19:21)
	for (int i = 0; i < 3; i++)
	{
		x_plus[19 + i] = x_minus[19 + i] + Delta_x_data[18 + i];
	}

	// Compute P_plus = (I - K*H)*P_minus*(I - K*H)' + K*R*K'
	// Compute KH = K*H
	arm_matrix_instance_f32 KH_mat;
	float32_t KH_data[441];
	arm_mat_init_f32(&KH_mat, 21, 21, KH_data);
	arm_mat_mult_f32(&K_mat, &H_mat, &KH_mat);

	// Create identity matrix I
	float32_t I_data[441];
	arm_matrix_instance_f32 I_mat;
	arm_mat_eye_f32(&I_mat, I_data, 21);

	// Compute I_KH = I - K*H
	arm_matrix_instance_f32 I_KH_mat;
	float32_t I_KH_data[441];
	arm_mat_init_f32(&I_KH_mat, 21, 21, I_KH_data);
	arm_mat_sub_f32(&I_mat, &KH_mat, &I_KH_mat);

	// Compute I_KH_T (transpose of I_KH)
	arm_matrix_instance_f32 I_KH_T_mat;
	float32_t I_KH_T_data[441];
	arm_mat_init_f32(&I_KH_T_mat, 21, 21, I_KH_T_data);
	arm_mat_trans_f32(&I_KH_mat, &I_KH_T_mat);

	// Compute temp1 = I_KH * P_minus
	arm_matrix_instance_f32 temp1_mat;
	float32_t temp1_data[441];
	arm_mat_init_f32(&temp1_mat, 21, 21, temp1_data);
	arm_mat_mult_f32(&I_KH_mat, &P_minus_mat, &temp1_mat);

	// Compute temp2 = (I_KH * P_minus) * I_KH_T
	arm_matrix_instance_f32 temp2_mat;
	float32_t temp2_data[441];
	arm_mat_init_f32(&temp2_mat, 21, 21, temp2_data);
	arm_mat_mult_f32(&temp1_mat, &I_KH_T_mat, &temp2_mat);

	// Compute K*R
	arm_matrix_instance_f32 KR_mat;
	float32_t KR_data[63];
	arm_mat_init_f32(&KR_mat, 21, 3, KR_data);
	arm_matrix_instance_f32 R_mat;
	arm_mat_init_f32(&R_mat, 3, 3, R);
	arm_mat_mult_f32(&K_mat, &R_mat, &KR_mat);

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
	arm_matrix_instance_f32 P_plus_mat;
	arm_mat_init_f32(&P_plus_mat, 21, 21, P_plus);
	arm_mat_add_f32(&temp2_mat, &KRK_mat, &P_plus_mat);

	// Pq_plus = Pq_minus (unchanged)
	memcpy(Pq_plus, Pq_minus, 36 * sizeof(float32_t));
}

// Assumed quaternion helper functions you already have:
// - arm_quaternion_scalar_f32()
// - arm_quaternion_vector_f32()
// - arm_quaternion_qconj_f32()
// - arm_quaternion_sandwich_f32()
// - arm_quaternion_exp_f32()
// - arm_quaternion_product_single_f32()

// - x_minus: (22 x 1)
// - P_minus: (21 x 21)
// - Pq_minus: (6 x 6)
// - Hq: (3 x 6)
// - Rq: (3 x 3)
// - R: (3 x 3)
// - magI: (3 x 1)
// - mag_meas: (3 x 1)

void update_mag(
	arm_matrix_instance_f32* x_minus,
    arm_matrix_instance_f32* P_minus,
    arm_matrix_instance_f32* Pq_minus,
    arm_matrix_instance_f32* Hq,
    arm_matrix_instance_f32* Rq,
    arm_matrix_instance_f32* R,
    arm_matrix_instance_f32* magI,
    arm_matrix_instance_f32* mag_meas,
    arm_matrix_instance_f32* x_plus,
    arm_matrix_instance_f32* P_plus,
    arm_matrix_instance_f32* Pq_plus,
    float32_t x_plus_buff[22*1],
    float32_t P_plus_buff[21*21],
    float32_t Pq_plus_buff[6*6]
) {

    // Kq = Pq_minus * Hq' * inv(Hq * Pq_minus * Hq' + Rq)

	float32_t temp1Buff[3*6], HqT[6*3], temp2Buff[9], temp3Buff[3*3],
			  invMat[3*3], firstTerm[6*3], KqBuff[6*3];

	arm_matrix_instance_f32 Kq;
	arm_mat_init_f32(&Kq, 6, 3, KqBuff);

	arm_mat_mult_f32(Hq, Pq_minus, &(arm_matrix_instance_f32){3, 6, temp1Buff});

	arm_mat_trans_f32(Hq, &(arm_matrix_instance_f32){6, 3, HqT});

	arm_mat_mult_f32(&(arm_matrix_instance_f32){3, 6, temp1Buff},
					 &(arm_matrix_instance_f32){6, 3, HqT},
					 &(arm_matrix_instance_f32){3, 3, temp2Buff});

	arm_mat_add_f32(&(arm_matrix_instance_f32){3, 3, temp2Buff},
					Rq,
					&(arm_matrix_instance_f32){3, 3, temp3Buff});

	arm_mat_inverse_f32(&(arm_matrix_instance_f32){3, 3, temp3Buff},
						&(arm_matrix_instance_f32){3, 3, invMat});

	arm_mat_mult_f32(Pq_minus,
					 &(arm_matrix_instance_f32){6, 3, HqT},
					 &(arm_matrix_instance_f32){6, 3, firstTerm});

	arm_mat_mult_f32(&(arm_matrix_instance_f32){6, 3, firstTerm},
					 &(arm_matrix_instance_f32){3, 3, invMat},
					 &Kq);

	// H = [skew(magI) zeros(3,18)];

	float32_t HBuff[3*21] = {0};
	float32_t magISkewBuff[3*3];
	arm_matrix_instance_f32 H, magISkew;

	arm_mat_init_f32(&H, 3, 21, HBuff);
	arm_mat_skew_f32(magI, &magISkew, magISkewBuff);
	arm_mat_place_f32(&magISkew, &H, 0, 0);

    // K = P_minus*H'/(H*P_minus*H' + R);

	arm_matrix_instance_f32 K;
	float32_t KBuff[21*3], HT[21*3], PMinusHT[21*3],
			  HPMinus[3*21], HPMinusHT[3*3], HPMinusHTR[3*3], invHPMinusHTR[3*3];

	arm_mat_init_f32(&K, 21, 3, KBuff);

	arm_mat_trans_f32(&H, &(arm_matrix_instance_f32){21, 3, HT});

	arm_mat_mult_f32(P_minus,
					 &(arm_matrix_instance_f32){21, 3, HT},
					 &(arm_matrix_instance_f32){21, 3, PMinusHT});

	arm_mat_mult_f32(&H,
					 Pq_minus,
					 &(arm_matrix_instance_f32){3, 21, HPMinus});

	arm_mat_mult_f32(&(arm_matrix_instance_f32){3, 21, HPMinus},
					 &(arm_matrix_instance_f32){21, 3, HT},
					 &(arm_matrix_instance_f32){3, 3, HPMinusHT});

	arm_mat_add_f32(&(arm_matrix_instance_f32){3, 3, HPMinusHT},
					R,
					&(arm_matrix_instance_f32){3, 3, HPMinusHTR});

	arm_mat_inverse_f32(&(arm_matrix_instance_f32){3, 3, HPMinusHTR},
						&(arm_matrix_instance_f32){3, 3, invHPMinusHTR});

	arm_mat_mult_f32(&(arm_matrix_instance_f32){21, 3, PMinusHT},
					 &(arm_matrix_instance_f32){3, 3, invHPMinusHTR},
					 &K);

	/*
	    q_minus = x_minus(1:4);
		dv = magI - q2v(sandwich(q_minus,[0;mag_meas]));
		err = Kq*dv;
		cq = err(1:3);
		cbg = err(4:6);
		dq = quaternion_exp(-cq/2);
	*/

	arm_matrix_instance_f32 y, dq, qPlus, qConjPlus, bgPlusSand, bgPlus;
	float32_t yBuff[4], dvBuff[3], qSandVec[3], errBuff[6],
			  cq[3], cbg[3], dqBuff[4], qPlusBuff[4];

	float32_t qMinus[4] = {x_minus->pData[0], x_minus->pData[1], x_minus->pData[2], x_minus->pData[3]};
	float32_t qMag[4] = {0, mag_meas->pData[0], mag_meas->pData[1], mag_meas->pData[2]};

	arm_quaternion_sandwich_f32(&(arm_matrix_instance_f32){4, 1, qMinus},
								&(arm_matrix_instance_f32){4, 1, qMag},
								&y,
								yBuff);

	qSandVec[0] = yBuff[1];
	qSandVec[1] = yBuff[2];
	qSandVec[2] = yBuff[3];

	arm_mat_sub_f32(magI,
					&(arm_matrix_instance_f32){3, 1, qSandVec},
					&(arm_matrix_instance_f32){3, 1, dvBuff});

	arm_mat_mult_f32(&Kq,
					 &(arm_matrix_instance_f32){3, 1, dvBuff},
					 &(arm_matrix_instance_f32){6, 1, errBuff});

	cq[0] = -errBuff[0] / 2;
	cq[1] = -errBuff[1] / 2;
	cq[2] = -errBuff[2] / 2;

	cbg[0] = errBuff[3];
	cbg[1] = errBuff[4];
	cbg[2] = errBuff[5];

	arm_quaternion_exp_f32(&(arm_matrix_instance_f32){3, 1, cq}, &dq, dqBuff);

	/*
	     q_plus = qmult(dq,q_minus);
    	bg_plus = x_minus(11:13) - q2v(sandwich(qconj(q_plus),[0;cbg]));
    	// check the sandwich output
	 */

	arm_mat_init_f32(&qPlus, 4, 1, qPlusBuff);
	arm_quaternion_product_single_f32(dq.pData, qMinus, qPlus.pData);

	float32_t cbgBuff[4] = {0, cbg[0], cbg[1], cbg[2]};
	float32_t qConjPlusBuff[4], bgPlusSandBuff[4], bgQ2V[3], bgPlusData[3], xMinusBG[3];

	arm_quaternion_qconj_f32(&qPlus, &qConjPlus, qConjPlusBuff);

	arm_quaternion_sandwich_f32(&qConjPlus,
								&(arm_matrix_instance_f32){4, 1, cbgBuff},
								&bgPlusSand,
								bgPlusSandBuff);

	bgQ2V[0] = bgPlusSandBuff[1];
	bgQ2V[1] = bgPlusSandBuff[2];
	bgQ2V[2] = bgPlusSandBuff[3];

	xMinusBG[0] = x_minus->pData[10];
	xMinusBG[1] = x_minus->pData[11];
	xMinusBG[2] = x_minus->pData[12];

	arm_mat_init_f32(&bgPlus, 3, 1, bgPlusData);

	arm_mat_sub_f32(&(arm_matrix_instance_f32){3, 1, xMinusBG},
					&(arm_matrix_instance_f32){3, 1, bgQ2V},
					&bgPlus);

	/*
	  	x_plus = x_minus;
    	x_plus(1:4) = q_plus;
    	x_plus(11:13) = bg_plus;
	 */

	x_plus_buff[0] = qPlus.pData[0];
	x_plus_buff[1] = qPlus.pData[1];
	x_plus_buff[2] = qPlus.pData[2];
	x_plus_buff[3] = qPlus.pData[3];

	x_plus_buff[10] = bgPlus.pData[0];
	x_plus_buff[11] = bgPlus.pData[1];
	x_plus_buff[12] = bgPlus.pData[2];

	// Pq_plus = (eye(6) - Kq * Hq) * Pq_minus * (eye(6) - Kq * Hq)' + Kq * Rq * Kq';

	arm_matrix_instance_f32 eye6;

	float32_t eye6Buff[6*6], eye21Buff[21*21], KqHq[6*6], eye6KqHq[6*6],
			  eye6KqHqPQMinus[6*6], eye6KqHqT[6*6], partResult1[6*6], KqT[3*6],
			  KqRq[6*3], partResult2[6*6];

	arm_mat_eye_f32(&eye6, eye6Buff, 6);

	arm_mat_mult_f32(&Kq, Hq, &(arm_matrix_instance_f32){6, 6, KqHq});

	arm_mat_sub_f32(&eye6,
					&(arm_matrix_instance_f32){6, 6, KqHq},
					&(arm_matrix_instance_f32){6, 6, eye6KqHq});

	arm_mat_mult_f32(&(arm_matrix_instance_f32){6, 6, eye6KqHq},
					 Pq_minus,
					 &(arm_matrix_instance_f32){6, 6, eye6KqHqPQMinus});

	arm_mat_trans_f32(&(arm_matrix_instance_f32){6, 6, eye6KqHq},
					  &(arm_matrix_instance_f32){6, 6, eye6KqHqT});

	arm_mat_mult_f32(&(arm_matrix_instance_f32){6, 6, eye6KqHqPQMinus},
				     &(arm_matrix_instance_f32){6, 6, eye6KqHqT},
					 &(arm_matrix_instance_f32){6, 6, partResult1});

	arm_mat_trans_f32(&Kq,
					  &(arm_matrix_instance_f32){3, 6, KqT});

	arm_mat_mult_f32(&Kq,
				     &Rq,
					 &(arm_matrix_instance_f32){6, 3, KqRq});

	arm_mat_mult_f32(&(arm_matrix_instance_f32){6, 3, KqRq},
					 &(arm_matrix_instance_f32){3, 6, KqT},
					 &(arm_matrix_instance_f32){6, 6, partResult2});

	arm_mat_add_f32(&(arm_matrix_instance_f32){6, 6, partResult1},
					&(arm_matrix_instance_f32){6, 6, partResult2},
					&(arm_matrix_instance_f32){6, 6, Pq_plus_buff});

	// P_plus = (eye(21,21) - K*H)*P_minus* (eye(21,21)-K * H)' + K*R*K';

	arm_matrix_instance_f32 eye21;
	float32_t KH[21*21], eye21KH[21*21],
			  eye21KHPMinus[21*21], eye21KHT[21*21], part1Result[21*21],
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

	arm_mat_add_f32(&(arm_matrix_instance_f32){21, 21, part1Result},
					&(arm_matrix_instance_f32){21, 21, part2Result},
					&(arm_matrix_instance_f32){21, 21, P_plus_buff});

}

void update_baro(arm_matrix_instance_f32* xMinus, arm_matrix_instance_f32* PMinus, arm_matrix_instance_f32* PqMinus,
				arm_matrix_instance_f32* Rb, float32_t pressure, arm_matrix_instance_f32* Hb) {



}



