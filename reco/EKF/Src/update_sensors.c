#include "Inc/ekf.h"

void update_GPS(arm_matrix_instance_f32* x_minus, arm_matrix_instance_f32* P_minus, arm_matrix_instance_f32* H,
				arm_matrix_instance_f32* R, arm_matrix_instance_f32* lla_meas, arm_matrix_instance_f32* x_plus,
				arm_matrix_instance_f32* P_plus, float32_t xPlusData[22*1], float32_t P_plus_data[21*21]) {
	// STEP 2: KALMAN GAIN - Adaptive underweighting based on position uncertainty
	float32_t trace_sub = P_minus->pData[3 * 21 + 3] + P_minus->pData[4 * 21 + 4] + P_minus->pData[5 * 21 + 5];
	float32_t beta = (trace_sub > 1000.0f) ? 0.25f : 0.0f;

	// Compute W = (1+beta)*H*P_minus*H' + R
	arm_matrix_instance_f32 HPT_mat;
	float32_t HP_data[63];
	arm_mat_init_f32(&HPT_mat, 3, 21, HP_data);
	arm_mat_mult_f32(H, P_minus, &HPT_mat);

	arm_matrix_instance_f32 HT_mat;
	float32_t HT_data[63], W_temp_data[9];
	arm_mat_init_f32(&HT_mat, 21, 3, HT_data);
	arm_mat_trans_f32(H, &HT_mat);

	arm_matrix_instance_f32 W_temp_mat;
	arm_mat_init_f32(&W_temp_mat, 3, 3, W_temp_data);
	arm_mat_mult_f32(&HPT_mat, &HT_mat, &W_temp_mat);

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

	// Gyro bias: unchanged
	for (int i = 0; i < 3; i++)
	{
		x_plus->pData[10 + i] = x_minus->pData[10 + i];
	}

	// Accel bias: ba_plus = x_minus(14:16) + Delta_x(13:15)
	for (int i = 0; i < 3; i++)
	{
		x_plus->pData[13 + i] = x_minus->pData[13 + i] + Delta_x_data[12 + i];
	}

	// Gyro scale factor: kg_plus = x_minus(17:19) + Delta_x(16:18)
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

    //     Kq = Pq_minus * Hq' / (Hq * Pq_minus * Hq' + Rq);

	float32_t temp1Buff[3*6], HqT[6*3], temp2Buff[9], temp3Buff[3*3],
			  firstTerm[6*3], KqBuff[6*3];

	arm_matrix_instance_f32 Kq, A, B;

	arm_mat_mult_f32(Hq, Pq_minus, &(arm_matrix_instance_f32){3, 6, temp1Buff});

	arm_mat_trans_f32(Hq, &(arm_matrix_instance_f32){6, 3, HqT});

	arm_mat_mult_f32(&(arm_matrix_instance_f32){3, 6, temp1Buff},
					 &(arm_matrix_instance_f32){6, 3, HqT},
					 &(arm_matrix_instance_f32){3, 3, temp2Buff});

	arm_mat_init_f32(&A, 3, 3, temp3Buff);
	arm_mat_init_f32(&B, 6, 3, firstTerm);

	// A
	/*
	arm_mat_add_f32(&(arm_matrix_instance_f32){3, 3, temp2Buff},
					Rq,
					&(arm_matrix_instance_f32){3, 3, temp3Buff});
	*/
	arm_mat_add_f32(&(arm_matrix_instance_f32){3, 3, temp2Buff}, Rq, &A);

	// B
	arm_mat_mult_f32(Pq_minus,
					 &(arm_matrix_instance_f32){6, 3, HqT},
					 &B);

	arm_matrix_instance_f64 ADouble, BDouble, KqDouble;
	float64_t ADoubleData[3*3], BDoubleData[6*3], KqDoubleData[6*3];

	arm_mat_init_f64(&ADouble, 3, 3, ADoubleData);
	arm_mat_init_f64(&BDouble, 6, 3, BDoubleData);
	arm_mat_init_f64(&KqDouble, 6, 3, KqDoubleData);
	arm_mat_init_f32(&Kq, 6, 3, KqBuff);

	copyMatrixDouble(&A, &ADouble);
	copyMatrixDouble(&B, &BDouble);

	arm_mat_linsolve_right_f64(&ADouble,
							  &BDouble,
							  &KqDouble,
							  KqDoubleData);

	copyMatrixFloat(&KqDouble, &Kq);

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
			  HPMinus[3*21], HPMinusHT[3*3], HPMinusHTR[3*3];

	arm_mat_trans_f32(&H, &(arm_matrix_instance_f32){21, 3, HT});

	arm_mat_mult_f32(P_minus,
					 &(arm_matrix_instance_f32){21, 3, HT},
					 &(arm_matrix_instance_f32){21, 3, PMinusHT});

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

	arm_matrix_instance_f64 KDouble;
	float64_t PMinusHTDouble[21*3], KBuffDouble[21*3];
	arm_mat_init_f64(&BDouble, 21, 3, PMinusHTDouble);

	copyMatrixDouble(&A, &ADouble);
	copyMatrixDouble(&B, &BDouble);


	arm_mat_linsolve_right_f64(&ADouble,
							   &BDouble,
							   &KDouble,
							   KBuffDouble);

	copyMatrixFloat(&KDouble, &K);

	/*
	    q_minus = x_minus(1:4);
		dv = magI - q2v(sandwich(q_minus,[0;mag_meas]));
		err = Kq*dv;
		cq = err(1:3);
		cbg = err(4:6);
		dq = quaternion_exp(-cq/2);
	*/

	arm_matrix_instance_f32 y, dq, qPlus, qConjPlus, bgPlusSand, bgPlus, dv;
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

	arm_mat_init_f32(&dv, 3, 1, dvBuff);

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

	memcpy(x_plus_buff, x_minus->pData, 22*sizeof(float32_t));

	x_plus_buff[0] = qPlus.pData[0];
	x_plus_buff[1] = qPlus.pData[1];
	x_plus_buff[2] = qPlus.pData[2];
	x_plus_buff[3] = qPlus.pData[3];

	x_plus_buff[10] = bgPlus.pData[0];
	x_plus_buff[11] = bgPlus.pData[1];
	x_plus_buff[12] = bgPlus.pData[2];

	arm_mat_init_f32(x_plus, 22, 1, x_plus_buff);

	// Pq_plus = (eye(6) - Kq * Hq) * Pq_minus * (eye(6) - Kq * Hq)' + Kq * Rq * Kq';

	arm_matrix_instance_f32 eye6;

	float32_t eye6Buff[6*6], KqHq[6*6], eye6KqHq[6*6],
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
				     Rq,
					 &(arm_matrix_instance_f32){6, 3, KqRq});

	arm_mat_mult_f32(&(arm_matrix_instance_f32){6, 3, KqRq},
					 &(arm_matrix_instance_f32){3, 6, KqT},
					 &(arm_matrix_instance_f32){6, 6, partResult2});

	arm_mat_init_f32(Pq_plus, 6, 6, Pq_plus_buff);

	arm_mat_add_f32(&(arm_matrix_instance_f32){6, 6, partResult1},
					&(arm_matrix_instance_f32){6, 6, partResult2},
					Pq_plus);

	// P_plus = (eye(21,21) - K*H)*P_minus* (eye(21,21)-K * H)' + K*R*K';

	arm_matrix_instance_f32 eye21;
	float32_t KH[21*21], eye21KH[21*21], eye21KHPMinus[21*21], eye21KHT[21*21], part1Result[21*21],
			  KR[21*3], KT[3*21], part2Result[21*21], eye21Buff[21*21];

	arm_mat_eye_f32(&eye21, eye21Buff, 21);

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

void update_baro(arm_matrix_instance_f32* xMinus, arm_matrix_instance_f32* PMinus, float32_t pressMeas,
				 float32_t Rb, arm_matrix_instance_f32* xPlus, arm_matrix_instance_f32* Pplus,
				 float32_t xPlusData[22*1], float32_t pPlusData[21*21]) {

	// Hb = Hb_func(x_minus);
    // K = P_minus*Hb'/(Hb*P_minus*Hb' + Rb);

	arm_matrix_instance_f32 Hb, K;
	float32_t HbData[1*21], HbTData[21*1], KData[21*1], temp1[1*21], temp2, temp3[21*1];
	float64_t temp2Double;

	arm_mat_init_f32(&K, 21, 1, KData);

	// Hb
	pressure_derivative(xMinus, &Hb, HbData);

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
	temp2Double = temp2;

	// PMinus*Hb'
	arm_mat_mult_f32(PMinus,
					 &(arm_matrix_instance_f32){21, 1, HbTData},
					 &(arm_matrix_instance_f32){21, 1, temp3});

	arm_matrix_instance_f64 temp3Double, KDouble;
	float64_t temp3DoubleData[21*1], KDoubleData[21*1];
	arm_mat_init_f64(&temp3Double, 21, 1, temp3DoubleData);
	arm_mat_init_f64(&KDouble, 21, 1, KDoubleData);

	copyMatrixDouble(&(arm_matrix_instance_f32){21, 1, temp3}, &temp3Double);

	arm_mat_scale_f64(&temp3Double, 1 / temp2Double, &KDouble);

    // q_minus = x_minus(1:4);
    // Delta_x = K*(press_meas - hb(x_minus));
	// q_plus = q_minus;

	arm_matrix_instance_f32 deltaX;
	float32_t deltaXData[21];

	arm_mat_init_f32(&deltaX, K.numRows, K.numCols, deltaXData);

	float64_t hbFunc = (float64_t) pressure_function(xMinus);

	arm_mat_scale_f64(&KDouble, (pressMeas - hbFunc), &KDouble);

	copyMatrixFloat(&KDouble, &deltaX);


	//  p_plus = x_minus(5:7) + Delta_x(4:6);
	//  v_plus = x_minus(8:10) + Delta_x(7:9);
	//  bg_plus = x_minus(11:13);
	//  ba_plus = x_minus(14:16) + Delta_x(13:15);
	//  kg_plus = x_minus(17:19) + Delta_x(16:18);
	//  ka_plus = x_minus(20:22) + Delta_x(19:21);
	//  x_plus = [q_plus; p_plus; v_plus; bg_plus; ba_plus; kg_plus; ka_plus];

	memcpy(&xPlusData[0], &xMinus->pData[0], 4*sizeof(float32_t));
	arm_add_f32(&xMinus->pData[4], &deltaX.pData[3], &xPlusData[4], 3);
	arm_add_f32(&xMinus->pData[7], &deltaX.pData[6], &xPlusData[7], 3);

	memcpy(&xPlusData[10], &xMinus->pData[10], 3 * sizeof(float32_t));
	arm_add_f32(&xMinus->pData[13], &deltaX.pData[12], &xPlusData[13], 3);
	arm_add_f32(&xMinus->pData[16], &deltaX.pData[15], &xPlusData[16], 3);
	arm_add_f32(&xMinus->pData[19], &deltaX.pData[18], &xPlusData[19], 3);

    // P_plus = (eye(21,21) - K*Hb)*P_minus* (eye(21,21)-K * Hb)' + K*Rb*K';

	arm_matrix_instance_f32 eye21;
	float32_t eye21Data[21*21], temp3Buff[21*21], temp4Buff[21*21],
			  temp5Buff[21*21], temp6Buff[21*21], KT[1*21], temp7Buff[21*21], temp8Buff[21*21];

	arm_mat_eye_f32(&eye21, eye21Data, 21);

	arm_mat_mult_f32(&K,
					 &Hb,
					 &(arm_matrix_instance_f32){21, 21, temp3Buff});

	arm_mat_sub_f32(&eye21,
					&(arm_matrix_instance_f32){21, 21, temp3Buff},
					&(arm_matrix_instance_f32){21, 21, temp4Buff});

	arm_mat_mult_f32(&(arm_matrix_instance_f32){21, 21, temp4Buff},
					 PMinus,
					 &(arm_matrix_instance_f32){21, 21, temp5Buff});

	arm_mat_trans_f32(&(arm_matrix_instance_f32){21, 21, temp4Buff},
					  &(arm_matrix_instance_f32){21, 21, temp6Buff});

	arm_mat_mult_f32(&(arm_matrix_instance_f32){21, 21, temp5Buff},
					 &(arm_matrix_instance_f32){21, 21, temp6Buff},
					 &(arm_matrix_instance_f32){21, 21, temp7Buff});

	arm_mat_scale_f32(&K, Rb, &K);

	arm_mat_trans_f32(&K, &(arm_matrix_instance_f32){1, 21, KT});

	arm_mat_mult_f32(&K,
					 &(arm_matrix_instance_f32){1, 21, KT},
					 &(arm_matrix_instance_f32){21, 21, temp8Buff});

	arm_mat_add_f32(&(arm_matrix_instance_f32){21, 21, temp7Buff},
			        &(arm_matrix_instance_f32){21, 21, temp8Buff},
					&(arm_matrix_instance_f32){21, 21, pPlusData});

	arm_mat_init_f32(xPlus, 22, 1, xPlusData);
	arm_mat_init_f32(Pplus, 21, 21, pPlusData);
}



