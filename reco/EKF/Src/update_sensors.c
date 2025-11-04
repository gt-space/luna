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
