#include "ekf.h"

void compute_wn(float32_t phi, float32_t h, float32_t vn, float32_t ve, arm_matrix_instance_f32* wn, float32_t we, float32_t buffer[3]) {

	float32_t computeRadiiVec[4];
	compute_radii(phi, computeRadiiVec);

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

void compute_what(arm_matrix_instance_f32* q, arm_matrix_instance_f32* bias_g, arm_matrix_instance_f32* sf_g,
				  float32_t phi, float32_t h, float32_t vn, float32_t ve, float32_t we, arm_matrix_instance_f32* w_meas,
				  arm_matrix_instance_f32* what, float32_t whatBuffer[3]) {

	arm_matrix_instance_f32 D_bn, D_bnT, wn, wnBody;
	float32_t D_bn_buff[9], wnBuff[3], productBuff[3], D_bnTData[9], sf_g_temp[3], w_corrected[3], wnBodyBuff[3];

	memcpy(sf_g_temp, sf_g->pData, 3*sizeof(float32_t));
	arm_offset_f32(sf_g_temp, 1, sf_g_temp, 3);

	for (uint8_t i = 0; i < 3; i++) {
		w_corrected[i] = (w_meas->pData[i] - bias_g->pData[i]) / sf_g_temp[i];
	}

	compute_wn(phi, h, vn, ve, &wn, we, wnBuff);

	quaternion2DCM(q, &D_bn, D_bn_buff);

	arm_mat_init_f32(&D_bnT, 3, 3, D_bnTData);
	arm_mat_trans_f32(&D_bn, &D_bnT);

	arm_mat_init_f32(&wnBody, 3, 1, wnBodyBuff);
	arm_mat_mult_f32(&D_bnT, &wn, &wnBody);

	arm_mat_init_f32(what, 3, 1, whatBuffer);
	arm_sub_f32(w_corrected, wnBody.pData, what->pData, 3);
}

void compute_ahat(arm_matrix_instance_f32* q, arm_matrix_instance_f32* sf_a, arm_matrix_instance_f32* bias_a, arm_matrix_instance_f32* a_meas, arm_matrix_instance_f32* ahat_n, float32_t ahatBuff[3]) {
	arm_matrix_instance_f32 D_bn, aBody;
	float32_t D_bn_buff[9], result[3], resultData1[3];

	quaternion2DCM(q, &D_bn, D_bn_buff);

	arm_offset_f32(sf_a->pData, 1.0f, resultData1, 3);

	arm_mat_init_f32(&aBody, 3, 1, result);
	for (uint8_t i = 0; i < 3; i++) {
	    aBody.pData[i] = (a_meas->pData[i] - bias_a->pData[i]) / resultData1[i];
	}

	arm_mat_init_f32(ahat_n, 3, 1, ahatBuff);

//	printf("A Hat:\n");
//	printMatrix(&aBody);
//	printf("\n\n");

	arm_mat_mult_f32(&D_bn, &aBody, ahat_n);
}
