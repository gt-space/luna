#include "compute.h"

void compute_wn(arm_matrix_instance_f32* x, arm_matrix_instance_f32* wn, float32_t we, float32_t* buffer) {
	float32_t phi = x->pData[5];
	float32_t h = x->pData[7];
	float32_t vn = x->pData[8];
	float32_t ve = x->pData[9];

	float32_t computeRadiiVec[4];
	compute_radii(phi, computeRadiiVec);

	float32_t R_phi = computeRadiiVec[0];
	float32_t R_lamb =  computeRadiiVec[1];

	arm_matrix_instance_f32 vec1;
	float32_t term1[3] = {arm_sind_f32(phi), 0, -arm_sind_f32(phi)};
	arm_mat_init_f32(&vec1, 3, 1, term1);

	arm_mat_scale_f32(&vec1, we, &vec1);

	arm_matrix_instance_f32 vec2;
	float32_t term2[3] = {ve / (R_lamb + h), -vn / (R_phi + h), -(ve * arm_tand_f32(phi)) / (R_lamb + h)};
	arm_mat_init_f32(&vec2, 3, 1, term2);

	float32_t finalTerm[3];
	arm_mat_init_f32(wn, 3, 1, finalTerm);
	arm_mat_add_f32(&vec1, &vec2, wn);
}

void compute_what(arm_matrix_instance_f32* x, arm_matrix_instance_f32* w_meas, float32_t we, arm_matrix_instance_f32* what, float32_t* whatBuffer) {
	arm_matrix_instance_f32 sf_g, bias_g, q, D_bn, wn, product;
	float32_t sf_g_buff[3], bias_g_buff[3], q_buff[3], D_bn_buff[9], wn_buff[3], productBuff[3];

	getStateQuaternion(x, &q, q_buff);
	getStateGBias(x, &bias_g, bias_g_buff);
	getStateGSF(x, &sf_g, sf_g_buff);

	quaternion2DCM(&q, &D_bn, D_bn_buff);
	compute_wn(x, &wn, we, wn_buff);

	arm_offset_f32(sf_g.pData, 1.0f, sf_g.pData, 3);

	for (uint32_t i = 0; i < sf_g.numRows * sf_g.numCols; i++) {
	    sf_g.pData[i] = 1.0f / sf_g.pData[i];
	}

	arm_sub_f32(w_meas->pData, bias_g.pData, w_meas->pData, 3);
	arm_mult_f32(w_meas->pData, sf_g.pData, w_meas->pData, 3);

	arm_mat_init_f32(&product, 3, 1, productBuff);
	arm_mat_mult_f32(&D_bn, &wn, &product);

	arm_mat_init_f32(what, 3, 1, whatBuffer);
	arm_sub_f32(sf_g.pData, productBuff, what->pData, 3);
}

void compute_ahat(arm_matrix_instance_f32* x, arm_matrix_instance_f32* a_meas, arm_matrix_instance_f32* ahat_n, float32_t* ahatBuff) {
	arm_matrix_instance_f32 sf_a, bias_a, q, D_bn;
	float32_t sfaBuff[3], biasABuff[3], qBuff[4], D_bn_buff[9];

	getStateQuaternion(x, &q, qBuff);
	getStateASF(x, &sf_a, sfaBuff);
	getStateABias(x, &bias_a, biasABuff);

	quaternion2DCM(&q, &D_bn, D_bn_buff);

	arm_offset_f32(sf_a.pData, 1.0f, sf_a.pData, 3);

	for (uint8_t i = 0; i < sf_a.numRows; i++) {
	    sf_a.pData[i] = 1.0f / sf_a.pData[i];
	}

	arm_sub_f32(a_meas->pData, bias_a.pData, a_meas->pData, 3);
	arm_mult_f32(a_meas->pData, sf_a.pData, a_meas->pData, 3);

	arm_mat_init_f32(ahat_n, 3, 1, ahatBuff);
	arm_mat_mult_f32(&D_bn, a_meas, ahat_n);
}

void compute_qdot(arm_matrix_instance_f32* x, arm_matrix_instance_f32* what, arm_matrix_instance_f32* qdot, float32_t* qDotBuff) {
	arm_matrix_instance_f32 q, wQuat;
	float32_t qBuff[4], wQuatBuff[4] = {0, what->pData[0], what->pData[1], what->pData[2]};

	getStateQuaternion(x, &q, qBuff);

	arm_mat_init_f32(&wQuat, 4, 1, wQuatBuff);
	arm_mat_init_f32(qdot, 4, 1, qDotBuff);

	arm_quaternion_product_single_f32(q.pData, wQuat.pData, qdot->pData);

	arm_mat_scale_f32(qdot, 0.5f, qdot);
}
