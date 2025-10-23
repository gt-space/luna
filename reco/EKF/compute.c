#include "compute.h"

void compute_wn(arm_matrix_instance_f32* wn, float32_t* x, float32_t we, float32_t* buffer) {
	float32_t phi = x[5];
	float32_t h = x[7];
	float32_t vn = x[8];
	float32_t ve = x[9];

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

void compute_qdot() {

}



void compute_what(arm_matrix_instance_f32* x, float32_t w_meas) {

}