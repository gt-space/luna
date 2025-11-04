#ifndef COMPUTE_F_
#define COMPUTE_F_

#include "ekf_utils.h"
#include "common.h"
#include "compute_hats.h"
#include "matrix_extensions.h"
#include "quaternion_extensions.h"
#include "trig_extensions.h"

void compute_F(arm_matrix_instance_f32* q, arm_matrix_instance_f32* sf_a, arm_matrix_instance_f32* sf_g,
			  arm_matrix_instance_f32* bias_g, arm_matrix_instance_f32* bias_a, float32_t phi, float32_t h,
			  float32_t vn, float32_t ve, float32_t vd, arm_matrix_instance_f32* a_meas, arm_matrix_instance_f32* w_meas,
			  float32_t we, arm_matrix_instance_f32* F, float32_t FBuff[21*21]);

void compute_G(arm_matrix_instance_f32* sf_g, arm_matrix_instance_f32* sf_a, arm_matrix_instance_f32* q,
			   arm_matrix_instance_f32* G, float32_t GBuff[21*12]);

#endif
