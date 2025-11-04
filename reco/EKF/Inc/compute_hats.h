#ifndef COMPUTE_HATS_
#define COMPUTE_HATS_

#include "common.h"
#include "ekf_utils.h"
#include "matrix_extensions.h"
#include "quaternion_extensions.h"
#include "trig_extensions.h"

void compute_wn(float32_t phi, float32_t h, float32_t vn, float32_t ve,
				arm_matrix_instance_f32* wn, float32_t we, float32_t buffer[3]);

void compute_what(arm_matrix_instance_f32* q, arm_matrix_instance_f32* bias_g, arm_matrix_instance_f32* sf_g,
				  float32_t phi, float32_t h, float32_t vn, float32_t ve, float32_t we, arm_matrix_instance_f32* w_meas,
				  arm_matrix_instance_f32* what, float32_t whatBuffer[3]);

void compute_ahat(arm_matrix_instance_f32* q, arm_matrix_instance_f32* sf_a, arm_matrix_instance_f32* bias_a,
				  arm_matrix_instance_f32* a_meas, arm_matrix_instance_f32* ahat_n, float32_t ahatBuff[3]);

#endif
