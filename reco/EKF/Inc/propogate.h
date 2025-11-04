#ifndef PROPOGATE_
#define PROPOGATE_

#include "common.h"
#include "ekf_utils.h"
#include "compute_F.h"
#include "matrix_extensions.h"
#include "quaternion_extensions.h"
#include "trig_extensions.h"

void compute_qdot(arm_matrix_instance_f32* x, arm_matrix_instance_f32* what,
				  arm_matrix_instance_f32* qdot, float32_t qDotBuff[4]);

void compute_lla_dot(float32_t phi, float32_t h, float32_t vn, float32_t ve, float32_t vd,
					 arm_matrix_instance_f32* llaDot, float32_t llaDotBuff[3]);

void compute_vdot(float32_t phi, float32_t h, float32_t vn, float32_t ve, float32_t vd,
				  float32_t ahat_n[3], float32_t we, arm_matrix_instance_f32* vdot, float32_t vdotBuff[3]);

void compute_Pdot(arm_matrix_instance_f32* q, arm_matrix_instance_f32* sf_a, arm_matrix_instance_f32* sf_g,
		  	  	  arm_matrix_instance_f32* bias_g, arm_matrix_instance_f32* bias_a, arm_matrix_instance_f32* a_meas,
				  arm_matrix_instance_f32* w_meas, arm_matrix_instance_f32* P, arm_matrix_instance_f32* Q,
				  float32_t phi, float32_t h, float32_t vn, float32_t ve, float32_t vd, float32_t we,
				  arm_matrix_instance_f32* Pdot, float32_t PdotBuff[21*21]);

void compute_Pqdot(float32_t *x, float32_t *Pq, float32_t *Qq, float32_t *w_meas,
                   arm_matrix_instance_f32* Pqdot, float32_t PqdotBuff[6*6]);

#endif
