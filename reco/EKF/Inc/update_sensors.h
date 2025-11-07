#ifndef _UPDATE_SENS
#define _UPDATE_SENS

#include "common.h"
#include "ekf_utils.h"
#include "matrix_extensions.h"
#include "quaternion_extensions.h"
#include "trig_extensions.h"

void update_GPS(float32_t *x_plus, float32_t *P_plus, float32_t *Pq_plus, float32_t *x_minus, float32_t *P_minus, float32_t *Pq_minus, float32_t *H, float32_t *R, float32_t *lla_meas);

void update_mag(arm_matrix_instance_f32* x_minus, arm_matrix_instance_f32* P_minus, arm_matrix_instance_f32* Pq_minus,
				arm_matrix_instance_f32* Hq, arm_matrix_instance_f32* Rq, arm_matrix_instance_f32* R,
				arm_matrix_instance_f32* magI, arm_matrix_instance_f32* mag_meas, arm_matrix_instance_f32* x_plus,
				arm_matrix_instance_f32* P_plus, arm_matrix_instance_f32* Pq_plus, float32_t* x_plus_buff,
				float32_t* P_plus_buff, float32_t* Pq_plus_buff);

#endif
