/*
 * quaternion_extensions.h
 *
 *  Created on: Oct 21, 2025
 *      Author: Raey Ayalew
 */

#ifndef _QUAT_EXTS
#define _QUAT_EXTS

#include "common.h"

static inline void arm_quaternion_scalar_f32(arm_matrix_instance_f32* quaternion,
											 arm_matrix_instance_f32 *scalarOut,
											 float32_t* buffer) {
	*buffer = quaternion->pData[0];
    arm_mat_init_f32(scalarOut, 1, 1, buffer);
}

static inline void arm_quaternion_vector_f32(arm_matrix_instance_f32* quaternion,
											 arm_matrix_instance_f32 *vectorOut,
											 float32_t buffer[3]) {
	buffer[0] = quaternion->pData[1];
	buffer[1] = quaternion->pData[2];
	buffer[2] = quaternion->pData[3];
    arm_mat_init_f32(vectorOut, 3, 1, buffer);
}

static inline void arm_quaternion_qconj_f32(arm_matrix_instance_f32* q, arm_matrix_instance_f32* qBar,
											float32_t qBarBuff[4]) {
	qBarBuff[0] = q->pData[0];
	qBarBuff[1] = -q->pData[1];
	qBarBuff[2] = -q->pData[2];
	qBarBuff[3] = -q->pData[3];
	arm_mat_init_f32(qBar, 4, 1, qBarBuff);
}

void arm_quaternion_sandwich_f32(arm_matrix_instance_f32* q, arm_matrix_instance_f32* x,
								 arm_matrix_instance_f32* y, float32_t yBuff[4]);

void arm_quaternion_exp_f32(arm_matrix_instance_f32* v,
                            arm_matrix_instance_f32* dq,
                            float32_t dqBuff[4]);

#endif /* _QUATERNION_EXT_H */
