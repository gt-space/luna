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

void arm_quaternion_exp_f32(const arm_matrix_instance_f32* v,
                            arm_matrix_instance_f32* dq,
                            float32_t dqBuff[4]);

static inline arm_quaternion_calculate_Xi(const arm_matrix_instance_f32* quat, arm_matrix_instance_f32* Xi, float32_t XiData[4*3]) {
    // quat is a 4x1 vector: [q1 q2 q3 q4]^T
    float32_t q1 = quat->pData[0];
    float32_t q2 = quat->pData[1];
    float32_t q3 = quat->pData[2];
    float32_t q4 = quat->pData[3];

    // Fill Xi (4x3). Row-major.
    XiData[0]  = -q2;  XiData[1]  = -q3;  XiData[2]  = -q4;
    XiData[3]  =  q1;  XiData[4]  = -q4;  XiData[5]  =  q3;
    XiData[6]  =  q4;  XiData[7]  =  q1;  XiData[8]  = -q2;
    XiData[9]  = -q3;  XiData[10] =  q2;  XiData[11] =  q1;

    // Initialize Xi as a 4×3 matrix using the supplied storage
    arm_mat_init_f32(Xi, 4, 3, XiData);
}

#endif /* _QUATERNION_EXT_H */
