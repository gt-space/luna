/*
 * quaternion_extensions.h
 *
 *  Created on: Oct 21, 2025
 *      Author: Raey Ayalew
 */

#ifndef _MATRIX_EXTS
#define _MATRIX_EXTS

#include "common.h"

void arm_mat_eye_f32(arm_matrix_instance_f32* outputMatrix, float32_t* outMatrixData, uint32_t dim);
void arm_mat_skew_f32(const arm_matrix_instance_f32* inputVector, arm_matrix_instance_f32* outputMatrix, float32_t outMatrixData[9]);
void arm_mat_outer_product_f32(const arm_matrix_instance_f32* inputVector, arm_matrix_instance_f32* outputMatrix, float32_t* outMatrixData);
void arm_mat_get_diag_f32(const arm_matrix_instance_f32* inputMatrix, arm_matrix_instance_f32* outputMatrix, float32_t* outputData);
void arm_mat_extract_diag(const arm_matrix_instance_f32* inputMatrix, arm_matrix_instance_f32* outputMatrix, float32_t* outputData);
arm_status arm_matrix_place_f32(const arm_matrix_instance_f32* subMatrix, arm_matrix_instance_f32* destMatrix, uint16_t rowOffset, uint16_t colOffset);


#endif /* _QUATERNION_EXT_H */
