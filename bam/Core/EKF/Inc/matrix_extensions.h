/*
 * quaternion_extensions.h
 *
 *  Created on: Oct 21, 2025
 *      Author: Raey Ayalew
 */

#ifndef _MATRIX_EXTS
#define _MATRIX_EXTS

#include "common.h"
#include "stdbool.h"

void arm_mat_eye_f32(arm_matrix_instance_f32* outputMatrix, float32_t* outMatrixData, uint32_t dim);
void arm_mat_skew_f32(const arm_matrix_instance_f32* inputVector, arm_matrix_instance_f32* outputMatrix, float32_t outMatrixData[9]);
void arm_mat_outer_product_f32(const arm_matrix_instance_f32* inputVector, arm_matrix_instance_f32* outputMatrix, float32_t* outMatrixData);
void arm_mat_get_diag_f32(const arm_matrix_instance_f32* inputMatrix, arm_matrix_instance_f32* outputMatrix, float32_t* outputData);
void arm_mat_extract_diag(const arm_matrix_instance_f32* inputMatrix, arm_matrix_instance_f32* outputMatrix, float32_t* outputData);
arm_status arm_mat_place_f32(const arm_matrix_instance_f32* subMatrix, arm_matrix_instance_f32* destMatrix, uint16_t rowOffset, uint16_t colOffset);
void arm_mat_linsolve_left_f32(arm_matrix_instance_f32* A, arm_matrix_instance_f32* B, arm_matrix_instance_f32* X, float32_t* XData);

void arm_mat_linsolve_right_f32(const arm_matrix_instance_f32* A, const arm_matrix_instance_f32* B,
								arm_matrix_instance_f32* X, float32_t* XData);

void arm_mat_outer_product_f64(const arm_matrix_instance_f64* inputVector, arm_matrix_instance_f64* outputMatrix, float64_t* outMatrixData);

void arm_mat_add_f64(arm_matrix_instance_f64* pSrcA, arm_matrix_instance_f64* pSrcB, arm_matrix_instance_f64* dest);

void arm_mat_scale_f64(arm_matrix_instance_f64* pSrcA, float64_t scaleVal, arm_matrix_instance_f64* dest);

void arm_mat_linsolve_right_f64(const arm_matrix_instance_f64* A, const arm_matrix_instance_f64* B,
								arm_matrix_instance_f64* X, float64_t* XData);

void arm_mat_linsolve_left_f64(const arm_matrix_instance_f64* A, const arm_matrix_instance_f64* B,
								arm_matrix_instance_f64* X, float64_t* XData);

void arm_mat_get_diag_f64(const arm_matrix_instance_f64* inputMatrix,
						  arm_matrix_instance_f64* outputMatrix,
						  float64_t* outputData);


#endif /* _QUATERNION_EXT_H */
