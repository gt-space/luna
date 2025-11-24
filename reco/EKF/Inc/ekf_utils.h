#ifndef _EKF_UTILS
#define _EKF_UTILS

#include "../CControl/ccontrol.h"
#include "common.h"
#include "matrix_extensions.h"
#include "quaternion_extensions.h"
#include "trig_extensions.h"
#include "stdio.h"
#include "stdbool.h"

void getStateQuaternion(arm_matrix_instance_f32* x, arm_matrix_instance_f32* quaternion, float32_t quaternionData[4]);
void getStatePosition(arm_matrix_instance_f32* x, arm_matrix_instance_f32* position, float32_t posData[3]);
void getStateVelocity(arm_matrix_instance_f32* x, arm_matrix_instance_f32* vel, float32_t velData[3]);
void getStateGBias(arm_matrix_instance_f32* x, arm_matrix_instance_f32* gBias, float32_t gData[3]);
void getStateABias(arm_matrix_instance_f32* x, arm_matrix_instance_f32* aBias, float32_t aData[3]);
void getStateGSF(arm_matrix_instance_f32* x, arm_matrix_instance_f32* g_sf, float32_t g_sf_data[3]);
void getStateASF(arm_matrix_instance_f32* x, arm_matrix_instance_f32* a_sf, float32_t a_sf_data[3]);
void quaternion2DCM(const arm_matrix_instance_f32* quaternion, arm_matrix_instance_f32* CB2I, float32_t CB2IBuffer[9]);
void compute_radii(float32_t phi, float32_t returnVector[4]);
void compute_g_dg(float32_t phi, float32_t h, float32_t gDgResult[3]);
void printMatrix(arm_matrix_instance_f32* matrix);
void printMatrixDouble(arm_matrix_instance_f64* matrix);
bool areMatricesEqual(arm_matrix_instance_f32* A, arm_matrix_instance_f32* B);
void copyMatrixDouble(arm_matrix_instance_f32* src, arm_matrix_instance_f64* dest);
void copyMatrixFloat(arm_matrix_instance_f64* src, arm_matrix_instance_f32* dest);
void arm_mat_to_colmajor(arm_matrix_instance_f64 *src, arm_matrix_instance_f64* dest, float64_t* destData);
void arm_mat_to_rowmajor(arm_matrix_instance_f64 *src, arm_matrix_instance_f64* dst, float64_t* data);
void copyMatrix(float32_t* src, float32_t* dest, uint16_t total);
void calculateEigSym(arm_matrix_instance_f32* A);

#endif
