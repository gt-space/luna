#ifndef _COMPUTE_COMMON
#define _COMPUTE_COMMON

#include "common.h"
#include "cmsis_dsp_extensions\matrix_extensions.h"
#include "cmsis_dsp_extensions\quaternion_extensions.h"
#include "cmsis_dsp_extensions\trig_extensions.h"

void quaternion2DCM(const arm_matrix_instance_f32* quaternion, arm_matrix_instance_f32* CB2I, float32_t* CB2IBuffer);
void compute_radii(float32_t phi, float32_t* returnVector);
void compute_g_dg();

#endif