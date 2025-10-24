#ifndef _COMPUTE_COMMON
#define _COMPUTE_COMMON

#include "common.h"
#include "cmsis_dsp_extensions/matrix_extensions.h"
#include "cmsis_dsp_extensions/quaternion_extensions.h"
#include "cmsis_dsp_extensions/trig_extensions.h"

inline void getStateQuaternion(arm_matrix_instance_f32* x, arm_matrix_instance_f32* quaternion, float32_t* quaternionData);
inline void getStatePosition(arm_matrix_instance_f32* x, arm_matrix_instance_f32* position, float32_t* posData);
inline void getStateVelocity(arm_matrix_instance_f32* x, arm_matrix_instance_f32* vel, float32_t* velData);
inline void getStateGBias(arm_matrix_instance_f32* x, arm_matrix_instance_f32* gBias, float32_t* gData);
inline void getStateABias(arm_matrix_instance_f32* x, arm_matrix_instance_f32* aBias, float32_t* aData);
inline void getStateGSF(arm_matrix_instance_f32* x, arm_matrix_instance_f32* g_sf, float32_t* g_sf_data);
inline void getStateASF(arm_matrix_instance_f32* x, arm_matrix_instance_f32* a_sf, float32_t* a_sf_data);
void quaternion2DCM(const arm_matrix_instance_f32* quaternion, arm_matrix_instance_f32* CB2I, float32_t* CB2IBuffer);
void compute_radii(float32_t phi, float32_t* returnVector);
void compute_g_dg();

#endif
