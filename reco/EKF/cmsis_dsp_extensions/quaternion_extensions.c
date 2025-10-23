#include "cmsis_dsp_extensions/quaternion_extensions.h"

void arm_quaternion_scalar_f32(arm_matrix_instance_f32* quaternion, arm_matrix_instance_f32 *scalarOut, float32_t* buffer)
{
	*buffer = quaternion->pData[0];
    arm_mat_init_f32(scalarOut, 1, 1, buffer);
}

void arm_quaternion_vector_f32(arm_matrix_instance_f32* quaternion, arm_matrix_instance_f32 *vectorOut, float32_t *buffer)
{
	buffer[0] = quaternion->pData[1];
	buffer[1] = quaternion->pData[2];
	buffer[2] = quaternion->pData[3];
    arm_mat_init_f32(vectorOut, 3, 1, buffer);
}
