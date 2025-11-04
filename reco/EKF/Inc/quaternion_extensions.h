/*
 * quaternion_extensions.h
 *
 *  Created on: Oct 21, 2025
 *      Author: Raey Ayalew
 */

#ifndef _QUAT_EXTS
#define _QUAT_EXTS

#include "common.h"

void arm_quaternion_scalar_f32(arm_matrix_instance_f32* quaternion,
		arm_matrix_instance_f32 *scalarOut,
		float32_t* buffer);

void arm_quaternion_vector_f32(arm_matrix_instance_f32* quaternion,
		arm_matrix_instance_f32 *vectorOut,
		float32_t *buffer);


#endif /* _QUATERNION_EXT_H */
