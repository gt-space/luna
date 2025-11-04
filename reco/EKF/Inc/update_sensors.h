#ifndef _UPDATE_SENS
#define _UPDATE_SENS

#include "common.h"
#include "ekf_utils.h"
#include "matrix_extensions.h"
#include "quaternion_extensions.h"
#include "trig_extensions.h"

void update_GPS(float32_t *x_plus, float32_t *P_plus, float32_t *Pq_plus, float32_t *x_minus, float32_t *P_minus, float32_t *Pq_minus, float32_t *H, float32_t *R, float32_t *lla_meas);

#endif
