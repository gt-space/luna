#include "ekf.h"

float32_t fading_memory_first_order(float32_t xHat, float32_t x_meas, float32_t G1) {
	float32_t y_k = x_meas - xHat;
	return G1 * y_k + xHat;
}

float32_t fading_memory_second_order(float32_t xHat,
									 float32_t xDotHat,
									 float32_t x_meas,
									 float32_t T_s,
									 float32_t G2,
									 float32_t HPrime,
									 float32_t* xPlus,
									 float32_t* xDotPlus) {

	float32_t x_minus = xDotHat * T_s + xHat;
	float32_t y_k = x_meas - x_minus;
	*xPlus = G2 * y_k + xHat;
	*xDotPlus = HPrime * y_k + xDotHat;
}
