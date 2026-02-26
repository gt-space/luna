#include "trig_extensions.h"

inline float32_t deg2rad(float32_t deg) {
	return deg * (M_PI / 180.0f);
}

inline float32_t rad2deg(float32_t rad) {
    return rad * (180.0f / M_PI);
}

inline float32_t arm_sind_f32(float32_t angleDeg) {
	return arm_sin_f32(deg2rad(angleDeg));
}

inline float32_t arm_cosd_f32(float32_t angleDeg) {
	return arm_cos_f32(deg2rad(angleDeg));
}

inline float32_t arm_tand_f32(float32_t angleDeg) {
	return tanf(deg2rad(angleDeg));
}