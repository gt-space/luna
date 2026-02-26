#ifndef _TRIG_EXTS
#define _TRIG_EXTS

#include "common.h"

inline float32_t deg2rad(float32_t deg);
inline float32_t rad2deg(float32_t rad);

inline float32_t arm_sind_f32(float32_t angleDeg);
inline float32_t arm_cosd_f32(float32_t angleDeg);
inline float32_t arm_tand_f32(float32_t angleDeg);

#endif