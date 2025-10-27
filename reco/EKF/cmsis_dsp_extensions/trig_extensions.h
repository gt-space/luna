#ifndef _TRIG_EXTS
#define _TRIG_EXTS

#include "common.h"

float32_t deg2rad(float32_t deg);
float32_t rad2deg(float32_t rad);

float32_t arm_sind_f32(float32_t angleDeg);
float32_t arm_cosd_f32(float32_t angleDeg);
float32_t arm_tand_f32(float32_t angleDeg);

float32_t arm_cscd_f32(float32_t angleDeg);
float32_t arm_secd_f32(float32_t angleDeg);
float32_t arm_cotd_f32(float32_t angleDeg);

#endif
