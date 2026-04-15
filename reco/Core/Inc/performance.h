#ifndef _PERFORMANCE
#define _PERFORMANCE

#ifdef PERF_ANALYSIS

#include "stdbool.h"
#include "main.h"
#include "stdio.h"
#include "arm_math_types.h"

typedef enum {
    PERF_MAIN_LOOP,
    PERF_UPDATE_EKF,
    PERF_WHAT,
    PERF_WN,
    PERF_COMPUTE_G_DG2,
    PERF_QUAT2DCM,
    PERF_AHAT,
    PERF_PROPAGATE,
    PERF_QDOT,
    PERF_LLADOT,
    PERF_RADIUS,
    PERF_VDOT,
    PERF_PDOT,
    PERF_COMPUTE_F,
    PERF_COMPUTE_G,
	PERF_PDOT_CALC,
    PERF_COMPUTE_DWDP,
    PERF_COMPUTE_DWDV,
    PERF_COMPUTE_DPDOT_DP,
    PERF_COMPUTE_DPDOT_DV,
    PERF_COMPUTE_DVDOT_DP,
    PERF_COMPUTE_DVDOT_DV,
    PERF_INTEGRATE,
    PERF_GPS,
    PERF_LINSOLVE_GPS,
    PERF_MAG,
    PERF_LINSOLVE_MAG,
    PERF_BARO,
    PERF_NEAREST_PSD,
    PERF_EIG,
    PERF_P2ALT,
	PERF_GATHER_BARO,
	PERF_GATHER_MAG,
	PERF_GATHER_IMU,
	PERF_FILTER_DP_DH,
	PERF_KALMAN_COVARIANCE_UPDATE,
	PERF_ADD_21x21,
	PERF_MULTIPLY_21x21,
	PERF_21x21_MEMCPY,
    PERF_COUNT   // PERF_COUNT is equal to the number of functions 
} perf_index_t;

typedef struct {
    float32_t alpha;
    float32_t overhead;
    float32_t main_loop_time;
    float32_t ema[PERF_COUNT];
    uint32_t min[PERF_COUNT];
    uint32_t max[PERF_COUNT];
    uint32_t indexNum;
    bool initialized;
} perf_t;

void perf_update(perf_t* perf, perf_index_t idx, uint32_t input);
void perf_init(perf_t* perf);
void perf_print(const perf_t* perf);
void perf_main_loop_time(perf_t* perf, float32_t time);

#define ALPHA 0.2f // Coefficient used in EMA. Higher alpha = faster response, more sensitive to noise and vice versa
#define PERF_ARG , perf_t* perf_data // Used in function declarations
#define PERF_PASS , perf_data            // Actual paramter passed to functions
#define PERF_START(startNum) uint32_t start##startNum = DWT->CYCCNT // Reference to start counting cycles from
#define PERF_END(funcName, endNum) perf_update(perf_data, funcName, DWT->CYCCNT - start##endNum) // Each PERF_START has its own PERF_END

#else

#define ALPHA 
#define PERF_ARG
#define PERF_PASS 
#define PERF_START(startNum)
#define PERF_END(funcName, endNum)

#endif
#endif




