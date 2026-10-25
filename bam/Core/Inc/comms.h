#ifndef COMMS_H_
#define COMMS_H_

#include "stdbool.h"
#include "assert.h"
#include "arm_math_types.h"
#include "stm32h5xx_hal.h"

typedef enum {
	DATA   			= 0xF2,
	RESET_STM  		= 0xCA,
	PROCESS 		= 0x51,
	MEASUREMENT 	= 0x52,
	STATE_VECTOR 	= 0x78,
	COVARIANCE 		= 0x50,
	TIMER			= 0x54,
	ALTIMETER		= 0x42,
	LAUNCH 			= 0x79,
} COMMS_OPCODE_T;

// 192 bytes
typedef struct __attribute__((packed)) {
		float32_t quaternion[4]; // attitude of vehicle
		float32_t llaPos[3]; // position of vehicle in long, lat, and altitude frame
		float32_t velocity[3]; // velocity of vehicle
		float32_t gBias[3]; // gyroscope bias offset
		float32_t aBias[3]; // acceleromater bias offset
		float32_t gSF[3]; // gyro scale factor
		float32_t aSF[3]; // acceleration scale factor
		float32_t linAccel[3]; // XYZ Acceleration
		float32_t angularRate[3]; // Angular Rates (pitch, yaw, roll)
		float32_t magData[3]; // XYZ Magnetometer Data
		float32_t temperature; // Temperature from barometer
		float32_t pressure;    // Pressure from barometer
		float32_t vref_ch1_dr1;  // Channel 1 Driver 1 Voltage (VREF-FB1-A)
		float32_t vref_ch1_dr2;  // Channel 1 Driver 2 Voltage (VREF-FB1-B)
		float32_t vref_ch2_dr1;  // Channel 2 Driver 1 Voltage (VREF-FB2-A)
		float32_t vref_ch2_dr2;  // Channel 2 Driver 2 Voltage (VREF-FB2-B)
		float32_t sns1_current; // Recovery Driver 1 current
		float32_t sns2_current; // Recovery Driver 2 current
		float32_t v_rail_24v; // 24 V Rail Voltage
		float32_t v_rail_3v3; // 3.3 V Rail Voltage
		float32_t fading_memory_baro;
		float32_t fading_memory_gps;
		uint8_t stage1En;        // Pulled high when STM32 says to deploy drouge
		uint8_t stage2En;        // Pulled high when STM32 says to deploy main
		uint8_t received;        // Tells FC whether launch message was received
		uint8_t reco_driver_faults[10];  // Tells which of the 10 channels has faulte.
		uint8_t blewUp;          // Whether EKF has blown up or not
		uint8_t drougeTimerEnable; // When high, timer will be used over EKF for drouge
		uint8_t mainTimerEnable;   // When high, timer will be used over altimeter for main
		uint8_t rbf_enabled;	 // When high, RBF is installed and vice versa
		uint8_t padding[3];      // Random shit to keep this struct divisble by 4
} reco_message_t;

typedef struct {
	float32_t quaternion[4]; // attitude of vehicle
	float32_t llaPos[3]; // position of vehicle in long, lat, and altitude frame
	float32_t velocity[3]; // velocity of vehicle
	float32_t gBias[3]; // gyroscope bias offset
	float32_t aBias[3]; // acceleromater bias offset
	float32_t gSF[3]; // gyro scale factor
	float32_t aSF[3]; // acceleration scale factor
} state_vector_t;

typedef struct __attribute__((packed)) {
	uint8_t opcode;
	uint8_t padding1[3];
	state_vector_t state_vector;
	uint8_t padding2[100];
} state_vector_message_t;

typedef struct __attribute__((packed)) {
	uint8_t opcode;
	uint8_t padding1[3];
	float32_t nu_gv_mat[9];
	float32_t nu_gu_mat[9];
	float32_t nu_av_mat[9];
	float32_t nu_au_mat[9];
	uint8_t padding2[44];
} process_noise_t;

typedef struct __attribute__((packed)) {
	uint8_t opcode;
	uint8_t padding1[3];
	float32_t gpsNoiseMatrix[9];
	float32_t barometer_noise;
	uint8_t padding2[148];
} measurement_noise_t;

typedef struct __attribute__((packed)) {
	uint8_t opcode;
	uint8_t padding1[3];
	float32_t att_unc0[3];
	float32_t pos_unc0[3];
	float32_t vel_unc0[3];
	float32_t gbias_unc0[3];
	float32_t abias_unc0[3];
	float32_t gsf_unc0[3];
	float32_t asf_unc0[3];
	uint8_t padding2[104];
} initial_covariance_t;

typedef struct __attribute__((packed)) {
	uint8_t opcode;
	uint8_t padding1[3];
	float32_t drougeTimer;
	float32_t mainTimer;
	uint8_t   drougeTimerEnable;
	uint8_t   mainTimerEnable;
	uint8_t padding2[178];
} timer_values_t;

typedef struct __attribute__((packed)) {
	uint8_t opcode;
	uint8_t padding1[3];
	uint32_t ekf_lockout;
	float32_t hOffsetAlt;
	float32_t hOffsetFilter;
	float32_t flight_baro_fmf_parameter;
	float32_t ground_baro_fmf_parameter;
	float32_t flight_gps_fmf_parameter;
	float32_t ground_gps_fmf_parameter;
	uint8_t padding2[160];
} altimeter_offsets_t;


// 25 bytes of actual data
typedef struct __attribute__((packed)) {
	uint8_t opcode;
	uint8_t padding1[3];
	float32_t gpsVel[3];
	float32_t gpsLLA[3];
	bool valid;
	uint8_t padding2[163];
} fc_message_t;


/*
_Static_assert(sizeof(state_vector_message_t) == 180, "Bad packet size");
_Static_assert(sizeof(process_noise_t) == 180, "Bad packet size");
_Static_assert(sizeof(timer_values_t) == 180, "Bad packet size");
*/

#endif
