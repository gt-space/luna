#ifndef COMMS_H_
#define COMMS_H_

#include "stdbool.h"
#include "arm_math_types.h"
#include "stm32h5xx_hal.h"

typedef enum {
	LAUNCH = 0x01,
	DATA   = 0x02,
} COMMS_OPCODE_T;

typedef struct __attribute__((packed)) {
	float32_t velocity_north;
	float32_t velocity_east;
	float32_t velocity_down;
	float32_t latitude;
	float32_t longitude;
	float32_t altitude;
	bool valid;
} fc_body;

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
	float32_t temperature;
	float32_t pressure;
} reco_body;

typedef struct __attribute__((packed)) {
	uint8_t opcode;
	fc_body body;
	uint32_t checksum;
	uint8_t padding[106];
} fc_message;

typedef struct reco_message {
	reco_body body;
	uint32_t checksum;
} reco_message;

void assembleRECOMessage(reco_message* message, float32_t x[22], float32_t linAccel[3],
				   float32_t angularRate[3], float32_t magData[3], float32_t temp, float32_t press, uint32_t checksum);


#endif
