#include "comms.h"

void assembleRECOMessage(reco_message* message, float32_t x[22], float32_t linAccel[3], float32_t angularRate[3], float32_t magData[3], float32_t temp, float32_t press) {

	memcpy(&message->quaternion, x, 22*sizeof(float32_t));
	memcpy(&message->linAccel, linAccel, 3*sizeof(float32_t));
	memcpy(&message->angularRate, angularRate, 3*sizeof(float32_t));
	memcpy(&message->magData, magData, 3*sizeof(float32_t));

	message->temperature = temp;
	message->pressure = press;
}


