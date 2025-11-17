#include "comms.h"

void assembleRECOMessage(reco_message* message, float32_t x[22], float32_t linAccel[3],
			       float32_t angularRate[3], float32_t magData[3], float32_t temp, float32_t press, uint32_t checksum) {

	memcpy(&message->body, x, 22*sizeof(float32_t));
	memcpy(&message->body.linAccel, linAccel, 3*sizeof(float32_t));
	memcpy(&message->body.angularRate, angularRate, 3*sizeof(float32_t));
	memcpy(&message->body.magData, magData, 3*sizeof(float32_t));

	message->body.temperature = temp;
	message->body.pressure = press;
	message->checksum = checksum;
}


