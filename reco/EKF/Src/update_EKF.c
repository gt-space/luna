#include "Inc/ekf.h"


bool drougeChuteCheck(float32_t vdNow, float32_t altNow, float32_t* vdStart, float32_t* altStart) {
	uint32_t now = HAL_GetTick();

	if (vdNow > 0) {
		if (*vdStart == UINT32_MAX) {
			*vdStart = now;
		}
	} else {
		*vdStart = UINT32_MAX;
	}

	if (altNow < 0) {
		if (*altStart == UINT32_MAX) {
			*altStart = now;
		}
	} else {
		*altStart = UINT32_MAX;
	}

    return (*vdStart != UINT32_MAX &&
            *altStart != UINT32_MAX &&
            (now - *vdStart >= 1000) &&
            (now - *altStart >= 1000));
}

bool mainChuteCheck(float32_t vdNow, float32_t altNow, float32_t* altStart) {
	uint32_t now = HAL_GetTick();

	if (vdNow < 0) {
		*altStart = UINT32_MAX;
	}

	if (altNow <= 1000.0f) {
		if (*altStart == UINT32_MAX) {
			*altStart = now;
		}
	} else {
		*altStart = UINT32_MAX;
	}

    return (*altStart != UINT32_MAX &&
            (vdNow > 0) &&
            (now - *altStart >= 250));

}

void update_EKF(arm_matrix_instance_f32* xPrev,
				arm_matrix_instance_f32* PPrev,
				arm_matrix_instance_f32* PqPrev,
				arm_matrix_instance_f32* Q,
				arm_matrix_instance_f32* Qq,
				arm_matrix_instance_f32* H,
				arm_matrix_instance_f32* Hq,
				arm_matrix_instance_f32* R,
				arm_matrix_instance_f32* Rq,
				float32_t Rb,
				arm_matrix_instance_f32* aMeas,
				arm_matrix_instance_f32* wMeas,
				arm_matrix_instance_f32* llaMeas,
				arm_matrix_instance_f32* magMeas,
				float32_t pressMeas,
				arm_matrix_instance_f32* magI,
				float32_t we,
				float32_t dt,
				arm_matrix_instance_f32* xPlus,
				arm_matrix_instance_f32* Pplus,
				arm_matrix_instance_f32* PqPlus,
				float32_t xPlusBuff[22*1],
				float32_t PPlusBuff[21*21],
				float32_t PqPlusBuff[6*6],
				float32_t* vdStart,
				float32_t* mainAltStart,
				float32_t* drougeAltStart,
				reco_message* message) {

	arm_matrix_instance_f32 q, lla, vel, gBias, aBias, GSF, ASF, wHat, aHatN;
	float32_t qData[4], llaData[3], velData[3],
			  gBiasData[3], aBiasData[3], gSFData[3],
			  aSFData[3], wHatData[3], aHatNBuff[3];

	float32_t phi = xPrev->pData[4];
	float32_t h = xPrev->pData[6];
	float32_t vn = xPrev->pData[7];
	float32_t ve = xPrev->pData[8];

	getStateQuaternion(xPrev, &q, qData);
	getStatePosition(xPrev, &lla, llaData);
	getStateVelocity(xPrev, &vel, velData);
	getStateGBias(xPrev, &gBias, gBiasData);
	getStateABias(xPrev, &aBias, aBiasData);
	getStateGSF(xPrev, &GSF, gSFData);
	getStateASF(xPrev, &ASF, aSFData);

	compute_what(&q, &gBias, &GSF, phi, h, vn, ve, we, wMeas, &wHat, wHatData);
	compute_ahat(&q, &ASF, &aBias, aMeas, &aHatN, aHatNBuff);

	propogate(xPrev, PPrev, PqPrev, &wHat, &aHatN, wMeas, aMeas,
			  Q, Qq, dt, we, xPlus, Pplus, PqPlus, xPlusBuff,
			  PPlusBuff, PqPlusBuff);

	arm_matrix_instance_f32 xPlusGPS, PplusGPS;
	float32_t xPlusGPSData[22*1], PplusGPSData[21*21];

	arm_matrix_instance_f32 xPlusMag, PplusMag, PqPlusMag;
	float32_t xPlusMagData[22*1], PplusMagData[21*21], PqPlusMagData[6*6];

	arm_matrix_instance_f32 xPlusBaro, PPlusBaro;
	float32_t xPlusBaroData[22*1], PPlusBaroData[21*21];

	if (atomic_load(&gpsEventCount)) {

		atomic_fetch_sub(&gpsEventCount, 1);
		update_GPS(xPlus, Pplus, H, R, llaMeas,
				   &xPlusGPS, &PplusGPS, xPlusGPSData, PplusGPSData);

		copyMatrix(xPlusGPSData, xPlus->pData, xPlus->numRows * xPlus->numCols);
		copyMatrix(PplusGPSData, Pplus->pData, Pplus->numRows * Pplus->numCols);
	}

	if (atomic_load(&magEventCount)) {
		// will have to
		atomic_fetch_sub(&magEventCount, 1);
		update_mag(xPlus, Pplus, PqPlus, Hq, Rq, R,
				   magI, magMeas, &xPlusMag, &PplusMag, &PqPlusMag,
				   xPlusMagData, PplusMagData, PqPlusMagData);

		copyMatrix(xPlusMagData, xPlus->pData, xPlus->numRows * xPlus->numCols);
		copyMatrix(PqPlusMagData, PqPlus->pData, PqPlus->numRows * PqPlus->numCols);
		copyMatrix(PplusMagData, Pplus->pData, Pplus->numRows * Pplus->numCols);
	}

	if (atomic_load(&baroEventCount)) {

		atomic_fetch_sub(&baroEventCount, 1);
		update_baro(xPlus, Pplus, pressMeas, Rb,
					&xPlusBaro, &PPlusBaro, xPlusBaroData, PPlusBaroData);

		copyMatrix(xPlusBaroData, xPlus->pData, xPlus->numRows * xPlus->numCols);
		copyMatrix(PPlusBaroData, Pplus->pData, Pplus->numRows * Pplus->numCols);
	}

	printf("[");
	for (int i = 0; i < 22; i++) {
        printf("%15.9e \n", xPlus->pData[i] - xPrev->pData[i]);
	}
	printf("]\n\n");

	float32_t currAltitude = xPlus->pData[6];
	float32_t prevAltitude = xPrev->pData[6];

	if (drougeChuteCheck(xPlus->pData[9], prevAltitude - currAltitude, vdStart, drougeAltStart)) {
		message->stage1En = true;
		HAL_GPIO_WritePin(STAGE1_EN_GPIO_Port, STAGE1_EN_Pin, GPIO_PIN_SET);

	}

	if (mainChuteCheck(xPlus->pData[9], currAltitude, mainAltStart)) {
		message->stage2En = true;
		HAL_GPIO_WritePin(STAGE2_EN_GPIO_Port, STAGE2_EN_Pin, GPIO_PIN_SET);
	}

	arm_matrix_instance_f32 PplusDiag;
	float32_t PplusDiagBuff[Pplus->numRows];
	arm_mat_extract_diag(Pplus, &PplusDiag, PplusDiagBuff);

	printf("Diag Mat\n");
	printMatrix(&PplusDiag);

	for (uint8_t i = 0; i < Pplus->numRows; i++) {
		if (PplusDiag.pData[i] < 0) {

			// printMatrix(&PplusDiag);
			// printf("Negative Trace at Location %d\n", i);
			float32_t newPPlusData[21*21];
			arm_matrix_instance_f32 newPPlus;
			nearestPSD(Pplus, &newPPlus, newPPlusData);

			copyMatrix(newPPlusData, Pplus->pData, Pplus->numRows * Pplus->numCols);
			break;
		}
	}

}
