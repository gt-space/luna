#include "ekf.h"

bool drougeChuteCheck(float32_t vdNow, float32_t deltaAlt, uint32_t* vdStart, uint32_t* altStart) {
	uint32_t now = HAL_GetTick();

	if (vdNow > 0) {
		if (*vdStart == UINT32_MAX) {
			*vdStart = now;
		}
	} else {
		*vdStart = UINT32_MAX;
	}

	if (deltaAlt < 0) {
		if (*altStart == UINT32_MAX) {
			*altStart = now;
		}
	} else {
		*altStart = UINT32_MAX;
	}

    return (*vdStart != UINT32_MAX &&
            *altStart != UINT32_MAX &&
            (now - *vdStart >= 3000) &&
            (now - *altStart >= 3000));
}

bool mainChuteCheck(float32_t vdNow, float32_t altNow, uint32_t* altStart) {
	uint32_t now = HAL_GetTick();

	if (vdNow < 0) {
		*altStart = UINT32_MAX;
	}

	if (altNow <= 304.8f) {
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
				arm_matrix_instance_f32* Q,
				arm_matrix_instance_f32* H,
				arm_matrix_instance_f32* R,
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
				float32_t xPlusBuff[22*1],
				float32_t PPlusBuff[21*21],
				uint32_t* vdStart,
				uint32_t* mainAltStart,
				uint32_t* drougeAltStart,
				reco_message* message,
				fc_message* fcData) {

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

	propogate(xPrev, PPrev, &wHat, &aHatN, wMeas, aMeas,
			  Q, dt, we, xPlus, Pplus, xPlusBuff,
			  PPlusBuff);

	arm_matrix_instance_f32 xPlusGPS, PplusGPS;
	float32_t xPlusGPSData[22*1], PplusGPSData[21*21];

	arm_matrix_instance_f32 xPlusMag, PplusMag;
	float32_t xPlusMagData[22*1], PplusMagData[21*21];

	arm_matrix_instance_f32 xPlusBaro, PPlusBaro;
	float32_t xPlusBaroData[22*1], PPlusBaroData[21*21];

	if (fcData->body.valid) {

		fcData->body.valid--;
		update_GPS(xPlus, Pplus, H, R, llaMeas,
				   &xPlusGPS, &PplusGPS, xPlusGPSData, PplusGPSData);

		copyMatrix(xPlusGPSData, xPlus->pData, xPlus->numRows * xPlus->numCols);
		copyMatrix(PplusGPSData, Pplus->pData, Pplus->numRows * Pplus->numCols);
	}

	if (false) {
		// atomic_load(&magEventCount)
		// will have to
		// atomic_fetch_sub(&magEventCount, 1);
		update_mag(xPlus, Pplus, R,
				   magI, magMeas, &xPlusMag, &PplusMag,
				   xPlusMagData, PplusMagData);

		copyMatrix(xPlusMagData, xPlus->pData, xPlus->numRows * xPlus->numCols);
		copyMatrix(PplusMagData, Pplus->pData, Pplus->numRows * Pplus->numCols);
	}

	if (atomic_load(&baroEventCount)) {

		atomic_fetch_sub(&baroEventCount, 1);
		update_baro(xPlus, Pplus, pressMeas, Rb,
					&xPlusBaro, &PPlusBaro, xPlusBaroData, PPlusBaroData);

		copyMatrix(xPlusBaroData, xPlus->pData, xPlus->numRows * xPlus->numCols);
		copyMatrix(PPlusBaroData, Pplus->pData, Pplus->numRows * Pplus->numCols);
	}


	printf("Previous State Vector:\n");
	printMatrix(xPrev);

	printf("Current State Vector:\n");
	printMatrix(xPlus);

	printf("Delta X State Vector:\n");
	printf("[");
	for (int i = 0; i < 22; i++) {
        printf("%15.9e \n", xPlus->pData[i] - xPrev->pData[i]);
	}
	printf("]\n\n");

	float32_t currAltitude = xPlus->pData[6];
	float32_t prevAltitude = xPrev->pData[6];
	float32_t deltaAlt = currAltitude - prevAltitude;
	float32_t downVel = xPlus->pData[9];

	if (drougeChuteCheck(downVel, deltaAlt, vdStart, drougeAltStart)) {
		message->stage1En = true;
		HAL_GPIO_WritePin(STAGE1_EN_GPIO_Port, STAGE1_EN_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(STAGE1_EN_GPIO_Port, STAGE1_EN_Pin, GPIO_PIN_RESET);

	}

	if (mainChuteCheck(downVel, currAltitude, mainAltStart)) {
		message->stage2En = true;
		HAL_GPIO_WritePin(STAGE2_EN_GPIO_Port, STAGE2_EN_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(STAGE2_EN_GPIO_Port, STAGE2_EN_Pin, GPIO_PIN_RESET);
	}

	arm_matrix_instance_f32 PplusDiag;
	float32_t PplusDiagBuff[Pplus->numRows];
	arm_mat_extract_diag(Pplus, &PplusDiag, PplusDiagBuff);

	//printf("Diag Mat\n");
	//printMatrix(&PplusDiag);

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


//if (fcData->body.valid) {
//
//	fcData->body.valid--;
//	update_GPS(xPlus, Pplus, H, R, llaMeas,
//			   &xPlusGPS, &PplusGPS, xPlusGPSData, PplusGPSData);
//
//	copyMatrix(xPlusGPSData, xPlus->pData, xPlus->numRows * xPlus->numCols);
//	copyMatrix(PplusGPSData, Pplus->pData, Pplus->numRows * Pplus->numCols);
//}
//
//if (false) {
//	// atomic_load(&magEventCount)
//	// will have to
//	// atomic_fetch_sub(&magEventCount, 1);
//	update_mag(xPlus, Pplus, R,
//			   magI, magMeas, &xPlusMag, &PplusMag,
//			   xPlusMagData, PplusMagData);
//
//	copyMatrix(xPlusMagData, xPlus->pData, xPlus->numRows * xPlus->numCols);
//	copyMatrix(PplusMagData, Pplus->pData, Pplus->numRows * Pplus->numCols);
//}
//
//if (atomic_load(&baroEventCount)) {
//
//	atomic_fetch_sub(&baroEventCount, 1);
//	update_baro(xPlus, Pplus, pressMeas, Rb,
//				&xPlusBaro, &PPlusBaro, xPlusBaroData, PPlusBaroData);
//
//	copyMatrix(xPlusBaroData, xPlus->pData, xPlus->numRows * xPlus->numCols);
//	copyMatrix(PPlusBaroData, Pplus->pData, Pplus->numRows * Pplus->numCols);
//}
