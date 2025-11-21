#include "Inc/ekf.h"

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

		xPlus->pData = xPlusGPSData;
		Pplus->pData = PplusGPSData;
	}

	if (atomic_load(&magEventCount)) {
		// will have to
		atomic_fetch_sub(&magEventCount, 1);
		update_mag(xPlus, Pplus, PqPlus, Hq, Rq, R,
				   magI, magMeas, &xPlusMag, &PplusMag, &PqPlusMag,
				   xPlusMagData, PplusMagData, PqPlusMagData);

		xPlus->pData = xPlusMagData;
		Pplus->pData = PplusGPSData;
		PqPlus->pData = PqPlusMagData;
	}

	if (atomic_load(&baroEventCount)) {

		atomic_fetch_sub(&baroEventCount, 1);
		update_baro(xPlus, Pplus, pressMeas, Rb,
					&xPlusBaro, &PPlusBaro, xPlusBaroData, PPlusBaroData);

		xPlus->pData = xPlusBaroData;
		Pplus->pData = PPlusBaroData;
	}

	for (uint8_t i = 0; i < Pplus->numRows; i++) {
		if (Pplus->pData[i*i] < 0) {
			float32_t newPPlusData[21*21];
			arm_matrix_instance_f32 newPPlus;
			nearestPSD(Pplus, Pplus, newPPlusData);
		}
	}

}
