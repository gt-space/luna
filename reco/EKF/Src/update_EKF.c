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
				float32_t PqPlusBuff[6*6]) {

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

	// --- START: Covariance Check---
    // check P_plus for Inf/NaN
    // "0 if none, 1 if it blows up"
    *covariance_blew_up = false; // Default to 0 (false)
    for (int i = 0; i < (21*21); i++) {
        if (isnan(PPlusBuff[i]) || isinf(PPlusBuff[i])) {
            *covariance_blew_up = true; // Set flag to 1 (true)
            break; // Found an error
        }
    }

	if (*covariance_blew_up) {
        return; // Exit the function early
    }
    // --- END: Covariance Check ---
	if (gpsReady) {
		update_GPS(xPlus, Pplus, PqPlus, H, R,
				   lla_meas, xPlus, Pplus, xPlus->pData, Pplus->pData);
	}

	if (magReady) {
		// will have to
		update_mag(xPlus, Pplus, PqPlus, Hq, Rq, R,
				   magI, magMeas, xPlus, Pplus, PqPlus,
				   xPlus->pData, Pplus->pData, PqPlus->pData);
	}

	if (baroReady) {
		update_baro(xPlus, Pplus, pressMeas, Rb, xPlus, Pplus, xPlus->pData, Pplus->pData);
	}

	for (int currentDiagNum = 0; i < Pplus->numRows; i++) {
		float32_t newPPlusData[21*21];
		arm_matrix_instance_f32 newPPlus;
		nearestPSD(Pplus, Pplus, newPPlusData);
	}
}
