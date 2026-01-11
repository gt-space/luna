#include "ekf.h"

bool drougeChuteCheck(float32_t deltaAlt, uint32_t* altStart) {
	uint32_t now = HAL_GetTick();

	// deltaAlt is the altitude between the previous and current iteration
	if (deltaAlt < 0) {
		// Start the timer for three seconds when the timer hasn't started
		// and the altitude is decreasing 
		if (*altStart == UINT32_MAX) {
			*altStart = now;
		}
	} else {
		*altStart = UINT32_MAX;
	}

	// Says to launch the drouge if the timers have started (!= UINT32_MAX)
	// AND we have six seconds of negative down velocity
    return (*altStart != UINT32_MAX && (now - *altStart >= 6000));
}

bool mainChuteCheck(float32_t altNow, uint32_t* altStart) {
	uint32_t now = HAL_GetTick();

	// altNow = current altitude
	// 899.16 m = 2950 ft
	// FAR Altitude = 633 m
	// 3000 ft to meters is 914.4 m
	// Add both together to get when we need to deploy
	if (altNow <= 1547.4f) {
		if (*altStart == UINT32_MAX) {
			*altStart = now;
		}
	} else {
		*altStart = UINT32_MAX;
	}

	// Launch Main Chute IF:
	// Altitude is lower than 2950 ft AND
    return (*altStart != UINT32_MAX && (now - *altStart >= 1000));

}

/**
 * @brief Perform a single iteration of the Extended Kalman Filter (EKF) update.
 *
 * This function implements a full EKF update cycle for a INS/GPS
 * navigation system. It includes the following steps:
 * 1. Extracts the current state vector components (quaternion, position, velocity,
 *    gyroscope and accelerometer biases, scale factors) from the previous state.
 * 2. Computes the angular velocity (\f$\omega\hat{}\f$) and specific force (\f$\hat{\mathbf{a}}\f$)
 *    in the non-inertial body frame using IMU measurements and estimated biases.
 * 3. Propagates the state vector and covariance matrix forward in time using
 *    the process model and process noise covariance (Q).
 * 4. Conditionally updates the state and covariance with measurements from:
 *    - GPS (position)
 *    - Magnetometer (if available)
 *    - Barometer (pressure)
 * 5. Computes checks for deployment of drouge and main parachutes based on
 *    altitude and vertical velocity.
 * 6. Ensures the updated covariance matrix remains positive semi-definite by
 *    correcting negative eigenvalues if necessary.
 *
 * @param[in]  xPrev        	Previous state vector (21×1 or 22×1 including augmented states).
 * @param[in]  PPrev        	Previous covariance matrix (21×21).
 * @param[in]  Q            	Process noise covariance matrix (12×12).
 * @param[in]  H            	Measurement model matrix (variable size).
 * @param[in]  R            	Measurement noise covariance matrix (variable size).
 * @param[in]  Rq           	Measurement noise covariance for magnetometer (3×3).
 * @param[in]  Rb           	Barometer measurement variance (scalar).
 * @param[in]  aMeas        	Measured accelerations from the IMU (3×1).
 * @param[in]  wMeas        	Measured angular rates from the IMU (3×1).
 * @param[in]  llaMeas      	Measured GPS LLA position (3×1).
 * @param[in]  magMeas      	Measured magnetic field (3×1).
 * @param[in]  pressMeas    	Measured barometric pressure (Pa).
 * @param[in]  magI         	Magnetic reference in the inertial frame (3×1).
 * @param[in]  we           	Earth rotation rate (rad/s).
 * @param[in]  dt           	Time step for propagation (seconds).
 * @param[out] xPlus        	Updated state vector after propagation and measurement updates.
 * @param[out] Pplus        	Updated covariance matrix after propagation and measurement updates.
 * @param[out] xPlusBuff    	User-provided buffer backing @p xPlus.
 * @param[out] PPlusBuff    	User-provided buffer backing @p Pplus.
 * @param[in,out] vdStart       Pointer to vertical velocity trigger for drogue chute deployment.
 * @param[in,out] mainAltStart  Pointer to altitude trigger for main chute deployment.
 * @param[in,out] drougeAltStart Pointer to altitude trigger for drogue chute deployment.
 * @param[in]  fcData       	Flight controller data structure containing sensor validity flags.
 * @param[in,out] fallbackDR	Boolean flag indicating if we need to fallback to dead reckoning
 */
void update_EKF(arm_matrix_instance_f32* xPrev,
				arm_matrix_instance_f32* PPrev,
				arm_matrix_instance_f32* Q,
				arm_matrix_instance_f32* H,
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
				float32_t xPlusBuff[22*1],
				float32_t PPlusBuff[21*21],
				fc_message* fcData,
				bool* fallbackDR,
				uint32_t i) {

	// Define matrices that are components of the state vector and their
	// backing matrices
	arm_matrix_instance_f32 q, lla, vel, gBias, aBias, GSF, ASF, wHat, aHatN;
	float32_t qData[4], llaData[3], velData[3],
			  gBiasData[3], aBiasData[3], gSFData[3],
			  aSFData[3], wHatData[3], aHatNBuff[3];

	float32_t phi = xPrev->pData[4];
	float32_t h = xPrev->pData[6];
	float32_t vn = xPrev->pData[7];
	float32_t ve = xPrev->pData[8];

	// Fill the components of the state vector defined with the values from 
	// the current state
	getStateQuaternion(xPrev, &q, qData);
	getStatePosition(xPrev, &lla, llaData);
	getStateVelocity(xPrev, &vel, velData);
	getStateGBias(xPrev, &gBias, gBiasData);
	getStateABias(xPrev, &aBias, aBiasData);
	getStateGSF(xPrev, &GSF, gSFData);
	getStateASF(xPrev, &ASF, aSFData);

	// Computes the angular velocity using the gyro data from the IMU in the 
	// non-inertial body frame in rad/s
	compute_what(&q, &gBias, &GSF, phi, h, vn, ve, we, wMeas, &wHat, wHatData);

	// Computes the linear acceleration in the non-inertial body frame using acceleration
	// data from the IMU in m/s^2
	compute_ahat(&q, &ASF, &aBias, aMeas, &aHatN, aHatNBuff);

	propogate(xPrev, PPrev, &wHat, &aHatN, wMeas, aMeas,
			  Q, dt, we, xPlus, Pplus, xPlusBuff,
			  PPlusBuff);

	arm_matrix_instance_f32 xPlusGPS, PplusGPS;
	static float32_t xPlusGPSData[22*1], PplusGPSData[21*21];

	arm_matrix_instance_f32 xPlusMag, PplusMag;
	static float32_t xPlusMagData[22*1], PplusMagData[21*21];

	arm_matrix_instance_f32 xPlusBaro, PPlusBaro;
	static float32_t xPlusBaroData[22*1], PPlusBaroData[21*21];

	if (fcData->body.valid) {

		// fcData->body.valid
		fcData->body.valid = false;
		update_GPS(xPlus, Pplus, H, R, llaMeas,
				   &xPlusGPS, &PplusGPS, xPlusGPSData, PplusGPSData);

		memcpy(xPlusGPSData, xPlus->pData, sizeof(float32_t) * xPlus->numRows * xPlus->numCols);
		memcpy(PplusGPSData, Pplus->pData, sizeof(float32_t) * Pplus->numRows * Pplus->numCols);
	}

	if (false) {
		// atomic_load(&magEventCount)
		// will have to
		// atomic_fetch_sub(&magEventCount, 1);
		atomic_fetch_sub(&magEventCount, 1);
		update_mag(xPlus, Pplus, Rq,
				   magI, magMeas, &xPlusMag, &PplusMag,
				   xPlusMagData, PplusMagData);

		memcpy(xPlusMagData, xPlus->pData, sizeof(float32_t) * xPlus->numRows * xPlus->numCols);
		memcpy(PplusMagData, Pplus->pData, sizeof(float32_t) * Pplus->numRows * Pplus->numCols);
	}

	if (atomic_load(&baroEventCount)) {

		//printf("Pressure: %f Pa\n", pressMeas);
		atomic_fetch_sub(&baroEventCount, 1);
		update_baro(xPlus, Pplus, pressMeas, Rb,
					&xPlusBaro, &PPlusBaro, xPlusBaroData, PPlusBaroData);

		memcpy(xPlusBaroData, xPlus->pData, sizeof(float32_t) * xPlus->numRows * xPlus->numCols);
		memcpy(PPlusBaroData, Pplus->pData, sizeof(float32_t) * Pplus->numRows * Pplus->numCols);
	}

	//	printf("Current State Vector:\n");
	//	printMatrix(xPlus);

	//	printf("Delta X State Vector:\n");
	//	printf("[");
	//	for (int i = 0; i < 22; i++) {
	//        printf("%15.9e \n", xPlus->pData[i] - xPrev->pData[i]);
	//	}
	//	printf("]\n\n");
	//
	//	printf("Current Covariance Matrix:\n");
	//	printMatrix(Pplus);

	// Checks if the covariance matrix is a positive semi-definite matrix and if not calculates
	// calculates the nearest positive semi-definite matrix.
	for (uint8_t i = 0; i < Pplus->numRows; i++) {
		if (Pplus->pData[i * Pplus->numCols + i] < 0) {
			float32_t newPPlusData[21*21];
			arm_matrix_instance_f32 newPPlus;
			nearestPSD(Pplus, &newPPlus, newPPlusData);

			memcpy(Pplus->pData, newPPlusData, sizeof(float32_t) * Pplus->numRows * Pplus->numCols);
			break;
		}
	}

	for (uint8_t i = 0; i < Pplus->numRows; i++) {
		float32_t val = Pplus->pData[i * Pplus->numCols + i];
		if (val > 1e6f || isnan(val) || isinf(val)) {
			// Fall Back to dead reckoning
			break;
		}
	}

}

/* All Debug Parameters */
//	printf("Acceleration Measurement: [%f. %f, %f] m/s\n", aMeas->pData[0], aMeas->pData[1], aMeas->pData[2]);
//	printf("A Bias: [%f, %f, %f]\n", aBias.pData[0], aBias.pData[1], aBias.pData[2]);
//    printf("A Scale Factor: [%f, %f, %f]\n\n", ASF.pData[0], ASF.pData[1], ASF.pData[2]);
//    printf("Gyro Measurement: [%f. %f, %f] rad/s\n", wMeas->pData[0], wMeas->pData[1], wMeas->pData[1]);
//	printf("G Bias: [%f, %f, %f]\n", gBias.pData[0], gBias.pData[1], gBias.pData[2]);
//    printf("G Scale Factor: [%f, %f, %f]\n\n", GSF.pData[0], GSF.pData[1], GSF.pData[2]);

//	printf("Previous State Vector:\n");
//	printMatrix(xPrev);

//	printf("Previous Covariance Matrix:\n");
//	printMatrix(PPrev);

//	printMatrix(&gBias);
//	printMatrix(&aBias);
//	printMatrix(&GSF);
//	printMatrix(&ASF);


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
