#include "ekf.h"
/**
 * @brief Determines whether to deploy the drogue chute based on descent rate.
 *
 * This function monitors the change in altitude (`deltaAlt`) to detect sustained
 * descent (negative altitude change). When descent is detected, a timer is started.
 * If the descent persists continuously for a specified duration (6 seconds),
 * the function returns true indicating that the drogue chute should be deployed.
 *
 * If ascent or no descent is detected (`deltaAlt >= 0`), the timer is reset.
 *
 * @param[in]  deltaAlt     Change in altitude between consecutive iterations (meters).
 *                         Negative values indicate descent.
 * @param[in,out] altStart  Pointer to the timestamp marking the start of descent.
 *                         Must be initialized to UINT32_MAX before first use.
 *                         Updated internally to track descent duration.
 * @param[in]  currentTime  Current system time in milliseconds.
 *
 * @return true  If continuous descent has been detected for at least 6000 ms (6 seconds).
 * @return false Otherwise.
 *
 * @note
 * - `altStart` acts as a state variable and must persist between function calls.
 * - UINT32_MAX is used as a sentinel value indicating that the timer is not active.
 * - The function assumes `currentTime` is monotonically increasing and does not handle wraparound.
 */
bool drougeChuteCheck(float32_t deltaAlt, uint32_t* altStart, uint32_t currentTime) {
	uint32_t now = currentTime;

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

/**
 * @brief Determines whether to deploy the main chute based on altitude threshold.
 *
 * This function checks if the current altitude (`altNow`) is below a predefined
 * deployment threshold (1547.4 meters). When the altitude falls below this threshold,
 * a timer is started. If the altitude remains below the threshold continuously
 * for at least 1 second (1000 ms), the function returns true indicating that
 * the main chute should be deployed.
 *
 * If the altitude rises above the threshold, the timer is reset.
 *
 * @param[in]  altNow       Current altitude in meters.
 * @param[in,out] altStart  Pointer to the timestamp marking when altitude first
 *                         dropped below the threshold. Must be initialized to
 *                         UINT32_MAX before first use. Updated internally.
 * @param[in]  currentTime  Current system time in milliseconds.
 *
 * @return true  If altitude has remained below 1547.4 meters for at least 1000 ms.
 * @return false Otherwise.
 *
 * @note
 * - The threshold (1547.4 m) corresponds to approximately 5075 ft.
 * - `altStart` acts as a persistent state variable across function calls.
 * - UINT32_MAX is used as a sentinel value indicating that the timer is inactive.
 * - The function assumes `currentTime` is monotonically increasing and does not handle wraparound.
 */
bool mainChuteCheck(float32_t altNow, uint32_t* altStart, uint32_t currentTime) {
	uint32_t now = currentTime;

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
				fc_message_t* fcData,
				bool* fallbackDR,
				uint32_t i
				PERF_ARG) {

	PERF_START(1);

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
	compute_what(&q, &gBias, &GSF, phi, h, vn, ve, we, wMeas, &wHat, wHatData PERF_PASS);

	// Computes the linear acceleration in the non-inertial body frame using acceleration
	// data from the IMU in m/s^2
	compute_ahat(&q, &ASF, &aBias, aMeas, &aHatN, aHatNBuff PERF_PASS);

	propogate(xPrev, PPrev, &wHat, &aHatN, wMeas, aMeas,
			  Q, dt, we, xPlus, Pplus, xPlusBuff,
			  PPlusBuff PERF_PASS);

	static arm_matrix_instance_f32 xPlusGPS, PplusGPS;
	static float32_t xPlusGPSData[22*1], PplusGPSData[21*21];

	static arm_matrix_instance_f32 xPlusMag, PplusMag;
	static float32_t xPlusMagData[22*1], PplusMagData[21*21];

	static arm_matrix_instance_f32 xPlusBaro, PPlusBaro;
	static float32_t xPlusBaroData[22*1], PPlusBaroData[21*21];

	bool new_baro_measurement;
	bool new_gps_measurement;

	#ifdef PERF_ANALYSIS
	new_baro_measurement = true;
	new_gps_measurement = true;
	#else
	new_baro_measurement = atomic_load(&baroEventCount);
	new_gps_measurement = fcData->valid;
	#endif

	if (new_baro_measurement) {

		// fcData->body.valid
		fcData->valid = false;
		update_GPS(xPlus, Pplus, H, R, llaMeas,
				   &xPlusGPS, &PplusGPS, xPlusGPSData, PplusGPSData PERF_PASS);

		copy_mat_f32(xPlus, &xPlusGPS);
		copy_mat_f32(Pplus, &PplusGPS);

	}

	if (false) {
		// atomic_load(&magEventCount)
		// will have to
		// atomic_fetch_sub(&magEventCount, 1);
		atomic_fetch_sub(&magEventCount, 1);
		update_mag(xPlus, Pplus, Rq,
				   magI, magMeas, &xPlusMag, &PplusMag,
				   xPlusMagData, PplusMagData PERF_PASS);

		copy_mat_f32(xPlus, &xPlusMag);
		copy_mat_f32(Pplus, &PplusMag);
	}

	if (new_gps_measurement) {

		// atomic_load(&baroEventCount)
		//printf("Pressure: %f Pa\n", pressMeas);
		atomic_fetch_sub(&baroEventCount, 1);
		update_baro_new(xPlus, Pplus, pressMeas, Rb,
						&xPlusBaro, &PPlusBaro, xPlusBaroData, PPlusBaroData PERF_PASS);

		copy_mat_f32(xPlus, &xPlusBaro);
		copy_mat_f32(Pplus, &PPlusBaro);
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
			nearestPSD(Pplus, &newPPlus, newPPlusData PERF_PASS);

			memcpy(Pplus->pData, newPPlusData, sizeof(float32_t) * Pplus->numRows * Pplus->numCols);
			break;
		}
	}

	for (uint8_t i = 0; i < Pplus->numRows; i++) {
		float32_t val = Pplus->pData[i * Pplus->numCols + i];
		if (val > 1e6f || isnan(val) || isinf(val)) {
			// Fall Back to dead reckoning
			*fallbackDR = true;
			break;
		}
	}

	PERF_END(PERF_UPDATE_EKF, 1);
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
