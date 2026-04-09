#include "ekf.h"

static float32_t baroGroundInitBeta = 1.0f;
static float32_t baroFlightInitBeta = 1.0f;
static float32_t gpsGroundInitBeta = 1.0f;

float32_t get_initial_baro_ground_beta(void) {
	return baroGroundInitBeta;
}

float32_t get_initial_baro_flight_beta(void) {
	return baroFlightInitBeta;
}

float32_t get_initial_gps_ground_beta(void) {
	return gpsGroundInitBeta;
}

void set_initial_gains(float32_t newBaroGroundBeta, float32_t newBaroFlightBeta, float32_t newGPSGroundBeta) {
	baroGroundInitBeta = newBaroGroundBeta;
	baroFlightInitBeta = newBaroFlightBeta;
	gpsGroundInitBeta = newGPSGroundBeta;
	return;
}

/**
 * @brief Initialize a first-order fading memory filter (FMF).
 *
 * This function initializes the internal state and gain for a first-order
 * fading memory filter. Can be used to estimate many different states however only
 * used for GPS and barometer altitude on the ground. B
 *
 * @param[out] filterParams Pointer to the filter parameter structure to initialize
 * @param[in] initialState Initial state estimate \f$\hat{x}_0\f$ from x0 in compute_initial_consts.c. Can be set by operators.
 * @param[in] beta Fading memory parameter \f$\beta\f$, where \f$0 < \beta < 1\f$. Can be set by operators.
 *
 *
 * @note
 * - Larger \f$\beta\f$ results in slower response and more smoothing.
 * - Smaller \f$\beta\f$ results in faster response and less smoothing.
 * */
void fmf_first_order_init(fmf_first_order_t* filterParams, float32_t initialState, float32_t beta) {
	filterParams->currentStateEst = initialState; // Initial State
	filterParams->gain = 1 - beta;				  // Gain of FMF
	return;
}


/**
 * @brief Initialize a first-order fading memory filter (FMF).
 *
 * This function initializes the internal state and gain for a second-order
 * fading memory filter. While this can be used for many different states, it
 * is only used for barometer in flight
 *
 * @param[out] filterParams Pointer to the filter parameter structure to initialize.
 * @param[in]  initialState Initial estimate of the state variable
 * @param[in]  beta         Fading memory factor (0 < beta < 1), controls how quickly past data is forgotten.
 * @param[in]  dt           Timestep / sampling interval in seconds
 *
 * @note
 * - Larger \f$\beta\f$ results in slower response and more smoothing.
 * - Smaller \f$\beta\f$ results in faster response and less smoothing.
 * - The initial state should be derived in from the initial state of the EKF found as x0 in compute_inital_consts.c
 * - The beta is set by operators via the getter and setter functions found in fading_memory.c. These get called
 * 	 in the HAL_SPI_TxRx_Callback function in main.c
 * - The dt is set in main.c
 *
 */
void fmf_second_order_init(fmf_second_order_t* filterParams, float32_t initialState, float32_t beta, float32_t dt) {
	filterParams->currentStateEst = initialState;  // Initial State
	filterParams->currentDerivativeEst = 0;		   // Initial Velocity of Vehicle is 0 m/s
	filterParams->timestep = dt;				   //
	filterParams->gain = 1 - powf(beta, 2);		   // G = 1 - B^2
	filterParams->HPrime = powf(1 - beta, 2) / dt; // H = (1 - B)^2 / dt
	return;
}

/**
 * @brief Perform one update step of a first-order fading memory filter.
 *
 * This function updates the state estimate using a first-order fading memory filter.
 * The system assumes a constant state in the absence of perturbations.
 *
 * @param[in] filterParams Pointer to the filter parameter structure
 * @param[in] x_meas Measured state value
 *
 * @return Updated state estimate \f$\hat{x}_{k+1}\f$
 *
 * @note The gain must be initialized as \f$G = (1 - \beta)\f$
 * @note The state can be any output from any of the sensors. On RECO, we use this
 * 		 to determine our biases in our altitude state derived from our barometer
 * 		 and GPS
 */
float32_t fmf_first_order(fmf_first_order_t* filterParams, float32_t x_meas) {

    // a-priori step: do nothing because we assume the solution is constant in time in the
	// absence of perturbations; x^-_k+1 = x^+_k

	// a-posteriori step: assume the system is influenced by first order perturbations,
	// so compute the residual y_k = x_meas - x̂_k

    // multiply the residual by G and add to x^-_k+1 to get x^+_k+1

	float32_t xHat = filterParams->currentStateEst;
	float32_t gain = filterParams->gain;

	float32_t y_k = x_meas - xHat;
	return gain * y_k + xHat;
}

/**
 * @brief Perform one update step of a second-order fading memory filter.
 *
 * This function updates both the state estimate and its derivative using
 * a second-order fading memory filter. The system assumes linear evolution
 * of the state between time steps.
 *
 * @param[in,out] filterParams Pointer to the filter parameter structure.
 *                            Updated in-place with new state and derivative estimates.
 * @param[in] x_meas Measured state value
 *
 * @return Updated state estimate \f$\hat{x}_{k+1}\f$
 *
 * @note Gains must be initialized as:
 * - \f$G = 1 - \beta^2\f$
 * - \f$H = (1 - \beta)^2\f$
 * - \f$H' = \frac{H}{T_s}\f$
 *
 * @note Please see \ref ekf_init() and \ref reset_filter() for how the fading memory
 * 		 filter is initialized
 *
 * @note The state can be any output from any of the sensors. On RECO, we use this
 * 		 to determine our biases in our altitude state derived from our barometer
 * 		 and GPS
 *
 * @note Unlike the fmf_first_order which returns the current state estimate,
 * 		 fmf_second_order stores the state estimate and its derivative in the
 * 		 @param filterParams argument
 *
 */
void fmf_second_order(fmf_second_order_t* filterParams, float32_t x_meas) {

	 /*
	   a-priori step: we assume the solution is linear in time so we propagate the state estimate to x^-_k+1 = x^+_k + T_s * ẋ̂
       this also implies the state derivative is constant, so we don't change it

       a-posteriori step: assume the system is influenced by second order perturbations, so compute the residual y_k = x_meas - x^-_k+1
       it follows then that the state derivative residual is ẏ_k = y_k / T_s
       multiply both residuals by their respective gains and add to get x^+_k+1
	 */


	float32_t xHat = filterParams->currentStateEst;
	float32_t xDotHat = filterParams->currentDerivativeEst;
	float32_t T_s = filterParams->timestep;
	float32_t gain = filterParams->gain;
	float32_t HPrime = filterParams->HPrime;


	float32_t x_minus = xDotHat * T_s + xHat;
	float32_t y_k = x_meas - x_minus;
	filterParams->currentStateEst = gain * y_k + xHat;
	filterParams->currentDerivativeEst = HPrime * y_k + xDotHat;
}
