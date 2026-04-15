#include "ekf.h"

const float32_t we = 7.29211e-5; // Earth Sidereal Rotation (rad/s)
static float32_t RbInit = 50.0f; // Barometer Pressure Noise

// Initial Uncertainity in our states
static float32_t att_unc0[3] = {1e-2f, 1e-2f, 1e-2f};
static float32_t pos_unc0[3] = {1e-4f, 1e-4f, 1.0f};
static float32_t vel_unc0[3] = {1e-3f, 1e-3f, 1e-3f};
static float32_t gbias_unc0[3] = {2e-6f, 2e-6f, 2e-6f};
static float32_t abias_unc0[3] = {2.0f, 2.0f, 2.0f};
static float32_t gsf_unc0[3] = {1e-4f, 1e-4f, 1e-4f};
static float32_t asf_unc0[3] = {1e-4f, 1e-4f, 1e-4f};

// Initial Matrices for Filter
static arm_matrix_instance_f32 PInit; // Initial Covariance Matrix
static arm_matrix_instance_f32 QInit; // Initial Process Noise Matrix
static arm_matrix_instance_f32 xInit; // Initial State Matrix
static arm_matrix_instance_f32 RInit; // Inital GPS Measurment Noise Matrix

static arm_matrix_instance_f32 nu_gv_mat; // Gyro Covariance
static arm_matrix_instance_f32 nu_gu_mat; // Gyro Bias Covariance
static arm_matrix_instance_f32 nu_av_mat; // Accel covariance
static arm_matrix_instance_f32 nu_au_mat; // Accel Bias Covariance

float32_t PInitData[21*21]; // Backing Array for Initial Covariance
float32_t QInitData[12*12]; // Backing Array

float32_t RInitData[3*3] = {2.5e-7f, 0, 0,
							0, 2.5e-7f, 0,
							0, 0, 400.0f};

float32_t RqInitData[] = {2.5e-5, 0, 0,
					   	  0, 2.5e-5, 0,
						  0, 0, 2.5e-5};

// The initial state of the filter. Should be initialized by current attitude,
// current locations (lat, long, altitude), biases, and scale factors. xInit
float32_t xInitData[22*1] =  {-0.1822355f,
						  0.0f,
						  0.0f,
						  0.9832549f,
						  33.8785836f,
						  -84.3012703f,
						  297.668f,
						  0,
						  0,
						  0,
						  0,
						  0,
						  0,
						  0,
						  0,
						  0,
						  0,
						  0,
						  0,
						  0,
						  0,
						  0};

// Gyro Covariance Data
float32_t nu_gv_data[3*3] = {0.02f, 0, 0,
		  	  	  	  	  	 0, 0.02f, 0,
							 0, 0, 0.02f};

// Gyro Bias Covariance Data
float32_t nu_gu_data[3*3] = {0.00002f, 0, 0,
							 0,	0.00002f, 0,
							 0, 0, 0.00002f};

// Accelerometer Covariance Data
float32_t nu_av_data[3*3] = {2.0f, 0, 0,
							 0,	2.0f, 0,
							 0, 0, 2.0f};

// Accel Bias Covariance Data
float32_t nu_au_data[3*3] = {0.002f, 0, 0,
							 0, 0.002f, 0,
							 0, 0, 0.002f};

/**
 * @brief Initializes the Extended Kalman Filter (EKF) state, covariance,
 *        and noise matrices.
 *
 * This function sets up the EKF by initializing all required matrices,
 * including the state vector, covariance matrices, process noise,
 * and measurement noise. It computes the initial covariance (P0) and
 * process noise (Q0) based on predefined uncertainties and the provided
 * timestep. It also initializes measurement models and sensor noise
 * parameters for GPS, barometer, and magnetometer.
 *
 * Specifically, this function:
 * - Initializes internal matrix structures and backing arrays
 * - Computes initial covariance matrix (P0)
 * - Computes initial process noise matrix (Q0) using the timestep
 * - Loads the initial state vector (x0) and covariance into provided outputs
 * - Initializes measurement Jacobians (H, Hb)
 * - Sets measurement noise matrices (R, Rq) and barometer noise (Rb)
 * - Computes the reference magnetic field vector (magI)
 *
 * @param[out] xPrev Pointer to the initialized state vector (22x1)
 * @param[out] PPrev Pointer to the initialized covariance matrix (21x21)
 * @param[out] H Pointer to the GPS measurement Jacobian matrix (3x21)
 * @param[out] Hb Pointer to the barometer measurement Jacobian matrix
 * @param[out] R Pointer to the GPS measurement noise matrix (3x3)
 * @param[out] Rq Pointer to the magnetometer measurement noise matrix (3x3)
 * @param[out] Q Pointer to the process noise matrix (12x12)
 * @param[out] magI Pointer to the reference magnetic field vector (3x1)
 * @param[out] Rb Pointer to the initialized barometer noise scalar
 * @param[in]  dt Discrete time step used to scale the process noise matrix
 *
 * @note The input matrices must be pre-allocated with correct dimensions
 *       and valid backing memory before calling this function.
 *
 * @warning This function relies on internally defined initial conditions
 *          and uncertainty parameters. Modify those via setter functions
 *          before calling if different initialization is required.
 */
void ekf_init(arm_matrix_instance_f32* xPrev,
			  arm_matrix_instance_f32* PPrev,
			  arm_matrix_instance_f32* H,
			  arm_matrix_instance_f32* Hb,
			  arm_matrix_instance_f32* R,
			  arm_matrix_instance_f32* Rq,
			  arm_matrix_instance_f32* Q,
			  arm_matrix_instance_f32* magI,
			  fmf_first_order_t* groundBaro,
			  fmf_first_order_t* groundGPS,
			  fmf_second_order_t* flightBaro,
			  float32_t* Rb,
			  float32_t dt) {

	// Initialize the initial matrices struct with the dimensions of
	// the matrix and its associated backing array
	arm_mat_init_f32(&xInit, 22, 1, xInitData);
	arm_mat_init_f32(&PInit, 21, 21, PInitData);
	arm_mat_init_f32(&QInit, 12, 12, QInitData);
	arm_mat_init_f32(&RInit, 3, 3, RInitData);

	arm_mat_init_f32(&nu_gv_mat, 3, 3, nu_gv_data);
	arm_mat_init_f32(&nu_gu_mat, 3, 3, nu_gu_data);
	arm_mat_init_f32(&nu_av_mat, 3, 3, nu_av_data);
	arm_mat_init_f32(&nu_au_mat, 3, 3, nu_au_data);

	// Calculate PInit and QInit using the loaded uncertanties
	// for PInit and the covariance and bias covariance for
	// QInit
	compute_P0();
	compute_Q0(dt);

	// Initializes the flight state vector and covariance matrix
	// using xInit and PInit defined in this file
	get_x0(xPrev);
	get_P0(PPrev);

	// Initializes the barometer measurement Jacobian (H),
	// the GPS measurement Jacobian (Hb),
	// the GPS measurement noise matrix (R),
	// and the magnetometer measurement noise (Rq),
	// barometer noise (Rb)
	get_H(H, H->pData);
	initialize_Hb(xPrev, Hb, Hb->pData);
	get_R0(R);
	get_Rq0(Rq);
	*Rb = get_Rb0();

	// Initialize FMF by setting our state estimates
	// to the altitude in our initial state vector
	// and set the gains for each of the FMF

	float32_t initialAltitude = xPrev->pData[6];

	fmf_first_order_init(groundBaro, initialAltitude, get_initial_baro_ground_beta());
	fmf_first_order_init(groundGPS, initialAltitude, get_initial_gps_ground_beta());
	fmf_second_order_init(flightBaro, initialAltitude, get_initial_baro_flight_beta(), dt);


	compute_magI(magI, magI->pData);
}

// Initial Process Noise Matrix
void compute_Q0(float32_t dt)
{
    // Initialize Q as zeros
    memset(QInitData, 0, sizeof(float32_t) * 12 * 12);
    arm_mat_init_f32(&QInit, 12, 12, QInitData);

    // Place submatrices as per structure
    arm_mat_place_f32(&nu_gv_mat, &QInit, 0, 0);   // nu_gv_mat at (0,0)
    arm_mat_place_f32(&nu_gu_mat, &QInit, 3, 3);   // nu_gu_mat at (3,3)
    arm_mat_place_f32(&nu_av_mat, &QInit, 6, 6);   // nu_av_mat at (6,6)
    arm_mat_place_f32(&nu_au_mat, &QInit, 9, 9);   // nu_au_mat at (9,9)

    // Scale the entire matrix by (10 * dt)
    arm_mat_scale_f32(&QInit, 1, &QInit);
}

// Initial Covariance Matrix 
void compute_P0(void) {
    // Initialize matrix (row-major)
    arm_mat_init_f32(&PInit, 21, 21, PInitData);

    // Zero entire matrix
    for (int i = 0; i < 21*21; i++)
    	PInitData[i] = 0.0f;

    // Fill diagonal
    int idx = 0;

    // 1. Attitude uncertainty (3 elements)
    for (int i = 0; i < 3; i++, idx++)
    	PInitData[idx * 21 + idx] = att_unc0[i];

    // 2. Position uncertainty (3)
    for (int i = 0; i < 3; i++, idx++)
    	PInitData[idx * 21 + idx] = pos_unc0[i];

    // 3. Velocity uncertainty (3)
    for (int i = 0; i < 3; i++, idx++)
    	PInitData[idx * 21 + idx] = vel_unc0[i];

    // 4. Gyro bias uncertainty (3)
    for (int i = 0; i < 3; i++, idx++)
    	PInitData[idx * 21 + idx] = gbias_unc0[i];

    // 5. Accel bias uncertainty (3)
    for (int i = 0; i < 3; i++, idx++)
    	PInitData[idx * 21 + idx] = abias_unc0[i];

    // 6. Gyro scale-factor uncertainty (3)
    for (int i = 0; i < 3; i++, idx++)
    	PInitData[idx * 21 + idx] = gsf_unc0[i];

    // 7. Accel scale-factor uncertainty (3)
    for (int i = 0; i < 3; i++, idx++)
    	PInitData[idx * 21 + idx] = asf_unc0[i];
}


// Setter Functions

void set_uncertanties(float32_t new_att_unc[3],
					  float32_t new_pos_unc[3],
					  float32_t new_vel_unc[3],
					  float32_t new_gbias_unc[3],
					  float32_t new_abias_unc[3],
					  float32_t new_gsf_unc[3],
					  float32_t new_asf_unc[3]) {

	memcpy(att_unc0, new_att_unc, 3*sizeof(float32_t));
	memcpy(pos_unc0, new_pos_unc, 3*sizeof(float32_t));
	memcpy(vel_unc0, new_vel_unc, 3*sizeof(float32_t));
	memcpy(gbias_unc0, new_gbias_unc, 3*sizeof(float32_t));
	memcpy(abias_unc0, new_abias_unc, 3*sizeof(float32_t));
	memcpy(gsf_unc0, new_gsf_unc, 3*sizeof(float32_t));
	memcpy(asf_unc0, new_asf_unc, 3*sizeof(float32_t));
	return;
}

void set_x0(float32_t new_initial_state[22*1]) {
	memcpy(xInitData, new_initial_state, 22*sizeof(float32_t));
}

void set_R0(float32_t R_set[9]) {
	memcpy(RInitData, R_set, 9*sizeof(float32_t));
}

void set_Rq0(float32_t Rq_set[9]) {
	memcpy(RqInitData, Rq_set, 9*sizeof(float32_t));
}

void set_nu_gv0(float32_t nu_gv_set[9]) {
	memcpy(nu_gv_data, nu_gv_set, 9*sizeof(float32_t));
}

void set_nu_gu0(float32_t nu_gu_set[9]) {
	memcpy(nu_gu_data, nu_gu_set, 9*sizeof(float32_t));
}

void set_nu_av0(float32_t nu_av_set[9]) {
	memcpy(nu_av_data, nu_av_set, 9*sizeof(float32_t));
}

void set_nu_au0(float32_t nu_au_set[9]) {
	memcpy(nu_au_data, nu_au_set, 9*sizeof(float32_t));
}

void set_Rb0(float32_t Rb_set) {
	RbInit = Rb_set;
}

// Getter Functions

void get_x0(arm_matrix_instance_f32* x) {
	memcpy(x->pData, xInitData, 22*sizeof(float32_t));
}

void get_P0(arm_matrix_instance_f32* P) {
	memcpy(P->pData, PInitData, 21*21*sizeof(float32_t));
}

void get_Q0(arm_matrix_instance_f32* Q) {
	memcpy(Q->pData, QInitData, 12*12*sizeof(float32_t));
}

void get_R0(arm_matrix_instance_f32* R) {
	memcpy(R->pData, RInitData, 3*3*sizeof(float32_t));
}

void get_Rq0(arm_matrix_instance_f32* Rq) {
	memcpy(Rq->pData, RqInitData, 3*3*sizeof(float32_t));
}

float32_t get_Rb0() {
	return RbInit;
}

// GPS Measurement Jacobian
void get_H(arm_matrix_instance_f32* H, float32_t HBuff[3*21]) {

	memset(HBuff, 0, 3 * 21 * sizeof(float32_t));
	arm_mat_init_f32(H, 3, 21, HBuff);

	arm_matrix_instance_f32 eye3;
	float32_t eye3Data[9];
	arm_mat_eye_f32(&eye3, eye3Data, 3);

	arm_mat_place_f32(&eye3, H, 0, 3);
}

// The magnetic field at the launch site
void compute_magI(arm_matrix_instance_f32* magI, float32_t magIBuff[3]) {
	magIBuff[0] = 0.4891;
	magIBuff[1] = 0.1040;
	magIBuff[2]	= 0.8660;

	arm_mat_init_f32(magI, 3, 1, magIBuff);
}




