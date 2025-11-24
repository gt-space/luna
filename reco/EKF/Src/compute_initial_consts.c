#include "Inc/ekf.h"

const float32_t a[8] = {5.0122185, -4.9929004e-5, -5.415637e-10,
						-3.837231e-14, 2.55155e-18, -5.321706e-23,
						4.813401e-28, -1.6294356e-33};

const float32_t p0 = 102715.47296217596;
const float32_t we = 7.29211e-5;
const float32_t Rb = 75319.477207151;
const float32_t dh = 0;

const float32_t q0Buff[4] = {0.707106781186548, 0, 0.707106781186547, 0};
const float32_t lla0Buff[3] = {30.9275, -81.51472222222, 45};

const float32_t a1[] = 	{4.42073738361512e-10, 0, 0,
0, 2.7180339353827352e-10, 0,
0, 0, 3.2481139335141032e-10};
// gyro

const float32_t a2[] = {0.000795780911065406, 0, 0,
0, 0.0028214027742403335, 0,
0, 0, 0.002042316641325292};
// accel

const float32_t att_unc0 = 0.0698131700798f;
const float32_t pos_unc0 = 200;
const float32_t vel_unc0 = 4;
const float32_t gbias_unc0 = 0.000145444104333f;
const float32_t abias_unc0 = 40e-6f * 9.8f * 10;
const float32_t gsf_unc0 = 1e-4;
const float32_t asf_unc0 = 1e-4;

float32_t pressure_function(arm_matrix_instance_f32* x) {

	float32_t h = x->pData[6] + dh;

    float32_t poly =
        a[0] +
        a[1]*h +
        a[2]*h*h +
        a[3]*h*h*h +
        a[4]*h*h*h*h +
        a[5]*h*h*h*h*h +
        a[6]*h*h*h*h*h*h +
        a[7]*h*h*h*h*h*h*h;

	return powf(10.0, poly);
}

void pressure_derivative(arm_matrix_instance_f32* x, arm_matrix_instance_f32* Hb, float32_t HbData[1*21]) {

	float32_t h = x->pData[6] + dh;
	memset(HbData, 0, 21*sizeof(float32_t));

    // Compute the polynomial f(h)
    float32_t poly =
        a[0] +
        a[1]*h +
        a[2]*h*h +
        a[3]*h*h*h +
        a[4]*h*h*h*h +
        a[5]*h*h*h*h*h +
        a[6]*h*h*h*h*h*h +
        a[7]*h*h*h*h*h*h*h;

    // Compute the derivative f'(h)
    float32_t dpoly =
        a[1] +
        2.0f*a[2]*h +
        3.0f*a[3]*h*h +
        4.0f*a[4]*h*h*h +
        5.0f*a[5]*h*h*h*h +
        6.0f*a[6]*h*h*h*h*h +
        7.0f*a[7]*h*h*h*h*h*h;

    // dp/dh = log(10) * f'(h) * 10^f(h)
    HbData[5] = logf(10.0f) * dpoly * powf(10.0f, poly);

    arm_mat_init_f32(Hb, 1, 21, HbData);

}

void get_H(arm_matrix_instance_f32* H, float32_t HBuff[3*21]) {

	memset(HBuff, 0, 3 * 21 * sizeof(float32_t));
	arm_mat_init_f32(H, 3, 21, HBuff);

	arm_matrix_instance_f32 eye3;
	float32_t eye3Data[9];
	arm_mat_eye_f32(&eye3, eye3Data, 3);

	arm_mat_place_f32(&eye3, H, 0, 3);
}

void get_R(arm_matrix_instance_f32* R, float32_t RBuff[3*3]) {
	float32_t diagR[] = {2.05913450e-9, 2.20393443e-9,	1.70201301e+2};
	arm_mat_get_diag_f32(&(arm_matrix_instance_f32){3, 1, diagR}, R, RBuff);
}

void get_Rq(arm_matrix_instance_f32* Rq, float32_t RqBuff[3*3]) {

	float32_t RqInit[9] = {3.959466584378683e-5, 0, 0,
								0, 0.0007353003366215782, 0,
								0, 0, 2.525484755727429e-5};

	memcpy(RqBuff, RqInit, 9*sizeof(float32_t));
	arm_mat_init_f32(Rq, 3, 3, RqBuff);
}

void get_nu_gv_mat(arm_matrix_instance_f32* mat, float32_t buffer[3*3]) {

	/*
	4.42073738361512e-10 0 0;
	0 2.7180339353827352e-10 0;
	0 0 3.2481139335141032e-10
	*/
	mat->numRows = 3;
	mat->numCols = 3;
	mat->pData = &a1;
}

// nu_gu_mat = deg2rad(3/3600) * eye(3);
// Gyro Bias
void get_nu_gu_mat(arm_matrix_instance_f32* mat, float32_t buffer[3*3]) {
    arm_mat_eye_f32(mat, buffer, 3);
    float32_t scale = deg2rad(3.0f / 3600.0f * 10);
    arm_mat_scale_f32(mat, scale, mat);
}

// nu_av_mat = (200e-6 * 9.81) * eye(3);
void get_nu_av_mat(arm_matrix_instance_f32* mat, float32_t buffer[3*3]) {

	mat->numRows = 3;
	mat->numCols = 3;
	mat->pData = a2;
}

// nu_au_mat = (40e-6 * 9.8) * eye(3);
// Accelerometer bias
void get_nu_au_mat(arm_matrix_instance_f32* mat, float32_t buffer[3*3]) {
    arm_mat_eye_f32(mat, buffer, 3);
    float32_t scale = (40e-6f * 9.8f * 10);
    arm_mat_scale_f32(mat, scale, mat);
}

void compute_Q(arm_matrix_instance_f32* Q,
                      float32_t Q_buff[12*12],
                      const arm_matrix_instance_f32* nu_gv,
                      const arm_matrix_instance_f32* nu_gu,
                      const arm_matrix_instance_f32* nu_av,
                      const arm_matrix_instance_f32* nu_au,
                      float32_t dt)
{
    // Initialize Q as zeros
    memset(Q_buff, 0, sizeof(float32_t) * 12 * 12);
    arm_mat_init_f32(Q, 12, 12, Q_buff);

    // Place submatrices as per structure
    arm_mat_place_f32(nu_gv, Q, 0, 0);   // nu_gv_mat at (0,0)
    arm_mat_place_f32(nu_gu, Q, 3, 3);   // nu_gu_mat at (3,3)
    arm_mat_place_f32(nu_av, Q, 6, 6);   // nu_av_mat at (6,6)
    arm_mat_place_f32(nu_au, Q, 9, 9);   // nu_au_mat at (9,9)

    // Scale the entire matrix by (10 * dt)
    arm_mat_scale_f32(Q, 10.0f * dt, Q);
}

void compute_Qq(arm_matrix_instance_f32* Qq,
                       float32_t Qq_buff[6*6],
                       const arm_matrix_instance_f32* nu_gv,
                       const arm_matrix_instance_f32* nu_gu,
                       float32_t dt)
{
    // Initialize Qq as zeros
    memset(Qq_buff, 0, sizeof(float32_t) * 6 * 6);
    arm_mat_init_f32(Qq, 6, 6, Qq_buff);

    // Place submatrices
    arm_mat_place_f32(nu_gv, Qq, 0, 0);  // nu_gv_mat at (0,0)
    arm_mat_place_f32(nu_gu, Qq, 3, 3);  // nu_gu_mat at (3,3)

    // Scale the entire matrix by (10 * dt)
    arm_mat_scale_f32(Qq, 10.0f * dt, Qq);
}

void compute_P0(arm_matrix_instance_f32 *P0,
		   float32_t P0data[21*21],
		   float32_t att_unc0,
		   float32_t pos_unc0,
		   float32_t vel_unc0,
		   float32_t gbias_unc0,
		   float32_t abias_unc0,
		   float32_t gsf_unc0,
		   float32_t asf_unc0) {
    // Initialize matrix (row-major)
    arm_mat_init_f32(P0, 21, 21, P0data);

    // Zero entire matrix
    for (int i = 0; i < 21*21; i++)
        P0data[i] = 0.0f;

    // Fill diagonal
    int idx = 0;

    // 1. Attitude uncertainty (3 elements)
    for (int i = 0; i < 3; i++, idx++)
        P0data[idx * 21 + idx] = att_unc0;

    // 2. Position uncertainty (3)
    for (int i = 0; i < 3; i++, idx++)
        P0data[idx * 21 + idx] = pos_unc0;

    // 3. Velocity uncertainty (3)
    for (int i = 0; i < 3; i++, idx++)
        P0data[idx * 21 + idx] = vel_unc0;

    // 4. Gyro bias uncertainty (3)
    for (int i = 0; i < 3; i++, idx++)
        P0data[idx * 21 + idx] = gbias_unc0;

    // 5. Accel bias uncertainty (3)
    for (int i = 0; i < 3; i++, idx++)
        P0data[idx * 21 + idx] = abias_unc0;

    // 6. Gyro scale-factor uncertainty (3)
    for (int i = 0; i < 3; i++, idx++)
        P0data[idx * 21 + idx] = gsf_unc0;

    // 7. Accel scale-factor uncertainty (3)
    for (int i = 0; i < 3; i++, idx++)
        P0data[idx * 21 + idx] = asf_unc0;
}


void compute_Pq0(arm_matrix_instance_f32* Pq0,
                 float32_t Pq0_buff[6*6],
                 float32_t att_unc0,
                 float32_t gbias_unc0) {

    arm_mat_init_f32(Pq0, 6, 6, Pq0_buff);

    // Zero entire matrix
    for (int i = 0; i < 36; i++)
    	Pq0_buff[i] = 0.0f;

    int idx = 0;

    // 1. Attitude uncertainty (3)
    for (int i = 0; i < 3; i++, idx++)
    	Pq0_buff[idx * 6 + idx] = att_unc0;

    // 2. Gyro bias uncertainty (3)
    for (int i = 0; i < 3; i++, idx++)
    	Pq0_buff[idx * 6 + idx] = gbias_unc0;
}

void compute_magI(arm_matrix_instance_f32* magI, float32_t magIBuff[3]) {
	magIBuff[0] = 0.4891;
	magIBuff[1] = 0.1040;
	magIBuff[2]	= 0.8660;

	arm_mat_init_f32(magI, 3, 1, magIBuff);
}

void get_Hq(arm_matrix_instance_f32* magI, arm_matrix_instance_f32* Hq, float32_t HqData[3*6]) {
    // Temporary buffer for the skew matrix
    float32_t skewData[9];  // 3x3

    // Create the skew-symmetric matrix
    arm_matrix_instance_f32 skewMat;
    arm_mat_skew_f32(magI, &skewMat, skewData);

    // Initialize Hq as 3x6
    arm_mat_init_f32(Hq, 3, 6, HqData);

    // Copy skewMat into the left 3 columns of Hq
    for (uint32_t row = 0; row < 3; row++) {
        for (uint32_t col = 0; col < 3; col++) {
            HqData[row * 6 + col] = skewData[row * 3 + col];
        }
    }

    // Right 3 columns are zeros (already zeroed if you pre-initialize, otherwise set explicitly)
    for (uint32_t row = 0; row < 3; row++) {
        for (uint32_t col = 3; col < 6; col++) {
            HqData[row * 6 + col] = 0.0f;
        }
    }
}



