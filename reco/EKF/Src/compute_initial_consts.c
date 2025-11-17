#include "Inc/ekf.h"

const float32_t a[8] = {5.0122185, -4.9929004e-5, -5.415637e-10,
						-3.837231e-14, 2.55155e-18, -5.321706e-23,
						4.813401e-28, -1.6294356e-33};

const float32_t p0 = 102715.47296217596;
const float32_t we = 7.29211e-5;
const float32_t Rb = 2.5e-3;

const float32_t q0Buff[4] = {0.707106781186548, 0, 0.707106781186547, 0};
const float32_t lla0Buff[3] = {30.9275, -81.51472222222, 45};

const float32_t att_unc0 = 1e-4;
const float32_t pos_unc0 = 1e-4;
const float32_t vel_unc0 = 1e-4;
const float32_t gbias_unc0 = 1e-4;
const float32_t abias_unc0 = 1e-4;
const float32_t gsf_unc0 = 1e-4;
const float32_t asf_unc0 = 1e-4;

float32_t pressure_function(arm_matrix_instance_f32* x) {

	float32_t h = x->pData[6];

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

	float32_t h = x->pData[6];
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
	float32_t diagR[] = {1.35e-5, 1.65e-5, 2};
	arm_mat_get_diag_f32(&(arm_matrix_instance_f32){3, 1, diagR}, R, RBuff);
}

void get_Rq(arm_matrix_instance_f32* Rq, float32_t RqBuff[3*3]) {
	float32_t diagRq[] = {3.2e-7, 4.1e-7, 3.2e-7};
	arm_mat_get_diag_f32(&(arm_matrix_instance_f32){3, 1, diagRq}, Rq, RqBuff);
}

void get_nu_gv_mat(arm_matrix_instance_f32* mat, float32_t buffer[3*3]) {
    arm_mat_eye_f32(mat, buffer, 3);
    float32_t scale = deg2rad(12e-3f);
    arm_mat_scale_f32(mat, scale, mat);
}

// nu_gu_mat = deg2rad(3/3600) * eye(3);
void get_nu_gu_mat(arm_matrix_instance_f32* mat, float32_t buffer[3*3]) {
    arm_mat_eye_f32(mat, buffer, 3);
    float32_t scale = deg2rad(3.0f / 3600.0f);
    arm_mat_scale_f32(mat, scale, mat);
}

// nu_av_mat = (200e-6 * 9.81) * eye(3);
void get_nu_av_mat(arm_matrix_instance_f32* mat, float32_t buffer[3*3]) {
    arm_mat_eye_f32(mat, buffer, 3);
    float32_t scale = (200e-6f * 9.81f);
    arm_mat_scale_f32(mat, scale, mat);
}

// nu_au_mat = (40e-6 * 9.8) * eye(3);
void get_nu_au_mat(arm_matrix_instance_f32* mat, float32_t buffer[3*3]) {
    arm_mat_eye_f32(mat, buffer, 3);
    float32_t scale = (40e-6f * 9.8f);
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

void compute_P0(arm_matrix_instance_f32* P0,
                float32_t P0_buff[21*21],
                float32_t att_unc0,
                float32_t pos_unc0,
			    float32_t vel_unc0,
			    float32_t gbias_unc0,
			    float32_t abias_unc0,
			    float32_t gsf_unc0,
			    float32_t asf_unc0)
{
    // There are 7 groups of 3 = 21 diagonal entries
    float32_t diagVals[21];

    // Fill each group of 3
    for (int i = 0; i < 3; i++) diagVals[i]      = att_unc0;
    for (int i = 3; i < 6; i++) diagVals[i]      = pos_unc0;
    for (int i = 6; i < 9; i++) diagVals[i]      = vel_unc0;
    for (int i = 9; i < 12; i++) diagVals[i]     = gbias_unc0;
    for (int i = 12; i < 15; i++) diagVals[i]    = abias_unc0;
    for (int i = 15; i < 18; i++) diagVals[i]    = gsf_unc0;
    for (int i = 18; i < 21; i++) diagVals[i]    = asf_unc0;

    // Wrap diagVals in a temporary 21x1 "matrix" for use with arm_mat_get_diag_f32
    arm_matrix_instance_f32 diagInput;
    arm_mat_init_f32(&diagInput, 21, 1, diagVals);

    // Generate diagonal matrix
    arm_mat_get_diag_f32(&diagInput, P0, P0_buff);
}

void compute_Pq0(arm_matrix_instance_f32* Pq0,
                 float32_t Pq0_buff[6*6],
                 float32_t att_unc0,
                 float32_t gbias_unc0)
{
    // 6 diagonal entries total
    float32_t diagVals[6];

    // First 3 are attitude uncertainties
    for (int i = 0; i < 3; i++) diagVals[i] = att_unc0;

    // Last 3 are gyro bias uncertainties
    for (int i = 3; i < 6; i++) diagVals[i] = gbias_unc0;

    // Wrap diagVals in a temporary 6x1 "matrix" for use with arm_mat_get_diag_f32
    arm_matrix_instance_f32 diagInput;
    arm_mat_init_f32(&diagInput, 6, 1, diagVals);

    // Generate diagonal matrix
    arm_mat_get_diag_f32(&diagInput, Pq0, Pq0_buff);
}



