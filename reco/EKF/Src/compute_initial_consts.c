#include "ekf.h"

const float32_t we = 7.29211e-5; // Earth Sidereal Rotation (rad/s)
const float32_t Rb = 2500.0f; // Barometer Pressure Noise

// Initial Uncertainity in our states
const float32_t att_unc0 = 4e-3f;
const float32_t pos_unc0[3] = {1e-8f, 1e-8f, 1.0f};
const float32_t vel_unc0 = 1e-4f;
const float32_t gbias_unc0 = 2e-6f;
const float32_t abias_unc0 = 0.2f;
const float32_t gsf_unc0 = 1e-6f;
const float32_t asf_unc0 = 1e-6f;

// GPS Measurement Jacobian
void get_H(arm_matrix_instance_f32* H, float32_t HBuff[3*21]) {

	memset(HBuff, 0, 3 * 21 * sizeof(float32_t));
	arm_mat_init_f32(H, 3, 21, HBuff);

	arm_matrix_instance_f32 eye3;
	float32_t eye3Data[9];
	arm_mat_eye_f32(&eye3, eye3Data, 3);

	arm_mat_place_f32(&eye3, H, 0, 3);
}

// Mag Noise
void get_Rq(arm_matrix_instance_f32* Rq, float32_t RqBuff[3*3]) {

	float32_t copyMat[] = {2.5e-5, 0, 0,
						   0, 2.5e-5, 0,
						   0, 0, 2.5e-5};

	memcpy(RqBuff, copyMat, 9*sizeof(float32_t));
	arm_mat_init_f32(Rq, 3, 3, RqBuff);
}

// GPS Noise
void get_R(arm_matrix_instance_f32* R, float32_t RBuff[3*3]) {

	float32_t copyMat[9] = {5e-10f, 0, 0,
							0, 1e-9f, 0,
							0, 0, 400.0f};

	memcpy(RBuff, copyMat, 9*sizeof(float32_t));
	arm_mat_init_f32(R, 3, 3, RBuff);
}

// Gyro Covariance
void get_nu_gv_mat(arm_matrix_instance_f32* mat, float32_t buffer[3*3]) {

	float32_t copyMat[3*3] = {2e-4f, 0, 0,
							  0, 2e-4f, 0,
							  0, 0, 2e-4f};

	memcpy(buffer, copyMat, 9*sizeof(float32_t));
	arm_mat_init_f32(mat, 3, 3, buffer);
}

// nu_gu_mat = deg2rad(3/3600) * eye(3);
// Gyro Bias Covariance
void get_nu_gu_mat(arm_matrix_instance_f32* mat, float32_t buffer[3*3]) {

    float32_t scale = 1e-3;
    scale = scale * scale;

    float32_t copyBuff[3*3] = {scale, 0, 0,
                               0, scale, 0,
                               0, 0, scale};

    memcpy(buffer, copyBuff, 9*sizeof(float32_t));
    arm_mat_init_f32(mat, 3, 3, buffer);
}

// nu_av_mat = (200e-6 * 9.81) * eye(3);
// Accelerometer Covariance
void get_nu_av_mat(arm_matrix_instance_f32* mat, float32_t buffer[3*3]) {

	float32_t copyBuff[3*3] = {0.1f, 0, 0,
							   0, 0.1f, 0,
							   0, 0, 0.1f};

	memcpy(buffer, copyBuff, 9*sizeof(float32_t));
	arm_mat_init_f32(mat, 3, 3, buffer);
}

// nu_au_mat = (40e-6 * 9.8 / 3600) * eye(3);
// Accelerometer Bias Covariance
void get_nu_au_mat(arm_matrix_instance_f32* mat, float32_t buffer[3*3]) {
    float32_t scale = 2e-2;
    scale = scale * scale;

    float32_t copyBuff[3*3] = {scale, 0, 0,
                               0, scale, 0,
                               0, 0, scale};

    memcpy(buffer, copyBuff, 9*sizeof(float32_t));
    arm_mat_init_f32(mat, 3, 3, buffer);
}

// Initial Process Noise Matrix
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

// Initial Covariance Matrix 
void compute_P0(arm_matrix_instance_f32 *P0,
		   float32_t P0data[21*21],
		   float32_t att_unc0,
		   float32_t pos_unc0[3],
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
        P0data[idx * 21 + idx] = pos_unc0[i];

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

// The magnetic field at the launch site
void compute_magI(arm_matrix_instance_f32* magI, float32_t magIBuff[3]) {
	magIBuff[0] = 0.4891;
	magIBuff[1] = 0.1040;
	magIBuff[2]	= 0.8660;

	arm_mat_init_f32(magI, 3, 1, magIBuff);
}




