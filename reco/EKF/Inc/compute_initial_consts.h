#ifndef _COMPUTE_INITS
#define _COMPUTE_INITS

#include "common.h"
#include "matrix_extensions.h"
#include "quaternion_extensions.h"
#include "trig_extensions.h"

extern const float32_t a[8];
extern const float32_t we;
extern const float32_t p0;
extern const float32_t q0Buff[4];

float32_t pressure_function(float32_t h, float32_t p0);
float32_t pressure_derivative(float32_t h);

void get_H(arm_matrix_instance_f32* H, float32_t HBuff[3*21]);
void get_R(arm_matrix_instance_f32* R, float32_t RBuff[3*3]);
void get_Rq(arm_matrix_instance_f32* Rq, float32_t RqBuff[3*3]);
void get_nu_gv_mat(arm_matrix_instance_f32* mat, float32_t buffer[3*3]);
void get_nu_gu_mat(arm_matrix_instance_f32* mat, float32_t* buffer[3*3]);
void get_nu_av_mat(arm_matrix_instance_f32* mat, float32_t* buffer[3*3]);
void get_nu_au_mat(arm_matrix_instance_f32* mat, float32_t* buffer[3*3]);

void compute_Q(arm_matrix_instance_f32* Q,
                      float32_t Q_buff[12*12],
                      const arm_matrix_instance_f32* nu_gv,
                      const arm_matrix_instance_f32* nu_gu,
                      const arm_matrix_instance_f32* nu_av,
                      const arm_matrix_instance_f32* nu_au,
                      float32_t dt);

void compute_Qq_matrix(arm_matrix_instance_f32* Qq,
                       float32_t Qq_buff[6*6],
                       const arm_matrix_instance_f32* nu_gv,
                       const arm_matrix_instance_f32* nu_gu,
                       float32_t dt);

void compute_P0(arm_matrix_instance_f32* P0,
                float32_t P0_buff[21*21],
                float32_t att_unc0,
                float32_t pos_unc0,
			    float32_t vel_unc0,
			    float32_t gbias_unc0,
			    float32_t abias_unc0,
			    float32_t gsf_unc0,
			    float32_t asf_unc0);

void compute_Pq0(arm_matrix_instance_f32* Pq0,
                 float32_t Pq0_buff[6*6],
                 float32_t att_unc0,
                 float32_t gbias_unc0);

void compute_Hb(const arm_matrix_instance_f32* x,
                arm_matrix_instance_f32* Hb,
                float32_t HbData[1*21]);

#endif
