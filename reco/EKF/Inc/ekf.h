/*
 * ekf.h
 *
 *  Created on: Nov 9, 2025
 *      Author: tigis
 */

#ifndef EKF_H_
#define EKF_H_

#include "common.h"
#include "matrix_extensions.h"
#include "quaternion_extensions.h"
#include "trig_extensions.h"
#include "ekf_utils.h"
#include "string.h"

#include "stdbool.h"

extern bool gpsReady;
extern bool baroReady;
extern bool magReady;

extern const float32_t a[8];
extern const float32_t we;
extern const float32_t p0;
extern const float32_t Rb;
extern const float32_t q0Buff[4];

extern const float32_t att_unc0;
extern const float32_t pos_unc0;
extern const float32_t vel_unc0;
extern const float32_t gbias_unc0;
extern const float32_t abias_unc0;
extern const float32_t gsf_unc0;
extern const float32_t asf_unc0;

float32_t pressure_function(arm_matrix_instance_f32* x);

void pressure_derivative(arm_matrix_instance_f32* x,
						 arm_matrix_instance_f32* Hb,
						 float32_t HbData[1*21]);

void get_H(arm_matrix_instance_f32* H, float32_t HBuff[3*21]);
void get_R(arm_matrix_instance_f32* R, float32_t RBuff[3*3]);
void get_Rq(arm_matrix_instance_f32* Rq, float32_t RqBuff[3*3]);
void get_nu_gv_mat(arm_matrix_instance_f32* mat, float32_t buffer[3*3]);
void get_nu_gu_mat(arm_matrix_instance_f32* mat, float32_t buffer[3*3]);
void get_nu_av_mat(arm_matrix_instance_f32* mat, float32_t buffer[3*3]);
void get_nu_au_mat(arm_matrix_instance_f32* mat, float32_t buffer[3*3]);

void compute_Q(arm_matrix_instance_f32* Q,
                      float32_t Q_buff[12*12],
                      const arm_matrix_instance_f32* nu_gv,
                      const arm_matrix_instance_f32* nu_gu,
                      const arm_matrix_instance_f32* nu_av,
                      const arm_matrix_instance_f32* nu_au,
                      float32_t dt);

void compute_Qq(arm_matrix_instance_f32* Qq,
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

void compute_wn(float32_t phi, float32_t h, float32_t vn, float32_t ve,
				arm_matrix_instance_f32* wn, float32_t we, float32_t buffer[3]);

void compute_what(arm_matrix_instance_f32* q, arm_matrix_instance_f32* bias_g, arm_matrix_instance_f32* sf_g,
				  float32_t phi, float32_t h, float32_t vn, float32_t ve, float32_t we, arm_matrix_instance_f32* w_meas,
				  arm_matrix_instance_f32* what, float32_t whatBuffer[3]);

void compute_ahat(arm_matrix_instance_f32* q, arm_matrix_instance_f32* sf_a, arm_matrix_instance_f32* bias_a,
				  arm_matrix_instance_f32* a_meas, arm_matrix_instance_f32* ahat_n, float32_t ahatBuff[3]);

void compute_qdot(arm_matrix_instance_f32* x, arm_matrix_instance_f32* what,
				  arm_matrix_instance_f32* qdot, float32_t qDotBuff[4]);

void compute_lla_dot(float32_t phi, float32_t h, float32_t vn, float32_t ve, float32_t vd,
					 arm_matrix_instance_f32* llaDot, float32_t llaDotBuff[3]);

void compute_vdot(float32_t phi, float32_t h, float32_t vn, float32_t ve, float32_t vd,
				  float32_t ahat_n[3], float32_t we, arm_matrix_instance_f32* vdot, float32_t vdotBuff[3]);

void compute_dpdot_dp(float32_t phi, float32_t h, float32_t vn,
					  float32_t ve, arm_matrix_instance_f32* dpdot_dp, float32_t dpDotBuff[9]);

void compute_dpdot_dv(float32_t phi, float32_t h,
					  arm_matrix_instance_f32* dpdot_dv, float32_t dpDotBuff[9]);

void compute_dvdot_dp(float32_t phi, float32_t h, float32_t vn, float32_t ve, float32_t vd,
					  float32_t we,  arm_matrix_instance_f32* dvdot_dp, float32_t dvdotBuff[9]);

void compute_dvdot_dv(float32_t phi, float32_t h, float32_t vn, float32_t ve, float32_t vd,
					  float32_t we, arm_matrix_instance_f32* dvdot_dv, float32_t dvdotBuff[9]);

void compute_dwdp(float32_t phi, float32_t h, float32_t ve, float32_t vn, float32_t we,
				  arm_matrix_instance_f32* dwdp, float32_t dwdpBuffer[9]);

void compute_dwdv(float32_t phi, float32_t h, arm_matrix_instance_f32* dwdv, float32_t dwdvBuffer[9]);

void compute_F(arm_matrix_instance_f32* q, arm_matrix_instance_f32* sf_a, arm_matrix_instance_f32* sf_g,
			  arm_matrix_instance_f32* bias_g, arm_matrix_instance_f32* bias_a, float32_t phi, float32_t h,
			  float32_t vn, float32_t ve, float32_t vd, arm_matrix_instance_f32* a_meas, arm_matrix_instance_f32* w_meas,
			  float32_t we, arm_matrix_instance_f32* F, float32_t FBuff[21*21]);

void compute_G(arm_matrix_instance_f32* sf_g, arm_matrix_instance_f32* sf_a, arm_matrix_instance_f32* q,
			   arm_matrix_instance_f32* G, float32_t GBuff[21*12]);

void compute_Pdot(arm_matrix_instance_f32* q, arm_matrix_instance_f32* sf_a, arm_matrix_instance_f32* sf_g,
		  	  	  arm_matrix_instance_f32* bias_g, arm_matrix_instance_f32* bias_a, arm_matrix_instance_f32* a_meas,
				  arm_matrix_instance_f32* w_meas, arm_matrix_instance_f32* P, arm_matrix_instance_f32* Q,
				  float32_t phi, float32_t h, float32_t vn, float32_t ve, float32_t vd, float32_t we,
				  arm_matrix_instance_f32* Pdot, float32_t PdotBuff[21*21]);

void compute_Pqdot(float32_t *x, float32_t *Pq, float32_t *Qq, float32_t *w_meas,
                   arm_matrix_instance_f32* Pqdot, float32_t PqdotBuff[6*6]);

void propogate(arm_matrix_instance_f32* xPlus, arm_matrix_instance_f32* Pplus, arm_matrix_instance_f32* PqPlus,
			   arm_matrix_instance_f32* what, arm_matrix_instance_f32* aHatN, arm_matrix_instance_f32* wMeas,
			   arm_matrix_instance_f32* aMeas, arm_matrix_instance_f32* Q, arm_matrix_instance_f32* Qq, float32_t dt,
			   float32_t we, arm_matrix_instance_f32* xMinus, arm_matrix_instance_f32* PMinus, arm_matrix_instance_f32* PqMinus,
			   float32_t xMinusBuff[22], float32_t PMinusBuff[21*21], float32_t PqMinusBuff[6*6]);

void update_GPS(float32_t *x_plus, float32_t *P_plus, float32_t *Pq_plus, float32_t *x_minus, float32_t *P_minus, float32_t *Pq_minus, float32_t *H, float32_t *R, float32_t *lla_meas);

void update_mag(arm_matrix_instance_f32* x_minus, arm_matrix_instance_f32* P_minus, arm_matrix_instance_f32* Pq_minus,
				arm_matrix_instance_f32* Hq, arm_matrix_instance_f32* Rq, arm_matrix_instance_f32* R,
				arm_matrix_instance_f32* magI, arm_matrix_instance_f32* mag_meas, arm_matrix_instance_f32* x_plus,
				arm_matrix_instance_f32* P_plus, arm_matrix_instance_f32* Pq_plus, float32_t x_plus_buff[21*1],
				float32_t P_plus_buff[21*21], float32_t Pq_plus_buff[6*6]);

void update_baro(arm_matrix_instance_f32* xMinus, arm_matrix_instance_f32* PMinus, float32_t pressMeas,
				 float32_t Rb, arm_matrix_instance_f32* xPlus, arm_matrix_instance_f32* Pplus,
				 float32_t xPlusData[22*1], float32_t pPlusData[21*21]);

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
				float32_t PqPlusBuff[6*6]);

void nearestPSD();


#endif /* EKF_INC_EKF_H_ */
