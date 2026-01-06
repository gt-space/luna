/*
 * ekf.h
 *
 *  Created on: Nov 9, 2025
 *      Author: Raey Ayalew
 */

#ifndef EKF_H_
#define EKF_H_

#include "../CControl/ccontrol.h"

#include "common.h"
#include "matrix_extensions.h"
#include "quaternion_extensions.h"
#include "trig_extensions.h"
#include "ekf_utils.h"

#include "string.h"
#include "stdbool.h"
#include "stdint.h"
#include "stdatomic.h"

#include "comms.h"
#include "main.h"

#define SPEEDOFSOUND 295.069495691898f

// hOffset is the height offset to account for bias in the barometer (in meters)
#define hOffset 100.0f

extern const float32_t att_unc0;
extern const float32_t pos_unc0;
extern const float32_t vel_unc0;
extern const float32_t gbias_unc0;
extern const float32_t abias_unc0;
extern const float32_t gsf_unc0;
extern const float32_t asf_unc0;
extern const float32_t dh;

extern volatile atomic_uchar safeToWrite;
extern volatile atomic_uchar gpsEventCount;
extern volatile atomic_uchar magEventCount;
extern volatile atomic_uchar baroEventCount;

// Parachute Logic
bool drougeChuteCheck(float32_t vdNow, float32_t altNow, uint32_t* vdStart, uint32_t* altStart);
bool mainChuteCheck(float32_t vdNow, float32_t altNow, uint32_t* altStart);

// Altimeter / Barometer Functions
float32_t lerp(float32_t logP);
float32_t pressure_altimeter_uncorrected(float32_t P);
float32_t pressure_altimeter_corrected(float32_t P);
float32_t laguerre_solve(float32_t x0, float32_t yHat);
float32_t logP2alt(float32_t logP);
float32_t filter_P(float32_t h);
float32_t filter_dLogNorm_dH(float32_t h);
float32_t filter_lognormP(float32_t h);
float32_t filter_dP_dH(float32_t h);

// Matrix Initialization Code
void get_H(arm_matrix_instance_f32* H, float32_t HBuff[3*21]);
void get_R(arm_matrix_instance_f32* R, float32_t RBuff[3*3]);
void get_Rq(arm_matrix_instance_f32* Rq, float32_t RqBuff[3*3]);
void get_nu_gv_mat(arm_matrix_instance_f32* mat, float32_t buffer[3*3]);
void get_nu_gu_mat(arm_matrix_instance_f32* mat, float32_t buffer[3*3]);
void get_nu_av_mat(arm_matrix_instance_f32* mat, float32_t buffer[3*3]);
void get_nu_au_mat(arm_matrix_instance_f32* mat, float32_t buffer[3*3]);
void compute_magI(arm_matrix_instance_f32* magI, float32_t magIBuff[3]);
void initialize_Hb(arm_matrix_instance_f32* x, arm_matrix_instance_f32* Hb, float32_t HbBuff[1*21]);

void compute_Q(arm_matrix_instance_f32* Q,
                      float32_t Q_buff[12*12],
                      const arm_matrix_instance_f32* nu_gv,
                      const arm_matrix_instance_f32* nu_gu,
                      const arm_matrix_instance_f32* nu_av,
                      const arm_matrix_instance_f32* nu_au,
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

// EKF Code
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

void integrate(arm_matrix_instance_f32* x, arm_matrix_instance_f32* P, arm_matrix_instance_f32* qdot,
			   arm_matrix_instance_f32* pdot, arm_matrix_instance_f32* vdot, arm_matrix_instance_f32* Pdot,
			   float32_t dt, arm_matrix_instance_f32* xMinus, arm_matrix_instance_f32* Pminus,
			   float32_t xMinusBuff[22], float32_t PMinusBuff[21*21]);

void propogate(arm_matrix_instance_f32* xPlus, arm_matrix_instance_f32* Pplus, arm_matrix_instance_f32* what,
		   	   arm_matrix_instance_f32* aHatN, arm_matrix_instance_f32* wMeas, arm_matrix_instance_f32* aMeas,
			   arm_matrix_instance_f32* Q, float32_t dt, float32_t we,
			   arm_matrix_instance_f32* xMinus, arm_matrix_instance_f32* PMinus, float32_t xMinusBuff[22],
			   float32_t PMinusBuff[21*21]);

void update_GPS(const arm_matrix_instance_f32* x_minus, const arm_matrix_instance_f32* P_minus, const arm_matrix_instance_f32* H,
				const arm_matrix_instance_f32* R, const arm_matrix_instance_f32* lla_meas, arm_matrix_instance_f32* x_plus,
				arm_matrix_instance_f32* P_plus, float32_t xPlusData[22*1], float32_t P_plus_data[21*21]);

void update_mag(const arm_matrix_instance_f32* x_minus, const arm_matrix_instance_f32* P_minus, const arm_matrix_instance_f32* R,
				const arm_matrix_instance_f32* magI, const arm_matrix_instance_f32* mag_meas, arm_matrix_instance_f32* x_plus,
				arm_matrix_instance_f32* P_plus, float32_t x_plus_buff[22*1], float32_t P_plus_buff[21*21]);

void update_baro(const arm_matrix_instance_f32* xMinus, const arm_matrix_instance_f32* PMinus, const float32_t pressMeas,
		 	 	 const float32_t Rb, arm_matrix_instance_f32* xPlus, arm_matrix_instance_f32* Pplus,
				 float32_t xPlusData[22*1], float32_t pPlusData[21*21]);

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
				uint32_t* vdStart,
				uint32_t* altStart,
				uint32_t* altStart2,
				reco_message* message,
				fc_message* fcData,
				bool* stage1Enabled,
				bool* stage2Enabled,
				bool launched);

void nearestPSD(arm_matrix_instance_f32* P,
                arm_matrix_instance_f32* PCorrect,
                float32_t PCorrData[21*21]);

#endif /* EKF_INC_EKF_H_ */
