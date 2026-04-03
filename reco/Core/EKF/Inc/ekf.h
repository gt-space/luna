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
#include "performance.h"
#include "main.h"

#define CREATE_MAT_F32(name, rows, cols)              \
    float32_t name##Data[(rows) * (cols)] = {0};             \
    arm_matrix_instance_f32 name;                            \
    arm_mat_init_f32(&(name), (rows), (cols), name##Data);

#define SPEEDOFSOUND 295.069495691898f

// CORDIC DEFINES

// hOffset is the height offset to account for bias in the barometer (in meters)

extern const float32_t dh;
extern const float32_t we;

extern volatile atomic_uchar safeToWrite;
extern volatile atomic_uchar gpsEventCount;
extern volatile atomic_uchar magEventCount;
extern volatile atomic_uchar baroEventCount;

// Parachute Logic
bool drougeChuteCheck(float32_t altNow, uint32_t* altStart, uint32_t currentTime);
bool mainChuteCheck(float32_t altNow, uint32_t* altStart, uint32_t currentTime);

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
void setHeightOffsetAltimeter(float32_t newHOffset);
void setHeightOffsetFilter(float32_t new_h_offset_filter);

// Matrix Initialization Code
void ekf_init(arm_matrix_instance_f32* xPrev,
			  arm_matrix_instance_f32* PPrev,
			  arm_matrix_instance_f32* H,
			  arm_matrix_instance_f32* Hb,
			  arm_matrix_instance_f32* R,
			  arm_matrix_instance_f32* Rq,
			  arm_matrix_instance_f32* Q,
			  arm_matrix_instance_f32* magI,
			  float32_t* Rb,
			  float32_t dt);

void compute_P0(void);
void compute_Q0(float32_t);
void compute_magI(arm_matrix_instance_f32* magI, float32_t magIBuff[3]);

void initialize_Hb(arm_matrix_instance_f32* x,
				   arm_matrix_instance_f32* Hb,
				   float32_t HbBuff[1*21]);

// Setter Functions for Initial Matrices/States
void set_uncertanties(float32_t new_att_unc[3],
					  float32_t new_pos_unc[3],
					  float32_t new_vel_unc[3],
					  float32_t new_gbias_unc[3],
					  float32_t new_abias_unc[3],
					  float32_t new_gsf_unc[3],
					  float32_t new_asf_unc[3]);

void set_x0(float32_t new_initial_state[22*1]);
void set_nu_gv0(float32_t nu_gv_set[9]);
void set_nu_gu0(float32_t nu_gu_set[9]);
void set_nu_av0(float32_t nu_av_set[9]);
void set_nu_au0(float32_t nu_au_set[9]);
void set_R0(float32_t R_set[9]);
void set_Rq0(float32_t Rq_set[9]);
void set_Rb0(float32_t Rb_set);

// Getter Functions for Initial Matrices/States
void get_x0(arm_matrix_instance_f32* x);
void get_P0(arm_matrix_instance_f32* P);
void get_Q0(arm_matrix_instance_f32* Q);
void get_R0(arm_matrix_instance_f32* R);
void get_Rq0(arm_matrix_instance_f32* Rq);
void get_H(arm_matrix_instance_f32* H, float32_t HBuff[3*21]);
float32_t get_Rb0();

// EKF Code
void compute_wn(float32_t phi, float32_t h, float32_t vn, float32_t ve,
				arm_matrix_instance_f32* wn, float32_t we, float32_t buffer[3] PERF_ARG);

void compute_what(arm_matrix_instance_f32* q, arm_matrix_instance_f32* bias_g, arm_matrix_instance_f32* sf_g,
				  float32_t phi, float32_t h, float32_t vn, float32_t ve, float32_t we, arm_matrix_instance_f32* w_meas,
				  arm_matrix_instance_f32* what, float32_t whatBuffer[3] PERF_ARG);

void compute_ahat(arm_matrix_instance_f32* q, arm_matrix_instance_f32* sf_a, arm_matrix_instance_f32* bias_a,
				  arm_matrix_instance_f32* a_meas, arm_matrix_instance_f32* ahat_n, float32_t ahatBuff[3] PERF_ARG);

void compute_qdot(arm_matrix_instance_f32* x, arm_matrix_instance_f32* what,
				  arm_matrix_instance_f32* qdot, float32_t qDotBuff[4] PERF_ARG);

void compute_lla_dot(float32_t phi, float32_t h, float32_t vn, float32_t ve, float32_t vd,
					 arm_matrix_instance_f32* llaDot, float32_t llaDotBuff[3] PERF_ARG);

void compute_vdot(float32_t phi, float32_t h, float32_t vn, float32_t ve, float32_t vd,
				  float32_t ahat_n[3], float32_t we, arm_matrix_instance_f32* vdot, float32_t vdotBuff[3] PERF_ARG);

void compute_dpdot_dp(float32_t phi, float32_t h, float32_t vn,
					  float32_t ve, arm_matrix_instance_f32* dpdot_dp, float32_t dpDotBuff[9] PERF_ARG);

void compute_dpdot_dv(float32_t phi, float32_t h,
					  arm_matrix_instance_f32* dpdot_dv, float32_t dpDotBuff[9] PERF_ARG);

void compute_dvdot_dp(float32_t phi, float32_t h, float32_t vn, float32_t ve, float32_t vd,
					  float32_t we,  arm_matrix_instance_f32* dvdot_dp, float32_t dvdotBuff[9] PERF_ARG);

void compute_dvdot_dv(float32_t phi, float32_t h, float32_t vn, float32_t ve, float32_t vd,
					  float32_t we, arm_matrix_instance_f32* dvdot_dv, float32_t dvdotBuff[9] PERF_ARG);

void compute_dwdp(float32_t phi, float32_t h, float32_t ve, float32_t vn, float32_t we,
				  arm_matrix_instance_f32* dwdp, float32_t dwdpBuffer[9] PERF_ARG);

void compute_dwdv(float32_t phi, float32_t h, arm_matrix_instance_f32* dwdv, float32_t dwdvBuffer[9] PERF_ARG);

void compute_F(arm_matrix_instance_f32* q, arm_matrix_instance_f32* sf_a, arm_matrix_instance_f32* sf_g,
			  arm_matrix_instance_f32* bias_g, arm_matrix_instance_f32* bias_a, float32_t phi, float32_t h,
			  float32_t vn, float32_t ve, float32_t vd, arm_matrix_instance_f32* a_meas, arm_matrix_instance_f32* w_meas,
			  float32_t we, arm_matrix_instance_f32* F, float32_t FBuff[21*21] PERF_ARG);

void compute_G(arm_matrix_instance_f32* sf_g, arm_matrix_instance_f32* sf_a, arm_matrix_instance_f32* q,
			   arm_matrix_instance_f32* G, float32_t GBuff[21*12] PERF_ARG);

void compute_Pdot(arm_matrix_instance_f32* q, arm_matrix_instance_f32* sf_a, arm_matrix_instance_f32* sf_g,
		  	  	  arm_matrix_instance_f32* bias_g, arm_matrix_instance_f32* bias_a, arm_matrix_instance_f32* a_meas,
				  arm_matrix_instance_f32* w_meas, arm_matrix_instance_f32* P, arm_matrix_instance_f32* Q,
				  float32_t phi, float32_t h, float32_t vn, float32_t ve, float32_t vd, float32_t we,
				  arm_matrix_instance_f32* Pdot, float32_t PdotBuff[21*21] PERF_ARG);

void integrate(arm_matrix_instance_f32* x, arm_matrix_instance_f32* P, arm_matrix_instance_f32* qdot,
			   arm_matrix_instance_f32* pdot, arm_matrix_instance_f32* vdot, arm_matrix_instance_f32* Pdot,
			   float32_t dt, arm_matrix_instance_f32* xMinus, arm_matrix_instance_f32* Pminus,
			   float32_t xMinusBuff[22], float32_t PMinusBuff[21*21] PERF_ARG);

void propogate(arm_matrix_instance_f32* xPlus, arm_matrix_instance_f32* Pplus, arm_matrix_instance_f32* what,
		   	   arm_matrix_instance_f32* aHatN, arm_matrix_instance_f32* wMeas, arm_matrix_instance_f32* aMeas,
			   arm_matrix_instance_f32* Q, float32_t dt, float32_t we,
			   arm_matrix_instance_f32* xMinus, arm_matrix_instance_f32* PMinus, float32_t xMinusBuff[22],
			   float32_t PMinusBuff[21*21] PERF_ARG);

void update_GPS(const arm_matrix_instance_f32* x_minus, const arm_matrix_instance_f32* P_minus, const arm_matrix_instance_f32* H,
				const arm_matrix_instance_f32* R, const arm_matrix_instance_f32* lla_meas, arm_matrix_instance_f32* x_plus,
				arm_matrix_instance_f32* P_plus, float32_t xPlusData[22*1], float32_t P_plus_data[21*21] PERF_ARG);

void update_mag(const arm_matrix_instance_f32* x_minus, const arm_matrix_instance_f32* P_minus, const arm_matrix_instance_f32* Rq,
				const arm_matrix_instance_f32* magI, const arm_matrix_instance_f32* mag_meas, arm_matrix_instance_f32* x_plus,
				arm_matrix_instance_f32* P_plus, float32_t x_plus_buff[22*1], float32_t P_plus_buff[21*21] PERF_ARG);

void update_baro(const arm_matrix_instance_f32* xMinus, const arm_matrix_instance_f32* PMinus, const float32_t pressMeas,
		 	 	 const float32_t Rb, arm_matrix_instance_f32* xPlus, arm_matrix_instance_f32* Pplus,
				 float32_t xPlusData[22*1], float32_t pPlusData[21*21] PERF_ARG);

void update_baro_new(const arm_matrix_instance_f32* xMinus, const arm_matrix_instance_f32* PMinus, const float32_t pressMeas,
				 	 const float32_t Rb, arm_matrix_instance_f32* xPlus, arm_matrix_instance_f32* Pplus,
					 float32_t xPlusData[22*1], float32_t pPlusData[21*21] PERF_ARG);

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
				uint32_t numIterations
				PERF_ARG);

void nearestPSD(arm_matrix_instance_f32* P,
                arm_matrix_instance_f32* PCorrect,
                float32_t PCorrData[21*21]
				PERF_ARG);



#endif /* EKF_INC_EKF_H_ */
