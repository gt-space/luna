#ifndef _COMPUTE
#define _COMPUTE

#include "common.h"
#include "compute_common.h"
#include "cmsis_dsp_extensions\matrix_extensions.h"
#include "cmsis_dsp_extensions\quaternion_extensions.h"
#include "cmsis_dsp_extensions\trig_extensions.h"

void propogate(void);
void compute_qdot(void);
void compute_lla_dot(void);
void compute_Pdot(void);
void compute_F(void);
void compute_dwdv(void);
void compute_dpdot_dp(void);
void compute_dpdot_dv(void);
void compute_dvdot_dp(void);
void compute_dvdot_dv(void);
void compute_G(void);
void compute_Pqdot(void);
void integrate(void);
void update_GPS(void);
void update_mag(void);
void update_baro(void);

#endif