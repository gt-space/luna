#ifndef COMPUTE_H_
#define COMPUTE_H_

#include "compute_common.h"

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
