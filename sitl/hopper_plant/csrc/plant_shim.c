/*
 * plant_shim.c - flat C API over the generated hopper_6dof plant.
 *
 * Rust calls these instead of touching hopper_6dof_U / _Y / _M directly,
 * so the Rust side doesn't depend on the generated struct layouts.
 */
#include <string.h>

#include "hopper_6dof.h"

void plant_initialize(void) { hopper_6dof_initialize(); }

/* Back to t = 0 exactly as at program start. The generated initialize()
 * only sets what C's zero-initialized globals don't already give it: it
 * leaves the model's time, rate counters and stop flag alone, so calling
 * it again after a run starts from the old time with touchdown still set.
 * Zeroing the model's data first reproduces a fresh start (the random
 * streams are reseeded by initialize()). */
void plant_reset(void)
{
  hopper_6dof_terminate();
  memset(hopper_6dof_M, 0, sizeof(*hopper_6dof_M));
  memset(&hopper_6dof_B, 0, sizeof(hopper_6dof_B));
  memset(&hopper_6dof_X, 0, sizeof(hopper_6dof_X));
  memset(&hopper_6dof_XDis, 0, sizeof(hopper_6dof_XDis));
  memset(&hopper_6dof_DW, 0, sizeof(hopper_6dof_DW));
  memset(&hopper_6dof_U, 0, sizeof(hopper_6dof_U));
  memset(&hopper_6dof_Y, 0, sizeof(hopper_6dof_Y));
  hopper_6dof_initialize();
}

/* Outputs at the current time; read them with plant_get_x. */
void plant_output(void) { hopper_6dof_output(); }

/* Advance one fixed step with the command set by plant_set_u. */
void plant_update(void) { hopper_6dof_update(); }

void plant_terminate(void) { hopper_6dof_terminate(); }

double plant_time(void) { return rtmGetT(hopper_6dof_M); }

/* The harness's touchdown stop. The model keeps stepping if called. */
int plant_stop_requested(void) { return rtmGetStopRequested(hopper_6dof_M) ? 1 : 0; }

const char *plant_error(void) { return rtmGetErrorStatus(hopper_6dof_M); }

/* u = [thrust N, TVC pitch rad, TVC yaw rad, RCS torque N m] */
void plant_set_u(const double u[4])
{
  for (int i = 0; i < 4; i++) {
    hopper_6dof_U.u_fsw[i] = u[i];
  }
}

/* x = [pos NED m (3), vel NED m/s (3), body rates rad/s (3), quaternion (4)] */
void plant_get_x(double x[13])
{
  for (int i = 0; i < 13; i++) {
    x[i] = hopper_6dof_Y.x_fsw[i];
  }
}
