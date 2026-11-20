#ifndef EC_BENCH_PROFILE_H
#define EC_BENCH_PROFILE_H

/* The CTV bench profile: every hardware number the EC needs, in one place.
 * Included by ec_app.c only.
 *
 * Nothing drives hardware until its block below is filled in from confirmed
 * values, reviewed, and its `approved` flag set to true. An unapproved block
 * keeps that output off and its channel isn't offered to the FC.
 *
 * Change EC_BENCH_PROFILE_ID (and the FC's matching constant) whenever any
 * value here changes: Arm is refused if the FC expects a different profile.
 */

#include "esc.h"
#include "pa12_drv.h"

/* ---- Profile identity, checked at Arm (FC: flight2 ec_link.rs) ---------- */
/* 32 bytes, sent as the "bundle SHA-256" field. Bump the trailing number on
 * every change. */
#define EC_BENCH_PROFILE_ID "ctv-bench-profile-v1"

/* ---- TVC (PA-12, A1 = UART7, A2 = UART5) --------------------------------- */
/* Park position (mm of extension, used only if safe_action is SAFE_PARK) and
 * the travel window the EC clamps every demand to (mm). Until neutral and
 * travel are trimmed (--ec-bench `jog` + `pos`), the window is the full stroke and
 * each actuator's own count limits below do the limiting. */
#define EC_TVC_PARK_MM_A1 13.45f
#define EC_TVC_PARK_MM_A2 13.45f
#define EC_TVC_MIN_MM 0.0f
#define EC_TVC_MAX_MM 26.9f

/* Per actuator, from its bench calibration (ID/baud scan and caliper points
 * with the EC_PA12_BENCH build, see pa12_bench.h):
 * counts = counts_per_mm * mm + counts_at_0mm; hard limits inside the stroke
 * limits read from the actuator (min_counts/max_counts = the actuator's
 * short/long stroke limits until travel is trimmed). safe_action on safing:
 * SAFE_HOLD (stop where it is), SAFE_PARK (go to park mm), SAFE_FORCE_OFF. */
static const fcec_pa12_profile_t ec_pa12_profile[FCEC_PA12_PORTS] = {
    { /* A1, UART7 */
        .approved = false,
        .id = 0U,                 /* TBD: scanned ID */
        .counts_per_mm = 137.0f,  /* TBD: measured (nominal 3686 / 26.9) */
        .counts_at_0mm = 0.0f,    /* TBD: measured */
        .min_counts = 0U,         /* TBD: inside short stroke limit */
        .max_counts = 3686U,      /* TBD: inside long stroke limit */
        .safe_action = FCEC_PA12_SAFE_HOLD, /* stop where it is */
        .poll_period_ms = 20U,
        .reply_timeout_ms = 15U,
    },
    { /* A2, UART5 */
        .approved = false,
        .id = 0U,
        .counts_per_mm = 137.0f,
        .counts_at_0mm = 0.0f,
        .min_counts = 0U,
        .max_counts = 3686U,
        .safe_action = FCEC_PA12_SAFE_HOLD, /* stop where it is */
        .poll_period_ms = 20U,
        .reply_timeout_ms = 15U,
    },
};

/* ---- Main ESC (PA5 / TIM2_CH1, both ESCs on one Y-harness) --------------- */
/* ZTW Mantis G2 35A x2, standard servo PWM (from the manual):
 * - The ESC learns its range in throttle calibration (full pulse at battery
 *   connect, two beeps, then stop); 1000/2000 us only work if it was
 *   calibrated to them. Uncalibrated, it beeps but won't run.
 * - It arms only on the stop pulse (else it beeps every 200 ms). Full at
 *   power-up enters calibration, held 2 + 3 s enters programming mode, so the
 *   EC never outputs more than min_pulse_us until armed and ready.
 * - Signal loss: it reduces power after 1 s and cuts after 2 s, so the EC's
 *   immediate stop pulse on link loss / disarm / fault is the fast path.
 * - Start-up protection: if the motor doesn't start within 2 s of throttle
 *   up, it cuts out and needs recalibration.
 * - arm_hold_ms counts from EC boot. Power the EC before the ESC battery, and
 *   don't Arm until the ESCs' arming beeps have finished.
 * Keep it unapproved until both ESCs are confirmed calibrated to 1000-2000 us
 * (the EscCalibration build does it). */
static const fcec_esc_config_t ec_esc_profile = {
    .approved = false,
    .min_pulse_us = 1000U,     /* stop, and the safe output */
    .max_pulse_us = 2000U,     /* full throttle (calibrated endpoint) */
    .frame_period_us = 20000U, /* 50 Hz */
    .arm_hold_ms = 5000U,      /* stop pulse >= 5 s (self-test ~2 s + beeps) */
    .max_throttle = 1.0f,      /* up to 100% */
    .max_rise_per_s = 0.0f     /* 0 = no slew limit */
};

#endif /* EC_BENCH_PROFILE_H */
