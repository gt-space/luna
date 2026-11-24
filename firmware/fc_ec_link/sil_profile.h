#ifndef SIL_PROFILE_H
#define SIL_PROFILE_H

/* ======================================================================== *
 *  SIL-ONLY PROFILE.  EVERY NUMBER IN THIS FILE IS FAKE.                    *
 * ======================================================================== *
 *
 * This is the profile of the software-in-the-loop engine controller
 * (sil_ec.c). It exists ONLY so the real EC firmware logic (ec_side.c,
 * esc.c, pa12_drv.c) can be exercised on a Linux/WSL host against a
 * simulated vehicle. It is NOT a hardware profile:
 *
 *   - It is NOT the bench profile. The real bench profile is
 *     firmware/ctv_engine_ctrl/CM7/Core/Inc/ec_bench_profile.h and it stays
 *     `approved = false` until each block is filled in from measured,
 *     reviewed values. This file never replaces it and must NEVER be
 *     compiled into the STM32 firmware (nothing in the CubeIDE/CMake build
 *     includes it).
 *   - `approved = true` below only means "the simulated actuators and the
 *     simulated ESC are present and willing". It says nothing about any
 *     physical PA-12, ESC, calibration or travel window.
 *   - The geometry is the NOMINAL datasheet number (stroke 26.9 mm,
 *     137 counts/mm, 0 counts at 0 mm), not a measurement of a real
 *     actuator. The ESC numbers (1000-2000 us, 20 ms frame) are the generic
 *     servo-PWM numbers, not a calibration of the real ZTW Mantis ESCs.
 *   - Do not copy values from here into ec_bench_profile.h, and do not
 *     treat anything flown or bench-tested against SIL as evidence that a
 *     real profile is correct.
 *
 * Profile identity: the ID below is deliberately the SAME string as the real
 * EC_BENCH_PROFILE_ID ("ctv-bench-profile-v1") so that the unmodified FC
 * (flight2/src/ec_link.rs BENCH_PROFILE_ID = profile_id(b"ctv-bench-profile-v1"))
 * can Hello/Arm against sil_ec without any change. The EC and the FC both
 * zero-pad the ASCII string to the 32-byte "bundle SHA-256" field (see
 * ec_app.c: memset + memcpy of the string), and sil_ec does the same. A SIL
 * EC is therefore NOT distinguishable from the real one by the bundle check;
 * it reports firmware_version SIL_FIRMWARE_VERSION instead (the real EC
 * reports 1), and prints "sil_ec" in its boot line.
 */

#include "esc.h"
#include "pa12_drv.h"

#define SIL_PROFILE_ID "ctv-bench-profile-v1" /* same bytes as the real profile ID */
#define SIL_FIRMWARE_VERSION 0x53494C01u      /* "SIL" + 1; the real EC reports 1 */

/* ---- TVC (simulated PA-12s, A1 = port 0, A2 = port 1) ------------------- */
/* Same structure as ec_bench_profile.h: park position, travel window. FAKE. */
#define SIL_TVC_PARK_MM_A1 13.45f
#define SIL_TVC_PARK_MM_A2 13.45f
#define SIL_TVC_MIN_MM 0.0f
#define SIL_TVC_MAX_MM 26.9f /* nominal stroke */

/* counts = counts_per_mm * mm + counts_at_0mm. NOMINAL, not measured. The
 * simulated device (sil_ec.c) uses these same numbers to turn counts into
 * millimetres, so the SIL calibration is exactly consistent by construction
 * (a real actuator's would not be). */
static const fcec_pa12_profile_t sil_pa12_profile[FCEC_PA12_PORTS] = {
    { /* A1, simulated */
        .approved = true,          /* FAKE: simulated hardware is "approved" */
        .id = 1U,                  /* FAKE: the simulated device answers to this */
        .counts_per_mm = 137.0f,   /* FAKE: nominal 3686 / 26.9 */
        .counts_at_0mm = 0.0f,     /* FAKE */
        .min_counts = 0U,          /* FAKE */
        .max_counts = 3686U,       /* FAKE: nominal full stroke */
        .safe_action = FCEC_PA12_SAFE_HOLD, /* same default as the real profile */
        .poll_period_ms = 20U,     /* same as the real profile */
        .reply_timeout_ms = 15U,   /* same as the real profile */
    },
    { /* A2, simulated */
        .approved = true,
        .id = 2U,
        .counts_per_mm = 137.0f,
        .counts_at_0mm = 0.0f,
        .min_counts = 0U,
        .max_counts = 3686U,
        .safe_action = FCEC_PA12_SAFE_HOLD,
        .poll_period_ms = 20U,
        .reply_timeout_ms = 15U,
    },
};

/* ---- Main ESC (simulated; one shared pulse for both ESCs) --------------- */
/* Generic servo PWM, NOT a calibration of the real ESCs. arm_hold_ms is
 * shorter than the real profile's 5000 ms so SIL runs start quickly; use
 * `sil_ec --esc-arm-hold-ms 5000` for the real value. */
static const fcec_esc_config_t sil_esc_profile = {
    .approved = true,          /* FAKE */
    .min_pulse_us = 1000U,     /* stop, and the safe output (generic) */
    .max_pulse_us = 2000U,     /* full throttle (generic) */
    .frame_period_us = 20000U, /* 50 Hz, as the real profile */
    .arm_hold_ms = 1000U,      /* SIL default; real profile: 5000 */
    .max_throttle = 1.0f,      /* as the real profile */
    .max_rise_per_s = 0.0f     /* no slew limit, as the real profile */
};

#endif /* SIL_PROFILE_H */
