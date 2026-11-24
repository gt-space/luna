/*
 * Main propeller ESC output: hardware-independent logic.
 *
 * One pulse-width signal (PA5) drives both ESCs through a Y-harness, so they
 * share throttle, arming and endpoints. This file turns the EC's main
 * throttle (0..1) into the pulse width to load at the next frame, holding the
 * safe (minimum) pulse until the arming hold has passed and whenever the EC
 * isn't driving the channel. It never touches hardware: the board adapter
 * loads the returned pulse into its timer and calls fcec_esc_service() once
 * per loop.
 *
 * Every ESC-specific number is in fcec_esc_config_t (the bench profile). A
 * profile that isn't marked approved, or isn't self-consistent, leaves the
 * ESC unconfigured: no capability, no readiness, and the adapter should not
 * start the signal at all.
 *
 * A pulse on the wire confirms the electrical command only. It says nothing
 * about rotation or thrust, or whether each ESC responded.
 */

#ifndef FCEC_ESC_H
#define FCEC_ESC_H

#include "ec_side.h"

typedef struct {
    bool approved;            /* reviewed bench profile; false = don't drive */
    uint16_t min_pulse_us;    /* throttle 0, and the safe output */
    uint16_t max_pulse_us;    /* throttle 1 */
    uint16_t frame_period_us; /* one pulse per frame */
    uint32_t arm_hold_ms;     /* safe pulse this long after start before throttle */
    float max_throttle;       /* approved cap, 0 < cap <= 1 */
    float max_rise_per_s;     /* throttle increase per second; 0 = no limit; cuts are immediate */
} fcec_esc_config_t;

typedef enum {
    FCEC_ESC_UNCONFIGURED = 0, /* no approved, valid profile */
    FCEC_ESC_ARMING = 1,       /* safe pulse, waiting out arm_hold_ms */
    FCEC_ESC_READY = 2,        /* follows the EC command */
    FCEC_ESC_FAULT = 3         /* safe pulse until fcec_esc_clear_fault() */
} fcec_esc_state_t;

typedef struct {
    fcec_esc_config_t config;
    fcec_esc_state_t state;
    uint64_t started_us;     /* when the safe pulse started */
    uint64_t last_update_us;
    bool have_update;
    float throttle;          /* applied throttle, after cap and slew */
    uint16_t pulse_us;       /* pulse for the next frame */
    bool have_gen;           /* EC command being output, for confirmation */
    uint32_t gen;
    uint64_t gen_since_us;
} fcec_esc_t;

/* Whether `config` is approved and self-consistent. */
bool fcec_esc_config_ok(const fcec_esc_config_t *config);

/* Starts the safe pulse at `now_us` (UNCONFIGURED if the profile isn't OK). */
void fcec_esc_init(fcec_esc_t *esc, const fcec_esc_config_t *config, uint64_t now_us);

/* One update: `target` is the commanded throttle, `drive` whether the EC is
 * driving the ESC now. Returns the pulse width for the next frame. */
uint16_t fcec_esc_update(fcec_esc_t *esc, float target, bool drive, uint64_t now_us);

/* The adapter or a monitor found a problem: safe pulse, not ready. */
void fcec_esc_fault(fcec_esc_t *esc);

/* Back to ARMING (a fresh arm hold) after the cause is fixed. */
void fcec_esc_clear_fault(fcec_esc_t *esc, uint64_t now_us);

bool fcec_esc_ready(const fcec_esc_t *esc);

/* Reads the EC command, updates the ESC, tells the EC whether the ESC is
 * ready and confirms what it output. Returns the pulse to load for the next
 * frame. Report a fault with fcec_esc_fault() and fcec_ec_report_fault(). */
uint16_t fcec_esc_service(fcec_esc_t *esc, fcec_ec_t *ec, uint64_t now_us);

#endif /* FCEC_ESC_H */
