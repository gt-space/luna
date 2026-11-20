/*
 * PA-12 TVC actuator driver: hardware-independent logic for the two
 * half-duplex TTL ports (A1 = UART7, A2 = UART5).
 *
 * Each port has its own state machine, so a slow or silent actuator never
 * holds up the other one or the link. Per port, highest priority first:
 *   1. the safing action, once per safing (see fcec_pa12_safe_action_t)
 *   2. the newest goal from the EC command (mm -> counts from the profile)
 *   3. a position read every `poll_period_ms`
 * with the PA-12 quiet times (5 ms after a write, 10 ms after a read).
 *
 * Goal writes get no reply in the default feedback mode, so a goal counts as
 * applied (fcec_ec_confirm_applied) once its last byte is on the wire
 * ("sent, unconfirmed"). A separate position read shows whether the actuator
 * moved. A port is ready once a valid read reply arrived recently; a reply
 * with hardware error flags reports a driver fault to the EC.
 *
 * The board adapter supplies `start_tx` (non-blocking; it drives the
 * direction pin and calls fcec_pa12_tx_done() from the main loop once the
 * final byte has left the wire) and feeds received bytes with
 * fcec_pa12_rx_byte(). All calls happen in the main loop, never in an ISR.
 */

#ifndef FCEC_PA12_DRV_H
#define FCEC_PA12_DRV_H

#include "ec_side.h"

#define FCEC_PA12_PORTS 2u
#define FCEC_PA12_WRITE_GAP_US 5000u
#define FCEC_PA12_READ_GAP_US 10000u
#define FCEC_PA12_TX_TIMEOUT_US 20000u
#define FCEC_PA12_READY_WINDOW_US 500000u
#define FCEC_PA12_MAX_COUNTS 4095u

/* What a port does when the EC safes. */
typedef enum {
    FCEC_PA12_SAFE_HOLD = 0,  /* stop where it is: one goal at the last measured
                                 position (if any), then no goals */
    FCEC_PA12_SAFE_PARK = 1,  /* go to the EC's park position (tvc_center_mm) */
    FCEC_PA12_SAFE_FORCE_OFF = 2 /* motor off, held by friction; no goals */
} fcec_pa12_safe_action_t;

/* One actuator's bench profile. counts = counts_per_mm * mm + counts_at_0mm,
 * from a measured calibration. */
typedef struct {
    bool approved;            /* reviewed values; false = port unused */
    uint8_t id;               /* PA-12 ID (factory 0) */
    float counts_per_mm;      /* measured slope, about 137 nominal */
    float counts_at_0mm;      /* measured offset */
    uint16_t min_counts;      /* hard limits, inside the actuator's stroke limits */
    uint16_t max_counts;
    fcec_pa12_safe_action_t safe_action;
    uint16_t poll_period_ms;  /* position read period */
    uint16_t reply_timeout_ms;
} fcec_pa12_profile_t;

typedef struct {
    void *context;
    /* Starts sending `len` bytes on `port` (0 = A1, 1 = A2). Must not block.
     * Returns false if it couldn't start. */
    bool (*start_tx)(void *context, unsigned port, const uint8_t *bytes, size_t len);
} fcec_pa12_ops_t;

typedef enum {
    FCEC_PA12_TX_NONE = 0,
    FCEC_PA12_TX_GOAL,
    FCEC_PA12_TX_READ,
    FCEC_PA12_TX_FORCE_OFF
} fcec_pa12_tx_kind_t;

typedef enum {
    FCEC_PA12_PHASE_IDLE = 0,
    FCEC_PA12_PHASE_TX,         /* waiting for fcec_pa12_tx_done() */
    FCEC_PA12_PHASE_WAIT_REPLY  /* read sent, collecting its reply */
} fcec_pa12_phase_t;

typedef struct {
    fcec_pa12_profile_t profile;
    bool configured;          /* profile approved and valid */

    fcec_pa12_phase_t phase;
    fcec_pa12_tx_kind_t tx_kind;
    uint64_t deadline_us;
    uint64_t quiet_until_us;
    uint64_t next_poll_us;
    uint8_t tx[16];
    uint8_t rx[16];
    uint8_t rx_len;

    bool goal_pending;        /* the current command's goal hasn't been sent */
    bool have_goal_gen;
    uint32_t goal_gen;        /* EC command the pending/sent goal belongs to */
    uint16_t goal_counts;
    uint32_t sent_gen;        /* command whose goal is on the wire */
    uint16_t sent_counts;
    bool have_sent_gen;
    bool force_off_pending;
    bool safe_goal_pending;   /* SAFE_HOLD: goal at the last measured position */
    uint16_t safe_goal_counts;
    uint32_t safe_goal_gen;
    bool safe_done;           /* safing action taken for the current safing */

    /* Feedback for the v4 Status. */
    uint32_t command_request_id;
    uint8_t command_state;    /* fcec_pa12_command_state_t */
    uint32_t feedback_request_id;
    uint8_t feedback_state;   /* fcec_pa12_feedback_state_t */
    bool have_position;
    uint16_t position_counts;
    float position_mm;
    uint16_t fault_flags;
    uint64_t position_time_us;
    uint64_t last_valid_reply_us;
    bool have_valid_reply;
    bool ready;
    bool faulted;
    uint32_t timeouts;
    uint32_t bad_replies;
} fcec_pa12_port_t;

typedef struct {
    fcec_pa12_port_t port[FCEC_PA12_PORTS];
    fcec_pa12_ops_t ops;
} fcec_pa12_t;

/* Whether `p` is approved and self-consistent. */
bool fcec_pa12_profile_ok(const fcec_pa12_profile_t *p);

/* Sets up both ports. A port with a bad or unapproved profile stays unused:
 * its channel isn't offered and nothing is ever sent on it. Returns the
 * FCEC_CH_TVC_* bits that are configured (add them to ec.capabilities). */
uint16_t fcec_pa12_init(fcec_pa12_t *drv, const fcec_pa12_profile_t profiles[FCEC_PA12_PORTS],
                        const fcec_pa12_ops_t *ops, uint64_t now_us);

/* Calibration: false if `mm` isn't finite or lands outside the hard limits
 * (then *counts is the clamped value). */
bool fcec_pa12_mm_to_counts(const fcec_pa12_profile_t *p, float mm, uint16_t *counts);
float fcec_pa12_counts_to_mm(const fcec_pa12_profile_t *p, uint16_t counts);

/* From the adapter, main loop only. */
void fcec_pa12_tx_done(fcec_pa12_t *drv, unsigned port, bool ok, uint64_t now_us);
void fcec_pa12_rx_byte(fcec_pa12_t *drv, unsigned port, uint8_t byte, uint64_t now_us);

/* Once per loop: reads the EC command, starts at most one transfer per idle
 * port, expires timeouts, and updates readiness/faults in the EC. */
void fcec_pa12_service(fcec_pa12_t *drv, fcec_ec_t *ec, uint64_t now_us);

/* Writes positions, faults, request IDs and states into the Status
 * measurements. */
void fcec_pa12_publish(const fcec_pa12_t *drv, fcec_measurements_t *m, uint64_t now_us);

#endif /* FCEC_PA12_DRV_H */
