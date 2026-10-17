/*
 * EC side of the FC <-> EC link: sessions, arming, freshness, timeouts,
 * duplicate transactions and safe outputs.
 *
 * No sockets and no clock: the caller passes in each received datagram and
 * the current EC time, and sends whatever this hands back. The same code runs
 * in the laptop fake EC (fake_ec.c) and is meant to run in the EC firmware
 * behind lwIP.
 *
 * Typical loop:
 *     fcec_ec_init(&ec, random_boot_id);
 *     ... bring up drivers ...
 *     fcec_ec_boot_done(&ec);
 *     for (;;) {
 *         for each received datagram:
 *             n = fcec_ec_receive(&ec, rx, rx_len, now_us, reply, sizeof reply);
 *             if (n > 0) send reply[0..n] back to the sender;
 *         fcec_ec_tick(&ec, now_us);
 *         drive the hardware from *fcec_ec_outputs(&ec);
 *         every 10 ms: fill ec.measurements, then
 *             n = fcec_ec_status(&ec, now_us, tx, sizeof tx); send it to the FC;
 *     }
 *
 * Not in this bare-bones version: the config bundle check on Arm (skipped for
 * the sprint) and a transaction queue (transactions are handled immediately).
 */

#ifndef EC_SIDE_H
#define EC_SIDE_H

#include "fc_ec_link.h"

#define FCEC_EC_TXN_HISTORY 8u
#define FCEC_EC_MAX_DEMAND_AGE_MS 50u
#define FCEC_EC_LINK_TIMEOUT_MS 350u
#define FCEC_EC_DEFAULT_DEMAND_PERIOD_MS 10u
#define FCEC_EC_DEMAND_TIMEOUT_PERIODS 3u

typedef struct {
    uint32_t txn_id;
    fcec_txn_result_t result;
} fcec_ec_txn_t;

typedef struct {
    /* Settings. fcec_ec_init() sets defaults; change them before the first
     * datagram if needed. */
    uint16_t capabilities;      /* FCEC_CH_* bits this EC can drive */
    uint16_t max_demand_age_ms; /* older Demand/Arm/ClearSafing are stale */
    uint16_t link_timeout_ms;   /* no valid FC message this long -> safing */
    uint32_t firmware_version;
    uint32_t calibration_id;
    float tvc_center_mm[2]; /* where safing parks the TVC actuators */

    /* Filled in by the firmware; sent in every Status. */
    fcec_measurements_t measurements;
    uint8_t reset_cause;
    fcec_counters_t counters; /* the firmware may bump deadline_miss */

    /* State. Read it, don't write it. */
    uint32_t boot_id;
    fcec_ec_state_t state;
    fcec_reason_t safing_reason;
    bool session_valid;
    uint32_t session_id;
    uint16_t demand_period_ms;
    uint32_t tx_seq;
    uint64_t last_rx_us;     /* last valid FC message in this session */
    uint64_t demand_due_us;  /* demand timer start: arm time or last demand */
    bool have_demand;
    uint64_t last_demand_us; /* when the newest demand was accepted */
    uint32_t last_received_seq;
    uint32_t last_accepted_demand_seq;
    uint32_t last_applied_demand_seq;
    uint64_t last_applied_time_us;
    fcec_reason_t last_reject_reason;
    fcec_demand_t demand;    /* newest accepted demand */
    fcec_outputs_t applied;  /* what the hardware should output now */
    bool have_txn;
    uint32_t newest_txn_id;
    fcec_ec_txn_t txns[FCEC_EC_TXN_HISTORY];
    uint32_t txn_count;
} fcec_ec_t;

/* Starts in BOOT_SAFE with safe outputs and no session. `boot_id` should be
 * random and nonzero, different on every boot. */
void fcec_ec_init(fcec_ec_t *ec, uint32_t boot_id);

/* Call once the drivers are up: BOOT_SAFE -> DISARMED. */
void fcec_ec_boot_done(fcec_ec_t *ec);

/* Handles one received datagram at EC time `now_us`. If it needs an answer,
 * writes it to `reply` and returns its length; otherwise returns 0. */
size_t fcec_ec_receive(fcec_ec_t *ec, const uint8_t *buf, size_t len, uint64_t now_us,
                       uint8_t *reply, size_t reply_cap);

/* Advances the link and demand timers. Call every loop. */
void fcec_ec_tick(fcec_ec_t *ec, uint64_t now_us);

/* Writes a Status datagram for EC time `now_us` and returns its length (0 if
 * `cap` is too small). */
size_t fcec_ec_status(fcec_ec_t *ec, uint64_t now_us, uint8_t *buf, size_t cap);

/* What the hardware should output right now. */
const fcec_outputs_t *fcec_ec_outputs(const fcec_ec_t *ec);

/* Whether to send TVC goals to the PA-12s now (ARMED or SAFING). Otherwise
 * send them nothing: they hold their last goal, so the EC never moves an
 * actuator without authority. */
bool fcec_ec_drive_tvc(const fcec_ec_t *ec);

#endif /* EC_SIDE_H */
