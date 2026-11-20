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
 *     ec.capabilities = channels that have a driver;  ec.auto_confirm = false;
 *     ... bring up drivers, then fcec_ec_set_ready() for each one that is up ...
 *     fcec_ec_boot_done(&ec);
 *     for (;;) {
 *         for each received datagram:
 *             n = fcec_ec_receive(&ec, rx, rx_len, now_us, reply, sizeof reply);
 *             if (n > 0) send reply[0..n] back to the sender;
 *         fcec_ec_tick(&ec, now_us);
 *         cmd = fcec_ec_command(&ec);   // newest command, latest value only
 *         each driver: output cmd.outputs for its channels in cmd.drive_mask at
 *             its next update boundary, then fcec_ec_confirm_applied(&ec, its
 *             channels, cmd.gen, what it actually output, now_us);
 *             on a fault: fcec_ec_report_fault();
 *         every 10 ms: fill ec.measurements, then
 *             n = fcec_ec_status(&ec, now_us, tx, sizeof tx); send it to the FC;
 *     }
 *
 * Command lifecycle: Received (a datagram arrived), Accepted (it passed the
 * session, freshness, channel and range checks: last_accepted_demand_seq),
 * Applied (every channel the demand enables confirmed it at its output-update
 * boundary: last_applied_demand_seq), Measured (a separate sensor saw it, in
 * the Status measurements). Receive only validates and stores; it never
 * waits on hardware.
 *
 * Not in this version: the config bundle check on Arm and a transaction
 * queue (transactions are handled immediately).
 */

#ifndef EC_SIDE_H
#define EC_SIDE_H

#include "fc_ec_link.h"

#define FCEC_EC_TXN_HISTORY 8u
#define FCEC_EC_MAX_DEMAND_AGE_MS 50u
#define FCEC_EC_LINK_TIMEOUT_MS 350u
#define FCEC_EC_DEFAULT_DEMAND_PERIOD_MS 10u
#define FCEC_EC_DEMAND_TIMEOUT_PERIODS 3u

/* Layout of Status `driver_faults`, built by fcec_ec_status(). A channel that
 * isn't in `capabilities` has no driver: it is unavailable, and none of its
 * bits are set. */
#define FCEC_DRIVER_FAULT_SHIFT 0u     /* bits 0-6: channel driver faulted (FCEC_CH_* order) */
#define FCEC_DRIVER_NOT_READY_SHIFT 8u /* bits 8-14: configured channel not ready */
#define FCEC_DRIVER_INTERLOCK_OPEN (1u << 16) /* local output interlock open */

typedef struct {
    uint32_t txn_id;
    fcec_txn_result_t result;
} fcec_ec_txn_t;

/* What the output drivers should do now. See fcec_ec_command(). */
typedef struct {
    const fcec_outputs_t *outputs; /* targets, after EC limits */
    uint16_t drive_mask;   /* channels whose driver must output `outputs` now */
    uint32_t gen;          /* changes whenever the command changes */
    uint32_t demand_seq;   /* demand this command came from; 0 = a safe command */
} fcec_ec_command_t;

typedef struct {
    /* Settings. fcec_ec_init() sets defaults; change them before the first
     * datagram if needed. */
    uint16_t capabilities;      /* FCEC_CH_* channels that have a configured driver */
    uint16_t max_demand_age_ms; /* older Demand/Arm/ClearSafing are stale */
    uint16_t link_timeout_ms;   /* no valid FC message this long -> safing */
    uint32_t firmware_version;
    uint32_t calibration_id;
    uint32_t watchdog_status; /* watchdog state/fault bitmap, maintained by firmware */
    float tvc_center_mm[2]; /* where safing parks the TVC actuators (the agreed
                               safe/equilibrium extension) */
    /* Approved limits per channel, inside the physical range. Accepted demand
     * values outside them are clamped and reported in limited_mask. */
    fcec_outputs_t limit_min;
    fcec_outputs_t limit_max;
    /* true: an output counts as applied as soon as it is commanded (the host
     * model, used by the fake EC and the protocol tests). false: only
     * fcec_ec_confirm_applied() makes it applied (real drivers). */
    bool auto_confirm;

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
    uint32_t last_applied_demand_seq; /* every enabled channel confirmed it */
    uint64_t last_applied_time_us;    /* EC time of the last of those confirmations */
    fcec_reason_t last_reject_reason;
    fcec_demand_t demand;    /* newest accepted demand */

    /* Drivers. Set through fcec_ec_set_ready(), fcec_ec_set_interlock(),
     * fcec_ec_report_fault() and fcec_ec_clear_fault(). */
    uint16_t ready_mask;     /* channels whose driver says it is ready */
    uint16_t fault_mask;     /* channels whose driver reported a fault */
    bool interlock_ok;       /* local interlocks allow outputs */

    fcec_outputs_t commanded; /* the current command's targets */
    uint16_t drive_mask;      /* channels the current command drives */
    uint32_t command_gen;
    uint32_t command_seq;     /* demand behind the command; 0 = safe command */
    uint16_t command_needs;   /* channels that must confirm before it's applied */
    uint16_t command_limited; /* channels the command clamped */
    uint16_t confirmed_mask;  /* channels that confirmed this command */
    fcec_outputs_t applied;   /* what the drivers last confirmed outputting */
    uint16_t limited_mask;    /* channels clamped in the last applied demand */
    uint32_t stale_confirms;  /* confirmations for a replaced command, ignored */

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

/* What the hardware should output right now (the current command's
 * targets). */
const fcec_outputs_t *fcec_ec_outputs(const fcec_ec_t *ec);

/* The current command for the output drivers. Only the newest command
 * matters: a driver that hasn't acted on an older one skips it. */
fcec_ec_command_t fcec_ec_command(const fcec_ec_t *ec);

/* A driver confirms that its `channels` reached their output-update boundary
 * for command `gen`, outputting `actual` (NULL: exactly the command). A
 * confirmation for a replaced command is ignored. When every channel the
 * demand enables has confirmed, last_applied_demand_seq advances. */
void fcec_ec_confirm_applied(fcec_ec_t *ec, uint16_t channels, uint32_t gen,
                             const fcec_outputs_t *actual, uint64_t now_us);

/* A driver for `channels` is (or is no longer) ready to accept commands.
 * Arm needs every channel in `capabilities` ready. Losing readiness while
 * armed safes the EC (LOCAL_FAULT). */
void fcec_ec_set_ready(fcec_ec_t *ec, uint16_t channels, bool ready);

/* Local interlocks closed (outputs allowed) or open. Arm needs them closed;
 * opening one while armed safes the EC (LOCAL_FAULT). */
void fcec_ec_set_interlock(fcec_ec_t *ec, bool ok);

/* A driver for `channels` failed. Outside BOOT_SAFE this latches SAFING
 * (LOCAL_FAULT); Arm stays refused until fcec_ec_clear_fault(). */
void fcec_ec_report_fault(fcec_ec_t *ec, uint16_t channels);

/* The driver for `channels` recovered. Doesn't clear safing: the FC still
 * has to send ClearSafing and a fresh Arm. */
void fcec_ec_clear_fault(fcec_ec_t *ec, uint16_t channels);

/* FCEC_REASON_NONE if Arm would be allowed by the drivers, else why not. */
fcec_reason_t fcec_ec_arm_blocker(const fcec_ec_t *ec);

/* Whether to send TVC goals to the PA-12s now (ARMED or SAFING). Otherwise
 * send them nothing: they hold their last goal, so the EC never moves an
 * actuator without authority. */
bool fcec_ec_drive_tvc(const fcec_ec_t *ec);

#endif /* EC_SIDE_H */
