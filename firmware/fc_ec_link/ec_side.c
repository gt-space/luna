#include "ec_side.h"

#include <string.h>

static uint64_t ms_to_us(uint32_t ms) {
    return (uint64_t)ms * 1000u;
}

/* ---- Outputs ---- */

/* Fans off and the ESC at minimum throttle. When safing, the TVC also parks
 * at center; otherwise it holds where it is. */
static void apply_safe(fcec_ec_t *ec, bool center_tvc) {
    for (size_t i = 0; i < 4; i++) {
        ec->applied.rcs_duty[i] = 0.0f;
    }
    ec->applied.main_throttle = 0.0f;

    if (center_tvc) {
        ec->applied.tvc_extension_mm[0] = ec->tvc_center_mm[0];
        ec->applied.tvc_extension_mm[1] = ec->tvc_center_mm[1];
    }
}

/* Latches safing. Keeps the first reason if already safing. */
static void enter_safing(fcec_ec_t *ec, fcec_reason_t reason) {
    if (ec->state != FCEC_STATE_SAFING) {
        ec->state = FCEC_STATE_SAFING;
        ec->safing_reason = reason;
    }
    apply_safe(ec, true);
}

/* Writes the newest accepted demand to the outputs. Channels it doesn't
 * enable stay safe: fans and ESC at 0, TVC where it is. */
static void apply_demand(fcec_ec_t *ec, uint32_t seq, uint64_t now_us) {
    const fcec_demand_t *d = &ec->demand;
    fcec_outputs_t out = ec->applied;

    for (unsigned i = 0; i < 2; i++) {
        if ((d->enable_mask & (FCEC_CH_TVC_A1 << i)) != 0) {
            out.tvc_extension_mm[i] = d->outputs.tvc_extension_mm[i];
        }
    }
    for (unsigned i = 0; i < 4; i++) {
        bool enabled = (d->enable_mask & (FCEC_CH_RCS_M1 << i)) != 0;
        out.rcs_duty[i] = enabled ? d->outputs.rcs_duty[i] : 0.0f;
    }
    out.main_throttle =
        (d->enable_mask & FCEC_CH_MAIN_ESC) != 0 ? d->outputs.main_throttle : 0.0f;

    ec->applied = out;
    ec->last_applied_demand_seq = seq;
    ec->last_applied_time_us = now_us;
}

/* ---- Replies ---- */

static void start_reply(fcec_ec_t *ec, fcec_packet_t *p, fcec_kind_t kind, uint32_t txn_id,
                        uint64_t now_us) {
    memset(p, 0, sizeof *p);
    p->kind = kind;
    p->header.ec_boot_id = ec->boot_id;
    p->header.session_id = ec->session_valid ? ec->session_id : 0;
    p->header.seq = ++ec->tx_seq;
    p->header.txn_id = txn_id;
    p->header.sent_time_us = now_us;
}

static size_t finish_reply(const fcec_packet_t *p, uint8_t *reply, size_t cap) {
    int len = fcec_encode(p, reply, cap);
    return len > 0 ? (size_t)len : 0;
}

static size_t send_result(fcec_ec_t *ec, const fcec_txn_result_t *result, uint32_t txn_id,
                          bool duplicate, uint64_t now_us, uint8_t *reply, size_t cap) {
    fcec_packet_t p;
    start_reply(ec, &p, FCEC_KIND_TXN_RESULT, txn_id, now_us);
    p.header.duplicate = duplicate;
    p.msg.txn_result = *result;
    return finish_reply(&p, reply, cap);
}

static size_t send_hello_result(fcec_ec_t *ec, uint32_t txn_id, fcec_outcome_t outcome,
                                fcec_reason_t reason, uint64_t now_us, uint8_t *reply,
                                size_t cap) {
    fcec_packet_t p;
    start_reply(ec, &p, FCEC_KIND_HELLO_RESULT, txn_id, now_us);

    fcec_hello_result_t *r = &p.msg.hello_result;
    r->outcome = outcome;
    r->ec_state = ec->state;
    r->reason = reason;
    r->capabilities = ec->capabilities;
    r->max_demand_age_ms = ec->max_demand_age_ms;
    r->firmware_version = ec->firmware_version;
    r->calibration_id = ec->calibration_id;
    r->link_timeout_ms = ec->link_timeout_ms;
    /* active_bundle_sha256 stays all zeros: no config on the EC yet. */

    return finish_reply(&p, reply, cap);
}

/* Counts a rejection and answers it if the request was a transaction.
 * Demands and heartbeats get no answer; the reason shows up in Status. */
static size_t reject(fcec_ec_t *ec, const fcec_packet_t *in, fcec_reason_t reason,
                     uint64_t now_us, uint8_t *reply, size_t cap) {
    ec->counters.rejected++;
    if (reason == FCEC_REASON_STALE) {
        ec->counters.stale++;
    }
    ec->last_reject_reason = reason;

    switch (in->kind) {
    case FCEC_KIND_HELLO:
        return send_hello_result(ec, in->header.txn_id, FCEC_OUTCOME_REJECTED, reason, now_us,
                                 reply, cap);
    case FCEC_KIND_ARM:
    case FCEC_KIND_DISARM:
    case FCEC_KIND_CLEAR_SAFING: {
        fcec_txn_result_t result = {in->kind, FCEC_OUTCOME_REJECTED, reason, ec->state};
        return send_result(ec, &result, in->header.txn_id, false, now_us, reply, cap);
    }
    default:
        return 0;
    }
}

/* ---- Transactions ---- */

static const fcec_ec_txn_t *find_txn(const fcec_ec_t *ec, uint32_t txn_id) {
    uint32_t stored = ec->txn_count < FCEC_EC_TXN_HISTORY ? ec->txn_count : FCEC_EC_TXN_HISTORY;
    for (uint32_t i = 0; i < stored; i++) {
        if (ec->txns[i].txn_id == txn_id) {
            return &ec->txns[i];
        }
    }
    return NULL;
}

static void remember_txn(fcec_ec_t *ec, uint32_t txn_id, const fcec_txn_result_t *result) {
    fcec_ec_txn_t *slot = &ec->txns[ec->txn_count % FCEC_EC_TXN_HISTORY];
    slot->txn_id = txn_id;
    slot->result = *result;
    ec->txn_count++;
    ec->have_txn = true;
    ec->newest_txn_id = txn_id;
}

static size_t handle_txn(fcec_ec_t *ec, const fcec_packet_t *in, uint64_t now_us,
                         uint8_t *reply, size_t cap) {
    fcec_reason_t reason = FCEC_REASON_NONE;

    switch (in->kind) {
    case FCEC_KIND_ARM:
        /* Bare bones: the config bundle check is skipped for now. */
        if (ec->state == FCEC_STATE_SAFING) {
            reason = FCEC_REASON_SAFING_LATCHED;
        } else if (ec->state == FCEC_STATE_BOOT_SAFE) {
            reason = FCEC_REASON_NOT_READY;
        } else if (ec->state == FCEC_STATE_DISARMED) {
            ec->state = FCEC_STATE_ARMED;
            ec->demand_due_us = now_us; /* a fresh demand must follow in time */
        }
        break;
    case FCEC_KIND_DISARM:
        if (ec->state == FCEC_STATE_ARMED) {
            ec->state = FCEC_STATE_DISARMED;
            apply_safe(ec, false);
        }
        break;
    case FCEC_KIND_CLEAR_SAFING:
        if (ec->state == FCEC_STATE_SAFING) {
            ec->state = FCEC_STATE_DISARMED;
            ec->safing_reason = FCEC_REASON_NONE;
        } else {
            reason = FCEC_REASON_WRONG_STATE;
        }
        break;
    default:
        return 0;
    }

    fcec_txn_result_t result = {
        in->kind,
        reason == FCEC_REASON_NONE ? FCEC_OUTCOME_ACCEPTED : FCEC_OUTCOME_REJECTED,
        reason,
        ec->state,
    };

    if (reason != FCEC_REASON_NONE) {
        ec->counters.rejected++;
        ec->last_reject_reason = reason;
    }

    remember_txn(ec, in->header.txn_id, &result);
    return send_result(ec, &result, in->header.txn_id, false, now_us, reply, cap);
}

/* ---- Messages ---- */

static size_t handle_hello(fcec_ec_t *ec, const fcec_packet_t *in, uint64_t now_us,
                           uint8_t *reply, size_t cap) {
    const fcec_header_t *h = &in->header;
    bool same_session = ec->session_valid && h->session_id == ec->session_id;

    if (!same_session) {
        if (h->session_id == 0) {
            return reject(ec, in, FCEC_REASON_BAD_SESSION, now_us, reply, cap);
        }
        if (ec->state == FCEC_STATE_ARMED) {
            return reject(ec, in, FCEC_REASON_WRONG_STATE, now_us, reply, cap);
        }
        if (ec->state == FCEC_STATE_BOOT_SAFE) {
            return reject(ec, in, FCEC_REASON_NOT_READY, now_us, reply, cap);
        }

        ec->session_valid = true;
        ec->session_id = h->session_id;
        ec->tx_seq = 0;
        ec->have_txn = false;
        ec->txn_count = 0;
        ec->have_demand = false;
        ec->last_received_seq = 0;
        ec->last_accepted_demand_seq = 0;
        ec->last_applied_demand_seq = 0;
        ec->demand_period_ms = in->msg.hello.demand_period_ms != 0
                                   ? in->msg.hello.demand_period_ms
                                   : FCEC_EC_DEFAULT_DEMAND_PERIOD_MS;
    }

    ec->last_rx_us = now_us;
    if (fcec_is_newer(h->seq, ec->last_received_seq)) {
        ec->last_received_seq = h->seq;
    }

    return send_hello_result(ec, h->txn_id, FCEC_OUTCOME_ACCEPTED, FCEC_REASON_NONE, now_us,
                             reply, cap);
}

static void handle_demand(fcec_ec_t *ec, const fcec_packet_t *in, uint64_t now_us) {
    uint32_t seq = in->header.seq;
    fcec_reason_t reason = fcec_check_demand(&in->msg.demand, ec->capabilities);

    if (reason != FCEC_REASON_NONE) {
        reject(ec, in, reason, now_us, NULL, 0);
        return;
    }

    if (ec->have_demand && !fcec_is_newer(seq, ec->last_accepted_demand_seq)) {
        ec->counters.old_or_duplicate++;
        return;
    }

    ec->demand = in->msg.demand;
    ec->have_demand = true;
    ec->last_accepted_demand_seq = seq;
    ec->last_demand_us = now_us;

    if (ec->state == FCEC_STATE_ARMED) {
        ec->demand_due_us = now_us;
        apply_demand(ec, seq, now_us);
    }
}

/* Whether the FC's information was current: the EC time it echoes is in the
 * past, not too old, and the message hasn't expired. */
static bool is_fresh(const fcec_ec_t *ec, const fcec_header_t *h, uint64_t now_us) {
    if (h->ec_time_ref_us > now_us) {
        return false;
    }
    if (now_us - h->ec_time_ref_us > ms_to_us(ec->max_demand_age_ms)) {
        return false;
    }
    return now_us <= h->ec_time_ref_us + ms_to_us(h->valid_for_ms);
}

static bool is_txn(fcec_kind_t kind) {
    return kind == FCEC_KIND_ARM || kind == FCEC_KIND_DISARM || kind == FCEC_KIND_CLEAR_SAFING;
}

/* ---- Public API ---- */

void fcec_ec_init(fcec_ec_t *ec, uint32_t boot_id) {
    memset(ec, 0, sizeof *ec);
    ec->capabilities = FCEC_CH_ALL;
    ec->max_demand_age_ms = FCEC_EC_MAX_DEMAND_AGE_MS;
    ec->link_timeout_ms = FCEC_EC_LINK_TIMEOUT_MS;
    ec->tvc_center_mm[0] = FCEC_TVC_STROKE_MM / 2.0f;
    ec->tvc_center_mm[1] = FCEC_TVC_STROKE_MM / 2.0f;
    ec->boot_id = boot_id;
    ec->state = FCEC_STATE_BOOT_SAFE;
    ec->demand_period_ms = FCEC_EC_DEFAULT_DEMAND_PERIOD_MS;
    apply_safe(ec, true);
}

void fcec_ec_boot_done(fcec_ec_t *ec) {
    if (ec->state == FCEC_STATE_BOOT_SAFE) {
        ec->state = FCEC_STATE_DISARMED;
    }
}

size_t fcec_ec_receive(fcec_ec_t *ec, const uint8_t *buf, size_t len, uint64_t now_us,
                       uint8_t *reply, size_t reply_cap) {
    fcec_packet_t in;
    fcec_result_t result = fcec_decode(buf, len, &in);

    if (result != FCEC_OK) {
        /* Corrupt or foreign: no reply, since the header can't be trusted. */
        if (result >= FCEC_ERR_BAD_CRC) {
            ec->counters.crc_errors++;
        } else {
            ec->counters.rejected++;
        }
        return 0;
    }

    const fcec_header_t *h = &in.header;

    if ((uint8_t)in.kind >= 0x80u) {
        ec->counters.rejected++; /* an EC -> FC kind */
        return 0;
    }

    /* Safing only removes authority, so any well-formed Terminate is obeyed. */
    if (in.kind == FCEC_KIND_TERMINATE) {
        enter_safing(ec, FCEC_REASON_FC_TERMINATE);
        fcec_txn_result_t done = {FCEC_KIND_TERMINATE, FCEC_OUTCOME_ACCEPTED, FCEC_REASON_NONE,
                                  ec->state};
        return send_result(ec, &done, h->txn_id, false, now_us, reply, reply_cap);
    }

    if (in.kind == FCEC_KIND_HELLO) {
        return handle_hello(ec, &in, now_us, reply, reply_cap);
    }

    if (h->ec_boot_id != ec->boot_id) {
        return reject(ec, &in, FCEC_REASON_BAD_BOOT_ID, now_us, reply, reply_cap);
    }
    if (!ec->session_valid || h->session_id != ec->session_id) {
        return reject(ec, &in, FCEC_REASON_BAD_SESSION, now_us, reply, reply_cap);
    }

    if (is_txn(in.kind)) {
        /* A repeat gets the original answer and changes nothing. */
        const fcec_ec_txn_t *earlier = find_txn(ec, h->txn_id);
        if (earlier != NULL) {
            ec->last_rx_us = now_us;
            return send_result(ec, &earlier->result, h->txn_id, true, now_us, reply, reply_cap);
        }
        if (ec->have_txn && !fcec_is_newer(h->txn_id, ec->newest_txn_id)) {
            return reject(ec, &in, FCEC_REASON_STALE, now_us, reply, reply_cap);
        }
    }

    /* Disarm only removes authority, so it doesn't need to be fresh. */
    bool needs_fresh = in.kind == FCEC_KIND_DEMAND || in.kind == FCEC_KIND_ARM ||
                       in.kind == FCEC_KIND_CLEAR_SAFING;
    if (needs_fresh && !is_fresh(ec, h, now_us)) {
        return reject(ec, &in, FCEC_REASON_STALE, now_us, reply, reply_cap);
    }

    ec->last_rx_us = now_us;
    if (fcec_is_newer(h->seq, ec->last_received_seq)) {
        ec->last_received_seq = h->seq;
    }

    switch (in.kind) {
    case FCEC_KIND_DEMAND:
        handle_demand(ec, &in, now_us);
        return 0;
    case FCEC_KIND_ARM:
    case FCEC_KIND_DISARM:
    case FCEC_KIND_CLEAR_SAFING:
        return handle_txn(ec, &in, now_us, reply, reply_cap);
    default:
        return 0; /* Heartbeat: keeping the link alive is all it does. */
    }
}

void fcec_ec_tick(fcec_ec_t *ec, uint64_t now_us) {
    if (ec->session_valid && now_us > ec->last_rx_us &&
        now_us - ec->last_rx_us > ms_to_us(ec->link_timeout_ms)) {
        ec->session_valid = false;
        if (ec->state != FCEC_STATE_BOOT_SAFE) {
            enter_safing(ec, FCEC_REASON_LINK_TIMEOUT);
        }
    }

    uint64_t demand_timeout_us =
        ms_to_us(FCEC_EC_DEMAND_TIMEOUT_PERIODS * (uint32_t)ec->demand_period_ms);
    if (ec->state == FCEC_STATE_ARMED && now_us > ec->demand_due_us &&
        now_us - ec->demand_due_us > demand_timeout_us) {
        enter_safing(ec, FCEC_REASON_DEMAND_TIMEOUT);
    }
}

size_t fcec_ec_status(fcec_ec_t *ec, uint64_t now_us, uint8_t *buf, size_t cap) {
    fcec_packet_t p;
    start_reply(ec, &p, FCEC_KIND_STATUS, 0, now_us);

    uint64_t demand_timeout_us =
        ms_to_us(FCEC_EC_DEMAND_TIMEOUT_PERIODS * (uint32_t)ec->demand_period_ms);
    fcec_status_t *s = &p.msg.status;

    s->ec_state = ec->state;
    s->output_authority = ec->state == FCEC_STATE_ARMED;
    s->session_valid = ec->session_valid;
    s->safing_latched = ec->state == FCEC_STATE_SAFING;
    s->demand_fresh = ec->have_demand && now_us >= ec->last_demand_us &&
                      now_us - ec->last_demand_us <= demand_timeout_us;
    s->safing_reason = ec->safing_reason;
    s->last_received_seq = ec->last_received_seq;
    s->last_accepted_demand_seq = ec->last_accepted_demand_seq;
    s->last_applied_demand_seq = ec->last_applied_demand_seq;
    s->last_applied_time_us = ec->last_applied_time_us;
    s->last_reject_reason = ec->last_reject_reason;
    s->limited_mask = 0; /* no limits tighter than the physical range yet */
    s->counters = ec->counters;
    s->reset_cause = ec->reset_cause;
    s->firmware_version = ec->firmware_version;
    s->applied = ec->applied;
    s->measurements = ec->measurements;

    return finish_reply(&p, buf, cap);
}

const fcec_outputs_t *fcec_ec_outputs(const fcec_ec_t *ec) {
    return &ec->applied;
}

bool fcec_ec_drive_tvc(const fcec_ec_t *ec) {
    return ec->state == FCEC_STATE_ARMED || ec->state == FCEC_STATE_SAFING;
}
