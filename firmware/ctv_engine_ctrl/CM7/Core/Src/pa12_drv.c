#include "pa12_drv.h"

#include <math.h>
#include <string.h>

#include "measurement.h"

#define HEADER 0xFFu
#define INSTR_READ 0xF2u
#define INSTR_WRITE 0xF3u
#define REG_FORCE 0x80u
#define REG_GOAL 0x86u
#define REG_PRESENT_POSITION 0x8Cu

/* ---- Packets: FF FF FF | ID | size | instr | factors | checksum ---- */

static size_t packet(uint8_t *out, uint8_t id, uint8_t instr, const uint8_t *factors, size_t n) {
    out[0] = out[1] = out[2] = HEADER;
    out[3] = id;
    out[4] = (uint8_t)(n + 2u);
    out[5] = instr;
    uint8_t sum = (uint8_t)(out[3] + out[4] + out[5]);
    for (size_t i = 0; i < n; i++) {
        out[6 + i] = factors[i];
        sum = (uint8_t)(sum + factors[i]);
    }
    out[6 + n] = (uint8_t)~sum;
    return 7u + n;
}

static size_t goal_packet(uint8_t *out, uint8_t id, uint16_t counts) {
    const uint8_t f[3] = {REG_GOAL, (uint8_t)(counts & 0xFFu), (uint8_t)(counts >> 8)};
    return packet(out, id, INSTR_WRITE, f, 3);
}

static size_t read_position_packet(uint8_t *out, uint8_t id) {
    const uint8_t f[2] = {REG_PRESENT_POSITION, 2u};
    return packet(out, id, INSTR_READ, f, 2);
}

static size_t force_off_packet(uint8_t *out, uint8_t id) {
    const uint8_t f[2] = {REG_FORCE, 0u};
    return packet(out, id, INSTR_WRITE, f, 2);
}

/* ---- Profile and calibration ---- */

bool fcec_pa12_profile_ok(const fcec_pa12_profile_t *p) {
    if (p == NULL || !p->approved || p->id >= 0xFEu) {
        return false;
    }
    if (!isfinite(p->counts_per_mm) || p->counts_per_mm == 0.0f || !isfinite(p->counts_at_0mm)) {
        return false;
    }
    if (p->min_counts >= p->max_counts || p->max_counts > FCEC_PA12_MAX_COUNTS) {
        return false;
    }
    return p->poll_period_ms > 0u && p->reply_timeout_ms > 0u;
}

bool fcec_pa12_mm_to_counts(const fcec_pa12_profile_t *p, float mm, uint16_t *counts) {
    float c = p->counts_per_mm * mm + p->counts_at_0mm;
    if (!isfinite(c)) {
        *counts = p->min_counts;
        return false;
    }
    float r = floorf(c + 0.5f);
    if (r < (float)p->min_counts) {
        *counts = p->min_counts;
        return false;
    }
    if (r > (float)p->max_counts) {
        *counts = p->max_counts;
        return false;
    }
    *counts = (uint16_t)r;
    return true;
}

float fcec_pa12_counts_to_mm(const fcec_pa12_profile_t *p, uint16_t counts) {
    return ((float)counts - p->counts_at_0mm) / p->counts_per_mm;
}

/* ---- Setup ---- */

uint16_t fcec_pa12_init(fcec_pa12_t *drv, const fcec_pa12_profile_t profiles[FCEC_PA12_PORTS],
                        const fcec_pa12_ops_t *ops, uint64_t now_us) {
    uint16_t caps = 0;
    memset(drv, 0, sizeof *drv);
    if (ops != NULL) {
        drv->ops = *ops;
    }
    for (unsigned i = 0; i < FCEC_PA12_PORTS; i++) {
        fcec_pa12_port_t *p = &drv->port[i];
        p->profile = profiles[i];
        p->configured = drv->ops.start_tx != NULL && fcec_pa12_profile_ok(&profiles[i]);
        p->next_poll_us = now_us;
        if (p->configured) {
            caps = (uint16_t)(caps | (FCEC_CH_TVC_A1 << i));
        }
    }
    return caps;
}

/* ---- Transfers ---- */

static void finish_read(fcec_pa12_port_t *p, uint8_t state, uint64_t now_us) {
    p->feedback_state = state;
    p->phase = FCEC_PA12_PHASE_IDLE;
    p->tx_kind = FCEC_PA12_TX_NONE;
    p->quiet_until_us = now_us + FCEC_PA12_READ_GAP_US;
    p->next_poll_us = now_us + (uint64_t)p->profile.poll_period_ms * 1000u;
}

void fcec_pa12_tx_done(fcec_pa12_t *drv, unsigned port, bool ok, uint64_t now_us) {
    if (port >= FCEC_PA12_PORTS) {
        return;
    }
    fcec_pa12_port_t *p = &drv->port[port];
    if (p->phase != FCEC_PA12_PHASE_TX) {
        return; /* late: already timed out */
    }
    fcec_pa12_tx_kind_t kind = p->tx_kind;
    p->phase = FCEC_PA12_PHASE_IDLE;
    p->tx_kind = FCEC_PA12_TX_NONE;
    p->quiet_until_us = now_us + FCEC_PA12_WRITE_GAP_US;

    switch (kind) {
    case FCEC_PA12_TX_GOAL:
        if (ok) {
            p->command_state = FCEC_PA12_COMMAND_SENT_UNCONFIRMED;
            p->have_sent_gen = true; /* service() confirms it to the EC */
        } else {
            p->command_state = FCEC_PA12_COMMAND_TX_FAILED;
            /* Retry if that command is still the current one. */
            p->goal_pending = true;
            p->goal_gen = p->sent_gen;
            p->goal_counts = p->sent_counts;
        }
        break;
    case FCEC_PA12_TX_FORCE_OFF:
        if (!ok) {
            p->force_off_pending = true;
        }
        break;
    case FCEC_PA12_TX_READ:
        if (ok) {
            p->phase = FCEC_PA12_PHASE_WAIT_REPLY;
            p->deadline_us = now_us + (uint64_t)p->profile.reply_timeout_ms * 1000u;
            p->rx_len = 0;
        } else {
            finish_read(p, FCEC_PA12_FEEDBACK_TRANSPORT_ERROR, now_us);
        }
        break;
    default:
        break;
    }
}

void fcec_pa12_rx_byte(fcec_pa12_t *drv, unsigned port, uint8_t byte, uint64_t now_us) {
    if (port >= FCEC_PA12_PORTS) {
        return;
    }
    fcec_pa12_port_t *p = &drv->port[port];
    if (p->phase != FCEC_PA12_PHASE_WAIT_REPLY) {
        return; /* not expecting anything: drop it */
    }
    if (p->rx_len < 3u && byte != HEADER) {
        p->rx_len = 0; /* resync on the header */
        return;
    }
    p->rx[p->rx_len++] = byte;
    if (p->rx_len < 5u) {
        return;
    }
    size_t total = 5u + p->rx[4];
    if (p->rx[4] < 2u || total > sizeof p->rx) {
        p->bad_replies++;
        finish_read(p, FCEC_PA12_FEEDBACK_INVALID_REPLY, now_us);
        return;
    }
    if (p->rx_len < total) {
        return;
    }

    uint8_t sum = 0;
    for (size_t i = 3; i + 1u < total; i++) {
        sum = (uint8_t)(sum + p->rx[i]);
    }
    if ((uint8_t)~sum != p->rx[total - 1u] || p->rx[3] != p->profile.id || p->rx[4] != 4u) {
        p->bad_replies++;
        finish_read(p, FCEC_PA12_FEEDBACK_INVALID_REPLY, now_us);
        return;
    }

    uint8_t err = p->rx[5];
    p->position_counts = (uint16_t)(p->rx[6] | ((uint16_t)p->rx[7] << 8));
    p->position_mm = fcec_pa12_counts_to_mm(&p->profile, p->position_counts);
    p->fault_flags = err;
    p->position_time_us = now_us;
    p->have_position = true;
    p->last_valid_reply_us = now_us;
    p->have_valid_reply = true;
    finish_read(p, err == 0u ? FCEC_PA12_FEEDBACK_VALID_REPLY
                             : FCEC_PA12_FEEDBACK_VALID_REPLY_WITH_FAULT, now_us);
}

static void start(fcec_pa12_t *drv, unsigned port, fcec_pa12_tx_kind_t kind, size_t len,
                  uint64_t now_us) {
    fcec_pa12_port_t *p = &drv->port[port];
    p->phase = FCEC_PA12_PHASE_TX;
    p->tx_kind = kind;
    p->deadline_us = now_us + FCEC_PA12_TX_TIMEOUT_US;
    if (!drv->ops.start_tx(drv->ops.context, port, p->tx, len)) {
        fcec_pa12_tx_done(drv, port, false, now_us);
    }
}

/* ---- Service ---- */

static void expire(fcec_pa12_t *drv, unsigned port, uint64_t now_us) {
    fcec_pa12_port_t *p = &drv->port[port];
    if (p->phase == FCEC_PA12_PHASE_TX && now_us >= p->deadline_us) {
        fcec_pa12_tx_done(drv, port, false, now_us);
    } else if (p->phase == FCEC_PA12_PHASE_WAIT_REPLY && now_us >= p->deadline_us) {
        p->timeouts++;
        finish_read(p, FCEC_PA12_FEEDBACK_TIMEOUT, now_us);
    }
}

static void update_ec(fcec_pa12_port_t *p, unsigned port, fcec_ec_t *ec, uint64_t now_us) {
    uint16_t bit = (uint16_t)(FCEC_CH_TVC_A1 << port);
    fcec_ec_command_t cmd = fcec_ec_command(ec);

    /* A goal whose last byte is out counts as applied (sent, unconfirmed). */
    if (p->have_sent_gen) {
        p->have_sent_gen = false;
        fcec_outputs_t actual = *cmd.outputs;
        actual.tvc_extension_mm[port] = fcec_pa12_counts_to_mm(&p->profile, p->sent_counts);
        fcec_ec_confirm_applied(ec, bit, p->sent_gen, &actual, now_us);
    }

    bool ready = p->have_valid_reply && now_us >= p->last_valid_reply_us &&
                 now_us - p->last_valid_reply_us <= FCEC_PA12_READY_WINDOW_US;
    if (ready != p->ready) {
        p->ready = ready;
        fcec_ec_set_ready(ec, bit, ready);
    }

    if (p->have_valid_reply) {
        if (p->fault_flags != 0u && !p->faulted) {
            p->faulted = true;
            fcec_ec_report_fault(ec, bit);
        } else if (p->fault_flags == 0u && p->faulted) {
            p->faulted = false;
            fcec_ec_clear_fault(ec, bit);
        }
    }

    /* What this port should send for the current command. */
    bool drive = (cmd.drive_mask & bit) != 0;
    bool safing = ec->state == FCEC_STATE_SAFING;
    if (!safing) {
        p->safe_done = false;
        p->safe_goal_pending = false;
        p->force_off_pending = false;
    }
    if (drive && safing && p->profile.safe_action != FCEC_PA12_SAFE_PARK) {
        /* Hold or Force-OFF: act once per safing, then send no goals (a goal
         * would move it, and turns force back on). */
        p->goal_pending = false;
        if (!p->safe_done) {
            p->safe_done = true;
            if (p->profile.safe_action == FCEC_PA12_SAFE_FORCE_OFF) {
                p->force_off_pending = true;
            } else if (p->have_position) {
                /* Stop where it is: one goal at the last measured position. */
                uint16_t c = p->position_counts;
                c = c < p->profile.min_counts ? p->profile.min_counts : c;
                c = c > p->profile.max_counts ? p->profile.max_counts : c;
                p->safe_goal_pending = true;
                p->safe_goal_counts = c;
                p->safe_goal_gen = cmd.gen;
            }
        }
    } else if (drive) {
        /* Normal goals, and the park goal when the safe action is PARK. */
        /* One goal per command; a newer command replaces an unsent goal. */
        if (!p->have_goal_gen || p->goal_gen != cmd.gen) {
            uint16_t counts;
            (void)fcec_pa12_mm_to_counts(&p->profile, cmd.outputs->tvc_extension_mm[port], &counts);
            p->goal_pending = true;
            p->have_goal_gen = true;
            p->goal_gen = cmd.gen;
            p->goal_counts = counts;
        }
    } else {
        p->goal_pending = false; /* no authority to move it */
    }
}

void fcec_pa12_service(fcec_pa12_t *drv, fcec_ec_t *ec, uint64_t now_us) {
    for (unsigned i = 0; i < FCEC_PA12_PORTS; i++) {
        fcec_pa12_port_t *p = &drv->port[i];
        if (!p->configured) {
            continue;
        }
        expire(drv, i, now_us);
        update_ec(p, i, ec, now_us);

        if (p->phase != FCEC_PA12_PHASE_IDLE || now_us < p->quiet_until_us) {
            continue;
        }
        if (p->force_off_pending) {
            p->force_off_pending = false;
            start(drv, i, FCEC_PA12_TX_FORCE_OFF, force_off_packet(p->tx, p->profile.id), now_us);
        } else if (p->safe_goal_pending) {
            p->safe_goal_pending = false;
            p->sent_gen = p->safe_goal_gen;
            p->sent_counts = p->safe_goal_counts;
            p->command_request_id++;
            p->command_state = FCEC_PA12_COMMAND_QUEUED;
            start(drv, i, FCEC_PA12_TX_GOAL, goal_packet(p->tx, p->profile.id, p->safe_goal_counts),
                  now_us);
        } else if (p->goal_pending) {
            p->goal_pending = false;
            p->sent_gen = p->goal_gen;
            p->sent_counts = p->goal_counts;
            p->command_request_id++;
            p->command_state = FCEC_PA12_COMMAND_QUEUED;
            start(drv, i, FCEC_PA12_TX_GOAL, goal_packet(p->tx, p->profile.id, p->goal_counts),
                  now_us);
        } else if (now_us >= p->next_poll_us) {
            p->feedback_request_id++;
            p->feedback_state = FCEC_PA12_FEEDBACK_PENDING;
            start(drv, i, FCEC_PA12_TX_READ, read_position_packet(p->tx, p->profile.id), now_us);
        }
    }
}

void fcec_pa12_publish(const fcec_pa12_t *drv, fcec_measurements_t *m, uint64_t now_us) {
    for (unsigned i = 0; i < FCEC_PA12_PORTS; i++) {
        const fcec_pa12_port_t *p = &drv->port[i];
        m->pa12_command_request_id[i] = p->command_request_id;
        m->pa12_command_state[i] = p->command_state;
        m->pa12_feedback_request_id[i] = p->feedback_request_id;
        m->pa12_feedback_state[i] = p->feedback_state;
        if (!p->configured || !p->have_position) {
            m->validity_mask &= ~(1u << i);
            m->measurement_quality[i] =
                p->configured ? FCEC_QUALITY_NOT_SAMPLED : FCEC_QUALITY_UNCONFIGURED;
            continue;
        }
        uint64_t age_us = now_us >= p->position_time_us ? now_us - p->position_time_us : 0u;
        uint16_t age_ms = age_us / 1000u > 0xFFFFu ? 0xFFFFu : (uint16_t)(age_us / 1000u);
        bool fresh = age_us <= FCEC_PA12_READY_WINDOW_US;
        (void)fcec_measurement_publish_pa12(m, i, p->position_counts, p->position_mm, age_ms,
                                            p->fault_flags, p->position_time_us, fresh);
        if (!fresh) {
            m->measurement_quality[i] = FCEC_QUALITY_STALE;
        } else if (p->fault_flags != 0u) {
            m->measurement_quality[i] = FCEC_QUALITY_DEVICE_FAULT;
        }
    }
}
