/*
 * Host tests for the PA-12 driver logic (pa12_drv.c) with a fake UART and
 * the EC state machine, in simulated time. Profile numbers are made up.
 *
 * Run with `make test` in this folder.
 */

#include "pa12_drv.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

static int checks;
static int failures;

#define CHECK(cond)                                                       \
    do {                                                                  \
        checks++;                                                         \
        if (!(cond)) {                                                    \
            failures++;                                                   \
            printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);        \
        }                                                                 \
    } while (0)

#define MS 1000u
#define BOOT_ID 0xB0070003u
#define SESSION_ID 0x5E550003u

/* ---- Fake UART ---- */

typedef struct {
    uint8_t bytes[16];
    size_t len;
    unsigned sends;
    bool busy;
} fake_port_t;

static fake_port_t uart[2];
static bool refuse_tx;

static bool fake_start_tx(void *context, unsigned port, const uint8_t *bytes, size_t len) {
    (void)context;
    if (refuse_tx) {
        return false;
    }
    CHECK(!uart[port].busy); /* never two transfers at once on one port */
    memcpy(uart[port].bytes, bytes, len);
    uart[port].len = len;
    uart[port].sends++;
    uart[port].busy = true;
    return true;
}

static const fcec_pa12_ops_t ops = {NULL, fake_start_tx};

static bool sent(unsigned port, const uint8_t *expect, size_t n) {
    return uart[port].len == n && memcmp(uart[port].bytes, expect, n) == 0;
}

static bool sent_goal(unsigned port) {
    return uart[port].len == 10 && uart[port].bytes[5] == 0xF3 && uart[port].bytes[6] == 0x86;
}

static bool sent_read(unsigned port) {
    return uart[port].len == 9 && uart[port].bytes[5] == 0xF2 && uart[port].bytes[6] == 0x8C;
}

static bool sent_force_off(unsigned port) {
    return uart[port].len == 9 && uart[port].bytes[5] == 0xF3 && uart[port].bytes[6] == 0x80;
}

static uint16_t goal_counts(unsigned port) {
    return (uint16_t)(uart[port].bytes[7] | (uart[port].bytes[8] << 8));
}

/* Finishes the transfer in progress on `port`. */
static void finish_tx(fcec_pa12_t *d, unsigned port, uint64_t now) {
    uart[port].busy = false;
    fcec_pa12_tx_done(d, port, true, now);
}

static void reply(fcec_pa12_t *d, unsigned port, uint8_t id, uint8_t err, uint16_t pos,
                  bool corrupt, uint64_t now) {
    uint8_t b[9] = {0xFF, 0xFF, 0xFF, id, 4, err, (uint8_t)(pos & 0xFF), (uint8_t)(pos >> 8), 0};
    uint8_t sum = 0;
    for (int i = 3; i < 8; i++) {
        sum = (uint8_t)(sum + b[i]);
    }
    b[8] = (uint8_t)(~sum + (corrupt ? 1 : 0));
    for (int i = 0; i < 9; i++) {
        fcec_pa12_rx_byte(d, port, b[i], now);
    }
}

/* Made-up calibration: 100 counts/mm, 1000 counts at 0 mm. */
static fcec_pa12_profile_t profile(uint8_t id) {
    fcec_pa12_profile_t p;
    memset(&p, 0, sizeof p);
    p.approved = true;
    p.id = id;
    p.counts_per_mm = 100.0f;
    p.counts_at_0mm = 1000.0f;
    p.min_counts = 1000;
    p.max_counts = 3690;
    p.poll_period_ms = 20;
    p.reply_timeout_ms = 15;
    return p;
}

/* ---- EC ---- */

static fcec_ec_t ec;
static uint32_t fc_seq;
static uint32_t fc_txn;
static fcec_packet_t ans;

static void send(fcec_kind_t kind, uint64_t now, float a1_mm, float a2_mm) {
    fcec_packet_t p;
    memset(&p, 0, sizeof p);
    p.kind = kind;
    p.header.ec_boot_id = BOOT_ID;
    p.header.session_id = SESSION_ID;
    p.header.seq = ++fc_seq;
    p.header.valid_for_ms = 50;
    p.header.sent_time_us = now;
    p.header.ec_time_ref_us = now;
    if (kind == FCEC_KIND_DEMAND) {
        p.msg.demand.outputs.tvc_extension_mm[0] = a1_mm;
        p.msg.demand.outputs.tvc_extension_mm[1] = a2_mm;
        p.msg.demand.enable_mask = FCEC_CH_TVC_A1 | FCEC_CH_TVC_A2;
    } else if (kind != FCEC_KIND_HEARTBEAT) {
        p.header.txn_id = ++fc_txn;
    }
    if (kind == FCEC_KIND_HELLO) {
        p.msg.hello.demand_period_ms = 10;
    }
    uint8_t rx[FCEC_MAX_DATAGRAM_LEN];
    uint8_t tx[FCEC_MAX_DATAGRAM_LEN];
    int len = fcec_encode(&p, rx, sizeof rx);
    CHECK(len > 0);
    size_t n = fcec_ec_receive(&ec, rx, (size_t)len, now, tx, sizeof tx);
    memset(&ans, 0, sizeof ans);
    if (n > 0) {
        CHECK(fcec_decode(tx, n, &ans) == FCEC_OK);
    }
}

static bool accepted(fcec_kind_t kind) {
    return ans.kind == FCEC_KIND_TXN_RESULT && ans.msg.txn_result.request_kind == kind &&
           ans.msg.txn_result.outcome == FCEC_OUTCOME_ACCEPTED;
}

/* Driver + EC with both ports configured, booted, session open at `now`. */
static void setup(fcec_pa12_t *d, const fcec_pa12_profile_t prof[2], uint64_t now) {
    memset(uart, 0, sizeof uart);
    refuse_tx = false;
    fcec_ec_init(&ec, BOOT_ID);
    ec.auto_confirm = false;
    ec.capabilities = fcec_pa12_init(d, prof, &ops, now);
    fcec_ec_set_ready(&ec, FCEC_CH_ALL, false);
    fcec_ec_boot_done(&ec);
    fc_seq = 0;
    fc_txn = 0;
    send(FCEC_KIND_HELLO, now, 0, 0);
}

/* Lets both ports read once and get valid replies at `pos`. */
static void read_both(fcec_pa12_t *d, uint64_t *now, uint16_t pos) {
    fcec_pa12_service(d, &ec, *now);
    for (unsigned i = 0; i < 2; i++) {
        CHECK(sent_read(i));
        finish_tx(d, i, *now);
        reply(d, i, d->port[i].profile.id, 0, pos, false, *now + 2 * MS);
    }
    *now += 2 * MS;
    fcec_pa12_service(d, &ec, *now);
}

/* ---- Tests ---- */

static void test_packets_match_the_manual(void) {
    /* Same bytes as the Rust codec's tests (firmware/pa12). */
    static const uint8_t goal[] = {0xFF, 0xFF, 0xFF, 0x01, 0x05, 0xF3, 0x86, 0xFF, 0x07, 0x7A};
    static const uint8_t read0[] = {0xFF, 0xFF, 0xFF, 0x00, 0x04, 0xF2, 0x8C, 0x02, 0x7B};
    static const uint8_t off1[] = {0xFF, 0xFF, 0xFF, 0x01, 0x04, 0xF3, 0x80, 0x00, 0x87};
    fcec_pa12_t d;
    fcec_pa12_profile_t prof[2] = {profile(0), profile(1)};
    prof[1].safe_action = FCEC_PA12_SAFE_FORCE_OFF;
    uint64_t now = 0;
    setup(&d, prof, now);

    fcec_pa12_service(&d, &ec, now);
    CHECK(sent(0, read0, sizeof read0));
    finish_tx(&d, 0, now);
    finish_tx(&d, 1, now);
    reply(&d, 0, 0, 0, 1500, false, now + MS);
    reply(&d, 1, 1, 0, 1500, false, now + MS);
    now += 30 * MS;
    fcec_pa12_service(&d, &ec, now);
    send(FCEC_KIND_ARM, now, 0, 0);
    CHECK(accepted(FCEC_KIND_ARM));
    finish_tx(&d, 0, now);
    finish_tx(&d, 1, now);
    reply(&d, 0, 0, 0, 1500, false, now);
    reply(&d, 1, 1, 0, 1500, false, now);

    /* 10.47 mm -> 2047 counts with this calibration. */
    now += 10 * MS;
    send(FCEC_KIND_DEMAND, now, 10.47f, 10.47f);
    fcec_pa12_service(&d, &ec, now);
    CHECK(sent(1, goal, sizeof goal));
    finish_tx(&d, 0, now);
    finish_tx(&d, 1, now);

    now += 10 * MS;
    send(FCEC_KIND_TERMINATE, now, 0, 0);
    fcec_pa12_service(&d, &ec, now);
    CHECK(sent(1, off1, sizeof off1));
}

static void test_calibration_and_limits(void) {
    fcec_pa12_profile_t p = profile(0);
    uint16_t c = 0;
    CHECK(fcec_pa12_mm_to_counts(&p, 13.45f, &c) && c == 2345);
    CHECK(fabsf(fcec_pa12_counts_to_mm(&p, 2345) - 13.45f) < 1e-3f);
    CHECK(!fcec_pa12_mm_to_counts(&p, -1.0f, &c) && c == 1000);
    CHECK(!fcec_pa12_mm_to_counts(&p, 30.0f, &c) && c == 3690);
    CHECK(!fcec_pa12_mm_to_counts(&p, NAN, &c) && c == 1000);

    CHECK(fcec_pa12_profile_ok(&p));
    p.approved = false;
    CHECK(!fcec_pa12_profile_ok(&p));
    p = profile(0);
    p.counts_per_mm = 0.0f;
    CHECK(!fcec_pa12_profile_ok(&p));
    p = profile(0);
    p.max_counts = 4096;
    CHECK(!fcec_pa12_profile_ok(&p));
    p = profile(0);
    p.min_counts = p.max_counts;
    CHECK(!fcec_pa12_profile_ok(&p));
    p = profile(0);
    p.id = 0xFE;
    CHECK(!fcec_pa12_profile_ok(&p));
    p = profile(0);
    p.reply_timeout_ms = 0;
    CHECK(!fcec_pa12_profile_ok(&p));
}

static void test_unapproved_port_is_never_touched(void) {
    fcec_pa12_t d;
    fcec_pa12_profile_t prof[2] = {profile(0), profile(0)};
    prof[1].approved = false;
    setup(&d, prof, 0);
    CHECK(ec.capabilities == FCEC_CH_TVC_A1);
    for (uint64_t t = 0; t < 200 * MS; t += 5 * MS) {
        fcec_pa12_service(&d, &ec, t);
        if (uart[0].busy) {
            finish_tx(&d, 0, t);
        }
    }
    CHECK(uart[1].sends == 0);
    CHECK(uart[0].sends > 0);
}

static void test_ready_arm_goal_applied_and_feedback(void) {
    fcec_pa12_t d;
    fcec_pa12_profile_t prof[2] = {profile(0), profile(0)};
    uint64_t now = 0;
    setup(&d, prof, now);

    /* Not ready until each actuator has answered a read. */
    send(FCEC_KIND_ARM, now, 0, 0);
    CHECK(ans.msg.txn_result.reason == FCEC_REASON_NOT_READY);
    /* Disarmed: reads only, never a goal. */
    read_both(&d, &now, 2345);
    CHECK((ec.ready_mask & (FCEC_CH_TVC_A1 | FCEC_CH_TVC_A2)) ==
          (FCEC_CH_TVC_A1 | FCEC_CH_TVC_A2));
    CHECK(d.port[0].feedback_state == FCEC_PA12_FEEDBACK_VALID_REPLY);

    send(FCEC_KIND_ARM, now, 0, 0);
    CHECK(accepted(FCEC_KIND_ARM));

    /* Goal waits for the quiet time after the read, then goes out. */
    send(FCEC_KIND_DEMAND, now, 12.0f, 15.0f);
    uint32_t seq = fc_seq;
    fcec_pa12_service(&d, &ec, now);
    CHECK(!uart[0].busy);
    now += 10 * MS;
    fcec_pa12_service(&d, &ec, now);
    CHECK(sent_goal(0) && goal_counts(0) == 2200);
    CHECK(sent_goal(1) && goal_counts(1) == 2500);
    CHECK(d.port[0].command_state == FCEC_PA12_COMMAND_QUEUED);
    CHECK(ec.last_applied_demand_seq == 0);

    /* Applied once both goals are on the wire. */
    finish_tx(&d, 0, now + 2 * MS);
    fcec_pa12_service(&d, &ec, now + 2 * MS);
    CHECK(ec.last_applied_demand_seq == 0);
    finish_tx(&d, 1, now + 2 * MS);
    fcec_pa12_service(&d, &ec, now + 2 * MS);
    CHECK(ec.last_applied_demand_seq == seq);
    CHECK(fabsf(ec.applied.tvc_extension_mm[1] - 15.0f) < 1e-3f);
    CHECK(d.port[0].command_state == FCEC_PA12_COMMAND_SENT_UNCONFIRMED);

    /* Same command: no second goal. Next is a read. */
    now += 25 * MS;
    fcec_pa12_service(&d, &ec, now);
    CHECK(sent_read(0));
    finish_tx(&d, 0, now);
    reply(&d, 0, 0, 0, 2200, false, now + MS);

    fcec_measurements_t m;
    memset(&m, 0, sizeof m);
    fcec_pa12_publish(&d, &m, now + 3 * MS);
    CHECK((m.validity_mask & 1u) != 0);
    CHECK(m.tvc_position_counts[0] == 2200 && fabsf(m.tvc_position_mm[0] - 12.0f) < 1e-3f);
    CHECK(m.tvc_position_age_ms[0] == 2);
    CHECK(m.pa12_command_request_id[0] == d.port[0].command_request_id);
    CHECK(m.pa12_command_state[0] == FCEC_PA12_COMMAND_SENT_UNCONFIRMED);
    CHECK(m.pa12_feedback_state[0] == FCEC_PA12_FEEDBACK_VALID_REPLY);
    CHECK(m.measurement_quality[0] == FCEC_QUALITY_OK);
}

static void test_silent_a1_does_not_stall_a2(void) {
    fcec_pa12_t d;
    fcec_pa12_profile_t prof[2] = {profile(0), profile(0)};
    uint64_t now = 0;
    setup(&d, prof, now);
    read_both(&d, &now, 2345);
    send(FCEC_KIND_ARM, now, 0, 0);
    CHECK(accepted(FCEC_KIND_ARM));
    now += 20 * MS;

    /* A1 starts a read that is never answered. */
    fcec_pa12_service(&d, &ec, now);
    CHECK(sent_read(0) && sent_read(1));
    finish_tx(&d, 0, now);
    finish_tx(&d, 1, now);
    reply(&d, 1, 0, 0, 2345, false, now + MS);

    /* A2 keeps taking new goals while A1 waits. A due read goes ahead of a
     * goal, so A2 spends some of these periods on its own read. */
    unsigned a2_goals = 0;
    for (int k = 1; k <= 6; k++) {
        now += 10 * MS;
        send(FCEC_KIND_DEMAND, now, 13.0f, 13.0f + (float)k);
        fcec_pa12_service(&d, &ec, now);
        if (uart[0].busy) {
            finish_tx(&d, 0, now); /* A1 keeps sending, and never gets a reply */
        }
        if (uart[1].busy) {
            if (sent_goal(1)) {
                a2_goals++;
            }
            bool was_read = sent_read(1);
            finish_tx(&d, 1, now);
            if (was_read) {
                reply(&d, 1, 0, 0, 2345, false, now + MS);
            }
        }
    }
    CHECK(a2_goals >= 2);
    /* A1's read timed out after 15 ms and it went back to work. */
    CHECK(d.port[0].timeouts >= 1); /* and again each later read */
    CHECK(d.port[0].feedback_state != FCEC_PA12_FEEDBACK_PENDING ||
          d.port[0].feedback_request_id > 1);
    /* A late tx_done for a finished transfer is ignored. */
    fcec_pa12_tx_done(&d, 0, true, now);
    CHECK(ec.state == FCEC_STATE_ARMED);
}

static void test_park_safing_sends_the_safe_goal(void) {
    fcec_pa12_t d;
    fcec_pa12_profile_t prof[2] = {profile(0), profile(0)};
    prof[0].safe_action = FCEC_PA12_SAFE_PARK;
    prof[1].safe_action = FCEC_PA12_SAFE_PARK;
    uint64_t now = 0;
    setup(&d, prof, now);
    read_both(&d, &now, 2345);
    send(FCEC_KIND_ARM, now, 0, 0);
    now += 20 * MS;
    send(FCEC_KIND_TERMINATE, now, 0, 0);
    CHECK(ec.state == FCEC_STATE_SAFING);
    fcec_pa12_service(&d, &ec, now);
    /* Park at tvc_center_mm (13.45 by default) = 2345 counts. */
    CHECK(sent_goal(0) && goal_counts(0) == 2345);
    CHECK(sent_goal(1) && goal_counts(1) == 2345);
}

static void test_hold_safing_stops_where_it_is(void) {
    fcec_pa12_t d;
    fcec_pa12_profile_t prof[2] = {profile(0), profile(0)}; /* HOLD is the default */
    CHECK(prof[0].safe_action == FCEC_PA12_SAFE_HOLD);
    uint64_t now = 0;
    setup(&d, prof, now);
    read_both(&d, &now, 2345);
    send(FCEC_KIND_ARM, now, 0, 0);
    CHECK(accepted(FCEC_KIND_ARM));

    /* Moving towards 20 mm (3000 counts); last measured 2600 on A1. The next
     * read is not due yet (20 ms after the first), so the goal goes out. */
    now += 12 * MS;
    send(FCEC_KIND_DEMAND, now, 20.0f, 20.0f);
    fcec_pa12_service(&d, &ec, now);
    CHECK(sent_goal(0) && goal_counts(0) == 3000);
    finish_tx(&d, 0, now);
    finish_tx(&d, 1, now);
    now += 25 * MS;
    fcec_pa12_service(&d, &ec, now);
    CHECK(sent_read(0));
    finish_tx(&d, 0, now);
    reply(&d, 0, 0, 0, 2600, false, now + MS);
    if (uart[1].busy) {
        finish_tx(&d, 1, now);
        reply(&d, 1, 0, 0, 2700, false, now + MS);
    }

    /* Safing: A1 gets one goal at 2600 (stop), not the 2345 park. */
    now += 15 * MS;
    send(FCEC_KIND_TERMINATE, now, 0, 0);
    fcec_pa12_service(&d, &ec, now);
    CHECK(sent_goal(0) && goal_counts(0) == 2600);
    finish_tx(&d, 0, now);
    if (uart[1].busy) {
        CHECK(sent_goal(1) && goal_counts(1) == 2700);
        finish_tx(&d, 1, now);
    }

    /* Then no more goals while safing, whatever the FC sends. */
    unsigned goals = 0;
    for (int k = 0; k < 10; k++) {
        now += 10 * MS;
        send(FCEC_KIND_DEMAND, now, 5.0f, 5.0f);
        fcec_pa12_service(&d, &ec, now);
        for (unsigned i = 0; i < 2; i++) {
            if (uart[i].busy) {
                goals += sent_goal(i);
                finish_tx(&d, i, now);
                if (sent_read(i)) {
                    reply(&d, i, 0, 0, 2600, false, now + MS);
                }
            }
        }
    }
    CHECK(goals == 0);
}

static void test_force_off_safing_blocks_goals_until_rearmed(void) {
    fcec_pa12_t d;
    fcec_pa12_profile_t prof[2] = {profile(0), profile(0)};
    prof[0].safe_action = FCEC_PA12_SAFE_FORCE_OFF;
    prof[1].safe_action = FCEC_PA12_SAFE_FORCE_OFF;
    uint64_t now = 0;
    setup(&d, prof, now);
    read_both(&d, &now, 2345);
    send(FCEC_KIND_ARM, now, 0, 0);
    now += 20 * MS;
    send(FCEC_KIND_TERMINATE, now, 0, 0);
    fcec_pa12_service(&d, &ec, now);
    CHECK(sent_force_off(0) && sent_force_off(1));
    finish_tx(&d, 0, now);
    finish_tx(&d, 1, now);

    /* Still safing: demands are stored, never sent, and Force-OFF isn't repeated. */
    unsigned goals = 0, offs = 0;
    for (int k = 0; k < 10; k++) {
        now += 10 * MS;
        send(FCEC_KIND_DEMAND, now, 20.0f, 20.0f);
        fcec_pa12_service(&d, &ec, now);
        for (unsigned i = 0; i < 2; i++) {
            if (uart[i].busy) {
                goals += sent_goal(i);
                offs += sent_force_off(i);
                finish_tx(&d, i, now);
                if (sent_read(i)) {
                    reply(&d, i, 0, 0, 2345, false, now + MS);
                }
            }
        }
    }
    CHECK(goals == 0 && offs == 0);

    send(FCEC_KIND_CLEAR_SAFING, now, 0, 0);
    CHECK(accepted(FCEC_KIND_CLEAR_SAFING));
    send(FCEC_KIND_ARM, now, 0, 0);
    CHECK(accepted(FCEC_KIND_ARM));
    now += 20 * MS;
    send(FCEC_KIND_DEMAND, now, 20.0f, 20.0f);
    fcec_pa12_service(&d, &ec, now);
    CHECK(sent_goal(0) && goal_counts(0) == 3000);
}

static void test_bad_replies_faults_and_lost_readiness(void) {
    fcec_pa12_t d;
    fcec_pa12_profile_t prof[2] = {profile(3), profile(3)};
    uint64_t now = 0;
    setup(&d, prof, now);
    read_both(&d, &now, 2345);
    send(FCEC_KIND_ARM, now, 0, 0);
    CHECK(accepted(FCEC_KIND_ARM));

    /* Corrupt checksum, then a reply from the wrong ID: both invalid. */
    now += 20 * MS;
    fcec_pa12_service(&d, &ec, now);
    finish_tx(&d, 0, now);
    finish_tx(&d, 1, now);
    reply(&d, 0, 3, 0, 2345, true, now + MS);
    reply(&d, 1, 4, 0, 2345, false, now + MS);
    CHECK(d.port[0].feedback_state == FCEC_PA12_FEEDBACK_INVALID_REPLY);
    CHECK(d.port[1].feedback_state == FCEC_PA12_FEEDBACK_INVALID_REPLY);
    CHECK(ec.state == FCEC_STATE_ARMED);

    /* Hardware error flags: driver fault, EC safes. */
    now += 30 * MS;
    fcec_pa12_service(&d, &ec, now);
    finish_tx(&d, 0, now);
    reply(&d, 0, 3, 0x20, 2345, false, now + MS);
    CHECK(d.port[0].feedback_state == FCEC_PA12_FEEDBACK_VALID_REPLY_WITH_FAULT);
    fcec_pa12_service(&d, &ec, now + MS);
    CHECK(ec.state == FCEC_STATE_SAFING && ec.safing_reason == FCEC_REASON_LOCAL_FAULT);

    fcec_measurements_t m;
    memset(&m, 0, sizeof m);
    fcec_pa12_publish(&d, &m, now + 2 * MS);
    CHECK(m.pa12_fault_flags[0] == 0x20 && m.measurement_quality[0] == FCEC_QUALITY_DEVICE_FAULT);

    /* Silence: readiness drops after the window. */
    fcec_pa12_t d2;
    setup(&d2, prof, 0);
    now = 0;
    read_both(&d2, &now, 2345);
    CHECK((ec.ready_mask & FCEC_CH_TVC_A1) != 0);
    for (uint64_t t = now; t < now + 600 * MS; t += 5 * MS) {
        fcec_pa12_service(&d2, &ec, t);
        for (unsigned i = 0; i < 2; i++) {
            if (uart[i].busy) {
                finish_tx(&d2, i, t); /* reads go out, nothing answers */
            }
        }
    }
    CHECK((ec.ready_mask & FCEC_CH_TVC_A1) == 0);
    memset(&m, 0, sizeof m);
    fcec_pa12_publish(&d2, &m, now + 600 * MS);
    CHECK((m.validity_mask & 1u) == 0 && m.measurement_quality[0] == FCEC_QUALITY_STALE);

    /* A UART that won't start counts as a failed send, without blocking. */
    fcec_pa12_t d3;
    setup(&d3, prof, 0);
    refuse_tx = true;
    fcec_pa12_service(&d3, &ec, 0);
    CHECK(d3.port[0].feedback_state == FCEC_PA12_FEEDBACK_TRANSPORT_ERROR);
    CHECK(d3.port[0].phase == FCEC_PA12_PHASE_IDLE);
}

/* The FC sends a demand every 5 ms. Each is a new goal, so with goals ahead of
 * reads a goal was always pending when the quiet time ended and no read was
 * ever sent: readiness lapsed 500 ms after Arm and the EC safed with
 * LOCAL_FAULT. Reads must still happen, readiness must hold, and goals must
 * keep going out. */
static void test_reads_are_not_starved_by_a_steady_demand_stream(void) {
    fcec_pa12_t d;
    fcec_pa12_profile_t prof[2] = {profile(0), profile(0)};
    uint64_t now = 0;
    setup(&d, prof, now);
    read_both(&d, &now, 2345);
    send(FCEC_KIND_ARM, now, 0, 0);
    CHECK(accepted(FCEC_KIND_ARM));

    unsigned seen_sends[2] = {uart[0].sends, uart[1].sends};
    uint64_t finish_at[2] = {0, 0};
    uint64_t reply_at[2] = {0, 0};
    bool awaiting[2] = {false, false};
    unsigned reads[2] = {0, 0};
    unsigned goals[2] = {0, 0};
    bool stayed_armed = true;
    bool stayed_ready = true;
    uint64_t end = now + 3000 * MS;
    uint64_t start_t = now;

    for (uint64_t t = now; t < end; t += MS) {
        if ((t - start_t) % (5 * MS) == 0) {
            /* a different goal every time, as a controller's output is */
            float a1 = 10.0f + (float)(((t - start_t) / (5 * MS)) % 9) * 0.25f;
            send(FCEC_KIND_DEMAND, t, a1, 12.0f);
        }
        for (unsigned i = 0; i < 2; i++) {
            if (uart[i].busy && t >= finish_at[i]) {
                bool was_read = sent_read(i);
                finish_tx(&d, i, t);
                if (was_read) {
                    awaiting[i] = true;
                    reply_at[i] = t + 2 * MS;
                }
            }
            if (awaiting[i] && t >= reply_at[i]) {
                reply(&d, i, 0, 0, 2345, false, t);
                awaiting[i] = false;
                reads[i]++;
            }
        }
        fcec_pa12_service(&d, &ec, t);
        for (unsigned i = 0; i < 2; i++) {
            if (uart[i].sends != seen_sends[i]) {
                seen_sends[i] = uart[i].sends;
                finish_at[i] = t + MS;
                goals[i] += sent_goal(i) ? 1u : 0u;
            }
        }
        stayed_armed = stayed_armed && ec.state == FCEC_STATE_ARMED;
        stayed_ready = stayed_ready && (ec.ready_mask & (FCEC_CH_TVC_A1 | FCEC_CH_TVC_A2)) ==
                                           (FCEC_CH_TVC_A1 | FCEC_CH_TVC_A2);
    }

    CHECK(stayed_armed);
    CHECK(stayed_ready);
    for (unsigned i = 0; i < 2; i++) {
        /* about one read per 20 ms poll period (plus the bus time of the read) */
        CHECK(reads[i] >= 3000u / 40u);
        /* and the goals keep flowing: far more than one per poll period */
        CHECK(goals[i] >= 3000u / 40u);
    }
}

int main(void) {
    test_packets_match_the_manual();
    test_calibration_and_limits();
    test_unapproved_port_is_never_touched();
    test_ready_arm_goal_applied_and_feedback();
    test_silent_a1_does_not_stall_a2();
    test_park_safing_sends_the_safe_goal();
    test_hold_safing_stops_where_it_is();
    test_force_off_safing_blocks_goals_until_rearmed();
    test_bad_replies_faults_and_lost_readiness();
    test_reads_are_not_starved_by_a_steady_demand_stream();

    printf("%d checks, %d failed\n", checks, failures);
    return failures == 0 ? 0 : 1;
}
