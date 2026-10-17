/*
 * Host tests for the EC-side link logic, with simulated time.
 *
 * Run with `make test` in this folder.
 */

#include "ec_side.h"

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

#define BOOT_ID 0xB0070001u
#define SESSION_ID 0x5E550001u
#define T0 1000000u /* EC time of the first message, in us */
#define MS 1000u

static fcec_ec_t ec;
static uint32_t fc_seq;
static uint32_t fc_txn;
static fcec_packet_t reply;
static size_t reply_len;

/* An FC packet that is fresh at `now`: it echoes `now` as the EC time it
 * last saw. */
static void make(fcec_packet_t *p, fcec_kind_t kind, uint64_t now) {
    memset(p, 0, sizeof *p);
    p->kind = kind;
    p->header.ec_boot_id = BOOT_ID;
    p->header.session_id = SESSION_ID;
    p->header.seq = ++fc_seq;
    p->header.valid_for_ms = 50;
    p->header.sent_time_us = now;
    p->header.ec_time_ref_us = now;
}

/* Delivers `p` to the EC at `now` and decodes any reply into `reply`. */
static size_t deliver(const fcec_packet_t *p, uint64_t now) {
    uint8_t rx[FCEC_MAX_DATAGRAM_LEN];
    uint8_t tx[FCEC_MAX_DATAGRAM_LEN];
    int len = fcec_encode(p, rx, sizeof rx);
    CHECK(len > 0);

    reply_len = fcec_ec_receive(&ec, rx, (size_t)len, now, tx, sizeof tx);
    memset(&reply, 0, sizeof reply);
    if (reply_len > 0) {
        CHECK(fcec_decode(tx, reply_len, &reply) == FCEC_OK);
    }
    return reply_len;
}

static void send_kind(fcec_kind_t kind, uint64_t now) {
    fcec_packet_t p;
    make(&p, kind, now);
    if (kind != FCEC_KIND_DEMAND && kind != FCEC_KIND_HEARTBEAT) {
        p.header.txn_id = ++fc_txn;
    }
    if (kind == FCEC_KIND_HELLO) {
        p.msg.hello.demand_period_ms = 10;
        p.msg.hello.heartbeat_period_ms = 50;
    }
    deliver(&p, now);
}

static void demand_packet(fcec_packet_t *p, uint64_t now) {
    make(p, FCEC_KIND_DEMAND, now);
    p->msg.demand.outputs.tvc_extension_mm[0] = 10.0f;
    p->msg.demand.outputs.tvc_extension_mm[1] = 20.0f;
    p->msg.demand.outputs.rcs_duty[0] = 0.1f;
    p->msg.demand.outputs.rcs_duty[1] = 0.2f;
    p->msg.demand.outputs.rcs_duty[2] = 0.3f;
    p->msg.demand.outputs.rcs_duty[3] = 0.4f;
    p->msg.demand.outputs.main_throttle = 0.7f;
    p->msg.demand.enable_mask = FCEC_CH_ALL;
}

static void send_demand(uint64_t now) {
    fcec_packet_t p;
    demand_packet(&p, now);
    deliver(&p, now);
}

static bool reply_is(fcec_kind_t request, fcec_outcome_t outcome, fcec_reason_t reason) {
    return reply_len > 0 && reply.kind == FCEC_KIND_TXN_RESULT &&
           reply.msg.txn_result.request_kind == request &&
           reply.msg.txn_result.outcome == outcome && reply.msg.txn_result.reason == reason;
}

static bool outputs_safe(bool tvc_centered) {
    const fcec_outputs_t *o = fcec_ec_outputs(&ec);
    bool ok = o->main_throttle == 0.0f;
    for (size_t i = 0; i < 4; i++) {
        ok = ok && o->rcs_duty[i] == 0.0f;
    }
    if (tvc_centered) {
        ok = ok && o->tvc_extension_mm[0] == FCEC_TVC_STROKE_MM / 2.0f &&
             o->tvc_extension_mm[1] == FCEC_TVC_STROKE_MM / 2.0f;
    }
    return ok;
}

/* Fresh EC, booted, with a session opened at T0. */
static void start(void) {
    fcec_ec_init(&ec, BOOT_ID);
    fcec_ec_boot_done(&ec);
    fc_seq = 0;
    fc_txn = 0;
    send_kind(FCEC_KIND_HELLO, T0);
}

/* start(), armed at T0 + 1 ms. */
static void start_armed(void) {
    start();
    send_kind(FCEC_KIND_ARM, T0 + 1 * MS);
}

static void test_boot_and_hello(void) {
    fcec_ec_init(&ec, BOOT_ID);
    fc_seq = 0;
    fc_txn = 0;
    CHECK(ec.state == FCEC_STATE_BOOT_SAFE);
    CHECK(outputs_safe(true));
    CHECK(!fcec_ec_drive_tvc(&ec));

    send_kind(FCEC_KIND_HELLO, T0);
    CHECK(reply.kind == FCEC_KIND_HELLO_RESULT);
    CHECK(reply.msg.hello_result.outcome == FCEC_OUTCOME_REJECTED);
    CHECK(reply.msg.hello_result.reason == FCEC_REASON_NOT_READY);
    CHECK(!ec.session_valid);

    fcec_ec_boot_done(&ec);
    CHECK(ec.state == FCEC_STATE_DISARMED);

    send_kind(FCEC_KIND_HELLO, T0 + 1 * MS);
    CHECK(reply.kind == FCEC_KIND_HELLO_RESULT);
    CHECK(reply.msg.hello_result.outcome == FCEC_OUTCOME_ACCEPTED);
    CHECK(reply.msg.hello_result.ec_state == FCEC_STATE_DISARMED);
    CHECK(reply.msg.hello_result.capabilities == FCEC_CH_ALL);
    CHECK(reply.msg.hello_result.max_demand_age_ms == 50);
    CHECK(reply.msg.hello_result.link_timeout_ms == 350);
    CHECK(reply.header.ec_boot_id == BOOT_ID);
    CHECK(reply.header.session_id == SESSION_ID);
    CHECK(reply.header.txn_id == fc_txn);
    CHECK(reply.header.sent_time_us == T0 + 1 * MS);
    CHECK(ec.session_valid && ec.session_id == SESSION_ID);

    /* A retried Hello in the same session is answered again. */
    send_kind(FCEC_KIND_HELLO, T0 + 2 * MS);
    CHECK(reply.msg.hello_result.outcome == FCEC_OUTCOME_ACCEPTED);

    /* Session 0 means "no session" and can't be opened. */
    fcec_ec_init(&ec, BOOT_ID);
    fcec_ec_boot_done(&ec);
    fcec_packet_t p;
    make(&p, FCEC_KIND_HELLO, T0);
    p.header.session_id = 0;
    deliver(&p, T0);
    CHECK(reply.msg.hello_result.reason == FCEC_REASON_BAD_SESSION);
}

static void test_demands_need_freshness_and_arming(void) {
    start();
    fcec_packet_t p;

    /* Echoes an EC time 60 ms old: stale. */
    demand_packet(&p, T0 + 70 * MS);
    p.header.ec_time_ref_us = T0 + 10 * MS;
    CHECK(deliver(&p, T0 + 70 * MS) == 0);
    CHECK(ec.counters.stale == 1);
    CHECK(ec.last_reject_reason == FCEC_REASON_STALE);
    CHECK(!ec.have_demand);

    /* Expired: valid for 0 ms after an EC time 1 ms ago. */
    demand_packet(&p, T0 + 71 * MS);
    p.header.ec_time_ref_us = T0 + 70 * MS;
    p.header.valid_for_ms = 0;
    deliver(&p, T0 + 71 * MS);
    CHECK(ec.counters.stale == 2);

    /* Echoes an EC time from the future. */
    demand_packet(&p, T0 + 72 * MS);
    p.header.ec_time_ref_us = T0 + 80 * MS;
    deliver(&p, T0 + 72 * MS);
    CHECK(ec.counters.stale == 3);

    /* Fresh but disarmed: stored, not applied. */
    send_demand(T0 + 73 * MS);
    CHECK(ec.have_demand && ec.last_accepted_demand_seq == fc_seq);
    CHECK(ec.last_applied_demand_seq == 0);
    CHECK(outputs_safe(true));

    send_kind(FCEC_KIND_ARM, T0 + 74 * MS);
    CHECK(reply_is(FCEC_KIND_ARM, FCEC_OUTCOME_ACCEPTED, FCEC_REASON_NONE));
    CHECK(reply.msg.txn_result.ec_state == FCEC_STATE_ARMED);
    CHECK(ec.state == FCEC_STATE_ARMED && fcec_ec_drive_tvc(&ec));
    /* Arming doesn't replay the stored demand. */
    CHECK(outputs_safe(true));

    send_demand(T0 + 75 * MS);
    const fcec_outputs_t *o = fcec_ec_outputs(&ec);
    CHECK(o->tvc_extension_mm[0] == 10.0f && o->tvc_extension_mm[1] == 20.0f);
    CHECK(o->rcs_duty[3] == 0.4f && o->main_throttle == 0.7f);
    CHECK(ec.last_applied_demand_seq == fc_seq);
    CHECK(ec.last_applied_time_us == T0 + 75 * MS);

    /* An older demand arriving late is dropped. */
    uint32_t applied_seq = fc_seq;
    demand_packet(&p, T0 + 76 * MS);
    p.header.seq = applied_seq - 1;
    p.msg.demand.outputs.main_throttle = 0.1f;
    deliver(&p, T0 + 76 * MS);
    CHECK(ec.counters.old_or_duplicate == 1);
    CHECK(fcec_ec_outputs(&ec)->main_throttle == 0.7f);

    /* NaN anywhere rejects the whole demand. */
    demand_packet(&p, T0 + 77 * MS);
    p.msg.demand.outputs.rcs_duty[1] = NAN;
    deliver(&p, T0 + 77 * MS);
    CHECK(ec.last_reject_reason == FCEC_REASON_NON_FINITE);
    CHECK(ec.last_applied_demand_seq == applied_seq);
}

static void test_disabled_channels_stay_safe(void) {
    start_armed();
    fcec_packet_t p;
    demand_packet(&p, T0 + 2 * MS);
    p.msg.demand.enable_mask = FCEC_CH_TVC_A1 | FCEC_CH_TVC_A2;
    deliver(&p, T0 + 2 * MS);

    const fcec_outputs_t *o = fcec_ec_outputs(&ec);
    CHECK(o->tvc_extension_mm[0] == 10.0f);
    CHECK(o->main_throttle == 0.0f && o->rcs_duty[0] == 0.0f);
}

static void test_unsupported_channel_is_rejected(void) {
    start_armed();
    ec.capabilities = FCEC_CH_ALL & ~FCEC_CH_MAIN_ESC;
    send_demand(T0 + 2 * MS);
    CHECK(ec.last_reject_reason == FCEC_REASON_UNSUPPORTED_CHANNEL);
    CHECK(ec.last_applied_demand_seq == 0);
}

static void test_demand_timeout_safes_and_latches(void) {
    start_armed();
    send_demand(T0 + 2 * MS);

    /* 3 periods of 10 ms after the last demand is still fine; past it isn't. */
    fcec_ec_tick(&ec, T0 + 32 * MS);
    CHECK(ec.state == FCEC_STATE_ARMED);
    fcec_ec_tick(&ec, T0 + 32 * MS + 1);
    CHECK(ec.state == FCEC_STATE_SAFING);
    CHECK(ec.safing_reason == FCEC_REASON_DEMAND_TIMEOUT);
    CHECK(outputs_safe(true));
    CHECK(fcec_ec_drive_tvc(&ec));

    /* Latched: demands don't move anything and Arm is refused. */
    send_demand(T0 + 40 * MS);
    CHECK(outputs_safe(true));
    send_kind(FCEC_KIND_ARM, T0 + 41 * MS);
    CHECK(reply_is(FCEC_KIND_ARM, FCEC_OUTCOME_REJECTED, FCEC_REASON_SAFING_LATCHED));

    /* Only ClearSafing unlatches, and it lands in DISARMED, not ARMED. */
    send_kind(FCEC_KIND_CLEAR_SAFING, T0 + 42 * MS);
    CHECK(reply_is(FCEC_KIND_CLEAR_SAFING, FCEC_OUTCOME_ACCEPTED, FCEC_REASON_NONE));
    CHECK(ec.state == FCEC_STATE_DISARMED);
    CHECK(ec.safing_reason == FCEC_REASON_NONE);

    send_kind(FCEC_KIND_CLEAR_SAFING, T0 + 43 * MS);
    CHECK(reply_is(FCEC_KIND_CLEAR_SAFING, FCEC_OUTCOME_REJECTED, FCEC_REASON_WRONG_STATE));

    send_kind(FCEC_KIND_ARM, T0 + 44 * MS);
    CHECK(ec.state == FCEC_STATE_ARMED);
}

static void test_arming_without_demands_times_out(void) {
    start_armed(); /* armed at T0 + 1 ms */
    fcec_ec_tick(&ec, T0 + 31 * MS);
    CHECK(ec.state == FCEC_STATE_ARMED);
    fcec_ec_tick(&ec, T0 + 31 * MS + 1);
    CHECK(ec.state == FCEC_STATE_SAFING);
    CHECK(ec.safing_reason == FCEC_REASON_DEMAND_TIMEOUT);
}

static void test_heartbeats_dont_feed_the_demand_timer(void) {
    start_armed();
    send_demand(T0 + 2 * MS);
    send_kind(FCEC_KIND_HEARTBEAT, T0 + 20 * MS);
    send_kind(FCEC_KIND_HEARTBEAT, T0 + 30 * MS);
    fcec_ec_tick(&ec, T0 + 33 * MS);
    CHECK(ec.state == FCEC_STATE_SAFING);
    CHECK(ec.safing_reason == FCEC_REASON_DEMAND_TIMEOUT);
}

static void test_terminate_always_safes(void) {
    start_armed();
    send_demand(T0 + 2 * MS);

    fcec_packet_t p;
    make(&p, FCEC_KIND_TERMINATE, T0 + 3 * MS);
    p.header.ec_boot_id = 0xDEADu; /* wrong boot, no session: still obeyed */
    p.header.session_id = 0;
    p.header.txn_id = 99;
    p.msg.terminate.reason = FCEC_REASON_FC_TERMINATE;
    deliver(&p, T0 + 3 * MS);

    CHECK(reply_is(FCEC_KIND_TERMINATE, FCEC_OUTCOME_ACCEPTED, FCEC_REASON_NONE));
    CHECK(reply.header.txn_id == 99);
    CHECK(ec.state == FCEC_STATE_SAFING);
    CHECK(ec.safing_reason == FCEC_REASON_FC_TERMINATE);
    CHECK(outputs_safe(true));

    /* A later timeout doesn't overwrite the first reason. */
    fcec_ec_tick(&ec, T0 + 900 * MS);
    CHECK(ec.safing_reason == FCEC_REASON_FC_TERMINATE);
}

static void test_link_timeout_drops_session_and_safes(void) {
    start(); /* Hello at T0 */
    send_kind(FCEC_KIND_HEARTBEAT, T0 + 300 * MS);
    fcec_ec_tick(&ec, T0 + 650 * MS);
    CHECK(ec.session_valid);
    CHECK(ec.state == FCEC_STATE_DISARMED);

    fcec_ec_tick(&ec, T0 + 650 * MS + 1);
    CHECK(!ec.session_valid);
    CHECK(ec.state == FCEC_STATE_SAFING);
    CHECK(ec.safing_reason == FCEC_REASON_LINK_TIMEOUT);

    /* The FC comes back: new Hello, then ClearSafing, then Arm. */
    send_kind(FCEC_KIND_HELLO, T0 + 700 * MS);
    CHECK(reply.msg.hello_result.outcome == FCEC_OUTCOME_ACCEPTED);
    CHECK(reply.msg.hello_result.ec_state == FCEC_STATE_SAFING);
    send_kind(FCEC_KIND_CLEAR_SAFING, T0 + 701 * MS);
    send_kind(FCEC_KIND_ARM, T0 + 702 * MS);
    CHECK(ec.state == FCEC_STATE_ARMED);
}

static void test_duplicate_transactions_have_no_side_effects(void) {
    start();
    fcec_packet_t arm;
    make(&arm, FCEC_KIND_ARM, T0 + 1 * MS);
    arm.header.txn_id = ++fc_txn;
    deliver(&arm, T0 + 1 * MS);
    CHECK(ec.state == FCEC_STATE_ARMED);

    send_kind(FCEC_KIND_DISARM, T0 + 2 * MS);
    CHECK(reply_is(FCEC_KIND_DISARM, FCEC_OUTCOME_ACCEPTED, FCEC_REASON_NONE));
    CHECK(ec.state == FCEC_STATE_DISARMED);

    /* The first Arm arrives again: original answer, nothing re-armed. */
    arm.header.seq = ++fc_seq;
    deliver(&arm, T0 + 3 * MS);
    CHECK(reply_is(FCEC_KIND_ARM, FCEC_OUTCOME_ACCEPTED, FCEC_REASON_NONE));
    CHECK(reply.header.duplicate);
    CHECK(reply.msg.txn_result.ec_state == FCEC_STATE_ARMED);
    CHECK(ec.state == FCEC_STATE_DISARMED);

    /* A transaction older than the newest and no longer remembered. */
    fcec_packet_t old;
    make(&old, FCEC_KIND_ARM, T0 + 4 * MS);
    old.header.txn_id = fc_txn - 100;
    deliver(&old, T0 + 4 * MS);
    CHECK(reply_is(FCEC_KIND_ARM, FCEC_OUTCOME_REJECTED, FCEC_REASON_STALE));
    CHECK(ec.state == FCEC_STATE_DISARMED);

    /* Disarm needs no freshness: it only removes authority. */
    send_kind(FCEC_KIND_ARM, T0 + 5 * MS);
    fcec_packet_t disarm;
    make(&disarm, FCEC_KIND_DISARM, T0 + 500 * MS);
    disarm.header.txn_id = ++fc_txn;
    disarm.header.ec_time_ref_us = 0;
    deliver(&disarm, T0 + 6 * MS);
    CHECK(ec.state == FCEC_STATE_DISARMED);
}

static void test_wrong_boot_or_session_is_rejected(void) {
    start();
    fcec_packet_t p;

    make(&p, FCEC_KIND_ARM, T0 + 1 * MS);
    p.header.txn_id = ++fc_txn;
    p.header.ec_boot_id = BOOT_ID + 1; /* from before an EC reboot */
    deliver(&p, T0 + 1 * MS);
    CHECK(reply_is(FCEC_KIND_ARM, FCEC_OUTCOME_REJECTED, FCEC_REASON_BAD_BOOT_ID));
    CHECK(reply.header.ec_boot_id == BOOT_ID); /* tells the FC the real one */

    make(&p, FCEC_KIND_ARM, T0 + 2 * MS);
    p.header.txn_id = ++fc_txn;
    p.header.session_id = SESSION_ID + 1;
    deliver(&p, T0 + 2 * MS);
    CHECK(reply_is(FCEC_KIND_ARM, FCEC_OUTCOME_REJECTED, FCEC_REASON_BAD_SESSION));

    demand_packet(&p, T0 + 3 * MS);
    p.header.session_id = SESSION_ID + 1;
    CHECK(deliver(&p, T0 + 3 * MS) == 0);
    CHECK(!ec.have_demand);
    CHECK(ec.state == FCEC_STATE_DISARMED);
}

static void test_new_session_refused_while_armed(void) {
    start_armed();
    send_demand(T0 + 2 * MS);

    fcec_packet_t p;
    make(&p, FCEC_KIND_HELLO, T0 + 3 * MS);
    p.header.session_id = SESSION_ID + 1;
    p.header.txn_id = ++fc_txn;
    deliver(&p, T0 + 3 * MS);
    CHECK(reply.msg.hello_result.outcome == FCEC_OUTCOME_REJECTED);
    CHECK(reply.msg.hello_result.reason == FCEC_REASON_WRONG_STATE);
    CHECK(ec.session_id == SESSION_ID);
    CHECK(ec.state == FCEC_STATE_ARMED);
}

static void test_status_reports_the_link(void) {
    start_armed();
    send_demand(T0 + 2 * MS);
    ec.measurements.sample_id = 5;
    ec.measurements.validity_mask = 0x3;
    ec.measurements.tvc_position_mm[0] = 9.5f;

    uint8_t buf[FCEC_MAX_DATAGRAM_LEN];
    size_t len = fcec_ec_status(&ec, T0 + 3 * MS, buf, sizeof buf);
    CHECK(len == 220);

    fcec_packet_t s;
    CHECK(fcec_decode(buf, len, &s) == FCEC_OK);
    CHECK(s.kind == FCEC_KIND_STATUS);
    CHECK(s.header.ec_boot_id == BOOT_ID && s.header.session_id == SESSION_ID);
    CHECK(s.header.sent_time_us == T0 + 3 * MS);
    CHECK(s.msg.status.ec_state == FCEC_STATE_ARMED);
    CHECK(s.msg.status.output_authority && s.msg.status.session_valid);
    CHECK(s.msg.status.demand_fresh && !s.msg.status.safing_latched);
    CHECK(s.msg.status.last_received_seq == fc_seq);
    CHECK(s.msg.status.last_accepted_demand_seq == fc_seq);
    CHECK(s.msg.status.last_applied_demand_seq == fc_seq);
    CHECK(s.msg.status.applied.main_throttle == 0.7f);
    CHECK(s.msg.status.measurements.sample_id == 5);
    CHECK(s.msg.status.measurements.tvc_position_mm[0] == 9.5f);

    /* The EC numbers its own messages. */
    uint32_t first = s.header.seq;
    len = fcec_ec_status(&ec, T0 + 13 * MS, buf, sizeof buf);
    CHECK(fcec_decode(buf, len, &s) == FCEC_OK);
    CHECK(s.header.seq == first + 1);
}

static void test_corrupt_datagrams_are_counted(void) {
    start();
    uint8_t rx[FCEC_MAX_DATAGRAM_LEN];
    uint8_t tx[FCEC_MAX_DATAGRAM_LEN];
    fcec_packet_t p;
    demand_packet(&p, T0 + 1 * MS);
    int len = fcec_encode(&p, rx, sizeof rx);
    rx[60] ^= 0x10;

    CHECK(fcec_ec_receive(&ec, rx, (size_t)len, T0 + 1 * MS, tx, sizeof tx) == 0);
    CHECK(ec.counters.crc_errors == 1);
    CHECK(!ec.have_demand);
}

int main(void) {
    test_boot_and_hello();
    test_demands_need_freshness_and_arming();
    test_disabled_channels_stay_safe();
    test_unsupported_channel_is_rejected();
    test_demand_timeout_safes_and_latches();
    test_arming_without_demands_times_out();
    test_heartbeats_dont_feed_the_demand_timer();
    test_terminate_always_safes();
    test_link_timeout_drops_session_and_safes();
    test_duplicate_transactions_have_no_side_effects();
    test_wrong_boot_or_session_is_rejected();
    test_new_session_refused_while_armed();
    test_status_reports_the_link();
    test_corrupt_datagrams_are_counted();

    printf("%d checks, %d failed\n", checks, failures);
    return failures == 0 ? 0 : 1;
}
