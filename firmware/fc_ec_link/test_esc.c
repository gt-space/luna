/*
 * Host tests for the ESC output logic (esc.c) and its link to the EC state
 * machine, with simulated time and made-up profile numbers.
 *
 * Run with `make test` in this folder.
 */

#include "esc.h"

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
#define FRAME_US 2500u

/* Test numbers only, not a real ESC profile. */
static fcec_esc_config_t profile(void) {
    fcec_esc_config_t c;
    memset(&c, 0, sizeof c);
    c.approved = true;
    c.min_pulse_us = 1000;
    c.max_pulse_us = 2000;
    c.frame_period_us = FRAME_US;
    c.arm_hold_ms = 2000;
    c.max_throttle = 0.7f;
    c.max_rise_per_s = 1.0f;
    return c;
}

static void test_profile_must_be_approved_and_consistent(void) {
    fcec_esc_config_t c = profile();
    CHECK(fcec_esc_config_ok(&c));
    CHECK(!fcec_esc_config_ok(NULL));

    c.approved = false;
    CHECK(!fcec_esc_config_ok(&c));
    c = profile();
    c.min_pulse_us = 0;
    CHECK(!fcec_esc_config_ok(&c));
    c = profile();
    c.max_pulse_us = c.min_pulse_us;
    CHECK(!fcec_esc_config_ok(&c));
    c = profile();
    c.frame_period_us = c.max_pulse_us;
    CHECK(!fcec_esc_config_ok(&c));
    c = profile();
    c.max_throttle = 0.0f;
    CHECK(!fcec_esc_config_ok(&c));
    c.max_throttle = 1.5f;
    CHECK(!fcec_esc_config_ok(&c));
    c.max_throttle = NAN;
    CHECK(!fcec_esc_config_ok(&c));
    c = profile();
    c.max_rise_per_s = -1.0f;
    CHECK(!fcec_esc_config_ok(&c));
    c.max_rise_per_s = INFINITY;
    CHECK(!fcec_esc_config_ok(&c));
    c.max_rise_per_s = NAN;
    CHECK(!fcec_esc_config_ok(&c));
    c.max_rise_per_s = 0.0f; /* no slew limit */
    CHECK(fcec_esc_config_ok(&c));
}

static void test_no_slew_limit_jumps_straight_to_target(void) {
    fcec_esc_config_t c = profile();
    c.max_rise_per_s = 0.0f;
    c.max_throttle = 1.0f;
    fcec_esc_t esc;
    fcec_esc_init(&esc, &c, 0);
    fcec_esc_update(&esc, 0.0f, true, 2000 * MS);
    CHECK(fcec_esc_update(&esc, 1.0f, true, 2001 * MS) == 2000);
    CHECK(fcec_esc_update(&esc, 0.25f, true, 2002 * MS) == 1250);
    /* Still held at the safe pulse during the arming hold. */
    fcec_esc_init(&esc, &c, 0);
    CHECK(fcec_esc_update(&esc, 1.0f, true, 1000 * MS) == 1000);
}

static void test_unconfigured_esc_sends_no_signal(void) {
    fcec_esc_config_t c = profile();
    c.approved = false;
    fcec_esc_t esc;
    fcec_esc_init(&esc, &c, 0);
    CHECK(esc.state == FCEC_ESC_UNCONFIGURED);
    CHECK(fcec_esc_update(&esc, 0.5f, true, 10000 * MS) == 0);
    CHECK(!fcec_esc_ready(&esc));
    fcec_esc_fault(&esc);
    CHECK(esc.state == FCEC_ESC_UNCONFIGURED);
}

static void test_holds_safe_pulse_until_armed(void) {
    fcec_esc_config_t c = profile();
    fcec_esc_t esc;
    fcec_esc_init(&esc, &c, 0);
    CHECK(esc.state == FCEC_ESC_ARMING);
    CHECK(esc.pulse_us == 1000);

    /* A throttle request before the hold is over is ignored. */
    CHECK(fcec_esc_update(&esc, 0.5f, true, 1999 * MS) == 1000);
    CHECK(!fcec_esc_ready(&esc));
    CHECK(fcec_esc_update(&esc, 0.0f, true, 2000 * MS) == 1000);
    CHECK(fcec_esc_ready(&esc));
}

static void test_throttle_is_capped_slewed_and_cut_at_once(void) {
    fcec_esc_config_t c = profile();
    fcec_esc_t esc;
    fcec_esc_init(&esc, &c, 0);
    fcec_esc_update(&esc, 0.0f, true, 2000 * MS);

    /* 1.0/s rise: 100 ms buys 0.1. */
    CHECK(fcec_esc_update(&esc, 1.0f, true, 2100 * MS) == 1100);
    CHECK(fabsf(esc.throttle - 0.1f) < 1e-4f);
    fcec_esc_update(&esc, 1.0f, true, 2400 * MS);
    CHECK(fabsf(esc.throttle - 0.4f) < 1e-4f);
    /* Held at the 0.7 cap however long it asks. */
    CHECK(fcec_esc_update(&esc, 1.0f, true, 5000 * MS) == 1700);

    /* Down is immediate, and so is "not driving". */
    CHECK(fcec_esc_update(&esc, 0.2f, true, 5001 * MS) == 1200);
    CHECK(fcec_esc_update(&esc, 0.6f, false, 5002 * MS) == 1000);
    /* Junk targets mean zero. */
    CHECK(fcec_esc_update(&esc, NAN, true, 6000 * MS) == 1000);
    CHECK(fcec_esc_update(&esc, -0.5f, true, 7000 * MS) == 1000);
}

static void test_fault_holds_safe_and_rearms_from_scratch(void) {
    fcec_esc_config_t c = profile();
    fcec_esc_t esc;
    fcec_esc_init(&esc, &c, 0);
    fcec_esc_update(&esc, 0.0f, true, 2000 * MS);
    fcec_esc_update(&esc, 0.5f, true, 3000 * MS);
    CHECK(esc.pulse_us > 1000);

    fcec_esc_fault(&esc);
    CHECK(!fcec_esc_ready(&esc) && esc.pulse_us == 1000);
    CHECK(fcec_esc_update(&esc, 0.5f, true, 4000 * MS) == 1000);

    fcec_esc_clear_fault(&esc, 4000 * MS);
    CHECK(esc.state == FCEC_ESC_ARMING);
    fcec_esc_update(&esc, 0.0f, true, 5999 * MS);
    CHECK(!fcec_esc_ready(&esc));
    fcec_esc_update(&esc, 0.0f, true, 6000 * MS);
    CHECK(fcec_esc_ready(&esc));
}

/* ---- With the EC state machine ---- */

#define BOOT_ID 0xB0070002u
#define SESSION_ID 0x5E550002u

static fcec_ec_t ec;
static uint32_t fc_seq;
static uint32_t fc_txn;
static fcec_packet_t reply;

static void send(fcec_kind_t kind, uint64_t now, float throttle) {
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
        p.msg.demand.outputs.main_throttle = throttle;
        p.msg.demand.enable_mask = FCEC_CH_MAIN_ESC;
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
    memset(&reply, 0, sizeof reply);
    if (n > 0) {
        CHECK(fcec_decode(tx, n, &reply) == FCEC_OK);
    }
}

static bool txn_ok(fcec_kind_t kind) {
    return reply.kind == FCEC_KIND_TXN_RESULT && reply.msg.txn_result.request_kind == kind &&
           reply.msg.txn_result.outcome == FCEC_OUTCOME_ACCEPTED;
}

static void test_esc_drives_the_ec_channel(void) {
    fcec_esc_config_t c = profile();
    fcec_esc_t esc;
    fcec_ec_init(&ec, BOOT_ID);
    ec.auto_confirm = false;
    ec.capabilities = FCEC_CH_MAIN_ESC;
    fcec_ec_set_ready(&ec, FCEC_CH_ALL, false);
    fcec_esc_init(&esc, &c, 0);
    fcec_ec_boot_done(&ec);
    fc_seq = 0;
    fc_txn = 0;

    /* Arming hold: the EC can't be armed yet. */
    uint64_t t = 1000 * MS;
    send(FCEC_KIND_HELLO, t, 0.0f);
    CHECK(fcec_esc_service(&esc, &ec, t) == 1000);
    send(FCEC_KIND_ARM, t, 0.0f);
    CHECK(reply.msg.txn_result.reason == FCEC_REASON_NOT_READY);

    t = 2000 * MS;
    fcec_esc_service(&esc, &ec, t);
    CHECK((ec.ready_mask & FCEC_CH_MAIN_ESC) != 0);
    send(FCEC_KIND_ARM, t, 0.0f);
    CHECK(txn_ok(FCEC_KIND_ARM));

    /* A demand is applied only after its pulse has been on the wire a frame. */
    send(FCEC_KIND_DEMAND, t + 1 * MS, 0.5f);
    uint32_t seq = fc_seq;
    fcec_esc_service(&esc, &ec, t + 1 * MS);
    CHECK(ec.last_applied_demand_seq == 0);
    fcec_esc_service(&esc, &ec, t + 1 * MS + FRAME_US);
    CHECK(ec.last_applied_demand_seq == seq);
    /* Slewing: what was applied is still far below the 0.5 asked for. */
    CHECK(ec.applied.main_throttle < 0.01f);

    /* Keep the demand fresh and let it ramp; applied tracks the real output. */
    uint64_t now = t + 1 * MS + FRAME_US;
    for (int i = 0; i < 60; i++) {
        now += 10 * MS;
        send(FCEC_KIND_DEMAND, now, 0.5f);
        fcec_esc_service(&esc, &ec, now);
        now += FRAME_US;
        fcec_esc_service(&esc, &ec, now);
    }
    CHECK(fabsf(esc.throttle - 0.5f) < 1e-4f);
    CHECK(fabsf(ec.applied.main_throttle - 0.5f) < 1e-4f);
    CHECK(ec.state == FCEC_STATE_ARMED);

    /* Terminate: the next pulse is the safe one, and the EC sees it applied. */
    send(FCEC_KIND_TERMINATE, now, 0.0f);
    CHECK(fcec_esc_service(&esc, &ec, now) == 1000);
    fcec_esc_service(&esc, &ec, now + FRAME_US);
    CHECK(ec.applied.main_throttle == 0.0f);

    /* An ESC fault makes the EC lose readiness. */
    fcec_esc_fault(&esc);
    fcec_esc_service(&esc, &ec, now + 2 * FRAME_US);
    CHECK((ec.ready_mask & FCEC_CH_MAIN_ESC) == 0);
}

static void test_unconfigured_esc_never_becomes_ready(void) {
    fcec_esc_config_t c = profile();
    c.approved = false;
    fcec_esc_t esc;
    fcec_ec_init(&ec, BOOT_ID);
    ec.auto_confirm = false;
    ec.capabilities = 0;
    fcec_ec_set_ready(&ec, FCEC_CH_ALL, false);
    fcec_esc_init(&esc, &c, 0);
    fcec_ec_boot_done(&ec);
    CHECK(fcec_esc_service(&esc, &ec, 100000 * MS) == 0);
    CHECK((ec.ready_mask & FCEC_CH_MAIN_ESC) == 0);
}

int main(void) {
    test_profile_must_be_approved_and_consistent();
    test_unconfigured_esc_sends_no_signal();
    test_holds_safe_pulse_until_armed();
    test_throttle_is_capped_slewed_and_cut_at_once();
    test_no_slew_limit_jumps_straight_to_target();
    test_fault_holds_safe_and_rearms_from_scratch();
    test_esc_drives_the_ec_channel();
    test_unconfigured_esc_never_becomes_ready();

    printf("%d checks, %d failed\n", checks, failures);
    return failures == 0 ? 0 : 1;
}
