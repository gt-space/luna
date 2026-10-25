/*
 * Host tests for fc_ec_link. They use the same spec test packets as the FC's
 * Rust tests, so passing both means the two sides agree byte for byte.
 *
 * Run with `make test` in this folder.
 */

#include "fc_ec_link.h"

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

/* Spec test packet: Heartbeat, seq 1. */
static const uint8_t SPEC_HEARTBEAT[52] = {
    0x45, 0x43, 0x04, 0x11, 0x00, 0x00, 0x01, 0x00, 0x44, 0x33, 0x22, 0x11, 0xEF, 0xBE, 0x00,
    0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0x00, 0x00, 0x00, 0x32, 0x00,
    0x00, 0x00, 0x40, 0x42, 0x0F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x84, 0x1E, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x65, 0x34, 0x5B, 0x8C,
};

/* Spec test packet: Demand, seq 2. */
static const uint8_t SPEC_DEMAND[84] = {
    0x45, 0x43, 0x04, 0x10, 0x20, 0x00, 0x01, 0x00, 0x44, 0x33, 0x22, 0x11, 0xEF, 0xBE, 0x00,
    0x00, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0x00, 0x00, 0x00, 0x32, 0x00,
    0x00, 0x00, 0x40, 0x42, 0x0F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x84, 0x1E, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x33, 0x33, 0x57, 0x41, 0x33, 0x33, 0x57, 0x41, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x80, 0x3E, 0x00, 0x00, 0x00, 0x3F, 0x00, 0x00, 0x80, 0x3F, 0x9A, 0x99, 0x19,
    0x3F, 0x7F, 0x00, 0x00, 0x00, 0x69, 0xC3, 0xC6, 0x5D,
};


/* Zeroes the whole packet, padding included, so packets compare with memcmp. */
static void init_packet(fcec_packet_t *p, fcec_kind_t kind) {
    memset(p, 0, sizeof *p);
    p->kind = kind;
}

/* Header used by the spec's test packets. */
static void spec_header(fcec_header_t *h, uint32_t seq) {
    h->duplicate = false;
    h->ec_boot_id = 0x11223344u;
    h->session_id = 0x0000BEEFu;
    h->seq = seq;
    h->txn_id = 0;
    h->bundle_tag = 7;
    h->valid_for_ms = 50;
    h->sent_time_us = 1000000u;
    h->ec_time_ref_us = 2000000u;
}

static void spec_outputs(fcec_outputs_t *o) {
    o->tvc_extension_mm[0] = 13.45f;
    o->tvc_extension_mm[1] = 13.45f;
    o->rcs_duty[0] = 0.0f;
    o->rcs_duty[1] = 0.25f;
    o->rcs_duty[2] = 0.5f;
    o->rcs_duty[3] = 1.0f;
    o->main_throttle = 0.6f;
}

static void spec_demand(fcec_demand_t *d) {
    spec_outputs(&d->outputs);
    d->enable_mask = FCEC_CH_ALL;
}

static void sample_status(fcec_status_t *s) {
    s->ec_state = FCEC_STATE_ARMED;
    s->output_authority = true;
    s->session_valid = true;
    s->safing_latched = false;
    s->demand_fresh = true;
    s->safing_reason = FCEC_REASON_NONE;
    s->last_received_seq = 41;
    s->last_accepted_demand_seq = 40;
    s->last_applied_demand_seq = 39;
    s->last_applied_time_us = 123456789u;
    s->last_reject_reason = FCEC_REASON_STALE;
    s->limited_mask = FCEC_CH_TVC_A2;
    s->counters.crc_errors = 1;
    s->counters.rejected = 2;
    s->counters.stale = 3;
    s->counters.old_or_duplicate = 4;
    s->counters.txn_queue_overflow = 5;
    s->counters.diag_overflow = 6;
    s->counters.deadline_miss = 7;
    s->counters.motion_coalesced = 8;
    s->counters.motion_rejected = 9;
    s->reset_cause = 3;
    s->firmware_version = 0x00010002u;
    s->active_config_id = 0x00010003u;
    s->driver_faults = 0x12u;
    s->watchdog_status = 0x34u;
    spec_outputs(&s->applied);

    fcec_measurements_t *m = &s->measurements;
    m->sample_id = 9;
    m->sample_time_us = 123450000u;
    m->validity_mask = 0x1FFFFu;
    for (unsigned i = 0; i < 17; ++i) m->measurement_sample_time_us[i] = 123450000u;
    for (unsigned i = 0; i < 13; ++i) m->adc_raw_code[i] = 1234u;
    m->tvc_position_mm[0] = 13.4f;
    m->tvc_position_mm[1] = 13.5f;
    m->tvc_position_counts[0] = 2047u;
    m->tvc_position_counts[1] = 2050u;
    m->tvc_position_age_ms[0] = 8;
    m->tvc_position_age_ms[1] = 9;
    m->pa12_fault_flags[0] = 0;
    m->pa12_fault_flags[1] = 0x20;
    m->pa12_command_request_id[0] = 17;
    m->pa12_command_request_id[1] = 22;
    m->pa12_feedback_request_id[0] = 18;
    m->pa12_feedback_request_id[1] = 23;
    m->pa12_command_state[0] = FCEC_PA12_COMMAND_SENT_UNCONFIRMED;
    m->pa12_command_state[1] = FCEC_PA12_COMMAND_SENT_UNCONFIRMED;
    m->pa12_feedback_state[0] = FCEC_PA12_FEEDBACK_VALID_REPLY_WITH_FAULT;
    m->pa12_feedback_state[1] = FCEC_PA12_FEEDBACK_VALID_REPLY_WITH_FAULT;
    m->rail_voltage_v[0] = 22.2f;
    m->rail_voltage_v[1] = 12.0f;
    m->rail_voltage_v[2] = 4.4f;
    m->rail_voltage_v[3] = 3.3f;
    m->rail_current_a[0] = 1.5f;
    m->rail_current_a[1] = 0.8f;
    m->rail_current_a[2] = 2.0f;
    m->rail_current_a[3] = 0.2f;
    m->rcs_current_a[0] = 0.1f;
    m->rcs_current_a[1] = 0.2f;
    m->rcs_current_a[2] = 0.3f;
    m->rcs_current_a[3] = 0.4f;
    m->motor_rail_voltage_v = 4.38f;
    m->board_temp_c[0] = 31.5f;
    m->board_temp_c[1] = 33.0f;
}

/* Recomputes the CRC after a test edits the body. */
static void reseal(uint8_t *bytes, size_t len) {
    size_t body_len = len - FCEC_CRC_LEN;
    uint32_t crc = fcec_crc32(bytes, body_len);
    for (size_t i = 0; i < 4; i++) {
        bytes[body_len + i] = (uint8_t)(crc >> (8 * i));
    }
}

/* Encodes `packet`, checks it equals `bytes`, and checks `bytes` decodes back
 * to `packet`. */
static void check_matches(const fcec_packet_t *packet, const uint8_t *bytes, size_t len) {
    uint8_t buf[FCEC_MAX_DATAGRAM_LEN];
    int written = fcec_encode(packet, buf, sizeof buf);
    CHECK(written == (int)len);
    CHECK(written > 0 && memcmp(buf, bytes, len) == 0);

    fcec_packet_t decoded;
    CHECK(fcec_decode(bytes, len, &decoded) == FCEC_OK);
    CHECK(memcmp(&decoded, packet, sizeof decoded) == 0);
}

static void test_crc_is_zlib_crc32(void) {
    CHECK(fcec_crc32((const uint8_t *)"123456789", 9) == 0xCBF43926u);
}

static void test_heartbeat_matches_spec_bytes(void) {
    fcec_packet_t p;
    init_packet(&p, FCEC_KIND_HEARTBEAT);
    spec_header(&p.header, 1);
    check_matches(&p, SPEC_HEARTBEAT, sizeof SPEC_HEARTBEAT);
}

static void test_demand_matches_spec_bytes(void) {
    fcec_packet_t p;
    init_packet(&p, FCEC_KIND_DEMAND);
    spec_header(&p.header, 2);
    spec_demand(&p.msg.demand);
    check_matches(&p, SPEC_DEMAND, sizeof SPEC_DEMAND);
}

static void test_status_matches_spec_bytes(void) {
    fcec_packet_t p;
    fcec_packet_t decoded;
    uint8_t bytes[FCEC_MAX_DATAGRAM_LEN];
    init_packet(&p, FCEC_KIND_STATUS);
    spec_header(&p.header, 3);
    p.header.valid_for_ms = 0;
    p.header.ec_time_ref_us = 0;
    sample_status(&p.msg.status);
    int len = fcec_encode(&p, bytes, sizeof bytes);
    CHECK(len == (int)(FCEC_HEADER_LEN + 388u + FCEC_CRC_LEN));
    CHECK(fcec_decode(bytes, (size_t)len, &decoded) == FCEC_OK);
    CHECK(memcmp(&decoded, &p, sizeof p) == 0);
}

static void test_every_kind_round_trips_at_spec_length(void) {
    static const struct {
        fcec_kind_t kind;
        int payload_len;
    } kinds[] = {
        {FCEC_KIND_HELLO, 48},      {FCEC_KIND_ARM, 36},
        {FCEC_KIND_DISARM, 0},      {FCEC_KIND_TERMINATE, 4},
        {FCEC_KIND_CLEAR_SAFING, 0}, {FCEC_KIND_DEMAND, 32},
        {FCEC_KIND_HEARTBEAT, 0},   {FCEC_KIND_TXN_RESULT, 8},
        {FCEC_KIND_HELLO_RESULT, 52}, {FCEC_KIND_STATUS, 388},
    };

    for (size_t i = 0; i < sizeof kinds / sizeof kinds[0]; i++) {
        fcec_packet_t p;
        init_packet(&p, kinds[i].kind);
        spec_header(&p.header, 77);
        p.header.duplicate = true;

        switch (kinds[i].kind) {
        case FCEC_KIND_HELLO:
            p.msg.hello.fc_boot_id = 5;
            p.msg.hello.run_id = 6;
            p.msg.hello.demand_period_ms = 10;
            p.msg.hello.heartbeat_period_ms = 50;
            memset(p.msg.hello.expected_bundle_sha256, 0xAB, 32);
            break;
        case FCEC_KIND_ARM:
            p.msg.arm.run_id = 6;
            memset(p.msg.arm.expected_bundle_sha256, 0xAB, 32);
            break;
        case FCEC_KIND_TERMINATE:
            p.msg.terminate.reason = FCEC_REASON_FC_TERMINATE;
            break;
        case FCEC_KIND_DEMAND:
            spec_demand(&p.msg.demand);
            break;
        case FCEC_KIND_TXN_RESULT:
            p.msg.txn_result.request_kind = FCEC_KIND_ARM;
            p.msg.txn_result.outcome = FCEC_OUTCOME_REJECTED;
            p.msg.txn_result.reason = FCEC_REASON_NOT_READY;
            p.msg.txn_result.ec_state = FCEC_STATE_DISARMED;
            break;
        case FCEC_KIND_HELLO_RESULT:
            p.msg.hello_result.outcome = FCEC_OUTCOME_ACCEPTED;
            p.msg.hello_result.ec_state = FCEC_STATE_DISARMED;
            p.msg.hello_result.reason = FCEC_REASON_NONE;
            p.msg.hello_result.capabilities = FCEC_CH_ALL;
            p.msg.hello_result.max_demand_age_ms = 50;
            p.msg.hello_result.firmware_version = 3;
            p.msg.hello_result.calibration_id = 4;
            p.msg.hello_result.link_timeout_ms = 350;
            memset(p.msg.hello_result.active_bundle_sha256, 0xAB, 32);
            break;
        case FCEC_KIND_STATUS:
            sample_status(&p.msg.status);
            break;
        case FCEC_KIND_DISARM:
        case FCEC_KIND_CLEAR_SAFING:
        case FCEC_KIND_HEARTBEAT:
            break;
        }

        uint8_t buf[FCEC_MAX_DATAGRAM_LEN];
        int len = fcec_encode(&p, buf, sizeof buf);
        CHECK(fcec_payload_len(kinds[i].kind) == kinds[i].payload_len);
        CHECK(len == (int)FCEC_HEADER_LEN + kinds[i].payload_len + (int)FCEC_CRC_LEN);
        CHECK(buf[3] == (uint8_t)kinds[i].kind);
        CHECK(buf[6] == ((uint8_t)kinds[i].kind < 0x80u ? FCEC_SOURCE_FC : FCEC_SOURCE_EC));

        fcec_packet_t decoded;
        CHECK(len > 0 && fcec_decode(buf, (size_t)len, &decoded) == FCEC_OK);
        CHECK(memcmp(&decoded, &p, sizeof p) == 0);
    }
}

static void test_status_flags_land_on_their_bits(void) {
    fcec_packet_t p;
    init_packet(&p, FCEC_KIND_STATUS);
    p.msg.status.session_valid = true;
    p.msg.status.demand_fresh = true;

    uint8_t buf[FCEC_MAX_DATAGRAM_LEN];
    CHECK(fcec_encode(&p, buf, sizeof buf) > 0);
    CHECK(buf[FCEC_HEADER_LEN + 1] == 0x0A);
}

static void test_rejects_malformed_datagrams(void) {
    fcec_packet_t out;
    uint8_t bytes[FCEC_MAX_DATAGRAM_LEN + 1];

    memcpy(bytes, SPEC_DEMAND, sizeof SPEC_DEMAND);
    bytes[60] ^= 0x01;
    CHECK(fcec_decode(bytes, sizeof SPEC_DEMAND, &out) == FCEC_ERR_BAD_CRC);

    memcpy(bytes, SPEC_HEARTBEAT, sizeof SPEC_HEARTBEAT);
    bytes[0] = 'X';
    CHECK(fcec_decode(bytes, sizeof SPEC_HEARTBEAT, &out) == FCEC_ERR_BAD_MAGIC);

    memcpy(bytes, SPEC_HEARTBEAT, sizeof SPEC_HEARTBEAT);
    bytes[2] = 2;
    CHECK(fcec_decode(bytes, sizeof SPEC_HEARTBEAT, &out) == FCEC_ERR_UNSUPPORTED_VERSION);

    CHECK(fcec_decode(SPEC_HEARTBEAT, 40, &out) == FCEC_ERR_TOO_SHORT);

    memset(bytes, 0, sizeof bytes);
    CHECK(fcec_decode(bytes, sizeof bytes, &out) == FCEC_ERR_TOO_LONG);
}

static void test_rejects_unknown_kind_and_wrong_length(void) {
    fcec_packet_t out;
    uint8_t bytes[FCEC_MAX_DATAGRAM_LEN];

    memcpy(bytes, SPEC_HEARTBEAT, sizeof SPEC_HEARTBEAT);
    bytes[3] = 0x42;
    reseal(bytes, sizeof SPEC_HEARTBEAT);
    CHECK(fcec_decode(bytes, sizeof SPEC_HEARTBEAT, &out) == FCEC_ERR_INVALID_ENUM);

    /* A heartbeat that claims to carry a demand-sized payload. */
    memcpy(bytes, SPEC_HEARTBEAT, sizeof SPEC_HEARTBEAT);
    bytes[4] = 32;
    reseal(bytes, sizeof SPEC_HEARTBEAT);
    CHECK(fcec_decode(bytes, sizeof SPEC_HEARTBEAT, &out) == FCEC_ERR_BAD_LENGTH);

    /* A demand cut short: the declared length is right, the datagram isn't. */
    memcpy(bytes, SPEC_DEMAND, 76);
    memset(&bytes[76], 0, FCEC_CRC_LEN);
    reseal(bytes, 80);
    CHECK(fcec_decode(bytes, 80, &out) == FCEC_ERR_BAD_LENGTH);
}

static void test_rejects_wrong_source_and_bad_enums(void) {
    fcec_packet_t out;
    fcec_packet_t status;
    uint8_t bytes[FCEC_MAX_DATAGRAM_LEN];

    memcpy(bytes, SPEC_DEMAND, sizeof SPEC_DEMAND);
    bytes[6] = FCEC_SOURCE_EC;
    reseal(bytes, sizeof SPEC_DEMAND);
    CHECK(fcec_decode(bytes, sizeof SPEC_DEMAND, &out) == FCEC_ERR_WRONG_SOURCE);

    init_packet(&status, FCEC_KIND_STATUS);
    spec_header(&status.header, 3);
    sample_status(&status.msg.status);
    int status_len = fcec_encode(&status, bytes, sizeof bytes);
    bytes[FCEC_HEADER_LEN] = 9; /* ec_state */
    reseal(bytes, (size_t)status_len);
    CHECK(fcec_decode(bytes, (size_t)status_len, &out) == FCEC_ERR_INVALID_ENUM);

    status_len = fcec_encode(&status, bytes, sizeof bytes);
    bytes[FCEC_HEADER_LEN + 24] = 18; /* last_reject_reason, one past the last */
    reseal(bytes, (size_t)status_len);
    CHECK(fcec_decode(bytes, (size_t)status_len, &out) == FCEC_ERR_INVALID_ENUM);
}

static void test_sequence_comparison_wraps(void) {
    CHECK(fcec_is_newer(1, 0));
    CHECK(!fcec_is_newer(0, 1));
    CHECK(!fcec_is_newer(5, 5));
    CHECK(fcec_is_newer(0, 0xFFFFFFFFu));
    CHECK(fcec_is_newer(3, 0xFFFFFFFFu - 3));
    CHECK(!fcec_is_newer(0xFFFFFFFFu, 0));
    CHECK(!fcec_is_newer(0x80000000u, 0)); /* exactly half way is not newer */
}

static void test_demand_check_matches_the_spec(void) {
    fcec_demand_t d;

    spec_demand(&d);
    CHECK(fcec_check_demand(&d, FCEC_CH_ALL) == FCEC_REASON_NONE);

    spec_demand(&d);
    d.outputs.rcs_duty[2] = NAN;
    CHECK(fcec_check_demand(&d, FCEC_CH_ALL) == FCEC_REASON_NON_FINITE);

    spec_demand(&d);
    d.outputs.tvc_extension_mm[0] = INFINITY;
    CHECK(fcec_check_demand(&d, FCEC_CH_ALL) == FCEC_REASON_NON_FINITE);

    spec_demand(&d);
    d.outputs.tvc_extension_mm[1] = FCEC_TVC_STROKE_MM + 0.1f;
    CHECK(fcec_check_demand(&d, FCEC_CH_ALL) == FCEC_REASON_OUT_OF_RANGE);

    spec_demand(&d);
    d.outputs.rcs_duty[0] = -0.01f;
    CHECK(fcec_check_demand(&d, FCEC_CH_ALL) == FCEC_REASON_OUT_OF_RANGE);

    spec_demand(&d);
    d.outputs.main_throttle = 1.01f;
    CHECK(fcec_check_demand(&d, FCEC_CH_ALL) == FCEC_REASON_OUT_OF_RANGE);

    spec_demand(&d);
    CHECK(fcec_check_demand(&d, (uint16_t)(FCEC_CH_ALL & ~FCEC_CH_MAIN_ESC)) ==
          FCEC_REASON_UNSUPPORTED_CHANNEL);
}

static void test_encode_reports_bad_input(void) {
    fcec_packet_t p;
    uint8_t buf[100];

    init_packet(&p, FCEC_KIND_STATUS);
    CHECK(fcec_encode(&p, buf, sizeof buf) == FCEC_ERR_BUFFER_TOO_SMALL);

    init_packet(&p, (fcec_kind_t)0x42);
    CHECK(fcec_encode(&p, buf, sizeof buf) == FCEC_ERR_INVALID_ENUM);
}

int main(void) {
    test_crc_is_zlib_crc32();
    test_heartbeat_matches_spec_bytes();
    test_demand_matches_spec_bytes();
    test_status_matches_spec_bytes();
    test_every_kind_round_trips_at_spec_length();
    test_status_flags_land_on_their_bits();
    test_rejects_malformed_datagrams();
    test_rejects_unknown_kind_and_wrong_length();
    test_rejects_wrong_source_and_bad_enums();
    test_sequence_comparison_wraps();
    test_demand_check_matches_the_spec();
    test_encode_reports_bad_input();

    printf("%d checks, %d failed\n", checks, failures);
    return failures == 0 ? 0 : 1;
}
