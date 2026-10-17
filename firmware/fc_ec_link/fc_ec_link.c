#include "fc_ec_link.h"

#include <math.h>
#include <string.h>

#define HELLO_LEN 48
#define ARM_LEN 36
#define TERMINATE_LEN 4
#define DEMAND_LEN 32
#define TXN_RESULT_LEN 8
#define HELLO_RESULT_LEN 52
#define STATUS_LEN 168

#define STATUS_OUTPUT_AUTHORITY 0x01u
#define STATUS_SESSION_VALID 0x02u
#define STATUS_SAFING_LATCHED 0x04u
#define STATUS_DEMAND_FRESH 0x08u

/* ---- Byte writer / reader. Callers size and check lengths up front, so
 * individual reads and writes don't bounds-check. ---- */

typedef struct {
    uint8_t *buf;
    size_t pos;
} writer_t;

typedef struct {
    const uint8_t *buf;
    size_t pos;
} reader_t;

static void put_u8(writer_t *w, uint8_t value) {
    w->buf[w->pos++] = value;
}

static void put_u16(writer_t *w, uint16_t value) {
    put_u8(w, (uint8_t)(value & 0xFFu));
    put_u8(w, (uint8_t)(value >> 8));
}

static void put_u32(writer_t *w, uint32_t value) {
    put_u16(w, (uint16_t)(value & 0xFFFFu));
    put_u16(w, (uint16_t)(value >> 16));
}

static void put_u64(writer_t *w, uint64_t value) {
    put_u32(w, (uint32_t)(value & 0xFFFFFFFFu));
    put_u32(w, (uint32_t)(value >> 32));
}

static void put_f32(writer_t *w, float value) {
    uint32_t bits;
    memcpy(&bits, &value, sizeof bits);
    put_u32(w, bits);
}

static void put_bytes(writer_t *w, const uint8_t *bytes, size_t len) {
    memcpy(&w->buf[w->pos], bytes, len);
    w->pos += len;
}

static void put_zeros(writer_t *w, size_t len) {
    memset(&w->buf[w->pos], 0, len);
    w->pos += len;
}

static void put_f32s(writer_t *w, const float *values, size_t count) {
    for (size_t i = 0; i < count; i++) {
        put_f32(w, values[i]);
    }
}

static void put_u16s(writer_t *w, const uint16_t *values, size_t count) {
    for (size_t i = 0; i < count; i++) {
        put_u16(w, values[i]);
    }
}

static uint8_t get_u8(reader_t *r) {
    return r->buf[r->pos++];
}

static uint16_t get_u16(reader_t *r) {
    uint16_t low = get_u8(r);
    uint16_t high = get_u8(r);
    return (uint16_t)(low | (high << 8));
}

static uint32_t get_u32(reader_t *r) {
    uint32_t low = get_u16(r);
    uint32_t high = get_u16(r);
    return low | (high << 16);
}

static uint64_t get_u64(reader_t *r) {
    uint64_t low = get_u32(r);
    uint64_t high = get_u32(r);
    return low | (high << 32);
}

static float get_f32(reader_t *r) {
    uint32_t bits = get_u32(r);
    float value;
    memcpy(&value, &bits, sizeof value);
    return value;
}

static void get_bytes(reader_t *r, uint8_t *out, size_t len) {
    memcpy(out, &r->buf[r->pos], len);
    r->pos += len;
}

static void skip(reader_t *r, size_t len) {
    r->pos += len;
}

static void get_f32s(reader_t *r, float *out, size_t count) {
    for (size_t i = 0; i < count; i++) {
        out[i] = get_f32(r);
    }
}

static void get_u16s(reader_t *r, uint16_t *out, size_t count) {
    for (size_t i = 0; i < count; i++) {
        out[i] = get_u16(r);
    }
}

/* ---- Enum fields: reject values with no meaning. ---- */

static bool get_ec_state(reader_t *r, fcec_ec_state_t *out) {
    uint8_t value = get_u8(r);
    *out = (fcec_ec_state_t)value;
    return value <= FCEC_STATE_SAFING;
}

static bool get_outcome(reader_t *r, fcec_outcome_t *out) {
    uint8_t value = get_u8(r);
    *out = (fcec_outcome_t)value;
    return value <= FCEC_OUTCOME_REJECTED;
}

static bool get_reason(reader_t *r, fcec_reason_t *out) {
    uint16_t value = get_u16(r);
    *out = (fcec_reason_t)value;
    return value <= FCEC_REASON_FC_TERMINATE;
}

static bool get_kind(reader_t *r, fcec_kind_t *out) {
    uint8_t value = get_u8(r);
    *out = (fcec_kind_t)value;
    return fcec_payload_len(*out) >= 0;
}

/* ---- Payloads. Field order and widths are the spec's byte table. ---- */

static void put_outputs(writer_t *w, const fcec_outputs_t *o) {
    put_f32s(w, o->tvc_extension_mm, 2);
    put_f32s(w, o->rcs_duty, 4);
    put_f32(w, o->main_throttle);
}

static void get_outputs(reader_t *r, fcec_outputs_t *o) {
    get_f32s(r, o->tvc_extension_mm, 2);
    get_f32s(r, o->rcs_duty, 4);
    o->main_throttle = get_f32(r);
}

static void put_counters(writer_t *w, const fcec_counters_t *c) {
    put_u16(w, c->crc_errors);
    put_u16(w, c->rejected);
    put_u16(w, c->stale);
    put_u16(w, c->old_or_duplicate);
    put_u16(w, c->txn_queue_overflow);
    put_u16(w, c->diag_overflow);
    put_u16(w, c->deadline_miss);
}

static void get_counters(reader_t *r, fcec_counters_t *c) {
    c->crc_errors = get_u16(r);
    c->rejected = get_u16(r);
    c->stale = get_u16(r);
    c->old_or_duplicate = get_u16(r);
    c->txn_queue_overflow = get_u16(r);
    c->diag_overflow = get_u16(r);
    c->deadline_miss = get_u16(r);
}

static void put_measurements(writer_t *w, const fcec_measurements_t *m) {
    put_u32(w, m->sample_id);
    put_u64(w, m->sample_time_us);
    put_u32(w, m->validity_mask);
    put_f32s(w, m->tvc_position_mm, 2);
    put_u16s(w, m->tvc_position_age_ms, 2);
    put_u16s(w, m->pa12_fault_flags, 2);
    put_f32s(w, m->rail_voltage_v, 4);
    put_f32s(w, m->rail_current_a, 4);
    put_f32s(w, m->rcs_current_a, 4);
    put_f32(w, m->motor_rail_voltage_v);
    put_f32s(w, m->board_temp_c, 2);
}

static void get_measurements(reader_t *r, fcec_measurements_t *m) {
    m->sample_id = get_u32(r);
    m->sample_time_us = get_u64(r);
    m->validity_mask = get_u32(r);
    get_f32s(r, m->tvc_position_mm, 2);
    get_u16s(r, m->tvc_position_age_ms, 2);
    get_u16s(r, m->pa12_fault_flags, 2);
    get_f32s(r, m->rail_voltage_v, 4);
    get_f32s(r, m->rail_current_a, 4);
    get_f32s(r, m->rcs_current_a, 4);
    m->motor_rail_voltage_v = get_f32(r);
    get_f32s(r, m->board_temp_c, 2);
}

static void put_status(writer_t *w, const fcec_status_t *s) {
    uint8_t flags = (uint8_t)((s->output_authority ? STATUS_OUTPUT_AUTHORITY : 0u) |
                              (s->session_valid ? STATUS_SESSION_VALID : 0u) |
                              (s->safing_latched ? STATUS_SAFING_LATCHED : 0u) |
                              (s->demand_fresh ? STATUS_DEMAND_FRESH : 0u));

    put_u8(w, (uint8_t)s->ec_state);
    put_u8(w, flags);
    put_u16(w, (uint16_t)s->safing_reason);
    put_u32(w, s->last_received_seq);
    put_u32(w, s->last_accepted_demand_seq);
    put_u32(w, s->last_applied_demand_seq);
    put_u64(w, s->last_applied_time_us);
    put_u16(w, (uint16_t)s->last_reject_reason);
    put_u16(w, s->limited_mask);
    put_counters(w, &s->counters);
    put_u8(w, s->reset_cause);
    put_zeros(w, 1);
    put_u32(w, s->firmware_version);
    put_outputs(w, &s->applied);
    put_measurements(w, &s->measurements);
}

static bool get_status(reader_t *r, fcec_status_t *s) {
    bool ok = get_ec_state(r, &s->ec_state);
    uint8_t flags = get_u8(r);
    s->output_authority = (flags & STATUS_OUTPUT_AUTHORITY) != 0;
    s->session_valid = (flags & STATUS_SESSION_VALID) != 0;
    s->safing_latched = (flags & STATUS_SAFING_LATCHED) != 0;
    s->demand_fresh = (flags & STATUS_DEMAND_FRESH) != 0;
    ok = get_reason(r, &s->safing_reason) && ok;
    s->last_received_seq = get_u32(r);
    s->last_accepted_demand_seq = get_u32(r);
    s->last_applied_demand_seq = get_u32(r);
    s->last_applied_time_us = get_u64(r);
    ok = get_reason(r, &s->last_reject_reason) && ok;
    s->limited_mask = get_u16(r);
    get_counters(r, &s->counters);
    s->reset_cause = get_u8(r);
    skip(r, 1);
    s->firmware_version = get_u32(r);
    get_outputs(r, &s->applied);
    get_measurements(r, &s->measurements);
    return ok;
}

static void put_payload(writer_t *w, const fcec_packet_t *p) {
    switch (p->kind) {
    case FCEC_KIND_HELLO:
        put_u32(w, p->msg.hello.fc_boot_id);
        put_u32(w, p->msg.hello.run_id);
        put_u16(w, p->msg.hello.demand_period_ms);
        put_u16(w, p->msg.hello.heartbeat_period_ms);
        put_zeros(w, 4);
        put_bytes(w, p->msg.hello.expected_bundle_sha256, 32);
        break;
    case FCEC_KIND_ARM:
        put_u32(w, p->msg.arm.run_id);
        put_bytes(w, p->msg.arm.expected_bundle_sha256, 32);
        break;
    case FCEC_KIND_TERMINATE:
        put_u16(w, (uint16_t)p->msg.terminate.reason);
        put_zeros(w, 2);
        break;
    case FCEC_KIND_DEMAND:
        put_outputs(w, &p->msg.demand.outputs);
        put_u16(w, p->msg.demand.enable_mask);
        put_zeros(w, 2);
        break;
    case FCEC_KIND_TXN_RESULT:
        put_u8(w, (uint8_t)p->msg.txn_result.request_kind);
        put_u8(w, (uint8_t)p->msg.txn_result.outcome);
        put_u16(w, (uint16_t)p->msg.txn_result.reason);
        put_u8(w, (uint8_t)p->msg.txn_result.ec_state);
        put_zeros(w, 3);
        break;
    case FCEC_KIND_HELLO_RESULT:
        put_u8(w, (uint8_t)p->msg.hello_result.outcome);
        put_u8(w, (uint8_t)p->msg.hello_result.ec_state);
        put_u16(w, (uint16_t)p->msg.hello_result.reason);
        put_u16(w, p->msg.hello_result.capabilities);
        put_u16(w, p->msg.hello_result.max_demand_age_ms);
        put_u32(w, p->msg.hello_result.firmware_version);
        put_u32(w, p->msg.hello_result.calibration_id);
        put_u16(w, p->msg.hello_result.link_timeout_ms);
        put_zeros(w, 2);
        put_bytes(w, p->msg.hello_result.active_bundle_sha256, 32);
        break;
    case FCEC_KIND_STATUS:
        put_status(w, &p->msg.status);
        break;
    case FCEC_KIND_DISARM:
    case FCEC_KIND_CLEAR_SAFING:
    case FCEC_KIND_HEARTBEAT:
        break;
    }
}

static bool get_payload(reader_t *r, fcec_packet_t *p) {
    bool ok = true;

    switch (p->kind) {
    case FCEC_KIND_HELLO:
        p->msg.hello.fc_boot_id = get_u32(r);
        p->msg.hello.run_id = get_u32(r);
        p->msg.hello.demand_period_ms = get_u16(r);
        p->msg.hello.heartbeat_period_ms = get_u16(r);
        skip(r, 4);
        get_bytes(r, p->msg.hello.expected_bundle_sha256, 32);
        break;
    case FCEC_KIND_ARM:
        p->msg.arm.run_id = get_u32(r);
        get_bytes(r, p->msg.arm.expected_bundle_sha256, 32);
        break;
    case FCEC_KIND_TERMINATE:
        ok = get_reason(r, &p->msg.terminate.reason);
        skip(r, 2);
        break;
    case FCEC_KIND_DEMAND:
        get_outputs(r, &p->msg.demand.outputs);
        p->msg.demand.enable_mask = get_u16(r);
        skip(r, 2);
        break;
    case FCEC_KIND_TXN_RESULT:
        ok = get_kind(r, &p->msg.txn_result.request_kind);
        ok = get_outcome(r, &p->msg.txn_result.outcome) && ok;
        ok = get_reason(r, &p->msg.txn_result.reason) && ok;
        ok = get_ec_state(r, &p->msg.txn_result.ec_state) && ok;
        skip(r, 3);
        break;
    case FCEC_KIND_HELLO_RESULT:
        ok = get_outcome(r, &p->msg.hello_result.outcome);
        ok = get_ec_state(r, &p->msg.hello_result.ec_state) && ok;
        ok = get_reason(r, &p->msg.hello_result.reason) && ok;
        p->msg.hello_result.capabilities = get_u16(r);
        p->msg.hello_result.max_demand_age_ms = get_u16(r);
        p->msg.hello_result.firmware_version = get_u32(r);
        p->msg.hello_result.calibration_id = get_u32(r);
        p->msg.hello_result.link_timeout_ms = get_u16(r);
        skip(r, 2);
        get_bytes(r, p->msg.hello_result.active_bundle_sha256, 32);
        break;
    case FCEC_KIND_STATUS:
        ok = get_status(r, &p->msg.status);
        break;
    case FCEC_KIND_DISARM:
    case FCEC_KIND_CLEAR_SAFING:
    case FCEC_KIND_HEARTBEAT:
        break;
    }

    return ok;
}

static uint8_t source_of(fcec_kind_t kind) {
    return (uint8_t)((uint8_t)kind < 0x80u ? FCEC_SOURCE_FC : FCEC_SOURCE_EC);
}

/* ---- Public API ---- */

int fcec_payload_len(fcec_kind_t kind) {
    switch (kind) {
    case FCEC_KIND_HELLO:
        return HELLO_LEN;
    case FCEC_KIND_ARM:
        return ARM_LEN;
    case FCEC_KIND_DISARM:
    case FCEC_KIND_CLEAR_SAFING:
    case FCEC_KIND_HEARTBEAT:
        return 0;
    case FCEC_KIND_TERMINATE:
        return TERMINATE_LEN;
    case FCEC_KIND_DEMAND:
        return DEMAND_LEN;
    case FCEC_KIND_TXN_RESULT:
        return TXN_RESULT_LEN;
    case FCEC_KIND_HELLO_RESULT:
        return HELLO_RESULT_LEN;
    case FCEC_KIND_STATUS:
        return STATUS_LEN;
    }
    return -1;
}

int fcec_encode(const fcec_packet_t *packet, uint8_t *buf, size_t cap) {
    int payload_len = fcec_payload_len(packet->kind);
    if (payload_len < 0) {
        return FCEC_ERR_INVALID_ENUM;
    }

    size_t body_len = FCEC_HEADER_LEN + (size_t)payload_len;
    size_t len = body_len + FCEC_CRC_LEN;
    if (cap < len) {
        return FCEC_ERR_BUFFER_TOO_SMALL;
    }

    const fcec_header_t *h = &packet->header;
    writer_t w = {buf, 0};

    put_u8(&w, 'E');
    put_u8(&w, 'C');
    put_u8(&w, FCEC_VERSION);
    put_u8(&w, (uint8_t)packet->kind);
    put_u16(&w, (uint16_t)payload_len);
    put_u8(&w, source_of(packet->kind));
    put_u8(&w, (uint8_t)(h->duplicate ? 1u : 0u));
    put_u32(&w, h->ec_boot_id);
    put_u32(&w, h->session_id);
    put_u32(&w, h->seq);
    put_u32(&w, h->txn_id);
    put_u32(&w, h->bundle_tag);
    put_u16(&w, h->valid_for_ms);
    put_zeros(&w, 2);
    put_u64(&w, h->sent_time_us);
    put_u64(&w, h->ec_time_ref_us);
    put_payload(&w, packet);

    if (w.pos != body_len) {
        return FCEC_ERR_INTERNAL;
    }

    put_u32(&w, fcec_crc32(buf, body_len));
    return (int)len;
}

fcec_result_t fcec_decode(const uint8_t *buf, size_t len, fcec_packet_t *out) {
    memset(out, 0, sizeof *out);

    if (len < FCEC_HEADER_LEN + FCEC_CRC_LEN) {
        return FCEC_ERR_TOO_SHORT;
    }
    if (len > FCEC_MAX_DATAGRAM_LEN) {
        return FCEC_ERR_TOO_LONG;
    }
    if (buf[0] != 'E' || buf[1] != 'C') {
        return FCEC_ERR_BAD_MAGIC;
    }
    if (buf[2] != FCEC_VERSION) {
        return FCEC_ERR_UNSUPPORTED_VERSION;
    }

    size_t body_len = len - FCEC_CRC_LEN;
    reader_t crc_reader = {buf, body_len};
    if (get_u32(&crc_reader) != fcec_crc32(buf, body_len)) {
        return FCEC_ERR_BAD_CRC;
    }

    reader_t r = {buf, 3};
    if (!get_kind(&r, &out->kind)) {
        return FCEC_ERR_INVALID_ENUM;
    }

    size_t payload_len = (size_t)fcec_payload_len(out->kind);
    uint16_t declared = get_u16(&r);
    if (declared != payload_len || body_len - FCEC_HEADER_LEN != payload_len) {
        return FCEC_ERR_BAD_LENGTH;
    }

    if (get_u8(&r) != source_of(out->kind)) {
        return FCEC_ERR_WRONG_SOURCE;
    }

    fcec_header_t *h = &out->header;
    h->duplicate = (get_u8(&r) & 1u) != 0;
    h->ec_boot_id = get_u32(&r);
    h->session_id = get_u32(&r);
    h->seq = get_u32(&r);
    h->txn_id = get_u32(&r);
    h->bundle_tag = get_u32(&r);
    h->valid_for_ms = get_u16(&r);
    skip(&r, 2);
    h->sent_time_us = get_u64(&r);
    h->ec_time_ref_us = get_u64(&r);

    return get_payload(&r, out) ? FCEC_OK : FCEC_ERR_INVALID_ENUM;
}

fcec_reason_t fcec_check_outputs(const fcec_outputs_t *o) {
    const float tvc[2] = {o->tvc_extension_mm[0], o->tvc_extension_mm[1]};
    const float fractions[5] = {
        o->rcs_duty[0], o->rcs_duty[1], o->rcs_duty[2], o->rcs_duty[3], o->main_throttle,
    };

    for (size_t i = 0; i < 2; i++) {
        if (!isfinite(tvc[i])) {
            return FCEC_REASON_NON_FINITE;
        }
    }
    for (size_t i = 0; i < 5; i++) {
        if (!isfinite(fractions[i])) {
            return FCEC_REASON_NON_FINITE;
        }
    }

    for (size_t i = 0; i < 2; i++) {
        if (tvc[i] < 0.0f || tvc[i] > FCEC_TVC_STROKE_MM) {
            return FCEC_REASON_OUT_OF_RANGE;
        }
    }
    for (size_t i = 0; i < 5; i++) {
        if (fractions[i] < 0.0f || fractions[i] > 1.0f) {
            return FCEC_REASON_OUT_OF_RANGE;
        }
    }

    return FCEC_REASON_NONE;
}

fcec_reason_t fcec_check_demand(const fcec_demand_t *demand, uint16_t capabilities) {
    fcec_reason_t reason = fcec_check_outputs(&demand->outputs);
    if (reason != FCEC_REASON_NONE) {
        return reason;
    }

    if ((demand->enable_mask & ~capabilities) != 0) {
        return FCEC_REASON_UNSUPPORTED_CHANNEL;
    }

    return FCEC_REASON_NONE;
}

bool fcec_is_newer(uint32_t a, uint32_t b) {
    /* a is newer when (a - b) mod 2^32 is in 1 .. 2^31 - 1. */
    return (uint32_t)(a - b - 1u) < 0x7FFFFFFFu;
}

#ifndef FCEC_EXTERNAL_CRC32
uint32_t fcec_crc32(const uint8_t *data, size_t len) {
    uint32_t crc = 0xFFFFFFFFu;

    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int bit = 0; bit < 8; bit++) {
            crc = (crc >> 1) ^ (0xEDB88320u & (0u - (crc & 1u)));
        }
    }

    return ~crc;
}
#endif

const char *fcec_result_str(fcec_result_t result) {
    switch (result) {
    case FCEC_OK:
        return "ok";
    case FCEC_ERR_TOO_SHORT:
        return "too short";
    case FCEC_ERR_TOO_LONG:
        return "too long";
    case FCEC_ERR_BAD_MAGIC:
        return "bad magic";
    case FCEC_ERR_UNSUPPORTED_VERSION:
        return "unsupported version";
    case FCEC_ERR_BAD_CRC:
        return "bad CRC";
    case FCEC_ERR_INVALID_ENUM:
        return "invalid enum value";
    case FCEC_ERR_BAD_LENGTH:
        return "bad length";
    case FCEC_ERR_WRONG_SOURCE:
        return "wrong source";
    case FCEC_ERR_BUFFER_TOO_SMALL:
        return "buffer too small";
    case FCEC_ERR_INTERNAL:
        return "internal length mismatch";
    }
    return "unknown";
}

const char *fcec_reason_str(fcec_reason_t reason) {
    switch (reason) {
    case FCEC_REASON_NONE:
        return "none";
    case FCEC_REASON_BAD_SESSION:
        return "bad session";
    case FCEC_REASON_BAD_BOOT_ID:
        return "bad boot ID";
    case FCEC_REASON_WRONG_STATE:
        return "wrong state";
    case FCEC_REASON_CONFIG_MISMATCH:
        return "config mismatch";
    case FCEC_REASON_NOT_READY:
        return "not ready";
    case FCEC_REASON_SAFING_LATCHED:
        return "safing latched";
    case FCEC_REASON_UNSUPPORTED_VERSION:
        return "unsupported version";
    case FCEC_REASON_UNSUPPORTED_CHANNEL:
        return "unsupported channel";
    case FCEC_REASON_BAD_LENGTH:
        return "bad length";
    case FCEC_REASON_OUT_OF_RANGE:
        return "out of range";
    case FCEC_REASON_NON_FINITE:
        return "non-finite";
    case FCEC_REASON_STALE:
        return "stale";
    case FCEC_REASON_BUSY:
        return "busy";
    case FCEC_REASON_DEMAND_TIMEOUT:
        return "demand timeout";
    case FCEC_REASON_LINK_TIMEOUT:
        return "link timeout";
    case FCEC_REASON_LOCAL_FAULT:
        return "local fault";
    case FCEC_REASON_FC_TERMINATE:
        return "FC terminate";
    }
    return "unknown";
}

const char *fcec_state_str(fcec_ec_state_t state) {
    switch (state) {
    case FCEC_STATE_BOOT_SAFE:
        return "BOOT_SAFE";
    case FCEC_STATE_DISARMED:
        return "DISARMED";
    case FCEC_STATE_ARMED:
        return "ARMED";
    case FCEC_STATE_SAFING:
        return "SAFING";
    }
    return "unknown";
}
