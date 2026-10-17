/*
 * Wire format for the CTV flight computer (FC) <-> engine controller (EC)
 * link, EC side.
 *
 * Every datagram is a 48-byte header, a fixed-size payload chosen by the
 * message kind, and a CRC-32 over both. All fields are little-endian with no
 * padding and are read and written one at a time, never by copying structs,
 * so this matches the FC's Rust code (common::comm::ctv::ec_link) byte for
 * byte. Change both together.
 *
 * Plain C99 with no HAL or lwIP dependency: give fcec_decode() the bytes of a
 * UDP payload and it fills in a struct; give fcec_encode() a struct and it
 * fills in the bytes to send.
 *
 * The CRC is computed in software. To use the STM32 CRC unit instead, define
 * FCEC_EXTERNAL_CRC32 and provide fcec_crc32() yourself, with the CRC unit set
 * to the default polynomial and init value, input inversion BYTE, output
 * inversion ENABLE, and the result XORed with 0xFFFFFFFF.
 */

#ifndef FC_EC_LINK_H
#define FC_EC_LINK_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* UDP port the EC listens on. The EC replies to the address FC packets come
 * from, on FCEC_FC_PORT. */
#define FCEC_EC_PORT 8378u
#define FCEC_FC_PORT 4574u

/* EC static IP, 192.168.1.50 (placeholder until the EC firmware has one). */
#define FCEC_EC_IP_0 192u
#define FCEC_EC_IP_1 168u
#define FCEC_EC_IP_2 1u
#define FCEC_EC_IP_3 50u

#define FCEC_VERSION 1u
#define FCEC_HEADER_LEN 48u
#define FCEC_CRC_LEN 4u
#define FCEC_MAX_DATAGRAM_LEN 512u

/* Full stroke of the 1.06" PA-12-T, in mm. */
#define FCEC_TVC_STROKE_MM 26.9f

/* Bits of the channel masks used in demands, status and capabilities. */
#define FCEC_CH_TVC_A1 (1u << 0)
#define FCEC_CH_TVC_A2 (1u << 1)
#define FCEC_CH_RCS_M1 (1u << 2)
#define FCEC_CH_RCS_M2 (1u << 3)
#define FCEC_CH_RCS_M3 (1u << 4)
#define FCEC_CH_RCS_M4 (1u << 5)
#define FCEC_CH_MAIN_ESC (1u << 6)
#define FCEC_CH_ALL 0x7Fu

/* Message kind. Kinds below 0x80 go FC -> EC, the rest EC -> FC. */
typedef enum {
    FCEC_KIND_HELLO = 0x01,
    FCEC_KIND_ARM = 0x02,
    FCEC_KIND_DISARM = 0x03,
    FCEC_KIND_TERMINATE = 0x04,
    FCEC_KIND_CLEAR_SAFING = 0x05,
    FCEC_KIND_DEMAND = 0x10,
    FCEC_KIND_HEARTBEAT = 0x11,
    FCEC_KIND_TXN_RESULT = 0x80,
    FCEC_KIND_HELLO_RESULT = 0x81,
    FCEC_KIND_STATUS = 0x90
} fcec_kind_t;

typedef enum {
    FCEC_SOURCE_FC = 1,
    FCEC_SOURCE_EC = 2
} fcec_source_t;

typedef enum {
    FCEC_STATE_BOOT_SAFE = 0, /* just booted; no session, no output authority */
    FCEC_STATE_DISARMED = 1,  /* outputs safe; demands stored, not applied */
    FCEC_STATE_ARMED = 2,     /* demands drive the outputs */
    FCEC_STATE_SAFING = 3     /* safed and latched until ClearSafing */
} fcec_ec_state_t;

typedef enum {
    FCEC_OUTCOME_ACCEPTED = 0,
    FCEC_OUTCOME_REJECTED = 1
} fcec_outcome_t;

/* Why something was rejected, or why the EC safed. */
typedef enum {
    FCEC_REASON_NONE = 0,
    FCEC_REASON_BAD_SESSION = 1,
    FCEC_REASON_BAD_BOOT_ID = 2,
    FCEC_REASON_WRONG_STATE = 3,
    FCEC_REASON_CONFIG_MISMATCH = 4,
    FCEC_REASON_NOT_READY = 5,
    FCEC_REASON_SAFING_LATCHED = 6,
    FCEC_REASON_UNSUPPORTED_VERSION = 7,
    FCEC_REASON_UNSUPPORTED_CHANNEL = 8,
    FCEC_REASON_BAD_LENGTH = 9,
    FCEC_REASON_OUT_OF_RANGE = 10,
    FCEC_REASON_NON_FINITE = 11,
    FCEC_REASON_STALE = 12,
    FCEC_REASON_BUSY = 13,
    FCEC_REASON_DEMAND_TIMEOUT = 14,
    FCEC_REASON_LINK_TIMEOUT = 15,
    FCEC_REASON_LOCAL_FAULT = 16,
    FCEC_REASON_FC_TERMINATE = 17
} fcec_reason_t;

/* Return codes of fcec_encode() (negative) and fcec_decode(). */
typedef enum {
    FCEC_OK = 0,
    FCEC_ERR_TOO_SHORT = -1,           /* shorter than header + CRC */
    FCEC_ERR_TOO_LONG = -2,            /* over FCEC_MAX_DATAGRAM_LEN */
    FCEC_ERR_BAD_MAGIC = -3,           /* doesn't start with "EC" */
    FCEC_ERR_UNSUPPORTED_VERSION = -4, /* version isn't FCEC_VERSION */
    FCEC_ERR_BAD_CRC = -5,             /* CRC doesn't match the contents */
    FCEC_ERR_INVALID_ENUM = -6,        /* a kind, state, outcome or reason with no meaning */
    FCEC_ERR_BAD_LENGTH = -7,          /* payload length wrong for the kind */
    FCEC_ERR_WRONG_SOURCE = -8,        /* source byte wrong for the kind */
    FCEC_ERR_BUFFER_TOO_SMALL = -9,    /* encode buffer shorter than the packet */
    FCEC_ERR_INTERNAL = -10            /* payload writer disagrees with the length table */
} fcec_result_t;

/* Header fields the sender chooses. Magic, version, kind, payload length and
 * source are filled in from the kind when encoding. */
typedef struct {
    bool duplicate;          /* on a TxnResult: repeats an earlier outcome */
    uint32_t ec_boot_id;     /* random per EC boot; FC copies the latest one */
    uint32_t session_id;     /* picked by the FC in Hello; 0 = no session */
    uint32_t seq;            /* per sender and session, +1 per message */
    uint32_t txn_id;         /* transactions and their results, else 0 */
    uint32_t bundle_tag;     /* first 4 bytes of the bundle SHA-256; logs only */
    uint16_t valid_for_ms;   /* FC -> EC: expires this long after ec_time_ref_us */
    uint64_t sent_time_us;   /* sender's monotonic clock */
    uint64_t ec_time_ref_us; /* FC -> EC: EC time of the newest Status it had */
} fcec_header_t;

/* Outputs for every channel, in physical units. */
typedef struct {
    float tvc_extension_mm[2]; /* A1, A2 */
    float rcs_duty[4];         /* M1..M4, 0..1; duty, not speed */
    float main_throttle;       /* 0..1, one signal shared by both ESCs */
} fcec_outputs_t;

typedef struct {
    uint32_t fc_boot_id;
    uint32_t run_id;
    uint16_t demand_period_ms;
    uint16_t heartbeat_period_ms;
    uint8_t expected_bundle_sha256[32]; /* all zeros = no bundle */
} fcec_hello_t;

typedef struct {
    fcec_outcome_t outcome;
    fcec_ec_state_t ec_state;
    fcec_reason_t reason;
    uint16_t capabilities; /* FCEC_CH_* bits the EC can drive */
    uint16_t max_demand_age_ms;
    uint32_t firmware_version;
    uint32_t calibration_id;
    uint16_t link_timeout_ms;
    uint8_t active_bundle_sha256[32]; /* all zeros = no bundle */
} fcec_hello_result_t;

typedef struct {
    uint32_t run_id;
    uint8_t expected_bundle_sha256[32];
} fcec_arm_t;

typedef struct {
    fcec_reason_t reason;
} fcec_terminate_t;

/* Periodic demand. Its command ID is the header seq. */
typedef struct {
    fcec_outputs_t outputs;
    uint16_t enable_mask; /* FCEC_CH_* bits this demand drives */
} fcec_demand_t;

typedef struct {
    fcec_kind_t request_kind;
    fcec_outcome_t outcome;
    fcec_reason_t reason;
    fcec_ec_state_t ec_state;
} fcec_txn_result_t;

/* Link health counters. Each wraps at 0xFFFF. */
typedef struct {
    uint16_t crc_errors;       /* bad magic, version, length or CRC */
    uint16_t rejected;         /* rejected after decoding */
    uint16_t stale;            /* rejected as too old */
    uint16_t old_or_duplicate; /* demands not newer than the last accepted */
    uint16_t txn_queue_overflow;
    uint16_t diag_overflow;
    uint16_t deadline_miss;
} fcec_counters_t;

/* A value only means something if its validity_mask bit is set: 0-1 TVC
 * position, 2-5 rail voltage, 6-9 rail current, 10-13 RCS current, 14 motor
 * rail voltage, 15-16 board temperature. */
typedef struct {
    uint32_t sample_id;
    uint64_t sample_time_us;
    uint32_t validity_mask;
    float tvc_position_mm[2];
    uint16_t tvc_position_age_ms[2];
    uint16_t pa12_fault_flags[2];
    float rail_voltage_v[4]; /* input, 12 V, 4.4 V, 3.3 V */
    float rail_current_a[4]; /* same rails */
    float rcs_current_a[4];  /* M1..M4 */
    float motor_rail_voltage_v;
    float board_temp_c[2]; /* U3, U4 */
} fcec_measurements_t;

typedef struct {
    fcec_ec_state_t ec_state;
    bool output_authority;
    bool session_valid;
    bool safing_latched;
    bool demand_fresh;
    fcec_reason_t safing_reason;
    uint32_t last_received_seq;
    uint32_t last_accepted_demand_seq;
    uint32_t last_applied_demand_seq; /* written to outputs, not reached */
    uint64_t last_applied_time_us;
    fcec_reason_t last_reject_reason;
    uint16_t limited_mask; /* channels clamped in the last applied demand */
    fcec_counters_t counters;
    uint8_t reset_cause;
    uint32_t firmware_version;
    fcec_outputs_t applied; /* after EC limits */
    fcec_measurements_t measurements;
} fcec_status_t;

/* One datagram. `kind` picks which member of `msg` is used; Disarm,
 * ClearSafing and Heartbeat have no payload. */
typedef struct {
    fcec_header_t header;
    fcec_kind_t kind;
    union {
        fcec_hello_t hello;
        fcec_arm_t arm;
        fcec_terminate_t terminate;
        fcec_demand_t demand;
        fcec_txn_result_t txn_result;
        fcec_hello_result_t hello_result;
        fcec_status_t status;
    } msg;
} fcec_packet_t;

/* Payload length of `kind` in bytes, or -1 if it isn't a kind. */
int fcec_payload_len(fcec_kind_t kind);

/* Encodes `packet` into `buf` (capacity `cap`). Returns the number of bytes
 * written, or a negative fcec_result_t. */
int fcec_encode(const fcec_packet_t *packet, uint8_t *buf, size_t cap);

/* Decodes one datagram into `out`. Only checks the bytes are well formed;
 * sessions, freshness and ranges are up to the caller. `out` is zeroed first,
 * so it is safe to inspect even on error. */
fcec_result_t fcec_decode(const uint8_t *buf, size_t len, fcec_packet_t *out);

/* FCEC_REASON_NONE if every output is finite and within its physical range,
 * else FCEC_REASON_NON_FINITE or FCEC_REASON_OUT_OF_RANGE. */
fcec_reason_t fcec_check_outputs(const fcec_outputs_t *outputs);

/* Like fcec_check_outputs(), plus FCEC_REASON_UNSUPPORTED_CHANNEL if the
 * demand enables a channel not in `capabilities`. */
fcec_reason_t fcec_check_demand(const fcec_demand_t *demand, uint16_t capabilities);

/* Whether sequence number or transaction ID `a` comes after `b`, treating the
 * uint32_t as wrapping. */
bool fcec_is_newer(uint32_t a, uint32_t b);

/* CRC-32/ISO-HDLC (the zlib/Ethernet CRC-32). "123456789" -> 0xCBF43926. */
uint32_t fcec_crc32(const uint8_t *data, size_t len);

/* Short names for logs. */
const char *fcec_result_str(fcec_result_t result);
const char *fcec_reason_str(fcec_reason_t reason);
const char *fcec_state_str(fcec_ec_state_t state);

#endif /* FC_EC_LINK_H */
