/*
 * sil_ec: software-in-the-loop engine controller.
 *
 * Runs the REAL EC firmware logic on a Linux/WSL host, behind a UDP socket,
 * against a SIMULATED vehicle, so the flight computer (flight2, ec_link.rs)
 * can be exercised end to end without hardware.
 *
 *   REAL firmware code (compiled unchanged from this directory):
 *     ec_side.c      the EC state machine: sessions, arming, freshness, demand
 *                    and link timeouts, Terminate/ClearSafing, profile check
 *     esc.c          the main ESC driver (arm hold, cap/slew, safe pulse,
 *                    confirm-after-one-frame)
 *     pa12_drv.c     the PA-12 TVC driver (packets, polling, readiness,
 *                    safing action, confirm-on-last-byte)
 *     measurement.c  PA-12 position publication into Status
 *     fc_ec_link.c   the wire protocol
 *
 *   Re-implemented here (mirrors the STM32 glue, ctv_engine_ctrl/CM7/Core/Src):
 *     ec_app.c       EC_App_Init / EC_App_Process: app_init() / app_process()
 *                    below are the same wiring and the same ordering
 *     ec_udp.c       the UDP transport: sender filter, peer learning, replies
 *                    to the datagram's source, Status to the peer
 *
 *   SIMULATED hardware (the only invented parts; see the "Simulated
 *   hardware" sections):
 *     the PA-12 UART bus + actuators (pa12_uart.c), the TIM2 ESC pulse
 *     (esc_tim2.c) and the monotonic clock (HAL_GetTick). The profile is
 *     the FAKE one in sil_profile.h, never ec_bench_profile.h.
 *
 * Not simulated: board health (rails, temperatures; EC_BOARD_HEALTH), the RCS
 * fans (also unconfigured on the real EC), watchdog, reset cause, Ethernet.
 *
 * Output (stdout, one line each, key=value tokens, see sil_ec --help):
 *   EVENT ...   every state change, session open/close, transaction result
 *   TICK  ...   the applied outputs, every --period-ms (default 100)
 *
 * Build and run:  make sil_ec && ./sil_ec -p 8378
 */

#define _POSIX_C_SOURCE 200809L

#include "ec_side.h"
#include "esc.h"
#include "fc_ec_link.h"
#include "pa12_drv.h"
#include "sil_profile.h"

#include <arpa/inet.h>
#include <errno.h>
#include <fcntl.h>
#include <getopt.h>
#include <math.h>
#include <netinet/in.h>
#include <poll.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <time.h>
#include <unistd.h>

#define EC_STATUS_PERIOD_US (10000ULL) /* as ec_app.c */
#define DEFAULT_BOOT_ID 0x51B00701u    /* deterministic mode */
#define DEFAULT_PERIOD_MS 100u
#define DETERMINISTIC_TICK_US 1000ULL
#define PA12_DEVICE_DELAY_US 300ULL /* actuator turnaround before it answers */

/* ---- Command line ------------------------------------------------------- */

typedef struct {
    uint16_t port;
    uint16_t fc_port; /* only accept datagrams from this source port; 0 = any */
    const char *bind_addr;
    bool deterministic;
    bool have_boot_id;
    uint32_t boot_id;
    float speed_mm_s;
    float initial_mm;
    uint32_t pa12_baud;
    bool have_safe_action;
    fcec_pa12_safe_action_t safe_action;
    uint32_t esc_arm_hold_ms;
    bool no_esc;
    bool no_tvc[FCEC_PA12_PORTS];
    uint32_t period_ms;
    const char *csv_path;
    bool have_drop;
    uint32_t drop_after_ms;
    uint32_t drop_for_ms; /* 0 = forever */
    unsigned stuck_mask;  /* bit i: PA-12 port i frozen */
    unsigned silent_mask; /* bit i: PA-12 port i never answers */
    uint32_t inject_at_ms;
    uint32_t exit_after_ms; /* 0 = run until signalled */
} sil_cfg_t;

static sil_cfg_t cfg;

static volatile sig_atomic_t running = 1;

static void on_signal(int sig) {
    (void)sig;
    running = 0;
}

static uint64_t monotonic_us(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000u + (uint64_t)ts.tv_nsec / 1000u;
}

static void sleep_until_us(uint64_t start_us, uint64_t target_us) {
    uint64_t now = monotonic_us() - start_us;
    if (now >= target_us) {
        return;
    }
    uint64_t wait = target_us - now;
    struct timespec ts;
    ts.tv_sec = (time_t)(wait / 1000000u);
    ts.tv_nsec = (long)(wait % 1000000u) * 1000L;
    nanosleep(&ts, NULL);
}

static uint32_t random_boot_id(void) {
    uint32_t id = 0;
    FILE *f = fopen("/dev/urandom", "rb");
    if (f != NULL) {
        if (fread(&id, sizeof id, 1, f) != 1) {
            id = 0;
        }
        fclose(f);
    }
    if (id == 0) {
        id = (uint32_t)monotonic_us() ^ (uint32_t)getpid();
    }
    return id != 0 ? id : 1u;
}

static const char *reason_name(fcec_reason_t r) {
    static const char *const names[] = {
        "NONE",           "BAD_SESSION",  "BAD_BOOT_ID",      "WRONG_STATE",
        "CONFIG_MISMATCH", "NOT_READY",   "SAFING_LATCHED",   "UNSUPPORTED_VERSION",
        "UNSUPPORTED_CHANNEL", "BAD_LENGTH", "OUT_OF_RANGE",  "NON_FINITE",
        "STALE",          "BUSY",         "DEMAND_TIMEOUT",   "LINK_TIMEOUT",
        "LOCAL_FAULT",    "FC_TERMINATE"};
    unsigned i = (unsigned)r;
    return i < sizeof names / sizeof names[0] ? names[i] : "UNKNOWN";
}

/* ---- Simulated hardware: clock ------------------------------------------ */

/* Physical simulation time (us since start). The firmware sees ec_now_us
 * instead: whole milliseconds, like HAL_GetTick() * 1000 in the real EC. */
static uint64_t hw_now_us;

/* ---- Simulated hardware: ESC pulse timer (replaces esc_tim2.c) ---------- */

/* A pulse loaded with sim_esc_load() goes out from the next frame boundary,
 * never mid-pulse (compare preload on, as ESC_TIM2_Start() sets up). */
static struct {
    bool running;
    uint32_t frame_us;
    uint16_t active_us; /* the pulse on the wire now */
    uint16_t loaded_us; /* waiting for the next frame boundary */
    uint64_t next_frame_us;
    uint64_t frames;
} sim_esc;

static bool sim_esc_start(uint16_t frame_period_us, uint16_t first_pulse_us) {
    if (frame_period_us == 0U || first_pulse_us >= frame_period_us) {
        return false; /* as ESC_TIM2_Start() */
    }
    sim_esc.running = true;
    sim_esc.frame_us = frame_period_us;
    sim_esc.active_us = first_pulse_us;
    sim_esc.loaded_us = first_pulse_us;
    sim_esc.next_frame_us = hw_now_us + frame_period_us;
    return true;
}

static void sim_esc_load(uint16_t pulse_us) {
    sim_esc.loaded_us = pulse_us;
}

static void sim_esc_advance(void) {
    while (sim_esc.running && hw_now_us >= sim_esc.next_frame_us) {
        sim_esc.active_us = sim_esc.loaded_us;
        sim_esc.next_frame_us += sim_esc.frame_us;
        sim_esc.frames++;
    }
}

/* ---- Simulated hardware: PA-12 bus and actuators (replaces pa12_uart.c) -- */

/* Same wire protocol as the packets in test_pa12.c / pa12_drv.c:
 *   FF FF FF | ID | size | instr | factors | checksum
 * The actuator model follows test_pa12.c and fake_ec.c: a goal write gets no
 * reply (default protocol mode), a position read gets a 9-byte reply with
 * error flags 0, and the actuator keeps moving to its last goal at a fixed
 * speed even if no new goal arrives. Force-off stops it (held by friction)
 * until the next goal write. */
#define PA12_FIFO_DEPTH 16u
#define PA_REG_FORCE 0x80u
#define PA_REG_GOAL 0x86u
#define PA_REG_PRESENT_POSITION 0x8Cu

typedef struct {
    /* bus */
    bool tx_active;
    uint64_t tx_done_us;
    uint8_t tx[PA12_FIFO_DEPTH];
    size_t tx_len;
    bool reply_pending;
    uint8_t reply[PA12_FIFO_DEPTH];
    size_t reply_len;
    uint64_t reply_at_us;
    /* actuator */
    bool torque_on;
    float pos_counts;
    float goal_counts;
    unsigned goals_received;
    unsigned reads_answered;
} sim_pa12_port_t;

static sim_pa12_port_t sim_pa12[FCEC_PA12_PORTS];
static fcec_pa12_profile_t pa12_prof[FCEC_PA12_PORTS]; /* sil_pa12_profile + CLI overrides */
static bool inject_reported;

static bool injection_active(void) {
    return hw_now_us >= (uint64_t)cfg.inject_at_ms * 1000u;
}

static bool pa12_stuck(unsigned port) {
    return injection_active() && ((cfg.stuck_mask >> port) & 1u) != 0u;
}

static bool pa12_silent(unsigned port) {
    return injection_active() && ((cfg.silent_mask >> port) & 1u) != 0u;
}

static uint64_t pa12_wire_us(size_t bytes) {
    return (uint64_t)bytes * 10u * 1000000u / cfg.pa12_baud; /* 8N1 */
}

static float pa12_counts_to_mm(unsigned port, float counts) {
    return (counts - pa12_prof[port].counts_at_0mm) / pa12_prof[port].counts_per_mm;
}

static float pa12_mm_to_counts_f(unsigned port, float mm) {
    return pa12_prof[port].counts_per_mm * mm + pa12_prof[port].counts_at_0mm;
}

/* The actuator receives a complete packet. */
static void sim_pa12_device_rx(unsigned port) {
    sim_pa12_port_t *p = &sim_pa12[port];
    const uint8_t *b = p->tx;
    size_t n = p->tx_len;
    if (n < 7u || b[0] != 0xFFu || b[1] != 0xFFu || b[2] != 0xFFu ||
        b[3] != pa12_prof[port].id || (size_t)b[4] + 5u != n) {
        return; /* not for this actuator, or malformed */
    }
    uint8_t sum = 0;
    for (size_t i = 3; i + 1u < n; i++) {
        sum = (uint8_t)(sum + b[i]);
    }
    if ((uint8_t)~sum != b[n - 1u]) {
        return; /* bad checksum: ignored */
    }
    uint8_t instr = b[5];
    const uint8_t *f = &b[6];
    size_t nf = n - 7u;

    if (instr == 0xF3u && nf == 3u && f[0] == PA_REG_GOAL) { /* write goal */
        uint16_t counts = (uint16_t)(f[1] | ((unsigned)f[2] << 8));
        if (counts > FCEC_PA12_MAX_COUNTS) {
            counts = FCEC_PA12_MAX_COUNTS;
        }
        p->goal_counts = (float)counts;
        p->torque_on = true; /* a goal turns the force back on */
        p->goals_received++;
    } else if (instr == 0xF3u && nf == 2u && f[0] == PA_REG_FORCE) { /* force on/off */
        p->torque_on = f[1] != 0u;
    } else if (instr == 0xF2u && nf == 2u && f[0] == PA_REG_PRESENT_POSITION && f[1] == 2u) {
        if (pa12_silent(port)) {
            return; /* injected fault: no answer */
        }
        float c = p->pos_counts;
        c = c < 0.0f ? 0.0f : (c > (float)FCEC_PA12_MAX_COUNTS ? (float)FCEC_PA12_MAX_COUNTS : c);
        uint16_t pos = (uint16_t)(c + 0.5f);
        uint8_t *r = p->reply;
        r[0] = r[1] = r[2] = 0xFFu;
        r[3] = pa12_prof[port].id;
        r[4] = 4u;
        r[5] = 0u; /* error flags */
        r[6] = (uint8_t)(pos & 0xFFu);
        r[7] = (uint8_t)(pos >> 8);
        uint8_t s = 0;
        for (int i = 3; i < 8; i++) {
            s = (uint8_t)(s + r[i]);
        }
        r[8] = (uint8_t)~s;
        p->reply_len = 9u;
        p->reply_pending = true;
        p->reply_at_us = p->tx_done_us + PA12_DEVICE_DELAY_US + pa12_wire_us(p->reply_len);
        p->reads_answered++;
    }
}

/* fcec_pa12_ops_t.start_tx: as pa12_uart.c, non-blocking. */
static bool sim_pa12_start_tx(void *context, unsigned port, const uint8_t *bytes, size_t len) {
    (void)context;
    if (port >= FCEC_PA12_PORTS || sim_pa12[port].tx_active || len == 0u ||
        len > PA12_FIFO_DEPTH) {
        return false;
    }
    sim_pa12_port_t *p = &sim_pa12[port];
    memcpy(p->tx, bytes, len);
    p->tx_len = len;
    p->reply_pending = false; /* flush_rx() */
    p->tx_active = true;
    p->tx_done_us = hw_now_us + pa12_wire_us(len);
    return true;
}

static const fcec_pa12_ops_t sim_pa12_ops = {NULL, sim_pa12_start_tx};

/* PA12_UART_Poll(): finishes transmissions and feeds received bytes. */
static void sim_pa12_poll(fcec_pa12_t *drv, uint64_t ec_now_us) {
    for (unsigned i = 0; i < FCEC_PA12_PORTS; i++) {
        sim_pa12_port_t *p = &sim_pa12[i];
        if (p->tx_active && hw_now_us >= p->tx_done_us) {
            p->tx_active = false;
            sim_pa12_device_rx(i); /* the actuator got the whole packet */
            fcec_pa12_tx_done(drv, i, true, ec_now_us);
        }
        if (p->reply_pending && hw_now_us >= p->reply_at_us) {
            p->reply_pending = false;
            for (size_t k = 0; k < p->reply_len; k++) {
                fcec_pa12_rx_byte(drv, i, p->reply[k], ec_now_us);
            }
        }
        /* If the driver gave up on a transfer (timeout), release the line. */
        if (p->tx_active && drv->port[i].phase != FCEC_PA12_PHASE_TX) {
            p->tx_active = false;
            p->reply_pending = false;
        }
    }
}

/* Moves each actuator toward its goal. */
static void sim_pa12_physics(float dt_s) {
    for (unsigned i = 0; i < FCEC_PA12_PORTS; i++) {
        sim_pa12_port_t *p = &sim_pa12[i];
        if (!p->torque_on || pa12_stuck(i)) {
            continue;
        }
        float step = cfg.speed_mm_s * pa12_prof[i].counts_per_mm * dt_s;
        float error = p->goal_counts - p->pos_counts;
        if (error > step) {
            error = step;
        } else if (error < -step) {
            error = -step;
        }
        p->pos_counts += error;
    }
}

/* ---- Output ------------------------------------------------------------- */

static FILE *csv;

/* ---- UDP transport (mirrors ec_udp.c) ----------------------------------- */

typedef struct {
    uint32_t rx_accepted;
    uint32_t rx_wrong_sender;
    uint32_t rx_invalid_size;
    uint32_t rx_dropped_fault; /* --drop-after-ms */
    uint32_t replies_sent;
    uint32_t statuses_sent;
    uint32_t tx_failed;
} sil_udp_stats_t;

static int udp_fd = -1;
static struct sockaddr_in peer_addr;
static bool peer_known;
static sil_udp_stats_t udp_stats;
static bool link_dropped; /* fault injection: the EC is deaf and mute */

typedef size_t (*udp_receive_handler_t)(const uint8_t *rx, size_t len, uint64_t now_us,
                                        uint8_t *reply, size_t reply_cap, bool *accepted);

static bool udp_init(void) {
    udp_fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (udp_fd < 0) {
        perror("socket");
        return false;
    }
    int yes = 1;
    setsockopt(udp_fd, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof yes);
    struct sockaddr_in local;
    memset(&local, 0, sizeof local);
    local.sin_family = AF_INET;
    local.sin_port = htons(cfg.port);
    if (cfg.bind_addr == NULL) {
        local.sin_addr.s_addr = htonl(INADDR_ANY);
    } else if (inet_pton(AF_INET, cfg.bind_addr, &local.sin_addr) != 1) {
        fprintf(stderr, "sil_ec: bad --bind address '%s'\n", cfg.bind_addr);
        return false;
    }
    if (bind(udp_fd, (struct sockaddr *)&local, sizeof local) < 0) {
        perror("bind");
        return false;
    }
    fcntl(udp_fd, F_SETFL, fcntl(udp_fd, F_GETFL, 0) | O_NONBLOCK);
    peer_known = false;
    return true;
}

static bool udp_has_peer(void) {
    return peer_known;
}

static bool udp_send_status(const uint8_t *data, size_t len) {
    if (!peer_known || link_dropped) {
        return false;
    }
    if (len == 0u || len > FCEC_MAX_DATAGRAM_LEN ||
        sendto(udp_fd, data, len, 0, (struct sockaddr *)&peer_addr, sizeof peer_addr) < 0) {
        udp_stats.tx_failed++;
        return false;
    }
    udp_stats.statuses_sent++;
    return true;
}

/* ---- Event logging ------------------------------------------------------ */

static void event_prefix(uint64_t ec_now_us, const char *kind) {
    printf("EVENT t_ms=%llu kind=%s", (unsigned long long)(ec_now_us / 1000u), kind);
}

static const char *kind_name(fcec_kind_t k) {
    switch (k) {
    case FCEC_KIND_HELLO: return "Hello";
    case FCEC_KIND_ARM: return "Arm";
    case FCEC_KIND_DISARM: return "Disarm";
    case FCEC_KIND_TERMINATE: return "Terminate";
    case FCEC_KIND_CLEAR_SAFING: return "ClearSafing";
    default: return "Other";
    }
}

static void log_reply(const uint8_t *reply, size_t n, const struct sockaddr_in *from,
                      uint64_t ec_now_us);

/* ---- The EC application (mirrors ec_app.c, same order and wiring) -------- */

static fcec_ec_t ec;
static fcec_esc_t esc;
static bool esc_running;
static fcec_pa12_t pa12;
static bool initialized;
static uint64_t next_status_us;

static fcec_ec_state_t shown_state;
static bool shown_session;
static uint16_t shown_ready;
static uint16_t shown_fault;

/* Prints a line for every change in state, session, readiness or faults. Call
 * after anything that can change them, so none is missed between prints. */
static void note_changes(uint64_t ec_now_us) {
    if (ec.session_valid != shown_session) {
        event_prefix(ec_now_us, "session");
        printf(" state=%s session_id=0x%08X\n", ec.session_valid ? "opened" : "closed",
               (unsigned)ec.session_id);
        shown_session = ec.session_valid;
    }
    if (ec.state != shown_state) {
        event_prefix(ec_now_us, "state");
        printf(" from=%s to=%s reason=%s", fcec_state_str(shown_state), fcec_state_str(ec.state),
               ec.state == FCEC_STATE_SAFING ? reason_name(ec.safing_reason) : "NONE");
        if (ec.state == FCEC_STATE_SAFING) {
            long long since = ec.demand_due_us <= ec_now_us
                                  ? (long long)((ec_now_us - ec.demand_due_us) / 1000u)
                                  : -1;
            printf(" since_demand_ms=%lld", since);
        }
        printf("\n");
        shown_state = ec.state;
    }
    if (ec.ready_mask != shown_ready || ec.fault_mask != shown_fault) {
        event_prefix(ec_now_us, "drivers");
        printf(" ready_mask=0x%02X fault_mask=0x%02X capabilities=0x%02X\n",
               (unsigned)ec.ready_mask, (unsigned)ec.fault_mask, (unsigned)ec.capabilities);
        shown_ready = ec.ready_mask;
        shown_fault = ec.fault_mask;
    }
}

/* The handler ec_app.c gives the transport. */
static size_t on_ec_udp_message(const uint8_t *rx, size_t len, uint64_t now_us, uint8_t *reply,
                                size_t reply_cap, bool *accepted) {
    const uint64_t previous_rx_us = ec.last_rx_us;
    const size_t n = fcec_ec_receive(&ec, rx, len, now_us, reply, reply_cap);
    /* Mirrors fake_ec.c: remember the sender when the EC accepts the
     * message as valid or generates a protocol-level transaction reply. */
    *accepted = (ec.last_rx_us != previous_rx_us || n > 0U);
    return n;
}

/* ec_udp.c on_receive(), for every datagram waiting on the socket. */
static void udp_poll(uint64_t ec_now_us, udp_receive_handler_t handler) {
    for (;;) {
        uint8_t rx[FCEC_MAX_DATAGRAM_LEN + 1];
        struct sockaddr_in from;
        socklen_t from_len = sizeof from;
        ssize_t got = recvfrom(udp_fd, rx, sizeof rx, 0, (struct sockaddr *)&from, &from_len);
        if (got < 0) {
            if (errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR) {
                perror("recvfrom");
            }
            return;
        }
        if (link_dropped) {
            udp_stats.rx_dropped_fault++; /* injected fault: heard nothing */
            continue;
        }
        /* IP and port filtering prevents accidental cross-talk, not spoofing.
         * The real EC checks 192.168.1.11 : 4574; on a host only the port. */
        if (cfg.fc_port != 0u && ntohs(from.sin_port) != cfg.fc_port) {
            udp_stats.rx_wrong_sender++;
            continue;
        }
        size_t n = (size_t)got;
        if (n < FCEC_HEADER_LEN + FCEC_CRC_LEN || n > FCEC_MAX_DATAGRAM_LEN) {
            udp_stats.rx_invalid_size++;
            continue;
        }

        uint8_t reply[FCEC_MAX_DATAGRAM_LEN];
        bool accepted = false;
        size_t reply_len = handler(rx, n, ec_now_us, reply, sizeof reply, &accepted);
        if (reply_len > sizeof reply) {
            udp_stats.tx_failed++;
            continue;
        }
        if (accepted) {
            peer_addr = from;
            peer_known = true;
            udp_stats.rx_accepted++;
        }
        /* Reply to this datagram's source, even for a protocol-level rejection. */
        if (reply_len > 0u) {
            if (sendto(udp_fd, reply, reply_len, 0, (struct sockaddr *)&from, sizeof from) >= 0) {
                udp_stats.replies_sent++;
            } else {
                udp_stats.tx_failed++;
            }
            log_reply(reply, reply_len, &from, ec_now_us);
        }
        note_changes(ec_now_us);
    }
}

static void log_reply(const uint8_t *reply, size_t n, const struct sockaddr_in *from,
                      uint64_t ec_now_us) {
    fcec_packet_t r;
    if (fcec_decode(reply, n, &r) != FCEC_OK) {
        return;
    }
    if (r.kind == FCEC_KIND_HELLO_RESULT) {
        bool ok = r.msg.hello_result.outcome == FCEC_OUTCOME_ACCEPTED;
        event_prefix(ec_now_us, "txn");
        printf(" request=Hello outcome=%s reason=%s txn_id=%u session_id=0x%08X from_port=%u\n",
               ok ? "ACCEPTED" : "REJECTED", reason_name(r.msg.hello_result.reason),
               (unsigned)r.header.txn_id, (unsigned)r.header.session_id,
               (unsigned)ntohs(from->sin_port));
    } else if (r.kind == FCEC_KIND_TXN_RESULT) {
        bool ok = r.msg.txn_result.outcome == FCEC_OUTCOME_ACCEPTED;
        event_prefix(ec_now_us, "txn");
        printf(" request=%s outcome=%s reason=%s txn_id=%u ec_state=%s%s\n",
               kind_name(r.msg.txn_result.request_kind), ok ? "ACCEPTED" : "REJECTED",
               reason_name(r.msg.txn_result.reason), (unsigned)r.header.txn_id,
               fcec_state_str(r.msg.txn_result.ec_state),
               r.header.duplicate ? " duplicate=1" : "");
    }
}

static bool app_init(uint32_t boot_id, uint64_t now_us) {
    if (initialized || boot_id == 0U) {
        return false;
    }
    fcec_ec_init(&ec, boot_id);
    /* Real drivers: an output only counts as applied when its driver confirms
     * it (fcec_ec_confirm_applied), and only configured channels are offered.
     * Nothing is ready until its driver says so. */
    ec.auto_confirm = false;
    ec.capabilities = 0U;
    fcec_ec_set_ready(&ec, FCEC_CH_ALL, false);
    ec.firmware_version = SIL_FIRMWARE_VERSION; /* the real EC reports 1 */

    /* Profile: identity checked at Arm, TVC park + travel window. */
    static const char profile_id[] = SIL_PROFILE_ID;
    _Static_assert(sizeof profile_id <= sizeof ec.bundle_sha256, "profile ID too long");
    memset(ec.bundle_sha256, 0, sizeof ec.bundle_sha256);
    memcpy(ec.bundle_sha256, profile_id, sizeof profile_id - 1U);
    ec.check_bundle = true;
    ec.tvc_center_mm[0] = SIL_TVC_PARK_MM_A1;
    ec.tvc_center_mm[1] = SIL_TVC_PARK_MM_A2;
    ec.limit_min.tvc_extension_mm[0] = SIL_TVC_MIN_MM;
    ec.limit_min.tvc_extension_mm[1] = SIL_TVC_MIN_MM;
    ec.limit_max.tvc_extension_mm[0] = SIL_TVC_MAX_MM;
    ec.limit_max.tvc_extension_mm[1] = SIL_TVC_MAX_MM;

    /* Main ESC: no signal at all unless its profile is approved. */
    static fcec_esc_config_t esc_prof;
    esc_prof = sil_esc_profile;
    if (cfg.esc_arm_hold_ms != UINT32_MAX) {
        esc_prof.arm_hold_ms = cfg.esc_arm_hold_ms;
    }
    if (cfg.no_esc) {
        esc_prof.approved = false;
    }
    fcec_esc_init(&esc, &esc_prof, now_us);
    if (fcec_esc_config_ok(&esc.config) &&
        sim_esc_start(esc.config.frame_period_us, esc.config.min_pulse_us)) {
        esc_running = true;
        ec.capabilities |= FCEC_CH_MAIN_ESC;
        ec.limit_max.main_throttle = esc.config.max_throttle;
    }

    /* PA-12s: only approved actuators are offered; nothing is sent to the
     * others. */
    ec.capabilities |= fcec_pa12_init(&pa12, pa12_prof, &sim_pa12_ops, now_us);

    /* Nothing is valid until a driver has a real reading for it. */
    ec.measurements.validity_mask = 0U;

    if (!udp_init()) {
        return false; /* Remain out of BOOT_SAFE until transport exists. */
    }
    fcec_ec_boot_done(&ec);
    next_status_us = now_us + EC_STATUS_PERIOD_US;
    initialized = true;
    return true;
}

static void app_process(uint64_t now_us) {
    if (!initialized) {
        return;
    }
    /* This runs even if no packets are received, so link and demand timeouts
     * still trigger the EC state machine. */
    fcec_ec_tick(&ec, now_us);

    /* Output drivers: each reads fcec_ec_command(&ec), outputs its channels
     * in drive_mask at its own update boundary without blocking, then
     * confirms (fcec_ec_confirm_applied) or reports a fault. */
    if (esc_running) {
        sim_esc_load(fcec_esc_service(&esc, &ec, now_us));
    }
    sim_pa12_poll(&pa12, now_us); /* PA12_UART_Poll() */
    fcec_pa12_service(&pa12, &ec, now_us);

    if (now_us < next_status_us) {
        return;
    }
    fcec_pa12_publish(&pa12, &ec.measurements, now_us);
    /* One measurement batch per Status. */
    ec.measurements.sample_id++;
    ec.measurements.sample_time_us = now_us;
    if (udp_has_peer()) {
        static uint8_t status[FCEC_MAX_DATAGRAM_LEN];
        const size_t n = fcec_ec_status(&ec, now_us, status, sizeof(status));
        if (n > 0U) {
            (void)udp_send_status(status, n);
        }
    }

    /* Move to the next future 10 ms slot without bursting old statuses. */
    const uint64_t late = now_us - next_status_us;
    const uint64_t intervals = late / EC_STATUS_PERIOD_US + 1ULL;
    if (intervals > 1ULL) {
        /* This counter records missed status deadlines, saturating per step. */
        uint64_t missed = intervals - 1ULL;
        if (missed > 0xFFFFULL) {
            missed = 0xFFFFULL;
        }
        ec.counters.deadline_miss = (uint16_t)(ec.counters.deadline_miss + (uint16_t)missed);
    }
    next_status_us += intervals * EC_STATUS_PERIOD_US;
}

/* ---- Telemetry ---------------------------------------------------------- */

static const char *CSV_HEADER =
    "t_ms,state,reason,a1_mm,a1_goal_mm,a1_meas_mm,a2_mm,a2_goal_mm,a2_meas_mm,thr,esc_us,"
    "esc_load_us,dem_seq,app_seq,drive,ready,fault";

static void emit_tick(uint64_t ec_now_us) {
    fcec_ec_command_t cmd = fcec_ec_command(&ec);
    double pos[FCEC_PA12_PORTS], goal[FCEC_PA12_PORTS], meas[FCEC_PA12_PORTS];
    for (unsigned i = 0; i < FCEC_PA12_PORTS; i++) {
        pos[i] = (double)pa12_counts_to_mm(i, sim_pa12[i].pos_counts);
        goal[i] = (double)pa12_counts_to_mm(i, sim_pa12[i].goal_counts);
        bool valid = (ec.measurements.validity_mask & (1u << i)) != 0u;
        meas[i] = valid ? (double)ec.measurements.tvc_position_mm[i] : (double)NAN;
    }
    const char *state = fcec_state_str(ec.state);
    const char *reason = ec.state == FCEC_STATE_SAFING ? reason_name(ec.safing_reason) : "NONE";
    unsigned long long t_ms = (unsigned long long)(ec_now_us / 1000u);
    double thr = (double)esc.throttle;
    unsigned seq_dem = (unsigned)ec.last_accepted_demand_seq;
    unsigned seq_app = (unsigned)ec.last_applied_demand_seq;

    printf("TICK t_ms=%llu state=%s reason=%s a1_mm=%.3f a1_goal_mm=%.3f a1_meas_mm=%.3f "
           "a2_mm=%.3f a2_goal_mm=%.3f a2_meas_mm=%.3f thr=%.3f esc_us=%u esc_load_us=%u "
           "dem_seq=%u app_seq=%u drive=0x%02X ready=0x%02X fault=0x%02X\n",
           t_ms, state, reason, pos[0], goal[0], meas[0], pos[1], goal[1], meas[1], thr,
           (unsigned)sim_esc.active_us, (unsigned)sim_esc.loaded_us, seq_dem, seq_app,
           (unsigned)cmd.drive_mask, (unsigned)ec.ready_mask, (unsigned)ec.fault_mask);
    if (csv != NULL) {
        fprintf(csv, "%llu,%s,%s,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%u,%u,%u,%u,%u,%u,%u\n",
                t_ms, state, reason, pos[0], goal[0], meas[0], pos[1], goal[1], meas[1], thr,
                (unsigned)sim_esc.active_us, (unsigned)sim_esc.loaded_us, seq_dem, seq_app,
                (unsigned)cmd.drive_mask, (unsigned)ec.ready_mask, (unsigned)ec.fault_mask);
        fflush(csv);
    }
}

/* ---- Fault injection ---------------------------------------------------- */

static void update_injection(uint64_t ec_now_us) {
    if (cfg.have_drop) {
        uint64_t start = (uint64_t)cfg.drop_after_ms * 1000u;
        bool in_window = hw_now_us >= start &&
                         (cfg.drop_for_ms == 0u || hw_now_us < start + (uint64_t)cfg.drop_for_ms * 1000u);
        if (in_window != link_dropped) {
            link_dropped = in_window;
            event_prefix(ec_now_us, "inject");
            printf(" what=%s\n", link_dropped ? "drop_start" : "drop_end");
        }
    }
    if (!inject_reported && (cfg.stuck_mask != 0u || cfg.silent_mask != 0u) && injection_active()) {
        inject_reported = true;
        event_prefix(ec_now_us, "inject");
        printf(" what=pa12 stuck_mask=0x%X silent_mask=0x%X\n", cfg.stuck_mask, cfg.silent_mask);
    }
}

/* ---- Options ------------------------------------------------------------ */

static void usage(FILE *out) {
    fprintf(out,
        "usage: sil_ec [options]\n"
        "  Real EC firmware logic (ec_side/esc/pa12_drv) on the host with simulated\n"
        "  PA-12 actuators and ESC, behind UDP. FAKE profile: see sil_profile.h.\n"
        "\n"
        "  -p, --port N           UDP port to listen on (default %u)\n"
        "      --fc-port N        only accept datagrams from this source port, as the real\n"
        "                         EC does (4574, where flight2 sends from); 0 = accept any\n"
        "                         (default 0: unlike the real EC, so tools can use any port)\n"
        "  -b, --bind ADDR        local IPv4 address (default 0.0.0.0)\n"
        "      --deterministic    virtual clock in exact 1 ms steps paced to wall time,\n"
        "                         fixed boot ID %08X (override with --boot-id)\n"
        "      --boot-id HEX      EC boot ID (default random; nonzero)\n"
        "      --speed MM_S       simulated actuator speed (default 20)\n"
        "      --pa12-initial-mm MM  actuator position at boot (default park, 13.45)\n"
        "      --pa12-baud N      simulated PA-12 bus baud (default 115200)\n"
        "      --safe-action hold|park|off   PA-12 safing action (default: hold,\n"
        "                         the profile default)\n"
        "      --esc-arm-hold-ms N   ESC safe-pulse hold from boot (SIL default 1000,\n"
        "                         real profile 5000)\n"
        "      --no-esc | --no-a1 | --no-a2   leave that channel unconfigured\n"
        "      --period-ms N      TICK / CSV period (default %u)\n"
        "      --csv PATH         also write the ticks to a CSV file\n"
        "      --exit-after-ms N  stop after N ms of EC time (default: run forever)\n"
        "  Fault injection:\n"
        "      --drop-after-ms N  after N ms stop hearing and answering the FC\n"
        "      --drop-for-ms N    ... for N ms, then resume (default: forever)\n"
        "      --pa12-stuck a1|a2|both    freeze that actuator (goals accepted, no motion)\n"
        "      --pa12-silent a1|a2|both   that actuator stops answering reads\n"
        "      --inject-at-ms N   when --pa12-stuck/--pa12-silent start (default 0)\n"
        "\n"
        "stdout (one line each, space-separated key=value):\n"
        "  EVENT t_ms=<ec ms> kind=<boot|listening|state|session|drivers|txn|inject|exit> ...\n"
        "  TICK  t_ms= state= reason= a1_mm= a1_goal_mm= a1_meas_mm= a2_mm= a2_goal_mm=\n"
        "        a2_meas_mm= thr= esc_us= esc_load_us= dem_seq= app_seq= drive= ready= fault=\n",
        (unsigned)FCEC_EC_PORT, DEFAULT_BOOT_ID, DEFAULT_PERIOD_MS);
}

static bool parse_u32(const char *name, const char *s, uint32_t max, uint32_t *out) {
    char *end = NULL;
    errno = 0;
    unsigned long v = strtoul(s, &end, 0);
    if (errno != 0 || end == s || *end != '\0' || v > max) {
        fprintf(stderr, "sil_ec: bad value '%s' for %s\n", s, name);
        return false;
    }
    *out = (uint32_t)v;
    return true;
}

static bool parse_float(const char *name, const char *s, float *out) {
    char *end = NULL;
    errno = 0;
    double v = strtod(s, &end);
    if (errno != 0 || end == s || *end != '\0' || !isfinite(v) || v < 0.0) {
        fprintf(stderr, "sil_ec: bad value '%s' for %s\n", s, name);
        return false;
    }
    *out = (float)v;
    return true;
}

static bool parse_ports(const char *s, unsigned *mask) {
    if (strcmp(s, "a1") == 0) {
        *mask = 1u;
    } else if (strcmp(s, "a2") == 0) {
        *mask = 2u;
    } else if (strcmp(s, "both") == 0) {
        *mask = 3u;
    } else {
        fprintf(stderr, "sil_ec: expected a1, a2 or both, got '%s'\n", s);
        return false;
    }
    return true;
}

enum {
    OPT_FC_PORT = 256, OPT_DETERMINISTIC, OPT_BOOT_ID, OPT_SPEED, OPT_INITIAL_MM, OPT_BAUD,
    OPT_SAFE_ACTION, OPT_ESC_HOLD, OPT_NO_ESC, OPT_NO_A1, OPT_NO_A2, OPT_PERIOD, OPT_CSV,
    OPT_EXIT_AFTER, OPT_DROP_AFTER, OPT_DROP_FOR, OPT_STUCK, OPT_SILENT, OPT_INJECT_AT
};

static bool parse_args(int argc, char **argv) {
    static const struct option longopts[] = {
        {"port", required_argument, NULL, 'p'},
        {"bind", required_argument, NULL, 'b'},
        {"help", no_argument, NULL, 'h'},
        {"fc-port", required_argument, NULL, OPT_FC_PORT},
        {"deterministic", no_argument, NULL, OPT_DETERMINISTIC},
        {"boot-id", required_argument, NULL, OPT_BOOT_ID},
        {"speed", required_argument, NULL, OPT_SPEED},
        {"pa12-initial-mm", required_argument, NULL, OPT_INITIAL_MM},
        {"pa12-baud", required_argument, NULL, OPT_BAUD},
        {"safe-action", required_argument, NULL, OPT_SAFE_ACTION},
        {"esc-arm-hold-ms", required_argument, NULL, OPT_ESC_HOLD},
        {"no-esc", no_argument, NULL, OPT_NO_ESC},
        {"no-a1", no_argument, NULL, OPT_NO_A1},
        {"no-a2", no_argument, NULL, OPT_NO_A2},
        {"period-ms", required_argument, NULL, OPT_PERIOD},
        {"csv", required_argument, NULL, OPT_CSV},
        {"exit-after-ms", required_argument, NULL, OPT_EXIT_AFTER},
        {"drop-after-ms", required_argument, NULL, OPT_DROP_AFTER},
        {"drop-for-ms", required_argument, NULL, OPT_DROP_FOR},
        {"pa12-stuck", required_argument, NULL, OPT_STUCK},
        {"pa12-silent", required_argument, NULL, OPT_SILENT},
        {"inject-at-ms", required_argument, NULL, OPT_INJECT_AT},
        {NULL, 0, NULL, 0}};

    memset(&cfg, 0, sizeof cfg);
    cfg.port = FCEC_EC_PORT;
    cfg.fc_port = 0; /* accept any source port; --fc-port 4574 is the real EC's filter */
    cfg.speed_mm_s = 20.0f; /* as fake_ec.c */
    cfg.initial_mm = SIL_TVC_PARK_MM_A1;
    cfg.pa12_baud = 115200u;
    cfg.esc_arm_hold_ms = UINT32_MAX; /* profile default */
    cfg.period_ms = DEFAULT_PERIOD_MS;

    uint32_t v = 0;
    int opt;
    while ((opt = getopt_long(argc, argv, "p:b:h", longopts, NULL)) != -1) {
        switch (opt) {
        case 'p':
            if (!parse_u32("--port", optarg, 65535u, &v)) return false;
            cfg.port = (uint16_t)v;
            break;
        case 'b':
            cfg.bind_addr = optarg;
            break;
        case 'h':
            usage(stdout);
            exit(0);
        case OPT_FC_PORT:
            if (!parse_u32("--fc-port", optarg, 65535u, &v)) return false;
            cfg.fc_port = (uint16_t)v;
            break;
        case OPT_DETERMINISTIC:
            cfg.deterministic = true;
            break;
        case OPT_BOOT_ID:
            if (!parse_u32("--boot-id", optarg, UINT32_MAX, &v) || v == 0u) return false;
            cfg.boot_id = v;
            cfg.have_boot_id = true;
            break;
        case OPT_SPEED:
            if (!parse_float("--speed", optarg, &cfg.speed_mm_s)) return false;
            break;
        case OPT_INITIAL_MM:
            if (!parse_float("--pa12-initial-mm", optarg, &cfg.initial_mm)) return false;
            break;
        case OPT_BAUD:
            if (!parse_u32("--pa12-baud", optarg, 4000000u, &v) || v < 1200u) return false;
            cfg.pa12_baud = v;
            break;
        case OPT_SAFE_ACTION:
            cfg.have_safe_action = true;
            if (strcmp(optarg, "hold") == 0) {
                cfg.safe_action = FCEC_PA12_SAFE_HOLD;
            } else if (strcmp(optarg, "park") == 0) {
                cfg.safe_action = FCEC_PA12_SAFE_PARK;
            } else if (strcmp(optarg, "off") == 0) {
                cfg.safe_action = FCEC_PA12_SAFE_FORCE_OFF;
            } else {
                fprintf(stderr, "sil_ec: --safe-action wants hold, park or off\n");
                return false;
            }
            break;
        case OPT_ESC_HOLD:
            if (!parse_u32("--esc-arm-hold-ms", optarg, 3600000u, &v)) return false;
            cfg.esc_arm_hold_ms = v;
            break;
        case OPT_NO_ESC:
            cfg.no_esc = true;
            break;
        case OPT_NO_A1:
            cfg.no_tvc[0] = true;
            break;
        case OPT_NO_A2:
            cfg.no_tvc[1] = true;
            break;
        case OPT_PERIOD:
            if (!parse_u32("--period-ms", optarg, 3600000u, &v) || v == 0u) return false;
            cfg.period_ms = v;
            break;
        case OPT_CSV:
            cfg.csv_path = optarg;
            break;
        case OPT_EXIT_AFTER:
            if (!parse_u32("--exit-after-ms", optarg, 0xFFFFFFFFu, &v)) return false;
            cfg.exit_after_ms = v;
            break;
        case OPT_DROP_AFTER:
            if (!parse_u32("--drop-after-ms", optarg, 0xFFFFFFFFu, &v)) return false;
            cfg.drop_after_ms = v;
            cfg.have_drop = true;
            break;
        case OPT_DROP_FOR:
            if (!parse_u32("--drop-for-ms", optarg, 0xFFFFFFFFu, &v)) return false;
            cfg.drop_for_ms = v;
            break;
        case OPT_STUCK:
            if (!parse_ports(optarg, &cfg.stuck_mask)) return false;
            break;
        case OPT_SILENT:
            if (!parse_ports(optarg, &cfg.silent_mask)) return false;
            break;
        case OPT_INJECT_AT:
            if (!parse_u32("--inject-at-ms", optarg, 0xFFFFFFFFu, &v)) return false;
            cfg.inject_at_ms = v;
            break;
        default:
            usage(stderr);
            return false;
        }
    }
    if (optind != argc) {
        usage(stderr);
        return false;
    }
    if (cfg.deterministic && !cfg.have_boot_id) {
        cfg.boot_id = DEFAULT_BOOT_ID;
        cfg.have_boot_id = true;
    }
    return true;
}

/* ---- Main --------------------------------------------------------------- */

int main(int argc, char **argv) {
    if (!parse_args(argc, argv)) {
        return 2;
    }
    setvbuf(stdout, NULL, _IOLBF, 0);

    struct sigaction sa;
    memset(&sa, 0, sizeof sa);
    sa.sa_handler = on_signal;
    sigaction(SIGINT, &sa, NULL);
    sigaction(SIGTERM, &sa, NULL);

    if (cfg.csv_path != NULL) {
        csv = fopen(cfg.csv_path, "w");
        if (csv == NULL) {
            perror(cfg.csv_path);
            return 1;
        }
        fprintf(csv, "%s\n", CSV_HEADER);
    }

    /* Simulated actuators start at rest at --pa12-initial-mm, holding it. */
    for (unsigned i = 0; i < FCEC_PA12_PORTS; i++) {
        pa12_prof[i] = sil_pa12_profile[i];
        if (cfg.have_safe_action) {
            pa12_prof[i].safe_action = cfg.safe_action;
        }
        if (cfg.no_tvc[i]) {
            pa12_prof[i].approved = false;
        }
        sim_pa12[i].pos_counts = pa12_mm_to_counts_f(i, cfg.initial_mm);
        sim_pa12[i].goal_counts = sim_pa12[i].pos_counts;
        sim_pa12[i].torque_on = true;
    }

    uint32_t boot_id = cfg.have_boot_id ? cfg.boot_id : random_boot_id();
    uint64_t start_us = monotonic_us();
    uint64_t ec_now_us = 0;
    hw_now_us = 0;
    if (!app_init(boot_id, ec_now_us)) {
        fprintf(stderr, "sil_ec: EC_App_Init failed (port %u in use?)\n", (unsigned)cfg.port);
        return 1;
    }
    shown_state = ec.state;
    shown_session = ec.session_valid;
    shown_ready = ec.ready_mask;
    shown_fault = ec.fault_mask;

    event_prefix(ec_now_us, "boot");
    printf(" sim=sil_ec boot_id=0x%08X port=%u fc_port=%u capabilities=0x%02X profile=%s "
           "firmware_version=0x%08X mode=%s speed_mm_s=%.2f safe_action=%d esc_arm_hold_ms=%u\n",
           (unsigned)ec.boot_id, (unsigned)cfg.port, (unsigned)cfg.fc_port,
           (unsigned)ec.capabilities, SIL_PROFILE_ID, (unsigned)ec.firmware_version,
           cfg.deterministic ? "deterministic" : "realtime", (double)cfg.speed_mm_s,
           (int)pa12_prof[0].safe_action, (unsigned)esc.config.arm_hold_ms);
    event_prefix(ec_now_us, "state");
    printf(" from=BOOT_SAFE to=%s reason=NONE\n", fcec_state_str(ec.state));
    shown_state = ec.state;
    event_prefix(ec_now_us, "listening");
    printf(" port=%u\n", (unsigned)cfg.port);

    uint64_t next_print_us = 0;
    uint64_t last_hw_us = 0;
    uint64_t det_tick = 0;
    struct pollfd pfd = {udp_fd, POLLIN, 0};

    while (running) {
        if (cfg.deterministic) {
            det_tick++;
            hw_now_us = det_tick * DETERMINISTIC_TICK_US;
            sleep_until_us(start_us, hw_now_us);
        } else {
            poll(&pfd, 1, 1);
            hw_now_us = monotonic_us() - start_us;
        }
        ec_now_us = hw_now_us / 1000u * 1000u; /* HAL_GetTick() * 1000 */

        update_injection(ec_now_us);
        sim_esc_advance();

        udp_poll(ec_now_us, on_ec_udp_message);
        app_process(ec_now_us);
        note_changes(ec_now_us);

        float dt_s = (float)(hw_now_us - last_hw_us) / 1e6f;
        last_hw_us = hw_now_us;
        sim_pa12_physics(dt_s);

        if (ec_now_us >= next_print_us) {
            emit_tick(ec_now_us);
            do {
                next_print_us += (uint64_t)cfg.period_ms * 1000u;
            } while (next_print_us <= ec_now_us);
        }
        if (cfg.exit_after_ms != 0u && ec_now_us >= (uint64_t)cfg.exit_after_ms * 1000u) {
            break;
        }
    }

    event_prefix(ec_now_us, "exit");
    printf(" rx_accepted=%u rx_wrong_sender=%u rx_invalid_size=%u rx_dropped_fault=%u "
           "replies=%u statuses=%u tx_failed=%u pa12_goals=%u,%u pa12_reads=%u,%u "
           "esc_frames=%llu\n",
           (unsigned)udp_stats.rx_accepted, (unsigned)udp_stats.rx_wrong_sender,
           (unsigned)udp_stats.rx_invalid_size, (unsigned)udp_stats.rx_dropped_fault,
           (unsigned)udp_stats.replies_sent, (unsigned)udp_stats.statuses_sent,
           (unsigned)udp_stats.tx_failed, sim_pa12[0].goals_received, sim_pa12[1].goals_received,
           sim_pa12[0].reads_answered, sim_pa12[1].reads_answered,
           (unsigned long long)sim_esc.frames);
    if (csv != NULL) {
        fclose(csv);
    }
    close(udp_fd);
    return 0;
}
