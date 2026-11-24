/*
 * End-to-end test of sil_ec (the software-in-the-loop EC): starts the
 * program, plays the FC's side of the link over UDP with the real wire code
 * (fc_ec_link.c), and checks both what the EC answers and what its simulated
 * hardware outputs (parsed from the TICK/EVENT lines on its stdout).
 *
 * Run with `make test` in this folder, or
 *     make sil_ec_san test_sil_ec && ./test_sil_ec ./sil_ec_san
 * (the first argument is the sil_ec binary).
 *
 * Several short scenarios, each with its own sil_ec process:
 *   1. Hello -> Arm -> demands -> outputs track demands with the simulated
 *      lag -> demands stop -> SAFING in the demand timeout -> TVC hold, ESC
 *      at its minimum pulse -> ClearSafing -> Arm -> Terminate latches ->
 *      link timeout -> ClearSafing needs a fresh Hello.
 *   2. SAFE_PARK safing action and a stuck actuator.
 *   3. A silent actuator loses readiness while armed -> LOCAL_FAULT.
 *   4. --drop-after-ms/--drop-for-ms: the EC goes deaf and mute.
 *   5. Informational: what flight2's 200 Hz demands do to this EC.
 *   6. --deterministic output is identical from run to run; --csv works.
 */

#define _POSIX_C_SOURCE 200809L

#include "fc_ec_link.h"

#include <arpa/inet.h>
#include <errno.h>
#include <fcntl.h>
#include <math.h>
#include <netinet/in.h>
#include <poll.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/wait.h>
#include <time.h>
#include <unistd.h>

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

#define NOT_READY_SHIFT 8u /* FCEC_DRIVER_NOT_READY_SHIFT in ec_side.h */

static const char *sil_exe = "./sil_ec";
static unsigned base_port;

/* ---- Time --------------------------------------------------------------- */

static uint64_t mono_us(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000u + (uint64_t)ts.tv_nsec / 1000u;
}

static void sleep_us(uint64_t us) {
    struct timespec ts;
    ts.tv_sec = (time_t)(us / 1000000u);
    ts.tv_nsec = (long)(us % 1000000u) * 1000L;
    nanosleep(&ts, NULL);
}

/* ---- The sil_ec process and its stdout ---------------------------------- */

#define MAX_TICKS 20000u
#define MAX_EVENTS 2000u

typedef struct {
    uint64_t wall_us; /* when this test read the line */
    double t_ms;      /* EC time */
    char state[16];
    char reason[32];
    double a1, a1_goal, a1_meas, a2, a2_goal, a2_meas, thr;
    unsigned esc_us, esc_load_us, dem_seq, app_seq, drive, ready, fault;
} tick_t;

typedef struct {
    uint64_t wall_us;
    char line[320];
} event_t;

typedef struct {
    pid_t pid;
    int fd;
    char buf[16384];
    size_t len;
    bool eof;
    tick_t *ticks;
    size_t nticks;
    event_t *events;
    size_t nevents;
    unsigned ec_port;
    uint64_t spawn_us;
} sil_t;

/* Finds " key=" in `line` and copies its value (up to a space) to `out`. */
static bool kv_get(const char *line, const char *key, char *out, size_t cap) {
    char pat[48];
    snprintf(pat, sizeof pat, " %s=", key);
    const char *p = strstr(line, pat);
    if (p == NULL) {
        return false;
    }
    p += strlen(pat);
    size_t n = strcspn(p, " \n");
    if (n >= cap) {
        n = cap - 1u;
    }
    memcpy(out, p, n);
    out[n] = '\0';
    return true;
}

static double kv_double(const char *line, const char *key) {
    char v[64];
    return kv_get(line, key, v, sizeof v) ? strtod(v, NULL) : (double)NAN;
}

static unsigned kv_uint(const char *line, const char *key) {
    char v[64];
    return kv_get(line, key, v, sizeof v) ? (unsigned)strtoul(v, NULL, 0) : 0xFFFFFFFFu;
}

static void parse_line(sil_t *s, const char *line) {
    uint64_t now = mono_us();
    if (strncmp(line, "TICK ", 5) == 0 && s->nticks < MAX_TICKS) {
        tick_t *t = &s->ticks[s->nticks++];
        memset(t, 0, sizeof *t);
        t->wall_us = now;
        t->t_ms = kv_double(line, "t_ms");
        kv_get(line, "state", t->state, sizeof t->state);
        kv_get(line, "reason", t->reason, sizeof t->reason);
        t->a1 = kv_double(line, "a1_mm");
        t->a1_goal = kv_double(line, "a1_goal_mm");
        t->a1_meas = kv_double(line, "a1_meas_mm");
        t->a2 = kv_double(line, "a2_mm");
        t->a2_goal = kv_double(line, "a2_goal_mm");
        t->a2_meas = kv_double(line, "a2_meas_mm");
        t->thr = kv_double(line, "thr");
        t->esc_us = kv_uint(line, "esc_us");
        t->esc_load_us = kv_uint(line, "esc_load_us");
        t->dem_seq = kv_uint(line, "dem_seq");
        t->app_seq = kv_uint(line, "app_seq");
        t->drive = kv_uint(line, "drive");
        t->ready = kv_uint(line, "ready");
        t->fault = kv_uint(line, "fault");
    } else if (strncmp(line, "EVENT ", 6) == 0 && s->nevents < MAX_EVENTS) {
        event_t *e = &s->events[s->nevents++];
        e->wall_us = now;
        snprintf(e->line, sizeof e->line, " %s", line + 5); /* leading space for kv_get */
    }
}

/* Reads whatever sil_ec has printed. */
static void sil_pump(sil_t *s) {
    while (!s->eof) {
        if (s->len >= sizeof s->buf - 1u) {
            s->len = 0; /* a line longer than the buffer: drop it */
        }
        ssize_t n = read(s->fd, s->buf + s->len, sizeof s->buf - 1u - s->len);
        if (n > 0) {
            s->len += (size_t)n;
        } else if (n == 0) {
            s->eof = true;
        } else {
            break; /* EAGAIN */
        }
        char *start = s->buf;
        char *nl;
        while ((nl = memchr(start, '\n', s->len - (size_t)(start - s->buf))) != NULL) {
            *nl = '\0';
            parse_line(s, start);
            start = nl + 1;
        }
        size_t rest = s->len - (size_t)(start - s->buf);
        memmove(s->buf, start, rest);
        s->len = rest;
    }
}

static unsigned next_instance;

/* Starts sil_ec with `extra` flags (NULL-terminated). Uses an ephemeral-looking
 * port block so parallel runs don't collide. */
static bool sil_start(sil_t *s, unsigned fc_port, const char *const *extra) {
    memset(s, 0, sizeof *s);
    s->ticks = calloc(MAX_TICKS, sizeof *s->ticks);
    s->events = calloc(MAX_EVENTS, sizeof *s->events);
    s->ec_port = base_port + next_instance++;
    if (s->ticks == NULL || s->events == NULL) {
        return false;
    }

    char port[16], fcp[16];
    snprintf(port, sizeof port, "%u", s->ec_port);
    snprintf(fcp, sizeof fcp, "%u", fc_port);
    const char *argv[64];
    size_t n = 0;
    argv[n++] = sil_exe;
    argv[n++] = "-p";
    argv[n++] = port;
    argv[n++] = "--fc-port";
    argv[n++] = fcp;
    for (size_t i = 0; extra != NULL && extra[i] != NULL && n < 60; i++) {
        argv[n++] = extra[i];
    }
    argv[n] = NULL;

    int fds[2];
    if (pipe(fds) != 0) {
        return false;
    }
    s->spawn_us = mono_us();
    s->pid = fork();
    if (s->pid < 0) {
        return false;
    }
    if (s->pid == 0) {
        dup2(fds[1], STDOUT_FILENO);
        close(fds[0]);
        close(fds[1]);
        execv(sil_exe, (char *const *)argv);
        perror(sil_exe);
        _exit(127);
    }
    close(fds[1]);
    s->fd = fds[0];
    fcntl(s->fd, F_SETFL, fcntl(s->fd, F_GETFL, 0) | O_NONBLOCK);
    return true;
}

/* Stops the process; returns its exit status (0 = clean). */
static int sil_stop(sil_t *s) {
    int status = -1;
    if (s->pid > 0) {
        kill(s->pid, SIGTERM);
        for (int i = 0; i < 500; i++) {
            sil_pump(s);
            if (waitpid(s->pid, &status, WNOHANG) == s->pid) {
                s->pid = 0;
                break;
            }
            sleep_us(10000);
        }
        if (s->pid > 0) {
            kill(s->pid, SIGKILL);
            waitpid(s->pid, &status, 0);
            s->pid = 0;
            status = -1;
        } else {
            status = WIFEXITED(status) ? WEXITSTATUS(status) : -2;
        }
    }
    sil_pump(s);
    close(s->fd);
    return status;
}

static void sil_free(sil_t *s) {
    free(s->ticks);
    free(s->events);
}

/* The index of the first event at or after `from` containing both patterns,
 * or -1. */
static long ev_find(const sil_t *s, size_t from, const char *p1, const char *p2) {
    for (size_t i = from; i < s->nevents; i++) {
        if (strstr(s->events[i].line, p1) != NULL &&
            (p2 == NULL || strstr(s->events[i].line, p2) != NULL)) {
            return (long)i;
        }
    }
    return -1;
}

static const tick_t *last_tick(const sil_t *s) {
    return s->nticks > 0 ? &s->ticks[s->nticks - 1u] : NULL;
}

/* First tick read at or after wall time `wall_us`. */
static const tick_t *tick_after(const sil_t *s, uint64_t wall_us) {
    for (size_t i = 0; i < s->nticks; i++) {
        if (s->ticks[i].wall_us >= wall_us) {
            return &s->ticks[i];
        }
    }
    return last_tick(s);
}

/* ---- The FC side of the link -------------------------------------------- */

typedef struct {
    bool valid;
    uint32_t txn;
    fcec_txn_result_t r;
} result_slot_t;

typedef struct {
    int fd;
    struct sockaddr_in ec;
    uint16_t local_port;
    uint32_t session, seq, txn, ec_boot;
    bool have_boot;
    uint64_t ec_time_ref;
    bool have_status;
    fcec_status_t status;
    uint64_t status_wall_us;
    unsigned statuses;
    result_slot_t results[8];
    unsigned next_result;
    bool have_hello;
    fcec_hello_result_t hello;
    uint64_t last_hb_us;
} fc_t;

static sil_t *cur_sil;
static fc_t *cur_fc;
static bool send_heartbeats;

static bool fc_open(fc_t *f, unsigned ec_port) {
    memset(f, 0, sizeof *f);
    f->fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (f->fd < 0) {
        return false;
    }
    struct sockaddr_in local;
    memset(&local, 0, sizeof local);
    local.sin_family = AF_INET;
    local.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    if (bind(f->fd, (struct sockaddr *)&local, sizeof local) != 0) {
        return false;
    }
    socklen_t len = sizeof local;
    getsockname(f->fd, (struct sockaddr *)&local, &len);
    f->local_port = ntohs(local.sin_port);
    fcntl(f->fd, F_SETFL, fcntl(f->fd, F_GETFL, 0) | O_NONBLOCK);
    memset(&f->ec, 0, sizeof f->ec);
    f->ec.sin_family = AF_INET;
    f->ec.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    f->ec.sin_port = htons((uint16_t)ec_port);
    f->session = 0x5E550000u + ec_port;
    return true;
}

static void fc_poll(fc_t *f) {
    for (;;) {
        uint8_t buf[FCEC_MAX_DATAGRAM_LEN + 1];
        ssize_t n = recv(f->fd, buf, sizeof buf, 0);
        if (n < 0) {
            return;
        }
        fcec_packet_t p;
        if (fcec_decode(buf, (size_t)n, &p) != FCEC_OK ||
            p.header.sent_time_us < f->ec_time_ref) {
            continue;
        }
        f->ec_time_ref = p.header.sent_time_us;
        switch (p.kind) {
        case FCEC_KIND_HELLO_RESULT:
            f->hello = p.msg.hello_result;
            f->have_hello = true;
            if (p.msg.hello_result.outcome == FCEC_OUTCOME_ACCEPTED) {
                f->ec_boot = p.header.ec_boot_id;
                f->have_boot = true;
            }
            break;
        case FCEC_KIND_TXN_RESULT: {
            result_slot_t *r = &f->results[f->next_result++ % 8u];
            r->valid = true;
            r->txn = p.header.txn_id;
            r->r = p.msg.txn_result;
            break;
        }
        case FCEC_KIND_STATUS:
            f->status = p.msg.status;
            f->have_status = true;
            f->status_wall_us = mono_us();
            f->statuses++;
            break;
        default:
            break;
        }
    }
}

static void fc_send(fc_t *f, fcec_kind_t kind, uint32_t txn, const fcec_packet_t *body) {
    fcec_packet_t p;
    memset(&p, 0, sizeof p);
    if (body != NULL) {
        p = *body;
    }
    p.kind = kind;
    p.header.ec_boot_id = f->have_boot ? f->ec_boot : 0u;
    p.header.session_id = f->session;
    p.header.seq = ++f->seq;
    p.header.txn_id = txn;
    p.header.valid_for_ms = 50;
    p.header.sent_time_us = mono_us();
    p.header.ec_time_ref_us = f->ec_time_ref;
    uint8_t buf[FCEC_MAX_DATAGRAM_LEN];
    int len = fcec_encode(&p, buf, sizeof buf);
    CHECK(len > 0);
    if (len > 0) {
        sendto(f->fd, buf, (size_t)len, 0, (struct sockaddr *)&f->ec, sizeof f->ec);
    }
}

/* Lets `us` microseconds pass, reading sil_ec's stdout and the FC socket, and
 * sending heartbeats at 20 Hz while send_heartbeats is set. */
static void step_us(uint64_t us) {
    uint64_t end = mono_us() + us;
    for (;;) {
        sil_pump(cur_sil);
        fc_poll(cur_fc);
        uint64_t now = mono_us();
        if (send_heartbeats && now - cur_fc->last_hb_us >= 50000u) {
            fc_send(cur_fc, FCEC_KIND_HEARTBEAT, 0, NULL);
            cur_fc->last_hb_us = now;
        }
        if (now >= end) {
            return;
        }
        uint64_t left = end - now;
        sleep_us(left < 1000u ? left : 1000u);
    }
}

static void step_ms(unsigned ms) {
    step_us((uint64_t)ms * 1000u);
}

static const fcec_ec_state_t UNKNOWN_STATE = (fcec_ec_state_t)99;

typedef struct {
    bool answered;
    fcec_outcome_t outcome;
    fcec_reason_t reason;
    fcec_ec_state_t state;
} answer_t;

static const uint8_t *profile_id(void) {
    static uint8_t id[32];
    static const char name[] = "ctv-bench-profile-v1"; /* what flight2 sends */
    memset(id, 0, sizeof id);
    memcpy(id, name, sizeof name - 1u);
    return id;
}

/* Sends a transaction (retrying the same txn_id every 40 ms, as the FC does)
 * until it is answered or `timeout_ms` passes. */
static answer_t fc_txn(fcec_kind_t kind, const uint8_t *bundle, unsigned timeout_ms) {
    fc_t *f = cur_fc;
    answer_t a = {false, FCEC_OUTCOME_REJECTED, FCEC_REASON_NONE, UNKNOWN_STATE};
    uint32_t txn = ++f->txn;
    uint64_t end = mono_us() + (uint64_t)timeout_ms * 1000u;
    fcec_packet_t body;
    memset(&body, 0, sizeof body);
    if (kind == FCEC_KIND_ARM) {
        body.msg.arm.run_id = 1;
        memcpy(body.msg.arm.expected_bundle_sha256, bundle, 32);
    } else if (kind == FCEC_KIND_TERMINATE) {
        body.msg.terminate.reason = FCEC_REASON_FC_TERMINATE;
    }
    while (mono_us() < end) {
        fc_send(f, kind, txn, &body);
        uint64_t retry = mono_us() + 40000u;
        while (mono_us() < retry) {
            step_ms(2);
            for (unsigned i = 0; i < 8u; i++) {
                if (f->results[i].valid && f->results[i].txn == txn) {
                    a.answered = true;
                    a.outcome = f->results[i].r.outcome;
                    a.reason = f->results[i].r.reason;
                    a.state = f->results[i].r.ec_state;
                    return a;
                }
            }
        }
    }
    return a;
}

static answer_t fc_hello(void) {
    fc_t *f = cur_fc;
    answer_t a = {false, FCEC_OUTCOME_REJECTED, FCEC_REASON_NONE, UNKNOWN_STATE};
    uint32_t txn = ++f->txn;
    fcec_packet_t body;
    memset(&body, 0, sizeof body);
    body.msg.hello.fc_boot_id = 0xFC000001u;
    body.msg.hello.run_id = 1;
    body.msg.hello.demand_period_ms = 10; /* 30 ms demand timeout */
    body.msg.hello.heartbeat_period_ms = 50;
    memcpy(body.msg.hello.expected_bundle_sha256, profile_id(), 32);
    for (int tries = 0; tries < 20 && !a.answered; tries++) {
        f->have_hello = false;
        fc_send(f, FCEC_KIND_HELLO, txn, &body);
        for (int i = 0; i < 20 && !f->have_hello; i++) {
            step_ms(2);
        }
        if (f->have_hello) {
            a.answered = true;
            a.outcome = f->hello.outcome;
            a.reason = f->hello.reason;
            a.state = f->hello.ec_state;
        }
    }
    return a;
}

/* The demand the Rust FC sends: both TVC actuators and the main ESC. */
static void fc_demand(double a1, double a2, double thr) {
    fcec_packet_t body;
    memset(&body, 0, sizeof body);
    body.msg.demand.outputs.tvc_extension_mm[0] = (float)a1;
    body.msg.demand.outputs.tvc_extension_mm[1] = (float)a2;
    body.msg.demand.outputs.main_throttle = (float)thr;
    body.msg.demand.enable_mask = FCEC_CH_TVC_A1 | FCEC_CH_TVC_A2 | FCEC_CH_MAIN_ESC;
    fc_send(cur_fc, FCEC_KIND_DEMAND, 0, &body);
}

/* Demand send period. The spec's 100 Hz by default; flight2 actually sends
 * every 5 ms (see test_200hz_demands for what that does to this EC). */
static unsigned demand_period_ms = 10;

/* Demands every demand_period_ms for `ms`. */
static void run_demands(unsigned ms, double a1, double a2, double thr) {
    uint64_t t = mono_us();
    uint64_t end = t + (uint64_t)ms * 1000u;
    while (t < end) {
        fc_demand(a1, a2, thr);
        t += (uint64_t)demand_period_ms * 1000u;
        uint64_t now = mono_us();
        if (t > now) {
            step_us(t - now);
        } else {
            step_us(0);
        }
    }
}

/* Waits for an event; returns its index or -1. */
static long wait_event(size_t from, const char *p1, const char *p2, unsigned timeout_ms) {
    uint64_t end = mono_us() + (uint64_t)timeout_ms * 1000u;
    do {
        long i = ev_find(cur_sil, from, p1, p2);
        if (i >= 0) {
            return i;
        }
        step_ms(2);
    } while (mono_us() < end);
    return ev_find(cur_sil, from, p1, p2);
}

static bool wait_ready(unsigned timeout_ms) {
    uint64_t end = mono_us() + (uint64_t)timeout_ms * 1000u;
    while (mono_us() < end) {
        step_ms(5);
        if (cur_fc->have_status && cur_fc->status.driver_faults == 0u &&
            cur_fc->status.ec_state == FCEC_STATE_DISARMED) {
            return true;
        }
    }
    return false;
}

/* Starts a sil_ec and an FC socket, waits until it listens, and says Hello. */
static bool scenario_start(sil_t *s, fc_t *f, const char *const *extra, bool hello) {
    /* The FC socket first: sil_ec only accepts datagrams from its source port. */
    unsigned port = base_port + next_instance;
    if (!fc_open(f, port) || !sil_start(s, f->local_port, extra)) {
        printf("could not start sil_ec (%s)\n", sil_exe);
        sil_free(s);
        return false;
    }
    cur_sil = s;
    cur_fc = f;
    send_heartbeats = false;
    if (wait_event(0, "kind=listening", NULL, 5000) < 0) {
        printf("sil_ec did not start listening\n");
        (void)sil_stop(s);
        close(f->fd);
        sil_free(s);
        return false;
    }
    if (hello) {
        answer_t a = fc_hello();
        CHECK(a.answered && a.outcome == FCEC_OUTCOME_ACCEPTED);
        send_heartbeats = true;
    }
    return true;
}

/* Stops sil_ec, checks it exited cleanly with its summary line, and returns
 * that line's rx_wrong_sender count. */
static unsigned scenario_end(sil_t *s, fc_t *f) {
    send_heartbeats = false;
    int status = sil_stop(s);
    close(f->fd);
    CHECK(status == 0);
    if (getenv("SIL_TEST_VERBOSE") != NULL) { /* what sil_ec printed, for debugging */
        for (size_t i = 0; i < s->nevents; i++) {
            printf("  [event] %s\n", s->events[i].line);
        }
    }
    long exit_ev = ev_find(s, 0, "kind=exit", NULL);
    CHECK(exit_ev >= 0);
    unsigned wrong_sender = exit_ev >= 0 ? kv_uint(s->events[exit_ev].line, "rx_wrong_sender") : 0u;
    sil_free(s);
    return wrong_sender;
}

static bool near(double a, double b, double tol) {
    return isfinite(a) && fabs(a - b) <= tol;
}

static unsigned event_t_ms(const sil_t *s, long idx) {
    return idx >= 0 ? kv_uint(s->events[idx].line, "t_ms") : 0xFFFFFFFFu;
}

/* ---- Scenario 1: the full FC session ------------------------------------ */

static void test_full_session(void) {
    printf("scenario 1: Hello/Arm/demands/safing/Terminate/ClearSafing\n");
    static const char *const args[] = {"--period-ms", "5", "--speed", "40", "--esc-arm-hold-ms",
                                       "800", NULL};
    sil_t s;
    fc_t f;
    if (!scenario_start(&s, &f, args, false)) {
        failures++;
        return;
    }

    /* Boot: capabilities offered only for configured drivers, profile ID. */
    long boot = ev_find(&s, 0, "kind=boot", NULL);
    CHECK(boot >= 0);
    CHECK(kv_uint(s.events[boot].line, "capabilities") ==
          (FCEC_CH_TVC_A1 | FCEC_CH_TVC_A2 | FCEC_CH_MAIN_ESC));
    CHECK(strstr(s.events[boot].line, "profile=ctv-bench-profile-v1") != NULL);

    answer_t a = fc_hello();
    CHECK(a.answered && a.outcome == FCEC_OUTCOME_ACCEPTED && a.state == FCEC_STATE_DISARMED);
    send_heartbeats = true;
    CHECK(f.have_hello);
    CHECK(f.hello.capabilities == (FCEC_CH_TVC_A1 | FCEC_CH_TVC_A2 | FCEC_CH_MAIN_ESC));
    CHECK(f.hello.link_timeout_ms == 350u);
    CHECK(f.hello.max_demand_age_ms == 50u);
    CHECK(memcmp(f.hello.active_bundle_sha256, profile_id(), 32) == 0);
    CHECK(f.hello.firmware_version == 0x53494C01u);
    CHECK(f.have_boot && f.ec_boot == kv_uint(s.events[boot].line, "boot_id"));

    /* The profile ID is checked at Arm. */
    uint8_t zeros[32];
    memset(zeros, 0, sizeof zeros);
    a = fc_txn(FCEC_KIND_ARM, zeros, 2000);
    CHECK(a.answered && a.outcome == FCEC_OUTCOME_REJECTED &&
          a.reason == FCEC_REASON_CONFIG_MISMATCH);

    /* The ESC is held at its safe pulse for the arm hold after boot, so a
     * correct Arm this early is refused: nothing is ready until the drivers
     * say so. (Skipped if this machine was too slow to get here in time.) */
    if (mono_us() - s.spawn_us < 500000u) {
        a = fc_txn(FCEC_KIND_ARM, profile_id(), 2000);
        CHECK(a.answered && a.outcome == FCEC_OUTCOME_REJECTED &&
              a.reason == FCEC_REASON_NOT_READY);
    } else {
        printf("  (too slow to test Arm before the ESC arm hold ends)\n");
    }

    CHECK(wait_ready(4000));
    CHECK(f.status.measurements.pa12_feedback_state[0] == FCEC_PA12_FEEDBACK_VALID_REPLY ||
          f.status.measurements.pa12_feedback_state[0] == FCEC_PA12_FEEDBACK_PENDING);
    CHECK((f.status.measurements.validity_mask & 3u) == 3u); /* both positions valid */

    /* DISARMED: demands are stored, not applied. */
    size_t before = s.nticks;
    uint64_t mark = mono_us();
    run_demands(150, 20.0, 8.0, 0.5);
    step_ms(20);
    const tick_t *t = last_tick(&s);
    CHECK(t != NULL && strcmp(t->state, "DISARMED") == 0);
    CHECK(t != NULL && t->dem_seq > 0u);
    for (size_t i = before; i < s.nticks; i++) {
        CHECK(near(s.ticks[i].a1, 13.45, 0.02) && near(s.ticks[i].a2, 13.45, 0.02));
        CHECK(s.ticks[i].esc_us == 1000u);
        CHECK(s.ticks[i].wall_us >= mark);
    }

    /* Arm, and demands straight away: the demand timer starts at Arm. */
    a = fc_txn(FCEC_KIND_ARM, profile_id(), 2000);
    CHECK(a.answered && a.outcome == FCEC_OUTCOME_ACCEPTED && a.state == FCEC_STATE_ARMED);

    /* Outputs track the demands with the simulated actuator lag (40 mm/s). */
    uint64_t demands_start = mono_us();
    run_demands(500, 20.0, 8.0, 0.5);
    const tick_t *early = tick_after(&s, demands_start + 60000u);
    CHECK(early != NULL && strcmp(early->state, "ARMED") == 0);
    CHECK(early != NULL && early->a1 > 13.6 && early->a1 < 18.0); /* moving, not there yet */
    CHECK(early != NULL && early->a2 < 13.3 && early->a2 > 9.0);
    CHECK(early != NULL && near(early->a1_goal, 20.0, 0.02)); /* but the goal is */
    step_ms(5); /* (not longer: the demand timeout is 30 ms) */
    t = last_tick(&s);
    CHECK(t != NULL && near(t->a1, 20.0, 0.1) && near(t->a2, 8.0, 0.1));
    CHECK(t != NULL && near(t->a1_goal, 20.0, 0.02) && near(t->a2_goal, 8.0, 0.02));
    CHECK(t != NULL && near(t->a1_meas, 20.0, 0.1)); /* what the FC is told */
    CHECK(t != NULL && t->esc_us >= 1499u && t->esc_us <= 1501u && near(t->thr, 0.5, 0.001));
    CHECK(t != NULL && t->drive == (FCEC_CH_TVC_A1 | FCEC_CH_TVC_A2 | FCEC_CH_MAIN_ESC));
    {
        /* No jumps: it never moves faster than the actuator speed. */
        bool monotonic = true, slew_ok = true;
        const tick_t *prev = NULL;
        for (size_t i = 0; i < s.nticks; i++) {
            const tick_t *c = &s.ticks[i];
            if (c->wall_us < demands_start) {
                continue;
            }
            if (prev != NULL) {
                if (c->a1 < prev->a1 - 0.001) {
                    monotonic = false;
                }
                if (c->a1 - prev->a1 > 40.0 * (c->t_ms - prev->t_ms) / 1000.0 + 0.1) {
                    slew_ok = false;
                }
            }
            prev = c;
        }
        CHECK(monotonic);
        CHECK(slew_ok);
    }
    CHECK(f.have_status && f.status.ec_state == FCEC_STATE_ARMED && f.status.output_authority);
    CHECK(near((double)f.status.measurements.tvc_position_mm[0], 20.0, 0.1));
    CHECK(f.status.last_accepted_demand_seq != 0u);
    /* Not asserted: with the profile's 20 ms ESC frame, the ESC driver only
     * confirms a command that stays unchanged for a whole frame, and every
     * accepted demand is a new command, so "applied" lags behind at 100 Hz. */
    {
        unsigned streaming = 0, caught_up = 0;
        for (size_t i = 0; i < s.nticks; i++) {
            const tick_t *c = &s.ticks[i];
            if (c->wall_us >= demands_start + 100000u && c->wall_us < demands_start + 480000u) {
                streaming++;
                caught_up += c->app_seq == c->dem_seq ? 1u : 0u;
            }
        }
        printf("  info: while 100 Hz demands stream, last_applied_demand_seq == last_accepted in "
               "%u of %u ticks\n", caught_up, streaming);
    }

    /* Step the demand mid-flight, then stop sending demands. The TVC is still
     * moving when the EC safes (about 4 mm of the 6 mm to go at 40 mm/s). */
    run_demands(60, 26.0, 1.0, 0.8);
    uint64_t last_demand_us = mono_us();
    size_t ev_from = s.nevents;
    long safing = wait_event(ev_from, "kind=state", "to=SAFING", 1000);
    uint64_t seen_us = mono_us();
    CHECK(safing >= 0);
    if (safing >= 0) {
        const char *line = s.events[safing].line;
        char reason[40];
        CHECK(kv_get(line, "reason", reason, sizeof reason) &&
              strcmp(reason, "DEMAND_TIMEOUT") == 0);
        unsigned since = kv_uint(line, "since_demand_ms");
        printf("  info: demand timeout after %u ms (EC time), seen %llu ms after the last demand\n",
               since, (unsigned long long)((seen_us - last_demand_us) / 1000u));
        CHECK(since >= 30u && since <= 50u); /* 3 declared periods of 10 ms */
        CHECK(seen_us - last_demand_us < 100000u);
    }

    /* Safe outputs: throttle 0 and the ESC's minimum pulse on the wire
     * (within one 20 ms frame), TVC holds where it was. */
    step_ms(80);
    t = last_tick(&s);
    CHECK(t != NULL && strcmp(t->state, "SAFING") == 0 && strcmp(t->reason, "DEMAND_TIMEOUT") == 0);
    CHECK(t != NULL && t->esc_us == 1000u && t->esc_load_us == 1000u && near(t->thr, 0.0, 0.0001));
    CHECK(t != NULL && t->a1 > 21.0 && t->a1 < 25.9); /* stopped part way to 26 */
    double hold_a1 = t != NULL ? t->a1 : 0.0;
    double hold_a2 = t != NULL ? t->a2 : 0.0;
    CHECK(t != NULL && near(t->a1_goal, t->a1, 0.05)); /* hold: goal = where it is */
    CHECK(t != NULL && t->a2 > 1.2 && t->a2 < 7.0);
    CHECK(t != NULL && near(t->a2_goal, t->a2, 0.05));
    step_ms(300);
    t = last_tick(&s);
    CHECK(t != NULL && near(t->a1, hold_a1, 0.02) && near(t->a2, hold_a2, 0.02));
    CHECK(t != NULL && t->esc_us == 1000u);
    CHECK(f.status.ec_state == FCEC_STATE_SAFING && f.status.safing_latched);
    CHECK(f.status.safing_reason == FCEC_REASON_DEMAND_TIMEOUT);

    /* Safing is latched: Arm is refused until ClearSafing. */
    a = fc_txn(FCEC_KIND_ARM, profile_id(), 2000);
    CHECK(a.answered && a.outcome == FCEC_OUTCOME_REJECTED &&
          a.reason == FCEC_REASON_SAFING_LATCHED);
    a = fc_txn(FCEC_KIND_CLEAR_SAFING, NULL, 2000);
    CHECK(a.answered && a.outcome == FCEC_OUTCOME_ACCEPTED && a.state == FCEC_STATE_DISARMED);

    /* Arm again, drive back down, then Terminate. */
    a = fc_txn(FCEC_KIND_ARM, profile_id(), 2000);
    CHECK(a.answered && a.outcome == FCEC_OUTCOME_ACCEPTED && a.state == FCEC_STATE_ARMED);
    run_demands(200, 10.0, 10.0, 0.4);
    t = last_tick(&s);
    CHECK(t != NULL && strcmp(t->state, "ARMED") == 0 && t->esc_us >= 1399u && t->esc_us <= 1401u);
    ev_from = s.nevents;
    a = fc_txn(FCEC_KIND_TERMINATE, NULL, 2000);
    CHECK(a.answered && a.outcome == FCEC_OUTCOME_ACCEPTED && a.state == FCEC_STATE_SAFING);
    long term = wait_event(ev_from, "kind=state", "to=SAFING", 500);
    CHECK(term >= 0 && strstr(s.events[term > 0 ? term : 0].line, "reason=FC_TERMINATE") != NULL);

    /* Demands after Terminate are never applied; Arm stays refused. */
    step_ms(60);
    t = last_tick(&s);
    double frozen_a1 = t != NULL ? t->a1 : 0.0;
    CHECK(t != NULL && t->esc_us == 1000u && t->thr == 0.0);
    run_demands(150, 26.0, 0.0, 1.0);
    t = last_tick(&s);
    CHECK(t != NULL && strcmp(t->state, "SAFING") == 0 && strcmp(t->reason, "FC_TERMINATE") == 0);
    CHECK(t != NULL && near(t->a1, frozen_a1, 0.02) && t->esc_us == 1000u);
    a = fc_txn(FCEC_KIND_ARM, profile_id(), 2000);
    CHECK(a.answered && a.outcome == FCEC_OUTCOME_REJECTED &&
          a.reason == FCEC_REASON_SAFING_LATCHED);

    /* Silence: the link times out (350 ms), the session is dropped, and the
     * EC stays SAFING with its first reason. */
    send_heartbeats = false;
    ev_from = s.nevents;
    long closed = wait_event(ev_from, "kind=session", "state=closed", 1500);
    CHECK(closed >= 0);
    t = last_tick(&s);
    CHECK(t != NULL && strcmp(t->state, "SAFING") == 0 && strcmp(t->reason, "FC_TERMINATE") == 0);

    /* ClearSafing needs a session: refused until the FC says Hello again. */
    a = fc_txn(FCEC_KIND_CLEAR_SAFING, NULL, 2000);
    CHECK(a.answered && a.outcome == FCEC_OUTCOME_REJECTED && a.reason == FCEC_REASON_BAD_SESSION);
    a = fc_hello();
    CHECK(a.answered && a.outcome == FCEC_OUTCOME_ACCEPTED && a.state == FCEC_STATE_SAFING);
    send_heartbeats = true;
    step_ms(60); /* a Status or two, so the next message is fresh */
    a = fc_txn(FCEC_KIND_CLEAR_SAFING, NULL, 2000);
    CHECK(a.answered && a.outcome == FCEC_OUTCOME_ACCEPTED && a.state == FCEC_STATE_DISARMED);

    /* And it can fly again: Arm, demands move the TVC, Disarm makes it safe. */
    a = fc_txn(FCEC_KIND_ARM, profile_id(), 2000);
    CHECK(a.answered && a.outcome == FCEC_OUTCOME_ACCEPTED && a.state == FCEC_STATE_ARMED);
    run_demands(400, 5.0, 5.0, 0.25);
    t = last_tick(&s);
    CHECK(t != NULL && strcmp(t->state, "ARMED") == 0 && near(t->a1_goal, 5.0, 0.02));
    CHECK(t != NULL && t->esc_us >= 1249u && t->esc_us <= 1251u);
    a = fc_txn(FCEC_KIND_DISARM, NULL, 2000);
    CHECK(a.answered && a.outcome == FCEC_OUTCOME_ACCEPTED && a.state == FCEC_STATE_DISARMED);
    step_ms(80);
    t = last_tick(&s);
    CHECK(t != NULL && strcmp(t->state, "DISARMED") == 0 && t->esc_us == 1000u);

    /* Datagrams from another source port are ignored by the EC (no answer). */
    {
        int other = socket(AF_INET, SOCK_DGRAM, 0);
        fcec_packet_t p;
        memset(&p, 0, sizeof p);
        p.kind = FCEC_KIND_HELLO;
        p.header.session_id = 0x77u;
        p.header.seq = 1;
        p.header.txn_id = 1;
        uint8_t buf[FCEC_MAX_DATAGRAM_LEN];
        int len = fcec_encode(&p, buf, sizeof buf);
        CHECK(len > 0 && other >= 0);
        sendto(other, buf, (size_t)len, 0, (struct sockaddr *)&f.ec, sizeof f.ec);
        struct pollfd pfd = {other, POLLIN, 0};
        CHECK(poll(&pfd, 1, 100) == 0);
        close(other);
    }

    CHECK(scenario_end(&s, &f) == 1u);
}

/* ---- Scenario 2: park + stuck actuator ---------------------------------- */

static void test_park_and_stuck_actuator(void) {
    printf("scenario 2: SAFE_PARK and a stuck actuator\n");
    static const char *const args[] = {"--period-ms", "5", "--speed", "60", "--esc-arm-hold-ms",
                                       "200", "--safe-action", "park", "--pa12-stuck", "a2",
                                       NULL};
    sil_t s;
    fc_t f;
    if (!scenario_start(&s, &f, args, true)) {
        failures++;
        return;
    }
    CHECK(wait_ready(4000));
    answer_t a = fc_txn(FCEC_KIND_ARM, profile_id(), 2000);
    CHECK(a.answered && a.outcome == FCEC_OUTCOME_ACCEPTED);
    run_demands(400, 24.0, 3.0, 0.3);
    const tick_t *t = last_tick(&s);
    CHECK(t != NULL && near(t->a1, 24.0, 0.2) && near(t->a1_goal, 24.0, 0.02));
    /* A2 took the goal but never moved, and the EC's own reading says so. */
    CHECK(t != NULL && near(t->a2_goal, 3.0, 0.02));
    CHECK(t != NULL && near(t->a2, 13.45, 0.01));
    CHECK(t != NULL && near(t->a2_meas, 13.45, 0.01));

    long safing = wait_event(0, "kind=state", "to=SAFING", 1000);
    CHECK(safing >= 0 && strstr(s.events[safing > 0 ? safing : 0].line, "reason=DEMAND_TIMEOUT") != NULL);
    step_ms(600);
    t = last_tick(&s);
    /* PARK: both goals go to the park position; A1 gets there, A2 can't. */
    CHECK(t != NULL && strcmp(t->state, "SAFING") == 0);
    CHECK(t != NULL && near(t->a1_goal, 13.45, 0.02) && near(t->a2_goal, 13.45, 0.02));
    CHECK(t != NULL && near(t->a1, 13.45, 0.1));
    CHECK(t != NULL && t->esc_us == 1000u);
    printf("  info: stuck A2 stayed at %.2f mm (goal was 3.00), A1 parked at %.2f mm\n",
           t != NULL ? t->a2 : 0.0, t != NULL ? t->a1 : 0.0);

    scenario_end(&s, &f);
}

/* ---- Scenario 3: a silent actuator -------------------------------------- */

static void test_silent_actuator_loses_readiness(void) {
    printf("scenario 3: a silent actuator safes the armed EC (LOCAL_FAULT)\n");
    static const char *const args[] = {"--period-ms", "5", "--esc-arm-hold-ms", "200",
                                       "--pa12-silent", "a1", "--inject-at-ms", "1500", NULL};
    sil_t s;
    fc_t f;
    if (!scenario_start(&s, &f, args, true)) {
        failures++;
        return;
    }
    CHECK(wait_ready(4000));
    answer_t a = fc_txn(FCEC_KIND_ARM, profile_id(), 2000);
    CHECK(a.answered && a.outcome == FCEC_OUTCOME_ACCEPTED);
    /* Keep demanding throughout: the demand timeout never fires. */
    uint64_t end = mono_us() + 3000000u;
    long safing = -1;
    while (mono_us() < end && safing < 0) {
        run_demands(20, 15.0, 15.0, 0.3);
        safing = ev_find(&s, 0, "kind=state", "to=SAFING");
    }
    CHECK(safing >= 0);
    long inject = ev_find(&s, 0, "kind=inject", NULL);
    CHECK(inject >= 0);
    if (safing >= 0 && inject >= 0) {
        CHECK(strstr(s.events[safing].line, "reason=LOCAL_FAULT") != NULL);
        unsigned lag = event_t_ms(&s, safing) - event_t_ms(&s, inject);
        printf("  info: LOCAL_FAULT %u ms after the actuator went silent\n", lag);
        CHECK(lag >= 400u && lag <= 650u); /* the 500 ms readiness window */
    }
    step_ms(100);
    const tick_t *t = last_tick(&s);
    CHECK(t != NULL && strcmp(t->state, "SAFING") == 0 && t->esc_us == 1000u);
    CHECK(f.status.safing_reason == FCEC_REASON_LOCAL_FAULT);
    /* The Status says which driver isn't ready (bit 8 + channel). */
    CHECK((f.status.driver_faults & (1u << NOT_READY_SHIFT)) != 0u);
    CHECK(inject >= 0 && wait_event((size_t)inject, "kind=drivers", "ready_mask=0x42", 100) >= 0);

    scenario_end(&s, &f);
}

/* ---- Scenario 4: the EC stops answering ---------------------------------- */

static void test_drop_fault_injection(void) {
    printf("scenario 4: --drop-after-ms / --drop-for-ms\n");
    static const char *const args[] = {"--period-ms", "5", "--esc-arm-hold-ms", "200",
                                       "--drop-after-ms", "1200", "--drop-for-ms", "900", NULL};
    sil_t s;
    fc_t f;
    if (!scenario_start(&s, &f, args, true)) {
        failures++;
        return;
    }
    CHECK(wait_ready(4000));
    answer_t a = fc_txn(FCEC_KIND_ARM, profile_id(), 2000);
    CHECK(a.answered && a.outcome == FCEC_OUTCOME_ACCEPTED);

    uint64_t end = mono_us() + 2500u * 1000u;
    long start_ev = -1;
    while (mono_us() < end && start_ev < 0) {
        run_demands(20, 18.0, 18.0, 0.5);
        start_ev = ev_find(&s, 0, "kind=inject", "what=drop_start");
    }
    CHECK(start_ev >= 0);
    unsigned drop_ms = event_t_ms(&s, start_ev);
    uint64_t statuses_at_drop = f.statuses;
    /* We keep sending; the EC hears nothing, so the demand timeout fires. */
    for (int i = 0; i < 40; i++) {
        fc_demand(18.0, 18.0, 0.5);
        step_ms(5);
    }
    long safing = wait_event(0, "kind=state", "to=SAFING", 500);
    CHECK(safing >= 0);
    if (safing >= 0) {
        CHECK(strstr(s.events[safing].line, "reason=DEMAND_TIMEOUT") != NULL);
        unsigned lag = event_t_ms(&s, safing) - drop_ms;
        printf("  info: SAFING %u ms after the drop (demand timeout 30 ms)\n", lag);
        CHECK(lag >= 20u && lag <= 90u);
    }
    /* No Status after the drop (allow the one or two already in flight). */
    CHECK(f.statuses - statuses_at_drop <= 3u);
    long closed = wait_event(0, "kind=session", "state=closed", 1000);
    CHECK(closed >= 0);
    if (closed >= 0 && start_ev >= 0) {
        unsigned lag = event_t_ms(&s, closed) - drop_ms;
        printf("  info: session closed %u ms after the drop (link timeout 350 ms)\n", lag);
        CHECK(lag >= 330u && lag <= 480u);
    }
    const tick_t *t = last_tick(&s);
    CHECK(t != NULL && strcmp(t->state, "SAFING") == 0 && t->esc_us == 1000u);

    /* The drop ends after 900 ms: the EC answers again, but is still safed
     * and needs the FC to say Hello and ClearSafing. */
    CHECK(wait_event(0, "kind=inject", "what=drop_end", 2000) >= 0);
    send_heartbeats = true;
    step_ms(50);
    a = fc_hello();
    CHECK(a.answered && a.outcome == FCEC_OUTCOME_ACCEPTED && a.state == FCEC_STATE_SAFING);
    step_ms(60);
    a = fc_txn(FCEC_KIND_CLEAR_SAFING, NULL, 2000);
    CHECK(a.answered && a.outcome == FCEC_OUTCOME_ACCEPTED && a.state == FCEC_STATE_DISARMED);

    scenario_end(&s, &f);
}

/* ---- Scenario 5: what flight2's 200 Hz demands do to this EC ------------ */

/* Informational, no pass/fail on the firmware's behaviour: flight2 sends a
 * demand every 5 ms, but the PA-12 driver needs a 5 ms quiet gap after every
 * goal write and sends goals ahead of position reads. A new goal is pending
 * every time the gap ends, so the periodic position read can starve; the
 * port loses readiness after 500 ms without a valid reply, and an armed EC
 * safes with LOCAL_FAULT. This prints what actually happens. */
static void test_200hz_demands(void) {
    printf("scenario 5: flight2's 200 Hz demands (informational)\n");
    static const char *const args[] = {"--period-ms", "5", "--esc-arm-hold-ms", "200", NULL};
    sil_t s;
    fc_t f;
    if (!scenario_start(&s, &f, args, true)) {
        failures++;
        return;
    }
    CHECK(wait_ready(4000));
    answer_t a = fc_txn(FCEC_KIND_ARM, profile_id(), 2000);
    CHECK(a.answered && a.outcome == FCEC_OUTCOME_ACCEPTED);
    long armed = ev_find(&s, 0, "kind=state", "to=ARMED");
    demand_period_ms = 5;
    uint64_t end = mono_us() + 1500000u;
    long safing = -1;
    while (mono_us() < end && safing < 0) {
        run_demands(20, 15.0, 15.0, 0.3);
        safing = ev_find(&s, 0, "kind=state", "to=SAFING");
    }
    demand_period_ms = 10;
    if (safing >= 0 && armed >= 0) {
        char reason[40] = "";
        kv_get(s.events[safing].line, "reason", reason, sizeof reason);
        printf("  info: with 5 ms demands the EC went SAFING (%s) %u ms after Arm\n", reason,
               event_t_ms(&s, safing) - event_t_ms(&s, armed));
    } else {
        printf("  info: with 5 ms demands the EC stayed armed for 1.5 s\n");
    }
    const tick_t *t = last_tick(&s);
    if (t != NULL) {
        printf("  info: last tick state=%s ready=0x%02X app_seq=%u dem_seq=%u\n", t->state, t->ready,
               t->app_seq, t->dem_seq);
    }
    scenario_end(&s, &f);
}

/* ---- Scenario 6: deterministic mode and CSV ----------------------------- */

typedef struct {
    char text[16384];
    size_t len;
} capture_t;

/* Runs sil_ec to completion and captures its stdout. */
static bool run_to_exit(capture_t *c, const char *const *extra) {
    int fds[2];
    if (pipe(fds) != 0) {
        return false;
    }
    char port[16];
    snprintf(port, sizeof port, "%u", base_port + next_instance++);
    const char *argv[32];
    size_t n = 0;
    argv[n++] = sil_exe;
    argv[n++] = "-p";
    argv[n++] = port;
    for (size_t i = 0; extra[i] != NULL && n < 30; i++) {
        argv[n++] = extra[i];
    }
    argv[n] = NULL;
    pid_t pid = fork();
    if (pid == 0) {
        dup2(fds[1], STDOUT_FILENO);
        close(fds[0]);
        close(fds[1]);
        execv(sil_exe, (char *const *)argv);
        _exit(127);
    }
    close(fds[1]);
    c->len = 0;
    for (;;) {
        ssize_t got = read(fds[0], c->text + c->len, sizeof c->text - 1u - c->len);
        if (got <= 0) {
            break;
        }
        c->len += (size_t)got;
    }
    c->text[c->len] = '\0';
    close(fds[0]);
    int status = 0;
    waitpid(pid, &status, 0);
    return WIFEXITED(status) && WEXITSTATUS(status) == 0;
}

static size_t count_lines_with(const char *text, const char *prefix) {
    size_t n = 0;
    size_t plen = strlen(prefix);
    for (const char *p = text; *p != '\0';) {
        if (strncmp(p, prefix, plen) == 0) {
            n++;
        }
        const char *nl = strchr(p, '\n');
        if (nl == NULL) {
            break;
        }
        p = nl + 1;
    }
    return n;
}

static void test_deterministic_and_csv(void) {
    printf("scenario 6: --deterministic and --csv\n");
    char csv_path[64];
    snprintf(csv_path, sizeof csv_path, "/tmp/test_sil_ec_%d.csv", (int)getpid());
    const char *const args[] = {"--deterministic", "--exit-after-ms", "400", "--period-ms", "50",
                                "--csv", csv_path, NULL};
    static capture_t a, b;
    CHECK(run_to_exit(&a, args));
    CHECK(run_to_exit(&b, args));
    /* Exactly the same lines (the ports differ, so compare from the TICKs). */
    const char *ta = strstr(a.text, "TICK ");
    const char *tb = strstr(b.text, "TICK ");
    CHECK(ta != NULL && tb != NULL && strcmp(ta, tb) == 0);
    CHECK(strstr(a.text, "boot_id=0x51B00701") != NULL);
    CHECK(strstr(a.text, "mode=deterministic") != NULL);
    /* 0, 50, ..., 400 ms. */
    size_t ticks = count_lines_with(a.text, "TICK ");
    CHECK(ticks == 9u);
    CHECK(strstr(a.text, "TICK t_ms=0 ") != NULL || strstr(a.text, "TICK t_ms=1 ") != NULL);

    FILE *f = fopen(csv_path, "r");
    CHECK(f != NULL);
    if (f != NULL) {
        char line[512];
        size_t rows = 0;
        bool header_ok = false;
        while (fgets(line, sizeof line, f) != NULL) {
            if (rows == 0) {
                header_ok = strncmp(line, "t_ms,state,reason,a1_mm,", 24) == 0;
            }
            rows++;
        }
        fclose(f);
        CHECK(header_ok);
        CHECK(rows == ticks + 1u); /* header + one row per TICK */
        unlink(csv_path);
    }
}

int main(int argc, char **argv) {
    if (argc > 1) {
        sil_exe = argv[1];
    }
    signal(SIGPIPE, SIG_IGN);
    base_port = 20000u + (unsigned)getpid() % 20000u;

    test_full_session();
    test_park_and_stuck_actuator();
    test_silent_actuator_loses_readiness();
    test_drop_fault_injection();
    test_200hz_demands();
    test_deterministic_and_csv();

    printf("%d checks, %d failed\n", checks, failures);
    return failures == 0 ? 0 : 1;
}
