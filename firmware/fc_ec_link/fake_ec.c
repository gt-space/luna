/*
 * Fake engine controller for testing the FC side on a laptop, before the real
 * EC has Ethernet.
 *
 * Runs the same link logic as the EC firmware (ec_side.c) behind a UDP
 * socket, simulates the PA-12 positions and the board rails, and prints state
 * changes plus a once-a-second summary.
 *
 *     make fake_ec && ./fake_ec            # listens on UDP 8378
 *     ./fake_ec -p 9000                    # another port
 *
 * Replies and Status go to the address the FC's packets come from. The FC
 * sends from its EC socket (UDP 4574), so that's where they arrive.
 */

#define _POSIX_C_SOURCE 200809L

#include "ec_side.h"

#include <arpa/inet.h>
#include <errno.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <poll.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <time.h>
#include <unistd.h>

#define STATUS_PERIOD_US 10000u
#define SUMMARY_PERIOD_US 1000000u
#define PA12_SPEED_MM_PER_S 20.0f /* simulated actuator speed */

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
    return id != 0 ? id : 1;
}

/* Last goal sent to each simulated PA-12. Like the real one, it keeps moving
 * to that goal even after the EC stops sending new ones. */
static float pa12_goal_mm[2];

/* Moves each PA-12 toward its goal at a fixed speed and makes up plausible
 * rail readings. */
static void simulate_measurements(fcec_ec_t *ec, uint64_t now_us, float dt_s) {
    const fcec_outputs_t *out = fcec_ec_outputs(ec);
    fcec_measurements_t *m = &ec->measurements;
    float step = PA12_SPEED_MM_PER_S * dt_s;

    if (fcec_ec_drive_tvc(ec)) {
        pa12_goal_mm[0] = out->tvc_extension_mm[0];
        pa12_goal_mm[1] = out->tvc_extension_mm[1];
    }

    for (size_t i = 0; i < 2; i++) {
        float error = pa12_goal_mm[i] - m->tvc_position_mm[i];
        if (error > step) {
            error = step;
        } else if (error < -step) {
            error = -step;
        }
        m->tvc_position_mm[i] += error;
    }

    float fan_current = 0.0f;
    for (size_t i = 0; i < 4; i++) {
        m->rcs_current_a[i] = 0.1f * out->rcs_duty[i];
        fan_current += m->rcs_current_a[i];
    }

    m->sample_id++;
    m->sample_time_us = now_us;
    m->tvc_position_age_ms[0] = 5;
    m->tvc_position_age_ms[1] = 5;
    m->rail_voltage_v[0] = 22.2f - 2.0f * out->main_throttle;
    m->rail_voltage_v[1] = 12.0f;
    m->rail_voltage_v[2] = 4.4f - 0.2f * fan_current;
    m->rail_voltage_v[3] = 3.3f;
    m->rail_current_a[0] = 0.3f + 60.0f * out->main_throttle;
    m->rail_current_a[1] = 0.1f;
    m->rail_current_a[2] = fan_current;
    m->rail_current_a[3] = 0.2f;
    m->motor_rail_voltage_v = m->rail_voltage_v[2];
    m->board_temp_c[0] = 30.0f;
    m->board_temp_c[1] = 31.0f;
    m->validity_mask = 0x1FFFFu;
}

static void print_time(uint64_t now_us) {
    printf("[%7.3f s] ", (double)now_us / 1e6);
}

static void print_summary(const fcec_ec_t *ec, uint64_t now_us) {
    const fcec_outputs_t *o = fcec_ec_outputs(ec);
    const fcec_counters_t *c = &ec->counters;
    print_time(now_us);
    printf("%-9s session=%s rx=%u acc=%u app=%u | tvc %.2f %.2f mm (at %.2f %.2f) "
           "rcs %.2f %.2f %.2f %.2f thr %.2f | crc=%u rej=%u stale=%u old=%u\n",
           fcec_state_str(ec->state), ec->session_valid ? "yes" : "no",
           (unsigned)ec->last_received_seq, (unsigned)ec->last_accepted_demand_seq,
           (unsigned)ec->last_applied_demand_seq, (double)o->tvc_extension_mm[0],
           (double)o->tvc_extension_mm[1], (double)ec->measurements.tvc_position_mm[0],
           (double)ec->measurements.tvc_position_mm[1], (double)o->rcs_duty[0],
           (double)o->rcs_duty[1], (double)o->rcs_duty[2], (double)o->rcs_duty[3],
           (double)o->main_throttle, (unsigned)c->crc_errors, (unsigned)c->rejected,
           (unsigned)c->stale, (unsigned)c->old_or_duplicate);
}

int main(int argc, char **argv) {
    uint16_t port = FCEC_EC_PORT;
    int opt;
    while ((opt = getopt(argc, argv, "p:")) != -1) {
        if (opt == 'p') {
            port = (uint16_t)strtoul(optarg, NULL, 10);
        } else {
            fprintf(stderr, "usage: %s [-p port]\n", argv[0]);
            return 2;
        }
    }

    int fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (fd < 0) {
        perror("socket");
        return 1;
    }
    int yes = 1;
    setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof yes);

    struct sockaddr_in local;
    memset(&local, 0, sizeof local);
    local.sin_family = AF_INET;
    local.sin_addr.s_addr = htonl(INADDR_ANY);
    local.sin_port = htons(port);
    if (bind(fd, (struct sockaddr *)&local, sizeof local) < 0) {
        perror("bind");
        return 1;
    }
    fcntl(fd, F_SETFL, fcntl(fd, F_GETFL, 0) | O_NONBLOCK);

    struct sigaction sa;
    memset(&sa, 0, sizeof sa);
    sa.sa_handler = on_signal;
    sigaction(SIGINT, &sa, NULL);
    sigaction(SIGTERM, &sa, NULL);

    fcec_ec_t ec;
    fcec_ec_init(&ec, random_boot_id());
    ec.firmware_version = 0x00000001u; /* fake EC */
    ec.measurements.tvc_position_mm[0] = ec.tvc_center_mm[0];
    ec.measurements.tvc_position_mm[1] = ec.tvc_center_mm[1];
    pa12_goal_mm[0] = ec.tvc_center_mm[0];
    pa12_goal_mm[1] = ec.tvc_center_mm[1];
    fcec_ec_boot_done(&ec);

    uint64_t start = monotonic_us();
    uint64_t now = 0;
    uint64_t next_status = 0;
    uint64_t next_summary = SUMMARY_PERIOD_US;
    uint64_t last_sim = 0;
    struct sockaddr_in fc;
    bool have_fc = false;
    fcec_ec_state_t shown_state = ec.state;
    bool shown_session = false;

    printf("fake EC listening on UDP %u, boot ID 0x%08X, state %s\n", (unsigned)port,
           (unsigned)ec.boot_id, fcec_state_str(ec.state));
    fflush(stdout);

    while (running) {
        struct pollfd pfd = {fd, POLLIN, 0};
        poll(&pfd, 1, 1);
        now = monotonic_us() - start;

        for (;;) {
            uint8_t rx[FCEC_MAX_DATAGRAM_LEN + 1];
            uint8_t tx[FCEC_MAX_DATAGRAM_LEN];
            struct sockaddr_in from;
            socklen_t from_len = sizeof from;
            ssize_t n = recvfrom(fd, rx, sizeof rx, 0, (struct sockaddr *)&from, &from_len);
            if (n < 0) {
                if (errno != EAGAIN && errno != EWOULDBLOCK) {
                    perror("recvfrom");
                }
                break;
            }

            uint64_t rx_before = ec.last_rx_us;
            size_t reply_len = fcec_ec_receive(&ec, rx, (size_t)n, now, tx, sizeof tx);

            /* Remember who the FC is once it has sent something we accepted. */
            if (ec.last_rx_us != rx_before || reply_len > 0) {
                fc = from;
                have_fc = true;
            }
            if (reply_len > 0) {
                sendto(fd, tx, reply_len, 0, (struct sockaddr *)&from, sizeof from);
                fcec_packet_t r;
                if (fcec_decode(tx, reply_len, &r) == FCEC_OK) {
                    print_time(now);
                    if (r.kind == FCEC_KIND_HELLO_RESULT) {
                        bool ok = r.msg.hello_result.outcome == FCEC_OUTCOME_ACCEPTED;
                        printf("Hello from %s:%u session 0x%08X -> %s\n",
                               inet_ntoa(from.sin_addr), (unsigned)ntohs(from.sin_port),
                               (unsigned)r.header.session_id,
                               ok ? "accepted" : fcec_reason_str(r.msg.hello_result.reason));
                    } else {
                        static const char *const names[] = {"?",         "Hello",  "Arm",
                                                            "Disarm",    "Terminate",
                                                            "ClearSafing"};
                        unsigned k = (unsigned)r.msg.txn_result.request_kind;
                        bool ok = r.msg.txn_result.outcome == FCEC_OUTCOME_ACCEPTED;
                        printf("%s txn %u -> %s%s\n", k < 6 ? names[k] : "?",
                               (unsigned)r.header.txn_id,
                               ok ? "accepted" : fcec_reason_str(r.msg.txn_result.reason),
                               r.header.duplicate ? " (duplicate)" : "");
                    }
                }
            }
        }

        fcec_ec_tick(&ec, now);

        if (ec.session_valid != shown_session) {
            print_time(now);
            printf("session %s\n", ec.session_valid ? "opened" : "closed");
            shown_session = ec.session_valid;
        }
        if (ec.state != shown_state) {
            print_time(now);
            printf("state %s -> %s", fcec_state_str(shown_state), fcec_state_str(ec.state));
            if (ec.state == FCEC_STATE_SAFING) {
                printf(" (%s)", fcec_reason_str(ec.safing_reason));
            }
            printf("\n");
            shown_state = ec.state;
        }

        if (now >= next_status) {
            simulate_measurements(&ec, now, (float)(now - last_sim) / 1e6f);
            last_sim = now;
            if (have_fc) {
                uint8_t tx[FCEC_MAX_DATAGRAM_LEN];
                size_t len = fcec_ec_status(&ec, now, tx, sizeof tx);
                sendto(fd, tx, len, 0, (struct sockaddr *)&fc, sizeof fc);
            }
            next_status += STATUS_PERIOD_US;
            if (next_status <= now) {
                ec.counters.deadline_miss++;
                next_status = now + STATUS_PERIOD_US;
            }
        }

        if (now >= next_summary) {
            if (have_fc) {
                print_summary(&ec, now);
            }
            next_summary += SUMMARY_PERIOD_US;
        }
        fflush(stdout);
    }

    printf("\nfake EC stopped\n");
    close(fd);
    return 0;
}
