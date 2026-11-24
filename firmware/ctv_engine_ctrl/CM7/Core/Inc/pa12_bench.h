#ifndef PA12_BENCH_H
#define PA12_BENCH_H

/* Bench-only PA-12 bring-up: find each actuator's baud rate and ID, read its
 * settings, and move it to raw goal counts for calibration. Driven from the
 * debugger through the `pa12_bench` struct (CubeIDE Live Expressions).
 *
 * Built only with EC_PA12_BENCH defined (CMake preset "Bench"). It uses the
 * blocking pa12.c calls, so it stalls the main loop for up to a few seconds
 * during a scan: never enable it in a flight or normal bench build. It never
 * writes a non-volatile register (ID, stroke limits, feedback mode, PID).
 */

#include <stdint.h>

#include "pa12.h"

#ifdef EC_PA12_BENCH

typedef enum {
    PA12_BENCH_IDLE = 0,
    PA12_BENCH_SCAN = 1,      /* find baud + ID on `port` */
    PA12_BENCH_READ_INFO = 2, /* read stroke limits, feedback mode, voltage */
    PA12_BENCH_GOTO = 3,      /* move `port` to `goal_counts` (inside its stroke limits) */
    PA12_BENCH_FORCE_OFF = 4  /* motor power off; the next goal turns it back on */
} pa12_bench_command_t;

typedef struct {
    uint32_t found;           /* 1 once a scan got a reply */
    uint32_t baud;
    uint32_t id;
    uint32_t info_valid;      /* 1 once READ_INFO succeeded */
    uint32_t feedback_mode;   /* reg 0x10 */
    uint32_t short_limit;     /* reg 0x06, counts */
    uint32_t long_limit;      /* reg 0x08, counts */
    uint32_t voltage_dv;      /* reg 0x92, 0.1 V */
    uint32_t position;        /* reg 0x8C, counts, polled every 100 ms */
    uint32_t moving;          /* reg 0x96 */
    uint32_t error_flags;     /* status byte of the last reply */
    int32_t last_result;      /* PA12_Result of the last poll */
    uint32_t polls;
    uint32_t poll_failures;
} pa12_bench_port_t;

typedef struct {
    /* Set `port` and `goal_counts` first, then `command`. It goes back to 0
     * when done; the outcome is in `last_command_result`. */
    uint32_t command;
    uint32_t port;            /* 0 = A1 (UART7), 1 = A2 (UART5) */
    uint32_t goal_counts;
    int32_t last_command_result; /* 0 OK, >0 PA12_Result, -1 refused */
    uint32_t scan_baud;       /* scan progress */
    uint32_t scan_id;
    pa12_bench_port_t p[2];
} pa12_bench_t;

extern volatile pa12_bench_t pa12_bench;

void PA12_Bench_Init(PA12_Device *a1, PA12_Device *a2);
void PA12_Bench_Process(void);

#endif /* EC_PA12_BENCH */
#endif /* PA12_BENCH_H */
