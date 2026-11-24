#include "pa12_bench.h"

#ifdef EC_PA12_BENCH

#include "main.h"

extern IWDG_HandleTypeDef hiwdg1;

volatile pa12_bench_t pa12_bench;

/* Factory default first, then the other rates the PA-12 supports. */
static const uint32_t bauds[] = {57600U, 115200U, 9600U, 19200U, 38400U};
#define BAUD_COUNT (sizeof bauds / sizeof bauds[0])
#define MAX_ID 253U          /* 0xFE is broadcast and never replies */
#define SCAN_RX_TIMEOUT_MS 20U
#define RX_TIMEOUT_MS 50U
#define TX_TIMEOUT_MS 20U
#define POLL_PERIOD_MS 100U
#define READ_GAP_MS 10U      /* quiet time after a read (protocol guidance) */

/* Non-volatile registers are only ever read here. */
#define REG_SHORT_LIMIT 0x06U
#define REG_LONG_LIMIT 0x08U
#define REG_FEEDBACK_MODE 0x10U
#define REG_FORCE 0x80U
#define REG_VOLTAGE 0x92U
#define REG_MOVING 0x96U

static PA12_Device *dev[2];
static uint32_t last_poll_ms;

static void kick(void) {
    HAL_IWDG_Refresh(&hiwdg1);
}

static bool set_baud(PA12_Device *d, uint32_t baud) {
    d->uart->Init.BaudRate = baud;
    return HAL_UART_Init(d->uart) == HAL_OK;
}

/* A reply with device error flags still proves the baud rate and ID. */
static bool replied(PA12_Result r) {
    return r == PA12_OK || r == PA12_DEVICE_ERROR;
}

static int32_t scan(uint32_t port) {
    PA12_Device *d = dev[port];
    volatile pa12_bench_port_t *p = &pa12_bench.p[port];
    p->found = 0U;
    p->info_valid = 0U;
    PA12_SetTimeouts(d, TX_TIMEOUT_MS, SCAN_RX_TIMEOUT_MS);

    for (uint32_t b = 0U; b < BAUD_COUNT; ++b) {
        pa12_bench.scan_baud = bauds[b];
        if (!set_baud(d, bauds[b])) {
            continue;
        }
        for (uint32_t id = 0U; id <= MAX_ID; ++id) {
            kick();
            pa12_bench.scan_id = id;
            d->id = (uint8_t)id;
            if (replied(PA12_Ping(d))) {
                p->found = 1U;
                p->baud = bauds[b];
                p->id = id;
                p->error_flags = d->last_device_error;
                PA12_SetTimeouts(d, TX_TIMEOUT_MS, RX_TIMEOUT_MS);
                return 0;
            }
        }
    }
    /* Nothing answered: back to factory settings. */
    (void)set_baud(d, bauds[0]);
    d->id = 0U;
    PA12_SetTimeouts(d, TX_TIMEOUT_MS, RX_TIMEOUT_MS);
    return (int32_t)PA12_TIMEOUT;
}

static int32_t read_info(uint32_t port) {
    PA12_Device *d = dev[port];
    volatile pa12_bench_port_t *p = &pa12_bench.p[port];
    uint8_t mode = 0U, volts = 0U;
    uint16_t short_limit = 0U, long_limit = 0U;
    PA12_Result r;

    p->info_valid = 0U;
    if ((r = PA12_ReadU8(d, REG_FEEDBACK_MODE, &mode)) != PA12_OK) return (int32_t)r;
    HAL_Delay(READ_GAP_MS);
    if ((r = PA12_ReadU16(d, REG_SHORT_LIMIT, &short_limit)) != PA12_OK) return (int32_t)r;
    HAL_Delay(READ_GAP_MS);
    if ((r = PA12_ReadU16(d, REG_LONG_LIMIT, &long_limit)) != PA12_OK) return (int32_t)r;
    HAL_Delay(READ_GAP_MS);
    if ((r = PA12_ReadU8(d, REG_VOLTAGE, &volts)) != PA12_OK) return (int32_t)r;
    HAL_Delay(READ_GAP_MS);

    p->feedback_mode = mode;
    p->short_limit = short_limit;
    p->long_limit = long_limit;
    p->voltage_dv = volts;
    p->info_valid = 1U;
    return 0;
}

static int32_t run(uint32_t command, uint32_t port) {
    if (port > 1U) {
        return -1;
    }
    volatile pa12_bench_port_t *p = &pa12_bench.p[port];
    switch (command) {
    case PA12_BENCH_SCAN:
        return scan(port);
    case PA12_BENCH_READ_INFO:
        return p->found ? read_info(port) : -1;
    case PA12_BENCH_GOTO: {
        uint32_t goal = pa12_bench.goal_counts;
        /* Only inside the actuator's own stroke limits, read from it first. */
        if (!p->found || !p->info_valid || goal > 4095U || goal < p->short_limit ||
            goal > p->long_limit) {
            return -1;
        }
        return (int32_t)PA12_SetGoalPosition(dev[port], (uint16_t)goal);
    }
    case PA12_BENCH_FORCE_OFF:
        return p->found ? (int32_t)PA12_WriteU8(dev[port], REG_FORCE, 0U) : -1;
    default:
        return -1;
    }
}

static void poll(uint32_t port) {
    PA12_Device *d = dev[port];
    volatile pa12_bench_port_t *p = &pa12_bench.p[port];
    uint16_t position = 0U;
    uint8_t moving = 0U;
    PA12_Result r = PA12_GetPosition(d, &position);
    if (r == PA12_OK || r == PA12_DEVICE_ERROR) {
        p->position = position;
        HAL_Delay(READ_GAP_MS);
        r = PA12_GetMoving(d, &moving);
        if (r == PA12_OK || r == PA12_DEVICE_ERROR) {
            p->moving = moving;
        }
    }
    p->error_flags = d->last_device_error;
    p->last_result = (int32_t)r;
    p->polls++;
    if (!replied(r)) {
        p->poll_failures++;
    }
}

void PA12_Bench_Init(PA12_Device *a1, PA12_Device *a2) {
    dev[0] = a1;
    dev[1] = a2;
    PA12_SetTimeouts(a1, TX_TIMEOUT_MS, RX_TIMEOUT_MS);
    PA12_SetTimeouts(a2, TX_TIMEOUT_MS, RX_TIMEOUT_MS);
    last_poll_ms = HAL_GetTick();
}

void PA12_Bench_Process(void) {
    uint32_t command = pa12_bench.command;
    if (command != PA12_BENCH_IDLE) {
        pa12_bench.last_command_result = run(command, pa12_bench.port);
        pa12_bench.command = PA12_BENCH_IDLE;
        kick();
    }

    uint32_t now = HAL_GetTick();
    if (now - last_poll_ms < POLL_PERIOD_MS) {
        return;
    }
    last_poll_ms = now;
    for (uint32_t port = 0U; port < 2U; ++port) {
        if (pa12_bench.p[port].found) {
            poll(port);
            kick();
        }
    }
}

#endif /* EC_PA12_BENCH */
