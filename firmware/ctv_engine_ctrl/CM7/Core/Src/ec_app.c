#include "ec_app.h"
#include "ec_bench_profile.h"
#include "ec_udp.h"
#include "esc.h"
#include "esc_tim2.h"
#include "fc_ec_link.h"
#include "main.h"
#include "pa12_drv.h"
#include "pa12_uart.h"

#include <string.h>

#ifdef EC_BOARD_HEALTH
#include "ec_health.h"
#endif

#define EC_STATUS_PERIOD_US (10000ULL)

/* The bench-only PA-12 tool (EC_PA12_BENCH) owns the PA-12 UARTs with
 * blocking calls, so the real driver stays off in that build. */
#ifndef EC_PA12_BENCH
#define EC_PA12_DRIVER 1
#endif

#ifdef EC_ESC_CALIBRATION
/* Set to 1 from the debugger after the ESCs' two beeps: switches to the stop
 * pulse, which stores the low endpoint. Never goes back to full. */
volatile uint32_t esc_calibration_step;
#endif

static fcec_ec_t ec;
#ifndef EC_ESC_CALIBRATION
static fcec_esc_t esc;
#endif
static bool esc_running;
#ifdef EC_PA12_DRIVER
static fcec_pa12_t pa12;
#endif
static bool initialized;
static uint64_t next_status_us;

static size_t on_ec_udp_message(const uint8_t *rx, size_t len,
                                uint64_t now_us,
                                uint8_t *reply, size_t reply_cap,
                                bool *accepted) {
    const uint64_t previous_rx_us = ec.last_rx_us;
    const size_t n = fcec_ec_receive(&ec, rx, len, now_us, reply, reply_cap);
    /* Mirrors fake_ec.c: remember the sender when the EC accepts the
     * message as valid or generates a protocol-level transaction reply.
     */
    *accepted = (ec.last_rx_us != previous_rx_us || n > 0U);
    return n;
}

bool EC_App_Init(uint32_t boot_id) {
    if (initialized || boot_id == 0U) {
        return false;
    }
    fcec_ec_init(&ec, boot_id);
    /* Real drivers: an output only counts as applied when its driver confirms
     * it (fcec_ec_confirm_applied), and only configured channels are offered.
     * Nothing is ready until its driver says so. A driver adds its channel
     * only when its block in ec_bench_profile.h is approved; with none
     * approved the FC sees no capabilities and Arm is refused (NOT_READY).
     * The RCS fans (M1-M4) stay unconfigured on this branch.
     */
    ec.auto_confirm = false;
    ec.capabilities = 0U;
    fcec_ec_set_ready(&ec, FCEC_CH_ALL, false);
    ec.firmware_version = 1U; /* development identifier only */

    /* Bench profile: identity checked at Arm, TVC park + travel window. */
    static const char profile_id[] = EC_BENCH_PROFILE_ID;
    _Static_assert(sizeof profile_id <= sizeof ec.bundle_sha256, "profile ID too long");
    memset(ec.bundle_sha256, 0, sizeof ec.bundle_sha256);
    memcpy(ec.bundle_sha256, profile_id, sizeof profile_id - 1U);
    ec.check_bundle = true;
    ec.tvc_center_mm[0] = EC_TVC_PARK_MM_A1;
    ec.tvc_center_mm[1] = EC_TVC_PARK_MM_A2;
    ec.limit_min.tvc_extension_mm[0] = EC_TVC_MIN_MM;
    ec.limit_min.tvc_extension_mm[1] = EC_TVC_MIN_MM;
    ec.limit_max.tvc_extension_mm[0] = EC_TVC_MAX_MM;
    ec.limit_max.tvc_extension_mm[1] = EC_TVC_MAX_MM;

    const uint64_t now_us = (uint64_t)HAL_GetTick() * 1000ULL;

    /* Main ESC: no signal at all unless its profile is approved. */
#ifdef EC_ESC_CALIBRATION
    /* Supervised ESC throttle calibration (props off): full pulse from boot,
     * stop pulse once the operator sets esc_calibration_step = 1 after the two
     * beeps. The FC is never offered the ESC in this build. */
    if (ec_esc_profile.min_pulse_us > 0U &&
        ec_esc_profile.max_pulse_us > ec_esc_profile.min_pulse_us &&
        ec_esc_profile.frame_period_us > ec_esc_profile.max_pulse_us) {
        esc_running = ESC_TIM2_Start(ec_esc_profile.frame_period_us,
                                     ec_esc_profile.max_pulse_us);
    }
#else
    fcec_esc_init(&esc, &ec_esc_profile, now_us);
    if (fcec_esc_config_ok(&ec_esc_profile) &&
        ESC_TIM2_Start(ec_esc_profile.frame_period_us, ec_esc_profile.min_pulse_us)) {
        esc_running = true;
        ec.capabilities |= FCEC_CH_MAIN_ESC;
        ec.limit_max.main_throttle = ec_esc_profile.max_throttle;
    }
#endif

#ifdef EC_PA12_DRIVER
    /* PA-12s: only approved actuators are offered; nothing is sent to the
     * others. */
    if (PA12_UART_Init()) {
        ec.capabilities |= fcec_pa12_init(&pa12, ec_pa12_profile, PA12_UART_Ops(), now_us);
    }
#endif

    /* Nothing is valid until a driver has a real reading for it. */
    ec.measurements.validity_mask = 0U;
#ifdef EC_BOARD_HEALTH
    EC_Health_Init(); /* scans I2C3 once; ADC and TMP112 run in Process */
#endif

    if (!EC_UDP_Init(on_ec_udp_message)) {
        return false; /* Remain out of BOOT_SAFE until transport exists. */
    }
    fcec_ec_boot_done(&ec);
    next_status_us = (uint64_t)HAL_GetTick() * 1000ULL + EC_STATUS_PERIOD_US;
    initialized = true;
    return true;
}

void EC_App_Process(uint64_t now_us) {
    if (!initialized) {
        return;
    }
    /* This runs even if no packets are received, so link and demand timeouts
     * still trigger the EC state machine.
     */
    fcec_ec_tick(&ec, now_us);

    /* Output drivers: each reads fcec_ec_command(&ec), outputs its channels
     * in drive_mask at its own update boundary without blocking, then
     * confirms (fcec_ec_confirm_applied) or reports a fault. The RCS fans
     * (Motors_*) are not driven on this branch.
     */
#ifdef EC_ESC_CALIBRATION
    if (esc_running) {
        ESC_TIM2_Load(esc_calibration_step != 0U ? ec_esc_profile.min_pulse_us
                                                 : ec_esc_profile.max_pulse_us);
    }
#else
    if (esc_running) {
        ESC_TIM2_Load(fcec_esc_service(&esc, &ec, now_us));
    }
#endif
#ifdef EC_PA12_DRIVER
    PA12_UART_Poll(&pa12, now_us);
    fcec_pa12_service(&pa12, &ec, now_us);
#endif

#ifdef EC_BOARD_HEALTH
    /* Rails every 10 ms, board temperatures every 250 ms (bounded). */
    EC_Health_Process(now_us, &ec.measurements);
#endif

    if (now_us < next_status_us) {
        return;
    }
#ifdef EC_PA12_DRIVER
    fcec_pa12_publish(&pa12, &ec.measurements, now_us);
#endif
#ifndef EC_BOARD_HEALTH
    /* One measurement batch per Status (board health numbers its own). */
    ec.measurements.sample_id++;
    ec.measurements.sample_time_us = now_us;
#endif
    if (EC_UDP_HasPeer()) {
        static uint8_t status[FCEC_MAX_DATAGRAM_LEN];
        const size_t n = fcec_ec_status(&ec, now_us,
                                        status, sizeof(status));
        if (n > 0U) {
            (void)EC_UDP_SendStatus(status, n);
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
        ec.counters.deadline_miss = (uint16_t)(ec.counters.deadline_miss +
                                               (uint16_t)missed);
    }
    next_status_us += intervals * EC_STATUS_PERIOD_US;
}

const fcec_ec_t *EC_App_State(void) {
    return initialized ? &ec : NULL;
}
