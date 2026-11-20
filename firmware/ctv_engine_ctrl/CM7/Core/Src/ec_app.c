#include "ec_app.h"
#include "ec_udp.h"
#include "fc_ec_link.h"
#include "main.h"

#define EC_STATUS_PERIOD_US (10000ULL)

static fcec_ec_t ec;
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
     * No output driver is connected yet, so nothing is configured or ready:
     * the FC sees no capabilities and Arm is refused (NOT_READY). Each driver
     * adds its channels here once it exists. The RCS fans (M1-M4) stay
     * unconfigured on this branch.
     */
    ec.auto_confirm = false;
    ec.capabilities = 0U;
    fcec_ec_set_ready(&ec, FCEC_CH_ALL, false);
    ec.firmware_version = 1U; /* development identifier only */
    /* Do not advertise measurements until valid real readings exist. */
    ec.measurements.validity_mask = 0U;

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

    /* Output drivers go here: each one reads fcec_ec_command(&ec), outputs
     * its channels in drive_mask at its own update boundary without blocking,
     * then calls fcec_ec_confirm_applied() (or fcec_ec_report_fault()). None
     * is connected yet: do not call Motors_*, PA12_* or the ESC timer here
     * until that driver and its interlocks are reviewed.
     */

    if (now_us < next_status_us) {
        return;
    }
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
