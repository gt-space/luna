#ifndef EC_UDP_H
#define EC_UDP_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* Called from the NO_SYS lwIP receive context (main loop, not interrupt).
 * Handler sets *accepted only when the protocol acknowledges this sender:
 * the state machine updates last_rx_us or produces a reply.
 * It can write a reply packet to reply[] and return its length, or 0.
 */
typedef size_t (*EC_UDP_ReceiveHandler)(const uint8_t *rx, size_t rx_len,
                                        uint64_t now_us,
                                        uint8_t *reply, size_t reply_cap,
                                        bool *accepted);

typedef struct {
    uint32_t rx_accepted;
    uint32_t rx_wrong_sender;
    uint32_t rx_invalid_size;
    uint32_t rx_copy_failed;
    uint32_t replies_sent;
    uint32_t statuses_sent;
    uint32_t tx_failed;
} EC_UDP_Stats;

/* All functions run on the NO_SYS lwIP foreground thread. */
bool EC_UDP_Init(EC_UDP_ReceiveHandler handler);
bool EC_UDP_HasPeer(void);
bool EC_UDP_SendStatus(const uint8_t *data, size_t len);
const EC_UDP_Stats *EC_UDP_GetStats(void);

#endif /* EC_UDP_H */
