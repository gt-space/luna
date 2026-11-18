#include "ec_udp.h"
#include "fc_ec_link.h"
#include "main.h"
#include "lwip.h"
#include "lwip/ip_addr.h"
#include "lwip/pbuf.h"
#include "lwip/udp.h"

/* CubeMX defines gnetif in lwip.c. */
extern struct netif gnetif;

static struct udp_pcb *ec_pcb;
static ip_addr_t fc_expected_ip;
static ip_addr_t fc_peer_ip;
static u16_t fc_peer_port;
static bool peer_known;
static EC_UDP_ReceiveHandler handler;
static EC_UDP_Stats stats;

static bool send_to(const ip_addr_t *dst, u16_t port,
                    const uint8_t *data, size_t len) {
    if (ec_pcb == NULL || dst == NULL || data == NULL || len == 0U ||
        len > FCEC_MAX_DATAGRAM_LEN || !netif_is_link_up(&gnetif)) {
        ++stats.tx_failed;
        return false;
    }

    struct pbuf *packet = pbuf_alloc(PBUF_TRANSPORT, (u16_t)len, PBUF_RAM);
    if (packet == NULL) {
        ++stats.tx_failed;
        return false;
    }

    const bool ok = (pbuf_take(packet, data, (u16_t)len) == ERR_OK) &&
                    (udp_sendto(ec_pcb, packet, dst, port) == ERR_OK);
    pbuf_free(packet);
    if (!ok) {
        ++stats.tx_failed;
    }
    return ok;
}

static void on_receive(void *arg, struct udp_pcb *pcb, struct pbuf *p,
                       const ip_addr_t *source, u16_t port) {
    (void)arg;
    (void)pcb;
    if (p == NULL) {
        return;
    }

    /* IP and port filtering prevents accidental cross-talk, not spoofing. */
    if (source == NULL || !ip_addr_cmp(source, &fc_expected_ip) || port != (u16_t)FCEC_FC_PORT) {
        ++stats.rx_wrong_sender;
        pbuf_free(p);
        return;
    }

    const size_t n = (size_t)p->tot_len;
    if (n < FCEC_HEADER_LEN + FCEC_CRC_LEN || n > FCEC_MAX_DATAGRAM_LEN) {
        ++stats.rx_invalid_size;
        pbuf_free(p);
        return;
    }

    static uint8_t rx[FCEC_MAX_DATAGRAM_LEN];
    if (pbuf_copy_partial(p, rx, (u16_t)n, 0U) != (u16_t)n) {
        ++stats.rx_copy_failed;
        pbuf_free(p);
        return;
    }

    /* Copy source too: do not depend on a pointer provided by the stack
     * remaining valid after the incoming pbuf is released.
     */
    const ip_addr_t sender_ip = *source;
    const u16_t sender_port = port;
    pbuf_free(p);

    static uint8_t reply[FCEC_MAX_DATAGRAM_LEN];
    bool accepted = false;
    size_t reply_len = handler(rx, n, (uint64_t)HAL_GetTick() * 1000ULL,
                               reply, sizeof(reply), &accepted);
    if (reply_len > sizeof(reply)) {
        ++stats.tx_failed;
        return; /* Defensive guard against a broken callback. */
    }
    if (accepted) {
        fc_peer_ip = sender_ip;
        fc_peer_port = sender_port;
        peer_known = true;
        ++stats.rx_accepted;
    }
    /* Reply to this datagram's source, even for a protocol-level rejection. */
    if (reply_len > 0U && send_to(&sender_ip, sender_port, reply, reply_len)) {
        ++stats.replies_sent;
    }
}

bool EC_UDP_Init(EC_UDP_ReceiveHandler receiver) {
    if (ec_pcb != NULL || receiver == NULL) {
        return false;
    }
    IP_ADDR4(&fc_expected_ip, 192, 168, 1, 11);
    handler = receiver;
    peer_known = false;
    ec_pcb = udp_new();
    if (ec_pcb == NULL) {
        handler = NULL;
        return false;
    }
    if (udp_bind(ec_pcb, IP_ADDR_ANY, (u16_t)FCEC_EC_PORT) != ERR_OK) {
        udp_remove(ec_pcb);
        ec_pcb = NULL;
        handler = NULL;
        return false;
    }
    udp_recv(ec_pcb, on_receive, NULL);
    return true;
}

bool EC_UDP_HasPeer(void) {
    return peer_known;
}

bool EC_UDP_SendStatus(const uint8_t *data, size_t len) {
    if (!peer_known) {
        return false;
    }
    if (send_to(&fc_peer_ip, fc_peer_port, data, len)) {
        ++stats.statuses_sent;
        return true;
    }
    return false;
}

const EC_UDP_Stats *EC_UDP_GetStats(void) {
    return &stats;
}
