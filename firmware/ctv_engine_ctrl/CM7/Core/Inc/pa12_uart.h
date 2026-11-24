#ifndef PA12_UART_H
#define PA12_UART_H

/* Non-blocking UART7 (A1) / UART5 (A2) adapter for the PA-12 driver
 * (pa12_drv.c), polled from the main loop, no interrupts.
 *
 * Uses each UART's 16-byte hardware FIFO: a whole packet (<= 10 bytes) is
 * loaded at once, the direction buffer is switched back to receive when the
 * TC flag says the final byte has left the wire, and reply bytes are drained
 * from the RX FIFO. A main-loop stall of up to ~2.5 ms loses nothing.
 */

#include <stdbool.h>
#include <stdint.h>

#include "pa12_drv.h"

/* After MX_UART5/7_Init. Enables the FIFOs and sets both directions to
 * receive. Returns false if the UARTs can't be set up. */
bool PA12_UART_Init(void);

/* The ops to hand to fcec_pa12_init(). */
const fcec_pa12_ops_t *PA12_UART_Ops(void);

/* Every loop, before fcec_pa12_service(): finishes transmissions and feeds
 * received bytes to the driver. */
void PA12_UART_Poll(fcec_pa12_t *drv, uint64_t now_us);

#endif /* PA12_UART_H */
