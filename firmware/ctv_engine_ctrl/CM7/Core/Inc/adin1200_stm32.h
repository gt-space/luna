#ifndef ADIN1200_STM32_H
#define ADIN1200_STM32_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "stm32h7xx_hal.h"

/* Expected PHYAD from the board straps; verify by reading ID registers. */
#define ADIN1200_PHY_ADDRESS   0U

typedef struct {
    bool link_up;               /* True when link and autonegotiation are complete */
    bool auto_negotiation_done;
    uint32_t speed_mbps;       /* 10 or 100, valid only when link_up is true */
    bool full_duplex;           /* Valid only when link_up is true */
    uint16_t phy_status_1;     /* Raw PHY_STATUS_1 (Clause 22, address 0x1A) */
} ADIN1200_LinkInfo;

/* Requires HAL_ETH_Init() to have succeeded on the supplied MAC handle. */
HAL_StatusTypeDef ADIN1200_ReadId(ETH_HandleTypeDef *heth,
                                  uint16_t *id1, uint16_t *id2);

/* Poll ADIN1200's non-latched link, autonegotiation, and resolved-mode bits. */
HAL_StatusTypeDef ADIN1200_ReadLinkInfo(ETH_HandleTypeDef *heth,
                                        ADIN1200_LinkInfo *info);

#endif /* ADIN1200_STM32_H */
