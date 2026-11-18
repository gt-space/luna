#include "adin1200_stm32.h"

#define ADIN1200_REG_PHY_ID1         0x02U
#define ADIN1200_REG_PHY_ID2         0x03U
#define ADIN1200_REG_PHY_STATUS_1    0x1AU

#define ADIN1200_PS1_LINK_UP         (1U << 6)
#define ADIN1200_PS1_ANEG_COMPLETE   (1U << 12)
#define ADIN1200_PS1_RESOLVED_SHIFT  7U
#define ADIN1200_PS1_RESOLVED_MASK   (7U << ADIN1200_PS1_RESOLVED_SHIFT)

HAL_StatusTypeDef ADIN1200_ReadId(ETH_HandleTypeDef *heth,
                                  uint16_t *id1, uint16_t *id2)
{
    uint32_t r1 = 0, r2 = 0;
    if (heth == NULL || id1 == NULL || id2 == NULL) return HAL_ERROR;
    if (HAL_ETH_ReadPHYRegister(heth, ADIN1200_PHY_ADDRESS,
                                ADIN1200_REG_PHY_ID1, &r1) != HAL_OK)
        return HAL_ERROR;
    if (HAL_ETH_ReadPHYRegister(heth, ADIN1200_PHY_ADDRESS,
                                ADIN1200_REG_PHY_ID2, &r2) != HAL_OK)
        return HAL_ERROR;
    *id1 = (uint16_t)r1;
    *id2 = (uint16_t)r2;
    /* Model ID check; revision in low 4 bits can vary. */
    if (*id1 != 0x0283U || ((*id2 & 0xFFF0U) != 0xBC20U))
        return HAL_ERROR;
    return HAL_OK;
}

HAL_StatusTypeDef ADIN1200_ReadLinkInfo(ETH_HandleTypeDef *heth,
                                        ADIN1200_LinkInfo *info)
{
    uint32_t value = 0;
    if (heth == NULL || info == NULL) return HAL_ERROR;
    *info = (ADIN1200_LinkInfo){0};
    if (HAL_ETH_ReadPHYRegister(heth, ADIN1200_PHY_ADDRESS,
                                ADIN1200_REG_PHY_STATUS_1, &value) != HAL_OK)
        return HAL_ERROR;

    info->phy_status_1 = (uint16_t)value;
    info->auto_negotiation_done =
        (value & ADIN1200_PS1_ANEG_COMPLETE) != 0U;

    /* Board is strapped for autonegotiation; do not consume HCD_TECH
       until the physical link and autonegotiation are complete. */
    if ((value & ADIN1200_PS1_LINK_UP) == 0U ||
        !info->auto_negotiation_done)
        return HAL_OK;

    /* ADIN1200 PHY_STATUS_1 [9:7] HCD_TECH:
       000 = 10 HD, 001 = 10 FD, 010 = 100 HD, 011 = 100 FD. */
    uint32_t mode = (value & ADIN1200_PS1_RESOLVED_MASK) >>
                    ADIN1200_PS1_RESOLVED_SHIFT;
    if (mode > 3U) return HAL_ERROR;

    info->speed_mbps = (mode >= 2U) ? 100U : 10U;
    info->full_duplex = (mode & 1U) != 0U;
    info->link_up = true;
    return HAL_OK;
}
