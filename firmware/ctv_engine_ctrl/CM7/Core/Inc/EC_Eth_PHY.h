/*
 * EC_Eth_PHY.h
 *
 *  Created on: Nov 13, 2026
 *      Author: Stone
 */

#ifndef EC_ETH_PHY_EC_ETH_PHY_H_
#define EC_ETH_PHY_EC_ETH_PHY_H_
#include "adin1200_stm32.h"
#include "lwip/netif.h"

void EC_ETH_LinkCheck(struct netif *netif);

#endif /* EC_ETH_PHY_EC_ETH_PHY_H_ */
