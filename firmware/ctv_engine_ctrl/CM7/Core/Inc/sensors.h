#ifndef SENSORS_H
#define SENSORS_H

#include "stm32h7xx_hal.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Call once after CubeMX's MX_ADC3_Init(). */
HAL_StatusTypeDef Sensors_ADC_Init(void);

/*
 * Selects one of the board's ADC3 channels, performs one software-triggered
 * conversion, and writes the raw ADC count into *result on success.
 *
 * Supported channels: 0, 2-9, 13-16.
 * Uses blocking polling (10 ms timeout). Not safe for concurrent callers.
 */
HAL_StatusTypeDef ADC_ReadChannel(uint32_t channel, uint16_t *result);

#ifdef __cplusplus
}
#endif

#endif /* SENSORS_H */
