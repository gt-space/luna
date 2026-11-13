#ifndef PA12_H
#define PA12_H

#include "main.h"
#include <stdbool.h>
#include <stdint.h>

/* Progressive Automations PA-12 UART: IR Protocol 1.0, derived from the
 * user-provided PA12_Arduino library. External SN74LVC2G241 direction:
 * DIR HIGH = transmit, DIR LOW = receive. Hardware UART is FULL DUPLEX;
 * the external logic implements a shared half-duplex actuator data line.
 */

typedef enum {
    PA12_OK = 0,
    PA12_INVALID_ARGUMENT,
    PA12_UART_ERROR,
    PA12_TIMEOUT,
    PA12_BAD_HEADER,
    PA12_BAD_ID,
    PA12_BAD_LENGTH,
    PA12_BAD_CHECKSUM,
    PA12_DEVICE_ERROR
} PA12_Result;

typedef struct {
    UART_HandleTypeDef *uart;
    GPIO_TypeDef *direction_port;
    uint16_t direction_pin;
    uint8_t id;
    uint8_t last_device_error;    /* Raw status byte returned by actuator. */
    HAL_StatusTypeDef last_hal_status;
    uint32_t tx_timeout_ms;
    uint32_t rx_timeout_ms;
    bool expect_write_ack;        /* false matches library's fire-and-forget mode */
} PA12_Device;

void PA12_Init(PA12_Device *dev,
               UART_HandleTypeDef *uart,
               GPIO_TypeDef *dir_port,
               uint16_t dir_pin,
               uint8_t device_id);

/* Default write behavior: return after TX finishes, without requiring status.
 * If the actuator is configured to reply to WRITE, set true to check it.
 */
void PA12_SetExpectWriteAck(PA12_Device *dev, bool enabled);
void PA12_SetTimeouts(PA12_Device *dev, uint32_t tx_ms, uint32_t rx_ms);

PA12_Result PA12_Ping(PA12_Device *dev);
PA12_Result PA12_ReadU8(PA12_Device *dev, uint8_t addr, uint8_t *value);
PA12_Result PA12_ReadU16(PA12_Device *dev, uint8_t addr, uint16_t *value);
PA12_Result PA12_WriteU8(PA12_Device *dev, uint8_t addr, uint8_t value);
PA12_Result PA12_WriteU16(PA12_Device *dev, uint8_t addr, uint16_t value);

/* Read-only register queries are best for initial bench bring-up. */
PA12_Result PA12_GetPosition(PA12_Device *dev, uint16_t *position);
PA12_Result PA12_GetSpeed(PA12_Device *dev, uint16_t *speed);
PA12_Result PA12_GetTemperature(PA12_Device *dev, uint8_t *temperature);
PA12_Result PA12_GetMoving(PA12_Device *dev, uint8_t *moving);

/* Motion commands: caller must first validate position/speed limits,
 * mechanical stops, supply and application interlocks. Positions are raw
 * PA-12 register units; 0..4095 is the range in Arduino examples.
 */
PA12_Result PA12_SetGoalPosition(PA12_Device *dev, uint16_t position);
PA12_Result PA12_SetMovingSpeed(PA12_Device *dev, uint16_t speed_raw);
PA12_Result PA12_SetPositionAndSpeed(PA12_Device *dev, uint16_t position,
                                     uint16_t speed_raw);

#endif /* PA12_H */
