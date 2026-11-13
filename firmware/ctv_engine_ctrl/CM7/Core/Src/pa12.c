#include "pa12.h"
#include <stddef.h>

/* Protocol constants from PA12_Arduino/utility/IR_Protocol_format.h */
#define PA12_HEADER                 0xFFU
#define PA12_CMD_PING               0xF1U
#define PA12_CMD_READ               0xF2U
#define PA12_CMD_WRITE              0xF3U
#define PA12_BROADCAST_ID           0xFEU
#define PA12_GOAL_POSITION_REG      0x86U
#define PA12_MOVING_SPEED_REG       0x88U
#define PA12_PRESENT_POSITION_REG   0x8CU
#define PA12_PRESENT_SPEED_REG      0x8EU
#define PA12_PRESENT_TEMPERATURE_REG 0x93U
#define PA12_MOVING_REG             0x96U

#define PA12_MAX_PARAMETERS         16U
#define PA12_DEFAULT_TX_TIMEOUT_MS  50U
#define PA12_DEFAULT_RX_TIMEOUT_MS  250U

static bool valid_device(const PA12_Device *dev)
{
    return dev != NULL && dev->uart != NULL && dev->direction_port != NULL &&
           dev->id < PA12_BROADCAST_ID &&
           dev->tx_timeout_ms != 0U && dev->rx_timeout_ms != 0U;
}

static uint8_t checksum(const uint8_t *packet, size_t without_checksum)
{
    uint8_t sum = 0U;
    for (size_t i = 3U; i < without_checksum; ++i) {
        sum = (uint8_t)(sum + packet[i]);
    }
    return (uint8_t)~sum;
}

static PA12_Result get_byte(PA12_Device *dev, uint8_t *value, uint32_t start_ms)
{
    uint32_t elapsed = HAL_GetTick() - start_ms;
    if (elapsed >= dev->rx_timeout_ms) {
        return PA12_TIMEOUT;
    }
    uint32_t remaining = dev->rx_timeout_ms - elapsed;
    HAL_StatusTypeDef status = HAL_UART_Receive(dev->uart, value, 1U, remaining);
    dev->last_hal_status = status;
    if (status == HAL_TIMEOUT) {
        return PA12_TIMEOUT;
    }
    return (status == HAL_OK) ? PA12_OK : PA12_UART_ERROR;
}

static PA12_Result receive_status(PA12_Device *dev,
                                  uint8_t *data, uint8_t expected_data_len)
{
    uint32_t start_ms = HAL_GetTick();
    uint8_t b = 0U;
    uint8_t matched = 0U;
    /* Look for FF FF FF, tolerating up to 32 leading stray bytes. */
    for (unsigned attempts = 0U; attempts < 35U && matched < 3U; ++attempts) {
        PA12_Result r = get_byte(dev, &b, start_ms);
        if (r != PA12_OK) return r;
        matched = (b == PA12_HEADER) ? (uint8_t)(matched + 1U) : 0U;
    }
    if (matched != 3U) return PA12_BAD_HEADER;

    uint8_t id = 0U, length = 0U, error = 0U;
    PA12_Result r = get_byte(dev, &id, start_ms);
    if (r != PA12_OK) return r;
    r = get_byte(dev, &length, start_ms);
    if (r != PA12_OK) return r;

    if (id != dev->id) return PA12_BAD_ID;
    /* Response: ID LEN ERROR [DATA...] CHECKSUM, LEN = DATA length + 2. */
    if (length != (uint8_t)(expected_data_len + 2U)) return PA12_BAD_LENGTH;

    r = get_byte(dev, &error, start_ms);
    if (r != PA12_OK) return r;
    dev->last_device_error = error;
    uint8_t sum = (uint8_t)(id + length + error);
    for (uint8_t i = 0U; i < expected_data_len; ++i) {
        uint8_t value = 0U;
        r = get_byte(dev, &value, start_ms);
        if (r != PA12_OK) return r;
        if (data != NULL) data[i] = value;
        sum = (uint8_t)(sum + value);
    }
    uint8_t check = 0U;
    r = get_byte(dev, &check, start_ms);
    if (r != PA12_OK) return r;
    if ((uint8_t)(sum + check) != 0xFFU) return PA12_BAD_CHECKSUM;
    return (error == 0U) ? PA12_OK : PA12_DEVICE_ERROR;
}

static void flush_pending_rx(PA12_Device *dev)
{
    /* Clear stale data from previous optional write-status packets. This is
     * only called when no request is in flight. H7 HAL supports RX flush.
     */
    __HAL_UART_CLEAR_OREFLAG(dev->uart);
    __HAL_UART_SEND_REQ(dev->uart, UART_RXDATA_FLUSH_REQUEST);
}

static PA12_Result transaction(PA12_Device *dev,
                               uint8_t command, const uint8_t *parameters,
                               uint8_t parameter_count, uint8_t *result,
                               uint8_t result_count, bool expect_response)
{
    if (!valid_device(dev) || parameter_count > PA12_MAX_PARAMETERS ||
        result_count > PA12_MAX_PARAMETERS ||
        (parameter_count != 0U && parameters == NULL) ||
        (result_count != 0U && result == NULL)) {
        return PA12_INVALID_ARGUMENT;
    }

    /* FF FF FF ID LEN INSTRUCTION [PARAMETERS] CHECKSUM */
    uint8_t packet[7U + PA12_MAX_PARAMETERS];
    const uint8_t length = (uint8_t)(parameter_count + 2U);
    const uint16_t packet_size = (uint16_t)(7U + parameter_count);
    packet[0] = packet[1] = packet[2] = PA12_HEADER;
    packet[3] = dev->id;
    packet[4] = length;
    packet[5] = command;
    for (uint8_t i = 0U; i < parameter_count; ++i) {
        packet[6U + i] = parameters[i];
    }
    packet[6U + parameter_count] = checksum(packet, packet_size - 1U);

    dev->last_device_error = 0U;
    flush_pending_rx(dev);
    HAL_GPIO_WritePin(dev->direction_port, dev->direction_pin, GPIO_PIN_SET);
    /* HAL_UART_Transmit() waits until transmission complete (TC) before OK. */
    HAL_StatusTypeDef st = HAL_UART_Transmit(dev->uart, packet,
                                            packet_size, dev->tx_timeout_ms);
    dev->last_hal_status = st;
    HAL_GPIO_WritePin(dev->direction_port, dev->direction_pin, GPIO_PIN_RESET);

    if (st == HAL_TIMEOUT) return PA12_TIMEOUT;
    if (st != HAL_OK) return PA12_UART_ERROR;
    if (!expect_response) return PA12_OK;
    return receive_status(dev, result, result_count);
}

void PA12_Init(PA12_Device *dev, UART_HandleTypeDef *uart,
               GPIO_TypeDef *dir_port, uint16_t dir_pin, uint8_t device_id)
{
    if (dev == NULL) return;
    dev->uart = uart;
    dev->direction_port = dir_port;
    dev->direction_pin = dir_pin;
    dev->id = device_id;
    dev->last_device_error = 0U;
    dev->last_hal_status = HAL_OK;
    dev->tx_timeout_ms = PA12_DEFAULT_TX_TIMEOUT_MS;
    dev->rx_timeout_ms = PA12_DEFAULT_RX_TIMEOUT_MS;
    dev->expect_write_ack = false;
    if (dir_port != NULL) {
        HAL_GPIO_WritePin(dir_port, dir_pin, GPIO_PIN_RESET); /* receive idle */
    }
}

void PA12_SetExpectWriteAck(PA12_Device *dev, bool enabled)
{
    if (dev != NULL) dev->expect_write_ack = enabled;
}

void PA12_SetTimeouts(PA12_Device *dev, uint32_t tx_ms, uint32_t rx_ms)
{
    if (dev == NULL || tx_ms == 0U || rx_ms == 0U) return;
    dev->tx_timeout_ms = tx_ms;
    dev->rx_timeout_ms = rx_ms;
}

PA12_Result PA12_Ping(PA12_Device *dev)
{
    return transaction(dev, PA12_CMD_PING, NULL, 0U, NULL, 0U, true);
}

static PA12_Result read_n(PA12_Device *dev, uint8_t address,
                          uint8_t *buffer, uint8_t n)
{
    const uint8_t params[2] = {address, n};
    return transaction(dev, PA12_CMD_READ, params, 2U, buffer, n, true);
}

static PA12_Result write_n(PA12_Device *dev, uint8_t address,
                           const uint8_t *buffer, uint8_t n)
{
    if (n > (PA12_MAX_PARAMETERS - 1U)) return PA12_INVALID_ARGUMENT;
    uint8_t params[1U + PA12_MAX_PARAMETERS] = {0};
    params[0] = address;
    for (uint8_t i = 0U; i < n; ++i) params[1U + i] = buffer[i];
    return transaction(dev, PA12_CMD_WRITE, params,
                       (uint8_t)(n + 1U), NULL, 0U,
                       (dev != NULL) && dev->expect_write_ack);
}

PA12_Result PA12_ReadU8(PA12_Device *dev, uint8_t addr, uint8_t *value)
{
    if (value == NULL) return PA12_INVALID_ARGUMENT;
    return read_n(dev, addr, value, 1U);
}

PA12_Result PA12_ReadU16(PA12_Device *dev, uint8_t addr, uint16_t *value)
{
    if (value == NULL) return PA12_INVALID_ARGUMENT;
    uint8_t bytes[2] = {0};
    PA12_Result r = read_n(dev, addr, bytes, 2U);
    if (r == PA12_OK) *value = (uint16_t)(bytes[0] | ((uint16_t)bytes[1] << 8));
    return r;
}

PA12_Result PA12_WriteU8(PA12_Device *dev, uint8_t addr, uint8_t value)
{
    return write_n(dev, addr, &value, 1U);
}

PA12_Result PA12_WriteU16(PA12_Device *dev, uint8_t addr, uint16_t value)
{
    uint8_t bytes[2] = {(uint8_t)value, (uint8_t)(value >> 8)};
    return write_n(dev, addr, bytes, 2U);
}

PA12_Result PA12_GetPosition(PA12_Device *dev, uint16_t *position)
{
    return PA12_ReadU16(dev, PA12_PRESENT_POSITION_REG, position);
}

PA12_Result PA12_GetSpeed(PA12_Device *dev, uint16_t *speed)
{
    return PA12_ReadU16(dev, PA12_PRESENT_SPEED_REG, speed);
}

PA12_Result PA12_GetTemperature(PA12_Device *dev, uint8_t *temperature)
{
    return PA12_ReadU8(dev, PA12_PRESENT_TEMPERATURE_REG, temperature);
}

PA12_Result PA12_GetMoving(PA12_Device *dev, uint8_t *moving)
{
    return PA12_ReadU8(dev, PA12_MOVING_REG, moving);
}

PA12_Result PA12_SetGoalPosition(PA12_Device *dev, uint16_t position)
{
    if (position > 4095U) return PA12_INVALID_ARGUMENT;
    return PA12_WriteU16(dev, PA12_GOAL_POSITION_REG, position);
}

PA12_Result PA12_SetMovingSpeed(PA12_Device *dev, uint16_t speed_raw)
{
    return PA12_WriteU16(dev, PA12_MOVING_SPEED_REG, speed_raw);
}

PA12_Result PA12_SetPositionAndSpeed(PA12_Device *dev, uint16_t position,
                                     uint16_t speed_raw)
{
    if (position > 4095U) return PA12_INVALID_ARGUMENT;
    const uint8_t params[5] = {
        PA12_GOAL_POSITION_REG,
        (uint8_t)position, (uint8_t)(position >> 8),
        (uint8_t)speed_raw, (uint8_t)(speed_raw >> 8)
    };
    return transaction(dev, PA12_CMD_WRITE, params, 5U, NULL, 0U,
                       (dev != NULL) && dev->expect_write_ack);
}
