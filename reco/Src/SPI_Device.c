#include "SPI_Device.h"

/**
 * @brief Transmit data to an SPI device with manual chip-select control.
 *
 * Drives the device chip-select (CS) line low, transmits the specified data
 * buffer over SPI using the STM32 HAL, and then releases the CS line. Global
 * interrupts are temporarily disabled to ensure the transaction is not
 * interrupted.
 *
 * @param[in] SPI_Device  Pointer to the SPI device descriptor containing
 *                        the HAL SPI handle and chip-select GPIO.
 * @param[in] txData      Pointer to the transmit data buffer.
 * @param[in] size        Number of bytes to transmit.
 * @param[in] timeout     Timeout duration in milliseconds.
 *
 * @return HAL_StatusTypeDef
 *         - HAL_OK on successful transmission
 *         - HAL_ERROR, HAL_BUSY, or HAL_TIMEOUT on failure
 *
 * @note This function assumes the SPI peripheral is already initialized.
 * @note Interrupts are disabled during the entire SPI transaction.
 */
HAL_StatusTypeDef SPI_Device_Transmit(spi_device_t* SPI_Device, uint8_t* txData, uint16_t size, uint32_t timeout) {
    __disable_irq();
    HAL_GPIO_WritePin(SPI_Device->GPIO_Port, SPI_Device->GPIO_Pin, GPIO_PIN_RESET);
    HAL_StatusTypeDef status = HAL_SPI_Transmit(SPI_Device->hspi, txData, size, timeout);
    HAL_GPIO_WritePin(SPI_Device->GPIO_Port, SPI_Device->GPIO_Pin, GPIO_PIN_SET);
    __enable_irq();
    return status;
}

/**
 * @brief Receive data from an SPI device with manual chip-select control.
 *
 * Drives the device chip-select (CS) line low, receives data over SPI using
 * the STM32 HAL, and then releases the CS line. Global interrupts are
 * temporarily disabled to ensure the transaction is not interrupted.
 *
 * @param[in]  SPI_Device Pointer to the SPI device descriptor containing
 *                        the HAL SPI handle and chip-select GPIO.
 * @param[out] rxData     Pointer to the receive data buffer.
 * @param[in]  size       Number of bytes to receive.
 * @param[in]  timeout    Timeout duration in milliseconds.
 *
 * @return HAL_StatusTypeDef
 *         - HAL_OK on successful reception
 *         - HAL_ERROR, HAL_BUSY, or HAL_TIMEOUT on failure
 *
 * @note This function assumes the SPI peripheral is already initialized.
 * @note Interrupts are disabled during the entire SPI transaction.
 */
HAL_StatusTypeDef SPI_Device_Receive(spi_device_t* SPI_Device, uint8_t* rxData, uint16_t size, uint32_t timeout) {
	__disable_irq();
    HAL_GPIO_WritePin(SPI_Device->GPIO_Port, SPI_Device->GPIO_Pin, GPIO_PIN_RESET);
    HAL_StatusTypeDef status = HAL_SPI_Receive(SPI_Device->hspi, rxData, size, timeout);
    HAL_GPIO_WritePin(SPI_Device->GPIO_Port, SPI_Device->GPIO_Pin, GPIO_PIN_SET);
    __enable_irq();
    return status;
}

/**
 * @brief Transmit and receive data simultaneously from an SPI device.
 *
 * Drives the device chip-select (CS) line low, performs a full-duplex SPI
 * transaction using the STM32 HAL transmit-receive API, and then releases
 * the CS line. Global interrupts are temporarily disabled to ensure the
 * transaction is not interrupted.
 *
 * @param[in]  SPI_Device Pointer to the SPI device descriptor containing
 *                        the HAL SPI handle and chip-select GPIO.
 * @param[in]  txData     Pointer to the transmit data buffer.
 * @param[out] rxData     Pointer to the receive data buffer.
 * @param[in]  size       Number of bytes to transmit and receive.
 * @param[in]  timeout    Timeout duration in milliseconds.
 *
 * @return HAL_StatusTypeDef
 *         - HAL_OK on successful transaction
 *         - HAL_ERROR, HAL_BUSY, or HAL_TIMEOUT on failure
 *
 * @note This function is suitable for devices that require simultaneous
 *       command and data exchange.
 * @note Interrupts are disabled during the entire SPI transaction.
 */
HAL_StatusTypeDef SPI_Device_TransmitReceive(spi_device_t* SPI_Device, uint8_t* txData, uint8_t* rxData, uint16_t size, uint32_t timeout) {
    __disable_irq();
    HAL_GPIO_WritePin(SPI_Device->GPIO_Port, SPI_Device->GPIO_Pin, GPIO_PIN_RESET);
    HAL_StatusTypeDef status = HAL_SPI_TransmitReceive(SPI_Device->hspi, txData, rxData, size, timeout);
    HAL_GPIO_WritePin(SPI_Device->GPIO_Port, SPI_Device->GPIO_Pin, GPIO_PIN_SET);
    __enable_irq();
    return status;
}

/**
 * @brief Transmit and receive data in separate SPI transactions under one CS assertion.
 *
 * Drives the device chip-select (CS) line low, transmits a command or address
 * sequence, then receives data in a separate SPI transaction while keeping
 * CS asserted. This is commonly required by sensors that expect a command
 * phase followed by a read phase.
 *
 * Global interrupts are temporarily disabled to ensure the combined
 * transaction is not interrupted.
 *
 * @param[in]  SPI_Device Pointer to the SPI device descriptor containing
 *                        the HAL SPI handle and chip-select GPIO.
 * @param[in]  txData     Pointer to the transmit data buffer.
 * @param[out] rxData     Pointer to the receive data buffer.
 * @param[in]  size1      Number of bytes to transmit.
 * @param[in]  size2      Number of bytes to receive.
 * @param[in]  timeout    Timeout duration in milliseconds.
 *
 * @return HAL_StatusTypeDef
 *         - HAL_OK on successful transmission and reception
 *         - HAL_ERROR, HAL_BUSY, or HAL_TIMEOUT on failure
 *
 * @note This function is useful for SPI devices that do not support
 *       full-duplex transfers.
 * @note Interrupts are disabled during the entire SPI transaction.
 */
HAL_StatusTypeDef SPI_Device_TransmitReceiveSeparate(spi_device_t* SPI_Device, uint8_t* txData, uint8_t* rxData, uint16_t size1, uint16_t size2, uint32_t timeout) {
    __disable_irq();
    HAL_GPIO_WritePin(SPI_Device->GPIO_Port, SPI_Device->GPIO_Pin, GPIO_PIN_RESET);
    HAL_StatusTypeDef status = HAL_SPI_Transmit(SPI_Device->hspi, txData, size1, timeout);
    status = HAL_SPI_Receive(SPI_Device->hspi, rxData, size2, timeout);
    HAL_GPIO_WritePin(SPI_Device->GPIO_Port, SPI_Device->GPIO_Pin, GPIO_PIN_SET);
    __enable_irq();
    return status;
}
