#include "SPI_Device.h"

HAL_StatusTypeDef SPI_Device_Transmit(spi_device_t* SPI_Device, uint8_t* txData, uint16_t size, uint32_t timeout) {
    __disable_irq();
    HAL_GPIO_WritePin(SPI_Device->GPIO_Port, SPI_Device->GPIO_Pin, GPIO_PIN_RESET);
    HAL_StatusTypeDef status = HAL_SPI_Transmit(SPI_Device->hspi, txData, size, timeout);
    HAL_GPIO_WritePin(SPI_Device->GPIO_Port, SPI_Device->GPIO_Pin, GPIO_PIN_SET);
    __enable_irq();
    return status;
}

HAL_StatusTypeDef SPI_Device_Receive(spi_device_t* SPI_Device, uint8_t* rxData, uint16_t size, uint32_t timeout) {
	__disable_irq();
    HAL_GPIO_WritePin(SPI_Device->GPIO_Port, SPI_Device->GPIO_Pin, GPIO_PIN_RESET);
    HAL_StatusTypeDef status = HAL_SPI_Receive(SPI_Device->hspi, rxData, size, timeout);
    HAL_GPIO_WritePin(SPI_Device->GPIO_Port, SPI_Device->GPIO_Pin, GPIO_PIN_SET);
    __enable_irq();
    return status;
}

HAL_StatusTypeDef SPI_Device_TransmitReceive(spi_device_t* SPI_Device, uint8_t* txData, uint8_t* rxData, uint16_t size, uint32_t timeout) {
    __disable_irq();
    HAL_GPIO_WritePin(SPI_Device->GPIO_Port, SPI_Device->GPIO_Pin, GPIO_PIN_RESET);
    HAL_StatusTypeDef status = HAL_SPI_TransmitReceive(SPI_Device->hspi, txData, rxData, size, timeout);
    HAL_GPIO_WritePin(SPI_Device->GPIO_Port, SPI_Device->GPIO_Pin, GPIO_PIN_SET);
    __enable_irq();
    return status;
}

HAL_StatusTypeDef SPI_Device_TransmitReceiveSeparate(spi_device_t* SPI_Device, uint8_t* txData, uint8_t* rxData, uint16_t size1, uint16_t size2, uint32_t timeout) {
    __disable_irq();
    HAL_GPIO_WritePin(SPI_Device->GPIO_Port, SPI_Device->GPIO_Pin, GPIO_PIN_RESET);
    HAL_StatusTypeDef status = HAL_SPI_Transmit(SPI_Device->hspi, txData, size1, timeout);
    status = HAL_SPI_Receive(SPI_Device->hspi, rxData, size2, timeout);
    HAL_GPIO_WritePin(SPI_Device->GPIO_Port, SPI_Device->GPIO_Pin, GPIO_PIN_SET);
    __enable_irq();
    return status;
}
