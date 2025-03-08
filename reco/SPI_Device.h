#ifndef _SPI_Device
#define _SPI_Device

#include "stm32h7xx.h"

typedef struct {
    SPI_HandleTypeDef* hspi;   // SPI Handler
    GPIO_TypeDef* GPIO_Port;   // GPIO Port for the device (CS)
    uint16_t GPIO_Pin;         // GPIO Pin for the device (CS)
} spi_device_t;

HAL_StatusTypeDef SPI_Device_Transmit(spi_device_t* SPI_Device, uint8_t* txData, uint16_t size, uint32_t timeout);
HAL_StatusTypeDef SPI_Device_Receive(spi_device_t* SPI_Device, uint8_t* rxData, uint16_t size, uint32_t timeout);
HAL_StatusTypeDef SPI_Device_TransmitReceive(spi_device_t* SPI_Device, uint8_t* txData, uint8_t* rxData, uint16_t size, uint32_t timeout);
HAL_StatusTypeDef SPI_Device_TransmitReceiveSeparate(spi_device_t* SPI_Device, uint8_t* txData, uint8_t* rxData, uint16_t size1, uint16_t size2, uint32_t timeout);

#endif
