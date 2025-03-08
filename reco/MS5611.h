#ifndef __MS5611
#define __MS5611

#include "stm32h7xx_hal.h"
#include "SPI_Device.h"

typedef enum {
      LOWEST_D1 = 0x40,
      LOW_D1 = 0x42,
      MED_D1 = 0x44,
      HIGH_D1 = 0x46,
      HIGHEST_D1 = 0x48,
      LOWEST_D2 = 0x50,
      LOW_D2 = 0x52,
      MED_D2 = 0x54,
      HIGH_D2 = 0x56,
      HIGHEST_D2 = 0x58,
} baro_accuracy_t;

typedef enum {
    READ_ADC = 0x00,
    BARO_RESET = 0x1E,
    PROM_READ = 0xA0,
} baro_commands_t;

typedef enum {
    LOWEST_TIME = 1,
    LOW_TIME = 2,
    MED_TIME = 3,
    HIGH_TIME = 5,
    HIGHEST_TIME = 10,
} baro_conversion_time_t;

// Make sure you use the correct percision value for each

typedef struct {
    double temperature;
    double pressure;
    uint64_t dT; // Digital Pressure
    int32_t firstTemp;
    baro_accuracy_t tempAccuracy; // Use only the D1 values
    baro_accuracy_t pressureAccuracy; // Use only the D2 values
    baro_conversion_time_t convertTime;
    uint16_t coefficients[6]; // [C1, C2, C3, C4, C5, C6]
} baro_handle_t;

void initBarometer(spi_device_t* baroSPI, baro_handle_t* baroHandle);
void resetBarometer(spi_device_t* baroSPI);
void getPROMData(spi_device_t* baroSPI, baro_handle_t* baroHandle);
void getCurrTempPressure(spi_device_t* baroSPI, baro_handle_t* baroHandle);
void getPressure(spi_device_t* baroSPI, baro_handle_t* baroHandle);
void getTemp(spi_device_t* baroSPI, baro_handle_t* baroHandle, TIM_HandleTypeDef htim);
void startPressureConversion(spi_device_t* baroSPI, baro_handle_t* baroHandle);
void startTempConversion(spi_device_t* baroSPI, baro_handle_t* baroHandle);



#endif




