#ifndef __MS5611
#define __MS5611

#include "stm32h5xx_hal.h"
#include "SPI_Device.h"
#include "stdint.h"
#include "arm_math_types.h"
#include "comms.h"

// D1
typedef enum {
    LOWEST_PRESS = 0x40,
    LOW_PRESS = 0x42,
    MED_PRESS = 0x44,
    HIGH_PRESS = 0x46,
    HIGHEST_PRESS = 0x48,
} baro_pressure_accuracy_t;

// D2
typedef enum {
    LOWEST_TEMP = 0x50,
    LOW_TEMP = 0x52,
    MED_TEMP = 0x54,
    HIGH_TEMP = 0x56,
    HIGHEST_TEMP = 0x58,
} baro_temp_accuracy_t;

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

typedef enum
{
  BARO_COMMS_OK       			 = 0x00,
  BARO_COMMS_ERROR    			 = 0x01,
  BARO_COMMS_BUSY    			 = 0x02,
  BARO_COMMS_TIMEOUT 			 = 0x03,
} baro_status_t;

// Make sure you use the correct precision value for each

typedef struct {
    float32_t temperature;
    float32_t pressure;
    baro_pressure_accuracy_t pressureAccuracy; // Use only the D1 values
    baro_temp_accuracy_t tempAccuracy; // Use only the D2 values
    baro_conversion_time_t convertTime;
    int32_t dT;
    int32_t firstTemp;
    uint16_t coefficients[6]; // [C1, C2, C3, C4, C5, C6]
} baro_handle_t;

baro_status_t initBarometer(spi_device_t* baroSPI,
				   	   	    baro_handle_t* baroHandle);

baro_status_t resetBarometer(spi_device_t* baroSPI);

baro_status_t getPROMData(spi_device_t* baroSPI,
						  baro_handle_t* baroHandle);

baro_status_t getCurrTempPressure(spi_device_t* baroSPI,
		                 	 	  baro_handle_t* baroHandle);

baro_status_t startPressureConversion(spi_device_t* baroSPI,
									  baro_handle_t* baroHandle);

baro_status_t startTemperatureConversion(spi_device_t* baroSPI,
										 baro_handle_t* baroHandle);

baro_status_t calculateTemp(spi_device_t* baroSPI,
							baro_handle_t* baroHandle);

baro_status_t calculatePress(spi_device_t* baroSPI,
							 baro_handle_t* baroHandle);



#endif




