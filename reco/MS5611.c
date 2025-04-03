#include "MS5611.h"

void resetBarometer(spi_device_t* baroSPI) {
  	uint8_t baroReset = BARO_RESET;
    SPI_Device_Transmit(baroSPI, &baroReset, 1, HAL_MAX_DELAY);
    return;
}

void getPROMData(spi_device_t* baroSPI, baro_handle_t* baroHandle) {

    uint8_t PROM_COMMAND = PROM_READ + 2;
    uint8_t rxBuffer[2] = {0, 0};

    for (int i = 0; i < 6; i++) {
      HAL_Delay(1);
      SPI_Device_TransmitReceiveSeparate(baroSPI, &PROM_COMMAND, rxBuffer, 1, 2, HAL_MAX_DELAY);
      baroHandle->coefficients[i] = (uint16_t) rxBuffer[0] << 8 | rxBuffer[1];
      PROM_COMMAND += 2;
    }

	/*
    uint8_t PROM_COMMAND[3] = {PROM_READ + 2, 0xFF, 0xFF};
    uint8_t rxBuffer[3] = {0, 0, 0};

    for (int i = 0; i < 6; i++) {
      SPI_Device_TransmitReceive(baroSPI, PROM_COMMAND, rxBuffer, 3, HAL_MAX_DELAY);
      baroHandle->coefficients[i] = (uint16_t) rxBuffer[1] << 8 | rxBuffer[2];
      PROM_COMMAND[0] += 2;
    }
    */

    /*
    C5: a << 8,
    C6: b >> 23
    C2: c << 16
    C4: d >> 7
    C1: e << 15
    C3: f >> 8
    */

    return;
}

void initBarometer(spi_device_t* baroSPI, baro_handle_t* baroHandle) {
    resetBarometer(baroSPI);
    HAL_Delay(1);
    getPROMData(baroSPI, baroHandle);

    /*
    baroHandle->coefficients[0] = baroHandle->coefficients[0] << 8;
    baroHandle->coefficients[1] = baroHandle->coefficients[1];
    baroHandle->coefficients[2] = baroHandle->coefficients[2];
    baroHandle->coefficients[3] = baroHandle->coefficients[3];
    baroHandle->coefficients[4] = baroHandle->coefficients[4];
    baroHandle->coefficients[5] = baroHandle->coefficients[5];
    */

    return;
}

// To be implemented
void getTemp(spi_device_t* baroSPI, baro_handle_t* baroHandle, TIM_HandleTypeDef htim) {

    uint8_t readADCCommand[4] = {READ_ADC, 0xFF, 0xFF, 0xFF};
    uint8_t digitalTempBuff[4] = {0};

    SPI_Device_Transmit(baroSPI, &(baroHandle->tempAccuracy), 1, HAL_MAX_DELAY);
    uint16_t startTime = __HAL_TIM_GET_COUNTER(&htim);
    //printf("Start Time: %d", startTime);

    while ((__HAL_TIM_GET_COUNTER(&htim) - startTime) < 6) {
    	//printf("Counter Time: %d", __HAL_TIM_GET_COUNTER(&htim));
    	continue;
    }

    SPI_Device_TransmitReceive(baroSPI, readADCCommand, digitalTempBuff, 4, HAL_MAX_DELAY);

    uint32_t digitalTemp = (uint32_t) (digitalTempBuff[1] << 16) | (digitalTempBuff[2] << 8) | (digitalTempBuff[3] << 0);
    int64_t dT = digitalTemp - ((baroHandle->coefficients[4] << 8));
    int32_t firstTemp = 2000 + (dT * (int64_t) baroHandle->coefficients[5]) / (1 << 23);

    if (firstTemp >= 2000) {
            baroHandle->temperature = ((double) firstTemp) / 100;
            baroHandle->dT = dT;
            baroHandle->firstTemp = firstTemp;
            return;
     }

    int32_t T2 = ((dT * dT) >> 31);
    int32_t secondTemp = firstTemp - T2;

    baroHandle->temperature = ((double) secondTemp) / 100;
    baroHandle->dT = dT;
    baroHandle->firstTemp = firstTemp;

    printf("Exit\n");
    return;

}

void startTempConversion(spi_device_t* baroSPI, baro_handle_t* baroHandle) {

    SPI_Device_Transmit(baroSPI, &(baroHandle->tempAccuracy), 1, HAL_MAX_DELAY);

}


// To be implemented
void getPressure(spi_device_t* baroSPI, baro_handle_t* baroHandle) {

    uint8_t readADCCommand[4] = {READ_ADC, 0xFF, 0xFF, 0xFF};
    uint8_t digitalPressBuff[4] = {0};

    SPI_Device_TransmitReceive(baroSPI, readADCCommand, digitalPressBuff, 4, HAL_MAX_DELAY);

    uint32_t digitalPress = (uint32_t) (digitalPressBuff[1] << 16) | (digitalPressBuff[2] << 8) | (digitalPressBuff[3] << 0);

	int64_t dT = baroHandle->dT;
	int32_t firstTemp = baroHandle->firstTemp;

	int64_t offset = ((int64_t) baroHandle->coefficients[1] << 16) + (((int64_t) baroHandle->coefficients[3] * dT) >> 7);
    int64_t sensitivity = ((int64_t) baroHandle->coefficients[0] << 15) + (((int64_t) baroHandle->coefficients[2] * dT) >> 8);

    if (baroHandle->temperature >= 20.00) {
    	int32_t firstPress = ((( (int64_t) digitalPress * (sensitivity >> 21)) - offset) >> 15);
        baroHandle->pressure = ((double) firstPress) / 1000;
        return;
    }

    int32_t T2 = ((dT * dT) >> 31);
    int64_t OFF2 = 5 * ((firstTemp - 2000) * (firstTemp - 2000)) / 2;
    int64_t SENS2 = 5 * ((firstTemp - 2000) * (firstTemp - 2000)) / 4;

    if (firstTemp < -15) {
        OFF2 = OFF2 + 7 * ((firstTemp + 1500) * (firstTemp + 1500));
        SENS2 = SENS2 + 11 * ((firstTemp + 1500) * (firstTemp + 1500)) / 2;
    }

    offset = offset - OFF2;
    sensitivity = sensitivity - SENS2;

    int32_t secondPress = (digitalPress * (sensitivity >> 21) - offset) >> 15;
    baroHandle->pressure = ((double) secondPress / 1000);
}

void startPressureConversion(spi_device_t* baroSPI, baro_handle_t* baroHandle) {

    SPI_Device_Transmit(baroSPI, &(baroHandle->pressureAccuracy), 1, HAL_MAX_DELAY);

    return;
}

/*
 * Returns pressure in kPa
 * Returns temperature in Celcius
 */
void getCurrTempPressure(spi_device_t* baroSPI, baro_handle_t* baroHandle) {

    uint8_t readADCCommand[4] = {READ_ADC, 0xFF, 0xFF, 0xFF};
    uint8_t digitalTempBuff[4] = {0};
    uint8_t digitalPressBuff[4] = {0};

    SPI_Device_Transmit(baroSPI, &(baroHandle->tempAccuracy), 1, HAL_MAX_DELAY);
    HAL_Delay(baroHandle->convertTime);

    SPI_Device_TransmitReceive(baroSPI, readADCCommand, digitalTempBuff, 4, HAL_MAX_DELAY);

    uint32_t digitalTemp = (uint32_t) (digitalTempBuff[1] << 16) | (digitalTempBuff[2] << 8) | (digitalTempBuff[3] << 0);

    SPI_Device_Transmit(baroSPI, &(baroHandle->pressureAccuracy), 1, HAL_MAX_DELAY);
    HAL_Delay(baroHandle->convertTime);

    SPI_Device_TransmitReceive(baroSPI, readADCCommand, digitalPressBuff, 4, HAL_MAX_DELAY);

    uint32_t digitalPress = (uint32_t) (digitalPressBuff[1] << 16) | (digitalPressBuff[2] << 8) | (digitalPressBuff[3] << 0);

    int64_t dT = digitalTemp - ((baroHandle->coefficients[4] << 8));
    int32_t firstTemp = 2000 + (dT * (int64_t) baroHandle->coefficients[5]) / (1 << 23);

    int64_t offset = ((int64_t) baroHandle->coefficients[1] << 16) + (((int64_t) baroHandle->coefficients[3] * dT) >> 7);
    int64_t sensitivity = ((int64_t) baroHandle->coefficients[0] << 15) + (((int64_t) baroHandle->coefficients[2] * dT) >> 8);

    if (firstTemp >= 2000) {
    	int32_t firstPress = ((( (int64_t) digitalPress * (sensitivity >> 21)) - offset) >> 15);
        baroHandle->temperature = ((double) firstTemp) / 100;
        baroHandle->pressure = ((double) firstPress) / 1000;
        baroHandle->dT = dT;
        baroHandle->firstTemp = firstTemp;
        return;
    }

    int32_t T2 = ((dT * dT) >> 31);
    int64_t OFF2 = 5 * ((firstTemp - 2000) * (firstTemp - 2000)) / 2;
    int64_t SENS2 = 5 * ((firstTemp - 2000) * (firstTemp - 2000)) / 4;

    if (firstTemp < -15) {
        OFF2 = OFF2 + 7 * ((firstTemp + 1500) * (firstTemp + 1500));
        SENS2 = SENS2 + 11 * ((firstTemp + 1500) * (firstTemp + 1500)) / 2;
    }

    offset = offset - OFF2;
    sensitivity = sensitivity - SENS2;

    volatile int32_t secondPress = (digitalPress * (sensitivity >> 21) - offset) >> 15;
    volatile int32_t secondTemp = firstTemp - T2;

    baroHandle->temperature = ((double) secondTemp) / 100;
    baroHandle->pressure = ((double) secondPress) / 1000;
    baroHandle->dT = dT;
    baroHandle->firstTemp = firstTemp;
    return;

}
