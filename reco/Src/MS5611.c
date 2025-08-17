#include "MS5611.h"

/**
 * @brief Resets the MS5611 barometer.
 *
 * Sends the reset command to the MS5611 via SPI, forcing the sensor
 * into a known startup state. After reset, the PROM and calibration
 * coefficients should be re-read before taking measurements.
 *
 * @param[in] baroSPI Pointer to the SPI device handle used for communication with the barometer.
 *
 * @return baro_status_t
 *         - @p BARO_COMMS_OK if the reset command was transmitted successfully
 *         - Error status (e.g., SPI failure) otherwise
 *
 * @note The MS5611 requires a delay (typically >2.8 ms) after reset
 *       before it is ready to accept further commands.
 */
baro_status_t resetBarometer(spi_device_t* baroSPI) {
  	uint8_t baroReset = BARO_RESET;
    return SPI_Device_Transmit(baroSPI, &baroReset, 1, HAL_MAX_DELAY);
}

/**
 * @brief Reads the factory PROM calibration coefficients from the MS5611.
 *
 * Sends successive PROM read commands to retrieve the six calibration coefficients
 * required for temperature and pressure calculations. The coefficients are stored
 * in the provided barometer handle.
 *
 * @param[in]  baroSPI     Pointer to the SPI device handle used for communication with the barometer.
 * @param[out] baroHandle  Pointer to the barometer handle structure where the PROM coefficients are stored.
 *
 * @return baro_status_t
 *         - @p BARO_COMMS_OK if the coefficients were read successfully
 *         - Error status (e.g., SPI failure) otherwise
 *
 * @note A short delay is inserted between each PROM read to ensure proper sensor response.
 * @see MS5611 Datasheet for details on PROM coefficients (C1–C6).
 */
baro_status_t getPROMData(spi_device_t* baroSPI, baro_handle_t* baroHandle) {

	baro_status_t status;
    uint8_t PROM_COMMAND = PROM_READ + 2;
    uint8_t rxBuffer[2] = {0, 0};

    for (int i = 0; i < 6; i++) {
      HAL_Delay(1);

      if ((status = SPI_Device_TransmitReceiveSeparate(baroSPI, &PROM_COMMAND, rxBuffer, 1, 2, HAL_MAX_DELAY)) != BARO_COMMS_OK) {
    	  return status;
      }

      baroHandle->coefficients[i] = (uint16_t) rxBuffer[0] << 8 | rxBuffer[1];
      PROM_COMMAND += 2;
    }

    return BARO_COMMS_OK;
}


/**
 * @brief Initializes the MS5611 barometer.
 *
 * Performs the full initialization sequence for the MS5611:
 * - Resets the sensor
 * - Reads the PROM calibration coefficients into the provided barometer handle
 *
 * @param[in]  baroSPI     Pointer to the SPI device handle used for communication with the barometer.
 * @param[out] baroHandle  Pointer to the barometer handle structure where the calibration coefficients
 *                         will be stored.
 *
 * @return baro_status_t
 *         - @p BARO_COMMS_OK if initialization succeeded
 *         - Error status (e.g., SPI failure) otherwise
 *
 * @note This function must be called before performing any temperature or pressure measurements.
 */
baro_status_t initBarometer(spi_device_t* baroSPI, baro_handle_t* baroHandle) {

	baro_status_t status;

    if ((status = resetBarometer(baroSPI)) != BARO_COMMS_OK) {
    	return status;
    }

    HAL_Delay(1);

    if ((status = getPROMData(baroSPI, baroHandle)) != BARO_COMMS_OK) {
    	return status;
    }

    return BARO_COMMS_OK;
}

/**
 * @brief Reads and calculates the current temperature and pressure from the MS5611 barometer.
 *
 * This function triggers conversions for both temperature and pressure using the
 * MS5611 barometer via SPI, retrieves the raw ADC values, and then applies the
 * calibration coefficients stored in the provided @p baroHandle. It performs
 * first-order and, if required, second-order temperature compensation (as specified
 * in the MS5611 datasheet) to compute accurate temperature (°C) and pressure (mbar).
 *
 * The compensated values are written back into the @p baroHandle structure:
 * - @p baroHandle->temperature (°C, float)
 * - @p baroHandle->pressure (mbar, float)
 *
 * @param[in]  baroSPI     Pointer to the SPI device handle used for communication
 *                         with the barometer.
 * @param[in,out] baroHandle Pointer to a barometer handle structure. Must contain:
 *                           - Calibration coefficients loaded from PROM
 *                           - Conversion accuracy commands (tempAccuracy, pressureAccuracy)
 *                           - Conversion delay time (convertTime)
 *                           On return, this structure is updated with the latest
 *                           temperature and pressure values.
 *
 * @return baro_status_t
 *         - @p BARO_COMMS_OK if communication and compensation succeeded
 *         - Error status (e.g., SPI failure) otherwise
 *
 * @note
 * - The function performs blocking SPI transfers and uses HAL_Delay() for conversion wait times.
 * - Second-order temperature compensation is applied when the first-order
 *   computed temperature is below 20°C, as required by the MS5611.
 * - Results are scaled as follows:
 *   - Temperature is in °C (floating-point).
 *   - Pressure is in mbar (floating-point).
 *
 * @see MS5611 Datasheet for full details of the compensation algorithm.
 */
baro_status_t getCurrTempPressure(spi_device_t* baroSPI, baro_handle_t* baroHandle) {

	baro_status_t status;

    uint8_t readADCCommand[4] = {READ_ADC, 0xFF, 0xFF, 0xFF};
    uint8_t digitalTempBuff[4] = {0};
    uint8_t digitalPressBuff[4] = {0};

    if ((status = SPI_Device_Transmit(baroSPI, &(baroHandle->tempAccuracy), 1, HAL_MAX_DELAY)) != BARO_COMMS_OK) {
    	return status;
    }

    HAL_Delay(baroHandle->convertTime);

    if ((status = SPI_Device_TransmitReceive(baroSPI, readADCCommand, digitalTempBuff, 4, HAL_MAX_DELAY)) != BARO_COMMS_OK) {
    	return status;
    }

    uint32_t digitalTemp = ((uint32_t) digitalTempBuff[1] << 16) |
    					   ((uint32_t) digitalTempBuff[2] << 8)  |
						   ((uint32_t) digitalTempBuff[3]);

    if ((status = SPI_Device_Transmit(baroSPI, &(baroHandle->pressureAccuracy), 1, HAL_MAX_DELAY)) != BARO_COMMS_OK) {
    	return status;
    }

    HAL_Delay(baroHandle->convertTime);

    if ((status = SPI_Device_TransmitReceive(baroSPI, readADCCommand, digitalPressBuff, 4, HAL_MAX_DELAY)) != BARO_COMMS_OK) {
    	return status;
    }

    uint32_t digitalPress = ((uint32_t) digitalPressBuff[1] << 16) |
    						((uint32_t) digitalPressBuff[2] << 8)  |
							((uint32_t) digitalPressBuff[3]);

    int32_t dT = digitalTemp - (baroHandle->coefficients[4] << 8);
    int32_t firstTemp = 2000 + (((int64_t) dT * baroHandle->coefficients[5]) >> 23);

    int64_t offset = ((int64_t) baroHandle->coefficients[1] << 16) + (((int64_t) baroHandle->coefficients[3] * dT) >> 7);
    int64_t sensitivity = ((int64_t) baroHandle->coefficients[0] << 15) + (((int64_t) baroHandle->coefficients[2] * dT) >> 8);

    if (firstTemp < 2000) {

        int32_t T2 = (dT * dT) >> 31;
        int64_t OFF2 = 5 * ((firstTemp - 2000) * (firstTemp - 2000)) / 2;
        int64_t SENS2 = 5 * ((firstTemp - 2000) * (firstTemp - 2000)) / 4;

        if (firstTemp < -1500) {
            OFF2 = OFF2 + 7 * ((firstTemp + 1500) * (firstTemp + 1500));
            SENS2 = SENS2 + 11 * ((firstTemp + 1500) * (firstTemp + 1500)) / 2;
        }

        offset = offset - OFF2;
        sensitivity = sensitivity - SENS2;

        int32_t secondPress = (( (int64_t) digitalPress * (sensitivity >> 21)) - offset) >> 15;
        int32_t secondTemp = firstTemp - T2;

        baroHandle->temperature = ((float) secondTemp) / 100;
        baroHandle->pressure = ((float) secondPress) / 1000;

    } else {

        int32_t firstPress = (( (int64_t) digitalPress * (sensitivity >> 21)) - offset) >> 15;
        baroHandle->temperature = ((float) firstTemp) / 100;
        baroHandle->pressure = ((float) firstPress) / 1000;

    }

    return BARO_COMMS_OK;

}
