#include "MS5611.h"

/**
 * Process to setup barometer:
 * 		1. Set tempAccuracy, pressureAccuracy, convertTime
 * 		2. Run initBarometer
 */

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

      baroHandle->coefficients[i] = (uint16_t) rxBuffer[0] << 8 |  rxBuffer[1];
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
 * - @p baroHandle->temperature (°C, float32_t)
 * - @p baroHandle->pressure (mbar, float32_t)
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
 *   - Pressure is in kPa (floating-point).
 *
 * @see MS5611 Datasheet for full details of the compensation algorithm.
 */
baro_status_t getCurrTempPressure(spi_device_t* baroSPI, baro_handle_t* baroHandle) {

	baro_status_t status;

    uint8_t readADCCommand[4] = {READ_ADC, 0, 0, 0};
    uint8_t digitalTempBuff[4] = {0, 0, 0, 0};
    uint8_t digitalPressBuff[4] = {0, 0, 0, 0};

    if ((status = SPI_Device_Transmit(baroSPI, &(baroHandle->tempAccuracy), 1, HAL_MAX_DELAY)) != BARO_COMMS_OK) {
    	return status;
    }

    HAL_Delay(baroHandle->convertTime);

    if ((status = SPI_Device_TransmitReceive(baroSPI, readADCCommand, digitalTempBuff, 4, HAL_MAX_DELAY)) != BARO_COMMS_OK) {
    	return status;
    }

    int32_t digitalTemp = ((uint32_t) digitalTempBuff[1] << 16) |
    					   ((uint32_t) digitalTempBuff[2] << 8)  |
						   ((uint32_t) digitalTempBuff[3]);

    if ((status = SPI_Device_Transmit(baroSPI, &(baroHandle->pressureAccuracy), 1, HAL_MAX_DELAY)) != BARO_COMMS_OK) {
    	return status;
    }

    HAL_Delay(baroHandle->convertTime);

    if ((status = SPI_Device_TransmitReceive(baroSPI, readADCCommand, digitalPressBuff, 4, HAL_MAX_DELAY)) != BARO_COMMS_OK) {
    	return status;
    }

    int32_t digitalPress = ((uint32_t) digitalPressBuff[1] << 16) |
    						((uint32_t) digitalPressBuff[2] << 8)  |
							((uint32_t) digitalPressBuff[3]);

    int32_t dT = digitalTemp - ((int32_t) baroHandle->coefficients[4] << 8);
    int32_t firstTemp = 2000 + (((int64_t) dT * baroHandle->coefficients[5]) >> 23);

    baroHandle->dT = dT;
    baroHandle->firstTemp = firstTemp;

    int64_t offset = ((int64_t) baroHandle->coefficients[1] << 16) + (((int64_t) baroHandle->coefficients[3] * dT) >> 7);
    int64_t sensitivity = ((int64_t) baroHandle->coefficients[0] << 15) + (((int64_t) baroHandle->coefficients[2] * dT) >> 8);

    if (firstTemp < 2000) {

        int32_t T2 = ((int64_t) dT * (int64_t) dT) >> 31;
        int32_t norm = firstTemp - 2000;
        int32_t norm2 = norm * norm;

        int64_t OFF2 = 5 * norm2 / 2;
        int64_t SENS2 = 5 * norm2 / 4;

        if (firstTemp < -1500) {

            norm = firstTemp + 1500;
            norm2 = norm * norm;

            OFF2 += 7 * norm2;
            SENS2 += 11 * norm2 / 2;
        }

        offset = offset - OFF2;
        sensitivity = sensitivity - SENS2;

        int32_t secondPress = ((((int64_t) digitalPress * sensitivity) >> 21) - offset) >> 15;
        int32_t secondTemp = firstTemp - T2;

        baroHandle->temperature = ((float32_t) secondTemp) / 100;
        baroHandle->pressure = ((float32_t) secondPress);

    } else {

        int32_t firstPress = ((((int64_t) digitalPress * sensitivity) >> 21) - offset) >> 15;
        baroHandle->temperature = ((float32_t) firstTemp) / 100;
        baroHandle->pressure = (float32_t) firstPress;

    }

    return BARO_COMMS_OK;

}

/**
 * @brief Start a pressure conversion on the barometric sensor.
 *
 * Sends the pressure conversion command to the barometer over SPI using the
 * configured pressure oversampling/accuracy setting stored in the baro handle.
 * The conversion is performed internally by the sensor and must complete
 * before the ADC value is read.
 *
 * @param[in] baroSPI     Pointer to the SPI device handle for the barometer.
 * @param[in] baroHandle  Pointer to the barometer handle containing the
 *                        pressure conversion command/accuracy setting.
 *
 * @return baro_status_t
 *         - BARO_COMMS_OK on success
 *         - Error code returned by SPI_Device_Transmit() on communication failure
 *
 * @note The required conversion delay depends on the selected pressure
 *       oversampling setting and must be respected before reading the ADC.
 */

baro_status_t startPressureConversion(spi_device_t* baroSPI, baro_handle_t* baroHandle) {

	baro_status_t status;

    if ((status = SPI_Device_Transmit(baroSPI, &(baroHandle->pressureAccuracy), 1, HAL_MAX_DELAY)) != BARO_COMMS_OK) {
    	return status;
    }

    return BARO_COMMS_OK;

}

/**
 * @brief Start a temperature conversion on the barometric sensor.
 *
 * Sends the temperature conversion command to the barometer over SPI using the
 * configured temperature oversampling/accuracy setting stored in the baro
 * handle. The conversion is performed internally by the sensor and must
 * complete before the ADC value is read.
 *
 * @param[in] baroSPI     Pointer to the SPI device handle for the barometer.
 * @param[in] baroHandle  Pointer to the barometer handle containing the
 *                        temperature conversion command/accuracy setting.
 *
 * @return baro_status_t
 *         - BARO_COMMS_OK on success
 *         - Error code returned by SPI_Device_Transmit() on communication failure
 *
 * @note The required conversion delay depends on the selected temperature
 *       oversampling setting and must be respected before reading the ADC.
 */
baro_status_t startTemperatureConversion(spi_device_t* baroSPI, baro_handle_t* baroHandle) {

	baro_status_t status;

    if ((status = SPI_Device_Transmit(baroSPI, &(baroHandle->tempAccuracy), 1, HAL_MAX_DELAY)) != BARO_COMMS_OK) {
    	return status;
    }

	return BARO_COMMS_OK;
}

/**
 * @brief Read and calculate the compensated temperature from the barometer.
 *
 * Reads the raw temperature ADC value from the barometer and computes the
 * first- and second-order compensated temperature using factory calibration
 * coefficients. Intermediate values required for pressure compensation
 * (dT and first-order temperature) are stored in the baro handle for later use.
 *
 * The resulting temperature is stored in the baro handle in degrees Celsius.
 *
 * @param[in] baroSPI     Pointer to the SPI device handle for the barometer.
 * @param[in,out] baroHandle
 *                        Pointer to the barometer handle containing calibration
 *                        coefficients and storage for computed values.
 *
 * @return baro_status_t
 *         - BARO_COMMS_OK on success
 *         - Error code returned by SPI_Device_TransmitReceive() on communication failure
 *
 * @note This function must be called after a temperature conversion has
 *       completed.
 * @note Second-order temperature compensation is applied for temperatures
 *       below 20 °C.
 * @note The equations behind all te calculations can be found in the MS5611 datasheet
 */
baro_status_t calculateTemp(spi_device_t* baroSPI, baro_handle_t* baroHandle) {

	baro_status_t status;
    uint8_t readADCCommand[4] = {READ_ADC, 0, 0, 0};
    uint8_t digitalTempBuff[4] = {0, 0, 0, 0};

    if ((status = SPI_Device_TransmitReceive(baroSPI, readADCCommand, digitalTempBuff, 4, HAL_MAX_DELAY)) != BARO_COMMS_OK) {
    	return status;
    }

    int32_t digitalTemp = ((uint32_t) digitalTempBuff[1] << 16) |
    					   ((uint32_t) digitalTempBuff[2] << 8)  |
						   ((uint32_t) digitalTempBuff[3]);


    int32_t dT = digitalTemp - ((int32_t) baroHandle->coefficients[4] << 8);
    int32_t firstTemp = 2000 + (((int64_t) dT * baroHandle->coefficients[5]) >> 23);

    baroHandle->dT = dT;
    baroHandle->firstTemp = firstTemp;

    if (firstTemp < 2000) {

        int32_t T2 = ((int64_t) dT * (int64_t) dT) >> 31;
        int32_t secondTemp = firstTemp - T2;

        baroHandle->temperature = ((float32_t) secondTemp) / 100;

    } else {
        baroHandle->temperature = ((float32_t) firstTemp) / 100;
    }

    return BARO_COMMS_OK;
}

/**
 * @brief Read and calculate the compensated pressure from the barometer.
 *
 * Reads the raw pressure ADC value from the barometer and computes the
 * first- and second-order compensated pressure using factory calibration
 * coefficients and previously computed temperature terms.
 *
 * The resulting pressure is stored in the baro handle in sensor output units
 * (typically Pascals, depending on sensor scaling).
 *
 * @param[in] baroSPI     Pointer to the SPI device handle for the barometer.
 * @param[in,out] baroHandle
 *                        Pointer to the barometer handle containing calibration
 *                        coefficients, temperature compensation terms, and
 *                        storage for the computed pressure.
 *
 * @return baro_status_t
 *         - BARO_COMMS_OK on success
 *         - Error code returned by SPI_Device_TransmitReceive() on communication failure
 *
 * @note This function must be called after:
 *       - A pressure conversion has completed, and
 *       - calculateTemp() has been called to compute temperature compensation terms.
 * @note Second-order compensation is applied for low-temperature operation.
 * @note The equations behind all te calculations can be found in the MS5611 datasheet
 */
baro_status_t calculatePress(spi_device_t* baroSPI, baro_handle_t* baroHandle) {

	baro_status_t status;
    uint8_t readADCCommand[4] = {READ_ADC, 0, 0, 0};
    uint8_t digitalPressBuff[4] = {0, 0, 0, 0};

    if ((status = SPI_Device_TransmitReceive(baroSPI, readADCCommand, digitalPressBuff, 4, HAL_MAX_DELAY)) != BARO_COMMS_OK) {
    	return status;
    }

    int32_t digitalPress =  ((uint32_t) digitalPressBuff[1] << 16) |
    						((uint32_t) digitalPressBuff[2] << 8)  |
							((uint32_t) digitalPressBuff[3]);

    int32_t dT = baroHandle->dT;
    int32_t firstTemp = baroHandle->firstTemp;

    int64_t offset = ((int64_t) baroHandle->coefficients[1] << 16) + (((int64_t) baroHandle->coefficients[3] * dT) >> 7);
    int64_t sensitivity = ((int64_t) baroHandle->coefficients[0] << 15) + (((int64_t) baroHandle->coefficients[2] * dT) >> 8);

    if (firstTemp < 2000) {

        int32_t norm = firstTemp - 2000;
        int32_t norm2 = norm * norm;

        int64_t OFF2 = 5 * norm2 / 2;
        int64_t SENS2 = 5 * norm2 / 4;

        if (firstTemp < -1500) {

            norm = firstTemp + 1500;
            norm2 = norm * norm;

            OFF2 += 7 * norm2;
            SENS2 += 11 * norm2 / 2;
        }

        offset = offset - OFF2;
        sensitivity = sensitivity - SENS2;

        int32_t secondPress = ((((int64_t) digitalPress * sensitivity) >> 21) - offset) >> 15;
        baroHandle->pressure = ((float32_t) secondPress);

    } else {

        int32_t firstPress = ((((int64_t) digitalPress * sensitivity) >> 21) - offset) >> 15;
        baroHandle->pressure = (float32_t) firstPress;

    }

    return BARO_COMMS_OK;
}

