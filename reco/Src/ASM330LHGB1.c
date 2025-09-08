#include "ASM330LHGB1.h"

// Hash of reserved registers where each index represents the register number
static const uint8_t IMU_RESERVED_REG_HASH[] = {1, 0, 0, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0,
		 	 	 	 	 	 	 	 	 	 	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
												0, 0, 1, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0,
												0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1,
												1, 0, 0, 0, 0, 1, 0, 0, 1, 1, 1, 1, 0,
												0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
												1, 1, 1, 1, 1, 1, 1, 1, 0, 1, 0, 0, 1,
												0, 0, 0, 0, 0, 1, 1, 0, 0, 1, 1, 1, 1,
												1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0,
												0, 1, 1, 0, 0, 0, 0, 0, 0, 0};

// Hash of writeable registers where each index represents the register number
static const uint8_t IMU_WRITEABLE_REG_HASH[] = {0, 1, 1, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1,
		  	  	  	  	  	  	  	  	  	  	 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
												 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
												 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
												 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
												 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
												 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 1, 1, 0,
												 1, 1, 1, 1, 1, 0, 0, 1, 0, 0, 0, 0, 0,
												 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1,
												 1, 0, 0, 0, 0, 0, 0, 0, 0, 0};

// Array of all ctrl_reg numbers
static const uint8_t CTRL_REG_NUM_IMU[] = {IMU_PIN_CTRL, IMU_CTRL1_XL, IMU_CTRL2_G, IMU_CTRL3_C, IMU_CTRL4_C, IMU_CTRL5_C,
								       IMU_CTRL6_C, IMU_CTRL7_G, IMU_CTRL8_XL, IMU_CTRL9_XL, IMU_CTRL10_C};

static const uint8_t PIN_CTRL_MASK = (uint8_t) ~((1 << 5) | (1 << 4) | (1 << 3) | (1 << 2) | (1 << 1) | (1 << 0));
static const uint8_t CTRL1_XL_MASK = (uint8_t) ~((1 << 0));
static const uint8_t CTRL3_C_MASK  = (uint8_t) ~((1 << 1));
static const uint8_t CTRL4_C_MASK  = (uint8_t) ~((1 << 0) | (1 << 4) | (1 << 7));
static const uint8_t CTRL5_C_MASK  = (uint8_t) ~((1 << 7) | (1 << 4));
static const uint8_t CTRL7_G_MASK  = (uint8_t) ~((1 << 0) | (1 << 2) | (1 << 3));
static const uint8_t CTRL8_XL_MASK = (uint8_t) ~((1 << 1));
static const uint8_t CTRL9_XL_MASK = (uint8_t) ~((1 << 0));

// This itself is the mask it doesn't need to be inverted. THe 0xFFFF is for registers where all bits are used.
static const uint8_t CTRL10_C_MASK = (uint8_t) (1 << 5);
static const uint8_t CTRL_REG_IMU_MASK[] = {PIN_CTRL_MASK, CTRL1_XL_MASK, 0xFF, CTRL3_C_MASK, CTRL4_C_MASK,
									   CTRL5_C_MASK, 0xFF, CTRL7_G_MASK, CTRL8_XL_MASK, CTRL9_XL_MASK,
									   CTRL10_C_MASK};

/* Sensitivity Values */
// Linear Acceleration (m/s^2)
static const float ACCEL_SENS_2G  = 0.061f / 1000.0f * 9.80665f;
static const float ACCEL_SENS_4G  = 0.122f / 1000.0f * 9.80665f;
static const float ACCEL_SENS_8G  = 0.244f / 1000.0f * 9.80665f;
static const float ACCEL_SENS_16G = 0.488f / 1000.0f * 9.80665f;

// Angular Velocity (milidegrees/sec)
static const float GYRO_SENS_125  = 4.37f / 1000.0f;
static const float GYRO_SENS_250  = 8.75f / 1000.0f;
static const float GYRO_SENS_500  = 17.5f / 1000.0f;
static const float GYRO_SENS_1000 = 35.0f / 1000.0f;
static const float GYRO_SENS_2000 = 70.0f / 1000.0f;
static const float GYRO_SENS_4000 = 140.0f / 1000.0f;


/**
 * @brief  Generate an IMU register address with optional read flag.
 *
 * This function masks the register number to 7 bits and sets the MSB if the
 * read flag is enabled. It ensures the returned address conforms to the IMU's
 * SPI/I²C addressing format.
 *
 * @param[in]  imuRegNum  Register number (lower 7 bits are used).
 * @param[in]  readFlag   If true, the read bit (bit 7) is set in the address.
 *
 * @return The formatted 8-bit register address with the read flag encoded.
 */
uint8_t generateIMUAddress(imu_reg_t imuRegNum, bool readFlag) {

  uint8_t newAddress = imuRegNum & 0x7F;

  if (readFlag) {
    newAddress |= (1 << 7);
  }

  return newAddress;
}

/**
 * @brief  Write a single value to an IMU register.
 *
 * @param[in]  imuSPI        Pointer to the SPI device structure used for communication.
 * @param[in]  imuRegNum     Register number to write to.
 * @param[in]  valueToWrite  Byte value to write into the register.
 *
 * @retval status  The status of the SPI communication.
 *
 * @see imu_status_t  Definition of the possible status codes.
 */

imu_status_t writeIMUSingleRegister(spi_device_t* imuSPI, imu_reg_t imuRegNum, uint8_t valueToWrite) {

	if (imuRegNum < IMU_MIN_REG || IMU_MAX_REG < imuRegNum || !IMU_WRITEABLE_REG_HASH[imuRegNum]) {
		return IMU_INVALID_REG;
	}

	uint8_t actualRegNumber = generateIMUAddress(imuRegNum, false);
	uint8_t command[] = {actualRegNumber, valueToWrite};

	imu_status_t status = SPI_Device_Transmit(imuSPI, command, 2, HAL_MAX_DELAY);

	return status;
}

/**
 * @brief  Read a single value from an IMU register.
 *
 *
 * @param[in]   imuSPI        Pointer to the SPI device structure used for communication.
 * @param[in]   imuRegNum     Register number to read from.
 * @param[out]  receivedData  Pointer to a variable where the received byte will be stored.
 *
 * @retval status  The status of the SPI communication.
 *
 * @see imu_status_t  Definition of the possible status codes.
 */
imu_status_t readIMUSingleRegister(spi_device_t* imuSPI, imu_reg_t imuRegNum, uint8_t* receivedData) {

	if (imuRegNum < IMU_MIN_REG || IMU_MAX_REG < imuRegNum || IMU_RESERVED_REG_HASH[imuRegNum]) {
		return IMU_INVALID_REG;
	}

	uint8_t actualRegNumber = generateIMUAddress(imuRegNum, true);
	imu_status_t status = SPI_Device_TransmitReceiveSeparate(imuSPI, &actualRegNumber, receivedData, 1, 1, HAL_MAX_DELAY);

	return status;
}

/**
 * @brief  Read a 16-bit value from two consecutive IMU registers.
 *
 * This function reads two 8-bit registers (upper and lower) from the IMU
 * and combines them into a single 16-bit value. If either read operation
 * fails, the corresponding status code is returned immediately.
 *
 * @param[in]   imuSPI            Pointer to the SPI device structure used for communication.
 * @param[in]   upperRegAddress   Register address containing the upper 8 bits of the value.
 * @param[in]   lowerRegAddress   Register address containing the lower 8 bits of the value.
 * @param[out]  receivedData      Pointer to a variable where the combined 16-bit value will be stored.
 *
 * @retval status  The status of the SPI communication.
 *
 * @see imu_status_t  Definition of the possible status codes.
 */
imu_status_t readIMUDoubleRegister(spi_device_t* imuSPI, imu_reg_t upperRegAddress, imu_reg_t lowerRegAddress, uint16_t* receivedData) {

	uint8_t upper8;
	uint8_t lower8;
	imu_status_t status;

	if ((status = readIMUSingleRegister(imuSPI, upperRegAddress, &upper8)) != IMU_COMMS_OK) {
		return status;
	}

	if ((status = readIMUSingleRegister(imuSPI, lowerRegAddress, &lower8)) != IMU_COMMS_OK) {
		return status;
	}

	*receivedData = (uint16_t) upper8 << 8 | (uint16_t) lower8;
	return IMU_COMMS_OK;
}

/**
 * @brief Reads multiple consecutive registers from the IMU.
 *
 * @param[in] imuSPI
 *     Pointer to the SPI device structure associated with the IMU.
 * @param[in] startRegNum
 *     The first register in the consecutive range to read.
 * @param[in] endRegNum
 *     The last register in the consecutive range to read.
 * @param[out] regReadValues
 *     Pointer to an array that will store the values read from the
 *     specified register range. The array must be at least
 *     (@p endRegNum - @p startRegNum + 1) bytes long.
 *
 * @retval IMU_COMMS_OK
 *     All registers in the range were successfully read.
 * @retval IMU_INVALID_REG
 *     One or more registers in the specified range are invalid or reserved.
 * @retval status
 *     A non-OK SPI communication status code returned by HAL functions.
 *
 * @note
 *     The @p regReadValues array must be preallocated with sufficient size
 *     to hold all register values in the specified range, otherwise a buffer
 *     overflow will occur.
 *
 * @seealso imu_status_t, HAL_SPI_Transmit, HAL_SPI_Receive
 */
imu_status_t readIMUMultipleRegisters(spi_device_t* imuSPI, imu_reg_t startRegNum, imu_reg_t endRegNum, uint8_t* regReadValues) {

	imu_status_t status;

	if (startRegNum < IMU_MIN_REG) {
		return IMU_INVALID_REG;
	}

	if (endRegNum > IMU_MAX_REG) {
		return IMU_INVALID_REG;
	}

	// Check that all registers within the range of the starting register (startRegNum) and the ending register
	// (endRegNum) are all readable. Return the invalid register status code if not.
	for (uint8_t imuRegNum = startRegNum; imuRegNum <= endRegNum; imuRegNum++) {
		if (IMU_RESERVED_REG_HASH[imuRegNum]) {
			return IMU_INVALID_REG;
		}
	}

	// Generate the starting address that indicating consecutive reads and calculate the total number of registers
	// that will be written to.
	uint8_t startingRegAddr = generateIMUAddress(startRegNum, true);
	uint8_t numRegRead = endRegNum - startRegNum + 1;

	// Pull CS line low to start transmission
    HAL_GPIO_WritePin(imuSPI->GPIO_Port, imuSPI->GPIO_Pin, GPIO_PIN_RESET);

    // Send the starting register address to the IMU and ensure that the communication is ok.
	if ((status = HAL_SPI_Transmit(imuSPI->hspi, &startingRegAddr, 1, HAL_MAX_DELAY)) != IMU_COMMS_OK) {
	    HAL_GPIO_WritePin(imuSPI->GPIO_Port, imuSPI->GPIO_Pin, GPIO_PIN_SET);
		return status;
	}

    // Read the value from each register from startReg to endReg inclusive per 8 SPI clock cycles.
	if ((status = HAL_SPI_Receive(imuSPI->hspi, regReadValues, numRegRead, HAL_MAX_DELAY)) != IMU_COMMS_OK) {
	    HAL_GPIO_WritePin(imuSPI->GPIO_Port, imuSPI->GPIO_Pin, GPIO_PIN_SET);
	    return status;
	}

    // Pull CS line high to end transmission
    HAL_GPIO_WritePin(imuSPI->GPIO_Port, imuSPI->GPIO_Pin, GPIO_PIN_SET);

    return IMU_COMMS_OK;
}


/**
 * @brief	Writes a sequence of consecutive IMU registers with the provided values.
 *
 * @param[in] imuSPI
 *     Pointer to the SPI device structure used for communication with the IMU.
 * @param[in] startRegNum
 *     The first register in the consecutive range to write to.
 * @param[in] endRegNum
 *     The last register in the consecutive range to write to.
 * @param[in] valuesToWrite
 *     Pointer to an array containing the values to be written. The array
 *     length must be at least (endRegNum - startRegNum + 1).
 *
 * @retval IMU_COMMS_OK
 *     The registers were successfully written.
 * @retval IMU_INVALID_REG
 *     One or more registers in the specified range are invalid or not writable.
 * @retval status
 *     The status of the SPI communication. Returned if a transmission error occurs.
 *
 * @note
 *     The size of the @p valuesToWrite array must be such that it can hold values
 *     that will be written from startRegNum to endRegNum inclusive. If not, a buffer overflow
 *     will occur
 *
 * @seealso
 *     imu_status_t for possible status code definitions.
 */
imu_status_t writeIMUMultipleRegisters(spi_device_t* imuSPI, imu_reg_t startRegNum, imu_reg_t endRegNum, uint8_t* valuesToWrite) {

	imu_status_t status;

	// Check that all registers within the range of the starting register (startRegNum) and the ending register
	// (endRegNum) are all writeable. Return the invalid register status code if not to be handled by caller.

	if (startRegNum < IMU_MIN_REG) {
		return IMU_INVALID_REG;
	}

	if (endRegNum > IMU_MAX_REG) {
		return IMU_INVALID_REG;
	}

	for (uint8_t imuRegNum = startRegNum; imuRegNum <= endRegNum; imuRegNum++) {
		if (!IMU_WRITEABLE_REG_HASH[imuRegNum]) {
			return IMU_INVALID_REG;
		}
	}

	// Generate the starting address that indicating consecutive writes and calculate the total number of registers
	// that will be written to.
	uint8_t startingRegAddr = generateIMUAddress(startRegNum, false);
	uint8_t numRegWrite = endRegNum - startRegNum + 1;

	// Pull CS line low to start transmission
    HAL_GPIO_WritePin(imuSPI->GPIO_Port, imuSPI->GPIO_Pin, GPIO_PIN_RESET);

    // Send the starting register address to the IMU and ensure that the communication is ok.
    if ((status = HAL_SPI_Transmit(imuSPI->hspi, &startingRegAddr, 1, HAL_MAX_DELAY)) != IMU_COMMS_OK) {
        HAL_GPIO_WritePin(imuSPI->GPIO_Port, imuSPI->GPIO_Pin, GPIO_PIN_SET);
        return status;
    }

    // Send the value that you want to be written into each register per 8 SPI clock cycles.
    if ((status = HAL_SPI_Transmit(imuSPI->hspi, valuesToWrite, numRegWrite, HAL_MAX_DELAY)) != IMU_COMMS_OK) {
        HAL_GPIO_WritePin(imuSPI->GPIO_Port, imuSPI->GPIO_Pin, GPIO_PIN_SET);
        return status;
    }

    // Pull CS line high to end transmission
    HAL_GPIO_WritePin(imuSPI->GPIO_Port, imuSPI->GPIO_Pin, GPIO_PIN_SET);

    return IMU_COMMS_OK;
}

/**
 * @brief Initializes the IMU registers based on the handler configuration.
 *
 * This function iterates through all control registers specified by
 * the IMU handler. For registers marked as modified, it writes the
 * corresponding value to the IMU. For unmodified registers, it reads
 * the current register values into the handler. After all registers
 * are synchronized, it sets the linear acceleration and angular rate
 * sensitivity values based on the full-scale settings in the control registers.
 *
 * @param[in] imuSPI
 *     Pointer to the SPI device structure used for communication with the IMU.
 * @param[in,out] imuHandler
 *     Pointer to the IMU handler structure containing register configuration.
 *     This structure will be updated with current register values and computed
 *     sensitivity factors.
 *
 * @retval IMU_COMMS_OK
 *     Initialization completed successfully.
 * @retval IMU_COMMS_ERROR or other imu_status_t values
 *     A SPI communication error occurred while reading or writing registers.
 *
 * @see imu_status_t, writeIMURegister, readIMUSingleRegister
 */

imu_status_t initializeIMU(spi_device_t* imuSPI, imu_handler_t* imuHandler) {

	imu_status_t status;
	uint8_t* rawReg = (uint8_t*) &imuHandler->pin_ctrl;

	for (int currRegIdx = 0; currRegIdx < IMU_CTRL_REG_NUM; currRegIdx++) {
		if (imuHandler->modifiedRegisters[currRegIdx]) {
			*rawReg &= CTRL_REG_IMU_MASK[currRegIdx];

			if ((status = writeIMUSingleRegister(imuSPI, CTRL_REG_NUM_IMU[currRegIdx], *rawReg)) != IMU_COMMS_OK) {
				return status;
			}

		} else {
			if ((status = readIMUSingleRegister(imuSPI, CTRL_REG_NUM_IMU[currRegIdx], rawReg)) != IMU_COMMS_OK) {
				return status;
			}
		}
		rawReg++;
	}

	// Set linear acceleration based on values of the linear full scale bits in the ctrl1_xl register
	switch (imuHandler->ctrl1_xl.flags.FS_XL) {
		case IMU_ACCEL_FS_XL_2G:
			imuHandler->accelSens = ACCEL_SENS_2G;
			break;
		case IMU_ACCEL_FS_XL_4G:
			imuHandler->accelSens = ACCEL_SENS_4G;
			break;
		case IMU_ACCEL_FS_XL_8G:
			imuHandler->accelSens = ACCEL_SENS_8G;
			break;
		case IMU_ACCEL_FS_XL_16G:
			imuHandler->accelSens = ACCEL_SENS_16G;
			break;
	}

	// Set the angular velocity bits based on the values in the angular full scale bits in the ctrl2_g register
	if (imuHandler->ctrl2_g.flags.FS_4000) {
		imuHandler->angularRateSens = GYRO_SENS_4000;
	} else if (imuHandler->ctrl2_g.flags.FS_125) {
		imuHandler->angularRateSens = GYRO_SENS_125;
	} else {

		switch (imuHandler->ctrl2_g.flags.FS_G) {
			case IMU_GYRO_250_DPS:
				imuHandler->angularRateSens = GYRO_SENS_250;
				break;
			case IMU_GYRO_500_DPS:
				imuHandler->angularRateSens = GYRO_SENS_500;
				break;
			case IMU_GYRO_1000_DPS:
				imuHandler->angularRateSens = GYRO_SENS_1000;
				break;
			case IMU_GYRO_2000_DPS:
				imuHandler->angularRateSens = GYRO_SENS_2000;
				break;
		}

	}

	return IMU_COMMS_OK;
}

/**
 * @brief Reads the IMU's pitch (angular velocity around the X-axis).
 *
 * This function reads the raw 16-bit pitch value from the IMU's gyro output
 * registers, converts it to a signed integer, and multiplies it by the
 * angular rate sensitivity factor stored in the IMU handler to produce the
 * pitch in milli-degrees per second.
 *
 * @param[in]   imuSPI
 *     Pointer to the SPI device structure used for communication with the IMU.
 * @param[in]   imuHandler
 *     Pointer to the IMU handler structure containing sensitivity settings.
 * @param[out]  pitchOutput
 *     Pointer to a float variable where the computed pitch rate  value will be stored
 *     in milli-degrees per second.
 *
 * @retval status
 *     The status of the SPI communication, as returned by readIMUDoubleRegister().
 *
 * @see imu_status_t, readIMUDoubleRegister
 */

imu_status_t getPitchRate(spi_device_t* imuSPI, imu_handler_t* imuHandler, float* pitchOutput) {
	uint16_t pitchRaw;
	imu_status_t status = readIMUDoubleRegister(imuSPI, IMU_OUTX_H_G, IMU_OUTX_L_G, &pitchRaw);
	*pitchOutput = ((int16_t) pitchRaw) * imuHandler->angularRateSens; // milidegrees/sec
	return status;
}

/**
 * @brief Reads the IMU's roll (angular velocity around the Y-axis).
 *
 * @param[in]   imuSPI
 *     Pointer to the SPI device structure used for communication with the IMU.
 * @param[in]   imuHandler
 *     Pointer to the IMU handler structure containing sensitivity settings.
 * @param[out]  rollOutput
 *     Pointer to a float variable where the computed roll rate value will be stored
 *     in milli-degrees per second.
 *
 * @retval status
 *     The status of the SPI communication, as returned by readIMUDoubleRegister().
 *
 * @see imu_status_t, readIMUDoubleRegister
 */

imu_status_t getRollRate(spi_device_t* imuSPI, imu_handler_t* imuHandler, float* rollOutput) {
	uint16_t rollRaw;
	imu_status_t status = readIMUDoubleRegister(imuSPI, IMU_OUTY_H_G, IMU_OUTY_L_G, &rollRaw);
	*rollOutput = ((int16_t) rollRaw) * imuHandler->angularRateSens; // milidegrees/sec
	return status;
}

/**
 * @brief Reads the IMU's yaw (angular velocity around the Z-axis).
 *
 * @param[in]   imuSPI
 *     Pointer to the SPI device structure used for communication with the IMU.
 * @param[in]   imuHandler
 *     Pointer to the IMU handler structure containing sensitivity settings.
 * @param[out]  yawOutput
 *     Pointer to a float variable where the computed yaw rate value will be stored
 *     in milli-degrees per second.
 *
 * @retval status
 *     The status of the SPI communication, as returned by readIMUDoubleRegister().
 *
 * @see imu_status_t, readIMUDoubleRegister
 */

imu_status_t getYawRate(spi_device_t* imuSPI, imu_handler_t* imuHandler, float* yawOutput) {
	uint16_t yawRaw;
	imu_status_t status = readIMUDoubleRegister(imuSPI, IMU_OUTZ_H_G, IMU_OUTZ_L_G, &yawRaw);
	*yawOutput = ((int16_t) yawRaw) * imuHandler->angularRateSens; // milidegrees/sec
	return status;
}

/**
 * @brief Reads the IMU's X-axis linear acceleration.
 *
 * @param[in]   imuSPI
 *     Pointer to the SPI device structure used for communication with the IMU.
 * @param[in]   imuHandler
 *     Pointer to the IMU handler structure containing sensitivity settings.
 * @param[out]  xAccelOutput
 *     Pointer to a float variable where the computed X-axis acceleration will be stored
 *     in meters per second squared (m/s²).
 *
 * @retval status
 *     The status of the SPI communication, as returned by readIMUDoubleRegister().
 *
 * @see imu_status_t, readIMUDoubleRegister
 */
imu_status_t getXAccel(spi_device_t* imuSPI, imu_handler_t* imuHandler, float* xAccelOutput) {
	uint16_t xAccelRaw;
	imu_status_t status = readIMUDoubleRegister(imuSPI, IMU_OUTX_H_A, IMU_OUTX_L_A, &xAccelRaw);
	*xAccelOutput = ((int16_t) xAccelRaw) * imuHandler->accelSens; // m/s^2
	return status;
}

/**
 * @brief Reads the IMU's Y-axis linear acceleration.
 *
 * @param[in]   imuSPI
 *     Pointer to the SPI device structure used for communication with the IMU.
 * @param[in]   imuHandler
 *     Pointer to the IMU handler structure containing sensitivity settings.
 * @param[out]  yAccelOutput
 *     Pointer to a float variable where the computed Y-axis acceleration will be stored
 *     in meters per second squared (m/s²).
 *
 * @retval status
 *     The status of the SPI communication, as returned by readIMUDoubleRegister().
 *
 * @see imu_status_t, readIMUDoubleRegister
 */
imu_status_t getYAccel(spi_device_t* imuSPI, imu_handler_t* imuHandler, float* yAccelOutput) {
	uint16_t yAccelRaw;
	imu_status_t status = readIMUDoubleRegister(imuSPI, IMU_OUTY_H_A, IMU_OUTY_L_A, &yAccelRaw);
	*yAccelOutput = ((int16_t) yAccelRaw) * imuHandler->accelSens; // m/s^2
	return status;
}

/**
 * @brief Reads the IMU's Z-axis linear acceleration.
 *
 * @param[in]   imuSPI
 *     Pointer to the SPI device structure used for communication with the IMU.
 * @param[in]   imuHandler
 *     Pointer to the IMU handler structure containing sensitivity settings.
 * @param[out]  zAccelOutput
 *     Pointer to a float variable where the computed Z-axis acceleration will be stored
 *     in meters per second squared (m/s²).
 *
 * @retval status
 *     The status of the SPI communication, as returned by readIMUDoubleRegister().
 *
 * @see imu_status_t, readIMUDoubleRegister
 */
imu_status_t getZAccel(spi_device_t* imuSPI, imu_handler_t* imuHandler, float* zAccelOutput) {
	uint16_t zAccelRaw;
	imu_status_t status = readIMUDoubleRegister(imuSPI, IMU_OUTZ_H_A, IMU_OUTZ_L_A, &zAccelRaw);
	*zAccelOutput = ((int16_t) zAccelRaw) * imuHandler->accelSens; // m/s^2
	return status;
}
