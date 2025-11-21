#include "LIS2MDL.h"
#include "stdio.h"

/**
 * Process to setup magnetometer:
 * 		1. Change set_lis2mdl_flags() to be the flags that you want to set.
 * 		2. Run lis2mdl_initialize_mag().
 */

static const uint8_t MAG_READABLE_REG_HASH[] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
				                         0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
				                         0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
				                         0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0,
				                         0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1,
				                         1, 1, 1, 1, 1, 1, 1};

static const uint8_t MAG_WRITEABLE_REG_HASH[] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        							     0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
										 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
										 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0,
										 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 0, 1, 1, 0, 0,
										 0, 0, 0, 0, 0, 0, 0};

static const uint8_t CTRL_REG_NUM_MAG[] = {MAG_CFG_REG_A, MAG_CFG_REG_B, MAG_CFG_REG_C, MAG_INT_CRTL_REG};
static const uint8_t INT_CTRL_REG_MASK = (uint8_t) ~((1 << 3) | (1 << 4));
static const float32_t MAG_SENS = 1.5f; // mGauss/LSB

void print_bytes_binary1(const uint8_t *data, size_t len) {
    for (size_t i = 0; i < len; i++) {
        for (int bit = 7; bit >= 0; bit--) {
            printf("%c", (data[i] & (1 << bit)) ? '1' : '0');
        }
        if (i < len - 1) {
            printf(" "); // space between bytes
        }
    }
    printf("\n");
}

/**
 * @brief Generates the LIS2MDL register address with the read/write flag.
 *
 * This function masks the provided register number to 7 bits and sets
 * the most significant bit if a read operation is requested, producing
 * the full SPI register address to be transmitted to the LIS2MDL.
 *
 * @param[in] magRegNum
 *     The register number in the LIS2MDL to access.
 *
 * @param[in] readFlag
 *     Set to true if generating a read address; false for a write address.
 *
 * @return
 *     The 8-bit register address formatted for SPI communication.
 *
 * @see LIS2MDL datasheet, mag_reg_t
 */
uint8_t lis2mdl_generate_reg_address(mag_reg_t magRegNum, bool readFlag) {

  uint8_t newAddress = magRegNum & 0x7F;

  if (readFlag) {
    newAddress |= (1 << 7);
  }

  return newAddress;
}

/**
 * @brief  Write a single value to a LIS2MDL register.
 *
 * @param[in]  magSPI        Pointer to the SPI device structure used for communication.
 * @param[in]  magRegNum     Register number to write to.
 * @param[in]  valueToWrite  Byte value to write into the register.
 *
 * @retval status  The status of the SPI communication.
 *
 * @see mag_status_t  Definition of the possible status codes.
 */
mag_status_t lis2mdl_write_single_reg(spi_device_t* magSPI, mag_reg_t magRegNum, uint8_t valueToWrite) {

	if (magRegNum < MAG_MIN_REG || MAG_MAX_REG < magRegNum || !MAG_WRITEABLE_REG_HASH[magRegNum]) {
		return MAG_INVALID_REG;
	}

	uint8_t actualAddress = lis2mdl_generate_reg_address(magRegNum, false);
	uint8_t command[] = {actualAddress, valueToWrite};

	mag_status_t status = SPI_Device_Transmit(magSPI, command, 2, HAL_MAX_DELAY);

	return status;
}

/**
 * @brief  Read a single value from a LIS2MDL register.
 *
 * @param[in]   magSPI        Pointer to the SPI device structure used for communication.
 * @param[in]   magRegNum     Register number to read from.
 * @param[out]  receivedData  Pointer to a variable where the received byte will be stored.
 *
 * @retval status  The status of the SPI communication.
 *
 * @see mag_status_t  Definition of the possible status codes.
 */
mag_status_t lis2mdl_read_single_reg(spi_device_t* magSPI, mag_reg_t magRegNum, uint8_t* recievedData) {

	if (magRegNum < MAG_MIN_REG || MAG_MAX_REG < magRegNum || !MAG_READABLE_REG_HASH[magRegNum]) {
		return MAG_INVALID_REG;
	}

	uint8_t actualAddress = lis2mdl_generate_reg_address(magRegNum, true);
	mag_status_t status = SPI_Device_TransmitReceiveSeparate(magSPI, &actualAddress, recievedData, 1, 1, HAL_MAX_DELAY);

	return status;
}

/**
 * @brief  Read a 16-bit value from two consecutive LIS2MDL registers.
 *
 * This function reads two 8-bit registers (upper and lower) from the LIS2MDL
 * and combines them into a single 16-bit value. If either read operation
 * fails, the corresponding status code is returned immediately.
 *
 * @param[in]   magSPI            Pointer to the SPI device structure used for communication.
 * @param[in]   upperRegAddress   Register address containing the upper 8 bits of the value.
 * @param[in]   lowerRegAddress   Register address containing the lower 8 bits of the value.
 * @param[out]  receivedData      Pointer to a variable where the combined 16-bit value will be stored.
 *
 * @retval status  The status of the SPI communication.
 *
 * @see mag_status_t  Definition of the possible status codes.
 */
mag_status_t lis2mdl_read_double_reg(spi_device_t* magSPI, mag_reg_t upperRegAddress, mag_reg_t lowerRegAddress, uint16_t* receivedData) {

	uint8_t upper8;
	uint8_t lower8;
	mag_status_t status;

	if ((status = lis2mdl_read_single_reg(magSPI, upperRegAddress, &upper8)) != MAG_COMMS_OK) {
		return status;
	}

	if ((status = lis2mdl_read_single_reg(magSPI, lowerRegAddress, &lower8)) != MAG_COMMS_OK) {
		return status;
	}

	*receivedData = (uint16_t) upper8 << 8 | (uint16_t) lower8;
	return MAG_COMMS_OK;
}

/**
 * @brief Reads multiple consecutive registers from the LIS2MDL.
 *
 * @param[in] magSPI
 *     Pointer to the SPI device structure associated with the LIS2MDL.
 * @param[in] startRegNum
 *     The first register in the consecutive range to read.
 * @param[in] endRegNum
 *     The last register in the consecutive range to read.
 * @param[out] regReadValues
 *     Pointer to an array that will store the values read from the
 *     specified register range. The array must be at least
 *     (@p endRegNum - @p startRegNum + 1) bytes long.
 *
 * @retval MAG_COMMS_OK
 *     All registers in the range were successfully read.
 * @retval MAG_INVALID_REG
 *     One or more registers in the specified range are invalid or reserved.
 * @retval status
 *     A non-OK SPI communication status code returned by HAL functions.
 *
 * @note
 *     The @p regReadValues array must be preallocated with sufficient size
 *     to hold all register values in the specified range, otherwise a buffer
 *     overflow will occur.
 *
 * @seealso mag_status_t, HAL_SPI_Transmit, HAL_SPI_Receive
 */
mag_status_t lis2mdl_read_multiple_reg(spi_device_t* magSPI, mag_reg_t startRegNum, mag_reg_t endRegNum, uint8_t* regReadValues) {

	mag_status_t status;

	if (startRegNum < MAG_MIN_REG) {
		return MAG_INVALID_REG;
	}

	if (endRegNum > MAG_MAX_REG) {
		return MAG_INVALID_REG;
	}

	// Check that all registers within the range of the starting register (startRegNum) and the ending register
	// (endRegNum) are all readable. Return the invalid register status code if not.
	for (uint8_t magRegNum = startRegNum; magRegNum <= endRegNum; magRegNum++) {
		if (!MAG_READABLE_REG_HASH[magRegNum]) {
			return MAG_INVALID_REG;
		}
	}

	// Generate the starting address that indicating consecutive reads and calculate the total number of registers
	// that will be written to.
	uint8_t startingRegAddr = lis2mdl_generate_reg_address(startRegNum, true);
	uint8_t numRegRead = endRegNum - startRegNum + 1;

	// Pull CS line low to start transmission
    HAL_GPIO_WritePin(magSPI->GPIO_Port, magSPI->GPIO_Pin, GPIO_PIN_RESET);

    // Send the starting register address to the magnetometer and ensure that the communication is ok.
	if ((status = HAL_SPI_Transmit(magSPI->hspi, &startingRegAddr, 1, HAL_MAX_DELAY)) != MAG_COMMS_OK) {
	    HAL_GPIO_WritePin(magSPI->GPIO_Port, magSPI->GPIO_Pin, GPIO_PIN_SET);
		return status;
	}

    // Read the value from each register from startReg to endReg inclusive per 8 SPI clock cycles.
	if ((status = HAL_SPI_Receive(magSPI->hspi, regReadValues, numRegRead, HAL_MAX_DELAY)) != MAG_COMMS_OK) {
	    HAL_GPIO_WritePin(magSPI->GPIO_Port, magSPI->GPIO_Pin, GPIO_PIN_SET);
	    return status;
	}

    // Pull CS line high to end transmission
    HAL_GPIO_WritePin(magSPI->GPIO_Port, magSPI->GPIO_Pin, GPIO_PIN_SET);

    return MAG_COMMS_OK;
}

/**
 * @brief	Writes a sequence of consecutive LIS2MDL registers with the provided values.
 *
 * @param[in] magSPI
 *     Pointer to the SPI device structure used for communication with the LIS2MDL.
 * @param[in] startRegNum
 *     The first register in the consecutive range to write to.
 * @param[in] endRegNum
 *     The last register in the consecutive range to write to.
 * @param[in] valuesToWrite
 *     Pointer to an array containing the values to be written. The array
 *     length must be at least (endRegNum - startRegNum + 1).
 *
 * @retval MAG_COMMS_OK
 *     The registers were successfully written.
 * @retval MAG_INVALID_REG
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
 *     mag_status_t for possible status code definitions.
 */
mag_status_t lis2mdl_write_multiple_reg(spi_device_t* magSPI, mag_reg_t startRegNum, mag_reg_t endRegNum, uint8_t* valuesToWrite) {

	mag_status_t status;

	if (startRegNum < MAG_MIN_REG) {
		return MAG_INVALID_REG;
	}

	if (endRegNum > MAG_MAX_REG) {
		return MAG_INVALID_REG;
	}

	// Check that all registers within the range of the starting register (startRegNum) and the ending register
	// (endRegNum) are all writeable. Return the invalid register status code if not to be handled by caller.
	for (uint8_t magRegNum = startRegNum; magRegNum <= endRegNum; magRegNum++) {
		if (!MAG_WRITEABLE_REG_HASH[magRegNum]) {
			return MAG_INVALID_REG;
		}
	}

	// Generate the starting address that indicating consecutive writes and calculate the total number of registers
	// that will be written to.
	uint8_t startingRegAddr = lis2mdl_generate_reg_address(startRegNum, false);
	uint8_t numRegWrite = endRegNum - startRegNum + 1;

	// Pull CS line low to start transmission
    HAL_GPIO_WritePin(magSPI->GPIO_Port, magSPI->GPIO_Pin, GPIO_PIN_RESET);

    // Send the starting register address to the magnetometer and ensure that the communication is ok.
    if ((status = HAL_SPI_Transmit(magSPI->hspi, &startingRegAddr, 1, HAL_MAX_DELAY)) != MAG_COMMS_OK) {
        HAL_GPIO_WritePin(magSPI->GPIO_Port, magSPI->GPIO_Pin, GPIO_PIN_SET);
        return status;
    }

    // Send the value that you want to be written into each register per 8 SPI clock cycles.
    if ((status = HAL_SPI_Transmit(magSPI->hspi, valuesToWrite, numRegWrite, HAL_MAX_DELAY)) != MAG_COMMS_OK) {
        HAL_GPIO_WritePin(magSPI->GPIO_Port, magSPI->GPIO_Pin, GPIO_PIN_SET);
        return status;
    }

    // Pull CS line high to end transmission
    HAL_GPIO_WritePin(magSPI->GPIO_Port, magSPI->GPIO_Pin, GPIO_PIN_SET);

    return MAG_COMMS_OK;
}

/**
 * @brief Initializes the LIS2MDL registers based on the handler configuration.
 *
 * This function iterates through all control registers specified by the magnetometer handler.
 * For registers marked as modified, it writes the corresponding value to the device. For unmodified
 * registers, it reads the current values into the handler. After all registers are synchronized,
 * the sensitivity value in the handler is set.
 *
 * @param[in] magSPI
 *     Pointer to the SPI device structure used for communication with the LIS2MDL.
 * @param[in,out] magHandler
 *     Pointer to the magnetometer handler structure containing register configuration. This
 *     structure will be updated with current register values and the computed sensitivity.
 *
 * @retval MAG_COMMS_OK
 *     Initialization completed successfully.
 * @retval MAG_COMMS_ERROR or other mag_status_t values
 *     A SPI communication error occurred while reading or writing registers.
 *
 * @see mag_status_t, writeMagRegister, readMagSingleRegister
 */
mag_status_t lis2mdl_initialize_mag(spi_device_t* magSPI, mag_handler_t* magHandler) {

	mag_status_t status;
	uint8_t* rawReg = (uint8_t*) &magHandler->cfg_reg_a;

	for (int currRegIdx = 0; currRegIdx < MAG_CTRL_REG_NUM; currRegIdx++) {
		mag_reg_t currRegNum = CTRL_REG_NUM_MAG[currRegIdx];

		if (magHandler->modifiedRegisters[currRegIdx]) {

			if (currRegNum == MAG_INT_CRTL_REG) {
				*rawReg &= INT_CTRL_REG_MASK;
			}

			// print_bytes_binary1(rawReg, 1);

			if ((status = lis2mdl_write_single_reg(magSPI, currRegNum, *rawReg)) != MAG_COMMS_OK) {
				return status;
			}

		} else {
			if ((status = lis2mdl_read_single_reg(magSPI, currRegNum, rawReg)) != MAG_COMMS_OK) {
				return status;
			}
		}
		rawReg++;
	}

	magHandler->sensitivity = MAG_SENS;
	return MAG_COMMS_OK;
}

/**
 * @brief Reads the LIS2MDL's X-axis magnetic field.
 *
 * @param[in]   magSPI
 *     Pointer to the SPI device structure used for communication with the magnetometer.
 * @param[in]   magHandler
 *     Pointer to the magnetometer handler structure containing sensitivity settings.
 * @param[out]  magXOutput
 *     Pointer to a float variable where the computed X-axis magnetic field will be stored
 *     in milliGauss (mGauss).
 *
 * @retval status
 *     The status of the SPI communication, as returned by readMagDoubleRegister().
 *
 * @see mag_status_t, readMagDoubleRegister
 */
mag_status_t lis2mdl_get_x_mag(spi_device_t* magSPI, mag_handler_t* magHandler, float32_t* magXOutput) {
	uint16_t rawXMag;
	mag_status_t status = lis2mdl_read_double_reg(magSPI, MAG_OUTX_H_REG, MAG_OUTX_L_REG, &rawXMag);
	*magXOutput = ((int16_t) rawXMag) * magHandler->sensitivity; // mGauss
	return status;
}

/**
 * @brief Reads the LIS2MDL's Y-axis magnetic field.
 *
 * @param[in]   magSPI
 *     Pointer to the SPI device structure used for communication with the magnetometer.
 * @param[in]   magHandler
 *     Pointer to the magnetometer handler structure containing sensitivity settings.
 * @param[out]  magYOutput
 *     Pointer to a float variable where the computed Y-axis magnetic field will be stored
 *     in milliGauss (mGauss).
 *
 * @retval status
 *     The status of the SPI communication, as returned by readMagDoubleRegister().
 *
 * @see mag_status_t, readMagDoubleRegister
 */
mag_status_t lis2mdl_get_y_mag(spi_device_t* magSPI, mag_handler_t* magHandler, float32_t* magYOutput) {
	uint16_t rawYMag;
	mag_status_t status = lis2mdl_read_double_reg(magSPI, MAG_OUTY_H_REG, MAG_OUTY_L_REG, &rawYMag);
	*magYOutput = ((int16_t) rawYMag) * magHandler->sensitivity; // mGauss
	return status;
}

/**
 * @brief Reads the LIS2MDL's Z-axis magnetic field.
 *
 * @param[in]   magSPI
 *     Pointer to the SPI device structure used for communication with the magnetometer.
 * @param[in]   magHandler
 *     Pointer to the magnetometer handler structure containing sensitivity settings.
 * @param[out]  magZOutput
 *     Pointer to a float variable where the computed Z-axis magnetic field will be stored
 *     in milliGauss (mGauss).
 *
 * @retval status
 *     The status of the SPI communication, as returned by readMagDoubleRegister().
 *
 * @see mag_status_t, readMagDoubleRegister
 */
mag_status_t lis2mdl_get_z_mag(spi_device_t* magSPI, mag_handler_t* magHandler, float32_t* magZOutput) {
	uint16_t rawZMag;
	mag_status_t status = lis2mdl_read_double_reg(magSPI, MAG_OUTZ_H_REG, MAG_OUTZ_L_REG, &rawZMag);
	*magZOutput = ((int16_t) rawZMag) * magHandler->sensitivity; // mGauss
	return status;
}

mag_status_t lis2mdl_get_mag_data(spi_device_t* magSPI, mag_handler_t* magHandler, reco_message* message) {

	uint8_t regReturn[6];
	mag_status_t status;


	if ((status = lis2mdl_read_multiple_reg(magSPI, MAG_OUTX_L_REG, MAG_OUTZ_H_REG, regReturn)) != MAG_COMMS_OK) {
		return status;
	}

	uint16_t rawValue;
	for (int i = 0; i < 6; i += 2) {
		rawValue = ((uint16_t) regReturn[i+1] << 8) | (uint16_t) regReturn[i];
		message->magData[i / 2] = ((int16_t) rawValue) * magHandler->sensitivity;
	}

	return MAG_COMMS_OK;
}

/**
 * @brief Configure the LIS2MDL magnetometer control registers with predefined flags.
 *
 * This function sets specific configuration bits in the LIS2MDL magnetometer's
 * control registers (CFG_REG_A and CFG_REG_C) through the provided handler.
 * The following settings are applied:
 * - Temperature compensation: disabled
 * - Low-power mode: disabled (high resolution mode enabled)
 * - Output data rate: 100 Hz
 * - Operating mode: continuous-conversion
 * - I²C interface: disabled (SPI only)
 * - Block data update (BDU): enabled
 * - SPI mode: 4-wire
 *
 * @param[in,out] magHandler
 * Pointer to a @ref mag_handler_t structure that holds the register
 * configuration fields for the LIS2MDL.
 *
 * @note The handler must be initialized to all zeros before using this function.
 *       This function only modifies configuration flags; it does not directly
 *       write to the sensor hardware.
 */
void set_lis2mdl_flags(mag_handler_t* magHandler) {

	magHandler->cfg_reg_a.flags.COMP_TEMP_EN = MAG_COMP_TEMP_DISABLE;
	magHandler->cfg_reg_a.flags.LP = MAG_HIGH_RESOLUTION;
	magHandler->cfg_reg_a.flags.ODR = MAG_ODR_100_HZ;
	magHandler->cfg_reg_a.flags.MD = MAG_CONTINUOUS_MODE;
	magHandler->modifiedRegisters[0] = true;

	magHandler->cfg_reg_c.flags.I2C_DIS = MAG_DISABLE_I2C;
	magHandler->cfg_reg_c.flags.BDU		= MAG_BDU_ENABLE;
	magHandler->cfg_reg_c.flags.SIM 	= MAG_SPI_4_WIRE;
	magHandler->modifiedRegisters[1] = true;
	magHandler->modifiedRegisters[2] = true;


	magHandler->int_ctrl_reg.reg = 0b11100000;
	magHandler->modifiedRegisters[3] = true;
}


