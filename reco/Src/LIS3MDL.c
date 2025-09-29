#include "LIS3MDL.h"

// Hash of reserved registers where each index represents the register number
static const uint8_t MAG_RESERVED_REG_HASH[] = {1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1,
		 	 	 	 	 	 	 	 	 	 	0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
												1, 1, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0,
												0, 0, 0, 0, 0, 0, 0};


// Hash of writeable registers where each index represents the register number
static const uint8_t MAG_WRITEABLE_REG_HASH[] = {0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 0, 0,
		  	  	  	  	  	  	  	  	  	  	 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
												 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 0, 0,
												 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 1, 1};

// Array of all ctrl_reg numbers
static const uint8_t CTRL_REG_NUM_MAG[] = {MAG_CTRL_REG1, MAG_CTRL_REG2, MAG_CTRL_REG3, MAG_CTRL_REG4, MAG_CTRL_REG5};

static const uint8_t CTRL_REG1_MASK = (uint8_t) 0xFF;
static const uint8_t CTRL_REG2_MASK = (uint8_t) ~((1 << 7) | (1 << 4) | (1 << 1) | (1 << 0));
static const uint8_t CTRL_REG3_MASK = (uint8_t) ~((1 << 7) | (1 << 6) | (1 << 4) | (1 << 3));
static const uint8_t CTRL_REG4_MASK = (uint8_t) ~((1 << 7) | (1 << 6) | (1 << 5) | (1 << 4) | (1 << 0));
static const uint8_t CTRL_REG5_MASK = (uint8_t) ~((1 << 5) | (1 << 4) | (1 << 3) | (1 << 2) | (1 << 1) | (1 << 0));

// Array of all ctrl_reg masks
static const uint8_t CTRL_REG_MAG_MASK[] = {CTRL_REG1_MASK, CTRL_REG2_MASK, CTRL_REG3_MASK, CTRL_REG4_MASK, CTRL_REG5_MASK};

// Sensitivity values
static const float MAG_SENS_4_GAUSS  = 6842.0f; // LSB/Gauss
static const float MAG_SENS_8_GAUSS  = 3421.0f; // LSB/Gauss
static const float MAG_SENS_12_GAUSS = 2281.0f; // LSB/Gauss
static const float MAG_SENS_16_GAUSS = 1711.0f; // LSB/Gauss

/**
 * @brief Generates the LISMDL3 register address with read/write and consecutive flags.
 *
 * This function masks the provided register number to 6 bits and sets the
 * most significant bits based on the read and consecutive flags to produce
 * the full SPI register address for communication with the LISMDL3.
 *
 * @param[in] magRegNum
 *     The register number in the LISMDL3 to access (0x00–0x3F).
 * @param[in] readFlag
 *     Set to true to generate a read address; false for a write address.
 * @param[in] consecutiveFlag
 *     Set to true to enable consecutive read/write mode; false for a single register access.
 *
 * @return
 *     The 8-bit register address formatted for SPI communication.
 *
 * @see LISMDL3 datasheet, mag_reg_t
 */
uint8_t lis3mdl_generate_reg_address(mag_reg_t magRegNum, bool readFlag, bool consecutiveFlag) {

	// Clear the 7th bit and the 6th bit
	uint8_t newAddress = magRegNum & 0x3F;

	// If the readFlag is true, set the 7th bit to 1 to indicate read.
	// If the readFlag is false, set the 6th bit to 0 to indicate writes.
	if (readFlag) {
		newAddress |= (1 << 7);
	}

	// If the consecutiveFlag is true, set the 6th bit to 1 to indicate consecutive read/write register.
	// If the consecutiveFlag is false, set the 6th bit to 0 to indicate singular read/write register.
	if (consecutiveFlag) {
		newAddress |= (1 << 6);
	}

	return newAddress;
}

/**
 * @brief  Write a single value to a LIS3MDL register.
 *
 * @param[in]  magSPI        Pointer to the SPI device structure used for communication.
 * @param[in]  magRegNum     Register number to write to.
 * @param[in]  valueToWrite  Byte value to write into the register.
 *
 * @retval status  The status of the SPI communication.
 *
 * @see mag_status_t  Definition of the possible status codes.
 */
mag_status_t lis3mdl_write_single_reg(spi_device_t* magSPI, mag_reg_t magRegNum, uint8_t valueToWrite) {

	// Check if the register you want to write to aren't greater than or less than the max register number
	// and the minimum register number respectively. Ensure that the register is writeable. If not, return
	// the invalid register number error code.
	if (magRegNum < MAG_MIN_REG || MAG_MAX_REG < magRegNum || !MAG_WRITEABLE_REG_HASH[magRegNum]) {
		return MAG_INVALID_REG;
	}

	uint8_t actualAddress = lis3mdl_generate_reg_address(magRegNum, false, false);
	uint8_t command[] = {actualAddress, valueToWrite};

	mag_status_t status = SPI_Device_Transmit(magSPI, command, 2, HAL_MAX_DELAY);

	return status;
}

/**
 * @brief  Read a single value from a LIS3MDL register.
 *
 * @param[in]   magSPI        Pointer to the SPI device structure used for communication.
 * @param[in]   magRegNum     Register number to read from.
 * @param[out]  receivedData  Pointer to a variable where the received byte will be stored.
 *
 * @retval status  The status of the SPI communication.
 *
 * @see mag_status_t  Definition of the possible status codes.
 */
mag_status_t lis3mdl_read_single_reg(spi_device_t* magSPI, mag_reg_t magRegNum, uint8_t* recievedData) {

	if (magRegNum < MAG_MIN_REG || MAG_MAX_REG < magRegNum || MAG_RESERVED_REG_HASH[magRegNum]) {
		return MAG_INVALID_REG;
	}

	uint8_t actualAddress = lis3mdl_generate_reg_address(magRegNum, true, false);
	mag_status_t status = SPI_Device_TransmitReceiveSeparate(magSPI, &actualAddress, recievedData, 1, 1, HAL_MAX_DELAY);

	return status;
}

/**
 * @brief  Read a 16-bit value from two consecutive LIS3MDL registers.
 *
 * This function reads two 8-bit registers (upper and lower) from the magnetometer
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
mag_status_t lis3mdl_read_double_reg(spi_device_t* magSPI, mag_reg_t upperRegAddress, mag_reg_t lowerRegAddress, uint16_t* receivedData) {

	// Define variables to hold the values of the higher 8 bits, lower 8 bits, and status respectively
	uint8_t upper8;
	uint8_t lower8;
	mag_status_t status;

	// Read the values from the lower 8 bits. If SPI communication fails return the error code
	if ((status = lis3mdl_read_single_reg(magSPI, upperRegAddress, &upper8)) != MAG_COMMS_OK) {
		return status;
	}

	// Read the values from the higher 8 bits. If SPI communication fails return error code
	if ((status = lis3mdl_read_single_reg(magSPI, lowerRegAddress, &lower8)) != MAG_COMMS_OK) {
		return status;
	}

	// Combine the upper 8 and lower 8 bits together to get our 16-bit number and return OK SPI communication
	*receivedData = (uint16_t) upper8 << 8 | (uint16_t) lower8;
	return MAG_COMMS_OK;
}

/**
 * @brief Reads multiple consecutive registers from the LIS3MDL.
 *
 * @param[in] magSPI
 *     Pointer to the SPI device structure associated with the LIS3MDL.
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
mag_status_t lis3mdl_read_multiple_reg(spi_device_t* magSPI, mag_reg_t startRegNum, mag_reg_t endRegNum, uint8_t* regReadValues) {

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
		if (MAG_RESERVED_REG_HASH[magRegNum]) {
			return MAG_INVALID_REG;
		}
	}

	// Generate the starting address that indicating consecutive reads and calculate the total number of registers
	// that will be written to.
	uint8_t startingRegAddr = lis3mdl_generate_reg_address(startRegNum, true, true);
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
 * @brief	Writes a sequence of consecutive LIS3MDL registers with the provided values.
 *
 * @param[in] magSPI
 *     Pointer to the SPI device structure used for communication with the LIS3MDL.
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
mag_status_t lis3mdl_write_multiple_reg(spi_device_t* magSPI, mag_reg_t startRegNum, mag_reg_t endRegNum, uint8_t* valuesToWrite) {

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
	// that will be written to. The valuesToWrite array must have the size of numRegWrite or else we run over the array boundary
	uint8_t startingRegAddr = lis3mdl_generate_reg_address(startRegNum, false, true);
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

/*
CTRL_REG1: 0b00000010
CTRL_REG2: 0b01100000
CTRL_REG3: 0b00000000
CTRL_REG4: 0b00000000
*/

/**
 * @brief Initializes the LIS3MDL registers based on the handler configuration.
 *
 * This function iterates through all control registers specified by the magnetometer handler.
 * For registers marked as modified, it writes the corresponding value to the device. For unmodified
 * registers, it reads the current values into the handler. After all registers are synchronized,
 * the sensitivity value in the handler is set.
 *
 * @param[in] magSPI
 *     Pointer to the SPI device structure used for communication with the LIS3MDL.
 *
 * @param[in,out] magHandler
 *     Pointer to the magnetometer handler structure containing register configuration. This
 *     structure will be updated with current register values and the computed sensitivity.
 *
 * @retval MAG_COMMS_OK
 *     Initialization completed successfully.
 *
 * @retval MAG_COMMS_ERROR or other mag_status_t values
 *     A SPI communication error occurred while reading or writing registers.
 *
 * @see mag_status_t, writeMagRegister, readMagSingleRegister
 */
mag_status_t lis3mdl_initialize_mag(spi_device_t* magSPI, mag_handler_t* magHandler) {

	// Define rawReg as a pointer to the first ctrl register
	mag_status_t status;
	uint8_t* rawReg = (uint8_t*) &magHandler->ctrl_reg1;

	// Access all ctrl registers within the struct by incrementing the pointer address (rawReg) by 1.
	// If the register has been modified, indicated by a 1 in its respective index, the value pointed to
	// by rawReg is written into its respective register. If it wasn't modified, indicated by a 0 in the
	// modifiedRegisters, we read the value from the magnetometer into the rawReg pointer.
	for (int currRegIdx = 0; currRegIdx < MAG_CTRL_REG_NUM; currRegIdx++) {

		if (magHandler->modifiedRegisters[currRegIdx]) {
			*rawReg &= CTRL_REG_MAG_MASK[currRegIdx];

			if ((status = lis3mdl_write_single_reg(magSPI, CTRL_REG_NUM_MAG[currRegIdx], *rawReg)) != MAG_COMMS_OK) {
				return status;
			}

		} else {

			if ((status = lis3mdl_read_single_reg(magSPI, CTRL_REG_NUM_MAG[currRegIdx], rawReg)) != MAG_COMMS_OK) {
				return status;
			}

		}

		// Incrememnt rawReg address
		rawReg++;
	}

	// Based on scale chosen by FS bits in the ctrl_reg2 set the sensitivity
	switch (magHandler->ctrl_reg2.flags.FS) {
		case MAG_FS_4GAUSS:
			magHandler->sensitivity = MAG_SENS_4_GAUSS;
			break;
		case MAG_FS_8GAUSS:
			magHandler->sensitivity = MAG_SENS_8_GAUSS;
			break;
		case MAG_FS_12GAUSS:
			magHandler->sensitivity = MAG_SENS_12_GAUSS;
			break;
		case MAG_FS_16GAUSS:
			magHandler->sensitivity = MAG_SENS_16_GAUSS;
			break;
		default:
			magHandler->sensitivity = 0;
			return -1;
	}

	return MAG_COMMS_OK;
}

/**
 * @brief Reads the LIS3MDL's X-axis magnetic field.
 *
 * @param[in]   magSPI
 *     Pointer to the SPI device structure used for communication with the magnetometer.
 * @param[in]   magHandler
 *     Pointer to the magnetometer handler structure containing sensitivity settings.
 * @param[out]  magXOutput
 *     Pointer to a float variable where the computed X-axis magnetic field will be stored
 *     as Gauss.
 *
 * @retval status
 *     The status of the SPI communication, as returned by readMagDoubleRegister().
 *
 * @see mag_status_t, readMagDoubleRegister
 */
mag_status_t lis3mdl_get_x_mag(spi_device_t* magSPI, mag_handler_t* magHandler, float* xOutputMag) {
	uint16_t xMagRaw;
	mag_status_t status = lis3mdl_read_double_reg(magSPI, MAG_OUT_X_H, MAG_OUT_X_L, &xMagRaw);
	*xOutputMag = ((int16_t) xMagRaw) / magHandler->sensitivity; // Gauss
	return status;
}

/**
 * @brief Reads the LIS3MDL's Y-axis magnetic field.
 *
 * @param[in]   magSPI
 *     Pointer to the SPI device structure used for communication with the magnetometer.
 * @param[in]   magHandler
 *     Pointer to the magnetometer handler structure containing sensitivity settings.
 * @param[out]  magYOutput
 *     Pointer to a float variable where the computed Y-axis magnetic field will be stored
 *     as Gauss.
 *
 * @retval status
 *     The status of the SPI communication, as returned by readMagDoubleRegister().
 *
 * @see mag_status_t, readMagDoubleRegister
 */
mag_status_t lis3mdl_get_y_mag(spi_device_t* magSPI, mag_handler_t* magHandler, float* yOutputMag) {
	uint16_t yMagRaw;
	mag_status_t status = lis3mdl_read_double_reg(magSPI, MAG_OUT_Y_H, MAG_OUT_Y_L, &yMagRaw);
	*yOutputMag = ((int16_t) yMagRaw) / magHandler->sensitivity; // Gauss
	return status;
}

/**
 * @brief Reads the LIS3MDL's Z-axis magnetic field.
 *
 * @param[in]   magSPI
 *     Pointer to the SPI device structure used for communication with the magnetometer.
 * @param[in]   magHandler
 *     Pointer to the magnetometer handler structure containing sensitivity settings.
 * @param[out]  magZOutput
 *     Pointer to a float variable where the computed Z-axis magnetic field will be stored
 *     as Gauss.
 *
 * @retval status
 *     The status of the SPI communication, as returned by readMagDoubleRegister().
 *
 * @see mag_status_t, readMagDoubleRegister
 */
mag_status_t lis3mdl_get_z_mag(spi_device_t* magSPI, mag_handler_t* magHandler, float* zOutputMag) {
	uint16_t zMagRaw;
	mag_status_t status = lis3mdl_read_double_reg(magSPI, MAG_OUT_Z_H, MAG_OUT_Z_L, &zMagRaw);
	*zOutputMag = ((int16_t) zMagRaw) / magHandler->sensitivity; // Gauss
	return status;
}


