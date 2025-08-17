#include "LIS3MDLOld.h"

const uint8_t MAG_RESERVED_REG_HASH[] = {0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1};

uint8_t generateMagAddressOld(mag_reg_t magRegNum, bool readFlag, bool consecutiveFlag) {

	uint8_t newAddress = magRegNum & 0x3F;

	if (readFlag) {
		newAddress |= (1 << 7);
	}

	if (consecutiveFlag) {
		newAddress |= (1 << 6);
	}

	return (uint8_t) newAddress;
}

/*
	0: Reg is not reserved
	1: Reg is reserved
	2: Out of bounds
*/

inline uint8_t ensureMagNotReservedOld(mag_reg_t regToCheck) {

	if (regToCheck >= 0 && regToCheck <= MAG_MAX_REG) {
        return 2;
    }

	return MAG_RESERVED_REG_HASH[regToCheck];

}

short writeMagRegisterOld(spi_device_t* magSPI, mag_reg_t magRegNum, uint8_t valueToWrite) {

	/*
	ensureMagNotReserved(magRegNum);
	*/

	uint8_t actualAddress = generateMagAddressOld(magRegNum, false, false);
	uint8_t command[] = {actualAddress, valueToWrite};

	short result = SPI_Device_Transmit(magSPI, command, 2, HAL_MAX_DELAY);

	return result;
}

uint8_t readMagSingleRegisterOld(spi_device_t* magSPI, mag_reg_t magRegNum) {

	/*
	if (ensureMagRegNotReserved(magRegNum)) {
		return 1;
	}
	*/

	ensureMagNotReservedOld(magRegNum);

	uint8_t actualAddress = generateMagAddressOld(magRegNum, true, false);
	uint8_t regValue;

	uint8_t status = SPI_Device_TransmitReceiveSeparate(magSPI, &actualAddress, &regValue, 1, 1, HAL_MAX_DELAY);

	return regValue;
}

uint16_t readMagDoubleRegisterOld(spi_device_t* magSPI, mag_reg_t upperRegAddress, mag_reg_t lowerRegAddress) {

	/*

	if (ensureMagRegNotReserved(upperRegAddress) || ensureMagRegNotReserved(lowerRegAddress)) {
		return 1;
	}
	*/

	ensureMagNotReservedOld(upperRegAddress);
	ensureMagNotReservedOld(lowerRegAddress);

	uint8_t upper8 = readMagSingleRegisterOld(magSPI, upperRegAddress);
	uint8_t lower8 = readMagSingleRegisterOld(magSPI, lowerRegAddress);

	uint16_t finalResult = (uint16_t) upper8 << 8 | (uint16_t) lower8;
	return finalResult;
}



uint8_t* readMagMultipleRegistersOld(spi_device_t* magSPI, mag_reg_t magRegNum, uint8_t numRegisters) {

	/*
	if (ensureMagRegNotReserved(magRegNum)) {
		return NULL;
	}
	*/

	magRegNum = generateMagAddressOld(magRegNum, true, true);

	short SPIResult;
	uint8_t regValue[numRegisters];

	SPIResult = SPI_Device_Transmit(magSPI, &magRegNum, 1, HAL_MAX_DELAY);
	SPIResult = SPI_Device_Receive(magSPI, regValue, numRegisters, HAL_MAX_DELAY);

	return regValue;
}

/*
CTRL_REG1: 0b00000010
CTRL_REG2: 0b01100000
CTRL_REG3: 0b00000000
CTRL_REG4: 0b00000000
*/

uint8_t initializeMagOld(spi_device_t* magSPI, mag_handler_t* magHandler) {

	uint8_t status = writeMagRegisterOld(magSPI, MAG_CTRL_REG1, magHandler->CTRL_REG_FLAGS[0]);
	status = writeMagRegisterOld(magSPI, MAG_CTRL_REG2, magHandler->CTRL_REG_FLAGS[1]);
	status = writeMagRegisterOld(magSPI, MAG_CTRL_REG3, magHandler->CTRL_REG_FLAGS[2]);
	status = writeMagRegisterOld(magSPI, MAG_CTRL_REG4, magHandler->CTRL_REG_FLAGS[3]);
	status = writeMagRegisterOld(magSPI, MAG_CTRL_REG5, magHandler->CTRL_REG_FLAGS[4]);

	if ((magHandler->CTRL_REG_FLAGS[1] & ((1 << 5) | (1 << 6))) == MAG_CTRL2_4_GAUSS) {
		magHandler->sensitivity = 6842;
	} else if ((magHandler->CTRL_REG_FLAGS[1] & MAG_CTRL2_8_GAUSS) == MAG_CTRL2_8_GAUSS) {
		magHandler->sensitivity = 3421;
	} else if ((magHandler->CTRL_REG_FLAGS[1] & MAG_CTRL2_12_GAUSS) == MAG_CTRL2_12_GAUSS) {
		magHandler->sensitivity = 2281;
	} else if ((magHandler->CTRL_REG_FLAGS[1] & MAG_CTRL2_16_GAUSS) == MAG_CTRL2_16_GAUSS) {
		magHandler->sensitivity = 1711;
	}

	return status;

}

uint8_t getStatusRegisterOld(spi_device_t* magSPI) {
	return readMagSingleRegister(magSPI, MAG_STATUS_REG);
}

/*
inline double getXMagRaw(spi_device_t* magSPI, mag_handler_t* magHandler) {
	return ((double) ((int16_t) readMagDoubleRegister(magSPI, MAG_OUT_X_H, MAG_OUT_X_L)));
}

inline double getYMagRaw(spi_device_t* magSPI, mag_handler_t* magHandler) {
	return ((double) ((int16_t) readMagDoubleRegister(magSPI, MAG_OUT_Y_H, MAG_OUT_Y_L)));
}

inline double getZMagRaw(spi_device_t* magSPI, mag_handler_t* magHandler) {
	return ((double) ((int16_t) readMagDoubleRegister(magSPI, MAG_OUT_Z_H, MAG_OUT_Z_L)));
}

inline double getXMag(spi_device_t* magSPI, mag_handler_t* magHandler) {
	int16_t magX = (int16_t) readMagDoubleRegister(magSPI, MAG_OUT_X_H, MAG_OUT_X_L);
	return ((double) magX / magHandler->sensitivity);
}

inline double getYMag(spi_device_t* magSPI, mag_handler_t* magHandler) {
	int16_t magY =  (int16_t) readMagDoubleRegister(magSPI, MAG_OUT_Y_H, MAG_OUT_Y_L);
	return ((double) magY / magHandler->sensitivity);
}

inline double getZMag(spi_device_t* magSPI, mag_handler_t* magHandler) {
	int16_t magZ = (int16_t) readMagDoubleRegister(magSPI, MAG_OUT_Z_H, MAG_OUT_Z_L);
	return ((double) magZ / magHandler->sensitivity);
}
*/


