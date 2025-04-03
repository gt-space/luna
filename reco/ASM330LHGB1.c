#include "ASM330LHGB1.h"

uint8_t IMU_RESERVED_REG_HASH[] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
								   0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
								   0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1,
								   1, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
								   1, 1, 0, 1, 0, 0, 1, 0, 0, 0, 0, 0, 1, 1, 0, 0, 1, 1, 1, 1, 1,
								   1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0};

uint8_t generateIMUAddress(imu_reg_t imuRegNum, bool readFlag) {

  uint8_t newAddress = imuRegNum & 0x7F;

  if (readFlag) {
    newAddress |= (1 << 7);
  }

  return (uint8_t) newAddress;
}

inline uint8_t ensureIMUNotReserved(imu_reg_t regToCheck) {

	if (regToCheck > 0 && regToCheck < IMU_MAX_REG) {
        return 2;
    }

	return IMU_RESERVED_REG_HASH[regToCheck];
}

uint8_t writeIMURegister(spi_device_t* imuSPI, imu_reg_t imuRegNum, uint8_t valueToWrite) {

	ensureIMUNotReserved(imuRegNum);

	uint8_t actualRegNumber = generateIMUAddress(imuRegNum, false);
	uint8_t command[] = {actualRegNumber, valueToWrite};

	uint8_t SPIResult = SPI_Device_Transmit(imuSPI, command, 2, HAL_MAX_DELAY);

	return SPIResult;
}

uint8_t readIMUSingleRegister(spi_device_t* imuSPI, imu_reg_t imuRegNum) {

	ensureIMUNotReserved(imuRegNum);

	uint8_t actualRegNumber = generateIMUAddress(imuRegNum, true);
	uint8_t regValue;

	uint8_t status = SPI_Device_TransmitReceiveSeparate(imuSPI, &actualRegNumber, &regValue, 1, 1, HAL_MAX_DELAY);

	return regValue;
}

uint16_t readIMUDoubleRegister(spi_device_t* imuSPI, imu_reg_t upperRegAddress, imu_reg_t lowerRegAddress) {

	ensureIMUNotReserved(upperRegAddress);
	ensureIMUNotReserved(lowerRegAddress);

	uint8_t upper8 = readIMUSingleRegister(imuSPI, upperRegAddress);
	uint8_t lower8 = readIMUSingleRegister(imuSPI, lowerRegAddress);

	uint16_t finalResult = (uint16_t) upper8 << 8 | (uint16_t) lower8;
	return finalResult;
}

uint8_t initializeIMU(spi_device_t* imuSPI) {

	uint8_t status = writeIMURegister(imuSPI, PIN_CTRL, 0b01111111);
	status = writeIMURegister(imuSPI, CTRL6_C, 0b00000000);
	status = writeIMURegister(imuSPI, CTRL7_G, 0b00000000);
	status = writeIMURegister(imuSPI, CTRL1_XL, 0b10000100);
	status = writeIMURegister(imuSPI, CTRL2_G, 0b10001100);

	return status;
}

inline double getPitch(spi_device_t* imuSPI) {
	return (0.140 * (double) ((int16_t) readIMUDoubleRegister(imuSPI, OUTX_H_G, OUTX_L_G)));
}

inline double getRoll(spi_device_t* imuSPI) {
	return (0.140 * (double) ((int16_t) readIMUDoubleRegister(imuSPI, OUTY_H_G, OUTY_L_G)));
}

inline double getYaw(spi_device_t* imuSPI) {
	return (0.140 * (double) ((int16_t) readIMUDoubleRegister(imuSPI, OUTZ_H_G, OUTZ_L_G)));
}

inline double getXAccel(spi_device_t* imuSPI) {
	//int16_t rawX = (int16_t) readIMUDoubleRegister(imuSPI, OUTX_H_A, OUTX_L_A);
	return (0.000488 * (double) ((int16_t) readIMUDoubleRegister(imuSPI, OUTX_H_A, OUTX_L_A)));
}

inline double getYAccel(spi_device_t* imuSPI) {
	//int16_t rawY = (int16_t) readIMUDoubleRegister(imuSPI, OUTY_H_A, OUTY_L_A);
	return (0.000488 * (double) ((int16_t) readIMUDoubleRegister(imuSPI, OUTY_H_A, OUTY_L_A)));
}

inline double getZAccel(spi_device_t* imuSPI) {
	//int16_t rawZ = (int16_t) readIMUDoubleRegister(imuSPI, OUTZ_H_A, OUTZ_L_A);
	return (0.000488 * (double) ((int16_t) readIMUDoubleRegister(imuSPI, OUTZ_H_A, OUTZ_L_A)));
}
