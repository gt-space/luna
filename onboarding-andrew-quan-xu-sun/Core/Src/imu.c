#include "imu.h"

extern SPI_HandleTypeDef hspi1;

HAL_StatusTypeDef imu_read_reg(uint8_t reg, uint8_t* val) {
	uint8_t tx[2] = { reg | 0x80, 0x00 };
	uint8_t rx[2] = { 0 };
	HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_RESET);
	HAL_StatusTypeDef status = HAL_SPI_TransmitReceive(&hspi1, tx, rx, 2, 100);
	HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_SET);

	if(status == HAL_OK) {
		*val = rx[1];
	}
	return status;
}

HAL_StatusTypeDef imu_write_reg(uint8_t reg, uint8_t val) {
	uint8_t tx[2] = { reg & 0x7F, val };
	HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_RESET);
	HAL_StatusTypeDef status = HAL_SPI_Transmit(&hspi1, tx, 2, 100);
	HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_SET);
	return status;
}

HAL_StatusTypeDef imu_burst_read_regs(uint8_t startReg, uint8_t* buf, uint16_t len) {
	if(buf == NULL || len == 0) return HAL_ERROR;
	uint8_t addr = startReg | 0x80;
	HAL_StatusTypeDef ret = HAL_OK;
	HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_RESET);
	if(HAL_SPI_Transmit(&hspi1, &addr, 1, 100) != HAL_OK || HAL_SPI_Receive(&hspi1, buf, len, 100) != HAL_OK) {
		ret = HAL_ERROR;
	}
	HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_SET);
	return ret;
}

HAL_StatusTypeDef imu_configure_accelerometer(IMU_LA_ODR_Setting odr, IMU_LA_FS_Setting fs, IMU_LA_LPF2_Setting lpf2) {
	uint8_t val = (uint8_t) (odr | fs | lpf2);
	return imu_write_reg(IMU_REG_CTRL1_XL, val);
}

HAL_StatusTypeDef imu_configure_gyroscope(IMU_G_ODR_Setting odr, IMU_G_FS_Setting fs) {
	uint8_t val = (uint8_t) (odr | fs);
	return imu_write_reg(IMU_REG_CTRL2_G, val);
}

HAL_StatusTypeDef imu_enable_bdu(void) {
	uint8_t val;
	HAL_StatusTypeDef status = imu_read_reg(IMU_REG_CTRL3_C, &val);
	if(status != HAL_OK) return status;
	val |= IMU_REG_CTRL3_C_BDU;
	return imu_write_reg(IMU_REG_CTRL3_C, val);
}

IMU_READING imu_read_all(IMU_LA_FS_Setting fsLA, IMU_G_FS_Setting fsG) {
	uint8_t raw[ALL_READ_LENGTH] = { 0 };
	IMU_READING reading = { 0 };
	HAL_StatusTypeDef st = imu_burst_read_regs(ALL_READ_START, raw, ALL_READ_LENGTH);
	float gs = G_FS_to_Sensitivity(fsG);
	float as = LA_FS_to_Sensitivity(fsLA);
	reading.ERROR = true;
	if(st != HAL_OK || gs == SENSITIVITY_CONVERSION_FAIL || as == SENSITIVITY_CONVERSION_FAIL) return reading;

	#define RAW16(h, l) ((int16_t) (((uint16_t) raw[h] << 8) | raw[l]))
	reading.TEMP = TEMPERATURE_OFFSET + RAW16(IMU_IDX_OUT_TEMP_H, IMU_IDX_OUT_TEMP_L) * TEMPERATURE_SENSITIVITY;
	reading.G_X = RAW16(IMU_IDX_OUTX_H_G, IMU_IDX_OUTX_L_G) * gs;
	reading.G_Y = RAW16(IMU_IDX_OUTY_H_G, IMU_IDX_OUTY_L_G) * gs;
	reading.G_Z = RAW16(IMU_IDX_OUTZ_H_G, IMU_IDX_OUTZ_L_G) * gs;
	reading.LA_X = RAW16(IMU_IDX_OUTX_H_A, IMU_IDX_OUTX_L_A) * as;
	reading.LA_Y = RAW16(IMU_IDX_OUTY_H_A, IMU_IDX_OUTY_L_A) * as;
	reading.LA_Z = RAW16(IMU_IDX_OUTZ_H_A, IMU_IDX_OUTZ_L_A) * as;
	#undef RAW16

	reading.ERROR = false;
	return reading;
}
