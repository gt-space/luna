/*
 * lsm6dsr.h
 *
 *  Created on: Aug 9, 2026
 *  Author: Edwin Wu
 */

#ifndef INC_LSM6DSR_H_
#define INC_LSM6DSR_H_

#include <stdint.h>
#include <main.h>

/* ---- LSM6DSR register addresses (from the datasheet) ---- */
#define LSM6DSR_WHO_AM_I    0x0F   /* identity check, must read back 0x6B */
#define LSM6DSR_CTRL1_XL    0x10   /* accelerometer: data rate + range */
#define LSM6DSR_CTRL2_G     0x11   /* gyroscope: data rate + range */
#define LSM6DSR_CTRL3_C     0x12   /* control: block-update + auto-increment */
#define LSM6DSR_STATUS_REG  0x1E   /* flags when new data is ready */
#define LSM6DSR_OUTX_L_G    0x22   /* gyro output X,Y,Z start here */
#define LSM6DSR_OUTX_L_A    0x28   /* accel output X,Y,Z start here */

#define LSM6DSR_WHOAMI_EXPECTED  0x6B

// Edwin Struct
typedef struct {
	int16_t accelX;
	int16_t accelY;
	int16_t accelZ;

	int16_t gyroX;
	int16_t gyroY;
	int16_t gyroZ;
} lsm6dsr_data_t;

//Function Def
uint8_t lsm6dsr_read_reg(SPI_HandleTypeDef *hspi, uint8_t reg);
void lsm6dsr_write_reg(SPI_HandleTypeDef *hspi, uint8_t reg, uint8_t value);
void lsm6dsr_read_data(SPI_HandleTypeDef *hspi, lsm6dsr_data_t *data);



#endif /* INC_LSM6DSR_H_ */
