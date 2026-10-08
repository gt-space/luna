/*
 * lsm6dsr.c
 *
 * Created on: Aug 22, 2026
 * Author: Edwin Wu
 */


#include "lsm6dsr.h"

uint8_t lsm6dsr_read_reg(SPI_HandleTypeDef *hspi, uint8_t reg)
{
    uint8_t transmit = reg | 0x80;
    uint8_t receive  = 0;

    HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(hspi, &transmit, 1, HAL_MAX_DELAY);
    HAL_SPI_Receive(hspi, &receive, 1, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_SET);

    return receive;
}

// Edwin Code
void lsm6dsr_write_reg(SPI_HandleTypeDef *hspi, uint8_t reg, uint8_t value)
{
	HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(hspi, &reg, 1, HAL_MAX_DELAY);
	HAL_SPI_Transmit(hspi, &value, 1, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_SET);

}

void lsm6dsr_read_data(SPI_HandleTypeDef *hspi, lsm6dsr_data_t *data)
{
	uint8_t lowX_A = lsm6dsr_read_reg(hspi, LSM6DSR_OUTX_L_A);
	uint8_t highX_A = lsm6dsr_read_reg(hspi, LSM6DSR_OUTX_L_A + 1);
	data->accelX = (int16_t)((highX_A << 8) | lowX_A);


	uint8_t lowY_A = lsm6dsr_read_reg(hspi, LSM6DSR_OUTX_L_A + 2);
	uint8_t highY_A = lsm6dsr_read_reg(hspi, LSM6DSR_OUTX_L_A + 3);
	data->accelY = (int16_t)((highY_A << 8) | lowY_A);

	uint8_t lowZ_A = lsm6dsr_read_reg(hspi, LSM6DSR_OUTX_L_A + 4);
	uint8_t highZ_A = lsm6dsr_read_reg(hspi, LSM6DSR_OUTX_L_A + 5);
	data->accelZ = (int16_t)((highZ_A << 8) | lowZ_A);


	uint8_t lowX_G = lsm6dsr_read_reg(hspi, LSM6DSR_OUTX_L_G);
	uint8_t highX_G = lsm6dsr_read_reg(hspi, LSM6DSR_OUTX_L_G + 1);
	data->gyroX = (int16_t)((highX_G << 8) | lowX_G);

	uint8_t lowY_G = lsm6dsr_read_reg(hspi, LSM6DSR_OUTX_L_G + 2);
	uint8_t highY_G = lsm6dsr_read_reg(hspi, LSM6DSR_OUTX_L_G + 3);
	data->gyroY = (int16_t)((highY_G << 8) | lowY_G);

	uint8_t lowZ_G = lsm6dsr_read_reg(hspi, LSM6DSR_OUTX_L_G + 4);
	uint8_t highZ_G = lsm6dsr_read_reg(hspi, LSM6DSR_OUTX_L_G + 5);
	data->gyroZ = (int16_t)((highZ_G << 8) | lowZ_G);



}




