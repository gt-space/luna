#ifndef IMU_H
#define IMU_H

#include "main.h"

typedef struct {
    int16_t accel_x;
    int16_t accel_y;
    int16_t accel_z;
    int16_t gyro_x;
    int16_t gyro_y;
    int16_t gyro_z;
} IMU_RawData;

typedef struct {
    float accel_x;
    float accel_y;
    float accel_z;
    float gyro_x;
    float gyro_y;
    float gyro_z;
} IMU_Data;

HAL_StatusTypeDef imu_read_register(
    SPI_HandleTypeDef *spi,
    uint8_t address,
    uint8_t *value
);

HAL_StatusTypeDef imu_write_register(
    SPI_HandleTypeDef *spi,
    uint8_t address,
    uint8_t value
);

HAL_StatusTypeDef imu_init(SPI_HandleTypeDef *spi);

HAL_StatusTypeDef imu_read_data(
    SPI_HandleTypeDef *spi,
    IMU_RawData *data
);

void imu_convert_data(
    const IMU_RawData *raw,
    IMU_Data *data
);

#endif