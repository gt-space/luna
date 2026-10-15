#include "imu.h"

HAL_StatusTypeDef imu_read_register(
    SPI_HandleTypeDef *spi,
    uint8_t address,
    uint8_t *value
) {
    uint8_t tx[2] = {address | 0x80, 0x00};
    uint8_t rx[2] = {0};

    HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_RESET);

    HAL_StatusTypeDef status =
        HAL_SPI_TransmitReceive(spi, tx, rx, 2, 100);

    HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_SET);

    if (status == HAL_OK) {
        *value = rx[1];
    }

    return status;
}

HAL_StatusTypeDef imu_write_register(
    SPI_HandleTypeDef *spi,
    uint8_t address,
    uint8_t value
) {
    uint8_t tx[2] = {address & 0x7F, value};

    HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_RESET);

    HAL_StatusTypeDef status =
        HAL_SPI_Transmit(spi, tx, 2, 100);

    HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_SET);

    return status;
}

HAL_StatusTypeDef imu_init(SPI_HandleTypeDef *spi)
{
    HAL_StatusTypeDef status;

    status = imu_write_register(spi, 0x12, 0x44);
    if (status != HAL_OK) {
        return status;
    }

    // accel: 208 Hz +- 8u
    status = imu_write_register(spi, 0x10, 0x5C);
    if (status != HAL_OK) {
        return status;
    }

    // gyro: 208 Hz, +-500u
    return imu_write_register(spi, 0x11, 0x54);
}

void imu_convert_data(
    const IMU_RawData *raw,
    IMU_Data *data
)
{
    const float accel_scale = 0.000244f * 9.80665f;

    const float gyro_scale = 0.0175f;

    data->accel_x = raw->accel_x * accel_scale;
    data->accel_y = raw->accel_y * accel_scale;
    data->accel_z = raw->accel_z * accel_scale;

    data->gyro_x = raw->gyro_x * gyro_scale;
    data->gyro_y = raw->gyro_y * gyro_scale;
    data->gyro_z = raw->gyro_z * gyro_scale;
}

static int16_t combine_bytes(uint8_t low, uint8_t high)
{
    uint16_t combined =
        (uint16_t)low | ((uint16_t)high << 8);

    return (int16_t)combined;
}

HAL_StatusTypeDef imu_read_data(
    SPI_HandleTypeDef *spi,
    IMU_RawData *data
)
{
    uint8_t bytes[12];

    for (uint8_t i = 0; i < 12; i++) {
        HAL_StatusTypeDef status =
            imu_read_register(spi, 0x22 + i, &bytes[i]);

        if (status != HAL_OK) {
            return status;
        }
    }

    data->gyro_x = combine_bytes(bytes[0], bytes[1]);
    data->gyro_y = combine_bytes(bytes[2], bytes[3]);
    data->gyro_z = combine_bytes(bytes[4], bytes[5]);

    data->accel_x = combine_bytes(bytes[6], bytes[7]);
    data->accel_y = combine_bytes(bytes[8], bytes[9]);
    data->accel_z = combine_bytes(bytes[10], bytes[11]);

    return HAL_OK;
}