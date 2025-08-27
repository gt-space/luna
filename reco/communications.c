#include "communications.h"

// Need something to define the data transmission rate/port/pin for init

bool getFlow(baro_handle_t* baro, spi_device_t* baroSPI, flow_data_t* flow_data) {
  baro_status_t baroResp = getCurrTempPressure(baroSPI, baro);
  if (baroResp != BARO_COMMS_OK) {
    return false;
  }
  flow_data->pressure = baro->pressure;
  flow_data->temperature = baro->temperature;
  return true
}

bool getHeading(mag_handler_t* mag, spi_device_t* magSPI, heading_data_t* heading_data) {
  mag_status_t xMagResp = lis3mdl_get_x_mag(magSPI, mag, &heading_data->xMag);
  mag_status_t yMagResp = lis3mdl_get_y_mag(magSPI, mag, &heading_data->yMag);
  mag_status_t zMagResp = lis3mdl_get_z_mag(magSPI, mag, &heading_data->zMag);
  if (xMagResp != IMU_COMMS_OK ||
      yMagResp != IMU_COMMS_OK ||
      zMagResp != IMU_COMMS_OK) {
      return false;
  }
  return true;
}

bool getAcceleration(imu_handler_t* imu, spi_device_t* imuSPI, acceleration_data_t* acceleration_data) {
  imu_status_t xAccelResp = getXAccel(imuSPI, imu, &acceleration_data->xActualAccel);
  imu_status_t yAccelResp = getYAccel(imuSPI, imu, &acceleration_data->yActualAccel);
  imu_status_t zAccelResp = getZAccel(imuSPI, imu, &acceleration_data->zActualAccel);
  imu_status_t pitchResp = getPitch(imuSPI, imu, &acceleration_data->pitch);
  imu_status_t rollResp = getRoll(imuSPI, imu, &acceleration_data->roll);
  imu_status_t yawResp = getYaw(imuSPI, imu, &acceleration_data->yaw);

  if (xAccelResp != IMU_COMMS_OK ||
      yAccelResp != IMU_COMMS_OK ||
      zAccelResp != IMU_COMMS_OK ||
      pitchResp  != IMU_COMMS_OK ||
      rollResp   != IMU_COMMS_OK ||
      yawResp    != IMU_COMMS_OK) {
      return false;
  }
  return true;
}

// TBD
double getLocation();

bool calculate_checksum(CRC_HandleTypeDef* hcrc, sensor_data_t* sensor_data) {
  // there are a couple of things that need to be done for this to work
  // 1. Create a CRC_HandleTypeDef
  // 2. Use MX_CRC_Init(void) to config the hcrc correctly
  //    hcrc.Init.InputDataFormat needs to be set correctly to handle the use of floats : CRC_INPUTDATA_FORMAT_WORDS
  // 3. Pass in the hcrc into the function

  float data[11] = {
    sensor_data->flow_data.pressure,
    sensor_data->flow_data.temperature,
    sensor_data->heading_data.xMag,
    sensor_data->heading_data.yMag,
    sensor_data->heading_data.zMag,
    sensor_data->acceleration_data.xActualAccel,
    sensor_data->acceleration_data.yActualAccel,
    sensor_data->acceleration_data.zActualAccel,
    sensor_data->acceleration_data.pitch,
    sensor_data->acceleration_data.roll,
    sensor_data->acceleration_data.yaw,
  };

  uint32_t crc = HAL_CRC_Calculate(hcrc, (uint32_t*)data, 11);
  sensor_data->checksum = crc;
  return true;
}

