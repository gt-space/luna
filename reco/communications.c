#include "communications.h"

// Need something to define the data transmission rate/port/pin for init

void getFlow(baro_handle_t* baro, spi_device_t* baroSPI) {
  getCurrTempPressure(baroSPI, baro);
}

heading_data_t getHeading(mag_handler_t* mag, spi_device_t* magSPI) {
  heading_data_t heading_data;
  heading_data.xMag = getXMag(magSPI, mag);
  heading_data.yMag = getYMag(magSPI, mag);
  heading_data.zMag = getZMag(magSPI, mag);
  return heading_data;
}

acceleration_data_t getAcceleration(imu_handler_t* imu, spi_device_t* imuSPI) {
  acceleration_data_t acceleration_data;
  getXAccel(imuSPI, imu, acceleration_data.xActualAccel);
  getYAccel(imuSPI, imu, acceleration_data.yActualAccel);
  getZAccel(imuSPI, imu, acceleration_data.zActualAccel);
  getPitch(imuSPI, imu, acceleration_data.pitch);
  getRoll(imuSPI, imu, acceleration_data.roll);
  getYaw(imuSPI, imu, acceleration_data.yaw);
  return acceleration_data;
}

// TBD
double getLocation();
