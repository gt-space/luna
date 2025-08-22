#include "communications.h"

// Need something to define the data transmission rate/port/pin for init

// Returns the following in order?
// Pressure ( baroHandle->pressure )
// Temperature ( baroHandle->temperature )
double getFlow() {
  baro_handle_t* baro = comms_handle_t->sensors->baroHandler;
  spi_device_t* baroSPI = comms_handle_t->spi->baroSPIDevice;
  getCurrTempPressure(baroSPI, baro);

  // return temp and pressure from baro
}

// Returns the following in order?
// X-axis Magnetic Field Reading ( xActualMag )
// Y-axis Magnetic Field Reading ( yActualMag )
// Z-axis Magnetic Field Reading ( zActualMag )
double getHeading() {
  mag_handler_t* mag = comms_handle_t->sensors->magHandler;
  spi_device_t* magSPI = comms_handle_t->spi->magSPIDevice;

  double xActualMag = getXMag(magSPI, mag);
	double yActualMag = getYMag(magSPI, mag);
	double zActualMag = getZMag(magSPI, mag);
  
  // return the 3 values
}

// Returns the following in order?
// X/Y/Z Linear Acceleration ( xActualAccel , yActualAccel, zActualAccel )
// X/Y/Z Angular Acceleration ( pitch, roll , yaw )
double getAcceleration() {
  //imu_handler_t* imu = comms_handle_t->sensors->imuHandler;
  //spi_device_t* imuSPI = comms_handle_t->spi->imuSPIDevice;
  // double xActualAccel = getXAccel(imuSPI);
  // double yActualAccel = getYAccel(imuSPI);
  // double zActualAccel = getZAccel(imuSPI);

  // double pitch = getPitch(imuSPI);
  // double roll = getRoll(imuSPI);
  // double yaw = getYaw(imuSPI);

  // return the 6 values
}

// TBD
double getLocation();

// Returns whether the FC has requested data
int FCNeedsData() {
  
}

double sendData(communications_commands_t command) {
  
}
