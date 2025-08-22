#include "LIS3MDL.h"
#include "MS5611.h"
#include "ASM330LHGB1.h"
#include "SPI_Device.h"
#include "math.h"
#include "string.h"
// purpose is so FC can reliably send info and receive data from RECO

// Needs to do the following:
//    Allow the FC to send commands/messages to RECO
//    Match the command to its appropriate action
//    Send information back to the FC if it has requested data from RECO
//    Be flexible to support further usecases

// Constraints:
//    Must be done reliably (handle errors, recover)
//    Add minimum overhead to the overal system

// Protocol Specification:
//    Currently, Reco works in 2 steps:
//        1. Init sensors
//        2. Collect data from sensors
//        If any setup of the comm protocol needs to be done, it should be done during INIT
//    Comm protocol is envisioned to send four types of data
//        1. Flow Properties
//            Pressure (baroHandle->pressure)
//            Temperature (baroHandle->temperature)
//        2. Heading Data
//            X-axis Mag Reading (xActualMag)
//            Y-axis Mag Reading (yActualMag)
//            Z-axis Mag Reading (zActualMag)
//        3. Acceleration Data
//            X/Y/Z Linear Accel (xActualAccel, yActualAccel, zActualAccel)
//            X/Y/Z Angular Accel (pitch, roll, yaw)
//        4. Location Data (sourced from EKF)
//            X/Y/Z Location (xLoc, yLoc, zLoc)
//
//     For now, I will plan to request everything seperate
//
//     Check if FC is requesting Data, if so, send the data requested using the specific command
//
//     Use `FC_NCS_Pin` to check if FC is requesting data (read using `HAL_GPIO_ReadPin()`
//
//     Send/Receive data usinng `SPI_Device_Recieve()` and `SPI_Device_Transmit()`
//
//
//
//
//
//

typedef enum {
  REQUEST_FLOW = 0x01,
  REQUEST_HEADING = 0x02,
  REQUEST_ACCELERATION = 0x03,
  REQUEST_LOCATION = 0x04,
} communications_commands_t;

typedef struct {
  baro_handle_t* baroHandler;
  mag_handler_t* magHandler;
  imu_handler_t* imuHandler:
} comms_sensors_handler_t;

typedef struct {
  spi_device_t* baroSPIDevice;
  spi_device_t* magSPIDevice;
  spi_device_t* imuSPIDevice;
} comms_spi_handler_t;

typedef struct {
    comms_sensors_handler_t sensors;
    comms_spi_handler_t spi;
} comms_handle_t;

typedef struct {
  float xMax;
  float yMag;
  float zMag;
} heading_data_t;

typedef struct {
  float xActualAccel;
  float yActualAccel;
  float zActualAccel;
  float pitch;
  float roll;
  float yaw;
} acceleration_data_t;

// Need something to define the data transmission rate/port/pin for init

void getFlow();

heading_data_t getHeading();

acceleration_data_t getAcceleration();

// TBD
double getLocation();
