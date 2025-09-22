#include "LIS3MDL.h"
#include "MS5611.h"
#include "ASM330LHGB1.h"
#include "SPI_Device.h"
#include "math.h"
#include "string.h"

// Need something to define the data transmission rate/port/pin for init

typedef enum {
  REQUEST_DATA = 0x00,
  SYNC_CLOCK = 0x01,
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
  float temperature;
  float pressure;
} flow_data_t;

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

typedef struct {
  flow_data_t flow_data;
  heading_data_t heading_data;
  acceleration_data_t acceleration_data;
} sensor_data_t;

typedef struct {
  sensor_data_t sensor_data;
  u_int32_t checksum;
} data_to_send_t;

bool getFlow(baro_handle_t* baro, spi_device_t* baroSPI, flow_data_t* flow_data);

bool getHeading(mag_handler_t* mag, spi_device_t* magSPI, heading_data_t* heading_data);

bool getAcceleration(imu_handler_t* imu, spi_device_t* imuSPI, acceleration_data_t* acceleration_data);

// TBD
double getLocation();

bool calculate_checksum(sensor_data_t* data);
