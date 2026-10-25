/**
  ******************************************************************************
  * @file    ZED-F9P-02B.h
  * @author  Rushyendranath Reddy Nalamalapu
  * @brief   Header file for ZED-F9P-02B.c module.
  ******************************************************************************
  * Notes: Need to manage discrepancy between GPS data loop of 20Hz and design
  * req of 200Hz.
  */

// Guard preprocessor from multiple copies of the driver
#ifndef BAM_GPS_DRIVER_H
#define BAM_GPS_DRIVER_H

// Import standard libraries
#include <stdint.h>
#include <stdbool.h>

// I2C Address Macro
#define GPS_I2C_ADDR (0x42<<1) // 0x42 is the 7-bit address, shift for STM HAL

// Define data to hold 200 Hz Values
typedef struct {
    float latitude;
    float longitude;
    float altitude;
    bool GPS_DRDY; // GPS Data Ready Flag
} gps_data_t;

void extract_gps_data(gps_data_t *gps_output);

#endif // BAM_GPS_DRIVER_H