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

// Import necessary libraries
#include <stdint.h>
#include <stdbool.h>
#include "stm32h5xx_hal.h"

// I2C Address Macro
#define GPS_I2C_ADDR_7BIT (0x42u)                 // Module 7bit adddress
#define GPS_I2C_ADDR ((uint16_t)(GPS_I2C_ADDR_7BIT<<1)) // 0x42 is the 7-bit address, shift for STM HAL

// Return data type 
typedef enum {
  GPS_OK = 0,           // GNSS_FIX is ok
  GPS_NO_NEW_DATA,      // Likely to be called a lot due to Hz mismatch
  GPS_NO_FIX,           // Message decoded, but GNSS_FIX unclear
  GPS_I2C_ERROR,        // Inform if error occurs
} GPS_Status;

// Define data to hold 200 Hz Values
typedef struct {
    double latitude;
    double longitude;
    double altitude;
    uint8_t fix_type;
    bool fix_ok;
    bool GPS_DRDY;    // GPS Data Ready Flag
} gps_data_t;

/**  Call once I2C is running and is used to configure measurement
 *   Returns an error if config write failed at bus level.
 * 
 */ 
GPS_Status GPS_Init(I2C_HandleTypeDef *hi2c);


/** Call each look. gps_output position fields are modified
 *  ONLY if GPS_OK is valid.
 * 
 */
GPS_Status extract_gps_data(gps_data_t *gps_output);


#endif // BAM_GPS_DRIVER_H