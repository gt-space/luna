/*
 * gps.h
 *
 *      Author: Rushyendranath Reddy Nalamalapu
 *      Notes: Need to resolve discrepancy between GPS data loop of 20Hz
 *       and the required design of 200 Hz.
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
    bool valid_data_flag;
} gps_data_t;

void extract_gps_data(gps_data_t *gps_output);

#endif // BAM_GPS_DRIVER_H