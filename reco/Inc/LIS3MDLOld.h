#ifndef _LIS3MDL
#define _LIS3MDL

#define MAG_MAX_REG 0x33

#include "stdbool.h"
#include "SPI_Device.h"

typedef struct {
	uint8_t CTRL_REG_FLAGS[5];
	int16_t sensitivity;
} mag_handler_t;

typedef enum {
	MAG_WHO_AM_I = 0x0F,
	MAG_OFFSET_X_REG_L_M = 0x05,
	MAG_OFFSET_X_REG_H_M = 0x06,
	MAG_OFFSET_Y_REG_L_M = 0x07,
	MAG_OFFSET_Y_REG_H_M = 0x08,
	MAG_OFFSET_Z_REG_L_M = 0x09,
	MAG_OFFSET_Z_REG_H_M = 0x0A,
	MAG_CTRL_REG1 = 0x20,
	MAG_CTRL_REG2 = 0x21,
	MAG_CTRL_REG3 = 0x22,
	MAG_CTRL_REG4 = 0x23,
	MAG_CTRL_REG5 = 0x24,
	MAG_STATUS_REG = 0x27,
	MAG_OUT_X_L = 0x28,
	MAG_OUT_X_H = 0x29,
	MAG_OUT_Y_L = 0x2A,
	MAG_OUT_Y_H = 0x2B,
	MAG_OUT_Z_L = 0x2C,
	MAG_OUT_Z_H = 0x2D,
	MAG_TEMP_OUT_L = 0x2E,
	MAG_TEMP_OUT_H = 0x2F,
	MAG_INT_CFG = 0x30,
	MAG_INT_SRC = 0x31,
	MAG_INT_THS_L = 0x32,
	MAG_INT_THS_H = 0x33,
} mag_reg_t;

typedef enum {
	MAG_CTRL1_TEMP_ENABLE	      = (1 << 7),
	MAG_CTRL1_XY_LP 			  = 0,
	MAG_CTRL1_XY_MP         	  = (1 << 5),
	MAG_CTRL1_XY_HP				  = (1 << 6),
	MAG_CTRL1_XY_UHP			  = (1 << 6) | (1 << 5),
	MAG_CTRL1_FAST_ODR			  = (1 << 1),
	MAG_CTRL1_ST				  = (1 << 0)
} CTRL_REG1_CONFIG_FLAGS_T;

typedef enum {
	MAG_CTRL2_4_GAUSS		  	  = 0,
	MAG_CTRL2_8_GAUSS        	  = (1 << 5),
	MAG_CTRL2_12_GAUSS		  	  = (1 << 6),
	MAG_CTRL2_16_GAUSS		  	  = (1 << 6) | (1 << 5),
	MAG_CTRL2_REBOOT		  	  = (1 << 3),
	MAG_CTRL2_SOFT_RST		  	  = (1 << 2)
} CTRL_REG2_CONFIG_FLAGS_T;

typedef enum {
	MAG_CTRL3_CONTINUOUS		  = 0,
	MAG_CTRL3_SINGLE_CONVERT      = (1 << 0),
	MAG_CTRL3_POWER_DOWN_1		  = (1 << 1),
	MAG_CTRL3_POWER_DOWN_2		  = (1 << 1) | (1 << 0),
	MAG_CTRL3_SPI_3_WIRE		  = (1 << 2),
	MAG_CTRL3_LP_MODE		  	  = (1 << 5)
} CTRL_REG3_CONFIG_FLAGS_T;

typedef enum {
	MAG_CTRL4_Z_LP	  	  		  = 0,
	MAG_CTRL4_Z_MP         		  = (1 << 2),
	MAG_CTRL4_Z_HP		  		  = (1 << 3),
	MAG_CTRL4_Z_UHP		  		  = (1 << 3) | (1 << 2),
	MAG_CTRL4_BIG_ENDIAN 		  = (1 << 1)
} CTRL_REG4_CONFIG_FLAGS_T;

typedef enum {
	MAG_CTRL5_FAST_READ 		  = (1 << 7),
	MAG_CTRL5_BDU				  = (1 << 6)
} CTRL_REG5_CONFIG_FLAGS_T;

uint8_t generateMagAddressOld(mag_reg_t regAddress, bool readFlag, bool consecutiveFlag);
uint8_t ensureMagNotReservedOld(mag_reg_t regToCheck);
uint8_t* readMagMultipleRegistersOld(spi_device_t* magSPI, mag_reg_t regAddress, uint8_t numRegisters);
uint8_t readMagSingleRegisterOld(spi_device_t* magSPI, mag_reg_t regAddress);
uint16_t readMagDoubleRegisterOld(spi_device_t* magSPI, mag_reg_t upperRegAddress, mag_reg_t lowerRegAddress);
short writeMagRegisterOld(spi_device_t* magSPI, mag_reg_t regAddress, uint8_t valueToWrite);

/*
uint8_t getStatusRegisterOld(spi_device_t* magSPI);
double getXMag(spi_device_t* magSPI, mag_handler_t* magHandler);
double getYMag(spi_device_t* magSPI, mag_handler_t* magHandler);
double getZMag(spi_device_t* magSPI, mag_handler_t* magHandler);
double getXMagRaw(spi_device_t* magSPI, mag_handler_t* magHandler);
double getYMagRaw(spi_device_t* magSPI, mag_handler_t* magHandler);
double getZMagRaw(spi_device_t* magSPI, mag_handler_t* magHandler);

*/

#endif

// REG1 = 0b00000010
// REG2 = 0b00000000
// REG3 = 0b00000000
// REG4 = 0b00000000
// REG5 = 0b00000000
