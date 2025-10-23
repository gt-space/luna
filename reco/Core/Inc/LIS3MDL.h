#ifndef _LIS3MDL
#define _LIS3MDL

#define MAG_MIN_REG 0x0
#define MAG_MAX_REG 0x33
#define MAG_CTRL_REG_NUM 0x5

#include "stdbool.h"
#include "SPI_Device.h"
#include "stm32h5xx_hal.h"

// bit fields start at the 0th bit

typedef union __attribute__((packed)) {
    uint8_t reg;
    struct {
    	uint8_t ST 		 	: 1;
    	uint8_t FAST_ODR 	: 1;
    	uint8_t DO			: 3;
    	uint8_t OMXY	    : 2;
    	uint8_t TEMP_EN		: 1;
    } flags;
} ctrl_reg1_t;

typedef union __attribute__((packed)) {
    uint8_t reg;
    struct {
    	uint8_t NOT_USED_1	: 2;
    	uint8_t SOFT_RST	: 1;
    	uint8_t REBOOT	    : 1;
    	uint8_t NOT_USED_2	: 1;
    	uint8_t FS			: 2;
    	uint8_t NOT_USED_3	: 1;
    } flags;
} ctrl_reg2_t;

typedef union __attribute__((packed)) {
    uint8_t reg;
    struct {
    	uint8_t MD			: 2;
    	uint8_t SIM			: 1;
    	uint8_t NOT_USED_1	: 2;
    	uint8_t LP			: 1;
    	uint8_t NOT_USED_2  : 2;
    } flags;
} ctrl_reg3_t;

typedef union __attribute__((packed)) {
    uint8_t reg;
    struct {
    	uint8_t NOT_USED_1  : 1;
    	uint8_t BLE			: 1;
    	uint8_t OMZ			: 2;
    	uint8_t NOT_USED_2  : 4;
    } flags;
} ctrl_reg4_t;

typedef union __attribute__((packed)) {
    uint8_t reg;
    struct {
    	uint8_t NOT_USED_1  : 6;
    	uint8_t BDU			: 1;
    	uint8_t FAST_READ   : 1;
    } flags;
} ctrl_reg5_t;

typedef enum {
	MAG_OFFSET_X_REG_L_M = 0x05,
	MAG_OFFSET_X_REG_H_M = 0x06,
	MAG_OFFSET_Y_REG_L_M = 0x07,
	MAG_OFFSET_Y_REG_H_M = 0x08,
	MAG_OFFSET_Z_REG_L_M = 0x09,
	MAG_OFFSET_Z_REG_H_M = 0x0A,
	MAG_WHO_AM_I = 0x0F,
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
	MAG_ST_ENABLE = 1,
	MAG_ST_DISABLE = 0,
} CTRL_REG1_ST_FLAGS_T;

typedef enum {
	MAG_FAST_ODR_ENABLE = 1,
	MAG_FAST_ODR_DISABLE = 0,
} CTRL_REG1_FASTODR_FLAGS_T;

typedef enum {
	MAG_DO_LESS_THAN_1HZ = 0b000,
	MAG_DO_1_HZ		     = 0b001,
	MAG_DO_2_HZ		     = 0b010,
	MAG_DO_5_HZ			 = 0b011,
	MAG_DO_10_HZ		 = 0b100,
	MAG_DO_20_HZ		 = 0b101,
	MAG_DO_40_HZ		 = 0b110,
	MAG_DO_80_HZ         = 0b111,
} CTRL_REG1_DO_FLAGS_T;

typedef enum {
	MAG_OM_XY_LP 			 	  = 0b00,
	MAG_OM_XY_MP			 	  = 0b01,
	MAG_OM_XY_HP			 	  = 0b10,
	MAG_OM_XY_UHP			 	  = 0b11,
} CTRL_REG1_OM_XY_FLAGS_T;

typedef enum {
	MAG_TEMP_ENABLE 			  = 1,
	MAG_TEMP_DISABLE 			  = 0,
} CTRL_REG1_TEMP_FLAGS_T;

typedef enum {
	MAG_SOFT_RST_DISABLE		  = 0,		// Default Value
	MAG_SOFT_RST_ENABLE  		  = 1,
} CTRL_REG2_SOFT_RST_FLAGS_T;

typedef enum {
	MAG_REBOOT_NORMAL 			  = 0, 		// Default Value
	MAG_REBBOT_MEM_CONTENT		  = 1,
} CTRL_REG2_REBOOT_FLAGS_T;

typedef enum {
	MAG_FS_4GAUSS				  = 0b00,
	MAG_FS_8GAUSS				  = 0b01,
	MAG_FS_12GAUSS				  = 0b10,
	MAG_FS_16GAUSS				  = 0b11,
} CTRL_REG2_FS_FLAGS_T;

typedef enum {
	MAG_LP_ENABLE				  = 0,	   // Default Value
	MAG_LP_DISABLE				  = 1,
} CTRL_REG3_LP_FLAGS_T;

typedef enum {
	MAG_SPI_4_WIRE				  = 0,     // Default Value
	MAG_SPI_3_WIRE				  = 1,
} CTRL_REG3_SIM_FLAGS_T;

typedef enum {
	MAG_CONINUOUS_CONV			  = 0b00,
	MAG_SINGLE_CONV				  = 0b01,
	MAG_POWER_DOWN1 			  = 0b10,
	MAG_POWER_DOWN2				  = 0b11,
} CTRL_REG3_MD_FLAGS_T;

typedef enum {
	MAG_OM_Z_LP					  = 0b00,
	MAG_OM_Z_MP					  = 0b01,
	MAG_OM_Z_HP					  = 0b10,
	MAG_OM_Z_UHP				  = 0b11,
} CTRL_REG4_OM_Z_FLAGS_T;

typedef enum {
	MAG_LITTLE_ENDIAN			  = 0,
	MAG_BIG_ENDIAN				  = 1,
} CTRL_REG4_BLE_FLAGS_T;

typedef enum {
	MAG_FAST_READ_ENABLE		  = 1,
	MAG_FAST_READ_DISABLE		  = 0,
} CTRL_REG5_FAST_READ_FLAGS_T;

typedef enum {
	MAG_BDU_CONTINUOUS			  = 0, // Default value
	MAG_BDU_NOT_CONTINUOUS		  = 1,
} CTRL_REG5_BDU_FLAGS_T;

// The 0th index of the modifiedRegister is the ctrl_reg1, the 1nd
// index is ctrl_reg2, the 2rd is ctrl_reg3 and so on and so forth.
typedef struct {
	ctrl_reg1_t ctrl_reg1;
	ctrl_reg2_t ctrl_reg2;
	ctrl_reg3_t ctrl_reg3;
	ctrl_reg4_t ctrl_reg4;
	ctrl_reg5_t ctrl_reg5;
	float sensitivity; // Gauss
	bool modifiedRegisters[MAG_CTRL_REG_NUM];
} mag_handler_t;

typedef enum {
  MAG_COMMS_OK       			 = 0x00,
  MAG_COMMS_ERROR    			 = 0x01,
  MAG_COMMS_BUSY    			 = 0x02,
  MAG_COMMS_TIMEOUT 			 = 0x03,
  MAG_INVALID_REG			 	 = 0x04,
} mag_status_t;


mag_status_t lis3mdl_initialize_mag(spi_device_t* magSPI,
					       	   	    mag_handler_t* magHandler);

uint8_t lis3mdl_generate_reg_address(mag_reg_t regAddress,
								 	 bool readFlag,
									 bool consecutiveFlag);

mag_status_t lis3mdl_write_single_reg(spi_device_t* magSPI,
									  mag_reg_t magRegNum,
									  uint8_t valueToWrite);

mag_status_t lis3mdl_read_single_reg(spi_device_t* magSPI,
		                             mag_reg_t magRegNum,
								     uint8_t* recievedData);

mag_status_t lis3mdl_read_double_reg(spi_device_t* magSPI,
								   	 mag_reg_t upperRegAddress,
									 mag_reg_t lowerRegAddress,
									 uint16_t* receivedData);

mag_status_t lis3mdl_read_multiple_reg(spi_device_t* magSPI,
									   mag_reg_t startRegNum,
									   mag_reg_t endRegNum,
									   uint8_t* regReadValues);

mag_status_t lis3mdl_write_multiple_reg(spi_device_t* magSPI,
									    mag_reg_t startRegNum,
										mag_reg_t endRegNum,
										uint8_t* valuesToWrite);


mag_status_t lis3mdl_get_x_mag(spi_device_t* magSPI,
					 	 	   mag_handler_t* magHandler,
							   float* xOutputMag);

mag_status_t lis3mdl_get_y_mag(spi_device_t* magSPI,
					 	 	   mag_handler_t* magHandler,
							   float* yOutputMag);

mag_status_t lis3mdl_get_z_mag(spi_device_t* magSPI,
					 	 	   mag_handler_t* magHandler,
							   float* zOutputMag);

#endif

