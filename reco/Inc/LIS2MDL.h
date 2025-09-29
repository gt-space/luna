#ifndef _LIS2MDL
#define _LIS2MDL

#define MAG_MIN_REG 0x0
#define MAG_MAX_REG 0x6F
#define MAG_CTRL_REG_NUM 0x4

#include "stdbool.h"
#include "SPI_Device.h"
#include "stm32h5xx_hal.h"

typedef enum {
	MAG_OFFSET_X_REG_L = 0x45,
	MAG_OFFSET_X_REG_H = 0x46,
	MAG_OFFSET_Y_REG_L = 0x47,
	MAG_OFFSET_Y_REG_H = 0x48,
	MAG_OFFSET_Z_REG_L = 0x49,
	MAG_OFFSET_Z_REG_H = 0x4A,
	MAG_WHO_AM_I	   = 0x4F,
	MAG_CFG_REG_A	   = 0x60,
	MAG_CFG_REG_B      = 0x61,
	MAG_CFG_REG_C	   = 0x62,
	MAG_INT_CRTL_REG   = 0x63,
	MAG_INT_SOURCE_REG = 0x64,
	MAG_INT_THS_L_REG  = 0x65,
	MAG_INT_THS_H_REG  = 0x66,
	MAG_STATUS_REG     = 0x67,
	MAG_OUTX_L_REG	   = 0x68,
	MAG_OUTX_H_REG     = 0x69,
	MAG_OUTY_L_REG	   = 0x6A,
	MAG_OUTY_H_REG     = 0x6B,
	MAG_OUTZ_L_REG     = 0x6C,
	MAG_OUTZ_H_REG	   = 0x6D,
	MAG_TEMP_OUT_L_REG = 0x6E,
	MAG_TEMP_OUT_H_REG = 0x6F,
} mag_reg_t;

// bit fields start at the 0th bit

typedef union __attribute__((packed)) {
    uint8_t reg;
    struct {
    	uint8_t MD					: 2;
    	uint8_t ODR					: 2;
    	uint8_t LP					: 1;
    	uint8_t SOFT_RST			: 1;
    	uint8_t REBOOT      		: 1;
    	uint8_t COMP_TEMP_EN		: 1;
    } flags;
} cfg_reg_a_t;

typedef union __attribute__((packed)) {
    uint8_t reg;
    struct {
    	uint8_t LPF					: 1;
    	uint8_t OFF_CANC			: 1;
    	uint8_t SET_FREQ			: 1;
    	uint8_t INT_ON_DATA_OFF		: 1;
    	uint8_t OFF_CANC_ONE_SHOT 	: 1;
    	uint8_t NOT_USED			: 3;
    } flags;
} cfg_reg_b_t;

typedef union __attribute__((packed)) {
    uint8_t reg;
    struct {
    	uint8_t DRDY_ON_PIN			: 1;
    	uint8_t SELF_TEST			: 1;
    	uint8_t SIM					: 1;
    	uint8_t BLE					: 1;
    	uint8_t BDU		      		: 1;
    	uint8_t I2C_DIS				: 1;
    	uint8_t INT_ON_PIN			: 1;
    	uint8_t NOT_USED			: 1;
    } flags;
} cfg_reg_c_t;

typedef union __attribute__((packed)) {
	uint8_t reg;
	struct {
		uint8_t IEN					: 1;
		uint8_t IEL					: 1;
		uint8_t IEA					: 1;
		uint8_t NOT_USED			: 2;
		uint8_t ZIEN				: 1;
		uint8_t YIEN				: 1;
		uint8_t XIEN				: 1;
	} flags;
} int_ctrl_reg_t;

// CFG_REG_A Flags

typedef enum {
	MAG_COMP_TEMP_DISABLE		  = 0, // Default
	MAG_COMP_TEMP_ENABLE		  = 1,
} CTRL_REGA_COMP_TEMP_FLAGS_T;

typedef enum {
	MAG_REBOOT_MEM				  = 0, // Default
	MAG_NO_REBOOT_MEM			  = 1,
} CTRL_REGA_REBOOT_FLAGS_T;

typedef enum {
	MAG_SOFT_RST_DISABLE		  = 0, // Default
	MAG_SOFT_RST_ENABLE			  = 1,
} CTRL_REGA_SOFT_RST_FLAGS_T;

typedef enum {
	MAG_HIGH_RESOLUTION			  = 0, // Default
	MAG_LOW_POWER				  = 1,
} CTRL_REGA_LP_FLAGS_T;

typedef enum {
	MAG_ODR_10_HZ				  = 0b00, // Default
	MAG_ODR_20_HZ				  = 0b01,
	MAG_ODR_50_HZ				  = 0b10,
	MAG_ODR_100_HZ				  = 0b11,
} CTRL_REGA_ODR_FLAGS_T;

// Note: IDLE_MODE_1 and IDLE_MODE_2 are identical
typedef enum {
	MAG_CONTINUOUS_MODE			  = 0b00,
	MAG_SINGLE_MODE				  = 0b01,
	MAG_IDLE_MODE_1				  = 0b10,
	MAG_IDLE_MODE_2				  = 0b11, // Default
} CTRL_REGA_MD_FLAGS_T;

// CFG_REG_B Flags

typedef enum {
	MAG_SINGLE_OFF_CANC_DISABLE   = 0, // Default
	MAG_SINGLE_OFF_CANC_ENABLE	  = 1,
} CTRL_REGB_OFF_CANC_ONE_SHOT_FLAGS_T;

typedef enum {
	MAG_INT_AFTER_CORRECTION	  = 0, // Default
	MAG_INT_BEFORE_CORRECTION	  = 1,
} CTRL_REGB_INT_ON_DATAOFF_FLAGS_T;

typedef enum {
	MAG_SET_PULSE_64_ODR		  = 0, // Default
	MAG_SET_PULSE_PD			  = 1,
} CTRL_REGB_SET_FREQ_FLAGS_T;

typedef enum {
	MAG_OFFSET_CANCEL_DISABLE	  = 0, // Default
	MAG_OFFSET_CANCEL_ENABLE	  = 1,
} CTRL_REGB_OFF_CANC_FLAGS_T;

typedef enum {
	MAG_LPF_DISABLE				  = 0, // Default
	MAG_LPF_ENABLE				  = 1,
} CTRL_REGB_LPF_FLAGS_T;

// CFG_REG_C Flags

typedef enum {
	MAG_DEFAULT_INT				  = 0, // Default
	MAG_INT_ON_DRDY				  = 1,
} CTRL_REGC_INT_ON_PIN_FLAGS_T;

typedef enum {
	MAG_ENABLE_I2C				  = 0, // Default
	MAG_DISABLE_I2C				  = 1,
} CTRL_REGC_I2C_DIS_FLAGS_T;

typedef enum {
	MAG_BDU_DISABLE				  = 0, // Default
	MAG_BDU_ENABLE				  = 1,
} CTRL_REGC_BDU_FLAGS_T;

typedef enum {
	MAG_BLE_DISABLE				  = 0, // Default
	MAG_BLE_ENABLE				  = 1,
} CTRL_REGC_BLE_FLAGS_T;

typedef enum {
	MAG_SPI_4_WIRE				  = 1,
	MAG_SPI_3_WIRE				  = 0, // Default
} CTRL_REGC_SIM_FLAGS_T;

typedef enum {
	MAG_SELF_TEST_DISABLE 		  = 0, // Default
	MAG_SELF_TEST_ENABLE 		  = 1,
} CFG_REGC_ST_FLAGS_T;

typedef enum {
	MAG_DEFAULT_DRDY			  = 0, // Default
	MAG_DRDY_ON_INT				  = 1,
} CFG_REGC_DRDY_ON_PIN_FLAGS_T;

typedef enum
{
  MAG_COMMS_OK       			 = 0x00,
  MAG_COMMS_ERROR    			 = 0x01,
  MAG_COMMS_BUSY    			 = 0x02,
  MAG_COMMS_TIMEOUT 			 = 0x03,
  MAG_INVALID_REG				 = 0x04,
} mag_status_t;

typedef struct {
	cfg_reg_a_t cfg_reg_a;
	cfg_reg_b_t cfg_reg_b;
	cfg_reg_c_t cfg_reg_c;
	int_ctrl_reg_t int_ctrl_reg;
	float sensitivity; // mGauss
	bool modifiedRegisters[MAG_CTRL_REG_NUM];
} mag_handler_t;

uint8_t lis2mdl_generate_reg_address(mag_reg_t imuRegNum,
									 bool readFlag);

mag_status_t lis2mdl_write_single_reg(spi_device_t* magSPI,
		                      	      mag_reg_t magRegNum,
									  uint8_t valueToWrite);

mag_status_t lis2mdl_read_single_reg(spi_device_t* magSPI,
									 mag_reg_t magRegNum,
									 uint8_t* recievedData);


mag_status_t lis2mdl_read_double_reg(spi_device_t* magSPI,
								     mag_reg_t upperRegAddress,
								     mag_reg_t lowerRegAddress,
								     uint16_t* receivedData);

mag_status_t lis2mdl_read_multiple_reg(spi_device_t* magSPI,
									   mag_reg_t startRegNum,
									   mag_reg_t endRegNum,
									   uint8_t* regReadValues);

mag_status_t lis2mdl_write_multiple_reg(spi_device_t* magSPI,
									   mag_reg_t startRegNum,
									   mag_reg_t endRegNum,
									   uint8_t* valuesToWrite);


mag_status_t lis2mdl_initialize_mag(spi_device_t* magSPI,
						   	   	    mag_handler_t* magHandler);

mag_status_t lis2mdl_get_x_mag(spi_device_t* magSPI,
					 	 	   mag_handler_t* magHandler,
							   float* magXOutput);

mag_status_t lis2mdl_get_y_mag(spi_device_t* magSPI,
		             	 	   mag_handler_t* magHandler,
							   float* magYOutput);

mag_status_t lis2mdl_get_z_mag(spi_device_t* magSPI,
		             	 	   mag_handler_t* magHandler,
							   float* magZOutput);

void set_lis2mdl_flags(mag_handler_t* magHandler);


#endif
