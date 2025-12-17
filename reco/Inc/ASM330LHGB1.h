#ifndef _ASM330LHBG1
#define _ASM330LHBG1

#define IMU_MAX_REG 0x7E
#define IMU_MIN_REG 0x01
#define IMU_CTRL_REG_NUM 11

#include "stdbool.h"
#include "SPI_Device.h"
#include "stm32h5xx_hal.h"
#include "arm_math_types.h"
#include "comms.h"
#include "trig_extensions.h"

typedef enum {
	IMU_ZERO_REG_PLACEHOLDER	 = 0x00,
    IMU_FUNC_CFG_ACCESS          = 0x01,
    IMU_PIN_CTRL                 = 0x02,
    IMU_FIFO_CTRL1               = 0x07,
    IMU_FIFO_CTRL2               = 0x08,
    IMU_FIFO_CTRL3               = 0x09,
    IMU_FIFO_CTRL4               = 0x0A,
    IMU_COUNTER_BDR_REG1         = 0x0B,
    IMU_COUNTER_BDR_REG2         = 0x0C,
    IMU_INT1_CTRL                = 0x0D,
    IMU_INT2_CTRL                = 0x0E,
    IMU_WHO_AM_I                 = 0x0F,
    IMU_CTRL1_XL                 = 0x10,
    IMU_CTRL2_G                  = 0x11,
    IMU_CTRL3_C                  = 0x12,
    IMU_CTRL4_C                  = 0x13,
    IMU_CTRL5_C                  = 0x14,
    IMU_CTRL6_C                  = 0x15,
    IMU_CTRL7_G                  = 0x16,
    IMU_CTRL8_XL                 = 0x17,
    IMU_CTRL9_XL                 = 0x18,
    IMU_CTRL10_C                 = 0x19,
    IMU_ALL_INT_SRC              = 0x1A,
    IMU_WAKE_UP_SRC              = 0x1B,
    IMU_D6D_SRC                  = 0x1D,
    IMU_STATUS_REG               = 0x1E,
    IMU_OUT_TEMP_L               = 0x20,
    IMU_OUT_TEMP_H               = 0x21,
    IMU_OUTX_L_G                 = 0x22,
    IMU_OUTX_H_G                 = 0x23,
    IMU_OUTY_L_G                 = 0x24,
    IMU_OUTY_H_G                 = 0x25,
    IMU_OUTZ_L_G                 = 0x26,
    IMU_OUTZ_H_G                 = 0x27,
    IMU_OUTX_L_A                 = 0x28,
    IMU_OUTX_H_A                 = 0x29,
    IMU_OUTY_L_A                 = 0x2A,
    IMU_OUTY_H_A                 = 0x2B,
    IMU_OUTZ_L_A                 = 0x2C,
    IMU_OUTZ_H_A                 = 0x2D,
    IMU_EMB_FUNC_STATUS_MAINPAGE = 0x35,
    IMU_FSM_STATUS_A_MAINPAGE    = 0x36,
    IMU_FSM_STATUS_B_MAINPAGE    = 0x37,
    IMU_MLC_STATUS_MAINPAGE      = 0x38,
    IMU_FIFO_STATUS1             = 0x3A,
    IMU_FIFO_STATUS2             = 0x3B,
    IMU_TIMESTAMP0_REG           = 0x40,
    IMU_TIMESTAMP1_REG           = 0x41,
    IMU_TIMESTAMP2_REG           = 0x42,
    IMU_TIMESTAMP3_REG           = 0x43,
    IMU_INT_CFG0                 = 0x56,
    IMU_INT_CFG1                 = 0x58,
    IMU_THS_6D                   = 0x59,
    IMU_WAKE_UP_THS              = 0x5B,
    IMU_WAKE_UP_DUR              = 0x5C,
    IMU_FREE_FALL                = 0x5D,
    IMU_MD1_CFG                  = 0x5E,
    IMU_MD2_CFG                  = 0x5F,
    IMU_I3C_BUS_AVB              = 0x62,
    IMU_INTERNAL_FREQ_FINE       = 0x63,
    IMU_X_OFS_USR                = 0x73,
    IMU_Y_OFS_USR                = 0x74,
    IMU_Z_OFS_USR                = 0x75,
    IMU_FIFO_DATA_OUT_TAG        = 0x78,
    IMU_FIFO_DATA_OUT_X_L        = 0x79,
    IMU_FIFO_DATA_OUT_X_H        = 0x7A,
    IMU_FIFO_DATA_OUT_Y_L        = 0x7B,
    IMU_FIFO_DATA_OUT_Y_H        = 0x7C,
    IMU_FIFO_DATA_OUT_Z_L        = 0x7D,
    IMU_FIFO_DATA_OUT_Z_H        = 0x7E
} imu_reg_t;

typedef union __attribute__((packed)) {
    uint8_t reg;
    struct {
    	uint8_t NOT_USED_1  : 6;
    	uint8_t SDO_PU_EN	: 1;
    	uint8_t NOT_USED_2  : 1;
    } flags;
} pin_ctrl_t;

typedef union __attribute__((packed)) {
    uint8_t reg;
    struct {
    	uint8_t NOT_USED_1  : 1;
    	uint8_t LPF2_XL_EN  : 1;
    	uint8_t FS_XL		: 2;
    	uint8_t ODR			: 4;
    } flags;
} ctrl1_xl_t;

typedef union __attribute__((packed)) {
    uint8_t reg;
    struct {
    	uint8_t FS_4000		: 1;
    	uint8_t FS_125		: 1;
    	uint8_t FS_G		: 2;
    	uint8_t ODR_G		: 4;
    } flags;
} ctrl2_g_t;

typedef union __attribute__((packed)) {
    uint8_t reg;
    struct {
    	uint8_t SW_RESET	: 1;
    	uint8_t NOT_USED_1  : 1;
    	uint8_t IF_INC      : 1;
    	uint8_t SIM			: 1;
    	uint8_t PP_OD		: 1;
    	uint8_t H_LACTIVE   : 1;
    	uint8_t BDU			: 1;
    	uint8_t BOOT		: 1;
    } flags;
} ctrl3_c_t;

typedef union __attribute__((packed)) {
    uint8_t reg;
    struct {
    	uint8_t NOT_USED_1		: 1;
    	uint8_t LPF1_SEL_G  	: 1;
    	uint8_t I2C_DISABLE 	: 1;
    	uint8_t DRDY_READY  	: 1;
    	uint8_t NOT_USED_2  	: 1;
    	uint8_t INT2_on_INT1    : 1;
    	uint8_t SLEEP_G			: 1;
    	uint8_t NOT_USED_3		: 1;
    } flags;
} ctrl4_c_t;

typedef union __attribute__((packed)) {
    uint8_t reg;
    struct {
    	uint8_t ST_XL			: 2;
    	uint8_t ST_G			: 2;
    	uint8_t NOT_USED_1      : 1;
    	uint8_t ROUNDING		: 2;
    	uint8_t NOT_USED_2		: 1;
    } flags;
} ctrl5_c_t;

typedef union __attribute__((packed)) {
    uint8_t reg;
    struct {
    	uint8_t FTYPE			: 3;
    	uint8_t USR_OFF_W		: 1;
    	uint8_t XL_HM_MODE		: 1;
    	uint8_t LVL2_EN			: 1;
    	uint8_t LVL1_EN			: 1;
    	uint8_t TRIG_EN			: 1;
    } flags;
} ctrl6_c_t;

typedef union __attribute__((packed)) {
    uint8_t reg;
    struct {
    	uint8_t NOT_USED_1		: 1;
    	uint8_t USR_OFF_ON_OUT  : 1;
    	uint8_t NOT_USED_2		: 2;
    	uint8_t HPM_G			: 2;
    	uint8_t HP_EN_G			: 1;
    	uint8_t G_HM_MODE		: 1;
    } flags;
} ctrl7_g_t;

typedef union __attribute__((packed)) {
    uint8_t reg;
    struct {
    	uint8_t LOW_PASS_ON_6D		: 1;
    	uint8_t NOT_USED_1			: 1;
    	uint8_t HP_SLOPE_XL_EN  	: 1;
    	uint8_t FASTSETTL_MODE_XL 	: 1;
    	uint8_t HP_REF_MODE_XL      : 1;
    	uint8_t HPCF_XL				: 3;
    } flags;
} ctrl8_xl_t;

typedef union __attribute__((packed)) {
    uint8_t reg;
    struct {
    	uint8_t NOT_USED_1			: 1;
    	uint8_t I3C_DISABLE			: 1;
    	uint8_t DEN_LH				: 1;
    	uint8_t DEN_XL_EN			: 1;
    	uint8_t DEN_XL_G			: 1;
    	uint8_t DEN_Z				: 1;
    	uint8_t DEN_Y				: 1;
    	uint8_t DEN_X				: 1;
    } flags;
} ctrl9_xl_t;

typedef union __attribute__((packed)) {
    uint8_t reg;
    struct {
    	uint8_t NOT_USED_1			: 5;
    	uint8_t TIMESTAMP_EN		: 1;
    	uint8_t NOT_USED_2			: 2;
    } flags;
} ctrl10_c_t;

typedef enum {
    IMU_ENABLE_MOSI  				= 1, // Select this to enable MOSI for SPI
    IMU_DISABLE_MOSI 				= 0, // Default
} PIN_CTRL_SDO_PU_EN_FLAGS_T;

typedef enum {
	IMU_LPF2_XL_ENABLE 				= 1,
	IMU_LPF2_XL_DISABLE 			= 0, // Disable
} CTRL1_XL_LPF2_XL_EN;

typedef enum {
	IMU_ACCEL_FS_XL_2G 				= 0b00,
	IMU_ACCEL_FS_XL_4G				= 0b10,
	IMU_ACCEL_FS_XL_8G				= 0b11,
	IMU_ACCEL_FS_XL_16G				= 0b01,
} CTRL1_XL_FS_XL_FLAGS_T;

typedef enum {
	// These are dependent on XL_HM_MODE in CTRL6_
	IMU_ACCEL_POWER_DOWN			= 0b0000,
	IMU_ACCEL_1POINT6_HZ			= 0b1011,
	IMU_ACCEL_12POINT5_HZ			= 0b0001,
	IMU_ACCEL_26_HZ					= 0b0010,
	IMU_ACCEL_52_HZ					= 0b0011,
	IMU_ACCEL_104_HZ				= 0b0100,
	IMU_ACCEL_208_HZ				= 0b0101,
	IMU_ACCEL_416_HZ				= 0b0110,
	IMU_ACCEL_833_HZ				= 0b0111,
	IMU_ACCEL_1667_HZ				= 0b1000,
} CTRL1_XL_ODR_XL_FLAGS_T;

typedef enum {
	IMU_GYRO_SELECT_FS125_FS_G		= 0,
	IMU_GYRO_FS_4000 				= 1,
} CTRL2_G_FS_4000HZ_FLAGS_T;

typedef enum {
	IMU_GYRO_SELECT_FS125_FSG		= 0,
	IMU_GYRO_FS_125					= 1,
} CTRL2_G_FS_125HZ_FLAGS_T;

typedef enum {
	IMU_GYRO_250_DPS				= 0b00,
	IMU_GYRO_500_DPS			    = 0b01,
	IMU_GYRO_1000_DPS				= 0b10,
	IMU_GYRO_2000_DPS				= 0b11,
} CTRL2_G_FS_G_FLAGS_T;

typedef enum {
	IMU_GYRO_ODR_POWER_DOWN			= 0b0000,
	IMU_GYRO_ODR_12POINT_HZ			= 0b0001,
	IMU_GYRO_ODR_26_HZ				= 0b0010,
	IMU_GYRO_ODR_52_HZ				= 0b0011,
	IMU_GYRO_ODR_104_HZ				= 0b0100,
	IMU_GYRO_ODR_208_HZ				= 0b0101,
	IMU_GYRO_ODR_416_HZ				= 0b0110,
	IMU_GYRO_ODR_833_HZ				= 0b0111,
	IMU_GYRO_ODR_1667_HZ			= 0b1000,
} CTRL2_G_ODR_G_FLAGS_T;

// Default is 0 for CTRL3 bits (exceptions noted)
typedef enum {
	IMU_REBOOT_MEM					= 0,
	IMU_NORMAL_MODE					= 1,
} CTRL3_C_BOOT_FLAGS_T;

typedef enum {
	IMU_BDU_DISABLE					= 0,
	IMU_BDU_ENABLE					= 1,
} CTRL3_C_BDU_FLAGS_T;

typedef enum {
	IMU_H_LACTIVE_INT_HIGH			= 0,
	IMU_H_LACTIVE_INT_LOW			= 1,
} CTRL3_C_H_LACTIVE_FLAGS_T;

typedef enum {
	IMU_PUSH_PULL					= 0,
	IMU_OPEN_DRAIN					= 1,
} CTRL3_C_PP_OD_FLAGS_T;

typedef enum {
	IMU_SPI_4_WIRE					= 0,
	IMU_SPI_3_WIRE					= 1,
} CTRL3_C_SIM_FLAGS_T;

typedef enum {
	IMU_MULTI_INCREMENT_DISABLE		= 0,
	IMU_MULTI_INCREMENT_ENABLE		= 1, // Default
} CTRL3_C_IF_INC_FLAGS_T;

typedef enum {
	IMU_SW_RESET_NORMAL				= 0,
	IMU_SW_RESET_DEVICE				= 1,
} CTRL3_C_SW_RESET_FLAGS_T;

typedef enum {
	IMU_GYRO_SLEEP_MODE_DISABLE		= 0,
	IMU_GYRO_SLEEP_MODE_ENABLE		= 1,
} CTRL4_C_SLEEP_G_FLAGS_T;

typedef enum {
	IMU_USE_BOTH_INT				= 0,
	IMU_USE_ONLY_INT1				= 1,
} CTRL4_C_INT2_ON_INT1_FLAGS_T;

typedef enum {
	IMU_ENABLE_DRDY_IMMEDIATE		= 0,
	IMU_ENABLE_DRDY_SETTLE			= 1,
} CTRL4_DRDY_MASK_FLAGS_T;

typedef enum {
	IMU_ENABLE_COMMS_CTRL4			= 0,
	IMU_DISABLE_I2C					= 1,
} CTRL4_I2C_DIABLE_FLAGS_T;

typedef enum {
	IMU_GYRO_LPF_BANDWIDTH_DISABLE  = 0,
	IMU_GYRO_LPF_BANDWIDTH_FTYPE	= 1,
} CTRL4_LPF1_SEL_G_FLAGS_T;

typedef enum {
	IMU_DEN_EDGE_TRIG_DISABLE		= 0,
	IMU_DEN_EDGE_TRIP_ENABLE		= 1,
} CTRL6_TRIG_EN_FLAGS_T;

typedef enum {
	IMU_DEN_LEVEL_TRIG_DISABLE		= 0,
	IMU_DEN_LEVEL_TRIP_ENABLE		= 1,
} CTRL6_LVL1_EN_FLAGS_T;

typedef enum {
	IMU_DEN_LEVEL_LATCH_DISABLE		= 0,
	IMU_DEN_LEVEL_LATCH_ENABLE		= 1,
} CTRL6_LVL2_EN_FLAGS_T;

typedef enum {
	IMU_ENABLE_ACCEL_HIGH_PERF		= 0,
	IMU_DISABLE_ACCEL_HIGH_PERF		= 1,
} CTRL6_XL_HM_MODE_FLAGS_T;

typedef enum {
	IMU_HIGH_ACCEL_OFFSET_WEIGHT	= 0,
	IMU_LOW_ACCEL_OFFSET_WEIGHT		= 1,
} CTRL6_USR_OFF_W_FLAGS_T;

typedef enum {
	IMU_FTYPE_000					= 0b000,
	IMU_FTYPE_001					= 0b001,
	IMU_FTYPE_010					= 0b010,
	IMU_FTYPE_011					= 0b011,
	IMU_FTYPE_100					= 0b100,
	IMU_FTYPE_101					= 0b101,
	IMU_FTYPE_110					= 0b110,
	IMU_FTYPE_111					= 0b111,
} CTRL6_FTYPE_FLAGS_T;

typedef enum {
	IMU_ENABLE_GYRO_HIGH_PERF       = 0,
	IMU_DISABLE_GYRO_HIGH_PERF		= 1,
} CTRL7_G_HM_MODE_FLAGS_T;

typedef enum {
	IMU_GYRO_HPF_DISABLE			= 0,
	IMU_GYRO_HPF_ENABLE				= 1,
} CTRL7_G_HP_EN_G_FLAGS_T;

typedef enum {
	IMU_GYRO_HPF_CUTOFF_16MHZ		= 0b00,
	IMU_GYRO_HPF_CUTOFF_65MHZ		= 0b01,
	IMU_GYRO_HPF_CUTOFF_260MHZ		= 0b10,
	IMU_GYRO_HPF_CUTOFF_1POINT04HZ  = 0b11,
} CTRL7_G_HPM_G_FLAGS_T;

typedef enum {
	IMU_USR_OFF_ON_OUT_ENABLE		= 0,
	IMU_USR_OFF_ON_OUT_DISABLE		= 1,
} CTRL7_G_USR_OFF_ON_OUT_FLAGS_T;

typedef enum {
	IMU_ODR2_TO_6D_INT				= 0,
	IMU_LPF2_TO_6D_INT				= 1,
} CTRL8_XL_LOW_PASS_ON_6D_FLAGS_T;

typedef enum {
	// This shit really makes no sense unless you look at the figure
	IMU_HP_SLOPE_XL_DISABLE			= 0,
	IMU_HP_SLOPE_XL_ENABLE			= 1,
} CTRL8_G_HP_SLOPE_XL_EN_FLAGS_T;

typedef enum {
	IMU_FASTSETTL_MODE_XL_DISABLE	= 0,
	IMU_FASTSETTL_MODE_XL_ENABLE	= 1,
} CTRL8_G_FASTSETTL_MODE_XL_FLAGS_T;

typedef enum {
	IMU_HP_REF_MODE_XL_DISABLE		= 0,
	IMU_HP_REF_MODE_XL_ENABLE		= 1,
} CTRL8_G_HP_REF_MODE_XL_FLAGS_T;

typedef enum {
	IMU_HPCF_XL_000					= 0b000,
	IMU_HPCF_XL_001					= 0b001,
	IMU_HPCF_XL_010					= 0b010,
	IMU_HPCF_XL_011					= 0b011,
	IMU_HPCF_XL_100					= 0b100,
	IMU_HPCF_XL_101					= 0b101,
	IMU_HPCF_XL_110					= 0b110,
	IMU_HPCF_XL_111					= 0b111,
} CTRL8_G_HPCF_XL_FLAGS_T;

typedef enum {
	IMU_ENABLE_COMMS_CTRL9			= 0,
	IMU_DISABLE_I3C_CTRL9			= 1,
} CTRL9_I3C_DISABLE_FLAGS_T;

typedef enum {
	IMU_DEN_LH_ACTIVE_LOW			= 0,
	IMU_DEN_LH_ACTIVE_HIGH			= 1,
} CTRL9_DEN_LH_FLAGS_T;

typedef enum {
	IMU_DEN_WITHOUT_ACCEL			= 0,
	IMU_DEN_WITH_ACCEL				= 1,
} CTRL9_DEN_XL_EN_FLAGS_T;

typedef enum {
	IMU_DEN_XL_G_GYRO_AXIS			= 0,
	IMU_DEN_XL_G_ACCEL_AXIS			= 1,
} CTRL9_DEN_XL_G_FLAGS_T;

typedef enum {
	IMU_DEN_Z_AXIS_NOT_IN_LSB		= 0,
	IMU_DEN_Z_AXIS_IN_LSB			= 1, // Default
} CTRL9_DEN_Z_FLAGS_T;

typedef enum {
	IMU_DEN_Y_AXIS_NOT_IN_LSB		= 0,
	IMU_DEN_Y_AXIS_IN_LSB			= 1, // Default
} CTRL9_DEN_Y_FLAGS_T;

typedef enum {
	IMU_DEN_X_AXIS_NOT_IN_LSB		= 0,
	IMU_DEN_X_AXIS_IN_LSB			= 1, // Default
} CTRL9_DEN_X_FLAGS_T;

typedef enum {
	IMU_TIMESTAMP_DISABLE			= 0,
	IMU_TIMESTAMP_ENABLE			= 1,
} CTRL10_TIMESTAMP_EN_FLAGS_T;

typedef enum
{
    IMU_COMMS_OK       			 = 0x00,
    IMU_COMMS_ERROR    			 = 0x01,
    IMU_COMMS_BUSY    			 = 0x02,
    IMU_COMMS_TIMEOUT 			 = 0x03,
    IMU_INVALID_REG		 		 = 0x04,
} imu_status_t;

typedef struct __attribute__((packed)) {
    pin_ctrl_t pin_ctrl;
    ctrl1_xl_t ctrl1_xl;
    ctrl2_g_t ctrl2_g;
    ctrl3_c_t ctrl3_c;
    ctrl4_c_t ctrl4_c;
    ctrl5_c_t ctrl5_c;
    ctrl6_c_t ctrl6_c;
    ctrl7_g_t ctrl7_g;
    ctrl8_xl_t ctrl8_xl;
    ctrl9_xl_t ctrl9_xl;
    ctrl10_c_t ctrl10_c;
	float32_t accelSens; 			// m/s^2
	float32_t angularRateSens; 	// milidegree / s
	bool modifiedRegisters[IMU_CTRL_REG_NUM];
} imu_handler_t;

uint8_t generateIMUAddress(imu_reg_t imuRegNum,
						   bool readFlag);

imu_status_t initializeIMU(spi_device_t* imuSPI,
						   imu_handler_t* imuHandler);

imu_status_t writeIMUSingleRegister(spi_device_t* imuSPI,
						 	 	 	imu_reg_t imuRegNum,
									uint8_t valueToWrite);

imu_status_t readIMUSingleRegister(spi_device_t* imuSPI,
							  	   imu_reg_t imuRegNum,
								   uint8_t* receivedData);

imu_status_t readIMUDoubleRegister(spi_device_t* imuSPI,
							   	   imu_reg_t upperRegAddress,
								   imu_reg_t lowerRegAddress,
								   uint16_t* receivedData);

imu_status_t readIMUMultipleRegisters(spi_device_t* imuSPI,
									  imu_reg_t startRegNum,
									  imu_reg_t endRegNum,
									  uint8_t* regReadValues);

imu_status_t writeIMUMultipleRegisters(spi_device_t* imuSPI,
									  imu_reg_t startRegNum,
									  imu_reg_t endRegNum,
									  uint8_t* valuesToWrite);

imu_status_t getPitchRate(spi_device_t* imuSPI,
			   	   	  	  imu_handler_t* imuHandler,
						  float32_t* pitchOutput);

imu_status_t getRollRate(spi_device_t* imuSPI,
			  	  	 	 imu_handler_t* imuHandler,
						 float32_t* rollOutput);

imu_status_t getYawRate(spi_device_t* imuSPI,
			 	 		imu_handler_t* imuHandler,
						float32_t* yawOutput);

imu_status_t getXAccel(spi_device_t* imuSPI,
					   imu_handler_t* imuHandler,
					   float32_t* xAccelOutput);

imu_status_t getYAccel(spi_device_t* imuSPI,
					   imu_handler_t* imuHandler,
					   float32_t* yAccelOutput);

imu_status_t getZAccel(spi_device_t* imuSPI,
					   imu_handler_t* imuHandler,
					   float32_t* zAccelOutput);

void setIMUFlags(imu_handler_t* imuHandler);

imu_status_t getIMUData(spi_device_t* imuSPI,
						imu_handler_t* imuHandler,
						float32_t angularRate[3],
						float32_t linAccel[3]);


#endif
