#ifndef _ASM330LHBG1
#define _ASM330LHBG1

#define IMU_MAX_REG 0x7E
#define IMU_MIN_REG 0x01

#include "stdbool.h"
#include "stm32h7xx_hal.h"
#include "SPI_Device.h"

typedef enum {
    IMU_FUNC_CFG_ACCESS        = 0x01,
    IMU_PIN_CTRL               = 0x02,
    IMU_SENSOR_SYNC_TIME_FRAME = 0x04,
    IMU_SENSOR_SYNC_RES_RATIO  = 0x05,
    IMU_FIFO_CTRL1             = 0x06,
    IMU_FIFO_CTRL2             = 0x07,
    IMU_FIFO_CTRL3             = 0x08,
    IMU_FIFO_CTRL4             = 0x09,
    IMU_FIFO_CTRL5             = 0x0A,
    IMU_ORIENT_CFG_G           = 0x0B,
    IMU_INT1_CTRL              = 0x0D,
    IMU_INT2_CTRL              = 0x0E,
    IMU_WHO_AM_I_IMU           = 0x0F,
    IMU_CTRL1_XL               = 0x10,
    IMU_CTRL2_G                = 0x11,
    IMU_CTRL3_C                = 0x12,
    IMU_CTRL4_C                = 0x13,
    IMU_CTRL5_C                = 0x14,
    IMU_CTRL6_C                = 0x15,
    IMU_CTRL7_G                = 0x16,
    IMU_CTRL8_XL               = 0x17,
    IMU_CTRL9_XL               = 0x18,
    IMU_CTRL10_C               = 0x19,
    IMU_ALL_INT_SRC            = 0x1A,
    IMU_WAKE_UP_SRC            = 0x1B,
    IMU_TAP_SRC                = 0x1C,
    IMU_D6D_SRC                = 0x1D,
    IMU_STATUS_REG_IMU         = 0x1E,
    IMU_OUT_TEMP_L             = 0x20,
    IMU_OUT_TEMP_H             = 0x21,
    IMU_OUTX_L_G               = 0x22,
    IMU_OUTX_H_G               = 0x23,
    IMU_OUTY_L_G               = 0x24,
    IMU_OUTY_H_G               = 0x25,
    IMU_OUTZ_L_G               = 0x26,
    IMU_OUTZ_H_G               = 0x27,
    IMU_OUTX_L_A               = 0x28,
    IMU_OUTX_H_A               = 0x29,
    IMU_OUTY_L_A               = 0x2A,
    IMU_OUTY_H_A               = 0x2B,
    IMU_OUTZ_L_A               = 0x2C,
    IMU_OUTZ_H_A               = 0x2D,
    IMU_EMB_FUNC_STATUS_MAINPAGE = 0x35,
    IMU_FSM_STATUS_A_MAINPAGE  = 0x36,
    IMU_FSM_STATUS_B_MAINPAGE  = 0x37,
    IMU_STATUS_MASTER_MAINPAGE = 0x39,
    IMU_FIFO_STATUS1           = 0x3A,
    IMU_FIFO_STATUS2           = 0x3B,
    IMU_TIMESTAMP0_REG         = 0x40,
    IMU_TIMESTAMP1_REG         = 0x41,
    IMU_TIMESTAMP2_REG         = 0x42,
    IMU_STEP_TIMESTAMP_L       = 0x49,
    IMU_STEP_TIMESTAMP_H       = 0x4A,
    IMU_STEP_COUNTER_L         = 0x4B,
    IMU_STEP_COUNTER_H         = 0x4C,
    IMU_EMB_FUNC_FIFO_STATUS   = 0x4D,
    IMU_FSM_ENABLE_A           = 0x4E,
    IMU_FSM_ENABLE_B           = 0x4F,
    IMU_EMB_FUNC_INIT_A        = 0x50,
    IMU_EMB_FUNC_INIT_B        = 0x51,
    IMU_FSM_LONG_COUNTER_L     = 0x52,
    IMU_FSM_LONG_COUNTER_H     = 0x53,
    IMU_EMB_FUNC_SRC           = 0x56,
    IMU_FSM_OUTS1              = 0x58,
    IMU_FSM_OUTS2              = 0x59,
    IMU_FSM_OUTS3              = 0x5A,
    IMU_FSM_OUTS4              = 0x5B,
    IMU_FSM_OUTS5              = 0x5C,
    IMU_FSM_OUTS6              = 0x5D,
    IMU_FSM_OUTS7              = 0x5E,
    IMU_FSM_OUTS8              = 0x5F,
    IMU_FSM_OUTS9              = 0x60,
    IMU_FSM_OUTS10             = 0x61,
    IMU_FSM_OUTS11             = 0x62,
    IMU_FSM_OUTS12             = 0x63,
    IMU_FSM_OUTS13             = 0x64,
    IMU_FSM_OUTS14             = 0x65,
    IMU_FSM_OUTS15             = 0x66,
    IMU_FSM_OUTS16             = 0x67,
    IMU_EMB_FUNC_ODR_CFG_B     = 0x7F,
    IMU_I3C_BUS_AVB            = 0x62,
    IMU_INTERNAL_FREQ_FINE     = 0x63,
    IMU_X_OFS_USR              = 0x73,
    IMU_Y_OFS_USR              = 0x74,
    IMU_Z_OFF_USR              = 0x75,
    IMU_FIFO_DATA_OUT_TAG      = 0x78,
    IMU_FIFO_DATA_OUT_X_L      = 0x79,
    IMU_FIFO_DATA_OUT_X_H      = 0x7A,
    IMU_FIFO_DATA_OUT_Y_L      = 0x7B,
    IMU_FIFO_DATA_OUT_Y_H      = 0x7C,
    IMU_FIFO_DATA_OUT_Z_L      = 0x7D,
    IMU_FIFO_DATA_OUT_Z_H      = 0x7E
} imu_reg_t;

uint8_t generateIMUAddress(imu_reg_t imuRegNum, bool readFlag);
uint8_t ensureIMUNotReserved(imu_reg_t regToCheck);
uint8_t writeIMURegister(spi_device_t* imuSPI, imu_reg_t imuRegNum, uint8_t valueToWrite);
uint8_t readIMUSingleRegister(spi_device_t* imuSPI, imu_reg_t imuRegNum);
uint16_t readIMUDoubleRegister(spi_device_t* imuSPI, imu_reg_t upperRegAddress, imu_reg_t lowerRegAddress);
double getPitch(spi_device_t* imuSPI);
double getRoll(spi_device_t* imuSPI);
double getYaw(spi_device_t* imuSPI);
double getXAccel(spi_device_t* imuSPI);
double getYAccel(spi_device_t* imuSPI);
double getZAccel(spi_device_t* imuSPI);

#endif
