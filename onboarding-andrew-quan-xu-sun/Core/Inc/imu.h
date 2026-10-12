#ifndef IMU_H
#define IMU_H

#include <stdbool.h>
#include <stdint.h>
#include "main.h"

#define IMU_REG_WHO_AM_I 0x0F
#define IMU_REG_WHO_AM_I_EXPECTED 0x6B

#define IMU_REG_CTRL1_XL 0x10 // Accelerometer Settings
#define IMU_REG_CTRL2_G 0x11 // Gyroscope Settings
#define IMU_REG_CTRL3_C 0x12 // Some general settings, but also includes Block Data Update
#define IMU_REG_CTRL3_C_BDU (1 << 6) // BDU bit

#define IMU_IDX_OUT_TEMP_L 0
#define IMU_IDX_OUT_TEMP_H 1
#define IMU_IDX_OUTX_L_G 2
#define IMU_IDX_OUTX_H_G 3
#define IMU_IDX_OUTY_L_G 4
#define IMU_IDX_OUTY_H_G 5
#define IMU_IDX_OUTZ_L_G 6
#define IMU_IDX_OUTZ_H_G 7
#define IMU_IDX_OUTX_L_A 8
#define IMU_IDX_OUTX_H_A 9
#define IMU_IDX_OUTY_L_A 10
#define IMU_IDX_OUTY_H_A 11
#define IMU_IDX_OUTZ_L_A 12
#define IMU_IDX_OUTZ_H_A 13

// for burst read
#define ALL_READ_START 0x20 // OUT_TEMP_L register
#define ALL_READ_LENGTH 14

#define TEMPERATURE_OFFSET 25.0f
#define TEMPERATURE_SENSITIVITY (1.0f / 256.0f) // 256 LSB/degrees Celsius
#define SENSITIVITY_CONVERSION_FAIL (-1.0)

// conversions
#define MG_TO_MS2 0.00980665f
#define MDPS_TO_RADS 0.0000174533f

/*
 * Linear acceleration ODR
 * Note that the IMU defaults to high-performance (XL_HM_MODE = 0)
 * Low-Power = LP
 * Normal Mode = NM
 * High Performance = HP
 * Comments are in the format [XL_HM_MODE = 1 | XL_HM_MODE = 0] aka [LP, HP]
 */
typedef enum
{
	LA_ODR_OFF = 0x00, // 			 Power-down:              0000 00 00
	LA_ODR_1 = 0xB0,   // [1.6 Hz (LP only) | 12.5 Hz (HP) ]: 1011 00 00
	LA_ODR_2 = 0x10,   // [12.5 Hz (LP)     | 12.5 Hz (HP) ]: 0001 00 00
	LA_ODR_3 = 0x20,   // [26 Hz (LP)       | 26 Hz (HP)   ]: 0010 00 00
	LA_ODR_4 = 0x30,   // [52 Hz (LP)       | 52 Hz (HP)   ]: 0011 00 00
	LA_ODR_5 = 0x40,   // [104 Hz (NM)      | 104 Hz (HP)  ]: 0100 00 00
	LA_ODR_6 = 0x50,   // [208 Hz (NM)      | 208 Hz (HP)  ]: 0101 00 00
	LA_ODR_7 = 0x60,   // [416 Hz (HP)      | 416 Hz (HP)  ]: 0110 00 00
	LA_ODR_8 = 0x70,   // [833 Hz (HP)      | 833 Hz (HP)  ]: 0111 00 00
	LA_ODR_9 = 0x80,   // [1.66 kHz (HP)    | 1.66 kHz (HP)]: 1000 00 00
	LA_ODR_10 = 0x90,  // [3.33 kHz (HP)    | 3.33 kHz (HP)]: 1001 00 00
	LA_ODR_11 = 0xA0,  // [6.66 kHz (HP)    | 6.66 kHz (HP)]: 1010 00 00
} IMU_LA_ODR_Setting;

// Linear acceleration full-scale range (default: 2G)
typedef enum
{
	LA_FS_2G = 0x00, // 0000 00 00
	LA_FS_4G = 0x08, // 0000 10 00
	LA_FS_8G = 0x0C, // 0000 11 00
	LA_FS_16G = 0x04 // 0000 01 00
} IMU_LA_FS_Setting;

static inline float LA_FS_to_Sensitivity(IMU_LA_FS_Setting setting) {
	switch (setting) {
	case LA_FS_2G:
		return 0.061f * MG_TO_MS2;
	case LA_FS_4G:
		return 0.122f * MG_TO_MS2;
	case LA_FS_8G:
		return 0.244f * MG_TO_MS2;
	case LA_FS_16G:
		return 0.488f * MG_TO_MS2;
	default:
		return SENSITIVITY_CONVERSION_FAIL;
	}
}

// whether to output from LPF2 second stage or not (default: disabled -> output from first-stage digital filter)
typedef enum
{
	LA_LPF2_ON = 0x02, // 0000 00 10
	LA_LPF2_OFF = 0x00, // 0000 00 00
} IMU_LA_LPF2_Setting;

/*
 * Angular velocity ODR
 * Note that the IMU defaults to high-performance (G_HM_MODE = 0)
 * Low-Power = LP
 * Normal Mode = NM
 * High Performance = HP
 * Comments are in the format [G_HM_MODE = 1 | G_HM_MODE = 0] aka [LP, HP]
 */
typedef enum
{
	G_ODR_OFF = 0x00, // 			 Power-down:              0000 00 00
	G_ODR_1 = 0xB0,   // [1.6 Hz (LP only) | 12.5 Hz (HP) ]: 1011 00 00
	G_ODR_2 = 0x10,   // [12.5 Hz (LP)     | 12.5 Hz (HP) ]: 0001 00 00
	G_ODR_3 = 0x20,   // [26 Hz (LP)       | 26 Hz (HP)   ]: 0010 00 00
	G_ODR_4 = 0x30,   // [52 Hz (LP)       | 52 Hz (HP)   ]: 0011 00 00
	G_ODR_5 = 0x40,   // [104 Hz (NM)      | 104 Hz (HP)  ]: 0100 00 00
	G_ODR_6 = 0x50,   // [208 Hz (NM)      | 208 Hz (HP)  ]: 0101 00 00
	G_ODR_7 = 0x60,   // [416 Hz (HP)      | 416 Hz (HP)  ]: 0110 00 00
	G_ODR_8 = 0x70,   // [833 Hz (HP)      | 833 Hz (HP)  ]: 0111 00 00
	G_ODR_9 = 0x80,   // [1.66 kHz (HP)    | 1.66 kHz (HP)]: 1000 00 00
	G_ODR_10 = 0x90,  // [3.33 kHz (HP)    | 3.33 kHz (HP)]: 1001 00 00
	G_ODR_11 = 0xA0,  // [6.66 kHz (HP)    | 6.66 kHz (HP)]: 1010 00 00
} IMU_G_ODR_Setting;

// Angular velocity full-scale range (default: 250 dps)
typedef enum
{
	G_FS_250 = 0x00, // 0000 00 00
	G_FS_500 = 0x04, // 0000 01 00
	G_FS_1000 = 0x08, // 0000 10 00
	G_FS_2000 = 0x0C, // 0000 11 00
} IMU_G_FS_Setting;

static inline float G_FS_to_Sensitivity(IMU_G_FS_Setting setting) {
	switch (setting) {
	case G_FS_250:
		return 8.75f * MDPS_TO_RADS;
	case G_FS_500:
		return 17.50f * MDPS_TO_RADS;
	case G_FS_1000:
		return 35.0f * MDPS_TO_RADS;
	case G_FS_2000:
		return 70.0f * MDPS_TO_RADS;
	default:
		return SENSITIVITY_CONVERSION_FAIL;
	}
}

typedef struct
{
	bool ERROR;
	float TEMP; // in Celsius
	float LA_X; // in m/s^2
	float LA_Y; // in m/s^2
	float LA_Z; // in m/s^2
	float G_X; // in rad/s
	float G_Y; // in rad/s
	float G_Z; // in rad/s
} IMU_READING;

HAL_StatusTypeDef imu_read_reg(uint8_t reg, uint8_t* val);
HAL_StatusTypeDef imu_write_reg(uint8_t reg, uint8_t val);

HAL_StatusTypeDef imu_burst_read_regs(uint8_t startReg, uint8_t* buf, uint16_t len);

HAL_StatusTypeDef imu_configure_accelerometer(IMU_LA_ODR_Setting odr, IMU_LA_FS_Setting fs, IMU_LA_LPF2_Setting lpf2);
HAL_StatusTypeDef imu_configure_gyroscope(IMU_G_ODR_Setting odr, IMU_G_FS_Setting fs);

HAL_StatusTypeDef imu_enable_bdu(void);

IMU_READING imu_read_all(IMU_LA_FS_Setting fsLA, IMU_G_FS_Setting fsG);

#endif
