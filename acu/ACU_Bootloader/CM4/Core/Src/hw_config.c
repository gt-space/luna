#include "hw_config.h"

volatile const hw_config_t hw_config __attribute__((section(".hw_config")));

const hw_config_t hw_config_default = {
	.meta_version = 0,
	.ipv4 = {192, 168, 1, 100},
	.u_board_id = 0,
	.magic = HW_CONFIG_MAGIC_VAL
};

HAL_StatusTypeDef get_hw_config(uint8_t *buf) {
	return HAL_I2C_Mem_Read(
			&hi2c1,
			EEPROM_ADDR,
			0x0,
			I2C_MEMADD_SIZE_16BIT,
			buf,
			sizeof(hw_config_t),
			10
		);
}

static uint32_t write_hw_config(volatile const hw_config_t *config) {
	FLASH_EraseInitTypeDef erase_init_struct;
	uint32_t sector_error;
	uint8_t flash_buf[32];
	memset(flash_buf, 0, 32);
	memcpy(flash_buf, (const void *) config, sizeof(hw_config_t));

	HAL_FLASH_Unlock();
	__HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_ALL_ERRORS_BANK2);

	erase_init_struct.TypeErase = FLASH_TYPEERASE_SECTORS;
	erase_init_struct.VoltageRange = FLASH_VOLTAGE_RANGE_3;
	erase_init_struct.Sector = FLASH_SECTOR_0;
	erase_init_struct.Banks = FLASH_BANK_2;
	erase_init_struct.NbSectors = 1;

	if (HAL_FLASHEx_Erase(&erase_init_struct, &sector_error) != HAL_OK) return HAL_FLASH_GetError();
	if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_FLASHWORD, (uint32_t) &hw_config, (uint32_t) flash_buf) != HAL_OK) return HAL_FLASH_GetError();

	HAL_FLASH_Lock();
	return HAL_FLASH_ERROR_NONE;
}

HwConfigValidationResult validate_hw_config(uint8_t *buf, uint8_t eeprom_present) {
	uint32_t flash_err_code;

	if(!eeprom_present) {
		// Program default values into flash.
		flash_err_code = write_hw_config((volatile const hw_config_t *) &hw_config_default);
		if (flash_err_code != HAL_FLASH_ERROR_NONE) {
			return (HwConfigValidationResult){HW_CONFIG_FLASH_ERROR, HW_CONFIG_MAGIC_VAL, flash_err_code};
		} else {
			return (HwConfigValidationResult){DEFAULT_HW_CONFIG_OK, HW_CONFIG_MAGIC_VAL, HAL_FLASH_ERROR_NONE};
		}
	}

	hw_config_t new_hw_config = {
	    .meta_version = (uint16_t)((buf[1] << 8) | buf[0]),
	    .ipv4         = {buf[5], buf[4], buf[3], buf[2]},
	    .u_board_id   = (uint16_t)((buf[7] << 8) | buf[6]),
	    .magic        = ((uint32_t)buf[11] << 24) | ((uint32_t)buf[10] << 16) | ((uint32_t)buf[9] << 8) | buf[8]
	};

	// Check if the hw_config stored in Flash is equal to the read one over I2C
	if (memcmp((const void *) &hw_config, &new_hw_config, sizeof(hw_config_t)) == 0) {
	    // Sanity check that hw_config.magic is valid. If not valid, program the default values into flash. Otherwise do nothing
		if (hw_config.magic != HW_CONFIG_MAGIC_VAL) {
			flash_err_code = write_hw_config((volatile const hw_config_t *) &hw_config_default);
			if (flash_err_code != HAL_FLASH_ERROR_NONE) {
				return (HwConfigValidationResult){INVALID_OLD_HW_CONFIG_MAGIC_AND_FLASH_ERROR, hw_config.magic, flash_err_code};
			} else {
				return (HwConfigValidationResult){INVALID_OLD_HW_CONFIG_MAGIC, hw_config.magic, HAL_FLASH_ERROR_NONE};
			}
		} else {
			return (HwConfigValidationResult){OLD_HW_CONFIG_OK, hw_config.magic, HAL_FLASH_ERROR_NONE};
		}
	} else {
		// Check that new_hw_config.magic is valid. If not valid, program the default values into flash. Otherwise program new_hw_config into flash
		if (new_hw_config.magic != HW_CONFIG_MAGIC_VAL) {
			// hw_config.magic is not valid. Program default values into flash
			flash_err_code = write_hw_config((volatile const hw_config_t *) &hw_config_default);
			if (flash_err_code != HAL_FLASH_ERROR_NONE) {
				return (HwConfigValidationResult){INVALID_NEW_HW_CONFIG_MAGIC_AND_FLASH_ERROR, new_hw_config.magic, flash_err_code};
			} else {
				return (HwConfigValidationResult){INVALID_NEW_HW_CONFIG_MAGIC, new_hw_config.magic, HAL_FLASH_ERROR_NONE};
			}
		} else {
			// Program new_hw_config into flash
			flash_err_code = write_hw_config((volatile const hw_config_t *) &new_hw_config);
			if (flash_err_code != HAL_FLASH_ERROR_NONE) {
				return (HwConfigValidationResult){HW_CONFIG_FLASH_ERROR, new_hw_config.magic, flash_err_code};
			} else {
				return (HwConfigValidationResult){NEW_HW_CONFIG_OK, new_hw_config.magic, HAL_FLASH_ERROR_NONE};
			}
		}
	}
}

