#ifndef __HW_CONFIG_H
#define __HW_CONFIG_H

#include <stdint.h>
#include "stm32h7xx_hal.h"
#include "i2c.h"
#include "string.h"

#define EEPROM_ADDR 0x50
#define NUM_EEPROM_READ_ATTEMPTS 100
#define HW_CONFIG_MAGIC_VAL ((uint32_t) (('y' << 24) | ('j' << 16) | ('s' << 8) | 'p'))

/**
 * The app that writes to EEPROM must set magic to "yjsp" where magic[0] is 'p'.
 * This allows the M4 networking app to know that hw_config is valid.
 * The magic field serves as an extra protection ensuring that the hw_config
 * data persisted across switching apps. The NOLOAD attribute on this section
 * of memory should guarantee that, but this is another layer of protection.
 */
typedef struct __attribute__((packed)) {
	uint16_t meta_version;
	uint8_t ipv4[4];
	uint16_t u_board_id;
	uint32_t magic;
} hw_config_t;

typedef enum {
	OLD_HW_CONFIG_OK,
	NEW_HW_CONFIG_OK,
	DEFAULT_HW_CONFIG_OK,
	INVALID_OLD_HW_CONFIG_MAGIC,
	INVALID_NEW_HW_CONFIG_MAGIC,
	INVALID_OLD_HW_CONFIG_MAGIC_AND_FLASH_ERROR,
	INVALID_NEW_HW_CONFIG_MAGIC_AND_FLASH_ERROR,
	HW_CONFIG_FLASH_ERROR
} HwConfigValidationErrCode;

typedef struct {
	HwConfigValidationErrCode err;
	uint32_t bad_magic_val;
	uint32_t flash_err;
} HwConfigValidationResult;

extern volatile const hw_config_t hw_config;
extern const hw_config_t hw_config_default;

/**
 * Performs a single attempt of reading carrier board EEPROM and storing info into hw_config.
 */
HAL_StatusTypeDef get_hw_config(uint8_t *buf);

HwConfigValidationResult validate_hw_config(uint8_t *buf, uint8_t eeprom_present);

#endif
