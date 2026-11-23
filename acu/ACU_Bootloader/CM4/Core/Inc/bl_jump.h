#include <stdint.h>
#include "main.h"

/*
 * magic field is valid if magic = "yjs4" or magic = "yjs7".
 * The '4' indicates this image_header is for the m4 core.
 * The '7' indicates this image_header is for the m7 core.
 */
typedef struct __attribute__((packed)) {
		uint16_t meta_version;
		uint16_t ipc_major;
		uint16_t ipc_minor;
		uint32_t image_size_bytes;
		uint32_t image_crc;
		uint8_t  git_hash[20];
		uint32_t magic;
} image_header_t;

/**
 * Assigns a name to the addresses defined in mem.ld.
 */
extern const uint8_t __m4_img_start[];
extern const uint8_t __m4_img_end[];
extern const uint8_t __m4_ram_start[];
extern const uint8_t __m4_ram_end[];

extern const uint8_t __m7_img_start[];
extern const uint8_t __m7_img_end[];
extern const uint8_t __m7_ram_start[];
extern const uint8_t __m7_ram_end[];

typedef enum {
	APP_VALIDATION_OK,
	INVALID_CORE_ARG,
	INVALID_APP_MAGIC,
	IMG_SIZE_OUT_OF_RANGE,
	CRC_MISMATCH,
	MSP_OUT_OF_RANGE,
	MSP_NOT_ALIGNED,
	RESET_HANDLER_NOT_THUMB,
	RESET_HANDLER_OUT_OF_RANGE
} AppValidationErrCode;

typedef struct {
	AppValidationErrCode err;
	uint32_t bad_val;
} AppValidationResult;

AppValidationResult validate_app(uint8_t core);

void start_user_application(void);

void jump_to_network_app(void);

uint32_t crc32(const uint8_t *data, uint32_t num_bytes);
