#include "bl_jump.h"


typedef void (*pFunction)(void);

/**
 * Ensure this value does not live on the bootloader stack.
 * Once the stack pointer is switched to run the networking app, these
 * variable will still be accessible. This is a function pointer
 * to the reset handler of the network app. The network app msp is also
 * stored in static space purely for convenience.
 */
static pFunction network_app_rst_handler;
static uint32_t network_app_msp;

volatile const image_header_t network_app_image_header __attribute__((section(".m4_img_hdr")));
volatile const image_header_t user_app_image_header __attribute__((section(".m7_img_hdr")));

static void reset_mpu(void) {
	HAL_MPU_Disable();

	uint32_t regions = (MPU->TYPE & MPU_TYPE_DREGION_Msk) >> MPU_TYPE_DREGION_Pos;

	// Select each region number, clear permissions, clear base address
	for (uint32_t i = 0; i < regions; i++) {
		MPU->RNR = i;
		MPU->RASR = 0;
		MPU->RBAR = 0;
	}

	/**
	 * Section 2.24 of STM32 Cortex-M4 MCUs and MPUs programming manual
	 * States that a modification to MPU should be followed by DSB and ISB to ensure
	 * that subsequent operations conform to the intended MPU configuration.
	 *
	 * Page 46 of ARM Cortex-M Programming Guide to Memory Barrier Instructions Application Note 321
	 * states that ISB should be added if MPU changes affect program memory.
	 */
	__DSB();
	__ISB();
}

void jump_to_network_app(void) {
	__disable_irq(); // PRIMASK. Prevents any interrupt from being handled by CPU

    /* Hard reset of the whole Ethernet peripheral (RM0399 9.7.28, RCC_AHB1RSTR
     * bit 15 ETH1MACRST): every Ethernet register, DMA included, returns to its
     * power-on value. No graceful stop (RM0399 61.9.5) first: any frame in
     * flight is discarded on purpose, and the network app re-initializes
     * Ethernet from scratch. Only this bit is touched; never reset the whole
     * AHB1 bus here (bit 14 ARTRST must not be set while running from flash).
     */
    __HAL_RCC_ETH1MAC_FORCE_RESET();
    __HAL_RCC_ETH1MAC_RELEASE_RESET();
    /* Device memory: accesses to the same peripheral arrive in program order,
     * and a read must be answered by the RCC itself. So once this read returns,
     * the RELEASE write has reached the RCC. The DSB makes the core wait for
     * the read before continuing (AN321 4.9 steps 1-2).
    */
    (void)RCC->AHB1RSTR;
    __DSB();


	for (uint32_t i = 0; i < sizeof(NVIC->ICER) / sizeof(NVIC->ICER[0]); i++) {
	    NVIC->ICER[i] = 0xFFFFFFFF; /* disable every interrupt in this group of 32 */
	    NVIC->ICPR[i] = 0xFFFFFFFF; /* clear every pending one in this group of 32 */
	}

	/* AN321 4.9 step 5: the IRQ synchronizer may re-pend the Ethernet IRQs for a
	 * few cycles after the reset. Ethernet is in reset so it can't keep asserting:
	 * this loop runs at most once or twice. Currently the ETH interrupts are not
	 * enabled so this only serves a purpose if they do get enabled. */
	while (NVIC_GetPendingIRQ(ETH_IRQn) || NVIC_GetPendingIRQ(ETH_WKUP_IRQn)) {
	    NVIC_ClearPendingIRQ(ETH_IRQn);
	    NVIC_ClearPendingIRQ(ETH_WKUP_IRQn);
	}

	// Stop SysTick and clear a pending SysTick interrupt. SysTick is a system exception and not controlled by NVIC
	SysTick->CTRL = 0;
	SysTick->LOAD = 0;
	SysTick->VAL  = 0;
	SCB->ICSR = SCB_ICSR_PENDSTCLR_Msk;

	reset_mpu();

	__set_MSP(network_app_msp);

	// Hand control to network app.
	network_app_rst_handler();
	__builtin_unreachable();
}

uint32_t crc32(const uint8_t *data, uint32_t num_bytes) {
    uint32_t crc = 0xFFFFFFFFU;

    for (uint32_t i = 0; i < num_bytes; i++) {
        crc ^= data[i];
        for (uint8_t bit = 0; bit < 8; bit++) {
        	crc = (crc >> 1) ^ (0xEDB88320U & -(crc & 1U));
        }
    }

    return ~crc;
}

/**
 * Must be called before jump_to_network_app
 */
AppValidationResult validate_app(uint8_t core) {
	if (core != 4 && core != 7) return (AppValidationResult){INVALID_CORE_ARG, (uint32_t) core};

	image_header_t image_header;
	volatile uint32_t *vector_table;
	uint32_t msp;
	uint32_t reset_handler_addr;
	uint32_t img_start, img_end;
	uint32_t ram_start, ram_end;

	if (core == 4) {
		image_header = network_app_image_header;
		vector_table = (volatile uint32_t *) __m4_img_start;
		/*
		 * Needs to be stored outside of stack. M7 does not because the bootloader
		 * does not jump to the user application. MSP stored in static space for convenience.
		 */
		network_app_msp = vector_table[0];
		network_app_rst_handler = (pFunction) vector_table[1];
		ram_start = (uint32_t) __m4_ram_start;
		ram_end = (uint32_t) __m4_ram_end;
		img_start = (uint32_t)__m4_img_start;
		img_end = (uint32_t) __m4_img_end;

	} else {
		image_header = user_app_image_header;
		vector_table = (volatile uint32_t *) __m7_img_start;
		ram_start = (uint32_t) __m7_ram_start;
		ram_end = (uint32_t) __m7_ram_end;
		img_start = (uint32_t) __m7_img_start;
		img_end = (uint32_t)__m7_img_end;
	}

	msp = (uint32_t) vector_table[0];
	reset_handler_addr = (uint32_t) vector_table[1];

	if (
		((image_header.magic >> 24) & 0xFF) != 'y' ||
		((image_header.magic >> 16) & 0xFF) != 'j' ||
		((image_header.magic >> 8) & 0xFF) != 's' ||
		(image_header.magic & 0xFF) != core + '0'
	) return (AppValidationResult){INVALID_APP_MAGIC, image_header.magic};

	if (image_header.image_size_bytes == 0 || image_header.image_size_bytes > img_end - img_start) {
		return (AppValidationResult){IMG_SIZE_OUT_OF_RANGE, image_header.image_size_bytes};
	}

	uint32_t calculated_crc = crc32((const uint8_t *) img_start, image_header.image_size_bytes);
	if (image_header.image_crc != calculated_crc) {
		return (AppValidationResult){CRC_MISMATCH, calculated_crc};
	}

	if (msp <= ram_start || msp > ram_end) {
		return (AppValidationResult){MSP_OUT_OF_RANGE, msp};
	}

	// Procedure Call Standard for the Arm Architecture Page 21: Stack must be double word aligned.
	// It defines an externally visible interface in section 2.3 as an interface between separately compiled or separately assembled routines.
	if ((msp % 8) != 0) {
		return (AppValidationResult){MSP_NOT_ALIGNED, msp};
	}

	// Least significant bit of a function indicates instruction type. Cortex-M4/M7 only supports Thumb instructions
	if ((reset_handler_addr & 0x1) == 0) {
		return (AppValidationResult){RESET_HANDLER_NOT_THUMB, reset_handler_addr};
	}

	if (reset_handler_addr < img_start || reset_handler_addr >= img_start + image_header.image_size_bytes) {
		return (AppValidationResult){RESET_HANDLER_OUT_OF_RANGE, reset_handler_addr};
	}

	return (AppValidationResult){APP_VALIDATION_OK, 0};
}

void start_user_application(void) {

}

