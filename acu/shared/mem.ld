/* shared/memory.ld: the one place flash and RAM layout is defined */

MEMORY
{
    /* ---- Flash ---- */
    BL_FLASH (rx) :                ORIGIN = 0x08000000, LENGTH = 128K          /* bank 1 sector 0 */
    M4_IMAGE_HEADER_FLASH (rw) :   ORIGIN = 0x08020000, LENGTH = 1K            /* bank 1 sector 1 */
    M4_IMAGE_FLASH (rx) :          ORIGIN = 0x08020400, LENGTH = 256K - 1K     /* bank 1 sectors 1-2 */
    M7_IMAGE_HEADER_FLASH (rw) :   ORIGIN = 0x08060000, LENGTH = 1K            /* bank 1 sector 3 */
    M7_IMAGE_FLASH (rx) :          ORIGIN = 0x08060400, LENGTH = 640K - 1K     /* bank 1 sectors 3-7 */
    HW_CONFIG_FLASH (rw) :         ORIGIN = 0x08100000, LENGTH = 128K          /* bank 2 sector 0 */
    APPLICATION_DEFINED (rw) :     ORIGIN = 0x08120000, LENGTH = 896K          /* bank 2 sectors 1-7 */

    /* ---- RAM, M4 view (alias addresses, RM0399 Table 6) ---- */
    M4_RAM (xrw) :                  ORIGIN = 0x10000000, LENGTH = 48K           /* SRAM1: stack, .data, .bss */
    LWIP_HEAP (rw) :                ORIGIN = 0x10020000, LENGTH = 32K           /* SRAM2: placed by LWIP_RAM_HEAP_POINTER */
    ETH_RX_DESC (rw) :              ORIGIN = 0x10040000, LENGTH = 96            /* SRAM3: 4 Rx descriptors x 24 B */
    ETH_TX_DESC (rw) :              ORIGIN = 0x10040060, LENGTH = 96            /* SRAM3: 4 Tx descriptors x 24 B */
    ETH_RX_POOL (rw) :              ORIGIN = 0x100400C0, LENGTH = 0x7F20        /* SRAM3: Rx pool and rest of SRAM3 */

    /* AXI RAM */
    M7_RAM (rw) :                   ORIGIN = 0x24000000, LENGTH = 48K
}

SECTIONS {
    .m4_img_hdr (NOLOAD) : { *(.m4_img_hdr) } >M4_IMAGE_HEADER_FLASH
    .m7_img_hdr (NOLOAD) : { *(.m7_img_hdr) } >M7_IMAGE_HEADER_FLASH
    .hw_config (NOLOAD) : { *(.hw_config) } >HW_CONFIG_FLASH

    /* Ethernet buffers in SRAM3: no initial values, nothing stored in flash */
    .RxDescripSection (NOLOAD) : { *(.RxDescripSection) } >ETH_RX_DESC
    .TxDescripSection (NOLOAD) : { *(.TxDescripSection) } >ETH_TX_DESC
    .Rx_PoolSection  (NOLOAD) : { *(.Rx_PoolSection)  } >ETH_RX_POOL
}

/* Symbols for C code: addresses only, no storage */
__m4_img_start      = ORIGIN(M4_IMAGE_FLASH);
__m4_img_end        = __m4_img_start + LENGTH(M4_IMAGE_FLASH);
__m7_img_start      = ORIGIN(M7_IMAGE_FLASH);
__m7_img_end        = __m7_img_start + LENGTH(M7_IMAGE_FLASH);

__m4_ram_start      = ORIGIN(M4_RAM);
__m4_ram_end        = __m4_ram_start + LENGTH(M4_RAM);
__m7_ram_start      = ORIGIN(M7_RAM);
__m7_ram_end        = __m7_ram_start + LENGTH(M7_RAM);

/* Build-time checks: a bad layout fails the linking step */
ASSERT(ORIGIN(BL_FLASH) + LENGTH(BL_FLASH) <= ORIGIN(M4_IMAGE_HEADER_FLASH), "Bootloader code overlaps M4 header")
ASSERT(ORIGIN(M4_IMAGE_HEADER_FLASH) + LENGTH(M4_IMAGE_HEADER_FLASH) <= ORIGIN(M4_IMAGE_FLASH), "M4 header overlaps M4 code")
ASSERT(ORIGIN(M4_IMAGE_FLASH) % 0x400 == 0, "M4 image start not aligned for VTOR")
ASSERT(ORIGIN(M7_IMAGE_HEADER_FLASH) + LENGTH(M7_IMAGE_HEADER_FLASH) <= ORIGIN(M7_IMAGE_FLASH), "M7 header overlaps M7 code")
ASSERT(ORIGIN(M7_IMAGE_FLASH) % 0x400 == 0, "M7 image start not aligned for VTOR")
ASSERT(ORIGIN(HW_CONFIG_FLASH) + LENGTH(HW_CONFIG_FLASH) <= ORIGIN(APPLICATION_DEFINED), "HW config overlaps application defined flash")

ASSERT(ORIGIN(ETH_RX_POOL) % 32 == 0, "Eth Rx pool must be 32-byte aligned")
ASSERT(ORIGIN(ETH_RX_DESC) + LENGTH(ETH_RX_DESC) <= ORIGIN(ETH_TX_DESC), "Eth Rx Descriptors overlapping Eth Tx Descriptors")
ASSERT(ORIGIN(ETH_TX_DESC) + LENGTH(ETH_TX_DESC) <= ORIGIN(ETH_RX_POOL), "Eth Tx Descriptors overlapping Eth Rx Buffer Pool")