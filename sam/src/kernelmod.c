#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/io.h>
#include <linux/utsname.h> // For accessing hostname

MODULE_LICENSE("GPL"); // needed for kernel to accept this module

// Base address and size of the control module
#define CONTROL_MODULE_BASE  0x44E10000
#define CONTROL_MODULE_END  0x44E11FFF
#define CONTROL_MODULE_SIZE (CONTROL_MODULE_END - CONTROL_MODULE_BASE + 1) // add 1 to include end

// Offsets for specific registers from start of control module
#define CONF_GPMC_AD0  0x800 // for valve 1 on ground rev4
#define CONF_GPMC_AD4  0x810 // for valve 2 on ground rev4
#define CONF_LCD_DATA2 0x8A8 // for valve 6 on flight rev4

// Byte addressable pointer for memory-mapped I/O region
static void __iomem *control_module_base;

static void modify_registers_by_hostname(const char *hostname) {
    uint32_t reg_val;

    // might have to change to beaglebone.local
    // strncmp is used because hostname length as defined in utsname.h has like 65 characters
    if (strncmp(hostname, "beaglebone", 10) == 0 || strncmp(hostname, "gsam-v4-1", 9) == 0 || strncmp(hostname, "gsam-v4-2", 9) == 0) {
        pr_info("Configuring registers for ground sam rev4\n");

        // Modify CONF_GPMC_AD0 register (valve 1)
        reg_val = ioread32(control_module_base + CONF_GPMC_AD0);
        reg_val |= (1 << 4); // Enable pull-up resistor
        reg_val |= (1 << 3); // Disable pull resistor (if enabled it should be pullup)
        reg_val |= 7;        // Set mode 7 (GPIO)
        iowrite32(reg_val, control_module_base + CONF_GPMC_AD0);
        pr_info("Updated CONF_GPMC_AD0: 0x%08X\n", reg_val);

        // Modify CONF_GPMC_AD4 register (valve 2)
        reg_val = ioread32(control_module_base + CONF_GPMC_AD4);
        reg_val |= (1 << 4); // Enable pull-up resistor
        reg_val |= (1 << 3); // Disable pull resistor (if enabled it should be pullup)
        reg_val |= 7;        // Set mode 7 (GPIO)
        iowrite32(reg_val, control_module_base + CONF_GPMC_AD4);
        pr_info("Updated CONF_GPMC_AD4: 0x%08X\n", reg_val);

    } else if (strncmp(hostname, "fsam-01", 7) == 0) {
        pr_info("Configuring registers for flight sam rev4\n");

        // Modify CONF_LCD_DATA2 register (valve 6)
        reg_val = ioread32(control_module_base + CONF_LCD_DATA2);
        reg_val |= (1 << 4); // Enable pull-up resistor
        reg_val |= (1 << 3); // Disable pull resistor (if enabled it should be pullup)
        reg_val |= 7;        // Set mode 7 (GPIO)
        iowrite32(reg_val, control_module_base + CONF_LCD_DATA2);
        pr_info("Updated CONF_LCD_DATA2: 0x%08X\n", reg_val);

    } else {
        pr_warn("No register modifications applied.\n");
    }
}

// Module initialization function
static int __init regmod_init(void) {
    pr_info("Loading regmod kernel module...\n");
    const char *hostname = init_utsname()->nodename; // might have to get hostname slightly differently
    pr_info("Detected hostname: %s\n", hostname);

    // Map the control module memory into kernel virtual memory
    control_module_base = ioremap(CONTROL_MODULE_BASE, CONTROL_MODULE_SIZE);
    if (!control_module_base) {
        pr_err("Failed to map control module memory\n");
        return -ENOMEM;
    }

    // Modify registers based on the hostname
    modify_registers_by_hostname(hostname);

    return 0;
}

// Module cleanup function
static void __exit regmod_exit(void) {
    if (control_module_base) {
        iounmap(control_module_base);
    }
    pr_info("Unloading regmod kernel module\n");
}

// Register init and exit functions
module_init(regmod_init);
module_exit(regmod_exit);