#ifndef EC_HEALTH_H
#define EC_HEALTH_H

/* Board health for the v4 Status: rail voltages and currents from ADC3, and
 * the two TMP112 board temperatures on I2C3.
 *
 * ADC: a polled sweep every 10 ms through sensors.c (one conversion per
 * channel, 2 ms bound), converted by measurement.c with VDDA from VREFINT.
 * I2C3: register-level polled transfers with a deadline (CubeMX set I2C3 up
 * as SMBus without interrupts), TMP112s read every 250 ms by temperature.c.
 * At boot it scans I2C3 and records which addresses answered.
 *
 * Values are the nominal Rev 1 front ends and PLACEHOLDER limits (see
 * ec_health.c) until the Rev 2 values are confirmed. RCS fan currents are not
 * configured (RCS is skipped on this branch).
 */

#include <stdint.h>

#include "fc_ec_link.h"

/* Bench visibility: watch `ec_health` in the debugger. */
typedef struct {
    uint32_t adc_sweeps;
    uint32_t adc_failures;      /* sweeps where a conversion failed */
    uint32_t vrefint_code;
    float vdda_v;               /* from VREFINT and its factory calibration */
    uint32_t i2c_found_count;   /* addresses that answered the boot scan */
    uint8_t i2c_found[16];
    uint32_t temp_polls;
    uint32_t temp_status;       /* last FCEC_TEMP_* flags */
} ec_health_debug_t;

extern volatile ec_health_debug_t ec_health;

/* After MX_ADC3_Init/Sensors_ADC_Init and MX_I2C3_SMBUS_Init. */
void EC_Health_Init(void);

/* Call every loop; fills `m` and bumps its sample ID once per ADC sweep. */
void EC_Health_Process(uint64_t now_us, fcec_measurements_t *m);

#endif /* EC_HEALTH_H */
