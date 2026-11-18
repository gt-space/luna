#ifndef FCEC_TEMPERATURE_H
#define FCEC_TEMPERATURE_H

#include <stdbool.h>
#include <stdint.h>

#include "fc_ec_link.h"

#define FCEC_TMP112_U3_ADDRESS 0x40u
#define FCEC_TMP112_U4_ADDRESS 0x41u
#define FCEC_TMP112_TEMP_REGISTER 0x00u
#define FCEC_TMP112_CONFIG_REGISTER 0x01u
#define FCEC_TMP112_ALERT_PIN 9u

/* Shared on-wire status bits. Temperature validity remains in validity_mask. */
#define FCEC_TEMP_U3_DEVICE_FAILED (1u << 0)
#define FCEC_TEMP_U4_DEVICE_FAILED (1u << 1)
#define FCEC_TEMP_BUS_FAILED       (1u << 2)
#define FCEC_TEMP_ALERT_ACTIVE     (1u << 3)
#define FCEC_TEMP_U3_OVERTEMP      (1u << 4)
#define FCEC_TEMP_U4_OVERTEMP      (1u << 5)
#define FCEC_TEMP_ALERT_READ_FAILED (1u << 6)

typedef enum {
    FCEC_I2C_READ_OK = 0,
    FCEC_I2C_READ_NACK,
    FCEC_I2C_READ_BUS_ERROR,
    FCEC_I2C_READ_TIMEOUT
} fcec_i2c_read_result_t;

typedef struct {
    void *context;
    /* Must return within timeout_us, including when SCL/SDA is stuck. */
    fcec_i2c_read_result_t (*read_register)(void *context, uint8_t address,
                                            uint8_t reg, uint16_t *value,
                                            uint32_t timeout_us);
    /* Reads active-low PH9. */
    bool (*read_alert_pin_high)(void *context, bool *high);
} fcec_temperature_ops_t;

typedef struct {
    float temperature_c[2];
    uint8_t status;
    uint8_t valid_mask; /* bit 0 U3, bit 1 U4 */
    uint64_t sample_time_us[2]; /* host read time; conversion age is device-period bounded */
} fcec_temperature_result_t;

/* Reads PH9 first, then both devices independently. Per-device read failures
 * identify unavailable values; timeout/controller errors also set BUS_FAILED.
 * Invalid values are NAN. */
bool fcec_temperature_poll(const fcec_temperature_ops_t *ops,
                           uint32_t transaction_timeout_us,
                           float overtemperature_limit_c,
                           uint64_t now_us,
                           fcec_temperature_result_t *result);

/* Publishes values and validity/status bits into an EC measurement snapshot.
 * The caller still owns sample_id and the complete batch timestamp. */
void fcec_temperature_publish(const fcec_temperature_result_t *result,
                              fcec_measurements_t *measurements);

#endif
