#include "temperature.h"

#include <math.h>
#include <stddef.h>

#define TEMP_U3_VALIDITY_BIT (1u << 15)
#define TEMP_U4_VALIDITY_BIT (1u << 16)

static const uint8_t addresses[2] = {
    FCEC_TMP112_U3_ADDRESS, FCEC_TMP112_U4_ADDRESS
};

static bool read_temperature(const fcec_temperature_ops_t *ops, uint8_t address,
                             uint32_t timeout_us, float *temperature_c,
                             bool *bus_failed) {
    uint16_t config = 0u;
    uint16_t raw = 0u;
    fcec_i2c_read_result_t status = ops->read_register(
        ops->context, address, FCEC_TMP112_CONFIG_REGISTER, &config, timeout_us);
    if (status == FCEC_I2C_READ_NACK) return false;
    if (status != FCEC_I2C_READ_OK) {
        *bus_failed = true;
        return false;
    }
    status = ops->read_register(
        ops->context, address, FCEC_TMP112_TEMP_REGISTER, &raw, timeout_us);
    if (status == FCEC_I2C_READ_NACK) return false;
    if (status != FCEC_I2C_READ_OK) {
        *bus_failed = true;
        return false;
    }

    /* EM (configuration bit 4) selects left-justified 12- or 13-bit data. */
    unsigned bits = (config & (1u << 4)) != 0u ? 13u : 12u;
    int32_t counts = (int32_t)(raw >> (16u - bits));
    if ((counts & (1 << (bits - 1u))) != 0) counts -= (1 << bits);
    *temperature_c = (float)counts * 0.0625f;
    return true;
}

bool fcec_temperature_poll(const fcec_temperature_ops_t *ops,
                           uint32_t transaction_timeout_us,
                           float overtemperature_limit_c,
                           uint64_t now_us,
                           fcec_temperature_result_t *result) {
    bool bus_failed = false;
    bool alert_high = true;
    unsigned i;

    if (result == NULL) return false;
    result->temperature_c[0] = NAN;
    result->temperature_c[1] = NAN;
    result->sample_time_us[0] = 0u;
    result->sample_time_us[1] = 0u;
    result->status = 0u;
    result->valid_mask = 0u;
    if (ops == NULL || ops->read_register == NULL ||
        ops->read_alert_pin_high == NULL || transaction_timeout_us == 0u ||
        !isfinite(overtemperature_limit_c))
        return false;

    /* Capture shared ALERT before I2C reads can clear an interrupt-mode flag. */
    if (!ops->read_alert_pin_high(ops->context, &alert_high)) {
        result->status |= FCEC_TEMP_ALERT_READ_FAILED;
    } else if (!alert_high) {
        result->status |= FCEC_TEMP_ALERT_ACTIVE;
    }

    for (i = 0u; i < 2u; ++i) {
        bool ok = read_temperature(ops, addresses[i], transaction_timeout_us,
                                  &result->temperature_c[i], &bus_failed);
        if (!ok) {
            result->status |= (uint8_t)(i == 0u ? FCEC_TEMP_U3_DEVICE_FAILED
                                                 : FCEC_TEMP_U4_DEVICE_FAILED);
            continue;
        }
        result->valid_mask |= (uint8_t)(1u << i);
        result->sample_time_us[i] = now_us;
        if (result->temperature_c[i] >= overtemperature_limit_c)
            result->status |= (uint8_t)(i == 0u ? FCEC_TEMP_U3_OVERTEMP
                                                 : FCEC_TEMP_U4_OVERTEMP);
    }
    if (bus_failed) result->status |= FCEC_TEMP_BUS_FAILED;
    return true;
}

void fcec_temperature_publish(const fcec_temperature_result_t *result,
                              fcec_measurements_t *measurements) {
    uint64_t newest_sample_time = 0u;
    unsigned i;
    if (result == NULL || measurements == NULL) return;

    measurements->validity_mask &= ~(TEMP_U3_VALIDITY_BIT | TEMP_U4_VALIDITY_BIT);
    measurements->temperature_status = result->status;
    for (i = 0u; i < 2u; ++i) {
        uint32_t bit = i == 0u ? TEMP_U3_VALIDITY_BIT : TEMP_U4_VALIDITY_BIT;
        const unsigned measurement_index = 15u + i;
        measurements->measurement_sample_time_us[measurement_index] = result->sample_time_us[i];
        if ((result->valid_mask & (1u << i)) == 0u ||
            !isfinite(result->temperature_c[i])) {
            measurements->measurement_quality[measurement_index] = FCEC_QUALITY_TRANSPORT_FAULT;
            continue;
        }
        measurements->board_temp_c[i] = result->temperature_c[i];
        measurements->measurement_quality[measurement_index] =
            (result->status & (i == 0u ? FCEC_TEMP_U3_OVERTEMP : FCEC_TEMP_U4_OVERTEMP))
                ? FCEC_QUALITY_DEVICE_FAULT : FCEC_QUALITY_OK;
        measurements->validity_mask |= bit;
        if (result->sample_time_us[i] > newest_sample_time)
            newest_sample_time = result->sample_time_us[i];
    }
    if (newest_sample_time > measurements->sample_time_us)
        measurements->sample_time_us = newest_sample_time;
}
