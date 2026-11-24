#include "ec_health.h"

/* Parked: only built with EC_BOARD_HEALTH (CMake option, default OFF). */
#ifdef EC_BOARD_HEALTH

#include "main.h"
#include "measurement.h"
#include "sensors.h"
#include "temperature.h"

extern IWDG_HandleTypeDef hiwdg1;

volatile ec_health_debug_t ec_health;

#define ADC_PERIOD_US 10000ULL
#define TEMP_PERIOD_US 250000ULL
#define I2C_TIMEOUT_US 2000U
#define OVERTEMP_LIMIT_C 80.0f /* PLACEHOLDER until the board owner sets it */

/* ---- ADC ---- */

/* Board order used by measurement.c: vin, iin, v12, i12, v4v4, i4v4, v3v3,
 * i3v3, im1..im4, vmotor (Rev 2 .ioc / MEASUREMENTS.md). */
static const uint32_t adc_channel[FCEC_ADC3_EXTERNAL_COUNT] = {
    ADC_CHANNEL_5,  ADC_CHANNEL_9,  ADC_CHANNEL_4,  ADC_CHANNEL_8,  ADC_CHANNEL_3,
    ADC_CHANNEL_7,  ADC_CHANNEL_2,  ADC_CHANNEL_6,  ADC_CHANNEL_13, ADC_CHANNEL_14,
    ADC_CHANNEL_15, ADC_CHANNEL_16, ADC_CHANNEL_0,
};

/* Nominal Rev 1 front ends (scale 1, offset 0 on top of measurement.c's
 * dividers/gains). Limits are PLACEHOLDERS, generous on purpose: confirm the
 * Rev 2 dividers/shunts and set real windows before trusting range flags.
 * Fields: scale, offset, min, max, floor_pin_v, saturation_code,
 * stale_after_us, configured. */
#define VOLT(lo, hi) {1.0f, 0.0f, (lo), (hi), -1.0f, 65000U, 30000U, true}
#define AMPS(lo, hi) {1.0f, 0.0f, (lo), (hi), 0.002f, 65000U, 30000U, true}
#define UNUSED_CH {1.0f, 0.0f, 0.0f, 0.0f, -1.0f, 65000U, 30000U, false}
static const fcec_measurement_cal_t cal[FCEC_ADC3_EXTERNAL_COUNT] = {
    VOLT(0.0f, 30.0f),  /* vin: 6S pack input */
    AMPS(0.0f, 6.6f),   /* iin */
    VOLT(0.0f, 15.0f),  /* v12 */
    AMPS(0.0f, 6.6f),   /* i12 */
    VOLT(0.0f, 5.5f),   /* v4v4 */
    AMPS(0.0f, 3.3f),   /* i4v4 */
    VOLT(0.0f, 4.0f),   /* v3v3 (ratiometric check) */
    AMPS(0.0f, 1.32f),  /* i3v3 */
    UNUSED_CH, UNUSED_CH, UNUSED_CH, UNUSED_CH, /* im1..im4: RCS skipped */
    VOLT(0.0f, 5.5f),   /* vmotor */
};

static uint64_t next_adc_us;
static uint64_t next_temp_us;

static void adc_sweep(uint64_t now_us, fcec_measurements_t *m) {
    uint16_t codes[FCEC_ADC3_EXTERNAL_COUNT] = {0};
    uint64_t times[FCEC_ADC3_EXTERNAL_COUNT] = {0};
    uint16_t vrefint = 0U;
    bool ok = true;

    for (unsigned i = 0U; i < FCEC_ADC3_EXTERNAL_COUNT && ok; ++i) {
        if (!cal[i].configured) {
            continue;
        }
        ok = ADC_ReadChannel(adc_channel[i], &codes[i]) == HAL_OK;
        times[i] = now_us;
    }
    if (ok) {
        ok = ADC_ReadChannel(ADC_CHANNEL_VREFINT, &vrefint) == HAL_OK;
    }

    float vdda = 0.0f;
    bool vdda_ok = ok && fcec_measurement_derive_vdda(vrefint, *VREFINT_CAL_ADDR, 65535U, 65535U,
                                                     (float)VREFINT_CAL_VREF / 1000.0f, &vdda);
    ec_health.adc_sweeps++;
    if (!ok) {
        ec_health.adc_failures++;
    }
    ec_health.vrefint_code = vrefint;
    ec_health.vdda_v = vdda;

    fcec_measurement_sweep_t sweep = {
        .adc_full_scale = 65535U,
        .calibrated_vdda_v = vdda,
        .adc_ok = ok,
        .vdda_valid = vdda_ok,
        .raw_codes = codes,
        .sample_times_us = times,
        .now_us = now_us,
    };
    fcec_measurement_value_t values[FCEC_ADC3_EXTERNAL_COUNT];
    fcec_measurement_convert(&sweep, cal, values);
    fcec_measurement_publish_adc(values, m);
}

/* ---- I2C3, register level, polled with a deadline ---- */

static bool i2c_wait(uint32_t flag, uint32_t deadline_ms, bool *nack) {
    for (;;) {
        uint32_t isr = I2C3->ISR;
        if ((isr & I2C_ISR_NACKF) != 0U) {
            *nack = true;
            return false;
        }
        if ((isr & flag) != 0U) {
            return true;
        }
        if ((int32_t)(HAL_GetTick() - deadline_ms) >= 0) {
            return false;
        }
    }
}

static uint32_t deadline_for(uint32_t timeout_us) {
    return HAL_GetTick() + (timeout_us + 999U) / 1000U + 1U;
}

/* Ends a failed transfer with its own short deadline and clears the flags.
 * If the bus stays busy (no STOP), resets the peripheral. */
static void i2c_abort(void) {
    uint32_t deadline = deadline_for(1000U);
    I2C3->ICR = I2C_ICR_NACKCF;
    if ((I2C3->ISR & I2C_ISR_STOPF) == 0U && (I2C3->ISR & I2C_ISR_BUSY) != 0U) {
        I2C3->CR2 |= I2C_CR2_STOP;
        while ((I2C3->ISR & I2C_ISR_STOPF) == 0U) {
            if ((int32_t)(HAL_GetTick() - deadline) >= 0) {
                I2C3->CR1 &= ~I2C_CR1_PE;
                (void)I2C3->CR1; /* PE must stay low for a few APB cycles */
                (void)I2C3->CR1;
                (void)I2C3->CR1;
                I2C3->CR1 |= I2C_CR1_PE;
                break;
            }
        }
    }
    I2C3->ICR = I2C_ICR_STOPCF | I2C_ICR_NACKCF | I2C_ICR_BERRCF | I2C_ICR_ARLOCF;
}

static uint32_t cr2_for(uint8_t address, bool read, uint32_t nbytes, bool autoend) {
    return ((uint32_t)address << 1) | (read ? I2C_CR2_RD_WRN : 0U) |
           (nbytes << I2C_CR2_NBYTES_Pos) | (autoend ? I2C_CR2_AUTOEND : 0U) | I2C_CR2_START;
}

/* Does anything ACK `address`? (empty write, then STOP) */
static bool i2c_probe(uint8_t address) {
    uint32_t deadline = deadline_for(I2C_TIMEOUT_US);
    bool nack = false;
    if ((I2C3->ISR & I2C_ISR_BUSY) != 0U) {
        i2c_abort();
        return false;
    }
    I2C3->ICR = I2C_ICR_STOPCF | I2C_ICR_NACKCF;
    I2C3->CR2 = cr2_for(address, false, 0U, true);
    bool stopped = i2c_wait(I2C_ISR_STOPF, deadline, &nack);
    if (nack) {
        i2c_abort();
        return false;
    }
    I2C3->ICR = I2C_ICR_STOPCF;
    if (!stopped) {
        i2c_abort();
    }
    return stopped;
}

/* Register pointer write, repeated start, 2-byte read (MSB first). */
static fcec_i2c_read_result_t tmp112_read(void *context, uint8_t address, uint8_t reg,
                                          uint16_t *value, uint32_t timeout_us) {
    (void)context;
    uint32_t deadline = deadline_for(timeout_us);
    bool nack = false;
    uint8_t bytes[2] = {0U, 0U};

    if ((I2C3->ISR & I2C_ISR_BUSY) != 0U) {
        i2c_abort();
        return FCEC_I2C_READ_BUS_ERROR;
    }
    I2C3->ICR = I2C_ICR_STOPCF | I2C_ICR_NACKCF | I2C_ICR_BERRCF | I2C_ICR_ARLOCF;

    I2C3->CR2 = cr2_for(address, false, 1U, false);
    if (!i2c_wait(I2C_ISR_TXIS, deadline, &nack)) {
        i2c_abort();
        return nack ? FCEC_I2C_READ_NACK : FCEC_I2C_READ_TIMEOUT;
    }
    I2C3->TXDR = reg;
    if (!i2c_wait(I2C_ISR_TC, deadline, &nack)) {
        i2c_abort();
        return nack ? FCEC_I2C_READ_NACK : FCEC_I2C_READ_TIMEOUT;
    }

    I2C3->CR2 = cr2_for(address, true, 2U, true);
    for (unsigned i = 0U; i < 2U; ++i) {
        if (!i2c_wait(I2C_ISR_RXNE, deadline, &nack)) {
            i2c_abort();
            return nack ? FCEC_I2C_READ_NACK : FCEC_I2C_READ_TIMEOUT;
        }
        bytes[i] = (uint8_t)I2C3->RXDR;
    }
    if (!i2c_wait(I2C_ISR_STOPF, deadline, &nack)) {
        i2c_abort();
        return FCEC_I2C_READ_TIMEOUT;
    }
    I2C3->ICR = I2C_ICR_STOPCF;
    *value = (uint16_t)(((uint16_t)bytes[0] << 8) | bytes[1]);
    return FCEC_I2C_READ_OK;
}

/* PH9 is the shared active-low ALERT; IDR reads it even in AF mode. */
static bool alert_pin_high(void *context, bool *high) {
    (void)context;
    *high = (GPIOH->IDR & GPIO_PIN_9) != 0U;
    return true;
}

static const fcec_temperature_ops_t temp_ops = {NULL, tmp112_read, alert_pin_high};

/* ---- Public ---- */

void EC_Health_Init(void) {
    ec_health.i2c_found_count = 0U;
    for (uint8_t address = 0x08U; address <= 0x77U; ++address) {
        HAL_IWDG_Refresh(&hiwdg1); /* a stuck bus costs ~2 ms per address */
        if (i2c_probe(address) && ec_health.i2c_found_count < 16U) {
            ec_health.i2c_found[ec_health.i2c_found_count++] = address;
        }
    }
    uint64_t now_us = (uint64_t)HAL_GetTick() * 1000ULL;
    next_adc_us = now_us;
    next_temp_us = now_us;
}

void EC_Health_Process(uint64_t now_us, fcec_measurements_t *m) {
    if (now_us >= next_adc_us) {
        adc_sweep(now_us, m);
        m->sample_id++;
        m->sample_time_us = now_us;
        next_adc_us = now_us + ADC_PERIOD_US;
    }
    if (now_us >= next_temp_us) {
        fcec_temperature_result_t r;
        if (fcec_temperature_poll(&temp_ops, I2C_TIMEOUT_US, OVERTEMP_LIMIT_C, now_us, &r)) {
            fcec_temperature_publish(&r, m);
            ec_health.temp_status = r.status;
        }
        ec_health.temp_polls++;
        next_temp_us = now_us + TEMP_PERIOD_US;
    }
}

#endif /* EC_BOARD_HEALTH */
