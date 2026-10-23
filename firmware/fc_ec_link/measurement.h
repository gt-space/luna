#ifndef FCEC_MEASUREMENT_H
#define FCEC_MEASUREMENT_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "fc_ec_link.h"

#define FCEC_ADC3_EXTERNAL_COUNT 13u

typedef enum {
    FCEC_MEAS_OK = 0,
    FCEC_MEAS_RATIOMETRIC,
    FCEC_MEAS_STALE,
    FCEC_MEAS_SATURATED_HIGH,
    FCEC_MEAS_AT_FLOOR,
    FCEC_MEAS_OUT_OF_RANGE,
    FCEC_MEAS_UNSETTLED,
    FCEC_MEAS_NOT_SAMPLED,
    FCEC_MEAS_ADC_FAULT,
    FCEC_MEAS_UNCONFIGURED
} fcec_measurement_quality_t;

typedef struct {
    float scale;              /* correction applied after nominal conversion */
    float offset;             /* engineering units */
    float minimum;
    float maximum;
    float floor_pin_v;        /* negative disables floor check */
    uint16_t saturation_code; /* codes at/above this are invalid */
    uint32_t stale_after_us;
    bool configured;
} fcec_measurement_cal_t;

typedef struct {
    float value;
    uint16_t raw_code;
    uint64_t sample_time_us;
    fcec_measurement_quality_t quality;
    bool valid;
} fcec_measurement_value_t;

typedef struct {
    uint16_t adc_full_scale; /* e.g. 65535 for configured 16-bit ADC */
    float calibrated_vdda_v;
    bool adc_ok;
    bool vdda_valid;
    const uint16_t *raw_codes; /* 13 entries, board order below */
    const uint64_t *sample_times_us; /* 13 acquisition times */
    uint64_t now_us;
} fcec_measurement_sweep_t;

/* Board order: vin, iin, v12, i12, v4v4, i4v4, v3v3, i3v3,
 * im1, im2, im3, im4, vmotor. */
void fcec_measurement_convert(const fcec_measurement_sweep_t *sweep,
                              const fcec_measurement_cal_t cal[FCEC_ADC3_EXTERNAL_COUNT],
                              fcec_measurement_value_t out[FCEC_ADC3_EXTERNAL_COUNT]);

/* Copies converted ADC results, acquisition metadata, quality, and raw ADC
 * codes into the EC status measurement block. It owns the ADC-derived values
 * and validity bits; PA-12 and temperature publishers own their respective
 * fields. The caller increments sample_id once per publication batch. */
void fcec_measurement_publish_adc(
    const fcec_measurement_value_t values[FCEC_ADC3_EXTERNAL_COUNT],
    fcec_measurements_t *measurements);

/* Publishes one validated PA-12 position sample while preserving the raw
 * protocol count for diagnosis. actuator_index is 0 for A1 and 1 for A2. */
bool fcec_measurement_publish_pa12(fcec_measurements_t *measurements,
                                   unsigned actuator_index,
                                   uint16_t position_count,
                                   float position_mm,
                                   uint16_t age_ms,
                                   uint16_t fault_flags,
                                   uint64_t sample_time_us,
                                   bool valid);

/* Derive VDDA from the factory VREFINT calibration word. The caller supplies
 * the calibration word's documented voltage for the exact MCU revision. */
bool fcec_measurement_derive_vdda(uint16_t vrefint_code, uint16_t factory_code,
                                 uint16_t sample_full_scale, uint16_t factory_full_scale,
                                 float factory_voltage_v, float *vdda_v);

#endif
