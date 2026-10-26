#include "measurement.h"

#include <math.h>

/* Nominal board transfer: voltage divider multiplier; or INA180 output V/A. */
static const float nominal_factor[FCEC_ADC3_EXTERNAL_COUNT] = {
    11.0f, 2.0f, 11.0f, 2.0f, 2.0f, 1.0f, 2.0f, 0.4f,
    0.2f, 0.2f, 0.2f, 0.2f, 2.0f
};

static bool is_rail_voltage(unsigned i) { return i == 0u || i == 2u || i == 4u || i == 6u || i == 12u; }

static uint8_t wire_quality(fcec_measurement_quality_t quality) {
    return (uint8_t)quality;
}

void fcec_measurement_publish_adc(
    const fcec_measurement_value_t values[FCEC_ADC3_EXTERNAL_COUNT],
    fcec_measurements_t *m) {
    static const uint8_t wire_index[FCEC_ADC3_EXTERNAL_COUNT] = {
        2u, 7u, 3u, 8u, 4u, 9u, 5u, 10u, 11u, 12u, 13u, 14u, 6u
    };
    if (values == NULL || m == NULL) return;

    for (unsigned i = 0; i < FCEC_ADC3_EXTERNAL_COUNT; ++i) {
        const fcec_measurement_value_t *v = &values[i];
        const uint8_t index = wire_index[i];
        const uint32_t bit = (index >= 2u && index <= 5u) ? (1u << index) :
                             (index >= 7u && index <= 10u) ? (1u << (index - 1u)) :
                             (index == 6u) ? (1u << 14u) : (1u << (index - 1u));
        m->measurement_sample_time_us[index] = v->sample_time_us;
        m->measurement_quality[index] = wire_quality(v->quality);
        m->adc_raw_code[i] = v->raw_code;
        if (v->valid && isfinite(v->value)) {
            m->validity_mask |= bit;
        } else {
            m->validity_mask &= ~bit;
        }
        switch (i) {
        case 0: m->rail_voltage_v[0] = v->value; break;
        case 2: m->rail_voltage_v[1] = v->value; break;
        case 4: m->rail_voltage_v[2] = v->value; break;
        case 6: m->rail_voltage_v[3] = v->value; break;
        case 12: m->motor_rail_voltage_v = v->value; break;
        case 1: m->rail_current_a[0] = v->value; break;
        case 3: m->rail_current_a[1] = v->value; break;
        case 5: m->rail_current_a[2] = v->value; break;
        case 7: m->rail_current_a[3] = v->value; break;
        default: m->rcs_current_a[i - 8u] = v->value; break;
        }
    }
}

bool fcec_measurement_publish_pa12(fcec_measurements_t *m,
                                   unsigned actuator_index,
                                   uint16_t position_count,
                                   float position_mm,
                                   uint16_t age_ms,
                                   uint16_t fault_flags,
                                   uint64_t sample_time_us,
                                   bool valid) {
    if (m == NULL || actuator_index >= 2u) return false;
    const uint32_t bit = 1u << actuator_index;
    m->tvc_position_counts[actuator_index] = position_count;
    m->tvc_position_mm[actuator_index] = position_mm;
    m->tvc_position_age_ms[actuator_index] = age_ms;
    m->pa12_fault_flags[actuator_index] = fault_flags;
    m->measurement_sample_time_us[actuator_index] = sample_time_us;
    m->measurement_quality[actuator_index] = valid ? FCEC_QUALITY_OK : FCEC_QUALITY_TRANSPORT_FAULT;
    if (valid && isfinite(position_mm)) m->validity_mask |= bit;
    else m->validity_mask &= ~bit;
    return true;
}

bool fcec_measurement_derive_vdda(uint16_t vrefint_code, uint16_t factory_code,
                                 uint16_t sample_full_scale, uint16_t factory_full_scale,
                                 float factory_voltage_v, float *vdda_v) {
    float result;
    if (vdda_v == NULL || vrefint_code == 0u || factory_code == 0u ||
        sample_full_scale == 0u || factory_full_scale == 0u ||
        !isfinite(factory_voltage_v) || factory_voltage_v <= 0.0f)
        return false;
    result = factory_voltage_v * ((float)factory_code / (float)factory_full_scale) /
             ((float)vrefint_code / (float)sample_full_scale);
    if (!isfinite(result) || result <= 0.0f) return false;
    *vdda_v = result;
    return true;
}

void fcec_measurement_convert(const fcec_measurement_sweep_t *sweep,
                              const fcec_measurement_cal_t cal[FCEC_ADC3_EXTERNAL_COUNT],
                              fcec_measurement_value_t out[FCEC_ADC3_EXTERNAL_COUNT]) {
    unsigned i;
    if (out == NULL) return;
    for (i = 0u; i < FCEC_ADC3_EXTERNAL_COUNT; ++i) {
        fcec_measurement_value_t *v = &out[i];
        float pin_v;
        v->value = NAN;
        v->raw_code = 0u;
        v->sample_time_us = 0u;
        v->quality = FCEC_MEAS_ADC_FAULT;
        v->valid = false;
        if (sweep == NULL || cal == NULL || sweep->raw_codes == NULL || sweep->sample_times_us == NULL) continue;
        v->raw_code = sweep->raw_codes[i];
        v->sample_time_us = sweep->sample_times_us[i];
        if (!cal[i].configured || !isfinite(cal[i].scale) || !isfinite(cal[i].offset) ||
            !isfinite(cal[i].minimum) || !isfinite(cal[i].maximum) ||
            !isfinite(cal[i].floor_pin_v) || cal[i].saturation_code == 0u ||
            cal[i].minimum > cal[i].maximum || sweep->adc_full_scale == 0u) {
            v->quality = FCEC_MEAS_UNCONFIGURED;
            continue;
        }
        if (!sweep->adc_ok || !sweep->vdda_valid || !isfinite(sweep->calibrated_vdda_v) ||
            sweep->calibrated_vdda_v <= 0.0f) {
            v->quality = FCEC_MEAS_ADC_FAULT;
            continue;
        }
        if (v->raw_code >= cal[i].saturation_code) {
            v->quality = FCEC_MEAS_SATURATED_HIGH;
            continue;
        }
        if (v->sample_time_us > sweep->now_us ||
            sweep->now_us - v->sample_time_us > cal[i].stale_after_us) {
            v->quality = FCEC_MEAS_STALE;
            continue;
        }
        pin_v = (float)v->raw_code * sweep->calibrated_vdda_v / (float)sweep->adc_full_scale;
        if (cal[i].floor_pin_v >= 0.0f && !is_rail_voltage(i) && pin_v <= cal[i].floor_pin_v) {
            v->quality = FCEC_MEAS_AT_FLOOR;
            continue;
        }
        v->value = ((pin_v * nominal_factor[i]) * cal[i].scale) + cal[i].offset;
        if (!isfinite(v->value) || v->value < cal[i].minimum || v->value > cal[i].maximum) {
            v->quality = FCEC_MEAS_OUT_OF_RANGE;
            v->value = NAN;
            continue;
        }
        v->quality = (i == 6u) ? FCEC_MEAS_RATIOMETRIC : FCEC_MEAS_OK;
        v->valid = true;
    }
}
