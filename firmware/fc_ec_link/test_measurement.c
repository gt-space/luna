/*
 * Host tests for the EC measurement helpers: ADC conversion and publication
 * (measurement.c), the ADC3 route table (measurement_stm32h745.c) and the
 * TMP112 poller (temperature.c), with fake hardware.
 *
 * Run with `make test` in this folder.
 */

#include "measurement.h"
#include "measurement_stm32h745.h"
#include "temperature.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

static int checks;
static int failures;

#define CHECK(cond)                                                       \
    do {                                                                  \
        checks++;                                                         \
        if (!(cond)) {                                                    \
            failures++;                                                   \
            printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);        \
        }                                                                 \
    } while (0)

#define NEAR(a, b, tol) (fabsf((a) - (b)) <= (tol))

#define N FCEC_ADC3_EXTERNAL_COUNT
#define NOW 1000000u
#define FULL_SCALE 65535u

/* Board order: vin, iin, v12, i12, v4v4, i4v4, v3v3, i3v3, im1..im4, vmotor. */
enum { VIN, IIN, V12, I12, V4V4, I4V4, V3V3, I3V3, IM1, IM2, IM3, IM4, VMOTOR };

static uint16_t codes[N];
static uint64_t times[N];
static fcec_measurement_cal_t cal[N];
static fcec_measurement_value_t out[N];

/* ADC code for `pin_v` at a 3.3 V reference. */
static uint16_t code_for(float pin_v) {
    return (uint16_t)(pin_v / 3.3f * (float)FULL_SCALE + 0.5f);
}

static fcec_measurement_sweep_t sweep(void) {
    fcec_measurement_sweep_t s;
    memset(&s, 0, sizeof s);
    s.adc_full_scale = FULL_SCALE;
    s.calibrated_vdda_v = 3.3f;
    s.adc_ok = true;
    s.vdda_valid = true;
    s.raw_codes = codes;
    s.sample_times_us = times;
    s.now_us = NOW;
    return s;
}

/* Nominal calibration, wide limits, every channel at a mid-scale pin voltage. */
static void reset_inputs(void) {
    for (unsigned i = 0; i < N; i++) {
        memset(&cal[i], 0, sizeof cal[i]);
        cal[i].scale = 1.0f;
        cal[i].offset = 0.0f;
        cal[i].minimum = -100.0f;
        cal[i].maximum = 100.0f;
        cal[i].floor_pin_v = 0.01f;
        cal[i].saturation_code = 65000u;
        cal[i].stale_after_us = 10000u;
        cal[i].configured = true;
        codes[i] = code_for(1.0f);
        times[i] = NOW - 1000u;
    }
}

static void test_nominal_conversion_matches_the_board(void) {
    reset_inputs();
    fcec_measurement_sweep_t s = sweep();
    fcec_measurement_convert(&s, cal, out);

    /* 1.0 V at the pin through each front end (MEASUREMENTS.md). */
    CHECK(out[VIN].valid && NEAR(out[VIN].value, 11.0f, 0.01f));  /* x11 divider */
    CHECK(out[V12].valid && NEAR(out[V12].value, 11.0f, 0.01f));
    CHECK(out[V4V4].valid && NEAR(out[V4V4].value, 2.0f, 0.01f)); /* x2 */
    CHECK(out[VMOTOR].valid && NEAR(out[VMOTOR].value, 2.0f, 0.01f));
    CHECK(out[IIN].valid && NEAR(out[IIN].value, 2.0f, 0.01f));   /* 0.5 V/A */
    CHECK(out[I4V4].valid && NEAR(out[I4V4].value, 1.0f, 0.01f)); /* 1.0 V/A */
    CHECK(out[I3V3].valid && NEAR(out[I3V3].value, 0.4f, 0.01f)); /* 2.5 V/A */
    CHECK(out[IM1].valid && NEAR(out[IM1].value, 0.2f, 0.01f));   /* 5.0 V/A */
    CHECK(out[VIN].quality == FCEC_QUALITY_OK);
    /* 3.3 V tracks the ADC reference, so it is only a sanity check. */
    CHECK(out[V3V3].valid && out[V3V3].quality == FCEC_QUALITY_RATIOMETRIC);
    CHECK(out[VIN].raw_code == codes[VIN] && out[VIN].sample_time_us == times[VIN]);

    /* Calibration scale and offset apply after the nominal factor. */
    cal[VIN].scale = 1.01f;
    cal[VIN].offset = -0.1f;
    fcec_measurement_convert(&s, cal, out);
    CHECK(NEAR(out[VIN].value, 11.0f * 1.01f - 0.1f, 0.01f));
}

static void test_bad_inputs_are_never_reported_as_values(void) {
    reset_inputs();
    fcec_measurement_sweep_t s = sweep();

    cal[VIN].configured = false;
    codes[V12] = 65000u;               /* at the saturation code */
    times[V4V4] = NOW - 20000u;        /* older than stale_after_us */
    times[V3V3] = NOW + 1u;            /* from the future */
    codes[IIN] = code_for(0.005f);     /* below the current floor */
    codes[VMOTOR] = 0u;                /* voltages have no floor check: 0 V is real */
    cal[I12].maximum = 1.0f;           /* 2 A is out of range */
    fcec_measurement_convert(&s, cal, out);

    CHECK(!out[VIN].valid && out[VIN].quality == FCEC_QUALITY_UNCONFIGURED);
    CHECK(!out[V12].valid && out[V12].quality == FCEC_QUALITY_SATURATED_HIGH);
    CHECK(!out[V4V4].valid && out[V4V4].quality == FCEC_QUALITY_STALE);
    CHECK(!out[V3V3].valid && out[V3V3].quality == FCEC_QUALITY_STALE);
    CHECK(!out[IIN].valid && out[IIN].quality == FCEC_QUALITY_AT_FLOOR);
    CHECK(out[VMOTOR].valid && out[VMOTOR].value == 0.0f);
    CHECK(!out[I12].valid && out[I12].quality == FCEC_QUALITY_OUT_OF_RANGE);
    CHECK(isnan(out[VIN].value) && isnan(out[I12].value));

    /* A bad reference or ADC invalidates everything configured. */
    reset_inputs();
    s.vdda_valid = false;
    fcec_measurement_convert(&s, cal, out);
    for (unsigned i = 0; i < N; i++) {
        CHECK(!out[i].valid && out[i].quality == FCEC_QUALITY_ADC_FAULT);
    }
    s = sweep();
    s.adc_ok = false;
    fcec_measurement_convert(&s, cal, out);
    CHECK(!out[VIN].valid && out[VIN].quality == FCEC_QUALITY_ADC_FAULT);
    s = sweep();
    s.raw_codes = NULL;
    fcec_measurement_convert(&s, cal, out);
    CHECK(!out[VIN].valid);
}

static void test_publish_puts_each_value_in_its_wire_slot(void) {
    reset_inputs();
    for (unsigned i = 0; i < N; i++) {
        codes[i] = (uint16_t)(1000u + i);
    }
    fcec_measurement_sweep_t s = sweep();
    fcec_measurement_convert(&s, cal, out);
    out[IM3].valid = false; /* one invalid channel */

    fcec_measurements_t m;
    memset(&m, 0, sizeof m);
    m.validity_mask = 1u << 12; /* stale bit from an earlier batch: must clear */
    fcec_measurement_publish_adc(out, &m);

    CHECK(m.rail_voltage_v[0] == out[VIN].value && m.rail_voltage_v[1] == out[V12].value);
    CHECK(m.rail_voltage_v[2] == out[V4V4].value && m.rail_voltage_v[3] == out[V3V3].value);
    CHECK(m.motor_rail_voltage_v == out[VMOTOR].value);
    CHECK(m.rail_current_a[0] == out[IIN].value && m.rail_current_a[3] == out[I3V3].value);
    CHECK(m.rcs_current_a[0] == out[IM1].value && m.rcs_current_a[3] == out[IM4].value);
    for (unsigned i = 0; i < N; i++) {
        CHECK(m.adc_raw_code[i] == codes[i]);
    }
    /* Validity: 2-5 rails, 6-9 rail currents, 10-13 fan currents, 14 motor rail. */
    uint32_t expected = 0x3Cu | 0x3C0u | (0xFu << 10) | (1u << 14);
    expected &= ~(1u << 12); /* IM3 invalid */
    CHECK(m.validity_mask == expected);
    /* Per-value metadata uses the wire order (vin = index 2, im1 = 11, vmotor = 6). */
    CHECK(m.measurement_sample_time_us[2] == times[VIN]);
    CHECK(m.measurement_quality[5] == FCEC_QUALITY_RATIOMETRIC); /* v3v3 */
    CHECK(m.measurement_sample_time_us[6] == times[VMOTOR]);
    CHECK(m.measurement_sample_time_us[11] == times[IM1]);
}

static void test_pa12_and_vdda_helpers(void) {
    fcec_measurements_t m;
    memset(&m, 0, sizeof m);
    CHECK(fcec_measurement_publish_pa12(&m, 1, 2048, 13.4f, 7, 0, NOW, true));
    CHECK(m.validity_mask == 0x2u && m.tvc_position_counts[1] == 2048);
    CHECK(m.tvc_position_age_ms[1] == 7 && m.measurement_quality[1] == FCEC_QUALITY_OK);
    CHECK(fcec_measurement_publish_pa12(&m, 1, 2048, NAN, 7, 0, NOW, true));
    CHECK(m.validity_mask == 0u);
    CHECK(fcec_measurement_publish_pa12(&m, 0, 0, 0.0f, 0, 0, NOW, false));
    CHECK(m.measurement_quality[0] == FCEC_QUALITY_TRANSPORT_FAULT);
    CHECK(!fcec_measurement_publish_pa12(&m, 2, 0, 0.0f, 0, 0, NOW, true));

    float vdda = 0.0f;
    CHECK(fcec_measurement_derive_vdda(24000, 24000, 65535, 65535, 3.3f, &vdda));
    CHECK(NEAR(vdda, 3.3f, 1e-4f));
    CHECK(fcec_measurement_derive_vdda(26400, 24000, 65535, 65535, 3.3f, &vdda));
    CHECK(NEAR(vdda, 3.0f, 1e-4f));
    CHECK(!fcec_measurement_derive_vdda(0, 24000, 65535, 65535, 3.3f, &vdda));
    CHECK(!fcec_measurement_derive_vdda(24000, 24000, 65535, 65535, NAN, &vdda));
}

static void test_adc3_routes_match_the_rev2_pinout(void) {
    /* CubeMX .ioc (Rev 2): PF3 INP5, PF4 INP9, PF5 INP4, PF6 INP8, PF7 INP3,
     * PF8 INP7, PF9 INP2, PF10 INP6, PH2-PH5 INP13-16, PC2_C INP0. */
    static const struct { uint8_t port, pin, input; } expected[13] = {
        {'F', 3, 5}, {'F', 4, 9}, {'F', 5, 4}, {'F', 6, 8}, {'F', 7, 3},
        {'F', 8, 7}, {'F', 9, 2}, {'F', 10, 6}, {'H', 2, 13}, {'H', 3, 14},
        {'H', 4, 15}, {'H', 5, 16}, {FCEC_GPIO_NO_PORT, 0, 0},
    };
    size_t count = 0;
    const fcec_stm32h745_adc3_route_t *r = fcec_stm32h745_adc3_routes(&count);
    CHECK(count == 14);
    for (unsigned i = 0; i < 13; i++) {
        CHECK(r[i].port == expected[i].port && r[i].pin == expected[i].pin);
        CHECK(r[i].adc_input == expected[i].input && !r[i].internal);
        CHECK(r[i].rank == i + 1u);
    }
    CHECK(r[13].internal && r[13].rank == 14u); /* VREFINT last */
}

/* ---- TMP112 ---- */

typedef struct {
    fcec_i2c_read_result_t result[2]; /* U3, U4 */
    uint16_t config[2];
    uint16_t temp[2];
    bool alert_read_ok;
    bool alert_high;
    unsigned reads;
} fake_i2c_t;

static fcec_i2c_read_result_t fake_read(void *context, uint8_t address, uint8_t reg,
                                        uint16_t *value, uint32_t timeout_us) {
    fake_i2c_t *f = context;
    unsigned dev = address == FCEC_TMP112_U3_ADDRESS ? 0u : 1u;
    f->reads++;
    CHECK(timeout_us > 0u);
    CHECK(address == FCEC_TMP112_U3_ADDRESS || address == FCEC_TMP112_U4_ADDRESS);
    if (f->result[dev] != FCEC_I2C_READ_OK) {
        return f->result[dev];
    }
    *value = reg == FCEC_TMP112_CONFIG_REGISTER ? f->config[dev] : f->temp[dev];
    return FCEC_I2C_READ_OK;
}

static bool fake_alert(void *context, bool *high) {
    fake_i2c_t *f = context;
    *high = f->alert_high;
    return f->alert_read_ok;
}

static fake_i2c_t fake_ok(void) {
    fake_i2c_t f;
    memset(&f, 0, sizeof f);
    f.alert_read_ok = true;
    f.alert_high = true;
    f.temp[0] = 0x1900; /* 25.0 C, 12-bit */
    f.temp[1] = 0xF600; /* -10.0 C, 12-bit */
    return f;
}

static void test_tmp112_decoding_and_publication(void) {
    fake_i2c_t f = fake_ok();
    fcec_temperature_ops_t ops = {&f, fake_read, fake_alert};
    fcec_temperature_result_t r;

    CHECK(fcec_temperature_poll(&ops, 1000u, 80.0f, NOW, &r));
    CHECK(r.valid_mask == 0x3u && r.status == 0u);
    CHECK(NEAR(r.temperature_c[0], 25.0f, 1e-4f));
    CHECK(NEAR(r.temperature_c[1], -10.0f, 1e-4f));
    CHECK(r.sample_time_us[0] == NOW);

    /* Extended mode: 13-bit data. */
    f.config[0] = 1u << 4;
    f.temp[0] = 0x4B00; /* 150.0 C */
    CHECK(fcec_temperature_poll(&ops, 1000u, 80.0f, NOW, &r));
    CHECK(NEAR(r.temperature_c[0], 150.0f, 1e-4f));
    CHECK(r.status == FCEC_TEMP_U3_OVERTEMP);

    fcec_measurements_t m;
    memset(&m, 0, sizeof m);
    fcec_temperature_publish(&r, &m);
    CHECK((m.validity_mask & (3u << 15)) == (3u << 15));
    CHECK(m.board_temp_c[0] == r.temperature_c[0]);
    CHECK(m.measurement_quality[15] == FCEC_QUALITY_DEVICE_FAULT); /* over limit */
    CHECK(m.measurement_quality[16] == FCEC_QUALITY_OK);
    CHECK(m.temperature_status == FCEC_TEMP_U3_OVERTEMP);
    CHECK(m.sample_time_us == NOW);
}

static void test_tmp112_failures_are_kept_apart(void) {
    fcec_temperature_result_t r;

    /* U4 missing (NACK): only U4 invalid, not a bus fault. */
    fake_i2c_t f = fake_ok();
    fcec_temperature_ops_t ops = {&f, fake_read, fake_alert};
    f.result[1] = FCEC_I2C_READ_NACK;
    CHECK(fcec_temperature_poll(&ops, 1000u, 80.0f, NOW, &r));
    CHECK(r.valid_mask == 0x1u);
    CHECK(r.status == FCEC_TEMP_U4_DEVICE_FAILED);
    CHECK(isnan(r.temperature_c[1]));

    /* U3 times out: U3 invalid and the bus flagged; U4 still read. */
    f = fake_ok();
    f.result[0] = FCEC_I2C_READ_TIMEOUT;
    CHECK(fcec_temperature_poll(&ops, 1000u, 80.0f, NOW, &r));
    CHECK(r.valid_mask == 0x2u);
    CHECK(r.status == (FCEC_TEMP_U3_DEVICE_FAILED | FCEC_TEMP_BUS_FAILED));

    /* ALERT low, then ALERT unreadable. */
    f = fake_ok();
    f.alert_high = false;
    CHECK(fcec_temperature_poll(&ops, 1000u, 80.0f, NOW, &r));
    CHECK(r.status == FCEC_TEMP_ALERT_ACTIVE);
    f.alert_read_ok = false;
    CHECK(fcec_temperature_poll(&ops, 1000u, 80.0f, NOW, &r));
    CHECK(r.status == FCEC_TEMP_ALERT_READ_FAILED);

    /* A failed device clears its validity bit in the snapshot. */
    fcec_measurements_t m;
    memset(&m, 0, sizeof m);
    m.validity_mask = 3u << 15;
    f = fake_ok();
    f.result[0] = FCEC_I2C_READ_NACK;
    fcec_temperature_poll(&ops, 1000u, 80.0f, NOW, &r);
    fcec_temperature_publish(&r, &m);
    CHECK((m.validity_mask & (3u << 15)) == (1u << 16));
    CHECK(m.measurement_quality[15] == FCEC_QUALITY_TRANSPORT_FAULT);

    /* No unbounded waits: a zero timeout or missing ops is refused. */
    CHECK(!fcec_temperature_poll(&ops, 0u, 80.0f, NOW, &r));
    CHECK(!fcec_temperature_poll(NULL, 1000u, 80.0f, NOW, &r));
    CHECK(!fcec_temperature_poll(&ops, 1000u, NAN, NOW, &r));
}

int main(void) {
    test_nominal_conversion_matches_the_board();
    test_bad_inputs_are_never_reported_as_values();
    test_publish_puts_each_value_in_its_wire_slot();
    test_pa12_and_vdda_helpers();
    test_adc3_routes_match_the_rev2_pinout();
    test_tmp112_decoding_and_publication();
    test_tmp112_failures_are_kept_apart();

    printf("%d checks, %d failed\n", checks, failures);
    return failures == 0 ? 0 : 1;
}
