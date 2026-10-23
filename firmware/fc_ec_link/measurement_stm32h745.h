#ifndef FCEC_MEASUREMENT_STM32H745_H
#define FCEC_MEASUREMENT_STM32H745_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define FCEC_STM32H745_ADC3_INPUT_COUNT 14u
#define FCEC_GPIO_NO_PORT 0u

typedef struct {
    uint8_t port;          /* ASCII 'F' or 'H'; 0 for internal/dual-pad input */
    uint8_t pin;           /* GPIO bit number, or 0 when port is 0 */
    uint8_t adc_input;     /* ADC3 input number */
    bool internal;         /* true for VREFINT, whose HAL channel selector is special */
    uint32_t rank;         /* rank in the regular ADC3 sequence */
    uint32_t min_sample_cycles; /* requested minimum; adapter maps to its driver API */
} fcec_stm32h745_adc3_route_t;

/* Hardware operations supplied by the target firmware. GPIO callbacks must
 * enable the corresponding GPIO/SYSCFG clocks if the MCU requires it. The
 * sample-time callback argument is in ADC clock cycles, not HAL enum values. */
typedef struct {
    void *context;
    bool (*configure_gpio_analog)(void *context, uint8_t port, uint8_t pin);
    bool (*close_pc2_analog_switch)(void *context);
    bool (*configure_adc3_input)(void *context, uint8_t adc_input, bool internal,
                                 uint32_t rank, uint32_t min_sample_cycles);
} fcec_stm32h745_ops_t;

/* Returns the fixed board route table. The first thirteen entries are ordered
 * to match the raw-code array consumed by measurement.c; the final entry is
 * VREFINT. */
const fcec_stm32h745_adc3_route_t *fcec_stm32h745_adc3_routes(size_t *count);

/* Applies the board pin/switch/channel configuration through caller-supplied
 * operations. This deliberately has no STM32 HAL or CMSIS dependency. */
bool fcec_measurement_stm32h745_configure(const fcec_stm32h745_ops_t *ops,
                                          uint32_t external_min_sample_cycles,
                                          uint32_t vrefint_min_sample_cycles,
                                          uint32_t first_rank);

#endif
