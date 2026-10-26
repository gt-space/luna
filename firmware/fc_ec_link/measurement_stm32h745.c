#include "measurement_stm32h745.h"

/* Board order: vin, iin, v12, i12, v4v4, i4v4, v3v3, i3v3,
 * im1, im2, im3, im4, vmotor, VREFINT. ADC3 input IDs are from the
 * STM32H745 pin mapping. PC2_C is a dual pad, not a GPIO port pin. */
static const fcec_stm32h745_adc3_route_t routes[FCEC_STM32H745_ADC3_INPUT_COUNT] = {
    {'F', 3u,  5u, false,  1u, 0u},
    {'F', 4u,  9u, false,  2u, 0u},
    {'F', 5u,  4u, false,  3u, 0u},
    {'F', 6u,  8u, false,  4u, 0u},
    {'F', 7u,  3u, false,  5u, 0u},
    {'F', 8u,  7u, false,  6u, 0u},
    {'F', 9u,  2u, false,  7u, 0u},
    {'F', 10u, 6u, false,  8u, 0u},
    {'H', 2u, 13u, false,  9u, 0u},
    {'H', 3u, 14u, false, 10u, 0u},
    {'H', 4u, 15u, false, 11u, 0u},
    {'H', 5u, 16u, false, 12u, 0u},
    {FCEC_GPIO_NO_PORT, 0u, 0u, false, 13u, 0u}, /* PC2_C -> ADC3_INP0 */
    {FCEC_GPIO_NO_PORT, 0u, 19u, true, 14u, 0u}  /* ADC3 VREFINT */
};

const fcec_stm32h745_adc3_route_t *fcec_stm32h745_adc3_routes(size_t *count) {
    if (count != NULL) *count = FCEC_STM32H745_ADC3_INPUT_COUNT;
    return routes;
}

bool fcec_measurement_stm32h745_configure(const fcec_stm32h745_ops_t *ops,
                                          uint32_t external_min_sample_cycles,
                                          uint32_t vrefint_min_sample_cycles) {
    size_t i;
    if (ops == NULL || ops->configure_gpio_analog == NULL ||
        ops->close_pc2_analog_switch == NULL || ops->configure_adc3_input == NULL ||
        external_min_sample_cycles == 0u || vrefint_min_sample_cycles == 0u)
        return false;

    /* Configure only the actual GPIO-backed analog inputs. PC2_C itself is
     * selected through the switch callback; PC3_C is not board-routed. */
    for (i = 0u; i < FCEC_STM32H745_ADC3_INPUT_COUNT; ++i) {
        if (routes[i].port != FCEC_GPIO_NO_PORT &&
            !ops->configure_gpio_analog(ops->context, routes[i].port, routes[i].pin))
            return false;
    }
    if (!ops->close_pc2_analog_switch(ops->context)) return false;

    for (i = 0u; i < FCEC_STM32H745_ADC3_INPUT_COUNT; ++i) {
        uint32_t sample_cycles = routes[i].internal ? vrefint_min_sample_cycles
                                                    : external_min_sample_cycles;
        if (!ops->configure_adc3_input(ops->context, routes[i].adc_input,
                                       routes[i].internal, routes[i].rank,
                                       sample_cycles))
            return false;
    }
    return true;
}
