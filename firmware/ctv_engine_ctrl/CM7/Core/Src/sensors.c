#include "sensors.h"

/* ADC3 is initialized in CubeMX-generated CM7/Core/Src/main.c. */
extern ADC_HandleTypeDef hadc3;

#define ADC_READ_TIMEOUT_MS  (10U)

static uint8_t adc_ready = 0U;

/* Only accept channels wired to analog measurement signals on this PCB. */
static uint8_t ADC_IsBoardChannel(uint32_t channel)
{
    switch (channel)
    {
        case ADC_CHANNEL_0:  /* PC2_C: motor supply voltage */
        case ADC_CHANNEL_2:  /* PF9: 3.3 V rail voltage */
        case ADC_CHANNEL_3:  /* PF7: 4.4 V rail voltage */
        case ADC_CHANNEL_4:  /* PF5: 12 V rail voltage */
        case ADC_CHANNEL_5:  /* PF3: 22.2 V rail voltage */
        case ADC_CHANNEL_6:  /* PF10: 3.3 V rail current */
        case ADC_CHANNEL_7:  /* PF8: 4.4 V rail current */
        case ADC_CHANNEL_8:  /* PF6: 12 V rail current */
        case ADC_CHANNEL_9:  /* PF4: 22.2 V rail current */
        case ADC_CHANNEL_13: /* PH2: motor 1 current */
        case ADC_CHANNEL_14: /* PH3: motor 2 current */
        case ADC_CHANNEL_15: /* PH4: motor 3 current */
        case ADC_CHANNEL_16: /* PH5: motor 4 current */
            return 1U;
        default:
            return 0U;
    }
}

HAL_StatusTypeDef Sensors_ADC_Init(void)
{
    adc_ready = 0U;

    /* The ADC must not be converting during calibration. Call once at boot. */
    if (HAL_ADCEx_Calibration_Start(&hadc3, ADC_CALIB_OFFSET,
                                    ADC_SINGLE_ENDED) != HAL_OK)
    {
        return HAL_ERROR;
    }

    adc_ready = 1U;
    return HAL_OK;
}

HAL_StatusTypeDef ADC_ReadChannel(uint32_t channel, uint16_t *result)
{
    ADC_ChannelConfTypeDef config = {0};
    HAL_StatusTypeDef status;
    uint16_t raw;

    if ((result == NULL) || (adc_ready == 0U) ||
        (ADC_IsBoardChannel(channel) == 0U))
    {
        return HAL_ERROR;
    }

    /* Non-continuous, single-rank ADC3: select the input for this reading. */
    config.Channel = channel;
    config.Rank = ADC_REGULAR_RANK_1;
    config.SamplingTime = ADC_SAMPLETIME_64CYCLES_5;
    config.SingleDiff = ADC_SINGLE_ENDED;
    config.OffsetNumber = ADC_OFFSET_NONE;
    config.Offset = 0U;
    config.OffsetSignedSaturation = DISABLE;

    status = HAL_ADC_ConfigChannel(&hadc3, &config);
    if (status != HAL_OK)
    {
        return status;
    }

    status = HAL_ADC_Start(&hadc3);
    if (status != HAL_OK)
    {
        return status;
    }

    status = HAL_ADC_PollForConversion(&hadc3, ADC_READ_TIMEOUT_MS);
    if (status != HAL_OK)
    {
        (void)HAL_ADC_Stop(&hadc3);
        return status;  /* HAL_TIMEOUT or HAL_ERROR; *result unchanged */
    }

    raw = (uint16_t)HAL_ADC_GetValue(&hadc3);

    status = HAL_ADC_Stop(&hadc3);
    if (status != HAL_OK)
    {
        return status;
    }

    *result = raw;
    return HAL_OK;
}
