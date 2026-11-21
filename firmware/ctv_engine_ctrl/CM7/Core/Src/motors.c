#include "motors.h"

/* HAL timers are initialized by CubeMX in main.c. */
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim5;

static const uint32_t motor_channels[4] = {
    TIM_CHANNEL_1, TIM_CHANNEL_2, TIM_CHANNEL_3, TIM_CHANNEL_4
};

static uint8_t initialized = 0U;
static uint8_t armed = 0U;
static int16_t previous_duty[4];

static void set_pwm_pair(uint8_t index, uint32_t in1, uint32_t in2)
{
    __HAL_TIM_SET_COMPARE(&htim4, motor_channels[index], in1);
    __HAL_TIM_SET_COMPARE(&htim5, motor_channels[index], in2);
}

static void all_coast(void)
{
    for (uint8_t i = 0; i < 4U; ++i) {
        set_pwm_pair(i, 0U, 0U);
        previous_duty[i] = 0;
    }
}

static uint32_t compare_for_permille(TIM_HandleTypeDef *timer, uint16_t magnitude)
{
    /* CCR may equal ARR+1 to force 100% in PWM mode 1. */
    uint32_t period = __HAL_TIM_GET_AUTORELOAD(timer) + 1UL;
    return (uint32_t)((((uint64_t)period * magnitude) + 500ULL) / 1000ULL);
}

HAL_StatusTypeDef Motors_Init(void)
{
    HAL_GPIO_WritePin(mtr_nSLEEP_GPIO_Port, mtr_nSLEEP_Pin, GPIO_PIN_RESET);
    armed = 0U;
    initialized = 0U;
    all_coast();

    if ((htim4.Init.Period != htim5.Init.Period) ||
        (htim4.Init.Prescaler != htim5.Init.Prescaler)) {
        return HAL_ERROR;
    }

    for (uint8_t i = 0U; i < 4U; ++i) {
        if (HAL_TIM_PWM_Start(&htim4, motor_channels[i]) != HAL_OK) {
            return HAL_ERROR;
        }
        if (HAL_TIM_PWM_Start(&htim5, motor_channels[i]) != HAL_OK) {
            return HAL_ERROR;
        }
    }

    /* Allow compare registers to be loaded while drivers are asleep. */
    HAL_Delay(1U);
    initialized = 1U;
    return HAL_OK;
}

HAL_StatusTypeDef Motors_Arm(void)
{
    if (initialized == 0U) {
        return HAL_ERROR;
    }
    all_coast();
    HAL_Delay(1U); /* Give both independent timers time to apply 0% duty. */
    HAL_GPIO_WritePin(mtr_nSLEEP_GPIO_Port, mtr_nSLEEP_Pin, GPIO_PIN_SET);
    HAL_Delay(1U); /* Allow driver wake-up before any motor commands. */
    armed = 1U;
    return HAL_OK;
}

void Motors_Disarm(void)
{
    HAL_GPIO_WritePin(mtr_nSLEEP_GPIO_Port, mtr_nSLEEP_Pin, GPIO_PIN_RESET);
    armed = 0U;
    all_coast();
}

HAL_StatusTypeDef Motors_Coast(uint8_t motor_number)
{
    if (motor_number < 1U || motor_number > 4U || initialized == 0U) {
        return HAL_ERROR;
    }
    uint8_t i = (uint8_t)(motor_number - 1U);
    set_pwm_pair(i, 0U, 0U);
    previous_duty[i] = 0;
    return HAL_OK;
}

HAL_StatusTypeDef Motors_Brake(uint8_t motor_number)
{
    if (motor_number < 1U || motor_number > 4U || armed == 0U) {
        return HAL_ERROR;
    }
    uint8_t i = (uint8_t)(motor_number - 1U);
    /* Both inputs HIGH -> brake (DRV8837, not coast). */
    set_pwm_pair(i, __HAL_TIM_GET_AUTORELOAD(&htim4) + 1UL,
                    __HAL_TIM_GET_AUTORELOAD(&htim5) + 1UL);
    previous_duty[i] = 0;
    return HAL_OK;
}

HAL_StatusTypeDef Motors_SetDuty(uint8_t motor_number, int16_t duty_permille)
{
    if (motor_number < 1U || motor_number > 4U || armed == 0U ||
        duty_permille < -1000 || duty_permille > 1000) {
        return HAL_ERROR;
    }

    uint8_t i = (uint8_t)(motor_number - 1U);
    if (duty_permille == 0) {
        return Motors_Coast(motor_number);
    }

    /* TIM4 and TIM5 are distinct, unsynchronized timers. On reversal, coast
     * for longer than one PWM period to avoid transient opposing commands.
     */
    if ((previous_duty[i] > 0 && duty_permille < 0) ||
        (previous_duty[i] < 0 && duty_permille > 0)) {
        set_pwm_pair(i, 0U, 0U);
        HAL_Delay(1U);
    }

    uint16_t magnitude = (duty_permille > 0)
        ? (uint16_t)duty_permille : (uint16_t)(-duty_permille);
    if (duty_permille > 0) {
        set_pwm_pair(i, compare_for_permille(&htim4, magnitude), 0U);
    } else {
        set_pwm_pair(i, 0U, compare_for_permille(&htim5, magnitude));
    }
    previous_duty[i] = duty_permille;
    return HAL_OK;
}

uint8_t Motors_IsArmed(void)
{
    return armed;
}
