#include "esc_tim2.h"

#include "main.h"

extern TIM_HandleTypeDef htim2;

/* TIM2 sits on APB1 (D2PPRE1). With TIMPRE = 0 (reset value) the timer clock
 * is PCLK1, or twice PCLK1 when the APB1 prescaler isn't 1. */
static uint32_t tim2_clock_hz(void) {
    uint32_t pclk1 = HAL_RCC_GetPCLK1Freq();
    if ((RCC->CFGR & RCC_CFGR_TIMPRE) != 0U) {
        return 0U; /* not handled: refuse rather than guess */
    }
    return ((RCC->D2CFGR & RCC_D2CFGR_D2PPRE1) == 0U) ? pclk1 : 2U * pclk1;
}

bool ESC_TIM2_Start(uint16_t frame_period_us, uint16_t first_pulse_us) {
    uint32_t clk = tim2_clock_hz();
    if (clk < 1000000U || clk % 1000000U != 0U || frame_period_us == 0U ||
        first_pulse_us >= frame_period_us) {
        return false;
    }

    (void)HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
    htim2.Instance->PSC = clk / 1000000U - 1U; /* 1 us per count */
    __HAL_TIM_SET_AUTORELOAD(&htim2, (uint32_t)frame_period_us - 1U);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, first_pulse_us);
    htim2.Instance->CR1 |= TIM_CR1_ARPE;
    htim2.Instance->CCMR1 |= TIM_CCMR1_OC1PE; /* new pulses wait for the frame boundary */
    htim2.Instance->EGR = TIM_EGR_UG;          /* load PSC/ARR/CCR now */
    return HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1) == HAL_OK;
}

void ESC_TIM2_Load(uint16_t pulse_us) {
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, pulse_us);
}

void ESC_TIM2_Stop(void) {
    (void)HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
}
