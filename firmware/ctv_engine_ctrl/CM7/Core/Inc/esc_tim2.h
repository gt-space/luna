#ifndef ESC_TIM2_H
#define ESC_TIM2_H

#include <stdbool.h>
#include <stdint.h>

/* Shared main-ESC signal on PA5 (TIM2_CH1), one pulse per frame.
 *
 * Start sets 1 us timer ticks from the real TIM2 clock, a frame of
 * `frame_period_us` and the first pulse, with compare preload on: a pulse
 * loaded with ESC_TIM2_Load() goes out from the next frame boundary, never
 * mid-pulse. Returns false (and leaves the pin idle) if the clock can't give
 * whole microseconds or the timer won't start.
 */
bool ESC_TIM2_Start(uint16_t frame_period_us, uint16_t first_pulse_us);
void ESC_TIM2_Load(uint16_t pulse_us);
void ESC_TIM2_Stop(void);

#endif /* ESC_TIM2_H */
