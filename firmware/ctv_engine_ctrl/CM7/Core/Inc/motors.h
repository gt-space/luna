#ifndef MOTORS_H
#define MOTORS_H

#include "main.h"
#include <stdint.h>

/* DRV8837: IN1 = TIM4 CH1-4, IN2 = TIM5 CH1-4, common nSLEEP = PK2.
 * All four drivers share nSLEEP; disable/arm operations affect every motor.
 * Motor numbering in this API is 1..4, corresponding to schematic MTR1..4.
 */

/* Configure PWM outputs at 0% duty and leave the drivers asleep.
 * Call AFTER MX_GPIO_Init(), MX_TIM4_Init(), MX_TIM5_Init().
 */
HAL_StatusTypeDef Motors_Init(void);

/* Explicitly allow motor drive; caller must verify power, limits, and interlocks.
 * Motors_Init() never enables the DRV8837s on its own.
 */
HAL_StatusTypeDef Motors_Arm(void);

/* Immediately request shared sleep and clear all compare registers. */
void Motors_Disarm(void);

/* Signed PWM/coast drive: -1000..+1000 permille; 0 = coast.
 * Positive commands IN1 PWM / IN2 LOW, negative commands IN1 LOW / IN2 PWM.
 * Return HAL_ERROR unless the motor drivers have been armed.
 * This is duty control, NOT closed-loop speed regulation.
 */
HAL_StatusTypeDef Motors_SetDuty(uint8_t motor_number, int16_t duty_permille);
HAL_StatusTypeDef Motors_Coast(uint8_t motor_number);

/* Explicit dynamic braking: IN1=IN2=HIGH, only while armed. */
HAL_StatusTypeDef Motors_Brake(uint8_t motor_number);

uint8_t Motors_IsArmed(void);

#endif /* MOTORS_H */
