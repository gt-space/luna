/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32h5xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define FLT_C2_Pin GPIO_PIN_13
#define FLT_C2_GPIO_Port GPIOC
#define FLT_D2_Pin GPIO_PIN_14
#define FLT_D2_GPIO_Port GPIOC
#define FLT_E2_Pin GPIO_PIN_15
#define FLT_E2_GPIO_Port GPIOC
#define FLT_B2_Pin GPIO_PIN_0
#define FLT_B2_GPIO_Port GPIOH
#define FLT_A2_Pin GPIO_PIN_1
#define FLT_A2_GPIO_Port GPIOH
#define MAG_NCS_Pin GPIO_PIN_0
#define MAG_NCS_GPIO_Port GPIOC
#define BAR_NCS_Pin GPIO_PIN_1
#define BAR_NCS_GPIO_Port GPIOC
#define IMU_NCS_Pin GPIO_PIN_2
#define IMU_NCS_GPIO_Port GPIOC
#define VREF_CH2_DR1_Pin GPIO_PIN_0
#define VREF_CH2_DR1_GPIO_Port GPIOA
#define SNS_2_Pin GPIO_PIN_1
#define SNS_2_GPIO_Port GPIOA
#define SNS_1_Pin GPIO_PIN_2
#define SNS_1_GPIO_Port GPIOA
#define VREF_CH1_DR1_Pin GPIO_PIN_3
#define VREF_CH1_DR1_GPIO_Port GPIOA
#define UC_NCS_Pin GPIO_PIN_4
#define UC_NCS_GPIO_Port GPIOA
#define SENSOR_SCLK_Pin GPIO_PIN_5
#define SENSOR_SCLK_GPIO_Port GPIOA
#define SENSOR_MISO_Pin GPIO_PIN_6
#define SENSOR_MISO_GPIO_Port GPIOA
#define SENSOR_MOSI_Pin GPIO_PIN_7
#define SENSOR_MOSI_GPIO_Port GPIOA
#define VSNS_3V3_Pin GPIO_PIN_5
#define VSNS_3V3_GPIO_Port GPIOC
#define VREF_CH2_DR2_Pin GPIO_PIN_0
#define VREF_CH2_DR2_GPIO_Port GPIOB
#define VREF_CH1_DR2_Pin GPIO_PIN_1
#define VREF_CH1_DR2_GPIO_Port GPIOB
#define UC_MOSI_Pin GPIO_PIN_2
#define UC_MOSI_GPIO_Port GPIOB
#define STAGE2_EN_Pin GPIO_PIN_10
#define STAGE2_EN_GPIO_Port GPIOB
#define FLT_C1_Pin GPIO_PIN_12
#define FLT_C1_GPIO_Port GPIOB
#define FLT_D1_Pin GPIO_PIN_6
#define FLT_D1_GPIO_Port GPIOC
#define FLT_A1_Pin GPIO_PIN_7
#define FLT_A1_GPIO_Port GPIOC
#define FLT_B1_Pin GPIO_PIN_8
#define FLT_B1_GPIO_Port GPIOC
#define STAGE1_EN_Pin GPIO_PIN_9
#define STAGE1_EN_GPIO_Port GPIOC
#define LATCH_A_Pin GPIO_PIN_8
#define LATCH_A_GPIO_Port GPIOA
#define LATCH_B_Pin GPIO_PIN_9
#define LATCH_B_GPIO_Port GPIOA
#define LATCH_C_Pin GPIO_PIN_10
#define LATCH_C_GPIO_Port GPIOA
#define LATCH_D_Pin GPIO_PIN_11
#define LATCH_D_GPIO_Port GPIOA
#define LATCH_E_Pin GPIO_PIN_12
#define LATCH_E_GPIO_Port GPIOA
#define UC_SCLK_Pin GPIO_PIN_10
#define UC_SCLK_GPIO_Port GPIOC
#define UC_MISO_Pin GPIO_PIN_11
#define UC_MISO_GPIO_Port GPIOC
#define FLT_E1_Pin GPIO_PIN_12
#define FLT_E1_GPIO_Port GPIOC
#define VRBF_Pin GPIO_PIN_5
#define VRBF_GPIO_Port GPIOB
#define IMU_INT_Pin GPIO_PIN_6
#define IMU_INT_GPIO_Port GPIOB
#define SEL_2_Pin GPIO_PIN_7
#define SEL_2_GPIO_Port GPIOB
#define SEL_1_Pin GPIO_PIN_8
#define SEL_1_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
