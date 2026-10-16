/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "stm32h7xx_hal.h"

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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define m1_en_Pin GPIO_PIN_3
#define m1_en_GPIO_Port GPIOD
#define swclk_Pin GPIO_PIN_14
#define swclk_GPIO_Port GPIOA
#define swo_Pin GPIO_PIN_3
#define swo_GPIO_Port GPIOB
#define swdio_Pin GPIO_PIN_13
#define swdio_GPIO_Port GPIOA
#define Actuator_data_dir_a2_Pin GPIO_PIN_9
#define Actuator_data_dir_a2_GPIO_Port GPIOB
#define Actuator_data_dir_a1_Pin GPIO_PIN_8
#define Actuator_data_dir_a1_GPIO_Port GPIOB
#define m2_en_Pin GPIO_PIN_2
#define m2_en_GPIO_Port GPIOD
#define m4_en_Pin GPIO_PIN_0
#define m4_en_GPIO_Port GPIOD
#define m3_en_Pin GPIO_PIN_1
#define m3_en_GPIO_Port GPIOD
#define Vsns_22V2_Pin GPIO_PIN_3
#define Vsns_22V2_GPIO_Port GPIOF
#define Vsns_12V0_Pin GPIO_PIN_5
#define Vsns_12V0_GPIO_Port GPIOF
#define Isns_22V2_Pin GPIO_PIN_4
#define Isns_22V2_GPIO_Port GPIOF
#define Isns_12V0_Pin GPIO_PIN_6
#define Isns_12V0_GPIO_Port GPIOF
#define Vsns_4V4_Pin GPIO_PIN_7
#define Vsns_4V4_GPIO_Port GPIOF
#define Isns_4V4_Pin GPIO_PIN_8
#define Isns_4V4_GPIO_Port GPIOF
#define Isns_3V3_Pin GPIO_PIN_10
#define Isns_3V3_GPIO_Port GPIOF
#define Vsns_3V3_Pin GPIO_PIN_9
#define Vsns_3V3_GPIO_Port GPIOF
#define Isns_M1_Pin GPIO_PIN_2
#define Isns_M1_GPIO_Port GPIOH
#define Eth_nRST_Pin GPIO_PIN_0
#define Eth_nRST_GPIO_Port GPIOJ
#define Isns_M2_Pin GPIO_PIN_3
#define Isns_M2_GPIO_Port GPIOH
#define Isns_M3_Pin GPIO_PIN_4
#define Isns_M3_GPIO_Port GPIOH
#define Isns_M4_Pin GPIO_PIN_5
#define Isns_M4_GPIO_Port GPIOH
#define Eth_LINK_ST_Pin GPIO_PIN_1
#define Eth_LINK_ST_GPIO_Port GPIOJ
#define Vsns_motor_Pin GPIO_PIN_2
#define Vsns_motor_GPIO_Port GPIOC
#define ESC_Ctrl_Pin GPIO_PIN_5
#define ESC_Ctrl_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
