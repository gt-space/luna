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
#define MAG_NCS_Pin GPIO_PIN_0
#define MAG_NCS_GPIO_Port GPIOC
#define BAR_NCS_Pin GPIO_PIN_1
#define BAR_NCS_GPIO_Port GPIOC
#define IMU_NCS_Pin GPIO_PIN_2
#define IMU_NCS_GPIO_Port GPIOC
#define uC_NCS_Pin GPIO_PIN_1
#define uC_NCS_GPIO_Port GPIOA
#define MAG_INT_Pin GPIO_PIN_3
#define MAG_INT_GPIO_Port GPIOA
#define SENSOR_SCLK_Pin GPIO_PIN_5
#define SENSOR_SCLK_GPIO_Port GPIOA
#define SENSOR_MISO_Pin GPIO_PIN_6
#define SENSOR_MISO_GPIO_Port GPIOA
#define SENSOR_MOSI_Pin GPIO_PIN_7
#define SENSOR_MOSI_GPIO_Port GPIOA
#define MAG_DRDY_Pin GPIO_PIN_5
#define MAG_DRDY_GPIO_Port GPIOC

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
