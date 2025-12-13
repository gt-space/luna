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
#define SD_SPI_HANDLE hspi2
#define SD_CS_GPIO_Port SD_NCS_GPIO_Port
#define SD_CS_Pin SD_NCS_Pin
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
#define MAG_INT_Pin GPIO_PIN_3
#define MAG_INT_GPIO_Port GPIOA
#define UC_NCS_Pin GPIO_PIN_4
#define UC_NCS_GPIO_Port GPIOA
#define SENSOR_SCLK_Pin GPIO_PIN_5
#define SENSOR_SCLK_GPIO_Port GPIOA
#define SENSOR_MISO_Pin GPIO_PIN_6
#define SENSOR_MISO_GPIO_Port GPIOA
#define SENSOR_MOSI_Pin GPIO_PIN_7
#define SENSOR_MOSI_GPIO_Port GPIOA
#define MAG_DRDY_Pin GPIO_PIN_5
#define MAG_DRDY_GPIO_Port GPIOC
#define STAGE2_EN_Pin GPIO_PIN_0
#define STAGE2_EN_GPIO_Port GPIOB
#define VREF_FB1_E_Pin GPIO_PIN_1
#define VREF_FB1_E_GPIO_Port GPIOB
#define UC_MOSI_Pin GPIO_PIN_2
#define UC_MOSI_GPIO_Port GPIOB
#define VREF_FB2_D_Pin GPIO_PIN_10
#define VREF_FB2_D_GPIO_Port GPIOB
#define VREF_FB1_D_Pin GPIO_PIN_12
#define VREF_FB1_D_GPIO_Port GPIOB
#define VREF_FB2_E_Pin GPIO_PIN_13
#define VREF_FB2_E_GPIO_Port GPIOB
#define VREF_FB2_Pin GPIO_PIN_7
#define VREF_FB2_GPIO_Port GPIOC
#define VREF_FB1_Pin GPIO_PIN_8
#define VREF_FB1_GPIO_Port GPIOC
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

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
