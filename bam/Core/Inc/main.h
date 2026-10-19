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
#define nCS_SD_Pin GPIO_PIN_0
#define nCS_SD_GPIO_Port GPIOH
#define nCS_MAG_C_Pin GPIO_PIN_1
#define nCS_MAG_C_GPIO_Port GPIOH
#define ISNS_STM32_FILT_Pin GPIO_PIN_0
#define ISNS_STM32_FILT_GPIO_Port GPIOC
#define ISNS_IMU_A_FILT_Pin GPIO_PIN_1
#define ISNS_IMU_A_FILT_GPIO_Port GPIOC
#define ISNS_BARO_A_FILT_Pin GPIO_PIN_2
#define ISNS_BARO_A_FILT_GPIO_Port GPIOC
#define ISNS_MAG_A_FILT_Pin GPIO_PIN_3
#define ISNS_MAG_A_FILT_GPIO_Port GPIOC
#define STATUS_LED_Pin GPIO_PIN_0
#define STATUS_LED_GPIO_Port GPIOA
#define SENSE_3V3_Pin GPIO_PIN_1
#define SENSE_3V3_GPIO_Port GPIOA
#define SENSE_5V_Pin GPIO_PIN_2
#define SENSE_5V_GPIO_Port GPIOA
#define SENSE_12V_Pin GPIO_PIN_3
#define SENSE_12V_GPIO_Port GPIOA
#define nCS_BARO_A_Pin GPIO_PIN_4
#define nCS_BARO_A_GPIO_Port GPIOA
#define SENSOR_A_CLK_Pin GPIO_PIN_5
#define SENSOR_A_CLK_GPIO_Port GPIOA
#define SENSOR_A_MISO_Pin GPIO_PIN_6
#define SENSOR_A_MISO_GPIO_Port GPIOA
#define SENSOR_A_MOSI_Pin GPIO_PIN_7
#define SENSOR_A_MOSI_GPIO_Port GPIOA
#define SENSOR_C_MOSI_Pin GPIO_PIN_2
#define SENSOR_C_MOSI_GPIO_Port GPIOB
#define nCS_BARO_B_Pin GPIO_PIN_12
#define nCS_BARO_B_GPIO_Port GPIOB
#define SENSOR_B_CLK_Pin GPIO_PIN_13
#define SENSOR_B_CLK_GPIO_Port GPIOB
#define SENSOR_B_MISO_Pin GPIO_PIN_14
#define SENSOR_B_MISO_GPIO_Port GPIOB
#define SENSOR_B_MOSI_Pin GPIO_PIN_15
#define SENSOR_B_MOSI_GPIO_Port GPIOB
#define EFUSE_PG_Pin GPIO_PIN_8
#define EFUSE_PG_GPIO_Port GPIOA
#define nCS_BARO_C_Pin GPIO_PIN_9
#define nCS_BARO_C_GPIO_Port GPIOA
#define nCS_IMU_C_Pin GPIO_PIN_10
#define nCS_IMU_C_GPIO_Port GPIOA
#define INT2_IMU_C_Pin GPIO_PIN_11
#define INT2_IMU_C_GPIO_Port GPIOA
#define INT1_IMU_C_Pin GPIO_PIN_12
#define INT1_IMU_C_GPIO_Port GPIOA
#define T_JTMS_Pin GPIO_PIN_13
#define T_JTMS_GPIO_Port GPIOA
#define T_JCLK_Pin GPIO_PIN_14
#define T_JCLK_GPIO_Port GPIOA
#define T_JTDI_Pin GPIO_PIN_15
#define T_JTDI_GPIO_Port GPIOA
#define SENSOR_C_CLK_Pin GPIO_PIN_10
#define SENSOR_C_CLK_GPIO_Port GPIOC
#define SENSOR_C_MISO_Pin GPIO_PIN_11
#define SENSOR_C_MISO_GPIO_Port GPIOC
#define UART_RX_Pin GPIO_PIN_2
#define UART_RX_GPIO_Port GPIOD
#define T_JTDO_Pin GPIO_PIN_3
#define T_JTDO_GPIO_Port GPIOB
#define UART_TX_Pin GPIO_PIN_6
#define UART_TX_GPIO_Port GPIOB
#define GPS_SDA_Pin GPIO_PIN_7
#define GPS_SDA_GPIO_Port GPIOB
#define GPS_SCL_Pin GPIO_PIN_8
#define GPS_SCL_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
