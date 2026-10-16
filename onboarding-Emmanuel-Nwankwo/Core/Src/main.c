/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */
extern void initialise_monitor_handles(void);
HAL_StatusTypeDef imu_read_register(uint8_t register_address, uint8_t *received_loc, uint32_t timeout);
HAL_StatusTypeDef imu_write_register(uint8_t register_address, uint8_t write_data, uint32_t timeout);
HAL_StatusTypeDef imu_read_int16(uint8_t low_register_address, int16_t *raw_var, uint32_t timeout);
void print_measurement(const char *name, double value, const char *unit, uint8_t dp);
typedef struct {
    uint8_t low_x_address;
    uint8_t low_y_address;
    uint8_t low_z_address;
    double x;
    double y;
    double z;
    double sens;
} Accelerometer;
typedef struct {
    uint8_t low_x_address;
    uint8_t low_y_address;
    uint8_t low_z_address;
    double x;
    double y;
    double z;
    double sens;
} Gyroscope;
HAL_StatusTypeDef imu_read_xyz(uint8_t x_low_address, int16_t *x_var, int16_t *y_var, int16_t *z_var, uint32_t timeout);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  initialise_monitor_handles();
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */
  printf("Semihosting initialized successfully!\r\n");

  uint32_t timeout = 100;
  uint8_t WHO_AM_I;
  uint8_t WHO_AM_I_address = 0x0F;

  HAL_StatusTypeDef read_status = imu_read_register(WHO_AM_I_address, &WHO_AM_I, timeout);

  if (read_status == HAL_OK) {
    printf("WHO_AM_I: 0x%02X\r\n", WHO_AM_I);
  }
  else {
    printf("Unable to read IMU register, status: %d\r\n", read_status);
  }

  uint8_t XL_address = 0x10;
  uint8_t XL_val = 0x50;
  uint8_t G_address = 0x11;
  uint8_t G_val = 0x50;
  uint8_t Ctrl_address = 0x12;
  uint8_t Ctrl_val = 0x44;

  HAL_StatusTypeDef XL_status = imu_write_register(XL_address, XL_val, timeout);
  HAL_StatusTypeDef G_status = imu_write_register(G_address, G_val, timeout);
  HAL_StatusTypeDef Ctrl_status = imu_write_register(Ctrl_address, Ctrl_val, timeout);

  printf("Accelerometer write status: %d\r\n", XL_status);
  printf("Gyroscope write status: %d\r\n", G_status);
  printf("Control register write status: %d\r\n", Ctrl_status);

  uint8_t XL_val_test;
  uint8_t G_val_test;
  uint8_t Ctrl_val_test;

  HAL_StatusTypeDef XL_read_status = imu_read_register(XL_address, &XL_val_test, timeout);
  HAL_StatusTypeDef G_read_status = imu_read_register(G_address, &G_val_test, timeout);
  HAL_StatusTypeDef Ctrl_read_status = imu_read_register(Ctrl_address, &Ctrl_val_test, timeout);

  printf("Accelerometer read status: %d\r\n", XL_read_status);
  printf("Gyroscope read status: %d\r\n", G_read_status);
  printf("Control register read status: %d\r\n", Ctrl_read_status);

  printf("\r\n");

  if (XL_read_status == HAL_OK && XL_val_test == XL_val) {
    printf("CTRL1_XL: 0x%02X\r\n", XL_val_test);
  }
  if (G_read_status == HAL_OK && G_val_test == G_val) {
    printf("CTRL2_G: 0x%02X\r\n", G_val_test);
  }
  if (Ctrl_read_status == HAL_OK && Ctrl_val_test == Ctrl_val) {
    printf("CTRL3_C: 0x%02X\r\n", Ctrl_val_test);
  }

  Accelerometer acc = { .low_x_address=0x28, .low_y_address=0x2A, .low_z_address=0x2C, .sens=0.061};
  Gyroscope gy = { .low_x_address=0x22, .low_y_address=0x24, .low_z_address=0x26,.sens=8.75};

  int16_t XL_x_raw;
  int16_t XL_y_raw;
  int16_t XL_z_raw;
  int16_t G_x_raw;
  int16_t G_y_raw;
  int16_t G_z_raw;

  uint32_t last_sample_time = HAL_GetTick();
  uint32_t last_print_time = HAL_GetTick();

  const uint32_t sample_period_ms = 5;
  const uint32_t print_period_ms = 500;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {
    uint32_t current_time = HAL_GetTick();

    if (current_time - last_sample_time >= sample_period_ms) {
      last_sample_time += sample_period_ms;

      HAL_StatusTypeDef acc_read_status = imu_read_xyz(acc.low_x_address, &XL_x_raw, &XL_y_raw, &XL_z_raw, timeout);
      HAL_StatusTypeDef gy_read_status = imu_read_xyz(gy.low_x_address, &G_x_raw, &G_y_raw, &G_z_raw, timeout);

      if (acc_read_status == HAL_OK) {
        acc.x = (double) (XL_x_raw * acc.sens) / 1000;
        acc.y = (double) (XL_y_raw * acc.sens) / 1000;
        acc.z = (double) (XL_z_raw * acc.sens) / 1000;
      }

      if (gy_read_status == HAL_OK) {
        gy.x = (double) (G_x_raw * gy.sens) / 1000;
        gy.y = (double) (G_y_raw * gy.sens) / 1000;
        gy.z = (double) (G_z_raw * gy.sens) / 1000;
      }
    }

    if (current_time - last_print_time >= print_period_ms) {
      last_print_time += print_period_ms;

      print_measurement("Accelerometer X", acc.x, "g", 3);
      print_measurement("Accelerometer Y", acc.y, "g", 3);
      print_measurement("Accelerometer Z", acc.z, "g", 3);

      print_measurement("Gyroscope X", gy.x, "dps", 3);
      print_measurement("Gyroscope Y", gy.y, "dps", 3);
      print_measurement("Gyroscope Z", gy.z, "dps", 3);

      printf("\r\n");
    }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(IMU_nCS_GPIO_Port, IMU_nCS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : IMU_nCS_Pin */
  GPIO_InitStruct.Pin = IMU_nCS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(IMU_nCS_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
HAL_StatusTypeDef imu_read_register(uint8_t register_address, uint8_t *received_loc, uint32_t timeout) {
  uint8_t read_command = register_address | 0x80;
  uint8_t tx_buffer[2] = {read_command, 0x00};
  uint8_t rx_buffer[2] = {0x00, 0x00};

  HAL_GPIO_WritePin(IMU_nCS_GPIO_Port, IMU_nCS_Pin, GPIO_PIN_RESET);
  HAL_StatusTypeDef status = HAL_SPI_TransmitReceive(&hspi1, &tx_buffer[0], &rx_buffer[0], 2, timeout);
  HAL_GPIO_WritePin(IMU_nCS_GPIO_Port, IMU_nCS_Pin, GPIO_PIN_SET);

  if (status == HAL_OK) {
    *received_loc = rx_buffer[1];
  }
  return status;
}

HAL_StatusTypeDef imu_write_register(uint8_t register_address, uint8_t write_data, uint32_t timeout) {
  uint8_t write_buffer[2];
  uint8_t write_command = register_address & 0x7F;

  write_buffer[0] = write_command;
  write_buffer[1] = write_data;

  HAL_GPIO_WritePin(IMU_nCS_GPIO_Port, IMU_nCS_Pin, GPIO_PIN_RESET);
  HAL_StatusTypeDef write_status = HAL_SPI_Transmit(&hspi1, write_buffer, 2, timeout);
  HAL_GPIO_WritePin(IMU_nCS_GPIO_Port, IMU_nCS_Pin, GPIO_PIN_SET);

  return write_status;
}

HAL_StatusTypeDef imu_read_int16(uint8_t low_register_address, int16_t *raw_var, uint32_t timeout) {
    uint8_t low_val;
    uint8_t high_val;
    uint8_t high_register_address = low_register_address + 0x01;

    HAL_StatusTypeDef low_read_status = imu_read_register(low_register_address, &low_val, timeout);
    if (low_read_status != HAL_OK) {
        return low_read_status;
    }

    HAL_StatusTypeDef high_read_status = imu_read_register(high_register_address, &high_val, timeout);
    if (high_read_status != HAL_OK) {
        return high_read_status;
    }

    *raw_var = (int16_t) ((high_val << 8) | low_val);
    return HAL_OK;
}

void print_measurement(const char *name, double value, const char *unit, uint8_t dp) {
  char sign = value < 0 ? '-' : '+';

  if (value < 0) {
    value = -value;
  }

  uint32_t scale = 1;
  for (uint8_t i = 0; i < dp; i++) {
    scale *= 10;
  }

  uint32_t scaled = (uint32_t)(value * scale);
  uint32_t whole_part = scaled / scale;
  uint32_t decimal_part = scaled % scale;

  printf("%s: %c%lu.%0*lu %s\r\n", name, sign, (unsigned long)whole_part, dp, (unsigned long)decimal_part, unit);
}

HAL_StatusTypeDef imu_read_xyz(uint8_t x_low_address, int16_t *x_var, int16_t *y_var, int16_t *z_var, uint32_t timeout) {
  uint8_t read_command = x_low_address | 0x80;
  uint8_t tx_buffer[7] = {read_command};
  uint8_t rx_buffer[7];

  HAL_GPIO_WritePin(IMU_nCS_GPIO_Port, IMU_nCS_Pin, GPIO_PIN_RESET);
  HAL_StatusTypeDef read_status = HAL_SPI_TransmitReceive(&hspi1, tx_buffer, rx_buffer, 7, timeout);
  HAL_GPIO_WritePin(IMU_nCS_GPIO_Port, IMU_nCS_Pin, GPIO_PIN_SET);

  if (read_status == HAL_OK) {
    *x_var = (int16_t) ((rx_buffer[2] << 8) | rx_buffer[1]);
    *y_var = (int16_t) ((rx_buffer[4] << 8) | rx_buffer[3]);
    *z_var = (int16_t) ((rx_buffer[6] << 8) | rx_buffer[5]);
  }

  return read_status;
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
