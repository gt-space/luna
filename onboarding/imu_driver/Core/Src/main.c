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
#include <stdio.h>  /* printf, routed to the debugger by semihosting */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* One raw IMU reading: signed counts straight from the output registers.
 * Accel x 0.244 = mg (+/-8 g); gyro x 70 = mdps (2000 dps). */
typedef struct
{
  int16_t accel_x;
  int16_t accel_y;
  int16_t accel_z;
  int16_t gyro_x;
  int16_t gyro_y;
  int16_t gyro_z;
} imu_raw_sample_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* LSM6DSM registers */
#define LSM6DSM_REG_WHO_AM_I   0x0F
#define LSM6DSM_WHO_AM_I_VALUE 0x6A  /* LSM6DSM; the Notion page's 0x6B is the LSM6DSR */
#define LSM6DSM_SPI_READ       0x80  /* top bit of the address byte set = read */
#define IMU_SPI_TIMEOUT_MS     10
#define LSM6DSM_REG_CTRL1_XL   0x10  /* accel ODR + full scale   (datasheet 10.13) */
#define LSM6DSM_REG_CTRL2_G    0x11  /* gyro ODR + full scale    (datasheet 10.14) */
#define LSM6DSM_REG_CTRL3_C    0x12  /* BDU, auto-increment     (datasheet 10.15) */
/* Config values written at startup (0101 = 208 Hz in both ODR tables) */
#define LSM6DSM_CTRL1_XL_208HZ_8G     0x5C  /* 0101 11 00: ODR_XL = 208 Hz, FS_XL = +/-8 g   (Tables 52-53) */
#define LSM6DSM_CTRL2_G_208HZ_2000DPS 0x5C  /* 0101 11 00: ODR_G = 208 Hz, FS_G = 2000 dps   (Tables 55-56) */
#define LSM6DSM_CTRL3_C_BDU_IF_INC    0x44  /* 0100 0100: BDU = 1 (bit 6), IF_INC = 1 (bit 2) (Table 58) */

/* Output register addresses (first byte of each 6-byte block) */
#define LSM6DSM_REG_OUTX_L_G    0x22  /* gyro X low byte; 6 bytes X/Y/Z low,high to 0x27  (datasheet 10.29) */
#define LSM6DSM_REG_OUTX_L_XL   0x28  /* accel X low byte; 6 bytes X/Y/Z low,high to 0x2D (datasheet 10.35) */

/* Sensitivities for the ranges above (datasheet 4.1, Table 3, p23). Integers to avoid floats on the M0. */
#define LSM6DSM_ACCEL_UG_PER_LSB   244  /* +/-8 g: 0.244 mg/LSB = 244 ug/LSB */
#define LSM6DSM_GYRO_MDPS_PER_LSB  70   /* 2000 dps: 70 mdps/LSB */

/* Checkpoint 3 loop timing */
#define IMU_LOOP_PERIOD_MS   5     /* 5 ms = 200 Hz (T = 1/f) */
#define REPORT_PERIOD_MS     1000  /* print a status line once per second */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

/* USER CODE BEGIN PV */
/* Last WHO_AM_I value read. volatile so the debugger always shows the real value. */
volatile uint8_t imu_who_am_i = 0;

/* Loop timing stats, also readable in the debugger's Watch panel. */
volatile uint32_t loop_rate_hz  = 0;  /* passes completed in the last second (target 200) */
volatile uint32_t loop_overruns = 0;  /* passes whose work ran past the next 5 ms deadline */
volatile uint32_t imu_read_failures = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */
HAL_StatusTypeDef imu_read_register(uint8_t reg, uint8_t *value);
HAL_StatusTypeDef imu_write_register(uint8_t reg, uint8_t value);
extern void initialise_monitor_handles(void);  /* provided by rdimon */

HAL_StatusTypeDef imu_write_register_verified(uint8_t reg, uint8_t value);
HAL_StatusTypeDef imu_configure(void);
HAL_StatusTypeDef imu_read_sample(imu_raw_sample_t *sample);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* Read one IMU register over SPI.
 * Byte 1 out: register address with the read bit set. Byte 2 out: dummy.
 * The IMU answers during byte 2, so the value lands in rx[1]. */
HAL_StatusTypeDef imu_read_register(uint8_t reg, uint8_t *value)
{
  uint8_t tx[2] = { (uint8_t)(reg | LSM6DSM_SPI_READ), 0x00 };
  uint8_t rx[2] = { 0 };

  HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_RESET);  /* select IMU */
  HAL_StatusTypeDef status =
      HAL_SPI_TransmitReceive(&hspi1, tx, rx, 2, IMU_SPI_TIMEOUT_MS);
  HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_SET);    /* release */

  *value = rx[1];
  return status;
}

/* Write one IMU register over SPI: address with the read bit clear, then value.  */
HAL_StatusTypeDef imu_write_register(uint8_t reg, uint8_t value)
{
  uint8_t tx[2] = { (uint8_t)(reg & (uint8_t)~LSM6DSM_SPI_READ), value };

  HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_RESET);
  HAL_StatusTypeDef status =
      HAL_SPI_Transmit(&hspi1, tx, 2, IMU_SPI_TIMEOUT_MS);
  HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_SET);

  return status;
}

/* Write a register, then read it back to confirm the IMU actually took the value.
 * HAL_OK from the write alone only means the SPI transfer ran, not that anyone heard it.
 * Returns HAL_ERROR if the read-back doesn't match. */
HAL_StatusTypeDef imu_write_register_verified(uint8_t reg, uint8_t value)
{
  HAL_StatusTypeDef status = imu_write_register(reg, value);
  if (status != HAL_OK) return status;

  uint8_t readback = 0;
  status = imu_read_register(reg, &readback);
  if (status != HAL_OK) return status;

  if (readback != value)
  {
    printf("IMU reg 0x%02X: wrote 0x%02X, read back 0x%02X\r\n", reg, value, readback);
    return HAL_ERROR;
  }
  return HAL_OK;
}

HAL_StatusTypeDef imu_configure(void)
{
  HAL_StatusTypeDef status;

  /* Stop at the first failure so the returned status is the real error. */

  /* CTRL3_C: BDU = 1 (bit 6), IF_INC = 1 (bit 2) */
  status = imu_write_register_verified(LSM6DSM_REG_CTRL3_C, LSM6DSM_CTRL3_C_BDU_IF_INC);
  if (status != HAL_OK) return status;

  /* CTRL1_XL: ODR_XL = 208 Hz, FS_XL = +/-8 g */
  status = imu_write_register_verified(LSM6DSM_REG_CTRL1_XL, LSM6DSM_CTRL1_XL_208HZ_8G);
  if (status != HAL_OK) return status;

  /* CTRL2_G: ODR_G = 208 Hz, FS_G = 2000 dps */
  status = imu_write_register_verified(LSM6DSM_REG_CTRL2_G, LSM6DSM_CTRL2_G_208HZ_2000DPS);
  if (status != HAL_OK) return status;

  return HAL_OK;
}

/* Read all six axes into *sample.
 * The 12 output registers are consecutive: gyro 0x22-0x27, then accel 0x28-0x2D,
 * each axis stored low byte first. This reads them one register at a time;
 * checkpoint 4 replaces the loop with a single burst read into the same raw[] layout. */
HAL_StatusTypeDef imu_read_sample(imu_raw_sample_t *sample)
{
  uint8_t raw[12];

  for (uint8_t i = 0; i < 12; i++)
  {
    HAL_StatusTypeDef status = imu_read_register(LSM6DSM_REG_OUTX_L_G + i, &raw[i]);
    if (status != HAL_OK) return status;
  }

  /* Combine low/high byte pairs into signed 16-bit counts: value = (high << 8) | low */
  sample->gyro_x  = (int16_t)(((uint16_t)raw[1]  << 8) | raw[0]);
  sample->gyro_y  = (int16_t)(((uint16_t)raw[3]  << 8) | raw[2]);
  sample->gyro_z  = (int16_t)(((uint16_t)raw[5]  << 8) | raw[4]);
  sample->accel_x = (int16_t)(((uint16_t)raw[7]  << 8) | raw[6]);
  sample->accel_y = (int16_t)(((uint16_t)raw[9]  << 8) | raw[8]);
  sample->accel_z = (int16_t)(((uint16_t)raw[11] << 8) | raw[10]);

  return HAL_OK;
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
  /* Semihosting: connect stdout to the debugger console.
     Only works with the debugger attached; without it the first printf halts the chip. */
  initialise_monitor_handles();
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */
  printf("imu_driver started\r\n");
  fflush(stdout);

  /* Checkpoint 2: turn on accel + gyro (208 Hz, +/-8 g, 2000 dps).
     D10 on = configuration failed (see the Debug Console for which register). */
  HAL_StatusTypeDef cfg_status = imu_configure();
  if (cfg_status == HAL_OK)
  {
    printf("IMU configured OK\r\n");
  }
  else
  {
    printf("IMU configure FAILED, status = %d\r\n", cfg_status);
    HAL_GPIO_WritePin(LED_D10_GPIO_Port, LED_D10_Pin, GPIO_PIN_SET);
  }

  /* Checkpoint 3 scheduler state.
     next_tick   = when the next pass is supposed to start (ms, from HAL_GetTick)
     report_tick = when the current 1-second reporting window started */
  uint32_t next_tick   = HAL_GetTick();
  uint32_t report_tick = next_tick;
  uint32_t passes      = 0;
  int32_t accel_mg[3]   = { 0 };  /* latest scaled sample, printed once per second */
  int32_t gyro_mdps[3]  = { 0 };
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    /* Checkpoint 3: wait until this pass's scheduled start time.
       (int32_t)(now - next_tick) < 0 means "next_tick is still in the future".
       The subtraction keeps working when the 32-bit ms counter wraps after ~49 days. */
    while ((int32_t)(HAL_GetTick() - next_tick) < 0)
    {
    }
    next_tick += IMU_LOOP_PERIOD_MS;  /* schedule from the planned time, not "now", so error can't accumulate */
    passes++;

    /* Checkpoint 1: read WHO_AM_I every pass.
       D9 on  = IMU answered 0x6A (SPI works).
       D11 on = SPI error or wrong value (check SW1 is on the STM32 side). */
    uint8_t who_am_i = 0;
    HAL_StatusTypeDef status = imu_read_register(LSM6DSM_REG_WHO_AM_I, &who_am_i);
    imu_who_am_i = who_am_i;

    if (status == HAL_OK && who_am_i == LSM6DSM_WHO_AM_I_VALUE)
    {
      HAL_GPIO_WritePin(LED_D9_GPIO_Port, LED_D9_Pin, GPIO_PIN_SET);
      HAL_GPIO_WritePin(LED_D11_GPIO_Port, LED_D11_Pin, GPIO_PIN_RESET);
    }
    else
    {
      HAL_GPIO_WritePin(LED_D9_GPIO_Port, LED_D9_Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(LED_D11_GPIO_Port, LED_D11_Pin, GPIO_PIN_SET);
    }

    imu_raw_sample_t sample;
    status = imu_read_sample(&sample);
    if (status == HAL_OK) 
    {
      /* sample is still raw counts; convert to physical units.
         Multiply in 32 bits: raw * 244 overflows int16_t. */
      accel_mg[0]  = (int32_t)sample.accel_x * LSM6DSM_ACCEL_UG_PER_LSB / 1000;
      accel_mg[1]  = (int32_t)sample.accel_y * LSM6DSM_ACCEL_UG_PER_LSB / 1000;
      accel_mg[2]  = (int32_t)sample.accel_z * LSM6DSM_ACCEL_UG_PER_LSB / 1000;
      gyro_mdps[0] = (int32_t)sample.gyro_x  * LSM6DSM_GYRO_MDPS_PER_LSB;
      gyro_mdps[1] = (int32_t)sample.gyro_y  * LSM6DSM_GYRO_MDPS_PER_LSB;
      gyro_mdps[2] = (int32_t)sample.gyro_z  * LSM6DSM_GYRO_MDPS_PER_LSB;
    }
    else
    {
      /* No printf here: at 200 Hz a persistent failure would flood the console.
         Counted instead and shown in the once-per-second report. */
      imu_read_failures++;
      HAL_GPIO_WritePin(LED_D9_GPIO_Port, LED_D9_Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(LED_D10_GPIO_Port, LED_D10_Pin, GPIO_PIN_SET);
    }

    /* Overrun check: if the work above ran past the next deadline, this pass was too slow.
       Count it and restart the schedule from now instead of bursting to catch up. */
    if ((int32_t)(HAL_GetTick() - next_tick) > 0)
    {
      loop_overruns++;
      next_tick = HAL_GetTick();
    }

    /* Once per second: report the loop rate and the latest sample.
       A semihosting printf takes several ms, longer than one 5 ms period, so it only
       happens here. Afterwards the schedule restarts from now so the slow print
       isn't counted as an overrun. */
    if ((uint32_t)(HAL_GetTick() - report_tick) >= REPORT_PERIOD_MS)
    {
      loop_rate_hz = passes;
      passes = 0;
      report_tick += REPORT_PERIOD_MS;

      /* Still and flat: one accel axis ~ +/-1000 mg, the others ~0; gyro near 0. */
      printf("rate=%lu Hz overruns=%lu read_fail=%lu WHO_AM_I=0x%02X | "
             "Accel [mg] x=%ld y=%ld z=%ld | Gyro [mdps] x=%ld y=%ld z=%ld\r\n",
             (unsigned long)loop_rate_hz, (unsigned long)loop_overruns,
             (unsigned long)imu_read_failures, imu_who_am_i,
             (long)accel_mg[0], (long)accel_mg[1], (long)accel_mg[2],
             (long)gyro_mdps[0], (long)gyro_mdps[1], (long)gyro_mdps[2]);

      next_tick = HAL_GetTick() + IMU_LOOP_PERIOD_MS;
    }
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
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
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
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
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
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LED_D10_Pin|LED_D9_Pin|LED_D11_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : IMU_CS_Pin */
  GPIO_InitStruct.Pin = IMU_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(IMU_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_D10_Pin LED_D9_Pin LED_D11_Pin */
  GPIO_InitStruct.Pin = LED_D10_Pin|LED_D9_Pin|LED_D11_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

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
