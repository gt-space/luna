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

typedef union {
	uint8_t reg;
	struct {
		uint8_t reserved : 1;
		uint8_t lpf2_xl_en : 1;
		uint8_t fs_xl : 2;
		uint8_t odr_xl : 4;
	} CONFIG_SPLIT;
} IMU_ACCEL_CONFIG;


typedef union {
	uint8_t reg;
	struct {
		uint8_t FS_4000 : 1;
		uint8_t FS_125 : 1;
		uint8_t FS_G : 2;
		uint8_t ODR_G : 4;
	} CONFIG_SPLIT;
} IMU_GYRO_CONFIG;

typedef union {
	uint8_t reg;
	struct {
		uint8_t SW_RESET : 1;
		uint8_t RESERVED : 1;
		uint8_t IF_INC : 1;
		uint8_t SIM : 1;
		uint8_t PP_OD : 1;
		uint8_t H_LACTIVE : 1;
		uint8_t BDU : 1;
		uint8_t BOOT : 1;
	} CONFIG_SPLIT;

} GENERAL_CONFIG;

typedef enum {
	OUTX_L_G = 0x22,
	OUTX_H_G = 0x23,
	OUTY_L_G = 0x24,
	OUTY_H_G = 0x25,
	OUTZ_L_G = 0x26,
	OUTZ_H_G = 0x27
} GYRO_REGISTERS;

typedef enum {
	OUTX_L_A = 0x28,
	OUTX_H_A = 0x29,
	OUTY_L_A = 0x2A,
	OUTY_H_A = 0x2B,
	OUTZ_L_A = 0x2C,
	OUTZ_H_A = 0x2D
} ACCEL_REGISTERS;

typedef enum {
    IMU_ODR_OFF = 0,
    IMU_ODR_12_5HZ = 1,
    IMU_ODR_26HZ = 2,
    IMU_ODR_52HZ = 3,
    IMU_ODR_104HZ = 4,
    IMU_ODR_208HZ = 5,
    IMU_ODR_416HZ = 6,
    IMU_ODR_833HZ = 7
} IMU_ODR;

typedef enum {
    IMU_ACCEL_2G = 0,
    IMU_ACCEL_16G = 1,
    IMU_ACCEL_4G = 2,
    IMU_ACCEL_8G = 3
} IMU_ACCEL_RANGE;

typedef enum {
    IMU_GYRO_250DPS = 0,
    IMU_GYRO_500DPS = 1,
    IMU_GYRO_1000DPS = 2,
    IMU_GYRO_2000DPS = 3
} IMU_GYRO_RANGE;

typedef union {
    int16_t value;

    struct {
        uint8_t low : 8;
        uint8_t high : 8;
    } bytes;
} IMU_RawData;


typedef struct{
	int16_t gyro_x;
	int16_t gyro_y;
	int16_t gyro_z;

	int16_t accel_x;
	int16_t accel_y;
	int16_t accel_z;
} IMU_FullData;

typedef struct {
    float accel_x;
    float accel_y;
    float accel_z;

    float gyro_x;
    float gyro_y;
    float gyro_z;
} IMU_ScaledData;


/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define IMU_WHO_AM_I 0x0F
#define IMU_CTRL1_XL  0x10
#define IMU_CTRL2_G   0x11
#define IMU_CTRL3_C   0x12

#define ACCEL_SENSITIVITY 0.000122f
#define GYRO_SENSITIVITY  0.0175f
#define GRAVITY           9.80665f

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc;

SPI_HandleTypeDef hspi1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */

extern void initialise_monitor_handles(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
HAL_StatusTypeDef IMU_ReadRegister(uint8_t reg, uint8_t *value) {
    if (value == NULL) {
        return HAL_ERROR;
    }

    uint8_t tx[2] = {reg | 0x80, 0x00};
    uint8_t rx[2] = {0};

    HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET);

    HAL_StatusTypeDef status = HAL_SPI_TransmitReceive(
        &hspi1,
        tx,
        rx,
        2,
        100
    );

    HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET);

    if (status == HAL_OK) {
        *value = rx[1];
    }

    return status;
}

HAL_StatusTypeDef IMU_WriteRegister(uint8_t reg, uint8_t value) {
    uint8_t tx[2] = {reg & 0x7F, value};

    HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET);

    HAL_StatusTypeDef status = HAL_SPI_Transmit(
        &hspi1,
        tx,
        2,
        100
    );

    HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET);

    return status;
}

HAL_StatusTypeDef IMU_INIT(void) {
	IMU_ACCEL_CONFIG accel = {0};
	IMU_GYRO_CONFIG gyro = {0};
	GENERAL_CONFIG general = {0};

	accel.CONFIG_SPLIT.odr_xl = IMU_ODR_208HZ;
	accel.CONFIG_SPLIT.fs_xl = IMU_ACCEL_4G;

	gyro.CONFIG_SPLIT.ODR_G = IMU_ODR_208HZ;
	gyro.CONFIG_SPLIT.FS_G = IMU_GYRO_500DPS;

	general.CONFIG_SPLIT.BDU = 1;
	general.CONFIG_SPLIT.IF_INC = 1;

	if (accel.reg != 0x58 ||
	    gyro.reg != 0x54 ||
	    general.reg != 0x44) {
	    return HAL_ERROR;
	}

	HAL_StatusTypeDef status;

	status = IMU_WriteRegister(IMU_CTRL3_C, general.reg);

	if (status != HAL_OK) {
		return status;
	}

	status = IMU_WriteRegister(IMU_CTRL1_XL, accel.reg);

	if (status != HAL_OK) {
		return status;
	}

	status = IMU_WriteRegister(IMU_CTRL2_G, gyro.reg);

	return status;

}

HAL_StatusTypeDef IMU_ReadBurst(uint8_t startReg, uint8_t *buffer, uint16_t length) {
    if (buffer == NULL || length == 0 || length > 12) {
        return HAL_ERROR;
    }

    uint8_t tx[13] = {0};
    uint8_t rx[13] = {0};

    tx[0] = startReg | 0x80;

    //select IMU
    HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET);

    HAL_StatusTypeDef status = HAL_SPI_TransmitReceive(
        &hspi1,
        tx,
        rx,
        length + 1,
        100
    );

    // deselect IMU
    HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET);

    if (status != HAL_OK) {
        return status;
    }

    for (uint16_t i = 0; i < length; i++) {
        buffer[i] = rx[i + 1];
    }

    return HAL_OK;
}


HAL_StatusTypeDef IMU_ReadData(IMU_FullData *sensor) {
	if (sensor == NULL) {
		return HAL_ERROR;
	}

	uint8_t raw[12] = {0};
	HAL_StatusTypeDef status = IMU_ReadBurst(OUTX_L_G, raw, 12);

	if (status != HAL_OK) {
		return status;
	}

	IMU_RawData data = {0};

    // Gyroscope X
    data.bytes.low = raw[0];
    data.bytes.high = raw[1];
    sensor->gyro_x = data.value;

    // Gyroscope Y
    data.bytes.low = raw[2];
    data.bytes.high = raw[3];
    sensor->gyro_y = data.value;

    // Gyroscope Z
    data.bytes.low = raw[4];
    data.bytes.high = raw[5];
    sensor->gyro_z = data.value;

    // Accelerometer X
    data.bytes.low = raw[6];
    data.bytes.high = raw[7];
    sensor->accel_x = data.value;

    // Accelerometer Y
    data.bytes.low = raw[8];
    data.bytes.high = raw[9];
    sensor->accel_y = data.value;

    // Accelerometer Z
    data.bytes.low = raw[10];
    data.bytes.high = raw[11];
    sensor->accel_z = data.value;

    return HAL_OK;
}

void IMU_ConvertData(const IMU_FullData *raw, IMU_ScaledData *scaled) {
	scaled->accel_x = raw->accel_x * ACCEL_SENSITIVITY * GRAVITY;
	scaled->accel_y = raw->accel_y * ACCEL_SENSITIVITY * GRAVITY;
	scaled->accel_z = raw->accel_z * ACCEL_SENSITIVITY * GRAVITY;

	scaled->gyro_x = raw->gyro_x * GYRO_SENSITIVITY;
	scaled->gyro_y = raw->gyro_y * GYRO_SENSITIVITY;
	scaled->gyro_z = raw->gyro_z * GYRO_SENSITIVITY;
}



/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
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


  printf("Semihosting! \r\n");
  fflush(stdout);


  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */

  uint8_t id = 0;
  IMU_FullData raw = {0};
  IMU_ScaledData scaled = {0};
  uint8_t imu_ready = 0;
  HAL_StatusTypeDef status = IMU_ReadRegister(IMU_WHO_AM_I, &id);

  if (status == HAL_OK) {

      printf("who am i result: 0x%02X\r\n", id);

      if (id == 0x6B) {
          printf("IMU detected successfully!\r\n");

          status = IMU_INIT();


          if (status == HAL_OK) {
        	  printf("IMU initialized \n");
        	  imu_ready = 1;
          } else {
        	  printf("IMU failed to initialize. \n");
          }

      } else {
          printf("Unexpected IMU ID!\r\n");
      }

  } else {
      printf("SPI communication failed!\r\n");
  }

  fflush(stdout);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  uint32_t lastRead = HAL_GetTick();
  uint32_t cycleCount = 0;
  while (1)
  {
	  uint32_t currentTime = HAL_GetTick();

	  if (imu_ready && (uint32_t) (currentTime - lastRead) >= 5) {
		  lastRead += 5;

		  status = IMU_ReadData(&raw);

		  if (status == HAL_OK) {
			  IMU_ConvertData(&raw, &scaled);

			  if (cycleCount % 100 == 0) {
				  printf("Gyroscope Data: X = %.3f, Y = %.3f, Z = %.3f", scaled.gyro_x, scaled.gyro_y, scaled.gyro_z);
				  printf("Acceleration Data: X = %.3f, Y = %.3f, Z = %.3f", scaled.accel_x, scaled.accel_y, scaled.accel_z);
				  printf("-------------------------------------------------------------------------------------------------");
			  }



		  }
		  cycleCount++;
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSI14;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSI14State = RCC_HSI14_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.HSI14CalibrationValue = 16;
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
  * @brief ADC Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC_Init(void)
{

  /* USER CODE BEGIN ADC_Init 0 */

  /* USER CODE END ADC_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC_Init 1 */

  /* USER CODE END ADC_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc.Instance = ADC1;
  hadc.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc.Init.Resolution = ADC_RESOLUTION_12B;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
  hadc.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc.Init.LowPowerAutoWait = DISABLE;
  hadc.Init.LowPowerAutoPowerOff = DISABLE;
  hadc.Init.ContinuousConvMode = DISABLE;
  hadc.Init.DiscontinuousConvMode = DISABLE;
  hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc.Init.DMAContinuousRequests = DISABLE;
  hadc.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  if (HAL_ADC_Init(&hadc) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_2;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC_Init 2 */

  /* USER CODE END ADC_Init 2 */

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
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LED1_Pin|LED2_Pin|LED3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : CS_Pin */
  GPIO_InitStruct.Pin = CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LED1_Pin LED2_Pin LED3_Pin */
  GPIO_InitStruct.Pin = LED1_Pin|LED2_Pin|LED3_Pin;
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
