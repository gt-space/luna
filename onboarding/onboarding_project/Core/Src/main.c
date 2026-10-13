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

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

typedef struct {
	float accel_x;
	float accel_y;
	float accel_z;

	float gyro_x;
	float gyro_y;
	float gyro_z;
} IMU_Data;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define WHO_AM_I_REG 0x0F
#define WHO_AM_I_VAL 0x6B

//config register addresses
#define CTRL1_XL_REG 0x10
#define CTRL2_G_REG 0x11
#define CTRL3_C_REG 0x12

//sensor output registers addresses
#define OUTX_L_G_REG 0x22
//#define OUTX_L_A_REG 0x28

//config values
#define CTRL1_CONFIG 0x68
#define GYRO_CONFIG 0x64
#define CTRL3_CONFIG 0x44

#define SAMPLE_FREQ 200

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

/* USER CODE BEGIN PV */
volatile uint8_t who_am_i;
IMU_Data reading;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */

uint8_t IMU_ReadRegister(uint8_t reg);
void IMU_WriteRegister(uint8_t reg, uint8_t value);

void IMU_Init(void);
void IMU_ReadData(IMU_Data *reading);
void IMU_ReadRegisters(uint8_t reg, uint8_t *data, uint16_t length);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

//read means set bit 7 to 1
uint8_t IMU_ReadRegister(uint8_t reg) {
    uint8_t tx[2]; //stores number between 0 to 255, int8_t stores -128 to 127
    uint8_t rx[2];

    // Set the read bit (bit 7)
    tx[0] = reg | 0x80; //convert each hexadecimal digit to 4 digit binary
    //we are storing a 8 bit numerical value

    tx[1] = 0x00;

    // Select IMU
    HAL_GPIO_WritePin(IMU_CS_GPIO_Port,
                      IMU_CS_Pin,
                      GPIO_PIN_RESET);

    // Send and receive 2 bytes
    HAL_SPI_TransmitReceive(&hspi1,
                            tx,
                            rx,
                            2,
                            HAL_MAX_DELAY); //tx[0] is the register to read, tx[1] is the value to write when used in write context
    //reading tx[0] to get the register value

    // Deselect IMU
    HAL_GPIO_WritePin(IMU_CS_GPIO_Port,
                      IMU_CS_Pin,
                      GPIO_PIN_SET);

    return rx[1];
}

//@param reg - register to write to
//@param value - value to write
void IMU_WriteRegister(uint8_t reg, uint8_t value) {
    uint8_t tx[2];

    // Clear bit 7 for a write
    tx[0] = reg & 0x7F;

    // Value we want to write
    tx[1] = value;

    // Select IMU
    HAL_GPIO_WritePin(IMU_CS_GPIO_Port,
                      IMU_CS_Pin,
                      GPIO_PIN_RESET); //reset = 0

    // Send register address + value
    HAL_SPI_Transmit(&hspi1,
                     tx,
                     2,
                     HAL_MAX_DELAY);

    // Deselect IMU
    HAL_GPIO_WritePin(IMU_CS_GPIO_Port,
                      IMU_CS_Pin,
                      GPIO_PIN_SET); //set = 1
}

void IMU_Init(void) {
	//enable BDU first
	IMU_WriteRegister(CTRL3_C_REG, CTRL3_CONFIG);
	IMU_WriteRegister(CTRL2_G_REG, GYRO_CONFIG);
	IMU_WriteRegister(CTRL1_XL_REG, CTRL1_CONFIG);
}

void IMU_ReadRegisters(uint8_t reg, uint8_t *data, uint16_t length) {
	uint8_t read_command = reg | 0x80;

	HAL_GPIO_WritePin (
		IMU_CS_GPIO_Port,
		IMU_CS_Pin,
		GPIO_PIN_RESET
	);

	HAL_SPI_Transmit(
		&hspi1,
		&read_command,
		1,
		HAL_MAX_DELAY
	);

	HAL_SPI_Receive (
		&hspi1,
		data,
		length,
		HAL_MAX_DELAY
	);

	HAL_GPIO_WritePin (
		IMU_CS_GPIO_Port,
		IMU_CS_Pin,
		GPIO_PIN_SET
	);
}

void IMU_ReadData(IMU_Data *reading) {
	uint8_t raw[12];
	IMU_ReadRegisters(OUTX_L_G_REG, raw, 12);

	int16_t gx = (int16_t)((uint16_t)raw[1] << 8 | raw[0]);
	int16_t gy = (int16_t)((uint16_t)raw[3] << 8 | raw[2]);
	int16_t gz = (int16_t)((uint16_t)raw[5] << 8 | raw[4]);

	int16_t ax = (int16_t)(((uint16_t)raw[7] << 8) | raw[6]);
	int16_t ay = (int16_t)(((uint16_t)raw[9] << 8) | raw[8]);
	int16_t az = (int16_t)(((uint16_t)raw[11] << 8) | raw[10]);

	//equivalent to (*reading).accel_x
	reading->accel_x = ax * 0.0011964f;
	reading->accel_y = ay * 0.0011964f;
	reading->accel_z = az * 0.0011964f;

	reading->gyro_x = gx * 0.0175f;
	reading->gyro_y = gy * 0.0175f;
	reading->gyro_z = gz * 0.0175f;
}

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

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */

  who_am_i = IMU_ReadRegister(WHO_AM_I_REG);
  //program must be suspended before i can step over

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  IMU_Init();
  HAL_Delay(50);

  uint32_t curr_tick = HAL_GetTick();
  uint32_t last_check = HAL_GetTick();
  uint32_t sampled_hertz =0;
  uint32_t sample_count = 0;

  /* USER CODE END 3 */
  while (1) {

	  while ((int32_t)(HAL_GetTick() - curr_tick) < 0) {
		  //waiting
		  //behaves like HAL_Delay
	  }
	  //check to make sure it is actually 200HZ
	  IMU_ReadData(&reading);
	  sample_count++;
	  curr_tick += 1000 / SAMPLE_FREQ;

	  if(HAL_GetTick() - last_check >= 1000) {
		  //checking sampled_hertz each time for count per second
		  sampled_hertz = sample_count;
		  last_check += 1000;
		  sample_count = 0;
	  }

  }
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

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : IMU_CS_Pin */
  GPIO_InitStruct.Pin = IMU_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(IMU_CS_GPIO_Port, &GPIO_InitStruct);

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
