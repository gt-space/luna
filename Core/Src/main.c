
/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdint.h>
#include <stdio.h>
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

typedef struct
{
    int16_t gyro_x;
    int16_t gyro_y;
    int16_t gyro_z;

    int16_t accel_x;
    int16_t accel_y;
    int16_t accel_z;
} IMU_RawData;

typedef struct
{
    float gyro_x_dps;
    float gyro_y_dps;
    float gyro_z_dps;

    float accel_x_ms2;
    float accel_y_ms2;
    float accel_z_ms2;
} IMU_Data;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

// Gyroscope
#define OUTX_L_G   0x22
#define OUTX_H_G   0x23
#define OUTY_L_G   0x24
#define OUTY_H_G   0x25
#define OUTZ_L_G   0x26
#define OUTZ_H_G   0x27

// Accelerometer
#define OUTX_L_A   0x28
#define OUTX_H_A   0x29
#define OUTY_L_A   0x2A
#define OUTY_H_A   0x2B
#define OUTZ_L_A   0x2C
#define OUTZ_H_A   0x2D

#define WHO_AM_I_REG   0x0F
#define CTRL1_XL       0x10
#define CTRL2_G        0x11
#define CTRL3_C        0x12

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

/* USER CODE BEGIN PV */

static const uint8_t imu_data_registers[12] =
{
    OUTX_L_G,
    OUTX_H_G,
    OUTY_L_G,
    OUTY_H_G,
    OUTZ_L_G,
    OUTZ_H_G,

    OUTX_L_A,
    OUTX_H_A,
    OUTY_L_A,
    OUTY_H_A,
    OUTZ_L_A,
    OUTZ_H_A
};

static IMU_RawData imu_raw;
static IMU_Data imu_data;

volatile uint8_t who_am_i;
volatile float accel_magnitude;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);

/* USER CODE BEGIN PFP */

extern void initalise_monitor_handles(void);
uint8_t IMU_ReadRegister(uint8_t reg);
void IMU_ReadRegisters(uint8_t reg, uint8_t *data, uint16_t len);
void IMU_WriteRegister(uint8_t reg, uint8_t val);
void IMU_ReadRaw(IMU_RawData *imu);
void IMU_ReadRawBurst(IMU_RawData *imu);
void IMU_Convert(const IMU_RawData *raw, IMU_Data *data);
void IMU_Display(IMU_Data *imu);
void IMU_Init(void);

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

    HAL_Init();

    /* USER CODE BEGIN Init */
    /* USER CODE END Init */

    SystemClock_Config();

    /* USER CODE BEGIN SysInit */
    /* USER CODE END SysInit */

    MX_GPIO_Init();
    MX_SPI1_Init();

    /* USER CODE BEGIN 2 */

    initialise_monitor_handles();

    printf("Semihosting works!\r\n");
    fflush(stdout);

    IMU_Init();

    uint32_t next_sample = HAL_GetTick();

    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1)
    {
        /* USER CODE END WHILE */

        if ((int32_t)(HAL_GetTick() - next_sample) >= 0)
        {
            next_sample += 5;

            IMU_ReadRawBurst(&imu_raw);

            IMU_Convert(&imu_raw, &imu_data);

            float ax = imu_data.accel_x_ms2;
            float ay = imu_data.accel_y_ms2;
            float az = imu_data.accel_z_ms2;

            accel_magnitude = sqrtf(
                ax * ax + ay * ay + az * az
            );
        }

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

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;

    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }

    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                               | RCC_CLOCKTYPE_PCLK1;
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
  * @retval None
  */
static void MX_SPI1_Init(void)
{
    /* USER CODE BEGIN SPI1_Init 0 */
    /* USER CODE END SPI1_Init 0 */

    /* USER CODE BEGIN SPI1_Init 1 */
    /* USER CODE END SPI1_Init 1 */

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
  * @retval None
  */
static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* USER CODE BEGIN MX_GPIO_Init_1 */
    /* USER CODE END MX_GPIO_Init_1 */

    __HAL_RCC_GPIOA_CLK_ENABLE();

    HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_SET);

    GPIO_InitStruct.Pin = IMU_CS_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;

    HAL_GPIO_Init(IMU_CS_GPIO_Port, &GPIO_InitStruct);

    /* USER CODE BEGIN MX_GPIO_Init_2 */
    /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

uint8_t IMU_ReadRegister(uint8_t reg)
{
    uint8_t tx[2];
    uint8_t rx[2] = {0};

    tx[0] = reg | 0x80;
    tx[1] = 0x00;

    HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_RESET);

    HAL_StatusTypeDef status =
        HAL_SPI_TransmitReceive(&hspi1, tx, rx, 2, 100);

    HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_SET);

    if (status != HAL_OK)
    {
        Error_Handler();
    }

    return rx[1];
}

void IMU_ReadRegisters(uint8_t reg, uint8_t *data, uint16_t len)
{
    uint8_t command = reg | 0x80;

    HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_RESET);

    HAL_StatusTypeDef status =
        HAL_SPI_Transmit(&hspi1, &command, 1, 100);

    if (status == HAL_OK)
    {
        status = HAL_SPI_Receive(&hspi1, data, len, 100);
    }

    HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_SET);

    if (status != HAL_OK)
    {
        Error_Handler();
    }
}

void IMU_WriteRegister(uint8_t reg, uint8_t val)
{
    uint8_t tx[2];

    tx[0] = reg & 0x7F;
    tx[1] = val;

    HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_RESET);

    HAL_StatusTypeDef status =
        HAL_SPI_Transmit(&hspi1, tx, 2, 100);

    HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_SET);

    if (status != HAL_OK)
    {
        Error_Handler();
    }
}

void IMU_ReadRaw(IMU_RawData *imu)
{
    uint8_t raw[12];

    for (int i = 0; i < 12; i++)
    {
        raw[i] = IMU_ReadRegister(imu_data_registers[i]);
    }

    imu->gyro_x = (int16_t)(((uint16_t)raw[1] << 8) | raw[0]);
    imu->gyro_y = (int16_t)(((uint16_t)raw[3] << 8) | raw[2]);
    imu->gyro_z = (int16_t)(((uint16_t)raw[5] << 8) | raw[4]);

    imu->accel_x = (int16_t)(((uint16_t)raw[7] << 8) | raw[6]);
    imu->accel_y = (int16_t)(((uint16_t)raw[9] << 8) | raw[8]);
    imu->accel_z = (int16_t)(((uint16_t)raw[11] << 8) | raw[10]);
}

void IMU_ReadRawBurst(IMU_RawData *imu)
{
    uint8_t raw[12];

    IMU_ReadRegisters(OUTX_L_G, raw, 12);

    imu->gyro_x = (int16_t)(((uint16_t)raw[1] << 8) | raw[0]);
    imu->gyro_y = (int16_t)(((uint16_t)raw[3] << 8) | raw[2]);
    imu->gyro_z = (int16_t)(((uint16_t)raw[5] << 8) | raw[4]);

    imu->accel_x = (int16_t)(((uint16_t)raw[7] << 8) | raw[6]);
    imu->accel_y = (int16_t)(((uint16_t)raw[9] << 8) | raw[8]);
    imu->accel_z = (int16_t)(((uint16_t)raw[11] << 8) | raw[10]);
}

void IMU_Convert(const IMU_RawData *raw, IMU_Data *data)
{
    const float accel_sensitivity = 0.244f;
    const float gyro_sensitivity = 0.070f;

    data->accel_x_ms2 =
        raw->accel_x * accel_sensitivity / 1000.0f * 9.80665f;

    data->accel_y_ms2 =
        raw->accel_y * accel_sensitivity / 1000.0f * 9.80665f;

    data->accel_z_ms2 =
        raw->accel_z * accel_sensitivity / 1000.0f * 9.80665f;

    data->gyro_x_dps =
        raw->gyro_x * gyro_sensitivity;

    data->gyro_y_dps =
        raw->gyro_y * gyro_sensitivity;

    data->gyro_z_dps =
        raw->gyro_z * gyro_sensitivity;
}

void IMU_Display(IMU_Data *data)
{
	printf(
	    "Accel: %.2f %.2f %.2f | Gyro: %.2f %.2f %.2f\r\n",
	    data.accel_x_ms2,
		data.accel_y_ms2,
	    data.accel_z_ms2,
	    data.gyro_x_dps,
	    data.gyro_y_dps,
	    data.gyro_z_dps
	);
}

void IMU_Init(void)
{
    uint8_t who = IMU_ReadRegister(WHO_AM_I_REG);

    who_am_i = who;

    if (who != 0x6B)
    {
        Error_Handler();
    }

    IMU_WriteRegister(CTRL3_C, 0x44);
    IMU_WriteRegister(CTRL1_XL, 0x5C);
    IMU_WriteRegister(CTRL2_G, 0x5C);

    HAL_Delay(20);
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
    /* USER CODE BEGIN Error_Handler_Debug */
    __disable_irq();
    while (1)
    {
    }
    /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
    /* USER CODE BEGIN 6 */
    (void)file;
    (void)line;
    /* USER CODE END 6 */
}
#endif
