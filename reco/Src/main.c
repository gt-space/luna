/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "MS5611.h"
#include "ASM330LHGB1.h"
#include "LIS2MDL.h"
#include "SPI_Device.h"
#include "comms.h"

#include "stdio.h"
#include "stdbool.h"

#include "math.h"
#include "arm_math.h"

#include "../EKF/Inc/common.h"
#include "../EKF/Inc/matrix_extensions.h"
#include "../EKF/Inc/quaternion_extensions.h"
#include "../EKF/Inc/trig_extensions.h"
#include "../EKF/Inc/ekf_utils.h"
#include "../EKF/Inc/ekf.h"
#include "../EKF/tests.h"

#include "../CControl/ccontrol.h"
#include "stdatomic.h"

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

CRC_HandleTypeDef hcrc;

RTC_HandleTypeDef hrtc;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi3;
DMA_HandleTypeDef handle_GPDMA1_Channel5;
DMA_HandleTypeDef handle_GPDMA1_Channel4;

TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim13;
TIM_HandleTypeDef htim14;

/* USER CODE BEGIN PV */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_GPDMA1_Init(void);
static void MX_ICACHE_Init(void);
static void MX_SPI1_Init(void);
static void MX_RTC_Init(void);
static void MX_SPI3_Init(void);
static void MX_CRC_Init(void);
static void MX_TIM14_Init(void);
static void MX_TIM13_Init(void);
static void MX_TIM5_Init(void);
/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
reco_message doubleBuffReco[2] = {0};
fc_message fcData;

spi_device_t barometerSPIactual = {0};
spi_device_t imuSPIactual = {0};
spi_device_t magnetometerSPIactual = {0};

baro_handle_t baroHandlerActual = {0};
mag_handler_t magHandlerActual = {0};
imu_handler_t imuHandlerActual = {0};

spi_device_t* baroSPI = &barometerSPIactual;
spi_device_t* imuSPI = &imuSPIactual;
spi_device_t* magSPI = &magnetometerSPIactual;

baro_handle_t* baroHandler = &baroHandlerActual;
mag_handler_t* magHandler = &magHandlerActual;
imu_handler_t* imuHandler = &imuHandlerActual;

volatile bool convertTemp = false;

// Atomic buffer indexes
volatile atomic_uchar sendIdx = 0;  // CPU writes here
volatile atomic_uchar writeIdx  = 1;  // DMA/SPI reads here

volatile atomic_uchar gpsEventCount = 0;
volatile atomic_uchar magEventCount = 0;
volatile atomic_uchar baroEventCount = 0;

float32_t magDataStaging[3] = {0};
float32_t llaDataStaging[6] = {0};
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
	baro_handle_t* localBaro = &baroHandlerActual;
	mag_handler_t* localMag = &magHandlerActual;
	imu_handler_t* localIMU = &imuHandlerActual;

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
  MX_GPDMA1_Init();
  MX_ICACHE_Init();
  MX_SPI1_Init();
  MX_RTC_Init();
  MX_SPI3_Init();
  MX_CRC_Init();
  MX_TIM14_Init();
  MX_TIM13_Init();
  MX_TIM5_Init();
  /* USER CODE BEGIN 2 */

  // Initialize SPI Device Wrapper Libraries
  baroSPI->hspi = &hspi1;
  baroSPI->GPIO_Port = BAR_NCS_GPIO_Port;
  baroSPI->GPIO_Pin = BAR_NCS_Pin;

  magSPI->hspi = &hspi1;
  magSPI->GPIO_Port = MAG_NCS_GPIO_Port;
  magSPI->GPIO_Pin = MAG_NCS_Pin;

  imuSPI->hspi = &hspi1;
  imuSPI->GPIO_Port = IMU_NCS_GPIO_Port;
  imuSPI->GPIO_Pin = IMU_NCS_Pin;


  // Set the flags and initialize magnetometer
  set_lis2mdl_flags(magHandler);
  lis2mdl_initialize_mag(magSPI, magHandler);

  // Set the flags of IMU and initialize IMU
  setIMUFlags(imuHandler);
  initializeIMU(imuSPI, imuHandler);

  // Initialize barometer
  baroHandler->tempAccuracy = LOWEST_D1;
  baroHandler->pressureAccuracy = LOWEST_D2;
  baroHandler->convertTime = LOWEST_TIME;
  initBarometer(baroSPI, baroHandler);

  uint8_t mag_who_am_i = 0;
  uint8_t imu_who_am_i = 0;

  // Reads the WHO_AM_I Register
  readIMUSingleRegister(imuSPI, IMU_WHO_AM_I, &imu_who_am_i);
  HAL_Delay(1000);

  // Read magnetometer config registers
  uint8_t cfgReg[] = {0, 0, 0};
  lis2mdl_read_multiple_reg(magSPI, MAG_CFG_REG_A, MAG_CFG_REG_C, cfgReg);

  // Read IMU config registers
  uint8_t cfgRegIMU[] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
  readIMUMultipleRegisters(imuSPI, IMU_CTRL1_XL, IMU_CTRL10_C, cfgRegIMU);

  // Read magnetometer WHO AM I register
  lis2mdl_read_single_reg(magSPI, MAG_WHO_AM_I, &mag_who_am_i);
  HAL_Delay(1000);

  // Reads pressure and temperature
  getCurrTempPressure(baroSPI, baroHandler);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  // Get the startTime at this pont in miliseconds

  float32_t dt = 0.001f;

  arm_matrix_instance_f32 H, R, Rq, nu_gv_mat, nu_gu_mat,
  	  	  	  	  	  	  nu_av_mat, nu_au_mat, Q, Qq, PPrev,
						  PqPrev, Hb, Hq, xPrev, magI;

  arm_matrix_instance_f32 xPlus, Pplus, PqPlus;

  float32_t HBuff[3*21], HqBuff[3*6], RBuff[3*3], RqBuff[3*3], buff1[3*3], buff2[3*3],
  	  	  	buff3[3*3], buff4[3*3], QBuff[12*12], QqBuff[6*6], PBuffPrev[21*21], magIBuff[3],
			PqBuffPrev[6*6], HbBuff[1*21], xPlusData[22*1], PPlusData[21*21], PqPlusData[6*6];

  float32_t xPrevData[22*1] = {0.707598f,
							   -0.0004724356f,
							   0.7066147f,
							   -0.0007329581f,
							   30.9275f,
						       -81.51472222222f,
							   45.0f,
							   0,
							   0,
							   0,
							   0,
							   0,
							   0,
							   0,
							   0,
							   0,
							   0,
							   0,
							   0,
							   0,
							   0,
							   0};

  arm_mat_init_f32(&xPrev, 22, 1, xPrevData);
  arm_mat_init_f32(&xPlus, 22, 1, xPlusData);
  arm_mat_init_f32(&Pplus, 21, 21, PPlusData);
  arm_mat_init_f32(&PqPlus, 6, 6, PqPlusData);

  get_H(&H, HBuff);
  get_R(&R, RBuff);
  get_Rq(&Rq, RqBuff);
  compute_magI(&magI, magIBuff);
  get_Hq(&magI, &Hq, HqBuff);

  get_nu_gv_mat(&nu_gv_mat, buff1); // Use Jaden's values (gyro variance)
  get_nu_gu_mat(&nu_gu_mat, buff2); // mutiply by 2
  get_nu_av_mat(&nu_av_mat, buff3); // Use Jaden's values
  get_nu_au_mat(&nu_au_mat, buff4); // multiply by 2

  compute_Q(&Q, QBuff, &nu_gv_mat, &nu_gu_mat, &nu_av_mat, &nu_au_mat, dt);
  compute_Qq(&Qq, QqBuff, &nu_gv_mat, &nu_gu_mat, dt);
  compute_P0(&PPrev, PBuffPrev, att_unc0, pos_unc0, vel_unc0, gbias_unc0, abias_unc0, gsf_unc0, asf_unc0);

  compute_Pq0(&PqPrev, PqBuffPrev, att_unc0, gbias_unc0);
  pressure_derivative(&xPrev, &Hb, HbBuff);

  arm_matrix_instance_f32 aMeas, wMeas, llaMeas, magMeas;

  /*
  test_what();
  test_ahat();
  test_qdot();
  test_compute_vdot();
  test_compute_dwdp();
  test_compute_dwdv();
  test_compute_dpdot_dp();
  test_compute_dpdot_dv();
  test_compute_dvdot_dp();
  test_compute_dvdot_dv();
  test_compute_F();
  test_compute_G();
  test_compute_Pdot();
  test_compute_Pqdot();
  test_integrate();
  test_propogate();
  test_right_divide();
  test_update_GPS();
  test_update_mag();
  test_update_baro();
  test_nearest_PSD();
  test_update_EKF();
  */

  // Print headers for .csv file
  /*
  printf("Time (sec), X Acceleration (m/s^2), Y Acceleration (m/s^2), Z Acceleration (m/s^2), "
		 "Pitch Rate (milidegrees/sec), Yaw Rate (milidegrees/sec), Roll Rate (milidegrees/sec), "
		 "X Mag (Gauss), Y Mag (Gauss), Z Mag (Gauss), Temperature (degree C), Pressure (kPa)\n");
  */

  printf("Opcode, Vn (m/s), Ve (m/s), Vd (m/s), Lat (deg), Long (deg), Altitude (m), Valid?\n");

  HAL_TIM_Base_Start(&htim5);
  HAL_TIM_Base_Start_IT(&htim13);
  HAL_TIM_Base_Start_IT(&htim14);

  HAL_SPI_TransmitReceive_DMA(&hspi3, (uint8_t*) &doubleBuffReco[sendIdx], (uint8_t*) &fcData, 144);

  float32_t vdStart = 0;
  float32_t mainAltStart = 0;
  float32_t drougeAltStart = 0;
  int i = 0;

  while (1)
  {
    /* USER CODE END WHILE */
	uint32_t startTime = __HAL_TIM_GET_COUNTER(&htim5);

    /* USER CODE BEGIN 3 */
    // Get data from sensors
    getIMUData(imuSPI, imuHandler, doubleBuffReco[writeIdx].angularRate, doubleBuffReco[writeIdx].linAccel);

    // Magnetomer Data
    __disable_irq();
    memcpy(doubleBuffReco[writeIdx].magData, magDataStaging, 3*sizeof(float32_t));
    __enable_irq();

    // Barometer Data
    doubleBuffReco[writeIdx].pressure = baroHandler->pressure;
    doubleBuffReco[writeIdx].temperature = baroHandler->temperature;

    // GPS Data
    __disable_irq();
    memcpy(doubleBuffReco[writeIdx].llaPos, llaDataStaging, 6*sizeof(float32_t));
    __enable_irq();

	arm_mat_init_f32(&aMeas, 3, 1, doubleBuffReco[writeIdx].linAccel);
	arm_mat_init_f32(&wMeas, 3, 1, doubleBuffReco[writeIdx].angularRate);
	arm_mat_init_f32(&magMeas, 3, 1, doubleBuffReco[writeIdx].magData);
	arm_mat_init_f32(&llaMeas, 3, 1, doubleBuffReco[writeIdx].llaPos);

	update_EKF(&xPrev, &PPrev, &PqPrev,
			   &Q, &Qq, &H, &Hq,
			   &R, &Rq, Rb, &aMeas,
			   &wMeas, &llaMeas, &magMeas,
			   doubleBuffReco[sendIdx].pressure, &magI, we, dt, &xPlus,
			   &Pplus, &PqPlus, PPlusData, PPlusData, PqPlusData, &vdStart,
			   &mainAltStart, &drougeAltStart, &doubleBuffReco[sendIdx]);


	memcpy(xPrev.pData, xPlus.pData, 22*sizeof(float32_t));
	memcpy(Pplus.pData, PPrev.pData, 21*21*sizeof(float32_t));
	memcpy(PqPlus.pData, PqPrev.pData, 6*6*sizeof(float32_t));

	memcpy(&doubleBuffReco[writeIdx], xPrev.pData, 22*sizeof(float32_t));

	uint32_t endTime = __HAL_TIM_GET_COUNTER(&htim5);
	printf("Iteration Number: %d\n", i);
	printf("Elapsed Time: %f\n", ((endTime - startTime) / 1000.0f));
	i++;
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_CSI;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.CSIState = RCC_CSI_ON;
  RCC_OscInitStruct.CSICalibrationValue = RCC_CSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLL1_SOURCE_CSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 125;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1_VCIRANGE_2;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1_VCORANGE_WIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_PCLK3;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the programming delay
  */
  __HAL_FLASH_SET_PROGRAM_DELAY(FLASH_PROGRAMMING_DELAY_2);
}

/**
  * @brief CRC Initialization Function
  * @param None
  * @retval None
  */
static void MX_CRC_Init(void)
{

  /* USER CODE BEGIN CRC_Init 0 */

  /* USER CODE END CRC_Init 0 */

  /* USER CODE BEGIN CRC_Init 1 */

  /* USER CODE END CRC_Init 1 */
  hcrc.Instance = CRC;
  hcrc.Init.DefaultPolynomialUse = DEFAULT_POLYNOMIAL_ENABLE;
  hcrc.Init.DefaultInitValueUse = DEFAULT_INIT_VALUE_ENABLE;
  hcrc.Init.InputDataInversionMode = CRC_INPUTDATA_INVERSION_NONE;
  hcrc.Init.OutputDataInversionMode = CRC_OUTPUTDATA_INVERSION_DISABLE;
  hcrc.InputDataFormat = CRC_INPUTDATA_FORMAT_BYTES;
  if (HAL_CRC_Init(&hcrc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CRC_Init 2 */

  /* USER CODE END CRC_Init 2 */

}

/**
  * @brief GPDMA1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPDMA1_Init(void)
{

  /* USER CODE BEGIN GPDMA1_Init 0 */

  /* USER CODE END GPDMA1_Init 0 */

  /* Peripheral clock enable */
  __HAL_RCC_GPDMA1_CLK_ENABLE();

  /* GPDMA1 interrupt Init */
    HAL_NVIC_SetPriority(GPDMA1_Channel4_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(GPDMA1_Channel4_IRQn);
    HAL_NVIC_SetPriority(GPDMA1_Channel5_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(GPDMA1_Channel5_IRQn);

  /* USER CODE BEGIN GPDMA1_Init 1 */

  /* USER CODE END GPDMA1_Init 1 */
  /* USER CODE BEGIN GPDMA1_Init 2 */

  /* USER CODE END GPDMA1_Init 2 */

}

/**
  * @brief ICACHE Initialization Function
  * @param None
  * @retval None
  */
static void MX_ICACHE_Init(void)
{

  /* USER CODE BEGIN ICACHE_Init 0 */

  /* USER CODE END ICACHE_Init 0 */

  /* USER CODE BEGIN ICACHE_Init 1 */

  /* USER CODE END ICACHE_Init 1 */

  /** Enable instruction cache (default 2-ways set associative cache)
  */
  if (HAL_ICACHE_Enable() != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ICACHE_Init 2 */

  /* USER CODE END ICACHE_Init 2 */

}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  RTC_PrivilegeStateTypeDef privilegeState = {0};
  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef sDate = {0};

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */

  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutRemap = RTC_OUTPUT_REMAP_NONE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  hrtc.Init.OutPutPullUp = RTC_OUTPUT_PULLUP_NONE;
  hrtc.Init.BinMode = RTC_BINARY_NONE;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }
  privilegeState.rtcPrivilegeFull = RTC_PRIVILEGE_FULL_NO;
  privilegeState.backupRegisterPrivZone = RTC_PRIVILEGE_BKUP_ZONE_NONE;
  privilegeState.backupRegisterStartZone2 = RTC_BKP_DR0;
  privilegeState.backupRegisterStartZone3 = RTC_BKP_DR0;
  if (HAL_RTCEx_PrivilegeModeSet(&hrtc, &privilegeState) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN Check_RTC_BKUP */

  /* USER CODE END Check_RTC_BKUP */

  /** Initialize RTC and set the Time and Date
  */
  sTime.Hours = 0x0;
  sTime.Minutes = 0x0;
  sTime.Seconds = 0x0;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  sDate.WeekDay = RTC_WEEKDAY_MONDAY;
  sDate.Month = RTC_MONTH_JANUARY;
  sDate.Date = 0x1;
  sDate.Year = 0x0;

  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

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
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 0x7;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  hspi1.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi1.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi1.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi1.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi1.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi1.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
  hspi1.Init.IOSwap = SPI_IO_SWAP_DISABLE;
  hspi1.Init.ReadyMasterManagement = SPI_RDY_MASTER_MANAGEMENT_INTERNALLY;
  hspi1.Init.ReadyPolarity = SPI_RDY_POLARITY_HIGH;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief SPI3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI3_Init(void)
{

  /* USER CODE BEGIN SPI3_Init 0 */

  /* USER CODE END SPI3_Init 0 */

  /* USER CODE BEGIN SPI3_Init 1 */

  /* USER CODE END SPI3_Init 1 */
  /* SPI3 parameter configuration*/
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_SLAVE;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi3.Init.NSS = SPI_NSS_HARD_INPUT;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 0x7;
  hspi3.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  hspi3.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi3.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi3.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi3.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi3.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi3.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
  hspi3.Init.IOSwap = SPI_IO_SWAP_DISABLE;
  hspi3.Init.ReadyMasterManagement = SPI_RDY_MASTER_MANAGEMENT_INTERNALLY;
  hspi3.Init.ReadyPolarity = SPI_RDY_POLARITY_HIGH;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI3_Init 2 */

  /* USER CODE END SPI3_Init 2 */

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 250 - 1;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 4294967295;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */

}

/**
  * @brief TIM13 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM13_Init(void)
{

  /* USER CODE BEGIN TIM13_Init 0 */

  /* USER CODE END TIM13_Init 0 */

  /* USER CODE BEGIN TIM13_Init 1 */

  /* USER CODE END TIM13_Init 1 */
  htim13.Instance = TIM13;
  htim13.Init.Prescaler = 250-1;
  htim13.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim13.Init.Period = 9999;
  htim13.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim13.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim13) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM13_Init 2 */

  /* USER CODE END TIM13_Init 2 */

}

/**
  * @brief TIM14 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM14_Init(void)
{

  /* USER CODE BEGIN TIM14_Init 0 */

  /* USER CODE END TIM14_Init 0 */

  /* USER CODE BEGIN TIM14_Init 1 */

  /* USER CODE END TIM14_Init 1 */
  htim14.Instance = TIM14;
  htim14.Init.Prescaler = 250-1;
  htim14.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim14.Init.Period = 2499;
  htim14.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim14.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim14) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM14_Init 2 */

  /* USER CODE END TIM14_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, MAG_NCS_Pin|BAR_NCS_Pin|IMU_NCS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(MAG_INT_GPIO_Port, MAG_INT_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, MAG_DRDY_Pin|STAGE1_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(STAGE2_EN_GPIO_Port, STAGE2_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : MAG_NCS_Pin BAR_NCS_Pin IMU_NCS_Pin */
  GPIO_InitStruct.Pin = MAG_NCS_Pin|BAR_NCS_Pin|IMU_NCS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : MAG_INT_Pin */
  GPIO_InitStruct.Pin = MAG_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(MAG_INT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : MAG_DRDY_Pin */
  GPIO_InitStruct.Pin = MAG_DRDY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(MAG_DRDY_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : STAGE2_EN_Pin */
  GPIO_InitStruct.Pin = STAGE2_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(STAGE2_EN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PB10 */
  GPIO_InitStruct.Pin = GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : VREF_FB2_Pin VREF_FB1_Pin */
  GPIO_InitStruct.Pin = VREF_FB2_Pin|VREF_FB1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : STAGE1_EN_Pin */
  GPIO_InitStruct.Pin = STAGE1_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(STAGE1_EN_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void print_bytes_binary(const uint8_t *data, size_t len) {
    for (size_t i = 0; i < len; i++) {
        for (int bit = 7; bit >= 0; bit--) {
            printf("%c", (data[i] & (1 << bit)) ? 1 : '0');
        }
        if (i < len - 1) {
            printf(" "); // space between bytes
        }
    }
    printf("\n");
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
	if (hspi->Instance == SPI3) {
		printf("%d, %f, %f, %f, %f, %f, %f, %d, WriteIdx: %d, SendIdx: %d\n", fcData.opcode,
												   fcData.body.gpsVel[0],
												   fcData.body.gpsVel[1],
												   fcData.body.gpsVel[2],
												   fcData.body.gpsLLA[0],
											       fcData.body.gpsLLA[1],
												   fcData.body.gpsLLA[2],
												   fcData.body.valid,
												   writeIdx,
												   sendIdx);


		if (fcData.body.valid) {
			atomic_fetch_add(&gpsEventCount, 1);
			memcpy(llaDataStaging, fcData.body.gpsLLA, 6*sizeof(float32_t));
		}

		__disable_irq();
		uint8_t tmp = writeIdx;
		writeIdx = sendIdx;
		sendIdx = tmp;
		HAL_SPI_TransmitReceive_DMA(&hspi3, (uint8_t*) &doubleBuffReco[sendIdx], (uint8_t*) &fcData, 144);
		__enable_irq();
	}

}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {

	if (htim->Instance == TIM13) {
		lis2mdl_get_mag_data(magSPI, magHandler, magDataStaging);
		atomic_fetch_add(&magEventCount, 1);
	} else if (htim->Instance == TIM14) {

		if (convertTemp) {
			calculatePress(baroSPI, baroHandler);
			startTemperatureConversion(baroSPI, baroHandler);
 			convertTemp = false;
		} else {
			calculateTemp(baroSPI, baroHandler);
			startPressureConversion(baroSPI, baroHandler);
			convertTemp = true;
			atomic_fetch_add(&baroEventCount, 1);
		}
	}
}

void atomic_xor_u8(volatile uint8_t *ptr)
{
    uint8_t status;
    do {
        uint8_t old = __LDREXB(ptr);
        status = __STREXB(old ^ 1, ptr);
    } while (status != 0);

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

#ifdef  USE_FULL_ASSERT
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
