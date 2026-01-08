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
#include "../EKF/Inc/tests.h"

#include "../CControl/ccontrol.h"
#include "motion_mc.h"
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

TIM_HandleTypeDef htim2;
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
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// Double buffer used to hold data that will be received and sent to FC
reco_message doubleBuffReco[2] = {0}; // Sent to FC
fc_message fcData[2] = {0}; // Received from FC

// Defines SPI wrappers for each of our sensors.
// Wrapper handles flipping CS line and ensuring that sending
// and receiving of commands is atomic and cannot be interrupted
spi_device_t barometerSPIactual = {0}; 
spi_device_t imuSPIactual = {0};
spi_device_t magnetometerSPIactual = {0};

// Each handler struct contains data that is important for the processing of data
// for each sensor. These include config register values (IMU/MAG) and the accuracy
// of barometer temperature and pressure values
baro_handle_t baroHandlerActual = {0};
mag_handler_t magHandlerActual = {0};
imu_handler_t imuHandlerActual = {0};

spi_device_t* baroSPI = &barometerSPIactual;
spi_device_t* imuSPI = &imuSPIactual;
spi_device_t* magSPI = &magnetometerSPIactual;

baro_handle_t* baroHandler = &baroHandlerActual;
mag_handler_t* magHandler = &magHandlerActual;
imu_handler_t* imuHandler = &imuHandlerActual;

// Converted temp tells the system once the main EKF has started that we have 
// already have a valid temperature and that you can calculate pressure next
volatile bool convertedTemp = true;

// Will hold the amount of time between launch and system start. 
// Used in timer backups and goldfish timer.
uint32_t launchTime = 0;

// Defines atomic variables which determine which of the above buffers
// in doubleBuffReco and fcData are safe to write and which will take in data/send data to
// FC
volatile atomic_uchar sendIdx = 0;  // CPU writes here
volatile atomic_uchar writeIdx  = 1;  // DMA/SPI reads here

// If any of these variables is 1, it means that the EKF should incorporate
// the sensor measurement into the filter
volatile atomic_uchar gpsEventCount = 0;
volatile atomic_uchar magEventCount = 0;
volatile atomic_uchar baroEventCount = 0;

// Staging variables that hold sensor meassurements
float32_t magDataStaging[3] = {0};
float32_t llaDataStaging[6] = {0};
float32_t llaBuff[3] = {0};

bool launched = false; 		// Gets set to true when FC sends the RECO Launch command
bool stage1Enabled = false; // Is set true when EKF/backups determine we are at apogee
bool stage2Enabled = false; // Is set false when bacometer determines we are at 2950 ft
bool fallbackDR = false; 	// Is used to determine whether EKF blew up and we need to fallback

void checkForFault(uint8_t faultingDrivers[5]); // Check for faults on the recovery drivers
void solveFault(uint8_t faultingDrivers[5]);    // Solve the fault by bringing the pins low
void setFault(uint8_t faultingDrivers[5]);      // Set up the fault for next interation by bring FLT pins HIGH
void logVREF(void);								// Log VREF values to be saved by RECO-FC Comms
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
  MX_TIM2_Init();
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
  baroHandler->pressureAccuracy = LOWEST_PRESS;
  baroHandler->tempAccuracy = LOWEST_TEMP;
  baroHandler->convertTime = LOWEST_TIME;

  initBarometer(baroSPI, baroHandler);

  // Seed the dT and TEMP values such that we can start
  // the asynchronous barometer reading process
  // Setting convertedTemp to true tells the program that 
  // we have valid temperature data and collect pressure data next

  getCurrTempPressure(baroSPI, baroHandler);
  startPressureConversion(baroSPI, baroHandler);
  convertedTemp = true;

  // Variables that check that the magnetometer and the accelerometer 
  // are functioning correctly 
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

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  // Timestemp between iterations of EKF
  float32_t dt = 0.0015f;

  // Matrices that are used to run the filter
  // x is the state vector. R is the uncertainity in the measurements from the sensors.
  // Q describes how uncertain we are in the modeling of the system dynamics.
  // P describes the overall uncertainity in the current state of the filter.
  arm_matrix_instance_f32 H, R, Rq, nu_gv_mat, nu_gu_mat,
						  nu_av_mat, nu_au_mat, Q, PPrev,
						  Hb, xPrev, magI, xPlus, Pplus;

  // The buffers that actually hold the data of the matrices above
  float32_t HBuff[3*21], RBuff[3*3], RqBuff[3*3], buff1[3*3], buff2[3*3],
				buff3[3*3], buff4[3*3], QBuff[12*12], PBuffPrev[21*21], magIBuff[3],
				  HbBuff[1*21], xPlusData[22*1], PPlusData[21*21];

	// The initial state of the filter. Should be initialized by current attitude,
	// current locations (lat, long, altitude), biases, and scale factors.
	float32_t xPrevData[22*1] =  {1.0f,
								  0.0f,
								  0.0f,
								  0.0f,
								  35.044722,
								  -118.156619,
								  304.19,
								  0,
								  0,
								  0,
								  -0.006512509819065554,
								  -0.023189516912629,
								  -0.011958224912895268,
								  0.17097415819490253,
								  -0.1957076875048044,
								  0.05918231868563595,
								  0,
								  0,
								  0,
								  0,
								  0,
								  0};

	// Initializes the previous state vector, the next state vector, and the covariance matrices
	arm_mat_init_f32(&xPrev, 22, 1, xPrevData);
	arm_mat_init_f32(&xPlus, 22, 1, xPlusData);
	arm_mat_init_f32(&Pplus, 21, 21, PPlusData);

	// Initializes the barometer measurement Jacobian (H),
	// the GPS measurement noise matrix (R),
	// and the magnetometer measurement noise (Rq)
	get_H(&H, HBuff);
	get_R(&R, RBuff);
	get_Rq(&Rq, RqBuff);
	compute_magI(&magI, magIBuff);

	// Initializes covariances for the accelerometer and the gyroscope.
	// More details about what types of matrices are initialized are included
	// at the definition of the functions.
	get_nu_gv_mat(&nu_gv_mat, buff1);
	get_nu_gu_mat(&nu_gu_mat, buff2);
	get_nu_av_mat(&nu_av_mat, buff3);
	get_nu_au_mat(&nu_au_mat, buff4);

	// These were defined earlier in the code.
	compute_Q(&Q, QBuff, &nu_gv_mat, &nu_gu_mat, &nu_av_mat, &nu_au_mat, dt);
	compute_P0(&PPrev, PBuffPrev, att_unc0, pos_unc0, vel_unc0, gbias_unc0, abias_unc0, gsf_unc0, asf_unc0);

	initialize_Hb(&xPrev, &Hb, HbBuff);

	// Contains the measurements from our sensors which are accelerometer,
	// gyroscope, GPS, and magnetometer.
	arm_matrix_instance_f32 aMeas, wMeas, llaMeas, magMeas;

	/* Test Suite for the Filter. Not used in Production */
	// test_what();
	// test_ahat();
	// test_qdot();
	// test_lla_dot();
	// test_compute_vdot();
	// test_compute_dwdp();
	// test_compute_dwdv();
	// test_compute_dpdot_dp();
	// test_compute_dpdot_dv();
	// test_compute_dvdot_dp();
	// test_compute_dvdot_dv();
	// test_compute_F();
	// test_compute_G();
	// test_compute_Pdot();
	// test_integrate();
	// test_propogate();
	// test_right_divide();
	// test_eig();
	// test_nearest_PSD();
	// test_update_GPS();
	// test_update_mag();
	// test_update_baro();
	// test_update_EKF();

	// Time since launch is contained in the variable timeSinceLaunch variable.
	// HTIM13 and HTIM14 are timers that are used to tell when the program needs to collect
	// sensor data.
	HAL_TIM_Base_Start_IT(&htim13); // Timer for magnetometer
	HAL_TIM_Base_Start_IT(&htim14); // Timer for barometer

	// Start up the DMA transcation which communicates with FC
	HAL_SPI_TransmitReceive_DMA(&hspi3,
								(uint8_t*) &doubleBuffReco[sendIdx],
								(uint8_t*) &fcData[sendIdx],
								148);

	// Variables that measure how much time we have had a negative downwards velocity
	uint32_t vdStart = UINT32_MAX;
	uint32_t mainAltStart = UINT32_MAX;
	uint32_t drougeAltStart = UINT32_MAX;

	// Debug variables that can be printed out to terminal to assess filter performance
	volatile uint32_t i = 0;
	float32_t endTime = 0;

	// Gives us the pins that are faulting
	uint8_t faultingDrivers[5] = {0};

	//printf("Starting....\n");
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  // Variable to measure how long it takes to run the filter
	  // uint32_t currIterStartTimer = __HAL_TIM_GET_COUNTER(&htim5);

	// Fill the faulting drivers buffer with the failing RECO drivers
	checkForFault(faultingDrivers);

    // Get data from sensors
    getIMUData(imuSPI, imuHandler, doubleBuffReco[writeIdx].angularRate, doubleBuffReco[writeIdx].linAccel);

    // Magnetomer Data copied over for the filter
    __disable_irq();
    memcpy(doubleBuffReco[writeIdx].magData, magDataStaging, 3*sizeof(float32_t));
    __enable_irq();

    // Barometer Data for filter
    __disable_irq();
    doubleBuffReco[writeIdx].pressure = baroHandler->pressure; // Due to it simply writing a word it is atomic
    __enable_irq();

    // Temperature Data for logging by FC
    __disable_irq();
    doubleBuffReco[writeIdx].temperature = baroHandler->temperature; // Due to it simply writing a word it is atomic
    __enable_irq();

    // GPS Data for filter
    __disable_irq();
    memcpy(llaBuff, fcData[writeIdx].body.gpsLLA, 3*sizeof(float32_t));
    __enable_irq();

    // Initialize the measurement matrices
    arm_mat_init_f32(&aMeas, 3, 1, doubleBuffReco[writeIdx].linAccel);
    arm_mat_init_f32(&wMeas, 3, 1, doubleBuffReco[writeIdx].angularRate);
    arm_mat_init_f32(&magMeas, 3, 1, doubleBuffReco[writeIdx].magData);
    arm_mat_init_f32(&llaMeas, 3, 1, llaBuff);

    // Update the state of the filter
    update_EKF(&xPrev, &PPrev, &Q, &H,
    		   &R, &Rq, Rb, &aMeas,
			   &wMeas, &llaMeas, &magMeas,
			   doubleBuffReco[writeIdx].pressure, &magI, we, dt, &xPlus,
			   &Pplus, xPlusData, PPlusData, &fcData[writeIdx], &fallbackDR);

    // Solve the fault by bringing the faulting channels fault pins low
    solveFault(faultingDrivers);

    float32_t currAltitude = xPlus.pData[6]; // The altitude of the current state
    float32_t prevAltitude = xPrev.pData[6]; // The altitude of the previously computed state ss
    float32_t deltaAlt = currAltitude - prevAltitude; // The difference between altitudes
    float32_t downVel = xPlus.pData[9]; // Downwards velocitity of the current state

    /*
    * For drouge to deploy, the following must be true:
    * 		1. Altitude has been decreasing for six seconds
    * 		2. Vehicle has launched (set by the reco_enabled command from FC)
    * 		3. Stage 1 chutes must not have been enabled by anything else in the software
    */
    if (drougeChuteCheck(deltaAlt, drougeAltStart) && launched && !stage1Enabled) {
      // stage1Enabled = true;
      // doubleBuffReco[sendIdx].stage1En = true;
      // doubleBuffReco[writeIdx].stage1En = true;
      // HAL_GPIO_WritePin(STAGE1_EN_GPIO_Port, STAGE1_EN_Pin, GPIO_PIN_SET);
    }

    /*
    * For main to deploy, we must be at 2950 ft or lower.
    */
    if (mainChuteCheck(currAltitude, mainAltStart)) {
      // stage2Enabled = true;
      // doubleBuffReco[sendIdx].stage1En = true;
      // doubleBuffReco[writeIdx].stage1En = true;
      // HAL_GPIO_WritePin(STAGE2_EN_GPIO_Port, STAGE2_EN_Pin, GPIO_PIN_SET);
    }

    // Set the current state to the previous state
    memcpy(xPrev.pData, xPlus.pData, 22*sizeof(float32_t));
    memcpy(PPrev.pData, Pplus.pData, 21*21*sizeof(float32_t));
    memcpy(&doubleBuffReco[writeIdx], xPrev.pData, 22*sizeof(float32_t));

    // Setup for the next iteration by setting the faulting pins back to high
    // this should run about 2 microseconds later than solveFault()
    setFault(faultingDrivers);

    // Read the voltage of the voting logic
    logVREF();

    if (launched) {
      // Calculates the time since launch (elapsedTime is units of microseconds)
      uint32_t elapsedTime = HAL_GetTick() - launchTime;

      // Drouge Timer (given in miliseconds)
      if (elapsedTime < 52000 && stage1Enabled) {
    	  fallbackDR = true;
      }
    }

    // Switch the sending and writing buffer for RECO-FC commnication.
    __disable_irq();
    atomic_fetch_xor(&writeIdx, 1);
    atomic_fetch_xor(&sendIdx, 1);
    HAL_SPI_TransmitReceive_DMA(&hspi3, (uint8_t*) &doubleBuffReco[sendIdx], (uint8_t*) &fcData[sendIdx], 148);
    __enable_irq();

    // The end time of this iteration
    // uint32_t endTime = currIterStartTime / 1000.0f;

    /* Below lines are used when testing to determine filter performance */
    // printf("Write Idx: %d\n", writeIdx);
    // printf("Send Idx: %d\n", sendIdx);
    // printMatrix(&xPrev);
    // printf("Iteration Number: %d\n\n", i);
    // printf("Elapsed Time: %f\n", ((endTime - startTime) / 1000.0f));

    // Iteration Number
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
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 250-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1499999999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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
  htim5.Init.Period = 108999999;
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
  HAL_GPIO_WritePin(STAGE2_EN_GPIO_Port, STAGE2_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(STAGE1_EN_GPIO_Port, STAGE1_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LATCH_A_Pin|LATCH_B_Pin|LATCH_C_Pin|LATCH_D_Pin
                          |LATCH_E_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : MAG_NCS_Pin BAR_NCS_Pin IMU_NCS_Pin */
  GPIO_InitStruct.Pin = MAG_NCS_Pin|BAR_NCS_Pin|IMU_NCS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : FLT_A_Pin FLT_B_Pin FLT_C_Pin FLT_D_Pin
                           VREF_FB2_Pin VREF_FB1_Pin FLT_E_Pin */
  GPIO_InitStruct.Pin = FLT_A_Pin|FLT_B_Pin|FLT_C_Pin|FLT_D_Pin
                          |VREF_FB2_Pin|VREF_FB1_Pin|FLT_E_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : MAG_INT_Pin LATCH_A_Pin LATCH_B_Pin LATCH_C_Pin
                           LATCH_D_Pin LATCH_E_Pin */
  GPIO_InitStruct.Pin = MAG_INT_Pin|LATCH_A_Pin|LATCH_B_Pin|LATCH_C_Pin
                          |LATCH_D_Pin|LATCH_E_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : STAGE2_EN_Pin */
  GPIO_InitStruct.Pin = STAGE2_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(STAGE2_EN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : VREF_FB1_E_Pin VREF_FB2_D_Pin VREF_FB1_D_Pin VREF_FB2_E_Pin */
  GPIO_InitStruct.Pin = VREF_FB1_E_Pin|VREF_FB2_D_Pin|VREF_FB1_D_Pin|VREF_FB2_E_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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

void logVREF(void) {
    doubleBuffReco[writeIdx].vref_a_channel1 = HAL_GPIO_ReadPin(VREF_FB1_GPIO_Port, VREF_FB1_Pin);
    doubleBuffReco[writeIdx].vref_a_channel2 = HAL_GPIO_ReadPin(VREF_FB2_GPIO_Port, VREF_FB2_Pin);
    doubleBuffReco[writeIdx].vref_b_channel1 = HAL_GPIO_ReadPin(VREF_FB1_GPIO_Port, VREF_FB1_Pin);
    doubleBuffReco[writeIdx].vref_b_channel2 = HAL_GPIO_ReadPin(VREF_FB2_GPIO_Port, VREF_FB2_Pin);
    doubleBuffReco[writeIdx].vref_c_channel1 = HAL_GPIO_ReadPin(VREF_FB1_GPIO_Port, VREF_FB1_Pin);
    doubleBuffReco[writeIdx].vref_c_channel2 = HAL_GPIO_ReadPin(VREF_FB2_GPIO_Port, VREF_FB2_Pin);
    doubleBuffReco[writeIdx].vref_d_channel1 = HAL_GPIO_ReadPin(VREF_FB1_D_GPIO_Port, VREF_FB1_D_Pin);
    doubleBuffReco[writeIdx].vref_d_channel2 = HAL_GPIO_ReadPin(VREF_FB2_D_GPIO_Port, VREF_FB2_D_Pin);
    doubleBuffReco[writeIdx].vref_e_channel1 = HAL_GPIO_ReadPin(VREF_FB1_E_GPIO_Port, VREF_FB1_E_Pin);
    doubleBuffReco[writeIdx].vref_e_channel2 = HAL_GPIO_ReadPin(VREF_FB2_E_GPIO_Port, VREF_FB2_E_Pin);
}

void checkForFault(uint8_t faultingDrivers[5]) {

	// There are five recovery drivers. In flight, there is a possibility that they fault the
	// chip knows that because when we read the pin of fault it will be low. We can reset, each
	// recovery driver by bringing the LATCH pin low (LATCH is active low) and then resetting it
	// back to high. FLT is also active low signal. Check STM32 Pin Configuration in Notion to be sure.

	faultingDrivers[0] = !HAL_GPIO_ReadPin(FLT_A_GPIO_Port, FLT_A_Pin);
	faultingDrivers[1] = !HAL_GPIO_ReadPin(FLT_B_GPIO_Port, FLT_B_Pin);
	faultingDrivers[2] = !HAL_GPIO_ReadPin(FLT_C_GPIO_Port, FLT_C_Pin);
	faultingDrivers[3] = !HAL_GPIO_ReadPin(FLT_D_GPIO_Port, FLT_D_Pin);
	faultingDrivers[4] = !HAL_GPIO_ReadPin(FLT_E_GPIO_Port, FLT_E_Pin);

}

void solveFault(uint8_t faultingDrivers[5]) {

	if (faultingDrivers[0]) {
		HAL_GPIO_WritePin(LATCH_A_GPIO_Port, LATCH_A_Pin, GPIO_PIN_RESET);
	}

	if (faultingDrivers[1]) {
		HAL_GPIO_WritePin(LATCH_B_GPIO_Port, LATCH_B_Pin, GPIO_PIN_RESET);
	}

	if (faultingDrivers[2]) {
		HAL_GPIO_WritePin(LATCH_C_GPIO_Port, LATCH_C_Pin, GPIO_PIN_RESET);
	}

	if (faultingDrivers[3]) {
		HAL_GPIO_WritePin(LATCH_D_GPIO_Port, LATCH_D_Pin, GPIO_PIN_RESET);
	}

	if (faultingDrivers[4]) {
		HAL_GPIO_WritePin(LATCH_E_GPIO_Port, LATCH_E_Pin, GPIO_PIN_RESET);
	}

}

void setFault(uint8_t faultingDrivers[5]) {

	if (faultingDrivers[0]) {
		HAL_GPIO_WritePin(LATCH_A_GPIO_Port, LATCH_A_Pin, GPIO_PIN_SET);
	}

	if (faultingDrivers[1]) {
		HAL_GPIO_WritePin(LATCH_B_GPIO_Port, LATCH_B_Pin, GPIO_PIN_SET);
	}

	if (faultingDrivers[2]) {
		HAL_GPIO_WritePin(LATCH_C_GPIO_Port, LATCH_C_Pin, GPIO_PIN_SET);
	}

	if (faultingDrivers[3]) {
		HAL_GPIO_WritePin(LATCH_D_GPIO_Port, LATCH_D_Pin, GPIO_PIN_SET);
	}

	if (faultingDrivers[4]) {
		HAL_GPIO_WritePin(LATCH_E_GPIO_Port, LATCH_E_Pin, GPIO_PIN_SET);
	}

}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{

	if (hspi->Instance == SPI3) {

		if (fcData[sendIdx].opcode == 1) {
		  // Once we receive a launch command from FC, do the following:
		  // Set the launchTime, start drouge and goldfish timers, set the received flag, and set launched to be true.
		  // TIM2->SR &= ~TIM_SR_UIF & TIM5->SR &= ~TIM_SR_UIF are used to clear the UIF flag of the timer handler.
		  // If this is not done, it will start the timer, then immediately call the callback function
		  // signifying that the timer has elapsed and the system will infinetely loop/

			launchTime = HAL_GetTick();		// launchTime is a timestamp that can be used as a reference to determine timeSinceLaunch
			TIM2->SR &= ~TIM_SR_UIF;
			HAL_TIM_Base_Start_IT(&htim2);  // Start Goldfish Timer
			TIM5->SR &= ~TIM_SR_UIF;
			HAL_TIM_Base_Start_IT(&htim5); // Start Drouge Timer
			doubleBuffReco[sendIdx].received = 1; 
			doubleBuffReco[writeIdx].received = 1;
			launched = true;
		}

    // If we get GPS data and we don't have a fresh set of data, 
    // increment the counter indicating fresh set of GPS data
		if (fcData[sendIdx].opcode == 2 && !atomic_load(&gpsEventCount)) {
			atomic_fetch_add(&gpsEventCount, 1);
		}

    // Re-arm the DMA transaction such that RECO-FC comms
		HAL_SPI_TransmitReceive_DMA(&hspi3, (uint8_t*) &doubleBuffReco[sendIdx], (uint8_t*) &fcData[sendIdx], 148);
	}

}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {

	if (htim->Instance == TIM13) {
    // Get a fresh set of magnetometer data and store in magDataStaging
		lis2mdl_get_mag_data(magSPI, magHandler, magDataStaging);

    // Indicate that we have new data
		if (!atomic_load(&magEventCount)) {
			atomic_fetch_add(&magEventCount, 1);
		}

	} else if (htim->Instance == TIM14) {

		// Code to get fresh set of barometer code
		if (convertedTemp) {

		  // If we have already converted temperature data this means that we have also
		  // started converting pressure from the last time this part of the code ran.
		  // Get the raw pressure and calculate pressure
			calculatePress(baroSPI, baroHandler);
			startTemperatureConversion(baroSPI, baroHandler);
			convertedTemp = false;

		  if (!atomic_load(&baroEventCount)) {
			atomic_fetch_add(&baroEventCount, 1);
		  }

		} else {

			// Same as above but swap temperature for presssure and vice versa
			calculateTemp(baroSPI, baroHandler);
			startPressureConversion(baroSPI, baroHandler);
 			convertedTemp = true;

		}
	} else if (htim->Instance == TIM2) {
		// Goldfish Timer

		stage1Enabled = true;
        stage2Enabled = true;
        HAL_GPIO_WritePin(STAGE1_EN_GPIO_Port, STAGE1_EN_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(STAGE2_EN_GPIO_Port, STAGE2_EN_Pin, GPIO_PIN_SET);

        doubleBuffReco[writeIdx].stage1En = true;
        doubleBuffReco[sendIdx].stage1En = true;
        doubleBuffReco[writeIdx].stage2En = true;
        doubleBuffReco[sendIdx].stage2En = true;
        NVIC_SystemReset();

	} else if (htim->Instance == TIM5) {
		// Drouge Timer

        stage1Enabled = true;
        HAL_GPIO_WritePin(STAGE1_EN_GPIO_Port, STAGE1_EN_Pin, GPIO_PIN_SET);

        doubleBuffReco[writeIdx].stage1En = true;
        doubleBuffReco[sendIdx].stage1En = true;

	}
}

char MotionMC_LoadCalFromNVM(unsigned short int datasize, unsigned int *data) {
	return 1;
}

char MotionMC_SaveCalInNVM(unsigned short int datasize, unsigned int *data) {
	return 1;
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
