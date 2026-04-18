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
#include "stdatomic.h"

#include "fault_logic.h"

#ifdef PERF_ANALYSIS
#include "performance.h"
#endif

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MESSAGE_SIZE sizeof(reco_message_t)
#define ADC_NUM_CONVERSIONS 8 // Number of ADC channels in use
#define ADC_RAW_TO_VOLTAGE 0.0008058608f
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc2;
DMA_NodeTypeDef Node_GPDMA1_Channel3;
DMA_QListTypeDef List_GPDMA1_Channel3;
DMA_HandleTypeDef handle_GPDMA1_Channel3;

CRC_HandleTypeDef hcrc;

RTC_HandleTypeDef hrtc;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi3;
DMA_HandleTypeDef handle_GPDMA1_Channel5;
DMA_HandleTypeDef handle_GPDMA1_Channel4;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim8;
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
static void MX_ADC2_Init(void);
static void MX_TIM8_Init(void);
static void MX_TIM6_Init(void);
/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// Double buffer used to hold data that will be received and sent to FC
reco_message_t doubleBuffReco[2] = {0}; // Sent to FC
fc_message_t fcData[2] = {0}; // Received from FC

// Buffer to store ADC conversion results to be copied into reco_message_t
// struct which will be sent to FC
volatile uint16_t ADC_DATA[ADC_NUM_CONVERSIONS];

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
// Setting convertedTemp to true tells the program that
// we have valid temperature data and collect pressure data next
volatile bool convertedTemp = true;

// Will hold the amount of time between launch and system start. 
// Used in timer backups and goldfish timer. Should be launchCmdTime + 1.4s
volatile uint32_t launchTime = 0; 

// Defines atomic variables which determine which of the above buffers
// in doubleBuffReco and fcData are safe to write and which will take in data/send data to
// FC
volatile atomic_uchar sendIdx = 0;  // CPU writes here
volatile atomic_uchar writeIdx  = 1;  // DMA/SPI reads her

// If any of these variables is 1, it means that the EKF should incorporate
// the sensor measurement into the filter
volatile atomic_uchar gpsEventCount = 0;
volatile atomic_uchar magEventCount = 0;
volatile atomic_uchar baroEventCount = 0;

// Time in seconds between EKF iterations
float32_t dt = 0.005f;

// Booleans set by hardware timers to tell RECO that enough time has 
// passed such that we can get data new sensor data
bool magDRDY = false;
bool baroDRDY = false;

// Set to true by either launchProcedure() or EKF flashing from FC
bool resetFilterFlag = false;

// Booleans set when we are ready to get ADC data
bool adcDRDY = false;

// Staging variables that hold sensor meassurements
float32_t magDataStaging[3] = {0};
float32_t llaDataStaging[6] = {0};
float32_t llaBuff[3] = {0};

// Fading Memory Filter Parameters
fmf_first_order_t groundBaro = {0};
fmf_first_order_t groundGPS = {0};
fmf_second_order_t flightBaro = {0};

bool launched = false; 		  // Gets set to true when FC sends the RECO Launch command
bool stage1Enabled = false; // Is set true when EKF/backups determine we are at apogee
bool stage2Enabled = false; // Is set false when bacometer determines we are at 2950 ft
bool fallbackDR = false; 	  // Is used to determine whether EKF blew up and we need to fallback

// Set true by the RECO Launch command. This indicates that we have received launch
// commmand but launch doesn't actually happen till later.
volatile bool launchPending = false;  

// Launch Cmd Timer is the time that we get the RECO Launch command from
// FC since power on of RECO in miliseconds
volatile uint32_t launchCmdTime = 0;  

// Number of seconds since launch. Used to ensure in case of a loop that fails to
// exit for some reason that the parachutes will still be able to deploy.
volatile uint32_t drouge_timer_seconds = 0;

// Number of seconds after launch that the drouge timer should go off at.
volatile uint32_t drouge_timer_set = 109;

// Allows us to determine whether we want EKF to have authority to deploy parachutes
volatile bool ekf_enabled = true;

// If EKF says deploy below this amount of time (in ms) after launch then
// don't trust it
volatile uint32_t ekfLockoutTimer = 52000;

// Tells EKF whether to consider barometer measurement
volatile float32_t lockoutVelocity = 0;

// Structure that holds data concerning performance data
// about RECO functions
#ifdef PERF_ANALYSIS
perf_t perf = {0};
perf_t* perf_data = &perf;
#endif

void gather_mag_data(void);
void gather_baro_data(void);
void gather_adc_data(void);
void reset_filter(arm_matrix_instance_f32* xFilter,
				  arm_matrix_instance_f32* PFilter,
				  arm_matrix_instance_f32* QFilter,
				  arm_matrix_instance_f32* RFilter,
				  arm_matrix_instance_f32* RqFilter,
				  fmf_first_order_t* groundBaro,
				  fmf_first_order_t* groundGPS,
				  fmf_second_order_t* flightBaro,
				  float32_t* Rb);

void launch_procedure(arm_matrix_instance_f32* xFilter,
					  arm_matrix_instance_f32* PFilter,
					  arm_matrix_instance_f32* QFilter,
					  arm_matrix_instance_f32* RFilter,
					  arm_matrix_instance_f32* RqFilter,
					  fmf_first_order_t* groundBaro,
					  fmf_first_order_t* groundGPS,
					  fmf_second_order_t* flightBaro,
					  float32_t* Rb);

inline uint32_t get_system_time(void);
inline uint32_t get_precise_time(void);
void delay(uint32_t wait);

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
  MX_ADC2_Init();
  MX_TIM8_Init();
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */
  
  /*
   * Used to initialize performance analysis library. When PERF_ANALYSIS
   * is not defined, all macros, indicated by the prefix PERF, expand to nothing.
   * In fact, no artifact of the performance library exists in the final binary
   * unless PERF_ANALYSIS is defined.
   *
   * Look to /Core/Inc/performance.c and /Core/Src/performance.h for more information
   * on the library.
   */
  #ifdef PERF_ANALYSIS
  // Unlocks the DWT register in order to count cycles
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  ITM->LAR = 0xC5ACCE55;
  DWT->CYCCNT = 0;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

  // Initializes the performance struct 
  perf_init(perf_data);
  #endif

  // Start the RECO global timer.
  __HAL_TIM_SET_COUNTER(&htim2, 0);
  HAL_TIM_Base_Start(&htim2);

  // Calibrate ADC2. Must be done before we start doing conversions
  // with the ADC
  HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED);


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
  getCurrTempPressure(baroSPI, baroHandler);
  startPressureConversion(baroSPI, baroHandler);

  // Variables that check that the magnetometer and the accelerometer 
  // are functioning correctly 
  uint8_t mag_who_am_i = 0;
  uint8_t imu_who_am_i = 0;

  // Reads the WHO_AM_I Register
  readIMUSingleRegister(imuSPI, IMU_WHO_AM_I, &imu_who_am_i);

  // Read magnetometer config registers
  uint8_t cfgReg[] = {0, 0, 0};
  lis2mdl_read_multiple_reg(magSPI, MAG_CFG_REG_A, MAG_CFG_REG_C, cfgReg);

  // Read IMU config registers
  uint8_t cfgRegIMU[] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
  readIMUMultipleRegisters(imuSPI, IMU_CTRL1_XL, IMU_CTRL10_C, cfgRegIMU);

  // Read magnetometer WHO AM I register
  lis2mdl_read_single_reg(magSPI, MAG_WHO_AM_I, &mag_who_am_i);

  // Initialize fault detection and handling logic
  fault_logic_init();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  // Matrices that are used to run the filter
  // x is the state vector. R is the uncertainity in the measurements from the sensors.
  // Q describes how uncertain we are in the modeling of the system dynamics.
  // P describes the overall uncertainity in the current state of the filter.'
  // Look to compute_initial_consts.c for more info on all these matrices.

  CREATE_MAT_F32(xPrev, 22, 1);
  CREATE_MAT_F32(PPrev, 21, 21);
  CREATE_MAT_F32(xPlus, 22, 1);
  CREATE_MAT_F32(PPlus, 21, 21);
  CREATE_MAT_F32(H, 3, 21);
  CREATE_MAT_F32(Hb, 1, 21);
  CREATE_MAT_F32(R, 3, 3);
  CREATE_MAT_F32(Rq, 3, 3);
  CREATE_MAT_F32(Q, 12, 12);
  CREATE_MAT_F32(magI, 3, 1);
  float32_t Rb;

  // Initializes all EKF matrices to initial flight values
  ekf_init(&xPrev, &PPrev, &H, &Hb, &R, &Rq, &Q, &magI, &groundBaro, &groundGPS, &flightBaro, &Rb, dt);

	// Contains the measurements from our sensors which are accelerometer,
	// gyroscope, GPS, and magnetometer.
	arm_matrix_instance_f32 aMeas, wMeas, llaMeas, magMeas;

	// EKF Test Suite
	#ifdef EKF_TEST_SUITE
	volatile bool test1 = test_what();
	volatile bool test2 = test_ahat();
	volatile bool test3 = test_qdot();
	volatile bool test4 = test_lla_dot();
	volatile bool test5 = test_compute_vdot();
	volatile bool test6 = test_compute_dwdp();
	volatile bool test7 = test_compute_dwdv();
	volatile bool test8 = test_compute_dpdot_dp();
	volatile bool test9 = test_compute_dpdot_dv();
	volatile bool test10 = test_compute_dvdot_dp();
	volatile bool test11 = test_compute_dvdot_dv();
	volatile bool test12 = test_compute_F();
	volatile bool test13 = test_compute_G();
	volatile bool test14 = test_compute_Pdot();
	volatile bool test15 = test_integrate();
	volatile bool test16 = test_propogate();
	volatile bool test17 = test_right_divide();
	test_eig();
	volatile bool test19 = test_nearest_PSD();
	volatile bool test20 = test_update_GPS();
	test_update_mag();
	volatile bool test22 = test_update_baro();
	volatile bool test23 = test_update_EKF();
	test_p2alt();
	#endif

	 // Performance Tests
	 #ifdef PERF_ANALYSIS
	 test_baro_update_perf(perf_data);
	 #endif

	 // Update Booleans for FMF
	 bool update_gps_fmf = fcData[writeIdx].valid;
	 bool update_baro_fmf = atomic_load(&baroEventCount);

	// Variables that measure how much time we have had a negative downwards velocity
	uint32_t drougeAltStart = UINT32_MAX; // EKF Drouge
	uint32_t baroAltStart = UINT32_MAX;   // Barometer Main

	// The amount of time that has elapsed since launch in ms
	uint32_t elapsedTime = 0;

	// Number of iterations of EKF
	uint32_t numIterations = 0;
  
  // HTIM13 and HTIM14 operate at 100 Hz and 400 Hz respectively and call
  // the HAL_TIM_PeriodElapsedCallback() function once the period has ellapsed.
	HAL_TIM_Base_Start_IT(&htim13); // Timer for magnetometer
	HAL_TIM_Base_Start_IT(&htim14); // Timer for barometer

	// Start ADC2 in DMA transaction mode. Start the timer 8 (TIM8).
    // ADC2 uses rising edge of TIM8 to start conversions on all eight channels.
    // Once conversions are completed, ADC notifies the DMA which then transfers data
    // to the ADC_DATA variable
	HAL_ADC_Start_DMA(&hadc2, (uint32_t*) ADC_DATA, ADC_NUM_CONVERSIONS);
	HAL_TIM_Base_Start(&htim8);

	// Start up the DMA transcation which communicates with FC
	HAL_SPI_TransmitReceive_DMA(&hspi3,
								(uint8_t*) &doubleBuffReco[sendIdx],
								(uint8_t*) &fcData[sendIdx],
								MESSAGE_SIZE);

	#ifdef PERF_ANALYSIS
	volatile uint32_t times[2];
	volatile uint32_t cycles[2];
	#endif

	while (1)
	{

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	PERF_START(1);
	uint32_t iterationStartTime = get_precise_time();

	#ifdef PERF_ANALYSIS
	times[0] = iterationStartTime;
	cycles[0] = DWT->CYCCNT;
	#endif

    // Launch Pending is set by receiving a RECO Launch Command
    if (launchPending) {
    	launch_procedure(&xPrev, &PPrev, &Q, &R, &Rq, &groundBaro, &groundGPS, &flightBaro, &Rb);
    	launchPending = false; //  Make sure that this if statement can never be run again
    }

    if (launched) {
      elapsedTime =  get_system_time() - launchTime;
    }

    // If magnetometer data is ready, gather it.
    if (magDRDY) {
    	gather_mag_data();
    	magDRDY = false;
    }

    // If barometer data is ready, gather it.
    if (baroDRDY) {
    	gather_baro_data();
    	baroDRDY = false;
    }

    // If ADC Data is ready, gather it
    if (adcDRDY) {
    	gather_adc_data();
    	adcDRDY = false;
    }

    // If the resetFilterFlag is set, reset the filter
    if (resetFilterFlag) {
    	reset_filter(&xPrev, &PPrev, &Q, &R, &Rq, &groundBaro, &groundGPS, &flightBaro, &Rb);
    	resetFilterFlag = false;
    }

    // Used to determine whether to update the fading memory filter
	update_gps_fmf = fcData[writeIdx].valid;
	update_baro_fmf = atomic_load(&baroEventCount);

    // Check for faults and solve existing faults on all recovery drivers
    check_fault_pins(get_system_time(), &doubleBuffReco[writeIdx]);

    // Get data from IMU
    PERF_START(3);
    getIMUData(imuSPI, imuHandler, doubleBuffReco[writeIdx].angularRate, doubleBuffReco[writeIdx].linAccel);
    PERF_END(PERF_GATHER_IMU, 3);

    // Magnetomer Data copied over for the filter
    memcpy(doubleBuffReco[writeIdx].magData, magDataStaging, 3*sizeof(float32_t));

    // Barometer Data for filter
    doubleBuffReco[writeIdx].pressure = baroHandler->pressure; // Due to it simply writing a word it is atomic

    // Temperature Data for logging by FC
    doubleBuffReco[writeIdx].temperature = baroHandler->temperature; // Due to it simply writing a word it is atomic

    // GPS Data for filter
    memcpy(llaBuff, fcData[writeIdx].gpsLLA, 3*sizeof(float32_t));

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
			   &PPlus, xPlusData, PPlusData, &fcData[writeIdx], &fallbackDR, numIterations PERF_PASS);

    // Read the status of the RBF
    doubleBuffReco[writeIdx].rbf_enabled = HAL_GPIO_ReadPin(VRBF_GPIO_Port, VRBF_Pin);

    // Log to FC whether EKF has blown up (is atomic due to it being a byte write)
    doubleBuffReco[writeIdx].blewUp = fallbackDR;

    float32_t currAltitude = xPlus.pData[6]; // The altitude of the current state
    float32_t prevAltitude = xPrev.pData[6]; // The altitude of the previously computed state

    float32_t deltaAlt = currAltitude - prevAltitude; // The difference between altitudes
    lockoutVelocity = deltaAlt / dt;

    /*
    * For drouge to deploy, the following must be true:
    * 		1. Altitude has been decreasing for six seconds
    * 		2. Vehicle has launched (set by the reco_enabled command from FC)
    * 		3. We must not have fallen back to dead reckoning
    * 		4. EKF must be enabled by operator
    */
    if (drougeChuteCheck(deltaAlt, &drougeAltStart, get_system_time()) && launched && !fallbackDR && ekf_enabled) {

    	// If EKF activates before 52000 miliseconds have passed then fall back to dead reckoning.
       if (elapsedTime < ekfLockoutTimer) {
    	   fallbackDR = true;
       } else {
           stage1Enabled = true;
           doubleBuffReco[sendIdx].stage1En = true;
           doubleBuffReco[writeIdx].stage1En = true;
           HAL_GPIO_WritePin(STAGE1_EN_GPIO_Port, STAGE1_EN_Pin, GPIO_PIN_SET);
       }
    }

    PERF_START(2);
    // Takes in a pressure and returns an altitude
    float32_t baroAlt = pressure_altimeter_corrected(doubleBuffReco[writeIdx].pressure);
    PERF_END(PERF_P2ALT, 2);

    if (launched) {
    	// Use the second order filter in flight
    	// Don't run fading memory for GPS afterwards as it is only used
    	// to determine the offsets for initializations

    	if (update_baro_fmf) {
    		fmf_second_order(&flightBaro, baroAlt);
            doubleBuffReco[writeIdx].fading_memory_baro = flightBaro.currentStateEst;
            doubleBuffReco[sendIdx].fading_memory_baro = flightBaro.currentStateEst;
    	}

    } else {

    	// Use the first order filter on the ground to help determine
    	// biases in GPS and barometer

    	if (update_gps_fmf) {
            doubleBuffReco[writeIdx].fading_memory_gps  = fmf_first_order(&groundGPS, fcData[writeIdx].gpsLLA[2]);
            doubleBuffReco[sendIdx].fading_memory_gps  = fmf_first_order(&groundGPS, fcData[writeIdx].gpsLLA[2]);
    	}

    	if (update_baro_fmf) {
            doubleBuffReco[writeIdx].fading_memory_baro = fmf_first_order(&groundBaro, baroAlt);
            doubleBuffReco[sendIdx].fading_memory_baro = fmf_first_order(&groundBaro, baroAlt);
    	}
    }

    /*
    * For main to deploy, the following must occur:
    * 		1. Altitude is less than 2950 ft for 1 second
    * 		3. Vehicle has launched
    * 		4. Elapsed time is greater than ekfLockoutTimer ms
    */
    if (mainChuteCheck(baroAlt, &baroAltStart, get_system_time()) && launched && (elapsedTime > ekfLockoutTimer)) {
        stage2Enabled = true;
        doubleBuffReco[sendIdx].stage2En = true;
        doubleBuffReco[writeIdx].stage2En = true;
        HAL_GPIO_WritePin(STAGE1_EN_GPIO_Port, STAGE1_EN_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(STAGE2_EN_GPIO_Port, STAGE2_EN_Pin, GPIO_PIN_SET);
    }

    // Set the current state to the previous state
    copy_mat_f32(&xPlus, &xPrev);
    PERF_START(5);
    copy_mat_f32(&PPlus, &PPrev);
    PERF_END(PERF_21x21_MEMCPY, 5);
    memcpy(&doubleBuffReco[writeIdx], xPrev.pData, 22*sizeof(float32_t));

    // Switch the sending and writing buffer for RECO-FC commnication.
    __disable_irq();
    atomic_fetch_xor(&writeIdx, 1);
    atomic_fetch_xor(&sendIdx, 1);
    HAL_SPI_TransmitReceive_DMA(&hspi3, (uint8_t*) &doubleBuffReco[sendIdx], (uint8_t*) &fcData[sendIdx], MESSAGE_SIZE);
    __enable_irq();

    // Ensure that at least dt seconds have occured
	#ifndef PERF_ANALYSIS
    while ((get_precise_time() - iterationStartTime) < (uint32_t) (dt * 10000)) {
      continue;   // Skip entire loop until delay expires
    }
	#endif

    // Iteration Number Increment
	  numIterations++;


    PERF_END(PERF_MAIN_LOOP, 1);
    
    #ifdef PERF_ANALYSIS
    float32_t mainLoopTime = (float_t) ((get_precise_time() - iterationStartTime) / 10.0f);
    perf_main_loop_time(perf_data,  mainLoopTime);
    times[1] = get_precise_time();
	cycles[1] = DWT->CYCCNT;
    perf_data->indexNum = numIterations;
    perf_data->initialized = true;
    __asm__("nop"); // Necessary so we can have a breakpoint at the end of the loop
    #endif
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI
                              |RCC_OSCILLATORTYPE_CSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
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
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */

  /** Common config
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  hadc2.Init.LowPowerAutoWait = DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.NbrOfConversion = 8;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConv = ADC_EXTERNALTRIG_T8_TRGO;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc2.Init.DMAContinuousRequests = ENABLE;
  hadc2.Init.SamplingMode = ADC_SAMPLING_MODE_NORMAL;
  hadc2.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc2.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_247CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = ADC_REGULAR_RANK_4;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = ADC_REGULAR_RANK_5;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_6;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_14;
  sConfig.Rank = ADC_REGULAR_RANK_7;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_15;
  sConfig.Rank = ADC_REGULAR_RANK_8;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

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
    HAL_NVIC_SetPriority(GPDMA1_Channel3_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(GPDMA1_Channel3_IRQn);
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
  htim2.Init.Prescaler = 25000-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967294;
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
  htim5.Init.Prescaler = 25000 - 1;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 11999999;
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
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 25000 - 1;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 9999;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief TIM8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM8_Init(void)
{

  /* USER CODE BEGIN TIM8_Init 0 */

  /* USER CODE END TIM8_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 250-1;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 4999;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim8, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, MAG_NCS_Pin|BAR_NCS_Pin|IMU_NCS_Pin|STAGE1_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, STAGE2_EN_Pin|SEL_2_Pin|SEL_1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LATCH_A_Pin|LATCH_B_Pin|LATCH_C_Pin|LATCH_D_Pin
                          |LATCH_E_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : FLT_C2_Pin FLT_D2_Pin FLT_E2_Pin FLT_D1_Pin
                           FLT_A1_Pin FLT_B1_Pin FLT_E1_Pin */
  GPIO_InitStruct.Pin = FLT_C2_Pin|FLT_D2_Pin|FLT_E2_Pin|FLT_D1_Pin
                          |FLT_A1_Pin|FLT_B1_Pin|FLT_E1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : FLT_B2_Pin FLT_A2_Pin */
  GPIO_InitStruct.Pin = FLT_B2_Pin|FLT_A2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);

  /*Configure GPIO pins : MAG_NCS_Pin BAR_NCS_Pin IMU_NCS_Pin */
  GPIO_InitStruct.Pin = MAG_NCS_Pin|BAR_NCS_Pin|IMU_NCS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : STAGE2_EN_Pin SEL_2_Pin SEL_1_Pin */
  GPIO_InitStruct.Pin = STAGE2_EN_Pin|SEL_2_Pin|SEL_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : FLT_C1_Pin VRBF_Pin IMU_INT_Pin */
  GPIO_InitStruct.Pin = FLT_C1_Pin|VRBF_Pin|IMU_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : STAGE1_EN_Pin */
  GPIO_InitStruct.Pin = STAGE1_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(STAGE1_EN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LATCH_A_Pin LATCH_B_Pin LATCH_C_Pin LATCH_D_Pin
                           LATCH_E_Pin */
  GPIO_InitStruct.Pin = LATCH_A_Pin|LATCH_B_Pin|LATCH_C_Pin|LATCH_D_Pin
                          |LATCH_E_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

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

void reset_filter(arm_matrix_instance_f32* xFilter,
				  arm_matrix_instance_f32* PFilter,
				  arm_matrix_instance_f32* QFilter,
				  arm_matrix_instance_f32* RFilter,
				  arm_matrix_instance_f32* RqFilter,
				  fmf_first_order_t* groundBaro,
				  fmf_first_order_t* groundGPS,
				  fmf_second_order_t* flightBaro,
				  float32_t* Rb) {

	// set x0, P, Q, R
	get_x0(xFilter);
	get_P0(PFilter);
	get_Q0(QFilter);
	get_R0(RFilter);
	get_Rq0(RqFilter);

	// Initialize FMF by setting our state estimates
	// to the altitude in our initial state vector
	// and set the gains for each of the FMF

	// xPrev.pData[6] is our initial state vector altitude
	float32_t initialAltitude = xFilter->pData[6];

	fmf_first_order_init(groundBaro, initialAltitude, get_initial_baro_ground_beta());
	fmf_first_order_init(groundGPS, initialAltitude, get_initial_gps_ground_beta());
	fmf_second_order_init(flightBaro, initialAltitude, get_initial_baro_flight_beta(), dt);

	*Rb = get_Rb0();
}

void launch_procedure(arm_matrix_instance_f32* xFilter,
					  arm_matrix_instance_f32* PFilter,
					  arm_matrix_instance_f32* QFilter,
					  arm_matrix_instance_f32* RFilter,
					  arm_matrix_instance_f32* RqFilter,
					  fmf_first_order_t* groundBaro,
					  fmf_first_order_t* groundGPS,
					  fmf_second_order_t* flightBaro,
					  float32_t* Rb) {

	 // After receiving a RECO launch command, the rocket doesn't start actually
	 // moving until 1.5 to 3 seconds later. By waiting 1.4 seconds, we ensure that
	 // EKF runs at least 0.1 seconds
	while ((get_system_time() - launchCmdTime) < 1400) {
		continue;   // Skip entire loop until delay expires
	}

	// We are ready to start the flight EKF
	launched = true;              	  //  We have launched
	launchTime = get_system_time();   //  Set the time of launch for elapsed time

	// TIM5->SR &= ~TIM_SR_UIF are used to clear the UIF flag of the timer handler.
	// If we don't this it will immediately set off the goldfish and drouge timer.
	TIM5->SR &= ~TIM_SR_UIF;
	HAL_TIM_Base_Start_IT(&htim5); // Start goldfish timer

	TIM6->SR &= ~TIM_SR_UIF;
	HAL_TIM_Base_Start_IT(&htim6); // Start drouge timer

	// Reset the state vectors and the covariance matrix to pre-flight state
	reset_filter(xFilter, PFilter, QFilter, RFilter, RqFilter, groundBaro, groundGPS, flightBaro, Rb);
}

void gather_mag_data(void) {
	PERF_START(1);
    // Get a fresh set of magnetometer data and store in magDataStaging
	lis2mdl_get_mag_data(magSPI, magHandler, magDataStaging);

    // Indicate that we have new data
	if (!atomic_load(&magEventCount)) {
		atomic_fetch_add(&magEventCount, 1);
	}
	PERF_END(PERF_GATHER_MAG, 1);
}

void gather_baro_data(void) {
	PERF_START(1);
	// Code to get fresh set of barometer code
	if (convertedTemp) {

	  // If we have already converted temperature data this means that we have also
	  // started converting pressure from the last time this part of the code ran.
	  // Get the raw pressure and calculate pressure
		calculatePress(baroSPI, baroHandler);
		startTemperatureConversion(baroSPI, baroHandler);
		convertedTemp = false;

	  if (!atomic_load(&baroEventCount) && doubleBuffReco[writeIdx].pressure > 1100.0f && fabs(lockoutVelocity) < 350.0f) {
		  atomic_fetch_add(&baroEventCount, 1);
	  }

	} else {

		// Same as above but swap temperature for presssure and vice versa
		calculateTemp(baroSPI, baroHandler);
		startPressureConversion(baroSPI, baroHandler);
			convertedTemp = true;

	}
	PERF_END(PERF_GATHER_BARO, 1);
}

void gather_adc_data(void) {
	// The mapping between the index, the channel number of ADC 2, and the pin name
	// in STM32CubeIDE, and the pin name in Altium.
	// idx = 0: Channel 0: VREF_CH2_DR1 : VREF-FB2-A
	// idx = 1: Channel 1: SNS_2 		: SNS-2
	// idx = 2: Channel 5: VREF_CH1_DR2	: VREF-FB2-B
	// idx = 3: Channel 8: VSNS_3V3		: VSNS-3V3
	// idx = 4: Channel 9: VREF_CH2_DR2	: VREF-FB2-B
	// idx = 5: Channel 13: VSNS_24V	: VSNS-24V
	// idx = 6: Channel 14: SNS_1		: SNS-1
	// idx = 7: Channel 15: VREF_CH1_DR1: VREF-FB1-A

	// We multiply by 11 for the v_rail_24v because that is it's gain due to a resistor
	// divider.
	doubleBuffReco[writeIdx].vref_ch2_dr1 = ADC_RAW_TO_VOLTAGE * (float32_t) ADC_DATA[0];
	doubleBuffReco[writeIdx].sns2_current = ADC_RAW_TO_VOLTAGE * (float32_t) ADC_DATA[1];
	doubleBuffReco[writeIdx].vref_ch1_dr2 = ADC_RAW_TO_VOLTAGE * (float32_t) ADC_DATA[2];
	doubleBuffReco[writeIdx].v_rail_3v3   = ADC_RAW_TO_VOLTAGE * (float32_t) ADC_DATA[3] * 2;
	doubleBuffReco[writeIdx].vref_ch2_dr2 = ADC_RAW_TO_VOLTAGE * (float32_t) ADC_DATA[4];
	doubleBuffReco[writeIdx].v_rail_24v   = ADC_RAW_TO_VOLTAGE * (float32_t) ADC_DATA[5] * 11;
	doubleBuffReco[writeIdx].sns1_current = ADC_RAW_TO_VOLTAGE * (float32_t) ADC_DATA[6];
	doubleBuffReco[writeIdx].vref_ch1_dr1 = ADC_RAW_TO_VOLTAGE * (float32_t) ADC_DATA[7];

	//  Debugging for GUI
//		printf("Time: %d\n", get_system_time() / 1000);
//		printf("Write Idx: %d\n", (uint8_t) writeIdx);
//		printf("24V Rail: %f V\n", doubleBuffReco[writeIdx].v_rail_24v);
//		printf("3V3 Rail: %f V\n", doubleBuffReco[writeIdx].v_rail_3v3);

	// By toggling the SEL pins, we can now measure the current on the other channel.
	// SEL = LOW, measure current on channel 1. SEL = HIGH, measure current on channel 2
	// 1 and 2 refer to particular recovery driver.

	// RECO Microcontroller A: Driver A
	// RECO Microcontroller B: Driver B and D
	// RECO Microcontroller C: Driver C E
	HAL_GPIO_TogglePin(SEL_1_GPIO_Port, SEL_1_Pin);
	HAL_GPIO_TogglePin(SEL_2_GPIO_Port, SEL_2_Pin);
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi) {

	if (hspi->Instance == SPI3) {

		if (fcData[sendIdx].opcode == LAUNCH) {
		  // Once we receive a launch command from FC, do the following:
		  //    1. Set the current time since power on of RECO for the launchCmdTime
		  //    2. Set the launch pending flag to be true
		  //    3. Set the received bit in both messages to RECO to be true

		  launchCmdTime = get_system_time();   // Record when opcode arrived
		  launchPending = true;            // Arm delayed launch. Next iteration of the main loop will run launch_procedure()

		  // Set both buffers to have their received bit to 1
			doubleBuffReco[sendIdx].received = 1; 
			doubleBuffReco[writeIdx].received = 1;
		}

    // If we get GPS data and we don't have a fresh set of data, 
    // increment the counter indicating fresh set of GPS data
		if (fcData[sendIdx].opcode == DATA && !atomic_load(&gpsEventCount)) {
			atomic_fetch_add(&gpsEventCount, 1);
		}

		// If we get a message from FC then we reset RECO
		// by powering off and turning it back on
		if (fcData[sendIdx].opcode == RESET_STM) {
			NVIC_SystemReset();
		}

		if (fcData[sendIdx].opcode == PROCESS) {
			process_noise_t* newProcessNoise = (process_noise_t*) &fcData[sendIdx];

      // Set the initial values for all sub-matrices
			set_nu_gv0(newProcessNoise->nu_gv_mat);
			set_nu_gu0(newProcessNoise->nu_gu_mat);
			set_nu_av0(newProcessNoise->nu_av_mat);
			set_nu_au0(newProcessNoise->nu_au_mat);

      // Compute the new initial Q from the sub-matrices set in the above functions
			compute_Q0(dt);

      // Reset the filter to get the new Q matrix
			resetFilterFlag = true;
		}

		if (fcData[sendIdx].opcode == MEASUREMENT) {
			measurement_noise_t* newMeasurementNoise = (measurement_noise_t*) &fcData[sendIdx];
			set_R0(newMeasurementNoise->gpsNoiseMatrix); // Set GPS Noise Matrix
			set_Rb0(newMeasurementNoise->barometer_noise); // Set Barometer Noise
			resetFilterFlag = true; // Tell RECO to restart the filter on next iteration and to grab new initial values
		}

		if (fcData[sendIdx].opcode == STATE_VECTOR) {
			state_vector_message_t* newStateVector = (state_vector_message_t*) &fcData[sendIdx];
			set_x0((float32_t*) &newStateVector->state_vector); // Set the inital state vector
			resetFilterFlag = true;
		}

		if (fcData[sendIdx].opcode == COVARIANCE) {
			initial_covariance_t* newCovarianceVector = (initial_covariance_t*) &fcData[sendIdx];
			set_uncertanties(newCovarianceVector->att_unc0,
							 newCovarianceVector->pos_unc0,
							 newCovarianceVector->vel_unc0,
							 newCovarianceVector->gbias_unc0,
							 newCovarianceVector->abias_unc0,
							 newCovarianceVector->gsf_unc0,
							 newCovarianceVector->asf_unc0); // Set the initial uncertanties
			compute_P0(); // Compute the new covariance matrix with the new uncertanties
			resetFilterFlag = true;
		}

		//
		if (fcData[sendIdx].opcode == TIMER) {
			timer_values_t* newTimerValues = (timer_values_t*) &fcData[sendIdx];
			ekf_enabled = !newTimerValues->drougeTimerEnable;
			drouge_timer_set = (uint32_t) newTimerValues->drougeTimer;
			resetFilterFlag = true;
		}


		if (fcData[sendIdx].opcode == ALTIMETER) {
			altimeter_offsets_t* newAltimetersOffset = (altimeter_offsets_t*) &fcData[sendIdx];
			ekfLockoutTimer = newAltimetersOffset->ekf_lockout; //
			setHeightOffsetAltimeter(newAltimetersOffset->hOffsetAlt);
			setHeightOffsetFilter(newAltimetersOffset->hOffsetFilter);
			set_initial_gains(newAltimetersOffset->ground_baro_fmf_parameter,
							  newAltimetersOffset->flight_baro_fmf_parameter,
							  newAltimetersOffset->ground_gps_fmf_parameter);
			resetFilterFlag = true;
		}

    // Re-arm the DMA transaction such that RECO-FC comms
		HAL_SPI_TransmitReceive_DMA(&hspi3, (uint8_t*) &doubleBuffReco[sendIdx], (uint8_t*) &fcData[sendIdx], MESSAGE_SIZE);
	}

}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {

	//	  TIM14:  Timer used to get barometer data (updates at 400 Hz)
	//	  TIM13:  Timer used to get magnetometer data (updates at 100 Hz)
	//	  TIM5:   Goldfish Timer (goes off at 20 minutes)
	//	  TIM2:   Global Timer used to reference get_system_time(). Has a 10 Khz update per tick
	//	  TIM8:   Timer to tell DMA when to capture ADC data
	//	  TIM6:   Timer used for backup drouge timer. Updates drouge_timer_seconds once every second

	if (htim->Instance == TIM13) {
		magDRDY = true;
	} else if (htim->Instance == TIM14) {
		baroDRDY = true;
	} else if (htim->Instance == TIM5) {
    // Goldfish Timer

		stage1Enabled = true;
        stage2Enabled = true;

        // Set off any remaining ematches to safe the vehicle
        HAL_GPIO_WritePin(STAGE1_EN_GPIO_Port, STAGE1_EN_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(STAGE2_EN_GPIO_Port, STAGE2_EN_Pin, GPIO_PIN_SET);

        // Tell FC we have enabled both of stage1 and stage2 ematches
        doubleBuffReco[writeIdx].stage1En = true;
        doubleBuffReco[sendIdx].stage1En = true;
        doubleBuffReco[writeIdx].stage2En = true;
        doubleBuffReco[sendIdx].stage2En = true;

        // Delay for 30 seconds to ensure all other uCs have also enabled
        delay(30000);

        // Reset RECO
        NVIC_SystemReset();

	} else if (htim->Instance == TIM6) {
		// Drouge Timer

		drouge_timer_seconds++; // This else if block runs every second so increment the number of seconds

		if (drouge_timer_seconds > drouge_timer_set) {
			stage1Enabled = true;

	        // Set off any remaining ematches to safe the vehicle
	        HAL_GPIO_WritePin(STAGE1_EN_GPIO_Port, STAGE1_EN_Pin, GPIO_PIN_SET);

	        // Tell FC we have enabled both of stage1 and stage2 ematches
	        doubleBuffReco[writeIdx].stage1En = true;
	        doubleBuffReco[sendIdx].stage1En = true;
		}
	}
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
	adcDRDY = true;
}

/**
 * @brief Get coarse system time based on TIM2 counter.
 *
 * This function returns a scaled version of the TIM2 counter value,
 * effectively providing a lower-resolution time base derived from
 * the hardware timer.
 *
 * The TIM2 peripheral is configured as:
 * - 32-bit up-counter
 * - Prescaler = 25000 - 1
 * - Auto-reload = 0xFFFFFFFF (max period)
 *
 * The returned value is computed as:
 *     time = TIM2_CNT / 10
 *
 * This introduces an additional software division factor, reducing
 * resolution while extending the effective time range per unit.
 *
 * @note Currently, with the 250 MHz timer clock:
 *       - Timer tick = 250 MHz / 25000 = 10 kHz → 100 µs per increment
 *       - After division by 10 → 1 ms per returned unit
 *
 */
uint32_t get_system_time(void) {
	return __HAL_TIM_GET_COUNTER(&htim2) / 10;
}

/**
 * @brief Get high-resolution (0.1 ms resolution) timestamp from TIM2.
 *
 * Returns the raw 32-bit counter value from TIM2 without scaling.
 * This provides the highest available time resolution for profiling
 * or precise time measurements.
 *
 * @note Timer configuration:
 *       - 32-bit up-counter
 *       - Prescaler = 25000 - 1
 *       - Tick frequency depends on timer clock
 *
 * @note With a 250 MHz timer clock:
 *       - Tick frequency = 10 kHz (100 µs resolution)
 *
 * @note The counter will overflow after approximately:
 *       (2^32 / tick_frequency) seconds
 *       ≈ 4294967296 / 10000 ≈ 429496 seconds (~119 hours)
 *
 * @return Raw TIM2 counter value.
 */
uint32_t get_precise_time(void) {
	return __HAL_TIM_GET_COUNTER(&htim2);
}

/**
 * @brief Blocking delay using TIM2-based system time.
 *
 * This function implements a busy-wait delay using a coarse time base
 * derived from the TIM2 hardware timer via get_system_time().
 *
 * @param wait Duration to delay, in units of miliseconds.
 *
 * @note Time Base:
 *       get_system_time() returns TIM2_CNT / 10.
 *
 *       With TIM2 configured as:
 *       - 32-bit up-counter
 *       - Prescaler = 25000 - 1
 *       - Timer clock = 250 MHz
 *
 *       Then:
 *       - Timer tick frequency = 250 MHz / 25000 = 10 kHz (100 µs per tick)
 *       - After division by 10 → 1 ms resolution
 *
 *       Therefore, @p wait is approximately in milliseconds.
 *
 * @note Overflow Safety:
 *       This implementation correctly handles 32-bit timer wraparound
 *       due to unsigned integer arithmetic:
 *
 *           (current_time - start_time)
 *
 *       remains valid even if the counter overflows during the delay.
 *
 * @warning This is a blocking delay and will busy-wait the CPU.
 *          It should not be used in time-critical or low-power applications.
 */
void delay(uint32_t wait)
{
    uint32_t tickstart = get_system_time();

    while ((get_system_time() - tickstart) < wait)
    {
    }
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
