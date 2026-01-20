## Introduction
- RECO is the project that is used on Vespula to recover the rocket. The software portion of RECO handles the following:
	1. Gathering data from sensors (magnetometer, barometer, and IMU)
	2. Feeding them into an Extended Kalman Filter (EKF) which is used to determine our position.
	3. Launch parachutes, drogue and main, when we are below a certain altitude (in our case 3000 ft).
	4. Send our data to the flight computer (FC) for data analysis purposes and to receive GPS data as well

## Code
### Platform Overview
- RECO software is written in C, and is ran on an STM32H573RIT6. The RECO board has three of these microcontrollers, each having their own set of sensors, in order to ensure the robustness of the system.
	- Despite all other YJSP AVI projects being done in Rust, C was chosen for this project as Rust implementation for the STM32H5xx family is missing some key features (such as SPI communication) that are necessary for the system to work.

### Code/Directory Layout
- The RECO project can be broken into two parts:
	1. The sensor firmware drivers 
	2. The EKF implementation
	\* **Note**: C, unlike most other languages, splits the declaration and definition of functions and some variables into two types of files called *header files* (.h) and *source files* (.c) respectively. If you find something that isn't defined/declared in a source file, it most likely is defined in its associated header file.

- The sensor firmware drivers are stored in the `/Inc` and `/Src` folders:
	- IMU Code: `/Inc/ASM330LHGB1.c` and `/Inc/ASM330LHGB1.h`
	- Barometer Code: `/Inc/MS5611.c` and `/Inc/MS5611.h`
	- Magnetometer Code: `/Inc/LIS2MDL.c` and `/Inc/LIS2MDL.c`
		- The LIS3MDL is an older magnetometer used on older revisions of the board. This code will not run during flight

- The EKF implementation is located under `/EKF/Inc` and `/EKF/Src`. The EKF code leverages two external libraries:
	1. ARM CMSIS-DSP (basic linear algebra).
		- Documentation can be found at: https://arm-software.github.io/CMSIS_5/DSP/html/index.html
	2. CControl (complex linear algebra) 
		- In reality, this is only ever used to determine the eigenvalues and eigenvectors of a matrix which is done once in `EKF/Src/nearestPSD.c`

- The EKF code (under `EKF/Src/`) is structured as follows:
	1. `altimeter_pressure.c`: Contains all code dealing with estimating altitude from barometer pressure reading for **EKF**. This includes the polynomial fit for the aforementioned conversion along with some matrix code
	2. `filter_pressure.c`: Contains all code dealing with estimating altitude from barometer pressure reading for **main parachutes**. This includes the polynomial fit for the aforementioned conversion.
	3. `compute_hats.c`: Contains all code that computes the angular velocity and linear acceleration in the local body frame 
	4. `compute_F.c`: Contains two main functions, `compute_F()`and `compute_G()`, which are helper functions for `compute_Pdot()` in `propogate.c`
	5. `compute_initial_consts.c`: Computes all the initial matrices used at startup of the STM32. The includes the uncertainties in our states, the noise in our sensors, and more.
	6. `ekf_utils.c`: Helper functions used throughout the entire EKF project or used commonly in the EKF project.
	7. `propogate.c`: Uses accelerometer data and gyro data and integrates it to predict the next state.
	8. `update_sensors.c`: Adds the measurement update functions that occur whenever sensors other than the IMU get new data. This helps the filter refine the state and get a more accurate state estimate.
	9. `update_EKF.c`: Contains the checks for when to deploy main and drogue. Integrates all above functions into one function which is defined as one iteration of EKF.
### RECO-FC Communication
- While RECO for the most part is an independent system, it does communicate with the flight computer, FC, via SPI to get GPS data and receive commands from operators. The code that handles this communications can be found in the `HAL_SPI_TxRxCpltCallback()` under `main.c`. 

- The opcodes used between FC and RECO are as follows:
	1. `0x79`: RECO Launch
	2. `0xF2`: GPS Data
	3. `0xCA`: RECO Init/Reset
	4. `0x2E`: Send RECO parameters