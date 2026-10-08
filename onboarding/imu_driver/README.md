# LSM6DSR IMU driver (YJSP onboarding)

Edwin Wu. STM32F030C6 on the onboarding board. Reads the IMU over SPI
at 200 Hz and prints accel and gyro with semihosting printf.

## Compiler
GNU Tools for STM32 14.3.rel1 (STM32CubeIDE 2.2)

## How to run
1. Open imu_driver.ioc and click Generate Code (Drivers/ isn't committed).
2. Debug with the imu_driver launch config. Semihosting uses port 6123.

## Notes
- IMU settings: 208 Hz, ±8 g, ±2000 dps, BDU on.
- Accel prints in mm/s² and gyro in mdps, as whole numbers. Float printf
  makes the program too big for the chip's 32 KB of flash.