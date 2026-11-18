################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/ASM330LHGB1.c \
../Core/Src/LIS2MDL.c \
../Core/Src/LIS3MDL.c \
../Core/Src/MS5611.c \
../Core/Src/SPI_Device.c \
../Core/Src/fault_logic.c \
../Core/Src/main.c \
../Core/Src/performance.c \
../Core/Src/stm32h5xx_hal_msp.c \
../Core/Src/stm32h5xx_it.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32h5xx.c 

OBJS += \
./Core/Src/ASM330LHGB1.o \
./Core/Src/LIS2MDL.o \
./Core/Src/LIS3MDL.o \
./Core/Src/MS5611.o \
./Core/Src/SPI_Device.o \
./Core/Src/fault_logic.o \
./Core/Src/main.o \
./Core/Src/performance.o \
./Core/Src/stm32h5xx_hal_msp.o \
./Core/Src/stm32h5xx_it.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32h5xx.o 

C_DEPS += \
./Core/Src/ASM330LHGB1.d \
./Core/Src/LIS2MDL.d \
./Core/Src/LIS3MDL.d \
./Core/Src/MS5611.d \
./Core/Src/SPI_Device.d \
./Core/Src/fault_logic.d \
./Core/Src/main.d \
./Core/Src/performance.d \
./Core/Src/stm32h5xx_hal_msp.d \
./Core/Src/stm32h5xx_it.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32h5xx.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o Core/Src/%.su Core/Src/%.cyclo: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m33 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32H573xx -c -I../Core/Inc -I"/home/spectre/Clones/luna/bam/Core/CControl/Headers" -I"/home/spectre/Clones/luna/bam/Drivers/CMSIS_DSP/Include" -I"/home/spectre/Clones/luna/bam/Core/EKF/Inc" -I"/home/spectre/Clones/luna/bam/Drivers/CMSIS_DSP/PrivateInclude" -I../Drivers/STM32H5xx_HAL_Driver/Inc -I../Drivers/STM32H5xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H5xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/ASM330LHGB1.cyclo ./Core/Src/ASM330LHGB1.d ./Core/Src/ASM330LHGB1.o ./Core/Src/ASM330LHGB1.su ./Core/Src/LIS2MDL.cyclo ./Core/Src/LIS2MDL.d ./Core/Src/LIS2MDL.o ./Core/Src/LIS2MDL.su ./Core/Src/LIS3MDL.cyclo ./Core/Src/LIS3MDL.d ./Core/Src/LIS3MDL.o ./Core/Src/LIS3MDL.su ./Core/Src/MS5611.cyclo ./Core/Src/MS5611.d ./Core/Src/MS5611.o ./Core/Src/MS5611.su ./Core/Src/SPI_Device.cyclo ./Core/Src/SPI_Device.d ./Core/Src/SPI_Device.o ./Core/Src/SPI_Device.su ./Core/Src/fault_logic.cyclo ./Core/Src/fault_logic.d ./Core/Src/fault_logic.o ./Core/Src/fault_logic.su ./Core/Src/main.cyclo ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/main.su ./Core/Src/performance.cyclo ./Core/Src/performance.d ./Core/Src/performance.o ./Core/Src/performance.su ./Core/Src/stm32h5xx_hal_msp.cyclo ./Core/Src/stm32h5xx_hal_msp.d ./Core/Src/stm32h5xx_hal_msp.o ./Core/Src/stm32h5xx_hal_msp.su ./Core/Src/stm32h5xx_it.cyclo ./Core/Src/stm32h5xx_it.d ./Core/Src/stm32h5xx_it.o ./Core/Src/stm32h5xx_it.su ./Core/Src/syscalls.cyclo ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/syscalls.su ./Core/Src/sysmem.cyclo ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/sysmem.su ./Core/Src/system_stm32h5xx.cyclo ./Core/Src/system_stm32h5xx.d ./Core/Src/system_stm32h5xx.o ./Core/Src/system_stm32h5xx.su

.PHONY: clean-Core-2f-Src

