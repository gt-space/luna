################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/EKF/Src/altimeter_pressure.c \
../Core/EKF/Src/compute_F.c \
../Core/EKF/Src/compute_hats.c \
../Core/EKF/Src/compute_initial_consts.c \
../Core/EKF/Src/ekf_utils.c \
../Core/EKF/Src/fading_memory.c \
../Core/EKF/Src/filter_pressure.c \
../Core/EKF/Src/matrix_extensions.c \
../Core/EKF/Src/nearestPSD.c \
../Core/EKF/Src/propogate.c \
../Core/EKF/Src/quaternion_extensions.c \
../Core/EKF/Src/tests.c \
../Core/EKF/Src/update_EKF.c \
../Core/EKF/Src/update_sensors.c 

OBJS += \
./Core/EKF/Src/altimeter_pressure.o \
./Core/EKF/Src/compute_F.o \
./Core/EKF/Src/compute_hats.o \
./Core/EKF/Src/compute_initial_consts.o \
./Core/EKF/Src/ekf_utils.o \
./Core/EKF/Src/fading_memory.o \
./Core/EKF/Src/filter_pressure.o \
./Core/EKF/Src/matrix_extensions.o \
./Core/EKF/Src/nearestPSD.o \
./Core/EKF/Src/propogate.o \
./Core/EKF/Src/quaternion_extensions.o \
./Core/EKF/Src/tests.o \
./Core/EKF/Src/update_EKF.o \
./Core/EKF/Src/update_sensors.o 

C_DEPS += \
./Core/EKF/Src/altimeter_pressure.d \
./Core/EKF/Src/compute_F.d \
./Core/EKF/Src/compute_hats.d \
./Core/EKF/Src/compute_initial_consts.d \
./Core/EKF/Src/ekf_utils.d \
./Core/EKF/Src/fading_memory.d \
./Core/EKF/Src/filter_pressure.d \
./Core/EKF/Src/matrix_extensions.d \
./Core/EKF/Src/nearestPSD.d \
./Core/EKF/Src/propogate.d \
./Core/EKF/Src/quaternion_extensions.d \
./Core/EKF/Src/tests.d \
./Core/EKF/Src/update_EKF.d \
./Core/EKF/Src/update_sensors.d 


# Each subdirectory must supply rules for building sources it contributes
Core/EKF/Src/%.o Core/EKF/Src/%.su Core/EKF/Src/%.cyclo: ../Core/EKF/Src/%.c Core/EKF/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m33 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32H573xx -c -I../Core/Inc -I"/home/spectre/Clones/luna/bam/Core/CControl/Headers" -I"/home/spectre/Clones/luna/bam/Drivers/CMSIS_DSP/Include" -I"/home/spectre/Clones/luna/bam/Core/EKF/Inc" -I"/home/spectre/Clones/luna/bam/Drivers/CMSIS_DSP/PrivateInclude" -I../Drivers/STM32H5xx_HAL_Driver/Inc -I../Drivers/STM32H5xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H5xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-EKF-2f-Src

clean-Core-2f-EKF-2f-Src:
	-$(RM) ./Core/EKF/Src/altimeter_pressure.cyclo ./Core/EKF/Src/altimeter_pressure.d ./Core/EKF/Src/altimeter_pressure.o ./Core/EKF/Src/altimeter_pressure.su ./Core/EKF/Src/compute_F.cyclo ./Core/EKF/Src/compute_F.d ./Core/EKF/Src/compute_F.o ./Core/EKF/Src/compute_F.su ./Core/EKF/Src/compute_hats.cyclo ./Core/EKF/Src/compute_hats.d ./Core/EKF/Src/compute_hats.o ./Core/EKF/Src/compute_hats.su ./Core/EKF/Src/compute_initial_consts.cyclo ./Core/EKF/Src/compute_initial_consts.d ./Core/EKF/Src/compute_initial_consts.o ./Core/EKF/Src/compute_initial_consts.su ./Core/EKF/Src/ekf_utils.cyclo ./Core/EKF/Src/ekf_utils.d ./Core/EKF/Src/ekf_utils.o ./Core/EKF/Src/ekf_utils.su ./Core/EKF/Src/fading_memory.cyclo ./Core/EKF/Src/fading_memory.d ./Core/EKF/Src/fading_memory.o ./Core/EKF/Src/fading_memory.su ./Core/EKF/Src/filter_pressure.cyclo ./Core/EKF/Src/filter_pressure.d ./Core/EKF/Src/filter_pressure.o ./Core/EKF/Src/filter_pressure.su ./Core/EKF/Src/matrix_extensions.cyclo ./Core/EKF/Src/matrix_extensions.d ./Core/EKF/Src/matrix_extensions.o ./Core/EKF/Src/matrix_extensions.su ./Core/EKF/Src/nearestPSD.cyclo ./Core/EKF/Src/nearestPSD.d ./Core/EKF/Src/nearestPSD.o ./Core/EKF/Src/nearestPSD.su ./Core/EKF/Src/propogate.cyclo ./Core/EKF/Src/propogate.d ./Core/EKF/Src/propogate.o ./Core/EKF/Src/propogate.su ./Core/EKF/Src/quaternion_extensions.cyclo ./Core/EKF/Src/quaternion_extensions.d ./Core/EKF/Src/quaternion_extensions.o ./Core/EKF/Src/quaternion_extensions.su ./Core/EKF/Src/tests.cyclo ./Core/EKF/Src/tests.d ./Core/EKF/Src/tests.o ./Core/EKF/Src/tests.su ./Core/EKF/Src/update_EKF.cyclo ./Core/EKF/Src/update_EKF.d ./Core/EKF/Src/update_EKF.o ./Core/EKF/Src/update_EKF.su ./Core/EKF/Src/update_sensors.cyclo ./Core/EKF/Src/update_sensors.d ./Core/EKF/Src/update_sensors.o ./Core/EKF/Src/update_sensors.su

.PHONY: clean-Core-2f-EKF-2f-Src

