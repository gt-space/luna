################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/CMSIS_DSP/Source/ControllerFunctions/ControllerFunctions.c \
../Drivers/CMSIS_DSP/Source/ControllerFunctions/arm_pid_init_f32.c \
../Drivers/CMSIS_DSP/Source/ControllerFunctions/arm_pid_init_q15.c \
../Drivers/CMSIS_DSP/Source/ControllerFunctions/arm_pid_init_q31.c \
../Drivers/CMSIS_DSP/Source/ControllerFunctions/arm_pid_reset_f32.c \
../Drivers/CMSIS_DSP/Source/ControllerFunctions/arm_pid_reset_q15.c \
../Drivers/CMSIS_DSP/Source/ControllerFunctions/arm_pid_reset_q31.c \
../Drivers/CMSIS_DSP/Source/ControllerFunctions/arm_sin_cos_f32.c \
../Drivers/CMSIS_DSP/Source/ControllerFunctions/arm_sin_cos_q31.c 

OBJS += \
./Drivers/CMSIS_DSP/Source/ControllerFunctions/ControllerFunctions.o \
./Drivers/CMSIS_DSP/Source/ControllerFunctions/arm_pid_init_f32.o \
./Drivers/CMSIS_DSP/Source/ControllerFunctions/arm_pid_init_q15.o \
./Drivers/CMSIS_DSP/Source/ControllerFunctions/arm_pid_init_q31.o \
./Drivers/CMSIS_DSP/Source/ControllerFunctions/arm_pid_reset_f32.o \
./Drivers/CMSIS_DSP/Source/ControllerFunctions/arm_pid_reset_q15.o \
./Drivers/CMSIS_DSP/Source/ControllerFunctions/arm_pid_reset_q31.o \
./Drivers/CMSIS_DSP/Source/ControllerFunctions/arm_sin_cos_f32.o \
./Drivers/CMSIS_DSP/Source/ControllerFunctions/arm_sin_cos_q31.o 

C_DEPS += \
./Drivers/CMSIS_DSP/Source/ControllerFunctions/ControllerFunctions.d \
./Drivers/CMSIS_DSP/Source/ControllerFunctions/arm_pid_init_f32.d \
./Drivers/CMSIS_DSP/Source/ControllerFunctions/arm_pid_init_q15.d \
./Drivers/CMSIS_DSP/Source/ControllerFunctions/arm_pid_init_q31.d \
./Drivers/CMSIS_DSP/Source/ControllerFunctions/arm_pid_reset_f32.d \
./Drivers/CMSIS_DSP/Source/ControllerFunctions/arm_pid_reset_q15.d \
./Drivers/CMSIS_DSP/Source/ControllerFunctions/arm_pid_reset_q31.d \
./Drivers/CMSIS_DSP/Source/ControllerFunctions/arm_sin_cos_f32.d \
./Drivers/CMSIS_DSP/Source/ControllerFunctions/arm_sin_cos_q31.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/CMSIS_DSP/Source/ControllerFunctions/%.o Drivers/CMSIS_DSP/Source/ControllerFunctions/%.su Drivers/CMSIS_DSP/Source/ControllerFunctions/%.cyclo: ../Drivers/CMSIS_DSP/Source/ControllerFunctions/%.c Drivers/CMSIS_DSP/Source/ControllerFunctions/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m33 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32H573xx -c -I../Core/Inc -I"/home/spectre/Clones/luna/bam/Core/CControl/Headers" -I"/home/spectre/Clones/luna/bam/Drivers/CMSIS_DSP/Include" -I"/home/spectre/Clones/luna/bam/Core/EKF/Inc" -I"/home/spectre/Clones/luna/bam/Drivers/CMSIS_DSP/PrivateInclude" -I../Drivers/STM32H5xx_HAL_Driver/Inc -I../Drivers/STM32H5xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H5xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-CMSIS_DSP-2f-Source-2f-ControllerFunctions

clean-Drivers-2f-CMSIS_DSP-2f-Source-2f-ControllerFunctions:
	-$(RM) ./Drivers/CMSIS_DSP/Source/ControllerFunctions/ControllerFunctions.cyclo ./Drivers/CMSIS_DSP/Source/ControllerFunctions/ControllerFunctions.d ./Drivers/CMSIS_DSP/Source/ControllerFunctions/ControllerFunctions.o ./Drivers/CMSIS_DSP/Source/ControllerFunctions/ControllerFunctions.su ./Drivers/CMSIS_DSP/Source/ControllerFunctions/arm_pid_init_f32.cyclo ./Drivers/CMSIS_DSP/Source/ControllerFunctions/arm_pid_init_f32.d ./Drivers/CMSIS_DSP/Source/ControllerFunctions/arm_pid_init_f32.o ./Drivers/CMSIS_DSP/Source/ControllerFunctions/arm_pid_init_f32.su ./Drivers/CMSIS_DSP/Source/ControllerFunctions/arm_pid_init_q15.cyclo ./Drivers/CMSIS_DSP/Source/ControllerFunctions/arm_pid_init_q15.d ./Drivers/CMSIS_DSP/Source/ControllerFunctions/arm_pid_init_q15.o ./Drivers/CMSIS_DSP/Source/ControllerFunctions/arm_pid_init_q15.su ./Drivers/CMSIS_DSP/Source/ControllerFunctions/arm_pid_init_q31.cyclo ./Drivers/CMSIS_DSP/Source/ControllerFunctions/arm_pid_init_q31.d ./Drivers/CMSIS_DSP/Source/ControllerFunctions/arm_pid_init_q31.o ./Drivers/CMSIS_DSP/Source/ControllerFunctions/arm_pid_init_q31.su ./Drivers/CMSIS_DSP/Source/ControllerFunctions/arm_pid_reset_f32.cyclo ./Drivers/CMSIS_DSP/Source/ControllerFunctions/arm_pid_reset_f32.d ./Drivers/CMSIS_DSP/Source/ControllerFunctions/arm_pid_reset_f32.o ./Drivers/CMSIS_DSP/Source/ControllerFunctions/arm_pid_reset_f32.su ./Drivers/CMSIS_DSP/Source/ControllerFunctions/arm_pid_reset_q15.cyclo ./Drivers/CMSIS_DSP/Source/ControllerFunctions/arm_pid_reset_q15.d ./Drivers/CMSIS_DSP/Source/ControllerFunctions/arm_pid_reset_q15.o ./Drivers/CMSIS_DSP/Source/ControllerFunctions/arm_pid_reset_q15.su ./Drivers/CMSIS_DSP/Source/ControllerFunctions/arm_pid_reset_q31.cyclo ./Drivers/CMSIS_DSP/Source/ControllerFunctions/arm_pid_reset_q31.d ./Drivers/CMSIS_DSP/Source/ControllerFunctions/arm_pid_reset_q31.o ./Drivers/CMSIS_DSP/Source/ControllerFunctions/arm_pid_reset_q31.su ./Drivers/CMSIS_DSP/Source/ControllerFunctions/arm_sin_cos_f32.cyclo ./Drivers/CMSIS_DSP/Source/ControllerFunctions/arm_sin_cos_f32.d ./Drivers/CMSIS_DSP/Source/ControllerFunctions/arm_sin_cos_f32.o ./Drivers/CMSIS_DSP/Source/ControllerFunctions/arm_sin_cos_f32.su ./Drivers/CMSIS_DSP/Source/ControllerFunctions/arm_sin_cos_q31.cyclo ./Drivers/CMSIS_DSP/Source/ControllerFunctions/arm_sin_cos_q31.d ./Drivers/CMSIS_DSP/Source/ControllerFunctions/arm_sin_cos_q31.o ./Drivers/CMSIS_DSP/Source/ControllerFunctions/arm_sin_cos_q31.su

.PHONY: clean-Drivers-2f-CMSIS_DSP-2f-Source-2f-ControllerFunctions

