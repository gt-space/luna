################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
S_SRCS += \
../Core/Startup/startup_stm32h573ritx.s 

OBJS += \
./Core/Startup/startup_stm32h573ritx.o 

S_DEPS += \
./Core/Startup/startup_stm32h573ritx.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Startup/%.o: ../Core/Startup/%.s Core/Startup/subdir.mk
	arm-none-eabi-gcc -mcpu=cortex-m33 -g3 -DDEBUG -c -I"/home/spectre/Clones/luna/bam/Core/Inc" -I"/home/spectre/Clones/luna/bam/Drivers/STM32H5xx_HAL_Driver/Inc" -I"/home/spectre/Clones/luna/bam/Drivers/STM32H5xx_HAL_Driver/Inc/Legacy" -I"/home/spectre/Clones/luna/bam/Drivers/CMSIS/Device/ST/STM32H5xx/Include" -I"/home/spectre/Clones/luna/bam/Drivers/CMSIS/Include" -x assembler-with-cpp -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@" "$<"

clean: clean-Core-2f-Startup

clean-Core-2f-Startup:
	-$(RM) ./Core/Startup/startup_stm32h573ritx.d ./Core/Startup/startup_stm32h573ritx.o

.PHONY: clean-Core-2f-Startup

