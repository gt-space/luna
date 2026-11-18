################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Drivers/STM32H5xx_HAL_Driver/Src/subdir.mk
-include Drivers/CMSIS_DSP/Source/SupportFunctions/subdir.mk
-include Drivers/CMSIS_DSP/Source/StatisticsFunctions/subdir.mk
-include Drivers/CMSIS_DSP/Source/QuaternionMathFunctions/subdir.mk
-include Drivers/CMSIS_DSP/Source/MatrixFunctions/subdir.mk
-include Drivers/CMSIS_DSP/Source/FastMathFunctions/subdir.mk
-include Drivers/CMSIS_DSP/Source/ControllerFunctions/subdir.mk
-include Drivers/CMSIS_DSP/Source/ComplexMathFunctions/subdir.mk
-include Drivers/CMSIS_DSP/Source/CommonTables/subdir.mk
-include Drivers/CMSIS_DSP/Source/BasicMathFunctions/subdir.mk
-include Core/Startup/subdir.mk
-include Core/Src/subdir.mk
-include Core/EKF/Src/subdir.mk
-include Core/CControl/Sources/Miscellaneous/subdir.mk
-include Core/CControl/Sources/LinearAlgebra/CLapack/Tools/subdir.mk
-include Core/CControl/Sources/LinearAlgebra/CLapack/Routines/subdir.mk
-include Core/CControl/Sources/LinearAlgebra/subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(S_DEPS)),)
-include $(S_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

OPTIONAL_TOOL_DEPS := \
$(wildcard ../makefile.defs) \
$(wildcard ../makefile.init) \
$(wildcard ../makefile.targets) \


BUILD_ARTIFACT_NAME := BombusAvionicsModule
BUILD_ARTIFACT_EXTENSION := elf
BUILD_ARTIFACT_PREFIX :=
BUILD_ARTIFACT := $(BUILD_ARTIFACT_PREFIX)$(BUILD_ARTIFACT_NAME)$(if $(BUILD_ARTIFACT_EXTENSION),.$(BUILD_ARTIFACT_EXTENSION),)

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
BombusAvionicsModule.elf \

MAP_FILES += \
BombusAvionicsModule.map \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
BombusAvionicsModule.list \


# All Target
all: main-build

# Main-build Target
main-build: BombusAvionicsModule.elf secondary-outputs

# Tool invocations
BombusAvionicsModule.elf BombusAvionicsModule.map: $(OBJS) $(USER_OBJS) /home/spectre/Clones/luna/bam/STM32H573RITX_FLASH.ld makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-gcc -o "BombusAvionicsModule.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m33 -T"/home/spectre/Clones/luna/bam/STM32H573RITX_FLASH.ld" --specs=nosys.specs -Wl,-Map="BombusAvionicsModule.map" -Wl,--gc-sections -static --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

BombusAvionicsModule.list: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "BombusAvionicsModule.list"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) BombusAvionicsModule.elf BombusAvionicsModule.list BombusAvionicsModule.map default.size.stdout
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST)

fail-specified-linker-script-missing:
	@echo 'Error: Cannot find the specified linker script. Check the linker settings in the build configuration.'
	@exit 2

warn-no-linker-script-specified:
	@echo 'Warning: No linker script specified. Check the linker settings in the build configuration.'

.PHONY: all clean dependents main-build fail-specified-linker-script-missing warn-no-linker-script-specified

-include ../makefile.targets
