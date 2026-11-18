################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/CMSIS_DSP/Source/SupportFunctions/SupportFunctions.c \
../Drivers/CMSIS_DSP/Source/SupportFunctions/SupportFunctionsF16.c \
../Drivers/CMSIS_DSP/Source/SupportFunctions/arm_barycenter_f16.c \
../Drivers/CMSIS_DSP/Source/SupportFunctions/arm_barycenter_f32.c \
../Drivers/CMSIS_DSP/Source/SupportFunctions/arm_bitonic_sort_f32.c \
../Drivers/CMSIS_DSP/Source/SupportFunctions/arm_bubble_sort_f32.c \
../Drivers/CMSIS_DSP/Source/SupportFunctions/arm_copy_f16.c \
../Drivers/CMSIS_DSP/Source/SupportFunctions/arm_copy_f32.c \
../Drivers/CMSIS_DSP/Source/SupportFunctions/arm_copy_f64.c \
../Drivers/CMSIS_DSP/Source/SupportFunctions/arm_copy_q15.c \
../Drivers/CMSIS_DSP/Source/SupportFunctions/arm_copy_q31.c \
../Drivers/CMSIS_DSP/Source/SupportFunctions/arm_copy_q7.c \
../Drivers/CMSIS_DSP/Source/SupportFunctions/arm_f16_to_f64.c \
../Drivers/CMSIS_DSP/Source/SupportFunctions/arm_f16_to_float.c \
../Drivers/CMSIS_DSP/Source/SupportFunctions/arm_f16_to_q15.c \
../Drivers/CMSIS_DSP/Source/SupportFunctions/arm_f64_to_f16.c \
../Drivers/CMSIS_DSP/Source/SupportFunctions/arm_f64_to_float.c \
../Drivers/CMSIS_DSP/Source/SupportFunctions/arm_f64_to_q15.c \
../Drivers/CMSIS_DSP/Source/SupportFunctions/arm_f64_to_q31.c \
../Drivers/CMSIS_DSP/Source/SupportFunctions/arm_f64_to_q7.c \
../Drivers/CMSIS_DSP/Source/SupportFunctions/arm_fill_f16.c \
../Drivers/CMSIS_DSP/Source/SupportFunctions/arm_fill_f32.c \
../Drivers/CMSIS_DSP/Source/SupportFunctions/arm_fill_f64.c \
../Drivers/CMSIS_DSP/Source/SupportFunctions/arm_fill_q15.c \
../Drivers/CMSIS_DSP/Source/SupportFunctions/arm_fill_q31.c \
../Drivers/CMSIS_DSP/Source/SupportFunctions/arm_fill_q7.c \
../Drivers/CMSIS_DSP/Source/SupportFunctions/arm_float_to_f16.c \
../Drivers/CMSIS_DSP/Source/SupportFunctions/arm_float_to_f64.c \
../Drivers/CMSIS_DSP/Source/SupportFunctions/arm_float_to_q15.c \
../Drivers/CMSIS_DSP/Source/SupportFunctions/arm_float_to_q31.c \
../Drivers/CMSIS_DSP/Source/SupportFunctions/arm_float_to_q7.c \
../Drivers/CMSIS_DSP/Source/SupportFunctions/arm_heap_sort_f32.c \
../Drivers/CMSIS_DSP/Source/SupportFunctions/arm_insertion_sort_f32.c \
../Drivers/CMSIS_DSP/Source/SupportFunctions/arm_merge_sort_f32.c \
../Drivers/CMSIS_DSP/Source/SupportFunctions/arm_merge_sort_init_f32.c \
../Drivers/CMSIS_DSP/Source/SupportFunctions/arm_q15_to_f16.c \
../Drivers/CMSIS_DSP/Source/SupportFunctions/arm_q15_to_f64.c \
../Drivers/CMSIS_DSP/Source/SupportFunctions/arm_q15_to_float.c \
../Drivers/CMSIS_DSP/Source/SupportFunctions/arm_q15_to_q31.c \
../Drivers/CMSIS_DSP/Source/SupportFunctions/arm_q15_to_q7.c \
../Drivers/CMSIS_DSP/Source/SupportFunctions/arm_q31_to_f64.c \
../Drivers/CMSIS_DSP/Source/SupportFunctions/arm_q31_to_float.c \
../Drivers/CMSIS_DSP/Source/SupportFunctions/arm_q31_to_q15.c \
../Drivers/CMSIS_DSP/Source/SupportFunctions/arm_q31_to_q7.c \
../Drivers/CMSIS_DSP/Source/SupportFunctions/arm_q7_to_f64.c \
../Drivers/CMSIS_DSP/Source/SupportFunctions/arm_q7_to_float.c \
../Drivers/CMSIS_DSP/Source/SupportFunctions/arm_q7_to_q15.c \
../Drivers/CMSIS_DSP/Source/SupportFunctions/arm_q7_to_q31.c \
../Drivers/CMSIS_DSP/Source/SupportFunctions/arm_quick_sort_f32.c \
../Drivers/CMSIS_DSP/Source/SupportFunctions/arm_selection_sort_f32.c \
../Drivers/CMSIS_DSP/Source/SupportFunctions/arm_sort_f32.c \
../Drivers/CMSIS_DSP/Source/SupportFunctions/arm_sort_init_f32.c \
../Drivers/CMSIS_DSP/Source/SupportFunctions/arm_weighted_average_f16.c \
../Drivers/CMSIS_DSP/Source/SupportFunctions/arm_weighted_average_f32.c 

OBJS += \
./Drivers/CMSIS_DSP/Source/SupportFunctions/SupportFunctions.o \
./Drivers/CMSIS_DSP/Source/SupportFunctions/SupportFunctionsF16.o \
./Drivers/CMSIS_DSP/Source/SupportFunctions/arm_barycenter_f16.o \
./Drivers/CMSIS_DSP/Source/SupportFunctions/arm_barycenter_f32.o \
./Drivers/CMSIS_DSP/Source/SupportFunctions/arm_bitonic_sort_f32.o \
./Drivers/CMSIS_DSP/Source/SupportFunctions/arm_bubble_sort_f32.o \
./Drivers/CMSIS_DSP/Source/SupportFunctions/arm_copy_f16.o \
./Drivers/CMSIS_DSP/Source/SupportFunctions/arm_copy_f32.o \
./Drivers/CMSIS_DSP/Source/SupportFunctions/arm_copy_f64.o \
./Drivers/CMSIS_DSP/Source/SupportFunctions/arm_copy_q15.o \
./Drivers/CMSIS_DSP/Source/SupportFunctions/arm_copy_q31.o \
./Drivers/CMSIS_DSP/Source/SupportFunctions/arm_copy_q7.o \
./Drivers/CMSIS_DSP/Source/SupportFunctions/arm_f16_to_f64.o \
./Drivers/CMSIS_DSP/Source/SupportFunctions/arm_f16_to_float.o \
./Drivers/CMSIS_DSP/Source/SupportFunctions/arm_f16_to_q15.o \
./Drivers/CMSIS_DSP/Source/SupportFunctions/arm_f64_to_f16.o \
./Drivers/CMSIS_DSP/Source/SupportFunctions/arm_f64_to_float.o \
./Drivers/CMSIS_DSP/Source/SupportFunctions/arm_f64_to_q15.o \
./Drivers/CMSIS_DSP/Source/SupportFunctions/arm_f64_to_q31.o \
./Drivers/CMSIS_DSP/Source/SupportFunctions/arm_f64_to_q7.o \
./Drivers/CMSIS_DSP/Source/SupportFunctions/arm_fill_f16.o \
./Drivers/CMSIS_DSP/Source/SupportFunctions/arm_fill_f32.o \
./Drivers/CMSIS_DSP/Source/SupportFunctions/arm_fill_f64.o \
./Drivers/CMSIS_DSP/Source/SupportFunctions/arm_fill_q15.o \
./Drivers/CMSIS_DSP/Source/SupportFunctions/arm_fill_q31.o \
./Drivers/CMSIS_DSP/Source/SupportFunctions/arm_fill_q7.o \
./Drivers/CMSIS_DSP/Source/SupportFunctions/arm_float_to_f16.o \
./Drivers/CMSIS_DSP/Source/SupportFunctions/arm_float_to_f64.o \
./Drivers/CMSIS_DSP/Source/SupportFunctions/arm_float_to_q15.o \
./Drivers/CMSIS_DSP/Source/SupportFunctions/arm_float_to_q31.o \
./Drivers/CMSIS_DSP/Source/SupportFunctions/arm_float_to_q7.o \
./Drivers/CMSIS_DSP/Source/SupportFunctions/arm_heap_sort_f32.o \
./Drivers/CMSIS_DSP/Source/SupportFunctions/arm_insertion_sort_f32.o \
./Drivers/CMSIS_DSP/Source/SupportFunctions/arm_merge_sort_f32.o \
./Drivers/CMSIS_DSP/Source/SupportFunctions/arm_merge_sort_init_f32.o \
./Drivers/CMSIS_DSP/Source/SupportFunctions/arm_q15_to_f16.o \
./Drivers/CMSIS_DSP/Source/SupportFunctions/arm_q15_to_f64.o \
./Drivers/CMSIS_DSP/Source/SupportFunctions/arm_q15_to_float.o \
./Drivers/CMSIS_DSP/Source/SupportFunctions/arm_q15_to_q31.o \
./Drivers/CMSIS_DSP/Source/SupportFunctions/arm_q15_to_q7.o \
./Drivers/CMSIS_DSP/Source/SupportFunctions/arm_q31_to_f64.o \
./Drivers/CMSIS_DSP/Source/SupportFunctions/arm_q31_to_float.o \
./Drivers/CMSIS_DSP/Source/SupportFunctions/arm_q31_to_q15.o \
./Drivers/CMSIS_DSP/Source/SupportFunctions/arm_q31_to_q7.o \
./Drivers/CMSIS_DSP/Source/SupportFunctions/arm_q7_to_f64.o \
./Drivers/CMSIS_DSP/Source/SupportFunctions/arm_q7_to_float.o \
./Drivers/CMSIS_DSP/Source/SupportFunctions/arm_q7_to_q15.o \
./Drivers/CMSIS_DSP/Source/SupportFunctions/arm_q7_to_q31.o \
./Drivers/CMSIS_DSP/Source/SupportFunctions/arm_quick_sort_f32.o \
./Drivers/CMSIS_DSP/Source/SupportFunctions/arm_selection_sort_f32.o \
./Drivers/CMSIS_DSP/Source/SupportFunctions/arm_sort_f32.o \
./Drivers/CMSIS_DSP/Source/SupportFunctions/arm_sort_init_f32.o \
./Drivers/CMSIS_DSP/Source/SupportFunctions/arm_weighted_average_f16.o \
./Drivers/CMSIS_DSP/Source/SupportFunctions/arm_weighted_average_f32.o 

C_DEPS += \
./Drivers/CMSIS_DSP/Source/SupportFunctions/SupportFunctions.d \
./Drivers/CMSIS_DSP/Source/SupportFunctions/SupportFunctionsF16.d \
./Drivers/CMSIS_DSP/Source/SupportFunctions/arm_barycenter_f16.d \
./Drivers/CMSIS_DSP/Source/SupportFunctions/arm_barycenter_f32.d \
./Drivers/CMSIS_DSP/Source/SupportFunctions/arm_bitonic_sort_f32.d \
./Drivers/CMSIS_DSP/Source/SupportFunctions/arm_bubble_sort_f32.d \
./Drivers/CMSIS_DSP/Source/SupportFunctions/arm_copy_f16.d \
./Drivers/CMSIS_DSP/Source/SupportFunctions/arm_copy_f32.d \
./Drivers/CMSIS_DSP/Source/SupportFunctions/arm_copy_f64.d \
./Drivers/CMSIS_DSP/Source/SupportFunctions/arm_copy_q15.d \
./Drivers/CMSIS_DSP/Source/SupportFunctions/arm_copy_q31.d \
./Drivers/CMSIS_DSP/Source/SupportFunctions/arm_copy_q7.d \
./Drivers/CMSIS_DSP/Source/SupportFunctions/arm_f16_to_f64.d \
./Drivers/CMSIS_DSP/Source/SupportFunctions/arm_f16_to_float.d \
./Drivers/CMSIS_DSP/Source/SupportFunctions/arm_f16_to_q15.d \
./Drivers/CMSIS_DSP/Source/SupportFunctions/arm_f64_to_f16.d \
./Drivers/CMSIS_DSP/Source/SupportFunctions/arm_f64_to_float.d \
./Drivers/CMSIS_DSP/Source/SupportFunctions/arm_f64_to_q15.d \
./Drivers/CMSIS_DSP/Source/SupportFunctions/arm_f64_to_q31.d \
./Drivers/CMSIS_DSP/Source/SupportFunctions/arm_f64_to_q7.d \
./Drivers/CMSIS_DSP/Source/SupportFunctions/arm_fill_f16.d \
./Drivers/CMSIS_DSP/Source/SupportFunctions/arm_fill_f32.d \
./Drivers/CMSIS_DSP/Source/SupportFunctions/arm_fill_f64.d \
./Drivers/CMSIS_DSP/Source/SupportFunctions/arm_fill_q15.d \
./Drivers/CMSIS_DSP/Source/SupportFunctions/arm_fill_q31.d \
./Drivers/CMSIS_DSP/Source/SupportFunctions/arm_fill_q7.d \
./Drivers/CMSIS_DSP/Source/SupportFunctions/arm_float_to_f16.d \
./Drivers/CMSIS_DSP/Source/SupportFunctions/arm_float_to_f64.d \
./Drivers/CMSIS_DSP/Source/SupportFunctions/arm_float_to_q15.d \
./Drivers/CMSIS_DSP/Source/SupportFunctions/arm_float_to_q31.d \
./Drivers/CMSIS_DSP/Source/SupportFunctions/arm_float_to_q7.d \
./Drivers/CMSIS_DSP/Source/SupportFunctions/arm_heap_sort_f32.d \
./Drivers/CMSIS_DSP/Source/SupportFunctions/arm_insertion_sort_f32.d \
./Drivers/CMSIS_DSP/Source/SupportFunctions/arm_merge_sort_f32.d \
./Drivers/CMSIS_DSP/Source/SupportFunctions/arm_merge_sort_init_f32.d \
./Drivers/CMSIS_DSP/Source/SupportFunctions/arm_q15_to_f16.d \
./Drivers/CMSIS_DSP/Source/SupportFunctions/arm_q15_to_f64.d \
./Drivers/CMSIS_DSP/Source/SupportFunctions/arm_q15_to_float.d \
./Drivers/CMSIS_DSP/Source/SupportFunctions/arm_q15_to_q31.d \
./Drivers/CMSIS_DSP/Source/SupportFunctions/arm_q15_to_q7.d \
./Drivers/CMSIS_DSP/Source/SupportFunctions/arm_q31_to_f64.d \
./Drivers/CMSIS_DSP/Source/SupportFunctions/arm_q31_to_float.d \
./Drivers/CMSIS_DSP/Source/SupportFunctions/arm_q31_to_q15.d \
./Drivers/CMSIS_DSP/Source/SupportFunctions/arm_q31_to_q7.d \
./Drivers/CMSIS_DSP/Source/SupportFunctions/arm_q7_to_f64.d \
./Drivers/CMSIS_DSP/Source/SupportFunctions/arm_q7_to_float.d \
./Drivers/CMSIS_DSP/Source/SupportFunctions/arm_q7_to_q15.d \
./Drivers/CMSIS_DSP/Source/SupportFunctions/arm_q7_to_q31.d \
./Drivers/CMSIS_DSP/Source/SupportFunctions/arm_quick_sort_f32.d \
./Drivers/CMSIS_DSP/Source/SupportFunctions/arm_selection_sort_f32.d \
./Drivers/CMSIS_DSP/Source/SupportFunctions/arm_sort_f32.d \
./Drivers/CMSIS_DSP/Source/SupportFunctions/arm_sort_init_f32.d \
./Drivers/CMSIS_DSP/Source/SupportFunctions/arm_weighted_average_f16.d \
./Drivers/CMSIS_DSP/Source/SupportFunctions/arm_weighted_average_f32.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/CMSIS_DSP/Source/SupportFunctions/%.o Drivers/CMSIS_DSP/Source/SupportFunctions/%.su Drivers/CMSIS_DSP/Source/SupportFunctions/%.cyclo: ../Drivers/CMSIS_DSP/Source/SupportFunctions/%.c Drivers/CMSIS_DSP/Source/SupportFunctions/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m33 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32H573xx -c -I../Core/Inc -I"/home/spectre/Clones/luna/bam/Core/CControl/Headers" -I"/home/spectre/Clones/luna/bam/Drivers/CMSIS_DSP/Include" -I"/home/spectre/Clones/luna/bam/Core/EKF/Inc" -I"/home/spectre/Clones/luna/bam/Drivers/CMSIS_DSP/PrivateInclude" -I../Drivers/STM32H5xx_HAL_Driver/Inc -I../Drivers/STM32H5xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H5xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-CMSIS_DSP-2f-Source-2f-SupportFunctions

clean-Drivers-2f-CMSIS_DSP-2f-Source-2f-SupportFunctions:
	-$(RM) ./Drivers/CMSIS_DSP/Source/SupportFunctions/SupportFunctions.cyclo ./Drivers/CMSIS_DSP/Source/SupportFunctions/SupportFunctions.d ./Drivers/CMSIS_DSP/Source/SupportFunctions/SupportFunctions.o ./Drivers/CMSIS_DSP/Source/SupportFunctions/SupportFunctions.su ./Drivers/CMSIS_DSP/Source/SupportFunctions/SupportFunctionsF16.cyclo ./Drivers/CMSIS_DSP/Source/SupportFunctions/SupportFunctionsF16.d ./Drivers/CMSIS_DSP/Source/SupportFunctions/SupportFunctionsF16.o ./Drivers/CMSIS_DSP/Source/SupportFunctions/SupportFunctionsF16.su ./Drivers/CMSIS_DSP/Source/SupportFunctions/arm_barycenter_f16.cyclo ./Drivers/CMSIS_DSP/Source/SupportFunctions/arm_barycenter_f16.d ./Drivers/CMSIS_DSP/Source/SupportFunctions/arm_barycenter_f16.o ./Drivers/CMSIS_DSP/Source/SupportFunctions/arm_barycenter_f16.su ./Drivers/CMSIS_DSP/Source/SupportFunctions/arm_barycenter_f32.cyclo ./Drivers/CMSIS_DSP/Source/SupportFunctions/arm_barycenter_f32.d ./Drivers/CMSIS_DSP/Source/SupportFunctions/arm_barycenter_f32.o ./Drivers/CMSIS_DSP/Source/SupportFunctions/arm_barycenter_f32.su ./Drivers/CMSIS_DSP/Source/SupportFunctions/arm_bitonic_sort_f32.cyclo ./Drivers/CMSIS_DSP/Source/SupportFunctions/arm_bitonic_sort_f32.d ./Drivers/CMSIS_DSP/Source/SupportFunctions/arm_bitonic_sort_f32.o ./Drivers/CMSIS_DSP/Source/SupportFunctions/arm_bitonic_sort_f32.su ./Drivers/CMSIS_DSP/Source/SupportFunctions/arm_bubble_sort_f32.cyclo ./Drivers/CMSIS_DSP/Source/SupportFunctions/arm_bubble_sort_f32.d ./Drivers/CMSIS_DSP/Source/SupportFunctions/arm_bubble_sort_f32.o ./Drivers/CMSIS_DSP/Source/SupportFunctions/arm_bubble_sort_f32.su ./Drivers/CMSIS_DSP/Source/SupportFunctions/arm_copy_f16.cyclo ./Drivers/CMSIS_DSP/Source/SupportFunctions/arm_copy_f16.d ./Drivers/CMSIS_DSP/Source/SupportFunctions/arm_copy_f16.o ./Drivers/CMSIS_DSP/Source/SupportFunctions/arm_copy_f16.su ./Drivers/CMSIS_DSP/Source/SupportFunctions/arm_copy_f32.cyclo ./Drivers/CMSIS_DSP/Source/SupportFunctions/arm_copy_f32.d ./Drivers/CMSIS_DSP/Source/SupportFunctions/arm_copy_f32.o ./Drivers/CMSIS_DSP/Source/SupportFunctions/arm_copy_f32.su ./Drivers/CMSIS_DSP/Source/SupportFunctions/arm_copy_f64.cyclo ./Drivers/CMSIS_DSP/Source/SupportFunctions/arm_copy_f64.d ./Drivers/CMSIS_DSP/Source/SupportFunctions/arm_copy_f64.o ./Drivers/CMSIS_DSP/Source/SupportFunctions/arm_copy_f64.su ./Drivers/CMSIS_DSP/Source/SupportFunctions/arm_copy_q15.cyclo ./Drivers/CMSIS_DSP/Source/SupportFunctions/arm_copy_q15.d ./Drivers/CMSIS_DSP/Source/SupportFunctions/arm_copy_q15.o ./Drivers/CMSIS_DSP/Source/SupportFunctions/arm_copy_q15.su ./Drivers/CMSIS_DSP/Source/SupportFunctions/arm_copy_q31.cyclo ./Drivers/CMSIS_DSP/Source/SupportFunctions/arm_copy_q31.d ./Drivers/CMSIS_DSP/Source/SupportFunctions/arm_copy_q31.o ./Drivers/CMSIS_DSP/Source/SupportFunctions/arm_copy_q31.su ./Drivers/CMSIS_DSP/Source/SupportFunctions/arm_copy_q7.cyclo ./Drivers/CMSIS_DSP/Source/SupportFunctions/arm_copy_q7.d ./Drivers/CMSIS_DSP/Source/SupportFunctions/arm_copy_q7.o ./Drivers/CMSIS_DSP/Source/SupportFunctions/arm_copy_q7.su ./Drivers/CMSIS_DSP/Source/SupportFunctions/arm_f16_to_f64.cyclo ./Drivers/CMSIS_DSP/Source/SupportFunctions/arm_f16_to_f64.d ./Drivers/CMSIS_DSP/Source/SupportFunctions/arm_f16_to_f64.o ./Drivers/CMSIS_DSP/Source/SupportFunctions/arm_f16_to_f64.su ./Drivers/CMSIS_DSP/Source/SupportFunctions/arm_f16_to_float.cyclo ./Drivers/CMSIS_DSP/Source/SupportFunctions/arm_f16_to_float.d ./Drivers/CMSIS_DSP/Source/SupportFunctions/arm_f16_to_float.o ./Drivers/CMSIS_DSP/Source/SupportFunctions/arm_f16_to_float.su ./Drivers/CMSIS_DSP/Source/SupportFunctions/arm_f16_to_q15.cyclo ./Drivers/CMSIS_DSP/Source/SupportFunctions/arm_f16_to_q15.d ./Drivers/CMSIS_DSP/Source/SupportFunctions/arm_f16_to_q15.o ./Drivers/CMSIS_DSP/Source/SupportFunctions/arm_f16_to_q15.su ./Drivers/CMSIS_DSP/Source/SupportFunctions/arm_f64_to_f16.cyclo ./Drivers/CMSIS_DSP/Source/SupportFunctions/arm_f64_to_f16.d ./Drivers/CMSIS_DSP/Source/SupportFunctions/arm_f64_to_f16.o ./Drivers/CMSIS_DSP/Source/SupportFunctions/arm_f64_to_f16.su ./Drivers/CMSIS_DSP/Source/SupportFunctions/arm_f64_to_float.cyclo ./Drivers/CMSIS_DSP/Source/SupportFunctions/arm_f64_to_float.d ./Drivers/CMSIS_DSP/Source/SupportFunctions/arm_f64_to_float.o ./Drivers/CMSIS_DSP/Source/SupportFunctions/arm_f64_to_float.su ./Drivers/CMSIS_DSP/Source/SupportFunctions/arm_f64_to_q15.cyclo ./Drivers/CMSIS_DSP/Source/SupportFunctions/arm_f64_to_q15.d ./Drivers/CMSIS_DSP/Source/SupportFunctions/arm_f64_to_q15.o ./Drivers/CMSIS_DSP/Source/SupportFunctions/arm_f64_to_q15.su ./Drivers/CMSIS_DSP/Source/SupportFunctions/arm_f64_to_q31.cyclo ./Drivers/CMSIS_DSP/Source/SupportFunctions/arm_f64_to_q31.d ./Drivers/CMSIS_DSP/Source/SupportFunctions/arm_f64_to_q31.o ./Drivers/CMSIS_DSP/Source/SupportFunctions/arm_f64_to_q31.su ./Drivers/CMSIS_DSP/Source/SupportFunctions/arm_f64_to_q7.cyclo ./Drivers/CMSIS_DSP/Source/SupportFunctions/arm_f64_to_q7.d ./Drivers/CMSIS_DSP/Source/SupportFunctions/arm_f64_to_q7.o ./Drivers/CMSIS_DSP/Source/SupportFunctions/arm_f64_to_q7.su ./Drivers/CMSIS_DSP/Source/SupportFunctions/arm_fill_f16.cyclo ./Drivers/CMSIS_DSP/Source/SupportFunctions/arm_fill_f16.d ./Drivers/CMSIS_DSP/Source/SupportFunctions/arm_fill_f16.o ./Drivers/CMSIS_DSP/Source/SupportFunctions/arm_fill_f16.su ./Drivers/CMSIS_DSP/Source/SupportFunctions/arm_fill_f32.cyclo ./Drivers/CMSIS_DSP/Source/SupportFunctions/arm_fill_f32.d ./Drivers/CMSIS_DSP/Source/SupportFunctions/arm_fill_f32.o ./Drivers/CMSIS_DSP/Source/SupportFunctions/arm_fill_f32.su ./Drivers/CMSIS_DSP/Source/SupportFunctions/arm_fill_f64.cyclo ./Drivers/CMSIS_DSP/Source/SupportFunctions/arm_fill_f64.d ./Drivers/CMSIS_DSP/Source/SupportFunctions/arm_fill_f64.o ./Drivers/CMSIS_DSP/Source/SupportFunctions/arm_fill_f64.su ./Drivers/CMSIS_DSP/Source/SupportFunctions/arm_fill_q15.cyclo ./Drivers/CMSIS_DSP/Source/SupportFunctions/arm_fill_q15.d ./Drivers/CMSIS_DSP/Source/SupportFunctions/arm_fill_q15.o
	-$(RM) ./Drivers/CMSIS_DSP/Source/SupportFunctions/arm_fill_q15.su ./Drivers/CMSIS_DSP/Source/SupportFunctions/arm_fill_q31.cyclo ./Drivers/CMSIS_DSP/Source/SupportFunctions/arm_fill_q31.d ./Drivers/CMSIS_DSP/Source/SupportFunctions/arm_fill_q31.o ./Drivers/CMSIS_DSP/Source/SupportFunctions/arm_fill_q31.su ./Drivers/CMSIS_DSP/Source/SupportFunctions/arm_fill_q7.cyclo ./Drivers/CMSIS_DSP/Source/SupportFunctions/arm_fill_q7.d ./Drivers/CMSIS_DSP/Source/SupportFunctions/arm_fill_q7.o ./Drivers/CMSIS_DSP/Source/SupportFunctions/arm_fill_q7.su ./Drivers/CMSIS_DSP/Source/SupportFunctions/arm_float_to_f16.cyclo ./Drivers/CMSIS_DSP/Source/SupportFunctions/arm_float_to_f16.d ./Drivers/CMSIS_DSP/Source/SupportFunctions/arm_float_to_f16.o ./Drivers/CMSIS_DSP/Source/SupportFunctions/arm_float_to_f16.su ./Drivers/CMSIS_DSP/Source/SupportFunctions/arm_float_to_f64.cyclo ./Drivers/CMSIS_DSP/Source/SupportFunctions/arm_float_to_f64.d ./Drivers/CMSIS_DSP/Source/SupportFunctions/arm_float_to_f64.o ./Drivers/CMSIS_DSP/Source/SupportFunctions/arm_float_to_f64.su ./Drivers/CMSIS_DSP/Source/SupportFunctions/arm_float_to_q15.cyclo ./Drivers/CMSIS_DSP/Source/SupportFunctions/arm_float_to_q15.d ./Drivers/CMSIS_DSP/Source/SupportFunctions/arm_float_to_q15.o ./Drivers/CMSIS_DSP/Source/SupportFunctions/arm_float_to_q15.su ./Drivers/CMSIS_DSP/Source/SupportFunctions/arm_float_to_q31.cyclo ./Drivers/CMSIS_DSP/Source/SupportFunctions/arm_float_to_q31.d ./Drivers/CMSIS_DSP/Source/SupportFunctions/arm_float_to_q31.o ./Drivers/CMSIS_DSP/Source/SupportFunctions/arm_float_to_q31.su ./Drivers/CMSIS_DSP/Source/SupportFunctions/arm_float_to_q7.cyclo ./Drivers/CMSIS_DSP/Source/SupportFunctions/arm_float_to_q7.d ./Drivers/CMSIS_DSP/Source/SupportFunctions/arm_float_to_q7.o ./Drivers/CMSIS_DSP/Source/SupportFunctions/arm_float_to_q7.su ./Drivers/CMSIS_DSP/Source/SupportFunctions/arm_heap_sort_f32.cyclo ./Drivers/CMSIS_DSP/Source/SupportFunctions/arm_heap_sort_f32.d ./Drivers/CMSIS_DSP/Source/SupportFunctions/arm_heap_sort_f32.o ./Drivers/CMSIS_DSP/Source/SupportFunctions/arm_heap_sort_f32.su ./Drivers/CMSIS_DSP/Source/SupportFunctions/arm_insertion_sort_f32.cyclo ./Drivers/CMSIS_DSP/Source/SupportFunctions/arm_insertion_sort_f32.d ./Drivers/CMSIS_DSP/Source/SupportFunctions/arm_insertion_sort_f32.o ./Drivers/CMSIS_DSP/Source/SupportFunctions/arm_insertion_sort_f32.su ./Drivers/CMSIS_DSP/Source/SupportFunctions/arm_merge_sort_f32.cyclo ./Drivers/CMSIS_DSP/Source/SupportFunctions/arm_merge_sort_f32.d ./Drivers/CMSIS_DSP/Source/SupportFunctions/arm_merge_sort_f32.o ./Drivers/CMSIS_DSP/Source/SupportFunctions/arm_merge_sort_f32.su ./Drivers/CMSIS_DSP/Source/SupportFunctions/arm_merge_sort_init_f32.cyclo ./Drivers/CMSIS_DSP/Source/SupportFunctions/arm_merge_sort_init_f32.d ./Drivers/CMSIS_DSP/Source/SupportFunctions/arm_merge_sort_init_f32.o ./Drivers/CMSIS_DSP/Source/SupportFunctions/arm_merge_sort_init_f32.su ./Drivers/CMSIS_DSP/Source/SupportFunctions/arm_q15_to_f16.cyclo ./Drivers/CMSIS_DSP/Source/SupportFunctions/arm_q15_to_f16.d ./Drivers/CMSIS_DSP/Source/SupportFunctions/arm_q15_to_f16.o ./Drivers/CMSIS_DSP/Source/SupportFunctions/arm_q15_to_f16.su ./Drivers/CMSIS_DSP/Source/SupportFunctions/arm_q15_to_f64.cyclo ./Drivers/CMSIS_DSP/Source/SupportFunctions/arm_q15_to_f64.d ./Drivers/CMSIS_DSP/Source/SupportFunctions/arm_q15_to_f64.o ./Drivers/CMSIS_DSP/Source/SupportFunctions/arm_q15_to_f64.su ./Drivers/CMSIS_DSP/Source/SupportFunctions/arm_q15_to_float.cyclo ./Drivers/CMSIS_DSP/Source/SupportFunctions/arm_q15_to_float.d ./Drivers/CMSIS_DSP/Source/SupportFunctions/arm_q15_to_float.o ./Drivers/CMSIS_DSP/Source/SupportFunctions/arm_q15_to_float.su ./Drivers/CMSIS_DSP/Source/SupportFunctions/arm_q15_to_q31.cyclo ./Drivers/CMSIS_DSP/Source/SupportFunctions/arm_q15_to_q31.d ./Drivers/CMSIS_DSP/Source/SupportFunctions/arm_q15_to_q31.o ./Drivers/CMSIS_DSP/Source/SupportFunctions/arm_q15_to_q31.su ./Drivers/CMSIS_DSP/Source/SupportFunctions/arm_q15_to_q7.cyclo ./Drivers/CMSIS_DSP/Source/SupportFunctions/arm_q15_to_q7.d ./Drivers/CMSIS_DSP/Source/SupportFunctions/arm_q15_to_q7.o ./Drivers/CMSIS_DSP/Source/SupportFunctions/arm_q15_to_q7.su ./Drivers/CMSIS_DSP/Source/SupportFunctions/arm_q31_to_f64.cyclo ./Drivers/CMSIS_DSP/Source/SupportFunctions/arm_q31_to_f64.d ./Drivers/CMSIS_DSP/Source/SupportFunctions/arm_q31_to_f64.o ./Drivers/CMSIS_DSP/Source/SupportFunctions/arm_q31_to_f64.su ./Drivers/CMSIS_DSP/Source/SupportFunctions/arm_q31_to_float.cyclo ./Drivers/CMSIS_DSP/Source/SupportFunctions/arm_q31_to_float.d ./Drivers/CMSIS_DSP/Source/SupportFunctions/arm_q31_to_float.o ./Drivers/CMSIS_DSP/Source/SupportFunctions/arm_q31_to_float.su ./Drivers/CMSIS_DSP/Source/SupportFunctions/arm_q31_to_q15.cyclo ./Drivers/CMSIS_DSP/Source/SupportFunctions/arm_q31_to_q15.d ./Drivers/CMSIS_DSP/Source/SupportFunctions/arm_q31_to_q15.o ./Drivers/CMSIS_DSP/Source/SupportFunctions/arm_q31_to_q15.su ./Drivers/CMSIS_DSP/Source/SupportFunctions/arm_q31_to_q7.cyclo ./Drivers/CMSIS_DSP/Source/SupportFunctions/arm_q31_to_q7.d ./Drivers/CMSIS_DSP/Source/SupportFunctions/arm_q31_to_q7.o ./Drivers/CMSIS_DSP/Source/SupportFunctions/arm_q31_to_q7.su ./Drivers/CMSIS_DSP/Source/SupportFunctions/arm_q7_to_f64.cyclo ./Drivers/CMSIS_DSP/Source/SupportFunctions/arm_q7_to_f64.d ./Drivers/CMSIS_DSP/Source/SupportFunctions/arm_q7_to_f64.o ./Drivers/CMSIS_DSP/Source/SupportFunctions/arm_q7_to_f64.su ./Drivers/CMSIS_DSP/Source/SupportFunctions/arm_q7_to_float.cyclo ./Drivers/CMSIS_DSP/Source/SupportFunctions/arm_q7_to_float.d ./Drivers/CMSIS_DSP/Source/SupportFunctions/arm_q7_to_float.o ./Drivers/CMSIS_DSP/Source/SupportFunctions/arm_q7_to_float.su ./Drivers/CMSIS_DSP/Source/SupportFunctions/arm_q7_to_q15.cyclo ./Drivers/CMSIS_DSP/Source/SupportFunctions/arm_q7_to_q15.d ./Drivers/CMSIS_DSP/Source/SupportFunctions/arm_q7_to_q15.o ./Drivers/CMSIS_DSP/Source/SupportFunctions/arm_q7_to_q15.su ./Drivers/CMSIS_DSP/Source/SupportFunctions/arm_q7_to_q31.cyclo
	-$(RM) ./Drivers/CMSIS_DSP/Source/SupportFunctions/arm_q7_to_q31.d ./Drivers/CMSIS_DSP/Source/SupportFunctions/arm_q7_to_q31.o ./Drivers/CMSIS_DSP/Source/SupportFunctions/arm_q7_to_q31.su ./Drivers/CMSIS_DSP/Source/SupportFunctions/arm_quick_sort_f32.cyclo ./Drivers/CMSIS_DSP/Source/SupportFunctions/arm_quick_sort_f32.d ./Drivers/CMSIS_DSP/Source/SupportFunctions/arm_quick_sort_f32.o ./Drivers/CMSIS_DSP/Source/SupportFunctions/arm_quick_sort_f32.su ./Drivers/CMSIS_DSP/Source/SupportFunctions/arm_selection_sort_f32.cyclo ./Drivers/CMSIS_DSP/Source/SupportFunctions/arm_selection_sort_f32.d ./Drivers/CMSIS_DSP/Source/SupportFunctions/arm_selection_sort_f32.o ./Drivers/CMSIS_DSP/Source/SupportFunctions/arm_selection_sort_f32.su ./Drivers/CMSIS_DSP/Source/SupportFunctions/arm_sort_f32.cyclo ./Drivers/CMSIS_DSP/Source/SupportFunctions/arm_sort_f32.d ./Drivers/CMSIS_DSP/Source/SupportFunctions/arm_sort_f32.o ./Drivers/CMSIS_DSP/Source/SupportFunctions/arm_sort_f32.su ./Drivers/CMSIS_DSP/Source/SupportFunctions/arm_sort_init_f32.cyclo ./Drivers/CMSIS_DSP/Source/SupportFunctions/arm_sort_init_f32.d ./Drivers/CMSIS_DSP/Source/SupportFunctions/arm_sort_init_f32.o ./Drivers/CMSIS_DSP/Source/SupportFunctions/arm_sort_init_f32.su ./Drivers/CMSIS_DSP/Source/SupportFunctions/arm_weighted_average_f16.cyclo ./Drivers/CMSIS_DSP/Source/SupportFunctions/arm_weighted_average_f16.d ./Drivers/CMSIS_DSP/Source/SupportFunctions/arm_weighted_average_f16.o ./Drivers/CMSIS_DSP/Source/SupportFunctions/arm_weighted_average_f16.su ./Drivers/CMSIS_DSP/Source/SupportFunctions/arm_weighted_average_f32.cyclo ./Drivers/CMSIS_DSP/Source/SupportFunctions/arm_weighted_average_f32.d ./Drivers/CMSIS_DSP/Source/SupportFunctions/arm_weighted_average_f32.o ./Drivers/CMSIS_DSP/Source/SupportFunctions/arm_weighted_average_f32.su

.PHONY: clean-Drivers-2f-CMSIS_DSP-2f-Source-2f-SupportFunctions

