################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/CControl/Sources/Miscellaneous/anglevector.c \
../Core/CControl/Sources/Miscellaneous/cat.c \
../Core/CControl/Sources/Miscellaneous/cumsum.c \
../Core/CControl/Sources/Miscellaneous/cut.c \
../Core/CControl/Sources/Miscellaneous/find.c \
../Core/CControl/Sources/Miscellaneous/float2uint.c \
../Core/CControl/Sources/Miscellaneous/ind2sub.c \
../Core/CControl/Sources/Miscellaneous/insert.c \
../Core/CControl/Sources/Miscellaneous/issymmetric.c \
../Core/CControl/Sources/Miscellaneous/normalize.c \
../Core/CControl/Sources/Miscellaneous/pdist2.c \
../Core/CControl/Sources/Miscellaneous/popcount.c \
../Core/CControl/Sources/Miscellaneous/print.c \
../Core/CControl/Sources/Miscellaneous/radianvector.c \
../Core/CControl/Sources/Miscellaneous/rot2d.c \
../Core/CControl/Sources/Miscellaneous/saturation.c \
../Core/CControl/Sources/Miscellaneous/scalar.c \
../Core/CControl/Sources/Miscellaneous/sign.c \
../Core/CControl/Sources/Miscellaneous/sort.c \
../Core/CControl/Sources/Miscellaneous/sum.c \
../Core/CControl/Sources/Miscellaneous/uint2float.c \
../Core/CControl/Sources/Miscellaneous/unique.c \
../Core/CControl/Sources/Miscellaneous/unit.c \
../Core/CControl/Sources/Miscellaneous/vmax.c \
../Core/CControl/Sources/Miscellaneous/vmin.c 

OBJS += \
./Core/CControl/Sources/Miscellaneous/anglevector.o \
./Core/CControl/Sources/Miscellaneous/cat.o \
./Core/CControl/Sources/Miscellaneous/cumsum.o \
./Core/CControl/Sources/Miscellaneous/cut.o \
./Core/CControl/Sources/Miscellaneous/find.o \
./Core/CControl/Sources/Miscellaneous/float2uint.o \
./Core/CControl/Sources/Miscellaneous/ind2sub.o \
./Core/CControl/Sources/Miscellaneous/insert.o \
./Core/CControl/Sources/Miscellaneous/issymmetric.o \
./Core/CControl/Sources/Miscellaneous/normalize.o \
./Core/CControl/Sources/Miscellaneous/pdist2.o \
./Core/CControl/Sources/Miscellaneous/popcount.o \
./Core/CControl/Sources/Miscellaneous/print.o \
./Core/CControl/Sources/Miscellaneous/radianvector.o \
./Core/CControl/Sources/Miscellaneous/rot2d.o \
./Core/CControl/Sources/Miscellaneous/saturation.o \
./Core/CControl/Sources/Miscellaneous/scalar.o \
./Core/CControl/Sources/Miscellaneous/sign.o \
./Core/CControl/Sources/Miscellaneous/sort.o \
./Core/CControl/Sources/Miscellaneous/sum.o \
./Core/CControl/Sources/Miscellaneous/uint2float.o \
./Core/CControl/Sources/Miscellaneous/unique.o \
./Core/CControl/Sources/Miscellaneous/unit.o \
./Core/CControl/Sources/Miscellaneous/vmax.o \
./Core/CControl/Sources/Miscellaneous/vmin.o 

C_DEPS += \
./Core/CControl/Sources/Miscellaneous/anglevector.d \
./Core/CControl/Sources/Miscellaneous/cat.d \
./Core/CControl/Sources/Miscellaneous/cumsum.d \
./Core/CControl/Sources/Miscellaneous/cut.d \
./Core/CControl/Sources/Miscellaneous/find.d \
./Core/CControl/Sources/Miscellaneous/float2uint.d \
./Core/CControl/Sources/Miscellaneous/ind2sub.d \
./Core/CControl/Sources/Miscellaneous/insert.d \
./Core/CControl/Sources/Miscellaneous/issymmetric.d \
./Core/CControl/Sources/Miscellaneous/normalize.d \
./Core/CControl/Sources/Miscellaneous/pdist2.d \
./Core/CControl/Sources/Miscellaneous/popcount.d \
./Core/CControl/Sources/Miscellaneous/print.d \
./Core/CControl/Sources/Miscellaneous/radianvector.d \
./Core/CControl/Sources/Miscellaneous/rot2d.d \
./Core/CControl/Sources/Miscellaneous/saturation.d \
./Core/CControl/Sources/Miscellaneous/scalar.d \
./Core/CControl/Sources/Miscellaneous/sign.d \
./Core/CControl/Sources/Miscellaneous/sort.d \
./Core/CControl/Sources/Miscellaneous/sum.d \
./Core/CControl/Sources/Miscellaneous/uint2float.d \
./Core/CControl/Sources/Miscellaneous/unique.d \
./Core/CControl/Sources/Miscellaneous/unit.d \
./Core/CControl/Sources/Miscellaneous/vmax.d \
./Core/CControl/Sources/Miscellaneous/vmin.d 


# Each subdirectory must supply rules for building sources it contributes
Core/CControl/Sources/Miscellaneous/%.o Core/CControl/Sources/Miscellaneous/%.su Core/CControl/Sources/Miscellaneous/%.cyclo: ../Core/CControl/Sources/Miscellaneous/%.c Core/CControl/Sources/Miscellaneous/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m33 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32H573xx -c -I../Core/Inc -I"/home/spectre/Clones/luna/bam/Core/CControl/Headers" -I"/home/spectre/Clones/luna/bam/Drivers/CMSIS_DSP/Include" -I"/home/spectre/Clones/luna/bam/Core/EKF/Inc" -I"/home/spectre/Clones/luna/bam/Drivers/CMSIS_DSP/PrivateInclude" -I../Drivers/STM32H5xx_HAL_Driver/Inc -I../Drivers/STM32H5xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H5xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-CControl-2f-Sources-2f-Miscellaneous

clean-Core-2f-CControl-2f-Sources-2f-Miscellaneous:
	-$(RM) ./Core/CControl/Sources/Miscellaneous/anglevector.cyclo ./Core/CControl/Sources/Miscellaneous/anglevector.d ./Core/CControl/Sources/Miscellaneous/anglevector.o ./Core/CControl/Sources/Miscellaneous/anglevector.su ./Core/CControl/Sources/Miscellaneous/cat.cyclo ./Core/CControl/Sources/Miscellaneous/cat.d ./Core/CControl/Sources/Miscellaneous/cat.o ./Core/CControl/Sources/Miscellaneous/cat.su ./Core/CControl/Sources/Miscellaneous/cumsum.cyclo ./Core/CControl/Sources/Miscellaneous/cumsum.d ./Core/CControl/Sources/Miscellaneous/cumsum.o ./Core/CControl/Sources/Miscellaneous/cumsum.su ./Core/CControl/Sources/Miscellaneous/cut.cyclo ./Core/CControl/Sources/Miscellaneous/cut.d ./Core/CControl/Sources/Miscellaneous/cut.o ./Core/CControl/Sources/Miscellaneous/cut.su ./Core/CControl/Sources/Miscellaneous/find.cyclo ./Core/CControl/Sources/Miscellaneous/find.d ./Core/CControl/Sources/Miscellaneous/find.o ./Core/CControl/Sources/Miscellaneous/find.su ./Core/CControl/Sources/Miscellaneous/float2uint.cyclo ./Core/CControl/Sources/Miscellaneous/float2uint.d ./Core/CControl/Sources/Miscellaneous/float2uint.o ./Core/CControl/Sources/Miscellaneous/float2uint.su ./Core/CControl/Sources/Miscellaneous/ind2sub.cyclo ./Core/CControl/Sources/Miscellaneous/ind2sub.d ./Core/CControl/Sources/Miscellaneous/ind2sub.o ./Core/CControl/Sources/Miscellaneous/ind2sub.su ./Core/CControl/Sources/Miscellaneous/insert.cyclo ./Core/CControl/Sources/Miscellaneous/insert.d ./Core/CControl/Sources/Miscellaneous/insert.o ./Core/CControl/Sources/Miscellaneous/insert.su ./Core/CControl/Sources/Miscellaneous/issymmetric.cyclo ./Core/CControl/Sources/Miscellaneous/issymmetric.d ./Core/CControl/Sources/Miscellaneous/issymmetric.o ./Core/CControl/Sources/Miscellaneous/issymmetric.su ./Core/CControl/Sources/Miscellaneous/normalize.cyclo ./Core/CControl/Sources/Miscellaneous/normalize.d ./Core/CControl/Sources/Miscellaneous/normalize.o ./Core/CControl/Sources/Miscellaneous/normalize.su ./Core/CControl/Sources/Miscellaneous/pdist2.cyclo ./Core/CControl/Sources/Miscellaneous/pdist2.d ./Core/CControl/Sources/Miscellaneous/pdist2.o ./Core/CControl/Sources/Miscellaneous/pdist2.su ./Core/CControl/Sources/Miscellaneous/popcount.cyclo ./Core/CControl/Sources/Miscellaneous/popcount.d ./Core/CControl/Sources/Miscellaneous/popcount.o ./Core/CControl/Sources/Miscellaneous/popcount.su ./Core/CControl/Sources/Miscellaneous/print.cyclo ./Core/CControl/Sources/Miscellaneous/print.d ./Core/CControl/Sources/Miscellaneous/print.o ./Core/CControl/Sources/Miscellaneous/print.su ./Core/CControl/Sources/Miscellaneous/radianvector.cyclo ./Core/CControl/Sources/Miscellaneous/radianvector.d ./Core/CControl/Sources/Miscellaneous/radianvector.o ./Core/CControl/Sources/Miscellaneous/radianvector.su ./Core/CControl/Sources/Miscellaneous/rot2d.cyclo ./Core/CControl/Sources/Miscellaneous/rot2d.d ./Core/CControl/Sources/Miscellaneous/rot2d.o ./Core/CControl/Sources/Miscellaneous/rot2d.su ./Core/CControl/Sources/Miscellaneous/saturation.cyclo ./Core/CControl/Sources/Miscellaneous/saturation.d ./Core/CControl/Sources/Miscellaneous/saturation.o ./Core/CControl/Sources/Miscellaneous/saturation.su ./Core/CControl/Sources/Miscellaneous/scalar.cyclo ./Core/CControl/Sources/Miscellaneous/scalar.d ./Core/CControl/Sources/Miscellaneous/scalar.o ./Core/CControl/Sources/Miscellaneous/scalar.su ./Core/CControl/Sources/Miscellaneous/sign.cyclo ./Core/CControl/Sources/Miscellaneous/sign.d ./Core/CControl/Sources/Miscellaneous/sign.o ./Core/CControl/Sources/Miscellaneous/sign.su ./Core/CControl/Sources/Miscellaneous/sort.cyclo ./Core/CControl/Sources/Miscellaneous/sort.d ./Core/CControl/Sources/Miscellaneous/sort.o ./Core/CControl/Sources/Miscellaneous/sort.su ./Core/CControl/Sources/Miscellaneous/sum.cyclo ./Core/CControl/Sources/Miscellaneous/sum.d ./Core/CControl/Sources/Miscellaneous/sum.o ./Core/CControl/Sources/Miscellaneous/sum.su ./Core/CControl/Sources/Miscellaneous/uint2float.cyclo ./Core/CControl/Sources/Miscellaneous/uint2float.d ./Core/CControl/Sources/Miscellaneous/uint2float.o ./Core/CControl/Sources/Miscellaneous/uint2float.su ./Core/CControl/Sources/Miscellaneous/unique.cyclo ./Core/CControl/Sources/Miscellaneous/unique.d ./Core/CControl/Sources/Miscellaneous/unique.o ./Core/CControl/Sources/Miscellaneous/unique.su ./Core/CControl/Sources/Miscellaneous/unit.cyclo ./Core/CControl/Sources/Miscellaneous/unit.d ./Core/CControl/Sources/Miscellaneous/unit.o ./Core/CControl/Sources/Miscellaneous/unit.su ./Core/CControl/Sources/Miscellaneous/vmax.cyclo ./Core/CControl/Sources/Miscellaneous/vmax.d ./Core/CControl/Sources/Miscellaneous/vmax.o ./Core/CControl/Sources/Miscellaneous/vmax.su ./Core/CControl/Sources/Miscellaneous/vmin.cyclo ./Core/CControl/Sources/Miscellaneous/vmin.d ./Core/CControl/Sources/Miscellaneous/vmin.o ./Core/CControl/Sources/Miscellaneous/vmin.su

.PHONY: clean-Core-2f-CControl-2f-Sources-2f-Miscellaneous

