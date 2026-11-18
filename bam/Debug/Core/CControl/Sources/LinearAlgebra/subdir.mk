################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/CControl/Sources/LinearAlgebra/balance.c \
../Core/CControl/Sources/LinearAlgebra/chol.c \
../Core/CControl/Sources/LinearAlgebra/cholupdate.c \
../Core/CControl/Sources/LinearAlgebra/conv.c \
../Core/CControl/Sources/LinearAlgebra/conv2.c \
../Core/CControl/Sources/LinearAlgebra/conv2fft.c \
../Core/CControl/Sources/LinearAlgebra/convfft.c \
../Core/CControl/Sources/LinearAlgebra/det.c \
../Core/CControl/Sources/LinearAlgebra/dlyap.c \
../Core/CControl/Sources/LinearAlgebra/dot.c \
../Core/CControl/Sources/LinearAlgebra/eig.c \
../Core/CControl/Sources/LinearAlgebra/eig_generalized.c \
../Core/CControl/Sources/LinearAlgebra/expm.c \
../Core/CControl/Sources/LinearAlgebra/eye.c \
../Core/CControl/Sources/LinearAlgebra/hankel.c \
../Core/CControl/Sources/LinearAlgebra/inv.c \
../Core/CControl/Sources/LinearAlgebra/linsolve_chol.c \
../Core/CControl/Sources/LinearAlgebra/linsolve_gauss.c \
../Core/CControl/Sources/LinearAlgebra/linsolve_lower_triangular.c \
../Core/CControl/Sources/LinearAlgebra/linsolve_lup.c \
../Core/CControl/Sources/LinearAlgebra/linsolve_qr.c \
../Core/CControl/Sources/LinearAlgebra/linsolve_upper_triangular.c \
../Core/CControl/Sources/LinearAlgebra/lup.c \
../Core/CControl/Sources/LinearAlgebra/mul.c \
../Core/CControl/Sources/LinearAlgebra/nonlinsolve.c \
../Core/CControl/Sources/LinearAlgebra/norm.c \
../Core/CControl/Sources/LinearAlgebra/pinv.c \
../Core/CControl/Sources/LinearAlgebra/qr.c \
../Core/CControl/Sources/LinearAlgebra/rank.c \
../Core/CControl/Sources/LinearAlgebra/svd.c \
../Core/CControl/Sources/LinearAlgebra/tran.c 

OBJS += \
./Core/CControl/Sources/LinearAlgebra/balance.o \
./Core/CControl/Sources/LinearAlgebra/chol.o \
./Core/CControl/Sources/LinearAlgebra/cholupdate.o \
./Core/CControl/Sources/LinearAlgebra/conv.o \
./Core/CControl/Sources/LinearAlgebra/conv2.o \
./Core/CControl/Sources/LinearAlgebra/conv2fft.o \
./Core/CControl/Sources/LinearAlgebra/convfft.o \
./Core/CControl/Sources/LinearAlgebra/det.o \
./Core/CControl/Sources/LinearAlgebra/dlyap.o \
./Core/CControl/Sources/LinearAlgebra/dot.o \
./Core/CControl/Sources/LinearAlgebra/eig.o \
./Core/CControl/Sources/LinearAlgebra/eig_generalized.o \
./Core/CControl/Sources/LinearAlgebra/expm.o \
./Core/CControl/Sources/LinearAlgebra/eye.o \
./Core/CControl/Sources/LinearAlgebra/hankel.o \
./Core/CControl/Sources/LinearAlgebra/inv.o \
./Core/CControl/Sources/LinearAlgebra/linsolve_chol.o \
./Core/CControl/Sources/LinearAlgebra/linsolve_gauss.o \
./Core/CControl/Sources/LinearAlgebra/linsolve_lower_triangular.o \
./Core/CControl/Sources/LinearAlgebra/linsolve_lup.o \
./Core/CControl/Sources/LinearAlgebra/linsolve_qr.o \
./Core/CControl/Sources/LinearAlgebra/linsolve_upper_triangular.o \
./Core/CControl/Sources/LinearAlgebra/lup.o \
./Core/CControl/Sources/LinearAlgebra/mul.o \
./Core/CControl/Sources/LinearAlgebra/nonlinsolve.o \
./Core/CControl/Sources/LinearAlgebra/norm.o \
./Core/CControl/Sources/LinearAlgebra/pinv.o \
./Core/CControl/Sources/LinearAlgebra/qr.o \
./Core/CControl/Sources/LinearAlgebra/rank.o \
./Core/CControl/Sources/LinearAlgebra/svd.o \
./Core/CControl/Sources/LinearAlgebra/tran.o 

C_DEPS += \
./Core/CControl/Sources/LinearAlgebra/balance.d \
./Core/CControl/Sources/LinearAlgebra/chol.d \
./Core/CControl/Sources/LinearAlgebra/cholupdate.d \
./Core/CControl/Sources/LinearAlgebra/conv.d \
./Core/CControl/Sources/LinearAlgebra/conv2.d \
./Core/CControl/Sources/LinearAlgebra/conv2fft.d \
./Core/CControl/Sources/LinearAlgebra/convfft.d \
./Core/CControl/Sources/LinearAlgebra/det.d \
./Core/CControl/Sources/LinearAlgebra/dlyap.d \
./Core/CControl/Sources/LinearAlgebra/dot.d \
./Core/CControl/Sources/LinearAlgebra/eig.d \
./Core/CControl/Sources/LinearAlgebra/eig_generalized.d \
./Core/CControl/Sources/LinearAlgebra/expm.d \
./Core/CControl/Sources/LinearAlgebra/eye.d \
./Core/CControl/Sources/LinearAlgebra/hankel.d \
./Core/CControl/Sources/LinearAlgebra/inv.d \
./Core/CControl/Sources/LinearAlgebra/linsolve_chol.d \
./Core/CControl/Sources/LinearAlgebra/linsolve_gauss.d \
./Core/CControl/Sources/LinearAlgebra/linsolve_lower_triangular.d \
./Core/CControl/Sources/LinearAlgebra/linsolve_lup.d \
./Core/CControl/Sources/LinearAlgebra/linsolve_qr.d \
./Core/CControl/Sources/LinearAlgebra/linsolve_upper_triangular.d \
./Core/CControl/Sources/LinearAlgebra/lup.d \
./Core/CControl/Sources/LinearAlgebra/mul.d \
./Core/CControl/Sources/LinearAlgebra/nonlinsolve.d \
./Core/CControl/Sources/LinearAlgebra/norm.d \
./Core/CControl/Sources/LinearAlgebra/pinv.d \
./Core/CControl/Sources/LinearAlgebra/qr.d \
./Core/CControl/Sources/LinearAlgebra/rank.d \
./Core/CControl/Sources/LinearAlgebra/svd.d \
./Core/CControl/Sources/LinearAlgebra/tran.d 


# Each subdirectory must supply rules for building sources it contributes
Core/CControl/Sources/LinearAlgebra/%.o Core/CControl/Sources/LinearAlgebra/%.su Core/CControl/Sources/LinearAlgebra/%.cyclo: ../Core/CControl/Sources/LinearAlgebra/%.c Core/CControl/Sources/LinearAlgebra/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m33 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32H573xx -c -I../Core/Inc -I"/home/spectre/Clones/luna/bam/Core/CControl/Headers" -I"/home/spectre/Clones/luna/bam/Drivers/CMSIS_DSP/Include" -I"/home/spectre/Clones/luna/bam/Core/EKF/Inc" -I"/home/spectre/Clones/luna/bam/Drivers/CMSIS_DSP/PrivateInclude" -I../Drivers/STM32H5xx_HAL_Driver/Inc -I../Drivers/STM32H5xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H5xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-CControl-2f-Sources-2f-LinearAlgebra

clean-Core-2f-CControl-2f-Sources-2f-LinearAlgebra:
	-$(RM) ./Core/CControl/Sources/LinearAlgebra/balance.cyclo ./Core/CControl/Sources/LinearAlgebra/balance.d ./Core/CControl/Sources/LinearAlgebra/balance.o ./Core/CControl/Sources/LinearAlgebra/balance.su ./Core/CControl/Sources/LinearAlgebra/chol.cyclo ./Core/CControl/Sources/LinearAlgebra/chol.d ./Core/CControl/Sources/LinearAlgebra/chol.o ./Core/CControl/Sources/LinearAlgebra/chol.su ./Core/CControl/Sources/LinearAlgebra/cholupdate.cyclo ./Core/CControl/Sources/LinearAlgebra/cholupdate.d ./Core/CControl/Sources/LinearAlgebra/cholupdate.o ./Core/CControl/Sources/LinearAlgebra/cholupdate.su ./Core/CControl/Sources/LinearAlgebra/conv.cyclo ./Core/CControl/Sources/LinearAlgebra/conv.d ./Core/CControl/Sources/LinearAlgebra/conv.o ./Core/CControl/Sources/LinearAlgebra/conv.su ./Core/CControl/Sources/LinearAlgebra/conv2.cyclo ./Core/CControl/Sources/LinearAlgebra/conv2.d ./Core/CControl/Sources/LinearAlgebra/conv2.o ./Core/CControl/Sources/LinearAlgebra/conv2.su ./Core/CControl/Sources/LinearAlgebra/conv2fft.cyclo ./Core/CControl/Sources/LinearAlgebra/conv2fft.d ./Core/CControl/Sources/LinearAlgebra/conv2fft.o ./Core/CControl/Sources/LinearAlgebra/conv2fft.su ./Core/CControl/Sources/LinearAlgebra/convfft.cyclo ./Core/CControl/Sources/LinearAlgebra/convfft.d ./Core/CControl/Sources/LinearAlgebra/convfft.o ./Core/CControl/Sources/LinearAlgebra/convfft.su ./Core/CControl/Sources/LinearAlgebra/det.cyclo ./Core/CControl/Sources/LinearAlgebra/det.d ./Core/CControl/Sources/LinearAlgebra/det.o ./Core/CControl/Sources/LinearAlgebra/det.su ./Core/CControl/Sources/LinearAlgebra/dlyap.cyclo ./Core/CControl/Sources/LinearAlgebra/dlyap.d ./Core/CControl/Sources/LinearAlgebra/dlyap.o ./Core/CControl/Sources/LinearAlgebra/dlyap.su ./Core/CControl/Sources/LinearAlgebra/dot.cyclo ./Core/CControl/Sources/LinearAlgebra/dot.d ./Core/CControl/Sources/LinearAlgebra/dot.o ./Core/CControl/Sources/LinearAlgebra/dot.su ./Core/CControl/Sources/LinearAlgebra/eig.cyclo ./Core/CControl/Sources/LinearAlgebra/eig.d ./Core/CControl/Sources/LinearAlgebra/eig.o ./Core/CControl/Sources/LinearAlgebra/eig.su ./Core/CControl/Sources/LinearAlgebra/eig_generalized.cyclo ./Core/CControl/Sources/LinearAlgebra/eig_generalized.d ./Core/CControl/Sources/LinearAlgebra/eig_generalized.o ./Core/CControl/Sources/LinearAlgebra/eig_generalized.su ./Core/CControl/Sources/LinearAlgebra/expm.cyclo ./Core/CControl/Sources/LinearAlgebra/expm.d ./Core/CControl/Sources/LinearAlgebra/expm.o ./Core/CControl/Sources/LinearAlgebra/expm.su ./Core/CControl/Sources/LinearAlgebra/eye.cyclo ./Core/CControl/Sources/LinearAlgebra/eye.d ./Core/CControl/Sources/LinearAlgebra/eye.o ./Core/CControl/Sources/LinearAlgebra/eye.su ./Core/CControl/Sources/LinearAlgebra/hankel.cyclo ./Core/CControl/Sources/LinearAlgebra/hankel.d ./Core/CControl/Sources/LinearAlgebra/hankel.o ./Core/CControl/Sources/LinearAlgebra/hankel.su ./Core/CControl/Sources/LinearAlgebra/inv.cyclo ./Core/CControl/Sources/LinearAlgebra/inv.d ./Core/CControl/Sources/LinearAlgebra/inv.o ./Core/CControl/Sources/LinearAlgebra/inv.su ./Core/CControl/Sources/LinearAlgebra/linsolve_chol.cyclo ./Core/CControl/Sources/LinearAlgebra/linsolve_chol.d ./Core/CControl/Sources/LinearAlgebra/linsolve_chol.o ./Core/CControl/Sources/LinearAlgebra/linsolve_chol.su ./Core/CControl/Sources/LinearAlgebra/linsolve_gauss.cyclo ./Core/CControl/Sources/LinearAlgebra/linsolve_gauss.d ./Core/CControl/Sources/LinearAlgebra/linsolve_gauss.o ./Core/CControl/Sources/LinearAlgebra/linsolve_gauss.su ./Core/CControl/Sources/LinearAlgebra/linsolve_lower_triangular.cyclo ./Core/CControl/Sources/LinearAlgebra/linsolve_lower_triangular.d ./Core/CControl/Sources/LinearAlgebra/linsolve_lower_triangular.o ./Core/CControl/Sources/LinearAlgebra/linsolve_lower_triangular.su ./Core/CControl/Sources/LinearAlgebra/linsolve_lup.cyclo ./Core/CControl/Sources/LinearAlgebra/linsolve_lup.d ./Core/CControl/Sources/LinearAlgebra/linsolve_lup.o ./Core/CControl/Sources/LinearAlgebra/linsolve_lup.su ./Core/CControl/Sources/LinearAlgebra/linsolve_qr.cyclo ./Core/CControl/Sources/LinearAlgebra/linsolve_qr.d ./Core/CControl/Sources/LinearAlgebra/linsolve_qr.o ./Core/CControl/Sources/LinearAlgebra/linsolve_qr.su ./Core/CControl/Sources/LinearAlgebra/linsolve_upper_triangular.cyclo ./Core/CControl/Sources/LinearAlgebra/linsolve_upper_triangular.d ./Core/CControl/Sources/LinearAlgebra/linsolve_upper_triangular.o ./Core/CControl/Sources/LinearAlgebra/linsolve_upper_triangular.su ./Core/CControl/Sources/LinearAlgebra/lup.cyclo ./Core/CControl/Sources/LinearAlgebra/lup.d ./Core/CControl/Sources/LinearAlgebra/lup.o ./Core/CControl/Sources/LinearAlgebra/lup.su ./Core/CControl/Sources/LinearAlgebra/mul.cyclo ./Core/CControl/Sources/LinearAlgebra/mul.d ./Core/CControl/Sources/LinearAlgebra/mul.o ./Core/CControl/Sources/LinearAlgebra/mul.su ./Core/CControl/Sources/LinearAlgebra/nonlinsolve.cyclo ./Core/CControl/Sources/LinearAlgebra/nonlinsolve.d ./Core/CControl/Sources/LinearAlgebra/nonlinsolve.o ./Core/CControl/Sources/LinearAlgebra/nonlinsolve.su ./Core/CControl/Sources/LinearAlgebra/norm.cyclo ./Core/CControl/Sources/LinearAlgebra/norm.d ./Core/CControl/Sources/LinearAlgebra/norm.o ./Core/CControl/Sources/LinearAlgebra/norm.su ./Core/CControl/Sources/LinearAlgebra/pinv.cyclo ./Core/CControl/Sources/LinearAlgebra/pinv.d ./Core/CControl/Sources/LinearAlgebra/pinv.o ./Core/CControl/Sources/LinearAlgebra/pinv.su ./Core/CControl/Sources/LinearAlgebra/qr.cyclo ./Core/CControl/Sources/LinearAlgebra/qr.d ./Core/CControl/Sources/LinearAlgebra/qr.o ./Core/CControl/Sources/LinearAlgebra/qr.su ./Core/CControl/Sources/LinearAlgebra/rank.cyclo ./Core/CControl/Sources/LinearAlgebra/rank.d ./Core/CControl/Sources/LinearAlgebra/rank.o ./Core/CControl/Sources/LinearAlgebra/rank.su ./Core/CControl/Sources/LinearAlgebra/svd.cyclo ./Core/CControl/Sources/LinearAlgebra/svd.d ./Core/CControl/Sources/LinearAlgebra/svd.o ./Core/CControl/Sources/LinearAlgebra/svd.su
	-$(RM) ./Core/CControl/Sources/LinearAlgebra/tran.cyclo ./Core/CControl/Sources/LinearAlgebra/tran.d ./Core/CControl/Sources/LinearAlgebra/tran.o ./Core/CControl/Sources/LinearAlgebra/tran.su

.PHONY: clean-Core-2f-CControl-2f-Sources-2f-LinearAlgebra

