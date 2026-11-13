################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/FC_EC_link/ec_side.c \
../Drivers/FC_EC_link/fc_ec_link.c 

OBJS += \
./Drivers/FC_EC_link/ec_side.o \
./Drivers/FC_EC_link/fc_ec_link.o 

C_DEPS += \
./Drivers/FC_EC_link/ec_side.d \
./Drivers/FC_EC_link/fc_ec_link.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/FC_EC_link/%.o Drivers/FC_EC_link/%.su Drivers/FC_EC_link/%.cyclo: ../Drivers/FC_EC_link/%.c Drivers/FC_EC_link/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DCORE_CM7 -DUSE_HAL_DRIVER -DSTM32H745xx -DUSE_PWR_DIRECT_SMPS_SUPPLY -c -I../Core/Inc -I../../Drivers/STM32H7xx_HAL_Driver/Inc -I../../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../../Drivers/CMSIS/Include -I../../Drivers/FC_EC_link -I../../Drivers/ADIN -I../LWIP/App -I../LWIP/Target -I../../Middlewares/Third_Party/LwIP/src/include -I../../Middlewares/Third_Party/LwIP/system -I../../Middlewares/Third_Party/LwIP/src/include/netif/ppp -I../../Middlewares/Third_Party/LwIP/src/include/lwip -I../../Middlewares/Third_Party/LwIP/src/include/lwip/apps -I../../Middlewares/Third_Party/LwIP/src/include/lwip/priv -I../../Middlewares/Third_Party/LwIP/src/include/lwip/prot -I../../Middlewares/Third_Party/LwIP/src/include/netif -I../../Middlewares/Third_Party/LwIP/src/include/compat/posix -I../../Middlewares/Third_Party/LwIP/src/include/compat/posix/arpa -I../../Middlewares/Third_Party/LwIP/src/include/compat/posix/net -I../../Middlewares/Third_Party/LwIP/src/include/compat/posix/sys -I../../Middlewares/Third_Party/LwIP/src/include/compat/stdc -I../../Middlewares/Third_Party/LwIP/system/arch -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-FC_EC_link

clean-Drivers-2f-FC_EC_link:
	-$(RM) ./Drivers/FC_EC_link/ec_side.cyclo ./Drivers/FC_EC_link/ec_side.d ./Drivers/FC_EC_link/ec_side.o ./Drivers/FC_EC_link/ec_side.su ./Drivers/FC_EC_link/fc_ec_link.cyclo ./Drivers/FC_EC_link/fc_ec_link.d ./Drivers/FC_EC_link/fc_ec_link.o ./Drivers/FC_EC_link/fc_ec_link.su

.PHONY: clean-Drivers-2f-FC_EC_link

