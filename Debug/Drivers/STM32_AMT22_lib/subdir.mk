################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/STM32_AMT22_lib/AMT22.c 

C_DEPS += \
./Drivers/STM32_AMT22_lib/AMT22.d 

OBJS += \
./Drivers/STM32_AMT22_lib/AMT22.o 


# Each subdirectory must supply rules for building sources it contributes
Drivers/STM32_AMT22_lib/%.o Drivers/STM32_AMT22_lib/%.su Drivers/STM32_AMT22_lib/%.cyclo: ../Drivers/STM32_AMT22_lib/%.c Drivers/STM32_AMT22_lib/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L433xx -c -I../Core/Inc -I../Drivers/Motor -I../Drivers/STM32_AMT22_lib -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-STM32_AMT22_lib

clean-Drivers-2f-STM32_AMT22_lib:
	-$(RM) ./Drivers/STM32_AMT22_lib/AMT22.cyclo ./Drivers/STM32_AMT22_lib/AMT22.d ./Drivers/STM32_AMT22_lib/AMT22.o ./Drivers/STM32_AMT22_lib/AMT22.su

.PHONY: clean-Drivers-2f-STM32_AMT22_lib

