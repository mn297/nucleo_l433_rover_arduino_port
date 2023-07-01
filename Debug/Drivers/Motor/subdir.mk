################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/Motor/AMT22.c 

CPP_SRCS += \
../Drivers/Motor/RoverArmMotor.cpp \
../Drivers/Motor/movingAvg.cpp \
../Drivers/Motor/pid.cpp 

C_DEPS += \
./Drivers/Motor/AMT22.d 

OBJS += \
./Drivers/Motor/AMT22.o \
./Drivers/Motor/RoverArmMotor.o \
./Drivers/Motor/movingAvg.o \
./Drivers/Motor/pid.o 

CPP_DEPS += \
./Drivers/Motor/RoverArmMotor.d \
./Drivers/Motor/movingAvg.d \
./Drivers/Motor/pid.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/Motor/%.o Drivers/Motor/%.su Drivers/Motor/%.cyclo: ../Drivers/Motor/%.c Drivers/Motor/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L433xx -c -I../Core/Inc -I../Drivers/Motor -I../Drivers/STM32_AMT22_lib -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Drivers/Motor/%.o Drivers/Motor/%.su Drivers/Motor/%.cyclo: ../Drivers/Motor/%.cpp Drivers/Motor/subdir.mk
	arm-none-eabi-g++ "$<" -mcpu=cortex-m4 -std=gnu++14 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L433xx -c -I../Core/Inc -I../Drivers/Motor -I../Drivers/STM32_AMT22_lib -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -fno-strict-aliasing -fno-exceptions -fno-rtti -fno-use-cxa-atexit -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-Motor

clean-Drivers-2f-Motor:
	-$(RM) ./Drivers/Motor/AMT22.cyclo ./Drivers/Motor/AMT22.d ./Drivers/Motor/AMT22.o ./Drivers/Motor/AMT22.su ./Drivers/Motor/RoverArmMotor.cyclo ./Drivers/Motor/RoverArmMotor.d ./Drivers/Motor/RoverArmMotor.o ./Drivers/Motor/RoverArmMotor.su ./Drivers/Motor/movingAvg.cyclo ./Drivers/Motor/movingAvg.d ./Drivers/Motor/movingAvg.o ./Drivers/Motor/movingAvg.su ./Drivers/Motor/pid.cyclo ./Drivers/Motor/pid.d ./Drivers/Motor/pid.o ./Drivers/Motor/pid.su

.PHONY: clean-Drivers-2f-Motor

