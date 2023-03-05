################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../Drivers/Motor/RoverArmMotor.cpp \
../Drivers/Motor/movingAvg.cpp \
../Drivers/Motor/pid.cpp 

OBJS += \
./Drivers/Motor/RoverArmMotor.o \
./Drivers/Motor/movingAvg.o \
./Drivers/Motor/pid.o 

CPP_DEPS += \
./Drivers/Motor/RoverArmMotor.d \
./Drivers/Motor/movingAvg.d \
./Drivers/Motor/pid.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/Motor/%.o Drivers/Motor/%.su: ../Drivers/Motor/%.cpp Drivers/Motor/subdir.mk
	arm-none-eabi-g++ "$<" -mcpu=cortex-m4 -std=gnu++14 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L433xx -c -I../Core/Inc -I../Drivers/Motor -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/CMSIS/Include -I../Drivers/STM32_AMT22_lib -O0 -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-use-cxa-atexit -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-Motor

clean-Drivers-2f-Motor:
	-$(RM) ./Drivers/Motor/RoverArmMotor.d ./Drivers/Motor/RoverArmMotor.o ./Drivers/Motor/RoverArmMotor.su ./Drivers/Motor/movingAvg.d ./Drivers/Motor/movingAvg.o ./Drivers/Motor/movingAvg.su ./Drivers/Motor/pid.d ./Drivers/Motor/pid.o ./Drivers/Motor/pid.su

.PHONY: clean-Drivers-2f-Motor

