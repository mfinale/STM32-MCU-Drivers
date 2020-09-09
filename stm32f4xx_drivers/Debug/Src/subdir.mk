################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/012\ i2c_masterrx_testing_IT.c 

OBJS += \
./Src/012\ i2c_masterrx_testing_IT.o 

C_DEPS += \
./Src/012\ i2c_masterrx_testing_IT.d 


# Each subdirectory must supply rules for building sources it contributes
Src/012\ i2c_masterrx_testing_IT.o: ../Src/012\ i2c_masterrx_testing_IT.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DSTM32 -DSTM32F4 -DSTM32F446RETx -DDEBUG -DNUCLEO_F446RE -c -I../Inc -I"C:/Users/User/Desktop/GIT FOLDER/STM32-MCU-Drivers/stm32f4xx_drivers/drivers/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Src/012 i2c_masterrx_testing_IT.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

