################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/008_spi_txrx_arduino.c 

OBJS += \
./Src/008_spi_txrx_arduino.o 

C_DEPS += \
./Src/008_spi_txrx_arduino.d 


# Each subdirectory must supply rules for building sources it contributes
Src/008_spi_txrx_arduino.o: ../Src/008_spi_txrx_arduino.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DSTM32 -DSTM32F4 -DSTM32F446RETx -DDEBUG -DNUCLEO_F446RE -c -I../Inc -I"C:/Users/User/Documents/MCU1/stm32f4xx_drivers/drivers/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Src/008_spi_txrx_arduino.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

