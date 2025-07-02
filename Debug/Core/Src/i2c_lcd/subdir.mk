################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/i2c_lcd/i2c_lcd.c 

OBJS += \
./Core/Src/i2c_lcd/i2c_lcd.o 

C_DEPS += \
./Core/Src/i2c_lcd/i2c_lcd.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/i2c_lcd/%.o Core/Src/i2c_lcd/%.su: ../Core/Src/i2c_lcd/%.c Core/Src/i2c_lcd/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xB -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-i2c_lcd

clean-Core-2f-Src-2f-i2c_lcd:
	-$(RM) ./Core/Src/i2c_lcd/i2c_lcd.d ./Core/Src/i2c_lcd/i2c_lcd.o ./Core/Src/i2c_lcd/i2c_lcd.su

.PHONY: clean-Core-2f-Src-2f-i2c_lcd

