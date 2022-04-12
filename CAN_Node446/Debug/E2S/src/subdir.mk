################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../E2S/src/Flash_M4.c 

OBJS += \
./E2S/src/Flash_M4.o 

C_DEPS += \
./E2S/src/Flash_M4.d 


# Each subdirectory must supply rules for building sources it contributes
E2S/src/%.o E2S/src/%.su: ../E2S/src/%.c E2S/src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F446xx -c -I../Core/Inc -I"E:/Electronics/CubeIDE Projects/Courses/Mastering-Microcontroller-Udemy/CAN_Node446/E2S/Inc" -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-E2S-2f-src

clean-E2S-2f-src:
	-$(RM) ./E2S/src/Flash_M4.d ./E2S/src/Flash_M4.o ./E2S/src/Flash_M4.su

.PHONY: clean-E2S-2f-src

