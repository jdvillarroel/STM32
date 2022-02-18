################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../E2S/Src/e2s_AMFS.c 

OBJS += \
./E2S/Src/e2s_AMFS.o 

C_DEPS += \
./E2S/Src/e2s_AMFS.d 


# Each subdirectory must supply rules for building sources it contributes
E2S/Src/%.o: ../E2S/Src/%.c E2S/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F446xx -c -I../Core/Inc -I"I:/My Drive/Coding/Firmware/STM32/AMFS/E2S/Inc" -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-E2S-2f-Src

clean-E2S-2f-Src:
	-$(RM) ./E2S/Src/e2s_AMFS.d ./E2S/Src/e2s_AMFS.o

.PHONY: clean-E2S-2f-Src

