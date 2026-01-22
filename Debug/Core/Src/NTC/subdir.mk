################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/NTC/aio.c \
../Core/Src/NTC/ntc.c \
../Core/Src/NTC/ntc_config.c 

C_DEPS += \
./Core/Src/NTC/aio.d \
./Core/Src/NTC/ntc.d \
./Core/Src/NTC/ntc_config.d 

OBJS += \
./Core/Src/NTC/aio.o \
./Core/Src/NTC/ntc.o \
./Core/Src/NTC/ntc_config.o 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/NTC/%.o Core/Src/NTC/%.su Core/Src/NTC/%.cyclo: ../Core/Src/NTC/%.c Core/Src/NTC/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F746xx -c -I../Core/Inc -I../Drivers/STM32F7xx_HAL_Driver/Inc -I../Drivers/STM32F7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F7xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-NTC

clean-Core-2f-Src-2f-NTC:
	-$(RM) ./Core/Src/NTC/aio.cyclo ./Core/Src/NTC/aio.d ./Core/Src/NTC/aio.o ./Core/Src/NTC/aio.su ./Core/Src/NTC/ntc.cyclo ./Core/Src/NTC/ntc.d ./Core/Src/NTC/ntc.o ./Core/Src/NTC/ntc.su ./Core/Src/NTC/ntc_config.cyclo ./Core/Src/NTC/ntc_config.d ./Core/Src/NTC/ntc_config.o ./Core/Src/NTC/ntc_config.su

.PHONY: clean-Core-2f-Src-2f-NTC

