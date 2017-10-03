################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Application/User/Fonts.c \
../Application/User/SSD1331.c \
D:/STM32/projects/alko_v2/Src/main.c \
D:/STM32/projects/alko_v2/Src/stm32f4xx_hal_msp.c \
D:/STM32/projects/alko_v2/Src/stm32f4xx_it.c 

OBJS += \
./Application/User/Fonts.o \
./Application/User/SSD1331.o \
./Application/User/main.o \
./Application/User/stm32f4xx_hal_msp.o \
./Application/User/stm32f4xx_it.o 

C_DEPS += \
./Application/User/Fonts.d \
./Application/User/SSD1331.d \
./Application/User/main.d \
./Application/User/stm32f4xx_hal_msp.d \
./Application/User/stm32f4xx_it.d 


# Each subdirectory must supply rules for building sources it contributes
Application/User/%.o: ../Application/User/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 '-D__weak=__attribute__((weak))' '-D__packed=__attribute__((__packed__))' -DUSE_HAL_DRIVER -DSTM32F411xE -I"D:/STM32/projects/alko_v2/Inc" -I"D:/STM32/projects/alko_v2/Drivers/STM32F4xx_HAL_Driver/Inc" -I"D:/STM32/projects/alko_v2/Drivers/STM32F4xx_HAL_Driver/Inc/Legacy" -I"D:/STM32/projects/alko_v2/Drivers/CMSIS/Device/ST/STM32F4xx/Include" -I"D:/STM32/projects/alko_v2/Drivers/CMSIS/Include" -I"D:/STM32/projects/alko_v2/Inc"  -Og -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

Application/User/main.o: D:/STM32/projects/alko_v2/Src/main.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 '-D__weak=__attribute__((weak))' '-D__packed=__attribute__((__packed__))' -DUSE_HAL_DRIVER -DSTM32F411xE -I"D:/STM32/projects/alko_v2/Inc" -I"D:/STM32/projects/alko_v2/Drivers/STM32F4xx_HAL_Driver/Inc" -I"D:/STM32/projects/alko_v2/Drivers/STM32F4xx_HAL_Driver/Inc/Legacy" -I"D:/STM32/projects/alko_v2/Drivers/CMSIS/Device/ST/STM32F4xx/Include" -I"D:/STM32/projects/alko_v2/Drivers/CMSIS/Include" -I"D:/STM32/projects/alko_v2/Inc"  -Og -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

Application/User/stm32f4xx_hal_msp.o: D:/STM32/projects/alko_v2/Src/stm32f4xx_hal_msp.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 '-D__weak=__attribute__((weak))' '-D__packed=__attribute__((__packed__))' -DUSE_HAL_DRIVER -DSTM32F411xE -I"D:/STM32/projects/alko_v2/Inc" -I"D:/STM32/projects/alko_v2/Drivers/STM32F4xx_HAL_Driver/Inc" -I"D:/STM32/projects/alko_v2/Drivers/STM32F4xx_HAL_Driver/Inc/Legacy" -I"D:/STM32/projects/alko_v2/Drivers/CMSIS/Device/ST/STM32F4xx/Include" -I"D:/STM32/projects/alko_v2/Drivers/CMSIS/Include" -I"D:/STM32/projects/alko_v2/Inc"  -Og -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

Application/User/stm32f4xx_it.o: D:/STM32/projects/alko_v2/Src/stm32f4xx_it.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 '-D__weak=__attribute__((weak))' '-D__packed=__attribute__((__packed__))' -DUSE_HAL_DRIVER -DSTM32F411xE -I"D:/STM32/projects/alko_v2/Inc" -I"D:/STM32/projects/alko_v2/Drivers/STM32F4xx_HAL_Driver/Inc" -I"D:/STM32/projects/alko_v2/Drivers/STM32F4xx_HAL_Driver/Inc/Legacy" -I"D:/STM32/projects/alko_v2/Drivers/CMSIS/Device/ST/STM32F4xx/Include" -I"D:/STM32/projects/alko_v2/Drivers/CMSIS/Include" -I"D:/STM32/projects/alko_v2/Inc"  -Og -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


