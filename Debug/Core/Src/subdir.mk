################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/Firm.c \
../Core/Src/INA219.c \
../Core/Src/esp_loader.c \
../Core/Src/esp_targets.c \
../Core/Src/example_common.c \
../Core/Src/main.c \
../Core/Src/md5_hash.c \
../Core/Src/protocol.c \
../Core/Src/slip.c \
../Core/Src/stm32_port.c \
../Core/Src/stm32f4xx_hal_msp.c \
../Core/Src/stm32f4xx_it.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32f4xx.c 

OBJS += \
./Core/Src/Firm.o \
./Core/Src/INA219.o \
./Core/Src/esp_loader.o \
./Core/Src/esp_targets.o \
./Core/Src/example_common.o \
./Core/Src/main.o \
./Core/Src/md5_hash.o \
./Core/Src/protocol.o \
./Core/Src/slip.o \
./Core/Src/stm32_port.o \
./Core/Src/stm32f4xx_hal_msp.o \
./Core/Src/stm32f4xx_it.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32f4xx.o 

C_DEPS += \
./Core/Src/Firm.d \
./Core/Src/INA219.d \
./Core/Src/esp_loader.d \
./Core/Src/esp_targets.d \
./Core/Src/example_common.d \
./Core/Src/main.d \
./Core/Src/md5_hash.d \
./Core/Src/protocol.d \
./Core/Src/slip.d \
./Core/Src/stm32_port.d \
./Core/Src/stm32f4xx_hal_msp.d \
./Core/Src/stm32f4xx_it.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32f4xx.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o Core/Src/%.su: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F407xx -c -I../USB_DEVICE/App -I../USB_DEVICE/Target -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/Firm.d ./Core/Src/Firm.o ./Core/Src/Firm.su ./Core/Src/INA219.d ./Core/Src/INA219.o ./Core/Src/INA219.su ./Core/Src/esp_loader.d ./Core/Src/esp_loader.o ./Core/Src/esp_loader.su ./Core/Src/esp_targets.d ./Core/Src/esp_targets.o ./Core/Src/esp_targets.su ./Core/Src/example_common.d ./Core/Src/example_common.o ./Core/Src/example_common.su ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/main.su ./Core/Src/md5_hash.d ./Core/Src/md5_hash.o ./Core/Src/md5_hash.su ./Core/Src/protocol.d ./Core/Src/protocol.o ./Core/Src/protocol.su ./Core/Src/slip.d ./Core/Src/slip.o ./Core/Src/slip.su ./Core/Src/stm32_port.d ./Core/Src/stm32_port.o ./Core/Src/stm32_port.su ./Core/Src/stm32f4xx_hal_msp.d ./Core/Src/stm32f4xx_hal_msp.o ./Core/Src/stm32f4xx_hal_msp.su ./Core/Src/stm32f4xx_it.d ./Core/Src/stm32f4xx_it.o ./Core/Src/stm32f4xx_it.su ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/syscalls.su ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/sysmem.su ./Core/Src/system_stm32f4xx.d ./Core/Src/system_stm32f4xx.o ./Core/Src/system_stm32f4xx.su

.PHONY: clean-Core-2f-Src

