################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/app.c \
../Core/pidctl.c \
../Core/retargetio.c 

OBJS += \
./Core/app.o \
./Core/pidctl.o \
./Core/retargetio.o 

C_DEPS += \
./Core/app.d \
./Core/pidctl.d \
./Core/retargetio.d 


# Each subdirectory must supply rules for building sources it contributes
Core/%.o Core/%.su Core/%.cyclo: ../Core/%.c Core/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L432xx -c -I../Core/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/CMSIS/Include -I../Core -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core

clean-Core:
	-$(RM) ./Core/app.cyclo ./Core/app.d ./Core/app.o ./Core/app.su ./Core/pidctl.cyclo ./Core/pidctl.d ./Core/pidctl.o ./Core/pidctl.su ./Core/retargetio.cyclo ./Core/retargetio.d ./Core/retargetio.o ./Core/retargetio.su

.PHONY: clean-Core

