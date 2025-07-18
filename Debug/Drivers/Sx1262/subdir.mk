################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/Sx1262/lr_fhss_mac.c \
../Drivers/Sx1262/sx126x.c \
../Drivers/Sx1262/sx126x_driver_version.c \
../Drivers/Sx1262/sx126x_hal.c \
../Drivers/Sx1262/sx126x_lr_fhss.c 

OBJS += \
./Drivers/Sx1262/lr_fhss_mac.o \
./Drivers/Sx1262/sx126x.o \
./Drivers/Sx1262/sx126x_driver_version.o \
./Drivers/Sx1262/sx126x_hal.o \
./Drivers/Sx1262/sx126x_lr_fhss.o 

C_DEPS += \
./Drivers/Sx1262/lr_fhss_mac.d \
./Drivers/Sx1262/sx126x.d \
./Drivers/Sx1262/sx126x_driver_version.d \
./Drivers/Sx1262/sx126x_hal.d \
./Drivers/Sx1262/sx126x_lr_fhss.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/Sx1262/%.o Drivers/Sx1262/%.su Drivers/Sx1262/%.cyclo: ../Drivers/Sx1262/%.c Drivers/Sx1262/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0plus -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32G071xx -c -I../Core/Inc -I../Drivers/STM32G0xx_HAL_Driver/Inc -I../Drivers/STM32G0xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32G0xx/Include -I../Drivers/CMSIS/Include -I../Drivers/Sx1262 -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Drivers-2f-Sx1262

clean-Drivers-2f-Sx1262:
	-$(RM) ./Drivers/Sx1262/lr_fhss_mac.cyclo ./Drivers/Sx1262/lr_fhss_mac.d ./Drivers/Sx1262/lr_fhss_mac.o ./Drivers/Sx1262/lr_fhss_mac.su ./Drivers/Sx1262/sx126x.cyclo ./Drivers/Sx1262/sx126x.d ./Drivers/Sx1262/sx126x.o ./Drivers/Sx1262/sx126x.su ./Drivers/Sx1262/sx126x_driver_version.cyclo ./Drivers/Sx1262/sx126x_driver_version.d ./Drivers/Sx1262/sx126x_driver_version.o ./Drivers/Sx1262/sx126x_driver_version.su ./Drivers/Sx1262/sx126x_hal.cyclo ./Drivers/Sx1262/sx126x_hal.d ./Drivers/Sx1262/sx126x_hal.o ./Drivers/Sx1262/sx126x_hal.su ./Drivers/Sx1262/sx126x_lr_fhss.cyclo ./Drivers/Sx1262/sx126x_lr_fhss.d ./Drivers/Sx1262/sx126x_lr_fhss.o ./Drivers/Sx1262/sx126x_lr_fhss.su

.PHONY: clean-Drivers-2f-Sx1262

