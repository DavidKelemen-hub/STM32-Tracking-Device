################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/FanControl.c \
../Core/Src/GSM_Transmission.c \
../Core/Src/Inputs.c \
../Core/Src/Lights_Module.c \
../Core/Src/Presence_Detection.c \
../Core/Src/Rte.c \
../Core/Src/Servo.c \
../Core/Src/Task_CyclicEvent.c \
../Core/Src/Transmit_Arduino.c \
../Core/Src/freertos.c \
../Core/Src/main.c \
../Core/Src/stm32h7xx_hal_msp.c \
../Core/Src/stm32h7xx_it.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32h7xx.c 

OBJS += \
./Core/Src/FanControl.o \
./Core/Src/GSM_Transmission.o \
./Core/Src/Inputs.o \
./Core/Src/Lights_Module.o \
./Core/Src/Presence_Detection.o \
./Core/Src/Rte.o \
./Core/Src/Servo.o \
./Core/Src/Task_CyclicEvent.o \
./Core/Src/Transmit_Arduino.o \
./Core/Src/freertos.o \
./Core/Src/main.o \
./Core/Src/stm32h7xx_hal_msp.o \
./Core/Src/stm32h7xx_it.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32h7xx.o 

C_DEPS += \
./Core/Src/FanControl.d \
./Core/Src/GSM_Transmission.d \
./Core/Src/Inputs.d \
./Core/Src/Lights_Module.d \
./Core/Src/Presence_Detection.d \
./Core/Src/Rte.d \
./Core/Src/Servo.d \
./Core/Src/Task_CyclicEvent.d \
./Core/Src/Transmit_Arduino.d \
./Core/Src/freertos.d \
./Core/Src/main.d \
./Core/Src/stm32h7xx_hal_msp.d \
./Core/Src/stm32h7xx_it.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32h7xx.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o Core/Src/%.su Core/Src/%.cyclo: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32H7A3xxQ -c -I../Core/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/FanControl.cyclo ./Core/Src/FanControl.d ./Core/Src/FanControl.o ./Core/Src/FanControl.su ./Core/Src/GSM_Transmission.cyclo ./Core/Src/GSM_Transmission.d ./Core/Src/GSM_Transmission.o ./Core/Src/GSM_Transmission.su ./Core/Src/Inputs.cyclo ./Core/Src/Inputs.d ./Core/Src/Inputs.o ./Core/Src/Inputs.su ./Core/Src/Lights_Module.cyclo ./Core/Src/Lights_Module.d ./Core/Src/Lights_Module.o ./Core/Src/Lights_Module.su ./Core/Src/Presence_Detection.cyclo ./Core/Src/Presence_Detection.d ./Core/Src/Presence_Detection.o ./Core/Src/Presence_Detection.su ./Core/Src/Rte.cyclo ./Core/Src/Rte.d ./Core/Src/Rte.o ./Core/Src/Rte.su ./Core/Src/Servo.cyclo ./Core/Src/Servo.d ./Core/Src/Servo.o ./Core/Src/Servo.su ./Core/Src/Task_CyclicEvent.cyclo ./Core/Src/Task_CyclicEvent.d ./Core/Src/Task_CyclicEvent.o ./Core/Src/Task_CyclicEvent.su ./Core/Src/Transmit_Arduino.cyclo ./Core/Src/Transmit_Arduino.d ./Core/Src/Transmit_Arduino.o ./Core/Src/Transmit_Arduino.su ./Core/Src/freertos.cyclo ./Core/Src/freertos.d ./Core/Src/freertos.o ./Core/Src/freertos.su ./Core/Src/main.cyclo ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/main.su ./Core/Src/stm32h7xx_hal_msp.cyclo ./Core/Src/stm32h7xx_hal_msp.d ./Core/Src/stm32h7xx_hal_msp.o ./Core/Src/stm32h7xx_hal_msp.su ./Core/Src/stm32h7xx_it.cyclo ./Core/Src/stm32h7xx_it.d ./Core/Src/stm32h7xx_it.o ./Core/Src/stm32h7xx_it.su ./Core/Src/syscalls.cyclo ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/syscalls.su ./Core/Src/sysmem.cyclo ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/sysmem.su ./Core/Src/system_stm32h7xx.cyclo ./Core/Src/system_stm32h7xx.d ./Core/Src/system_stm32h7xx.o ./Core/Src/system_stm32h7xx.su

.PHONY: clean-Core-2f-Src

