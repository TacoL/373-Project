################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
S_SRCS += \
../Core/Startup/startup_stm32f401ccux.s 

OBJS += \
./Core/Startup/startup_stm32f401ccux.o 

S_DEPS += \
./Core/Startup/startup_stm32f401ccux.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Startup/%.o: ../Core/Startup/%.s Core/Startup/subdir.mk
	arm-none-eabi-gcc -mcpu=cortex-m4 -g3 -c -I"C:/Users/cpui/Desktop/373-Project/STEVAL-FCU001V2/Middlewares/Include" -I"C:/Users/cpui/Desktop/373-Project/STEVAL-FCU001V2/Drivers/BSP/Components/BLE" -I"C:/Users/cpui/Desktop/373-Project/STEVAL-FCU001V2/Middlewares/HCI" -I"C:/Users/cpui/Desktop/373-Project/STEVAL-FCU001V2/Middlewares/HCI/Controller" -I"C:/Users/cpui/Desktop/373-Project/STEVAL-FCU001V2/Middlewares/Interface" -I"C:/Users/cpui/Desktop/373-Project/STEVAL-FCU001V2/Middlewares/Utils" -x assembler-with-cpp -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@" "$<"

clean: clean-Core-2f-Startup

clean-Core-2f-Startup:
	-$(RM) ./Core/Startup/startup_stm32f401ccux.d ./Core/Startup/startup_stm32f401ccux.o

.PHONY: clean-Core-2f-Startup

