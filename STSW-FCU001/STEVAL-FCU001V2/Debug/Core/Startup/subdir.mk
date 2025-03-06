################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
S_SRCS += \
../Core/Startup/startup_stm32f401ccux.s 

OBJS += \
./Core/Startup/startup_stm32f401ccux.o 

S_DEPS += \
./Core/Startup/startup_stm32f401ccux.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Startup/startup_stm32f401ccux.o: ../Core/Startup/startup_stm32f401ccux.s Core/Startup/subdir.mk
	arm-none-eabi-gcc -mcpu=cortex-m4 -g3 -c -I"C:/Users/gmessina/OneDrive - STMicroelectronics/Drones/Development/Source_Code/STEVALFCU01V2/Middlewares/Include" -I"C:/Users/gmessina/OneDrive - STMicroelectronics/Drones/Development/Source_Code/STEVALFCU01V2/Drivers/BSP/Components/BLE" -I"C:/Users/gmessina/OneDrive - STMicroelectronics/Drones/Development/Source_Code/STEVALFCU01V2/Middlewares/HCI" -I"C:/Users/gmessina/OneDrive - STMicroelectronics/Drones/Development/Source_Code/STEVALFCU01V2/Middlewares/HCI/Controller" -I"C:/Users/gmessina/OneDrive - STMicroelectronics/Drones/Development/Source_Code/STEVALFCU01V2/Middlewares/Interface" -I"C:/Users/gmessina/OneDrive - STMicroelectronics/Drones/Development/Source_Code/STEVALFCU01V2/Middlewares/Utils" -x assembler-with-cpp -MMD -MP -MF"Core/Startup/startup_stm32f401ccux.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@" "$<"

