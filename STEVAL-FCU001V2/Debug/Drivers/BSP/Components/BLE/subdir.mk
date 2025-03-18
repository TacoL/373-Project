################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/BSP/Components/BLE/SPBTLE_RF.c 

OBJS += \
./Drivers/BSP/Components/BLE/SPBTLE_RF.o 

C_DEPS += \
./Drivers/BSP/Components/BLE/SPBTLE_RF.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/BSP/Components/BLE/%.o Drivers/BSP/Components/BLE/%.su Drivers/BSP/Components/BLE/%.cyclo: ../Drivers/BSP/Components/BLE/%.c Drivers/BSP/Components/BLE/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32F401xC -DDEBUG -c -I../Drivers/CMSIS/Include -I../Drivers/BSP/Components/lsm6dsr -I../Drivers/BSP/Components/lps22hh -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Core/Inc -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../X-CUBE-MEMS1/Target -I"/Users/sagehere/Documents/373Project/STEVAL-FCU001V2/Middlewares/Include" -I"/Users/sagehere/Documents/373Project/STEVAL-FCU001V2/Drivers/BSP/Components/BLE" -I"/Users/sagehere/Documents/373Project/STEVAL-FCU001V2/Middlewares/HCI" -I"/Users/sagehere/Documents/373Project/STEVAL-FCU001V2/Middlewares/HCI/Controller" -I"/Users/sagehere/Documents/373Project/STEVAL-FCU001V2/Middlewares/Interface" -I"/Users/sagehere/Documents/373Project/STEVAL-FCU001V2/Middlewares/Utils" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-BSP-2f-Components-2f-BLE

clean-Drivers-2f-BSP-2f-Components-2f-BLE:
	-$(RM) ./Drivers/BSP/Components/BLE/SPBTLE_RF.cyclo ./Drivers/BSP/Components/BLE/SPBTLE_RF.d ./Drivers/BSP/Components/BLE/SPBTLE_RF.o ./Drivers/BSP/Components/BLE/SPBTLE_RF.su

.PHONY: clean-Drivers-2f-BSP-2f-Components-2f-BLE

