################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../BacklogManager/backlog.c 

OBJS += \
./BacklogManager/backlog.o 

C_DEPS += \
./BacklogManager/backlog.d 


# Each subdirectory must supply rules for building sources it contributes
BacklogManager/%.o: ../BacklogManager/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo %cd%
	arm-none-eabi-gcc -mthumb -mfloat-abi=soft -D__weak=__attribute__((weak)) -D__packed=__attribute__((__packed__)) -DUSE_HAL_DRIVER -DSTM32L476xx -I"D:/BackUp/KVTS/MemoryManagementVTS/MemManage/Inc" -I"D:/BackUp/KVTS/MemoryManagementVTS/MemManage/BacklogManager" -I"D:/BackUp/KVTS/MemoryManagementVTS/MemManage/Drivers/SPIFlashDriver" -I"D:/BackUp/KVTS/MemoryManagementVTS/MemManage/Drivers/STM32L4xx_HAL_Driver/Inc" -I"D:/BackUp/KVTS/MemoryManagementVTS/MemManage/Drivers/STM32L4xx_HAL_Driver/Inc/Legacy" -I"D:/BackUp/KVTS/MemoryManagementVTS/MemManage/Drivers/CMSIS/Device/ST/STM32L4xx/Include" -I"D:/BackUp/KVTS/MemoryManagementVTS/MemManage/Drivers/CMSIS/Include"  -Og -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


