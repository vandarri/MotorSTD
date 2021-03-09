################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../User/src/system.c \
../User/src/systimer.c \
../User/src/systimer_conf.c \
../User/src/uart.c \
../User/src/uart_conf.c 

OBJS += \
./User/src/system.o \
./User/src/systimer.o \
./User/src/systimer_conf.o \
./User/src/uart.o \
./User/src/uart_conf.o 

C_DEPS += \
./User/src/system.d \
./User/src/systimer.d \
./User/src/systimer_conf.d \
./User/src/uart.d \
./User/src/uart_conf.d 


# Each subdirectory must supply rules for building sources it contributes
User/src/%.o: ../User/src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: ARM-GCC C Compiler'
	"$(TOOLCHAIN_ROOT)/bin/arm-none-eabi-gcc" -MMD -MT "$@" -DXMC1404_F064x0200 -I"$(PROJECT_LOC)/User/inc" -I"$(PROJECT_LOC)/Libraries/XMCLib/inc" -I"$(PROJECT_LOC)/Libraries/CMSIS/Include" -I"$(PROJECT_LOC)/Libraries/CMSIS/Infineon/XMC1400_series/Include" -I"$(PROJECT_LOC)" -I"$(PROJECT_LOC)/Libraries" -Os -ffunction-sections -fdata-sections -Wall -std=gnu99 -Wa,-adhlns="$@.lst" -pipe -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d) $@" -mcpu=cortex-m0 -mthumb -o "$@" "$<" 
	@echo 'Finished building: $<'
	@echo.

