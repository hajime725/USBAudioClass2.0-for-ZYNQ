################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
LD_SRCS += \
../src/lscript.ld 

C_SRCS += \
../src/xusbps_audio_example.c \
../src/xusbps_ch9.c \
../src/xusbps_ch9_audio.c \
../src/xusbps_class_audio.c 

OBJS += \
./src/xusbps_audio_example.o \
./src/xusbps_ch9.o \
./src/xusbps_ch9_audio.o \
./src/xusbps_class_audio.o 

C_DEPS += \
./src/xusbps_audio_example.d \
./src/xusbps_ch9.d \
./src/xusbps_ch9_audio.d \
./src/xusbps_class_audio.d 


# Each subdirectory must supply rules for building sources it contributes
src/%.o: ../src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: ARM v7 gcc compiler'
	arm-none-eabi-gcc -Wall -O0 -g3 -c -fmessage-length=0 -MT"$@" -mcpu=cortex-a9 -mfpu=vfpv3 -mfloat-abi=hard -IC:/xilinx/cora/linuxbase/XADC/vitis_proj/test/export/test/sw/test/standalone_domain/bspinclude/include -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


