################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
S_UPPER_SRCS += \
../src/arithmetic_logical_examples.S \
../src/branch_and_control_examples.S \
../src/main.S \
../src/memory_access_examples.S \
../src/move_examples.S \
../src/multiply_saturate_examples.S 

OBJS += \
./src/arithmetic_logical_examples.o \
./src/branch_and_control_examples.o \
./src/main.o \
./src/memory_access_examples.o \
./src/move_examples.o \
./src/multiply_saturate_examples.o 

S_UPPER_DEPS += \
./src/arithmetic_logical_examples.d \
./src/branch_and_control_examples.d \
./src/main.d \
./src/memory_access_examples.d \
./src/move_examples.d \
./src/multiply_saturate_examples.d 


# Each subdirectory must supply rules for building sources it contributes
src/%.o: ../src/%.S
	@echo 'Building file: $<'
	@echo 'Invoking: GNU ARM Cross Assembler'
	arm-none-eabi-gcc -mcpu=cortex-m4 -march=armv7e-m -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -O0 -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections  -g3 -x assembler-with-cpp -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


