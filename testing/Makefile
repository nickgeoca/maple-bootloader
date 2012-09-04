#////////////////////////////////////////////////////////
################################################################################
# Manually generated file. Do not edit?
################################################################################

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include subdir.mk
-include src/subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif


SOURCES = main.c

TARGET = main
OBJS = $(TARGET).o
LINKSCRIPT = user-RAM.ld
# maplish libs would go here perhaps?
#LIBS = 

CC = arm-none-eabi-gcc

.PHONY: all
all: $(TARGET).axf

$(TARGET).o: $(SOURCES) Makefile
	$(CC) -mthumb -S $< 
	$(CC) -mthumb -c $< -o $@
	
# Add inputs and outputs from these tool invocations to the build variables 

# All Target
all: post-build

# Tool invocations
$(TARGET).axf: $(OBJS) $(LINKSCRIPT) Makefile
	@echo 'Building target: $@'
	@echo 'Invoking: MCU Linker'
	arm-none-eabi-gcc -nostdlib -Xlinker -Map="$(TARGET).map" -Xlinker --gc-sections -mcpu=cortex-m3 -mthumb -T $(LINKSCRIPT) -o $@ $(OBJS) $(LIBS)
	@echo 'Finished building target: $@'
#	@echo ' '
#	$(MAKE) --no-print-directory post-build

# Other Targets
clean:
	-$(RM) $(OBJS) $(C_DEPS) $(EXECUTABLES) $(TARGET).axf
	-@echo ' '

post-build: $(TARGET).axf
	-@echo 'Performing post-build steps'
	-arm-none-eabi-size "$(TARGET).axf"; # arm-none-eabi-objcopy -O binary "si32-bootloader.axf" "si32-bootloader.bin" ; checksum -p SiM3U167 -d "si32-bootloader.bin";
	-@echo ' '
	arm-none-eabi-objdump -D $(TARGET).axf > $(TARGET)-dump

.PHONY: all clean dependents
.SECONDARY: post-build

-include ../makefile.targets
