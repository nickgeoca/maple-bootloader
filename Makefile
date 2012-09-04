# Makefile skeleton adapted from Peter Harrison's - www.micromouse.com
#############################

BUILDDIR = build
CC = arm-none-eabi-gcc
OBJ_DUMP = arm-none-eabi-objdump
TARGET = $(BUILDDIR)/si_boot
SOURCES = main.c hardware.c
_OBJ =  $(SOURCES:.c=.o)
OBJS = $(patsubst %, $(BUILDDIR)/%,$(_OBJ))

LINKSCRIPT = arch/arm_m3/sim3u167.ld
# Compiler Flags
CFLAGS = -nostdlib
CFLAGS += -Xlinker -Map="$(TARGET).map"
CFLAGS += -Xlinker --gc-sections
CFLAGS += -mcpu=cortex-m3 -mthumb
# Linker Flags
LDFLAGS = -o $@ -T $(LINKSCRIPT)
###############################

.PHONY: all
all: begin build post-build

build: $(TARGET).elf

begin:
	@echo 'Remove clean from build function.'
	-rm -r $(BUILDDIR)
	mkdir -p build

# Other Targets
clean:
	-rm -r $(BUILDDIR)
	-@echo ' '
	
gccversion:
	@$(CC) --version
	
$(TARGET).elf: $(OBJS)
	@echo 'Building target: $@'
	@echo 'Invoking linker'
	$(CC) $(CFLAGS) $(OBJS) $(LDFLAGS)
	@echo 'Finished building target: $@'

# Compile: create object files from C source files. ARM/Thumb
$(OBJS) : $(BUILDDIR)/%.o : %.c
	@echo $(MSG_COMPILING) $<
	$(CC) -c $(CFLAGS) $< -o $@	
	
post-build: $(TARGET).elf
	-@echo 'Performing post-build steps'
	-arm-none-eabi-size "$(TARGET).elf";
	-@echo ' '
	$(OBJ_DUMP) -D $(TARGET).elf > $(TARGET)-dump
	-@echo 'Object dump located in build directory'
	

.PHONY: all clean dependents
.SECONDARY: post-build
