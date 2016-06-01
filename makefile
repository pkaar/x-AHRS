################################################################################
# makefile
#
# Description:
#  Makefile to compile and link Kinetis MKL25Z128 projects.
#
# Notes:
#  This makefile is usable for Linux environment only. Do not open this file
#  with an editor which replaces tabs by spaces.
#
# History:
#  pka, 31/AUG/2014, initial code
################################################################################

# Set ARM Ltd GCC toolchain path
TCPATH  =
CC      = $(TCPATH)arm-none-eabi-gcc
LD      = $(TCPATH)arm-none-eabi-gcc

# Get project name and use it as target
TARGET := $(notdir $(shell pwd))

# Set linker script
LDSCRIPT := mkl25z128.ld

# Set compiler flags
CFLAGS  = -O0 -g -ggdb -Wall -fmessage-length=0 -mthumb -mcpu=cortex-m0 -lm

# Set dependencies flags
DPFLAGS = -MD -MP -MF $(basename $<).d

# Set assembler flags
ASFLAGS =

# Set linker flags and specify linker script
LDFLAGS = -Wl,-Map=$(TARGET).map -L. -T $(LDSCRIPT) -nostartfiles

# Create lists of all existing source, header and assembler files using relative
# paths only
CSRC := $(shell find -name '*.c')       # list of all .c files
HSRC := $(shell find -name '*.h')       # list of all .h files

# Create lists of all object, dependencies and assembler files to be generated
OBJS := $(CSRC:.c=.o)                   # list of all .o files
DEPS := $(CSRC:.c=.d)                   # list of all .d files
ASMS := $(CSRC:.c=.asm)                 # list of all .asm files

# Create lists of all directories containing source, header and assembler files
CDIR := $(sort $(dir $(CSRC)))          # directories of all .c files
HDIR := $(sort $(dir $(HSRC)))          # directories of all .h files

IDIR := $(sort $(CDIR) $(HDIR))         # include directories

# Attach -I at the beginning of all include directories
INCS    = $(foreach dir, $(IDIR),-I $(dir))
LIBS    =

VPATH   = $(IDIR)

# Set only the needed suffixes
.SUFFIXES:
.SUFFIXES: .c .s .d .o .elf

# Implicite rule for creating object, dependencies and assembler file from
# source file
%.o: %.c
	@echo ' '
	@echo 'Building file: $(@D)$(@F)'
	@echo 'Invoking: ARM Ltd GCC C Compiler'
	$(CC) -c $(CFLAGS) $(DPFLAGS) $(INCS) $< -o $@
	$(CC) -S $(CFLAGS) $(ASFLAGS) $(INCS) $< -o $(basename $<).asm
	@echo 'Finished building: $(@D)/$(@F)'

# Implicite rule for creating executable from object files
%.elf: $(OBJS)
	@echo ' '
	@echo 'Building target: $(TARGET).elf'
	@echo 'Invoking: ARM Ltd GCC C Linker'
	$(LD) $(OBJS) $(CFLAGS) $(LDFLAGS) $(LIBS) -o $(TARGET).elf
	@echo 'Finished building target: $(TARGET).elf'
	@echo ' '

all: $(TARGET).elf

clean:
	@echo ' '
	@echo 'Cleaning project: $(TARGET)'
	@rm -f $(OBJS)                  # remove .o files
	@rm -f $(ASMS)                  # remove .asm files
	@rm -f $(DEPS)                  # remove .d files
	@rm -f $(TARGET).elf            # remove .elf file
	@rm -f $(TARGET).map            # remove .map file
	@echo 'Finished cleaning project: $(TARGET)'
	@echo ' '

.PHONY: all clean

.SECONDARY: $(OBJS)