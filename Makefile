# stm32f103ve
CHIP     = STM32F401VC_FLASH
F_XTAL   = 8000000
SYSCLOCK_CL = 84000000
CHIP_LINE = stm32f4xx

CDEFS = -DSTM32F401xx
#CDEFS += -DSTM32F401xx
#CDEFS += -DHSE_VALUE=$(F_XTAL)UL
#CDEFS += -D$(SYSCLOCK_CL)
CDEFS += -DUSE_STDPERIPH_DRIVER
#CDEFS += -DSTM32_SD_USE_DMA
CDEFS += -DUSE_USB_OTG_FS
# enable modifications in STM's libraries
#CDEFS += -DMOD_MTHOMAS_STMLIB
# enable parameter-checking in STM's library
#CDEFS += -DUSE_FULL_ASSERT

RUN_MODE=FLASH_RUN
#RUN_MODE=RAM_RUN
VECTOR_TABLE_LOCATION=VECT_TAB_FLASH
#VECTOR_TABLE_LOCATION=VECT_TAB_RAM
OUTDIR = out
TARGET = project
TCHAIN_PREFIX = arm-none-eabi-
MCU = cortex-m3
THUMB = -mthumb

# Pathes to libraries
APPLIBDIR = libraries
STMSPDDIR = $(APPLIBDIR)/STM32F4xx_StdPeriph_Driver
STMSPDSRCDIR = $(STMSPDDIR)/src
STMSPDINCDIR = $(STMSPDDIR)/inc
#COREDIR = $(APPLIBDIR)/CMSIS/Include
#DEVICEDIR = $(APPLIBDIR)/CMSIS/CM3/DeviceSupport/ST/STM32F10x
#STMSTARTUP = $(DEVICEDIR)/gcc_ride7
CMSIS = $(APPLIBDIR)/CMSIS
USBSOURCES = $(APPLIBDIR)/usb/src

# List C source files here. (C dependencies are automatically generated.)
#SRC = $(USBSOURCES)/*.c
# 
## CMSIS for STM32
#SRC += $(COREDIR)/core_cm3.c
SRC += $(CMSIS)/system_$(CHIP_LINE).c
## used parts of the STM-Library
SRC += $(STMSPDSRCDIR)/misc.c
SRC += $(STMSPDSRCDIR)/$(CHIP_LINE)_spi.c
SRC += $(STMSPDSRCDIR)/$(CHIP_LINE)_gpio.c
SRC += $(STMSPDSRCDIR)/$(CHIP_LINE)_rcc.c
SRC += $(STMSPDSRCDIR)/$(CHIP_LINE)_exti.c
SRC += $(STMSPDSRCDIR)/$(CHIP_LINE)_dma.c
#SRC += $(STMSPDSRCDIR)/$(CHIP_LINE)_sdio.c
#SRC += $(STMSPDSRCDIR)/$(CHIP_LINE)_usart.c
SRC += $(STMSPDSRCDIR)/$(CHIP_LINE)_syscfg.c
#SRC += $(STMSPDSRCDIR)/misc.c

# USB
SRC += $(USBSOURCES)/usb_core.c
SRC += $(USBSOURCES)/usbd_core.c
SRC += $(USBSOURCES)/usbd_ioreq.c
SRC += $(USBSOURCES)/usbd_req.c
SRC += $(USBSOURCES)/usb_dcd.c
SRC += $(USBSOURCES)/usb_dcd_int.c
SRC += $(USBSOURCES)/usbd_usr.c
SRC += $(USBSOURCES)/usb_bsp.c
SRC += src/usbd_desc.c
SRC += src/stm32f401_discovery.c
#SRC += src/stm32f4xx_it.c
SRC += src/usbd_hid_core.c
SRC += main.c


# List C++ source files here.
# use file-extension .cpp for C++-files (not .C)
CPPSRC =

# List Assembler source files here.
# Make them always end in a capital .S. Files ending in a lowercase .s
# will not be considered source files but generated files (assembler
# output from the compiler), and will be deleted upon "make clean"!
# Even though the DOS/Win* filesystem matches both .s and .S the same,
# it will preserve the spelling of the filenames, and gcc itself does
# care about how the name is spelled on its command-line.
ASRC = $(CMSIS)/startup_stm32f401xx.s

# List any extra directories to look for include files here.
#    Each directory must be seperated by a space.
EXTRAINCDIRS  = $(CMSIS)/Include ./inc $(STMSPDINCDIR) $(APPLIBDIR)/usb/inc
# Extra libraries
#    Each library-name must be seperated by a space.
#    i.e. to link with libxyz.a, libabc.a and libefsl.a: 
#    EXTRA_LIBS = xyz abc efsl
# for newlib-lpc (file: libnewlibc-lpc.a):
#    EXTRA_LIBS = newlib-lpc
EXTRA_LIBS =

# Path to Linker-Scripts
LINKERSCRIPTPATH = .
LINKERSCRIPTINC  = .

# List any extra directories to look for library files here.
# Also add directories where the linker should search for
# includes from linker-script to the list
#     Each directory must be seperated by a space.
EXTRA_LIBDIRS = $(LINKERSCRIPTINC)

# Optimization level, can be [0, 1, 2, 3, s]. 
# 0 = turn off optimization. s = optimize for size.
# (Note: 3 is not always the best optimization level. See avr-libc FAQ.)
OPT = 2

# Output format. (can be ihex or binary or both)
#  binary to create a load-image in raw-binary format i.e. for SAM-BA, 
#  ihex to create a load-image in Intel hex format i.e. for lpc21isp
#LOADFORMAT = ihex
#LOADFORMAT = binary
LOADFORMAT = both

# Debugging format.
#DEBUG = stabs
DEBUG = dwarf-2

# Place project-specific -D and/or -U options for 
# Assembler with preprocessor here.
#ADEFS = -DUSE_IRQ_ASM_WRAPPER
ADEFS = -D__ASSEMBLY__

# Compiler flag to set the C Standard level.
CSTANDARD = -std=gnu99

#-----

ifdef VECTOR_TABLE_LOCATION
CDEFS += -D$(VECTOR_TABLE_LOCATION)
ADEFS += -D$(VECTOR_TABLE_LOCATION)
endif
CDEFS += -D$(RUN_MODE) -D$(CHIP)
ADEFS += -D$(RUN_MODE) -D$(CHIP)

#THUMB    = --mthumb-interwork
#THUMB_IW = -mthumb-interwork

# Flags for C and C++ (arm-elf-gcc/arm-elf-g++)
CFLAGS =  -g$(DEBUG)
CFLAGS += -O$(OPT)
CFLAGS += -mcpu=$(MCU)
CFLAGS += $(CDEFS)
CFLAGS += $(patsubst %,-I%,$(EXTRAINCDIRS)) -I.
# when using ".ramfunc"s without longcall:
#CFLAGS += -mlong-calls
# -mapcs-frame is important if gcc's interrupt attributes are used
# (at least from my eabi tests), not needed if assembler-wrapper is used 
#CFLAGS += -mapcs-frame 
#CFLAGS += -fomit-frame-pointer
#CFLAGS += -ffunction-sections -fdata-sections
#CFLAGS += -fpromote-loop-indices
#CFLAGS += -Wall -Wextra
#CFLAGS += -Wcast-align -Wpointer-arith
#CFLAGS += -Wredundant-decls -Wshadow -Wcast-qual -Wcast-align
#CFLAGS += -fexec-charset=cp1251
#CFLAGS += -Wa,-adhlns=$(addprefix $(OUTDIR)/, $(notdir $(addsuffix .lst, $(basename $<))))
# Compiler flags to generate dependency files:
#CFLAGS += -MD -MP -MF $(OUTDIR)/dep/$(@F).d
CFLAGS += -mlittle-endian -mfloat-abi=soft

# flags only for C
CONLYFLAGS += -Wimplicit -Wnested-externs 
CONLYFLAGS += $(CSTANDARD)

# flags only for C++ (arm-*-g++)
CPPFLAGS = -fno-rtti -fno-exceptions
CPPFLAGS = 

# Assembler flags.
#  -Wa,...:    tell GCC to pass this to the assembler.
#  -ahlns:     create listing
#  -g$(DEBUG): have the assembler create line number information
ASFLAGS  = -mcpu=$(MCU) -I. -x assembler-with-cpp
ASFLAGS += $(ADEFS)
ASFLAGS += -Wa,-adhlns=$(addprefix $(OUTDIR)/, $(notdir $(addsuffix .lst, $(basename $<))))
ASFLAGS += -Wa,-g$(DEBUG)
ASFLAGS += $(patsubst %,-I%,$(EXTRAINCDIRS))

MATH_LIB = -lm

# Link with the GNU C++ stdlib.
CPLUSPLUS_LIB = -lstdc++
#CPLUSPLUS_LIB += -lsupc++

# Linker flags.
#  -Wl,...:     tell GCC to pass this to linker.
#    -Map:      create map file
#    --cref:    add cross reference to  map file
LDFLAGS = -Wl,-Map=$(OUTDIR)/$(TARGET).map,--cref,--gc-sections
#not in CPP
#LDFLAGS += -nostartfiles
LDFLAGS += -lc
LDFLAGS += $(MATH_LIB)
LDFLAGS += -lc -lgcc
#LDFLAGS += $(CPLUSPLUS_LIB)
LDFLAGS += $(patsubst %,-L%,$(EXTRA_LIBDIRS))
LDFLAGS += $(patsubst %,-l%,$(EXTRA_LIBS)) 

# Set linker-script name depending on selected run-mode and submodel name
ifeq ($(RUN_MODE),RAM_RUN)
##LDFLAGS +=-T$(LINKERSCRIPTPATH)/$(CHIP).ld
##LDFLAGS +=-T$(LINKERSCRIPTPATH)/sram.lds
else 
LDFLAGS +=-T$(LINKERSCRIPTPATH)/$(CHIP).ld
##LDFLAGS +=-T$(LINKERSCRIPTPATH)/flash.lds
endif

# Define programs and commands.
CC      = $(TCHAIN_PREFIX)gcc
CPP     = $(TCHAIN_PREFIX)g++
AR      = $(TCHAIN_PREFIX)ar
OBJCOPY = $(TCHAIN_PREFIX)objcopy
OBJDUMP = $(TCHAIN_PREFIX)objdump
SIZE    = $(TCHAIN_PREFIX)size
NM      = $(TCHAIN_PREFIX)nm
#REMOVE  = $(REMOVE_CMD) -f
SHELL   = sh
###COPY    = cp
ifneq ($(or $(COMSPEC), $(ComSpec)),)
$(info COMSPEC detected $(COMSPEC) $(ComSpec))
ifeq ($(findstring cygdrive,$(shell env)),)
SHELL:=$(or $(COMSPEC),$(ComSpec))
SHELL_IS_WIN32=1
else
$(info cygwin detected)
endif
endif
$(info SHELL is $(SHELL))

# Define Messages
# English
MSG_ERRORS_NONE = Errors: none
MSG_BEGIN = --------  begin, mode: $(RUN_MODE)  --------
MSG_END = --------  end  --------
MSG_SIZE_BEFORE = Size before: 
MSG_SIZE_AFTER = Size after build:
MSG_LOAD_FILE = Creating load file:
MSG_EXTENDED_LISTING = Creating Extended Listing/Disassembly:
MSG_SYMBOL_TABLE = Creating Symbol Table:
MSG_LINKING = ---- Linking :
MSG_COMPILING = ---- Compiling C :
MSG_COMPILINGCPP = ---- Compiling C++ :
MSG_ASSEMBLING = ---- Assembling:
MSG_CLEANING = Cleaning project:
MSG_FORMATERROR = Can not handle output-format
MSG_ASMFROMC = "Creating asm-File from C-Source:"

# List of all source files.
ALLSRC     = $(ASRCARM) $(ASRC) $(SRCARM) $(SRC) $(CPPSRCARM) $(CPPSRC)
# List of all source files without directory and file-extension.
ALLSRCBASE = $(notdir $(basename $(ALLSRC)))

# Define all object files.
ALLOBJ     = $(addprefix $(OUTDIR)/, $(addsuffix .o, $(ALLSRCBASE)))

# Define all listing files (used for make clean).
LSTFILES   = $(addprefix $(OUTDIR)/, $(addsuffix .lst, $(ALLSRCBASE)))
# Define all depedency-files (used for make clean).
DEPFILES   = $(addprefix $(OUTDIR)/dep/, $(addsuffix .o.d, $(ALLSRCBASE)))

# Default target.
#all: begin gccversion sizebefore build sizeafter finished end
all: begin createdirs gccversion build sizebefore sizeafter

elf: $(OUTDIR)/$(TARGET).elf
#lss: $(OUTDIR)/$(TARGET).lss 
sym: $(OUTDIR)/$(TARGET).sym
hex: $(OUTDIR)/$(TARGET).hex
bin: $(OUTDIR)/$(TARGET).bin

ifeq ($(LOADFORMAT),ihex)
build: elf hex sym
else 
ifeq ($(LOADFORMAT),binary)
build: elf bin sym
else 
ifeq ($(LOADFORMAT),both)
build: elf hex bin sym
else 
$(error "$(MSG_FORMATERROR) $(FORMAT)")
endif
endif
endif

# Create output directories.
ifdef SHELL_IS_WIN32
createdirs:
	-@md $(OUTDIR) >NUL 2>&1 || echo "" >NUL
	-@md $(OUTDIR)\dep >NUL 2>&1 || echo "" >NUL
else
createdirs:
	-@mkdir $(OUTDIR) 2>/dev/null || echo "" >/dev/null
	-@mkdir $(OUTDIR)/dep 2>/dev/null || echo "" >/dev/null
endif

ifdef SHELL_IS_WIN32
REMOVE = cs-rm -f
SLASH = \
COPY = copy /Y
PROG= "C:\Program Files (x86)\STMicroelectronics\STM32 ST-LINK Utility\ST-LINK Utility\ST-LINK_CLI.EXE"
PROG_WRITE =  -Halt -P "$(OUTDIR)/$(TARGET).hex" -V
# -ME стереть весь чип
PROG_START = -Run
PROG_DEV = -c SWD UR
PROG_COMMAND = $(PROG) $(PROG_DEV) $(PROG_WRITE) -Rst
RESET = $(PROG) $(PROG_DEV) -Rst
else
REMOVE = rm -f
SLASH = /
COPY = cp -f
PROG= openocd
PROG_CFG =  -f interface/stlink-v2.cfg -f target/stm32f2x_stlink.cfg -c
PROG_WRITE = program $(OUTDIR)/$(TARGET).hex  verify reset
PROG_COMMAND = $(PROG) $(PROG_CFG) "$(PROG_WRITE)"
RESET = $(PROG) $(PROG_CFG) "init" -c "reset" -c "exit"
endif

doxygen:
	"f:\doxygen\bin\doxygen" Doxyfile

program: all
	$(PROG_COMMAND)

reset:
	$(RESET)

# Eye candy.
begin:
	@echo $(MSG_BEGIN)

# Display sizes of sections.
sizebefore:
#	@echo $(MSG_SIZE_BEFORE) 
	-@$(SIZE) --format=Berkeley --common $(OUTDIR)/$(TARGET).elf.old

sizeafter:
	@$(COPY) $(OUTDIR)$(SLASH)$(TARGET).elf $(OUTDIR)$(SLASH)$(TARGET).elf.old
#	@echo $(MSG_SIZE_AFTER)
	-@$(SIZE) --format=Berkeley --common $(OUTDIR)/$(TARGET).elf
	
# Display compiler version information.
gccversion : 
	@$(CC) --version
#	@echo $(ALLOBJ)

# Create final output file (.hex) from ELF output file.
%.hex: %.elf
#	@echo $(MSG_LOAD_FILE) $@
	@$(OBJCOPY) -O ihex $< $@
	
# Create final output file (.bin) from ELF output file.
%.bin: %.elf
#	@echo $(MSG_LOAD_FILE) $@
	@$(OBJCOPY) -O binary $< $@

# Create extended listing file/disassambly from ELF output file.
# using objdump testing: option -C
%.lss: %.elf
#	@echo $(MSG_EXTENDED_LISTING) $@
	@$(OBJDUMP) -h -S -C -r $< > $@
#	$(OBJDUMP) -x -S $< > $@

# Create a symbol table from ELF output file.
%.sym: %.elf
#	@echo $(MSG_SYMBOL_TABLE) $@
	@$(NM) -n $< > $@

# Link: create ELF output file from object files.
.SECONDARY : $(TARGET).elf
.PRECIOUS : $(ALLOBJ)
%.elf:  $(ALLOBJ)
#	@echo $(MSG_LINKING) $@
# use $(CC) for C-only projects or $(CPP) for C++-projects:
ifeq "$(strip $(CPPSRC)$(CPPARM))" ""
	$(CC) $(THUMB) $(CFLAGS) $(ALLOBJ) --output $@ -nostartfiles $(LDFLAGS)
else
	$(CPP) $(THUMB) $(CFLAGS) $(ALLOBJ) --output $@ $(LDFLAGS)
endif

# Assemble: create object files from assembler source files.
define ASSEMBLE_TEMPLATE
$(OUTDIR)/$(notdir $(basename $(1))).o : $(1)
	@echo $(MSG_ASSEMBLING) $$< to $$@
	$(CC) -c $(THUMB) $$(ASFLAGS) $$< -o $$@ 
endef
$(foreach src, $(ASRC), $(eval $(call ASSEMBLE_TEMPLATE, $(src)))) 

# Assemble: create object files from assembler source files. ARM-only
define ASSEMBLE_ARM_TEMPLATE
$(OUTDIR)/$(notdir $(basename $(1))).o : $(1)
	@echo $(MSG_ASSEMBLING_ARM) $$< to $$@
	$(CC) -c $$(ASFLAGS) $$< -o $$@ 
endef
$(foreach src, $(ASRCARM), $(eval $(call ASSEMBLE_ARM_TEMPLATE, $(src)))) 


# Compile: create object files from C source files.
define COMPILE_C_TEMPLATE
$(OUTDIR)/$(notdir $(basename $(1))).o : $(1)
	@echo $(MSG_COMPILING) $$< to $$@
	$(CC) -c $(THUMB) $$(CFLAGS) $$(CONLYFLAGS) $$< -o $$@ 
endef
$(foreach src, $(SRC), $(eval $(call COMPILE_C_TEMPLATE, $(src)))) 

# Compile: create object files from C source files. ARM-only
define COMPILE_C_ARM_TEMPLATE
$(OUTDIR)/$(notdir $(basename $(1))).o : $(1)
	@echo $(MSG_COMPILING_ARM) $$< to $$@
	$(CC) -c $$(CFLAGS) $$(CONLYFLAGS) $$< -o $$@ 
endef
$(foreach src, $(SRCARM), $(eval $(call COMPILE_C_ARM_TEMPLATE, $(src)))) 

# Compile: create object files from C++ source files.
define COMPILE_CPP_TEMPLATE
$(OUTDIR)/$(notdir $(basename $(1))).o : $(1)
	@echo $(MSG_COMPILINGCPP) $$< to $$@
	@$(CC) -c $(THUMB) $$(CFLAGS) $$(CPPFLAGS) $$< -o $$@ 
endef
$(foreach src, $(CPPSRC), $(eval $(call COMPILE_CPP_TEMPLATE, $(src)))) 

# Compile: create object files from C++ source files. ARM-only
define COMPILE_CPP_ARM_TEMPLATE
$(OUTDIR)/$(notdir $(basename $(1))).o : $(1)
	@echo $(MSG_COMPILINGCPP_ARM) $$< to $$@
	@$(CC) -c $$(CFLAGS) $$(CPPFLAGS) $$< -o $$@ 
endef
$(foreach src, $(CPPSRCARM), $(eval $(call COMPILE_CPP_ARM_TEMPLATE, $(src)))) 

# Compile: create assembler files from C source files. ARM/Thumb
$(SRC:.c=.s) : %.s : %.c
	@echo $(MSG_ASMFROMC) $< to $@
	@$(CC) $(THUMB) -S $(CFLAGS) $(CONLYFLAGS) $< -o $@

# Compile: create assembler files from C source files. ARM only
$(SRCARM:.c=.s) : %.s : %.c
	@echo $(MSG_ASMFROMC_ARM) $< to $@
	@$(CC) -S $(CFLAGS) $(CONLYFLAGS) $< -o $@

# Target: clean project.
clean: begin clean_list

clean_list :
	@echo $(MSG_CLEANING)
	$(REMOVE) $(OUTDIR)/$(TARGET).map
	$(REMOVE) $(OUTDIR)/$(TARGET).elf
	$(REMOVE) $(OUTDIR)/$(TARGET).hex
	$(REMOVE) $(OUTDIR)/$(TARGET).bin
	$(REMOVE) $(OUTDIR)/$(TARGET).sym
	$(REMOVE) $(OUTDIR)/$(TARGET).lss
	$(REMOVE) $(ALLOBJ)
	$(REMOVE) $(LSTFILES)
	$(REMOVE) $(DEPFILES)
	$(REMOVE) $(SRC:.c=.s)
	$(REMOVE) $(SRCARM:.c=.s)
	$(REMOVE) $(CPPSRC:.cpp=.s)
	$(REMOVE) $(CPPSRCARM:.cpp=.s)

## Create object files directory - now done if special make target
##$(shell mkdir $(OBJDIR) 2>/dev/null)

# Include the dependency files.
##-include $(shell mkdir dep 2>/dev/null) $(wildcard dep/*)
-include $(wildcard dep/*)

# Listing of phony targets.
.PHONY : all begin finish end sizebefore sizeafter gccversion \
build elf hex bin lss sym clean clean_list program createdirs

