# Hey Emacs, this is a -*- makefile -*-
#----------------------------------------------------------------------------
# WinAVR Makefile Template written by Eric B. Weddington, Jörg Wunsch, et al.
#
# Released to the Public Domain
#
# Additional material for this makefile was written by:
# Peter Fleury
# Tim Henigan
# Colin O'Flynn
# Reiner Patommel
# Markus Pfaff
# Sander Pool
# Frederik Rouleau
# Carlos Lamas
# Dario Carluccio
# Jiri Dobry
#
#----------------------------------------------------------------------------
# On command line:
#
# make all = Make software.
#
# make clean = Clean out built project files.
#
# make coff = Convert ELF to AVR COFF.
#
# make extcoff = Convert ELF to AVR Extended COFF.
#
# make program = Download the hex file to the device, using avrdude.
#                Please customize the avrdude settings below first!
#
# make debug = Start either simulavr or avarice as specified for debugging,
#              with avr-gdb or avr-insight as the front end for debugging.
#
# make filename.s = Just compile filename.c into the assembler code only.
#
# make filename.i = Create a preprocessed source file for use in submitting
#                   bug reports to the GCC project.
#
# To rebuild project do "make clean" then "make all".
#----------------------------------------------------------------------------
#
# Changes:
# 5.4.2008: Dario Carluccio
#           command line parameters for "avr-size" on Linux and Windows.

# include local configurations (debugger, programmer, ....)
-include make_conf.local

# include fallback / default configurations (debugger, programmer, ....)
include make_conf.default

# set up the TARGET and MCU based on HW
# TARGET = Target file name (without extension).
# MCU = MCU name
ifeq ($(HW),THERMOTRONIC)
	MCU = atmega169
	F_CPU = 1000000
	TARGET = thermotronic
else ifeq ($(HW),HONEYWELL)
	MCU = atmega169p
	F_CPU = 4000000
	TARGET = hr20
else ifeq ($(HW),HR25)
	MCU = atmega329pa
	F_CPU = 4000000
	TARGET = hr25
endif



# Output format. (can be srec, ihex, binary)
FORMAT = ihex


# Object files directory
#     To put object files in current directory, use a dot (.), do NOT make
#     this an empty or blank macro!
OBJDIR = obj


# List C source files here. (C dependencies are automatically generated.)
# order of this files have effect on code size (relaxing), therefore it divided to 2 parts
SRC =  \
    motor.c \
    controller.c \
    watch.c \
    com.c \
    modbus.c \
    main.c \
    keyboard.c \
    adc.c \
    menu.c \
    lcd.c

SRC_B_DIR=../common

SRC_B =  \
    rtc.c \
    cmac.c \
    eeprom.c \
    rfm.c \
    wireless.c \



# List Assembler source files here.
#     Make them always end in a capital .S.  Files ending in a lowercase .s
#     will not be considered source files but generated files (assembler
#     output from the compiler), and will be deleted upon "make clean"!
#     Even though the DOS/Win* filesystem matches both .s and .S the same,
#     it will preserve the spelling of the filenames, and gcc itself does
#     care about how the name is spelled on its command-line.
ASRC = \
    xtea-asm.S \
    avr_uart_i-asm.S \
    circularbytebuffer-asm.S \


# Optimization level, can be [0, 1, 2, 3, s].
#     0 = turn off optimization. s = optimize for size.
#     (Note: 3 is not always the best optimization level. See avr-libc FAQ.)
OPT = 1 -mcall-prologues


# Debugging format.
#     Native formats for AVR-GCC's -g are dwarf-2 [default] or stabs.
#     AVR Studio 4.10 requires dwarf-2.
#     AVR [Extended] COFF format requires stabs, plus an avr-objcopy run.
DEBUG = dwarf-2


# List any extra directories to look for include files here.
#     Each directory must be seperated by a space.
#     Use forward slashes for directory separators.
#     For a directory that has spaces, enclose it in quotes.
EXTRAINCDIRS = .. $(SRC_B_DIR)


# Compiler flag to set the C Standard level.
#     c89   = "ANSI" C
#     gnu89 = c89 plus GCC extensions
#     c99   = ISO C99 standard (not yet fully implemented)
#     gnu99 = c99 plus GCC extensions
CSTANDARD = -std=gnu99

# List of Makevariable names, which should be passed as defines to the compiler and assembler
define GLOBAL_DEFINE_VARS =
    RFM
    TEMP_COMPENSATE_OPTION
    HW_WINDOW_DETECTION
    MENU_SHOW_BATTERY
    MOTOR_COMPENSATE_BATTERY
    NO_AUTORETURN_FROM_ALT_MENUES
    CALIBRATION_RESETS_sumError
    REMOTE_SETTING_ONLY
    BLOCK_INTEGRATOR_AFTER_VALVE_CHANGE
    BOOST_CONTROLER_AFTER_CHANGE
    DISABLE_JTAG
endef

ifeq ($(RFM),1)
  define GLOBAL_DEFINE_VARS +=
    RFM_BAUD_RATE
    RFM_DEVICE_ADDRESS
    SECURITY_KEY_0
    SECURITY_KEY_1
    SECURITY_KEY_2
    SECURITY_KEY_3
    SECURITY_KEY_4
    SECURITY_KEY_5
    SECURITY_KEY_6
    SECURITY_KEY_7
    RFM_FREQ_MAIN
    RFM_FREQ_FINE
    RFM_TUNING
  endef
endif

ifeq ($(MODBUS),1)
  define GLOBAL_DEFINE_VARS +=
    MODBUS_BAUD_RATE
    MODBUS_DEVICE_ADDRESS
  endef
endif

C_DEFINE_VARS = $(GLOBAL_DEFINE_VARS)
A_DEFINE_VARS = $(GLOBAL_DEFINE_VARS)

# Place -D or -U options here for C sources
CDEFS = -DF_CPU=$(F_CPU)UL $(addprefix -D,$(foreach d,$(C_DEFINE_VARS),$(d)=$($(d))))

ifeq ($(RFM_WIRE),MARIOJTAG)
  CDEFS += -DRFM_WIRE_MARIOJTAG=1
endif
ifeq ($(RFM_WIRE),TK_INTERNAL)
  CDEFS += -DRFM_WIRE_TK_INTERNAL=1
endif
ifeq ($(RFM_WIRE),JD_INTERNAL)
  CDEFS += -DRFM_WIRE_JD_INTERNAL=1
endif


ifeq ($(HW),THERMOTRONIC)
  CDEFS += -DTHERMOTRONIC=1
endif
ifeq ($(HW),HR25)
  CDEFS += -DHR25=1
endif


# Place -D or -U options here for ASM sources
ADEFS = -DF_CPU=$(F_CPU) -DXTEA_ENC $(addprefix -D,$(foreach d,$(A_DEFINE_VARS),$(d)=$($(d))))
# We need only encryption
# ADEFS+=-DXTEA_DEC

# This function converts an mcu-name (as used by gcc) to a part-name as expected
# by avrdude
MCU_TO_PART= \
  $(patsubst m169p,m169, \
  $(patsubst atmega%,m%, \
  $(patsubst atiny%,t%,  \
  $1 )))


# Default options values, no manual flag overidden because of incompatibilites
# Incompatibilities handled below with $(error ...)
# Define RFM to 0 to disable support for the RFM Radio Moodule in the Code
ifeq ($(HW),THERMOTRONIC)
  RFM?=0
else
  RFM?=1
endif

# Check options and issue an error if consistent (rather than silently forcing compatible options)
ifeq ($(RFM),1)
 ifeq ($(HW),THERMOTRONIC)
  $(error ERROR: THERMOTRONIC with RFM12 not implemented.)
 endif
 ifneq ($(RFM_WIRE),JD_INTERNAL)
  ifeq ($(RFM_WIRE),TK_INTERNAL)
   ifneq ($(HW),HR25)
    $(error ERROR: RFM_WIRE_TK_INTERNAL applicable for HR25 hardware only.)
   endif
  else
   ifeq ($(RFM_WIRE),MARIOJTAG)
    ifeq ($(HW_WINDOW_DETECTION),1)
     $(error ERROR: HW_WINDOW_DETECTION incompatible with MARIOJTAG wiring.)
    endif
   else
    $(error ERROR: Unknown wiring option.)
   endif
  endif
  ifeq ($(DISABLE_JTAG),0)
   $(error ERROR: JTAG must be disabled to use MARIOJTAG or TK_INTERNAL wiring.)
  endif
 endif
else
 ifneq ($(RFM_WIRE),)
  $(error ERROR: Enable RFM to use RFM_WIRE_... options.)
 endif
 ifeq ($(REMOTE_SETTING_ONLY),1)
  $(error ERROR: Enable RFM to use REMOTE_SETTING_ONLY.)
 endif
endif


#---------------- Compiler Options C ----------------
#  -g*:          generate debugging information
#  -O*:          optimization level
#  -f...:        tuning, see GCC manual and avr-libc documentation
#  -Wall...:     warning level
#  -Wa,...:      tell GCC to pass this to the assembler.
#    -adhlns...: create assembler listing
CFLAGS = -g$(DEBUG)
CFLAGS += $(FLAGS)
CFLAGS += -O$(OPT)
CFLAGS += $(CDEFS)
CFLAGS += $(REV)
CFLAGS += -funsigned-char
CFLAGS += -funsigned-bitfields
CFLAGS += -fpack-struct
CFLAGS += -fshort-enums
CFLAGS += -Wall
CFLAGS += -Wstrict-prototypes
#CFLAGS += -mshort-calls
#CFLAGS += -fno-unit-at-a-time
#CFLAGS += -Wundef
#CFLAGS += -Wunreachable-code
#CFLAGS += -Wsign-compare
CFLAGS += -Wa,-adhlns=$(@:%.o=%.lst)
CFLAGS += $(patsubst %,-I%,$(EXTRAINCDIRS))
CFLAGS += $(CSTANDARD)
#optimizations flags
# tip: see here http://www.tty1.net/blog/2008-04-29-avr-gcc-optimisations_en.html
CFLAGS += -ffreestanding
CFLAGS += -Wl,--relax
#CFLAGS += --param inline-call-cost=2
#CFLAGS += -ffunction-sections
CFLAGS += -fdata-sections
CFLAGS += -fno-toplevel-reorder

#---------------- Assembler Options ----------------
#  -Wa,...:   tell GCC to pass this to the assembler.
#  -adhlns:   create listing
#  -gstabs:   have the assembler create line number information; note that
#             for use in COFF files, additional information about filenames
#             and function names needs to be present in the assembler source
#             files -- see avr-libc docs [FIXME: not yet described there]
#  -listing-cont-lines: Sets the maximum number of continuation lines of hex
#       dump that will be displayed for a given single line of source input.
ASFLAGS = $(ADEFS) -g -Wa,-adhlns=$(@:%.o=%.lst),-gstabs,--listing-cont-lines=100


#---------------- Library Options ----------------
# Minimalistic printf version
PRINTF_LIB_MIN = -Wl,-u,vfprintf -lprintf_min

# Floating point printf version (requires MATH_LIB = -lm below)
PRINTF_LIB_FLOAT = -Wl,-u,vfprintf -lprintf_flt

# If this is left blank, then it will use the Standard printf version.
PRINTF_LIB =
#PRINTF_LIB = $(PRINTF_LIB_MIN)
#PRINTF_LIB = $(PRINTF_LIB_FLOAT)


# Minimalistic scanf version
SCANF_LIB_MIN = -Wl,-u,vfscanf -lscanf_min

# Floating point + %[ scanf version (requires MATH_LIB = -lm below)
SCANF_LIB_FLOAT = -Wl,-u,vfscanf -lscanf_flt

# If this is left blank, then it will use the Standard scanf version.
SCANF_LIB =
#SCANF_LIB = $(SCANF_LIB_MIN)
#SCANF_LIB = $(SCANF_LIB_FLOAT)


MATH_LIB = -lm


# List any extra directories to look for libraries here.
#     Each directory must be seperated by a space.
#     Use forward slashes for directory separators.
#     For a directory that has spaces, enclose it in quotes.
EXTRALIBDIRS =



#---------------- External Memory Options ----------------

# 64 KB of external RAM, starting after internal RAM (ATmega128!),
# used for variables (.data/.bss) and heap (malloc()).
#EXTMEMOPTS = -Wl,-Tdata=0x801100,--defsym=__heap_end=0x80ffff

# 64 KB of external RAM, starting after internal RAM (ATmega128!),
# only used for heap (malloc()).
#EXTMEMOPTS = -Wl,--section-start,.data=0x801100,--defsym=__heap_end=0x80ffff

EXTMEMOPTS =



#---------------- Linker Options ----------------
#  -Wl,...:     tell GCC to pass this to linker.
#    -Map:      create map file
#    --cref:    add cross reference to  map file
LDFLAGS = -Wl,-Map=$(TARGET).map,--cref
LDFLAGS += $(EXTMEMOPTS)
LDFLAGS += $(patsubst %,-L%,$(EXTRALIBDIRS))
LDFLAGS += $(PRINTF_LIB) $(SCANF_LIB) $(MATH_LIB)
#LDFLAGS += -T linker_script.x
#LDFLAGS += --relax
#LDFLAGS += --combine -fwhole-program



#============================================================================


# Define programs and commands.
SHELL = sh
CC = avr-gcc
OBJCOPY = avr-objcopy
OBJDUMP = avr-objdump
SIZE = avr-size
AR = avr-ar rcs
NM = avr-nm
AVRDUDE = avrdude
REMOVE = rm -f
REMOVEDIR = rm -rf
COPY = cp
WINSHELL = cmd


# Define Messages
# English
MSG_ERRORS_NONE = Errors: none
MSG_BEGIN = -------- begin --------
MSG_END = --------  end  --------
MSG_SIZE_BEFORE = Size before:
MSG_SIZE_AFTER = Size after:
MSG_COFF = Converting to AVR COFF:
MSG_EXTENDED_COFF = Converting to AVR Extended COFF:
MSG_FLASH = Creating load file for Flash:
MSG_EEPROM = Creating load file for EEPROM:
MSG_EXTENDED_LISTING = Creating Extended Listing:
MSG_SYMBOL_TABLE = Creating Symbol Table:
MSG_LINKING = Linking:
MSG_COMPILING = Compiling C:
MSG_ASSEMBLING = Assembling:
MSG_CLEANING = Cleaning project:
MSG_CREATING_LIBRARY = Creating library:




# Define all object files.
OBJ = $(ASRC:%.S=$(OBJDIR)/%.o) $(SRC:%.c=$(OBJDIR)/%.o)  $(SRC_B:%.c=$(OBJDIR)/%.o)

# Define all listing files.
LST = $(SRC:%.c=$(OBJDIR)/%.lst) $(ASRC:%.S=$(OBJDIR)/%.lst) $(SRC_B:%.c=$(OBJDIR)/%.lst)


# Compiler flags to generate dependency files.
GENDEPFLAGS = -MMD -MP -MF .dep/$(@F).d


# Combine all necessary flags and optional flags.
# Add target processor to flags.
ALL_CFLAGS = -mmcu=$(MCU) -I. $(CFLAGS) $(GENDEPFLAGS)
ALL_ASFLAGS = -mmcu=$(MCU) -I. $(patsubst %,-I%,$(EXTRAINCDIRS)) -x assembler-with-cpp $(ASFLAGS)





# Default target.
all: begin gccversion sizebefore build sizeafter end

# Change the build target to build a HEX file or a library.
#build: elf hex eep bin lss sym info
build: elf hex eep bin lss info
#build: lib


elf: $(TARGET).elf
hex: $(TARGET).hex
eep: $(TARGET).eep
bin: $(TARGET).bin
lss: $(TARGET).lss
sym: $(TARGET).sym
LIBNAME=lib$(TARGET).a
lib: $(LIBNAME)
info: $(TARGET).txt



# Eye candy.
# AVR Studio 3.x does not check make's exit code but relies on
# the following magic strings to be generated by the compile job.
begin:
	@echo
	@echo $(MSG_BEGIN)

end:
	@echo $(MSG_END)
	@echo


# Display size of file.
HEXSIZE = $(SIZE) --target=$(FORMAT) $(TARGET).hex

ELFSIZE = $(SIZE) $(TARGET).elf


sizebefore:
	@if test -f $(TARGET).elf; then echo; echo $(MSG_SIZE_BEFORE); $(ELFSIZE); \
	2>/dev/null; echo; fi

sizeafter:
	@if test -f $(TARGET).elf; then echo; echo $(MSG_SIZE_AFTER); $(ELFSIZE); \
	2>/dev/null; echo; fi



# Display compiler version information.
gccversion :
	@$(CC) --version



# Program the device.
program: $(TARGET).hex $(TARGET).eep
	$(AVRDUDE) $(AVRDUDE_FLAGS) $(AVRDUDE_WRITE_FLASH) $(AVRDUDE_WRITE_EEPROM)


# Generate avr-gdb config/init file which does the following:
#     define the reset signal, load the target file, connect to target, and set
#     a breakpoint at main().
gdb-config:
	@$(REMOVE) $(GDBINIT_FILE)
	@echo define reset >> $(GDBINIT_FILE)
	@echo SIGNAL SIGHUP >> $(GDBINIT_FILE)
	@echo end >> $(GDBINIT_FILE)
	@echo file $(TARGET).elf >> $(GDBINIT_FILE)
	@echo target remote $(DEBUG_HOST):$(DEBUG_PORT)  >> $(GDBINIT_FILE)
ifeq ($(DEBUG_BACKEND),simulavr)
	@echo load  >> $(GDBINIT_FILE)
endif
	@echo break main >> $(GDBINIT_FILE)

debug: gdb-config $(TARGET).elf
ifeq ($(DEBUG_BACKEND), avarice)
	@echo Starting AVaRICE - Press enter when "waiting to connect" message displays.
	@avarice $(JTAG_DEV) $(DEBUG_HOST):$(DEBUG_PORT)
	

else
	@$(WINSHELL) /c start simulavr --gdbserver --device $(MCU) --clock-freq \
	$(DEBUG_MFREQ) --port $(DEBUG_PORT)
endif
	@$(WINSHELL) /c start avr-$(DEBUG_UI) --command=$(GDBINIT_FILE)




# Convert ELF to COFF for use in debugging / simulating in AVR Studio or VMLAB.
COFFCONVERT = $(OBJCOPY) --debugging
COFFCONVERT += --change-section-address .data-0x800000
COFFCONVERT += --change-section-address .bss-0x800000
COFFCONVERT += --change-section-address .noinit-0x800000
COFFCONVERT += --change-section-address .eeprom-0x810000



coff: $(TARGET).elf
	@echo
	@echo $(MSG_COFF) $(TARGET).cof
	$(COFFCONVERT) -O coff-avr $< $(TARGET).cof


extcoff: $(TARGET).elf
	@echo
	@echo $(MSG_EXTENDED_COFF) $(TARGET).cof
	$(COFFCONVERT) -O coff-ext-avr $< $(TARGET).cof

#create info file
%.txt: %.elf
	$(REMOVE) $@
	@echo "Configuration" >> $@
	@echo "Hardware type $(HW)" >> $@
	@echo "RFM=$(RFM)" >> $@
	@echo "RFM_DEVICE_ADDRESS=$(RFM_DEVICE_ADDRESS)" >> $@
	@echo "SECURITY_KEY=$(SECURITY_KEY_0) $(SECURITY_KEY_1) $(SECURITY_KEY_2) $(SECURITY_KEY_3) $(SECURITY_KEY_4) $(SECURITY_KEY_5) $(SECURITY_KEY_6) $(SECURITY_KEY_7)" >> $@
	@echo "RFM_FREQ_MAIN=$(RFM_FREQ_MAIN)" >> $@
	@echo "RFM_FREQ_FINE=$(RFM_FREQ_FINE)" >> $@
	@echo "RFM_TUNING=$(RFM_TUNING)" >> $@
	@echo "TEMP_COMPENSATE_OPTION=$(TEMP_COMPENSATE_OPTION)" >> $@
	@echo "REMOTE_SETTING_ONLY=$(REMOTE_SETTING_ONLY)" >> $@
	@echo "HW_WINDOW_DETECTION=$(HW_WINDOW_DETECTION)" >> $@
	@echo "MENU_SHOW_BATTERY=$(MENU_SHOW_BATTERY)" >> $@
	@echo "MOTOR_COMPENSATE_BATTERY=$(MOTOR_COMPENSATE_BATTERY)" >> $@
	@echo "NO_AUTORETURN_FROM_ALT_MENUES=$(NO_AUTORETURN_FROM_ALT_MENUES)" >> $@
	@echo "CALIBRATION_RESETS_sumError=$(CALIBRATION_RESETS_sumError)" >> $@
	@echo "BLOCK_INTEGRATOR_AFTER_VALVE_CHANGE=$(BLOCK_INTEGRATOR_AFTER_VALVE_CHANGE)" >> $@
	@echo "BOOST_CONTROLER_AFTER_CHANGE=$(BOOST_CONTROLER_AFTER_CHANGE)" >> $@
	@echo "RFM_WIRE=$(RFM_WIRE)" >> $@
	@echo "DISABLE_JTAG=$(DISABLE_JTAG)" >> $@
	@echo "==================================" >> $@
	@echo >> $@
	$(ELFSIZE) >> $@


# Create final output files (.hex, .eep) from ELF output file.
%.hex: %.elf
	@echo
	@echo $(MSG_FLASH) $@
	$(OBJCOPY) -O $(FORMAT) -R .eeprom -R .fuse $< $@

%.eep: %.elf
	@echo
	@echo $(MSG_EEPROM) $@
	-$(OBJCOPY) -j .eeprom --set-section-flags=.eeprom="alloc,load" \
	--change-section-lma .eeprom=0 --no-change-warnings -O $(FORMAT) $< $@ || exit 0

# Create extended listing file from ELF output file.
%.lss: %.elf
	@echo
	@echo $(MSG_EXTENDED_LISTING) $@
	$(OBJDUMP) -h -S -z $< > $@

# Create a symbol table from ELF output file.
%.sym: %.elf
	@echo
	@echo $(MSG_SYMBOL_TABLE) $@
	$(NM) -P -n $< > $@

# Create bin file for bootloader
%.bin: %.elf
	@echo
	@echo "creating Binary for use with bootloader"
	$(OBJCOPY) $< -S -R .eeprom -R .fuse -O binary $@

# Create library from object files.
.SECONDARY : $(TARGET).a
.PRECIOUS : $(OBJ)
%.a: $(OBJ)
	@echo
	@echo $(MSG_CREATING_LIBRARY) $@
	$(AR) $@ $(OBJ)


# Link: create ELF output file from object files.
.SECONDARY : $(TARGET).elf
.PRECIOUS : $(OBJ)
%.elf: $(OBJ)
	@echo
	@echo $(MSG_LINKING) $@
	$(CC) $(ALL_CFLAGS) $(OBJ) --output $@ $(LDFLAGS)


# Compile: create object files from C source files.
$(OBJDIR)/%.o : %.c
	@echo
	@echo $(MSG_COMPILING) $<
	$(CC) -c $(ALL_CFLAGS) $< -o $@

$(OBJDIR)/%.o : $(SRC_B_DIR)/%.c
	@echo
	@echo $(MSG_COMPILING) $<
	$(CC) -c $(ALL_CFLAGS) $< -o $@


# Compile: create assembler files from C source files.
%.s : %.c
	$(CC) -S $(ALL_CFLAGS) $< -o $@


# Assemble: create object files from assembler source files.
$(OBJDIR)/%.o : $(SRC_B_DIR)/%.S
	@echo
	@echo $(MSG_ASSEMBLING) $<
	$(CC) -c $(ALL_ASFLAGS) $< -o $@

$(OBJDIR)/%.o : %.S
	@echo
	@echo $(MSG_ASSEMBLING) $<
	$(CC) -c $(ALL_ASFLAGS) $< -o $@

# Create preprocessed source for use in sending a bug report.
%.i : %.c
	$(CC) -E -mmcu=$(MCU) -I. $(CFLAGS) $< -o $@


# Target: clean project.
clean: begin clean_list end

clean_list :
	@echo
	@echo $(MSG_CLEANING)
	$(REMOVE) $(TARGET).hex
	$(REMOVE) $(TARGET).eep
	$(REMOVE) $(TARGET).bin
	$(REMOVE) $(TARGET).cof
	$(REMOVE) $(TARGET).elf
	$(REMOVE) $(TARGET).map
	$(REMOVE) $(TARGET).sym
	$(REMOVE) $(TARGET).lss
	$(REMOVE) $(TARGET).txt
	$(REMOVE) $(SRC:%.c=$(OBJDIR)/%.o)
	$(REMOVE) $(SRC:%.c=$(OBJDIR)/%.lst)
	$(REMOVE) $(ASRC:%.S=$(OBJDIR)/%.o)
	$(REMOVE) $(ASRC:%.S=$(OBJDIR)/%.lst)
	$(REMOVE) $(SRC:.c=.s)
	$(REMOVE) $(SRC:.c=.d)
	$(REMOVE) $(SRC:.c=.i)
	$(REMOVE) $(SRC_B:%.c=$(OBJDIR)/%.o)
	$(REMOVE) $(SRC_B:%.c=$(OBJDIR)/%.lst)
	$(REMOVE) $(SRC_B:.c=.s)
	$(REMOVE) $(SRC_B:.c=.d)
	$(REMOVE) $(SRC_B:.c=.i)
	$(REMOVEDIR) .dep


# Create object files directory
$(shell mkdir $(OBJDIR) 2>/dev/null)


# Include the dependency files.
-include $(shell mkdir .dep 2>/dev/null) $(wildcard .dep/*)


# Listing of phony targets.
.PHONY : all begin finish end sizebefore sizeafter gccversion \
build elf hex eep lss sym coff extcoff \
clean clean_list program debug gdb-config
