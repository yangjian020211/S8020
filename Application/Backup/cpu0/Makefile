#top file directory
TOP_DIR ?= ../../..

# build file directory
BUILD_DIR ?= $(TOP_DIR)/Build

# binary file director
OUTPUT_DIR ?= $(TOP_DIR)/Output
OUTPUT_HEADER_STAGING_DIR ?= $(OUTPUT_DIR)/Staging/Inc
OUTPUT_LIB_STAGING_DIR ?= $(OUTPUT_DIR)/Staging/Lib

# utility file director
UTILITY_DIR ?= $(TOP_DIR)/Utility

# current directory
CURDIR ?= $(shell pwd)

include $(BUILD_DIR)/config.mk

###############################################################################

CPU_NUMBER = cpu0
CFLAGS   += $(CPU0_COMPILE_FLAGS)
CXXFLAGS += $(CPU0_COMPILE_FLAGS)


# source file(s), including *.s *.c *.cpp file(s)
SRC_S   := start.s

ifeq ($(APPLICATION_SCENARIO), sky)
TARGET = ar8020_backupcpu0.bin
SRC_C   := main_sky.c
SRC_CXX :=
endif

SRC_C   += command.c \
           ../../Example/test_upgrade.c \
           ../../Example/test_common.c \
           ../../Example/test_usbh.c \
           ../../Example/auto_test.c \
           ../../Example/test_hal_spi_flash.c \
		   $(NORFLASH_WPT_SRC_C)

# compile flags
CFLAGS   += -I$(OUTPUT_HEADER_STAGING_DIR) -I$(TOP_DIR)/Configure/Include -I../share -I../../Example
CXXFLAGS += -I$(OUTPUT_HEADER_STAGING_DIR) -I$(TOP_DIR)/Configure/Include -I../share -I../../Example

# libs
LD_LIBS := --start-group
LD_LIBS += -lfreertos -lutility -lfatfs -lhal -lboard -lAR8020drv -lm -lcommon -lc_nano -lgcc
ifeq ($(SRC_CXX), )
LD_LIBS += --end-group
else
LD_LIBS += -lstdc++ --end-group
endif

# link flags
LDFLAGS += -Bstatic
LDFLAGS += -L$(OUTPUT_LIB_STAGING_DIR) -L$(OUTPUT_LIB_STAGING_DIR)/$(CPU_NUMBER) $(LD_LIBS)

###############################################################################

# object file(s)
OBJ_S   := $(patsubst %.s,%.ao,$(SRC_S))
OBJ_C   := $(patsubst %.c,%.o,$(SRC_C))
OBJ_CXX := $(patsubst %.cpp,%.cpp.o,$(SRC_CXX))

all: .depend $(TARGET)

ifeq ($(SRC_CXX), )
$(TARGET): $(OBJ_S) $(OBJ_C) $(OBJ_CXX)
else
$(TARGET): $(OBJ_S) $(OBJ_C) $(OBJ_CXX) $(CRTBEGIN_OBJ) $(CRTEND_OBJ)
endif
	$(LD) $^ $(LDFLAGS) -Tar8020.lds -o $(TARGET).elf
	$(OBJCOPY) -O binary $(TARGET).elf $@
	$(OBJDUMP) -D $(TARGET).elf > $(TARGET).dis
	$(NM) $(TARGET).elf > $(TARGET).map
	@cp $(TARGET) $(OUTPUT_LIB_STAGING_DIR)

clean:
	rm -f  $(OBJ_S) $(OBJ_C) $(OBJ_CXX) .depend
	rm -f *.o *.elf  *.bin  *.dis  *.mif  *.map $(OUTPUT_DIR)/*.mif

.PHONY: all clean $(TARGET)

#########################################################################

include $(BUILD_DIR)/rules.mk

ifneq "$(MAKECMDGOALS)" "clean"
sinclude .depend
endif

#########################################################################



