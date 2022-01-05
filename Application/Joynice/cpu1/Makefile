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

CPU_NUMBER = cpu1
CFLAGS   += $(CPU1_COMPILE_FLAGS)
CXXFLAGS += $(CPU1_COMPILE_FLAGS)

# executable file
ifeq ($(APPLICATION_SCENARIO), sky)
	TARGET = ar8020_skycpu1.bin
endif

ifeq ($(APPLICATION_SCENARIO), ground)
	TARGET = ar8020_gndcpu1.bin
endif

# source file(s), including *.s *.c *.cpp file(s)
SRC_S   := start.s

SRC_C   := main.c command.c ../../Example/auto_test.c ../../Example/test_hal_nv.c ../jnc_hal_sys/jnc_hal_sys.c ../jnc_hal_sys/test_jnc_api.c $(NORFLASH_WPT_SRC_C)

SRC_CXX :=

# compile flags
CFLAGS   += -I$(OUTPUT_HEADER_STAGING_DIR) -I$(TOP_DIR)/Configure/Include -I../share -I../../Example
CXXFLAGS += -I$(OUTPUT_HEADER_STAGING_DIR) -I$(TOP_DIR)/Configure/Include -I../share -I../../Example

# libs
LD_LIBS := --start-group
LD_LIBS += -lutility -lhal -lboard -lAR8020drv -lm -lcommon -lc_nano -lgcc

# inserted by joynices. must after LD_LIBS defined AND before ifeq($(SRC_CXX))
include ./jnc_config.mk

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
	rm -f  *.o  *.elf  *.bin  *.dis  *.mif  *.map $(OUTPUT_DIR)/*.mif

.PHONY: all clean $(TARGET)

#########################################################################

include $(BUILD_DIR)/rules.mk

ifneq "$(MAKECMDGOALS)" "clean"
sinclude .depend
endif

#########################################################################



