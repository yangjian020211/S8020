TOP_DIR ?= ../../..

# build file directory
BUILD_DIR ?= $(TOP_DIR)/Build

# binary file director
OUTPUT_DIR ?= $(TOP_DIR)/Output
OUTPUT_HEADER_STAGING_DIR ?= $(OUTPUT_DIR)/Staging/Inc
OUTPUT_LIB_STAGING_DIR ?= $(OUTPUT_DIR)/Staging/Lib

# current directory
CURDIR ?= $(shell pwd)

include $(BUILD_DIR)/config.mk

###############################################################################
# static library name
LIB = libcmbacktrace.a

ifeq ($(CPU_NUMBER), cpu0)
    CFLAGS += $(CPU0_COMPILE_FLAGS)
endif

ifeq ($(CPU_NUMBER), cpu1)
    CFLAGS += $(CPU1_COMPILE_FLAGS)
endif

ifeq ($(CPU_NUMBER), cpu2)
    CFLAGS += $(CPU2_COMPILE_FLAGS)
endif


# source file(s), including c file(s)
SRC_C   := Src/cm_backtrace.c

INCDIRS := -IInc \
           -I$(TOP_DIR)/Kernel/FreeRTOS/portable/GCC/ARM_CM7/r0p1 \
           -I$(TOP_DIR)/Library/Artosyn/Common/Inc \
           -I$(TOP_DIR)/Kernel/FreeRTOS/include \
           -I$(TOP_DIR)/Configure/Include \
           -I$(TOP_DIR)/Driver/Chip/AR8020/USB/USB_LL_Driver/Inc \
           -I$(TOP_DIR)/Driver/Chip/AR8020/USB/USB_Host/Class/MSC/Inc \
           -I$(TOP_DIR)/Driver/Chip/AR8020/USB/USB_Host/Core/Inc \
           -I$(TOP_DIR)/Driver/Chip/AR8020/UART/Inc \
           -I$(TOP_DIR)/Driver/Chip/AR8020/SD/Inc \
           -I$(TOP_DIR)/Driver/Chip/AR8020/SysCtl/Inc \
           -I$(TOP_DIR)/Driver/HAL/Inc/ \
           -I$(TOP_DIR)/Library/Artosyn/Common/Inc

# object file(s)
OBJ_C   := $(patsubst %.c,%.o,$(SRC_C))

CFLAGS +=

all: .depend $(LIB) install

$(LIB):  $(OBJ_C)
	@echo "Generating static library:  $(CPU_NUMBER) " $(notdir $@)
	@$(AR) $(ARFLAGS) $@ $^

install:
	@mkdir -p $(OUTPUT_LIB_STAGING_DIR)/$(CPU_NUMBER)
	@mkdir -p $(OUTPUT_HEADER_STAGING_DIR)
	@cp -f $(LIB) $(OUTPUT_LIB_STAGING_DIR)/$(CPU_NUMBER)
	@cp -f Inc/cm_backtrace.h $(OUTPUT_HEADER_STAGING_DIR)
	@cp -f Inc/cmb_def.h $(OUTPUT_HEADER_STAGING_DIR)
	@cp -f Inc/cmb_cfg.h $(OUTPUT_HEADER_STAGING_DIR)

clean:
	@echo "Cleaning..."
	@$(RM) $(LIB) $(OBJ_C) $(OBJ_CPP) .depend $(OUTPUT_LIB_STAGING_DIR)/$(CPU_NUMBER)/$(LIB)
	@$(RM) $(OUTPUT_HEADER_STAGING_DIR)/cm_backtrace.h
	@$(RM) $(OUTPUT_HEADER_STAGING_DIR)/cmb_def.h
	@$(RM) $(OUTPUT_HEADER_STAGING_DIR)/cmb_cfg.h

.PHONY: all clean install
#########################################################################

include $(BUILD_DIR)/rules.mk

sinclude .depend

#########################################################################

