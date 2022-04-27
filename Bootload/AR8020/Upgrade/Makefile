# top file directory
TOP_DIR ?= ../../..

# build file directory
BUILD_DIR ?= $(TOP_DIR)/Build

# binary file director
OUTPUT_DIR ?= $(TOP_DIR)/Output
OUTPUT_HEADER_STAGING_DIR ?= $(OUTPUT_DIR)/Staging/Inc
OUTPUT_LIB_STAGING_DIR ?= $(OUTPUT_DIR)/Staging/Lib
OUTPUT_LIB_STAGING_DIR2?= $(OUTPUT_DIR)/Lib
SRC_C_DIR ?= $(TOP_DIR)/Driver/Chip/AR8020
INC_LIBRARY_H_DIR ?= $(TOP_DIR)/Library/Artosyn/Utility/Inc
SRC_LIBRARY_C_DIR ?= $(TOP_DIR)/Library/Artosyn/Utility/Src

# utility file director
UTILITY_DIR ?= $(TOP_DIR)/Utility

# current directory
CURDIR ?= $(shell pwd)

include $(BUILD_DIR)/config.mk

###############################################################################
# libs
CPU_NUMBER := cpu0

CFLAGS += $(CPU0_COMPILE_FLAGS)

LD_LIBS := --start-group
LD_LIBS += -lfatfs -lutility -lhal -lboard -lAR8020drv -lcommon -lsimplec -lgcc -lcmbacktrace -lAR8020drv_cpu2
LD_LIBS += --end-group

LDFLAGS += -Bstatic --gc-section
LDFLAGS += -L$(OUTPUT_LIB_STAGING_DIR)/$(CPU_NUMBER) $(LD_LIBS) -L$(OUTPUT_LIB_STAGING_DIR)/cpu2 -L$(OUTPUT_LIB_STAGING_DIR2)/cpu2
                 
# source file(s), including c file(s)
SRC_S   :=  Src/upgrade_start.s 
SRC_C   :=  Src/upgrade_main.c \
            Src/upgrade_command.c \
            Src/upgrade_uart.c \
			$(NORFLASH_WPT_SRC_C) \
			$(SRC_LIBRARY_C_DIR)/image.c

CFLAGS += -I./Inc \
          -I$(OUTPUT_HEADER_STAGING_DIR) \
          -I$(TOP_DIR)/Configure/Include \
		  -I$(TOP_DIR)/Driver/Chip/AR8020/SysCtl/Inc/ \
		  -I$(INC_LIBRARY_H_DIR)

# object file(s)
OBJ_C   := $(patsubst %.s,%.o,$(SRC_S)) $(patsubst %.c,%.o,$(SRC_C)) 

#$(patsubst %.s,%.o,$(SRC_C))

# executable file
TARGET = ar8020_upgrade.bin

###############################################################################

all: .depend $(TARGET) install

$(TARGET): $(OBJ_C)
	$(LD) $^ $(LDFLAGS) -TSrc/upgrade_ar8020.lds -o $(TARGET).elf 
	$(OBJCOPY) -O binary $(TARGET).elf $@
	$(OBJDUMP) -D -S $(TARGET).elf > $(TARGET).dis
	$(NM) $(TARGET).elf > $(TARGET).map

install:
	@cp $(TARGET) $(OUTPUT_LIB_STAGING_DIR)/ 
	

clean:
	rm -f $(OBJ_C) .depend 
	rm -f $(TARGET) $(TARGET).elf  $(TARGET).bin  $(TARGET).dis  $(TARGET).mif  $(TARGET).map $(OUTPUT_DIR)/$(TARGET).mif $(TARGET_FLASH)
	rm -f $(OUTPUT_LIB_STAGING_DIR)/$(TARGET)
	

.PHONY: all clean $(TARGET)

#########################################################################

include $(BUILD_DIR)/rules.mk

ifneq "$(MAKECMDGOALS)" "clean"
sinclude .depend
endif

#########################################################################



