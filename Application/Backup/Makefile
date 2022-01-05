#top file directory
TOP_DIR ?= ../..

# build file directory
BUILD_DIR ?= $(TOP_DIR)/Build

# binary file director
OUTPUT_DIR ?= $(TOP_DIR)/Output
OUTPUTCFG_DIR ?= $(TOP_DIR)/Output/Staging/Config
OUTPUT_HEADER_STAGING_DIR ?= $(OUTPUT_DIR)/Staging/Inc
OUTPUT_LIB_STAGING_DIR ?= $(OUTPUT_DIR)/Staging/Lib

# utility file director
UTILITY_DIR ?= $(TOP_DIR)/Utility

# current directory
CURDIR ?= $(shell pwd)

include $(BUILD_DIR)/config.mk

###############################################################################

# executable file
TARGET = ar8020_all
TARGET_CPU = ar8020_cpu
TARGET_CPU0 = ar8020_cpu0
TARGET_CPU1 = ar8020_cpu1
TARGET_CPU2 = ar8020_cpu2
TARGET_FLASH = ar8020.bin
TARGET_FLASH_BOOT = boot.bin
TARGET_FLASH_APP = app_backup.bin
TARGET_CONFIG	 = cfgdata.bin

APPLICATION_SCENARIO ?= all
ENCRYPTKEY1 ?= n
ENCRYPTKEY2 ?= n

###############################################################################

sky: APPLICATION_SCENARIO := sky
sky: all

all: .depend $(TARGET)

$(TARGET): $(TARGET_CPU)
	@echo $(APPLICATION_SCENARIO) > .APPLICATION_SCENARIO

$(TARGET_CPU):
	@$(MAKE) -C cpu0 all APPLICATION_SCENARIO=sky
	@$(MAKE) -C cpu1 all APPLICATION_SCENARIO=sky
	@$(MAKE) -C cpu2 all APPLICATION_SCENARIO=sky

clean:
	@rm -f *.bin
	@$(MAKE) -C cpu0 clean APPLICATION_SCENARIO=`cat .APPLICATION_SCENARIO`
	@$(MAKE) -C cpu1 clean APPLICATION_SCENARIO=`cat .APPLICATION_SCENARIO`
	@$(MAKE) -C cpu2 clean APPLICATION_SCENARIO=`cat .APPLICATION_SCENARIO`

.PHONY: all clean $(TARGET) $(TARGET_CPU)

#########################################################################

include $(BUILD_DIR)/rules.mk

sinclude .depend

#########################################################################



