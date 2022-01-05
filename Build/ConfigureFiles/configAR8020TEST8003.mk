
include $(BUILD_DIR)/define.mk

APPLICATION_DIR ?= $(TOP_DIR)/Application/AR8020Verification
NORFLASH_WPT_SRC_C := $(TOP_DIR)/Application/ConfigData/norflash_WPTable.c

DEBUG ?= n

ifeq ($(DEBUG), y)
CPU0_COMPILE_FLAGS = -g
CPU1_COMPILE_FLAGS = -g
CPU2_COMPILE_FLAGS = -O1 -g
DEBREL = Debug
else
CPU0_COMPILE_FLAGS = -O2 -s -ffunction-sections
CPU1_COMPILE_FLAGS = -O2 -s -ffunction-sections
CPU2_COMPILE_FLAGS = -O2 -s -ffunction-sections
DEBREL = Release
endif

export CPU0_COMPILE_FLAGS
export CPU1_COMPILE_FLAGS
export CPU2_COMPILE_FLAGS

###############################################################################

export CHIP = AR8020
export BOOT = AR8020
export BOARD = AR8020TEST8003

export USB_DEV_CLASS_HID_ENABLE = 1

FUNCTION_DEFS += -DSTM32F746xx -DUSE_WINBOND_SPI_NOR_FLASH -DUSE_ADV7611_EDID_CONFIG_BIN -DRF_8003X -DUSE_EXTERN_LNA
###############################################################################