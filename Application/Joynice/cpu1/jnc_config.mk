# joynices source files path config

# common config
DIR_HAL ?=  $(TOP_DIR)/../hal_8020
SRC_C   := command.c
SRC_C   += ../../Example/test_hal_nv.c
SRC_C   += ../jnc_hal_sys/jnc_hal_sys.c ../jnc_hal_sys/test_jnc_api.c
SRC_C   += $(NORFLASH_WPT_SRC_C)
CFLAGS   += -I../jnc_hal_sys -I$(DIR_HAL)
CXXFLAGS += -I../jnc_hal_sys -I$(DIR_HAL)

# 编译酷芯SDK测试程序（case1）
ifeq ($(jnc_mask), sky_test1)
SRC_CXX += $(wildcard ../hal_test1/*.cpp)
# 编译酷芯SDK测试程序（case2）
else ifeq ($(jnc_mask), sky_test2)
SRC_CXX += $(wildcard ../hal_test2/*.cpp)
# 默认（必须加上，否则SDK无法生成）
else
SRC_CXX += $(wildcard ../hal_test1/*.cpp)
endif

