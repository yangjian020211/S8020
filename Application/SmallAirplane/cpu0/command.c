#include "command.h"
#include "debuglog.h"
#include "debuglog.h"
#include <string.h>
#include <stdlib.h>
#include "cmsis_os.h"
#include "test_i2c_adv7611.h"
#include "test_hal_camera.h"
#include "hal_dma.h"
#include "test_upgrade.h"
#include "memory_config.h"
#include "hal_ret_type.h"
#include "test_hal_mipi.h"
#include "test_usbh.h"
#include "test_hal_nv.h"
#include "ar_freertos_specific.h"
#include "test_bb.h"
#include "cmd_line.h"
#include "factory.h"
#include "test_common.h"
#include "sys_event.h"
#include "memory.h"
//#include "test_xc6130.h"
#include "test_hal_uart.h"
#include "test_hal_i2c_24c256.h"
#include "test_hal_spi_flash.h" 
#include "testhal_pwm.h"
#include "testhal_gpio.h"
#include "test_hal_can.h"
#include "test_hal_adc.h"
#include "test_hal_i2c.h"
#include "test_bb_com.h"
#include "test_camera.h"
#include "test_hal_spi_flash.h"


void command_upgrade(void);
void command_malloc(char *size);


STRU_CMD_ENTRY_T g_cmdArray[] =
{
    {1, (f_cmdline)command_readMemory, "read", "<address>"},// 
    {2, (f_cmdline)command_writeMemory, "write", "<address> <data>"},
    {1, (f_cmdline)upgrade,  "upgrade", "<filename>"},
    {1, (f_cmdline)gndforskyupgrade, "gndforskyupgrade", "<filename>"},
    {2, (f_cmdline)command_TestHalCameraInit, "test_camera_init",  "<rate 0~1> <mode 0~8>"},
    {2, (f_cmdline)command_TestCameraWrite, "test_write_camera", "<subAddr(hex)> <value>(hex)"},
    {1, (f_cmdline)command_TestCameraRead, "test_read_camera", "<subAddr(hex)>"},
    
    {0, (f_cmdline)command_TestXc6130Sc2143Init, "TestXc6130Sc2143Init",  ""}, //TestXc6130Init test_hal_mipi_init write 0xa0010000 0x01d40736
    {0, (f_cmdline)command_TestXc6130Sc2143_1080p30fps_600fov, "TestXc6130Sc2143_1080p30fps_600fov",  ""},
    {0, (f_cmdline)command_TestXc6130Sc2143_1080p30fps_800fov, "TestXc6130Sc2143_1080p30fps_800fov",  ""},
    {0, (f_cmdline)command_TestXc6130Sc2143_1080p30fps_1200fov, "TestXc6130Sc2143_1080p30fps_1200fov",  ""},
    {0, (f_cmdline)command_SwitchI2CToXc6130, "SwitchI2CToXc6130",  ""},
    {0, (f_cmdline)command_SwitchI2CToSc2143, "SwitchI2CToSc2143",  ""},
    {2, (f_cmdline)command_TestXc6130Write, "TestXc6130Write", "<subAddr(hex)> <value>(hex)"},
    {1, (f_cmdline)command_TestXc6130Read, "TestXc6130Read", "<subAddr(hex)>"},
    {2, (f_cmdline)command_TestSc2143Write, "TestSc2143Write", "<subAddr(hex)> <value>(hex)"},
    {1, (f_cmdline)command_TestSc2143Read, "TestSc2143Read", "<subAddr(hex)>"},
    
    {4, (f_cmdline)command_TestHalMipiInit, "test_hal_mipi_init", "<toEncoderCh 0~1> <width>, <hight> <frameRate>"},
    {1, (f_cmdline)command_TestSetMipiLane, "TestSetMipiLane", "<lane>"},
    {1, (f_cmdline)command_startBypassVideo, "startbypassvideo", "channel"},
    {0, (f_cmdline)command_stopBypassVideo, "stopbypassvideo",  ""},// 
    {0, (f_cmdline)command_TestNvResetBbRcId, "NvResetBbRcId",  ""},
    {7, (f_cmdline)command_TestNvSetBbRcId, "NvSetBbRcId",  "<rc id1~5> <vt id0~1>"},
    {5, (f_cmdline)command_TestNvSetChipId, "NvSetChipId",  "<chip id1~5>"},
    {0, (f_cmdline)command_TestNvResetBbRcId, "NvResetBbRcId",  ""},
    {1, (f_cmdline)command_test_BB_uart, "command_test_BB_uart", "<param>"},
    {1, (f_cmdline)command_malloc, "malloc", "size"},
    {0, (f_cmdline)ar_top, "top", ""},
    {2, (f_cmdline)command_set_loglevel, "set_loglevel", "<cpuid> <loglevel>"},
    {0, (f_cmdline)FCT_Reset, "FCT_Reset", ""},
    {0, (f_cmdline)SYS_EVENT_MallocFreeCntCheck, "event_mem", ""},
    {0, (f_cmdline)memory_malloc_free_check, "pmem", ""},

    {2, (f_cmdline)command_TestHalAdRead, "TestHalAdRead", "<ch> <nch>"},

    
    {3, (f_cmdline)command_TestHalI2cInit, "TestHalI2cInit", "<ch> <i2c_addr> <speed>"},
    {5, (f_cmdline)command_TestHalI2cWrite, "TestHalI2cWrite", "<ch> <subAddr> <subAddrLen> <data> <dataLen>"},
    {4, (f_cmdline)command_TestHalI2cRead, "TestHalI2cRead", "<ch> <subAddr> <subAddrLen> <dataLen>"},

    
    {1, (f_cmdline)command_TestSpiSkyInit, "TestSpiSkyInit", "<maxLen>"},
    {1, (f_cmdline)command_TestSpiGrdInit, "TestSpiGrdInit", "<maxLen>"},
    {2, (f_cmdline)command_TestSpiGrdSend, "TestSpiGrdSend", "<start_value> <len>"},
    
	{0, (f_cmdline)command_TestGc2145Init, "TestGc2145Init",  ""}, 
    {0, (f_cmdline)command_TestGC2145DVP, "GC2145DVP",  ""}, 
    {2, (f_cmdline)command_TestGc2145Write, "Gc2145Write", "<subAddr(hex)> <value>(hex)"},
    {1, (f_cmdline)command_TestGc145Read, "Gc145Read", "<subAddr(hex)>"},
    {1, (f_cmdline)command_TestDVP2Encoder, "DVP2Encoder", "<ch>"},

    
    {0, (f_cmdline)command_TestXc6130Ov5648Init, "TestXc6130Ov5648Init",  ""}, 
    {0, (f_cmdline)command_SwitchI2CToOv5648, "SwitchI2CToOv5648",  ""},
    {2, (f_cmdline)command_TestOv5648Write, "TestOv5648Write", "<subAddr(hex)> <value>(hex)"},
    {1, (f_cmdline)command_TestOv5648Read, "TestOv5648Read", "<subAddr(hex)>"},
    {0, (f_cmdline)command_TestReadFlashReg, "ReadFlashReg", ""},
    {1, (f_cmdline)command_TestSetFlashStatusReg1, "SetFlashStatusReg1", "<reg_value>"},
    {1, (f_cmdline)command_TestSetFlashStatusReg2, "SetFlashStatusReg2", "<reg_value>"},
    {2, (f_cmdline)commandhal_TestSetGpio, "SetGPIO", "<GPIONUM> <VALUE>"},
    END_OF_CMD_ENTRY
};


unsigned int command_str2uint(char *str)
{
    return strtoul(str, NULL, 0); 
}

void delay_ms(uint32_t num)
{
    volatile int i;
    for (i = 0; i < num * 100; i++);
}

void command_malloc(char *size)
{
    unsigned int mallocSize;
	char *malloc_addr;
	
    mallocSize = command_str2uint(size);
	malloc_addr = malloc_safe(mallocSize);

	if (malloc_addr != 0)
	{
		DLOG_Info("0x%08x\n", malloc_addr);
	}

	return;
}

