#include <string.h>
#include <stdlib.h>
#include "cmd_line.h"
#include "command.h"
#include "debuglog.h"
#include "serial.h"
#include "test_upgrade.h"
#include "cmsis_os.h"
#include "testhal_pwm.h"
#include "test_hal_uart.h"
#include "testhal_gpio.h"
#include "test_usbh.h"
#include "test_hal_nv.h"
#include "ar_freertos_specific.h"
#include "uvc_task.h"
#include "hal_usb_host.h"
#include "cmd_line.h"
#include "factory.h"
#include "test_common.h"
#include "test_hal_spi_flash.h"


static void command_send_usb(char *len, char *ep);

static void command_switchto_ios(void);


STRU_CMD_ENTRY_T g_cmdArray[] =
{
    {1, (f_cmdline)command_readMemory, "read", "<address>"},// 
    {2, (f_cmdline)command_writeMemory, "write", "<address> <data>"},
    {1, (f_cmdline)upgrade,  "upgrade", "<filename>"},
    {3, (f_cmdline)commandhal_TestPwm,  "testhal_Testpwm", "<pu8_pwmNum> <pu8_lowus> <pu8_highus>"},
    {2, (f_cmdline)command_TestHalUartInit, "test_hal_uart_init", "<ch> <baudr>"},
    {2, (f_cmdline)command_TestHalUartIntSet,  "test_hal_uart_set_int", "<ch> <flag>"},
    {2, (f_cmdline)command_TestHalUartTx, "test_hal_uart_tx", "<ch> <len>"},
    {1, (f_cmdline)command_TestHalUartRx, "test_hal_uart_rx", "<ch>"},
    {2, (f_cmdline)commandhal_TestGpioNormal, "testhal_TestGpioNormal", "<gpionum> <highorlow>"},
    {3, (f_cmdline)commandhal_TestGpioInterrupt, "testhal_TestGpioInterrupt", "<gpionum> <inttype> <polarity>"},
    {1, (f_cmdline)command_startBypassVideo, "startbypassvideo", "channel"},
    {0, (f_cmdline)command_stopBypassVideo, "stopbypassvideo",  ""},
    {0, (f_cmdline)command_TestNvResetBbRcId, "NvResetBbRcId",  ""},
    {7, (f_cmdline)command_TestNvSetBbRcId, "NvSetBbRcId",  "<rc id1~5> <vt id0~1>"},
    {5, (f_cmdline)command_TestNvSetChipId, "NvSetChipId",  "<chip id1~5>"},
    {0, (f_cmdline)command_ViewUVC, "viewuvc",  ""},
    {0, (f_cmdline)ar_top, "top", ""},
    {2, (f_cmdline)command_startUVC, "startuvc", "<width> <height>"},
    {0, (f_cmdline)command_saveUVC, "saveuvc", ""},
    {0, (f_cmdline)command_stopSaveUVC, "stopsaveuvc", ""},
    {0, (f_cmdline)command_showUVC, "showuvc", ""},
    {2, (f_cmdline)command_set_loglevel, "set_loglevel", "<cpuid> <loglevel>"},
    {2, (f_cmdline)command_setUVCAttribute, "setuvcattr", "<index> <value>"},
    {2, (f_cmdline)command_getUVCAttribute, "getuvcattr", "<index> <type>"},
    {0, (f_cmdline)command_uvchelp, "uvchelp", ""},
    {0, (f_cmdline)FCT_Reset, "FCT_Reset", ""},
    {2, (f_cmdline)command_send_usb, "sendusb", "<datalen> <endpoint>"},
    {0, (f_cmdline)command_switchto_ios, "switchtoios", ""},
    {0, (f_cmdline)command_TestReadFlashReg, "ReadFlashReg", ""},
    {1, (f_cmdline)command_TestSetFlashStatusReg1, "SetFlashStatusReg1", "<reg_value>"},
    {1, (f_cmdline)command_TestSetFlashStatusReg2, "SetFlashStatusReg2", "<reg_value>"},
    END_OF_CMD_ENTRY
};


unsigned int command_str2uint(char *str)
{
    return strtoul(str, NULL, 0); 
}

static uint8_t g_stringSend[512] = {1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20};
static void command_send_usb(char *len, char *ep)
{
    uint16_t   length = command_str2uint(len);
    uint8_t    endpoint = strtoul(ep, NULL, 16);

    HAL_USB_SendData(g_stringSend, length, 0, endpoint);
}

extern uint8_t g_u8CurrentHost;     /* 0: normal   1: iphone */
static void command_switchto_ios(void)
{
    g_u8CurrentHost = 1;
}


