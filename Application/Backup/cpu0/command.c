#include "debuglog.h"
#include <string.h>
#include <stdlib.h>
#include "cmsis_os.h"
#include "memory_config.h"
#include "hal_ret_type.h"
#include "ar_freertos_specific.h"
#include "upgrade.h"
#include "cmd_line.h"
#include "test_common.h"
#include "test_upgrade.h"
#include "test_hal_spi_flash.h"
#include "factory.h"

static void command_switchto_ios(void);

STRU_CMD_ENTRY_T g_cmdArray[] =
{
    {1, (f_cmdline)upgrade,  "upgrade", "<filename>"},
    {2, (f_cmdline)command_set_loglevel, "set_loglevel", "<cpuid> <loglevel>"},
    {0, (f_cmdline)FCT_Reset, "FCT_Reset", ""},
    {0, (f_cmdline)command_switchto_ios, "switchtoios", ""},
    {0, (f_cmdline)command_TestReadFlashReg, "ReadFlashReg", ""},
    {1, (f_cmdline)command_TestSetFlashStatusReg1, "SetFlashStatusReg1", "<reg_value>"},
    {1, (f_cmdline)command_TestSetFlashStatusReg2, "SetFlashStatusReg2", "<reg_value>"},
    {0, (f_cmdline)command_TestReadFlashID, "ReadFlashID", ""},
    END_OF_CMD_ENTRY
};

extern uint8_t g_u8CurrentHost;     /* 0: normal   1: iphone */
static void command_switchto_ios(void)
{
    g_u8CurrentHost = 1;
}

