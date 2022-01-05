#include "debuglog.h"
#include "serial.h"
#include "debuglog.h"
#include <string.h>
#include <stdlib.h>
#include "cmsis_os.h"
#include "hal_sd.h"
#include "hal_ret_type.h"
#include "hal_nvic.h"
#include "cmd_line.h"
#include "test_bb_com.h"

#include "test_common.h"

STRU_CMD_ENTRY_T g_cmdArray[] =
{
    {1, (f_cmdline)command_readMemory,   "read",  "<address>"},
    {2, (f_cmdline)command_writeMemory,  "write", "<address> <data>"},
    {2, (f_cmdline)command_set_loglevel, "set_loglevel", "<cpuid> <loglevel>"},
    {1, (f_cmdline)command_TestSpiSkyInit, "TestSpiSkyInit", "<maxLen>"},
    {1, (f_cmdline)command_TestSpiGrdInit, "TestSpiGrdInit", "<maxLen>"},
    {2, (f_cmdline)command_TestSpiGrdSend, "TestSpiGrdSend", "<start_value> <len>"},
    {1, (f_cmdline)command_TestComInitSend, "TestComInitSend", "<Init_flag> <Init_len>"},
    {1, (f_cmdline)command_TestComInitRcv, "TestComInitRcv", "<Init_flag> <Init_len>"},
    END_OF_CMD_ENTRY
};
