#include <string.h>
#include <stdlib.h>
#include "command.h"
#include "debuglog.h"
#include "serial.h"
#include "cmsis_os.h"
#include "bb_ctrl.h"
#include "test_bb.h"
#include "cmd_line.h"
#include "test_common.h"


STRU_CMD_ENTRY_T g_cmdArray[] =
{
    {1, (f_cmdline)command_readMemory, "read", "<address>"},
    {2, (f_cmdline)command_writeMemory, "write", "<address> <data>"},
    {2, (f_cmdline)command_set_loglevel, "set_loglevel", "<cpuid> <loglevel>"},

    END_OF_CMD_ENTRY
};

