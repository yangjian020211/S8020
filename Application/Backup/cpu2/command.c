#include <string.h>
#include <stdlib.h>
#include "debuglog.h"
#include "serial.h"
#include "cmsis_os.h"
#include "test_common.h"
#include "hal_ret_type.h"
#include "cmd_line.h"

#include "test_common.h"


STRU_CMD_ENTRY_T g_cmdArray[] =
{
    {2, (f_cmdline)command_set_loglevel, "set_loglevel", "<cpuid> <loglevel>"},
    END_OF_CMD_ENTRY
};
