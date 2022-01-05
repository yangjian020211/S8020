#include "command.h"
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

static void command_set_loglevel(char* cpu, char* loglevel);

STRU_CMD_ENTRY_T g_cmdArray[] =
{
    {2, (f_cmdline)command_set_loglevel, "set_loglevel", "<cpuid> <loglevel>"},

    END_OF_CMD_ENTRY
};


unsigned int command_str2uint(char *str)
{
    return strtoul(str, NULL, 0); 
}

static void command_set_loglevel(char* cpu, char* loglevel)
{
    uint8_t level = command_str2uint(loglevel);
    if (memcmp(cpu, "cpu1", strlen("cpu1")) == 0)
    {
        dlog_set_output_level(level);
    }

    return;
}

