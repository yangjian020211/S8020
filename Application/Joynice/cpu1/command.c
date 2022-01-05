#include <string.h>
#include <stdlib.h>
#include "command.h"
#include "debuglog.h"
#include "serial.h"
#include "cmsis_os.h"
#include "cmd_line.h"
#include "test_hal_nv.h"
#include "../jnc_hal_sys/test_jnc_api.h"

void command_readMemory(char *addr);
void command_writeMemory(char *addr, char *value);
static void command_set_loglevel(char* cpu, char* loglevel);

STRU_CMD_ENTRY_T g_cmdArray[] =
{
    {1, (f_cmdline)command_readMemory, "read", "<address>"},
    {2, (f_cmdline)command_writeMemory, "write", "<address> <data>"},
    {2, (f_cmdline)command_set_loglevel, "set_loglevel", "<cpuid> <loglevel>"},
    
    {0, (f_cmdline)command_TestUsrDataWrite, "TestUsrDataWrite", ""},
    {0, (f_cmdline)command_TestUsrDataRead, "TestUsrDataRead", ""},
    {1, (f_cmdline)command_TestUsrDataDestroy, "TestUsrDataDestroy", "<addr 0~120K>"},

    {0, (f_cmdline)command_JncUsrDataWrite, "JncUsrDataWrite", ""},
    {0, (f_cmdline)command_JncUsrDataRead, "JncUsrDataRead", ""},
    {1, (f_cmdline)command_JncUsrDataDestroy, "JncUsrDataDestroy", "<addr 0~120K>"},

    {0, (f_cmdline)command_JncInit, "JncInit", ""},
    {0, (f_cmdline)command_JncRcWrite, "JncRcWrite", ""},
    {0, (f_cmdline)command_JncRcRead, "JncRcRead", ""},
    {0, (f_cmdline)command_JncTelemWrite, "JncTelemWrite", ""},
    {0, (f_cmdline)command_JncTelemRead, "JncTelemRead", ""},

    END_OF_CMD_ENTRY
};


unsigned int command_str2uint(char *str)
{
    return strtoul(str, NULL, 0); 
}

void command_readMemory(char *addr)
{
    unsigned int readAddress;
    unsigned char row;
    unsigned char column;

    readAddress = command_str2uint(addr);

    if (readAddress == 0xFFFFFFFF)
    {

        DLOG_Warning("read address is illegal\n");

        return;
    }

    /* align to 4 bytes */
    readAddress -= (readAddress % 4);

    /* print to serial */
    for (row = 0; row < 8; row++)
    {
        /* new line */
        DLOG_Info("0x%08x: 0x%08x 0x%08x 0x%08x 0x%08x 0x%08x 0x%08x 0x%08x 0x%08x\n ", 
                  readAddress,
                  *(uint32_t *)readAddress,
                  *(uint32_t *)(readAddress + 4),
                  *(uint32_t *)(readAddress + 8),
                  *(uint32_t *)(readAddress + 12),
                  *(uint32_t *)(readAddress + 16),
                  *(uint32_t *)(readAddress + 20),
                  *(uint32_t *)(readAddress + 24),
                  *(uint32_t *)(readAddress + 28));

        readAddress += 32;
    }
}

void command_writeMemory(char *addr, char *value)
{
    unsigned int writeAddress;
    unsigned int writeValue;

    writeAddress = command_str2uint(addr);

    if (writeAddress == 0xFFFFFFFF)
    {

        DLOG_Warning("write address is illegal\n");

        return;
    }

    writeValue   = command_str2uint(value);

    *((unsigned int *)(writeAddress)) = writeValue;
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

