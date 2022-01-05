#include <stdint.h>
#include <string.h>

#include "upgrade_command.h"
#include "debuglog.h"
#include "upgrade_sd.h"
#include "upgrade_uart.h"

int UPGRADE_CommandRun(char *cmdArray[], uint32_t cmdNum)
{
	if (memcmp(cmdArray[0], "uart_upgradeapp", strlen("uart_upgradeapp")) == 0)
    {
        UPGRADE_APPFromUart();
    }
    else if (memcmp(cmdArray[0], "uart_boot", strlen("uart_boot")) == 0)
    {
        UPGRADE_BootloadFromUart();
    }
    else
    {
        DLOG_Critical("%s Command not found. Please use the commands like:",cmdArray[0]);
        DLOG_Critical("uart_upgradeapp");
    }

    return 0;
}

