#include "debuglog.h"
#include "sys_event.h"
#include "hal.h"
#include "hal_sys_ctl.h"
#include "hal_uart.h"
#include "cmd_line.h"
#include "board_watchdog.h"

void CONSOLE_Init(void)
{
    DLOG_Init(CMD_exec_cmd, NULL, DLOG_CLIENT_PROCESSOR);
}

/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
int main(void)
{
    HAL_SYS_CTL_Init(NULL);
    WATCHDOG_Init();

    /* initialize the uart */
    CONSOLE_Init();
    dlog_set_output_level(LOG_LEVEL_WARNING);
    DLOG_Critical("cpu1 start!!! \n");

    /* We should never get here as control is now taken by the scheduler */
    for( ;; )
    {
        SYS_EVENT_Process();
        DLOG_Process(NULL);
    }
}

