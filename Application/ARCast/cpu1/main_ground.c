#include "debuglog.h"
#include "command.h"
#include "sys_event.h"
#include "hal.h"
#include "hal_sys_ctl.h"
#include "hal_uart.h"
#include "board_watchdog.h"
#include "cmd_line.h"

void console_init(uint32_t uart_num, uint32_t baut_rate)
{
    DLOG_Init(CMD_exec_cmd, NULL ,DLOG_CLIENT_PROCESSOR);
}

/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
int main(void)
{
    WATCHDOG_Init();
    HAL_SYS_CTL_Init(NULL);

    /* initialize the uart */
    console_init(0,115200);
    dlog_set_output_level(LOG_LEVEL_WARNING);
    DLOG_Info("cpu1 start!!! \n");

    /* We should never get here as control is now taken by the scheduler */
    for( ;; )
    {
        SYS_EVENT_Process();
        DLOG_Process(NULL);
    }
} 
