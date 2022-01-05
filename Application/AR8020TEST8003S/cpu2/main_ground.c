#include "serial.h"
#include "debuglog.h"
#include "test_bb.h"
#include "sys_event.h"
#include "hal_sys_ctl.h"
#include "hal_bb.h"
#include "hal.h"
#include "hal_gpio.h"
#include "bb_customerctx.h"
#include "test_bb.h"
#include "cmd_line.h"
#include "board_watchdog.h"
#include "test_search_id.h"

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
    STRU_HAL_SYS_CTL_CONFIG *pst_cfg;
    HAL_SYS_CTL_GetConfig(&pst_cfg);
    HAL_SYS_CTL_Init(pst_cfg);

    console_init(2, 115200);
    dlog_set_output_level(LOG_LEVEL_WARNING);
    DLOG_Critical("main ground function start \n");

    WATCHDOG_InitCpu2();
    HAL_BB_InitGround(NULL);

    /* We should never get here as control is now taken by the scheduler */
    for( ;; )
    {
        SYS_EVENT_Process();
        DLOG_Process(NULL);
    }
}
