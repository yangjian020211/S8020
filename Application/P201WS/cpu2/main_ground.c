#include "serial.h"
#include "debuglog.h"
#include "test_bb.h"
#include "command.h"
#include "sys_event.h"
#include "hal_sys_ctl.h"
#include "hal_bb.h"
#include "hal.h"
#include "hal_gpio.h"
#include "bb_customerctx.h"
#include "test_bb.h"
#include "cmd_line.h"
#include "board_watchdog.h"

void console_init(uint32_t uart_num, uint32_t baut_rate)
{
    DLOG_Init(CMD_exec_cmd, NULL ,DLOG_CLIENT_PROCESSOR);
}


static void xlna_open(void)
{
       HAL_GPIO_InPut(HAL_GPIO_NUM92);
       HAL_GPIO_InPut(HAL_GPIO_NUM96);
       HAL_GPIO_InPut(HAL_GPIO_NUM100);
}

static void xlna_bypass(void)
{
       HAL_GPIO_OutPut(HAL_GPIO_NUM100);
       HAL_GPIO_SetPin(HAL_GPIO_NUM100,HAL_GPIO_PIN_RESET);
       HAL_GPIO_InPut(HAL_GPIO_NUM92);
       HAL_GPIO_InPut(HAL_GPIO_NUM96);
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

    WATCHDOG_InitCpu2();
    console_init(2, 115200);
    dlog_set_output_level(LOG_LEVEL_WARNING);
    DLOG_Critical("main ground function start \n");

//  HAL_BB_RegisterLnaFuntion(xlna_open,xlna_bypass);

    HAL_BB_InitGround(NULL);

    /* We should never get here as control is now taken by the scheduler */
    for( ;; )
    {
        SYS_EVENT_Process();
        DLOG_Process(NULL);
    }
}
