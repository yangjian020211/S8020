#include "debuglog.h"
#include "command.h"
#include "sys_event.h"
#include "hal.h"
#include "hal_sys_ctl.h"
#include "hal_uart.h"
#include "board_watchdog.h"
#include "cmd_line.h"
#include "hal_softi2s.h"

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

    /* initialize the uart */
    CONSOLE_Init();
    dlog_set_output_level(LOG_LEVEL_WARNING);
    DLOG_Critical("cpu1 start!!! \n");
    STRU_HAL_SOFTI2S_INIT st_audioConfig = {AUDIO_DATA_START,HAL_GPIO_NUM35,HAL_GPIO_NUM70,HAL_GPIO_NUM20};
    HAL_SOFTI2S_Init(&st_audioConfig);

    /* We should never get here as control is now taken by the scheduler */
    for( ;; )
    {
        HAL_SOFTI2S_Funct();
        SYS_EVENT_Process();
        //DLOG_Process(NULL);
    }
} 

