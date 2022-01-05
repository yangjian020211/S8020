#include "debuglog.h"
#include "command.h"
#include "sys_event.h"
#include "hal.h"
#include "hal_sys_ctl.h"
#include "hal_uart.h"
#include "cmd_line.h"
#include "test_hal_uart.h"
#include "../jnc_hal_sys/jnc_hal_sys.h"

void console_init(uint32_t uart_num, uint32_t baut_rate)
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
    //uint8_t   log_data[512];
    HAL_SYS_CTL_Init(NULL);

    /* initialize the uart */
    console_init(0,115200);
    dlog_set_output_level(LOG_LEVEL_WARNING);
    DLOG_Critical("cpu1 start!!! \n");

    ss_hal_datalog_init();

    /* We should never get here as control is now taken by the scheduler */
    for( ;; )
    {
        /* write log if needed */
        //ss_hal_datalog_write((void *)log_data, 512);
        ss_hal_idle(500);
        
        DLOG_Process(NULL);
    }
} 


