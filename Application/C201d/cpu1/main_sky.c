#include "debuglog.h"
#include "sys_event.h"
#include "hal.h"
#include "hal_sys_ctl.h"
#include "hal_uart.h"
#include "cmd_line.h"
#include "board_watchdog.h"
#include "usr_sbus_uart_task.h"
#include "cmsis_os.h"
#include "usr_uart3_task.h"

void CONSOLE_Init(void)
{
    DLOG_Init(CMD_exec_cmd, NULL, DLOG_CLIENT_PROCESSOR);
}
void DLOG_Process(void* p);

void DLOG(void const *argument)
{
    while (1)
    {
        DLOG_Process(NULL);

        HAL_Delay(10);
    }
}
static void IO_Task(void const *argument)
{    
    while (1)    
    {            
        SYS_EVENT_Process();    
    }
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
#if 0
    portDISABLE_INTERRUPTS();

    osThreadDef(DLOG_Task, DLOG, osPriorityIdle, 0, 4 * 128);
    osThreadCreate(osThread(DLOG_Task), NULL);

    osThreadDef(IOTask, IO_Task, osPriorityNormal, 0, 16 * 128);
    osThreadCreate(osThread(IOTask),NULL);

    usr_bypass_uart_task(0);
    
    usr_bypass_sbus_uart_task(0);

    portENABLE_INTERRUPTS();
    
    osKernelStart();
    /* We should never get here as control is now taken by the scheduler */
    for( ;; )
    {
        //SYS_EVENT_Process();
        //DLOG_Process(NULL);
    }
#else
	#ifndef NET_REPEATER
    	usr_bypass_sbus_uart_task(0);
	#endif
	
    for( ;; )
    {
        SYS_EVENT_Process();
        DLOG_Process(NULL);
    }

#endif
} 

