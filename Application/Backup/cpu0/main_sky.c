#include "debuglog.h"
#include "serial.h"
#include "test_usbh.h"
#include "cmsis_os.h"
#include "sys_event.h"
#include "bb_spi.h"
#include "hal.h"
#include "hal_gpio.h"
#include "hal_bb.h"
#include "hal_hdmi_rx.h"
#include "hal_usb_otg.h"
#include "hal_sys_ctl.h"
#include "wireless_interface.h"
#include "upgrade.h"
#include "hal_nv.h"
#include "test_bb_led_ctrl.h"
#include "uart_task.h"
#include "cmd_line.h"
#include "uvc_task.h"
#include <stdlib.h>
#include "hal_uart.h"
#include "bb_reply_pc.h"
#include "test_search_id.h"
#include "app_sys_event_process.h"

#include "board_watchdog.h"

void console_init(uint32_t uart_num, uint32_t baut_rate)
{
    HAL_UART_Init(DEBUG_LOG_UART_PORT, HAL_UART_BAUDR_115200, NULL);
    DLOG_Init(CMD_exec_cmd, NULL, DLOG_SERVER_PROCESSOR);
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

/*void lna_open(void)
{
    HAL_GPIO_InPut(HAL_GPIO_NUM46);
    HAL_GPIO_InPut(HAL_GPIO_NUM47);
    HAL_GPIO_InPut(HAL_GPIO_NUM48);
    HAL_GPIO_InPut(HAL_GPIO_NUM49);
    DLOG_Warning("SPI_SS/SCK/RXD6/TXD6 INPUT \n");
}*/


/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
int main(void)
{
    STRU_HAL_SYS_CTL_CONFIG *pst_cfg;

    HAL_SYS_CTL_GetConfig( &pst_cfg);
    pst_cfg->u8_workMode = 0;
    HAL_SYS_CTL_Init(pst_cfg);
    WATCHDOG_Init();

    /* initialize the uart */
    console_init(0,115200);
    dlog_set_output_level(LOG_LEVEL_WARNING);
    DLOG_Critical("cpu0 start!!! \n");

    //lna_open();

    HAL_NV_Init();

    HAL_USB_ConfigPHY();

    HAL_USB_Init(HAL_USB_PORT_0, HAL_USB_DR_MODE_OTG);

    HAL_USB_Init(HAL_USB_PORT_1, HAL_USB_DR_MODE_OTG);

    portDISABLE_INTERRUPTS();

    osThreadDef(USBHStatus_Task, USBH_USBHostStatus, osPriorityHigh, 0, 4 * 128);
    osThreadCreate(osThread(USBHStatus_Task), NULL);

    osThreadDef(DLOG_Task, DLOG, osPriorityIdle, 0, 4 * 128);
    osThreadCreate(osThread(DLOG_Task), NULL);

    osThreadDef(IOTask, IO_Task, osPriorityNormal, 0, 16 * 128);
    osThreadCreate(osThread(IOTask), NULL);

    Wireless_TaskInit(WIRELESS_USE_RTOS);

    portENABLE_INTERRUPTS();

    osKernelStart();

    /* We should never get here as control is now taken by the scheduler */
    for( ;; )
    {
    }
}

/************************ (C) COPYRIGHT Artosyn *****END OF FILE****/

