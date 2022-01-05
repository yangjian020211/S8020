#include "debuglog.h"
#include "serial.h"
#include "command.h"
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
#include "hal_nv.h"
#include "hal_dma.h"
#include "com_task.h"
#include "uvc_task.h"
#include "test_bb_led_ctrl.h"
#include <stdlib.h>
#include "cmd_line.h"
#include "../jnc_hal_sys/jnc_hal_sys_cpu0.h"
#include "test_hal_uart.h"
#include "hal_uart.h"
#include "test_search_id.h"
#include "app_sys_event_process.h"



#define AMBA_POWER_GPIO     HAL_GPIO_NUM88

void amba_init(void)
{
    HAL_GPIO_OutPut(AMBA_POWER_GPIO);

    HAL_Delay(2);

    HAL_GPIO_SetPin(AMBA_POWER_GPIO, HAL_GPIO_PIN_SET);
}


void amba_turn_on_off(void)
{
    HAL_GPIO_SetPin(AMBA_POWER_GPIO, HAL_GPIO_PIN_RESET);

    HAL_Delay(30);

    HAL_GPIO_SetPin(AMBA_POWER_GPIO, HAL_GPIO_PIN_SET);
}


void CONSOLE_Init(void)
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


void HDMI_powerOn(void)
{
    HAL_GPIO_OutPut(HAL_GPIO_NUM97);
    HAL_GPIO_SetPin(HAL_GPIO_NUM97, HAL_GPIO_PIN_SET);
}

static void IO_Task(void const *argument)
{
    /* amba open */
    amba_turn_on_off();

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
    STRU_HAL_SYS_CTL_CONFIG *pst_cfg;
    HAL_SYS_CTL_GetConfig( &pst_cfg);
    pst_cfg->u8_workMode = 0;
    HAL_SYS_CTL_Init(pst_cfg);

    /* initialize the uart */
    CONSOLE_Init();
    dlog_set_output_level(LOG_LEVEL_WARNING);
    DLOG_Critical("cpu0 start!!! \n");

    BB_ledGpioInit();
    SYS_EVENT_RegisterHandler(SYS_EVENT_ID_BB_EVENT, BB_EventHandler);
    SYS_EVENT_RegisterHandler(SYS_EVENT_ID_BB_EVENT, BB_skyRcIdEventHandler);
    //SYS_EVENT_RegisterHandler(SYS_EVENT_ID_IDLE, HAL_NV_TaskWriteData);
    SYS_EVENT_RegisterHandler(SYS_EVENT_ID_TO_APP, Sky_AppSysEventHandler);

    HAL_USB_ConfigPHY();

    HDMI_powerOn();
    
    STRU_HDMI_CONFIGURE        st_configure;
    st_configure.e_colorDepth = HAL_HDMI_RX_16BIT;
    st_configure.e_getFormatMethod = HAL_HDMI_POLLING;
    st_configure.st_interruptGpio.e_interruptGpioNum = HAL_GPIO_NUM63;
    st_configure.st_interruptGpio.e_interruptGpioPolarity = HAL_GPIO_ACTIVE_LOW;
    st_configure.st_interruptGpio.e_interruptGpioType = HAL_GPIO_LEVEL_SENUMSITIVE;
    st_configure.u8_hdmiToEncoderCh = 0;
    HAL_HDMI_RX_Init(HAL_HDMI_RX_1, &st_configure);

    HAL_USB_Init(HAL_USB_PORT_0, HAL_USB_DR_MODE_HOST);

    HAL_USB_Init(HAL_USB_PORT_1, HAL_USB_DR_MODE_HOST);

    HAL_NV_Init();

    USBH_UVCInit(320, 240, ENUM_UVC_DATA_YUV);

    UPGRADE_SKYInit();

    /* amba init */
    amba_init();

    /* init log save function */
    ss_log_write_init();

    portDISABLE_INTERRUPTS();

    osThreadDef(USBHStatus_Task, USBH_USBHostStatus, osPriorityHigh, 0, 4 * 128);
    osThreadCreate(osThread(USBHStatus_Task), NULL);

    osThreadDef(DLOG_Task, DLOG, osPriorityIdle, 0, 4 * 128);
    osThreadCreate(osThread(DLOG_Task), NULL);

    osThreadDef(IOTask, IO_Task, osPriorityNormal, 0, 16 * 128);
    osThreadCreate(osThread(IOTask), NULL);

    Wireless_TaskInit(WIRELESS_USE_RTOS);

    //COMTASK_Init();

    portENABLE_INTERRUPTS();

    osKernelStart();

    /* We should never get here as control is now taken by the scheduler */
    for( ;; )
    {
    }
} 

/************************ (C) COPYRIGHT Artosyn *****END OF FILE****/
