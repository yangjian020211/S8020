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
#include "test_bb_led_ctrl.h"
#include "hal_uart.h"
#include "board_watchdog.h"
#include "cmd_line.h"
#include "bb_reply_pc.h"
#include "test_search_id.h"
#include "app_sys_event_process.h"
#include "usr_usb_task.h"


#ifdef USE_ADV7611_EDID_CONFIG_BIN
#pragma message("defined ADV_7611")
#endif


#ifdef USE_IT66021_EDID_CONFIG_BIN
#pragma message("defined IT_66021")
#endif
void console_init(uint32_t uart_num, uint32_t baut_rate)
{
    HAL_UART_Init(DEBUG_LOG_UART_PORT, HAL_UART_BAUDR_115200, NULL);
    DLOG_Init(CMD_exec_cmd, NULL, DLOG_SERVER_PROCESSOR);
}

void HDMI_powerOn(void)
{
    //HAL_GPIO_OutPut(102);
    //HAL_GPIO_SetPin(102, HAL_GPIO_PIN_SET);
    HAL_GPIO_OutPut(101);
    HAL_GPIO_SetPin(101, HAL_GPIO_PIN_SET);
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
    WATCHDOG_Init();

    STRU_HAL_SYS_CTL_CONFIG *pst_cfg;
    HAL_SYS_CTL_GetConfig( &pst_cfg);
    pst_cfg->u8_workMode = 0;
    pst_cfg->u16_cpu2Clk = 200; //To fix the 9363 RF sensitify problem
    HAL_SYS_CTL_Init(pst_cfg);

    /* initialize the uart */
    console_init(0,115200);
    dlog_set_output_level(LOG_LEVEL_WARNING);
    DLOG_Critical("cpu0 start!!! \n");

    BB_ledGpioInit();
    SYS_EVENT_RegisterHandler(SYS_EVENT_ID_BB_EVENT, BB_EventHandler);
    SYS_EVENT_RegisterHandler(SYS_EVENT_ID_BB_EVENT, BB_skyRcIdEventHandler);

    SYS_EVENT_RegisterHandler(SYS_EVENT_ID_USER_CFG_CHANGE, BB_ReplyPcHandler);
    SYS_EVENT_RegisterHandler(SYS_EVENT_ID_TO_APP, Sky_AppSysEventHandler);

    HAL_USB_ConfigPHY();

    HDMI_powerOn();
    
    STRU_HDMI_CONFIGURE        st_configure;
    st_configure.e_getFormatMethod = HAL_HDMI_POLLING;
    //st_configure.e_colorDepth = HAL_HDMI_RX_16BIT;
	st_configure.e_colorDepth = HAL_HDMI_RX_8BIT;
    st_configure.st_interruptGpio.e_interruptGpioNum = HAL_GPIO_NUM98;
    st_configure.st_interruptGpio.e_interruptGpioPolarity = HAL_GPIO_ACTIVE_LOW;
    st_configure.st_interruptGpio.e_interruptGpioType = HAL_GPIO_LEVEL_SENUMSITIVE;
    //st_configure.u8_hdmiToEncoderCh = 1;
    st_configure.u8_hdmiToEncoderCh = 0;
    HAL_HDMI_RX_Init(HAL_HDMI_RX_1, &st_configure);

    HAL_USB_Init(HAL_USB_PORT_0, HAL_USB_DR_MODE_DEVICE);
    HAL_USB_Init(HAL_USB_PORT_1, HAL_USB_DR_MODE_DEVICE);

    HAL_NV_Init();
	
	UPGRADE_SKYInit();

    portDISABLE_INTERRUPTS();

    osThreadDef(USBHStatus_Task, USBH_USBHostStatus, osPriorityHigh, 0, 4 * 128);
    osThreadCreate(osThread(USBHStatus_Task), NULL);

    osThreadDef(DLOG_Task, DLOG, osPriorityIdle, 0, 4 * 128);
    osThreadCreate(osThread(DLOG_Task), NULL);

    osThreadDef(IOTask, IO_Task, osPriorityNormal, 0, 16 * 128);
    osThreadCreate(osThread(IOTask), NULL);

    Wireless_TaskInit(WIRELESS_USE_RTOS);

    usr_usb0_interface();

    portENABLE_INTERRUPTS();

    osKernelStart();

    /* We should never get here as control is now taken by the scheduler */
    for( ;; )
    {
    }
} 

/************************ (C) COPYRIGHT Artosyn *****END OF FILE****/
