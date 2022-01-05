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
#include "p201_led_ctrl.h"
#include "hal_uart.h"
#include "board_watchdog.h"
#include "cmd_line.h"
#include "bb_reply_pc.h"
#include "test_search_id.h"
#include "test_hdmi.h"
#include "app_sys_event_process.h"

#include "usr_uart1_task.h"
#include "usr_uart3_p201_task.h"
#include "usr_uart6_task.h"
#include "usr_rs422_task.h"
#include "usr_usb_p201_task.h"
#include "usr_gs2971.h"
#include "test_hal_i2c.h"


#ifdef USE_ADV7611_EDID_CONFIG_BIN
#pragma message("defined ADV_7611")
#endif


#ifdef USE_IT66021_EDID_CONFIG_BIN
#pragma message("defined IT_66021")
#endif

#define SKY_UART_MODE       0

osSemaphoreId uart_semaphore_id;

static void check_board_version(void)
{
    uint32_t u32_Gpio87_val = 0;
    uint32_t u32_Gpio88_val = 0;
    uint32_t u32_Gpio89_val = 0;

    HAL_GPIO_InPut(HAL_GPIO_NUM87);
    HAL_GPIO_InPut(HAL_GPIO_NUM88);
    HAL_GPIO_InPut(HAL_GPIO_NUM89);

    HAL_GPIO_GetPin(HAL_GPIO_NUM87, &u32_Gpio87_val);
    HAL_GPIO_GetPin(HAL_GPIO_NUM88, &u32_Gpio88_val);
    HAL_GPIO_GetPin(HAL_GPIO_NUM89, &u32_Gpio89_val);

    DLOG_Critical("PWM 7: %d, PWM 8: %d, PWM 9: %d \n", u32_Gpio87_val, u32_Gpio88_val, u32_Gpio89_val);

    if (u32_Gpio87_val == 0 && u32_Gpio88_val == 1 && u32_Gpio89_val == 1)  //Board V3
    {
        command_TestHalI2cInit("2", "0x30", "1");
        command_TestHalI2cWrite("2", "0x22", "1", "0x3C", "1");
        command_TestHalI2cWrite("2", "0x10", "1", "0xAF", "1");

        HAL_GPIO_OutPut(HAL_GPIO_NUM90);
        HAL_GPIO_SetPin(HAL_GPIO_NUM90, HAL_GPIO_PIN_SET);

        DLOG_Critical("Hardware Version 3.0.\n");
    }
    else
    {
        DLOG_Critical("Hardware Version 2.0 and Before.\n");
    }
}

void console_init(uint32_t uart_num, uint32_t baut_rate)
{
    HAL_UART_Init(DEBUG_LOG_UART_PORT, HAL_UART_BAUDR_115200, NULL);
    DLOG_Init(CMD_exec_cmd, NULL, DLOG_SERVER_PROCESSOR);
}

void HDMI_powerOn(void)
{
    HAL_GPIO_OutPut(102);
    HAL_GPIO_SetPin(102, HAL_GPIO_PIN_SET);
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

    check_board_version();

    BB_ledGpioInit();
    SYS_EVENT_RegisterHandler(SYS_EVENT_ID_BB_EVENT, BB_EventHandler);
    SYS_EVENT_RegisterHandler(SYS_EVENT_ID_BB_EVENT, BB_skyRcIdEventHandler);

    SYS_EVENT_RegisterHandler(SYS_EVENT_ID_USER_CFG_CHANGE, BB_ReplyPcHandler);
    SYS_EVENT_RegisterHandler(SYS_EVENT_ID_TO_APP, Sky_AppSysEventHandler);

    HAL_USB_ConfigPHY();

    HDMI_powerOn();
    
    STRU_HDMI_CONFIGURE        st_configure;
    st_configure.e_getFormatMethod = HAL_HDMI_POLLING;
    st_configure.e_colorDepth = HAL_HDMI_RX_16BIT;
    st_configure.st_interruptGpio.e_interruptGpioNum = HAL_GPIO_NUM98;
    st_configure.st_interruptGpio.e_interruptGpioPolarity = HAL_GPIO_ACTIVE_LOW;
    st_configure.st_interruptGpio.e_interruptGpioType = HAL_GPIO_LEVEL_SENUMSITIVE;
    st_configure.u8_hdmiToEncoderCh = 1;
    HAL_HDMI_RX_Init(HAL_HDMI_RX_1, &st_configure);
	HAL_HDMI_RX_VideoCallbak(HDMI_RX_VideoCallbakSample);

    HAL_USB_Init(HAL_USB_PORT_0, HAL_USB_DR_MODE_OTG);
    HAL_USB_Init(HAL_USB_PORT_1, HAL_USB_DR_MODE_OTG);

    HAL_NV_Init();

    portDISABLE_INTERRUPTS();

    osSemaphoreDef(uart_semaphore);
    uart_semaphore_id = osSemaphoreCreate(osSemaphore(uart_semaphore), 1);

    osThreadDef(USBHStatus_Task, USBH_USBHostStatus, osPriorityHigh, 0, 4 * 128);
    osThreadCreate(osThread(USBHStatus_Task), NULL);

    osThreadDef(DLOG_Task, DLOG, osPriorityIdle, 0, 4 * 128);
    osThreadCreate(osThread(DLOG_Task), NULL);

    osThreadDef(IOTask, IO_Task, osPriorityNormal, 0, 16 * 128);
    osThreadCreate(osThread(IOTask), NULL);

    osThreadDef(ledTask, sky_led_Task, osPriorityNormal, 0, 16 * 128);
    osThreadCreate(osThread(ledTask), NULL);

    usr_p201_usb_interface();

    Usr_SBUS_Task(SKY_UART_MODE);
    Usr_RS232_Task(SKY_UART_MODE);
    Usr_TTL_Task(SKY_UART_MODE);
    Usr_Rs422_Task(SKY_UART_MODE);

    Wireless_TaskInit(WIRELESS_USE_RTOS);

    portENABLE_INTERRUPTS();

    //set GPIO_49 input for LNA
    HAL_GPIO_InPut(HAL_GPIO_NUM49);

    HAL_SDI_RX_Init();

    osKernelStart();

    /* We should never get here as control is now taken by the scheduler */
    for( ;; )
    {
    }
} 

/************************ (C) COPYRIGHT Artosyn *****END OF FILE****/
