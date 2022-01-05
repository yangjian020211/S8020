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
#include "test_bb_led_ctrl_2.h"
#include "test_hdmi.h"
#include "uart_task.h"
#include "cmd_line.h"
#include "uvc_task.h"
#include <stdlib.h>
#include "hal_dma.h"
#include "hal_sd.h"
#include "hal_uart.h"
#include "bb_reply_pc.h"
#include "test_search_id_2.h"
#include "app_sys_event_process.h"
#include "usr_usb_task.h"
#include "usr_uart3_task.h"
#include "board_watchdog.h"
#include "hal_norflash.h"
#include "memory_config.h"
#include "c201d_pt.h"
#include "sleep_mode.h"
#include "factory.h"
#include "usr_sbus_uart_task.h"
#include "usr_cmd_uart_task.h"
#include "test_hdmi_ch7038.h"
#include "hal_i2c.h"
#include "hal_mfi.h"
#include "hal_norflash.h"
#include "usr_protocol.h"
#include "XC7027/xc7027.h"

//#define SENSOR_XC7027

//#define USE_HDMI_CH7038

void BB_sky_uartDataHandler(void *p);
void console_init(uint32_t uart_num, uint32_t baut_rate)
{
    HAL_UART_Init(DEBUG_LOG_UART_PORT, HAL_UART_BAUDR_115200, NULL);
    DLOG_Init(CMD_exec_cmd, NULL, DLOG_SERVER_PROCESSOR);
}

void HDMI_powerOn(void)
{
    HAL_GPIO_OutPut(63);
    HAL_GPIO_SetPin(63, HAL_GPIO_PIN_SET);
}
static void fem_init(void)
{
    STRU_MIMO_MODE *pst_mimo_mode = NULL;
    pst_mimo_mode = (STRU_MIMO_MODE *)FCT_GetNodeAndData(FACTORY_SUBNODE_MIMO_MODE_ID,NULL);
    if(pst_mimo_mode != NULL)
    {
        if(pst_mimo_mode->st_skyMimoMode == MIMO_1T1R)
        {
            FemOff(1);
            DLOG_Warning("1T1R,FEM 1 OFF");
        }

    }
    else
    {
        DLOG_Warning("get mimo null");
    }

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

static void Sensor_Task(void const *argument)
{
    XC7027_Sensor_Start();
    DLOG_Critical("Start XC7027 Sensor.");
}

static void Sensor_Task_Loop(void const *argument)
{
    XC7027_SENSOR_LoopCallBack();
}

//sky or grd reverse, but usb1 host or device attribute not change
static void USB1_init(void)
{
    STRU_WIRELESS_RESET_DATA_RSV *p;

    p = (STRU_WIRELESS_RESET_DATA_RSV *)(SRAM_USR_GRD_SKY_SELECT_ST_ADDR);
    if(p->magic_header == 0x1234abcd)
    {
        if(p->sky_or_grd == 0)
        {
            DLOG_Warning("sram force to sky mode, USB1 HOST");
            HAL_USB_Init(HAL_USB_PORT_1, HAL_USB_DR_MODE_HOST);
            MFI_Init(HAL_GPIO_NUM109, 0xFF, HAL_I2C_COMPONENT_0);
        }
        else if(p->sky_or_grd == 1)
        {
            HAL_USB_Init(HAL_USB_PORT_1, HAL_USB_DR_MODE_DEVICE);
            DLOG_Warning("sram force to gnd mode, USB1 DEVICE");
        }
        else
        {
            HAL_USB_Init(HAL_USB_PORT_1, HAL_USB_DR_MODE_DEVICE);
        }
    }
    else
    {
        HAL_USB_Init(HAL_USB_PORT_1, HAL_USB_DR_MODE_DEVICE);
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
    int testmode = 0;
    HAL_SYS_CTL_GetConfig( &pst_cfg);
    pst_cfg->u8_workMode = 0;
    HAL_SYS_CTL_Init(pst_cfg);
    WATCHDOG_Init();

    /* initialize the uart */
    console_init(0,115200);
    dlog_set_output_level(LOG_LEVEL_WARNING);
    DLOG_Critical("cpu0 start!!! \n");

	DLOG_Critical("app 3.5\n");
    testmode = is_test_mode();
    BB_ledGpioInit();
    SYS_EVENT_RegisterHandler(SYS_EVENT_ID_BB_EVENT, BB_EventHandler);
    SYS_EVENT_RegisterHandler(SYS_EVENT_ID_BB_EVENT, BB_skyRcIdEventHandler);
    SYS_EVENT_RegisterHandler(SYS_EVENT_ID_USER_CFG_CHANGE, BB_ReplyPcHandler);
    SYS_EVENT_RegisterHandler(SYS_EVENT_ID_TO_APP, Sky_AppSysEventHandler);
//    SYS_EVENT_RegisterHandler(SYS_EVENT_ID_H264_RPT_EVENT, ENCODE_EventHandler);
//  SYS_EVENT_RegisterHandler(SYS_EVENT_ID_IDLE, (SYS_Event_Handler)WATCHDOG_Reset);

    HAL_NV_Init();

    //HAL_SD_Init();

    HAL_USB_ConfigPHY();

    //HDMI_powerOn();

//    HAL_HDMI_RX_VideoCallbak(HDMI_RX_VideoCallbakSample);

#ifndef USE_HDMI_CH7038
#ifndef SENSOR_XC7027
    STRU_HDMI_CONFIGURE        st_configure;
    st_configure.e_getFormatMethod = HAL_HDMI_INTERRUPT;
    st_configure.e_colorDepth = HAL_HDMI_RX_8BIT;

    st_configure.st_interruptGpio.e_interruptGpioNum = HAL_GPIO_NUM66;
    st_configure.st_interruptGpio.e_interruptGpioPolarity = HAL_GPIO_ACTIVE_LOW;
    st_configure.st_interruptGpio.e_interruptGpioType = HAL_GPIO_LEVEL_SENUMSITIVE;
    st_configure.u8_hdmiToEncoderCh = 1;
    if(!testmode)//patch,auto-test,can not boot
    {
        HAL_HDMI_RX_Init(HAL_HDMI_RX_1, &st_configure);
        HAL_HDMI_RX_VideoCallbak(HDMI_RX_VideoCallbakSample);
    }
#endif
#endif

    HAL_USB_Init(HAL_USB_PORT_0, HAL_USB_DR_MODE_DEVICE);

    USB1_init();

    UPGRADE_SKYInit();
    fem_init();

    portDISABLE_INTERRUPTS();

    osThreadDef(USBHStatus_Task, USBH_USBHostStatus, osPriorityHigh, 0, 4 * 128);
    osThreadCreate(osThread(USBHStatus_Task), NULL);

    osThreadDef(DLOG_Task, DLOG, osPriorityIdle, 0, 4 * 128);
    osThreadCreate(osThread(DLOG_Task), NULL);

    osThreadDef(IOTask, IO_Task, osPriorityNormal, 0, 16 * 128);
    osThreadCreate(osThread(IOTask), NULL);

    if(!testmode)
    {
        osThreadDef(ledTask, sky_led_Task, osPriorityNormal, 0, 16 * 128);
        osThreadCreate(osThread(ledTask), NULL);
    }

    osThreadDef(USBSend, usb_send_packet, osPriorityNormal, 0, 4 * 128);
    osThreadCreate(osThread(USBSend), NULL);
    
    //osThreadDef(ModPinSleepTask, Mod_Pin_SleepTask, osPriorityIdle, 0, 4 * 128);
    //osThreadCreate(osThread(ModPinSleepTask), NULL);

    osThreadDef(ModSkyPinSearchIdTask, Mod_Sky_Pin_SearchIdTask, osPriorityNormal, 0, 4 * 128);
    osThreadCreate(osThread(ModSkyPinSearchIdTask), NULL);

    usr_usb0_interface();

    c201d_pt(testmode);

    usr_bypass_uart_task(pst_cfg->u8_workMode,HAL_UART_COMPONENT_5);

    //usr_bypass_sbus_uart_task(pst_cfg->u8_workMode);

    usr_cmd_uart_task(pst_cfg->u8_workMode);


#ifdef USE_HDMI_CH7038
    test_hdmi_ch7038_init();
#endif

    Wireless_TaskInit(WIRELESS_USE_RTOS);

    portENABLE_INTERRUPTS();
    
#ifdef SENSOR_XC7027
    osTimerDef(SensorTask, Sensor_Task);
    osTimerId SensorTimer = osTimerCreate(osTimer(SensorTask), osTimerOnce, NULL);
    osTimerStart(SensorTimer, 100);

    osTimerDef(SensorLoopTask, Sensor_Task_Loop);
    osTimerId SensorLoopTimer = osTimerCreate(osTimer(SensorLoopTask), osTimerPeriodic, NULL);
    osTimerStart(SensorLoopTimer, 300);
#endif
//  XC7027_Sensor_Start();

    osKernelStart();

    /* We should never get here as control is now taken by the scheduler */
    for( ;; )
    {
    }
}

/************************ (C) COPYRIGHT Artosyn *****END OF FILE****/

