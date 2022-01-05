#include "debuglog.h"
#include "test_i2c_adv7611.h"
#include "command.h"
#include "serial.h"
#include "hal_sram_ground.h"
#include "cmsis_os.h"
#include "sys_event.h"
#include "upgrade.h"
#include "hal.h"
#include "hal_bb.h"
#include "test_usbh.h"
#include "hal_usb_otg.h"
#include "hal_sys_ctl.h"
#include "wireless_interface.h"
#include "hal_nv.h"
#include "test_bb_led_ctrl.h"
#include "board_watchdog.h"
#include "cmd_line.h"
#include "hal_uart.h"
#include "bb_reply_pc.h"
#include "app_sys_event_process.h"
#include "test_search_id.h"
#include "hal_i2c.h"
#include "hal_mfi.h"
#include "hal_gpio.h"
#include "usr_usb_task.h"
#include "usr_uart3_task.h"

#define USB_CTL_GPIO_NUM    HAL_GPIO_NUM80

static void USB_Gpio_Ctrl_Init(void)
{
    HAL_GPIO_OutPut(USB_CTL_GPIO_NUM);
    HAL_GPIO_SetPin(USB_CTL_GPIO_NUM, HAL_GPIO_PIN_RESET);
}

static void USER_Define_EventHandler(void* p)
{
    HAL_Delay(70);
    HAL_GPIO_SetPin(USB_CTL_GPIO_NUM, HAL_GPIO_PIN_SET);
    HAL_Delay(3);
    HAL_GPIO_SetPin(USB_CTL_GPIO_NUM, HAL_GPIO_PIN_RESET);
}

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
        HAL_SRAM_CheckChannelTimeout();
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
    pst_cfg->u8_workMode = 1;
    pst_cfg->u16_cpu2Clk = 200; //To fix the 9363 RF sensitify problem
    HAL_SYS_CTL_Init(pst_cfg);
   
    HAL_NV_Init();

    /* initialize the uart */
    console_init(0,115200);
    dlog_set_output_level(LOG_LEVEL_WARNING);
    DLOG_Critical("cpu0 start!!! \n");
//  USB_Gpio_Ctrl_Init();

    BB_ledGpioInit();    
    SYS_EVENT_RegisterHandler(SYS_EVENT_ID_BB_EVENT, BB_EventHandler);
    SYS_EVENT_RegisterHandler(SYS_EVENT_ID_BB_EVENT, BB_grdRcIdEventHandler);

    SYS_EVENT_RegisterHandler(SYS_EVENT_ID_USER_CFG_CHANGE, BB_ReplyPcHandler);
    SYS_EVENT_RegisterHandler(SYS_EVENT_ID_TO_APP, Grd_AppSysEventHandler);
//  SYS_EVENT_RegisterHandler(SYS_EVENT_ID_USER_DEFINE, USER_Define_EventHandler);

    HAL_USB_ConfigPHY();

    

    HAL_USB_Init(HAL_USB_PORT_0, HAL_USB_DR_MODE_DEVICE);
    HAL_USB_Init(HAL_USB_PORT_1, HAL_USB_DR_MODE_DEVICE);

    HAL_SRAM_ReceiveVideoConfig();

    HAL_SRAM_ChannelConfig(ENUM_HAL_SRAM_CHANNEL_TYPE_VIDEO0,
                           0);

    HAL_SRAM_ChannelConfig(ENUM_HAL_SRAM_CHANNEL_TYPE_VIDEO0,
                           1);
    
    
	
    UPGRADE_GNDInit();

    /* MFI CONFIGURE:
     * 1.RST GPIO number
     * 2.Android/iOS host recognize GPIO number
     * 3.MFI I2C Component */
    MFI_Init(HAL_GPIO_NUM90, 0xFF, HAL_I2C_COMPONENT_2);

    osThreadDef(USBHStatus_Task, USBH_USBHostStatus, osPriorityHigh, 0, 4 * 128);
    osThreadCreate(osThread(USBHStatus_Task), NULL);

    osThreadDef(DLOG_Task, DLOG, osPriorityIdle, 0, 4 * 128);
    osThreadCreate(osThread(DLOG_Task), NULL);

    osThreadDef(IOTask, IO_Task, osPriorityNormal, 0, 16 * 128);
    osThreadCreate(osThread(IOTask), NULL);

    usr_usb0_interface();

    usr_bypass_uart_task(pst_cfg->u8_workMode,HAL_UART_COMPONENT_1);

    Wireless_TaskInit(WIRELESS_USE_RTOS);

    osKernelStart();

    /* We should never get here as control is now taken by the scheduler */
    for( ;; )
    {
    }
}

/************************ (C) COPYRIGHT Artosyn *****END OF FILE****/

