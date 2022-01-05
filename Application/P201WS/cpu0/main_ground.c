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
#include "p201_led_ctrl.h"
#include "board_watchdog.h"
#include "cmd_line.h"
#include "hal_uart.h"
#include "bb_reply_pc.h"
#include "app_sys_event_process.h"
#include "test_search_id.h"
#include "usr_uart1_task.h"
#include "usr_uart3_p201_task.h"
#include "usr_uart6_task.h"
#include "usr_rs422_task.h"
#include "factory.h"
#include "test_hal_i2c.h"


#define USB_CTL_GPIO_NUM    HAL_GPIO_NUM80

osSemaphoreId uart_semaphore_id;

#define GROUND_UART_MODE       1

void HDMI_powerOn(void)
{
    HAL_GPIO_OutPut(102);
    HAL_GPIO_SetPin(102, HAL_GPIO_PIN_SET);
}

void HAL_SDI_RX_PowerOn()
{
    HAL_GPIO_OutPut(HAL_GPIO_NUM63);
    HAL_GPIO_SetPin(HAL_GPIO_NUM63, HAL_GPIO_PIN_SET);
    HAL_Delay(15);
    HAL_GPIO_SetPin(HAL_GPIO_NUM63, HAL_GPIO_PIN_RESET);
    HAL_Delay(15);
    HAL_GPIO_SetPin(HAL_GPIO_NUM63, HAL_GPIO_PIN_SET);
}

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

        HAL_GPIO_OutPut(HAL_GPIO_NUM82);
        HAL_GPIO_OutPut(HAL_GPIO_NUM83);

        HAL_GPIO_SetPin(HAL_GPIO_NUM82, HAL_GPIO_PIN_RESET);
        HAL_GPIO_SetPin(HAL_GPIO_NUM83, HAL_GPIO_PIN_SET);

        DLOG_Critical("Hardware Version 3.0. \n");
    }
    else
    {
        DLOG_Critical("Hardware Version 2.0 and Before.\n");
    }
}

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
    int usb_configure = 0;
    WATCHDOG_Init();
    
    STRU_HAL_SYS_CTL_CONFIG *pst_cfg;
    HAL_SYS_CTL_GetConfig( &pst_cfg);
    pst_cfg->u8_workMode = 1;
    pst_cfg->u16_cpu2Clk = 200; //To fix the 9363 RF sensitify problem
    HAL_SYS_CTL_Init(pst_cfg);

    /* initialize the uart */
    console_init(0,115200);
    dlog_set_output_level(LOG_LEVEL_WARNING);
    DLOG_Critical("cpu0 start!!! \n");

//  USB_Gpio_Ctrl_Init();
//
    check_board_version();

    BB_ledGpioInit();    
    SYS_EVENT_RegisterHandler(SYS_EVENT_ID_BB_EVENT, BB_EventHandler);
    SYS_EVENT_RegisterHandler(SYS_EVENT_ID_BB_EVENT, BB_grdRcIdEventHandler);

    SYS_EVENT_RegisterHandler(SYS_EVENT_ID_USER_CFG_CHANGE, BB_ReplyPcHandler);
    SYS_EVENT_RegisterHandler(SYS_EVENT_ID_TO_APP, Grd_AppSysEventHandler);
//  SYS_EVENT_RegisterHandler(SYS_EVENT_ID_USER_DEFINE, USER_Define_EventHandler);

    HAL_USB_ConfigPHY();

    HAL_SDI_RX_PowerOn();
    HDMI_powerOn();

    HAL_USB_Init(HAL_USB_PORT_0, HAL_USB_DR_MODE_OTG);
    HAL_USB_Init(HAL_USB_PORT_1, HAL_USB_DR_MODE_OTG);

    HAL_SRAM_ReceiveVideoConfig();
#if 0
    STRU_cfgNode *node;
    STRU_UART_BAUDR *p_uart_baudr = NULL;
    p_uart_baudr = FCT_GetNodeAndData(FACTORY_SUBNODE_UART_BAUDR_ID, NULL);
    if (p_uart_baudr != NULL)
    {
        usb_configure = p_uart_baudr->st_uartBaudr[3] & 0x0FF;
        if (usb_configure == 0x00)
        {
            HAL_SRAM_ChannelConfig(ENUM_HAL_SRAM_CHANNEL_TYPE_VIDEO0, HAL_USB_PORT_0, 0);

            HAL_SRAM_ChannelConfig(ENUM_HAL_SRAM_CHANNEL_TYPE_VIDEO0, HAL_USB_PORT_0, 1);
        }
        else
        {
            HAL_SRAM_ChannelConfig(ENUM_HAL_SRAM_CHANNEL_TYPE_VIDEO0, HAL_USB_PORT_1, 0);
            HAL_SRAM_ChannelConfig(ENUM_HAL_SRAM_CHANNEL_TYPE_VIDEO0, HAL_USB_PORT_1, 1);
        }
    }
    else
    {
        HAL_SRAM_ChannelConfig(ENUM_HAL_SRAM_CHANNEL_TYPE_VIDEO0, HAL_USB_PORT_1, 0);
        HAL_SRAM_ChannelConfig(ENUM_HAL_SRAM_CHANNEL_TYPE_VIDEO0, HAL_USB_PORT_1, 1);
    }
#endif

    HAL_SRAM_ChannelConfig(ENUM_HAL_SRAM_CHANNEL_TYPE_VIDEO0, 0);
    HAL_SRAM_ChannelConfig(ENUM_HAL_SRAM_CHANNEL_TYPE_VIDEO0, 1);


    HAL_NV_Init();

    osSemaphoreDef(uart_semaphore);
    uart_semaphore_id = osSemaphoreCreate(osSemaphore(uart_semaphore), 1);

    osThreadDef(USBHStatus_Task, USBH_USBHostStatus, osPriorityHigh, 0, 4 * 128);
    osThreadCreate(osThread(USBHStatus_Task), NULL);

    osThreadDef(DLOG_Task, DLOG, osPriorityIdle, 0, 4 * 128);
    osThreadCreate(osThread(DLOG_Task), NULL);

    osThreadDef(IOTask, IO_Task, osPriorityNormal, 0, 16 * 128);
    osThreadCreate(osThread(IOTask), NULL);

	osThreadDef(ledTask, grd_led_Task, osPriorityIdle, 0, 4 * 128);
	osThreadCreate(osThread(ledTask), NULL);

    Usr_SBUS_Task(GROUND_UART_MODE);
    Usr_RS232_Task(GROUND_UART_MODE);
    Usr_TTL_Task(GROUND_UART_MODE);
    Usr_Rs422_Task(GROUND_UART_MODE);

    Wireless_TaskInit(WIRELESS_USE_RTOS);

    HAL_GPIO_InPut(HAL_GPIO_NUM49);

    osKernelStart();

    /* We should never get here as control is now taken by the scheduler */
    for( ;; )
    {
    }
}

/************************ (C) COPYRIGHT Artosyn *****END OF FILE****/

