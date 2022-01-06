#include "debuglog.h"
#include "test_i2c_adv7611.h"
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
#include "test_bb_led_ctrl_2.h"
#include "upgrade.h"
#include "hal_uart.h"
#include "cmd_line.h"
#include "hal_sd.h"
#include "uart_task.h"
#include "bb_reply_pc.h"
#include "usr_usb_task.h"
#include "usr_uart3_task.h"
#include "board_watchdog.h"
#include "app_sys_event_process.h"
#include "test_search_id_2.h"
#include "hal_i2c.h"
#include "hal_mfi.h"
#include "hal_gpio.h"
#include "c201d_pt.h"
#include "sleep_mode.h"
#include "factory.h"
#include "usr_sbus_uart_task.h"
#include "usr_cmd_uart_task.h"
#include "hal_norflash.h"
#include "usr_protocol.h"
#include "test_nv_grd_slave.h"
#include "hal_timer.h"
#include "hal_nvic.h"
#include "systicks.h"
#include "ar8020.h"
#include "interrupt.h"


//#define ENABLE_NV_GRD_SLAVE

#define USB_CTL_GPIO_NUM    HAL_GPIO_NUM18

void console_init(uint32_t uart_num, uint32_t baut_rate)
{
    HAL_UART_Init(DEBUG_LOG_UART_PORT, HAL_UART_BAUDR_115200, NULL);
    DLOG_Init(CMD_exec_cmd, NULL, DLOG_SERVER_PROCESSOR);
}

void DLOG_Process(void* p);


#define RESET_TIMER        	(HAL_TIMER_NUM17)
#define RESET_TIMER_INTR	(TIMER_INTR17_VECTOR_NUM)
#define DELAY_RESET	(20000000)
static void DoSysReset(){
	HAL_TIMER_Stop(RESET_TIMER_INTR);
	DLOG_Critical("do sysreset-------\n");
	while(1);
}

static void SysResetInit(){
	DLOG_Critical("begin to register -------\n");
	if (HAL_OK != HAL_TIMER_RegisterTimer(RESET_TIMER, DELAY_RESET, DoSysReset)){
	    DLOG_Error("--->failed register reset timer");
	    return;
	}
	NVIC_SetPriority(RESET_TIMER_INTR,NVIC_EncodePriority(NVIC_PRIORITYGROUP_5,INTR_NVIC_PRIORITY_UART_DEFAULT,0));
	DLOG_Critical("register ok-------\n");
}
static void Sysreset()
{
	DLOG_Critical("begin to reset -------\n");
	int ret = HAL_TIMER_Start(RESET_TIMER_INTR);
	if(ret != HAL_OK){
	    DLOG_Error("start reset timer failed");
	}
}

void DLOG(void const *argument)
{
    while (1)
    {
        DLOG_Process(NULL);

        HAL_Delay(10);
    }
}

static void USB_OTG_Role_Identify(void* p)
{
    ENUM_USB_OTG_IDTENTITY usb_id = *(uint32_t *)p;
    DLOG_Critical("This usb id is: %d ", usb_id);
}

static void fem_init(void)
{
    STRU_MIMO_MODE *pst_mimo_mode = NULL;
    pst_mimo_mode = (STRU_MIMO_MODE *)FCT_GetNodeAndData(FACTORY_SUBNODE_MIMO_MODE_ID,NULL);
    if(pst_mimo_mode != NULL)
    {
        if(pst_mimo_mode->st_grdMimoMode == MIMO_1T1R)
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

static void IO_Task(void const *argument)
{
    while (1)
    {
        HAL_SRAM_CheckChannelTimeout();

        SYS_EVENT_Process();
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
        }
        else if(p->sky_or_grd == 1)
        {
            HAL_USB_Init(HAL_USB_PORT_1, HAL_USB_DR_MODE_DEVICE);
            DLOG_Warning("sram force to gnd mode, USB1 DEVICE");
        }
        else
        {
            HAL_USB_Init(HAL_USB_PORT_1, HAL_USB_DR_MODE_HOST);
        }
    }
    else
    {
        HAL_USB_Init(HAL_USB_PORT_1, HAL_USB_DR_MODE_HOST);
    }

}
static void USB_ID_Task(void const *argument)
{
	while(1)
	{
		DLOG_Critical("Loop USB0 ID State is: %d, Loop USB1 ID State is: %d", HAL_USB_Get_ID_State(HAL_USB_PORT_0), HAL_USB_Get_ID_State(HAL_USB_PORT_1));
		HAL_Delay(3000);
	}
}
static void USB_OTG_ID_CHANGE(void* p)
{
	STRU_SysEvent_USB_ID_STS *stSysUsbId = (STRU_SysEvent_USB_ID_STS *)p;
	DLOG_Critical("usb port is: %d , id state: %d", stSysUsbId->usb_port_id, stSysUsbId->usb_id_state);
}
/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
int main(void)
{
    int testmode = 0;
    STRU_HAL_SYS_CTL_CONFIG *pst_cfg;
    HAL_SYS_CTL_GetConfig( &pst_cfg);
    pst_cfg->u8_workMode = 1;
    HAL_SYS_CTL_Init(pst_cfg);

    /* initialize the uart */
    console_init(0,115200);
    dlog_set_output_level(LOG_LEVEL_WARNING);
    DLOG_Critical("cpu0 start!!! \n");

    WATCHDOG_Init();

    USB_Gpio_Ctrl_Init();
    //testmode = is_test_mode();
    DLOG_Critical("app 1.51\n");

    HAL_NV_Init();

    //HAL_SD_Init();

    BB_ledGpioInit();
    SYS_EVENT_RegisterHandler(SYS_EVENT_ID_BB_EVENT, BB_EventHandler);
    SYS_EVENT_RegisterHandler(SYS_EVENT_ID_BB_EVENT, BB_grdRcIdEventHandler);
    SYS_EVENT_RegisterHandler(SYS_EVENT_ID_USER_CFG_CHANGE, BB_ReplyPcHandler);
    SYS_EVENT_RegisterHandler(SYS_EVENT_ID_TO_APP, Grd_AppSysEventHandler);
    SYS_EVENT_RegisterHandler(SYS_EVENT_ID_USER_DEFINE, USER_Define_EventHandler);
    SYS_EVENT_RegisterHandler(SYS_EVENT_ID_USB_ID_STS, USB_OTG_ID_CHANGE);
    SYS_EVENT_RegisterHandler(SYS_EVENT_ID_USB_DEVICE_IDENTITY, USB_OTG_Role_Identify);


    *((volatile uint32_t*)0x40b000b8) = 0x03; //USB_PHY_TXPREEMPHASISTUNE

    //MFI_V3_Init(0xFF, HAL_I2C_COMPONENT_0);

    HAL_USB_ConfigPHY();

    HAL_GPIO_OutPut(29);
    HAL_GPIO_SetPin(29, 1);
    HAL_USB_Init(HAL_USB_PORT_0, HAL_USB_DR_MODE_DEVICE);
    HAL_USB_Init(HAL_USB_PORT_1, HAL_USB_DR_MODE_DEVICE);
//  USB1_init();

    HAL_SRAM_ReceiveVideoConfig();
    //sram0 -->audio ch or secod video way
    HAL_SRAM_ChannelConfig(ENUM_HAL_SRAM_CHANNEL_TYPE_VIDEO1,
                           0);
    //sram1 -->video ch(inclde hdmi src or usb src)
    HAL_SRAM_ChannelConfig(ENUM_HAL_SRAM_CHANNEL_TYPE_VIDEO0,
                           1);

    UPGRADE_GNDInit();
    fem_init();

    portDISABLE_INTERRUPTS();

    osThreadDef(USBHStatus_Task, USBH_USBHostStatus, osPriorityHigh, 0, 4 * 128);
    osThreadCreate(osThread(USBHStatus_Task), NULL);

    osThreadDef(DLOG_Task, DLOG, osPriorityIdle, 0, 4 * 128);
    osThreadCreate(osThread(DLOG_Task), NULL);

    osThreadDef(IOTask, IO_Task, osPriorityNormal, 0, 16 * 128);
    osThreadCreate(osThread(IOTask),NULL);

    osThreadDef(ledTask, grd_led_Task, osPriorityIdle, 0, 4 * 128);
    osThreadCreate(osThread(ledTask), NULL);

    osThreadDef(USBSend, usb_send_packet, osPriorityNormal, 0, 4 * 128);
    osThreadCreate(osThread(USBSend), NULL);
    
    //osThreadDef(ModPinSleepTask, Mod_Pin_SleepTask, osPriorityIdle, 0, 4 * 128);
    //osThreadCreate(osThread(ModPinSleepTask), NULL);
    

    osThreadDef(ModGndPinSearchIdTask, Mod_Grd_Pin_SearchIdTask, osPriorityIdle, 0, 4 * 128);
    osThreadCreate(osThread(ModGndPinSearchIdTask),NULL);

    usr_usb0_interface();

    //c201d_pt(testmode);

    usr_bypass_uart_task(pst_cfg->u8_workMode,HAL_UART_COMPONENT_5);

    //usr_bypass_sbus_uart_task(pst_cfg->u8_workMode);

    //usr_cmd_uart_task(pst_cfg->u8_workMode);

    //osThreadDef(UsbIDTask, USB_ID_Task, osPriorityNormal, 0, 16 * 128);
	//osThreadCreate(osThread(UsbIDTask), NULL);
    
#ifdef ENABLE_NV_GRD_SLAVE
    grd_slave_mode_init();
#endif

    Wireless_TaskInit(WIRELESS_USE_RTOS);

	//SysResetInit();
	
	//Sysreset();

	// MFI_Init(HAL_GPIO_NUM109, 0xFF, HAL_I2C_COMPONENT_0);

    portENABLE_INTERRUPTS();

    osKernelStart();
	

    /* We should never get here as control is now taken by the scheduler */
    for( ;; )
    {
    }
}
