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
#include "hal_usb_host.h"
#include "hal_softi2s.h"
#include "hal_nvic.h"
#include "hal_encodemp3.h"
#include "systicks.h"
#include "memory_config.h"
#include "hal_uart.h"
#include "it_typedef.h"
#include "it6602.h"
#include "test_bb_led_ctrl_2.h"
#include "arcast_appcommon.h"
#include "hal_pmu.h"
#include "hal_timer.h"
#include "board_watchdog.h"
#include "cmd_line.h"
#include "test_search_id.h"
#include "hal_bb.h"
#include "test_hdmi.h"

#include "bb_reply_pc.h"
#include "app_sys_event_process.h"
#include "usr_usb_task.h"

static uint32_t arcast_i=0;
void TIMER_avsyncInterruptHandle(uint32_t u32_vectorNum)
{
    arcast_i++;
    HAL_GPIO_SetPin(70, (arcast_i&1));

}

uint8_t Command_FormatStatus(void);
void CONSOLE_Init(void)
{
    HAL_UART_Init(DEBUG_LOG_UART_PORT, HAL_UART_BAUDR_115200, NULL);
    DLOG_Init(CMD_exec_cmd, NULL, DLOG_SERVER_PROCESSOR);
}

void HDMI_powerOn(void)
{
    HAL_GPIO_OutPut(101);
    HAL_GPIO_SetPin(101, HAL_GPIO_PIN_SET);
}

osSemaphoreId mp3_encoder_signal;
void DLOG(void const *argument)
{
    
    while (1)
    {
        Command_FormatStatus();
        DLOG_Process(NULL);
    }
}

static void IO_Task(void const *argument)
{
    while (1)
    {
        SYS_EVENT_Process();
    }
}

static void MP3_Endcoder_Task(void const *argument)
{
    osSemaphoreDef(mp3_encode);
    mp3_encoder_signal = osSemaphoreCreate(osSemaphore(mp3_encode), 1);
    while (1) {
        if (osOK == osSemaphoreWait(mp3_encoder_signal, osWaitForever)) {
            HAL_MP3EncodePcm();
        }
        
    }
}

static void GenericInitial(void const *argument)
{
    //HAL_PMU_Init();
    //Common_AVFORMATSysEventSKYInit();
    HAL_BB_ComRegisterSession(BB_COM_SESSION_3,
                                  BB_COM_SESSION_PRIORITY_HIGH,
                                  BB_COM_SESSION_DATA_RT,
                                  rcvFormatHandler_sky);
    vTaskDelete(NULL);
}

#define SKY_SEARCH_ID_PIN           (HAL_GPIO_NUM103)
#define KEY_LONG_PRESS              (1000)
#define READ_INTERVAL               (200)

extern uint32_t flag_searchIdTimerStart;

void ARCAST_Sky_Pin_SearchIdTask(void const *argument)
{
    uint8_t cnt = 0, key_release = 1;
    uint32_t pin_value;

    HAL_GPIO_SetMode(SKY_SEARCH_ID_PIN, HAL_GPIO_PIN_MODE2);
    HAL_GPIO_InPut(SKY_SEARCH_ID_PIN);

    while (1)
    {
        HAL_GPIO_GetPin(SKY_SEARCH_ID_PIN, &pin_value);

        if(pin_value == 0)
        {
            cnt++;
            if((cnt >= KEY_LONG_PRESS / READ_INTERVAL) && key_release)
            {
                if(!flag_searchIdTimerStart)
                {
                    DLOG_Warning("pin search id start");
                    BB_Sky_SearchIdHandler(NULL);
                    key_release = 0;
                }
            }
        }
        else
        {
            cnt = 0;
            key_release = 1;
        }

        HAL_Delay(READ_INTERVAL);
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
    pst_cfg->u16_cpu2Clk = 200; //To fix the 9363 RF sensitify problem
    HAL_SYS_CTL_Init(pst_cfg);
    WATCHDOG_Init();
    /* initialize the uart */
    CONSOLE_Init();
    dlog_set_output_level(LOG_LEVEL_WARNING);
    DLOG_Critical("cpu0 start!!! \n");

    BB_ledGpioInit();
    SYS_EVENT_RegisterHandler(SYS_EVENT_ID_BB_EVENT, BB_EventHandler);
    SYS_EVENT_RegisterHandler(SYS_EVENT_ID_BB_EVENT, BB_skyRcIdEventHandler);

    SYS_EVENT_RegisterHandler(SYS_EVENT_ID_USER_CFG_CHANGE, BB_ReplyPcHandler);
    SYS_EVENT_RegisterHandler(SYS_EVENT_ID_TO_APP, Sky_AppSysEventHandler);
    HAL_USB_ConfigPHY();

    HDMI_powerOn();
    //HAL_TIMER_RegisterTimer(HAL_TIMER_NUM12, 10, TIMER_avsyncInterruptHandle);
    //*((uint32_t *)(SRAM_MODULE_SHARE_AVSYNC_TICK)) = 0;
    
  

    Common_AVFORMATSysEventSKYInit();
    *((uint8_t *)(SRAM_MODULE_SHARE_AUDIO_PCM)) = HAL_SOFTI2S_ENCODE_IEC_48000;
    STRU_MP3_ENCODE_CONFIGURE_WAVE st_audioConfig;
    st_audioConfig.e_samplerate = HAL_MP3_ENCODE_48000;
    st_audioConfig.e_modes = HAL_MP3_ENCODE_STEREO;
    st_audioConfig.u32_rawDataAddr = AUDIO_DATA_START;
    st_audioConfig.u32_rawDataLenght = AUDIO_DATA_BUFF_SIZE;
    st_audioConfig.u32_encodeDataAddr = MPE3_ENCODER_DATA_ADDR;
    st_audioConfig.u32_newPcmDataFlagAddr = SRAM_MODULE_SHARE_AUDIO_PCM;
    st_audioConfig.u8_channel = 2;
    HAL_MP3EncodePcmInit(&st_audioConfig, 0);

    STRU_HDMI_CONFIGURE        st_configure;
    st_configure.e_getFormatMethod = HAL_HDMI_INTERRUPT;
    st_configure.e_colorDepth = HAL_HDMI_RX_8BIT;
    st_configure.st_interruptGpio.e_interruptGpioNum = HAL_GPIO_NUM98;
    st_configure.st_interruptGpio.e_interruptGpioPolarity = HAL_GPIO_ACTIVE_LOW;
    st_configure.st_interruptGpio.e_interruptGpioType = HAL_GPIO_LEVEL_SENUMSITIVE;
    st_configure.u8_hdmiToEncoderCh = 1;
    HAL_HDMI_RX_Init(HAL_HDMI_RX_1, &st_configure);

    HAL_USB_Init(HAL_USB_PORT_0, HAL_USB_DR_MODE_DEVICE);
    HAL_USB_Init(HAL_USB_PORT_1, HAL_USB_DR_MODE_DEVICE);

    HAL_NV_Init();
	
	UPGRADE_SKYInit();

    portDISABLE_INTERRUPTS();

    osThreadDef(USBHStatus_Task, USBH_USBHostStatus, osPriorityHigh, 0, 4 * 128);
    osThreadCreate(osThread(USBHStatus_Task), NULL);

    osThreadDef(GenericInitialTask, GenericInitial, osPriorityHigh, 0, 4 * 128);
    osThreadCreate(osThread(GenericInitialTask), NULL);

    osThreadDef(DLOG_Task, DLOG, osPriorityNormal, 0, 4 * 128);
    osThreadCreate(osThread(DLOG_Task), NULL);

    osThreadDef(IOTask, IO_Task, osPriorityNormal, 0, 16 * 128);
    osThreadCreate(osThread(IOTask), NULL);

    osThreadDef(ArcastSkyPinSearchIdTask, ARCAST_Sky_Pin_SearchIdTask, osPriorityNormal, 0, 4 * 128);
    osThreadCreate(osThread(ArcastSkyPinSearchIdTask),NULL);

    osThreadDef(ledTask, sky_led_Task, osPriorityNormal, 0, 16 * 128);
    osThreadCreate(osThread(ledTask), NULL);

/*    osThreadDef(MP3_EndcoderTask, MP3_Endcoder_Task, osPriorityHigh, 0, 16 * 128);
    osThreadCreate(osThread(MP3_EndcoderTask), NULL);*/

    Wireless_TaskInit(WIRELESS_USE_RTOS);

//    usr_usb0_interface();

    portENABLE_INTERRUPTS();

    osKernelStart();

    /* We should never get here as control is now taken by the scheduler */
    for( ;; )
    {
    }
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

