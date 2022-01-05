#include "debuglog.h"
#include "command.h"
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
#include "hal_gpio.h"
#include "hal_uart.h"
#include "arcast_appcommon.h"
#include "hal_pmu.h"
#include "minimp3.h"
#include <stdlib.h>
#include <string.h>
#include "test_usbh.h"
#include "systicks.h"
#include "hal_usb_host.h"
#include "hal_usb_device.h"
#include "hal_dma.h"
#include "test_i2c_adv7611.h"
#include "test_bb_led_ctrl_2.h"
#include "board_watchdog.h"
#include "cmd_line.h"
#include "bb_reply_pc.h"
#include "app_sys_event_process.h"
#include "test_search_id.h"
#include "usr_usb_task.h"
#include "hal_sram_ground.h"

void Command_BBSendCommand(void const *argument);

void console_init(uint32_t uart_num, uint32_t baut_rate)
{
    HAL_UART_Init(DEBUG_LOG_UART_PORT, HAL_UART_BAUDR_115200, NULL);
    DLOG_Init(CMD_exec_cmd, NULL, DLOG_SERVER_PROCESSOR);
}

osSemaphoreId mp3_decoder_signal;

void DLOG(void const *argument)
{
    uint16_t u16_mp3BuffLen = 0;
    while (1)
    {
        #if 0
        u16_mp3BuffLen = HAL_SRAM_GetMp3BufferLength();
        if (u16_mp3BuffLen) {
            osSemaphoreRelease(mp3_decoder_signal);
            DLOG_Critical("Release %d \n", u16_mp3BuffLen);
        }
        #endif
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

static void GenericInitial(void const *argument)
{
    //HAL_PMU_Init();
    Common_AVFORMATSysEventGroundInit();
    vTaskDelete(NULL);
}

mp3_decoder_t g_p_mp3Decoder = NULL;
//#define ARCAST_MP3                  (1)
#define MP3_BUFF_SIZE               (4096)
#define MP3_USERDATA_LEN            (8)
uint8_t u8_mp3Enable = 1;
uint8_t u8_mp3DataArray[2][1024];      

signed short u8_pcmDataArray[2][2304];

static uint8_t app_count = 0;
static uint8_t app_usbdisconnectcount = 0;

static void Mp3Decoder(void const *argument)
{
    //uint8_t *p_mp3DataBuff = (uint8_t *)malloc(MP3_BUFF_SIZE+512);
    uint8_t *p_mp3DataBuff = NULL;
    uint8_t *p_mp3DataBuffTmp = NULL;
    uint8_t *p_PCMDataBuff = NULL;
    uint16_t u16_mp3BuffLen = 0;
    uint16_t u16_mp3RecLen = 0;
    int32_t  s32_decoderBuffLen = 0;
    uint32_t u32_decoderBuffpos = 0;
    uint32_t tmp = 0;
    uint32_t check = 0;
    uint32_t         src;
    uint32_t         dest;
    uint32_t         i,k;
    uint16_t         dataLenTemp;
    uint8_t   buff_index = 0;
    int frame_size = 0;
    uint8_t          u8_swap;
    uint8_t u8_checkSum = 0;;
    uint32_t u32_timeStamp = 0;

    mp3_info_t mp3_info;
    p_mp3DataBuffTmp = p_mp3DataBuff;
    osSemaphoreDef(mp3_decode);
    mp3_decoder_signal = osSemaphoreCreate(osSemaphore(mp3_decode), 1);
    #if 0
    while(1) {
        if (osOK == osSemaphoreWait(mp3_decoder_signal, osWaitForever)) {
            u16_mp3BuffLen = HAL_SRAM_GetMp3BufferLength();
            DLOG_Critical("u16_mp3BuffLen %d \n", u16_mp3BuffLen);
            if (u16_mp3BuffLen > 0) {
                u16_mp3RecLen = HAL_SRAM_GetMp3Data(1024, p_mp3DataBuffTmp);
            }
        }

    }
    #endif
    while(1)
    {
        if (osOK == osSemaphoreWait(mp3_decoder_signal, osWaitForever)) {
            u16_mp3BuffLen = HAL_SRAM_GetMp3BufferLength();
            DLOG_Critical("u16_mp3BuffLen %d \n", u16_mp3BuffLen);
            if (u8_mp3Enable == 1)
            {
                if (u16_mp3BuffLen > 0)
                {

                    p_mp3DataBuffTmp = (uint8_t *)(u8_mp3DataArray[buff_index]);

                    if (u16_mp3BuffLen > 1024)
                    {
                        u16_mp3RecLen = HAL_SRAM_GetMp3Data(1024, p_mp3DataBuffTmp);            
                    }
                    else
                    {
                        u16_mp3RecLen = HAL_SRAM_GetMp3Data(u16_mp3BuffLen, p_mp3DataBuffTmp);            
                    }
                
                    for (i = 0; i < u16_mp3RecLen ; i+=4)
                    {
                        u8_swap                = p_mp3DataBuffTmp[i];
                        p_mp3DataBuffTmp[i]       = p_mp3DataBuffTmp[i+3];
                        p_mp3DataBuffTmp[i+3]     = u8_swap;

                        u8_swap                = p_mp3DataBuffTmp[i+1];
                        p_mp3DataBuffTmp[i+1]     = p_mp3DataBuffTmp[i+2];
                        p_mp3DataBuffTmp[i+2]     = u8_swap;
                    }
                    if (HAL_OK != HAL_USB_SendData(p_mp3DataBuffTmp, u16_mp3RecLen, 0, 0x85))
                    {
                        app_usbdisconnectcount++;
                        if (app_usbdisconnectcount > 100)
                        {
                            app_usbdisconnectcount = 0;
                            DLOG_Error("send audio data timeout");
                        }
                    }

                    buff_index++;
                    buff_index &= 1;

                }
            }
            else  
            {
                if (u16_mp3BuffLen > 0)
                {

                    if ((s32_decoderBuffLen + u16_mp3BuffLen) > (MP3_BUFF_SIZE + 512))
                    {
                        DLOG_Error("mp3 buff overflow :%d", s32_decoderBuffLen);
                        u16_mp3RecLen = HAL_SRAM_GetMp3Data(MP3_BUFF_SIZE, p_mp3DataBuffTmp);            
                        s32_decoderBuffLen = u16_mp3RecLen;
                    }
                    else
                    {
                        u16_mp3RecLen = HAL_SRAM_GetMp3Data(u16_mp3BuffLen, (p_mp3DataBuffTmp + s32_decoderBuffLen));            
                        s32_decoderBuffLen += u16_mp3RecLen;
                    }
                    do
                    {
                        frame_size = mp3_decode(g_p_mp3Decoder, p_mp3DataBuffTmp, s32_decoderBuffLen, u8_pcmDataArray[buff_index], &mp3_info);                
                        if (frame_size > 0)
                        {
                            
                            if (mp3_info.audio_bytes < 0)
                            {
                                DLOG_Error("audio_bytes error :%d frame_size=%d", mp3_info.audio_bytes, frame_size);
                            }
                            else
                            {
                                
                                k++;
                                if (k>100)
                                {
                                    
                                    u8_checkSum = 0;
                                    u8_checkSum += ((uint8_t)p_mp3DataBuffTmp[frame_size+0]+(uint8_t)p_mp3DataBuffTmp[frame_size+1]+
                                                    (uint8_t)p_mp3DataBuffTmp[frame_size+2]+(uint8_t)p_mp3DataBuffTmp[frame_size+4]+
                                                    (uint8_t)p_mp3DataBuffTmp[frame_size+5]+(uint8_t)p_mp3DataBuffTmp[frame_size+6]+
                                                    (uint8_t)p_mp3DataBuffTmp[frame_size+7]);
                                    if (u8_checkSum != p_mp3DataBuffTmp[frame_size+3])
                                    {
                                        DLOG_Error("timestamp error stream=%x checkSum=%x", (uint8_t)p_mp3DataBuffTmp[frame_size+3], u8_checkSum);
                                        DLOG_Error("stream data=%x %x %x %x %x %x %x %x", (uint8_t)p_mp3DataBuffTmp[frame_size+0],(uint8_t)p_mp3DataBuffTmp[frame_size+1],
                                                                                          (uint8_t)p_mp3DataBuffTmp[frame_size+2],(uint8_t)p_mp3DataBuffTmp[frame_size+4],
                                                                                          (uint8_t)p_mp3DataBuffTmp[frame_size+5],(uint8_t)p_mp3DataBuffTmp[frame_size+6],
                                                                                          (uint8_t)p_mp3DataBuffTmp[frame_size+7],(uint8_t)p_mp3DataBuffTmp[frame_size+3]);
                                    }
                                    else
                                    {
                                        k=0;
                                        u32_timeStamp |= (p_mp3DataBuffTmp[frame_size+4]&0xff)<<24;
                                        u32_timeStamp |= (p_mp3DataBuffTmp[frame_size+5]&0xff)<<16;
                                        u32_timeStamp |= (p_mp3DataBuffTmp[frame_size+6]&0xff)<<8;
                                        u32_timeStamp |= (p_mp3DataBuffTmp[frame_size+7])&0xff;
                                        DLOG_Info("time stamp =%x",u32_timeStamp);
                                        u32_timeStamp = 0;

                                    }                            
                                }
                                #if 1
                                p_PCMDataBuff = (uint8_t *)(u8_pcmDataArray[buff_index]);
                            
                                for (i = 0; i < mp3_info.audio_bytes ; i+=4)
                                {
                                    u8_swap                = p_PCMDataBuff[i];
                                    p_PCMDataBuff[i]       = p_PCMDataBuff[i+3];
                                    p_PCMDataBuff[i+3]     = u8_swap;

                                    u8_swap                = p_PCMDataBuff[i+1];
                                    p_PCMDataBuff[i+1]     = p_PCMDataBuff[i+2];
                                    p_PCMDataBuff[i+2]     = u8_swap;
                                }
                                                
                                if (HAL_OK != HAL_USB_SendData(p_mp3DataBuffTmp, u16_mp3RecLen, 0, 0x85))
                                {
                                    DLOG_Error("send audio data timeout");
                                }
                                #endif
                                buff_index++;
                                buff_index &= 1;
                            }
                            if (s32_decoderBuffLen < (frame_size+MP3_USERDATA_LEN))
                            {
                                DLOG_Error("mp3 data error s32_decoderBuffLen=%d",s32_decoderBuffLen);
                                s32_decoderBuffLen = 0;
                                k = 100;
                                break;
                            }
                            p_mp3DataBuffTmp += (frame_size+MP3_USERDATA_LEN);
                            s32_decoderBuffLen -= (frame_size+MP3_USERDATA_LEN);
                        }
                        else if (s32_decoderBuffLen > 500)
                        {
                            p_mp3DataBuffTmp += (s32_decoderBuffLen-500);
                            s32_decoderBuffLen = 500;
                            k = 100;
                        }
                    }while((frame_size > 0) && (s32_decoderBuffLen > MP3_USERDATA_LEN));
                    
                    memcpy(p_mp3DataBuff, p_mp3DataBuffTmp, s32_decoderBuffLen);
                    p_mp3DataBuffTmp = p_mp3DataBuff;

                }
            }
        }
    
    }
}

#define GRD_SEARCH_ID_PIN           (HAL_GPIO_NUM99)
#define KEY_LONG_PRESS              (1000)
#define READ_INTERVAL               (200)

extern uint32_t flag_searchIdTimerStart;

void ARCAST_Grd_Pin_SearchIdTask(void const *argument)
{
    uint8_t cnt = 0, key_release = 1;
    uint32_t pin_value;

    HAL_GPIO_SetMode(GRD_SEARCH_ID_PIN, HAL_GPIO_PIN_MODE2);
    HAL_GPIO_InPut(GRD_SEARCH_ID_PIN);

    while (1)
    {
        HAL_GPIO_GetPin(GRD_SEARCH_ID_PIN, &pin_value);

        if(pin_value == 0)
        {
            cnt++;
            if((cnt >= KEY_LONG_PRESS / READ_INTERVAL) && key_release)
            {
                if(!flag_searchIdTimerStart)
                {
                    DLOG_Warning("pin search id start");
                    BB_Grd_SearchIdHandler(NULL);
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

    WATCHDOG_Init();
    
    STRU_HAL_SYS_CTL_CONFIG *pst_cfg;
    HAL_SYS_CTL_GetConfig( &pst_cfg);
    pst_cfg->u8_workMode = 1;
    pst_cfg->u16_cpu2Clk = 200; //To fix the 9363 RF sensitify problem
    HAL_SYS_CTL_Init(pst_cfg);
//  USB_Gpio_Ctrl_Init();
    /* initialize the uart */
    console_init(0,115200);
    dlog_set_output_level(LOG_LEVEL_WARNING);
    DLOG_Critical("cpu0 start!!! \n");
    //WATCHDOGUPGRADE_Reset();
    //g_p_mp3Decoder = mp3_create();
	//WATCHDOGUPGRADE_Reset();
    //HAL_GPIO_InPut(HAL_GPIO_NUM99);

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

    HAL_SRAM_Channel_USB_Config(ENUM_HAL_SRAM_CHANNEL_TYPE_VIDEO1,
                                HAL_USB_PORT_0,
                                0);

    HAL_SRAM_Channel_USB_Config(ENUM_HAL_SRAM_CHANNEL_TYPE_VIDEO0,
                                HAL_USB_PORT_0,
                                1);

    HAL_NV_Init();

    UPGRADE_GNDInit();

    osThreadDef(USBHStatus_Task, USBH_USBHostStatus, osPriorityHigh, 0, 4 * 128);
    osThreadCreate(osThread(USBHStatus_Task), NULL);

    osThreadDef(DLOG_Task, DLOG, osPriorityIdle, 0, 4 * 128);
    osThreadCreate(osThread(DLOG_Task), NULL);

    osThreadDef(IOTask, IO_Task, osPriorityNormal, 0, 16 * 128);
    osThreadCreate(osThread(IOTask), NULL);
    
    osThreadDef(GenericInitialTask, GenericInitial, osPriorityIdle, 0, 4 * 128);
    osThreadCreate(osThread(GenericInitialTask), NULL);

    osThreadDef(SendCommand_Task, Command_BBSendCommand, osPriorityBelowNormal, 0, 4 * 128);
    osThreadCreate(osThread(SendCommand_Task), NULL);

    osThreadDef(ArcastGndPinSearchIdTask, ARCAST_Grd_Pin_SearchIdTask, osPriorityIdle, 0, 4 * 128);
    osThreadCreate(osThread(ArcastGndPinSearchIdTask),NULL);

    osThreadDef(ledTask, grd_led_Task, osPriorityIdle, 0, 4 * 128);
    osThreadCreate(osThread(ledTask), NULL);
#if 0
    osThreadDef(MP3_Task, Mp3Decoder, osPriorityHigh, 0, 4 * 128);
    osThreadCreate(osThread(MP3_Task), NULL);

#endif

    Wireless_TaskInit(WIRELESS_USE_RTOS);

    //usr_usb0_interface();

    osKernelStart();

    /* We should never get here as control is now taken by the scheduler */
    for( ;; )
    {
    }
}

/************************ (C) COPYRIGHT Artosyn *****END OF FILE****/

