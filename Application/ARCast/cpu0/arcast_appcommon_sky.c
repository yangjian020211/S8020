#include <stdint.h>
#include <string.h>

#include "debuglog.h"
#include "sys_event.h"
#include "memory_config.h"
#include "cmsis_os.h"
#include "hal_ret_type.h"
#include "hal_encodemp3.h"
#include "hal_softi2s.h"

#include "hal_usb_device.h"
#include "hal_bb.h"
#include "hal.h"
#include "arcast_appcommon.h"
#include "it_typedef.h"
#include "it6602.h"
#include "it_66021.h"
#include "eedid_create.h"
#include "systicks.h"
#include "hal_hdmi_rx.h"
#include "test_ite_hdmi.h"
#define ARCAST_DEBUGE

#ifdef  ARCAST_DEBUGE
#define DLOG_INFO(...) DLOG_Info(__VA_ARGS__)
#else
#define DLOG_INFO(...)
#endif

extern uint8_t hdmi_status;

extern unsigned char Default_Edid_Block[256];
static STRU_SysEvent_H264InputFormatChangeParameter g_st_format;
static uint32_t g_su32_timeStamp;
static uint8_t u8_edid[20] = {4,0,1,2,3,0,0,0,0,0,0,0,0,0,0,0,0,2,1,2};
static uint8_t s_u8_hpd = 0;
static uint32_t s_u32_auidoChange = 1;
static int8_t Command_TimeDiffDelay(uint32_t delayTime)
{
    if (SysTicks_GetDiff(g_su32_timeStamp, SysTicks_GetTickCount()) < delayTime)
    {
        return 0;
    }
    else
    {
        g_su32_timeStamp = SysTicks_GetTickCount();
        return 1;
    }
}

void Command_CloseH264(void)
{
    hdmi_status = 0;
    g_st_format.width = 0;
    g_st_format.hight = 0;
    g_st_format.framerate = 0;
    SYS_EVENT_NotifyInterCore(SYS_EVENT_ID_H264_INPUT_FORMAT_CHANGE, (void*)&g_st_format);
}

void Command_OpenH264(void)
{
    hdmi_status = 1;
    g_st_format.width = s_st_ARCastSupportedOutputFormat[st_ARCastStatus.u8_AVFormat[ARCAST_FROMART_VIDEO]][0];
    g_st_format.hight = s_st_ARCastSupportedOutputFormat[st_ARCastStatus.u8_AVFormat[ARCAST_FROMART_VIDEO]][1];
    g_st_format.framerate = s_st_ARCastSupportedOutputFormat[st_ARCastStatus.u8_AVFormat[ARCAST_FROMART_VIDEO]][2];
    SYS_EVENT_NotifyInterCore(SYS_EVENT_ID_H264_INPUT_FORMAT_CHANGE, (void*)&g_st_format);
    DLOG_INFO("start send video %d %d %d %d",g_st_format.width, g_st_format.hight, g_st_format.framerate, st_ARCastStatus.u8_AVFormat[ARCAST_FROMART_VIDEO]);
}

int8_t Common_ARStatus_FormatChange(void)
{
    Common_ARStatus_Clear(ARCAST_SKY_ENCODER);
    Common_ARStatus_Clear(ARCAST_COMMAND_SKY_FORMAT);
    Command_CloseH264();
}


int8_t Common_AVFORMAT_VideoFormatCheck(void* p)
{
    for (uint8_t i = 0; i < ARRAY_COUNT_OF(s_st_ARCastSupportedOutputFormat); i++)
    {
        if (((STRU_SysEvent_H264InputFormatChangeParameter*)p)->width == s_st_ARCastSupportedOutputFormat[i][0] &&
                ((STRU_SysEvent_H264InputFormatChangeParameter*)p)->hight == s_st_ARCastSupportedOutputFormat[i][1] && 
                ((STRU_SysEvent_H264InputFormatChangeParameter*)p)->framerate == s_st_ARCastSupportedOutputFormat[i][2])
        {
            return i;
        }
    }
    return 0xff;
}

void Common_AVFORMAT_Reinit(uint32_t sampleRate)
{
    if ((sampleRate !=st_ARCastStatus.u8_AVFormat[ARCAST_FROMART_AUDIO]) && (sampleRate != 1))
    {
        HAL_MP3EncodePcmUnInit();
        STRU_MP3_ENCODE_CONFIGURE_WAVE st_audioConfig;
        if (HAL_SOFTI2S_ENCODE_IEC_48000 == sampleRate)
        {                    
            st_audioConfig.e_samplerate = HAL_MP3_ENCODE_48000;
            st_ARCastStatus.u8_AVFormat[ARCAST_FROMART_AUDIO] = ARCAST_FROMART_AUDIO_48K;
            DLOG_INFO("Audio Sample Rate 48000");
        }
        else if (HAL_SOFTI2S_ENCODE_IEC_44100 == sampleRate)
        {
            st_audioConfig.e_samplerate = HAL_MP3_ENCODE_44100; 
            st_ARCastStatus.u8_AVFormat[ARCAST_FROMART_AUDIO] = ARCAST_FROMART_AUDIO_44p1K;
            DLOG_INFO("Audio Sample Rate 44100");                 
        } 
        
        st_audioConfig.e_modes = HAL_MP3_ENCODE_STEREO;
        st_audioConfig.u32_rawDataAddr = AUDIO_DATA_START;
        st_audioConfig.u32_rawDataLenght = AUDIO_DATA_BUFF_SIZE;
        st_audioConfig.u32_encodeDataAddr = MPE3_ENCODER_DATA_ADDR;
        st_audioConfig.u32_newPcmDataFlagAddr = SRAM_MODULE_SHARE_AUDIO_PCM;
        st_audioConfig.u8_channel = 2;
        HAL_MP3EncodePcmInit(&st_audioConfig, 0);
        Common_ARStatus_FormatChange();
    }
}

void Common_AVFORMAT_VideoSysEventCallBack(void* p)
{
    DLOG_Critical("video width=%d hight=%d framerate=%d vic=%d", ((STRU_SysEvent_H264InputFormatChangeParameter*)p)->width, 
                                                             ((STRU_SysEvent_H264InputFormatChangeParameter*)p)->hight, 
                                                             ((STRU_SysEvent_H264InputFormatChangeParameter*)p)->framerate,
                                                             ((STRU_SysEvent_H264InputFormatChangeParameter*)p)->vic);
    
    uint32_t sampleRate = 0xff;
    uint8_t u8_vid = Common_AVFORMAT_VideoFormatCheck(p);
    if (0xff == u8_vid)
    {
        if ((st_ARCastStatus.u8_AVFormat[ARCAST_FROMART_VIDEO] != u8_vid))
        {

            st_ARCastStatus.u8_AVFormat[ARCAST_FROMART_VIDEO] = u8_vid;
            Command_CloseH264();
            DLOG_Error("rec u8_vid=%d",u8_vid); 
        }
    }
    else if ((st_ARCastStatus.u8_AVFormat[ARCAST_FROMART_VIDEO] != u8_vid))
    {
        Common_ARStatus_FormatChange();
        st_ARCastStatus.u8_AVFormat[ARCAST_FROMART_VIDEO] = u8_vid;
        memcpy(&g_st_format, p, sizeof(STRU_SysEvent_H264InputFormatChangeParameter));
        //DLOG_Error("close encoder");
    }
        
}

void Common_AVFORMAT_AudioSysEventCallBack(void* p)
{
    HAL_HDMI_RX_GetAudioSampleRate(HAL_HDMI_RX_1, &s_u32_auidoChange);
    Common_AVFORMAT_Reinit(((STRU_SysEvent_AudioInputChangeParameter*)p)->u8_audioSampleRate);
}

void rcvFormatHandler_sky(void *p)
{
    uint32_t u32_rcvLen = 0;
    uint8_t  u8_recData[64];
    uint8_t  u8_ack;
    uint8_t  i;
    HAL_BB_ComReceiveMsg(BB_COM_SESSION_3, u8_recData, 64, &u32_rcvLen);

    if (1 == u32_rcvLen)
    {
        switch (u8_recData[0])
        {
            case ARCAST_COMMAND_SKY_FORMAT:
                {
                    Common_ARStatus_Set(ARCAST_COMMAND_SKY_FORMAT);
                    Common_ARStatus_Set(ARCAST_COMMAND_SKY_REQUEST_CAPABILITY);
                    DLOG_Critical("rec gnd ack FORMAT");
                }
                break;
            case  ARCAST_COMMAND_SKY_AVDATA:
                {
                    if ((0 == Common_ARStatus_Get(ARCAST_SKY_ENCODER)))
                    {
                        Command_OpenH264();
                        Common_ARStatus_Set(ARCAST_SKY_ENCODER);
                    }                  
                    u8_ack = ARCAST_COMMAND_SKY_AVDATA;
                    HAL_BB_ComSendMsg(BB_COM_SESSION_3, &u8_ack, 1);
                    DLOG_Critical("rec gnd ack AVdata");
                }
        }
    }
    else if (20 == u32_rcvLen)
    {
        u8_ack = ARCAST_COMMAND_GND_CAPABILITY;
        HAL_BB_ComSendMsg(BB_COM_SESSION_3, &u8_ack, 1);
        DLOG_Critical("rec gnd ack CAPABILITY");
        Command_CloseH264();
       
        Common_ARStatus_Clear(ARCAST_COMMAND_SKY_FORMAT);
        Common_ARStatus_Clear(ARCAST_SKY_ENCODER);
        Common_ARStatus_Set(ARCAST_COMMAND_SKY_REQUEST_CAPABILITY);
        
        if (0 != memcmp(u8_edid, u8_recData, u32_rcvLen))
        {

            memcpy(&st_ARCastStatus.st_avCapability, u8_recData, u32_rcvLen);
            memcpy(u8_edid, u8_recData, u32_rcvLen);
            Common_ARStatus_Set(ARCAST_COMMAND_GND_CAPABILITY);
                        
            DLOG_Critical("rec gnd ack new CAPABILITY");

        }
        
    }
    else
    {
        DLOG_Error("unkonwn Command %d %d", u32_rcvLen, u8_recData[0]);
    }
}
extern osSemaphoreId mp3_encoder_signal;
uint8_t Command_FormatStatus(void)
{
    uint8_t i = 0;
    volatile uint32_t * pu32_newPcmDataFlagAddr=(uint32_t *)(SRAM_MODULE_SHARE_AUDIO_PCM);
    if (Common_ARStatus_Get(ARCAST_SKY_ENCODER))
    {

        if (1 != s_u32_auidoChange)
        {
            if (0 != (*pu32_newPcmDataFlagAddr)) {
                //osSemaphoreRelease(mp3_encoder_signal);
                vTaskSuspendAll();
                HAL_MP3EncodePcm();
                xTaskResumeAll();
            }
        }
        else
        {
            //if (0 == Command_TimeDiffDelay(1000))
            //{
            //    return ARCAST_SKY_FORMAT_STABLE;
            //}
            //IT6602_fsm();
            HAL_HDMI_RX_GetAudioSampleRate(HAL_HDMI_RX_1, &s_u32_auidoChange);
            if (1 != s_u32_auidoChange) {
                Common_AVFORMAT_Reinit(s_u32_auidoChange);
            }
            DLOG_Error("no audio");
        }
        s_u8_hpd = 1;
        //IT6602_fsm();
        return ARCAST_SKY_FORMAT_STABLE;
    }
    else if (0 == Common_ARStatus_Get(ARCAST_COMMAND_SKY_REQUEST_CAPABILITY))
    {

        if (0 == Command_TimeDiffDelay(1000))
        {
            return ARCAST_SKY_FORMAT_CHANGE;
        }
        DLOG_Critical("power on request capability");
        uint8_t u8_command = ARCAST_COMMAND_SKY_REQUEST_CAPABILITY;
        HAL_BB_ComSendMsg(BB_COM_SESSION_3, &u8_command, 1);
        
        return ARCAST_SKY_FORMAT_CHANGE;
    }
    else if (Common_ARStatus_Get(ARCAST_COMMAND_GND_CAPABILITY))
    {

        Common_ARStatus_Clear(ARCAST_COMMAND_GND_CAPABILITY);
        if (st_ARCastStatus.st_avCapability.u8_VideoVidCount != 0)
        {
            //to hdp control
            if (1 == s_u8_hpd)
            {
                Command_CloseH264();
                create_eedid(Default_Edid_Block, &st_ARCastStatus.st_avCapability);
                it66021_init();
            }
        }

        return ARCAST_SKY_FORMAT_CHANGE;
        DLOG_Error("init ite ");

    }
    else if (0 == Common_ARStatus_Get(ARCAST_COMMAND_SKY_FORMAT))
    {
        
        if (0 == Command_TimeDiffDelay(1000))
        {
            return ARCAST_SKY_FORMAT_CHANGE;
        }

        if (0xff != st_ARCastStatus.u8_AVFormat[ARCAST_FROMART_VIDEO])
        {
            DLOG_Critical("video %d audio %d", st_ARCastStatus.u8_AVFormat[ARCAST_FROMART_VIDEO], st_ARCastStatus.u8_AVFormat[ARCAST_FROMART_AUDIO]);
            HAL_BB_ComSendMsg(BB_COM_SESSION_3, (uint8_t*)(&st_ARCastStatus.u8_AVFormat), 2);
        }
        else
        {
            DLOG_Critical("video %d audio %d", st_ARCastStatus.u8_AVFormat[ARCAST_FROMART_VIDEO], st_ARCastStatus.u8_AVFormat[ARCAST_FROMART_AUDIO]);
        }
        
        return ARCAST_SKY_FORMAT_CHANGE;
    }
}

void Common_AVFORMAT_ITEReset(void* p)
{
//  it66021_init();
    Command_CloseH264();
}

void Common_AVFORMATSysEventSKYInit(void)
{

    SYS_EVENT_RegisterHandler(SYS_EVENT_ID_H264_INPUT_FORMAT_CHANGE_ARCAST, Common_AVFORMAT_VideoSysEventCallBack);
    SYS_EVENT_RegisterHandler(SYS_EVENT_ID_AUDIO_INPUT_CHANGE, Common_AVFORMAT_AudioSysEventCallBack);
    SYS_EVENT_RegisterHandler(SYS_EVENT_ID_HDMI_RESET_EVENT, Common_AVFORMAT_ITEReset);

    st_ARCastStatus.u8_AVFormat[ARCAST_FROMART_AUDIO] = ARCAST_FROMART_AUDIO_48K;
    st_ARCastStatus.u8_AVFormat[ARCAST_FROMART_VIDEO] = 0xff;
    memset(&st_ARCastStatus.st_avCapability, 0, sizeof(&st_ARCastStatus.st_avCapability));
    Common_ARStatus_Set(ARCAST_COMMAND_SKY_FORMAT);
    HAL_HDMI_RX_VideoCallbak(Common_AVFORMAT_VideoSysEventCallBack);
    HAL_HDMI_RX_AudioCallbak(Common_AVFORMAT_AudioSysEventCallBack);
   // st_ARCastStatus.u8_ARStatus = 1;
}



void ATM_StatusCallBack(uint8_t *st_status, uint32_t data_len)
{

}
