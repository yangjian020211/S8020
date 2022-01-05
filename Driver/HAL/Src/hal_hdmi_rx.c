 /*****************************************************************************
Copyright: 2016-2020, Artosyn. Co., Ltd.
File name: hal_hdmi_rx.c
Description: The external HAL APIs to use HDMI RX.
Author: Artosyn Software Team
Version: 0.0.1
Date: 2016/12/20
History: 
        0.0.1    2016/12/20    The initial version of hal_hdmi_rx.c
*****************************************************************************/

#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "sys_event.h"
#include "adv_7611.h"
#include "it_66021.h"
#include "hal_hdmi_rx.h"
#include "hal_gpio.h"
#include "gpio.h"
#include "hal_ret_type.h"
#include "interrupt.h"
#include "debuglog.h"
#include "hal_nvic.h"
#include "it_typedef.h"
#include "it6602.h"
#include "memory_config.h"
#include "hal_encodemp3.h"
#include "hal_softi2s.h"

#ifdef USE_ADV7611_EDID_CONFIG_BIN
#pragma message("defined ADV_7611")
#endif


#ifdef USE_IT66021_EDID_CONFIG_BIN
#pragma message("defined IT_66021")
#endif


void HDMI_RxVideoNotifyH264Endcoder(void *pu8_rxBuf)
{
    #ifdef USE_IT66021_EDID_CONFIG_BIN
            SYS_EVENT_NotifyInterCore(SYS_EVENT_ID_H264_INPUT_FORMAT_CHANGE, pu8_rxBuf);
    #endif
    #ifdef USE_ADV7611_EDID_CONFIG_BIN
            SYS_EVENT_NotifyInterCore(SYS_EVENT_ID_H264_INPUT_FORMAT_CHANGE, pu8_rxBuf);
    #endif

}

void HDMI_RxAudioDefault(void *pu8_rxBuf)
{
    #ifdef USE_IT66021_EDID_CONFIG_BIN
            SYS_EVENT_NotifyInterCore(SYS_EVENT_ID_AUDIO_INPUT_CHANGE, pu8_rxBuf);
    #endif

    ;
}

HAL_HDMI_RxHandle HDMI_RxVideoHandle = HDMI_RxVideoNotifyH264Endcoder;
HAL_HDMI_RxHandle HDMI_RxAudioHandle = HDMI_RxAudioDefault;

static uint32_t u32_audioSampleRateChangeCount=0;
static ENUM_HAL_I2S_IEC_SAMPLERATE s_e_hdmiRxAudioSampleRateStatus = HAL_SOFTI2S_ENCODE_IEC_48000;

static volatile uint32_t *HAL_hdmi_pu32_newAudioSampleRate=(uint32_t *)(SRAM_MODULE_SHARE_AUDIO_RATE);
static ENUM_HAL_I2S_IEC_SAMPLERATE s_u32_hdmiRxSupportedOutputSampleRate[] = {HAL_SOFTI2S_ENCODE_IEC_44100,
                                                                              HAL_SOFTI2S_ENCODE_IEC_48000};


static STRU_HDMI_RX_STATUS s_st_hdmiRxStatus[HAL_HDMI_RX_MAX] = {0};
static void HAL_HDMI_RX_IrqHandler0(uint32_t u32_vectorNum);
static void HAL_HDMI_RX_IrqHandler1(uint32_t u32_vectorNum);

static STRU_HDMI_RX_OUTPUT_FORMAT s_st_hdmiRxSupportedOutputFormat[] =
{
    {720,  480,  60},
    {1280, 720,  30},
    {1280, 720,  25},
    {1280, 720,  50},
    {1280, 720,  60},
    {1920, 1080, 25},
    {1920, 1080, 30},
//    {1920, 1080, 50},
//    {1920, 1080, 60},
};

static uint8_t HDMI_RX_MapToDeviceIndex(ENUM_HAL_HDMI_RX e_hdmiIndex)
{
    return (e_hdmiIndex == HAL_HDMI_RX_0) ? 0 : 1;
}

uint32_t HDMI_RX_CheckVideoFormatSupportOrNot(uint16_t u16_width, uint16_t u16_hight, uint8_t u8_framerate)
{
    uint8_t i = 0;
    uint8_t array_size = sizeof(s_st_hdmiRxSupportedOutputFormat)/sizeof(s_st_hdmiRxSupportedOutputFormat[0]);

    for (i = 0; i < array_size; i++)
    {
        if ((u16_width == s_st_hdmiRxSupportedOutputFormat[i].u16_width) &&
            (u16_hight == s_st_hdmiRxSupportedOutputFormat[i].u16_hight) &&
            (u8_framerate == s_st_hdmiRxSupportedOutputFormat[i].u8_framerate))
        {
            break;
        }
    }
    
    if (i < array_size)
    {
        return HAL_OK;
    }
    else
    {
        return HAL_HDMI_RX_FALSE;
    }

}


static uint32_t HDMI_RX_CheckVideoFormatChangeOrNot(ENUM_HAL_HDMI_RX e_hdmiIndex, 
                                                      uint16_t u16_width, 
                                                      uint16_t u16_hight, 
                                                      uint8_t u8_framerate)
{
    if (e_hdmiIndex >= HAL_HDMI_RX_MAX)
    {
        return HAL_HDMI_RX_FALSE;
    }
    
    if ((s_st_hdmiRxStatus[e_hdmiIndex].u8_devEnable == 1) &&
        ((s_st_hdmiRxStatus[e_hdmiIndex].st_videoFormat.u16_width != u16_width) ||
         (s_st_hdmiRxStatus[e_hdmiIndex].st_videoFormat.u16_hight != u16_hight) ||
         (s_st_hdmiRxStatus[e_hdmiIndex].st_videoFormat.u8_framerate != u8_framerate)))
    {
        return HAL_OK;
    }
    else
    {
        return HAL_HDMI_RX_FALSE;
    }
}

static void HDMI_RX_CheckFormatStatus(ENUM_HAL_HDMI_RX e_hdmiIndex, uint32_t b_noDiffCheck)
{
    static uint8_t s_u8_formatNotSupportCount = 0;

    uint16_t u16_width;
    uint16_t u16_hight;
    uint8_t u8_framerate;
    uint8_t u8_vic;

    uint8_t u8_hdmiIndex = HDMI_RX_MapToDeviceIndex(e_hdmiIndex);

    #ifdef USE_ADV7611_EDID_CONFIG_BIN
        ADV_7611_GetVideoFormat(u8_hdmiIndex, &u16_width, &u16_hight, &u8_framerate);
    #endif

    #ifdef USE_IT66021_EDID_CONFIG_BIN
        IT_66021_GetVideoFormat(u8_hdmiIndex, &u16_width, &u16_hight, &u8_framerate, &u8_vic);
    #endif

    if (HDMI_RX_CheckVideoFormatSupportOrNot(u16_width, u16_hight, u8_framerate) == HAL_OK)
    {
        s_u8_formatNotSupportCount = 0;
        if ((b_noDiffCheck == HAL_OK) || 
            (HDMI_RX_CheckVideoFormatChangeOrNot(e_hdmiIndex, u16_width, u16_hight, u8_framerate) == HAL_OK))
        {
            STRU_SysEvent_H264InputFormatChangeParameter p;
            p.index = s_st_hdmiRxStatus[e_hdmiIndex].st_configure.u8_hdmiToEncoderCh;
            p.width = u16_width;
            p.hight = u16_hight;
            p.framerate = u8_framerate;
            p.vic = u8_vic;
            if (HAL_HDMI_RX_0 == e_hdmiIndex)
            {
                p.e_h264InputSrc = ENCODER_INPUT_SRC_HDMI_0;
            }
            else
            {
                p.e_h264InputSrc = ENCODER_INPUT_SRC_HDMI_1;
            }
            
            SYS_EVENT_NotifyLocal(SYS_EVENT_ID_VIDEO_EVENT, (void*)&p);

            s_st_hdmiRxStatus[e_hdmiIndex].st_videoFormat.u16_width    = u16_width;
            s_st_hdmiRxStatus[e_hdmiIndex].st_videoFormat.u16_hight    = u16_hight;
            s_st_hdmiRxStatus[e_hdmiIndex].st_videoFormat.u8_framerate = u8_framerate;
            s_st_hdmiRxStatus[e_hdmiIndex].st_videoFormat.u8_vic = u8_vic;
            
        }
    }
    else
    {
        if (HAL_HDMI_POLLING == s_st_hdmiRxStatus[e_hdmiIndex].st_configure.e_getFormatMethod)
        {
             // Format not supported
            if (s_u8_formatNotSupportCount <= HDMI_RX_FORMAT_NOT_SUPPORT_COUNT_MAX)
            {
                s_u8_formatNotSupportCount++;
            }            
        }
        else
        {
            s_u8_formatNotSupportCount = HDMI_RX_FORMAT_NOT_SUPPORT_COUNT_MAX;
        }

        if (s_u8_formatNotSupportCount == HDMI_RX_FORMAT_NOT_SUPPORT_COUNT_MAX)
        {
            STRU_SysEvent_H264InputFormatChangeParameter p;
            p.index = s_st_hdmiRxStatus[e_hdmiIndex].st_configure.u8_hdmiToEncoderCh;
            p.width = 0;
            p.hight = 0;
            p.framerate = 0;
            p.vic = u8_vic;
            if (HAL_HDMI_RX_0 == e_hdmiIndex)
            {
                p.e_h264InputSrc = ENCODER_INPUT_SRC_HDMI_0;
            }
            else
            {
                p.e_h264InputSrc = ENCODER_INPUT_SRC_HDMI_1;
            }
            
            SYS_EVENT_NotifyLocal(SYS_EVENT_ID_VIDEO_EVENT, (void*)&p);

            s_st_hdmiRxStatus[e_hdmiIndex].st_videoFormat.u16_width    = 0;
            s_st_hdmiRxStatus[e_hdmiIndex].st_videoFormat.u16_hight    = 0;
            s_st_hdmiRxStatus[e_hdmiIndex].st_videoFormat.u8_framerate = 0;
            s_st_hdmiRxStatus[e_hdmiIndex].st_videoFormat.u8_vic = u8_vic;
        }
       
    }
}

static uint32_t HDMI_RX_CheckAudioSampleRateSupportOrNot(uint32_t u32_sampleRate)
{
    uint8_t i = 0;
    uint8_t array_size = sizeof(s_u32_hdmiRxSupportedOutputSampleRate)/sizeof(s_u32_hdmiRxSupportedOutputSampleRate[0]);

    for (i = 0; i < array_size; i++)
    {
        if (u32_sampleRate == s_u32_hdmiRxSupportedOutputSampleRate[i])
        {
            break;
        }
    }

    if (i < array_size)
    {
        return HAL_OK;
    }
    else
    {
        return HAL_HDMI_RX_FALSE;
    }
}


static uint32_t HDMI_RX_CheckAudioSampleRateChangeOrNot(ENUM_HAL_HDMI_RX e_hdmiIndex,uint32_t u32_sampleRate)
{
    if (e_hdmiIndex >= HAL_HDMI_RX_MAX)
    {
        return HAL_HDMI_RX_FALSE;
    }

    if ((s_st_hdmiRxStatus[e_hdmiIndex].u8_devEnable == 1) &&
        (s_e_hdmiRxAudioSampleRateStatus != (ENUM_HAL_I2S_IEC_SAMPLERATE)u32_sampleRate))
    {
        u32_audioSampleRateChangeCount++;
        u32_audioSampleRateChangeCount = 0;
        s_e_hdmiRxAudioSampleRateStatus = (ENUM_HAL_I2S_IEC_SAMPLERATE)u32_sampleRate;
        return HAL_OK;
    }
    else
    {
        u32_audioSampleRateChangeCount = 0;
        return HAL_HDMI_RX_FALSE;
    }
}

uint8_t HDMI_RX_CheckAudioStatus(ENUM_HAL_HDMI_RX e_hdmiIndex)
{
    uint32_t u32_sampleRate=0;
    uint8_t u8_hdmiIndex = HDMI_RX_MapToDeviceIndex(e_hdmiIndex);
    
    #ifdef USE_ADV7611_EDID_CONFIG_BIN
        ADV_7611_GetAudioSampleRate(u8_hdmiIndex, &u32_sampleRate);
    #endif

    #ifdef USE_IT66021_EDID_CONFIG_BIN
        if (FALSE == IT_66021_GetAudioSampleRate(u8_hdmiIndex, &u32_sampleRate))
        {
            return 0;
        }
    #endif
    if ( HDMI_RX_CheckAudioSampleRateSupportOrNot(u32_sampleRate) == HAL_OK)
    {
    
        if ((HDMI_RX_CheckAudioSampleRateChangeOrNot(e_hdmiIndex, u32_sampleRate) == HAL_OK))
        {

            STRU_SysEvent_AudioInputChangeParameter p;
            p.u8_audioSampleRate = u32_sampleRate;
            *HAL_hdmi_pu32_newAudioSampleRate = u32_sampleRate;

            SYS_EVENT_NotifyLocal(SYS_EVENT_ID_AUDIO_EVENT, (void*)&p);
            
            return 1;
        }
    }
    return 0;
}


static void HDMI_RX_IdleCallback0(void *paramPtr)
{
    if (s_st_hdmiRxStatus[HAL_HDMI_RX_0].u8_devEnable == 1)
    {

    #ifdef USE_IT66021_EDID_CONFIG_BIN
        if (s_st_hdmiRxStatus[HAL_HDMI_RX_0].st_configure.e_getFormatMethod == HAL_HDMI_POLLING)
        {
            IT_66021_IrqHandler0();
        }
        else
        {
            IT6602_fsm();
        }

        HDMI_RX_CheckAudioStatus(HAL_HDMI_RX_0);
    #endif

        HDMI_RX_CheckFormatStatus(HAL_HDMI_RX_0, HAL_HDMI_RX_FALSE);
    }
}

static void HDMI_RX_IdleCallback1(void *paramPtr)
{
    if (s_st_hdmiRxStatus[HAL_HDMI_RX_1].u8_devEnable == 1)
    {

    #ifdef USE_IT66021_EDID_CONFIG_BIN
        if (s_st_hdmiRxStatus[HAL_HDMI_RX_1].st_configure.e_getFormatMethod == HAL_HDMI_POLLING)
        {
            IT_66021_IrqHandler1();
        }
        else
        {
            IT6602_fsm();
        }

        HDMI_RX_CheckAudioStatus(HAL_HDMI_RX_1);
    #endif    
        HDMI_RX_CheckFormatStatus(HAL_HDMI_RX_1, HAL_HDMI_RX_FALSE);
    }
}


HAL_RET_T HAL_HDMI_RX_Init(ENUM_HAL_HDMI_RX e_hdmiIndex, 
                           STRU_HDMI_CONFIGURE *pst_hdmiConfigure)
{
    s_st_hdmiRxStatus[e_hdmiIndex].u8_devEnable = 1;
    memcpy(&(s_st_hdmiRxStatus[e_hdmiIndex].st_configure),pst_hdmiConfigure,sizeof(STRU_HDMI_CONFIGURE));

    SYS_EVENT_RegisterHandler(SYS_EVENT_ID_VIDEO_EVENT, HDMI_RxVideoHandle);
    SYS_EVENT_RegisterHandler(SYS_EVENT_ID_AUDIO_EVENT, HDMI_RxAudioHandle);    

    if (s_st_hdmiRxStatus[e_hdmiIndex].st_configure.e_getFormatMethod == HAL_HDMI_INTERRUPT)
    {
        HAL_NVIC_SetPriority(GPIO_INTR_N0_VECTOR_NUM + ((pst_hdmiConfigure->st_interruptGpio.e_interruptGpioNum)>>5),INTR_NVIC_PRIORITY_HDMI_GPIO,0);
        switch (e_hdmiIndex)
        {
            case HAL_HDMI_RX_0:
            {
                HAL_GPIO_RegisterInterrupt(s_st_hdmiRxStatus[e_hdmiIndex].st_configure.st_interruptGpio.e_interruptGpioNum, 
                                           s_st_hdmiRxStatus[e_hdmiIndex].st_configure.st_interruptGpio.e_interruptGpioType, 
                                           s_st_hdmiRxStatus[e_hdmiIndex].st_configure.st_interruptGpio.e_interruptGpioPolarity, HAL_HDMI_RX_IrqHandler0);
                break;
            }
            case HAL_HDMI_RX_1:
            {
                 HAL_GPIO_RegisterInterrupt(s_st_hdmiRxStatus[e_hdmiIndex].st_configure.st_interruptGpio.e_interruptGpioNum, 
                                           s_st_hdmiRxStatus[e_hdmiIndex].st_configure.st_interruptGpio.e_interruptGpioType, 
                                           s_st_hdmiRxStatus[e_hdmiIndex].st_configure.st_interruptGpio.e_interruptGpioPolarity, HAL_HDMI_RX_IrqHandler1);
                
                break;
            }
            default :
            {
                return HAL_HDMI_INPUT_COUNT;
            }

        }
    }

    #ifdef USE_ADV7611_EDID_CONFIG_BIN
        ADV_7611_Initial(HDMI_RX_MapToDeviceIndex(e_hdmiIndex));
    #endif

    #ifdef USE_IT66021_EDID_CONFIG_BIN
        if (FALSE == IT_66021_Initial(HDMI_RX_MapToDeviceIndex(e_hdmiIndex), s_st_hdmiRxStatus[e_hdmiIndex].st_configure.e_colorDepth))
        {
            return HAL_HDMI_RX_ERR_INIT;
        }
        SYS_EVENT_RegisterHandler(SYS_EVENT_ID_IDLE, (SYS_Event_Handler)IT6602_fsm);
    #endif

    if (s_st_hdmiRxStatus[e_hdmiIndex].st_configure.e_getFormatMethod == HAL_HDMI_POLLING)
    {
        SYS_EVENT_RegisterHandler(SYS_EVENT_ID_IDLE, e_hdmiIndex == HAL_HDMI_RX_0 ? HDMI_RX_IdleCallback0 : HDMI_RX_IdleCallback1);
    }
    
    if (s_st_hdmiRxStatus[e_hdmiIndex].st_configure.e_getFormatMethod != HAL_HDMI_POLLING || 
        s_st_hdmiRxStatus[e_hdmiIndex].st_configure.e_getFormatMethod != HAL_HDMI_INTERRUPT)
    {
        return HAL_HDMI_GET_ERR_GORMAT_METHOD;
    }

    

    return HAL_OK;
}


HAL_RET_T HAL_HDMI_RX_UnInit(ENUM_HAL_HDMI_RX e_hdmiIndex)
{
    
    #ifdef USE_IT66021_EDID_CONFIG_BIN
			SYS_EVENT_UnRegisterHandler(SYS_EVENT_ID_IDLE, (SYS_Event_Handler)IT6602_fsm);
    #endif

    if (s_st_hdmiRxStatus[e_hdmiIndex].st_configure.e_getFormatMethod == HAL_HDMI_POLLING)
    {
        SYS_EVENT_UnRegisterHandler(SYS_EVENT_ID_IDLE, e_hdmiIndex == HAL_HDMI_RX_0 ? HDMI_RX_IdleCallback0 : HDMI_RX_IdleCallback1);
    } else {
	HAL_GPIO_OutPut(s_st_hdmiRxStatus[e_hdmiIndex].st_configure.st_interruptGpio.e_interruptGpioNum);
    }
     
    s_st_hdmiRxStatus[e_hdmiIndex].u8_devEnable = 0;

    return HAL_OK;
}


HAL_RET_T HAL_HDMI_RX_GetVideoFormat(ENUM_HAL_HDMI_RX e_hdmiIndex, 
                                     uint16_t *pu16_width, 
                                     uint16_t *pu16_hight, 
                                     uint8_t *pu8_framterate)
{
    #ifdef USE_ADV7611_EDID_CONFIG_BIN
        ADV_7611_GetVideoFormat(HDMI_RX_MapToDeviceIndex(e_hdmiIndex), pu16_width, pu16_hight, pu8_framterate);
    #endif

    #ifdef USE_IT66021_EDID_CONFIG_BIN
        uint8_t vic;
        IT_66021_GetVideoFormat(HDMI_RX_MapToDeviceIndex(e_hdmiIndex), pu16_width, pu16_hight, pu8_framterate, &vic);
    #endif

    return HAL_OK;
}


HAL_RET_T HAL_HDMI_RX_GetAudioSampleRate(ENUM_HAL_HDMI_RX e_hdmiIndex, uint32_t *pu32_sampleRate)
{
    #ifdef USE_ADV7611_EDID_CONFIG_BIN
        ADV_7611_GetAudioSampleRate(HDMI_RX_MapToDeviceIndex(e_hdmiIndex), pu32_sampleRate);
    #endif

    #ifdef USE_IT66021_EDID_CONFIG_BIN
        IT_66021_GetAudioSampleRate(HDMI_RX_MapToDeviceIndex(e_hdmiIndex), pu32_sampleRate);
    #endif
    return HAL_OK;
}

static void HAL_HDMI_RX_IrqHandler0(uint32_t u32_vectorNum)
{

    #ifdef USE_ADV7611_EDID_CONFIG_BIN
        if (ADV_7611_IrqHandler0())
        {
            HDMI_RX_IdleCallback0(NULL);
        }
    #endif

    #ifdef USE_IT66021_EDID_CONFIG_BIN        
        IT_66021_IrqHandler0();
        HDMI_RX_IdleCallback0(NULL);
    #endif
}

static void HAL_HDMI_RX_IrqHandler1(uint32_t u32_vectorNum)
{
    #ifdef USE_ADV7611_EDID_CONFIG_BIN
        if (ADV_7611_IrqHandler1())
        {
            HDMI_RX_IdleCallback1(NULL);
        }
    #endif

    #ifdef USE_IT66021_EDID_CONFIG_BIN
    IT_66021_IrqHandler1();
    HDMI_RX_IdleCallback1(NULL);
    #endif
}


HAL_RET_T HAL_HDMI_RX_VideoCallbak(HAL_HDMI_RxHandle pfun_hdmiVideoHandle)
{
    SYS_EVENT_UnRegisterHandler(SYS_EVENT_ID_VIDEO_EVENT, HDMI_RxVideoHandle);
    if (NULL != pfun_hdmiVideoHandle)
    {
        HDMI_RxVideoHandle = pfun_hdmiVideoHandle;
    }
    SYS_EVENT_RegisterHandler(SYS_EVENT_ID_VIDEO_EVENT, HDMI_RxVideoHandle);
    
    return HAL_OK;
   
}

HAL_RET_T HAL_HDMI_RX_AudioCallbak(HAL_HDMI_RxHandle pfun_hdmiAudioHandle)
{
    SYS_EVENT_UnRegisterHandler(SYS_EVENT_ID_VIDEO_EVENT, HDMI_RxVideoHandle);
    if (NULL != pfun_hdmiAudioHandle)
    {
        HDMI_RxAudioHandle = pfun_hdmiAudioHandle;        
    }
    SYS_EVENT_RegisterHandler(SYS_EVENT_ID_AUDIO_EVENT, HDMI_RxAudioHandle);

    return HAL_OK;
}
