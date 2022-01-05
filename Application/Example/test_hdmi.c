#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "debuglog.h"
#include "hal_hdmi_rx.h"
#include "sys_event.h"

uint8_t hdmi_status = 0;

uint8_t get_hdmi_status(void)
{
    return hdmi_status;
}

void HDMI_RX_VideoCallbakSample(void *p)
{
    uint16_t u16_width;
    uint16_t u16_hight;
    uint8_t u8_framerate;
    HAL_RET_T ret;
    
    STRU_SysEvent_H264InputFormatChangeParameter *param = (STRU_SysEvent_H264InputFormatChangeParameter*)p;
    DLOG_Info("video width=%d hight=%d framerate=%d", ((STRU_SysEvent_H264InputFormatChangeParameter*)p)->width, 
                                                      ((STRU_SysEvent_H264InputFormatChangeParameter*)p)->hight, 
                                                      ((STRU_SysEvent_H264InputFormatChangeParameter*)p)->framerate);
    
    if(param->width == 0 && param->hight == 0 && param->framerate == 0)
    {
        u16_width=0;u16_hight=0;u8_framerate=0;
        ret = HAL_HDMI_RX_GetVideoFormat(HAL_HDMI_RX_1, &u16_width, &u16_hight, &u8_framerate);
        if(ret != HAL_OK)
        {
#ifdef USE_IT66021_EDID_CONFIG_BIN
            IT_66021_reset(0);
#endif
            DLOG_Warning("reset hdmi, ret=%x",ret);
            hdmi_status = 0;
            return;
        }

        if(u16_width==0 && u16_hight==0 && u8_framerate == 0)
        {
            // notfiy encoder
            HDMI_RxVideoNotifyH264Endcoder(p);
            hdmi_status = 0;
            return;
        }

        ret = HDMI_RX_CheckVideoFormatSupportOrNot(u16_width, u16_hight, u8_framerate);
        if(ret != HAL_OK)
        {
#ifdef USE_IT66021_EDID_CONFIG_BIN
            IT_66021_reset(0);
#endif
            DLOG_Warning("reset hdmi, W=%d,H=%d,F=%d",u16_width, u16_hight, u8_framerate);
            hdmi_status = 0;
            return;
        }
    }
    else
    {
        ret = HDMI_RX_CheckVideoFormatSupportOrNot(param->width ,param->hight,param->framerate);
        if(ret != HAL_OK)
        {
#ifdef USE_IT66021_EDID_CONFIG_BIN
            IT_66021_reset(0);
#endif
            DLOG_Warning("reset hdmi, W=%d,H=%d,F=%d",u16_width, u16_hight, u8_framerate);
            hdmi_status = 0;
            return;
        }

    }

    // notfiy encoder
    hdmi_status = 1;
    HDMI_RxVideoNotifyH264Endcoder(p);
}

