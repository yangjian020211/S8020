/*****************************************************************************
Copyright: 2016-2020, Artosyn. Co., Ltd.
File name: test_search_id
Description: 
Author: Artosy Software Team
Version: 0.0.1
Date: 2017/12/14
History:
         0.0.1    2017/12/14    test_search_id
*****************************************************************************/

#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include "debuglog.h"
#include "hal_ret_type.h"
#include "hal_timer.h"
#include "hal_nv.h"
#include "sys_event.h"
#include "hal_bb.h"
#include "data_type.h"
#include "test_search_id_2.h"
#include "hal_gpio.h"
#include "hal.h"
#include "test_bb_led_ctrl_2.h"

uint32_t flag_searchIdTimerStart = 0;
uint8_t vt_id_timer_start_flag=0;

void BB_skyRcIdEventHandler(void *p)
{
    STRU_SysEvent_DEV_BB_STATUS *pstru_status = (STRU_SysEvent_DEV_BB_STATUS *)p;
    uint8_t id[7];

    if(pstru_status->pid == BB_GET_RCID)
    {
        DLOG_Critical("Get rcid: 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x,rssi_a %d, rssi_b %d", pstru_status->rcid[0], pstru_status->rcid[1], 
                                                                       pstru_status->rcid[2], pstru_status->rcid[3], pstru_status->rcid[4],
                                                                       pstru_status->agc1,pstru_status->agc2);
    }

    if (pstru_status->pid == BB_SKY_SEARCHING_STATES_CHAGE)
    {
        if (SKY_WAIT_VT_ID_MSG == pstru_status->e_sky_searchState)
        {
            DLOG_Warning("search id: SKY_WAIT_VT_ID_MSG");
        }
        else if (SKY_WAIT_RC_ID_MATCH == pstru_status->e_sky_searchState)
        {
            memcpy(id,pstru_status->vtid,2);
            memcpy(id+2,pstru_status->rcid,5);
            add_dev_info(id,pstru_status->agc1,pstru_status->agc2);
            
            if(!vt_id_timer_start_flag)
            {
                vt_id_timer_start_flag = 1;
                HAL_TIMER_Start(GET_VT_ID_TIMER);
                DLOG_Warning("got vtid: %dus timer start",GET_VT_ID_TIMEOUT);
            }
            //DLOG_Warning("search id: SKY_WAIT_RC_ID_MATCH");
        }
        else if (SKY_RC_SEARCH_END == pstru_status->e_sky_searchState)
        {
            DLOG_Warning("search id: SKY_RC_SEARCH_END rc id: %x %x %x %x %x; vt id: %x %x", 
                          pstru_status->rcid[0], pstru_status->rcid[1], pstru_status->rcid[2], pstru_status->rcid[3], pstru_status->rcid[4],
                          pstru_status->vtid[0], pstru_status->vtid[1]);

            if (flag_searchIdTimerStart)
            {
                HAL_TIMER_Stop(SEARCH_ID_TIMER);
                flag_searchIdTimerStart = 0;
                vt_id_timer_start_flag = 0;
            }
        }
    }
}

uint8_t timeout_loop = 0;
void TIMHAL_IRQSearchIdHandler(uint32_t u32_vectorNum)
{
    if (timeout_loop ++ > 6)
    {
        DLOG_Warning("search time out %d us", SEARCH_ID_TIMEOUT*timeout_loop);
        HAL_TIMER_Stop(SEARCH_ID_TIMER);
        flag_searchIdTimerStart = 0;
        HAL_BB_StopSearchRcId();
        timeout_loop = 0;
        vt_id_timer_start_flag = 0;
        set_link_led_status(LINK_UNLOCK);
    }
}

void TIMHAL_IRQGetVtIdHandler(uint32_t u32_vectorNum)
{
    uint8_t id[7];

    get_nearest_dev_info(id);
    HAL_BB_SetObjRcVtId(id+2,id);
    HAL_TIMER_Stop(GET_VT_ID_TIMER);
}

void BB_Sky_SearchIdHandler(void *p)
{
    if (flag_searchIdTimerStart)
    {
        HAL_TIMER_Stop(SEARCH_ID_TIMER);
        flag_searchIdTimerStart = 0;
    }

    if (HAL_OK == HAL_TIMER_RegisterTimer(SEARCH_ID_TIMER, SEARCH_ID_TIMEOUT, TIMHAL_IRQSearchIdHandler))
    {
        flag_searchIdTimerStart = 1;
    }
    else
    {
        DLOG_Critical("HAL_TIMER_RegisterTimer fail");
    }

    if(vt_id_timer_start_flag)
    {
        HAL_TIMER_Stop(GET_VT_ID_TIMER);
        vt_id_timer_start_flag = 0;
    }
    if (HAL_OK == HAL_TIMER_RegisterTimer(GET_VT_ID_TIMER, GET_VT_ID_TIMEOUT, TIMHAL_IRQGetVtIdHandler))
    {
        //flag_searchIdTimerStart = 1;
        HAL_TIMER_Stop(GET_VT_ID_TIMER);
        vt_id_timer_start_flag = 0;
    }
    else
    {
        DLOG_Critical("HAL_TIMER_RegisterTimer fail1");
    }
    
    timeout_loop = 0;
    
    reset_dev_info();

    HAL_TIMER_Start(SEARCH_ID_TIMER);
    HAL_BB_EnableSearchRcIdByRssi();

    HAL_BB_SearchRcId(1);

    set_link_led_status(LINK_SEARCH_ID);
    DLOG_Critical("search id enter");
}

void search_id(uint8_t dev_type)
{
    if(dev_type == 0)
    {
        BB_Sky_SearchIdHandler(NULL);
    }
}

void Mod_Sky_Pin_SearchIdTask(void const *argument)
{
    uint8_t cnt = 0,key_release = 1;
    uint32_t pin_value;
    
    HAL_GPIO_SetMode(EXTERN_SEARCH_ID_PIN, HAL_GPIO_PIN_MODE2);  
    HAL_GPIO_InPut(EXTERN_SEARCH_ID_PIN);
    while (1)
    {
        HAL_GPIO_GetPin(EXTERN_SEARCH_ID_PIN,&pin_value);
        if(pin_value == 0)
        {
            cnt++;
            if((cnt >= KEY_LONG_PRESS / READ_INTERVAL) && key_release)
            {
                if(!flag_searchIdTimerStart)
                {
                    DLOG_Warning("pin search id start");
                    BB_Sky_SearchIdHandler(NULL);
                    set_link_led_status(LINK_SEARCH_ID);
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
