#include <stdlib.h>
#include "debuglog.h"
#include "gpio.h"
#include "bb_ctrl_internal.h"
#include "filter.h"
#include "rf_if.h"

#define     GPIO_NUM73          (73) //Pin7
#define     GPIO_NUM69          (69) //Pin6
#define     GPIO_NUM65          (65) //Pin5

static uint8_t work_mode = 0; //0:auto 1:manual
static uint32_t pin[3] = {GPIO_NUM73, GPIO_NUM69, GPIO_NUM65};
static uint32_t pinValue[6][3] = { {0, 1, 0}, {1, 0, 0}, {0, 0, 0}, {1, 0, 1}, {0, 1, 1}, {0, 0, 1} };


int BB_FilterInit(void)
{
    uint32_t i = 0;

    for(i=0; i<3; i++)
    {
        GPIO_SetMode(pin[i], GPIO_MODE_2);
        GPIO_SetPinDirect(pin[i], GPIO_DATA_DIRECT_OUTPUT);
        GPIO_SetPinCtrl(pin[i], GPIO_CTRL_SOFTWARE);
        GPIO_SetPin(pin[i], pinValue[FILTER_NONE][i]);
    }
    
    return 0;
}


int BB_FilterSet(ENUM_BB_FILTER filter, uint8_t print)
{
    uint32_t i = 0;

    if(1 == work_mode)
    {
        if(print)
        {
            DLOG_Info("filter in manual mode ... ");
        }
        return -1;
    }
    
    if(filter <= FILTER_NONE)
    {
        for(i=0; i<3; i++)
        {
            GPIO_SetPin(pin[i], pinValue[filter][i]);
        }
        if(print)
        {
            DLOG_Warning("filter:%d pin:%d = %d, pin:%d = %d, pin:%d = %d", 
                filter, pin[0], pinValue[filter][0], pin[1], pinValue[filter][1], pin[2], pinValue[filter][2]);
        }
    }
    else
    {
        for(i=0; i<3; i++)
        {
            GPIO_SetPin(pin[i], pinValue[FILTER_NONE][i]);
        }
        if(print)
        {
            DLOG_Warning("err filter:%d pin:%d = %d, pin:%d = %d, pin:%d = %d", 
                filter, pin[0], pinValue[FILTER_NONE][0], pin[1], pinValue[FILTER_NONE][1], pin[2], pinValue[FILTER_NONE][2]);
        }
    }

    return 0;
}


ENUM_BB_FILTER BB_GetFilterByRcFrqCh(uint8_t ch)
{
    uint16_t frq = BB_GetRcFrqByCh(ch);
    ENUM_BB_FILTER filter = BB_GetFilterByFrq(frq/10);
    //DLOG_Warning("ch:%d frq:%d filter:%d", ch, frq, filter);
    
    return filter;
}

ENUM_BB_FILTER BB_GetFilterByVtFrqCh(uint8_t ch)
{
    uint16_t frq = BB_GetItFrqByCh(ch);
    ENUM_BB_FILTER filter = BB_GetFilterByFrq(frq/10);
    //DLOG_Warning("ch:%d frq:%d filter:%d", ch, frq, filter);

    return filter;
}

ENUM_BB_FILTER BB_GetFilterByFrq(uint16_t frq)
{
    ENUM_BB_FILTER ret = FILTER_NONE;

    if((frq > 624) && (frq <= 656))
    {
        ret = FILTER_0;
    }
    else if((frq > 656) && (frq <= 687))
    {
        ret = FILTER_1;
    }
    else if((frq > 687) && (frq <= 720))
    {
        ret = FILTER_2;
    }
    else if((frq > 720) && (frq <= 752))
    {
        ret = FILTER_3;
    }
    else if((frq > 752) && (frq <= 788))
    {
        ret = FILTER_4;
    }

    return ret;
}

int BB_SetFilterWorkMode(uint8_t value)
{
    if(value <= FILTER_NONE)
    {
        work_mode = 0;
        BB_FilterSet(value, 1);
        work_mode = 1;
    }
    else
    {
        work_mode = 0;
    }

}
