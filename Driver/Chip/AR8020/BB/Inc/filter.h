#ifndef __FILTER_H__
#define __FILTER_H__

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "debuglog.h"

typedef enum
{
    FILTER_0,       // TA0963A 624---650
    FILTER_1,       // TA0616A 662.5--687.5
    FILTER_2,       // TA0617B 687.5-712.5
    FILTER_3,       // TA2037A 729--746
    FILTER_4,       // TA1897A 758--788
    FILTER_NONE,    //600 ~ 800  
} ENUM_BB_FILTER;

int BB_FilterInit(void);

int BB_FilterSet(ENUM_BB_FILTER filter, uint8_t print);

ENUM_BB_FILTER BB_GetFilterByRcFrqCh(uint8_t ch);

ENUM_BB_FILTER BB_GetFilterByVtFrqCh(uint8_t ch);

ENUM_BB_FILTER BB_GetFilterByFrq(uint16_t frq);

int BB_SetFilterWorkMode(uint8_t value);

#ifdef __cplusplus
}
#endif 

#endif
