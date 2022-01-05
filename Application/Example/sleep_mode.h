#ifndef __SLEEP_MODE_H__
#define __SLEEP_MODE_H__

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "debuglog.h"

void command_EnterSleep(uint8_t *level);

void command_ExitSleep(void);

void SleepModeProcess(void *p);

void Mod_Pin_SleepTask(void const *argument);

void FemOn(uint8_t ch);

void FemOff(uint8_t ch);

void tx_bypass(int en);


#ifdef __cplusplus
}
#endif 

#endif
