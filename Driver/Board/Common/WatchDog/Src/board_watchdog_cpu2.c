#include <stdio.h>
#include <string.h>
#include "debuglog.h"
#include "memory_config.h"
#include "sys_event.h"
#include "board_watchdog.h"

static uint32_t u32_feedCount = 0;

void WATCHDOG_ResetCpu2(void)
{
    u32_feedCount++;
    *((volatile uint32_t *)SRAM_MODULE_WATCHDOG_CPU2_TICK) = u32_feedCount;
}

void WATCHDOG_InitCpu2(void)
{
    WATCHDOG_ResetCpu2();
    SYS_EVENT_RegisterHandler(SYS_EVENT_ID_IDLE, (SYS_Event_Handler)WATCHDOG_ResetCpu2);
}
