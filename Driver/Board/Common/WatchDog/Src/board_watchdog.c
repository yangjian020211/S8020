#include <stdio.h>
#include <string.h>
#include "debuglog.h"
#include "cpu_info.h"
#include "memory_config.h"
#include "systicks.h"
#include "hal_gpio.h"
#include "board_watchdog.h"
#include "sys_event.h"
#include "cmsis_os.h"
#include "interrupt.h"
#include "memory_config.h"
#include "inter_core.h"
#include "cm_backtrace.h"


#define GPIO_WATCHDOG               (HAL_GPIO_NUM42)
#define WATCHDOG_TIMEOUT_MS         (1600)
#define WATCHDOG_TRIGGER_DUMP_MS    (1500)


static uint32_t u32_cpu0WatchdogTime = 0;
static uint32_t u32_cpu1PreviousFeedTime = 0;
static uint32_t u32_cpu2PreviousFeedTime = 0;
static uint32_t u32_cpu1FeedWatchDogTime = 0;
static uint32_t u32_cpu2FeedWatchDogTime = 0;
static uint32_t u32_gpioState = 0;
static uint32_t u32_feedCount = 0;
static uint32_t u32_enableFlag = 0;

static uint32_t u32_cpu0PreviousFeedTime = 0;


void WATCHDOGUPGRADE_Reset(void);

static void local_irq_disable(void)
{
    __asm volatile(
                   "cpsid i                 @ local_irq_disable"
                   :
                   :
                   : "memory", "cc");

}

void WATCHDOG_Init(void)
{
    u32_enableFlag = 1;
    HAL_GPIO_OutPut(GPIO_WATCHDOG);
    WATCHDOGUPGRADE_Reset();
    SYS_EVENT_RegisterHandler(SYS_EVENT_ID_IDLE, (SYS_Event_Handler)WATCHDOG_Reset);
    
}

void WATCHDOG_InitUpgrade(void)
{
    HAL_GPIO_OutPut(GPIO_WATCHDOG);
    WATCHDOGUPGRADE_Reset();
}

void WATCHDOGUPGRADE_Reset(void)
{
    u32_gpioState++;
    u32_gpioState &= 1;
    HAL_GPIO_SetPin(GPIO_WATCHDOG, u32_gpioState);
    
    if (ENUM_CPU0_ID == CPUINFO_GetLocalCpuId())
    {
        u32_cpu0PreviousFeedTime = SysTicks_GetTickCount();
        *(volatile uint32_t *)SRAM_MODULE_WATCHDOG_BASE_TICK = u32_cpu0PreviousFeedTime;
    }
}

void WATCHDOG_Reset(void)
{
    uint8_t u8_cpuID = CPUINFO_GetLocalCpuId();
    uint32_t u32_timeDiff = 0;

    switch(u8_cpuID)
    {
        case 0:
        {
            u32_cpu0PreviousFeedTime = SysTicks_GetTickCount();
            *(volatile uint32_t *)SRAM_MODULE_WATCHDOG_BASE_TICK = u32_cpu0PreviousFeedTime;

            u32_timeDiff = SysTicks_GetDiff(u32_cpu0WatchdogTime, u32_cpu0PreviousFeedTime);

            u32_cpu1FeedWatchDogTime += u32_timeDiff;
            u32_cpu2FeedWatchDogTime += u32_timeDiff;

            if( u32_cpu1PreviousFeedTime != *((volatile uint32_t *)SRAM_MODULE_WATCHDOG_CPU1_TICK))
            {
                u32_cpu1PreviousFeedTime = *((volatile uint32_t *)SRAM_MODULE_WATCHDOG_CPU1_TICK);
                u32_cpu1FeedWatchDogTime = 0;
            }
            else
            {
                if(u32_cpu1FeedWatchDogTime > WATCHDOG_TIMEOUT_MS)
                {
                    local_irq_disable();
                    while(1);
                }
            }

            if( u32_cpu2PreviousFeedTime != *((volatile uint32_t *)SRAM_MODULE_WATCHDOG_CPU2_TICK))
            {
                u32_cpu2PreviousFeedTime = *((volatile uint32_t *)SRAM_MODULE_WATCHDOG_CPU2_TICK);
                u32_cpu2FeedWatchDogTime = 0;
            }
            else
            {
                if(u32_cpu2FeedWatchDogTime >  WATCHDOG_TIMEOUT_MS) 
                {
                    local_irq_disable();
                    while(1);
                }
            }   
            WATCHDOGUPGRADE_Reset();
            u32_cpu0WatchdogTime = SysTicks_GetTickCount();
            break;
        }

        case 1:
        {
            //check cpu0 watchdog
            if (u32_cpu0PreviousFeedTime != *((volatile uint32_t *)SRAM_MODULE_WATCHDOG_BASE_TICK))
            {
                u32_cpu0PreviousFeedTime = *((volatile uint32_t *)SRAM_MODULE_WATCHDOG_BASE_TICK);
                u32_cpu0WatchdogTime = SysTicks_GetTickCount();
            }
            else
            {
                u32_timeDiff = SysTicks_GetDiff(u32_cpu0WatchdogTime, SysTicks_GetTickCount());
                if (u32_timeDiff >= WATCHDOG_TRIGGER_DUMP_MS)
                {
                    InterCore_TriggerBacktraceIRQ1();
                    u32_timeDiff = 0;
                    u32_cpu0WatchdogTime = SysTicks_GetTickCount();
                }
            }

            u32_feedCount++;
            *((volatile uint32_t *)SRAM_MODULE_WATCHDOG_CPU1_TICK) = u32_feedCount;
            break;
        }

        default :
        {
            DLOG_Error("watch dog cpu id error %d", u8_cpuID);
            break;
        }
    }
}

uint32_t WATCHDOG_InitStatus(void)
{
    return u32_enableFlag;
}
