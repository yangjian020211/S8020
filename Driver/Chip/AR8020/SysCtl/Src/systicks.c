#include "systicks.h"
#include "ar8020.h"
#include "pll_ctrl.h"
#include "cpu_info.h"

static volatile uint32_t g_u32SysTickCount = 0;
static volatile uint32_t g_u32SysTickLoad = 0;

/**
  * @brief This function is init system tick module.
  * @note ticks should be the CPU frequency, then the count interval should be 1ms.
  * @retval None
  */
uint8_t SysTicks_Init(uint32_t ticks)
{
    if ((ticks - 1UL) > SysTick_LOAD_RELOAD_Msk) { return (1UL); }    /* Reload value impossible */

    SysTick->LOAD  = (uint32_t)(ticks - 1UL);                         /* set reload register */
    SysTick->VAL   = 0UL;                                             /* Load the SysTick Counter Value */
    SysTick->CTRL  = SysTick_CTRL_CLKSOURCE_Msk |
                     SysTick_CTRL_TICKINT_Msk   |
                     SysTick_CTRL_ENABLE_Msk;                         /* Enable SysTick IRQ and SysTick Timer */

    g_u32SysTickLoad = (uint32_t)(ticks - 1UL);

    return (0UL);                                                     /* Function successful */
}

/**
  * @brief This function is uninit system tick module.
  * @note None
  * @retval None
  */
uint8_t SysTicks_UnInit(void)
{
    SysTick->CTRL  &= ~(SysTick_CTRL_CLKSOURCE_Msk |
                        SysTick_CTRL_TICKINT_Msk   |
                        SysTick_CTRL_ENABLE_Msk);                     /* Disable SysTick IRQ and SysTick Timer */

    return (0UL);                                                     /* Function successful */
}

/**
  * @brief This function is called to increment a global variable "g_u32SysTickCount"
  *        used as application time base.
  * @note In the default implementation, this variable is incremented each in Systick ISR.
  * @retval None
  */
void SysTicks_IncTickCount(void)
{
    g_u32SysTickCount++;
}

/**
  * @brief Provides a ms tick value
  * @note: Call xTaskGetTickCount instead if the FreeRTOS is running.  
  * @retval Tick value
  */
uint32_t SysTicks_GetTickCount(void)
{
    return g_u32SysTickCount;
}

/**
  * @brief Provides a us tick value
  * @note: Call xTaskGetTickCount instead if the FreeRTOS is running.  
  * @retval Tick value
  */
uint64_t SysTicks_GetUsTickCount(void)
{
    uint64_t val = (uint64_t)(SysTick->VAL);
    uint64_t tmp_SysTickCount = (uint64_t)(g_u32SysTickCount);

    uint16_t u16_pllClk;
    uint64_t ret;
    static uint64_t calc_SysTickCount = 0;
    static uint64_t pre_val = 0;
    
    PLLCTRL_GetCoreClk(&u16_pllClk, CPUINFO_GetLocalCpuId());

    if(calc_SysTickCount > tmp_SysTickCount)
    {
        if (val > pre_val)
        {
            calc_SysTickCount += 1;
        }
    }
    else
    {
        if((val > pre_val) && (calc_SysTickCount == tmp_SysTickCount))
        {
            calc_SysTickCount = tmp_SysTickCount + 1;
        }
        else
        {
            calc_SysTickCount = tmp_SysTickCount;
        }
    }

    ret = (uint64_t)(calc_SysTickCount * 1000 + (uint32_t)(g_u32SysTickLoad - val) / u16_pllClk);

    pre_val = val;

    return ret;
}

/**
  * @brief This function provides delay based on variable incremented.
  * @note In the default implementation , SysTick timer is the source of time base.
  *       It is used to generate interrupts at regular time intervals where u32_Tick
  *       is incremented.
  *       call vTaskDelay instead if the FreeRTOS is running.  
  * @param Delay: specifies the delay time length, in milliseconds.
  * @retval None
  */
void SysTicks_DelayMS(uint32_t msDelay)
{
    uint32_t tickstart = 0;
    uint32_t tickcurrent = 0;
    tickstart = g_u32SysTickCount;
    while(1)
    {
        tickcurrent = g_u32SysTickCount;
        if (tickcurrent >= tickstart)
        {
            if ((tickcurrent - tickstart) >= msDelay)
            {
                break;
            }
        }
        else
        {
            if (((MAX_SYS_TICK_COUNT - tickstart) + tickcurrent) >= msDelay)
            {
                break;
            }
        }
    }
}

/**
  * @brief This function provides delay based on variable incremented.
  * @note In the default implementation , SysTick timer is the source of time base.
  *       It is used to generate interrupts at regular time intervals where u32_Tick
  *       is incremented.
  *       call vTaskDelay instead if the FreeRTOS is running.  
  * @param Delay: specifies the delay time length, in microseconds.
  * @retval None
  */
void SysTicks_DelayUS(uint64_t usDelay)
{
    uint64_t tickstart = 0;
    uint64_t tickcurrent = 0;
    tickstart = SysTicks_GetUsTickCount();
    while(1)
    {
        tickcurrent = SysTicks_GetUsTickCount();
        if (tickcurrent >= tickstart)
        {
            if ((tickcurrent - tickstart) >= usDelay)
            {
                break;
            }
        }
        else
        {
            if (((((uint64_t)0xFFFFFFFFFFFFFFFF) - tickstart) + tickcurrent) >= usDelay)
            {
                break;
            }
        }
    }
}


/**
  * @brief Standard millisecond sleep API.
  * @note
  * @retval None
  */
void msleep(uint32_t millisecs)
{
    SysTicks_DelayMS(millisecs);
}

/**
  * @brief Standard second sleep API.
  * @note
  * @retval None
  */
void ssleep(uint32_t seconds)
{
    SysTicks_DelayMS(seconds * 1000);
}

/**
  * @brief get time difference.
  * @note
  * @retval time difference
  */
uint32_t SysTicks_GetDiff(uint32_t u32_start, uint32_t u32_end)
{
    if (u32_end >= u32_start)
    {
        return (u32_end - u32_start);
    }
    else
    {
       return ((MAX_SYS_TICK_COUNT - u32_start) + u32_end);
    }
}

uint64_t SysTicks_GetUsDiff(uint64_t u64_start, uint64_t u64_end)
{
    if (u64_end >= u64_start)
    {
        return (u64_end - u64_start);
    }
    else
    {
       return ((((uint64_t)0xFFFFFFFFFFFFFFFF) - u64_start) + u64_end);
    }
}

__attribute__((weak)) void ar_osDelay(uint32_t u32_ms)
{
    SysTicks_DelayMS(u32_ms);
}

