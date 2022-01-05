#include "boot_systicks.h"
#include "boot_regmap.h"

static volatile uint32_t g_u32SysTickCount = 0;

#define __ASM            __asm                                        /*!< asm keyword for GNU Compiler          */
#define __STATIC_INLINE  static inline


/* Memory mapping of Cortex-M4 Hardware */
#define SCS_BASE            (0xE000E000UL)                            /*!< System Control Space Base Address  */
#define ITM_BASE            (0xE0000000UL)                            /*!< ITM Base Address                   */
#define DWT_BASE            (0xE0001000UL)                            /*!< DWT Base Address                   */
#define TPI_BASE            (0xE0040000UL)                            /*!< TPI Base Address                   */
#define CoreDebug_BASE      (0xE000EDF0UL)                            /*!< Core Debug Base Address            */
#define SysTick_BASE        (SCS_BASE +  0x0010UL)                    /*!< SysTick Base Address               */
#define NVIC_BASE           (SCS_BASE +  0x0100UL)                    /*!< NVIC Base Address                  */
#define SCB_BASE            (SCS_BASE +  0x0D00UL)                    /*!< System Control Block Base Address  */


#define SCB_CCR_IC_Pos                      17                                            /*!< SCB CCR: Instruction cache enable bit Position */
#define SCB_CCR_IC_Msk                     (1UL << SCB_CCR_IC_Pos)                        /*!< SCB CCR: Instruction cache enable bit Mask */
#define SCB                                 ((SCB_Type       *)     SCB_BASE      )       /*!< SCB configuration struct           */

/**
  \brief   Data Synchronization Barrier
  \details Acts as a special kind of Data Memory Barrier.
           It completes when all explicit memory accesses before this instruction complete.
 */
__attribute__((always_inline)) __STATIC_INLINE void __DSB(void)
{
  __ASM volatile ("dsb 0xF":::"memory");
}

/**
  \brief   Instruction Synchronization Barrier
  \details Instruction Synchronization Barrier flushes the pipeline in the processor,
           so that all instructions following the ISB are fetched from cache or memory,
           after the instruction has been completed.
 */
__attribute__((always_inline)) __STATIC_INLINE void __ISB(void)
{
  __ASM volatile ("isb 0xF":::"memory");
}



/** \brief Enable I-Cache

    The function turns on I-Cache
  */
__STATIC_INLINE void SCB_EnableICache (void)
{
    __DSB();
    __ISB();
    SCB->ICIALLU = 0UL;                     // invalidate I-Cache
    SCB->CCR |=  (uint32_t)SCB_CCR_IC_Msk;  // enable I-Cache
    __DSB();
    __ISB();
}


/** \brief Disable I-Cache

    The function turns off I-Cache
  */
__STATIC_INLINE void SCB_DisableICache (void)
{
  #if (__ICACHE_PRESENT == 1)
    __DSB();
    __ISB();
    SCB->CCR &= ~(uint32_t)SCB_CCR_IC_Msk;  // disable I-Cache
    SCB->ICIALLU = 0UL;                     // invalidate I-Cache
    __DSB();
    __ISB();
  #endif
}


/** \brief Invalidate I-Cache

    The function invalidates I-Cache
  */
__STATIC_INLINE void SCB_InvalidateICache (void)
{
    __DSB();
    __ISB();
    SCB->ICIALLU = 0UL;
    __DSB();
    __ISB();
}



void CPUINFO_ICacheEnable(uint8_t u8_icacheEnable)
{
    static uint8_t s_u8_icacheEnable = 0;

    if (u8_icacheEnable != 0)
    {
        /* Enable I-Cache */
        SCB_EnableICache();
        s_u8_icacheEnable = 1;
    }
    else
    {
        if (s_u8_icacheEnable != 0)
        {
            /* Disable I-Cache */
            SCB_DisableICache();
            s_u8_icacheEnable = 0;
        }
    }
}

void CPUINFO_ICacheInvalidate(void)
{
    SCB_InvalidateICache();
}


/**
  * @brief This function is init system tick module.
  * @note ticks should be the CPU frequency, then the count interval should be 1ms.
  * @retval None
  */
void SysTicks_Init(uint32_t ticks)
{
    if ((ticks - 1UL) > SysTick_LOAD_RELOAD_Msk) { return; }    /* Reload value impossible */

    SysTick->LOAD  = (uint32_t)(ticks - 1UL);                         /* set reload register */
    SysTick->VAL   = 0UL;                                             /* Load the SysTick Counter Value */
    SysTick->CTRL  = SysTick_CTRL_CLKSOURCE_Msk |
                     SysTick_CTRL_TICKINT_Msk   |
                     SysTick_CTRL_ENABLE_Msk;                         /* Enable SysTick IRQ and SysTick Timer */

    return ;                                                     /* Function successful */
}

/**
  * @brief This function is uninit system tick module.
  * @note None
  * @retval None
  */
void SysTicks_UnInit(void)
{
    SysTick->CTRL  &= ~(SysTick_CTRL_CLKSOURCE_Msk |
                        SysTick_CTRL_TICKINT_Msk   |
                        SysTick_CTRL_ENABLE_Msk);                     /* Disable SysTick IRQ and SysTick Timer */

    return ;                                                     /* Function successful */
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
  * @brief Provides a tick value
  * @note: Call xTaskGetTickCount instead if the FreeRTOS is running.
  * @retval Tick value
  */
uint32_t SysTicks_GetTickCount(void)
{
    return g_u32SysTickCount;
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

