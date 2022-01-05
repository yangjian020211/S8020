#include <stdint.h>
#include <string.h>

#include "interrupt.h"
#include "serial.h"
#include "debuglog.h"

#include "hal_ret_type.h"
#include "hal_nvic.h"
#include "hal_norflash.h"
#include "hal_uart.h"
#include "image_struct.h"
#include "upgrade_command.h"
#include "upgrade_core.h"
#include "upgrade_uart.h"
#include "md5.h"
#include "efuse.h"

#include "board_watchdog.h"
#define UPGRADE_UART_DEBUGE

#ifdef  UPGRADE_UART_DEBUGE
#define UART_UPGRADE
#define DLOG_INFO(...) DLOG_Info(__VA_ARGS__)
#else
#define DLOG_INFO(...)
#endif

static volatile uint32_t g_u32RecCount;
static volatile uint32_t g_u32ImageSize;
static volatile uint8_t g_u32RecFlage;
static char *g_pDst = (char *)RECEIVE_ADDR;
static uint8_t g_u8Amd5Sum[MD5_SIZE];

static uint8_t g_u8EncryptFlag;

static void UPGRADE_InitParament(void)
{
    g_u32RecCount = 0;
    g_u32ImageSize=0x1000;
    g_u32RecFlage=1;
    memset(g_u8Amd5Sum,0,16);
} 


static void UPGRADE_IRQHandler(uint32_t vectorNum)
{
    uint32_t          u32_isrType;
    uint32_t          u32_isrType2;
    uint32_t          u32_status;
    volatile uart_type   *uart_regs =(uart_type *)UART0_BASE;

    u32_isrType    = uart_regs->IIR_FCR;

    if (UART_IIR_RECEIVEDATA == (u32_isrType & 0xf))
    {
        uint8_t i = UART_RX_FIFOLEN;
        while (i--)
        {
            *(g_pDst +g_u32RecCount) = uart_regs->RBR_THR_DLL;        
            g_u32RecCount++;
        }
    }

    if (UART_IIR_DATATIMEOUT == (u32_isrType & 0xf))
    {

        *(g_pDst +g_u32RecCount) = uart_regs->RBR_THR_DLL;        
        g_u32RecCount++;
    }

    // TX empty interrupt.
    if (UART_IIR_THR_EMPTY == (u32_isrType & UART_IIR_THR_EMPTY))
    {
        uart_putFifo(0);
    }
}

static void UPGRADE_RollbackIsrHandler(void)
{
    HAL_NVIC_RegisterHandler(HAL_NVIC_UART_INTR0_VECTOR_NUM, UART_IntrSrvc, NULL);
}

static void UPGRADE_UartReceive(void)
{
    //sdram init Done
    while(!(SDRAM_INIT_DONE & 0x01))
    {
        ;
    }

    HAL_NVIC_RegisterHandler(HAL_NVIC_UART_INTR0_VECTOR_NUM, UPGRADE_IRQHandler, NULL);

    DLOG_INFO("Nor flash init start ...\n");
    HAL_NORFLASH_Init();
    DLOG_INFO("Nor flash end\n");    
    DLOG_INFO("interrupt\n");    
    uint32_t i=0;
    uint32_t rec=1024*10;
    uint32_t tmp = 0;
    DLOG_Output(100);
        
    while(1)
    {
        tmp++;
        if (tmp > (0xfff*10))
        {
            tmp = 0;
            WATCHDOGUPGRADE_Reset();
        }
        if (g_u32ImageSize <= g_u32RecCount)
        {
            break;
        }

        if((1 == g_u32RecFlage)&&(g_u32RecCount>100))
        {
            g_u32RecFlage =0;

            uint8_t* p8_sizeAddr = (uint8_t*)(RECEIVE_ADDR);
            g_u32ImageSize = GET_WORD_FROM_ANY_ADDR(p8_sizeAddr);
           
            DLOG_INFO("imagesize %x",g_u32ImageSize);
            DLOG_Output(100);
        }

        if(rec < g_u32RecCount)
        {
            DLOG_INFO("rec data %d%%\n",g_u32RecCount*100/g_u32ImageSize);
            rec += 1024*10;
            DLOG_Output(100);
       }
    }
    DLOG_INFO("receive finish %x\n",g_u32RecCount);
    UPGRADE_RollbackIsrHandler();
}


void UPGRADE_Init(void)
{
    
    UPGRADE_InitParament();
    UPGRADE_UartReceive(); 

}

void UPGRADE_APPFromUart(void)
{
    UPGRADE_Init();   
    DLOG_INFO("start checksum receive data\n");
    if(-1 != UPGRADE_MD5SUM(RECEIVE_ADDR, 1))
    {
        UPGRADE_ModifyBootInfo(APPLICATION_IMAGE_START0);    
    }
    
}

void UPGRADE_BootloadFromUart(void)
{
    UPGRADE_Init();
    DLOG_INFO("start checksum receive data\n");
    if(-1 != UPGRADE_MD5SUM(RECEIVE_ADDR, 1))
    {
        UPGRADE_ModifyBootInfo(BOOT_ADDR0);    
    }
}
