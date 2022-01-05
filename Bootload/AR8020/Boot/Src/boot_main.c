#include <string.h>
#include <stdint.h>
#include "boot_serial.h"
#include "boot_norflash.h"
#include "boot_core.h"
#include "image_struct.h"
#include "boot_systicks.h"
#include "boot_gpio.h"

//change version only change boot function(if boot no function upgrade keep it, don't change version to SDK version)
#define         BOOT_VERSION       "00.01.17"

STRU_Boot_Info st_bootInfo;


#if 0
void delay_ms(uint32_t num)
{
    volatile uint32_t i;

    for (i = 0; i < (num * 1000); i++)
    {
        ;
    }
}
#endif

int main(void)
{
    CPUINFO_ICacheEnable(1);
    QUAD_SPI_SetSpeed();

    memset(&st_bootInfo,0xff,sizeof(st_bootInfo));
    QUAD_SPI_ReadBlockByByte(BOOT_INFO_OFFSET,(uint8_t *)(&st_bootInfo),sizeof(st_bootInfo));

    uart_init(0,115200);
    BOOT_Printf("Boot Version: %s\r\n", BOOT_VERSION);
    BOOT_Printf("stag 1\r\n");

    BOOT_WatchDogInit();

#if 0
    //delay_ms(100);

    if('t' == uart_getc(0))
    {
        delay_ms(100);
        if('t' == uart_getc(0))
        {
            BOOT_Printf("into upgrade %x %x %x\r\n",st_bootInfo.success_boot,st_bootInfo.success_app,st_bootInfo.checkSum);
            if ((st_bootInfo.checkSum == (st_bootInfo.success_boot + st_bootInfo.success_app)) && (st_bootInfo.success_boot == 0))
            {
                BOOT_StartBoot(0);
            }
            else
            {
                BOOT_StartBoot(1);
            }
        }
    }
#endif

    
#ifndef UPGRADE_DUAL_APP_DESIGN_EN
    BOOT_Printf("boot app\r\n");
    if ((st_bootInfo.checkSum == (st_bootInfo.success_boot + st_bootInfo.success_app)) && (st_bootInfo.success_app == 0))
    {
        BOOT_StartApp(0);
    }
    else
    {
        BOOT_StartApp(1);
    }
#else
    if ((st_bootInfo.checkSum == (st_bootInfo.success_boot + st_bootInfo.success_app)) && (st_bootInfo.success_app == 1))
    {
        BOOT_Printf("boot app1\r\n");
        BOOT_StartApp(1);
    }
    else
    {
    	BOOT_Printf("boot app0\r\n");
        BOOT_StartApp(0);
    }

#endif
}

