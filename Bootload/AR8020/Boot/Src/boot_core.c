#include <stdint.h>
#include <string.h>
#include "boot_core.h"
#include "boot_serial.h"
#include "boot_gpio.h"
#include "boot_md5.h"
#include "image_struct.h"
#include "memory_config.h"
#include "boot_systicks.h"

extern STRU_Boot_Info st_bootInfo;

typedef struct
{
    uint32_t        magic_header;
    uint32_t        sky_or_grd;
    uint32_t        rsv[6];
} STRU_WIRELESS_RESET_DATA_RSV;

/****************************************
boot App
*****************************************/
#define CP_SIZE                  (1024*64)
void BOOT_ImageInfo(uint32_t u32_BaseAddr, uint8_t *pu8_md5, uint32_t *pu32_imageSize)
{

    uint8_t* p8_imageSizeAddr = (uint8_t*)(u32_BaseAddr + IMAGE_HAER_IMAGE_SIZE_OFFSET);
    *pu32_imageSize = GET_WORD_FROM_ANY_ADDR(p8_imageSizeAddr);

    uint8_t* p8_tmpAddr = (uint8_t*)(u32_BaseAddr + IMAGE_HAER_MD5_OFFSET);
    memcpy(pu8_md5, p8_tmpAddr, MD5_SIZE);

}
char  BOOT_HexToASCII(unsigned char  data_hex)
{
    char  ASCII_Data;
    ASCII_Data=data_hex & 0x0F;
    if(ASCII_Data<10)
        ASCII_Data=ASCII_Data+0x30; //‘0--9’
    else
        ASCII_Data=ASCII_Data+0x37; //‘A--F’
    return ASCII_Data;
}

void BOOT_HexGroupToString(unsigned int addr, unsigned int HexLength)
{

    uint8_t i=0;
    char OutStrBuffer[3];
    for(i=0;i<HexLength;i++)
    {

        OutStrBuffer[0]=BOOT_HexToASCII(((*((uint8_t*)(addr+i)))>>4)&0x0F);
        OutStrBuffer[1]=BOOT_HexToASCII((*((uint8_t*)(addr+i)))&0x0F);
        OutStrBuffer[2]='\0';
        BOOT_Printf("%s",OutStrBuffer);
    }
}

void BOOT_PrintInfo(uint32_t u32_addr)
{
    BOOT_Printf("Build Date:");
    BOOT_HexGroupToString(u32_addr + DATEY_OFFSET, 2);
    BOOT_Printf(" ");
    BOOT_HexGroupToString(u32_addr + DATEm_OFFSET, 2);
    BOOT_Printf(" ");
    BOOT_HexGroupToString(u32_addr + DATEH_OFFSET, 1);
    BOOT_Printf(":");
    BOOT_HexGroupToString(u32_addr + DATEM_OFFSET, 1);
    BOOT_Printf(":");
    BOOT_HexGroupToString(u32_addr + DATES_OFFSET, 1);
    BOOT_Printf("\r\nSDK Version:");
    BOOT_HexGroupToString(u32_addr + VERSION_MIAN_OFFSET, 1);
    BOOT_Printf(".");
    BOOT_HexGroupToString(u32_addr + VERSION_MAJOR_OFFSET, 1);
    BOOT_Printf(".");
    BOOT_HexGroupToString(u32_addr + VERSION_MINOR_OFFSET, 1);
    BOOT_Printf("\r\n");

}

void delay_ms(uint32_t num)
{
    volatile uint32_t i;

    for (i = 0; i < (num * 1000); i++)
    {
        ;
    }
}

extern void * imgcpy(void * __dest, void const * __src, size_t len);

void BOOT_BootApp(unsigned int address)
{
    uint32_t i = 0, j = 0;
    uint8_t* cpu0_app_size_addr = (uint8_t*)address;
    uint32_t cpu0_app_size = GET_WORD_FROM_ANY_ADDR(cpu0_app_size_addr);
    uint32_t cpu0_app_start_addr = address + 4;
  
    uint8_t* cpu1_app_size_addr = (uint8_t*)(cpu0_app_start_addr + cpu0_app_size);
    uint32_t cpu1_app_size = GET_WORD_FROM_ANY_ADDR(cpu1_app_size_addr);
    uint32_t cpu1_app_start_addr = cpu0_app_start_addr + cpu0_app_size + 4;

    uint8_t* cpu2_app_size_addr = (uint8_t*)(cpu1_app_start_addr + cpu1_app_size);
    uint32_t cpu2_app_size = GET_WORD_FROM_ANY_ADDR(cpu2_app_size_addr);
    uint32_t cpu2_app_start_addr = cpu1_app_start_addr + cpu1_app_size + 4;

    #pragma GCC diagnostic push
    #pragma GCC diagnostic ignored "-Wnonnull"
    j = cpu0_app_size/CP_SIZE;
    for (i = 0; i < j; i++)
    {
        BOOT_WatchDogFeed();
        imgcpy((void*)(ITCM0_START + i*CP_SIZE), (void*)(cpu0_app_start_addr + i*CP_SIZE), CP_SIZE);
    }
    imgcpy((void*)(ITCM0_START + i*CP_SIZE), (void*)(cpu0_app_start_addr + i*CP_SIZE), cpu0_app_size%CP_SIZE);    
    #pragma GCC diagnostic pop

    j = cpu1_app_size/CP_SIZE;
    for (i = 0; i < j; i++)
    {
        BOOT_WatchDogFeed();
        imgcpy((void*)(ITCM1_START + i*CP_SIZE), (void*)(cpu1_app_start_addr + i*CP_SIZE), CP_SIZE);
    }
    imgcpy((void*)(ITCM1_START + i*CP_SIZE), (void*)(cpu1_app_start_addr + i*CP_SIZE), cpu1_app_size%CP_SIZE);

    j = cpu2_app_size/CP_SIZE;
    for (i = 0; i < j; i++)
    {
        BOOT_WatchDogFeed();
        imgcpy((void*)(ITCM2_START + i*CP_SIZE), (void*)(cpu2_app_start_addr + i*CP_SIZE), CP_SIZE);
    }
    imgcpy((void*)(ITCM2_START + i*CP_SIZE), (void*)(cpu2_app_start_addr + i*CP_SIZE), cpu2_app_size%CP_SIZE);

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

    *((volatile uint32_t*)MCU1_CPU_WAIT) = 0;
    *((volatile uint32_t*)MCU2_CPU_WAIT) = 0;
    *((uint32_t*)MCU0_VECTOR_TABLE_REG) = 0;
    
    CPUINFO_ICacheInvalidate();
    CPUINFO_ICacheEnable(0);

    (*((void(*)())((*((uint32_t*)(ITCM0_START+4))))))();

}

void BOOT_StartBoot(uint8_t u8_flag)
{
    uint32_t u32_bootAddress = 0;
    if (0 == u8_flag)
    {
        u32_bootAddress = BOOT_ADDR0;
    }
    else
    {
        uart_puts(0,"boot code error, into backUp\r\n");
        u32_bootAddress = BOOT_ADDR1;
    }

    BOOT_PrintInfo(u32_bootAddress);

    #pragma GCC diagnostic push
    #pragma GCC diagnostic ignored "-Wnonnull"
    memcpy((void*)ITCM0_START, (void*)(u32_bootAddress+IMAGE_HAER_LENGTH), BOOT_SIZE);
    #pragma GCC diagnostic pop
    *((volatile uint32_t*)(MCU0_VECTOR_TABLE_REG)) = ITCM0_START;

    CPUINFO_ICacheInvalidate();
    CPUINFO_ICacheEnable(0);

    (*((void(*)())((*((uint32_t*)(ITCM0_START+4))))))();
}

void BOOT_StartApp(uint8_t u8_flag)
{
    uint32_t u32_bootAddress = 0;
    STRU_WIRELESS_RESET_DATA_RSV *p;

    if (0 == u8_flag)
    {
        uint8_t* p8_sizeAddr = (uint8_t*)(APPLICATION_IMAGE_START0+IMAGE_HAER_IMAGE_SIZE_OFFSET);
        uint32_t u32_imageSize=GET_WORD_FROM_ANY_ADDR(p8_sizeAddr)-IMAGE_HAER_LENGTH;
        uint8_t u8_modeGpioEnable = *((uint8_t *)(APPLICATION_IMAGE_START0 + VERSION_MODE_SWITCH));
        uint8_t u8_modeGpio = *((uint8_t *)(APPLICATION_IMAGE_START0 + VERSION_MODE_GPIO));
        BOOT_GPIOInPut(u8_modeGpio);
        BOOT_Printf("u8_modeGpioEnable=%d u8_modeGpio=%d gpio value=%d\r\n", u8_modeGpioEnable, u8_modeGpio, BOOT_GPIOGet(u8_modeGpio));

        switch (u8_modeGpioEnable)
        {
            case 1:
            {
                BOOT_Printf("mode gpio enable\r\n");
                if (BOOT_MODE_GND == BOOT_GPIOGet(u8_modeGpio))
                {
                    u32_bootAddress = APPLICATION_IMAGE_START0+IMAGE_HAER_LENGTH+u32_imageSize;
                    BOOT_Printf("gnd mode %x\r\n", u32_bootAddress);
                }
                else{
                    BOOT_Printf("sky mode\r\n");
                    u32_bootAddress = APPLICATION_IMAGE_START0;
                }
                break;
            }
            case 3:
            {
                u32_bootAddress = APPLICATION_IMAGE_START0+IMAGE_HAER_LENGTH+u32_imageSize;
                BOOT_Printf("mode gpio disable,start gnd mode\r\n");
                break;
            }
            case 0:
            case 2:
            {
                u32_bootAddress = APPLICATION_IMAGE_START0;
                BOOT_Printf("mode gpio disable,start sky mode\r\n");
                break;
            }
            default:
            {
                u32_bootAddress = APPLICATION_IMAGE_START0;
                BOOT_Printf("mode gpio disable,start default sky mode\r\n");
                break;
            }
        }

        p = (STRU_WIRELESS_RESET_DATA_RSV *)(SRAM_USR_GRD_SKY_SELECT_ST_ADDR);
        if(p->magic_header == 0x1234abcd)
        {
            BOOT_Printf("sram determine sky or grd\r\n");
            if(p->sky_or_grd == 0)
            {
                BOOT_Printf("sram force to sky mode\r\n");
                u32_bootAddress = APPLICATION_IMAGE_START0;
            }
            else if(p->sky_or_grd == 1)
            {
                u32_bootAddress = APPLICATION_IMAGE_START0+IMAGE_HAER_LENGTH+u32_imageSize;
                BOOT_Printf("sram force to gnd mode %x\r\n", u32_bootAddress);
            }
            else
            {
                BOOT_Printf("sram value error\r\n");
            }
        }
    }
    else
    {

#ifdef UPGRADE_DUAL_APP_DESIGN_EN

	uint8_t* p8_sizeAddr = (uint8_t*)(APPLICATION_IMAGE_START1 + IMAGE_HAER_IMAGE_SIZE_OFFSET);
	uint32_t u32_imageSize = GET_WORD_FROM_ANY_ADDR(p8_sizeAddr)- IMAGE_HAER_LENGTH;
	uint8_t u8_modeGpioEnable = *((uint8_t *)(APPLICATION_IMAGE_START1 + VERSION_MODE_SWITCH));
	uint8_t u8_modeGpio = *((uint8_t *)(APPLICATION_IMAGE_START1 + VERSION_MODE_GPIO));
	BOOT_GPIOInPut(u8_modeGpio);
	BOOT_Printf("u8_modeGpioEnable=%d u8_modeGpio=%d gpio value=%d\r\n", u8_modeGpioEnable, u8_modeGpio, BOOT_GPIOGet(u8_modeGpio));

	switch (u8_modeGpioEnable)
	{
	    	case 1:
	    	{
			BOOT_Printf("mode gpio enable\r\n");
			if (BOOT_MODE_GND == BOOT_GPIOGet(u8_modeGpio))
			{
		    		u32_bootAddress = APPLICATION_IMAGE_START1+IMAGE_HAER_LENGTH+u32_imageSize;
		    		BOOT_Printf("gnd mode %x\r\n", u32_bootAddress);
			}
			else{
		    		BOOT_Printf("sky mode\r\n");
		    		u32_bootAddress = APPLICATION_IMAGE_START1;
			}
			break;
	    	}
	    	case 3:
	    	{
			u32_bootAddress = APPLICATION_IMAGE_START1+IMAGE_HAER_LENGTH+u32_imageSize;
			BOOT_Printf("mode gpio disable,start gnd mode\r\n");
			break;
	    	}
		case 0:
		case 2:
		{
			u32_bootAddress = APPLICATION_IMAGE_START1;
			BOOT_Printf("mode gpio disable,start sky mode\r\n");
			break;
		}
		default:
		{
			u32_bootAddress = APPLICATION_IMAGE_START1;
			BOOT_Printf("mode gpio disable,start default sky mode\r\n");
			break;
	    	}
	}
	
	p = (STRU_WIRELESS_RESET_DATA_RSV *)(SRAM_USR_GRD_SKY_SELECT_ST_ADDR);
	if(p->magic_header == 0x1234abcd)
	{
	    BOOT_Printf("sram determine sky or grd\r\n");
	    if(p->sky_or_grd == 0)
	    {
		BOOT_Printf("sram force to sky mode\r\n");
		u32_bootAddress = APPLICATION_IMAGE_START1;
	    }
	    else if(p->sky_or_grd == 1)
	    {
		u32_bootAddress = APPLICATION_IMAGE_START1+IMAGE_HAER_LENGTH+u32_imageSize;
		BOOT_Printf("sram force to gnd mode %x\r\n", u32_bootAddress);
	    }
	    else
	    {
		BOOT_Printf("sram value error\r\n");
	    }
	}

#else
        u32_bootAddress = APPLICATION_IMAGE_START1;
        BOOT_Printf("app code error, into backUp sky mode ,image information:\r\n");
        BOOT_PrintInfo(u32_bootAddress);
        BOOT_Printf("app code error, into backUp sky mode ,image information:\r\n");
        BOOT_PrintInfo(u32_bootAddress);
        BOOT_Printf("app code error, into backUp sky mode ,image information:\r\n");
        BOOT_PrintInfo(u32_bootAddress);
        BOOT_Printf("app code error, into backUp sky mode ,image information:\r\n");
        BOOT_PrintInfo(u32_bootAddress);
        BOOT_Printf("app code error, into backUp sky mode ,image information:\r\n");
        BOOT_PrintInfo(u32_bootAddress);
        BOOT_Printf("app code error, into backUp sky mode ,image information:\r\n");
#endif
    }
    BOOT_WatchDogFeed();
    BOOT_PrintInfo(u32_bootAddress);
    BOOT_WatchDogFeed();
    BOOT_BootApp(u32_bootAddress+IMAGE_HAER_LENGTH);
}

uint8_t BOOT_CheckCode(uint32_t u32_BaseAddr)
{
    uint8_t i=0;
    uint8_t *u8_pdata = (uint8_t *)(u32_BaseAddr + IMAGE_HAER_LENGTH);
    uint8_t md5_value[16];
    uint8_t u8_Amd5Sum[16];
    uint32_t u32_imageSize = 0;

    uint8_t* p8_imageSizeAddr = (uint8_t*)(u32_BaseAddr + IMAGE_HAER_IMAGE_SIZE_OFFSET);
    u32_imageSize = GET_WORD_FROM_ANY_ADDR(p8_imageSizeAddr) - IMAGE_HAER_LENGTH;


    uint8_t* p8_md5Addr = (uint8_t*)(u32_BaseAddr + IMAGE_HAER_MD5_OFFSET);
    memcpy(u8_Amd5Sum, p8_md5Addr, MD5_SIZE);

    MD5_CTX md5;
    MD5Init(&md5);
    MD5Update(&md5, u8_pdata, u32_imageSize);
    MD5Final(&md5, md5_value);

    for(i=0;i<MD5_SIZE;i++)
    {
        if(md5_value[i] != u8_Amd5Sum[i])
        {
            return 1;
        }
    }
    return 0;
}
