/*****************************************************************************
Copyright: 2016-2020, Artosyn. Co., Ltd.
File name: hal_softi2s.c
Description: this module contains the helper fucntions necessary to control the general
             purpose softi2s block.softi2s use gpio to read i2s data.
             audio data buff limit 1M (AUDIO_DATA_END-AUDIO_DATA_START).
NOTE: this file don't use -O1 to complie
Author: Artosy Software Team
Version: 0.0.1
Date: 2017/02/21
History:
         0.0.1    2017/02/21    The initial version of hal_softi2s.c
*****************************************************************************/

#include <stdio.h>
#include <stdint.h>
#include <string.h>

#include "cpu_info.h"
#include "pll_ctrl.h"
#include "systicks.h"
#include "debuglog.h"
#include "hal_ret_type.h"
#include "hal_sys_ctl.h"
#include "hal_nvic.h"
#include "hal_gpio.h"
#include "hal_softi2s.h"
#include "dma.h"
#include "hal_usb_device.h"
#include "memory_config.h"
#include "inter_core.h"
#include "gpio.h"  
#include "reg_rw.h"
#include "board_watchdog.h"

volatile uint16_t g_u16_audioDataArray[ADUIO_DATA_BUFF_LENGHT]={0};
volatile uint32_t g_u32_audioDataConut=0;
volatile uint32_t g_u32_audioDataReady=1;
volatile uint8_t  g_u8_audioDataOffset=0;
volatile uint32_t g_u32_audioDataAddr;
volatile uint32_t g_u32_audioLeftInterruptAddr=0;
volatile uint32_t g_u32_audioRightInterruptAddr=0;
volatile uint32_t g_u32_audioFlage=1;
volatile uint32_t g_u32_audioDataAddr_out;
static volatile uint32_t g_u32_dstAddress = AUDIO_DATA_START;
#define LA_GPIO (90)

static void i2s_GPIO_ClearNvic(uint32_t u32_vectorNum)
{
    
    uint32_t i = 0;
    for (i=(u32_vectorNum-66)*32; i<8+(u32_vectorNum-66)*32; i++)
    {
        
        if(GPIO_Intr_GetIntrStatus(i))
        {
            GPIO_Intr_ClearIntr(i);
        }
    }
}

uint32_t GPIO_GetAddr(uint32_t gpioNum)
{
    
    uint32_t u32_RegNoAddr = 0;
    uint32_t u32_GroupNoAddr = 0;
    uint8_t u8_PinNo = ((gpioNum%32)%8);
    
    switch((gpioNum>>5))
    {
        case 0:
        {
            u32_GroupNoAddr = GPIO0_BASE_ADDR;
            break;
        }
        case 1:
        {
            u32_GroupNoAddr = GPIO1_BASE_ADDR;
            break;
        }
        case 2:
        {
            u32_GroupNoAddr = GPIO2_BASE_ADDR;
            break;
        }
        case 3:
        {
            u32_GroupNoAddr = GPIO3_BASE_ADDR;
            break;
        }

    }
    switch(((gpioNum%32)>>3))
    {
        case 0:
        {
            u32_RegNoAddr = GPIO_DATA_A_OFFSET;
            break;
        }
        case 1:
        {
            u32_RegNoAddr = GPIO_DATA_B_OFFSET;
            break;
        }
        case 2:
        {
            u32_RegNoAddr = GPIO_DATA_C_OFFSET;
            break;
        }
        case 3:
        {
            u32_RegNoAddr = GPIO_DATA_D_OFFSET;
            break;
        }

    }
    return u32_GroupNoAddr + u32_RegNoAddr;
}
#if 1
static uint32_t i2s_gpio_val=0;
void LeftAudio_48K_gpio(void)
{
   __asm volatile (      
    //"cpsid i \n"
    //"stmdb sp!, {r4-r11} \n"
    "ldr  r0, =g_u32_audioLeftInterruptAddr\n"
    "ldr  r1, [r0]\n" //clear interrupt
    "mov  r2, #0xff\n"
    "strb r2, [r1]\n" 

    //"ldr  r0, =g_u32_audioDataAddr_out\n"
    //"ldr  r1, [r0]\n"
    //"mov  r2, #0xff\n"
    //"strb r2, [r1]\n"  

    //"ldmia sp!, {r4-r11} \n"
    //"cpsie i\n"
    );
   HAL_GPIO_SetPin(LA_GPIO, i2s_gpio_val&1);
   i2s_gpio_val++;
   //HAL_GPIO_SetPin(LA_GPIO, HAL_GPIO_PIN_SET);
}

void RightAudio_48K_gpio(void)
{
    __asm volatile (      
    //"cpsid i \n"
    //"stmdb sp!, {r4-r11} \n"
    "ldr  r0, =g_u32_audioRightInterruptAddr\n"
    "ldr  r1, [r0]\n" //clear interrupt
    "mov  r2, #0xff\n"
    "strb r2, [r1]\n" 

    //"ldr  r0, =g_u32_audioDataAddr_out\n"
    //"ldr  r1, [r0]\n" 
    //"mov  r2, #0\n"
    //"strb r2, [r1]\n" 

    //"ldmia sp!, {r4-r11} \n"
    //"cpsie i\n"
    );
	HAL_GPIO_SetPin(LA_GPIO, i2s_gpio_val&1);
	i2s_gpio_val++;
	//HAL_GPIO_SetPin(LA_GPIO, HAL_GPIO_PIN_SET);
}
#endif

HAL_RET_T HAL_SOFTI2S_Init(STRU_HAL_SOFTI2S_INIT *st_i2sInit)
{

    volatile uint32_t *pu8_newAudioSampleRate=(uint32_t *)(SRAM_MODULE_SHARE_AUDIO_RATE);
    
    if (((st_i2sInit->e_audioLeftGpioNum)/32) == ((st_i2sInit->e_audioRightGpioNum)/32))
    {
        return HAL_SOFTI2S_ERR_INIT;
    }

    //SysTicks_UnInit();

    uint32_t i=0;
/*    for(i=16;i<98;i++)
    {
      HAL_NVIC_DisableIrq(i);
    }*/

    HAL_GPIO_OutPut(LA_GPIO);
    g_u32_audioDataAddr_out = GPIO_GetAddr(LA_GPIO);
    g_u8_audioDataOffset=(st_i2sInit->e_audioDataGpioNum%32)%8;
    g_u32_audioDataAddr= ((st_i2sInit->e_audioDataGpioNum%32)>>3)*0x04 + 0x50 + (st_i2sInit->e_audioDataGpioNum>>5)*0x40000 + 0x40400000;
    g_u32_audioLeftInterruptAddr = (st_i2sInit->e_audioLeftGpioNum>>5)*0x40000 + 0x4040004C;
    g_u32_audioRightInterruptAddr = (st_i2sInit->e_audioRightGpioNum>>5)*0x40000 + 0x4040004C;
    
    memset((uint8_t *)AUDIO_DATA_START,0,(AUDIO_DATA_END - AUDIO_DATA_START));

    HAL_GPIO_InPut(st_i2sInit->e_audioDataGpioNum);    
    //left
    HAL_NVIC_SetPriority(HAL_NVIC_GPIO_INTR_N0_VECTOR_NUM + (st_i2sInit->e_audioLeftGpioNum>>5),4,0);
    HAL_NVIC_SetPriority(HAL_NVIC_GPIO_INTR_N0_VECTOR_NUM + (st_i2sInit->e_audioRightGpioNum>>5),4,0);

    HAL_GPIO_RegisterInterrupt(st_i2sInit->e_audioLeftGpioNum, HAL_GPIO_EDGE_SENUMSITIVE, HAL_GPIO_ACTIVE_LOW, NULL);

    //right   
    HAL_GPIO_RegisterInterrupt(st_i2sInit->e_audioRightGpioNum, HAL_GPIO_EDGE_SENUMSITIVE, HAL_GPIO_ACTIVE_HIGH, NULL);
    DLOG_Info("i2s init %p %p %p %p\n",LeftAudio_48K,RightAudio_48K,LeftAudio_44p1K,RightAudio_44p1K);

    if (HAL_SOFTI2S_ENCODE_IEC_44100 == *pu8_newAudioSampleRate)
    {
        *((uint32_t *)(AUDIO_LEFT_INTERRUPT_ADDR)) = (uint32_t)&LeftAudio_44p1K;
        *((uint32_t *)(AUDIO_RIGHT_INTERRUPT_ADDR)) = (uint32_t)&RightAudio_44p1K;
    }
    else if (HAL_SOFTI2S_ENCODE_IEC_48000 == *pu8_newAudioSampleRate)
    {
        *((uint32_t *)(AUDIO_LEFT_INTERRUPT_ADDR)) = (uint32_t)&LeftAudio_48K;
        *((uint32_t *)(AUDIO_RIGHT_INTERRUPT_ADDR)) = (uint32_t)&RightAudio_48K;
    }

	//*((uint32_t *)(AUDIO_LEFT_INTERRUPT_ADDR)) = (uint32_t)&LeftAudio_48K_gpio;
	//i2s_GPIO_ClearNvic(st_i2sInit->e_audioLeftGpioNum);
    //*((uint32_t *)(AUDIO_RIGHT_INTERRUPT_ADDR)) = (uint32_t)&RightAudio_48K_gpio;
	//DLOG_Critical("i2s init %p %p\n",*((uint32_t *)(AUDIO_LEFT_INTERRUPT_ADDR)),*((uint32_t *)(AUDIO_RIGHT_INTERRUPT_ADDR)));

    //left
    HAL_NVIC_SetPriority(HAL_NVIC_GPIO_INTR_N0_VECTOR_NUM + (st_i2sInit->e_audioLeftGpioNum>>5),0,0);
    HAL_GPIO_RegisterInterrupt(st_i2sInit->e_audioLeftGpioNum, HAL_GPIO_EDGE_SENUMSITIVE, HAL_GPIO_ACTIVE_LOW, NULL);

    //right   
    HAL_NVIC_SetPriority(HAL_NVIC_GPIO_INTR_N0_VECTOR_NUM + (st_i2sInit->e_audioRightGpioNum>>5),0,0);
    HAL_GPIO_RegisterInterrupt(st_i2sInit->e_audioRightGpioNum, HAL_GPIO_EDGE_SENUMSITIVE, HAL_GPIO_ACTIVE_HIGH, NULL);

    return  HAL_OK;
}

//extern uint8_t InterCore_GetMsg(uint32_t* msg_p, uint8_t* buf, uint32_t max_length);
static uint32_t u8_audioSampleRateTmp=0xf;
void HAL_SOFTI2S_Funct(void)
{
    volatile uint32_t *pu32_newPcmDataFlagAddr=(uint32_t *)(SRAM_MODULE_SHARE_AUDIO_PCM);
    volatile uint32_t *pu8_newAudioSampleRate=(uint32_t *)(SRAM_MODULE_SHARE_AUDIO_RATE);
    uint32_t tick_count = 0;
    while(1)
    {
        if (tick_count++ >= 100000)    //100 ms
        {
            WATCHDOG_Reset();
            tick_count = 0;
        }

        if (0 == g_u32_audioDataReady)
        {                      
            #ifdef AUDIO_SDRAM
            memcpy((void *)g_u32_dstAddress,(void *)g_u16_audioDataArray,(ADUIO_DATA_BUFF_LENGHT*sizeof(uint16_t)));
            #else
            memcpy((void *)(g_u32_dstAddress+DTCM_CPU0_DMA_ADDR_OFFSET),(void *)g_u16_audioDataArray,(ADUIO_DATA_BUFF_LENGHT*sizeof(uint16_t)));
            #endif            
            g_u32_audioDataReady=1;            
            g_u32_dstAddress+=(ADUIO_DATA_BUFF_LENGHT*sizeof(uint16_t));
            if (AUDIO_DATA_BUFF_SIZE == g_u32_dstAddress-AUDIO_DATA_START)
            {
                *pu32_newPcmDataFlagAddr = 1;
                //DLOG_Info("OK %x %d %p\n",g_u32_dstAddress,*pu32_newPcmDataFlagAddr,pu32_newPcmDataFlagAddr); 
            }
            if (AUDIO_DATA_END == g_u32_dstAddress)
            {
                g_u32_dstAddress = AUDIO_DATA_START;
                *pu32_newPcmDataFlagAddr = 2;
                //DLOG_Info("OK %x %d %p\n",g_u32_dstAddress,*pu32_newPcmDataFlagAddr,pu32_newPcmDataFlagAddr);                          
            } 
        }
        else if (u8_audioSampleRateTmp != *pu8_newAudioSampleRate)
        {
            if (0 == *pu8_newAudioSampleRate)
            {
                *((uint32_t *)(AUDIO_LEFT_INTERRUPT_ADDR)) = (uint32_t)&LeftAudio_44p1K;
                *((uint32_t *)(AUDIO_RIGHT_INTERRUPT_ADDR)) = (uint32_t)&RightAudio_44p1K;
            }
            else if (2 == *pu8_newAudioSampleRate)
            {
                *((uint32_t *)(AUDIO_LEFT_INTERRUPT_ADDR)) = (uint32_t)&LeftAudio_48K;
                *((uint32_t *)(AUDIO_RIGHT_INTERRUPT_ADDR)) = (uint32_t)&RightAudio_48K;
            }
            u8_audioSampleRateTmp = *pu8_newAudioSampleRate;
            //DLOG_Info("OK %x %x %p %p\n",u8_audioSampleRateTmp,*pu8_newAudioSampleRate,*((uint32_t *)(AUDIO_RIGHT_INTERRUPT_ADDR)),*((uint32_t *)(AUDIO_LEFT_INTERRUPT_ADDR)));             
        }
        #if 1
        else
        {
            uint32_t msg = 0; 
            uint8_t buf[INTER_CORE_MSG_SHARE_MEMORY_DATA_LENGTH];
            InterCore_GetMsg(&msg, buf,   sizeof(buf));
        }
        #endif
        
    }
    
}
