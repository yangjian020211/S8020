#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "memory_config.h"
#include "ite_define.h"
#include "i2c.h"

#include "debuglog.h"
#include "sys_event.h"
#include "systicks.h"
#include "interrupt.h"
#include "ite_config.h"
#include "ite_Utility.h"
#include "ite_it680x.h"
#include "memory.h"


extern volatile VTiming CurVTiming;
volatile uint8_t g_u8_iteInterruptFlage = 0;
typedef unsigned char (*HDMI_SET_TABLE)[3];
static HDMI_SET_TABLE hdmi_edid_table = NULL;

static unsigned int s_u8Array_ARCastSupportedOutputFormat[][4] =
{
    { 720, 480,  50,  2},//4:3
    { 720, 480,  60,  3},//16:9
    { 720, 576,  50, 17},//4:3
    { 720, 576,  60, 18},//16:9
    {1280, 720,  60,  4},//16:9
    {1280, 720,  50, 19},//16:9
    {1920, 1080, 30, 34},//16:9
};

void ITE_Delay(uint32_t delay)
{
    if (40*delay>0xffffffff)
    {
        for(uint32_t i = 0; i < 0xffffffff; i++)
        {
            ;
        }
    }
    else
    {
        for(uint32_t i = 0; i < 40*delay; i++)
        {
            ;
        }        
    }
}

uint8_t ITE_I2C_WriteByte(uint8_t slv_addr, uint8_t sub_addr, uint8_t val)
{
    uint8_t flag;
    uint8_t data[2] = {sub_addr, val};
    I2C_Master_WaitTillIdle(IT_68021_I2C_COMPONENT_NUM, IT_68021_I2C_I2C_MAX_DELAY_MS);
    flag = I2C_Master_WriteData(IT_68021_I2C_COMPONENT_NUM, slv_addr >> 1, data, 2);
    if(flag==0)
    {
        DLOG_Info("=====  Write Reg0x%X=%X data error=====  \n",sub_addr,val);
        return flag;
    }
    I2C_Master_WaitTillIdle(IT_68021_I2C_COMPONENT_NUM, IT_68021_I2C_I2C_MAX_DELAY_MS);
    if(flag==0)
    {
        DLOG_Info("=====  Write Reg0x%X=%X timeout=====  \n",sub_addr,val);
        return flag;
    }
    return 0;
}


uint8_t ITE_I2C_ReadByte(uint8_t slv_addr, uint8_t sub_addr)
{
    uint8_t sub_addr_tmp = sub_addr;
    uint8_t val = 0;
    I2C_Master_WaitTillIdle(IT_68021_I2C_COMPONENT_NUM, IT_68021_I2C_I2C_MAX_DELAY_MS);
    I2C_Master_ReadData(IT_68021_I2C_COMPONENT_NUM, slv_addr >> 1, &sub_addr_tmp, 1, &val, 1);
    I2C_Master_WaitTillIdle(IT_68021_I2C_COMPONENT_NUM, IT_68021_I2C_I2C_MAX_DELAY_MS);
    return val;
}

uint8_t ITE_I2C_WriteBytes(uint8_t slv_addr, uint8_t sub_addr, uint8_t byteno, uint8_t *p_data)
{
    uint8_t flag;
    uint8_t *pdata = malloc_safe(byteno+1);
    if (NULL != pdata)
    {
        (*pdata) = sub_addr;
        memcpy(pdata+1, p_data, byteno);
        if( byteno>0 )
        {
            I2C_Master_WaitTillIdle(IT_68021_I2C_COMPONENT_NUM, IT_68021_I2C_I2C_MAX_DELAY_MS);
			flag = I2C_Master_WriteData(IT_68021_I2C_COMPONENT_NUM, slv_addr >> 1, pdata, byteno+1);
        }        
        if(flag==0)
        {
            DLOG_Info("=====  Write Reg0x%X=%X data error=====  \n",sub_addr,(int)p_data);
            free_safe(pdata);
            return flag;
        }
        I2C_Master_WaitTillIdle(IT_68021_I2C_COMPONENT_NUM, IT_68021_I2C_I2C_MAX_DELAY_MS);
        //FIX_ID_002 xxxxx
        if(flag==0)
        {
            DLOG_Info("=====  Write Reg0x%X=%X timeout=====  \n",sub_addr,(int)p_data);
            free_safe(pdata);
            return flag;
        }
        free_safe(pdata);
    }
    return 0;
}

void ITE_I2C_Set(unsigned char slv_addr, unsigned char sub_addr, unsigned char mask, unsigned char val)
{
    
    unsigned char sub_addr_tmp = sub_addr;
    unsigned char tmpal = 0;
    I2C_Master_ReadData(IT_68021_I2C_COMPONENT_NUM, slv_addr >> 1, &sub_addr_tmp, 1, &tmpal, 1);

    unsigned char data[2] = {sub_addr, ((tmpal&((~mask)&0xFF)))+(mask&val)};
    I2C_Master_WriteData(IT_68021_I2C_COMPONENT_NUM, slv_addr >> 1, data, 2);
}

static void ITE_I2CInitial(void)
{
    static uint8_t i2c_initialized = 0;
    if (i2c_initialized == 0)
    {
        I2C_Init(IT_68021_I2C_COMPONENT_NUM, I2C_Master_Mode, RX_I2C_IO_MAP_ADDR >> 1, I2C_Standard_Speed);
        INTR_NVIC_SetIRQPriority(I2C_INTR2_VECTOR_NUM,INTR_NVIC_EncodePriority(NVIC_PRIORITYGROUP_5,4,0));
        reg_IrqHandle(I2C_INTR2_VECTOR_NUM, I2C_Master_IntrSrvc, NULL);
        INTR_NVIC_EnableIRQ(I2C_INTR2_VECTOR_NUM);
        msleep(100);
        i2c_initialized = 1;
    }
}

uint8_t IT_68021_GetVideoFormat(uint8_t index, uint16_t* widthPtr, uint16_t* hightPtr, uint8_t* framteratePtr, uint8_t* vic)
{
    if (TRUE == IsVideoOn())
    {
        uint8_t i = 0;
        uint8_t array_size = sizeof(s_u8Array_ARCastSupportedOutputFormat)/sizeof(s_u8Array_ARCastSupportedOutputFormat[0]);
        struct it6802_dev_data *it6802data = get_it6802_dev_data();
        for (i = 0; i < array_size; i++)
        {
            if (it6802data->VIC == s_u8Array_ARCastSupportedOutputFormat[i][3])
            {
                *widthPtr = s_u8Array_ARCastSupportedOutputFormat[i][0];
                *hightPtr = s_u8Array_ARCastSupportedOutputFormat[i][1];
                *framteratePtr = s_u8Array_ARCastSupportedOutputFormat[i][2];
                *vic = it6802data->VIC;
                return TRUE;
            }
        }
        
        uint8_t hdmi_i2c_addr = (index == 0) ? RX_I2C_HDMI_MAP_ADDR : (RX_I2C_HDMI_MAP_ADDR + 2);

        uint32_t u32_HTotal   = (((ITE_I2C_ReadByte(hdmi_i2c_addr, 0x9D))&0x3F)<<8) + ITE_I2C_ReadByte(hdmi_i2c_addr, 0x9C);
        uint32_t u32_HActive  = (((ITE_I2C_ReadByte(hdmi_i2c_addr, 0x9F))&0x3F)<<8) + ITE_I2C_ReadByte(hdmi_i2c_addr, 0x9E);

        uint32_t u32_VTotal   = (((ITE_I2C_ReadByte(hdmi_i2c_addr, 0xA4))&0x0F)<<8) + ITE_I2C_ReadByte(hdmi_i2c_addr, 0xA3);
        uint32_t u32_VActive  = (((ITE_I2C_ReadByte(hdmi_i2c_addr, 0xA4))&0xF0)<<4) + ITE_I2C_ReadByte(hdmi_i2c_addr, 0xA5);

        uint8_t u8_rddata = ITE_I2C_ReadByte(hdmi_i2c_addr, 0x9A);
        uint32_t PCLK = (124*255/u8_rddata)/10;
        uint64_t u64_FrameRate = (uint64_t)(PCLK)*1000*1000;
        u64_FrameRate /= u32_HTotal;
        u64_FrameRate /= u32_VTotal;
         
        *widthPtr = (uint16_t)u32_HActive;
        *hightPtr = (uint16_t)u32_VActive;
       
        if ((u64_FrameRate > 55) || (u64_FrameRate > 65))   
        {
            *framteratePtr = 60;
        }
        else if ((u64_FrameRate > 45) || (u64_FrameRate > 55))
        {
            *framteratePtr = 50;    
        }
        else if ((u64_FrameRate > 25) || (u64_FrameRate > 35))
        {
            *framteratePtr = 30;
        }
        *vic = 0xff;
        return TRUE;
    }
    else
    {
        *widthPtr = 0;
        *hightPtr = 0;
        *framteratePtr = 0;
        *vic = 0xff;
        return FALSE;
    }
}

uint8_t IT_68021_GetAudioSampleRate(uint8_t index, uint32_t* sampleRate)
{
    struct it6802_dev_data *it6802data = get_it6802_dev_data();
    if (ASTATE_AudioOn == it6802data->m_AState)
    {
        *sampleRate = (int)it6802data->m_RxAudioCaps.SampleFreq;
        return TRUE;
    }
    else
    {
        *sampleRate = 1;
        return FALSE;
    }
}

uint8_t IT_68021_IrqHandler0(void)
{
    IT6802_Interrupt();
    IT6802_fsm();
    g_u8_iteInterruptFlage = 1;    
}

uint8_t IT_68021_IrqHandler1(void)
{
    IT6802_Interrupt();
    IT6802_fsm();
    g_u8_iteInterruptFlage = 1; 
}

uint8_t IT_68021_GetAndClearIrq(void)
{
    uint8_t rec = g_u8_iteInterruptFlage;
    g_u8_iteInterruptFlage = 0;
    return rec;
}

void IT_68021_Initial(uint8_t index)
{
    DLOG_Info("ite hdmi");
    ITE_I2CInitial();
    ITE_init();
}
