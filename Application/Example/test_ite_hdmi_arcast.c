#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "debuglog.h"
#include "hal_hdmi_rx.h"
#include "hal_adc.h"
#include "hal_i2c.h"
#include "it_typedef.h"
#include "it6602.h"
#include "it_66021.h"
#include "sys_event.h"
#include "hal_hdmi_rx.h"
#include "hal.h"
#include "arcast_appcommon.h"
#if 0
extern void ATM_StatusCallBack(uint8_t *st_status, uint32_t data_len);
static void chgbank(int bank);
static unsigned char hdmirxrd( unsigned char RegAddr);

void ite_hdmi_read(uint8_t *page, uint8_t *addr)
{
    uint32_t u32_addr = strtoul(addr, NULL, 0);
    uint32_t u32_page = strtoul(page, NULL, 0);
    chgbank(u32_page);
    dlog_error("ite addr=%x value=%x", u32_addr, hdmirxrd(u32_addr));
}

void ite_hdmi_audio(void)
{
    uint8_t i = 0;
    chgbank(0);
    dlog_error("ite addr=0x52 value=%x", i, hdmirxrd(0x52));
    for (i = 0x74; i < 0x7e; i++)
    {
        dlog_error("ite addr=%x value=%x", i, hdmirxrd(i));
    }
    dlog_error("ite addr=0xaa value=%x", i, hdmirxrd(0xaa));
    for (i = 0xab; i < 0xaf; i++)
    {
        dlog_error("ite addr=%x value=%x", i, hdmirxrd(i));
    }

}

static unsigned char hdmirxrd( unsigned char RegAddr)
{
    
    return IT_66021_ReadByte(0x92,RegAddr);
}

static unsigned char hdmirxwr( unsigned char RegAddr,unsigned char DataIn)
{    
    return IT_66021_WriteByte(0x92, RegAddr, DataIn);
}

static unsigned char  hdmirxset( unsigned char  offset, unsigned char  mask, unsigned char  ucdata )
{
    unsigned char  temp;
    temp = hdmirxrd(offset);
    temp = (temp&((~mask)&0xFF))+(mask&ucdata);
    return hdmirxwr(offset, temp);
}

void command_hdmi(void)
{
    uint16_t u16_width;
    uint16_t u16_hight;
    uint8_t u8_framerate;
    uint32_t u32_sampleRate;

    HAL_HDMI_RX_GetAudioSampleRate(HAL_HDMI_RX_1, &u32_sampleRate);
    HAL_HDMI_RX_GetVideoFormat(HAL_HDMI_RX_1, &u16_width, &u16_hight, &u8_framerate);
    dlog_info("video width=%d u16_hight=%d u8_framerate=%d ", u16_width, u16_hight, u8_framerate);
    dlog_info("audio sampleRate=%d ", u32_sampleRate);
    get_vid_info();
    show_vid_info();
}

static void chgbank( int bank )
{
    switch( bank ) {
    case 0 :
         hdmirxset(0x0F, 0x03, 0x00);
         break;
    case 1 :
         hdmirxset(0x0F, 0x03, 0x01);
         break;
    case 2 :
         hdmirxset(0x0F, 0x03, 0x02);
         break;
    case 3:
         hdmirxset(0x0F, 0x03, 0x03);
         break;
    default :
         break;
    }
}

void command_dump(void)
{
    uint8_t dump[0xFF]={0};
    uint8_t dump2[2]={0,0};
    uint8_t i = 0;
//    HAL_I2C_MasterInit(HAL_I2C_COMPONENT_2, 0x92 >> 1, HAL_I2C_STANDARD_SPEED);

    chgbank(0);
    printf("page0"); 
    for (i = 0; i < 0xFF; i++)
    {        
        printf("reg%02x=%02x\n",i,hdmirxrd(i)); 
        dlog_output(1000);
    }

    chgbank(1);
    printf("page1");
    for (i = 0; i < 0xFF; i++)
    {        
        printf("reg1%02x=%02x\n",i,hdmirxrd(i));
        dlog_output(100);
    }

    chgbank(2);
    printf("page2");
    for (i = 0; i < 0xFF; i++)
    {        
        printf("reg2%02x=%02x\n",i,hdmirxrd(i));
        dlog_output(100); 
    }


}

void resetIte(void)
{
    dlog_info("ite reset");
    HAL_GPIO_SetPin(HAL_GPIO_NUM21, HAL_GPIO_PIN_RESET);
    HAL_Delay(200);
    HAL_GPIO_SetPin(HAL_GPIO_NUM21, HAL_GPIO_PIN_SET);
    it66021_init();
}
#endif
//extern uint8_t u8_mp3Enable;
//extern uint8_t timestamp;
void command_hdmiHandler(uint8_t *index, uint8_t *value)
{
    uint32_t u32_index = strtoul(index, NULL, 0);
    uint32_t u32_value = strtoul(value, NULL, 0);
    uint8_t data[20];
    uint32_t sample;
    switch (u32_index)
    {
        case 0:
            //command_hdmi();
            break;
        case 1:
            DLOG_Critical("send capability");
            data[0]=0x41;data[1]=0x82;data[2]=0x02;data[3]=0x07;data[4]=0x02;data[5]=0x00;data[6]=0x01;
            data[7]=0x02;data[8]=0x02;data[9]=0x01;data[10]=0xd4;
            ATM_StatusCallBack(data, 11);
            break;
        case 2:
            DLOG_Critical("send format ack");
            data[0]=0x41;data[1]=0x82;data[2]=0x00;data[3]=0x02;data[4]=0x03;data[5]=0xc8;
            ATM_StatusCallBack(data, 6);
            break;
        case 3:
            DLOG_Critical("send ite interrupt");
            IT6602_Interrupt();
            IT6602_fsm();
            break;
        case 4:
            DLOG_Critical("reboot ite");
            IT6602_fsm();
            break;
        case 5:
            //resetIte();
            break;
        case 6:
            HAL_HDMI_RX_GetAudioSampleRate(HAL_HDMI_RX_1, &sample);
            DLOG_Critical("check audio sample %d",sample);
            break;
        case 7:
            it6602HPDCtrl(0,0);
            DLOG_Critical("HPD 0");
            break;
        case 8:
            it6602HPDCtrl(0,1);
            DLOG_Critical("HPD 1");
            break;
        case 9:
            DLOG_Critical("send capability");
            data[0]=0x41;data[1]=0x82;data[2]=0x02;data[3]=0x09;data[4]=0x04;data[5]=0x01;data[6]=0x02;data[7]=0x00;data[8]=0x03;
            data[9]=0x02;data[10]=0x02;data[11]=0x01;data[12]=0xdd;
            ATM_StatusCallBack(data, 13);
            break;
            #if 0
        case 10:
            DLOG_Critical("mp3 %d",u32_value);
            u8_mp3Enable = u32_value;
            break;
        case 11:
            DLOG_Critical("time stamp %d",timestamp);
            timestamp = u32_value;
            break;
        case 12:
            DLOG_Critical("dump ite register");
            //command_dump();
            break;
            #endif
    }
    //HDMI_RX_IdleCallback1(NULL);
    //command_hdmi();
}
#if 0
void command_hdmiHandler(uint8_t *index)
{
    uint32_t u32_index = strtoul(index, NULL, 0);
    /*switch (u32_index)
    {
        case 0:
            IT6602_fsm();
            break;
        case 1:
            IT6602_Interrupt();
            break;
        case 2:
            IT6602_Interrupt();
            IT6602_fsm();
            break;
        case 3:
            command_hdmi();
            break;
        case 4:
            command_dump();
            break;
    }*/
    //HDMI_RX_IdleCallback1(NULL);
    command_hdmi();
    dlog_info("command_hdmiHandler");
}
#endif


