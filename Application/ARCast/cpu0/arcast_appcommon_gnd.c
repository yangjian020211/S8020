#include <stdint.h>
#include <string.h>

#include "debuglog.h"
#include "sys_event.h"
#include "memory_config.h"
#include "cmsis_os.h"
#include "hal_ret_type.h"
#include "hal_encodemp3.h"
#include "hal_softi2s.h"
#include "hal_usb_device.h"
#include "hal_bb.h"
#include "hal.h"
#include "arcast_appcommon.h"

#define ARCAST_DEBUGE

#ifdef ARCAST_DEBUGE
#define DLOG_INFO(...) DLOG_Info(__VA_ARGS__)
#else
#define DLOG_INFO(...)
#endif


static uint8_t u8_priviousCommandDataLen = 0;
static uint8_t u8_mp3Enable = 0;
static uint8_t u8_commandArray[32] __attribute__ ((aligned (4)));
static osSemaphoreId usb_signal;
static uint8_t usb_callback_buff[128];
static uint8_t usb_callback_buff_length = 0;
static void AR8020_SendFormatCommand(void);


static void RecvFormatAck(void *buff, uint32_t data_len, uint8_t  port_id)
{
    //ATM_StatusCallBack(buff, data_len);
    usb_callback_buff_length = data_len;
    memcpy(usb_callback_buff, buff, data_len);
    osSemaphoreRelease(usb_signal);
    return;
}

static int8_t Common_ATMStatus_Get(uint8_t status)
{
    return ((st_ARCastStatus.u8_ATMStatus >> status) & 1);
}

static int8_t Common_ATMStatus_Clear(uint8_t status)
{
    st_ARCastStatus.u8_ATMStatus &= (~(1 << status));
}

static int8_t Common_ATMStatus_Reset()
{
    st_ARCastStatus.u8_ATMStatus = 0;
}



static HAL_RET_T ARCastGnd_sendCommand(uint8_t u8_command, uint8_t *pu8_Databuff, uint8_t u8_len)
{
    uint8_t i=0;
    uint8_t dataLen = 0;
    memset(u8_commandArray, 0, sizeof(u8_commandArray));
    
    u8_commandArray[0] = 0x41;
    u8_commandArray[1] = 0x82;
    u8_commandArray[2] = u8_command;
    u8_commandArray[3] = u8_len+1;
    memcpy(&u8_commandArray[4], pu8_Databuff, u8_len);

    for (i = 0; i < 4 + u8_len; i++)
    {
        u8_commandArray[4 + u8_len] += u8_commandArray[i];
    }
    DLOG_Info("send command: ");
    for (i = 0; i < 5 + u8_len; i++)
    {
        DLOG_Info("%x ",u8_commandArray[i]);
    }
    DLOG_Info("end Len=%d",5 + u8_len);
    dataLen = 5 + u8_len;
    dataLen   += 4 - (dataLen & 0x3);
    //return HAL_OK;
    u8_priviousCommandDataLen = dataLen;
    return HAL_USB_CustomerSendData(u8_commandArray, dataLen, 0);

}

static int8_t CheckVideoFormatSupportOrNot(uint16_t u16_width, uint16_t u16_hight, uint8_t u8_framerate)
{
    uint8_t i = 0;
    uint8_t array_size = sizeof(s_st_ARCastSupportedOutputFormat)/sizeof(s_st_ARCastSupportedOutputFormat[0]);

    for (i = 0; i < array_size; i++)
    {
        if ((u16_width == s_st_ARCastSupportedOutputFormat[i][0]) &&
            (u16_hight == s_st_ARCastSupportedOutputFormat[i][1]) &&
            (u8_framerate == s_st_ARCastSupportedOutputFormat[i][2]))
        {
            return i;
        }
    }
    
    return -1;
    

}

static void rcvFormatHandler_ground(void *p)
{
    uint32_t u32_rcvLen = 0;
    uint8_t  u8_recData[64];
    uint8_t i = 0 ;
    HAL_BB_ComReceiveMsg(BB_COM_SESSION_3, u8_recData, 64, &u32_rcvLen);
    DLOG_Critical("rec sky command");
    if (1 == u32_rcvLen)
    {
        switch (u8_recData[0])
        {
            case ARCAST_COMMAND_SKY_REQUEST_CAPABILITY:
                {
                    st_ARCastStatus.u8_ARStatus = 0;

                    if ((st_ARCastStatus.u8_ATMStatus & (1 << ARCAST_COMMAND_REPLY_EDID)))
                    {
                        st_ARCastStatus.u8_ATMStatus = 0;
                        st_ARCastStatus.u8_ATMStatus |=  (1 << ARCAST_COMMAND_REPLY_EDID);
                        HAL_BB_ComSendMsg(BB_COM_SESSION_3, (uint8_t *)(&st_ARCastStatus.st_avCapability), sizeof(STRU_ARCAST_AVCAPABILITY));   
                    }

                    Common_ARStatus_Set(ARCAST_COMMAND_GND_CAPABILITY);
                    Common_ARStatus_Set(ARCAST_COMMAND_SKY_REQUEST_CAPABILITY);
                    DLOG_Critical("rec sky ack SKY_REQUEST_CAPABILITY");
                    break;
                }
            case ARCAST_COMMAND_GND_CAPABILITY:
                {
                    st_ARCastStatus.u8_ARStatus = 0;
                    if ((st_ARCastStatus.u8_ATMStatus & (1 << ARCAST_COMMAND_REPLY_EDID)))
                    {
                        st_ARCastStatus.u8_ATMStatus = 0;
                        st_ARCastStatus.u8_ATMStatus |=  (1 << ARCAST_COMMAND_REPLY_EDID);
                    }
                    Common_ARStatus_Set(ARCAST_COMMAND_GND_CAPABILITY);
                    Common_ARStatus_Set(ARCAST_COMMAND_SKY_REQUEST_CAPABILITY);
                    DLOG_Critical("rec sky ack CAPABILITY");
                    break;
                }
            case ARCAST_COMMAND_SKY_AVDATA:
                {
                    Common_ARStatus_Set(ARCAST_COMMAND_SKY_AVDATA);
                    DLOG_Critical("rec SKY_AVDATA");
                    break;
                }

        }
        
    }
    else if (2 == u32_rcvLen)
    {
        if (st_ARCastStatus.u8_AVFormat[0] != u8_recData[0] || st_ARCastStatus.u8_AVFormat[1] != u8_recData[1])
        {
            st_ARCastStatus.u8_AVFormat[0] = u8_recData[0];
            st_ARCastStatus.u8_AVFormat[1] = u8_recData[1];
            
            Common_ARStatus_Set(ARCAST_COMMAND_SKY_FORMAT);

            st_ARCastStatus.u8_ATMStatus &= (~(1 << ARCAST_COMMAND_FORMAT));
            st_ARCastStatus.u8_ARStatus &= (~(1 << ARCAST_COMMAND_SKY_AVDATA));
            uint8_t ack = ARCAST_COMMAND_SKY_FORMAT;
            HAL_BB_ComSendMsg(BB_COM_SESSION_3, &ack, 1);
            DLOG_Critical("rec sky ack FORMAT video =%d audio=%d ",st_ARCastStatus.u8_AVFormat[0],st_ARCastStatus.u8_AVFormat[1]);
        }
        else
        {
            DLOG_Critical("request av data");
            uint8_t ack = ARCAST_COMMAND_SKY_AVDATA;
            HAL_BB_ComSendMsg(BB_COM_SESSION_3, &ack, 1);
            DLOG_Critical("rec sky ack FORMAT video =%d audio=%d , but don't change",st_ARCastStatus.u8_AVFormat[0],st_ARCastStatus.u8_AVFormat[1]);
        }
        
    }

}

void Command_BBSendCommand(void const *argument)
{
    uint8_t i = 0;
    while(1)
    {
        HAL_Delay(2000);
        if (Common_ATMStatus_Get(ARCAST_COMMAND_REPLY_EDID))
        {
            if (0 == Common_ARStatus_Get(ARCAST_COMMAND_SKY_AVDATA))
            {
                if (0 == Common_ARStatus_Get(ARCAST_COMMAND_GND_CAPABILITY))
                {
                    DLOG_Info("ARCAST_COMMAND_GND_CAPABILITY");  
                    HAL_BB_ComSendMsg(BB_COM_SESSION_3, (uint8_t *)(&st_ARCastStatus.st_avCapability), sizeof(STRU_ARCAST_AVCAPABILITY));
                }
                else if (Common_ARStatus_Get(ARCAST_COMMAND_SKY_FORMAT))
                {
                    
                    if (0xff != st_ARCastStatus.u8_AVFormat[0])
                    {
                        DLOG_Info("AR8020_SendFormatCommand");
                        AR8020_SendFormatCommand();
                    }
                    else
                    {
                        DLOG_Info("wait format");
                    }                    
                }
                else if (Common_ATMStatus_Get(ARCAST_COMMAND_FORMAT))
                {
                    DLOG_Info("request av data");
                    uint8_t ack = ARCAST_COMMAND_SKY_AVDATA;
                    HAL_BB_ComSendMsg(BB_COM_SESSION_3, &ack, 1);
                }
            }
        }
        else
        {
            DLOG_Error("ATM don't send edid"); 
        }
    }
}

void Common_AVFORMATSysEventGroundInit(void)
{
    uint8_t u8_command = 0x03;

    HAL_BB_ComRegisterSession(BB_COM_SESSION_3,
                                  BB_COM_SESSION_PRIORITY_HIGH,
                                  BB_COM_SESSION_DATA_RT,
                                  rcvFormatHandler_ground);

    osSemaphoreDef(_usb_signal);
    usb_signal = osSemaphoreCreate(osSemaphore(_usb_signal), 1);

    HAL_USB_RegisterCustomerRecvData(RecvFormatAck); 
    Common_ATMStatus_Reset();
    Common_ARStatus_Reset();
    memset(&st_ARCastStatus.st_avCapability, 0, sizeof(st_ARCastStatus.st_avCapability));
    while(1) {
        if (0 == Common_ATMStatus_Get(ARCAST_COMMAND_REPLY_EDID))
        {
            ARCastGnd_sendCommand(ARCAST_COMMAND_REQUEST_EDID, &u8_command, 1);
            DLOG_Critical("get edid \n");
            HAL_Delay(2000);
        }
        
        if (osOK == osSemaphoreWait(usb_signal, 1000)) {
                if (usb_callback_buff_length != 0) {
                    ATM_StatusCallBack(usb_callback_buff, usb_callback_buff_length);
                }
                usb_callback_buff_length = 0;
        }        
    }
}

static void ATM_ACKStatus(uint8_t u8_status)
{
    st_ARCastStatus.u8_ATMStatus |= (1 << u8_status);
    if (ARCAST_COMMAND_FORMAT == u8_status)
    {
        Common_ARStatus_Clear(ARCAST_COMMAND_SKY_FORMAT);
    }
    DLOG_Warning("ack command %x %x", u8_status, st_ARCastStatus.u8_ATMStatus);
}


static void AR8020_SendEDIDRequset(void)
{
    uint8_t u8_command = 0x03;
    
    while (HAL_OK != ARCastGnd_sendCommand(ARCAST_COMMAND_REQUEST_EDID, &u8_command, 1))
    {
        HAL_Delay(10);
    }
    
}


static void ATM_EDIDHandle(const uint8_t *pu8_EDIDInfo, uint8_t u8_Len)
{
    uint8_t i = 0;
    uint8_t u8_videoCount = pu8_EDIDInfo[0];
    memset(&st_ARCastStatus.st_avCapability, 0, sizeof(st_ARCastStatus.st_avCapability));
    for (i = 0; i < u8_Len; i++)
    {
        DLOG_Warning("rec data %d %d",i,pu8_EDIDInfo[i]);
    }
    if ((u8_videoCount+1) == u8_Len)
    {
        DLOG_Warning("don't support audio");
    }
    else
    {
        st_ARCastStatus.st_avCapability.u8_AudioAidCount = pu8_EDIDInfo[u8_videoCount + 1];  
        memcpy(&st_ARCastStatus.st_avCapability.u8_AudioAidList[0], &pu8_EDIDInfo[u8_videoCount+2], st_ARCastStatus.st_avCapability.u8_AudioAidCount);
        insertion_sort(&st_ARCastStatus.st_avCapability.u8_AudioAidList[0], st_ARCastStatus.st_avCapability.u8_AudioAidCount);       
    }

    st_ARCastStatus.st_avCapability.u8_VideoVidCount = u8_videoCount;    

    memcpy(&st_ARCastStatus.st_avCapability.u8_VideoVidList[0], &pu8_EDIDInfo[1], st_ARCastStatus.st_avCapability.u8_VideoVidCount);        
    insertion_sort(&st_ARCastStatus.st_avCapability.u8_VideoVidList[0], u8_videoCount);
    Common_ATMStatus_Reset();
    Common_ARStatus_Reset();
    st_ARCastStatus.u8_AVFormat[0] = 0xff;
    st_ARCastStatus.u8_AVFormat[1] = 0xff;
#if 1    
    DLOG_Warning("support video %d audio %d",st_ARCastStatus.st_avCapability.u8_VideoVidCount, st_ARCastStatus.st_avCapability.u8_AudioAidCount);
    DLOG_Info("rec support video =%d",st_ARCastStatus.st_avCapability.u8_VideoVidCount);
    for (i = 0; i < st_ARCastStatus.st_avCapability.u8_VideoVidCount; i++)
    {
        DLOG_Info("rec support video vid =%d",st_ARCastStatus.st_avCapability.u8_VideoVidList[i]);
    }
    DLOG_Info("rec support audio =%d",st_ARCastStatus.st_avCapability.u8_AudioAidCount);
    for (i = 0; i < st_ARCastStatus.st_avCapability.u8_AudioAidCount; i++)
    {
        DLOG_Info("rec support audio aid =%d",st_ARCastStatus.st_avCapability.u8_AudioAidList[i]);
    }
#endif
}

extern uint8_t u8_mp3Enable;
static void AR8020_SendFormatCommand(void)
{
    uint8_t u8_commandArray[3] = {st_ARCastStatus.u8_AVFormat[0], u8_mp3Enable,st_ARCastStatus.u8_AVFormat[1]};
    //uint8_t u8_commandArray[3] = {2, 1, 0};
    
    while (HAL_OK != ARCastGnd_sendCommand(ARCAST_COMMAND_FORMAT, u8_commandArray, 3))
    {
        HAL_Delay(1000);
    }

    
}
static void AR8020_ReplyACK(uint8_t u8_ackCommand)
{
    uint8_t u8_command = u8_ackCommand;

    while (HAL_OK != ARCastGnd_sendCommand(ARCAST_COMMAND_ACK, &u8_command, 1))
    {
        HAL_Delay(1000);
    }
}

static void ATM_Error(uint8_t u8_errorCommand)
{
    DLOG_Error("error command %x", u8_errorCommand);
    switch(u8_errorCommand)
    {
        case ARCAST_COMMAND_REQUEST_EDID:
            AR8020_SendEDIDRequset();
            break;

        case ARCAST_COMMAND_FORMAT:
            AR8020_SendFormatCommand();
            break;
    }
}

void ATM_StatusCallBack(uint8_t *st_status, uint32_t data_len)
{
    uint32_t i = 0;
    uint8_t  u8_checkSum = 0;

    for (i = 0; i < (data_len - 1); i++)
    {
        u8_checkSum += st_status[i];
    }
    DLOG_Critical("command head %x %x CheckSum %x %x", st_status[0], st_status[1], u8_checkSum, st_status[data_len-1]);
    #if 1
    if ((0x41 != st_status[0]) || (0x82 != st_status[1]) || (u8_checkSum != st_status[data_len-1]))
    {
        
        DLOG_Error("command error head %x %x CheckSum %x %x", st_status[0], st_status[1], u8_checkSum, st_status[data_len-1]);
        DLOG_Error("dataLen %x content %x %x %x %x %x ", data_len, st_status[2], st_status[3], st_status[4], st_status[5], st_status[6]);
        HAL_USB_CustomerSendData(u8_commandArray, u8_priviousCommandDataLen, 0);
    }
    else
    {
       
        for (i = 0; i < data_len; i++)
        {
             DLOG_Info("command %x", st_status[i]);
        }
        switch (st_status[2])
        {
            case ARCAST_COMMAND_ACK:
                ATM_ACKStatus(st_status[4]);
                break;

            case ARCAST_COMMAND_REPLY_EDID:
                ATM_EDIDHandle(&st_status[4], st_status[3]-1);
                ATM_ACKStatus(st_status[2]);
                AR8020_ReplyACK(st_status[2]);
                break;
            
            case ARCAST_COMMAND_ERROR:
                ATM_Error(st_status[4]);
                break;

            default:
                break;
        }        
    }
    #endif

}






