/*****************************************************************************
Copyright: 2016-2020, Artosyn. Co., Ltd.
File name: hal_mfi.h
Description: The external HAL APIs to use the mfi controller.
Author: Artosyn Software Team
Version: 0.0.1
Date: 2017/10/26
History: 
        0.0.1    2017/10/26    The initial version of hal_mfi.c
*****************************************************************************/

#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "debuglog.h"
#include "hal_ret_type.h"
#include "hal_i2c.h"
#include "hal_gpio.h"
#include "hal.h"
#include "systicks.h"
#include "i2c.h"
#include "driver_module_init.h"
#include "i2c_ll.h"
#include "sys_event.h"
#include "ar_freertos_specific.h"
#include "usbd_hid.h"
#include "hal_mfi.h"
#include "memory_config.h"

USBD_HandleTypeDef *g_pdev = NULL;

static volatile uint16_t g_mfi_i2c_addr = 0x11;
 
ENUM_HAL_I2C_COMPONENT  g_i2cMFIComponent;
uint8_t                 g_mfiAuthenReplyACK[64];
uint8_t                 g_mfiSeqNum = 0;
uint8_t                 s_iap2_detect[6] = {0xFF, 0x55, 0x02, 0x00, 0xEE, 0x10};
uint8_t                 g_u8MFIRecvBuff[512];
uint8_t                 g_readAuthenticate[1024];
uint8_t                 g_readAuthenticateResponse[148];

#ifdef C201S_ZZ
uint8_t                 g_iDentificationInfo[] = {
    0xff, 0x5a, 0x01, 0x89, 0x40, 0x03, 0x67, 0x0a, 0x21,
    0x40, 0x40, 0x01, 0x7F, 0x1D, 0x01,
    //C2 -> BlastOff Controller, len 2 -> 19
    0x00, 0x18, 0x00, 0x00, 0x42, 0x6C, 0x61, 0x73, 0x74, 0x4F, 0x66, 0x66, 0x20, 0x43, 0x6F, 0x6E, 0x74, 0x72, 0x6F, 0x6C, 0x6C, 0x65, 0x72, 0x00,
    //C201D -> ZR-100B, lne 5 -> 7
    0x00, 0x0C, 0x00, 0x01, 0x5A, 0x52, 0x2D, 0x31, 0x30, 0x30, 0x42, 0x00,
    //ARTOSYN -> ZERO ZERO ROBOTICS, len 7 -> 18
    0x00, 0x17, 0x00, 0x02, 0x5A, 0x45, 0x52, 0x4F, 0x20, 0x5A, 0x45, 0x52, 0x4F, 0x20, 0x52, 0x4F, 0x42, 0x4F, 0x54, 0x49, 0x43, 0x53, 0x00,
    //Serial len 8 -> 10
    0x00, 0x0F, 0x00, 0x03, 0x46, 0x46, 0x46, 0x46, 0x46, 0x46, 0x46, 0x46, 0x46, 0x46, 0x00,
    0x00, 0x0D, 0x00, 0x04, 0x76, 0x30, 0x2E, 0x30, 0x2E, 0x30, 0x2E, 0x30, 0x00,
    0x00, 0x0D, 0x00, 0x05, 0x76, 0x31, 0x2E, 0x30, 0x2E, 0x30, 0x2E, 0x30, 0x00,
    0x00, 0x0A, 0x00, 0x06, 0xEA, 0x02, 0xAE, 0x00, 0xAE, 0x02,
    0x00, 0x06, 0x00, 0x07, 0xAE, 0x01,
    0x00, 0x05, 0x00, 0x08, 0x00,
    0x00, 0x06, 0x00, 0x09, 0x00, 0x00,
    0x00, 0x2B, 0x00, 0x0A,
                 0x00, 0x05, 0x00, 0x00, 0x00,
                 0x00, 0x17, 0x00, 0x01, 0x63, 0x6F, 0x6D, 0x2E, 0x61, 0x72, 0x74, 0x6F, 0x73, 0x79, 0x6E, 0x2E, 0x63, 0x6F, 0x6D, 0x6D, 0x6F, 0x6E, 0x00,
                 0x00, 0x05, 0x00, 0x02, 0x01,
                 0x00, 0x06, 0x00, 0x03, 0x00, 0x00,
    0x00, 0x2A, 0x00, 0x0A,
                 0x00, 0x05, 0x00, 0x00, 0x01,
                 0x00, 0x16, 0x00, 0x01, 0x63, 0x6F, 0x6D, 0x2E, 0x61, 0x72, 0x74, 0x6F, 0x73, 0x79, 0x6E, 0x2E, 0x76, 0x69, 0x64, 0x65, 0x6F, 0x00,
                 0x00, 0x05, 0x00, 0x02, 0x01,
                 0x00, 0x06, 0x00, 0x03, 0x00, 0x00,
    0x00, 0x2A, 0x00, 0x0A, // audio
                 0x00, 0x05, 0x00, 0x00, 0x02,
                 0x00, 0x16, 0x00, 0x01, 0x63, 0x6F, 0x6D, 0x2E, 0x61, 0x72, 0x74, 0x6F, 0x73, 0x79, 0x6E, 0x2E, 0x61, 0x75, 0x64, 0x69, 0x6F, 0x00,
                 0x00, 0x05, 0x00, 0x02, 0x01,
                 0x00, 0x06, 0x00, 0x03, 0x00, 0x00,
    0x00, 0x2D, 0x00, 0x0A, // customer
                 0x00, 0x05, 0x00, 0x00, 0x03,
                 0x00, 0x19, 0x00, 0x01, 0x63, 0x6F, 0x6D, 0x2E, 0x61, 0x72, 0x74, 0x6F, 0x73, 0x79, 0x6E, 0x2E, 0x63, 0x75, 0x73, 0x74, 0x6F, 0x6D, 0x65, 0x72, 0x00,
                 0x00, 0x05, 0x00, 0x02, 0x01,
                 0x00, 0x06, 0x00, 0x03, 0x00, 0x00,
    0x00, 0x0F, 0x00, 0x0B, 0x4D, 0x44, 0x5A, 0x33, 0x35, 0x44, 0x38, 0x52, 0x35, 0x36, 0x00,
    0x00, 0x07, 0x00, 0x0C, 0x65, 0x6E, 0x00,
    0x00, 0x07, 0x00, 0x0D, 0x7A, 0x68, 0x00,
    0x00, 0x07, 0x00, 0x0D, 0x65, 0x6E, 0x00,
    0x00, 0x07, 0x00, 0x0D, 0x6A, 0x61, 0x00,
    0x00, 0x07, 0x00, 0x0D, 0x66, 0x72, 0x00,
    0x00, 0x07, 0x00, 0x0D, 0x64, 0x65, 0x00,
    0x00, 0x15, 0x00, 0x10, 0x00, 0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0x00, 0x01, 0x72, 0x63, 0x00, 0x00, 0x04, 0x00, 0x02,
    0x81,
};
#else

uint8_t                 g_iDentificationInfo[] = {
    0xff, 0x5a, 0x01, 0x5C, 0x40, 0x03, 0x67, 0x0a, 0x21,
    0x40, 0x40, 0x01, 0x52, 0x1D, 0x01,
    0x00, 0x07, 0x00, 0x00, 0x43, 0x32, 0x00,
    0x00, 0x0A, 0x00, 0x01, 0x43, 0x32, 0x30, 0x31, 0x44, 0x00,
    0x00, 0x0C, 0x00, 0x02, 0x41, 0x52, 0x54, 0x4F, 0x53, 0x59, 0x4E, 0x00,
    0x00, 0x0F, 0x00, 0x03, 0x46, 0x46, 0x46, 0x46, 0x46, 0x46, 0x46, 0x46, 0x46, 0x46, 0x00,       //Serial Number
    0x00, 0x0D, 0x00, 0x04, 0x76, 0x30, 0x2E, 0x30, 0x2E, 0x30, 0x2E, 0x30, 0x00,
    0x00, 0x0D, 0x00, 0x05, 0x76, 0x31, 0x2E, 0x30, 0x2E, 0x30, 0x2E, 0x30, 0x00,
    0x00, 0x0A, 0x00, 0x06, 0xEA, 0x02, 0xAE, 0x00, 0xAE, 0x02,
    0x00, 0x06, 0x00, 0x07, 0xAE, 0x01,
    0x00, 0x05, 0x00, 0x08, 0x00,
    0x00, 0x06, 0x00, 0x09, 0x00, 0x00,
    0x00, 0x2B, 0x00, 0x0A,
                 0x00, 0x05, 0x00, 0x00, 0x00,
                 0x00, 0x17, 0x00, 0x01, 0x63, 0x6F, 0x6D, 0x2E, 0x61, 0x72, 0x74, 0x6F, 0x73, 0x79, 0x6E, 0x2E, 0x63, 0x6F, 0x6D, 0x6D, 0x6F, 0x6E, 0x00,
                 0x00, 0x05, 0x00, 0x02, 0x01,
                 0x00, 0x06, 0x00, 0x03, 0x00, 0x00,
    0x00, 0x2A, 0x00, 0x0A,
                 0x00, 0x05, 0x00, 0x00, 0x01,
                 0x00, 0x16, 0x00, 0x01, 0x63, 0x6F, 0x6D, 0x2E, 0x61, 0x72, 0x74, 0x6F, 0x73, 0x79, 0x6E, 0x2E, 0x76, 0x69, 0x64, 0x65, 0x6F, 0x00,
                 0x00, 0x05, 0x00, 0x02, 0x01,
                 0x00, 0x06, 0x00, 0x03, 0x00, 0x00,
    0x00, 0x2A, 0x00, 0x0A, // audio
                 0x00, 0x05, 0x00, 0x00, 0x02,
                 0x00, 0x16, 0x00, 0x01, 0x63, 0x6F, 0x6D, 0x2E, 0x61, 0x72, 0x74, 0x6F, 0x73, 0x79, 0x6E, 0x2E, 0x61, 0x75, 0x64, 0x69, 0x6F, 0x00,
                 0x00, 0x05, 0x00, 0x02, 0x01,
                 0x00, 0x06, 0x00, 0x03, 0x00, 0x00,
    0x00, 0x2D, 0x00, 0x0A, // customer
                 0x00, 0x05, 0x00, 0x00, 0x03,
                 0x00, 0x19, 0x00, 0x01, 0x63, 0x6F, 0x6D, 0x2E, 0x61, 0x72, 0x74, 0x6F, 0x73, 0x79, 0x6E, 0x2E, 0x63, 0x75, 0x73, 0x74, 0x6F, 0x6D, 0x65, 0x72, 0x00,
                 0x00, 0x05, 0x00, 0x02, 0x01,
                 0x00, 0x06, 0x00, 0x03, 0x00, 0x00,
    0x00, 0x07, 0x00, 0x0C, 0x65, 0x6E, 0x00,
    0x00, 0x07, 0x00, 0x0D, 0x7A, 0x68, 0x00,
    0x00, 0x07, 0x00, 0x0D, 0x65, 0x6E, 0x00,
    0x00, 0x07, 0x00, 0x0D, 0x6A, 0x61, 0x00,
    0x00, 0x07, 0x00, 0x0D, 0x66, 0x72, 0x00,
    0x00, 0x07, 0x00, 0x0D, 0x64, 0x65, 0x00,
    0x00, 0x15, 0x00, 0x10, 0x00, 0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0x00, 0x01, 0x72, 0x63, 0x00, 0x00, 0x04, 0x00, 0x02,
    0x81,
};

#endif

#define IDENTIFICATION_LEN          (sizeof(g_iDentificationInfo) / sizeof(g_iDentificationInfo[0]))


static HAL_RET_T MFI_ReadDataOnce(ENUM_HAL_I2C_COMPONENT e_i2cComponent, 
                                     uint16_t u16_i2cAddr,
                                     uint8_t *pu8_wrData,
                                     uint8_t  u8_wrSize,
                                     uint8_t *pu8_rdData,
                                     uint32_t u32_rdSize,
                                     uint32_t u32_timeOut)
{
    EN_I2C_COMPONENT en_component;
    uint32_t start;

    switch (e_i2cComponent)
    {
        case HAL_I2C_COMPONENT_0:
            en_component = I2C_Component_0;
            break;
        case HAL_I2C_COMPONENT_1:
            en_component = I2C_Component_1;
            break;
        case HAL_I2C_COMPONENT_2:
            en_component = I2C_Component_2;
            break;
        case HAL_I2C_COMPONENT_3:
            en_component = I2C_Component_3;
            break;
        case HAL_I2C_COMPONENT_4:
            en_component = I2C_Component_4;
            break;
        case HAL_I2C_COMPONENT_5:
            en_component = I2C_Component_5;
            break;
        default:
            return HAL_I2C_ERR_READ_DATA;
    }

    if ( -1 == COMMON_driverInitGet(INITED_I2C, en_component) )
    {
        DLOG_Error("fail, channel = %d", en_component);
        return HAL_NOT_INITED;
    }

    I2C_Master_ClrTxAbrt(en_component);
    
    if (I2C_Master_GetBusyStatus(en_component))
    {
        return HAL_BUSY;
    }

    I2C_Master_ReadDataMfi(en_component, u16_i2cAddr, pu8_wrData, u8_wrSize, pu8_rdData, u32_rdSize);

    if (0 != u32_timeOut)
    {
        start = SysTicks_GetTickCount();
        while (I2C_Master_GetBusyStatus(en_component))
        {
            if ((SysTicks_GetDiff(start, SysTicks_GetTickCount())) >= u32_timeOut)
            {
                 return HAL_TIME_OUT;
            }

            SysTicks_DelayUS(100);
        }
    }

    return HAL_OK;
}


HAL_RET_T HAL_MFI_Write(ENUM_HAL_I2C_COMPONENT e_i2cComponent, 
                          uint16_t u16_i2cAddr,
                          uint8_t *pu8_wrData,
                          uint32_t u32_wrSize,
                          uint32_t u32_timeOut)
{
    uint8_t tmpCnt = 10;

    while(tmpCnt--)
    {
        HAL_I2C_MasterWriteData(e_i2cComponent, 
                                      u16_i2cAddr,
                                      pu8_wrData,
                                      u32_wrSize,
                                      u32_timeOut);
        SysTicks_DelayUS(500);

        if ((I2C_Master_GetTxAbrtSource(e_i2cComponent) & IC_TX_ABRT_7B_ADDR_NOACK) == 0)
        {
            DLOG_Critical("tmpCnt:%d",tmpCnt);
            return HAL_OK;
        }
    }

    return HAL_MFI_ERR;
}


HAL_RET_T MFI_MFI_Read(ENUM_HAL_I2C_COMPONENT e_i2cComponent, 
                         uint16_t u16_i2cAddr,
                         uint8_t *pu8_wrData,
                         uint8_t  u8_wrSize,
                         uint8_t *pu8_rdData,
                         uint32_t u32_rdSize,
                         uint32_t u32_timeOut)
{
    uint8_t tmpCnt = 10;

    while(tmpCnt--)
    {
        MFI_ReadDataOnce(e_i2cComponent, 
                             u16_i2cAddr,
                             pu8_wrData,
                             u8_wrSize,
                             pu8_rdData,
                             u32_rdSize,
                             u32_timeOut);
        
        SysTicks_DelayUS(500);
        
        if ((I2C_Master_GetTxAbrtSource(e_i2cComponent) & IC_TX_ABRT_7B_ADDR_NOACK) == 0)
        {
            DLOG_Critical("tmpCnt:%d",tmpCnt);
            return HAL_OK;
        }
    }

    return HAL_MFI_ERR;
}


uint8_t MFI_iAP2CalcChk(uint8_t *data, uint16_t len)
{
    uint16_t    i = 0;
    uint8_t     check_sum = 0;

    for (i = 0; i < len; i++)
    {
        check_sum += data[i];
    }

    return (uint8_t)(0x100 - check_sum);
}


void MFI_iAP2DetectApple(USBD_HandleTypeDef *pdev)
{
    USBD_HID_SendReport(pdev, s_iap2_detect, sizeof(s_iap2_detect), MFI_IN_ADDR);
}


void MFI_iAP2ParamSync(USBD_HandleTypeDef *pdev)
{
    STRU_iAP2PacketHeader    *st_iAP2PacketHeader = (STRU_iAP2PacketHeader *)g_mfiAuthenReplyACK;
    STRU_iAP2SyncPayload     *st_iAP2SyncPayload = (STRU_iAP2SyncPayload *)(g_mfiAuthenReplyACK + sizeof(STRU_iAP2PacketHeader));

    st_iAP2PacketHeader->start_msb = 0xFF;
    st_iAP2PacketHeader->start_lsb = 0x5A;
    st_iAP2PacketHeader->len_msb   = 0x00;
    st_iAP2PacketHeader->len_lsb   = 0x17;
    st_iAP2PacketHeader->ctrl_byte = IAP2_MSG_TYPE_SYN;
    st_iAP2PacketHeader->seq_num   = 0x00;
    st_iAP2PacketHeader->ack_num   = 0x00;
    st_iAP2PacketHeader->session_identifier = 0x00;
    st_iAP2PacketHeader->check_sum = MFI_iAP2CalcChk((uint8_t *)st_iAP2PacketHeader, 8);

    st_iAP2SyncPayload->link_version = 0x01;
    st_iAP2SyncPayload->max_outstanding_packet = 0x05;
    st_iAP2SyncPayload->max_recv_len_msb = 0x04;
    st_iAP2SyncPayload->max_recv_len_lsb = 0x00;
    st_iAP2SyncPayload->retrans_timeout_msb = 0x07;
    st_iAP2SyncPayload->retrans_timeout_lsb = 0xD0;
    st_iAP2SyncPayload->cumulative_ack_timeout_msb = 0x00;
    st_iAP2SyncPayload->cumulative_ack_timeout_lsb = 0x14;
    st_iAP2SyncPayload->max_num_retrans = 0x1E;
    st_iAP2SyncPayload->max_cumulative_ack = 0x05;
    st_iAP2SyncPayload->st_iap2_sessions.session_identifier = 0x0A;
    st_iAP2SyncPayload->st_iap2_sessions.session_type = 0x00;
    st_iAP2SyncPayload->st_iap2_sessions.session_version = 0x01;
    st_iAP2SyncPayload->check_sum = MFI_iAP2CalcChk((uint8_t *)st_iAP2SyncPayload, 13);

    USBD_HID_SendReport(pdev, (uint8_t *)st_iAP2PacketHeader, 23, MFI_IN_ADDR);
}


void MFI_iAP2ParamSyncComplete(USBD_HandleTypeDef *pdev)
{
    STRU_iAP2PacketHeader    *st_iAP2PacketHeader = (STRU_iAP2PacketHeader *)g_mfiAuthenReplyACK;

    st_iAP2PacketHeader->start_msb = 0xFF;
    st_iAP2PacketHeader->start_lsb = 0x5A;
    st_iAP2PacketHeader->len_msb   = 0x00;
    st_iAP2PacketHeader->len_lsb   = 0x09;
    st_iAP2PacketHeader->ctrl_byte = IAP2_MSG_TYPE_ACK;
    st_iAP2PacketHeader->seq_num   = 0x00;
    st_iAP2PacketHeader->ack_num   = ((STRU_iAP2PacketHeader *)g_u8MFIRecvBuff)->seq_num;
    st_iAP2PacketHeader->session_identifier = 0x00;
    st_iAP2PacketHeader->check_sum = MFI_iAP2CalcChk((uint8_t *)st_iAP2PacketHeader, 8);

    USBD_HID_SendReport(pdev, (uint8_t *)st_iAP2PacketHeader, 9, MFI_IN_ADDR);
}


void MFI_iAP2ReplyACK(USBD_HandleTypeDef *pdev)
{
    STRU_iAP2PacketHeader    *st_iAP2PacketHeader;

    st_iAP2PacketHeader = (STRU_iAP2PacketHeader *)g_mfiAuthenReplyACK;

    st_iAP2PacketHeader->start_msb = 0xFF;
    st_iAP2PacketHeader->start_lsb = 0x5A;
    st_iAP2PacketHeader->len_msb   = 0x00;
    st_iAP2PacketHeader->len_lsb   = 0x09;
    st_iAP2PacketHeader->ctrl_byte = IAP2_MSG_TYPE_ACK;
    st_iAP2PacketHeader->seq_num   = g_mfiSeqNum;
    st_iAP2PacketHeader->ack_num   = ((STRU_iAP2PacketHeader *)g_u8MFIRecvBuff)->seq_num;
    st_iAP2PacketHeader->session_identifier = 0x00;
    st_iAP2PacketHeader->check_sum = MFI_iAP2CalcChk((uint8_t *)st_iAP2PacketHeader, 8);

    g_mfiSeqNum++;

    USBD_HID_SendReport(pdev, (uint8_t *)st_iAP2PacketHeader, 9, MFI_IN_ADDR);
}

void MFI_iAP2RequestAuthenticationCertificate(USBD_HandleTypeDef *pdev)
{
    STRU_iAP2PacketHeader    *st_iAP2PacketHeader;
    STRU_iAP2ControlDef      *st_iAP2Control;
    STRU_iAP2ParamDef        *st_iAP2Param;
    uint16_t                 certificate_data_len = 0;
    uint8_t                  i2c_address;
    uint8_t                  certificate_pack_num = 0;
    uint8_t                 *certificate_data_ptr;
    uint8_t                  i;

    MFI_iAP2ReplyACK(pdev);

    i2c_address = MFI_I2C_CERTIFICATE_DATA_LEN;

    MFI_MFI_Read(g_i2cMFIComponent,
                 g_mfi_i2c_addr,
                 &i2c_address,
                 1,
                 (uint8_t *)&certificate_data_len,
                 2,
                 100);

    certificate_data_len = (certificate_data_len >> 8) + (certificate_data_len << 8);

    st_iAP2PacketHeader = (STRU_iAP2PacketHeader *)g_readAuthenticate;
    st_iAP2Control      = (STRU_iAP2ControlDef *)((uint8_t *)st_iAP2PacketHeader + sizeof(STRU_iAP2PacketHeader));
    st_iAP2Param        = (STRU_iAP2ParamDef *)((uint8_t *)st_iAP2Control + sizeof(STRU_iAP2ControlDef));

    st_iAP2PacketHeader->start_msb = 0xFF;
    st_iAP2PacketHeader->start_lsb = 0x5A;
    st_iAP2PacketHeader->len_msb = (uint8_t)((certificate_data_len + 20) >> 8);
    st_iAP2PacketHeader->len_lsb = (uint8_t)(certificate_data_len + 20);
    st_iAP2PacketHeader->ctrl_byte = IAP2_MSG_TYPE_ACK;
    st_iAP2PacketHeader->seq_num = g_mfiSeqNum;
    st_iAP2PacketHeader->ack_num = ((STRU_iAP2PacketHeader *)g_u8MFIRecvBuff)->seq_num;
    st_iAP2PacketHeader->session_identifier = ((STRU_iAP2PacketHeader *)g_u8MFIRecvBuff)->session_identifier;
    st_iAP2PacketHeader->check_sum = MFI_iAP2CalcChk((uint8_t *)st_iAP2PacketHeader, 8);

    st_iAP2Control->start_msb = 0x40;
    st_iAP2Control->start_lsb = 0x40;
    st_iAP2Control->len_msb = (uint8_t)((certificate_data_len + 10) >> 8);
    st_iAP2Control->len_lsb = (uint8_t)(certificate_data_len + 10);
    st_iAP2Control->msg_id_msb = (uint8_t)(IAP2_MSG_AUTHENTICATION_CERTIFICATE >> 8);
    st_iAP2Control->msg_id_lsb = (uint8_t)IAP2_MSG_AUTHENTICATION_CERTIFICATE;

    st_iAP2Param->param_len_msb = (uint8_t)((certificate_data_len + 4) >> 8);
    st_iAP2Param->param_len_lsb = (uint8_t)(certificate_data_len + 4);
    st_iAP2Param->param_id_msb = (uint8_t)0;
    st_iAP2Param->param_id_lsb = (uint8_t)0;

    certificate_pack_num = (certificate_data_len / 128);
    certificate_data_ptr = (uint8_t *)st_iAP2Param + sizeof(STRU_iAP2ParamDef);

    for (i = 0; i < certificate_pack_num; i++)
    {
        i2c_address++;

        MFI_MFI_Read(g_i2cMFIComponent,
                     g_mfi_i2c_addr,
                     &i2c_address,
                     1,
                     certificate_data_ptr,
                     128,
                     100);

        certificate_data_ptr += 128;
    }

    if (certificate_data_len % 128)
    {
        i2c_address++;

        MFI_MFI_Read(g_i2cMFIComponent,
                     g_mfi_i2c_addr,
                     &i2c_address,
                     1,
                     certificate_data_ptr,
                     (certificate_data_len % 128),
                     100);
    }

    g_readAuthenticate[certificate_data_len + 19] = MFI_iAP2CalcChk(&g_readAuthenticate[9], certificate_data_len + 10);

    USBD_HID_SendReport(pdev, g_readAuthenticate, certificate_data_len + 20, MFI_IN_ADDR);
}


void MFI_iAP2ChallengeResponse(USBD_HandleTypeDef *pdev)
{
    STRU_iAP2PacketHeader    *st_iAP2PacketHeader;
    STRU_iAP2ControlDef      *st_iAP2Control;
    STRU_iAP2ParamDef        *st_iAP2Param;
    uint8_t                  *msg_content;
    uint8_t                   data_len;
    uint8_t                   i;
    uint8_t                   i2c_buffer[200];
    uint16_t                  certificate_data_len = 0;
    uint8_t                   try_time = 0;

    MFI_iAP2ReplyACK(pdev);

    i2c_buffer[0] = MFI_I2C_CHALLENGE_DATA_LEN;
    data_len      = g_u8MFIRecvBuff[16] - 4;
    i2c_buffer[1] = data_len;

    HAL_MFI_Write(g_i2cMFIComponent, 
                  g_mfi_i2c_addr,
                  i2c_buffer,
                  2,
                  100);

    i2c_buffer[0] = MFI_I2C_CHALLENGE_DATA;

    for (i = 0; i < data_len; i++)
    {
        i2c_buffer[i+1] = g_u8MFIRecvBuff[i+19];
    }

    HAL_MFI_Write(g_i2cMFIComponent,
                  g_mfi_i2c_addr,
                  i2c_buffer,
                  data_len + 1,
                  100);

    i2c_buffer[0] = MFI_I2C_AUTHENTICATION_CONTROL_STATUS;
    i2c_buffer[1] = 1;

    HAL_MFI_Write(g_i2cMFIComponent,
                  g_mfi_i2c_addr,
                  i2c_buffer,
                  2,
                  100);

    memset((void *)i2c_buffer, 0, 200);

    i2c_buffer[0] = MFI_I2C_AUTHENTICATION_CONTROL_STATUS;

    while (i2c_buffer[1] != 0x10)
    {
        MFI_MFI_Read(g_i2cMFIComponent,
                     g_mfi_i2c_addr,
                     &i2c_buffer[0],
                     1,
                     &i2c_buffer[1],
                     1,
                     100);

        ar_osDelay(10);

        if (try_time++ >= 30)
            break;
    }

    memset((void *)i2c_buffer, 0, 200);

    i2c_buffer[0] = MFI_I2C_CHALLENGE_RESPONSE_DATA_LEN;

    MFI_MFI_Read(g_i2cMFIComponent,
                 g_mfi_i2c_addr,
                 &i2c_buffer[0],
                 1,
                 (uint8_t *)&certificate_data_len,
                 2,
                 100);

    certificate_data_len = (certificate_data_len >> 8) + (certificate_data_len << 8);

    memset((void *)i2c_buffer, 0, 200);

    i2c_buffer[0] = MFI_I2C_CHALLENGE_RESPONSE_DATA;

    MFI_MFI_Read(g_i2cMFIComponent,
                 g_mfi_i2c_addr,
                 &i2c_buffer[0],
                 1,
                 &i2c_buffer[1],
                 certificate_data_len,
                 100);

    st_iAP2PacketHeader = (STRU_iAP2PacketHeader *)g_readAuthenticateResponse;
    st_iAP2Control      = (STRU_iAP2ControlDef *)((uint8_t *)st_iAP2PacketHeader + sizeof(STRU_iAP2PacketHeader));
    st_iAP2Param        = (STRU_iAP2ParamDef *)((uint8_t *)st_iAP2Control + sizeof(STRU_iAP2ControlDef));
    msg_content         = (uint8_t *)st_iAP2Param + sizeof(STRU_iAP2ParamDef);

    st_iAP2PacketHeader->start_msb = 0xFF;
    st_iAP2PacketHeader->start_lsb = 0x5A;
    st_iAP2PacketHeader->len_msb = (uint8_t)((certificate_data_len + 20)>> 8);
    st_iAP2PacketHeader->len_lsb = (uint8_t)(certificate_data_len + 20);
    st_iAP2PacketHeader->ctrl_byte = IAP2_MSG_TYPE_ACK;
    st_iAP2PacketHeader->seq_num = g_mfiSeqNum;
    st_iAP2PacketHeader->ack_num = ((STRU_iAP2PacketHeader *)g_u8MFIRecvBuff)->seq_num;
    st_iAP2PacketHeader->session_identifier = ((STRU_iAP2PacketHeader *)g_u8MFIRecvBuff)->session_identifier;
    st_iAP2PacketHeader->check_sum = MFI_iAP2CalcChk(g_readAuthenticateResponse, 8);

    st_iAP2Control->start_msb = 0x40;
    st_iAP2Control->start_lsb = 0x40;
    st_iAP2Control->len_msb = (uint8_t)((certificate_data_len + 10)>> 8);
    st_iAP2Control->len_lsb = (uint8_t)(certificate_data_len + 10);
    st_iAP2Control->msg_id_msb = (uint8_t)(IAP2_MSG_AUTHENTICATION_RESP >> 8);
    st_iAP2Control->msg_id_lsb = (uint8_t)IAP2_MSG_AUTHENTICATION_RESP;

    st_iAP2Param->param_len_msb = (uint8_t)((certificate_data_len + 4)>> 8);
    st_iAP2Param->param_len_lsb = (uint8_t)(certificate_data_len + 4);
    st_iAP2Param->param_id_msb = 0x00;
    st_iAP2Param->param_id_lsb = 0x00;

    for (i = 0; i < 128; i++)
    {
        msg_content[i] = i2c_buffer[i+1];
    }

    msg_content[certificate_data_len] = MFI_iAP2CalcChk((uint8_t *)st_iAP2Control, certificate_data_len + 10);

    USBD_HID_SendReport(pdev, (uint8_t *)st_iAP2PacketHeader, certificate_data_len + 20, MFI_IN_ADDR);
}


void MFI_iAP2ReportInformation(USBD_HandleTypeDef *pdev)
{
    STRU_iAP2PacketHeader    *st_iAP2PacketHeader;

    MFI_iAP2ReplyACK(pdev);

    ar_osDelay(1);

    ((STRU_iAP2PacketHeader *)g_iDentificationInfo)->seq_num = g_mfiSeqNum;
    ((STRU_iAP2PacketHeader *)g_iDentificationInfo)->ack_num =
                            ((STRU_iAP2PacketHeader *)g_u8MFIRecvBuff)->seq_num;
    ((STRU_iAP2PacketHeader *)g_iDentificationInfo)->session_identifier =
                            ((STRU_iAP2PacketHeader *)g_u8MFIRecvBuff)->session_identifier;
    ((STRU_iAP2PacketHeader *)g_iDentificationInfo)->check_sum =
                            MFI_iAP2CalcChk(g_iDentificationInfo, 8);

    g_iDentificationInfo[IDENTIFICATION_LEN - 1] = MFI_iAP2CalcChk(&g_iDentificationInfo[9], IDENTIFICATION_LEN - 10);

    USBD_HID_SendReport(pdev, g_iDentificationInfo, IDENTIFICATION_LEN, MFI_IN_ADDR);
}

uint8_t g_powerUpadate[64];

HAL_RET_T MFI_iAP2StartPowerUpdate(USBD_HandleTypeDef *pdev)
{
    uint8_t payload[]   = {
        0x00, 0x04, 0x00, 0x00,
        0x00, 0x04, 0x00, 0x01,
        0x00, 0x04, 0x00, 0x06
    };

    STRU_iAP2PacketHeader *st_iAP2PacketHeader;
    STRU_iAP2ControlDef *st_iAP2Control;

    st_iAP2PacketHeader = (STRU_iAP2PacketHeader *)g_powerUpadate;
    st_iAP2Control      = (STRU_iAP2ControlDef *)((uint8_t *)st_iAP2PacketHeader + sizeof(STRU_iAP2PacketHeader));
    uint8_t *parameter  = (uint8_t *)st_iAP2Control + sizeof(STRU_iAP2ControlDef);

    st_iAP2PacketHeader->start_msb = 0xFF;
    st_iAP2PacketHeader->start_lsb = 0x5A;
    st_iAP2PacketHeader->len_msb   = 0x00;
    st_iAP2PacketHeader->len_lsb   = 0x1C;      // 9 (iAP2 Link Packet Header) + 6 (Ctrl Session Header) + 4 * 3 (Parameter) + 1 (CheckSum)
    st_iAP2PacketHeader->ctrl_byte = IAP2_MSG_TYPE_ACK;
    st_iAP2PacketHeader->seq_num   = g_mfiSeqNum;
    st_iAP2PacketHeader->ack_num = ((STRU_iAP2PacketHeader *)g_u8MFIRecvBuff)->seq_num;
    st_iAP2PacketHeader->session_identifier = ((STRU_iAP2PacketHeader *)g_u8MFIRecvBuff)->session_identifier;
    DLOG_Critical("session_identifier: %02A", st_iAP2PacketHeader->session_identifier);
    st_iAP2PacketHeader->check_sum = MFI_iAP2CalcChk((uint8_t *)st_iAP2PacketHeader, 8);

    st_iAP2Control->start_msb      = 0x40;
    st_iAP2Control->start_lsb      = 0x40;
    st_iAP2Control->len_msb        = 0x00;
    st_iAP2Control->len_lsb        = 0x12;
    st_iAP2Control->msg_id_msb     = 0xAE;      // 0xAE00
    st_iAP2Control->msg_id_lsb     = 0x00;      // 0xAE00

    memcpy(parameter, payload, sizeof(payload));

    g_powerUpadate[0x1C - 1] = MFI_iAP2CalcChk(&g_powerUpadate[9], 0x1C - 10);

/*
    int index = 0;
    uint8_t m_buffer[1024];
    memset(m_buffer, 0x00, 1024);
    uint8_t *pp = (uint8_t *)st_iAP2PacketHeader;
    for (int j = 0; j < 0x1C; j++) {
        index +=  sprintf(m_buffer + index, "%02X ", pp[j]);
    }
    DLOG_Critical("Buffer is: %s \n", m_buffer);
**/
    USBD_HID_SendReport(g_pdev, (uint8_t *)st_iAP2PacketHeader, 0x1C, MFI_IN_ADDR);

    return HAL_OK;    
}


uint16_t MFI_iAP2GetMsgID(uint8_t *msg_content, uint32_t msg_len)
{
    STRU_iAP2PacketHeader    *st_iAP2PacketHeader;
    STRU_iAP2ControlDef      *st_iAP2Control;
    uint16_t                  msg_id = 0;

    st_iAP2PacketHeader = (STRU_iAP2PacketHeader *)msg_content;
    st_iAP2Control      = (STRU_iAP2ControlDef *)(msg_content + sizeof(STRU_iAP2PacketHeader));

    if ((st_iAP2PacketHeader->start_msb == 0xFF)&&
        (st_iAP2PacketHeader->start_lsb == 0x5A))
    {
        if (st_iAP2PacketHeader->ctrl_byte == (IAP2_MSG_TYPE_SYN | IAP2_MSG_TYPE_ACK))
        {
            msg_id = IAP2_MSG_SYN_ACK;
        }
        else if ((st_iAP2Control->start_msb == 0x40)&&
                 (st_iAP2Control->start_lsb == 0x40))
        {
            msg_id   = st_iAP2Control->msg_id_msb;
            msg_id <<= 8;
            msg_id  += st_iAP2Control->msg_id_lsb;
        }
    }

    return msg_id;
}


void MFI_iAP2MsgParse(void *p)
{
    STRU_SysEvent_MFI_MSG   *st_mfiNotifyMsg = (STRU_SysEvent_MFI_MSG *)p;
    USBD_HandleTypeDef      *pdev = NULL;
    uint32_t                 msg_len;
    uint8_t                 *msg_buf;
    uint16_t                 msg_id;

    pdev    = (USBD_HandleTypeDef *)(st_mfiNotifyMsg->usb_dev);
    g_pdev  = (USBD_HandleTypeDef *)(st_mfiNotifyMsg->usb_dev);
    msg_len = st_mfiNotifyMsg->msg_len;
    msg_buf = (uint8_t *)(st_mfiNotifyMsg->msg_buf);

    if ((st_mfiNotifyMsg->msg_len == 0)&&(st_mfiNotifyMsg->msg_buf == 0))
    {
        MFI_iAP2DetectApple(pdev);
    }
    else if ((st_mfiNotifyMsg->msg_len == sizeof(s_iap2_detect))&&
             (memcmp(s_iap2_detect, (uint8_t *)st_mfiNotifyMsg->msg_buf, sizeof(s_iap2_detect)) == 0))
    {
        g_mfiSeqNum = 0;
        MFI_iAP2ParamSync(pdev);
    }
    else
    {
        msg_id = MFI_iAP2GetMsgID(msg_buf, msg_len);

        switch (msg_id)
        {
        case IAP2_MSG_SYN_ACK:
            MFI_iAP2ParamSyncComplete(pdev);
            break;

        case IAP2_MSG_REQ_AUTHENTICATION_CERTIFICATE:
            MFI_iAP2RequestAuthenticationCertificate(pdev);
            break;

        case IAP2_MSG_REQ_AUTHENTICATION_CHALLENGE_RESP:
            MFI_iAP2ChallengeResponse(pdev);
            break;

        case IAP2_MSG_AUTHENTICATION_SUCCEEDED:
            break;

        case IAP2_MSG_START_IDENTIFICATION:
            MFI_iAP2ReportInformation(pdev);
            break;

        case IAP2_MSG_IDENTIFICATION_ACCEPTED:
            MFI_iAP2ReplyACK(pdev);
/*
            ar_osDelay(1);
            MFI_iAP2StartPowerUpdate(pdev);
            break;

        case IAP2_MSG_POWER_UPDATE:
            MFI_iAP2ReplyACK(pdev);
//          g_mfiSeqNum = 0;
**/
            break;
/*
        case IAP2_MSG_AUTHENTICATION_FAILED:
            DLOG_Critical("******** IAP2_MSG_AUTHENTICATION_FAILED *********\n");
            MFI_iAP2ReplyACK(pdev);
            g_mfiSeqNum = 0;

            HAL_Delay(100);
            MFI_iAP2DetectApple(pdev);
            break;
**/
        default:
            break;
        }
    }
}

uint8_t g_app_launch[512];
uint32_t app_launch_seq = 0;
#define IAP2_MSG_APP_LAUNCH         0xEA02

HAL_RET_T HAL_MFI_ReqAppLaunch(void)
{
    HAL_RET_T ret = HAL_OK;
    uint8_t bundle_id[] = "cn.ZeroZeroRobotics.HoverCamera.vcoptr";

    HAL_MFI_ReqAppLaunchWithParam(bundle_id, strlen(bundle_id));
}


HAL_RET_T HAL_MFI_ReqAppLaunchWithParam(uint8_t *payload, uint8_t len)
{
    HAL_MFI_ReqAppLaunchWithAlert(payload, len, 0);
}

HAL_RET_T HAL_MFI_ReqAppLaunchWithAlert(uint8_t *payload, uint8_t len, uint8_t user_alert)
{
    STRU_iAP2PacketHeader *st_iAP2PacketHeader;
    STRU_iAP2ControlDef *st_iAP2Control;
    uint8_t *m_payload = payload;
    uint8_t pay_len = strlen(m_payload);

    if (g_pdev == NULL) {
        DLOG_Critical("*************** g_pdev == NULL *************\n");
        return HAL_MFI_ERR;
    }
    DLOG_Critical("Accessory request App launch !!!\n");

    st_iAP2PacketHeader = (STRU_iAP2PacketHeader *)g_app_launch;
    st_iAP2Control      = (STRU_iAP2ControlDef *)((uint8_t *)st_iAP2PacketHeader + sizeof(STRU_iAP2PacketHeader));
    uint8_t *payload_t = (uint8_t *)st_iAP2Control + sizeof(STRU_iAP2ControlDef);


    st_iAP2PacketHeader->start_msb = 0xFF;
    st_iAP2PacketHeader->start_lsb = 0x5A;
    st_iAP2PacketHeader->len_msb = 0x00;
    st_iAP2PacketHeader->len_lsb =  9 + (6 + (pay_len + 4 + 1) + 5) + 1;
    st_iAP2PacketHeader->ctrl_byte = IAP2_MSG_TYPE_ACK;
    st_iAP2PacketHeader->seq_num = g_mfiSeqNum;          //app_launch_seq;
    st_iAP2PacketHeader->ack_num = ((STRU_iAP2PacketHeader *)g_u8MFIRecvBuff)->seq_num;
    st_iAP2PacketHeader->session_identifier = 0x0a;
    st_iAP2PacketHeader->check_sum = MFI_iAP2CalcChk((uint8_t *)st_iAP2PacketHeader, 8);
    
    st_iAP2Control->start_msb = 0x40;
    st_iAP2Control->start_lsb = 0x40;
    st_iAP2Control->len_msb = 0x00;
    st_iAP2Control->len_lsb = 6 +  (pay_len + 4 + 1) + 5;
    st_iAP2Control->msg_id_msb = (uint8_t)(IAP2_MSG_APP_LAUNCH >> 8);
    st_iAP2Control->msg_id_lsb = (uint8_t)IAP2_MSG_APP_LAUNCH;
    payload_t[0] = 0x00;
    payload_t[1] = 0x04 + pay_len + 1;
    payload_t[2] = 0x00;
    payload_t[3] = 0x00;

    for (int j = 0; j < pay_len; j++)
        payload_t[4 + j] = m_payload[j];

    payload_t[4 + pay_len] = 0x00;

    payload_t[5 + pay_len] = 0x00;
    payload_t[6 + pay_len] = 0x05;
    payload_t[7 + pay_len] = 0x00;
    payload_t[8 + pay_len] = 0x01;
    payload_t[9 + pay_len] = user_alert;

    payload_t[10 + pay_len] = MFI_iAP2CalcChk(&g_app_launch[9], 6 +  (pay_len + 4 + 1) + 5);
    int index = 0;
    uint8_t m_buffer[1024];
    memset(m_buffer, 0x00, 1024);
    uint8_t *pp = (uint8_t *)st_iAP2PacketHeader;
/*
    for (int j = 0; j <  6 +  (pay_len + 4 + 1) + 5 + 1 + 9; j++) {
        index +=  sprintf(m_buffer + index, "%02X ", pp[j]);
    }
    DLOG_Critical("Buffer is: %s \n", m_buffer);
**/
    USBD_HID_SendReport(g_pdev, (uint8_t *)st_iAP2PacketHeader, 9 + (6 + (pay_len + 4 + 1) + 5) + 1, MFI_IN_ADDR);
    app_launch_seq++; 
    g_mfiSeqNum++;   
    return HAL_OK;    
}

static void bytesToUnicode(uint8_t *bytes_array, uint8_t *pbuf, uint8_t len)
{
  uint8_t i = 0;
  uint8_t value = 0;

  for(i = 0; i < len; i++) {
    value = bytes_array[i];

    if((value >> 4) < 0x0A) {
      pbuf[2 * i] = (value >> 4) + '0';
    } else {
      pbuf[2 * i] = (value >> 4) + 'A' - 10;
    }

    value = value & 0x0F;
    if(value < 0x0A) {
        pbuf[2 * i + 1] = value + '0';
    } else {
        pbuf[2 * i + 1] = value + 'A' - 10;
    }
  }
}

void MFI_Set_SerialNum(int index)
{
    uint8_t chip_id[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
    uint8_t serial[32];

    memcpy((void *)chip_id, (uint8_t *)(SRAM_SHARE_FLAG_ST_ADDR + SHARE_FLAG_CHIP_ID_OFFSET), 5);
    bytesToUnicode(chip_id, serial, 5);
    memcpy(g_iDentificationInfo + index, serial, 10);
}

void MFI_Init(ENUM_HAL_GPIO_NUM e_gpio_rst, ENUM_HAL_GPIO_NUM e_gpio_host, ENUM_HAL_I2C_COMPONENT e_i2c)
{
#ifdef C201S_ZZ
    int idx = 78;           //Serial index
    MFI_Set_SerialNum(idx);
#else
    int idx = 48;           //Serial index
    MFI_Set_SerialNum(idx); 
#endif

    g_mfi_i2c_addr = 0x11;
    g_i2cMFIComponent = e_i2c;

    HAL_GPIO_OutPut(e_gpio_rst);
    HAL_GPIO_SetPin(e_gpio_rst, HAL_GPIO_PIN_SET);

    HAL_GPIO_InPut(e_gpio_host);

    USBD_HID_SetHostTypeGPIO(e_gpio_host);

    HAL_I2C_MasterInit(g_i2cMFIComponent, g_mfi_i2c_addr, HAL_I2C_FAST_SPEED);

    SYS_EVENT_RegisterHandler(SYS_EVENT_ID_IAP2_MSG_EVENT, MFI_iAP2MsgParse);
}

void MFI_V3_Init(ENUM_HAL_GPIO_NUM e_gpio_host, ENUM_HAL_I2C_COMPONENT e_i2c)
{
#ifdef C201S_ZZ
    int idx = 78;           //Serial index
    MFI_Set_SerialNum(idx);
#else
    int idx = 48;           //Serial index
    MFI_Set_SerialNum(idx);
#endif

    g_mfi_i2c_addr = 0x10;

    g_i2cMFIComponent = e_i2c;

    HAL_GPIO_InPut(e_gpio_host);

    USBD_HID_SetHostTypeGPIO(e_gpio_host);

    HAL_I2C_MasterInit(g_i2cMFIComponent, g_mfi_i2c_addr, HAL_I2C_STANDARD_SPEED);

    SYS_EVENT_RegisterHandler(SYS_EVENT_ID_IAP2_MSG_EVENT, MFI_iAP2MsgParse);   
}

