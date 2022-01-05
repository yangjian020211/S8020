#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "debuglog.h"
#include "hal_nv.h"
#include "test_hal_nv.h"
#include "hal_bb.h"
#include "nor_flash.h"
#include "memory_config.h"
#include "hal.h"
#include "jnc_hal_sys.h"
#include "test_jnc_api.h"
#include "cpu_info.h"

#define     JNC_LEN_PER_WR          (50)
#define     JNC_MAX_LEN          (60*1024)

uint8_t jnc_wData[JNC_LEN_PER_WR];


void command_JncUsrDataWrite(void)
{
    uint32_t wLen = 0;
    uint32_t i;
    uint32_t cnt = 0;

    for (i = 0; i < JNC_LEN_PER_WR; i++)
    {
        jnc_wData[i] = i;
    }

    while((wLen + JNC_LEN_PER_WR) < (JNC_MAX_LEN / 60))
    {
        ss_hal_data_persist_write(jnc_wData, wLen, JNC_LEN_PER_WR);
        DLOG_Critical("%d:\t0x%x", cnt++, wLen);
        wLen += JNC_LEN_PER_WR;
        
        HAL_Delay(1000);
    }

    DLOG_Critical("finished %d", (wLen - JNC_LEN_PER_WR));
}

void command_JncUsrDataRead(void)
{
    uint8_t rData[JNC_LEN_PER_WR];
    uint32_t rLen = 0;
    uint32_t i;
    uint32_t cnt = 0;

    CPUINFO_DCacheInvalidateByAddr((uint32_t *)(0x10000000 + (3*1024+512)*1024), 512*1024);

    for (i = 0; i < JNC_LEN_PER_WR; i++)
    {
        jnc_wData[i] = i;
    }

    while((rLen + JNC_LEN_PER_WR) < (JNC_MAX_LEN / 60))
    {
        memset(rData, 0x00, JNC_LEN_PER_WR);
        if (false == ss_hal_data_persist_read(rData, rLen, JNC_LEN_PER_WR))
        {
            DLOG_Critical("read err %d:\t0x%x", cnt, rLen);
            break;
        }
        if (0 != memcmp(jnc_wData, rData, JNC_LEN_PER_WR))
        {
            DLOG_Critical("***************w**************");
            JncTelemPrint(jnc_wData, JNC_LEN_PER_WR);
            DLOG_Critical("****************r*************\n");
            JncTelemPrint(rData, JNC_LEN_PER_WR);
            DLOG_Critical("***************e**************\n");
            DLOG_Critical("cmp err %d:\t0x%x", cnt, rLen);
            break;
        }
        DLOG_Critical("%d:\t0x%x", cnt++, rLen);
        rLen += JNC_LEN_PER_WR;
        HAL_Delay(200);
    }

    DLOG_Critical("finished %d", (rLen - JNC_LEN_PER_WR));
}


void command_JncUsrDataDestroy(uint8_t *addr)
{
    uint8_t data[1024];
    uint32_t i;
    uint32_t offsetAddr = strtoul(addr, NULL, 0);

    DLOG_Info("baseAddr:0x%x offsetAddr:0x%x", NV_FLASH_USR_ADDR1, offsetAddr);

    memset(data, 0x12, sizeof(data));
    
    NOR_FLASH_WriteByteBuffer(NV_FLASH_USR_ADDR1 + offsetAddr, data, sizeof(data));
}


void command_JncInit(void)
{
    ss_hal_board_init();
    ss_hal_telem_init();

    DLOG_Critical("init finished");
}

void command_JncRcWrite(void)
{
    ss_hal_rc_write(0, 1000);
    ss_hal_rc_write(1, 1001);
    ss_hal_rc_write(2, 1002);
    ss_hal_rc_write(3, 1003);
    DLOG_Critical("rc write finished");
}

void command_JncRcRead(void)
{
    DLOG_Critical("%d %d %d %d",
                    ss_hal_rc_read(0),
                    ss_hal_rc_read(1),
                    ss_hal_rc_read(2),
                    ss_hal_rc_read(3));
}

void command_JncTelemWrite(void)
{
    uint8_t txBuf[1000];
    uint32_t i = 0;
    uint16_t txLen = 0;

    for(i=0; i<sizeof(txBuf); i++)
    {
        txBuf[i] = i;
    }

    txLen = ss_hal_telem_write(BB_COM_SESSION_2, txBuf, sizeof(txBuf));
    DLOG_Critical("session %d send %d bytes.", BB_COM_SESSION_2, txLen);
    txLen = ss_hal_telem_write(BB_COM_SESSION_3, txBuf, sizeof(txBuf));
    DLOG_Critical("session %d send %d bytes.", BB_COM_SESSION_3, txLen);
}

void JncTelemPrint(uint8_t *buf, uint16_t len)
{
    uint16_t algn;
    uint16_t unalgn;
    uint16_t i = 0;
    uint16_t j = 0;
    
    algn = len / 20;
    unalgn = len % 20;
    
    for(i=0; i<algn; i++)
    {
        DLOG_Critical("%02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x",
            buf[i*20+0],buf[i*20+1],buf[i*20+2],buf[i*20+3],buf[i*20+4],
            buf[i*20+5],buf[i*20+6],buf[i*20+7],buf[i*20+8],buf[i*20+9],
            buf[i*20+10],buf[i*20+11],buf[i*20+12],buf[i*20+13],buf[i*20+14],
            buf[i*20+15],buf[i*20+16],buf[i*20+17],buf[i*20+18],buf[i*20+19]);
        HAL_Delay(100);
    }

    for(j=0; j<unalgn; j++)
    {
        DLOG_Critical("%02x", buf[i*20+j]);
        HAL_Delay(10);
    }
}


void command_JncTelemRead(void)
{
    uint32_t i = 0;
    uint16_t rxLen = 0;
    uint8_t rxBuf[1024];

#if 1
    DLOG_Critical("**************** session 2 start *************\n\n");
    memset(rxBuf, 0x00, sizeof(rxBuf));
    rxLen = ss_hal_telem_read(BB_COM_SESSION_2, rxBuf, sizeof(rxBuf));
    JncTelemPrint(rxBuf, rxLen);
    DLOG_Critical("**************** session 2 end *************\n\n");
#endif

#if 1    
    DLOG_Critical("**************** session 3 start *************");
    memset(rxBuf, 0x00, sizeof(rxBuf));
    rxLen = ss_hal_telem_read(BB_COM_SESSION_3, rxBuf, sizeof(rxBuf));
    JncTelemPrint(rxBuf, rxLen);
    DLOG_Critical("**************** session 3 end *************\n\n");
#endif
}



