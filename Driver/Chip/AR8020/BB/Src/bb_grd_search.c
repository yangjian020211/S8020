#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include "debuglog.h"
#include "ar8020.h"
#include "interrupt.h"
#include "timer.h"
#include "reg_rw.h"
#include "bb_regs.h"
#include "bb_ctrl_internal.h"
#include "bb_snr_service.h"
#include "bb_grd_ctrl.h"
#include "bb_grd_sweep.h"
#include "bb_uart_com.h"
#include "memory_config.h"
#include "sys_event.h"
#include "systicks.h"

static uint8_t u8_searchingVtIdArray_bake[VT_ID_SIZE];
static uint8_t u8_searchingVtIdArray[VT_ID_SIZE];
static uint8_t u8_searchingSkyRcIdArray[RC_ID_SIZE] = {0x00};

static uint8_t u8_saveRcIdVtIdAfterLock = 1;   //default case: after lock save rc id and vt id to flash.

static ENUM_GRD_SEARCH_STATES e_grdRcSearchState = GRD_RC_SEARCH_END;

static void BB_grd_searchStateChangeEvent(ENUM_GRD_SEARCH_STATES state, uint8_t *rcid, uint8_t *vtid)
{
    STRU_SysEvent_DEV_BB_STATUS bb_searchStatus = {
        .pid = BB_GRD_SEARCHING_STATES_CHAGE,
        .e_grd_searchState = state,
    };

    if (rcid)
    {
        memcpy(bb_searchStatus.rcid, rcid, RC_ID_SIZE);
    }

    if (vtid)
    {
        memcpy(bb_searchStatus.vtid, vtid, VT_ID_SIZE);
    }

    SYS_EVENT_NotifyInterCore(SYS_EVENT_ID_BB_EVENT, (void *)&bb_searchStatus);
}

/*
 * set rc id to register and save rc/vt id, set rc id register, copy rc id sram buffer
*/
static void grd_SetSaveRCId(uint8_t *pu8_rcid, uint8_t *pu8_vtid, uint8_t flag)
{
    memcpy((void *)context.rcid, (void *)pu8_rcid, RC_ID_SIZE);
    memcpy((void *)context.vtid, (void *)pu8_vtid, VT_ID_SIZE);

    BB_GenHashId((uint8_t *)context.rcid,(uint8_t *)context.vtid,BB_GRD_MODE);
    BB_GenHashId((uint8_t *)context.rcid,(uint8_t *)context.vtid,BB_SKY_MODE);
    grd_SetRCId((uint8_t *)context.hashRcid);
    if(flag)
    {
        BB_saveRcid(pu8_rcid, pu8_vtid);
    }
}


uint8_t BB_grd_checkVtIdMatch(void)
{
    uint8_t sky_vtid[2] = { BB_ReadReg(PAGE2, 0xd3),
                            BB_ReadReg(PAGE2, 0xd4)
                          };
    if (BB_grd_isSearching())
    {
        return ((sky_vtid[0] == u8_searchingVtIdArray[0]) && (sky_vtid[1] == u8_searchingVtIdArray[1]));
    }
    else
    {
        if(context.hashVtid[0] == 0xff && context.hashVtid[1] == 0xff)
        {
            return 1;
        }
        return ((sky_vtid[0] == context.hashVtid[0]) && (sky_vtid[1] == context.hashVtid[1]));
    }
}


/*
 * start rc search, when start search, ground use the flash id as rc id, should no match rc id from sky
 * todo: generate vt ids, use systicks
*/
static void BB_grd_startRcSearch(uint8_t u8_autoSave)
{
    uint16_t ticks;// = SysTicks_GetTickCount();
    uint8_t ground_vt_msg[7];

    //BB_GenHashId((uint8_t *)context.rcid,(uint8_t *)context.vtid,BB_SKY_MODE);
    ticks = crc16((uint8_t *)context.chipid,RC_ID_SIZE);
    u8_searchingVtIdArray[0] = (ticks & 0xff);
    u8_searchingVtIdArray[1] = ((ticks >> 8) & 0xff);
    u8_searchingVtIdArray_bake[0] = u8_searchingVtIdArray[0];
    u8_searchingVtIdArray_bake[1] = u8_searchingVtIdArray[1];

    memcpy(ground_vt_msg, u8_searchingVtIdArray, VT_ID_SIZE);
    memcpy(ground_vt_msg + VT_ID_SIZE, ((uint8_t *)(SRAM_SHARE_FLAG_ST_ADDR + SHARE_FLAG_CHIP_ID_OFFSET)), RC_ID_SIZE);

    BB_DtSendToBuf(DT_NUM_GRD_SEND_VT_ID, ground_vt_msg);

    memset(u8_searchingSkyRcIdArray, 0, RC_ID_SIZE);
    e_grdRcSearchState = GRD_WAIT_SKY_RC_ID;
    BB_grd_searchStateChangeEvent(e_grdRcSearchState, NULL, NULL);

    u8_saveRcIdVtIdAfterLock = u8_autoSave;
    grd_SetRCId((uint8_t *)(SRAM_SHARE_FLAG_ST_ADDR + SHARE_FLAG_CHIP_ID_OFFSET));

    DLOG_Warning("==>GRD_WAIT_SKY_RC_ID vtid = %x %x %d", u8_searchingVtIdArray[0], u8_searchingVtIdArray[1], u8_saveRcIdVtIdAfterLock);

}


/*
 * stop send vt id
*/
static void BB_grd_stopRcSearch(void)
{
    BB_DtStopSend(DT_NUM_GRD_SEND_VT_ID);
    grd_SetRCId((uint8_t *)context.hashRcid);

    e_grdRcSearchState = GRD_RC_SEARCH_END;

    BB_grd_searchStateChangeEvent(GRD_RC_SEARCH_END, (uint8_t *)context.rcid, (uint8_t *)context.vtid);

    DLOG_Warning("==> GRD_RC_SEARCH_END");

}


/*
 * Set RC id to ground RC id register, wait vt id match
*/
void BB_grd_setConnectRcId(uint8_t *pu8_rcId, uint8_t * pu8_vtid)
{
    //save rc id, vt id
    memcpy((void *)context.rcid, pu8_rcId, RC_ID_SIZE);
    if (pu8_vtid)
    {
        memcpy((void *)context.vtid, pu8_vtid, VT_ID_SIZE);
    }
    memcpy(u8_searchingSkyRcIdArray, pu8_rcId, RC_ID_SIZE);

    BB_GenHashId((uint8_t *)context.rcid,(uint8_t *)context.vtid,BB_GRD_MODE);
    BB_GenHashId((uint8_t *)context.rcid,(uint8_t *)context.vtid,BB_SKY_MODE);
    grd_SetRCId((uint8_t *)context.hashRcid);
    BB_grd_stopRcSearch();

    DLOG_Critical("Lock to id rc: %x %x %x %x %x vt: %x %x", pu8_rcId[0], pu8_rcId[1], pu8_rcId[2], pu8_rcId[3], pu8_rcId[4], 
                                                             context.vtid[0], context.vtid[1]);
}


/*
 * use ground default rc id and softreset
*/
static int BB_grd_setDisconnectRcId(void)
{
    BB_DtStopSend(DT_NUM_GRD_SEND_VT_ID);
    grd_SetRCId((uint8_t *)context.hashRcid);
    BB_softReset(BB_GRD_MODE);

    DLOG_Warning("DisconnectRcId");
}

/*
 * set rcd id to BB registers and copy to sram share memory
*/
void grd_SetRCId(uint8_t *pu8_id)
{
    uint8_t *p = (uint8_t *)(SRAM_SHARE_FLAG_ST_ADDR + SHARE_FLAG_RC_ID_OFFSET);
	int i=0;
	
    BB_WriteReg(PAGE2, GRD_RC_ID_BIT39_32_REG, pu8_id[0]);
    BB_WriteReg(PAGE2, GRD_RC_ID_BIT31_24_REG, pu8_id[1]);
   	BB_WriteReg(PAGE2, GRD_RC_ID_BIT23_16_REG, pu8_id[2]);
    BB_WriteReg(PAGE2, GRD_RC_ID_BIT15_08_REG, pu8_id[3]);
    BB_WriteReg(PAGE2, GRD_RC_ID_BIT07_00_REG, pu8_id[4]);
	
	//for(i=0;i<5;i++) BB_WriteReg(PAGE2, GRD_RC_ID_BIT39_32_REG+i, pu8_id[i]);

    memcpy(p, (uint8_t *)context.rcid, RC_ID_SIZE);
}


void BB_grd_handleRcSearchCmd(STRU_WIRELESS_CONFIG_CHANGE* pcmd)
{
    uint8_t  class   = pcmd->u8_configClass;
    uint8_t  item    = pcmd->u8_configItem;
    uint32_t value   = pcmd->u32_configValue;
    uint32_t value1  = pcmd->u32_configValue1;

    uint8_t u8_rcArray[5]   = {(value>>24)&0xff, (value>>16)&0xff, (value>>8)&0xff, value&0xff, (value1&0xff)};
    uint8_t u8_vtIdArray[2] = {(value1>>8)&0xff, (value1>>16)&0xff};

    DLOG_Warning("%d %d %0.8x %0.8x", class, item, value, value1);

    if (WIRELESS_AUTO_SEARCH_ID == class)
    {
        switch (item)
        {
            case RCID_DISCONNECT:
                BB_grd_setDisconnectRcId();
                break;

            case RCID_CONNECT_ID:
                BB_grd_setConnectRcId(u8_rcArray, u8_vtIdArray);
                break;

            case RCID_AUTO_SEARCH:
                BB_grd_startRcSearch(value);
                break;

            case RCID_SAVE_RCID:
                grd_SetSaveRCId(u8_rcArray, u8_vtIdArray, 1);
                break;
                
            case SET_TMP_RC_VT_ID:
                grd_SetSaveRCId(u8_rcArray, u8_vtIdArray, 0);
                break;

            case RCID_STOP_SEARCH:
                BB_grd_stopRcSearch();
                break;
    
           default:
                //DLOG_Warning("error item=%d", item);
                break;
        }
    }
}


/*
 *sky already got the vt id, ground can stop search, and save rc id and vt id to flash
*/
void BB_grd_checkSearchEnd(void)
{
    if(e_grdRcSearchState == GRD_WAIT_SKY_VT_MATCH_DELAY14MS)
    {
        e_grdRcSearchState = GRD_WAIT_SKY_VT_MATCH;
    }
    else if (e_grdRcSearchState == GRD_WAIT_SKY_VT_MATCH && context.locked == 0x03) /* vt lock and vt id match*/
    {
        if (u8_saveRcIdVtIdAfterLock)
        {
            grd_SetSaveRCId(u8_searchingSkyRcIdArray, u8_searchingVtIdArray_bake, 1);
        }

        BB_grd_stopRcSearch();
        DLOG_Warning("search end rc:%x %x %x %x %x vt: %x %x",
                      u8_searchingSkyRcIdArray[0], u8_searchingSkyRcIdArray[1], u8_searchingSkyRcIdArray[2], u8_searchingSkyRcIdArray[3], u8_searchingSkyRcIdArray[4], 
                      u8_searchingVtIdArray_bake[0], u8_searchingVtIdArray_bake[1]);
    }
}

/*
 * when ground receive sky rc id, then ground send vt ids until end
*/
void BB_grd_ackSkyRcIdSendVtId(uint8_t *pu8_skyrcid)
{
    if (GRD_WAIT_SKY_RC_ID == e_grdRcSearchState)
    {
        BB_grd_searchStateChangeEvent(GRD_WAIT_SKY_VT_MATCH, NULL, NULL);
        memcpy(u8_searchingSkyRcIdArray, pu8_skyrcid, RC_ID_SIZE);
        BB_GenHashId((uint8_t *)u8_searchingSkyRcIdArray,(uint8_t *)u8_searchingVtIdArray,BB_GRD_MODE);
        BB_GenHashId((uint8_t *)u8_searchingSkyRcIdArray,(uint8_t *)u8_searchingVtIdArray,BB_SKY_MODE);
        u8_searchingVtIdArray[0] = context.hashVtid[0];
        u8_searchingVtIdArray[1] = context.hashVtid[1];
        grd_SetRCId((uint8_t *)context.hashRcid);

        e_grdRcSearchState = GRD_WAIT_SKY_VT_MATCH_DELAY14MS;
        DLOG_Warning("==>GRD_WAIT_SKY_VT_MATCH DELAY 14MS %x %x %x %x %x vt: %x %x", 
                      pu8_skyrcid[0], pu8_skyrcid[1], pu8_skyrcid[2], pu8_skyrcid[3], pu8_skyrcid[4],
                      u8_searchingVtIdArray[0], u8_searchingVtIdArray[1]);
    }

    BB_DtStopSend(DT_NUM_GRD_SEND_VT_ID);
}

uint8_t BB_grd_isSearching(void)
{
    return (GRD_RC_SEARCH_END != e_grdRcSearchState);
}
