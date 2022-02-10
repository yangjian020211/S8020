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
#include "bb_sky_ctrl.h"
#include "bb_uart_com.h"
#include "memory_config.h"
#include "sys_event.h"
#include "systicks.h"


typedef enum
{
    NO_SEARCH_RC_CMD    = 0,
    CMD_SEARCH_RC_START = 1,
    CMD_SEARCH_RC_END   = 2,
} ENUM_CMD_RC_SEARCH;


uint8_t u8_BackupVtIds[2];
uint8_t tmp_vtid[2];

extern STRU_SKY_STATUS stru_skystatus;
static ENUM_SKY_SEARCH_STATE e_skyRcSearchState = SKY_RC_SEARCH_END;

/*
 * user request to start search rc id
*/
void BB_sky_requestStartSearch(void)
{
    tmp_vtid[0]=0;tmp_vtid[1]=0;
    stru_skystatus.u8_userRequestRcSearch = CMD_SEARCH_RC_START;
}

uint8_t BB_sky_inSearching(void)
{
    return (e_skyRcSearchState != SKY_RC_SEARCH_END);
}

/*
 * sky stop search mode
*/
void BB_sky_requestStopSearch(void)
{
    stru_skystatus.u8_userRequestRcSearch = CMD_SEARCH_RC_END;
    tmp_vtid[0]=0;tmp_vtid[1]=0;
}



/*
 * when sky got vt id, set to vt it registers
*/
void BB_sky_ackVtidMessage(uint8_t *pu8_vtid)
{
    if (SKY_WAIT_VT_ID_MSG == e_skyRcSearchState && (tmp_vtid[0] != pu8_vtid[0] || tmp_vtid[1] != pu8_vtid[1]))
    {
        STRU_SysEvent_DEV_BB_STATUS bb_searchStatus = {
            .pid = BB_SKY_SEARCHING_STATES_CHAGE,
            .e_sky_searchState = SKY_WAIT_RC_ID_MATCH,
        };

        memcpy((void *)context.vtid, pu8_vtid, VT_ID_SIZE);
        BB_GenHashId((uint8_t *)context.rcid, (uint8_t *)context.vtid,BB_SKY_MODE);
        BB_GenHashId((uint8_t *)context.rcid, (uint8_t *)context.vtid,BB_GRD_MODE);
        BB_sky_vtId((uint8_t *)context.vtid);
        sky_setRcId((uint8_t *)context.hashRcid);

        tmp_vtid[0] = pu8_vtid[0];
        tmp_vtid[1] = pu8_vtid[1];

        e_skyRcSearchState = SKY_WAIT_RC_ID_MATCH;
        memcpy(stru_skystatus.cur_groundRcid, &pu8_vtid[2], RC_ID_SIZE);
        SYS_EVENT_NotifyInterCore(SYS_EVENT_ID_BB_EVENT, (void *)&bb_searchStatus);

        DLOG_Warning("%x %x ==> SKY_WAIT_RC_ID_MATCH", pu8_vtid[0], pu8_vtid[1]);
    }
}
void BB_sky_setSearchState(ENUM_SKY_SEARCH_STATE state)
{
    e_skyRcSearchState = state;
}

/*
 * when sky got vt id, 
*/
void BB_sky_nackVtidMessage(uint8_t *pu8_vtid)
{
    if (SKY_WAIT_VT_ID_MSG == e_skyRcSearchState)
    {
        STRU_SysEvent_DEV_BB_STATUS bb_searchStatus = {
            .pid = BB_SKY_SEARCHING_STATES_CHAGE,
            .e_sky_searchState = SKY_WAIT_RC_ID_MATCH,
        };

        memcpy((void *)bb_searchStatus.vtid, pu8_vtid, VT_ID_SIZE);
        //BB_sky_vtId(pu8_vtid);

        //tmp_vtid[0] = pu8_vtid[0];
        //tmp_vtid[1] = pu8_vtid[1];

        //e_skyRcSearchState = SKY_WAIT_RC_ID_MATCH;
        memcpy(bb_searchStatus.rcid, &pu8_vtid[2], RC_ID_SIZE);
        bb_searchStatus.agc1 = BB_ReadReg(PAGE2, AAGC_2_RD);
        bb_searchStatus.agc2 = BB_ReadReg(PAGE2, AAGC_3_RD);
        SYS_EVENT_NotifyInterCore(SYS_EVENT_ID_BB_EVENT, (void *)&bb_searchStatus);

        //DLOG_Warning("%x %x ==> SKY_GOT_VT_ID", pu8_vtid[0], pu8_vtid[1]);
    }
}

/*
 * if sky find rc crc match, it means search end, and then save rc_id & vt_id to flash
*/
void BB_sky_checkSearchEnd(uint8_t u8_lockStatus)
{
    static ENUM_SKY_SEARCH_STATE pre_state = SKY_RC_SEARCH_END;

    if (SKY_WAIT_RC_ID_MATCH == pre_state && SKY_ID_CRC_MATCH(u8_lockStatus))// && (1 == stru_skystatus.flag_groundVtMatch))
    {
        STRU_SysEvent_DEV_BB_STATUS bb_searchStatus = {
            .pid = BB_SKY_SEARCHING_STATES_CHAGE,
            .e_sky_searchState = SKY_RC_SEARCH_END,
        };
        memcpy(bb_searchStatus.rcid, (void *)context.rcid, RC_ID_SIZE);
        memcpy(bb_searchStatus.vtid, (void *)context.vtid, VT_ID_SIZE);

        SYS_EVENT_NotifyInterCore(SYS_EVENT_ID_BB_EVENT, (void *)&bb_searchStatus);
        e_skyRcSearchState = SKY_RC_SEARCH_END;

        BB_saveRcid((uint8_t *)context.rcid, (uint8_t *)context.vtid);
        BB_sky_vtId((uint8_t *)context.hashVtid);
        DLOG_Warning("==> SearchEnd rc:%x %x %x %x %x vt:%x %x", context.rcid[0], context.rcid[1], context.rcid[2], context.rcid[3], context.rcid[4],
                                                                 context.vtid[0], context.vtid[1]);
    }

    pre_state = e_skyRcSearchState;
}


void BB_sky_handleRcSearchCmd(void)
{
    if (CMD_SEARCH_RC_START == stru_skystatus.u8_userRequestRcSearch)
    {
        stru_skystatus.u8_userRequestRcSearch = NO_SEARCH_RC_CMD;
        e_skyRcSearchState = SKY_WAIT_VT_ID_MSG;

        STRU_SysEvent_DEV_BB_STATUS bb_searchStatus = {
            .pid = BB_SKY_SEARCHING_STATES_CHAGE,
            .e_sky_searchState = SKY_WAIT_VT_ID_MSG,
        };

        SYS_EVENT_NotifyInterCore(SYS_EVENT_ID_BB_EVENT, (void *)&bb_searchStatus);

        // set vt id to vt register 0x0 to avoid BB_sky_checkSearchEnd pass
        uint8_t  vtid[2] = {0x0, 0x0};
        memcpy((void *)u8_BackupVtIds, (void *)context.vtid, VT_ID_SIZE);
        memcpy((void *)context.vtid, vtid, VT_ID_SIZE);
        memcpy((void *)context.rcid, (void *)context.chipid, RC_ID_SIZE);

        BB_sky_vtId(vtid);

        context.dev_state = SEARCH_ID;
		DLOG_Warning("rc_set_unlock_patten");
		rc_set_unlock_patten(1);
        sky_soft_reset();

        BB_SetTrxMode(BB_RECEIVE_ONLY_MODE);
        DLOG_Warning("==>SEARCH_ID %d %x %x", BB_sky_isSearching(), vtid[0], vtid[1]);

        context.inSearching = BB_sky_inSearching();
    }
    else if (CMD_SEARCH_RC_END == stru_skystatus.u8_userRequestRcSearch)
    {
        if (BB_sky_isSearching())
        {
            memcpy((void *)context.vtid, (void *)u8_BackupVtIds, VT_ID_SIZE);
            BB_GenHashId((uint8_t *)context.rcid, (uint8_t *)context.vtid,BB_SKY_MODE);
            BB_GenHashId((uint8_t *)context.rcid, (uint8_t *)context.vtid,BB_GRD_MODE);
            BB_sky_vtId((uint8_t *)context.hashVtid);
            sky_setRcId((uint8_t *)context.hashRcid);
        }

        stru_skystatus.u8_userRequestRcSearch = NO_SEARCH_RC_CMD;
        sky_soft_reset();
        context.dev_state = CHECK_LOCK;
        e_skyRcSearchState = SKY_RC_SEARCH_END;

        DLOG_Warning("==>CHECK_LOCK");
        context.inSearching = BB_sky_inSearching();
    }
}

uint8_t BB_sky_isSearching(void)
{
    return (SKY_RC_SEARCH_END != e_skyRcSearchState);
}

uint8_t BB_sky_isErrorConnectInSearching(uint8_t status)
{
    uint8_t err = 0;

    if (SKY_RC_SEARCH_END != e_skyRcSearchState) //sky in searching
    {
        //sky try to find one ground which in searching
        if (SKY_WAIT_VT_ID_MSG == e_skyRcSearchState)
        {
            err = ((1 == stru_skystatus.flag_groundInSearching) ? 0 : 1);
        }
        else //sky in SKY_WAIT_RC_ID_MATCH
        {
            //not error connect:
            //1. ground is in searching, and the cur_groundRcid == ground rc id
            //2. ground is not in searching, id match

            uint8_t ground_id[RC_ID_SIZE];
            ground_id[0] = BB_ReadReg(PAGE2, 0xd2);
            ground_id[1] = BB_ReadReg(PAGE2, 0xd3);
            ground_id[2] = BB_ReadReg(PAGE2, 0xd4);
            ground_id[3] = BB_ReadReg(PAGE2, 0xd5);
            ground_id[4] = BB_ReadReg(PAGE2, 0xd6);

            if (1 == stru_skystatus.flag_groundInSearching && 0 == memcmp(ground_id, stru_skystatus.cur_groundRcid, RC_ID_SIZE))
            {
                err = 0;
            }
            else if (0 == stru_skystatus.flag_groundInSearching && SKY_ID_CRC_MATCH(status))
            {
                err = 0;
            }
            else
            {
                err = 1;
            }
        }
    }
    else //sky not in searching, check crc match
    {
        //err = (SKY_ID_CRC_MATCH(status)?0:1);
    }

    return err;
}
