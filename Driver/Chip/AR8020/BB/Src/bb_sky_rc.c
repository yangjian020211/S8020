#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <stdint.h>
#include "ar8020.h"
#include "reg_rw.h"
#include "timer.h"
#include "interrupt.h"
#include "systicks.h"
#include "bb_sky_ctrl.h"
#include "bb_regs.h"
#include "debuglog.h"
#include "sys_event.h"
#include "bb_ctrl_internal.h"
#include "bb_customerctx.h"
#include "bb_ctrl.h"
#include "filter.h"
#include "rf_if.h"



static uint8_t     sky_rc_ch_map[MAX_RC_FRQ_SIZE];
static uint8_t     changech = 0;
static uint32_t    g_min = 0, g_max = 0;
static uint32_t    g_rst = 0;
static uint8_t     rc_ch_lockstatus[MAX_RC_FRQ_SIZE][RC_CH_BYTES_CNT];
static uint8_t     ch_lockpos = 0;

static uint8_t     rc_ch_usecnt[MAX_RC_FRQ_SIZE];
static int8_t      rc_ch_score[MAX_RC_FRQ_SIZE];
static uint8_t     rc_ch_times[MAX_RC_FRQ_SIZE];
volatile uint8_t   g_syn = 1;

static uint32_t    rc_checktimes = 0;
static uint8_t     rc_statusCh = 0;
extern STRU_SKY_STATUS stru_skystatus;
STRU_SKY_LOCK_CNT st_skyLockStatusInLock = {0};
static int sky_get_lock_worst_ch(uint8_t start, uint8_t end);
static uint8_t sub_band_2g=0;
static uint8_t sub_band_5g=0;
uint8_t sub_band_2g_agc_gain=1;/////////1 near , 0 far
uint8_t sub_band_5g_agc_gain=1;

void BB_RcSelectionInit(void)
{
    uint8_t i = 0;
    //BB_ResetRcMap();
    for (i = 0; i < MAX_RC_FRQ_SIZE; ++i)
    {
       context.rc_ch_map[i]    = i;
       rc_ch_usecnt[i] = 1;
       rc_ch_times[i]  = 0;
    }

   memcpy(sky_rc_ch_map, (void *)context.rc_ch_map, MAX_RC_FRQ_SIZE);
}

int rc_score600m(uint8_t * map, uint8_t * use, uint8_t start, uint8_t end)
{
    uint8_t ch;
    int tmpCh = 0;

    changech = 0;
    for (ch = start; ch < end; ++ch)
    {
        uint8_t i;

        rc_ch_score[ch] = 0;

        for (i = 0; i < RC_CH_BYTES_CNT; ++i)
        {
            rc_ch_score[ch] += ((rc_ch_lockstatus[ch][i]) ? 1 : -1);
        }

        if (map[ch] != ch)
        {
            changech ++;
        }
    }

    //some error happen, reset the map
    if (changech > context.st_chSelectionParam.u8_rcChReplaceMaxCnt)
    {
        DLOG_Error("Error: ch %d %d", changech, context.st_chSelectionParam.u8_rcChReplaceMaxCnt);
        tmpCh = sky_get_lock_worst_ch(start, end);
        if(-1 != tmpCh)
        {
            g_min = tmpCh;
            g_rst = 1;
            return 0;
        }
    }

    int8_t max_score = 0;
    int8_t min_score = 0;
    for (ch = start; ch < end; ch++)
    {
        if (rc_ch_times[ch] >= RC_CH_BYTES_CNT && map[ch] == ch && use[ch] < MAXUSECOUNT)
        {
            g_max = ch;
            g_min = ch;
            max_score = rc_ch_score[ch];
            min_score = rc_ch_score[ch];
            break;
        }
    }

    //find max, that has not been replaced, and not replaced others
    for (ch = g_max+1; ch < end; ch++)
    {
        if (rc_ch_times[ch] >= RC_CH_BYTES_CNT && map[ch] == ch && use[ch] < MAXUSECOUNT && rc_ch_score[ch] > max_score)
        {
            max_score = rc_ch_score[ch];
            g_max = ch;
        }
    }

    for (ch = start; ch < end; ch++)
    {
        if (rc_ch_times[ch] >=RC_CH_BYTES_CNT && rc_ch_score[ch] < min_score)
        {
            min_score = rc_ch_score[ch];
            g_min = ch;
        }
    }

    DLOG_Info("replace:%d-%d", g_max, g_min);
#if 0
    if (rc_checktimes >= context.st_chSelectionParam.u16_rcChReplaceCycles)
    {
        for (ch = start; ch < end; ++ch)
        {
            DLOG_Info("ch:%d-%d %d %d %d", ch, context.rc_ch_map[ch], rc_ch_score[ch], rc_ch_usecnt[ch], rc_ch_times[ch]);
        }
    }
#endif
    //change channel is full, and the worst one not be replaced, try to restore one replaced channel
    if (context.st_chSelectionParam.u8_rcChReplaceMaxCnt == changech && map[g_min] == g_min)
    {
        uint8_t  sweep_min_ch = 0;
        int power_min = 0;

        //cal average sweep power
        //sky_CalAverSweepResult(start, end);

        //select the sweep best channel been replaced
        for (ch = start; ch < end; ++ch)
        {
            if (map[ch] != ch)  //channel been replaced
            {
                int power = sky_GetAverSweepResult(ch);
                if (power < power_min)
                {
                    power_min = power;
                    sweep_min_ch = ch;
                }
            }
        }
        
        int power1 = sky_GetAverSweepResult(g_min);
        int power2 = sky_GetAverSweepResult(sweep_min_ch);

        //compare the noise power, the channel_noise(score_min) > channel_noise(sweep_min_ch) + 5
        if (power1 - power2 > 5)
        {
            DLOG_Info("RST: %d %d %d", g_min, sweep_min_ch, power1 - power2);

            //reset the chnanel
            g_min = sweep_min_ch;
            g_rst = 1;
        }
        else
        {
            //DLOG_Info("diff: %d %d %d %d %d", g_min, sweep_min_ch, power1, power2, power1-power2);
        }
    }

    return 0;
}


static int rc_replace600m()
{
    if (g_rst == 0 && (rc_ch_score[g_max] - rc_ch_score[g_min]) >= 12)
    {
        memcpy(sky_rc_ch_map, (void *)context.rc_ch_map, sizeof(sky_rc_ch_map));

        //if the channel has not been replaced, replace the score min channel
        if (context.rc_ch_map[g_min] == g_min && (changech < context.st_chSelectionParam.u8_rcChReplaceMaxCnt))
        {
            --rc_ch_usecnt[sky_rc_ch_map[g_min]];
            sky_rc_ch_map[g_min] = g_max;
            ++rc_ch_usecnt[sky_rc_ch_map[g_min]];
            rc_ch_times[g_min] = 0;
            ++g_syn;

        }
        else if (context.rc_ch_map[g_min] == g_min && (changech >= context.st_chSelectionParam.u8_rcChReplaceMaxCnt))
        {
            
        }
        else if (context.rc_ch_map[g_min] != g_min) //if the channel been replacecd, restore the channel first
        {
            sky_reset_ch(g_min);
        }

        return 0;
    }
    else if(g_rst > 0)
    {
        sky_reset_ch(g_min);

        return 0;
    }
    else
    {
        return -1;
    }
}

/*
void sky_rcForBandSwitch(uint8_t lock)
{
    st_skyLockStatusInLock.u16_check_cnt ++;
    st_skyLockStatusInLock.u16_lock_cnt  += lock;

    if (st_skyLockStatusInLock.u16_check_cnt >= context.u_bandSwitchParam->sky_bandSwitchParm.u16_skyRcTotalCnt)
    {
        st_skyLockStatusInLock.flag_bandSwitch = (((uint32_t)st_skyLockStatusInLock.u16_lock_cnt * 100) < 
                                                  ((uint32_t)st_skyLockStatusInLock.u16_check_cnt * context.u_bandSwitchParam->sky_bandSwitchParm.u8_lockPercent));
       // DLOG_Info("!!!rc flag_bandSwitch = %d %d %d", st_skyLockStatusInLock.u16_lock_cnt, st_skyLockStatusInLock.u16_check_cnt, 
                                                         context.u_bandSwitchParam->sky_bandSwitchParm.u8_lockPercent);
                                                         st_skyLockStatusInLock.u16_check_cnt = 0;
       // st_skyLockStatusInLock.u16_lock_cnt  = 0;
    }
}
*/

uint8_t sky_CheckRcJudgeBandSwitch(void)
{
    return st_skyLockStatusInLock.flag_bandSwitch;
}

void sky_rcFrqStatusStatistics600m(void)
{
    uint8_t start = context.rc_start;
    uint8_t end = context.rc_end;
    uint8_t frq_num = BB_GetRcFrqNumPerFilter();
    uint8_t u8_rcCh     = context.sky_rc_channel;
    uint8_t max_ch_size = BB_GetRcFrqNum(context.e_curBand);
    uint8_t lock        = sky_checkRcLock(stru_skystatus.u8_rcStatus);
    uint8_t p[40];
    uint8_t i;

    rc_ch_lockstatus[u8_rcCh][ch_lockpos] = lock;

    if (rc_ch_times[u8_rcCh] < RC_CH_BYTES_CNT)
    {
        rc_ch_times[u8_rcCh] ++;
    }

    if (u8_rcCh == (end - 1))
    {
        ch_lockpos ++;
        if (ch_lockpos >= RC_CH_BYTES_CNT)
        {
            ch_lockpos = 0;
        }
    }

    if (u8_rcCh == (end - 1) && g_syn == 0)
    {
        if (rc_checktimes >= context.st_chSelectionParam.u16_rcChReplaceCycles)
        {
            if (0 == rc_score600m((uint8_t *)context.rc_ch_map, rc_ch_usecnt, start, end))
            {
                if (rc_replace600m() == 0 && changech != context.st_chSelectionParam.u8_rcChReplaceMaxCnt)
                {
                    rc_checktimes = context.st_chSelectionParam.u16_rcChReplaceCycles - 1;//context.st_chSelectionParam.u16_rcChReplaceCycles / 3;
                }
                else
                {
                    rc_checktimes = 0;
                }
                g_rst = 0;
#if 0
                DLOG_Info("syn:%d max:%d min:%d changech:%d max_frq:%d", g_syn, g_max, g_min, changech, frq_num);
                DLOG_Info("%d:%d:%d:%d", context.st_chSelectionParam.u16_rcChReplaceCycles, context.st_chSelectionParam.u8_rcChReplaceMaxCnt, context.st_chSelectionParam.u8_rcChSelectionEnable);      
                
                //if(flag)
                {
                    for(i=0;i<max_ch_size;i++)
                    {
                        if(i == sky_rc_ch_map[i])
                        {
                            p[i] = 1;
                        }
                        else
                        {
                            p[i] = 0;
                        }
                    }
                
                    DLOG_Info("0~7:   %d %d %d %d %d %d %d %d",p[0],p[1],p[2],p[3],p[4],p[5],p[6],p[7]);
                    DLOG_Info("8~15:  %d %d %d %d %d %d %d %d",p[8],p[9],p[10],p[11],p[12],p[13],p[14],p[15]);
                    DLOG_Info("16~23: %d %d %d %d %d %d %d %d",p[16],p[17],p[18],p[19],p[20],p[21],p[22],p[23]);
                    DLOG_Info("24~31: %d %d %d %d %d %d %d %d",p[24],p[25],p[26],p[27],p[28],p[29],p[30],p[31]);
                    DLOG_Info("32~39: %d %d %d %d %d %d %d %d",p[32],p[33],p[34],p[35],p[36],p[37],p[38],p[39]);
                }
#endif
            }
        }
    }

    if (g_syn > 0)
    {
        BB_Session0SendMsg(DT_NUM_CH_MAP, sky_rc_ch_map, sizeof(sky_rc_ch_map));
    }

    ++rc_checktimes;
}


void sky_rcHopFreq(void)
{
    uint8_t flag;
    
    if(context.freq_band_mode == SUB_BAND)
    {
        flag = 0;
        if(BB_sky_isSearching() && context.dev_state == CHECK_LOCK)
        {
            flag = 1;
        }
        
        if(context.dev_state == LOCK || context.dev_state == WAIT_VT_LOCK || flag)
        {
            context.sky_rc_channel++;
            if(context.sky_rc_channel >= context.sub_rc_end)
            {
                context.sky_rc_channel = context.sub_rc_start;
            }
        }
        else
        {
            if(context.e_curBand == RF_2G)
            {
                DLOG_Info("2g it=%d",sub_band_2g);
                if(sub_band_2g == 0)
                {
                    if(sub_band_2g_agc_gain)
                    {
                        sub_band_2g_agc_gain = 0;
                    }
                    else
                    {
                        sub_band_2g_agc_gain = 1;
                    }
                }
                context.sub_band_value = sub_band_2g;
                context.sub_rc_start = BB_GetSubBandStartCH(context.e_curBand,context.st_bandMcsOpt.e_bandwidth,sub_band_2g);
                context.sub_rc_end = BB_GetSubBandEndCH(context.e_curBand,context.st_bandMcsOpt.e_bandwidth,sub_band_2g);
                sub_band_2g++;
                if(sub_band_2g >= BB_GetItFrqNum(context.e_curBand))
                {
                    sub_band_2g = 0;
                }
                
            }
            else if(context.e_curBand == RF_5G)
            {
                DLOG_Info("5g it=%d",sub_band_5g);
                if(sub_band_5g == 0)
                {
                    if(sub_band_5g_agc_gain)
                    {
                        sub_band_5g_agc_gain = 0;
                    }
                    else
                    {
                        sub_band_5g_agc_gain = 1;
                    }
                }

                context.sub_band_value = sub_band_5g;
                context.sub_rc_start = BB_GetSubBandStartCH(context.e_curBand,context.st_bandMcsOpt.e_bandwidth,sub_band_5g);
                context.sub_rc_end = BB_GetSubBandEndCH(context.e_curBand,context.st_bandMcsOpt.e_bandwidth,sub_band_5g);
                sub_band_5g++;
                if(sub_band_5g >= BB_GetItFrqNum(context.e_curBand))
                {
                    sub_band_5g = 0;
                }

            }
            else
            {
                DLOG_Error("band error");
            }
            
            context.sky_rc_channel = context.sub_rc_start + (SysTicks_GetTickCount() % (context.sub_rc_end - context.sub_rc_start));
        }
    }
    else
    {
        bb_get_rc_channel();
    }
	uint8_t skip_rc_ch = context.sky_rc_channel;
	BB_set_Rcfrq(context.e_curBand, skip_rc_ch);
}

void sky_rcHopFreq600m(uint8_t div)
{
    static uint32_t print = 0;
    uint8_t flag = !(print++ % 1001);
    uint8_t setCh = 0;
    uint8_t max_ch_size = BB_GetRcFrqNum(context.e_curBand);
    uint8_t p[40];
    uint8_t i;

    if(1 == div)
    {
        context.sky_rc_channel += div;
        if((context.sky_rc_channel >= context.rc_end) || (context.sky_rc_channel < context.rc_start))
        {
            context.sky_rc_channel = context.rc_start;
        }
    }
    else
    {
        context.sky_rc_channel += div;
        context.sky_rc_channel %= max_ch_size;
    }

    if (context.st_chSelectionParam.u8_rcChSelectionEnable)
    {
        setCh = context.rc_ch_map[context.sky_rc_channel];
    }
    else
    {
        setCh = context.sky_rc_channel;
    }
    
    BB_set_Rcfrq(context.e_curBand, setCh);
    //sky_SetSweepCh(context.e_curBand, setCh);
    
    if(flag)
    {
        //DLOG_Warning("rc sweep:%d", setCh);
    }
    BB_FilterSet(BB_GetFilterByRcFrqCh(setCh),flag);

  
}

uint8_t sky_check_all_mask(uint32_t ch1, uint32_t ch2)
{
    uint32_t i = 0;

    for(i=ch1; i<ch2; i++)
    {
        if(i == context.rc_ch_map[i])
        {
            return 0;
        }
    }

    return 1;
}

uint32_t sky_get_best_ch(uint32_t ch1, uint32_t ch2)
{
    uint32_t ch = ch1;
    int8_t max_score = rc_ch_score[ch1];
    uint32_t i = 0;
    
    for(i=(ch1+1); i<ch2; i++)
    {
        if (rc_ch_score[i] > max_score)
        {
            max_score = rc_ch_score[i];
            ch = i;
        }
    }

    return ch;
}

static int sky_get_lock_worst_ch(uint8_t start, uint8_t end)
{
    int8_t ch = 0;
    int8_t i = 0;
    int8_t min_score = rc_ch_score[start];
    //uint8_t max_ch_size = BB_GetRcFrqNum(context.e_curBand);

    if(0 == changech)
    {
        return -1;
    }
    
    for(i=(start+1); i<end; i++)
    {
        if ((rc_ch_score[i] < min_score) && (ch != context.rc_ch_map[ch]))
        {
            min_score = rc_ch_score[i];
            ch = i;
        }
    }

    return (int)ch;
}


int sky_reset_ch(uint32_t ch)
{
    if(ch != context.rc_ch_map[ch])
    {
        memcpy(sky_rc_ch_map, (void *)context.rc_ch_map, sizeof(sky_rc_ch_map));
        
        --rc_ch_usecnt[sky_rc_ch_map[ch]];
        rc_ch_times[sky_rc_ch_map[ch]] = 0;
        
        sky_rc_ch_map[ch] = ch;
        
        ++rc_ch_usecnt[ch];
        rc_ch_times[ch] = 0;
        
        ++g_syn;
    }
    
    return 0;
}

int sky_send_rc_map(void)
{
    return BB_Session0SendMsg(DT_NUM_CH_MAP, sky_rc_ch_map, sizeof(sky_rc_ch_map));
}


