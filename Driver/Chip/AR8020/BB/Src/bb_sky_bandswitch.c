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
#include "bb_uart_com.h"
#include "bb_ctrl_internal.h"
#include "bb_customerctx.h"
#include "rf_if.h"
#include "bb_ctrl.h"

extern STRU_SKY_STATUS stru_skystatus;

void sky_bandSwitchLnaSwitch(void)
{
    if(context.enable_non_lbt)
    {
        if(context.lna_status != OPEN_LNA)
        {
            BB_open_lna();
            context.lna_status = OPEN_LNA;
        }

        return;
    }
    if(context.e_curBand == RF_5G)
    {
        if(context.factory_lna_mode == LNA_BYPASS)
        {
            if(context.lna_status != BYPASS_LNA)
            {
                BB_bypass_lna();
                context.lna_status = BYPASS_LNA;
            }
        }
        else
        {
            if(context.lna_status != OPEN_LNA)
            {
                BB_open_lna();
                context.lna_status = OPEN_LNA;
            }
        }
    }
    else
    {
        if(context.st_mimo_mode.enum_lna_mode == LNA_BYPASS)
        {
            if(context.lna_status != BYPASS_LNA)
            {
                BB_bypass_lna();
                context.lna_status = BYPASS_LNA;
            }
        }
        else if(context.st_mimo_mode.enum_lna_mode == LNA_OPEN)
        {
            if(context.lna_status != OPEN_LNA)
            {
                BB_open_lna();
                context.lna_status = OPEN_LNA;
            }
        }
        else if(context.st_mimo_mode.enum_lna_mode == LNA_AUTO)
        {
        	
	            if(context.swp_bypass == 1)
	            {
	                if(context.lna_status != BYPASS_LNA)
	                {
	                    BB_bypass_lna();
	                    context.lna_status = BYPASS_LNA;
	                }
	            }
	            else if(context.swp_bypass == 2)
	            {
	                if(context.lna_status != OPEN_LNA)
	                {
	                    BB_open_lna();
	                    context.lna_status = OPEN_LNA;
	                }
	        
	            }
            		
        }

    }

    BB_Lna_reset();

}
/*
 * RC hop if unlock, cycle count > (rc_2g + rc_5g count)
 */
uint8_t __attribute__ ((section(".h264"))) sky_rcUnLockHopBand(void)
{
    static uint32_t u32_count = 0;
    //static uint32_t u32_AgcToggleTick = 0;
    static uint8_t frq_2g = 0;
    static uint8_t frq_5g = 0;
	static uint8_t frq_pre = 0;
    static uint8_t cnt = 0;
    uint32_t feq_num;
	
	#ifdef RFSUB_BAND
    if(context.freq_band_mode == SUB_BAND)
    {
        feq_num = BB_GetSubBandEndCH(context.e_curBand,context.st_bandMcsOpt.e_bandwidth,0) -
            BB_GetSubBandStartCH(context.e_curBand,context.st_bandMcsOpt.e_bandwidth,0);
    }
    else
	#endif
	
    {
        feq_num = BB_GetRcFrqNum(context.e_curBand);
    }
    uint32_t timer = (feq_num + 1) * 14;

    if (context.st_bandMcsOpt.e_bandsupport == RF_2G_5G)
    {
        uint32_t u32_count1 = SysTicks_GetTickCount();
        if (SysTicks_GetDiff(u32_count, u32_count1) >= timer)
        {
            u32_count = u32_count1;
			frq_pre = context.sky_rc_channel;
            context.e_curBand = OTHER_BAND(context.e_curBand);
            BB_set_RF_Band(BB_SKY_MODE, context.e_curBand);
			reset_sweep_table(context.e_curBand);
			context.sky_rc_channel = frq_pre;
            if(context.freq_band_mode == FULL_BAND)
            {
                if(++cnt >= 2)
                {
                    cnt = 0;
                    sky_agcGainToggle();
                }
            }
		
			//context.rf_info.e_bw = context.st_bandMcsOpt.e_bandwidth;
			RF_GetFctFreqTable(context.st_bandMcsOpt.e_bandwidth);
			rc_set_unlock_patten(1);
            sky_rcHopFreq();
			#ifdef RFSUB_BAND
            if(context.freq_band_mode == SUB_BAND)
            {
                sky_agcGainToggle();
            }
			#endif
            sky_switchSetPower(context.e_curBand);
            sky_soft_reset();

            BB_set_ItFrqByCh(context.e_curBand, context.cur_IT_ch);
            DLOG_Warning("to:%d %d %d %d", context.e_curBand, context.rf_info.rc_ch_working_patten_size,context.sky_rc_channel, SysTicks_GetTickCount());

            sky_bandSwitchLnaSwitch();
            
            return 1;
        }
    }

    return 0;
}

void __attribute__ ((section(".h264"))) sky_handle_RF_band_cmd(uint8_t *arg)
{
    if (0 == context.stru_bandChange.flag_bandchange)   //if sky is not in band changing
    {
        STRU_BandChange *p = (STRU_BandChange *)arg;

        //check the request band
        if (context.e_curBand != (ENUM_RF_BAND)(p->e_toBand))
        {
            //ground ask for change, decide by sky
            if (p->flag_bandchange == 0)
            {
                //if (st_skySweep.e_bandOpt == SKY_EQUAL_2G_5G)   //sky accept the request when no difference, so reply ok
                                                                  //remove if conditon, because no SKY_MUST_NOT_SWITCH branch
                {
                    p->flag_bandchange = 1;
                    BB_Session0SendMsg(DT_NUM_RF_BAND_CHANGE, (uint8_t *)p, sizeof(STRU_BandChange)); //no problem even if the message drop, ground will send again.
                }
            }
            else    //ground ask for change right now, in two cases: 1) sky accept  2)ground request manually change, so do change
            {
                context.stru_bandChange.flag_bandchange = 1;            //accept the request
                context.stru_bandChange.u8_eqCntChg     = p->u8_eqCntChg;
                context.stru_bandChange.u8_ItCh         = p->u8_ItCh;
				context.stru_bandChange.chg_mcs			= p->chg_mcs;
				context.stru_bandChange.e_toBand		= p->e_toBand;
            }

            DLOG_Critical("Band:%d %d :%d", context.e_curBand, (p->e_toBand), context.stru_bandChange.u8_eqCntChg);
        }
    }
}


void __attribute__ ((section(".h264"))) sky_switchSetPower(ENUM_RF_BAND band)
{
    context.pwr = BB_get_band_power(context.e_curBand);

    DLOG_Critical("band: %d %d %d %d,pwr %d",  context.sky_rc_channel, context.stru_bandChange.u8_eqCntChg,
                                        context.stru_bandChange.u8_ItCh, context.e_curBand,context.pwr);
    if (context.e_powerMode == RF_POWER_CLOSE)
    {
        BB_SetPowerCloseMode(context.e_curBand);
    }
    BB_set_power(context.e_curBand,context.pwr);
    context.u8_aocAdjustPwr = context.pwr;
}


/*
 * call every 14ms cycle in ID_MATCH_LOCK status
*/
void __attribute__ ((section(".h264"))) sky_doRfBandChange(uint8_t u8_lockStatus)
{
    if (1 == context.stru_bandChange.flag_bandchange)
    {
        if (context.sync_cnt == (context.stru_bandChange.u8_eqCntChg) )
        {
            context.stru_bandChange.flag_bandchange = 0;
            context.stru_bandChange.u8_eqCntChg = 0;
            context.e_curBand =  context.stru_bandChange.e_toBand;
			RF_GetFctFreqTable(context.st_bandMcsOpt.e_bandwidth);
			reset_sweep_table(context.e_curBand);
			BB_set_RF_Band(BB_SKY_MODE, context.e_curBand);
            sky_switchSetPower(context.e_curBand);
			#ifdef RFSUB_BAND
            if(context.freq_band_mode == SUB_BAND)// && SKY_ID_CRC_MATCH(u8_lockStatus))
            {
                context.sub_band_value = context.stru_bandChange.u8_ItCh;
                sky_sub_band_excute(context.stru_bandChange.u8_ItCh);
            }
            else
			#endif	
            {
                context.sky_rc_channel = 0;
            }
			rc_set_unlock_patten(0);
            sky_rcHopFreq();
            BB_set_ItFrqByCh(context.e_curBand, context.stru_bandChange.u8_ItCh);
			if(context.qam_ldpc!=context.stru_bandChange.chg_mcs)
			{
	    		ENUM_RUN_MODE mode = AUTO;
				context.qam_ldpc=context.stru_bandChange.chg_mcs;
				sky_handle_mcs_mode_cmd((uint8_t*)&mode);
			}
            sky_bandSwitchLnaSwitch();
            //BB_SetAgcGain(context.e_curBand, (uint8_t)(stru_skystatus.en_agcmode));
        }
    }
}


void __attribute__ ((section(".h264"))) sky_handle_RF_band_cmd_pure_vt( ENUM_RF_BAND rf_band)
{
    if(context.e_curBand != rf_band)
    {
        context.stru_bandChange.flag_bandchange = 0;
        context.stru_bandChange.u8_eqCntChg = 0;
        context.e_curBand = OTHER_BAND( context.e_curBand );

        BB_set_RF_Band(BB_SKY_MODE, context.e_curBand);
		reset_sweep_table(context.e_curBand);
        sky_switchSetPower(context.e_curBand);

        context.sky_rc_channel = 0;
        context.stru_bandChange.u8_ItCh = 0;
        context.cur_IT_ch  = 0;
		RF_GetFctFreqTable(context.st_bandMcsOpt.e_bandwidth);
		rc_set_unlock_patten(0);
        sky_rcHopFreq();
        BB_set_ItFrqByCh( context.e_curBand, context.stru_bandChange.u8_ItCh);
    }
}
