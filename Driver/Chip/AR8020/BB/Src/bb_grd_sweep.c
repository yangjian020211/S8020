#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <math.h>
#include <stdlib.h>
#include "systicks.h"
#include "debuglog.h"
#include "bb_ctrl_internal.h"
#include "bb_grd_sweep.h"
#include "bb_regs.h"
#include "log10.h"
#include "bb_ctrl.h"
#include "cfg_parser.h"
#include "rf_if.h"
#include "filter.h"
#include "bb_snr_service.h"




#define BW_10M_START_TH                 (12)
#define BW_10M_END_TH                   (19)
#define BW_10M_MAX_CH_CNT               (BW_10M_END_TH - BW_10M_START_TH + 1)
#define BW_10M_VALID_CH_CNT             (BW_10M_MAX_CH_CNT - 2)
#define BW_10M_VALID_1_5M_CH_START      (BW_10M_START_TH + 1)


#define BW_20M_START_TH                 (8)
#define BW_20M_END_TH                   (23)
#define BW_20M_MAX_CH_CNT               (BW_20M_END_TH - BW_20M_START_TH + 1)
#define BW_20M_VALID_CH_CNT             (BW_20M_MAX_CH_CNT - 4)
#define BW_20M_VALID_1_5M_CH_START      (BW_20M_START_TH + 2)


#define NEXT_NUM(cur, max)              ( ((cur + 1) >= max) ? 0: (cur + 1) )

#define AGC_LEVEL_0                     (0x5f)  //-90dmb
#define AGC_LEVEL_1                     (0x64)  //-96dmb

#define MAX(a,b) (((a) > (b)) ?  (a) :  (b) )
#define MIN(a,b) (((a) < (b)) ?  (a) :  (b) )

#define CLEAR_MAIN_CNT  (1000)

#define IT_CHANGE_THD 5

static int flaglog = 0;

extern STRU_GRD_STATUS stru_grdstatus;
//extern uint8_t grd_rc_channel;
extern void BB_grd_NotifyItFreqByCh(ENUM_RF_BAND band, uint8_t u8_ch);

uint8_t sweep_with_filter = 1;

static uint8_t BB_GetSweepTotalCh(ENUM_RF_BAND e_rfBand, ENUM_CH_BW e_bw);

static uint8_t BB_UpdateOptCh(ENUM_RF_BAND e_rfBand, ENUM_CH_BW e_bw, uint8_t sweep_ch);

static uint8_t BB_JudgeAdjacentFrequency(uint8_t judge_ch);

static void BB_GetItMinMaxCh(ENUM_RF_BAND e_rfBand, ENUM_CH_BW e_bw, uint8_t *min_ch, uint8_t *max_ch);

static void BB_GetItAdjacentFrequency(uint8_t ch, uint8_t *pre, uint8_t *next);

//ENUM_RF_select BB_grd_cmpBandNoise( void );

/*
 * to start sweep
 */
void __attribute__ ((section(".h264"))) BB_SweepStart(ENUM_RF_BAND e_bandsupport, ENUM_CH_BW e_bw)
{
    //memset((uint8_t *)(&stru_sweepPower), 0x00, sizeof(STRU_SWEEP_NOISE_POWER));
    
    context.rf_info.u8_preMainCh  =  0xff;
    context.rf_info.u8_mainCh     =  0xff;
    context.rf_info.e_bw          =  e_bw;
    context.rf_info.u8_cmdSweep = 0;

    context.rf_info.e_bandsupport = context.st_bandMcsOpt.e_bandsupport;
    
    #ifdef RF_9363
    //if(RF_600M == context.e_curBand)
    {
        context.rf_info.u8_totalCyc   = 2;
        if(0 == sweep_with_filter)
        {
            BB_FilterSet(FILTER_NONE, 1); // set filter none.
        }
        else
        {
            BB_FilterSet(BB_GetFilterByVtFrqCh(context.rf_info.u8_prevSweepCh), 0);
        }
    }
    #endif
    
    #ifdef RF_8003X
    //else
    {
        //context.rf_info.u8_totalCyc   = (context.rf_info.e_bandsupport == RF_2G_5G) ? 0x04 : 0x03;
		context.rf_info.u8_totalCyc = 0x03;
    }
    #endif

    context.e_bandSelFromSweep    = UNKNOWN_BETTER;

#ifdef RF_9363
    //if (RF_600M == e_bandsupport)
    {
        context.rf_info.u8_bb1ItFrqSize = BB_GetItFrqNum(RF_600M);
        context.e_curBand   =  RF_600M;
    }
#endif

#ifdef RF_8003X
    if (RF_2G_5G == e_bandsupport)
    {
        context.rf_info.u8_bb1ItFrqSize = BB_GetItFrqNum(RF_2G);
        context.rf_info.u8_bb2ItFrqSize = BB_GetItFrqNum(RF_5G);
        context.e_curBand = RF_2G;
    }
    else if (RF_5G == e_bandsupport)
    {
        context.rf_info.u8_bb1ItFrqSize = BB_GetItFrqNum(RF_5G);
        context.e_curBand = RF_5G;
    }
    else if (RF_2G == e_bandsupport)
    {
        context.rf_info.u8_bb1ItFrqSize = BB_GetItFrqNum(RF_2G);
        context.e_curBand = RF_2G;
    }
#endif
    context.rf_info.flag_signalBlock = RF_SIGNAL_UNKNOWN_MODE;
    BB_WriteRegMask(PAGE2, GRD_SKY_IT_CH_SYNC, (context.rf_info.flag_signalBlock << 6), GRD_SKY_BLOCK_MODE_MASK);

    context.rf_info.e_prevSweepBand  = context.e_curBand;
    BB_set_SweepFrq(context.e_curBand, e_bw, 0);
	reset_sweep_table(context.e_curBand);
    context.u_bandSwitchParam = (UNION_BandSwitchParm *)CFGBIN_GetNodeAndData((STRU_cfgBin *)SRAM_CONFIGURE_MEMORY_ST_ADDR, RF_GRD_BAND_SWITCH_CFG_ID, NULL);
}


void __attribute__ ((section(".h264"))) BB_GetSweepNoise(int16_t *ptr_noise_power, uint32_t max)
{
    uint8_t col;
    uint8_t i;
    int16_t value;

    for(col = 0; col < context.rf_info.u8_bb1ItFrqSize; col++)
    {
        if(col >= max)
        {
            return;
        }

        if (RF_5G == context.rf_info.e_bandsupport)
        {
            value = (int16_t)(context.rf_info.sweep_pwr_avrg_value[col].value);
        }
        else
        {
            value = (int16_t)(context.rf_info.sweep_pwr_avrg_value[col].value);
        }

        for(i = 0; i < 8; i++)
        {
            ptr_noise_power[(col * 8) + i] = value;
        }
    }

    for(col = 0; col < context.rf_info.u8_bb2ItFrqSize; col++)
    {
        if((col + context.rf_info.u8_bb1ItFrqSize) >= max)
        {
            return;
        }
        value = (int16_t)(context.rf_info.sweep_pwr_avrg_value[col].value);
        for(i = 0; i < 8; i++)
        {
            ptr_noise_power[(col + context.rf_info.u8_bb1ItFrqSize) * 8 + i] = value;
        }
    }
}

int grd_lna_check_sweep_power(uint8_t ch,int32_t spower_data)
{

    static uint32_t   swp_lna_switch_cnt = 0;
    static uint32_t   swp_lna_switch_all_cnt = 0;
    static uint8_t    swp_recyles_cnt = 0,swp_watch_window_size = 0;

    if(context.rf_info.e_prevSweepBand == RF_2G && ch != context.cur_IT_ch )
    {
        swp_watch_window_size++;
        if(spower_data > GRD_SWEEP_LNA_SWITCH_THRESHOLD)
        {
            swp_lna_switch_cnt += 1;
            DLOG_Info("%d-%d,%d",ch,context.cur_IT_ch,spower_data);
        }
        
     //   if(swp_lna_switch_cnt > SWEEP_LNA_SWITCH_THRESHOLD_nper100)
     //   {
     //       context.swp_bypass = 1; /////////3/100
     //   }

        if(swp_watch_window_size > 100)
        {
            swp_watch_window_size = 0;
            swp_recyles_cnt++;
            swp_lna_switch_all_cnt += swp_lna_switch_cnt;
            DLOG_Info("lna=%d,swp=%d,%d-%d %d",context.lna_status,context.swp_bypass,swp_lna_switch_cnt,swp_lna_switch_all_cnt,swp_recyles_cnt);
            swp_lna_switch_cnt = 0;
        }

        if(swp_recyles_cnt > 9)
        {
            swp_recyles_cnt = 0;
            
            if(swp_lna_switch_all_cnt >= SWEEP_LNA_SWITCH_THRESHOLD_nper1000)
            {
                context.swp_bypass = 1;//25/1000,error, 
            }
            else if(swp_lna_switch_all_cnt == 0)
            {
                context.swp_bypass = 2;
            }

            if(swp_lna_switch_all_cnt > 0)
            {
                if(context.swp_bypass == 2)
                {
                    context.swp_bypass = 0;
                }
            }

            swp_lna_switch_all_cnt= 0;
        }

    }

    //DLOG_Warning("sw-b %d wk-b %d sw-c %d p %d",context.rf_info.e_prevSweepBand,context.e_curBand,ch,spower_data);
}


/*
 * adapt to AR8020 new sweep function
 * return 0: sweep fail. Do not switch to next sweep channel
 * return 1: sweep OK.
*/
static uint8_t __attribute__ ((section(".h264"))) BB_GetSweepPower(ENUM_RF_BAND e_rfBand, uint8_t bw, uint8_t row, uint8_t sweep_ch, uint8_t flag)
{
    uint8_t  num = 0;
    uint8_t  ret = 0;
    uint8_t  offset = sweep_ch;
    uint8_t  u8_maxCh = BB_GetSweepTotalCh(e_rfBand, bw);
    if(sweep_ch >= u8_maxCh)
    {
       DLOG_Error("Ch overflow:%d %d %d %d %d", e_rfBand, sweep_ch, u8_maxCh,bw,context.rf_info.sweep_freqsize);
	  //context.rf_info.u8_prevSweepCh=0;
       return 0;
		
    }

	int i=0,j=0; 
	
	static int tx_sweep=0;
    int32_t power = BB_SweepEnergy();
	signed char data = int2char(power);
    if (data == 0){
        return 0;
    }
    grd_lna_check_sweep_power(sweep_ch,data);
	GetSweepCh_normalsweep(context.rf_info.curBandIdx,sweep_ch,data,BB_GRD_MODE);
	CalcAverageSweepPower(sweep_ch);
	return 1;
}


int32_t __attribute__ ((section(".h264"))) BB_Sweep_updateCh(ENUM_RF_BAND e_rfBand, uint8_t mainch)
{
    uint8_t opt;
	int i=0;
    context.rf_info.u8_preMainCh     = context.rf_info.u8_mainCh;
    context.rf_info.u16_preMainCount = 0; //clear after 1000 cycles

    context.rf_info.u8_mainSweepCh = mainch;
    context.rf_info.u8_mainCh      = mainch;

#ifdef RF_9363
    //if(RF_600M == context.e_curBand)
    {
        opt = BB_GetItChInOneFilter(mainch);
    }
#endif

#ifdef RF_8003X
    //else
    {
        //re-select the option channel
        BB_selectBestCh(e_rfBand, SELECT_OPT, NULL, &opt, NULL, 0);
    }
#endif

    context.rf_info.u8_optCh = opt;
    context.rf_info.u8_optSweepCh = opt;
	for(i=0;i<MAX_IT_FRQ_SIZE;i++)context.rf_info.u8_bestBb1ChCnt[i]=0;
	for(i=0;i<MAX_IT_FRQ_SIZE;i++)context.rf_info.u8_bestBb2ChCnt[i]=0;
    
    return 0;
}

void __attribute__ ((section(".h264"))) BB_CmdSweepStart(ENUM_RF_BAND e_bandsupport, ENUM_CH_BW e_bw)
{
    context.rf_info.u8_curBb1Row = 0;
    context.rf_info.u8_prevSweepCh = 0;
    BB_set_SweepFrq(context.e_curBand, e_bw, 0);
    context.rf_info.u8_cmdSweep = 1;
    if(0 == sweep_with_filter)
    {
        BB_FilterSet(FILTER_NONE, 1); // set filter none.
    }
    else
    {
        BB_FilterSet(BB_GetFilterByVtFrqCh(context.rf_info.u8_prevSweepCh), 0);
    }
}

static int __attribute__ ((section(".h264"))) BB_CmdSweep( void )
{
    uint8_t u8_maxCh = BB_GetSweepTotalCh( context.e_curBand, context.rf_info.e_bw);
    uint8_t result = 0;
    uint8_t curRow = context.rf_info.u8_curBb1Row;

    result   = BB_GetSweepPower( context.e_curBand,
                                 context.rf_info.e_bw,
                                 curRow,
                                 context.rf_info.u8_prevSweepCh,
                                 0 );

    if( result )
    {
        uint8_t nextch = NEXT_NUM(context.rf_info.u8_prevSweepCh, u8_maxCh);
        if ( nextch < context.rf_info.u8_prevSweepCh )
        {
            context.rf_info.u8_curBb1Row = NEXT_NUM(context.rf_info.u8_curBb1Row, SWEEP_FREQ_BLOCK_ROWS);
            if ( context.rf_info.u8_curBb1Row == 0)
            {
                context.rf_info.u8_cmdSweep = 0;
                BB_FilterSet(BB_GetFilterByVtFrqCh(context.cur_IT_ch), 1); // recover filter
                BB_Sweep_updateCh(context.e_curBand, context.cur_IT_ch );
                return 0;
            }
        }
        
        context.rf_info.u8_prevSweepCh = nextch;

        BB_set_SweepFrq(context.e_curBand, context.rf_info.e_bw, context.rf_info.u8_prevSweepCh);
        //DLOG_Warning("%d %d", context.rf_info.u8_curBb1Row, context.rf_info.u8_prevSweepCh);
        if(1 == sweep_with_filter)
        {
            BB_FilterSet(BB_GetFilterByVtFrqCh(context.rf_info.u8_prevSweepCh), 0);
        }
    }

    return -1;
}

int __attribute__ ((section(".h264"))) BB_SetNewItCh_600m( uint8_t ch )
{
    uint8_t start = context.rc_start;
    context.cur_IT_ch  = ch;
    BB_Sweep_updateCh(context.e_curBand, context.cur_IT_ch );
    
    BB_GetItStarEndByItCh((uint8_t *)(&context.vt_start), (uint8_t *)(&context.vt_end), context.cur_IT_ch);
    BB_GetRcStarEndByItCh((uint8_t *)(&context.rc_start), (uint8_t *)(&context.rc_end), context.cur_IT_ch);
    if(start != context.rc_start)
    {
        BB_ResetRcMap();
    }
    //DLOG_Warning("vt_s:%d vt_e:%d rc_s:%d rc_e:%d m:%d", context.vt_start,context.vt_end,\
    //                                            context.rc_start, context.rc_end, context.cur_IT_ch);
    
    BB_set_ItFrqByCh(context.e_curBand, context.cur_IT_ch);
    BB_grd_NotifyItFreqByCh(context.e_curBand, context.cur_IT_ch);

    return 0;
}


static int __attribute__ ((section(".h264"))) BB_SweepBeforeFull( void )
{
    uint8_t u8_maxCh = BB_GetSweepTotalCh( context.e_curBand, context.rf_info.e_bw);
    uint8_t result = 0;
    uint8_t curRow = (context.e_curBand == RF_5G) ? (context.rf_info.u8_curBb2Row) : (context.rf_info.u8_curBb1Row);

#if(FULL_BAND_SWEEP_EN)
    static uint8_t full_band_sw_flag = 0;
    DLOG_Warning("before sw,chn %d,maxch %u\n", context.rf_info.u8_prevSweepCh, u8_maxCh);
#endif

    result   = BB_GetSweepPower( context.e_curBand,
                                 context.rf_info.e_bw,
                                 curRow,
                                 context.rf_info.u8_prevSweepCh,
                                 0 );

    if( result )
    {
        uint8_t nextch = NEXT_NUM(context.rf_info.u8_prevSweepCh, u8_maxCh);
        if ( nextch < context.rf_info.u8_prevSweepCh )
        {
            uint8_t flag = 0;
            if ( context.e_curBand == RF_5G )
            {
                context.rf_info.u8_curBb2Row = NEXT_NUM(context.rf_info.u8_curBb2Row, SWEEP_FREQ_BLOCK_ROWS);
                if ( context.rf_info.u8_curBb2Row == 0)
                {
                    context.rf_info.u8_isFull |= 0x01;
                }
            }
            else
            {
                context.rf_info.u8_curBb1Row = NEXT_NUM(context.rf_info.u8_curBb1Row, SWEEP_FREQ_BLOCK_ROWS);
                if ( context.rf_info.u8_curBb1Row == 0)
                {
                    context.rf_info.u8_isFull |= 0x01;
                }
            }

#if(FULL_BAND_SWEEP_EN)
            if( context.rf_info.u8_isFull ){
                //if(context.st_bandMcsOpt.e_rfbandMode == AUTO && context.st_bandMcsOpt.e_bandsupport == RF_2G_5G){
                if( context.st_bandMcsOpt.e_bandsupport == RF_2G_5G ){
                    if(!full_band_sw_flag){
                        context.e_curBand = RF_5G;
                        context.rf_info.u8_isFull = 0;
                        full_band_sw_flag = 1;
                        DLOG_Warning("RF_2G Full, switch to RF_5G\n");
                    }else{
                        context.e_curBand == RF_2G;
                    }
                }
            }
#endif


            if( context.rf_info.u8_isFull &&  context.itHopMode == AUTO )
            {
                uint8_t mainch = 0, opt = 0;

                #ifdef RF_9363
                //if(RF_600M == context.e_curBand)
                {
                    BB_selectBestCh600m( context.e_curBand, SELECT_MAIN_OPT, &mainch, &opt, NULL, 0 );
                    BB_SetNewItCh_600m(mainch);
                }
                #endif

                #ifdef RF_8003X
                //else
                {
                    BB_selectBestCh( context.e_curBand, SELECT_MAIN_OPT, &mainch, &opt, NULL, 0 );

                    context.rf_info.u8_mainSweepCh = mainch;
                    context.rf_info.u8_mainCh      = mainch;
                
                    context.rf_info.u8_prevSweepCh = context.rf_info.u8_mainSweepCh;

                    context.rf_info.u8_optCh = opt;
                    context.rf_info.u8_optSweepCh = opt;
                    context.vtFreqTime[context.cur_IT_ch] = SysTicks_GetTickCount();//save last main ch time value

                    context.cur_IT_ch  = mainch;
                    //BB_Sweep_updateCh(context.e_curBand, context.cur_IT_ch );
                    BB_set_ItFrqByCh(context.e_curBand, context.cur_IT_ch);
                    BB_grd_NotifyItFreqByCh(context.e_curBand, context.cur_IT_ch);
                }
                #endif
                
                context.rf_info.u8_cycleCnt = MAIN_CH_CYCLE;
                #ifdef RFSUB_BAND
                if(context.freq_band_mode == SUB_BAND)
                {
                    context.sub_band_main_ch = mainch;
                    context.sub_band_opt_ch = opt;
                    context.sub_band_main_opt_ch_time = SysTicks_GetTickCount();
                    DLOG_Warning("sweep ok, sub band %d->%d %d", context.sub_band_value,context.cur_IT_ch,opt);
                    context.sub_band_value = context.cur_IT_ch;
                    context.sub_rc_start = BB_GetSubBandStartCH(context.e_curBand,context.st_bandMcsOpt.e_bandwidth,context.sub_band_value);
                    context.sub_rc_end = BB_GetSubBandEndCH(context.e_curBand,context.st_bandMcsOpt.e_bandwidth,context.sub_band_value);
                    context.grd_rc_channel = context.sub_rc_start;
                }
				#endif
                BB_SetTrxMode(BB_NORMAL_MODE);

                return result;
            }
            else
            {
                context.rf_info.u8_prevSweepCh = nextch;
            }
        }
        else
        {
            context.rf_info.u8_prevSweepCh = nextch;
        }

        BB_set_SweepFrq( context.e_curBand, context.rf_info.e_bw, context.rf_info.u8_prevSweepCh );

        #ifdef RF_9363
        if(1 == sweep_with_filter)
        {
            BB_FilterSet(BB_GetFilterByVtFrqCh(context.rf_info.u8_prevSweepCh), 0);
        }
        #endif
    }

    return result;
}


static int __attribute__ ((section(".h264"))) BB_set_sweepChannel(void)
{
    uint8_t shift = 0;
    uint8_t u8_maxCh;
    uint8_t cycle = (context.rf_info.u8_cycleCnt % context.rf_info.u8_totalCyc);
    ENUM_RF_BAND e_curBand = context.e_curBand;
    ENUM_CH_BW   e_bw      = context.rf_info.e_bw;

    if(cycle == OTHER_CH_CYCLE)
    {
        u8_maxCh = BB_GetSweepTotalCh( e_curBand, e_bw);
        uint8_t u8_nextch = 0;
        do
        {
            u8_nextch = NEXT_NUM(context.rf_info.u8_spareSweepCh, u8_maxCh);
            if( u8_nextch < context.rf_info.u8_spareSweepCh )
            {
                if ( e_curBand == RF_5G )    //5G mode: switch to 5G next row
                {
                    context.rf_info.u8_curBb2Row = NEXT_NUM(context.rf_info.u8_curBb2Row, SWEEP_FREQ_BLOCK_ROWS);
                }
                else
                {
                    context.rf_info.u8_curBb1Row = NEXT_NUM(context.rf_info.u8_curBb1Row, SWEEP_FREQ_BLOCK_ROWS);
                }
            }
            context.rf_info.u8_spareSweepCh = u8_nextch;
        } while( u8_nextch == context.rf_info.u8_mainSweepCh || u8_nextch == context.rf_info.u8_optSweepCh );

        context.rf_info.e_prevSweepBand  = e_curBand;
        context.rf_info.u8_prevSweepCh = context.rf_info.u8_spareSweepCh;
    }
    else if( cycle == MAIN_CH_CYCLE )
    {
        context.rf_info.e_prevSweepBand  = e_curBand;
        context.rf_info.u8_prevSweepCh   = context.rf_info.u8_mainSweepCh;
        context.rf_info.u8_mainSweepRow  = NEXT_NUM(context.rf_info.u8_mainSweepRow, SWEEP_FREQ_BLOCK_ROWS);
#ifdef RF_9363
        if(RF_600M == context.e_curBand)
        {
            BB_FilterSet(BB_GetFilterByVtFrqCh(context.rf_info.u8_prevSweepCh), 0);
        }
#endif
    }
    else if( cycle == OPT_CH_CYCLE )
    {
        context.rf_info.e_prevSweepBand  = e_curBand;
        context.rf_info.u8_prevSweepCh   = context.rf_info.u8_optSweepCh;
        context.rf_info.u8_optSweepRow   = NEXT_NUM(context.rf_info.u8_optSweepRow, SWEEP_FREQ_BLOCK_ROWS);    
    }
    else if(cycle == OTHER_BAND_CH_CYCLE)  // if RF_2G or RF_5G only, no OTHER_BAND_CH_CYCLE
    {
        ENUM_RF_BAND newband = OTHER_BAND( context.e_curBand );
        u8_maxCh = BB_GetSweepTotalCh(newband, e_bw);
		//u8_maxCh = BB_GetSweepTotalCh(context.e_curBand, e_bw);

        context.rf_info.u8_optBandSweepCh = NEXT_NUM(context.rf_info.u8_optBandSweepCh, u8_maxCh);
        context.rf_info.u8_prevSweepCh    = context.rf_info.u8_optBandSweepCh;
		//context.rf_info.e_prevSweepBand  = e_curBand;
        context.rf_info.e_prevSweepBand   = newband;

        if ( context.rf_info.u8_prevSweepCh == 0 ) //start channel from 0, new row.
        {
            if ( newband == RF_5G )
			//if (context.e_curBand == RF_5G )
            {
                context.rf_info.u8_curBb2Row = NEXT_NUM(context.rf_info.u8_curBb2Row, SWEEP_FREQ_BLOCK_ROWS);
            }
            else
            {
                context.rf_info.u8_curBb1Row = NEXT_NUM(context.rf_info.u8_curBb1Row, SWEEP_FREQ_BLOCK_ROWS);
            }
        }
    }

    BB_set_SweepFrq(context.rf_info.e_prevSweepBand, e_bw, context.rf_info.u8_prevSweepCh);

    {
        STRU_WIRELESS_INFO_DISPLAY *osdptr = (STRU_WIRELESS_INFO_DISPLAY *)(SRAM_BB_STATUS_SHARE_MEMORY_ST_ADDR);
        if((context.rf_info.e_bandsupport == RF_2G_5G) && (e_curBand == RF_5G))
        {
            shift = BB_GetItFrqNum(RF_2G);
        }
        {
            static uint8_t itch_bak  = 0;
            static uint8_t itopt_bak = 0;
            osdptr->IT_channel  = context.rf_info.u8_mainCh + shift;
            osdptr->u8_optCh    = context.rf_info.u8_optCh  + shift;
            if ( osdptr->IT_channel != itch_bak ||  osdptr->u8_optCh != itopt_bak )
            {
                //DLOG_Info("From:(%d %d) To:(%d %d)", itch_bak, itopt_bak, osdptr->IT_channel, osdptr->u8_optCh);
                itch_bak  = osdptr->IT_channel;
                itopt_bak = osdptr->u8_optCh;
            }
        }
    }

    return 0;
}


static int __attribute__ ((section(".h264"))) BB_SweepAfterFull( uint8_t flag )
{
    ENUM_RF_BAND e_curBand   = context.e_curBand;
    ENUM_RF_BAND e_sweepBand = context.rf_info.e_prevSweepBand;
    ENUM_CH_BW   e_bw = context.rf_info.e_bw;

    uint8_t result   = 0;
    uint8_t cycle  = (context.rf_info.u8_cycleCnt % context.rf_info.u8_totalCyc);
    uint8_t flag_checkBandSwitch = 0;

    uint8_t u8_mainSweepCh = context.rf_info.u8_mainSweepCh;
    uint8_t u8_optSweepCh  = context.rf_info.u8_optSweepCh;
    uint8_t u8_prevSweepCh = context.rf_info.u8_prevSweepCh;

    uint8_t u8_sweepRow;
    if( e_sweepBand == e_curBand && u8_prevSweepCh == u8_mainSweepCh )  //main channel sweep result
    {
        u8_sweepRow = context.rf_info.u8_mainSweepRow;    
    }
    else if ( e_sweepBand == e_curBand && u8_prevSweepCh == u8_optSweepCh ) //opt channel sweep result
    {
        u8_sweepRow = context.rf_info.u8_optSweepRow;    
    }
    else
    {  
        if ( e_sweepBand == RF_5G )
        {
            u8_sweepRow = context.rf_info.u8_curBb2Row;
        }
        else
        {
            u8_sweepRow = context.rf_info.u8_curBb1Row;
        }

        if (e_sweepBand != e_curBand &&  (u8_prevSweepCh + 1) == BB_GetSweepTotalCh(e_sweepBand, e_bw)) //last channel
        {
            if ( (u8_sweepRow + 1) == SWEEP_FREQ_BLOCK_ROWS && (context.rf_info.u8_isFull & 0x10 )!= 0x10) //last row
            {
                context.rf_info.u8_isFull |= 0x10;
                DLOG_Info("option band sweep is full !!");
            }

            //both band sweep is ready, and sweep last channel
            if ( (context.rf_info.u8_isFull & 0x10) && (context.locked) &&
                 (!BB_grd_isSearching()) && \
                 context.st_bandMcsOpt.e_rfbandMode == AUTO && context.itHopMode == AUTO && \
                 context.st_bandMcsOpt.e_bandsupport == RF_2G_5G) 
            {
                flag_checkBandSwitch = 1;
            }
        }
    }
    result = BB_GetSweepPower(e_sweepBand, e_bw, u8_sweepRow, u8_prevSweepCh, flag);
    if (0 == result)
    {
        return 0;
    }
    BB_UpdateOptCh(e_sweepBand, e_bw, u8_prevSweepCh);
#if 0
    //check if band switch need
    if (flag_checkBandSwitch)
    {
        uint8_t i = 0;
        ENUM_RF_select rf_sel = BB_grd_cmpBandNoise();
        if (rf_sel == UNKNOWN_BETTER)
        {
            //dont know which band is better, dont take into count
            return 0;
        }

        context.rf_info.band_sel[context.rf_info.u8_bandSelCnt] = rf_sel;
        context.rf_info.u8_bandSelCnt ++;

        //get enough sweep data for band switch
        uint8_t cnt = sizeof(context.rf_info.band_sel) / sizeof(context.rf_info.band_sel[0]);
        if (context.rf_info.u8_bandSelCnt >= cnt)
        {
            uint8_t better_2g_count = 0;
            uint8_t better_5g_count = 0;

            for( i = 0; i < cnt; i++)
            {
                better_2g_count += (context.rf_info.band_sel[i] == BETTER_2G);
                better_5g_count += (context.rf_info.band_sel[i] == BETTER_5G);
            }

            if (context.rf_info.e_curBand == RF_2G && better_5g_count >= cnt * 5/6 )
            {
                context.e_bandSelFromSweep = BETTER_5G;
            }
            else if (context.rf_info.e_curBand == RF_5G && better_2g_count >= cnt * 5/6)
            {
                context.e_bandSelFromSweep = BETTER_2G;
            }
            else
            {
                context.e_bandSelFromSweep = EQUAL_2G_5G;
            }

            DLOG_Info("Band sel: %d %d %d %d %d", context.e_bandSelFromSweep, better_2g_count, better_5g_count,
                                                  context.u_bandSwitchParam->grd_bandSwitchParm.i8_GrdBand0_2_Band1SweepAverUnblock,
                                                  context.u_bandSwitchParam->grd_bandSwitchParm.i8_GrdBand1_2_Band0SweepAverUnblock);

            context.rf_info.u8_bandSelCnt = 0;

            //if mcs <= 1 && snr <= mcs1_snr
            if (context.qam_ldpc <= context.u_bandSwitchParam->grd_bandSwitchParm.u8_minMcsForBandSwitch && grd_snr_forBandChange())
            {
                //check band switch
                if ( (context.e_curBand == RF_2G && context.e_bandSelFromSweep == BETTER_5G) ||
                     (context.e_curBand == RF_5G && context.e_bandSelFromSweep == BETTER_2G))
                {
                    if ((context.st_bandMcsOpt.e_rfbandMode == AUTO) && (context.stru_bandSwitchParam.u8_bandSwitchAfterUnlock == 0x81))
                    {
                        grd_requestRfBandSwitch((context.e_curBand == RF_2G) ? RF_5G:RF_2G);
                        DLOG_Warning("band switch %d %d %d %d", context.e_curBand, context.e_bandSelFromSweep,
                                                                        context.qam_ldpc, context.u_bandSwitchParam->grd_bandSwitchParm.u8_minMcsForBandSwitch);
                    }
                }
            }            
        }
    }
#endif
    //if get the right cycle result, do next cycle sweep
    if (result)
    {
        context.rf_info.u8_cycleCnt ++;
		
        BB_set_sweepChannel();
    }

    return result;
}

//run time 0 slice
uint8_t __attribute__ ((section(".h264"))) BB_GetSweepedChResult( uint8_t flag )
{
    uint8_t  ret = 0;

    if(1 == context.rf_info.u8_cmdSweep)
    {
        BB_CmdSweep();
    }
    else if (!context.rf_info.u8_isFull )
    {
        ret = BB_SweepBeforeFull( );
		#ifdef RFSUB_BAND
        if(context.freq_band_mode == SUB_BAND && ret)
        {
            BB_softReset(BB_GRD_MODE);
        }
		#endif
    }
    else
    {
        ret = BB_SweepAfterFull( flag );
    }    

    return ret;
}

uint8_t __attribute__ ((section(".h264"))) BB_CompareCh1Ch2ByPowerAver(ENUM_RF_BAND e_rfBand, uint8_t u8_itCh1, uint8_t u8_itCh2, int32_t level)
{
    uint8_t value = 0;
                                             
    if(context.bandedge_enable)
    {
        if(u8_itCh1 == u8_itCh2)
        {
            return 0;
        }
        else if(u8_itCh2 == 0 || u8_itCh2 == context.rf_info.sweep_freqsize-1) //current it ch
        {
            if(context.lna_status == OPEN_LNA) // lna open ,ch0,ch6 always give up
            {
                return 1;
            }

            level -= (context.low_power_db); // lna bypass ,other ch only better ch0,ch6 snr 
        }
        else if(u8_itCh1 == 0 || u8_itCh1 == context.rf_info.sweep_freqsize-1)
        {
            if(context.lna_status == OPEN_LNA) // lna open ,ch0,ch6 always give up
            {
                return 0;
            }
            level += context.low_power_db; // lna bypass ,ch0,ch6 snr must better than low-power + 2(ref value)
        }
    }
    
    if (context.rf_info.sweep_pwr_avrg_value[u8_itCh1].value < (context.rf_info.sweep_pwr_avrg_value[u8_itCh2].value  - level))
    {
        value = 1;
    }
    
    return value;
}

uint8_t __attribute__ ((section(".h264"))) BB_CompareCh1Ch2ByPower(ENUM_RF_BAND e_rfBand, uint8_t u8_itCh1, uint8_t u8_itCh2, uint8_t u8_cnt)
{
    uint8_t value = 0;
    uint8_t row;
    uint8_t tmpCnt = 0;
    if (u8_cnt > SWEEP_FREQ_BLOCK_ROWS)
    {
        u8_cnt = SWEEP_FREQ_BLOCK_ROWS;
    }

    for( row = 0; row < SWEEP_FREQ_BLOCK_ROWS; row++)
    {      
        tmpCnt += ((context.rf_info.sweep_pwr_table[row][u8_itCh1].value < context.rf_info.sweep_pwr_table[row][u8_itCh2].value) ? (1) : (0));
    }
    
    value = ((tmpCnt >= u8_cnt) ? (1) : (0));

    return value;
}


static uint8_t __attribute__ ((section(".h264"))) is_inlist(uint8_t item, uint8_t *pu8_exlude, uint8_t u8_cnt)
{
    uint8_t flag = 0;
    uint8_t i;
    for ( i = 0; i < u8_cnt && flag == 0; i++) //channel in the excluding list
    {
        flag = ( item == pu8_exlude[i] );
    }

    return flag;
}

static uint8_t __attribute__ ((section(".h264"))) find_best(uint8_t *pu8_exlude, uint8_t u8_exclude_cnt, ENUM_RF_BAND e_rfBand, uint8_t log)
{
    uint8_t u8_start = 0;
    uint8_t ch = 0;
    int16_t aver, fluct;
    uint8_t num = BB_GetItFrqNum(e_rfBand);


    for ( u8_start = 0; 
          u8_start < num && is_inlist( u8_start, pu8_exlude, u8_exclude_cnt );  //channel in the excluding list
          u8_start++) 
    {}
    
    for( ch; ch < num; ch++)
    {
        if ( !is_inlist( ch, pu8_exlude, u8_exclude_cnt ) )
        {
            if(BB_CompareCh1Ch2ByPowerAver(e_rfBand, ch, u8_start, CMP_POWER_AVR_LEVEL))
            {
                u8_start = ch;
            }
        }
    }

    return u8_start;
}

static void __attribute__ ((section(".h264"))) BB_GetItMinMaxCh(ENUM_RF_BAND e_rfBand, ENUM_CH_BW e_bw, uint8_t *min_ch, uint8_t *max_ch)
{
    *max_ch = BB_GetItFrqNum(e_rfBand) - 1;
    *min_ch = 0;
}

static void __attribute__ ((section(".h264"))) BB_GetItAdjacentFrequency(uint8_t ch, uint8_t *pre, uint8_t *next)
{
    *pre  = ch - 1;
    *next = ch + 1;
}


/*
 * u8_opt: 
 *         SELECT_OPT: do not change the main channel. only select one optional channel
 *         SELECT_MAIN_OPT: select the main and opt channel
 *         CHANGE_MAIN: change the main channel, and select another one
 * return: 
 *         0: no suggest main and opt channel
 *         1: get main and opt channel
*/
uint8_t __attribute__ ((section(".h264"))) BB_selectBestCh(ENUM_RF_BAND e_band, ENUM_CH_SEL_OPT e_opt,
                        uint8_t *pu8_mainCh, uint8_t *pu8_optCh, uint8_t *pu8_other,
                        uint8_t log)
{
    uint8_t  ch = 0;
    int16_t  aver, fluct;
    uint32_t start = SysTicks_GetTickCount();
    uint8_t  exclude[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
    uint8_t  excludecnt = 0;
    ENUM_CH_BW e_bw = context.rf_info.e_bw;
    uint8_t bestCh  = 0xff;         //select one from all the channels
    uint8_t betterCh= 0xff;
    uint8_t other   = 0xff;
    //uint8_t index = 3;
    uint8_t min_ch;
    uint8_t max_ch;
    uint8_t pre_ch;
    uint8_t next_ch;
    uint8_t tmpCh;

    if (!context.rf_info.u8_isFull){
        return 0;
    }
    BB_GetItMinMaxCh(e_band, e_bw, &min_ch, &max_ch);
    
    //select main channel
    if (e_opt == SELECT_MAIN_OPT)
    {
        excludecnt = 0;
        bestCh = find_best(exclude, excludecnt, e_band, log);
        if ( pu8_mainCh )
        {
            *pu8_mainCh = bestCh;
        }
    }

    //select optional channel
    if ( e_opt == SELECT_MAIN_OPT || e_opt == SELECT_OPT )
    {
        exclude[0] = context.rf_info.u8_preMainCh;
        exclude[1] = context.rf_info.u8_mainCh;
        exclude[2] = bestCh;
        
        tmpCh = (e_opt == SELECT_OPT) ? (context.rf_info.u8_mainCh) : (bestCh);
        BB_GetItAdjacentFrequency(tmpCh, &pre_ch, &next_ch);
        exclude[3] = pre_ch;
        exclude[4] = next_ch;
        excludecnt = 5;

        if ( log )
        {
            DLOG_Info("exclude: %d %d %d", exclude[0], exclude[1], exclude[2]);
        }

        betterCh = find_best(exclude, excludecnt, e_band, log);

        if ((pre_ch >= min_ch) && (pre_ch <= max_ch))
        {
            if (BB_CompareCh1Ch2ByPowerAver(e_band, pre_ch, betterCh, 0))
            {
                betterCh = pre_ch;
            }
        }
        if ((next_ch >= min_ch) && (next_ch <= max_ch))
        {
            if (BB_CompareCh1Ch2ByPowerAver(e_band, next_ch, betterCh, 0))
            {
                betterCh = next_ch;
            }
        }
        
        if ( pu8_optCh )
        {
            *pu8_optCh= betterCh;
        }
    }

    if ( e_opt == SELECT_OTHER )
    {
        exclude[0] = context.rf_info.u8_preMainCh;
        exclude[1] = context.rf_info.u8_mainCh;
        exclude[2] = context.rf_info.u8_optCh;   
        excludecnt = 3;

        other = find_best(exclude, excludecnt, e_band, log);

        if (pu8_other)
        {
            *pu8_other = other;
        }
    }

    if ( log ){
        DLOG_Info("--ch--: %d %d %d", bestCh, betterCh, e_opt);
    }    


    return 1;
}

uint8_t __attribute__ ((section(".h264"))) BB_selectBestCh600m(ENUM_RF_BAND e_band, ENUM_CH_SEL_OPT e_opt,
                        uint8_t *pu8_mainCh, uint8_t *pu8_optCh, uint8_t *pu8_other,
                        uint8_t log)
{
    uint8_t  exclude[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
    uint8_t  excludecnt = 0;
    ENUM_CH_BW e_bw = context.rf_info.e_bw;
    uint8_t bestCh  = 0xff;         //select one from all the channels
    uint8_t betterCh= 0xff;
    uint8_t other   = 0xff;
    //uint8_t index = 3;
    uint8_t min_ch;
    uint8_t max_ch;
    uint8_t pre_ch;
    uint8_t next_ch;
    uint8_t tmpCh;

    
    bestCh = find_best(exclude, excludecnt, e_band, log);
    if ( pu8_mainCh )
    {
        *pu8_mainCh = bestCh;
    }

    tmpCh = BB_GetItChInOneFilter(bestCh);
    if ( pu8_optCh )
    {
        *pu8_optCh= tmpCh;
    }
    
    if (pu8_other)
    {
        *pu8_other = tmpCh;
    }
}

uint8_t __attribute__ ((section(".h264"))) get_opt_channel( void )
{
    uint8_t other = 0;
    int32_t level;
    uint8_t ret = context.rf_info.u8_optCh;
    ENUM_RF_BAND e_band = context.e_curBand;

    BB_selectBestCh(e_band, SELECT_OTHER, NULL, NULL, &other, 0);
    
    if ( context.agclevel >= AGC_LEVEL_1 )
    {
        level = 3;
    }
    else if( context.agclevel >= AGC_LEVEL_0 )
    {
        level = 6;
    }
    else
    {
        level = 9;
    }

    level = ((1 == BB_JudgeAdjacentFrequency(other)) ? (2 * level) : (level));
    if (BB_CompareCh1Ch2ByPower(e_band, other, context.rf_info.u8_optCh, SWEEP_FREQ_BLOCK_ROWS))
	    if (BB_CompareCh1Ch2ByPowerAver(e_band, other, context.rf_info.u8_optCh, 0))
	    {
	        #if 0
	        //print bandedge 
	            int32_t * pi32_power_average = (e_band == RF_5G) ? context.rf_info.i32_rf1PwrAvr :
	                                                         context.rf_info.i32_rf0PwrAvr;
	            DLOG_Warning("lna=%d,%d->%d, db,%d,%d,%d,%d,%d,%d,%d,",context.lna_status,context.rf_info.u8_optCh,other, \
	                pi32_power_average[0], \
	                pi32_power_average[1],pi32_power_average[2],pi32_power_average[3],pi32_power_average[4],pi32_power_average[5],pi32_power_average[6]);
	        #endif
	        ret = other; 
	    }

    return ret;
}

static uint8_t __attribute__ ((section(".h264"))) BB_GetSweepTotalCh(ENUM_RF_BAND e_rfBand, ENUM_CH_BW e_bw)
{
    uint8_t value = BB_GetItFrqNum(e_rfBand);
    return value;
}

static uint8_t __attribute__ ((section(".h264"))) BB_JudgeAdjacentFrequency(uint8_t judge_ch)
{
    if ((judge_ch == (context.rf_info.u8_mainCh + 1)) || (judge_ch == (context.rf_info.u8_mainCh - 1)))
    {
        return 1;
    }
    else
    {
        return 0;
    }
}

static uint8_t __attribute__ ((section(".h264"))) BB_UpdateOptCh(ENUM_RF_BAND e_rfBand, ENUM_CH_BW e_bw, uint8_t sweep_ch)
{
    uint8_t tmpCh = context.rf_info.u8_optCh;
    uint8_t u8_maxCh = BB_GetItFrqNum( e_rfBand );
    int32_t level;
    int i=0;
    if ( e_rfBand != context.e_curBand || sweep_ch == context.rf_info.u8_mainCh || sweep_ch == context.rf_info.u8_optCh)
    {
        return 0;
    }

    level = ((1 == BB_JudgeAdjacentFrequency(sweep_ch)) ? (2 * CMP_POWER_AVR_LEVEL) : (CMP_POWER_AVR_LEVEL));
    if (BB_CompareCh1Ch2ByPowerAver(e_rfBand, sweep_ch, tmpCh, 0))
    {
        tmpCh = sweep_ch;
    }    

    if (tmpCh != context.rf_info.u8_optCh)
    {
    	if(RF_5G == (context.e_curBand)){
	        context.rf_info.u8_bestBb2ChCnt[tmpCh] += 1;
	        if (context.rf_info.u8_bestBb2ChCnt[tmpCh] > 10)
	        {
	            context.rf_info.u8_optCh = tmpCh;
	            context.rf_info.u8_optSweepCh = tmpCh;
				for(i=0;i<u8_maxCh;i++)context.rf_info.u8_bestBb2ChCnt[i]=0;
	           
	        }
		}
		else
	    {
	    	 context.rf_info.u8_bestBb1ChCnt[tmpCh] += 1;
	        if (context.rf_info.u8_bestBb1ChCnt[tmpCh] > 10)
	        {
	            context.rf_info.u8_optCh = tmpCh;
	            context.rf_info.u8_optSweepCh = tmpCh;
				for(i=0;i<u8_maxCh;i++)context.rf_info.u8_bestBb1ChCnt[i]=0;
	           
	        }
		}
		
    }
    else
    {
    	if(RF_5G == (context.e_curBand))
        	context.rf_info.u8_bestBb2ChCnt[tmpCh] = 0;
		else
			context.rf_info.u8_bestBb1ChCnt[tmpCh] =0;
    }
}


/*
 * round: round of the loop
 * sort the noise from Low -> High
*/
void __attribute__ ((section(".h264"))) bubble_sort(int * data, int datasize, int round)
{
    uint8_t i;
    uint8_t j;

    for ( i = 0 ; i < round; i++)
    {
        for( j = i+1; j < datasize; j++ )
        {
            if ( data[i] > data[j] )
            {
                int tmp = data[i];  //do swap
                data[i] = data[j];
                data[j] = tmp;
            }
        }
    }
}


/*
 *  start to sweep another band
 */
void __attribute__ ((section(".h264"))) BB_SweepChangeBand(ENUM_RF_BAND e_toRfBand, uint8_t u8_mainCh, uint8_t u8_optCh)
{
    context.rf_info.u8_spareSweepCh   =  0;
    context.rf_info.u8_optBandSweepCh =  0;

    context.rf_info.u8_curBb1Row   =  0;
    context.rf_info.u8_curBb2Row   =  0;

    context.rf_info.u8_cycleCnt      =  MAIN_CH_CYCLE;
    context.rf_info.u8_mainSweepRow  =  0;
    context.rf_info.u8_optSweepRow   =  0;

    context.rf_info.u8_preMainCh  =  0xff;

    context.rf_info.u8_mainCh     =  u8_mainCh;

    context.rf_info.u8_prevSweepCh   = 0x0;
    context.rf_info.u16_preMainCount = 0x0;

    context.rf_info.u8_mainSweepCh = u8_mainCh;
    context.rf_info.u8_prevSweepCh = u8_mainCh;

    context.rf_info.u8_optCh       = u8_optCh;
    context.rf_info.u8_optSweepCh  = u8_optCh;

    context.e_curBand      =  e_toRfBand;  //current rf band for sweep
    context.rf_info.u8_bandSelCnt  =  0;

    BB_set_SweepFrq(e_toRfBand, context.rf_info.e_bw ,u8_mainCh);
}



int8_t __attribute__ ((section(".h264"))) grd_getSwitchDiffBase(int band0_2_band1, uint8_t block)
{
    int8_t ret;
    if (block)
    {
        ret =  (band0_2_band1) ? context.u_bandSwitchParam->grd_bandSwitchParm.i8_GrdBand0_2_Band1SweepAverBlock : 
                                 context.u_bandSwitchParam->grd_bandSwitchParm.i8_GrdBand1_2_Band0SweepAverBlock;
    }
    else
    {
        ret =  (band0_2_band1) ? context.u_bandSwitchParam->grd_bandSwitchParm.i8_GrdBand0_2_Band1SweepAverUnblock : 
                                 context.u_bandSwitchParam->grd_bandSwitchParm.i8_GrdBand1_2_Band0SweepAverUnblock;
    }

    ret += (context.stru_bandSwitchParam.i8_skyPower0 - context.stru_bandSwitchParam.i8_skyPower1);

    return ret;
}


/*
 * function to compare the 2G/5G band noise
*/

#if 0
ENUM_RF_select __attribute__ ((section(".h264"))) BB_grd_cmpBandNoise( void )
{
    int i32_2G_noisepower_average[context.rf_info.u8_bb1ItFrqSize];
    int i32_5G_noisepower_average[context.rf_info.u8_bb2ItFrqSize];
    int i32_2GAverage = 0;
    int i32_5GAverage = 0;
	int i=0;

    int8_t i8_diff0_1 = grd_getSwitchDiffBase(1, context.rf_info.flag_signalBlock);
    int8_t i8_diff1_0 = grd_getSwitchDiffBase(0, context.rf_info.flag_signalBlock);

    int count1 = context.u_bandSwitchParam->grd_bandSwitchParm.u8_GrdBand0CmpChannelCnt;
    int count2 = context.u_bandSwitchParam->grd_bandSwitchParm.u8_GrdBand1CmpChannelCnt;

	for(i=0;i<context.rf_info.u8_bb1ItFrqSize;i++){
		i32_2G_noisepower_average[i]=context.rf_info.sweep_pwr_avrg_value[i].value;
	}
	
    memcpy(i32_2G_noisepower_average, context.rf_info.i32_rf0PwrAvr, sizeof(int) * (context.rf_info.u8_bb1ItFrqSize));
    memcpy(i32_5G_noisepower_average, context.rf_info.i32_rf1PwrAvr, sizeof(int) * (context.rf_info.u8_bb2ItFrqSize));

    //bubble sort the Noise power, get the best channels
    bubble_sort(i32_2G_noisepower_average, context.rf_info.u8_bb1ItFrqSize, count1);
    bubble_sort(i32_5G_noisepower_average, context.rf_info.u8_bb2ItFrqSize, count2);

    uint8_t tmp = 0;
    for (tmp = 0; tmp < count1; tmp++)
    {
        i32_2GAverage += i32_2G_noisepower_average[tmp];
    }
    i32_2GAverage /= count1;

    for (tmp = 0; tmp < count2; tmp++)
    {
        i32_5GAverage += i32_5G_noisepower_average[tmp];
    }
    i32_5GAverage /= count2;

    //compare result.
    //if dont get the current RF mode(BLOCK, UNBLOCK), dont add in count
    if (context.rf_info.flag_signalBlock == RF_SIGNAL_UNKNOWN_MODE)
    {
        DLOG_Info("UNKNOWN_BLOCK");
        return UNKNOWN_BETTER;
    }
    else
    {
        if (i32_2GAverage >= i32_5GAverage + i8_diff0_1)
        {
            DLOG_Info("5G: %d %d %d %d", context.rf_info.flag_signalBlock, (i32_2GAverage - i32_5GAverage), i8_diff0_1, i8_diff1_0);
            return BETTER_5G;
        }
        else if (i32_2GAverage <= i32_5GAverage + i8_diff1_0)
        {
            DLOG_Info("2G: %d %d %d %d", context.rf_info.flag_signalBlock, (i32_2GAverage - i32_5GAverage), i8_diff0_1, i8_diff1_0);
            return BETTER_2G;
        }
        else
        {
            DLOG_Info("Equal: %d %d %d %d", context.rf_info.flag_signalBlock, (i32_2GAverage - i32_5GAverage), i8_diff0_1, i8_diff1_0);
            return EQUAL_2G_5G;
        }
    }
}
#endif

uint8_t agc_level_min[50];
uint8_t agc_idx = 0;

void __attribute__ ((section(".h264"))) grd_checkBlockMode(void)
{
    int      dist   = 0;
    uint16_t agcsum = 0;
    ENUM_RF_SIGNAL_BLOCK e_signal = context.rf_info.flag_signalBlock;
    uint8_t  num    = sizeof(agc_level_min) / sizeof(agc_level_min[0]);

    agc_level_min[agc_idx] = ((stru_grdstatus.agc_value1 < stru_grdstatus.agc_value2) ? \
                               stru_grdstatus.agc_value1 : stru_grdstatus.agc_value2);
    agc_idx ++;
    if (agc_idx >= num)
    {
        uint8_t i = 0;
        int8_t  i8_skyRfPowerDiff = (context.e_curBand == RF_2G) ? (23 - context.stru_bandSwitchParam.i8_skyPower0):((23 - context.stru_bandSwitchParam.i8_skyPower1));

        agc_idx = 0;
        for( ; i < num; i++ )
        {
            agcsum += agc_level_min[i];
        }
        context.agclevel = (agcsum / num);

        if (context.agclevel >= context.u_bandSwitchParam->grd_bandSwitchParm.u8_BlockAgcThresh + i8_skyRfPowerDiff)
        {
            int ret = grd_GetDistAverage(&dist);
            if (ret == 1)    //only do swith when get distance value, else keep the original mode
            {
                e_signal = ((dist <= context.u_bandSwitchParam->grd_bandSwitchParm.u16_BlockDistThresh) ? RF_SIGNAL_BLOCK_MODE : RF_SIGNAL_UNBLOCK_MODE);
            }
            else
            {
                //if do not get the distance, keep mode
            }
        }
        else
        {
            e_signal = RF_SIGNAL_UNBLOCK_MODE;
        }

        if (context.rf_info.flag_signalBlock != e_signal)
        {
            context.rf_info.flag_signalBlock = e_signal;
            BB_WriteRegMask(PAGE2, GRD_SKY_IT_CH_SYNC, (context.rf_info.flag_signalBlock<<6), GRD_SKY_BLOCK_MODE_MASK);
        }

        DLOG_Info("Block: (0x%x %d) (%d %d) %d res=%d", context.agclevel, dist,
                      context.u_bandSwitchParam->grd_bandSwitchParm.u8_BlockAgcThresh + i8_skyRfPowerDiff,
                      context.u_bandSwitchParam->grd_bandSwitchParm.u16_BlockDistThresh, i8_skyRfPowerDiff, context.rf_info.flag_signalBlock);
    }
}

/**
 * @brief: notify sky to change band
 */
int32_t __attribute__ ((section(".h264"))) grd_notifyRfbandChange(ENUM_RF_BAND e_band, uint8_t u8_itCh, uint8_t u8_optCh, uint8_t changeNow)
{
    int32_t ret = -1;
    uint8_t eq_cnt_chg;

    if (0 == context.stru_bandChange.flag_bandchange && e_band != context.e_curBand)
    {
        eq_cnt_chg = (context.sync_cnt + STATUS_CHG_DELAY);

        context.stru_bandChange.e_toBand  = e_band;
        context.stru_bandChange.flag_bandchange = changeNow;
        context.stru_bandChange.u8_eqCntChg     = eq_cnt_chg;
        context.stru_bandChange.u8_ItCh   = u8_itCh;
        context.stru_bandChange.u8_optCh  = u8_optCh;
		context.stru_bandChange.chg_mcs = 1;
		if(context.qam_ldpc < 3) context.stru_bandChange.chg_mcs = 1;
		else  context.stru_bandChange.chg_mcs = context.qam_ldpc-1;
		/*
		if(context.st_bandMcsOpt.e_bandwidth==BW_20M)
		{
			context.stru_bandChange.chg_mcs = 1;
		}
		else
		{
			if(context.qam_ldpc < 3) context.stru_bandChange.chg_mcs = 1;
			else  context.stru_bandChange.chg_mcs = context.qam_ldpc-1;
		}
		*/
        BB_DtSendToBuf(DT_NUM_RF_BAND_CHANGE, (uint8_t *)&(context.stru_bandChange));

        DLOG_Critical("Band:%d chgBand %d %d", context.e_curBand, e_band, sizeof(context.stru_bandChange));
        ret = 0;
    }
    else
    {
        DLOG_Info("in changging");
    }

    return ret;
}

/**
 * @brief: down count, and change band
 */
int32_t __attribute__ ((section(".h264"))) grd_doRfbandChange( uint8_t *pu8_mainCh, uint8_t *pu8_optCh )
{   
    if (1 == context.stru_bandChange.flag_bandchange)
    {
        if (context.sync_cnt == (context.stru_bandChange.u8_eqCntChg))  //count to 0, time to change band
        {            
            context.stru_bandChange.flag_bandchange = 0;
            context.e_curBand = OTHER_BAND(context.e_curBand);  //switch to another band
			RF_GetFctFreqTable(context.st_bandMcsOpt.e_bandwidth);
            BB_set_RF_Band(BB_GRD_MODE, context.e_curBand);
			reset_sweep_table(context.e_curBand);
			
            BB_SweepChangeBand(context.e_curBand, context.stru_bandChange.u8_ItCh, context.stru_bandChange.u8_optCh);
            BB_set_sweepChannel();         
			rc_set_unlock_patten(0);
			//grd_rc_hopfreq();
            if ( pu8_mainCh )
            {
                *pu8_mainCh = context.stru_bandChange.u8_ItCh;
            }

            if ( pu8_optCh )
            {
                *pu8_optCh = context.stru_bandChange.u8_optCh;
            }

            //clear the result, re-start statistic again
            context.rf_info.u8_bandSelCnt = 0;

            context.pwr = BB_get_band_power(context.e_curBand);
            if(context.e_powerMode == RF_POWER_CLOSE)
            {
                //must sete trigger the power setting, BB requires
                BB_SetPowerCloseMode(context.e_curBand);
            }
            BB_set_power(context.e_curBand,context.pwr);
            context.u8_aocAdjustPwr = context.pwr;

			context.qam_ldpc=context.stru_bandChange.chg_mcs;
			grd_set_txmsg_mcs_change(context.st_bandMcsOpt.e_bandwidth, context.qam_ldpc);

            //stop send out the band switch message
            BB_DtStopSend(DT_NUM_RF_BAND_MODE);

            DLOG_Critical("Band switch: %d %d %d %d,pwr %d", context.sync_cnt, context.stru_bandChange.u8_eqCntChg, 
                                                  context.stru_bandChange.u8_ItCh, context.e_curBand,context.pwr);
            context.need_lna_open = 1;

            return 1;
        }
   }

    return 0;
}


void __attribute__ ((section(".h264"))) grd_requestRfBandSwitch(ENUM_RF_BAND e_band)
{
    uint8_t mainch = 0, optch = 0;

    //get best main channel
    BB_selectBestCh(e_band, SELECT_MAIN_OPT, &mainch, &optch, NULL, 1);    

    //start delay and notify sky to change
    grd_notifyRfbandChange(e_band, mainch, optch, 1);
}


/**
 * @brief: down count, change band and reset ground. From test, reset ground is necessary if band switch in unLock case
 */
int32_t __attribute__ ((section(".h264"))) grd_RfBandSelectChannelDoSwitch(void)
{
    extern void BB_grd_NotifyItFreqByCh(ENUM_RF_BAND band, uint8_t u8_ch);

    uint8_t main_ch = 0, opt_ch = 0;
    context.stru_bandChange.flag_bandchange = 0;
    context.e_curBand = OTHER_BAND(context.e_curBand);  //switch to another band

    BB_selectBestCh(context.e_curBand, SELECT_MAIN_OPT, &main_ch, &opt_ch, NULL, 0);
    BB_set_RF_Band(BB_GRD_MODE, context.e_curBand);
	RF_GetFctFreqTable(context.st_bandMcsOpt.e_bandwidth);
	reset_sweep_table(context.e_curBand);
    BB_SweepChangeBand(context.e_curBand, main_ch, opt_ch);
    BB_set_sweepChannel();  
    //clear the result, re-start statistic again
    context.rf_info.u8_bandSelCnt = 0;
    context.stru_bandChange.u8_ItCh   = main_ch;
    context.stru_bandChange.u8_optCh  = opt_ch;

    BB_set_ItFrqByCh(context.e_curBand, context.stru_bandChange.u8_ItCh);
    BB_grd_NotifyItFreqByCh(context.e_curBand, context.stru_bandChange.u8_ItCh);
    BB_softReset(BB_GRD_MODE);

    //DLOG_Info("Band switch: %d %d %d %d", context.sync_cnt, context.stru_bandChange.u8_eqCntChg, context.stru_bandChange.u8_ItCh, context.e_curBand);

    return 1;
}

void __attribute__ ((section(".h264"))) grd_SetSweepMode(uint8_t mode)
{
    if(mode <= 1)
    {
        sweep_with_filter = mode;
    }

}

void rf_pwr_statistics(){
	static int pwr_id=0;
	static int isfull=0;
	
	int i=0;
	int temp=0;

	if(stru_grdstatus.agc_value1==0 || stru_grdstatus.agc_value2==0 )return;

	uint8_t agca = BB_RssiOffset(stru_grdstatus.agc_value1);
    uint8_t agcb = BB_RssiOffset(stru_grdstatus.agc_value2);
	
	int value = (agca > agcb) ? agcb: agca;

	if(isfull){
		for(i=0;i<MAX_IT_PWR_STATICS-1;i++){
			context.rf_info.working_pwr[i] = context.rf_info.working_pwr[i+1];
			temp +=context.rf_info.working_pwr[i];
		}  
		context.rf_info.working_pwr[MAX_IT_PWR_STATICS-1]=value;
		temp +=value;
		context.rf_info.working_pwr_avrg=temp/MAX_IT_PWR_STATICS;
	}
	else{
		context.rf_info.working_pwr[pwr_id] = value;
		pwr_id++;
		if(pwr_id>=MAX_IT_PWR_STATICS){
			isfull=1;
		}
	}
	#if 0
		static int k=0;
		k++;
		if(k==100){
			k=0;
			DLOG_Critical("working pwr %d %d %d %d %d %d %d %d %d %d",
			context.rf_info.working_pwr[0],
			context.rf_info.working_pwr[1],
			context.rf_info.working_pwr[2],
			context.rf_info.working_pwr[3],
			context.rf_info.working_pwr[4],
			context.rf_info.working_pwr[5],
			context.rf_info.working_pwr[6],
			context.rf_info.working_pwr[7],
			context.rf_info.working_pwr[8],
			context.rf_info.working_pwr[9]
			);
			DLOG_Critical("pwr avrg= %d",context.rf_info.working_pwr_avrg);
		}
	#endif
	
}

uint8_t  BB_isSweepFull(void)
{
    return context.rf_info.u8_isFull;
}

void BB_resetSweepFull(void)
{
    context.rf_info.u8_isFull = 0;
}

uint8_t BB_get_cur_opt_ch(void)
{
    return context.rf_info.u8_optCh;
}


static uint8_t  grd_check_sweep_noise(uint8_t mustchg){
	STRU_RF_DATA list[MAX_RC_FRQ_SIZE]={0};
	STRU_RF_DATA listr[MAX_RC_FRQ_SIZE]={0};
	uint8_t i=0,j=0;
	int sweep_noise=0;
	int sweep_noise_fluct=0;
	//sort by value,find the low to high list, sort all sweep channel
	for(i=0;i<context.rf_info.sweep_freqsize;i++){
		sweep_noise = context.rf_info.sweep_pwr_avrg_value[i].value ;
		sweep_noise_fluct = context.rf_info.sweep_pwr_fluct_value[i].value;
		list[i].value=sweep_noise + sweep_noise_fluct;
		list[i].id=context.rf_info.sweep_pwr_avrg_value[i].id;
	}
	selectionSortBy(listr,context.rf_info.sweep_freqsize,list,1);
	for(i=0;i<context.rf_info.sweep_freqsize;i++){
		context.rf_info.sort_result_list[i].id=listr[i].id;
		context.rf_info.sort_result_list[i].value=listr[i].value;
	}
	 int current_sweep_now = context.rf_info.sweep_pwr_avrg_value[context.cur_IT_ch].value;
	 if((current_sweep_now - listr[0].value) < IT_CHANGE_THD) return 0;
	 return 1;
}

void   grd_gen_it_working_ch(uint8_t mode){
	uint8_t ret=0;
	ret = grd_check_sweep_noise(0);
	int oldch=context.cur_IT_ch;
	
	if(mode==0 || ret)
	{
	  context.cur_IT_ch=context.rf_info.sort_result_list[0].id;
	  BB_grd_NotifyItFreqByCh(context.e_curBand, context.cur_IT_ch);
	  context.dev_state = DELAY_14MS;
	  DLOG_Critical("mode=%d,it_work_sweep[%d]=%d,it_newsweep[%d]=%d",
	  mode,
	  oldch, 
	  context.rf_info.sweep_pwr_avrg_value[oldch].value,
	  context.cur_IT_ch,
	  context.rf_info.sort_result_list[0].value);
	}
}

void grd_auto_change_rf_bw(void){
	static uint32_t timegap=0;

	#if 1
	static int k=0;
	k++;
	if(k==200)
	{
		k=0;
		  DLOG_Critical("en_auto=%d,bw=%d,working_pwr_avrg=%d,thd_10=%d,thd_20=%d,skypA=%d,skypB=%d,skytx_pwr=%d",
		  context.rf_info.rf_bw_cg_info.en_auto,
		  context.st_bandMcsOpt.e_bandwidth,
		  context.rf_info.working_pwr_avrg,
		  context.rf_info.rf_bw_cg_info.thd_10,
		  context.rf_info.rf_bw_cg_info.thd_20,
		  context.rf_info.skyp0,
		  context.rf_info.skyp1,
		  context.rf_info.skytxp
		  );
	}
	#endif
	
	if(context.rf_info.rf_bw_cg_info.en_auto==0)return;
	if(context.rf_info.working_pwr_avrg==0)return;
	if((SysTicks_GetDiff(timegap,SysTicks_GetTickCount())) < 5000)return;

	if(context.rf_info.working_pwr_avrg < context.rf_info.rf_bw_cg_info.thd_20)
	{
		if(context.rf_bw.bw==BW_20M) return;
		context.rf_bw.en_flag=1;
		context.rf_bw.valid=1;
		context.rf_bw.bw=BW_20M;
		context.rf_bw.timeout_cnt=context.sync_cnt+STATUS_CHG_DELAY;
		if(context.qam_ldpc < 3) context.rf_bw.ldpc=1;
		else  context.rf_bw.ldpc=context.qam_ldpc-1;
	}
	else if(context.rf_info.working_pwr_avrg > context.rf_info.rf_bw_cg_info.thd_10)
	{
		if(context.rf_bw.bw==BW_10M) return;
		context.rf_bw.en_flag=1;
		context.rf_bw.valid=1;
		context.rf_bw.bw=BW_10M;
		context.rf_bw.timeout_cnt=context.sync_cnt+STATUS_CHG_DELAY;
	}
	timegap = SysTicks_GetTickCount();
	
}


