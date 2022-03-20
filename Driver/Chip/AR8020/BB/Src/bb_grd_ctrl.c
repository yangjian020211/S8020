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
#include "filter.h"
#include "systicks.h"
#include "rf_if.h"
#include "pwr_ctrl.h"

#define SNR_STATIC_START_VALUE          (100)
#define SNR_STATIC_UP_THRESHOD          (5)
#define SNR_STATIC_DOWN_THRESHOD        (2)

#define DEFAULT_DIST_ZERO_10M           (1592)
#define DEFAULT_DIST_ZERO_20M           (4386)
#define DEFAULT_DIST_JUDGE_THRELD       (20)
#define DEFAULT_DIST_JUDGE_CNT          (30)

STRU_GRD_STATUS stru_grdstatus;

static init_timer_st grd_timer2_5;
//static init_timer_st grd_timer2_1;

static init_timer_st grd_timer2_6;
static init_timer_st grd_timer2_7;

static STRU_DELAY_CNT rc_mod_chg_delay = {0, 0};
static STRU_DELAY_CNT filter_chg_delay = {0, 0};
static STRU_DELAY_CNT band_mode_chg_delay = {0, 0};
static uint8_t filter_chg_it_ch = 0;

static uint8_t  Timer1_Delay1_Cnt = 0;
static uint8_t  snr_static_count  = SNR_STATIC_START_VALUE;
static uint8_t  hop_count = 0;
static uint8_t  flag_itFreqskip = 0;
static uint8_t  flag_snrPostCheck;

static STRU_SkyStatus g_stru_skyStatus;

//uint8_t grd_rc_channel = 0;
static uint8_t non_lbt_opt_ch = 0xff;
static int print_grd=1;


STRU_CALC_DIST_DATA s_st_calcDistData = 
{
    .e_status = INVALID,
    .u32_calcDistValue = 0,
    .u32_calcDistZero = 0,
    .u32_cnt = 0,
    .u32_lockCnt = 0
};

extern STRU_PWR_CTRL *pstru_pwr_ctrl_cfg;

extern STRU_PURE_VT_INFO vt_info;
static init_timer_st timer2_3 = 
{
    .base_time_group = 2,
    .time_num = 3,
    .ctrl = TIME_ENABLE | USER_DEFINED,
};
static init_timer_st timer2_2 = 
{
    .base_time_group = 2,
    .time_num = 2,
    .ctrl = TIME_ENABLE | USER_DEFINED,
};

#ifdef JYDS
#pragma message("JYDS macro defined")
#endif

static void Grd_Timer2_3_Init(void);
static void Grd_TIM2_3_IRQHandler(uint32_t u32_vectorNum);
static void grd_Timer2_5_Init(void);
static void grd_Timer2_1_Init(void);
static void Grd_TIM2_5_IRQHandler(uint32_t u32_vectorNum);


static void BB_grd_uartDataHandler(void);

static uint32_t grd_calc_dist_get_avg_value(uint32_t *u32_dist);

static void grd_disable_enable_rc_frq_mask_func(uint8_t flag);

static void grd_init_rc_frq_mask_func(void);

static void grd_write_mask_code_to_sky(uint8_t enable, uint64_t *mask);

static void grd_set_ItFrq(ENUM_RF_BAND e_band, uint8_t ch);

static void BB_grd_OSDPlot(void);

static void grd_handle_IT_mode_cmd(ENUM_RUN_MODE mode);

static void grd_calc_dist_zero_calibration(void);

static void grd_set_calc_dist_zero_point(uint32_t value);

static uint8_t grd_calc_dist_judge_raw_data(ENUM_CALC_DIST_STATUS e_status, uint32_t data, uint32_t threld);

static void grd_calc_dist(void);

static void grd_handle_all_cmds(void);

static void grd_handle_all_rf_cmds(void);

static void Grd_Timer2_6_Init(void);

static void Grd_Timer2_7_Init(void);

static void grd_handle_CH_bandwitdh_cmd(ENUM_CH_BW bw);

static void grd_handle_MCS_mode_cmd(ENUM_RUN_MODE mode);

static void grd_handle_brc_cmd(uint8_t coderate);

static void grd_rc_hopfreq(void);

static void grd_rc_hopfreq600m(void);

static void reset_it_span_cnt(void);

static void grd_handle_IT_CH_cmd(uint8_t ch);

static void wimax_vsoc_tx_isr(uint32_t u32_vectorNum);

static void grd_ChgRcRate(uint8_t rate);

static void grd_getSignalStatus(void);

static int grd_aoc_prejudge_snr(void);

static int grd_aoc_prejudge_agc(void);

static int grd_aoc_judge(void);

static int grd_aoc_tx_msg(int power);

static void grd_aoc(void);

static int grd_handle_cmd_ack(uint8_t *arg);

static void BB_grd_NotifyItFreqByValue(uint8_t *u8_itFrq);

static void grd_vt_mode_proc(uint8_t value);

static int grd_lock_status(void *p);

static int grd_rc_mod_chg_process(void);

static int grd_filter_chg_process_600m(void);

static int grd_write_sync_cnt(void);

static void grd_BB_write_ItRegs(uint8_t *u8_it);

static void grd_cmd_select_filter(uint8_t value);

static int grd_sub_band_check(void);

static int grd_sub_band_notify(uint8_t value);

static void grd_sub_band_excute(uint8_t value);

static int grd_sub_band_run(void);

static void reset_sub_band_notice(void);

static void grd_enter_slave_mode(uint8_t *rcid,uint8_t *vtid);

static void grd_limit_dist_process(void);

//void grd_init_rc_skip_patten(void);
static void grd_do_rcRate(void);
static void do_debug_process(void);

static void grd_do_rc_patten(void);
static void grd_do_rf_bw(void);
static void grd_handle_rc_patten_cmd(uint8_t *arg);
static void gprtit(uint32_t *str,int i);

static void grd_plot_itsweep(){
#if 1
int i=0;
int j=0;
 if(print_grd==0x01)
 {
		DLOG_Critical("rc_error=%d,rc_agca=%d,rc_agcb=%d",(100-g_stru_skyStatus.u8_rcCrcLockCnt),g_stru_skyStatus.u8_skyagc1,g_stru_skyStatus.u8_skyagc2);
		DLOG_Critical("current rc work patten len=%d",context.rf_info.rc_ch_working_patten_size);
		uint32_t str2[50]={0};
		for(j=0;j<context.rf_info.rc_ch_working_patten_size;j++)str2[j]=BB_GetRcFrqByCh(context.rf_info.rc_ch_working_patten[j]);
		gprtit(str2, 0);
		print_grd=0;
		
 }
	
  else if(print_grd==0x02)
  {
		uint32_t str[50]={0};
		int j=0;
		DLOG_Critical("grd sweep table");
		for(i=0;i<SWEEP_FREQ_BLOCK_ROWS;i++)
		{
			for(j=0;j<context.rf_info.sweep_freqsize;j++)str[j]=context.rf_info.sweep_pwr_table[i][j].value;
				gprtit(str,i);
		}
		DLOG_Critical("grd sweep avrg");
		for(j=0;j<context.rf_info.sweep_freqsize;j++)str[j]=context.rf_info.sweep_pwr_avrg_value[j].value;
				gprtit(str,0);
				
		DLOG_Critical("grd sweep fluct");
		for(j=0;j<context.rf_info.sweep_freqsize;j++)str[j]=context.rf_info.sweep_pwr_fluct_value[j].value;
				gprtit(str,0);

		DLOG_Critical("grd sort avrg ");
		for(j=0;j<context.rf_info.sweep_freqsize;j++)str[j]=context.rf_info.sort_result_list[j].value;
				gprtit(str,0);

		DLOG_Critical("rc work patten ");
		uint32_t str2[50]={0};
		for(j=0;j<context.rf_info.rc_ch_working_patten_size;j++)str2[j]=BB_GetRcFrqByCh(context.rf_info.rc_ch_working_patten[j]);
				gprtit(str2, 0);

		print_grd = 0x00;
   }

	
#endif

}

void BB_GRD_start(void)
{
    context.dev_state = INIT_DATA;
    context.qam_ldpc = context.u8_bbStartMcs;
    context.flag_mrc = 0;
	
    context.st_bandMcsOpt.e_rfbandMode = MANUAL; //grd defualt manual
    context.flag_in_upgrade=0;
	BB_SetTrxMode(BB_RECEIVE_ONLY_MODE); // receive only

    context.stru_bandSwitchParam.u8_bandSwitchAfterUnlock = 0;

    //BB_WriteRegMask(PAGE2, GRD_SKY_RC_CH_SYNC, 0x80, GRD_SKY_RC_RSV_BIT_MASK);
    //BB_WriteRegMask(PAGE2, GRD_SKY_RC_CH_SYNC, (context.stru_bandSwitchParam.u8_bandSwitchAfterUnlock<<6), GRD_SKY_BAND_SWITCH_MASK);

    grd_set_txmsg_mcs_change(context.st_bandMcsOpt.e_bandwidth, context.qam_ldpc);

    Grd_Timer2_6_Init();
    Grd_Timer2_7_Init();
    grd_Timer2_5_Init();
    //grd_Timer2_1_Init();

    BB_set_Rcfrq(context.e_curBand, 0);

    if(context.uplink_qam_mode != 0 && context.uplink_qam_mode < 4 ){
        grd_ChgRcRate(context.uplink_qam_mode);
        context.RcChgRate.default_qam_value = context.uplink_qam_mode;
        DLOG_Warning("up %d",context.uplink_qam_mode);
    }
    //do not notify sky until sweep end and get the best channel from sweep result
    context.cur_IT_ch = DEFAULT_IT_CH;
    grd_set_ItFrq(context.e_curBand, 1);
    BB_grd_NotifyItFreqByCh(context.e_curBand, 1);

    BB_SweepStart(context.st_bandMcsOpt.e_bandsupport, context.st_bandMcsOpt.e_bandwidth);
	BB_Sweep_updateCh(context.e_curBand, context.cur_IT_ch );

    grd_init_rc_frq_mask_func();

    BB_ComInit(grd_lock_status,0); 
    BB_GetDevInfo();

    grd_SetRCId((uint8_t *)context.hashRcid);

    context.realtime_mode = 0;
    BB_WriteRegMask(PAGE2, MCS_INDEX_MODE,context.realtime_mode ? 0x80 : 0x00,0x80);
    BB_WriteRegMask(PAGE2, MCS_INDEX_MODE,context.enable_non_lbt ? 0x40 : 0x00,0x40);

    reg_IrqHandle(BB_TX_ENABLE_VECTOR_NUM, wimax_vsoc_tx_isr, NULL);
    NVIC_SetPriority(BB_TX_ENABLE_VECTOR_NUM,NVIC_EncodePriority(NVIC_PRIORITYGROUP_5,INTR_NVIC_PRIORITY_BB_TX,0));
    NVIC_EnableIRQ(BB_TX_ENABLE_VECTOR_NUM);
    PWR_CTRL_Init(pstru_pwr_ctrl_cfg);
	
	//init_mcs();
	
}
#if 1
static void gptf(uint32_t *str,int i){
#if 0
	switch(str[1])
	{
		case 0xaa:DLOG_Critical("corse_sweep_pwr");break;
		case 0x01:DLOG_Critical("corse_avrg_sweep_pwr");break;
		case 0x02:DLOG_Critical("corse_fluct_sweep_pwr");break;
		case 0x03:DLOG_Critical("corse_sortid_sweep_pwr_avrg");break;
		case 0x04:DLOG_Critical("corse_sortvalue_sweep_pwr_avrg");break;
		case 0x05:DLOG_Critical("working_avrg_id");break;
		case 0xbb:DLOG_Critical("fine_selected_id_to_sweep");break;
		case 0x06:DLOG_Critical("working_sort_sweep_avrg");break;
		case 0x55:DLOG_Critical("fine_sweep_pwr");break;
		case 0x07:DLOG_Critical("fine_avrg_sweep_pwr");break;
		case 0x08:DLOG_Critical("fine_fluct_sweep_pwr");break;
		case 0x09:DLOG_Critical("fine_sweepid");break;
		case 0x0a:DLOG_Critical("fine_sweep_avrg_value");break;
		case 0x0b:DLOG_Critical("fine_sortid");break;
		case 0x0c:DLOG_Critical("fine_sortid_avrg_pwr_value");break;
	}
#endif

	DLOG_Critical("len=%d type=%2x %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d",
					str[0],str[1],str[2],str[3],str[4],str[5],str[6],str[7],str[8],str[9],
					str[10],str[11],str[12],str[13],str[14],str[15],str[16],str[17],str[18],str[19],
					str[20],str[21],str[22],str[23],str[24],str[25],str[26],str[27],str[28],str[29],
					str[30],str[31],str[32],str[33],str[34],str[35],str[36],str[37],str[38],str[39],str[40],str[41]);
}

static void gprtit(uint32_t *str,int i){
	
	DLOG_Critical(" %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d",
					str[0],str[1],str[2],str[3],str[4],str[5],str[6],str[7],str[8],str[9],
					str[10],str[11],str[12],str[13],str[14],str[15],str[16],str[17],str[18],str[19],
					str[20],str[21],str[22],str[23],str[24],str[25],str[26],str[27],str[28],str[29],
					str[30],str[31],str[32],str[33],str[34],str[35],str[36],str[37],str[38],str[39],str[40],str[41]);
	
}
#endif

static void BB_grd_uartDataHandler(void)
{
    uint8_t  data[128];
    uint8_t  pid;
    uint16_t len = sizeof(data);
    int rcvLen = 0;
    uint8_t proCnt = 5;
    
    while(proCnt--)
    {
        rcvLen = BB_Session0RecvMsg(data, len);
        if (rcvLen > 0)
        {
            pid = data[0];
            if (DT_NUM_SKY_LOCK_STATUS == pid)
            {
                g_stru_skyStatus.u8_rcCrcLockCnt = data[1];
                g_stru_skyStatus.u8_rcNrLockCnt  = data[2];
            }
            else if (DT_NUM_RC_MASK_CODE == pid)
            {
            }
            else if (DT_NUM_SKY_AGC_STATUS == pid)
            {
                STRU_skyAgc *agc = (STRU_skyAgc *)(data+1);
                g_stru_skyStatus.u8_skyagc1 = agc->u8_skyagc1;
                g_stru_skyStatus.u8_skyagc2 = agc->u8_skyagc2;
				context.rf_info.skyp0=agc->u8_skyagc1;
				context.rf_info.skyp1=agc->u8_skyagc2;
                g_stru_skyStatus.snr        = agc->snr;
				g_stru_skyStatus.tx_pwr		= agc->u8_tx_pwr;
				context.rf_info.skytxp		= agc->u8_tx_pwr;
            }
            else if(DT_NUM_RC_ID_SYNC == pid)
            {
                if (BB_grd_isSearching())
                {
                    STRU_SyncrcId *rcid = (STRU_SyncrcId *)(data +1);
                    
                    STRU_SysEvent_DEV_BB_STATUS status = {
                        .pid = BB_GET_RCID
                    };
                    memcpy((void *)status.rcid, (void *)rcid->u8_skyRcIdArray, RC_ID_SIZE);
                    SYS_EVENT_NotifyInterCore(SYS_EVENT_ID_BB_EVENT, (void *)&status);
                    BB_grd_ackSkyRcIdSendVtId(rcid->u8_skyRcIdArray);
                }
            }
            else if (DT_NUM_REMOTE_EVENT == pid)
            {
                uint32_t eventId;
                memcpy((uint8_t *)(&eventId), data+1, 4);
                SYS_EVENT_NotifyInterCore(eventId, data +5);
            }
            else if (DT_NUM_CH_MAP == pid)
            {
                memcpy((uint8_t *)context.rc_ch_map, data+1, MAX_RC_FRQ_SIZE);
                BB_Session0SendMsg(DT_NUM_CH_MAP, (uint8_t *)context.rc_ch_map, MAX_RC_FRQ_SIZE);
            }
            else if(DT_NUM_RF_BAND_CHANGE == pid)
            {
                uint8_t mainch = 0, optch = 0;
                STRU_BandChange *p = (STRU_BandChange *)(&data[1]);

                BB_selectBestCh(p->e_toBand, SELECT_MAIN_OPT, &mainch, &optch, NULL, 0 );   //get best main channel
                grd_notifyRfbandChange(p->e_toBand, mainch, optch, 1);                      //start delay and notify sky to change
            }
            else if(pid == DT_NUM_BANDSWITCH_ENABLE)
            {
                STRU_BAND_SWITCH_PARAM *param = (STRU_BAND_SWITCH_PARAM *)(data + 1);
                context.stru_bandSwitchParam.u8_bandSwitchAfterUnlock = param->u8_bandSwitchAfterUnlock;
                context.stru_bandSwitchParam.i8_skyPower0 = param->i8_skyPower0;
                context.stru_bandSwitchParam.i8_skyPower1 = param->i8_skyPower1;
            }
			else if(pid == DT_NUM_SKY_SWEEP_NOISE)
			{	
				
				uint32_t buff[50]={0};
				int i=0,j=0;	
				uint32_t type = data[2];
				uint32_t len = data[1];
				
				for(i=0;i<50;i++)buff[i]=0;
				
				if(len > 50) return;
				if(len < 2) return;
				if(type==0xaa || type==0x55 || type==0x01 || type==0x04 || type==0x06 || type==0x07 || type==0x0a || type==0x0c || type==0x0e )
				{
					for(i=0;i<len-2;i++) buff[i+2]=0-data[i+3];
					buff[0]=len;
					buff[1]=type;
					gptf(buff,0);
				}
				else if(type!=0x0d)
				{
					for(i=0;i<len-2;i++) buff[i+2]=data[i+3];
					buff[0]=len;
					buff[1]=type;
					gptf(buff,0);
				}
				if(type==0x0d){
					
					print_grd =0x01;
				}
				if(type==0x0b)
				{
					DLOG_Warning("new patten len= %d",len-2);
					for(i=0;i<len-2;i++)buff[i+2]=BB_GetRcFrqByCh(data[i+3]);
					buff[0]=len;
					buff[1]=0x0e;
					gptf(buff,0);
					
				}
			}
			else if(pid == DT_NUM_SKY_RC_PATTEN)
			{
				grd_handle_rc_patten_cmd(&data[1]);
			}
			else if(DT_NUM_GRD_RC_CHPATTEN ==pid)
			{
				context.rcChgPatten.en_flag_grd=0;
				context.rcChgPatten.valid_grd=0;
				//DLOG_Warning("grd get ack from grd_rc_chgpatten!");
			}
			else if(DT_NUM_AUTO_CH_BW_CHANGE==pid){
				context.rf_bw.valid=0;
			}
            grd_handle_cmd_ack(data);
        }
        else
        {
            break;
        }
    }
}


//---------------IT grd hop change--------------------------------

uint32_t it_span_cnt = 0;
void reset_it_span_cnt(void)
{
    it_span_cnt = 0;
}
void sub_band_rc_switch(void)
{
    uint32_t feq_num;
    
    feq_num = BB_GetSubBandEndCH(context.e_curBand,context.st_bandMcsOpt.e_bandwidth,0) -
        BB_GetSubBandStartCH(context.e_curBand,context.st_bandMcsOpt.e_bandwidth,0);

    uint32_t timer = (feq_num + 1) * 14*BB_GetItFrqNum(context.e_curBand)*4;
    
    if(SysTicks_GetDiff(context.sub_band_main_opt_ch_time, SysTicks_GetTickCount()) > timer)
    {
        context.sub_band_main_opt_ch_time = SysTicks_GetTickCount();
        DLOG_Warning("sub band %d", context.sub_band_value);
        context.sub_band_value = context.sub_band_value == context.sub_band_main_ch ? context.sub_band_opt_ch : context.sub_band_main_ch;
        DLOG_Warning("sub band to %d", context.sub_band_value);
        context.sub_rc_start = BB_GetSubBandStartCH(context.e_curBand,context.st_bandMcsOpt.e_bandwidth,context.sub_band_value);
        context.sub_rc_end = BB_GetSubBandEndCH(context.e_curBand,context.st_bandMcsOpt.e_bandwidth,context.sub_band_value);
        //grd_rc_channel = context.sub_rc_start;
		context.grd_rc_channel = context.sub_rc_start;
    }

}

void grd_fec_judge(void)
{
    if( context.dev_state == INIT_DATA )
    {
        context.locked = grd_is_bb_fec_lock();
        if (!context.locked )
        {
            if (context.fec_unlock_cnt ++ > 20)
            {
                if( context.itHopMode == AUTO && context.slave_flag == 0)
                {
                    uint8_t mainch, optch,ret;

                    #ifdef RF_9363
					//if(RF_600M == context.e_curBand)
					{
						mainch = BB_GetItChInOneFilter(context.cur_IT_ch);
                        ret = 1;
					}
                    #endif

                    #ifdef RF_8003X
					//else
					{
						ret = BB_selectBestCh(context.e_curBand, SELECT_MAIN_OPT, &mainch, &optch, (uint8_t *)NULL, 0);
					}
                    #endif
					
                    if ((ret))
                    {
                        context.vtFreqTime[context.cur_IT_ch] = SysTicks_GetTickCount();//save last main ch time value
                        context.cur_IT_ch  = mainch;
                        BB_Sweep_updateCh(context.e_curBand, mainch);
                        grd_set_ItFrq(context.e_curBand, mainch);
                        BB_grd_NotifyItFreqByCh(context.e_curBand, context.cur_IT_ch);
						#ifdef RFSUB_BAND
                        if(context.freq_band_mode == SUB_BAND)
                        {
                            sub_band_rc_switch();
                        }
						#endif
                    }                  
                }
                context.fec_unlock_cnt = 0;
            }
        }
        else
        {
            context.dev_state = CHECK_FEC_LOCK;
        }
    }
    else if(context.dev_state == CHECK_FEC_LOCK || context.dev_state == FEC_LOCK)
    {
        context.locked = grd_is_bb_fec_lock();
		if(context.dev_state == CHECK_FEC_LOCK)
		{
			init_mcs();
		}
        if(context.locked)
        {
            context.dev_state = FEC_LOCK;
            context.fec_unlock_cnt = 0;
			context.it_fec_unlock_cnt=0;
			#ifdef RFSUB_BAND
            if(context.freq_band_mode == SUB_BAND)
            {
                grd_sub_band_check();
            }
			#endif
            context.sub_bb_locked = 1;
			
        }
        else
        {
            if(context.slave_flag)
            {
                context.dev_state = CHECK_FEC_LOCK;
            }
           
            context.fec_unlock_cnt++;
			context.it_fec_unlock_cnt++;
			if(context.it_fec_unlock_cnt > context.rf_info.rc_unlock_timeout_cnt){
				rc_set_unlock_patten(1);
				context.it_fec_unlock_cnt=0;
			}
            if(context.fec_unlock_cnt >= context.rf_info.it_unlock_timeout_cnt)
            {
            	context.fec_unlock_cnt = 0;
                context.dev_state = CHECK_FEC_LOCK;
                DLOG_Critical("fec_unlock_cnt > %d",context.rf_info.it_unlock_timeout_cnt);
				#ifdef RFSUB_BAND
                if(context.freq_band_mode == SUB_BAND)
                {
                    if(BB_isSweepFull() && context.sub_bb_locked && context.slave_flag == 0)
                    {
                        context.sub_bb_locked = 0;
                        BB_SetTrxMode(BB_RECEIVE_ONLY_MODE); // receive only
                        context.vtFreqTime[context.cur_IT_ch] = SysTicks_GetTickCount();//save last main ch time value
                        context.cur_IT_ch = 0;
                        grd_set_ItFrq(context.e_curBand, context.cur_IT_ch);
                        BB_grd_NotifyItFreqByCh(context.e_curBand, context.cur_IT_ch);
                        
                        BB_SweepStart(context.st_bandMcsOpt.e_bandsupport, context.st_bandMcsOpt.e_bandwidth);
                        BB_Sweep_updateCh(context.e_curBand, context.cur_IT_ch );
                        BB_resetSweepFull();
                        DLOG_Warning("unlock, sub band set only rx");
                    }

                }
				#endif
                uint8_t it_ch_ok;
                if( context.itHopMode == AUTO && context.slave_flag == 0)
                {
                    uint8_t mainch, optch;

                    #ifdef RF_9363
					//if(RF_600M == context.e_curBand)
					{
						mainch = BB_GetItChInOneFilter(context.cur_IT_ch);
                        it_ch_ok = 1;
					}
                    #endif

                    #ifdef RF_8003X
					//else
					{
						it_ch_ok = BB_selectBestCh(context.e_curBand, SELECT_MAIN_OPT, &mainch, &optch, NULL, 0);
						//if VT is unlock, reset map
						//DLOG_Info("unlock: reset RC map");
						//BB_ResetRcMap();
					}
                    #endif
                    
                    if(it_ch_ok)
                    {
                        context.vtFreqTime[context.cur_IT_ch] = SysTicks_GetTickCount();//save last main ch time value
                        context.cur_IT_ch  = mainch;
                        BB_Sweep_updateCh(context.e_curBand, mainch);
                        grd_set_ItFrq(context.e_curBand, mainch);
                        BB_grd_NotifyItFreqByCh(context.e_curBand, context.cur_IT_ch);
                        //DLOG_Info("unlock: select channel %d %d", mainch, optch);
                        #ifdef RFSUB_BAND
                        if(context.freq_band_mode == SUB_BAND)
                        {
                            sub_band_rc_switch();
                        }
						#endif
                    }

                }

                

                if(context.qam_skip_mode == AUTO && context.qam_ldpc > context.u8_bbStartMcs)
                {
                    context.qam_ldpc = context.u8_bbStartMcs;
                    grd_set_txmsg_mcs_change(context.st_bandMcsOpt.e_bandwidth, context.qam_ldpc);
                }
            }
        }

    }
    else if(context.dev_state == DELAY_14MS)
    {
        BB_set_ItFrqByCh(context.e_curBand, context.cur_IT_ch);
        reset_it_span_cnt();
        context.dev_state = CHECK_FEC_LOCK;
    }
	//FIND_SAME_DEV is delay 14 ms, not define or add new dev_state value
    else if(context.dev_state == FIND_SAME_DEV)// command set it ch, need add 14ms delay to sync it ch switch with sky
    {
        context.dev_state = DELAY_14MS;
    }
}

const uint16_t snr_skip_threshold[] = { 0x23,    //bpsk 1/2
                                        0x2d,    //bpsk 1/2
                                        0x6c,    //qpsk 1/2
                                        0x181,   //16QAM 1/2
                                        0x492,   //64QAM 1/2
                                        0x52c};  //64QAM 2/3
uint8_t is_vt_time_satisfy(uint8_t optch)
{
    uint8_t i,ch,get;

    if(SysTicks_GetDiff(context.vtFreqTime[optch],SysTicks_GetTickCount()) < NON_LBT_VT_CH_SKIP_INTERVAL)
    {
        get = 0;
        for(i = 0;i < BB_GetItFrqNum(context.e_curBand);i++)
        {
            ch = (optch + 1 + i) % BB_GetItFrqNum(context.e_curBand);
            if( SysTicks_GetDiff(context.vtFreqTime[ch],SysTicks_GetTickCount()) > NON_LBT_VT_CH_SKIP_INTERVAL && (ch != context.cur_IT_ch))
            {
                get = 1;
                break;
            }
        }
        
        if(!get){
            //DLOG_Warning("not get obj VT-ch");
        }
        else
        {
            //DLOG_Warning("optch %d -> ch %d",optch,ch);
            optch = ch;
        }
    }

    return optch;
}
int grd_freq_skip_pre_judge(void)
{
    uint8_t flag = 0;
    int16_t aver, fluct;
    uint8_t bestch, optch,retrans;    
    
    if (context.dev_state != FEC_LOCK)
    {
        return 0;
    }
    
    if (it_span_cnt < 16)
    {
        it_span_cnt ++;
        return 0;
    }

    uint8_t Harqcnt  = ((context.u8_harqcnt_lock & 0xF0) >> 4);//sky retrans cnt
    uint8_t Harqcnt1 = ((BB_ReadReg(PAGE2, 0xd1)& 0xF0) >> 4);// grd request retrans cnt
    //if(Harqcnt > 0)
    //{
    //    DLOG_Info("Harq %d:%d %d snr:%x %x %d ret=%d", Harqcnt, Harqcnt1, context.cycle_count, grd_get_it_snr(), 
    //                                        (((uint16_t)BB_ReadReg(PAGE2, 0xc2)) << 8) | BB_ReadReg(PAGE2, 0xc3),
    //                                        (((uint16_t)BB_ReadReg(PAGE2, 0xd7)) << 8) | BB_ReadReg(PAGE2, 0xd8));
    //}
    retrans = 0;

    if (Harqcnt >= 7 && Harqcnt1 >= 7)      //check LDPC error
    {
        //DLOG_Info("Harq: %d %d", Harqcnt, Harqcnt1);
        flag = 0;
        retrans = 1;
    }
    else if (Harqcnt >= 3 && Harqcnt1 >= 3) //check snr.
    {
        flag = grd_check_piecewiseSnrPass(1, snr_skip_threshold[context.qam_ldpc]);
        if ( 1 == flag ) //snr pass
        {
            return 0;
        }
    }
    else
    {
        return 0;
    }

    #ifdef RF_9363
	//if(RF_600M == context.e_curBand)
	{
		optch = BB_GetItChInOneFilter(context.cur_IT_ch);
	}
    #endif

    #ifdef RF_8003X
	//else
	{
		optch = get_opt_channel();
	}
    #endif
    
    if (BB_CompareCh1Ch2ByPowerAver(context.e_curBand, optch, context.cur_IT_ch, CMP_POWER_AVR_LEVEL))
    {
        #ifndef NON_LBT_RELEASE
        //opt ch better than cur work ch
        if(context.enable_non_lbt)
        {
            optch = is_vt_time_satisfy(optch);
        }
        #endif
        
        if( 0 == flag ) //already Fail
        {
            reset_it_span_cnt( );
            context.vtFreqTime[context.cur_IT_ch] = SysTicks_GetTickCount();//save last main ch time value
            //DLOG_Warning("snr: skip it %d->%d",context.cur_IT_ch,optch);////2/
            context.cur_IT_ch  = optch;
            BB_Sweep_updateCh(context.e_curBand, context.cur_IT_ch );

            BB_grd_NotifyItFreqByCh(context.e_curBand, context.cur_IT_ch);

            //DLOG_Info("Set Ch0:%d %d %d %x\n",
            //context.cur_IT_ch, context.cycle_count, ((BB_ReadReg(PAGE2, FEC_5_RD)& 0xF0) >> 4), grd_get_it_snr());
            context.dev_state = DELAY_14MS;

			//grd_gen_it_working_ch(0);
			
            return 0;
        }
        else
        {
            context.next_IT_ch = optch;
            return 1;  //need to check in the next 1.25ms
        }
    }
    else if(context.enable_non_lbt)
    {
        if(retrans)
        {
            #ifndef NON_LBT_RELEASE
            if(context.enable_non_lbt)
            {
                optch = is_vt_time_satisfy(optch);
            }
            #endif

            reset_it_span_cnt( );
            context.vtFreqTime[context.cur_IT_ch] = SysTicks_GetTickCount();//save last main ch time value
            //DLOG_Warning("byretans-7: skip it %d->%d",context.cur_IT_ch,optch);////////1
            context.cur_IT_ch  = optch;
            BB_Sweep_updateCh(context.e_curBand, context.cur_IT_ch );

            BB_grd_NotifyItFreqByCh(context.e_curBand, context.cur_IT_ch);
            
            
            DLOG_Warning("Set Ch0:%d %d %d %x\n",
                       context.cur_IT_ch, context.cycle_count, ((BB_ReadReg(PAGE2, FEC_5_RD)& 0xF0) >> 4), grd_get_it_snr());
            context.dev_state = DELAY_14MS;
            return 0;

        }
    }
    else
    {
        return 0;
    }
}


void grd_freq_skip_post_judge(void)
{
    if(context.dev_state != FEC_LOCK )
    {
        return;
    }

    if ( 0 == grd_check_piecewiseSnrPass( 0, snr_skip_threshold[context.qam_ldpc] ) ) //Fail, need to hop
    {
        if( context.cur_IT_ch != context.next_IT_ch )
        {
            reset_it_span_cnt( );
            context.vtFreqTime[context.cur_IT_ch] = SysTicks_GetTickCount();//save last main ch time value
            //DLOG_Warning("skip1 it %d->%d",context.cur_IT_ch,context.next_IT_ch);
            context.cur_IT_ch = context.next_IT_ch;
            BB_Sweep_updateCh(context.e_curBand, context.cur_IT_ch );
            BB_grd_NotifyItFreqByCh(context.e_curBand, context.cur_IT_ch);
            DLOG_Warning("Set Ch1:%d %d %d %x\n", context.cur_IT_ch, context.cycle_count,
                                             ((BB_ReadReg(PAGE2, FEC_5_RD)& 0xF0) >> 4), grd_get_it_snr());
        }

        context.dev_state = DELAY_14MS;        
    }
}

void grd_ackSkyVtSkip(void)
{
    //static uint8_t seq = 0;
    uint8_t tmp,skySelVtCh = BB_ReadReg(PAGE2, GRD_NON_LBT_VT_CH);
    static uint8_t last_ch = 0xff;

    tmp = skySelVtCh & 0x3f;
    if(context.locked && (((skySelVtCh & 0x80)) == 0X80) && (last_ch != tmp))
    {
        //seq = ((skySelVtCh & 0xc0) >> 6) ;
        last_ch = tmp;
        if(tmp < BB_GetItFrqNum(context.e_curBand))
        {
            context.sky_sel_vt_ch = tmp;
            //DLOG_Warning("g-cur_ch %d, sky-sel-ch %d",context.cur_IT_ch,context.sky_sel_vt_ch);
            //reset_it_span_cnt( );
            //DLOG_Warning("skip it %d->%d",context.cur_IT_ch,context.sky_sel_vt_ch);
            context.vtFreqTime[context.cur_IT_ch] = SysTicks_GetTickCount();//save last main ch time value

            context.cur_IT_ch  = context.sky_sel_vt_ch;
            BB_Sweep_updateCh(context.e_curBand, context.cur_IT_ch );
            
            BB_grd_NotifyItFreqByCh(context.e_curBand, context.cur_IT_ch);
            BB_set_ItFrqByCh(context.e_curBand, context.cur_IT_ch);
            reset_it_span_cnt();
            context.dev_state = CHECK_FEC_LOCK;
        }
        else
        {
            //DLOG_Error("sky selvt %d %d",context.e_curBand,tmp);
        }
        //context.dev_state = DELAY_14MS;    
        //BB_set_ItFrqByCh(context.e_curBand, context.cur_IT_ch);
    }

}

uint8_t grd_is_bb_fec_lock(void)
{
    uint8_t vt_lock = (BB_ReadReg(PAGE2, FEC_5_RD) & 0x01);
    if (vt_lock)  //if vt unlock,not need to check vt idmatch
    {
        vt_lock |= (BB_grd_checkVtIdMatch() << 1);
    }

    if(vt_lock == 0x01)
    {
        if(context.slave_flag)
        {
            STRU_SysEvent_DEV_BB_STATUS lockEvent ={
                .pid        = BB_GOT_ERR_CONNNECT,
                .lockstatus = 1,//stru_skystatus.flag_errorConnect,
            };
            SYS_EVENT_NotifyInterCore(SYS_EVENT_ID_BB_EVENT, (void *)&lockEvent);
        }

        BB_softReset(BB_GRD_MODE);
        vt_lock = 0;
        DLOG_Info("reset bb,vtid not match");
    }

    if (context.locked != vt_lock)
    {
        context.locked = vt_lock;
        STRU_SysEvent_DEV_BB_STATUS lockEvent = 
        {
            .pid        = BB_LOCK_STATUS,
            .lockstatus = vt_lock,
        };

        SYS_EVENT_NotifyInterCore(SYS_EVENT_ID_BB_EVENT, (void *)&lockEvent);
        DLOG_Warning("Lock=0x%x,it=%d", vt_lock,context.cur_IT_ch);
    }

    return vt_lock;
}

//---------------QAM change--------------------------------
ENUM_BB_QAM Grd_get_QAM(void)
{
    static uint8_t iqam = 0xff;

    ENUM_BB_QAM qam = (ENUM_BB_QAM)(BB_ReadReg(PAGE2, GRD_FEC_QAM_CR_TLV) & 0x03);
    if(iqam != qam)
    {
        iqam = qam;
        //DLOG_Info("-QAM:%d ",qam);
    }

    return qam;
}
static void grd_command_lna_switch(uint8_t value)
{
    if(value == 0)
    {
        context.st_mimo_mode.enum_lna_mode = LNA_BYPASS;

        if(context.e_curBand == RF_5G)
        {
            return;
        }
        BB_bypass_lna();
        context.lna_status = BYPASS_LNA;
        BB_Lna_reset();
    }
    else if(value == 1)
    {
        context.st_mimo_mode.enum_lna_mode = LNA_AUTO;
        
        if(context.e_curBand == RF_5G)
        {
            return;
        }

        BB_Lna_reset();
        
    }
    else if(value == 4)
    {
        context.st_mimo_mode.enum_lna_mode = LNA_OPEN;

        if(context.e_curBand == RF_5G)
        {
            return;
        }

        BB_open_lna();
        context.lna_status = OPEN_LNA;
        BB_Lna_reset();
        
    }
    else
    {
        DLOG_Error("%d failed",value);
    }
}

static void grd_lna_switch(void)
{
    ENUM_LNA_STATUS status;

    if(context.enable_non_lbt)
    {
        if(context.lna_status != OPEN_LNA)
        {
            BB_open_lna();
            context.lna_status = OPEN_LNA;
        }

        return;
    }

    if(context.command_lna_switch_flag)
    {
        context.command_lna_switch_flag = 0;
        grd_command_lna_switch(context.command_lna_switch_value);
    }
    
    if(context.need_lna_open)//grd ,band switch 
    {
        context.need_lna_open = 0;

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
                BB_bypass_lna();
                context.lna_status = BYPASS_LNA;
            }
            else if(context.st_mimo_mode.enum_lna_mode == LNA_OPEN)
            {
                BB_open_lna();
                context.lna_status = OPEN_LNA;
            }
            else if(context.st_mimo_mode.enum_lna_mode == LNA_AUTO)
            {
    	
                if(context.swp_bypass == 1)
                {
                    BB_bypass_lna();
                    context.lna_status = BYPASS_LNA;
                }
                else if(context.swp_bypass == 2)
                {
                    BB_open_lna();
                    context.lna_status = OPEN_LNA;
                }               	  
            }
        }
        BB_Lna_reset();
        
        return;
    }
    
    if(context.st_mimo_mode.enum_lna_mode != LNA_AUTO)
    {
        return ;
    }

    if(context.e_curBand == RF_5G)
    {
        return;
    }

    if(context.locked)
    {
        BB_Lna_AddAgc(stru_grdstatus.agc_value1,stru_grdstatus.agc_value2);
    }
  /*else
    {
        if(context.lna_status != OPEN_LNA)
        {
            BB_open_lna();
            context.lna_status = OPEN_LNA;
            BB_Lna_reset();
        }
        return;
    }*/

    status  = BB_Lna_isNeedSwitch(context.e_curBand);
    if(status != INVALID_LNA)
    {
        if(status == BYPASS_LNA && context.lna_status == OPEN_LNA)
        {
            BB_bypass_lna();
            context.lna_status = BYPASS_LNA;
            BB_Lna_reset();
        }
		else if(status == OPEN_LNA && context.lna_status == BYPASS_LNA && context.swp_bypass == 2 )
        //else if(status == OPEN_LNA && context.lna_status == BYPASS_LNA)
        {
            BB_open_lna();
            context.lna_status = OPEN_LNA;
            BB_Lna_reset();
        }
    }

  /*if(context.qam_mode == MOD_16QAM || context.qam_mode == MOD_64QAM)
    {
        if(context.lna_status != BYPASS_LNA)
        {
            BB_bypass_lna();
            context.lna_status = BYPASS_LNA;
            BB_Lna_reset();
        }
    }*/

}
static void do_debug_process(){
	
	STRU_WIRELESS_INFO_DISPLAY *osdptr = (STRU_WIRELESS_INFO_DISPLAY *)(SRAM_BB_STATUS_SHARE_MEMORY_ST_ADDR);
	STRU_DEVICE_INFO *pst_devInfo      = (STRU_DEVICE_INFO *)(DEVICE_INFO_SHM_ADDR);
	if( context.u8_flagdebugRequest & 0x80)
    {
        context.u8_debugMode = context.u8_flagdebugRequest & 0x01;
        if( context.u8_debugMode != FALSE )
        {
            osdptr->head = 0x00;
            osdptr->tail = 0xff;    //end of the writing
        }
        if (context.u8_debugMode == TRUE)
        {
            BB_SPI_DisableEnable(0); //
        }
        else
        {
            BB_SPI_DisableEnable(1); //
        }
        osdptr->in_debug     = context.u8_debugMode;        
        pst_devInfo->isDebug = context.u8_debugMode;
        context.u8_flagdebugRequest = 0;
        //DLOG_Info("DebugMode %d %d\n", osdptr->in_debug, context.u8_debugMode);
    }

}
///////////////////////////////////////////////////////////////////////////////////

static void wimax_vsoc_tx_isr(uint32_t u32_vectorNum)
{
    NVIC_DisableIRQ(BB_TX_ENABLE_VECTOR_NUM);
    do_debug_process();
	
    NVIC_EnableIRQ(TIMER_INTR25_VECTOR_NUM);
    TIM_StartTimer(grd_timer2_5);///tx enable , delay 8ms

    TIM_StartTimer(grd_timer2_6);//tx enable, delay 3.5ms
    NVIC_EnableIRQ(TIMER_INTR26_VECTOR_NUM);
	
    //do rc switch count time
    //NVIC_EnableIRQ(TIMER_INTR21_VECTOR_NUM);
    //TIM_StartTimer(grd_timer2_1);///tx enable , delay 8ms
    if (1 == vt_info.valid)
    {
        TIM_StopTimer(timer2_3);
        TIM_RegisterTimer(timer2_3, 200000);
        TIM_StartTimer(timer2_3);

        if(context.slave_flag && (context.dev_state == FEC_LOCK))
        {
            TIM_StopTimer(timer2_2);
            TIM_RegisterTimer(timer2_2, 200000);
            TIM_StartTimer(timer2_2);
        }
    }

#ifdef RF_9363
    if((RF_600M == context.e_curBand) && (FALSE == context.u8_debugMode) && (AUTO == context.itHopMode))
    {
        BB_GetSweepedChResult( 0 );
    }
#endif
    context.cycle_count ++;
    context.wimax_irq_time = SysTicks_GetTickCount();
    grd_lna_switch();
}

static void Grd_TIM2_2_IRQHandler(uint32_t u32_vectorNum)
{
    uint8_t is_bb_tx_irq_exist;
    
    Reg_Read32(BASE_ADDR_TIMER2 + TMRNEOI_2);

    NVIC_DisableIRQ(TIMER_INTR22_VECTOR_NUM);

    TIM_StopTimer(timer2_2);

    if(SysTicks_GetDiff(context.wimax_irq_time,SysTicks_GetTickCount()) > 30)
    {
        is_bb_tx_irq_exist = 0;
    }
    else
    {
        is_bb_tx_irq_exist = 1;
    }
    
    if(context.itHopMode == AUTO)
    {
        uint8_t val = BB_GetItFrqNum(context.e_curBand);
        if(++context.cur_IT_ch >= val)
        {
            context.cur_IT_ch = 0;
        }
        
        if(is_bb_tx_irq_exist)
        {
            BB_add_cmds(31,context.cur_IT_ch,0,0,0);
            DLOG_Info("it.=%d",context.cur_IT_ch);
        }
        else
        {
            BB_set_ItFrqByCh(context.e_curBand,context.cur_IT_ch);
            DLOG_Info("it=%d",context.cur_IT_ch);
        }
        
        //BB_softReset(BB_GRD_MODE);
    }
    NVIC_EnableIRQ(TIMER_INTR22_VECTOR_NUM);
    TIM_StartTimer(timer2_2);

}

static void Grd_TIM2_3_IRQHandler(uint32_t u32_vectorNum)
{
    Reg_Read32(BASE_ADDR_TIMER2 + TMRNEOI_3);

    NVIC_DisableIRQ(TIMER_INTR23_VECTOR_NUM);
    TIM_StopTimer(timer2_3);
	do_debug_process();
    NVIC_EnableIRQ(TIMER_INTR23_VECTOR_NUM);
    TIM_StartTimer(timer2_3);
	
    if ( context.u8_debugMode )
    {
        return;
    }
    
    grd_handle_all_cmds();
    BB_GetDevInfo();
    BB_grd_OSDPlot();
}
static void Grd_TIM2_5_IRQHandler(uint32_t u32_vectorNum)
{
    Reg_Read32(BASE_ADDR_TIMER2 + TMRNEOI_5);
    NVIC_DisableIRQ(TIMER_INTR25_VECTOR_NUM);
    TIM_StopTimer(grd_timer2_5);
    if(context.u8_debugMode)
    {
        return;
    }
    grd_handle_all_rf_cmds();
}


void Grd_TIM2_6_IRQHandler(uint32_t u32_vectorNum)
{
    Reg_Read32(BASE_ADDR_TIMER2 + TMRNEOI_6);

    //Enable BB_TX intr
    NVIC_ClearPendingIRQ(BB_TX_ENABLE_VECTOR_NUM); //clear pending after TX Enable is LOW. MUST
    NVIC_EnableIRQ(BB_TX_ENABLE_VECTOR_NUM);

    //Disable TIM0 intr
    NVIC_DisableIRQ(TIMER_INTR26_VECTOR_NUM);
    TIM_StopTimer(grd_timer2_6);
    
    //Enable TIM1 intr
    TIM_StartTimer(grd_timer2_7);
    NVIC_EnableIRQ(TIMER_INTR27_VECTOR_NUM);

    if (FALSE == context.u8_debugMode )
    {
        if( context.flag_mrc == 1 )
        {
            context.flag_mrc = 2;
            BB_WriteRegMask(PAGE1, 0x83, 0x01, 0x01); 
        }
        else if( context.flag_mrc == 2 )
        {
            context.flag_mrc = 0;
            BB_WriteRegMask(PAGE1, 0x83, 0x00, 0x01);
        }
		grd_write_sync_cnt();
        grd_handle_all_cmds();
        //grd_CheckSdramBuffer();
        BB_DtSentToSession();
        BB_ComCycleMsgProcess();
        BB_ComCycleSendMsg(BB_COM_TYPE_UART, 0, NULL);
    }
	grd_plot_itsweep();
    Timer1_Delay1_Cnt = 0;
}

static void time_slice0(){
	grd_getSignalStatus();
	rf_pwr_statistics();
	grd_auto_change_rf_bw();
	#ifdef RF_8003X
	{
		BB_GetSweepedChResult(0);
	}
	#endif
	BB_grd_checkSearchEnd();
}
static void time_slice1(){
	grd_fec_judge();
	if(context.enable_non_lbt)
	{
	    if(BB_get_cur_opt_ch() != non_lbt_opt_ch)
	    {
	        DLOG_Warning("opt ch %d -> %d",non_lbt_opt_ch,BB_get_cur_opt_ch());
	        non_lbt_opt_ch = BB_get_cur_opt_ch();
	        BB_DtStopSend(DT_NUM_NON_LBT_NOTICE_OPTCH);
	        BB_DtSendToBuf(DT_NUM_NON_LBT_NOTICE_OPTCH, (uint8_t *)(&non_lbt_opt_ch));
	    }
	    grd_ackSkyVtSkip();
	}	
}
static void time_slice2(){
	grd_do_rc_patten();
	grd_do_rf_bw();
	if(context.rcHopMode == AUTO)
	{
	    #ifdef RF_9363
		{
			grd_rc_hopfreq600m();
		}
	    #endif

	    #ifdef RF_8003X
		{
			grd_rc_hopfreq();
		}
	    #endif
	    
	}
	
	if ( context.locked )
	{
	    grd_checkBlockMode();
	}
	//grd_write_sync_cnt();
	#ifdef RFSUB_BAND
	if(context.freq_band_mode == SUB_BAND)
	{
	    grd_sub_band_run();
	}
	#endif
	
	grd_rc_mod_chg_process();
	#ifdef RF_9363
	if(RF_600M == context.e_curBand)
	{
		grd_filter_chg_process_600m();
	}
	#endif
}

static void time_slice3(){
	BB_GetDevInfo();
	if (context.locked)
	{
		grd_judge_qam_band_switch();
	}
	uint8_t u8_mainCh, u8_optCh;
	if (1 == grd_doRfbandChange( &u8_mainCh, &u8_optCh) )
	{
		context.vtFreqTime[context.cur_IT_ch] = SysTicks_GetTickCount();//save last main ch time value
		context.cur_IT_ch  = u8_mainCh;
		BB_Sweep_updateCh(context.e_curBand, context.cur_IT_ch );
		#ifdef RFSUB_BAND
		if(context.freq_band_mode == SUB_BAND)
		{
			context.sub_band_value = context.cur_IT_ch;
			context.sub_band_main_ch = context.sub_band_value;
			context.sub_band_opt_ch = u8_optCh;
			context.sub_band_main_opt_ch_time = SysTicks_GetTickCount();
			if(context.locked)
			{
				grd_sub_band_excute(context.cur_IT_ch);
			}
			else
			{
				DLOG_Warning("sub band %d->%d", context.sub_band_value,context.cur_IT_ch);
				context.sub_rc_start = BB_GetSubBandStartCH(context.e_curBand,context.st_bandMcsOpt.e_bandwidth,context.sub_band_value);
				context.sub_rc_end = BB_GetSubBandEndCH(context.e_curBand,context.st_bandMcsOpt.e_bandwidth,context.sub_band_value);
				context.grd_rc_channel = context.sub_rc_start;
			}
		}
		else
		#endif	
		{
			context.grd_rc_channel = 0;
		}
		//BB_ResetRcMap();
		grd_rc_hopfreq();
		BB_set_ItFrqByCh(context.e_curBand, context.stru_bandChange.u8_ItCh);
		BB_grd_NotifyItFreqByCh(context.e_curBand, context.stru_bandChange.u8_ItCh);
	}
}

static void time_slice4(){
	if ((context.dev_state == FEC_LOCK) && (context.locked))
	{
		s_st_calcDistData.u32_lockCnt += 1;
		if (s_st_calcDistData.u32_lockCnt >= 3)
		{
		    s_st_calcDistData.u32_lockCnt = 3;
		    grd_calc_dist();
		    #ifdef JYDS
		    grd_limit_dist_process();
		    #endif
		}
		grd_aoc();    
	}
	else
	{
		s_st_calcDistData.u32_lockCnt = 0;
	}
}
static void time_slice5(){
	BB_grd_OSDPlot();
	//BB_DtSentToSession();
	BB_SpiGrdDataTransChProc();
}

static void time_slice6(){
	context.u8_harqcnt_lock = BB_ReadReg(PAGE2, FEC_5_RD);
	if(context.itHopMode == AUTO && context.locked && context.slave_flag == 0)
	{
		flag_snrPostCheck = grd_freq_skip_pre_judge( );
	}
	else
	{
		flag_snrPostCheck = 0;
	}
	
	UNION_BBSPIComHeadByte head;
	head.b.user_data_valid = 0;
	head.b.lock_state = (context.locked & 0x01);
	head.b.vt_match   = ((context.locked & 0x02) >> 1);
	head.b.flag_searchingSupport = BB_grd_isSearching();
	head.b.bb_data_size = 3;
	context.inSearching = BB_grd_isSearching();
	
	BB_WriteReg(PAGE2, SPI_DT_HEAD_REGISTER, head.u8_headByte);
	//BB_WriteReg(PAGE2, SPI_DT_END_ADDR, 0);
	if(!context.flag_in_upgrade)
	{
		BB_ComCycleMsgProcess();
		BB_ComCycleSendMsg(BB_COM_TYPE_SPI, head.b.bb_data_size, NULL);
	}

}

static void time_slice7(){
   NVIC_DisableIRQ(TIMER_INTR27_VECTOR_NUM);
   TIM_StopTimer(grd_timer2_7);
   
   if(flag_snrPostCheck)
   {
	   grd_freq_skip_post_judge( );
   }
   if(context.inSearching) return;
   if(context.itHopMode==MANUAL) return;
   if(context.rf_info.rf_bw_cg_info.en_it_hoping_quickly){
		grd_gen_it_working_ch(1);
   	}

}

void Grd_TIM2_7_IRQHandler(uint32_t u32_vectorNum)
{
    STRU_WIRELESS_INFO_DISPLAY *osdptr = (STRU_WIRELESS_INFO_DISPLAY *)(SRAM_BB_STATUS_SHARE_MEMORY_ST_ADDR);
    Reg_Read32(BASE_ADDR_TIMER2 + TMRNEOI_7); //disable the intr.
    static uint8_t rc_qam_mode = 0;
    if ( context.u8_debugMode )
    {
        NVIC_DisableIRQ(TIMER_INTR27_VECTOR_NUM);                
        TIM_StopTimer(grd_timer2_7);    
        return;
    }
    switch (Timer1_Delay1_Cnt)
    {
        case 0: Timer1_Delay1_Cnt++; time_slice0();break;          
        case 1: Timer1_Delay1_Cnt++; time_slice1();break;
        case 2: Timer1_Delay1_Cnt++; time_slice2();break;
        case 3: Timer1_Delay1_Cnt++; time_slice3();break;
        case 4: Timer1_Delay1_Cnt++; time_slice4();break;
        case 5: Timer1_Delay1_Cnt++; time_slice5();break;
        case 6: Timer1_Delay1_Cnt++; time_slice6();break;
        case 7: Timer1_Delay1_Cnt++; time_slice7();break;
        default: Timer1_Delay1_Cnt = 0; break;   
    }
}

static void Grd_Timer2_7_Init(void)
{
    grd_timer2_7.base_time_group = 2;
    grd_timer2_7.time_num = 7;
    grd_timer2_7.ctrl = 0;
    grd_timer2_7.ctrl |= TIME_ENABLE | USER_DEFINED;
    TIM_RegisterTimer(grd_timer2_7, 1250); //1.25ms
    reg_IrqHandle(TIMER_INTR27_VECTOR_NUM, Grd_TIM2_7_IRQHandler, NULL);
    NVIC_SetPriority(TIMER_INTR27_VECTOR_NUM,NVIC_EncodePriority(NVIC_PRIORITYGROUP_5,INTR_NVIC_PRIORITY_TIMER01,0));
}

void Grd_Timer2_6_Init(void)
{
    grd_timer2_6.base_time_group = 2;
    grd_timer2_6.time_num = 6;
    grd_timer2_6.ctrl = 0;
    grd_timer2_6.ctrl |= TIME_ENABLE | USER_DEFINED;
    
    TIM_RegisterTimer(grd_timer2_6, 3500); //2.5s
    reg_IrqHandle(TIMER_INTR26_VECTOR_NUM, Grd_TIM2_6_IRQHandler, NULL);
    NVIC_SetPriority(TIMER_INTR26_VECTOR_NUM,NVIC_EncodePriority(NVIC_PRIORITYGROUP_5,INTR_NVIC_PRIORITY_TIMER00,0));
}

static void Grd_Timer2_3_Init(void)
{
    TIM_RegisterTimer(timer2_3, 200000);

    reg_IrqHandle(TIMER_INTR23_VECTOR_NUM, Grd_TIM2_3_IRQHandler, NULL);
    NVIC_SetPriority(TIMER_INTR23_VECTOR_NUM,NVIC_EncodePriority(NVIC_PRIORITYGROUP_5,INTR_NVIC_PRIORITY_BB_TX,0));
}
static void Grd_Timer2_2_Init(void)
{
    TIM_RegisterTimer(timer2_2, 200000);

    reg_IrqHandle(TIMER_INTR22_VECTOR_NUM, Grd_TIM2_2_IRQHandler, NULL);
    NVIC_SetPriority(TIMER_INTR22_VECTOR_NUM,NVIC_EncodePriority(NVIC_PRIORITYGROUP_5,INTR_NVIC_PRIORITY_BB_TX,0));
}
static void grd_Timer2_5_Init(void)
{
    grd_timer2_5.base_time_group = 2;
    grd_timer2_5.time_num = 5;
    grd_timer2_5.ctrl = 0;
    grd_timer2_5.ctrl |= TIME_ENABLE | USER_DEFINED;

    TIM_RegisterTimer(grd_timer2_5, 8000);

    reg_IrqHandle(TIMER_INTR25_VECTOR_NUM, Grd_TIM2_5_IRQHandler, NULL);
    NVIC_SetPriority(TIMER_INTR25_VECTOR_NUM,NVIC_EncodePriority(NVIC_PRIORITYGROUP_5,INTR_NVIC_PRIORITY_TIMER00,0));
}

//=====================================Grd RC funcions =====
void grd_rc_hopfreq(void)
{
    static uint32_t unlock_cnt = 0;
    uint8_t max_ch_size;

    if(context.locked == 0)
    {
        unlock_cnt++;
    }
    else
    {
        unlock_cnt = 0;
    }

    if(unlock_cnt >= (3 * VT_CONTINUE_UNLOCK_NUM))
    {
        unlock_cnt = 0;
       if((context.st_bandMcsOpt.e_rfbandMode == AUTO && context.st_bandMcsOpt.e_bandsupport == RF_2G_5G))
        {
            grd_RfBandSelectChannelDoSwitch();  //do switch band and set VT channel
        }
		/*
		if ((context.stru_bandSwitchParam.u8_bandSwitchAfterUnlock==0x81 || RF_5G == context.e_curBand)\
				 && context.locked == 0 && \
				 context.st_bandMcsOpt.e_rfbandMode == AUTO && context.st_bandMcsOpt.e_bandsupport == RF_2G_5G)
		{
            grd_RfBandSelectChannelDoSwitch();  //do switch band and set VT channel
        }
        */
    }
#ifdef RFSUB_BAND

	if(context.freq_band_mode == SUB_BAND)
    {
        context.grd_rc_channel++;
        if(context.grd_rc_channel >= context.sub_rc_end)
        {
            context.grd_rc_channel = context.sub_rc_start;
        }
    }
    else
#endif
    {
    	bb_get_rc_channel();
    }
    BB_set_Rcfrq(context.e_curBand, context.grd_rc_channel);
  
}

void grd_rc_hopfreq600m(void)
{
    context.grd_rc_channel++;
    if((context.grd_rc_channel >= context.rc_end) || (context.grd_rc_channel < context.rc_start))
    {
        context.grd_rc_channel = context.rc_start;
    }

    //BB_WriteRegMask(PAGE2, GRD_SKY_RC_CH_SYNC, grd_rc_channel, GRD_SKY_RC_CH_MASK);

    if (context.st_chSelectionParam.u8_rcChSelectionEnable)
    {
        BB_set_Rcfrq(context.e_curBand, context.rc_ch_map[context.grd_rc_channel]);
    }
    else
    {
        BB_set_Rcfrq(context.e_curBand, context.grd_rc_channel);
    }
}

///////////////////////////////////////////////////////////////////////////////////

/*
 * ground get the sky IT QAM mode
*/
ENUM_BB_QAM grd_get_IT_QAM(void)
{
    return (ENUM_BB_QAM)(BB_ReadReg(PAGE2, GRD_FEC_QAM_CR_TLV) & 0x03);
}

/*
 * ground get the sky IT LDPC mode
*/
ENUM_BB_LDPC grd_get_IT_LDPC(void)
{
    return (ENUM_BB_LDPC)( (BB_ReadReg(PAGE2, GRD_FEC_QAM_CR_TLV) >>2) & 0x07);
}

static void grd_handle_IT_mode_cmd(ENUM_RUN_MODE mode)
{
    context.itHopMode = mode;
    DLOG_Critical("Set IT:%d", context.itHopMode);
    
    if ( mode == AUTO )
    {
        BB_set_ItFrqByCh(context.e_curBand, context.cur_IT_ch);
        BB_DtSendToBuf(DT_NUM_IT_CH_MODE, &mode);
    }
}


/*
  *   only set value to context, the request will be handle in 14ms interrupt.
 */
static void grd_handle_IT_CH_cmd(uint8_t ch)
{
    context.it_manual_ch = ch;
    //DLOG_Info("ch= %d", ch);    
}


/*
  * mode: set RC to auto or Manual
  * ch: the requested channel from  
  * 
 */
static void grd_handle_RC_mode_cmd(ENUM_RUN_MODE mode)
{
    uint8_t data[2] = {0, 0};
    
    if ( mode == AUTO )
    {
        rc_mod_chg_delay.valid = 1;
        rc_mod_chg_delay.cnt = (context.sync_cnt + STATUS_CHG_DELAY);
        memset((uint8_t *)(context.u8_rcValue), 0x00, 5);

        data[0] = (uint8_t)mode;
        data[1] = rc_mod_chg_delay.cnt;
        BB_DtSendToBuf(DT_NUM_RC_CH_MODE, data);
        //DLOG_Info("Set RC auto");
    }
}


/*
 *  handle command for 10M, 20M
*/
static void grd_handle_CH_bandwitdh_cmd(ENUM_CH_BW bw)
{
    if(context.st_bandMcsOpt.e_bandwidth != bw)
    {
        BB_set_RF_bandwitdh(BB_GRD_MODE, bw);

        BB_DtSendToBuf(DT_NUM_RF_CH_BW_CHANGE, (uint8_t *)(&bw));

        context.st_bandMcsOpt.e_bandwidth = bw; 
        
        DLOG_Info("band:%d", context.st_bandMcsOpt.e_bandwidth);
    }
}


/*
 *  handle command for LNA status
 0 ----> set grd lna always bypass
 1 ----> set grd lna auto
 2 ----> set sky lna always bypass
 3 ----> set sky lna auto
 4 ----> set grd lna always open
 5 ----> set sky lna always open
*/
static void grd_handle_LNA_status_cmd(uint8_t value)
{
    if(value == 0 || value == 1 || value == 4)
    {
        context.command_lna_switch_flag = 1;
        context.command_lna_switch_value = value;
    }
    else if(value == 2 || value == 3 || value == 5)
    {
        BB_DtSendToBuf(DT_NUM_LNA_STATUS_CHG, (uint8_t *)(&value));
    }
    
}

static void grd_handle_CH_qam_cmd(ENUM_BB_QAM qam)
{
    //set and soft-rest
    //if(context.qam_mode != qam)
    {
        BB_DtSendToBuf(DT_NUM_RF_CH_QAM_CHANGE, (uint8_t *)(&qam));

        context.qam_mode = qam; 
        context.ldpc = grd_get_IT_LDPC();   
        grd_set_mcs_registers(context.qam_mode, context.ldpc, context.st_bandMcsOpt.e_bandwidth);
    }

    DLOG_Info("CH_QAM =%d", context.qam_mode);    
}

void grd_handle_CH_ldpc_cmd(ENUM_BB_LDPC e_ldpc)
{
    //if(context.ldpc != e_ldpc)
    {
        BB_DtSendToBuf(DT_NUM_RF_CH_LDPC_CHANGE, (uint8_t *)(&e_ldpc));
        context.ldpc = e_ldpc;
        context.qam_mode = grd_get_IT_QAM();

        grd_set_mcs_registers(context.qam_mode, context.ldpc, context.st_bandMcsOpt.e_bandwidth);
    }
    //DLOG_Info("CH_LDPC =%d", context.ldpc);
}

static void grd_handle_take_picture_cmd(uint8_t arg)
{
    static uint8_t idx = 0;
    uint8_t par[2] = {idx, arg};
    
    BB_DtSendToBuf(DT_NUM_TAKE_PICTURE, par);
    idx++;
}

static void grd_handle_picture_quality_set_cmd(uint8_t arg)
{
    static uint8_t idx = 0;
    uint8_t par[2] = {idx, arg};

    BB_DtSendToBuf(DT_NUM_PICTURE_QUALITY_SET, par);
    idx++;
}

        
static void grd_handle_MCS_mode_cmd(ENUM_RUN_MODE mode)
{
    context.qam_skip_mode = mode;
    context.brc_mode = mode;

    BB_DtSendToBuf(DT_NUM_MCS_MODE_SELECT, (uint8_t *)(&mode));
    DLOG_Info("qam_skip_mode = %d\n", context.qam_skip_mode);
    
    if ( mode == AUTO )
    {
        ENUM_BB_QAM qam = grd_get_IT_QAM();
        ENUM_BB_LDPC ldpc = grd_get_IT_LDPC();

        if( qam == MOD_BPSK )
        {
            context.qam_ldpc = 1;
        }
        else if( qam == MOD_4QAM )
        {
            context.qam_ldpc = 2;
        }
        else if( qam == MOD_16QAM )
        {
            context.qam_ldpc = 3;
        }
        else if( qam == MOD_64QAM && ldpc == LDPC_1_2)
        {
            context.qam_ldpc = 4;
        }
        else if( qam == MOD_64QAM)
        {
            context.qam_ldpc = 5;
        }

        grd_set_txmsg_mcs_change(context.st_bandMcsOpt.e_bandwidth, context.qam_ldpc);        
    }
}

/*
  * handle H264 encoder brc 
 */
static void grd_handle_brc_bitrate_cmd(uint8_t u8_ch, uint8_t brcidx)
{
    if (0 == u8_ch)
    {
        BB_DtSendToBuf(DT_NUM_ENCODER_BRC_CHAGE_CH1, &brcidx);
        context.brc_bps[0] = brcidx;

        DLOG_Info("brc_coderate_ch1 = %d ", brcidx);
    }
    else
    {
        BB_DtSendToBuf(DT_NUM_ENCODER_BRC_CHAGE_CH2, &brcidx);
        context.brc_bps[1] = brcidx;

        DLOG_Info("brc_coderate_ch2 = %d ", brcidx);
    }
}

void BB_grd_notify_rc_skip_freq(uint8_t *u8_rcfrq)
{
    BB_DtSendToBuf(DT_NUM_RC_FRQ, u8_rcfrq);
	
}

static void grd_ask_auto_bw_set(ENUM_CH_BW bw){
	context.rf_bw.en_flag=1;
	context.rf_bw.valid=1;
	context.rf_bw.bw=bw;
	context.rf_bw.timeout_cnt=context.sync_cnt+STATUS_CHG_DELAY;
}

void grd_handle_one_cmd(STRU_WIRELESS_CONFIG_CHANGE* pcmd)
{
    uint8_t class  = pcmd->u8_configClass;
    uint8_t item   = pcmd->u8_configItem;
    uint32_t value = pcmd->u32_configValue;
    uint32_t value1 = pcmd->u32_configValue1;
    uint8_t frq[6] = {(value>>24)&0xff, (value>>16)&0xff, (value>>8)&0xff, value&0xff, (value1&0xff), 0};
    uint16_t frq_v; 
    uint8_t u8_rcArray[5]   = {(value>>24)&0xff, (value>>16)&0xff, (value>>8)&0xff, value&0xff, (value1&0xff)};
    uint8_t u8_vtIdArray[2] = {(value1>>8)&0xff, (value1>>16)&0xff};

    DLOG_Info("class item value %d %d 0x%0.8x ", class, item, value);
    if(class == WIRELESS_FREQ_CHANGE)
    {
        switch(item)
        {
            case FREQ_BAND_MODE:
            {
                context.st_bandMcsOpt.e_rfbandMode = (ENUM_RUN_MODE)value;
                break;
            }

            case FREQ_BAND_SELECT:
            {
                context.st_bandMcsOpt.e_rfbandMode = MANUAL;

                if(1 == vt_info.valid)
                {
                    if ((ENUM_RF_BAND)value != context.e_curBand)
                    {
                        grd_RfBandSelectChannelDoSwitch();
                    }
                }
                else
                {    
                    #ifdef JYDS
                    DLOG_Warning("unsupport");
                    #else
                    grd_notifyRfbandChange((ENUM_RF_BAND)value, 0, 1, 1);
                    DLOG_Info("grd_handle_one_cmd DT_NUM_RF_BAND_MODE");
                    #endif
                }
                break;
            }

            case FREQ_CHANNEL_MODE: //auto manual
            {
                grd_handle_IT_mode_cmd((ENUM_RUN_MODE)value);
                break;
            }
            
            case FREQ_CHANNEL_SELECT:
            {
                grd_handle_IT_CH_cmd((uint8_t)value);
                break;
            }

            case RC_CHANNEL_MODE:
            {
                grd_handle_RC_mode_cmd( (ENUM_RUN_MODE)value);
                break;
            }

            case RC_CHANNEL_SELECT:
            {
                BB_set_Rcfrq(context.e_curBand, value);
                DLOG_Warning("RC_CHANNEL_FREQ 0x%0.8x ", value);  
                break;
            }

            case RC_CHANNEL_FREQ:
            {
                rc_mod_chg_delay.valid = 1;
                rc_mod_chg_delay.cnt = context.sync_cnt + STATUS_CHG_DELAY;
                frq[5] = rc_mod_chg_delay.cnt;

                #ifdef RF_9363
                BB_grd_notify_rc_skip_freq(frq);    //set to manual and set the regigsters
                #endif

                #ifdef RF_8003X
                /*frq_v = RF8003xCalcRegister2Frq(value);
		                DLOG_Warning("%x->%d",value,frq_v);
		                frq[0] = frq_v >> 8;
		                frq[1] = frq_v;*/
                BB_grd_notify_rc_skip_freq( frq ); 
                #endif

                memcpy((uint8_t *)(context.u8_rcValue), frq, 5);

                DLOG_Info("set RC FREQ %x%x%x%x%x", frq[0], frq[1], frq[2], frq[3], frq[4]);
                break;
            }

            case IT_CHANNEL_FREQ:
            {
                grd_handle_IT_mode_cmd( (ENUM_RUN_MODE)MANUAL);

                #ifdef RF_9363
				//if(RF_600M == context.e_curBand)
				{
					grd_BB_write_ItRegs( frq );
				}
                #endif

                #ifdef RF_8003X
				//else
				{
					BB_write_ItRegs( frq );
				}
                #endif

                #ifdef RF_9363
                BB_grd_NotifyItFreqByValue( frq ); 
                #endif

                #ifdef RF_8003X
                /*frq_v = RF8003xCalcRegister2Frq(value);
                DLOG_Warning("%x->%d",value,frq_v);
                frq[0] = frq_v >> 8;
                frq[1] = frq_v;*/
                BB_grd_NotifyItFreqByValue( frq ); 
				DLOG_Warning("IT_CHANNEL_FREQ %d ", frq);  
                #endif
                break;
            }

            case IT_CHANNEL_SELECT:
            {
                if(context.slave_flag)
                {
                    BB_set_ItFrqByCh(context.e_curBand, value);
                }
                else
                {
                    grd_handle_IT_mode_cmd( (ENUM_RUN_MODE)MANUAL);
                    context.cur_IT_ch = value;
                    //BB_set_ItFrqByCh(context.e_curBand, value);
                    BB_grd_NotifyItFreqByCh(context.e_curBand, value);
                    context.dev_state = FIND_SAME_DEV;
                }
                //DLOG_Info("IT_CHANNEL_FREQ 0x%0.8x ", value); 
                break;
            }

            case FREQ_BAND_WIDTH_SELECT:
            {
                grd_handle_CH_bandwitdh_cmd((ENUM_CH_BW)value);
                break;
            }
			case AUTTO_BW_CHANGE:
			{
				grd_ask_auto_bw_set((ENUM_CH_BW)value);
				break;
			}
            case SET_LNA_STATUS:
            {
                grd_handle_LNA_status_cmd(value);
                break;
            }

                
            default:
            {
                //DLOG_Warning("%s", "unknown WIRELESS_FREQ_CHANGE command");
                break;
            }
        }
    }

    if (class == WIRELESS_AUTO_SEARCH_ID)
    {
        BB_grd_handleRcSearchCmd(pcmd);
    }

    if(class == WIRELESS_MCS_CHANGE)
    {
        switch(item)
        {
            case MCS_MODE_SELECT:
            {
                grd_handle_MCS_mode_cmd( (ENUM_RUN_MODE)value);
                
                //For osd information
                if ( context.brc_mode == MANUAL)
                {
                    context.brc_bps[0] = BB_get_bitrateByMcs(context.st_bandMcsOpt.e_bandwidth, context.qam_ldpc);
                    context.brc_bps[1] = BB_get_bitrateByMcs(context.st_bandMcsOpt.e_bandwidth, context.qam_ldpc);                
                }
            }
    
            break;

            case MCS_MODULATION_SELECT:
            {
                context.qam_ldpc = value;
                grd_set_txmsg_mcs_change(context.st_bandMcsOpt.e_bandwidth, value);
            }
            break;

            case MCS_CODE_RATE_SELECT:
            {
                //grd_set_txmsg_ldpc(value);
                break;
            }
            case MCS_IT_QAM_SELECT:
            {
                grd_handle_CH_qam_cmd((ENUM_BB_QAM)value);
                DLOG_Info("MCS_IT_QAM_SELECT %x", value);                
                break;
            }
            
            case MCS_IT_CODE_RATE_SELECT:
            {
                grd_handle_CH_ldpc_cmd((ENUM_BB_LDPC)value);
                DLOG_Info("MCS_IT_CODE_RATE_SELECT %x", value);  
                break;              
            }
                
            case MCS_RC_QAM_SELECT:
            {
                context.rc_qam_mode = (ENUM_BB_QAM)(value & 0x01);
                BB_set_QAM(context.rc_qam_mode);
                DLOG_Info("MCS_RC_QAM_SELECT %x", value);                
                break;
            }
            
            case MCS_RC_CODE_RATE_SELECT:
            {
                context.rc_ldpc = (ENUM_BB_LDPC)(value & 0x01);
                BB_set_LDPC(context.rc_ldpc);
                DLOG_Info("MCS_RC_CODE_RATE_SELECT %x", value); 
                break;               
            }
            case MCS_CHG_RC_RATE:
            {
                if (0 == context.RcChgRate.en_flag && value != context.uplink_qam_mode)
                {
                    context.RcChgRate.cnt= (context.sync_cnt + STATUS_CHG_DELAY);
                    context.RcChgRate.value= value;
                    context.RcChgRate.en_flag = 1;

                    BB_DtSendToBuf(DT_NUM_RC_RATE, (uint8_t *)&(context.RcChgRate.value));

                    DLOG_Warning("rcrate %d %d", value,context.RcChgRate.cnt); 
                }
                break;               
            }
            default:
                //DLOG_Warning("%s", "unknown WIRELESS_MCS_CHANGE command");
                break;
        }        
    }

    if(class == WIRELESS_ENCODER_CHANGE)
    {
        switch(item)
        {
            case ENCODER_DYNAMIC_BIT_RATE_SELECT_CH1:
                grd_handle_brc_bitrate_cmd(0, (uint8_t)value);
                DLOG_Info("ch1:%d",value);
                break;

            case ENCODER_DYNAMIC_BIT_RATE_SELECT_CH2:
                grd_handle_brc_bitrate_cmd(1, (uint8_t)value);
                DLOG_Info("ch2:%d",value);
                break;
            case ENCODER_TAKE_PICTURE:
                grd_handle_take_picture_cmd((uint8_t)value);
                break;
            case ENCODER_PICTURE_QUALITY_SET:
                //grd_handle_picture_quality_set_cmd((uint8_t)value);
                context.realtime_mode = value > 0 ? 1 : 0;
                BB_WriteRegMask(PAGE2, MCS_INDEX_MODE,context.realtime_mode ? 0x80 : 0x00,0x80);
                DLOG_Warning("realtime = %d",context.realtime_mode);
                break;

            default:
                //DLOG_Warning("%s", "unknown WIRELESS_ENCODER_CHANGE command");
                break;                
        }
    }

    if(class == WIRELESS_MISC)
    {
        BB_handle_misc_cmds(pcmd);
    }

    if(class == WIRELESS_OTHER)
    {
        switch(item)
        {
            case GET_DEV_INFO:
            {
                BB_GetDevInfo();
                break;
            }

            case SWITCH_ON_OFF_CH1:
            {
                BB_SwtichOnOffCh(0, (uint8_t)value);
                break;
            }

            case SWITCH_ON_OFF_CH2:
            {
                BB_SwtichOnOffCh(1, (uint8_t)value);
                break;
            }

            case BB_SOFT_RESET:
            {
                DLOG_Info("grd bb reset.");
                BB_softReset(BB_GRD_MODE);
                break;
            }

            case CALC_DIST_ZERO_CALI:
            {
                //DLOG_Info("calculation distance zero calibration.");
                grd_calc_dist_zero_calibration();
                break;
            }

            case SET_CALC_DIST_ZERO_POINT:
            {
                //DLOG_Info("set distance zero point.");
                grd_set_calc_dist_zero_point(value);
                break;
            }

            case SET_RC_FRQ_MASK:
            {
                grd_disable_enable_rc_frq_mask_func(value);
                break;
            }

            case SET_PURE_VT_MODE:
            {
                grd_vt_mode_proc((uint8_t)value);
                break;
            }

            case RW_BB_RF_REG:
            {
                BB_InsertCmd(1, pcmd);
                break;
            }
            
            case PWR_CTRL_SET:
            {
                BB_InsertCmd(1, pcmd);
                break;
            }

            case CMD_VT_SWEEP:
            {
                #ifdef RF_9363
                if(RF_600M == context.e_curBand)
                {
                    BB_CmdSweepStart(context.e_curBand, context.st_bandMcsOpt.e_bandwidth);
                }
                #endif

                break;
            }
            
            case CMD_SELECT_FILTER:
            {
                #ifdef RF_9363
                if(RF_600M == context.e_curBand)
                {
                    grd_cmd_select_filter((uint8_t)value);
                }
                #endif

                break;
            }
            
            case SET_SWEEP_MODE:
            {
                grd_SetSweepMode((uint8_t)value);
                break;
            }
            
            case SUB_BAND_SET_CH:
            {
                context.itHopMode = MANUAL;
                DLOG_Warning("subBB set it %d->%d",context.cur_IT_ch,(uint8_t)value);
                context.cur_IT_ch = (uint8_t)value;
                BB_grd_NotifyItFreqByCh(context.e_curBand, context.cur_IT_ch);
                context.dev_state = DELAY_14MS;

                break;
            }
            
            case SET_GRD_SLAVE_MODE:
            {
                grd_enter_slave_mode(u8_rcArray,u8_vtIdArray);
                break;
            }
            
            case SET_NON_LBT:
            {
                context.enable_non_lbt = (uint8_t)value;
                //BB_DtSendToBuf(DT_NUM_NON_LBT_ENABLE, (uint8_t *)(&value));
                DLOG_Warning("non-lbt status = %d",context.enable_non_lbt);
                BB_WriteRegMask(PAGE2, MCS_INDEX_MODE,context.enable_non_lbt ? 0x40 : 0x00,0x40);
                break;
            }
            
            case SET_BANDEDGE:
            {
                context.bandedge_enable = (uint8_t)value;
                context.low_power_db = (uint8_t)value1;
                DLOG_Warning("bandege %d,%d",context.bandedge_enable,context.low_power_db);
                break;
            }

            default:
                break;
        }
    }
}

void grd_handle_one_rf_cmd(STRU_WIRELESS_CONFIG_CHANGE* pcmd)
{
    uint8_t class  = pcmd->u8_configClass;
    uint8_t item   = pcmd->u8_configItem;
    uint32_t value = pcmd->u32_configValue;
    uint32_t value1 = pcmd->u32_configValue1;

    DLOG_Info("class item value %d %d 0x%0.8x ", class, item, value);

    if(class == WIRELESS_OTHER)
    {
        switch(item)
        {
            case RW_BB_RF_REG:
            {
                //DLOG_Warning("%08x", value);
                BB_NormalModePcRwReg((void *)(&pcmd->u32_configValue));
                break;
            }
            case PWR_CTRL_SET:
            {
                STRU_PWR_CTRL ctrl =
                {
                    .pwr_ctrl = (ENUM_PWR_CTRL)(pcmd->u32_configValue),
                };

                PWR_CTRL_ModeSet(&ctrl);
                break;
            }
			
			case WIRELESS_RF_PA_MODE:
					if(context.e_powerMode){
						BB_SetPowerCloseMode(RF_2G);
					}
					else {
						BB_SetPowerOpenMode(RF_2G);
					}
				break;
				
            default:
                break;
        }
    }
}

static void grd_do_rcRate(void)
{
    if (1 == context.RcChgRate.en_flag)
    {
        if (context.sync_cnt == (context.RcChgRate.cnt) )
        {
            context.RcChgRate.en_flag = 0;
            grd_ChgRcRate(context.RcChgRate.value);
            context.RcChgRate.timeout_cnt = 0;
            context.RcChgRate.en_reset = 1;
            DLOG_Warning("%d",context.RcChgRate.value);    
            //BB_softTxReset(BB_GRD_MODE);
        }
    }

    if (context.RcChgRate.en_reset)
    {
        context.RcChgRate.timeout_cnt++;
        if(context.dev_state == FEC_LOCK){
            context.RcChgRate.timeout_cnt = 0;
            //DLOG_Warning("1");
        }

        if(context.RcChgRate.timeout_cnt > 3){
            context.RcChgRate.en_reset = 0;
            grd_ChgRcRate(context.RcChgRate.default_qam_value);
            BB_softReset(BB_GRD_MODE);
            DLOG_Warning("reback2 %d",context.RcChgRate.default_qam_value);
        }

    }
}

static void grd_do_rc_patten(void)
{
	if (1 == context.rcChgPatten.en_flag)
	{
		if (context.sync_cnt == context.rcChgPatten.timeout_cnt)
		{
			//context.rf_info.rc_patten_nextchg_delay = SysTicks_GetTickCount();
			rc_update_working_patten();
			context.rcChgPatten.en_flag=0;
			context.rcChgPatten.timeout_cnt=0;
		}
	}
}

static void grd_do_rf_bw(void)
{
	uint8_t bw=0;
	if (1 == context.rf_bw.en_flag)
	{
		if (context.sync_cnt == context.rf_bw.timeout_cnt )
		{
			context.rf_bw.en_flag=0;
			context.rf_bw.timeout_cnt=0;
			bw=context.rf_bw.bw;
			if(context.st_bandMcsOpt.e_bandwidth != bw)
		    {
		    	context.st_bandMcsOpt.e_bandwidth = bw; 
				RF_GetFctFreqTable(context.st_bandMcsOpt.e_bandwidth);
				reset_sweep_table(context.e_curBand);
				
				context.rf_info.e_bw=bw;
				context.rf_info.u8_prevSweepCh=0;
				
				uint8_t main_ch = 0, opt_ch = 0;
    			BB_selectBestCh(context.e_curBand, SELECT_MAIN_OPT, &main_ch, &opt_ch, NULL, 0);
				//BB_SweepChangeBand(context.e_curBand, main_ch, opt_ch);
				context.rf_info.u8_cycleCnt      =  MAIN_CH_CYCLE;
			    context.rf_info.u8_mainSweepRow  =  0;
			    context.rf_info.u8_optSweepRow   =  0;
			    context.rf_info.u8_prevSweepCh   = 0x0;
			    context.rf_info.u16_preMainCount = 0x0;
			    context.rf_info.u8_mainSweepCh = main_ch;
			    context.rf_info.u8_prevSweepCh = main_ch;
				BB_set_RF_bandwitdh(BB_GRD_MODE, bw);
				context.qam_ldpc=context.rf_bw.ldpc;
				if( context.itHopMode == AUTO)
					BB_set_ItFrqByCh(context.e_curBand, context.stru_bandChange.u8_ItCh);
				grd_set_txmsg_mcs_change(context.st_bandMcsOpt.e_bandwidth, context.qam_ldpc);
				BB_softRxReset(BB_GRD_MODE);
				DLOG_Critical("set rf bandwidth=%d", context.st_bandMcsOpt.e_bandwidth);
			}
		}
	}
}

static void grd_notify_rf_bw()
{
	uint8_t buf[10];
	int i=0;
	static int gap =0;
	#define delay (2)
	gap++;
	if(context.rf_bw.en_flag==1 && context.rf_bw.valid==1 && gap < delay)
	{
		buf[0]= context.rf_bw.bw;
		buf[1]= context.rf_bw.timeout_cnt;
		buf[2]= context.rf_bw.ldpc;
		buf[3]= context.rf_bw.autobw;
		BB_Session0SendMsg(DT_NUM_AUTO_CH_BW_CHANGE, buf,4);
	}
	if(gap > 5) gap = 0;
}

static void grd_notify_rc_patten()
{
	uint8_t buf[10];
	int i=0;
	static int gap =0;
	#define delay (2)
	gap++;
	if(context.rcChgPatten.en_flag_grd==1 && context.rcChgPatten.valid_grd==1 && gap < delay)
	{
		for(i=0;i<context.rf_info.rc_ch_patten_need_id_size;i++)
		{
			buf[i+2]=context.rcChgPatten.patten_grd[i];
		}
		buf[0]= context.rcChgPatten.timeout_cnt_grd;
		buf[1]=context.rf_info.rc_patten_set_by_usr;
		BB_Session0SendMsg(DT_NUM_GRD_RC_CHPATTEN, buf, context.rf_info.rc_ch_patten_need_id_size+2);
		DLOG_Critical("grd notify sky :cnt=%d,aim_cnt=%d", context.sync_cnt, context.rcChgPatten.timeout_cnt_grd);
	}
	if(gap > 5) gap = 0;
}

static void grd_handle_rc_patten_cmd(uint8_t *arg)
{
	int i=0;
	uint8_t same=1;
	uint8_t buf[10];
	for(i=0;i<context.rf_info.rc_ch_patten_need_id_size;i++){
			if(context.rcChgPatten.patten[i]!=arg[i+2]){
				same=0;
			}
		}
	if(same) return;
	if(context.rcChgPatten.en_flag==0)
	{
		for(i=0;i<context.rf_info.rc_ch_patten_need_id_size;i++)context.rcChgPatten.patten[i]=0;
		context.rcChgPatten.en_flag=1;
		context.rcChgPatten.timeout_cnt=arg[0];
		//DLOG_Critical("grd get new patten ,sync_cnt=%d,aim=%d",context.sync_cnt,context.rcChgPatten.timeout_cnt);
		for(i=0;i<context.rf_info.rc_ch_patten_need_id_size;i++){
			context.rcChgPatten.patten[i]=arg[i+2];
			buf[0] = arg[i+2];
		}
		BB_Session0SendMsg(DT_NUM_SKY_RC_PATTEN, &buf[0], context.rf_info.rc_ch_patten_need_id_size+2);
		//DLOG_Critical("ack to sky grd notify sky :cnt=%d,aim_cnt=%d", context.sync_cnt, context.rcChgPatten.timeout_cnt);
	}
}

static void grd_handle_all_cmds(void)
{   
    int ret = 0;
    int cnt = 0;
    STRU_WIRELESS_CONFIG_CHANGE cfg;
    while( (cnt++ < 5) && (1 == BB_GetCmd(0, &cfg))){
        //DLOG_Warning("%08x", cfg.u32_configValue);
        grd_handle_one_cmd( &cfg );
    }
    if (1 == vt_info.valid){
        return;
    }
    grd_do_rcRate();
    BB_grd_uartDataHandler();
	grd_notify_rc_patten();
	grd_notify_rf_bw();
}
static void grd_handle_all_rf_cmds(void)
{   
    int ret = 0;
    int cnt = 0;
    STRU_WIRELESS_CONFIG_CHANGE cfg;
    while( (cnt++ < 5) && (1 == BB_GetCmd(1, &cfg)))
    {
        //DLOG_Warning("%d %08x", cnt, cfg.u32_configValue);
        grd_handle_one_rf_cmd( &cfg );
    }
}
static void BB_grd_OSDPlot(void)
{
    uint8_t u8_data;
    STRU_WIRELESS_INFO_DISPLAY *osdptr = (STRU_WIRELESS_INFO_DISPLAY *)(SRAM_BB_STATUS_SHARE_MEMORY_ST_ADDR);

    osdptr->msg_id       = 0x33;
    osdptr->head         = 0xff; //starting writing
    osdptr->tail         = 0x00;

    osdptr->agc_value[0] = stru_grdstatus.agc_value1;
    osdptr->agc_value[1] = stru_grdstatus.agc_value2;
    /*
    osdptr->agc_value[2] = BB_ReadReg(PAGE2, RX3_GAIN_ALL_R);
    osdptr->agc_value[3] = BB_ReadReg(PAGE2, RX4_GAIN_ALL_R);
    */

    osdptr->lock_status  = context.locked;
    
    osdptr->snr_vlaue[0] = stru_grdstatus.u16_grd_snr;
    osdptr->snr_vlaue[1] = get_snr_average();

    //masoic
    osdptr->u16_afterErr = (((uint16_t)BB_ReadReg(PAGE2, LDPC_ERR_AFTER_HARQ_HIGH_8)) << 8) | BB_ReadReg(PAGE2, LDPC_ERR_AFTER_HARQ_LOW_8);
    osdptr->ldpc_error   = (((uint16_t)BB_ReadReg(PAGE2, LDPC_ERR_HIGH_8)) << 8) | BB_ReadReg(PAGE2, LDPC_ERR_LOW_8); //1byte is enough
    osdptr->harq_count   = (context.u8_harqcnt_lock & 0xF0) | ((BB_ReadReg(PAGE2, 0xd1)& 0xF0) >> 4);

    osdptr->u8_mcs          = context.qam_ldpc;
    osdptr->modulation_mode = grd_get_IT_QAM();
    osdptr->code_rate       = grd_get_IT_LDPC();
    //context.qam_mode = osdptr->modulation_mode;

    u8_data = BB_ReadReg(PAGE2, TX_2);
    osdptr->rc_modulation_mode = (u8_data >> 6) & 0x01;
    osdptr->rc_code_rate       = (u8_data >> 0) & 0x01;
    osdptr->e_bandwidth        = context.st_bandMcsOpt.e_bandwidth;
    osdptr->in_debug           = context.u8_debugMode;

    memset(osdptr->sweep_energy, 0, sizeof(osdptr->sweep_energy));
    BB_GetSweepNoise(osdptr->sweep_energy, 21);

    if(context.brc_mode == AUTO)
    {
        osdptr->encoder_bitrate[0] = BB_get_bitrateByMcs(context.st_bandMcsOpt.e_bandwidth, context.qam_ldpc);
        osdptr->encoder_bitrate[1] = BB_get_bitrateByMcs(context.st_bandMcsOpt.e_bandwidth, context.qam_ldpc);
    }
    else
    {
        osdptr->encoder_bitrate[0] = context.brc_bps[0];
        osdptr->encoder_bitrate[1] = context.brc_bps[1];
    }

    osdptr->u8_rclock =  g_stru_skyStatus.u8_rcCrcLockCnt;
    osdptr->u8_nrlock =  g_stru_skyStatus.u8_rcNrLockCnt;

    osdptr->sky_agc[0]=  g_stru_skyStatus.u8_skyagc1;
    osdptr->sky_agc[1]=  g_stru_skyStatus.u8_skyagc2;
    osdptr->sky_snr   =  g_stru_skyStatus.snr;

    osdptr->dist_zero = (uint16_t)(s_st_calcDistData.u32_calcDistZero);
    osdptr->dist_value= (uint16_t)(s_st_calcDistData.u32_calcDistValue);
	osdptr->current_pwr= g_stru_skyStatus.tx_pwr;

    osdptr->head = 0x00;
    osdptr->tail = 0xff;    //end of the writing
}

static void grd_set_ItFrq(ENUM_RF_BAND e_band, uint8_t ch)
{
    BB_set_ItFrqByCh(e_band, ch);
    context.flag_mrc = 1;
}

static uint8_t grd_calc_dist_judge_raw_data(ENUM_CALC_DIST_STATUS e_status, uint32_t data, uint32_t threld)
{
    uint8_t *pData;
    uint32_t preData;
    uint8_t result = 1;
    static uint32_t cnt = 0;

    if ((CALI_ZERO == e_status) || (CALC_DIST_PREPARE == e_status))
    {
        if (0 != (s_st_calcDistData.u32_cnt))
        {
            pData = s_st_calcDistData.u8_rawData[s_st_calcDistData.u32_cnt - 1];
            preData = (pData[0] << 16) | (pData[1] << 8) | (pData[2] << 0);
            if ((abs(data - preData) * 3 / 2) < threld)
            {
                cnt = 0;
            }
            else
            {
                cnt++;
                if (cnt > DEFAULT_DIST_JUDGE_CNT)
                {
                    cnt = 0;
                    s_st_calcDistData.u32_cnt = 0;
                    s_st_calcDistData.e_status = e_status;
                    //DLOG_Info("***e_status %d", s_st_calcDistData.e_status);
                }
                result = 0;
            }
        }
    }

    return result;
}

static void grd_calc_dist(void)
{
    uint8_t u8_data2[3];
    uint32_t u32_data2 = 0;
    uint32_t cmpData;
    static uint32_t cmpCnt = 0;
    static uint32_t u32_printCnt = 1;

    u8_data2[2] = BB_ReadReg(PAGE3, 0xA7);
    u8_data2[1] = BB_ReadReg(PAGE3, 0xA6);
    u8_data2[0] = BB_ReadReg(PAGE3, 0xA5);

    cmpData = (u8_data2[0] << 16) | (u8_data2[1] << 8) | (u8_data2[2] << 0);

    switch(s_st_calcDistData.e_status)
    {
        case INVALID:
        {
            s_st_calcDistData.e_status = CALC_DIST_PREPARE;
            s_st_calcDistData.u32_cnt = 0;
            s_st_calcDistData.u32_calcDistZero = ((BW_20M == (context.st_bandMcsOpt.e_bandwidth)) ? (DEFAULT_DIST_ZERO_20M) : (DEFAULT_DIST_ZERO_10M));
            break;
        }
        case CALI_ZERO:
        {
            if (0 == grd_calc_dist_judge_raw_data(CALI_ZERO, cmpData, DEFAULT_DIST_JUDGE_THRELD))
            {
                return; // raw data error
            }
            memcpy(s_st_calcDistData.u8_rawData[s_st_calcDistData.u32_cnt % CALC_DIST_RAW_DATA_MAX_RECORD], u8_data2, 3);
            s_st_calcDistData.u32_cnt += 1;
            if (s_st_calcDistData.u32_cnt >= (CALC_DIST_RAW_DATA_MAX_RECORD))
            {
                s_st_calcDistData.u32_calcDistZero = grd_calc_dist_get_avg_value(&u32_data2);
                s_st_calcDistData.u32_cnt = 0;
                s_st_calcDistData.e_status = CALC_DIST_PREPARE;
                DLOG_Warning("CALI_ZERO Zero:%d", s_st_calcDistData.u32_calcDistZero);
            }
            break;
        }
        case CALC_DIST_PREPARE:
        {
            if (0 == grd_calc_dist_judge_raw_data(CALC_DIST_PREPARE, cmpData, DEFAULT_DIST_JUDGE_THRELD))
            {
                return; // raw data error
            }
            memcpy(s_st_calcDistData.u8_rawData[s_st_calcDistData.u32_cnt % CALC_DIST_RAW_DATA_MAX_RECORD], u8_data2, 3);
            s_st_calcDistData.u32_cnt += 1;
            if (s_st_calcDistData.u32_cnt >= CALC_DIST_RAW_DATA_MAX_RECORD)
            {
                grd_calc_dist_get_avg_value(&(s_st_calcDistData.u32_calcDistValue));

                s_st_calcDistData.u32_cnt = 0;
                s_st_calcDistData.e_status = CALC_DIST;
                DLOG_Info("CALC_DIST_PREPARE Value:%d", s_st_calcDistData.u32_calcDistValue);
            }
            break;
        }
        case CALC_DIST:
        {
            cmpData = abs(cmpData - s_st_calcDistData.u32_calcDistZero) * 3 / 2;
            if (abs(cmpData - s_st_calcDistData.u32_calcDistValue) < DEFAULT_DIST_JUDGE_THRELD)
            {
                cmpCnt = 0;
                memcpy(s_st_calcDistData.u8_rawData[s_st_calcDistData.u32_cnt % CALC_DIST_RAW_DATA_MAX_RECORD], u8_data2, 3);
                s_st_calcDistData.u32_cnt += 1;
                
                grd_calc_dist_get_avg_value(&(s_st_calcDistData.u32_calcDistValue));
            }
            else
            {
                cmpCnt++;
                if (cmpCnt > DEFAULT_DIST_JUDGE_CNT)
                {
                    cmpCnt = 0;
                    s_st_calcDistData.u32_cnt = 0;
                    s_st_calcDistData.e_status = CALC_DIST_PREPARE;
                    DLOG_Info("***return to status CALC_DIST_PREPARE");
                }
            }
            
            u32_printCnt += 1;
            if (0 == (u32_printCnt % 500))
            {
                DLOG_Info("Zero:%d Value:%d", s_st_calcDistData.u32_calcDistZero, s_st_calcDistData.u32_calcDistValue);
            }
            break;
        }
        default:
        {
            s_st_calcDistData.e_status = CALI_ZERO;
            s_st_calcDistData.u32_cnt = 0;
            cmpCnt = 0;
            break;
        }     
    }
}

static void grd_calc_dist_zero_calibration(void)
{
    s_st_calcDistData.e_status = CALI_ZERO;
    s_st_calcDistData.u32_calcDistValue = 0;
    s_st_calcDistData.u32_calcDistZero = 0;
    s_st_calcDistData.u32_cnt = 0;
    s_st_calcDistData.u32_lockCnt = 0;
}

static void grd_set_calc_dist_zero_point(uint32_t value)
{
#ifdef JYDS
    return;
#endif
    s_st_calcDistData.u32_calcDistZero = value;
}


static uint32_t grd_calc_dist_get_avg_value(uint32_t *u32_dist)
{
    uint32_t u32_data2 = 0;
    uint32_t u32_i;
    
    for(u32_i = 0; u32_i < CALC_DIST_RAW_DATA_MAX_RECORD; u32_i++)
    {
        u32_data2 += ((s_st_calcDistData.u8_rawData[u32_i][0] << 16) | 
                     (s_st_calcDistData.u8_rawData[u32_i][1] << 8) | 
                     (s_st_calcDistData.u8_rawData[u32_i][2] << 0));
    }
    u32_data2 = u32_data2 / CALC_DIST_RAW_DATA_MAX_RECORD;

    if (u32_data2 > s_st_calcDistData.u32_calcDistZero)
    {
        *u32_dist = (u32_data2 - s_st_calcDistData.u32_calcDistZero) * 3 / 2;
    }
    else
    {
        *u32_dist = 0;
    }

    return u32_data2;
}


static void grd_init_rc_frq_mask_func(void)
{
    BB_DtSendToBuf(DT_NUM_GRD_MASK_CODE, (uint8_t *)&context.st_chSelectionParam.u8_rcChSelectionEnable);
}


static void grd_disable_enable_rc_frq_mask_func(uint8_t flag)
{
    context.st_chSelectionParam.u8_rcChSelectionEnable = ((flag) ? 1 : 0);

    DLOG_Info("%s rc frq mask", flag ? "Enable" : "disable");
}


int grd_GetDistAverage(int *pDist)
{
    if ( s_st_calcDistData.e_status == CALC_DIST )
    {
        *pDist = s_st_calcDistData.u32_calcDistValue;  //
        return 1;
    }
    return 0;
}

static void grd_ChgRcRate(uint8_t rate)
{
    if (0 == rate)
    {
        BB_WriteRegMask(PAGE2, 0x05, 0xC0,0XC0); //reduce rc err count,max error 2 
        BB_WriteReg(PAGE2, 0x04, 0x38); // BPSK 1/2
    }
    else if(1 == rate)
    {
        BB_WriteReg(PAGE2, 0x04, 0x79); // QPSK 2/3
        BB_softReset(BB_GRD_MODE);
    }
    else if(2 == rate)
    {
        BB_WriteRegMask(PAGE2, 0x05, 0xC0,0XC0);
        BB_WriteReg(PAGE2, 0x04, 0x78); // QPSK 1/2
    }
    else if(3 == rate)
    {
        BB_WriteReg(PAGE2, 0x04, 0x39); // BPSK 2/3
        BB_softReset(BB_GRD_MODE);
    }
    else
    {
        ;
    }
    context.uplink_qam_mode = rate;

    //BB_softReset(BB_GRD_MODE);    
}

static void grd_getSignalStatus(void)
{
    stru_grdstatus.u16_grd_snr = grd_get_it_snr();
    stru_grdstatus.agc_value1 = BB_ReadReg(PAGE2, AAGC_2_RD);
    stru_grdstatus.agc_value2 = BB_ReadReg(PAGE2, AAGC_3_RD);
	context.ldpc_error_cnt = (((uint16_t)BB_ReadReg(PAGE2, LDPC_ERR_HIGH_8)) << 8) | BB_ReadReg(PAGE2, LDPC_ERR_LOW_8);
}


static int grd_aoc_prejudge_ldpc(void)
{
    uint16_t ldpc_error;
    int ret = 0;
    static int no_ldpc_err_cnt = 0;
    const uint8_t total_packet_num[4] = {32,64,128,192};
    uint8_t qam,bw;
    
    ldpc_error   = (((uint16_t)BB_ReadReg(PAGE2, LDPC_ERR_HIGH_8)) << 8) | BB_ReadReg(PAGE2, LDPC_ERR_LOW_8);
    qam = grd_get_IT_QAM();
    if(context.st_bandMcsOpt.e_bandwidth == BW_20M)
    {
        bw = 2;
    }
    else
    {
        bw = 1;
    }
    
    ldpc_error = ldpc_error*100/(total_packet_num[qam]*bw);//ldpc error rate percent

    if(ldpc_error > context.aoc.u16_ldpcThd)
    {
        ret = 1;
        no_ldpc_err_cnt = 0;
    }
    else if(ldpc_error > 0)
    {
        no_ldpc_err_cnt=0;
    }
    else
    {
        no_ldpc_err_cnt++;
    }

    if (no_ldpc_err_cnt >= context.aoc.u16_ldpcStacCnt)
    {
        no_ldpc_err_cnt = 0; // restart

        ret = -1;
    }

    return ret;
}


static int grd_aoc_prejudge_snr(void)
{
    #define MAX_NUM  64
    static uint16_t snr[MAX_NUM];
    static uint8_t index = 0;
    static uint8_t is_moving_avg = 0;
    uint32_t avr1 = 0;
    uint8_t i = 0;
    int ret = 0;

    snr[index % context.aoc.u8_snrAvgCnt] = stru_grdstatus.u16_grd_snr;
    index += 1;

    if (index >= context.aoc.u8_snrAvgCnt)
    {
        index = 0; // restart
        is_moving_avg = 1;
    }

    if(is_moving_avg)
    {
        avr1 = 0;
        for(i=0; i<context.aoc.u8_snrAvgCnt; i++)
        {
            avr1 += snr[i];
        }
        avr1 /= context.aoc.u8_snrAvgCnt;

        if(avr1 < (context.aoc.u16_snrThdL))
        {
            ret = 1;
        }
        else if (avr1 > (context.aoc.u16_snrThdH))
        {
            ret = -1;
        }
        else
        {
            ret = 0;
        }

    }

    return ret;
}

static int grd_aoc_prejudge_agc(void)
{
    static uint8_t rx1_gain[128];
    static uint8_t rx2_gain[128];
    static uint8_t index = 0;
    static uint8_t is_moving_avg = 0;
    uint16_t avr1 = 0;
    uint16_t avr2 = 0;
    uint8_t value = 0;
    uint8_t i = 0,agca,agcb;
    int ret = 0;

    agca = BB_RssiOffset(stru_grdstatus.agc_value1);
    agcb = BB_RssiOffset(stru_grdstatus.agc_value2);

    rx1_gain[index % context.aoc.u8_agcAvgCnt] = agca ;
    rx2_gain[index % context.aoc.u8_agcAvgCnt] = agcb ;
    index += 1;

    if (index >= context.aoc.u8_agcAvgCnt)
    {
        index = 0; // restart
        is_moving_avg = 1;
    }

    if(is_moving_avg)
    {
        avr1 = 0;
        for(i=0; i<context.aoc.u8_agcAvgCnt; i++)
        {
            avr1 += rx1_gain[i];
        }
        avr1 /= context.aoc.u8_agcAvgCnt;

        avr2 = 0;
        for(i=0; i<context.aoc.u8_agcAvgCnt; i++)
        {
            avr2 += rx2_gain[i];
        }
        avr2 /= context.aoc.u8_agcAvgCnt;

        value = ((avr1 <= avr2) ? (avr1) : (avr2));
        
        if(value < (context.aoc.u8_agcThdL))
        {
            ret = -1;
        }
        else if (value > (context.aoc.u8_agcThdH))
        {
            ret = 1;
        }
        else
        {
            ret = 0;
        }
    }

    return ret;
}

static int grd_aoc_judge(void)
{
    int agc_ret = grd_aoc_prejudge_agc();
    int snr_ret = grd_aoc_prejudge_snr();
    int ldpc_ret = grd_aoc_prejudge_ldpc();
    static uint8_t is_aoc_runnig = 0;
    static uint8_t delay_cnt = 0;
    
    #define AOC_STEP_INTERVAL    (8)
    if(++delay_cnt >= AOC_STEP_INTERVAL)
    {
        delay_cnt= 0;
        is_aoc_runnig = 0;
    }

    //DLOG_Info("agc=%d,snr=%d,ldpc=%d",agc_ret,snr_ret,ldpc_ret);
    
    if ((-1 == agc_ret) && (-1 == snr_ret)&& (-1 == ldpc_ret))
    //if ((-1 == agc_ret))
    {
        if(is_aoc_runnig)
        {
            return 0;
        }
        is_aoc_runnig = 1;
        delay_cnt = 0;
        return -1;
    }
	else if(1 == agc_ret || snr_ret == 1 || ldpc_ret == 1)
    //else if(1 == agc_ret)
    {
        if(is_aoc_runnig)
        {
            return 0;
        }
        is_aoc_runnig = 1;
        delay_cnt = 0;
        return 1;
    }
    else
    {
        return 0;
    }

}

static int grd_aoc_tx_msg(int power)
{
    int ret = -1;
    if(power != 0)
    {
        BB_DtSendToBuf(DT_NUM_AOC_ADJUST, (uint8_t *)(&power));
        ret = 0;
    }

    return ret;
}

static void grd_aoc(void)
{
    int flag = 0;

    flag = ((1 == (context.aoc.u8_aocEnable)) ? (grd_aoc_judge()): (0));
    if(flag != 0)
    {
        grd_aoc_tx_msg(flag);
    }
}

static int grd_handle_cmd_ack(uint8_t *arg)
{
    int ret = -1;
    uint32_t addr;
    uint8_t  len;

    if(0 == BB_GetDtInfo((ENUM_DT_NUM)(arg[0]), &len, &addr, NULL))
    {
        if(0 == memcmp((uint8_t *)addr, &arg[1], len))
        {
            ret = BB_DtStopSend((ENUM_DT_NUM)(arg[0]));
        }
        else
        {
            uint8_t *pdata = (uint8_t *)addr;
        }
    }
    else
    {
        DLOG_Info("DT=%d", arg[0]);
    }

    return ret;
}

static void BB_grd_NotifyItFreqByValue(uint8_t *u8_itFrq)
{
    BB_DtSendToBuf(DT_NUM_IT_FRQ, u8_itFrq);
}

void BB_grd_NotifyItFreqByCh(ENUM_RF_BAND band, uint8_t u8_ch)
{
    BB_WriteRegMask(PAGE2, GRD_SKY_IT_CH_SYNC, u8_ch, GRD_SKY_IT_CH_MASK);
}

static void grd_enter_slave_mode(uint8_t *rcid,uint8_t *vtid)
{
    TIM_StopTimer(timer2_3);
    //vt_info.itHopMode = context.itHopMode;
    vt_info.valid = 1;
    //context.itHopMode = MANUAL;
    BB_WriteReg(PAGE2, 0x02, 0x06);
    BB_SetTrxMode(BB_RECEIVE_ONLY_MODE);
    BB_softReset(BB_GRD_MODE);

    Grd_Timer2_3_Init();
    NVIC_EnableIRQ(TIMER_INTR23_VECTOR_NUM);
    TIM_StartTimer(timer2_3);

    if(rcid[0] == 0xff && rcid[1] == 0xff && rcid[2] == 0xff && rcid[3] == 0xff && rcid[4] == 0xff && 
        vtid[0] == 0xff && vtid[1] == 0xff)
    {
        context.hashVtid[0] = 0xff;
        context.hashVtid[1] = 0xff;
    }
    else
    {
        BB_GenHashId(rcid,vtid,BB_SKY_MODE);
    }
    //context.vtid[0] = vtid >> 8;
    //context.vtid[1] = vtid;
    context.cur_IT_ch = 0;
    BB_set_ItFrqByCh(context.e_curBand,context.cur_IT_ch);
    //BB_set_power(context.e_curBand,0);
    BB_softReset(BB_GRD_MODE);
    Grd_Timer2_2_Init();
    NVIC_EnableIRQ(TIMER_INTR22_VECTOR_NUM);
    TIM_StartTimer(timer2_2);
    
    context.slave_flag = 1;
    DLOG_Critical("slave mode vtid=%x:%x,%d",vtid[0],vtid[1],context.itHopMode);

}

static void grd_vt_mode_proc(uint8_t value)
{
    if(1 == value)
    {
        TIM_StopTimer(timer2_3);
        vt_info.itHopMode = context.itHopMode;
        vt_info.valid = 1;
        context.itHopMode = MANUAL;
        BB_WriteReg(PAGE2, 0x02, 0x06);
        //BB_SetTrxMode(BB_RECEIVE_ONLY_MODE);
        BB_softReset(BB_GRD_MODE);
        DLOG_Critical("enter pure vt mode.");

        Grd_Timer2_3_Init();
        NVIC_EnableIRQ(TIMER_INTR23_VECTOR_NUM);
        TIM_StartTimer(timer2_3);
    }
    else if(0 == value)
    {
        context.itHopMode = vt_info.itHopMode;
        vt_info.valid = 0;
        BB_WriteReg(PAGE2, 0x02, 0x15);
        //BB_SetTrxMode(BB_NORMAL_MODE);
        BB_softReset(BB_GRD_MODE);
        //DLOG_Warning("exit pure vt mode.");

        NVIC_DisableIRQ(TIMER_INTR23_VECTOR_NUM);
        TIM_StopTimer(timer2_3);
    }
    else
    {
        DLOG_Warning("value error.");
    }
}


/*
 * data only valid if Lock and vt id match
*/
static int grd_lock_status(void *p)
{
    return (0x03 == context.locked);
}

static int grd_rc_mod_chg_process(void)
{
    if(rc_mod_chg_delay.valid)
    {
        if(context.sync_cnt == rc_mod_chg_delay.cnt)
        {
            rc_mod_chg_delay.valid = 0;

            if( context.u8_rcValue[0] || context.u8_rcValue[1] || context.u8_rcValue[2] || \
                context.u8_rcValue[3] || context.u8_rcValue[4])
            {
                BB_write_RcRegs( (uint8_t *)(context.u8_rcValue) );
                context.rcHopMode = MANUAL;
            }
            else //if context.u8_rcValue[0~4] ==0, means the auto mode
            {
                context.rcHopMode = AUTO;
            }
        }
    }
}

static int grd_write_sync_cnt(void)
{
    context.sync_cnt += 1;
    
    BB_WriteReg(PAGE3, SYNC_CNT, context.sync_cnt);
}

static void grd_BB_write_ItRegs(uint8_t *u8_it)
{
    uint16_t frq = 0;

    RF_RegValue2Frq(u8_it, &frq);
    BB_FilterSet(BB_GetFilterByFrq(frq), 1);
    BB_write_ItRegs(u8_it);

    DLOG_Warning("it 0x%x 0x%x 0x%x 0x%x %d", u8_it[0], u8_it[1], u8_it[2], u8_it[3], frq);

}

static void grd_cmd_select_filter(uint8_t value)
{
    uint8_t par[2];
    uint8_t flag = 0;
    
    if(0 == value) // auto selsect
    {
        BB_selectBestCh( RF_600M, SELECT_MAIN_OPT, &filter_chg_it_ch, NULL, NULL, 0 );
        flag = 1;
    }
    else if((value >= 1) && (value <= 5))
    {
        filter_chg_it_ch = (value - 1) * 2;
        flag = 1;
    }
    else
    {
        DLOG_Warning("error value:%d", value);
    }

    if(flag)
    {
        filter_chg_delay.valid = 1;
        filter_chg_delay.cnt = context.sync_cnt + STATUS_CHG_DELAY;
        par[0] = filter_chg_it_ch;
        par[1] = filter_chg_delay.cnt;
        BB_DtSendToBuf(DT_NUM_FILTER_CHG, par);
    }
}

static int grd_filter_chg_process_600m(void)
{
    if(filter_chg_delay.valid)
    {
        if(context.sync_cnt == filter_chg_delay.cnt)
        {
            filter_chg_delay.valid = 0;

            BB_SetNewItCh_600m(filter_chg_it_ch);
            context.sky_rc_channel = context.rc_start;
            DLOG_Warning("rc ch:%d", context.sky_rc_channel);
        }
    }
}

static int grd_sub_band_check(void)
{
    if((0 == band_mode_chg_delay.valid) 
        && (context.locked) 
        && (context.sub_band_value != context.cur_IT_ch))
    {
        DLOG_Warning("subBB it %d->%d",context.sub_band_value,context.cur_IT_ch);
        context.sub_band_value = context.cur_IT_ch;
        grd_sub_band_notify(context.sub_band_value);
    }

    return 0;
}

static int grd_sub_band_notify(uint8_t value)
{
    uint8_t par[2];
    
    band_mode_chg_delay.valid = 1;
    band_mode_chg_delay.cnt = context.sync_cnt + STATUS_CHG_DELAY;
    par[0] = (uint8_t)value;
    par[1] = band_mode_chg_delay.cnt;
    BB_DtSendToBuf(DT_NUM_BAND_MODE_CHG, par);
    DLOG_Warning("subBand %d %d", par[0], par[1]);
    return 0;
}
static void grd_sub_band_excute(uint8_t value)
{
    if( !context.locked)
    {
        DLOG_Warning("nolock, %d-%d",value,context.cur_IT_ch);
        //return;
    }
    //DLOG_Warning("4.: sub band %d->%d", context.sub_band_value,context.cur_IT_ch);
    context.sub_rc_start = BB_GetSubBandStartCH(context.e_curBand,context.st_bandMcsOpt.e_bandwidth,value);
    context.sub_rc_end = BB_GetSubBandEndCH(context.e_curBand,context.st_bandMcsOpt.e_bandwidth,value);
    context.grd_rc_channel = context.sub_rc_start;
    BB_set_Rcfrq(context.e_curBand, context.grd_rc_channel);
    //BB_WriteRegMask(PAGE2, GRD_SKY_RC_CH_SYNC, grd_rc_channel, GRD_SKY_RC_CH_MASK);
    //grd_set_ItFrq(context.e_curBand, value);
    //BB_grd_NotifyItFreqByCh(context.e_curBand,value);
}

static int grd_sub_band_run(void)
{
    if(band_mode_chg_delay.valid)
    {
        if(band_mode_chg_delay.cnt == context.sync_cnt)
        {
            DLOG_Warning("subBand sync_cnt:%d cur_IT_ch:%d e_curBand:%d rc_start:%d rc_end:%d",
                context.sync_cnt,
                context.cur_IT_ch,
                context.e_curBand,
                context.sub_rc_start,
                context.sub_rc_end);
            grd_sub_band_excute(context.sub_band_value);
            band_mode_chg_delay.valid = 0;
        }
    }
    
    return 0;
}

static void reset_sub_band_notice(void)
{
    band_mode_chg_delay.valid = 0;
    BB_DtStopSend(DT_NUM_BAND_MODE_CHG);
}

static void grd_limit_dist_process(void)
{
    static uint8_t zero_cali = 1, output_video = 1;
    
    if(zero_cali)
    {
        zero_cali = 0;
        //grd_calc_dist_zero_calibration();
        s_st_calcDistData.u32_calcDistZero = 1600;
        DLOG_Warning("default zeroDist 1600");
        //return;
    }

    #define LONG_DIST_DISABLE_THRES 3200
    #define SHORT_DIST_ENABLE_THRES 3000
    if(s_st_calcDistData.u32_calcDistValue > LONG_DIST_DISABLE_THRES && output_video)
    {
        BB_SPI_WriteByteMask(PAGE1,0x8d,0,0xc0);
        output_video = 0;
        DLOG_Warning("%d m output video disable",s_st_calcDistData.u32_calcDistValue);
    }
    else if(s_st_calcDistData.u32_calcDistValue < SHORT_DIST_ENABLE_THRES && !output_video)
    {
        BB_SPI_WriteByteMask(PAGE1,0x8d,0xc0,0xc0);
        output_video = 1;
        DLOG_Warning("%d m output video enable",s_st_calcDistData.u32_calcDistValue);
    }

}

