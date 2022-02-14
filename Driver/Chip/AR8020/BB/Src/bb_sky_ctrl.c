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
#include "memory_config.h"
#include "pwr_ctrl.h"
#include "filter.h"
#include "hal.h"

#define     SDRAM_ENC_DEEP_REG_ADDR     (0xA0010000)  // master

#define     VSOC_SOFT_RESET             (0xA0030034)

typedef struct
{
    uint8_t  u8_flagOffset;  //offset have got
    uint8_t  u8_softEnable;

    int64_t  ll_offset;
    int      basefrq;
    int      baseOffset;
}STRU_SKY_FREQ_OFFSET;

const uint8_t bb_aes_off_max_uart_len[4] = {BPSK1_2_LEN,QPSK2_3_LEN,QPSK1_2_LEN,BPSK2_3_LEN};

/*#define MAX_FIND_SAME_DEV_NUM (32)
typedef struct
{
    uint8_t  id[MAX_FIND_SAME_DEV_NUM][RC_ID_SIZE];
    uint8_t  index;
}STRU_SKY_FIND_SAME_DEV_ID;

static STRU_SKY_FIND_SAME_DEV_ID stru_sky_find_dev_id;*/

STRU_SKY_STATUS stru_skystatus;
static init_timer_st sky_timer2_5;

static STRU_DELAY_CNT rc_mod_chg_delay = {0, 0};
static STRU_DELAY_CNT filter_chg_delay = {0, 0};
static STRU_DELAY_CNT band_mode_chg_delay = {0, 0};
static uint8_t filter_chg_it_ch = 0;
uint8_t u8_unlockCntFromLockToUnlock = 64;

extern uint8_t g_syn;
extern STRU_PURE_VT_INFO vt_info;
extern STRU_AGC_SET *pstru_agc_set_cfg;
extern STRU_PWR_CTRL *pstru_pwr_ctrl_cfg;
extern uint8_t sub_band_2g_agc_gain;//0 near , 1 far
extern uint8_t sub_band_5g_agc_gain;

static uint8_t enable_search_id_by_rssi;
static uint8_t search_id_by_rssi_timeout_flag;
static uint32_t non_lbt_optch_rcnt=0,non_lbt_optch_wcnt=0;
static uint8_t non_lbt_optch;

static void sky_handle_one_cmd(STRU_WIRELESS_CONFIG_CHANGE* pcmd);
static void sky_handle_one_rf_cmd(STRU_WIRELESS_CONFIG_CHANGE* pcmd);
static void sky_handle_all_cmds(void);
static void sky_handle_all_rf_cmds(void);

static void sky_calc_dist(void);


static uint16_t sky_get_snr_average(void);

static void sky_clearFrqOffset(void);

static void sky_Timer2_6_Init(void);
static void Sky_TIM2_6_IRQHandler(uint32_t u32_vectorNum);

static void wimax_vsoc_rx_isr(uint32_t u32_vectorNum);

static void sky_getSignalStatus(void);

static void sky_getGroudRcId(uint8_t* pu8_rcId);

static void BB_skyPlot(void);

static uint8_t get_rc_status(void);

static void sky_handle_all_spi_cmds(void);

static void sky_set_McsByIndex(ENUM_CH_BW bw, uint8_t mcs);

void sky_ChgRcRate(uint8_t rate);

static void BB_sky_uartDataHandler(void);

static void sky_aoc_execute(void *power);

static void sky_Timer2_5_Init(void);

static void sky_vt_mode_proc(uint8_t value);

static int sky_lock_status(void *p);

static int sky_rc_mod_chg_process(void);

static int sky_filter_chg_process(void);

static void sky_auto_adjust_agc_gain(void);

static void sky_handle_agc_gear_chg_cmd( uint8_t gear);

static void sky_agc_set_init(void);

static uint8_t sky_BB_write_RcRegs(uint8_t *u8_rc);

static void sky_restore_rf_frq(void);

static void sky_handle_it_ch_sync_cmd(void);

static int sky_band_mode_run(void);

static void sky_handle_sub_band_set_cmd(uint8_t *arg);

void BB_sky_nackVtidMessage(uint8_t *pu8_vtid);

void BB_sky_setSearchState(ENUM_SKY_SEARCH_STATE state);

static uint8_t sky_add_same_dev_id(uint8_t *rcid);

static uint16_t sky_read_usb0_senddata(void);

static uint16_t sky_read_usb1_senddata(void);

static void sky_vtSkip_process(void);


//static void check_pwr_mode_status(void);


void BB_SKY_start(void)
{
    context.rcHopMode = AUTO;
    context.cur_IT_ch = DEFAULT_IT_CH;
#ifdef JYDS
	context.st_bandMcsOpt.e_rfbandMode = MANUAL;
#else
    context.st_bandMcsOpt.e_rfbandMode = AUTO; //sky defualt auto
#endif
    BB_set_ItFrqByCh(context.e_curBand, context.cur_IT_ch);

    if(context.uplink_qam_mode != 0 && context.uplink_qam_mode < 4 ){
        sky_ChgRcRate(context.uplink_qam_mode);
        context.RcChgRate.default_qam_value = context.uplink_qam_mode;
        DLOG_Warning("up %d",context.uplink_qam_mode);
    }
    sky_agc_set_init();
    BB_SetTrxMode(BB_RECEIVE_ONLY_MODE); // receive only

    sky_Timer2_6_Init();

    sky_Timer2_5_Init();

    reg_IrqHandle(BB_RX_ENABLE_VECTOR_NUM, wimax_vsoc_rx_isr, NULL);
    NVIC_SetPriority(BB_RX_ENABLE_VECTOR_NUM, NVIC_EncodePriority(NVIC_PRIORITYGROUP_5,INTR_NVIC_PRIORITY_BB_RX,0));

    BB_SetAesOffUartMaxLen(bb_aes_off_max_uart_len[context.uplink_qam_mode]);
    BB_ComInit(sky_lock_status,context.aes_off);    

    //BB_RcSelectionInit();

    sky_setRcId((uint8_t *)context.hashRcid);
    BB_sky_vtId((uint8_t *)context.hashVtid);

    NVIC_EnableIRQ(BB_RX_ENABLE_VECTOR_NUM);

    sky_startSweep(context.e_curBand);

    context.dev_state = CHECK_LOCK;
    context.qam_ldpc  = context.u8_bbStartMcs;
    context.stru_bandSwitchParam.u8_bandSwitchAfterUnlock = (pstru_pwr_ctrl_cfg->pwr_ctrl == PWR_LEVEL0);

    sky_set_McsByIndex(context.st_bandMcsOpt.e_bandwidth, context.qam_ldpc);
    context.realtime_mode = 0;
    PWR_CTRL_Init(pstru_pwr_ctrl_cfg);
}


/*
 * 1: lock status
 * 0: unlock
*/
uint8_t sky_checkRcLock(uint8_t u8_lockStatus)
{
    //sky in search mode
    if (BB_sky_isSearching())
    {
        return (SKY_CRC_OK(u8_lockStatus) && (0 == stru_skystatus.flag_errorConnect));
    }
    else
    {
        return SKY_ID_CRC_MATCH(u8_lockStatus);
    }
}

/*
 * switch RC channel and AGC each 1s
*/
static void sky_checkRcUnlockTimeSwitchChannelAgc(uint8_t u8_lockStatus)
{
    static uint32_t ticks = 0;
    uint32_t feq_num = 0;
    uint32_t timer = 0;
    uint32_t now = SysTicks_GetTickCount();

#ifdef RF_9363
    //if(RF_600M == context.e_curBand)
    {
        feq_num = BB_GetRcFrqNumPerFilter();
        timer = feq_num*14;
    }
#endif

#ifdef RF_8003X
    //else
    {
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

        timer = (feq_num + 1) * 14;
        //DLOG_Warning("feq_num = %d", feq_num);
    }
#endif

    if (SysTicks_GetDiff(ticks, now) > timer)
    {
        ticks = now;
        if(context.rcHopMode == AUTO)
        {
            #ifdef RF_9363
            //if(RF_600M == context.e_curBand)
            {
                sky_rcHopFreq600m(feq_num + 1);
            }
            #endif

            #ifdef RF_8003X
            //else
            {
                sky_rcHopFreq();
            }
            #endif

        }

        sky_agcGainToggle();
        sky_soft_reset();
    }
}

/*
 * agc toggle only necessary when unlock status
*/
void sky_agcGainToggle(void)
{
    if(FLAG_VALID != stru_skystatus.agc_auto_chg_flag)
    {
        return;
    }

    if(AGC_GEAR_1 == stru_skystatus.fct_agc_gear)
    {
        //BB_SetAgcGain(context.e_curBand, AAGC_GAIN_FAR);
        BB_SetAgcGain(RF_2G, AAGC_GAIN_FAR);
        BB_SetAgcGain(RF_5G, AAGC_GAIN_FAR);
        stru_skystatus.en_agcmode = FAR_AGC;
        DLOG_Info("FAR_AGC");
        return;
    }
#ifdef RFSUB_BAND

    if(context.freq_band_mode == SUB_BAND)
    {
        if(context.e_curBand == RF_2G)
        {
            if(sub_band_2g_agc_gain)
            {
                BB_SetAgcGain(RF_2G, AAGC_GAIN_NEAR);
                BB_SetAgcGain(RF_5G, AAGC_GAIN_NEAR);
                stru_skystatus.en_agcmode = NEAR_AGC;
                DLOG_Info("NEAR_AGC");
            }
            else
            {
                BB_SetAgcGain(RF_2G, AAGC_GAIN_FAR);
                BB_SetAgcGain(RF_5G, AAGC_GAIN_FAR);
                stru_skystatus.en_agcmode = FAR_AGC;
                DLOG_Info("FAR_AGC.");
            }
        }
        else if(context.e_curBand == RF_5G)
        {
            if(sub_band_5g_agc_gain)
            {
                BB_SetAgcGain(RF_2G, AAGC_GAIN_NEAR);
                BB_SetAgcGain(RF_5G, AAGC_GAIN_NEAR);
                stru_skystatus.en_agcmode = NEAR_AGC;
                DLOG_Info("NEAR_AGC");
            }
            else
            {
                BB_SetAgcGain(RF_2G, AAGC_GAIN_FAR);
                BB_SetAgcGain(RF_5G, AAGC_GAIN_FAR);
                stru_skystatus.en_agcmode = FAR_AGC;
                DLOG_Info("FAR_AGC.");
            }
        }
        return;
    }
#endif
    if(FAR_AGC == stru_skystatus.en_agcmode)
    {
        //BB_SetAgcGain(context.e_curBand, AAGC_GAIN_NEAR);
        BB_SetAgcGain(RF_2G, AAGC_GAIN_NEAR);
        BB_SetAgcGain(RF_5G, AAGC_GAIN_NEAR);
        stru_skystatus.en_agcmode = NEAR_AGC;
        DLOG_Info("NEAR_AGC");
    }
    else
    {
        //BB_SetAgcGain(context.e_curBand, AAGC_GAIN_FAR);
        BB_SetAgcGain(RF_2G, AAGC_GAIN_FAR);
        BB_SetAgcGain(RF_5G, AAGC_GAIN_FAR);
        stru_skystatus.en_agcmode = FAR_AGC;
        DLOG_Info("FAR_AGC.");
    }
}

void sky_getAgcStatus(void)
{
    uint8_t i = 0;
    uint16_t sum_1 = 0, sum_2 = 0;
    uint8_t  aver1, aver2;
    uint8_t  num = sizeof(stru_skystatus.agc_value1);

    uint8_t  *pu8_ptr1 = stru_skystatus.agc_value1;
    uint8_t  *pu8_ptr2 = stru_skystatus.agc_value2;

    for( i = 0 ; i < num ; i++)
    {
        sum_1 += *(pu8_ptr1 + i);
        sum_2 += *(pu8_ptr2 + i);
    }

    aver1 = sum_1 / num;
    aver2 = sum_2 / num;

    {
        static int count1 = 0;
        if ( count1 ++ > 100)
        {
        	stru_skystatus.sky_ready_check_agc = 1;
            count1 = 0;
            STRU_skyAgc agc = {
                .u8_skyagc1 = aver1,
                .u8_skyagc2 = aver2,
                .snr        = sky_get_rc_snr(),
                .u8_tx_pwr	= context.u8_TargetPower[0],
            };

            BB_Session0SendMsg(DT_NUM_SKY_AGC_STATUS, (uint8_t *)&agc, sizeof(agc));
        }
    }
}


void sky_notify_encoder_brc(uint8_t u8_ch, uint8_t bridx)
{
    STRU_SysEvent_BB_ModulationChange event;
    event.encoder_brcidx = bridx;
    if (0 == u8_ch)
    {
        event.u8_bbCh = 0;
    }
    else
    {
        event.u8_bbCh = 1;
    }
	event.bw = context.st_bandMcsOpt.e_bandwidth;
    SYS_EVENT_NotifyInterCore(SYS_EVENT_ID_BB_SUPPORT_BR_CHANGE, (void*)&event);
    DLOG_Info("ch%d brc =%d \n", u8_ch, bridx);
}



void sky_set_McsByIndex(ENUM_CH_BW bw, uint8_t mcs)
{
    uint8_t value;
    uint8_t mcs_idx_reg0x0f_map[] =
    {
        0x4f,
        0x8f,
        0x5f,
        0x3f,
        0x3f,
        0x2f
    };

    uint8_t map_idx_to_mode_10m[] =
    {
        ((MOD_BPSK<<6)  | (BW_10M <<3)  | LDPC_1_2), //
        ((MOD_BPSK<<6)  | (BW_10M <<3)  | LDPC_1_2),
        ((MOD_4QAM<<6)  | (BW_10M <<3)  | LDPC_1_2),
        //((MOD_4QAM<<6)  | (BW_10M <<3)  | LDPC_2_3),
        ((MOD_16QAM<<6) | (BW_10M <<3)  | LDPC_1_2),
        //((MOD_16QAM<<6) | (BW_10M <<3)  | LDPC_2_3),
        ((MOD_64QAM<<6) | (BW_10M <<3)  | LDPC_1_2),
        ((MOD_64QAM<<6) | (BW_10M <<3)  | LDPC_2_3),
    };

    uint8_t map_idx_to_mode_20m[] =
    {
        ((MOD_BPSK<<6)  | (BW_20M <<3)  | LDPC_1_2), //
        ((MOD_BPSK<<6)  | (BW_20M <<3)  | LDPC_1_2),
        ((MOD_4QAM<<6)  | (BW_20M <<3)  | LDPC_1_2),
        ((MOD_16QAM<<6) | (BW_20M <<3)  | LDPC_1_2),
        ((MOD_16QAM<<6) | (BW_20M <<3)  | LDPC_2_3),
    };

    if ((BW_10M == bw && mcs >= sizeof(map_idx_to_mode_10m)) || (BW_20M == bw && mcs >= sizeof(map_idx_to_mode_20m)))
    {
        return;
    }

    BB_WriteReg( PAGE2, 0x0f, mcs_idx_reg0x0f_map[mcs] );
    if (BW_10M == bw)
    {
        BB_WriteReg( PAGE2, TX_2, map_idx_to_mode_10m[mcs]);
    }
    else
    {
        BB_WriteReg( PAGE2, TX_2, map_idx_to_mode_20m[mcs]);
    }

    //if ( context.brc_mode == AUTO )
    {
        sky_notify_encoder_brc(0, BB_get_bitrateByMcs(bw, mcs));
        sky_notify_encoder_brc(1, BB_get_bitrateByMcs(bw, mcs));
    }
}



//*********************TX RX initial(14ms irq)**************
static void wimax_vsoc_rx_isr(uint32_t u32_vectorNum)
{
    NVIC_DisableIRQ(BB_RX_ENABLE_VECTOR_NUM);
    STRU_WIRELESS_INFO_DISPLAY *osdptr = (STRU_WIRELESS_INFO_DISPLAY *)(SRAM_BB_STATUS_SHARE_MEMORY_ST_ADDR);
    STRU_DEVICE_INFO *pst_devInfo = (STRU_DEVICE_INFO *)(DEVICE_INFO_SHM_ADDR);

    if( context.u8_flagdebugRequest & 0x80)
    {
        context.u8_debugMode = (context.u8_flagdebugRequest & 0x01);

        if( context.u8_debugMode )
        {
            osdptr->head = 0x00;
            osdptr->tail = 0xff;    //end of the writing
        }

        context.u8_flagdebugRequest = 0;

        if (context.u8_debugMode == TRUE)
        {
            BB_SetTrxMode(BB_NORMAL_MODE);
            BB_SPI_DisableEnable(0);
        }
        else
        {
            BB_SPI_DisableEnable(1);
        }

        osdptr->in_debug     = context.u8_debugMode;
        pst_devInfo->isDebug = context.u8_debugMode;
    }

    NVIC_EnableIRQ(TIMER_INTR26_VECTOR_NUM);
    TIM_StartTimer(stru_skystatus.sky_timer2_6);

    NVIC_EnableIRQ(TIMER_INTR25_VECTOR_NUM);
    TIM_StartTimer(sky_timer2_5);

    if(context.bandedge_enable && context.delay_it_flag)
    {
        context.delay_it_flag = 0;
        DLOG_Warning("it ch %d -> %d",context.cur_IT_ch,context.delay_it_ch);
        context.vtFreqTime[context.cur_IT_ch] = SysTicks_GetTickCount();//save last main ch time value
    
        context.cur_IT_ch = context.delay_it_ch;
        BB_set_ItFrqByCh(context.e_curBand, context.cur_IT_ch);
        context.sky_sel_vt_ch_flag = 0;
    
        //to fixed grd and sky decide skip vt-ch at the same time
        if(context.vtskip_stat == 1)
        {
            context.vtskip_stat = 0;
            DLOG_Warning("not vt-ch skip");
        }
        if(context.delay_it_ch == 0 || context.delay_it_ch == 6)
        {
            BB_WriteReg(PAGE0, BB_POWER_A, BB_POWER_DEFAULT_VALUE - context.low_power_db*4);
            BB_WriteReg(PAGE0, BB_POWER_B, BB_POWER_DEFAULT_VALUE - context.low_power_db*4);
            BB_set_power(context.e_curBand,context.pwr - context.low_power_db);
            DLOG_Warning("bd low %d",context.low_power_db);
        }
        else
        {
            BB_WriteReg(PAGE0, BB_POWER_A, BB_POWER_DEFAULT_VALUE);
            BB_WriteReg(PAGE0, BB_POWER_B, BB_POWER_DEFAULT_VALUE);
            BB_set_power(context.e_curBand,context.pwr);
            context.u8_aocAdjustPwr = context.pwr;
            DLOG_Warning("bdk to %d",context.pwr);
        }
    }

    if(context.enable_non_lbt)
    {
        sky_vtSkip_process();
    }
    #ifndef NON_LBT_RELEASE
    else
    {
        if(context.fem_close)
        {
            BB_fem_open(0);
            BB_fem_open(1);
            context.fem_close = 0;
            DLOG_Warning("n-lbt fem_open");
        }
    }
    #endif
}

/*
 * get snr, agc, ground rcid.
*/
static void sky_getSignalStatus(void)
{
    UNION_BBSPIComHeadByte head;

    stru_skystatus.u16_sky_snr[stru_skystatus.u8_snr_idx] = sky_get_rc_snr();

    stru_skystatus.agc_curValue1 = BB_ReadReg(PAGE2, AAGC_2_RD);
    stru_skystatus.agc_curValue2 = BB_ReadReg(PAGE2, AAGC_3_RD);

    stru_skystatus.agc_value1[stru_skystatus.agc_idx] = stru_skystatus.agc_curValue1;
    stru_skystatus.agc_value2[stru_skystatus.agc_idx] = stru_skystatus.agc_curValue2;

    stru_skystatus.agc_idx ++;
    if ( stru_skystatus.agc_idx >= sizeof(stru_skystatus.agc_value2))
    {
        stru_skystatus.agc_idx = 0;
        stru_skystatus.agc_full = 1;
    }

    sky_getGroudRcId(stru_skystatus.cur_groundRcid);

    head.u8_headByte = BB_ReadReg(PAGE2, SPI_DT_HEAD_REGISTER);

    stru_skystatus.flag_groundInSearching = head.b.flag_searchingSupport;
    stru_skystatus.flag_groundVtMatch = head.b.vt_match;

    stru_skystatus.flag_errorConnect = BB_sky_isErrorConnectInSearching(stru_skystatus.u8_rcStatus);
    if (0 == stru_skystatus.flag_errorConnect)
    {
        //if sky is searching, check the rc id match to end of the searching
        context.inSearching = BB_sky_inSearching();
        if (context.inSearching)
        {
            BB_sky_checkSearchEnd(stru_skystatus.u8_rcStatus);
        }
    }

    stru_skystatus.ground_LockStatus = BB_ReadRegMask(PAGE2, SPI_DT_HEAD_REGISTER, LOCK_STATUS_MASK);
}
static void sky_command_lna_switch(uint8_t value)
{
    if(context.enable_non_lbt)
    {
        return;
    }

    if(value == 2)
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
    else if(value == 3)
    {
        context.st_mimo_mode.enum_lna_mode = LNA_AUTO;
        if(context.e_curBand == RF_5G)
        {
            return;
        }

        BB_Lna_reset();
        
    }
    else if(value == 5)
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

static void sky_lna_switch(void)
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

    if(context.e_curBand == RF_5G)
    {
        return;
    }

    if(context.st_mimo_mode.enum_lna_mode != LNA_AUTO)
    {
        return;
    }

    if(context.dev_state == LOCK || context.dev_state == WAIT_VT_LOCK)
    {
        BB_Lna_AddAgc(stru_skystatus.agc_curValue1,stru_skystatus.agc_curValue2);
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
            return;
        }
		//else if(status == OPEN_LNA && context.lna_status == BYPASS_LNA)
        else if(status == OPEN_LNA && context.lna_status == BYPASS_LNA && context.swp_bypass == 2)
        {
            BB_open_lna();
            context.lna_status = OPEN_LNA;
            BB_Lna_reset();
            return;
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
static void sky_do_rcRate(void)
{
    if (1 == context.RcChgRate.en_flag)
    {
        if (context.sync_cnt == (context.RcChgRate.cnt) )
        {
            context.RcChgRate.en_flag = 0;
            sky_ChgRcRate(context.RcChgRate.value);
            context.RcChgRate.timeout_cnt = 0;
            DLOG_Warning("0, %d %d",context.RcChgRate.cnt,context.RcChgRate.value);
            context.RcChgRate.en_reset = 1;
            //BB_softTxReset(BB_SKY_MODE);
        }
    }

    if (context.RcChgRate.en_reset)
    {
        context.RcChgRate.timeout_cnt++;
        if(context.dev_state == LOCK){
            context.RcChgRate.timeout_cnt = 0;
            //DLOG_Warning("1");
        }

        if(context.RcChgRate.timeout_cnt > 3){
            context.RcChgRate.en_reset = 0;
            sky_ChgRcRate(context.RcChgRate.default_qam_value);
            DLOG_Warning("reback2 %d",context.RcChgRate.default_qam_value);
            BB_softReset(BB_SKY_MODE);
        }

    }

}


static void sky_do_rc_patten(void)
{
	if (1 == context.rcChgPatten.en_flag)
    {
        if (context.sync_cnt == context.rcChgPatten.timeout_cnt)
        {
        	rc_update_working_patten();
            context.rcChgPatten.en_flag = 0;
            context.rcChgPatten.timeout_cnt = 0;
			context.rf_info.rc_patten_nextchg_delay=SysTicks_GetTickCount();
        }
    }
}

static void sky_do_rf_bw(void)
{
	if (1 == context.rf_bw.en_flag)
    {
        if (context.sync_cnt == context.rf_bw.timeout_cnt)
        {
			
            context.rf_bw.en_flag = 0;
            context.rf_bw.timeout_cnt = 0;
			context.st_bandMcsOpt.e_bandwidth = (ENUM_CH_BW)context.rf_bw.bw;
			RF_GetFctFreqTable(context.st_bandMcsOpt.e_bandwidth);
			reset_sweep_table(context.e_curBand);
			BB_set_RF_bandwitdh(BB_SKY_MODE, (ENUM_CH_BW)context.rf_bw.bw);
			BB_set_ItFrqByCh(context.e_curBand, context.stru_bandChange.u8_ItCh);
			if(context.qam_ldpc!=context.rf_bw.ldpc)
			{
	    		ENUM_RUN_MODE mode = AUTO;
				context.qam_ldpc=context.rf_bw.ldpc;
				sky_handle_mcs_mode_cmd((uint8_t*)&mode);
			}
			BB_softTxReset(BB_SKY_MODE);
			DLOG_Critical("set rf bandwidth=%d",context.st_bandMcsOpt.e_bandwidth);	
        }
    }
}

static void Sky_TIM2_5_IRQHandler(uint32_t u32_vectorNum)
{
    Reg_Read32(BASE_ADDR_TIMER2 + TMRNEOI_5);

    NVIC_DisableIRQ(TIMER_INTR25_VECTOR_NUM);
    TIM_StopTimer(sky_timer2_5);

    if(context.u8_debugMode)
    {
        return;
    }
    sky_handle_all_rf_cmds();

    if(1 == vt_info.valid)
    {
        return;
    }

    if (1 == sky_checkRcLock(stru_skystatus.u8_rcStatus))
    {
        BB_SPIComRecvData();
    }
}

static uint32_t u32_contiousUnlock = 0;
static uint8_t ground_band_switch = 0;
uint8_t flag_rchop = 0;

static void process_searchid_state()
{
	if (sky_checkRcLock(stru_skystatus.u8_rcStatus))
	{
		flag_rchop = 1;
		if(enable_search_id_by_rssi)
		{
			if(search_id_by_rssi_timeout_flag && context.select_obj_vt_ok)
			{
				context.dev_state = CHECK_LOCK;
				DLOG_Warning("SEARCH_ID->CHECK_LOCK");
				DLOG_Warning("rc_set_unlock_patten");
				rc_set_unlock_patten(1);
			}
		}
		else
		{
			context.dev_state = CHECK_LOCK;
			DLOG_Warning("rc_set_unlock_patten");
			DLOG_Warning("SEARCH_ID->CHECK_LOCK");
			rc_set_unlock_patten(1);
		}
	}
	else if (1 == stru_skystatus.flag_errorConnect || SKY_RC_ERR(stru_skystatus.u8_rcStatus))
	{
		sky_soft_reset();
		stru_skystatus.flag_errorConnect = 0;	//clear the error flag, or else will reset again.
		return;
	}
	else
	{
		//if dev state return from other state, it may stop in 5G mode, switch to 2G mode for search
		if (context.st_bandMcsOpt.e_bandsupport == RF_2G_5G && context.st_bandMcsOpt.e_rfbandMode == AUTO)
		{
			sky_rcUnLockHopBand();
		}
		else
		{
			sky_checkRcUnlockTimeSwitchChannelAgc(stru_skystatus.u8_rcStatus);
		}
	}
	BB_SetTrxMode(BB_RECEIVE_ONLY_MODE);

}

static void process_check_lock_state()
{
	#ifdef RFSUB_BAND
	if(context.freq_band_mode == SUB_BAND)
	{
		context.sub_band_value = INVALIDE_SUB_BAND_VALUE;
	}
	#endif
	if (1 == stru_skystatus.flag_errorConnect || SKY_RC_ERR(stru_skystatus.u8_rcStatus))
	{
		sky_soft_reset();

		//DLOG_Warning("reset_flag = 5");
		stru_skystatus.flag_errorConnect = 0;	//clear the error flag, or else will reset again.
	}
	else if (SKY_CRC_OK(stru_skystatus.u8_rcStatus)&& (!SKY_ID_CRC_MATCH(stru_skystatus.u8_rcStatus)) && (!BB_sky_isSearching()))
	{
		STRU_SysEvent_DEV_BB_STATUS lockEvent ={
			.pid		= BB_GOT_ERR_CONNNECT,
			.lockstatus = 1,//stru_skystatus.flag_errorConnect,
		};
		SYS_EVENT_NotifyInterCore(SYS_EVENT_ID_BB_EVENT, (void *)&lockEvent);

		sky_soft_reset();

	}
	else if (sky_checkRcLock(stru_skystatus.u8_rcStatus))
	{
		sky_handle_it_ch_sync_cmd();
		flag_rchop		  = 1;
		context.dev_state = WAIT_VT_LOCK;
		//rc_set_unlock_patten();
		
		DLOG_Warning("CHECK_LOCK->WAIT_VT_LOCK");
		BB_SetTrxMode(BB_NORMAL_MODE);
		if (stru_skystatus.check_lock_times > 0)
		{
		  uint32_t time = HAL_GetSysMsTick();
		  DLOG_Warning("Lock %d: %d ms", stru_skystatus.check_lock_times, time - stru_skystatus.rst_start_time);
		  stru_skystatus.check_lock_times --;
		}
	}
	else
	{
		BB_SetTrxMode(BB_RECEIVE_ONLY_MODE);
		//sky_notify_grd_goto_unlock_patten();
		if (BB_sky_isSearching() && u32_contiousUnlock ++ >= 20)
		{
			u32_contiousUnlock = 0;
			context.dev_state  = SEARCH_ID;
			DLOG_Warning("rc_set_unlock_patten");
			rc_set_unlock_patten(1);
			//sky_notify_grd_goto_unlock_patten();
			DLOG_Warning("CHECK_LOCK-> SEARCH_ID");
		}
		else if ( (context.st_bandMcsOpt.e_rfbandMode == AUTO && context.st_bandMcsOpt.e_bandsupport == RF_2G_5G))	 
		{
			sky_rcUnLockHopBand();				//switch band, toggle agc, change channel each 1s
		}
		else
		{
			sky_checkRcUnlockTimeSwitchChannelAgc(stru_skystatus.u8_rcStatus);
		}
	}

}

static void process_wait_it_lock_state()
{
	 //if Rc Lock, wait until the VT lock, then go to Lock state
	if (sky_checkRcLock(stru_skystatus.u8_rcStatus))
	{
	    u32_contiousUnlock = 0;
	    DLOG_Warning("WAIT_VT_LOCK->Lock");
	    context.dev_state = LOCK;  //go to Lock status
	}
	else
	{
	    DLOG_Info("%d %d %d %d",stru_skystatus.u8_rcStatus,stru_skystatus.flag_errorConnect,stru_skystatus.flag_groundInSearching,context.inSearching);
	}

    if (u32_contiousUnlock >= u8_unlockCntFromLockToUnlock)           //ID_MATCH_LOCK -> CHECK_ID_MATCH
    {
        u32_contiousUnlock = 0;
        context.dev_state  = CHECK_LOCK;
		rc_set_unlock_patten(1);
        BB_SetTrxMode(BB_RECEIVE_ONLY_MODE);
        sky_soft_reset();
        DLOG_Warning("WAIT_VT_LOCK -> CHECK_LOCK");
        //context.rc_skip_patten = 0xff;
    }

	if (stru_skystatus.check_lock_times > 0)
	{
        context.dev_state = CHECK_LOCK;
        stru_skystatus.rst_start_time = HAL_GetSysMsTick();
        sky_soft_reset();
        //DLOG_Warning("rst time: %d", stru_skystatus.rst_start_time);
	}

    flag_rchop = 1;
	//handler in rc lock
    if (u32_contiousUnlock ++ >= u8_unlockCntFromLockToUnlock)           //ID_MATCH_LOCK -> CHECK_ID_MATCH
    {
        u32_contiousUnlock = 0;
        context.dev_state  = CHECK_LOCK;
		DLOG_Warning("rc_set_unlock_patten");
		rc_set_unlock_patten(1);
        BB_SetTrxMode(BB_RECEIVE_ONLY_MODE);
        sky_switchSetPower(context.e_curBand);
        sky_soft_reset();
        //DLOG_Warning("LOCK->CHECK_LOCK reset");
        //context.rc_skip_patten = 0xff;
    }
    else if (sky_checkRcLock(stru_skystatus.u8_rcStatus))
    {
        u32_contiousUnlock = 0;
        sky_handle_all_spi_cmds();
        sky_getAgcStatus();
        sky_auto_adjust_agc_gain();
        if(context.enable_non_lbt)
        {
            if(context.sky_sel_vt_ch_flag)
            {
                BB_set_skySweepVtfrq(context.e_curBand,context.sky_sel_vt_ch);
            }
            else
            {
                BB_set_skySweepVtfrq(context.e_curBand,context.cur_IT_ch);
            }

        }
    }
}

static void process_lock_state()
{
	//if Rc Lock, wait until the VT lock, then go to Lock state
	if (sky_checkRcLock(stru_skystatus.u8_rcStatus))
	{
		u32_contiousUnlock = 0;
	}
	else
	{
		DLOG_Info("%d %d %d %d",stru_skystatus.u8_rcStatus,stru_skystatus.flag_errorConnect,stru_skystatus.flag_groundInSearching,context.inSearching);
	}

	if (stru_skystatus.check_lock_times > 0)
	{
		context.dev_state = CHECK_LOCK;
		//rc_set_unlock_patten();
		stru_skystatus.rst_start_time = HAL_GetSysMsTick();
		sky_soft_reset();
		DLOG_Warning("rst time: %d", stru_skystatus.rst_start_time);
	}
	
	flag_rchop = 1;
	context.stru_bandSwitchParam.u8_bandSwitchAfterUnlock |= 0x80;
	#ifdef RF_9363
	{
		sky_rcFrqStatusStatistics600m();
		sky_restore_rf_frq();
	}
	#endif
	

	if (1==stru_skystatus.flag_errorConnect)
	{
		context.dev_state = CHECK_LOCK;
		DLOG_Warning("rc_set_unlock_patten");
		rc_set_unlock_patten(1);
		//sky_notify_grd_goto_unlock_patten();
		sky_soft_reset();
		//context.rc_skip_patten = 0xff;
	}
	
	//handler in rc lock
	if (u32_contiousUnlock ++ >= u8_unlockCntFromLockToUnlock)			 //ID_MATCH_LOCK -> CHECK_ID_MATCH
	{
		u32_contiousUnlock = 0;
		context.dev_state  = CHECK_LOCK;
		DLOG_Warning("rc_set_unlock_patten");
		rc_set_unlock_patten(1);
		//sky_notify_grd_goto_unlock_patten();
		BB_SetTrxMode(BB_RECEIVE_ONLY_MODE);
		sky_switchSetPower(context.e_curBand);
		sky_soft_reset();
		DLOG_Warning("LOCK->CHECK_LOCK reset");
		//context.rc_skip_patten = 0xff;
	}
	else if (sky_checkRcLock(stru_skystatus.u8_rcStatus))
	{
		u32_contiousUnlock = 0;

		sky_handle_all_spi_cmds();
		sky_getAgcStatus();
		sky_auto_adjust_agc_gain();

		if(context.enable_non_lbt)
		{
			if(context.sky_sel_vt_ch_flag)
			{
				BB_set_skySweepVtfrq(context.e_curBand,context.sky_sel_vt_ch);
			}
			else
			{
				BB_set_skySweepVtfrq(context.e_curBand,context.cur_IT_ch);
			}

		}
	}	
}

static void process_state()
{
	switch (context.dev_state)
    {
        case SEARCH_ID: 
			process_searchid_state();
			break;
		
        case CHECK_LOCK: 
			process_check_lock_state();
			break;
		
        case WAIT_VT_LOCK:
			process_wait_it_lock_state();
			break;
			
        case LOCK:
			process_lock_state();
			break;
			
            break;
       default:
            context.dev_state = CHECK_LOCK;
            break;
    }
}


static void sky_notify_rc_patten()
{
	uint8_t buf[10];
	int i=0;
	static int gap =0;
	#define delay (4)
	gap++;
	if(context.rcChgPatten.en_flag==1 && context.rcChgPatten.valid==1 && gap < delay)
	{
		for(i=0;i<context.rf_info.rc_ch_patten_need_id_size;i++)
		{
			buf[i+2]=context.rcChgPatten.patten[i];
		}
		buf[0]= context.rcChgPatten.timeout_cnt;
		buf[1]= context.rf_info.rc_patten_set_by_usr;
		BB_Session0SendMsg(DT_NUM_SKY_RC_PATTEN, buf, context.rf_info.rc_ch_patten_need_id_size+2);
		//DLOG_Warning("sky notify grd :cnt=%d,aim_cnt=%d", context.sync_cnt, context.rcChgPatten.timeout_cnt);
	}
	if(gap > 10) gap = 0;
}

static void sky_gen_rc_patten(void)
{
	static int i=0;
	if(context.dev_state == CHECK_FEC_LOCK) return;
	if(context.dev_state == CHECK_LOCK) return;
	if(context.dev_state == WAIT_VT_LOCK) return;
	if(!context.rcChgPatten.en_flag)
	{
		if(i==100)
		{
			context.rcChgPatten.patten[0]=0x01;
			context.rcChgPatten.patten[1]=0x02;
			context.rcChgPatten.patten[2]=0x04;
			context.rcChgPatten.patten[3]=0x08;
			context.rcChgPatten.patten[4]=0x00;
			context.rcChgPatten.en_flag=1;
			context.rcChgPatten.valid=1;
			context.rcChgPatten.timeout_cnt=context.sync_cnt+STATUS_CHG_DELAY;
		}
		else if(i==200)
		{
			
			context.rcChgPatten.patten[0]=0x10;
			context.rcChgPatten.patten[1]=0x20;
			context.rcChgPatten.patten[2]=0x40;
			context.rcChgPatten.patten[3]=0x80;
			context.rcChgPatten.patten[4]=0x00;
			context.rcChgPatten.en_flag=1;
			context.rcChgPatten.valid=1;
			context.rcChgPatten.timeout_cnt=context.sync_cnt+STATUS_CHG_DELAY;
		}
		else if(i==300)
		{
			
			context.rcChgPatten.patten[0]=0x01;
			context.rcChgPatten.patten[1]=0x04;
			context.rcChgPatten.patten[2]=0x02;
			context.rcChgPatten.patten[3]=0x08;
			context.rcChgPatten.patten[4]=0x00;
			
			context.rcChgPatten.en_flag=1;
			context.rcChgPatten.valid=1;
			context.rcChgPatten.timeout_cnt=context.sync_cnt+STATUS_CHG_DELAY;
		}
		
		i++;
		
		if(i==400)
		{
			i=0;
		}
	}
}

static void Sky_TIM2_6_IRQHandler(uint32_t u32_vectorNum)
{
    uint8_t flag_exitFind = 0,trcid[5];
    uint8_t flag_bandswitch = 0;
    DEVICE_STATE pre_state = context.dev_state;
	flag_rchop = 0;
    Reg_Read32(BASE_ADDR_TIMER2 + TMRNEOI_6);

    NVIC_ClearPendingIRQ(BB_RX_ENABLE_VECTOR_NUM); //clear pending after TX Enable is LOW. MUST!
    NVIC_EnableIRQ(BB_RX_ENABLE_VECTOR_NUM);

    NVIC_DisableIRQ(TIMER_INTR26_VECTOR_NUM);
    TIM_StopTimer(stru_skystatus.sky_timer2_6);
    stru_skystatus.u32_cyclecnt++;

	//do sky_sweep always
	sky_SweepProcess();

	//update_sweep_data_to_grd();
	#if 0
	sky_gen_rc_patten();
	#else
	sky_gen_rc_working_patten();
	#endif
    if(context.u8_debugMode)
    {
        return;
    }

    if(1 == vt_info.valid)//pure video
    {
        sky_handle_all_cmds();
        BB_skyPlot();
        BB_GetDevInfo();
        return;
    }

    BB_sky_uartDataHandler();
	sky_notify_rc_patten();

	//check_pwr_mode_status();

    stru_skystatus.u8_rcStatus = get_rc_status();
    //clear ground lock status, the status will be checked in sky_getSignalStatus if
    stru_skystatus.ground_LockStatus = 0xff;	//ground_LockStatus is unknow

    if (SKY_CRC_OK(stru_skystatus.u8_rcStatus) )
    {
        sky_getSignalStatus();
        sky_calc_dist();
    }
    else
    {
        stru_skystatus.flag_errorConnect = 0;
    }

    if (1 == sky_checkRcLock(stru_skystatus.u8_rcStatus))
    {
        BB_SpiDataTransRcv();
    }

	process_state();
    
    if (pre_state != context.dev_state && (pre_state==LOCK || context.dev_state==LOCK))
    {
        STRU_SysEvent_DEV_BB_STATUS lockEvent =
		{
            .pid        = BB_LOCK_STATUS,
            .lockstatus = ((context.dev_state==LOCK) ? 3 : 0),
        };
        SYS_EVENT_NotifyInterCore(SYS_EVENT_ID_BB_EVENT, (void *)&lockEvent);
    }

	context.sync_cnt += 1;
	sky_do_rc_patten();
	sky_do_rf_bw();
    if(AUTO == context.rcHopMode && 1 == flag_rchop)
    {
        #ifdef RF_9363
        //if(RF_600M == context.e_curBand)
        {
            sky_rcHopFreq600m(1);
        }
        #endif

        #ifdef RF_8003X
        //else
        {
            sky_rcHopFreq();
        }
        #endif
    }

    BB_sky_handleRcSearchCmd();

    //context.sync_cnt += 1;
    sky_rc_mod_chg_process();

    #ifdef RF_9363
    if(RF_600M == context.e_curBand)
    {
        sky_filter_chg_process();
    }
    #endif
	#ifdef RFSUB_BAND
    if(context.freq_band_mode == SUB_BAND)
    {
        sky_band_mode_run();
    }
	#endif
	
    sky_handle_all_cmds();
    sky_doRfBandChange(stru_skystatus.u8_rcStatus);
    //sky_do_rcRate();
	//sky_do_rc_patten();
	BB_skyPlot();
    BB_GetDevInfo();
    BB_ComCycleMsgProcess();
    BB_ComCycleSendMsg(BB_COM_TYPE_UART, 0, NULL);

    sky_lna_switch();
}


static void sky_getGroudRcId(uint8_t* pu8_rcId)
{
    pu8_rcId[0] = BB_ReadReg(PAGE2, FEC_1_RD);
    pu8_rcId[1] = BB_ReadReg(PAGE2, FEC_2_RD_1);
    pu8_rcId[2] = BB_ReadReg(PAGE2, FEC_2_RD_2);
    pu8_rcId[3] = BB_ReadReg(PAGE2, FEC_2_RD_3);
    pu8_rcId[4] = BB_ReadReg(PAGE2, FEC_2_RD_4);
}


void sky_setRcId(uint8_t *pu8_rcId)
{
    uint8_t i;
    uint8_t addr[] = {FEC_7, FEC_8, FEC_9, FEC_10, FEC_11};
    uint8_t *p = (uint8_t *)(SRAM_SHARE_FLAG_ST_ADDR + SHARE_FLAG_RC_ID_OFFSET);

    for(i=0; i < sizeof(addr); i++)
    {
        BB_WriteReg(PAGE2, addr[i], pu8_rcId[i]);
    }

    memcpy(p, pu8_rcId, RC_ID_SIZE);
}


void BB_sky_vtId(uint8_t *pu8_vtid)
{
    BB_WriteReg(PAGE2, 0x5b, pu8_vtid[0]);
    BB_WriteReg(PAGE2, 0x5c, pu8_vtid[1]);
}

void BB_sky_requestVtSkip(void)
{
    //static uint8_t seq = 0;
    uint8_t ch,i,get;
    
    //seq++;
    //if(seq > 3)
    //    seq = 0;
    get = 0;
    if(non_lbt_optch_rcnt != non_lbt_optch_wcnt 
        && non_lbt_optch < BB_GetItFrqNum(context.e_curBand)
        && non_lbt_optch != context.cur_IT_ch
        && SysTicks_GetDiff(context.vtFreqTime[non_lbt_optch],SysTicks_GetTickCount()) > NON_LBT_VT_CH_SKIP_INTERVAL)
    {
        //context.vtFreqTime[non_lbt_optch] = SysTicks_GetTickCount();
        get = 3;
        ch = non_lbt_optch;
        goto skip_sky_select;
    }
    for(i = 0;i < BB_GetItFrqNum(context.e_curBand);i++)
    {
        ch = (context.cur_IT_ch + 2 + i) % BB_GetItFrqNum(context.e_curBand);
        if( SysTicks_GetDiff(context.vtFreqTime[ch],SysTicks_GetTickCount()) > NON_LBT_VT_CH_SKIP_INTERVAL && (ch != context.cur_IT_ch))
        {
            //context.vtFreqTime[ch] = SysTicks_GetTickCount();
            get = 1;
            break;
        }
    }
    if(!get){
        DLOG_Warning("not get VT-ch");
        ch = (context.cur_IT_ch + 1) % BB_GetItFrqNum(context.e_curBand);
        //context.vtFreqTime[ch] = SysTicks_GetTickCount();
        get = 2;
    }
skip_sky_select:
    context.sky_sel_vt_ch = ch;
    BB_WriteReg(PAGE2, SKY_NON_LBT_VT_CH,ch | 0x80);
    DLOG_Warning("req vtskip %d,time %d,cvt:%d -> vt:%d,",get,SysTicks_GetTickCount(),context.cur_IT_ch,ch);
}

void sky_SetSaveRCId(uint8_t *pu8_id, uint8_t *pu8_vtId, uint8_t flag)
{
    if(flag)
    {
        BB_saveRcid(pu8_id, pu8_vtId);
    }
    memcpy((void *)context.rcid, (void *)pu8_id, RC_ID_SIZE);
    memcpy((void *)context.vtid, (void *)pu8_vtId, VT_ID_SIZE);

    BB_GenHashId((uint8_t *)context.rcid, (uint8_t *)context.vtid,BB_SKY_MODE);
    BB_GenHashId((uint8_t *)context.rcid, (uint8_t *)context.vtid,BB_GRD_MODE);
    BB_sky_vtId((uint8_t *)context.hashVtid);
    sky_setRcId((uint8_t *)context.hashRcid);


}


void sky_soft_reset(void)
{
    BB_softReset(BB_SKY_MODE);
}



static uint8_t get_rc_status(void)
{
    uint8_t lock = BB_ReadReg(PAGE2, FEC_4_RD);
    STRU_WIRELESS_INFO_DISPLAY *osdptr = (STRU_WIRELESS_INFO_DISPLAY *)(SRAM_BB_STATUS_SHARE_MEMORY_ST_ADDR);

    static int total_count = 0;
    static int lock_count = 0;
    static int nr_lock    = 0;
	
    static uint8_t pre_nrlockcnt = 0;
    static uint8_t pre_lockcnt = 0;
	uint8_t islocked=0;
    total_count ++;
	islocked =  sky_checkRcLock(lock);
	osdptr->lock_status  = islocked;
	#if 0
		if(!islocked)
		DLOG_Critical("unlocked syncnt=%d",context.sync_cnt);
	#endif
	
	sky_statistics_rc_ch_error(islocked);
	//sky_statistics_rc_snr(islocked);
	
    lock_count += islocked;
    nr_lock    += ((lock & 0x04) ? 1 : 0);

    if (total_count >= 100)
    {
        DLOG_Info("-L:%d-%d-%d", lock_count, nr_lock, total_count);

        pre_nrlockcnt = nr_lock;
        pre_lockcnt   = lock_count;
        total_count   = 0;
        lock_count    = 0;
        nr_lock       = 0;

        STRU_rcLockCnt skycStatus = {
            .u8_rcCrcLockCnt = pre_lockcnt,
            .u8_rcNrLockCnt  = pre_nrlockcnt,
        };

        BB_Session0SendMsg(DT_NUM_SKY_LOCK_STATUS, (uint8_t *)&skycStatus, sizeof(STRU_rcLockCnt));

        //sync sky power & enable status to ground
        BB_Session0SendMsg(DT_NUM_BANDSWITCH_ENABLE, (uint8_t *)&context.stru_bandSwitchParam, sizeof(STRU_BAND_SWITCH_PARAM));

        osdptr->u8_rclock =  pre_lockcnt;
        osdptr->u8_nrlock =  pre_nrlockcnt;
    }

    return lock;
}


static void sky_Timer2_6_Init(void)
{
    stru_skystatus.sky_timer2_6.base_time_group = 2;
    stru_skystatus.sky_timer2_6.time_num = 6;
    stru_skystatus.sky_timer2_6.ctrl = 0;
    stru_skystatus.sky_timer2_6.ctrl |= TIME_ENABLE | USER_DEFINED;

    TIM_RegisterTimer(stru_skystatus.sky_timer2_6, 3800);

    reg_IrqHandle(TIMER_INTR26_VECTOR_NUM, Sky_TIM2_6_IRQHandler, NULL);
    NVIC_SetPriority(TIMER_INTR26_VECTOR_NUM,NVIC_EncodePriority(NVIC_PRIORITYGROUP_5,INTR_NVIC_PRIORITY_TIMER00,0));
}

static void sky_Timer2_5_Init(void)
{
    sky_timer2_5.base_time_group = 2;
    sky_timer2_5.time_num = 5;
    sky_timer2_5.ctrl = 0;
    sky_timer2_5.ctrl |= TIME_ENABLE | USER_DEFINED;

    TIM_RegisterTimer(sky_timer2_5, 8000);

    reg_IrqHandle(TIMER_INTR25_VECTOR_NUM, Sky_TIM2_5_IRQHandler, NULL);
    NVIC_SetPriority(TIMER_INTR25_VECTOR_NUM,NVIC_EncodePriority(NVIC_PRIORITYGROUP_5,INTR_NVIC_PRIORITY_TIMER00,0));
}


//////////////////////////////////////////////////////////////////////////////////////////
static void sky_handle_RC_mode_cmd(uint8_t *arg)
{
    memset((uint8_t *)(context.u8_rcValue), 0x00, 5);
    rc_mod_chg_delay.valid = 1;
    rc_mod_chg_delay.cnt = arg[1];
}

static void sky_handle_IT_mode_cmd(uint8_t *arg)
{
    context.itHopMode = (ENUM_RUN_MODE)arg[0];
    //DLOG_Info("it mode:%d", context.itHopMode);
}

static void sky_handle_RC_frq_cmd(uint8_t *arg)
{
    rc_mod_chg_delay.valid = 1;
    rc_mod_chg_delay.cnt = arg[5];
    
    //DLOG_Info("set RC FREQ %x%x%x%x%x", arg[0], arg[1], arg[2], arg[3], arg[4]);
#if 1
    memcpy((uint8_t *)(context.u8_rcValue), arg, 5);
#else
#ifdef RF_9363
    memcpy((uint8_t *)(context.u8_rcValue), arg, 5);
#endif

#ifdef RF_8003X
    
    uint16_t frq_v;
    STRU_FRQ_CHANNEL stru_frq_ch;
    
    frq_v = arg[0] << 8 | arg[1];
    memset(&stru_frq_ch,0,sizeof(STRU_FRQ_CHANNEL));
    RF8003xCalcFrq2Register(frq_v,&stru_frq_ch);
    context.u8_rcValue[0] = stru_frq_ch.frq1;
    context.u8_rcValue[1] = stru_frq_ch.frq2;
    context.u8_rcValue[2] = stru_frq_ch.frq3;
    context.u8_rcValue[3] = stru_frq_ch.frq4;
    context.u8_rcValue[4] = stru_frq_ch.frq5;
    DLOG_Warning("%d->%x:%x:%x:%x:%x:",frq_v,stru_frq_ch.frq1,stru_frq_ch.frq2,stru_frq_ch.frq3,stru_frq_ch.frq4,stru_frq_ch.frq5);

#endif
#endif
}

static void sky_handle_filger_chg_cmd(uint8_t *arg)
{
    uint8_t frq_num = BB_GetItFrqNum(context.e_curBand);

    if(arg[0] < frq_num)
    {
        filter_chg_delay.valid = 1;
        filter_chg_delay.cnt = arg[1];
        filter_chg_it_ch = arg[0];
        //DLOG_Warning("filter chg it_ch:%d sync:%d", filter_chg_it_ch, filter_chg_delay.cnt);
    }
}

static void sky_handle_rc_patten_cmd(uint8_t *arg)
{
	int i=0;

	if(context.rcChgPatten.en_flag==0)
	{
		context.rf_info.rc_patten_set_by_usr=arg[1];
		if(context.rf_info.rc_patten_set_by_usr){
			for(i=0;i<context.rf_info.rc_ch_patten_need_id_size;i++)
			{
				context.rcChgPatten.patten[i]=arg[i+2];
			}
			context.rcChgPatten.en_flag=1;
			context.rcChgPatten.valid=1;
			context.rcChgPatten.timeout_cnt=context.sync_cnt+STATUS_CHG_DELAY;
			DLOG_Critical("sky begin  to start update rc patten,aim cnt=%d",context.rcChgPatten.timeout_cnt);
		}else{
			
			DLOG_Critical("sky get  grd set rc patten to be auto mode,aim cnt=%d",context.rcChgPatten.timeout_cnt);
		}
	}
}
static void sky_handle_RC_Rate_cmd(uint8_t *arg)
{
    context.RcChgRate.value = arg[0];
    context.RcChgRate.cnt = arg[1];
    context.RcChgRate.en_flag = 1;
    DLOG_Warning("%d %d",arg[0],arg[1]);
}


static void sky_handle_IT_frq_cmd(uint8_t *arg)
{
#if 1
    if ( context.stru_itRegs.frq1 != arg[0] || context.stru_itRegs.frq2 != arg[1] ||
         context.stru_itRegs.frq3 != arg[2] || context.stru_itRegs.frq4 != arg[3] ||
         context.stru_itRegs.frq5 != arg[4])
    {
        context.itHopMode = MANUAL;
        BB_write_ItRegs( arg );
    }
#else
    #ifdef RF_9363
    if ( context.stru_itRegs.frq1 != arg[0] || context.stru_itRegs.frq2 != arg[1] ||
         context.stru_itRegs.frq3 != arg[2] || context.stru_itRegs.frq4 != arg[3] ||
         context.stru_itRegs.frq5 != arg[4])
    {
        context.itHopMode = MANUAL;
        BB_write_ItRegs( arg );
    }
    #endif

    #ifdef RF_8003X
    
    uint16_t frq_v;
    STRU_FRQ_CHANNEL stru_frq_ch;
    
    frq_v = arg[0] << 8 | arg[1];
    memset(&stru_frq_ch,0,sizeof(STRU_FRQ_CHANNEL));
    RF8003xCalcFrq2Register(frq_v,&stru_frq_ch);
    if ( context.stru_itRegs.frq1 != stru_frq_ch.frq1 || context.stru_itRegs.frq2 != stru_frq_ch.frq2 ||
         context.stru_itRegs.frq3 != stru_frq_ch.frq3 || context.stru_itRegs.frq4 != stru_frq_ch.frq4 ||
         context.stru_itRegs.frq5 != stru_frq_ch.frq5)
    {
        context.itHopMode = MANUAL;
        BB_write_ItRegs( arg );
    }
    DLOG_Warning("%d->%x:%x:%x:%x:%x:",frq_v,stru_frq_ch.frq1,stru_frq_ch.frq2,stru_frq_ch.frq3,stru_frq_ch.frq4,stru_frq_ch.frq5);

    #endif
#endif
}



static void sky_handle_CH_bandwitdh_cmd(uint8_t *arg)
{
    ENUM_CH_BW bw = (ENUM_CH_BW)(arg[0]);

    if(context.st_bandMcsOpt.e_bandwidth != bw)
    {
        //set and soft-rest
        BB_set_RF_bandwitdh(BB_SKY_MODE, bw);
        context.st_bandMcsOpt.e_bandwidth = bw;
        //DLOG_Info("band:%d", context.st_bandMcsOpt.e_bandwidth);
    }
}


/*
 *  handle command for 10M, 20M
*/
static void sky_handle_auto_bandwitdh_cmd(uint8_t *arg)
{
    ENUM_CH_BW bw = (ENUM_CH_BW)(arg[0]);
	if(context.rf_bw.timeout_cnt ==arg[1] && context.rf_bw.bw == arg[0]) return;
	if(context.st_bandMcsOpt.e_bandwidth != bw)
	{
		context.rf_bw.bw = arg[0];
		//context.st_bandMcsOpt.e_bandwidth=context.rf_bw.bw ;
	    context.rf_bw.timeout_cnt = arg[1];
	    context.rf_bw.ldpc = arg[2];
		context.rf_bw.autobw = arg[3];
		context.rf_info.rf_bw_cg_info.en_auto=context.rf_bw.autobw;
	    context.rf_bw.en_flag = 1;
	    DLOG_Warning("sky get: bw=%d ldpc=%d,auto=%d",arg[0],arg[2],arg[3]);
	}
}

static void sky_handle_CH_qam_cmd(uint8_t *arg)
{
    ENUM_BB_QAM qam = (ENUM_BB_QAM)(arg[0] & 0x03);

    //if(context.qam_mode != qam)
    {
        //set and soft-rest
        BB_set_QAM(qam);
        context.qam_mode = qam;
    }
}

static void sky_handle_CH_ldpc_cmd(uint8_t *arg)
{
    //if((context.ldpc) != arg[0])
    {
        context.ldpc = arg[0];
        BB_set_LDPC(context.ldpc);
        //DLOG_Info("CH_ldpc=>%d", context.ldpc);
    }
}
void sky_handle_mcs_mode_cmd(uint8_t *arg)
{
    ENUM_RUN_MODE mode = (ENUM_RUN_MODE)arg[0];

    context.qam_skip_mode = mode;
    context.brc_mode = mode;

    //DLOG_Info("mcs mode = %d", context.qam_skip_mode);

    if ( mode == AUTO) //MANUAL - > AUTO
    {
        sky_set_McsByIndex( context.st_bandMcsOpt.e_bandwidth, context.qam_ldpc );
    }
}


/*
  * handle H264 encoder brc
 */
static void sky_handle_brc_bitrate_cmd(uint8_t ch, uint8_t *arg)
{
    uint8_t bps = arg[0];

    if(context.brc_bps[ch] != bps)
    {
        context.brc_bps[ch] = bps;
        sky_notify_encoder_brc(ch, bps);
        //DLOG_Info("ch:d bps:%d", ch, bps);
    }
}

static void sky_handle_take_picture_cmd(uint8_t arg)
{
    uint8_t par[2] = {0, arg};

    SYS_EVENT_NotifyLocal(SYS_EVENT_ID_ENCODER_CMD, (void*)(&par[0]));
}

static void sky_handle_picture_quality_set_cmd(uint8_t arg)
{
    uint8_t par[2] = {1, arg};

    SYS_EVENT_NotifyLocal(SYS_EVENT_ID_ENCODER_CMD, (void*)(&par[0]));
}
static void sky_handle_MCS_cmd(void)
{
    uint8_t data0 = BB_ReadReg(PAGE2, MCS_INDEX_MODE);
    static uint32_t time;
    static uint32_t usb0_size = 0,usb1_size = 0;
    uint8_t val = BB_ReadReg(PAGE2, 0xd8);
    uint8_t flag;
    uint8_t realtime_flag;
    static uint8_t usb_cnt = 0,need_drop_usb = 0;
    static uint8_t drop_usb0 = 0xff;

    usb_cnt++;
    usb0_size += sky_read_usb0_senddata();
    usb1_size += sky_read_usb1_senddata();
    if(usb_cnt >= 8)
    {
        usb_cnt = 0;
        if(usb0_size > usb1_size)
        {
            drop_usb0 = 0;//drop usb1
        }
        else if(usb0_size < usb1_size)
        {
            drop_usb0 = 1;//drop usb0
        }
        
        if(usb0_size == 0 || usb1_size == 0)
        {
            need_drop_usb = 1;
			//DLOG_Warning("need drop");
        }
        usb0_size = 0;
        usb1_size = 0;
    }

    realtime_flag = (data0 & 0x80) > 0 ? 1 : 0;
    context.enable_non_lbt = (data0 & 0x40) > 0 ? 1 : 0;
    data0 = data0 & 0x0f;
    if(realtime_flag != context.realtime_mode)
    {
        context.realtime_mode = realtime_flag;
        sky_handle_picture_quality_set_cmd((uint8_t)context.realtime_mode);
        DLOG_Warning("realtime = %d",context.realtime_mode);
    }

    if(!need_drop_usb)
    {
        sky_set_McsByIndex(context.st_bandMcsOpt.e_bandwidth, data0);
        
        DLOG_Info("nmcs %d -> %d",context.qam_ldpc,data0);
        context.qam_ldpc = data0;
        return;
    }
    
    if(context.mcs_switch_state == 0 && context.qam_ldpc != data0)
    {
        //context.need_mcs_switch_flag = 1;
        context.mcs_delay_cnt = 0;
        context.mcs_empty_pack_cnt = 0;
        //context.mcs_switch_state = 1;
        if(val == 0)
        {
            //BB_WriteRegMask(PAGE0, 0x16,0x80,0x80);
            if(drop_usb0 == 0)
            {
                BB_WriteReg(PAGE2, 0x06,0x04);
            }
            else if(drop_usb0 == 1)
            {
                BB_WriteReg(PAGE2, 0x06,0x84);
            }
            else
            {
                DLOG_Info("undef");
            }
            time = SysTicks_GetTickCount();
            DLOG_Info("%d r206=%x, %d",drop_usb0,BB_ReadReg(PAGE2, 0x06),time);
            context.mcs_switch_state = 2;
        }
    }
    else if(context.mcs_switch_state == 1)
    {
        //nothing to do ,wait empty send status
        context.mcs_switch_state = 2;
    }
    else if(context.mcs_switch_state == 2)
    {
        flag= 0;
        context.mcs_empty_pack_cnt++;
        if(val == 0)
        {
            flag = 1;
        }
        else if(val == 0x80 || val == 0x40)
        {
            context.mcs_delay_cnt++;
            if(context.mcs_delay_cnt >= 2)
            {
                flag = 1;
                DLOG_Info("> 2 times");
            }
        }
        else if(context.mcs_empty_pack_cnt > 20)
        {
            flag = 1;
            DLOG_Warning("> 20");
        }

        if(flag)
        {
            //BB_WriteRegMask(PAGE0, 0x16,0x00,0x80);
            BB_WriteReg(PAGE2, 0x06,0x44);

            context.mcs_switch_state = 0;
            DLOG_Info("%x delta %d ms",BB_ReadReg(PAGE2, 0x06),SysTicks_GetDiff(time,SysTicks_GetTickCount()));
            
            sky_set_McsByIndex(context.st_bandMcsOpt.e_bandwidth, data0);

            DLOG_Info("mcs %d -> %d",context.qam_ldpc,data0);
            context.qam_ldpc = data0;

        }
    }

    
}
/*
void sky_handle_rc_ch_sync_cmd(void)
{
    uint8_t data0 = BB_ReadReg(PAGE2, GRD_SKY_RC_CH_SYNC);
    uint8_t u8_ch = (data0 & GRD_SKY_RC_CH_MASK);
    uint8_t u8_bandswitch = ((data0 & GRD_SKY_BAND_SWITCH_MASK) ? 1:0);

    if (data0 & GRD_SKY_RC_RSV_BIT_MASK)
    {
        if (u8_ch != context.sky_rc_channel && context.rcHopMode == AUTO)
        {
            //DLOG_Warning("+++++++++++rc:%d %d %d+++++++++++", u8_ch, context.sky_rc_channel, (BB_ReadReg(PAGE2, GRD_SKY_RC_CH_SYNC)) & GRD_SKY_RC_CH_MASK);
            //context.sky_rc_channel = u8_ch;
        }

        if (u8_bandswitch != (context.stru_bandSwitchParam.u8_bandSwitchAfterUnlock == 0x81))
        {
            BB_Session0SendMsg(DT_NUM_BANDSWITCH_ENABLE, (uint8_t *)(&context.stru_bandSwitchParam), sizeof(context.stru_bandSwitchParam));
        }
    }
    else
    {
        DLOG_Info("err %x", data0); //if AES not match, this happen
    }
}
*/
static void sky_handle_it_ch_sync_cmd(void)
{
    uint8_t start = context.rc_start;
    uint8_t data0 = BB_ReadRegMask(PAGE2, GRD_SKY_IT_CH_SYNC, GRD_SKY_IT_CH_MASK);

    if (data0 != context.cur_IT_ch && context.itHopMode == AUTO)
    {
        if (data0 > BB_GetItFrqNum(context.e_curBand))
        {
            return; //invalid vt channel
        }
        
        if(!context.bandedge_enable)
        {
            DLOG_Warning("it ch %d -> %d",context.cur_IT_ch,data0);
            context.vtFreqTime[context.cur_IT_ch] = SysTicks_GetTickCount();//save last main ch time value

            context.cur_IT_ch = data0;
            BB_set_ItFrqByCh(context.e_curBand, context.cur_IT_ch);
            context.sky_sel_vt_ch_flag = 0;

            //to fixed grd and sky decide skip vt-ch at the same time
            if(context.vtskip_stat == 1)
            {
                context.vtskip_stat = 0;
                DLOG_Warning("not vt-ch skip");
            }
        }
        else
        {
            context.delay_it_ch = data0;
            context.delay_it_flag = 1;
        }

        #ifdef RF_9363
        if(RF_600M == context.e_curBand)
        {
            BB_GetItStarEndByItCh((uint8_t *)(&context.vt_start), (uint8_t *)(&context.vt_end), context.cur_IT_ch);
            BB_GetRcStarEndByItCh((uint8_t *)(&context.rc_start), (uint8_t *)(&context.rc_end), context.cur_IT_ch);

			//if(start != context.rc_start)
            //{
            //    BB_RcSelectionInit();
           // }
            //DLOG_Warning("vt_s:%d vt_e:%d rc_s:%d rc_e:%d m:%d", context.vt_start,context.vt_end,\
            //                                            context.rc_start, context.rc_end, context.cur_IT_ch);
        }
        #endif
    }

    if(context.bandedge_enable && context.dev_state == CHECK_LOCK)
    {
        context.delay_it_ch = data0;
        context.delay_it_flag = 1;
    }

}

void sky_handleRcIdAutoSearchCmd(STRU_WIRELESS_CONFIG_CHANGE* pcmd)
{
    uint8_t  class  = pcmd->u8_configClass;
    uint8_t  item   = pcmd->u8_configItem;
    uint32_t value  = pcmd->u32_configValue;
    uint32_t value1 = pcmd->u32_configValue1;
    uint8_t u8_rcArray[5]   = {(value>>24)&0xff, (value>>16)&0xff, (value>>8)&0xff, (value)&0xff, value1&0xff};
    uint8_t u8_vtIdArray[2] = {(value1>>8)&0xff, (value1>>16)&0xff};

    if(class == WIRELESS_AUTO_SEARCH_ID)
    {
        switch (item)
        {
            case RCID_AUTO_SEARCH:
                BB_sky_requestStartSearch();
                break;

            case RCID_SAVE_RCID:
                stru_skystatus.u8_userRequestRcSearch  = 2; //stop search rc id
                sky_SetSaveRCId(u8_rcArray, u8_vtIdArray, 1);
                break;

            case SET_TMP_RC_VT_ID:
                stru_skystatus.u8_userRequestRcSearch  = 2; //set temporary rc/vt id
                sky_SetSaveRCId(u8_rcArray, u8_vtIdArray, 0);
                break;

            case RCID_STOP_SEARCH:
                BB_sky_requestStopSearch();
                break;

            case GET_OBJ_VT_ID:
                memcpy((uint8_t *)context.vtid,u8_vtIdArray,2);
                memcpy(stru_skystatus.cur_groundRcid, (const void *)u8_rcArray, RC_ID_SIZE);
                search_id_by_rssi_timeout_flag = 1;
                DLOG_Warning("set vt,%02x,%02x",u8_vtIdArray[0],u8_vtIdArray[1]);
                break;

            case ENABLE_SEARCH_ID_BY_RSSI:
                enable_search_id_by_rssi = 1;
                search_id_by_rssi_timeout_flag = 0;
                context.select_obj_vt_ok = 0;
                DLOG_Warning("en rssi");
                break;

            default:
                //DLOG_Warning("Error: id %x %x", class, item);
                break;
        }
    }
}



static void sky_get_sync_cnt(void)
{
    if(!context.aes_off){
        context.sync_cnt = BB_ReadReg(PAGE3, SYNC_CNT);
    }
    else{
        context.sync_cnt = BB_GetUartDummyCnt();
    }
}

static void sky_handle_all_spi_cmds(void)
{
    sky_handle_MCS_cmd();
    //sky_handle_rc_ch_sync_cmd();
    sky_handle_it_ch_sync_cmd();
    sky_get_sync_cnt();
}

int sky_ack_grd_cmd(uint8_t *arg, uint16_t len)
{
    extern STRU_DT_HEAD g_dtLen[];

    uint8_t idx;

    if(0 == BB_GetDtInfo(arg[0], NULL, NULL, &idx))
    {
        if (1 == g_dtLen[idx].ack)
        {
            return BB_Session0SendMsg(arg[0], arg + 1, len);
        }
    }

    return 0;
}

static void sky_handle_all_grd_cmds(uint8_t *arg, uint8_t len)
{
    switch(arg[0])
    {
        case DT_NUM_NON_LBT_NOTICE_OPTCH:
        {
            non_lbt_optch = arg[1];
            non_lbt_optch_wcnt++;
            break;
        }

        case DT_NUM_RC_CH_MODE:
        {
            sky_handle_RC_mode_cmd(&arg[1]);
            break;
        }
        case DT_NUM_IT_CH_MODE:
        {
            sky_handle_IT_mode_cmd(&arg[1]);
            if(AUTO == arg[1])
            {
                BB_set_ItFrqByCh(context.e_curBand, context.cur_IT_ch);
            }
            break;
        }
        case DT_NUM_RF_BAND_CHANGE:
        {
            sky_handle_RF_band_cmd(&arg[1]);
            break;
        }
        case DT_NUM_RF_CH_BW_CHANGE:
        {
            sky_handle_CH_bandwitdh_cmd(&arg[1]);
            break;
        }
		case  DT_NUM_AUTO_CH_BW_CHANGE:
		{
			sky_handle_auto_bandwitdh_cmd(&arg[1]);
			break;
		}
        case DT_NUM_MCS_MODE_SELECT:
        {
            sky_handle_mcs_mode_cmd(&arg[1]);
            break;
        }
        case DT_NUM_ENCODER_BRC_CHAGE_CH1:
        {
            sky_handle_brc_bitrate_cmd(0, &arg[1]);
            break;
        }
        case DT_NUM_ENCODER_BRC_CHAGE_CH2:
        {
            sky_handle_brc_bitrate_cmd(1, &arg[1]);
            break;
        }
        case DT_NUM_TAKE_PICTURE:
        {
            static uint8_t tp_idx = 0xFF;
            if(tp_idx != arg[1])
            {
                sky_handle_take_picture_cmd(arg[2]);
                tp_idx = arg[1];
            }
            break;
        }
        case DT_NUM_PICTURE_QUALITY_SET:
        {
            static uint8_t pqs_idx = 0xFF;
            if(pqs_idx != arg[1])
            {
                sky_handle_picture_quality_set_cmd(arg[2]);
                pqs_idx = arg[1];
            }
            break;
        }
        case DT_NUM_RF_CH_LDPC_CHANGE:
        {
            sky_handle_CH_ldpc_cmd(&arg[1]);
            break;
        }
        case DT_NUM_RF_CH_QAM_CHANGE:
        {
            sky_handle_CH_qam_cmd(&arg[1]);
            break;
        }
        case DT_NUM_GRD_MASK_CODE:
        {
            //BB_RcSelectionInit();
            break;
        }
        case DT_NUM_RC_FRQ:
        {
            sky_handle_RC_frq_cmd(&arg[1]);
            break;
        }
        case DT_NUM_IT_FRQ:
        {
            sky_handle_IT_frq_cmd(&arg[1]);
            break;
        }
        case DT_NUM_FRE_OFFSET:
        {
            //sky_handle_frqOffset(&arg[1]);
            break;
        }
        case DT_NUM_RC_RATE:
        {
            sky_handle_RC_Rate_cmd(&arg[1]);
            break;
        }

		case DT_NUM_SKY_RC_PATTEN:
		{
			context.rcChgPatten.valid=0;
			DLOG_Warning("sky get ack,cnt=%d",context.sync_cnt);
			break;

		}
		case DT_NUM_GRD_RC_CHPATTEN:
		{
			DLOG_Warning("sky get grd rc patten,cnt=%d",context.sync_cnt);
			sky_handle_rc_patten_cmd(&arg[1]);
		}
        case DT_NUM_RF_BAND_MODE:
        {
            context.st_bandMcsOpt.e_rfbandMode = (ENUM_RUN_MODE)arg[1];
            break;
        }
        case DT_NUM_LNA_STATUS_CHG:
        {
            sky_command_lna_switch(arg[1]);
            break;
        }
        /*
        case DT_NUM_NON_LBT_ENABLE:
        {
            context.enable_non_lbt = arg[1];
            DLOG_Warning("non-lbt status = %d",context.enable_non_lbt);

            break;
        }*/

        case DT_NUM_AOC_ADJUST:
        {
            sky_aoc_execute( (void *)(&arg[1]) );
            break;
        }

        case DT_NUM_GRD_SEND_VT_ID:
        {
            STRU_SyncrcId SyncrcId;
            if(!enable_search_id_by_rssi)
            {
                memcpy(SyncrcId.u8_skyRcIdArray, (void *)context.rcid, RC_ID_SIZE);
                BB_Session0SendMsg(DT_NUM_RC_ID_SYNC, (uint8_t *)&SyncrcId, sizeof(STRU_SyncrcId));
                BB_sky_ackVtidMessage(arg + 1);
            }
            else
            {
                if(!search_id_by_rssi_timeout_flag)
                {
                    BB_sky_nackVtidMessage(arg + 1);
                    sky_soft_reset();
                }
                else
                {
                    uint8_t *vtid = arg+1;
                    if(vtid[0] == context.vtid[0] && vtid[1] == context.vtid[1])
                    {
                        BB_sky_setSearchState(SKY_WAIT_RC_ID_MATCH);
                        memcpy(SyncrcId.u8_skyRcIdArray, (void *)context.rcid, RC_ID_SIZE);
                        BB_Session0SendMsg(DT_NUM_RC_ID_SYNC, (uint8_t *)&SyncrcId, sizeof(STRU_SyncrcId));
                        BB_GenHashId((uint8_t *)context.rcid, (uint8_t *)context.vtid,BB_SKY_MODE);
                        BB_GenHashId((uint8_t *)context.rcid, (uint8_t *)context.vtid,BB_GRD_MODE);
                        BB_sky_vtId((uint8_t *)context.vtid);
                        sky_setRcId((uint8_t *)context.hashRcid);

                        DLOG_Warning("select vtid %02x,%02x",vtid[0],vtid[1]);
                        context.select_obj_vt_ok = 1;
                    }
                }
            }
            break;
        }

        case DT_NUM_FILTER_CHG:
        {
            #ifdef RF_9363
            if(RF_600M == context.e_curBand)
            {
                sky_handle_filger_chg_cmd(&arg[1]);
            }
            #endif
            break;
        }

        case DT_NUM_BAND_MODE_CHG:
        {
            sky_handle_sub_band_set_cmd(&arg[1]);
            break;
        }

        default:
        {
            break;
        }
    }

    sky_ack_grd_cmd(arg, len);
}


static void sky_handle_one_cmd(STRU_WIRELESS_CONFIG_CHANGE* pcmd)
{
    uint8_t u8_data;
    uint8_t class  = pcmd->u8_configClass;
    uint8_t item   = pcmd->u8_configItem;
    uint32_t value = pcmd->u32_configValue;
    uint32_t value1 = pcmd->u32_configValue1;
    uint8_t frq[5] = {(value>>24)&0xff, (value>>16)&0xff, (value>>8)&0xff, value&0xff, (value1&0xff)};

    //DLOG_Info("class item value %d %d 0x%0.8d \r\n", class, item, value);
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
                    sky_handle_RF_band_cmd_pure_vt((ENUM_RF_BAND)value);
                }
                else
                {
                    #ifdef JYDS
                    DLOG_Warning("unsupport");
                    #else
                    //uint8_t data[2] = {context.sync_cnt, ((value & 0x03) | (1 << 2))};
                    STRU_BandChange data;
                    memset(&data, 0, sizeof(STRU_BandChange));
                    data.e_toBand = value;
                    data.flag_bandchange = 1;
                    data.u8_eqCntChg = context.stru_bandChange.u8_eqCntChg + 100;
                    sky_handle_RF_band_cmd((uint8_t *)&data);
                    #endif
                }
                break;
            }
            case FREQ_CHANNEL_MODE: //auto manual
            {
                context.itHopMode = (ENUM_RUN_MODE)value;
                if(AUTO == context.itHopMode)
                {
                    BB_set_ItFrqByCh(context.e_curBand, context.cur_IT_ch);
                }
                break;
            }

            case RC_CHANNEL_MODE:
            {
                context.rcHopMode = (ENUM_RUN_MODE)value;
                break;
            }

            case RC_CHANNEL_SELECT:
            {
                context.sky_rc_channel = (uint8_t)value;
                BB_set_Rcfrq(context.e_curBand, context.sky_rc_channel);
                break;
            }

            case RC_CHANNEL_FREQ:
            {
                context.rcHopMode = (ENUM_RUN_MODE)MANUAL;

                #ifdef RF_9363
                //if(RF_600M == context.e_curBand)
                {
                    sky_BB_write_RcRegs(frq);
                }
                #endif

                #ifdef RF_8003X
                //else
                {
                    BB_write_RcRegs(frq);
                }
                #endif

                //DLOG_Info("RC_CHANNEL_FREQ %x\r\n", value);
                break;
            }

            case IT_CHANNEL_FREQ:
            {
                context.itHopMode = MANUAL;
                BB_write_ItRegs(frq);
                //DLOG_Info("IT_CHANNEL_FREQ %x\r\n", value);
                break;
            }
            case IT_CHANNEL_SELECT:
            {
                context.itHopMode = MANUAL;
                BB_set_ItFrqByCh(context.e_curBand, value);
                break;
            }

            case FREQ_BAND_WIDTH_SELECT:
            {
                context.st_bandMcsOpt.e_bandwidth = (ENUM_CH_BW)value;
                BB_set_RF_bandwitdh(BB_SKY_MODE, context.st_bandMcsOpt.e_bandwidth);
                //DLOG_Info("FREQ_BAND_WIDTH_SELECT %x\r\n", value);
                break;
            }
            
            case SET_LNA_STATUS:
            {
                sky_command_lna_switch(value);
                break;
            }

            default:
            {
                break;
            }
        }
    }

    if(class == WIRELESS_MCS_CHANGE)
    {
        switch(item)
        {
            case MCS_MODE_SELECT:
            {
                sky_handle_mcs_mode_cmd((uint8_t *)(&value));
                break;
            }

            case MCS_MODULATION_SELECT:
            {
                break;
            }

            case MCS_CODE_RATE_SELECT:
            {
                break;
            }

            case MCS_IT_QAM_SELECT:
            {
                BB_set_QAM((ENUM_BB_QAM)value);
                break;
            }

            case MCS_IT_CODE_RATE_SELECT:
            {
                BB_set_LDPC((ENUM_BB_LDPC)value);
                break;
            }

            case MCS_RC_QAM_SELECT:
            {
                context.rc_qam_mode = (ENUM_BB_QAM)value;
                BB_WriteRegMask(PAGE2, 0x09, (value << 0) & 0x03, 0x03);
                //DLOG_Info("MCS_RC_QAM_SELECT %x\r\n", value);
                break;
            }

            case MCS_RC_CODE_RATE_SELECT:
            {
                context.rc_ldpc = (ENUM_BB_LDPC)value;
                BB_WriteRegMask(PAGE2, 0x09, (value << 2) & 0x1C, 0x1C);
                //DLOG_Info("MCS_RC_CODE_RATE_SELECT %x\r\n", value);
                break;
            }
            default:
                break;
        }
    }
    if(class == WIRELESS_ENCODER_CHANGE)
    {
        switch(item)
        {

            case ENCODER_DYNAMIC_BIT_RATE_SELECT_CH1:
                sky_notify_encoder_brc(0,(uint8_t)value);
                break;

            case ENCODER_DYNAMIC_BIT_RATE_SELECT_CH2:
                sky_notify_encoder_brc(1, (uint8_t)value);
                break;
            case ENCODER_TAKE_PICTURE:
                sky_handle_take_picture_cmd((uint8_t)value);
                break;
            case ENCODER_PICTURE_QUALITY_SET:
                //sky_handle_picture_quality_set_cmd((uint8_t)value);
                break;
            default:
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
                //BB_SwtichOnOffCh(0, (uint8_t)value);
                //DLOG_Info("sky invalid cmd.");
                break;
            }

            case SWITCH_ON_OFF_CH2:
            {
                //BB_SwtichOnOffCh(1, (uint8_t)value);
                //DLOG_Info("sky invalid cmd.");
                break;
            }

            case BB_SOFT_RESET:
            {
                //DLOG_Info("sky bb reset.");
                sky_soft_reset();
                break;
            }

            case SET_PURE_VT_MODE:
            {
                sky_vt_mode_proc((uint8_t)value);
                break;
            }

            case RW_BB_RF_REG:
            {
                BB_InsertCmd(1, pcmd);
                break;
            }

            case AGC_GEAR_CHG:
            {
                sky_handle_agc_gear_chg_cmd((uint8_t)value);
                break;
            }

            case PWR_CTRL_SET:
            {
                BB_InsertCmd(1, pcmd);
                break;
            }

            case MISC_CHECK_LOCK_TIME:
            {
				stru_skystatus.check_lock_times = value;
				//DLOG_Warning("check_lock_times %x", value);
				break;
            }
#if 0
            case SET_FIND_BESIDE_DEV:
            {
                if(value)
                {
                    sky_start_find_beside_dev();
                }
                else
                {
                    sky_stop_find_beside_dev();
                }
                break;
            }
#endif
            case SET_NON_LBT:
            {
                context.enable_non_lbt = (uint8_t)value;
                DLOG_Warning("nlbt %d",context.enable_non_lbt);
                break;
            }
            
            case SET_BANDEDGE:
            {
                context.bandedge_enable = (uint8_t)value;
                context.low_power_db = (uint8_t)value1;
                if(context.cur_IT_ch == 0 || context.cur_IT_ch == 6)
                {
                    BB_WriteReg(PAGE0, BB_POWER_A, BB_POWER_DEFAULT_VALUE - context.low_power_db*4);
                    BB_WriteReg(PAGE0, BB_POWER_B, BB_POWER_DEFAULT_VALUE - context.low_power_db*4);
                    BB_set_power(context.e_curBand,context.pwr - context.low_power_db);
                    DLOG_Warning("bd low %d",context.low_power_db);
                }

                DLOG_Warning("bd %d,%d",context.bandedge_enable,context.low_power_db);
                break;
            }

            default:
                break;
        }
    }

    if (class == WIRELESS_AUTO_SEARCH_ID)
    {
        sky_handleRcIdAutoSearchCmd(pcmd);
    }
}

static void sky_handle_one_rf_cmd(STRU_WIRELESS_CONFIG_CHANGE* pcmd)
{
    uint8_t u8_data;
    uint8_t class  = pcmd->u8_configClass;
    uint8_t item   = pcmd->u8_configItem;
    uint32_t value = pcmd->u32_configValue;


    //DLOG_Info("class item value %d %d 0x%0.8d \r\n", class, item, value);

    if(class == WIRELESS_OTHER)
    {
        switch(item)
        {
            case RW_BB_RF_REG:
            {
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


static void sky_handle_all_cmds(void)
{
    int ret = 0;
    int cnt = 0;
    STRU_WIRELESS_CONFIG_CHANGE cfg;
    while( (cnt++ < 5) && (1 == BB_GetCmd(0, &cfg)))
    {
        sky_handle_one_cmd( &cfg );
    }
}

static void sky_handle_all_rf_cmds(void)
{
    int ret = 0;
    int cnt = 0;
    STRU_WIRELESS_CONFIG_CHANGE cfg;
    while( (cnt++ < 5) && (1 == BB_GetCmd(1, &cfg)))
    {
        sky_handle_one_rf_cmd( &cfg );
    }
}


static uint32_t H264_SDRAM_GetBufferLevel(uint8_t e_channel)
{
    uint32_t buf_level = 0;

    Reg_Write32_Mask(SDRAM_ENC_DEEP_REG_ADDR + 0xDC, (unsigned int)(0x21 << 24), 0xFF000000);

    //read buffer counter

    if (e_channel == 0)
    {
        Reg_Write32_Mask(SDRAM_ENC_DEEP_REG_ADDR + 0xD8, (unsigned int)(0x04 <<  8), 0x00000F00);       // Switch to vdb debug register
    }
    else
    {
        Reg_Write32_Mask(SDRAM_ENC_DEEP_REG_ADDR + 0xD8, (unsigned int)(0x05 <<  8), 0x00000F00);       // Switch to vdb debug register
    }

    buf_level = Reg_Read32(SDRAM_ENC_DEEP_REG_ADDR + 0xF8);
    Reg_Write32_Mask(SDRAM_ENC_DEEP_REG_ADDR + 0xDC, (unsigned int)(0x00 << 24), 0xFF000000);

    return buf_level;
}

/*
 *
*/
static void BB_skyPlot(void)
{
    uint8_t u8_data,i;
	static int j;
    STRU_WIRELESS_INFO_DISPLAY *osdptr = (STRU_WIRELESS_INFO_DISPLAY *)(SRAM_BB_STATUS_SHARE_MEMORY_ST_ADDR);
    sky_GetSweepNoise(osdptr->sweep_energy, 21);

    osdptr->msg_id = 0x33;
    osdptr->head = 0xff; //starting writing
    osdptr->tail = 0x00;

    osdptr->IT_channel = context.sky_rc_channel + ((context.e_curBand == RF_5G) ? 50 : 0);

    osdptr->agc_value[0] = BB_ReadReg(PAGE2, AAGC_2_RD);
    osdptr->agc_value[1] = BB_ReadReg(PAGE2, AAGC_3_RD);

    osdptr->agc_value[2] = BB_ReadReg(PAGE2, FEC_4_RD);
    osdptr->agc_value[3] = BB_ReadReg(PAGE2, 0xd7);

    osdptr->snr_vlaue[0] = (((uint16_t)BB_ReadReg(PAGE2, SNR_REG_0)) << 8) | BB_ReadReg(PAGE2, SNR_REG_1);
    osdptr->snr_vlaue[1] = sky_get_snr_average();
    //osdptr->snr_vlaue[1] = BB_ReadReg(PAGE2, SNR_REG_0);

    u8_data = BB_ReadReg(PAGE2, TX_2);
    osdptr->modulation_mode = (u8_data >> 6) & 0x03;
    osdptr->code_rate       = (u8_data >> 0) & 0x07;
    //context.qam_mode = (ENUM_BB_QAM)osdptr->modulation_mode;

    u8_data = BB_ReadReg(PAGE2, 0x09);
    osdptr->rc_modulation_mode = (u8_data >> 2) & 0x07;
    osdptr->rc_code_rate       = (u8_data >> 0) & 0x03;

    osdptr->in_debug     = context.u8_debugMode;

    osdptr->sdram_buf_size[0] = 0x100000 - H264_SDRAM_GetBufferLevel(0);
    osdptr->sdram_buf_size[1] = 0x100000 - H264_SDRAM_GetBufferLevel(1);
	
	osdptr->current_pwr = context.u8_TargetPower[0];
	osdptr->e_bandwidth = context.st_bandMcsOpt.e_bandwidth;
	j++;
	
	#if 0
	int sweep_result[40];
	if(j==100){
		DLOG_Critical("current_pwr:%d\n", context.u8_TargetPower[0]);
		j=0;
	}
	#endif
	
    osdptr->head = 0x00;
    osdptr->tail = 0xff;    //end of the writing
}



static void sky_calc_dist(void)
{
    uint8_t u8_data[3];
    uint32_t u32_data = 0;

    u8_data[0] = BB_ReadReg(PAGE3, 0xA4);
    u8_data[1] = BB_ReadReg(PAGE3, 0xA3);
    u8_data[2] = BB_ReadReg(PAGE3, 0xA2);

    BB_WriteReg(PAGE0, 0x1D, u8_data[0]);
    BB_WriteReg(PAGE0, 0x1C, u8_data[1]);
    BB_WriteReg(PAGE0, 0x1B, u8_data[2]);
}


uint16_t sky_get_rc_snr( void )
{
    static uint32_t cnt = 0;
    uint16_t snr = (((uint16_t)BB_ReadReg(PAGE2, SNR_REG_0)) << 8) | BB_ReadReg(PAGE2, SNR_REG_1);
    if( cnt++ > 1500 )
    {
        cnt = 0;
        DLOG_Info("SNR1:%0.4x\n", snr);
    }

    return snr;
}
static uint16_t sky_get_snr_average(void)
{
    uint8_t i;
    uint32_t sum = 0;
    for (i = 0; i < sizeof(stru_skystatus.u16_sky_snr) / sizeof(stru_skystatus.u16_sky_snr[0]); i++)
    {
        sum += stru_skystatus.u16_sky_snr[i];
    }

    return (sum/i);
}



void sky_ChgRcRate(uint8_t rate)
{
    if (0 == rate)
    {
        BB_WriteReg(PAGE2, 0x09, 0x80); // BPSK 1/2
        BB_SetAesOffUartMaxLen(BPSK1_2_LEN);
    }
    else if(1 == rate)
    {
        BB_WriteReg(PAGE2, 0x09, 0x85); // QPSK 2/3
        BB_SetAesOffUartMaxLen(QPSK2_3_LEN);
        sky_soft_reset();
    }
    if (2 == rate)
    {
        BB_WriteReg(PAGE2, 0x09, 0x81); // QPSK 1/2
        BB_SetAesOffUartMaxLen(QPSK1_2_LEN);
    }
    else if(3 == rate)
    {
        BB_WriteReg(PAGE2, 0x09, 0x84); // BPSK 2/3
        BB_SetAesOffUartMaxLen(BPSK2_3_LEN);
        sky_soft_reset();
    }
    else
    {
        DLOG_Error("%d",rate);
    }
    context.uplink_qam_mode = rate;
	DLOG_Critical("RC rate = %d !");
    //sky_soft_reset();
}


static void BB_sky_uartDataHandler()
{
    uint8_t  data[128];
    uint32_t eventId;
    uint8_t proCnt = 5;
    int rcvLen = 0;
    uint8_t pid = 0;

    while(proCnt--)
    {
        rcvLen = BB_Session0RecvMsg(data, sizeof(data));

        if (rcvLen > 0)
        {
            pid = data[0];

            if (DT_NUM_REMOTE_EVENT == pid)
            {
                memcpy((uint8_t *)(&eventId), data+1, 4);
                SYS_EVENT_NotifyInterCore(eventId, data+5);
            }
            else if(DT_NUM_CH_MAP == pid)
            {
                uint8_t ch;
                memcpy((uint8_t *)context.rc_ch_map, data+1,  MAX_RC_FRQ_SIZE);
                g_syn = 0;
                for(ch = 0; ch < MAX_RC_FRQ_SIZE; ch++)
                {
                    if (context.rc_ch_map[ch] > MAX_RC_FRQ_SIZE)
                    {
                        DLOG_Warning("ch= %d map=%d", ch, context.rc_ch_map[ch]);
                    }
                }
            }
            else if((pid >= DT_NUM_RC_CH_MODE) && (pid < DT_NUM_MAX))
            {
                sky_handle_all_grd_cmds(data, (uint8_t)rcvLen);
            }
            else
            {
                //DLOG_Error("pid=%d", pid);
            }
        }
        else
        {
            break;
        }
    }
}

static void sky_aoc_execute(void *power)
{
    uint8_t org = BB_ReadReg(PAGE0, 0x09);

    int value = *(int *)power;

    if (0 == context.aoc.u8_aocEnable)
    {
        return;
    }

    if(context.bandedge_enable && (context.cur_IT_ch == 0 || context.cur_IT_ch == context.rf_info.sweep_freqsize-1))
    {
        return;
    }

    //open power exist freq-power makeup,
    //open power not support aoc auto
    if(context.e_powerMode == RF_POWER_OPEN)
    {
        return;
    }

    if(value > 0 )
    {
        if(context.u8_aocAdjustPwr >= context.pwr)
        {
            DLOG_Info("max %d dbm",context.u8_aocAdjustPwr);
            return;
        }
        context.u8_aocAdjustPwr++;

        if(context.e_powerMode == RF_POWER_OPEN)
        {
            org += 4;
            #define MAX_BB_POWER (0x1F)
            if(org >= MAX_BB_POWER)
            {
                org = MAX_BB_POWER;
            }

            BB_WriteReg(PAGE0, 0x09, org);
            BB_WriteReg(PAGE0, 0x16, org);

        }
        else
        {
            BB_set_power(context.e_curBand,context.u8_aocAdjustPwr);
        }
    }
    else if(value < 0)
    {
        if(context.u8_aocAdjustPwr <= context.aoc.u8_PwrThdMin)
        {
            DLOG_Info("min %d dbm",context.u8_aocAdjustPwr);
            return;
        }
        context.u8_aocAdjustPwr--;

        if(context.e_powerMode == RF_POWER_OPEN)
        {
            org -= 4;
            #define MIN_BB_POWER (0x06)
            if(org <= MIN_BB_POWER)
            {
                org = MIN_BB_POWER;
            }

            BB_WriteReg(PAGE0, 0x09, org);
            BB_WriteReg(PAGE0, 0x16, org);

        }
        else
        {
            BB_set_power(context.e_curBand,context.u8_aocAdjustPwr);
        }
    }
}

static void sky_vt_mode_proc(uint8_t value)
{
    if(1 == value)
    {
        vt_info.itHopMode = context.itHopMode;
        vt_info.valid = 1;
        context.itHopMode = MANUAL;
		context.sky_tx_only_flag=1;
        BB_WriteReg(PAGE2, 0x02, 0x06);
        BB_WriteRegMask(PAGE2, 0x20, 0x08, 0x08);
        BB_SetTrxMode(BB_NORMAL_MODE);
        sky_soft_reset();
        DLOG_Warning("enter purevt");
    }
    else if(0 == value)
    {
        context.itHopMode = vt_info.itHopMode;
		context.sky_tx_only_flag=0;
        vt_info.valid = 0;
        BB_WriteReg(PAGE2, 0x02, 0x07);
        //BB_SetTrxMode(BB_NORMAL_MODE);
        BB_WriteRegMask(PAGE2, 0x20, 0x00, 0x08);
        sky_soft_reset();
        DLOG_Warning("exit purevt");
    }
    else
    {
    }
}

static int sky_lock_status(void *p)
{
    if(enable_search_id_by_rssi && BB_sky_isSearching())
    {
        return 1;
    }

    if(context.dev_state == LOCK)
    {
        return 1;
    }
    else
    {
        return 0;
    }
}


static int sky_rc_mod_chg_process(void)
{
    if(rc_mod_chg_delay.valid)
    {
        if(context.sync_cnt == rc_mod_chg_delay.cnt)
        {
            rc_mod_chg_delay.valid = 0;

            if( context.u8_rcValue[0] || context.u8_rcValue[1] || context.u8_rcValue[2] || \
                context.u8_rcValue[3] || context.u8_rcValue[4])
            {
                #ifdef RF_9363
                //if(RF_600M == context.e_curBand)
                {
                    sky_BB_write_RcRegs( (uint8_t *)(context.u8_rcValue) );
                }
                #endif

                #ifdef RF_8003X
                //else
                {
                    BB_write_RcRegs( (uint8_t *)(context.u8_rcValue) );
                }
                #endif

                context.rcHopMode = MANUAL;
            }
            else //if context.u8_rcValue[0~4] ==0, means the auto mode
            {
                context.rcHopMode = AUTO;
            }
        }
    }
}
static int sky_filter_chg_process(void)
{
    uint8_t start = context.rc_start;

    if(filter_chg_delay.valid)
    {
        if(context.sync_cnt == filter_chg_delay.cnt)
        {
            filter_chg_delay.valid = 0;
            context.cur_IT_ch = filter_chg_it_ch;
            BB_set_ItFrqByCh(context.e_curBand, context.cur_IT_ch);
            BB_GetItStarEndByItCh((uint8_t *)(&context.vt_start), (uint8_t *)(&context.vt_end), context.cur_IT_ch);
            BB_GetRcStarEndByItCh((uint8_t *)(&context.rc_start), (uint8_t *)(&context.rc_end), context.cur_IT_ch);
            context.sky_rc_channel = context.rc_start;
            if(start != context.rc_start)
          	 {
              BB_RcSelectionInit();
           	 }
             //DLOG_Warning("vt_s:%d vt_e:%d rc_s:%d rc_e:%d m:%d rc_ch:%d", context.vt_start,context.vt_end,\
                                                       context.rc_start, context.rc_end, context.cur_IT_ch, context.sky_rc_channel);
        }
    }
}
static void sky_auto_adjust_agc_gain(void)
{
    uint8_t i = 0;
    uint16_t sum = 0;
    uint8_t  aver;
    uint8_t  num = sizeof(stru_skystatus.agc_value1);

    if(1 != stru_skystatus.agc_full)
    {
        return;
    }

    stru_skystatus.agc_full = 0;

    for( i = 0 ; i < num ; i++)
    {
        sum += ((stru_skystatus.agc_value1[i] <= stru_skystatus.agc_value2[i]) ? \
                    stru_skystatus.agc_value1[i] : stru_skystatus.agc_value2[i]);
    }
    aver = sum / num;

    if(AGC_GEAR_1 == stru_skystatus.fct_agc_gear)
    {
        if((stru_skystatus.en_agcmode) != FAR_AGC)
        {
            //BB_SetAgcGain(context.e_curBand, AAGC_GAIN_FAR);
            BB_SetAgcGain(RF_2G, AAGC_GAIN_FAR);
            BB_SetAgcGain(RF_5G, AAGC_GAIN_FAR);
            stru_skystatus.en_agcmode = FAR_AGC;
            DLOG_Warning("=>F 0x%x", aver);
        }

        return;
    }

    if ((aver >= stru_skystatus.fct_agc_thresh) && (stru_skystatus.en_agcmode != FAR_AGC))
    {
        //BB_SetAgcGain(context.e_curBand, AAGC_GAIN_FAR);
        BB_SetAgcGain(RF_2G, AAGC_GAIN_FAR);
        BB_SetAgcGain(RF_5G, AAGC_GAIN_FAR);
        stru_skystatus.en_agcmode = FAR_AGC;
        stru_skystatus.agc_auto_chg_flag = FLAG_INVALID;   //fix agc into far mode when lock missed
        DLOG_Warning("=>F 0x%x", aver);
    }

    if ((aver < stru_skystatus.fct_agc_thresh) && (stru_skystatus.en_agcmode != NEAR_AGC))
    {
        //BB_SetAgcGain(context.e_curBand, AAGC_GAIN_NEAR);
        BB_SetAgcGain(RF_2G, AAGC_GAIN_NEAR);
        BB_SetAgcGain(RF_5G, AAGC_GAIN_NEAR);
        stru_skystatus.en_agcmode = NEAR_AGC;
        stru_skystatus.agc_auto_chg_flag = FLAG_INVALID;   //fix agc into far mode when lock missed
        DLOG_Warning("=>N 0x%x", aver);
    }
}

static void sky_handle_agc_gear_chg_cmd( uint8_t gear)
{
    stru_skystatus.agc_auto_chg_flag = FLAG_VALID;
    BB_SetAgcGain(context.e_curBand, AAGC_GAIN_FAR);
    stru_skystatus.fct_agc_gear = (ENUM_AGC_GEAR)gear;
    DLOG_Warning("agc-gain:%d", stru_skystatus.fct_agc_gear);
}

static void sky_agc_set_init(void)
{
    if (NULL != pstru_agc_set_cfg)
    {
        stru_skystatus.fct_agc_gear = pstru_agc_set_cfg->e_gear;
        stru_skystatus.fct_agc_thresh = pstru_agc_set_cfg->u8_agcSwitchThresh;
        DLOG_Warning("fct %d 0x%x", stru_skystatus.fct_agc_gear, stru_skystatus.fct_agc_thresh);
    }
    else
    {
        stru_skystatus.fct_agc_gear = AGC_GEAR_2;
        stru_skystatus.fct_agc_thresh = 0x50;
        DLOG_Warning("default %d 0x%x", stru_skystatus.fct_agc_gear, stru_skystatus.fct_agc_thresh);
    }
}

static uint8_t sky_BB_write_RcRegs(uint8_t *u8_rc)
{
    uint16_t frq = 0;

    RF_RegValue2Frq(u8_rc, &frq);
    BB_FilterSet(BB_GetFilterByFrq(frq), 1);
    BB_write_RcRegs(u8_rc);

    //DLOG_Warning("rc 0x%x 0x%x 0x%x 0x%x %d", u8_rc[0], u8_rc[1], u8_rc[2], u8_rc[3], frq);
}

static void sky_restore_rf_frq(void)
{
    uint8_t flag = 0;
    uint32_t i = 0;
    uint32_t ch = 0;
    uint8_t max_ch_size = BB_GetRcFrqNum(context.e_curBand);

    for(i=0; i<5; i++)
    {
        if(1 == sky_check_all_mask(i*8, (i+1)*8))
        {
            ch = sky_get_best_ch(i*8, (i+1)*8);
            sky_reset_ch(ch);
            flag = 1;
            DLOG_Warning("reset ch:%d", ch);
        }
    }

    if(flag)
    {
        sky_send_rc_map();
    }
}

void sky_set_unlockCnt(uint8_t count)
{
    u8_unlockCntFromLockToUnlock = count;
}

static void sky_handle_sub_band_set_cmd(uint8_t *arg)
{
    DLOG_Warning("subBB: %d->%d,",context.sub_band_value,arg[0]);
    context.sub_band_value = arg[0];
    band_mode_chg_delay.valid = 1;
    band_mode_chg_delay.cnt = arg[1];
}


void sky_sub_band_excute(uint8_t value)
{
    context.sub_rc_start = BB_GetSubBandStartCH(context.e_curBand,context.st_bandMcsOpt.e_bandwidth,value);
    context.sub_rc_end = BB_GetSubBandEndCH(context.e_curBand,context.st_bandMcsOpt.e_bandwidth,value);
    context.sky_rc_channel = context.sub_rc_start;
    BB_set_Rcfrq(context.e_curBand, context.sky_rc_channel);
    //BB_set_ItFrqByCh(context.e_curBand, value);

}

static int sky_band_mode_run(void)
{
    if(band_mode_chg_delay.valid)
    {
        if(band_mode_chg_delay.cnt == context.sync_cnt)
        {
            band_mode_chg_delay.valid = 0;
            sky_sub_band_excute(context.sub_band_value);
            DLOG_Info("cnt:%d it:%d rc_s:%d rc_e:%d",
                context.sync_cnt,
                context.cur_IT_ch,
                context.sub_rc_start,
                context.sub_rc_end);
        }
    }

    return 0;
}
static uint16_t sky_read_usb0_senddata(void)
{
    uint16_t value = 0;

    value |= BB_ReadReg(PAGE0, 0xf2) << 8;
    value |= BB_ReadReg(PAGE0, 0xf3);
    return value;
}
static uint16_t sky_read_usb1_senddata(void)
{
    uint16_t value = 0;

    value |= BB_ReadReg(PAGE0, 0xf4) << 8;
    value |= BB_ReadReg(PAGE0, 0xf5);
    return value;
}
uint32_t sky_vt_ch_skip_time;
static void sky_vtSkip_process(void)
{
    int32_t swp_e,swp_e_thres;
    static int debug_cnt = 0,vt_change_cnt=0;
    
    swp_e = BB_SweepEnergy();
    if(debug_cnt++ > 100)
    {
        debug_cnt = 0;
        DLOG_Warning("nlbt enable, %d",swp_e);
    }
    
    if(swp_e == 0)
        return;
    
    if(context.st_bandMcsOpt.e_bandwidth == BW_10M)
    {
        swp_e_thres = -65;
    }
    else
    {
        swp_e_thres = -62;
    }

    if(context.vtskip_stat == 0)
    {
        #ifdef NON_LBT_RELEASE
        if(swp_e >= swp_e_thres &&
           SysTicks_GetDiff(sky_vt_ch_skip_time,SysTicks_GetTickCount()) > NON_LBT_VT_CH_SKIP_INTERVAL )
        #else
        	if(swp_e >= swp_e_thres)// || stru_skystatus.check_lock_times > 0)
        #endif
        {
            //DLOG_Warning("%d",swp_e);
            //if(stru_skystatus.check_lock_times > 0)
            //    stru_skystatus.check_lock_times--;
            BB_sky_requestVtSkip();
            context.vtskip_stat = 1; 
            vt_change_cnt = 0;
        }
    }
    else if(context.vtskip_stat == 1)
    {
        BB_set_ItFrqByCh(context.e_curBand, context.sky_sel_vt_ch);
        context.vtskip_stat = 2;
        //context.cur_IT_ch = context.sky_sel_vt_ch;
        context.sky_sel_vt_ch_flag = 1;
        BB_WriteReg(PAGE2, SKY_NON_LBT_VT_CH,0);
    }
    else if(context.vtskip_stat == 2)
    {
        vt_change_cnt++;
        if(!context.sky_sel_vt_ch_flag)
        {
            context.vtskip_stat = 0;
            if(vt_change_cnt > 2)
            {
                DLOG_Warning("->%d",vt_change_cnt);
            }
            vt_change_cnt = 0;

            #ifdef NON_LBT_RELEASE
            sky_vt_ch_skip_time = SysTicks_GetTickCount();
            #else
            if(context.fem_close)
            {
                BB_fem_open(0);
                BB_fem_open(1);
                context.fem_close = 0;
                DLOG_Warning("->BB_fem_open");
            }
            #endif
        }

        #ifndef NON_LBT_RELEASE

        if(vt_change_cnt > 2 && context.sky_sel_vt_ch_flag)
        {
            if(swp_e >= swp_e_thres)
            {
                if(!context.fem_close)
                {
                    BB_fem_close(0);
                    BB_fem_close(1);
                    context.fem_close = 1;
                    DLOG_Warning("->BB_fem_close");
                }
            }
        }

        #endif

    }
}


