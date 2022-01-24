/*****************************************************************************
Copyright: 2016-2020, Artosyn. Co., Ltd.
File name: RF_8003s.c
Description: The internal APIs to control the RF 8003s
Author: Artosy Software Team
Version: 0.0.1
Date: 2016/12/20
History: 
        0.0.1    2016/12/20    The initial version of rf8003s.c
*****************************************************************************/
#include "debuglog.h"
#include "bb_spi.h"
#include "rf_if.h"
#include "systicks.h"
#include "bb_ctrl_internal.h"
#include "cfg_parser.h"
#include "factory.h"
#include "log10.h"

#define     AAGC_GAIN_FAR_8003S                   (0x12)
#define     AAGC_GAIN_NEAR_8003S                  (0x3F)


extern uint8_t RF_8003s_regs0[128];
extern uint8_t RF_8003s_regs1[128];

static uint8_t cali_reg[2][10];

static uint8_t *pu8_rf0_regs;
static uint8_t *pu8_rf1_regs;

static STRU_FRQ_CHANNEL pstru_rcFreq_2g[240];//40*6
static STRU_FRQ_CHANNEL pstru_itFreq_2g[40];
static int16_t Gen2GrcFreq[240];

static uint8_t u8_2g_bb_power_a[240];
static uint8_t u8_2g_bb_power_b[240];

static uint8_t rcFreqCnt_2g;
static uint8_t itFreqCnt_2g;

static uint16_t *pRcFreqlist_2g = NULL;
static uint16_t *pItFreqlist_2g = NULL;


static STRU_FRQ_CHANNEL pstru_rcFreq_5g[240];//40*6
static STRU_FRQ_CHANNEL pstru_itFreq_5g[40];
static int16_t Gen5GrcFreq[240];

static uint8_t u8_5g_bb_power_a[240];
static uint8_t u8_5g_bb_power_b[240];

static uint8_t u8_rcFreqCnt_5g;
static uint8_t u8_itFreqCnt_5g;

static uint16_t *pRcFreqlist_5g = NULL;
static uint16_t *pItFreqlist_5g = NULL;


static STRU_RF_REG *pstru_rf0_regBeforeCali;
static STRU_RF_REG *pstru_rf1_regBeforeCali;

static STRU_RF_REG *pstru_rf0_regAfterCali;
static STRU_RF_REG *pstru_rf1_regAfterCali;

static STRU_BOARD_RF_PARA *pstru_rf_boardcfg;

static STRU_BB_POWER_CLOSE *pst_close_power = NULL; 
static STRU_BB_CH_OPEN_POWER_REF_VALUE *pst_bbPowerOpenRefValue = NULL;
static STRU_FRQ_CHANNEL pstru_skySweeFreq_2g[40];
static STRU_FRQ_CHANNEL pstru_skySweeFreq_5g[40];
static uint8_t u8_skySweeFreqCnt_2g;
static uint8_t u8_skySweeFreqCnt_5g;
static uint8_t u8_subBand10MrcFreqNum;
static uint8_t u8_subBand20MrcFreqNum;

static void RF8003s_CalcSweepRegTable(STRU_cfgBin *cfg);
static void BB_gen_open_power_makeup_value(ENUM_BB_MODE en_mode, ENUM_CH_BW bw);

void RF8003s_getCfgData(ENUM_BB_MODE en_mode, STRU_cfgBin *cfg)
{
    STRU_cfgNode  *rfcfg_node;

    pu8_rf0_regs = RF_8003s_regs0;
    pu8_rf1_regs = RF_8003s_regs1;

    pstru_rf_boardcfg  = CFGBIN_GetNodeData(cfg, RF_BOARDCFG_PARA_ID);
    if (NULL == pstru_rf_boardcfg)
    {
        DLOG_Error("Not find board cfg");
        return;
    }
    else
    {
        DLOG_Info("board name:%s", pstru_rf_boardcfg->boardCfgName);
    }

    STRU_RF_REG * p_rf_reg  = (STRU_RF_REG *)CFGBIN_GetNodeData(cfg, RF_BOARDCFG_DATA_ID);
    if (NULL != p_rf_reg)
    {
        if (en_mode == BB_SKY_MODE)
        {
            pstru_rf0_regBeforeCali = p_rf_reg;
            pstru_rf1_regBeforeCali = p_rf_reg; 

            pstru_rf0_regAfterCali = pstru_rf0_regBeforeCali + pstru_rf_boardcfg->u8_rf0SkyRegsCnt + pstru_rf_boardcfg->u8_rf0GrdRegsCnt;
            pstru_rf1_regAfterCali = pstru_rf0_regAfterCali;
        }
        else
        {
            pstru_rf0_regBeforeCali =  p_rf_reg + pstru_rf_boardcfg->u8_rf0SkyRegsCnt;
            pstru_rf1_regBeforeCali =  pstru_rf0_regBeforeCali + pstru_rf_boardcfg->u8_rf0GrdRegsCnt;
        
            pstru_rf0_regAfterCali = pstru_rf1_regBeforeCali + pstru_rf_boardcfg->u8_rf0SkyRegsCntAfterCali;
            pstru_rf1_regAfterCali = pstru_rf0_regAfterCali  + pstru_rf_boardcfg->u8_rf0GrdRegsCntAfterCali;
        }
    }

    RF8003s_CalcSweepRegTable(cfg);
}

void RF8003xCalcFrq2Register(uint16_t u16_frq, STRU_FRQ_CHANNEL *frq_regvalue)
{
    uint8_t  integer;
    uint32_t fraction;

    if (u16_frq > 4000)  //5.8G frequency
    {
        integer  = (uint32_t)u16_frq / 60;
        fraction = ((double) u16_frq / 60 - integer) * (1 << 24);
    }
    else
    {
        integer  = (uint32_t)u16_frq / 30;
        fraction = ((double) u16_frq / 30 - integer) * (1 << 24);
    }

    frq_regvalue->frq1 = (fraction & 0xff);
    frq_regvalue->frq2 = ((fraction >>  8) & 0xff);
    frq_regvalue->frq3 = ((fraction >> 16) & 0xff);
    frq_regvalue->frq4 = integer;

    DLOG_Info("%d %x %x %x %x", u16_frq, frq_regvalue->frq1, frq_regvalue->frq2, frq_regvalue->frq3, frq_regvalue->frq4);
}

uint16_t RF8003xCalcRegister2Frq(uint32_t u32_frq_reg)
{
    uint32_t tmp;
    
    if((u32_frq_reg & 0x000000ff) >= 0x55) // 5g
    {
        tmp = u32_frq_reg >> 8;
        tmp = ((tmp & 0xff ) << 16) | ((tmp >> 16) & 0xff) | (tmp & 0x00ff00);
        DLOG_Warning("%x.",tmp);
       return (((double)(tmp )) / (1 << 24) +((double) ((u32_frq_reg) & 0x000000ff))) * 60 + 0.5;
    }
    else // 2g
    {
        tmp = u32_frq_reg >> 8;
        tmp = ((tmp & 0xff ) << 16) | ((tmp >> 16) & 0xff) | (tmp & 0x00ff00);
        DLOG_Warning("%x",tmp);
        return (((double)(tmp )) / (1 << 24) +((double) ((u32_frq_reg) & 0x000000ff))) * 30 + 0.5;
    }
}

static void RF8003s_CalcFctNode2RegTable(uint32_t nodeid, STRU_FRQ_CHANNEL *frq_regvalue, uint8_t * frq_cnt)
{
    uint32_t i = 0;

    STRU_cfgNode *node;
    STRU_RF_CHANNEL * p_frq = FCT_GetNodeAndData(nodeid, &node);

    if (NULL == p_frq)
    {
        DLOG_Error("Fail Node = %x", nodeid);
        return;
    }

    DLOG_Info("Node id = %x %d", nodeid, p_frq->u32_rfChCount);

    if(p_frq->u32_rfChCount > 40)
    {
        p_frq->u32_rfChCount = 40;
    }
    *frq_cnt = p_frq->u32_rfChCount;
    for (i = 0; i < p_frq->u32_rfChCount; i++)
    {
        RF8003xCalcFrq2Register(p_frq->u16_rfChFrqList  [i], (frq_regvalue + i));
    }

}

static void RF8003s_CalcSweepRegTable(STRU_cfgBin *cfg)
{
    STRU_SKYSWEEPFRQ *sweepFrq = CFGBIN_GetNodeData(cfg, BB_SKY_SWEEP_FRQ_CFG_ID);

    if (sweepFrq)
    {
        uint8_t i;
        u8_skySweeFreqCnt_2g = sweepFrq->u8_band0SweepChannelCnt;
        u8_skySweeFreqCnt_5g = sweepFrq->u8_band1SweepChannelCnt;

        for (i = 0; i < u8_skySweeFreqCnt_2g; i++)
        {
            RF8003xCalcFrq2Register(sweepFrq->u16_band0SweepFrq[i], pstru_skySweeFreq_2g + i);
        }

        for (i = 0; i < u8_skySweeFreqCnt_5g; i++)
        {
            RF8003xCalcFrq2Register(sweepFrq->u16_band1SweepFrq[i], pstru_skySweeFreq_5g + i);
        }
    }
}

void RF8003s_GetFctFreqTable(ENUM_CH_BW e_bw)
{
    uint8_t i = 0,j=0;
    uint32_t nodeid_band0,nodeid_band1,freq_cnt;
    STRU_cfgNode *node,*node_vt;
    STRU_RF_CHANNEL *p_frq_vt;
	STRU_RF_CHANNEL *p_frq_2g,*p_frq_5g,*p_frq_vt_2g_10M,*p_frq_vt_5g_10M,*p_frq_vt_2g_20M,*p_frq_vt_5g_20M;

	//DLOG_Warning("e_bw=%d",e_bw);

    if (e_bw == BW_10M)
    {
        RF8003s_CalcFctNode2RegTable(FACTORY_SUBNODE_BAND0_VT_10M_FRQ_ID, pstru_itFreq_2g, &itFreqCnt_2g);
        RF8003s_CalcFctNode2RegTable(FACTORY_SUBNODE_BAND1_VT_10M_FRQ_ID, pstru_itFreq_5g, &u8_itFreqCnt_5g);
		nodeid_band0 = FACTORY_SUBNODE_BAND0_VT_10M_FRQ_ID;
		nodeid_band1 = FACTORY_SUBNODE_BAND1_VT_10M_FRQ_ID;
    }
    else
    {
        RF8003s_CalcFctNode2RegTable(FACTORY_SUBNODE_BAND0_VT_20M_FRQ_ID, pstru_itFreq_2g, &itFreqCnt_2g);
        RF8003s_CalcFctNode2RegTable(FACTORY_SUBNODE_BAND1_VT_20M_FRQ_ID, pstru_itFreq_5g, &u8_itFreqCnt_5g);
		nodeid_band0 = FACTORY_SUBNODE_BAND0_VT_20M_FRQ_ID;
		nodeid_band1 = FACTORY_SUBNODE_BAND1_VT_20M_FRQ_ID;
    }
	 // DLOG_Warning("itFreqCnt_2g=%d,u8_itFreqCnt_5g=%d",itFreqCnt_2g,u8_itFreqCnt_5g);
    p_frq_2g = (STRU_RF_CHANNEL *)FCT_GetNodeAndData(FACTORY_SUBNODE_BAND0_RC_10M_FRQ_ID, &node);
    if (NULL == p_frq_2g)
    {
       // DLOG_Error("Fail Node = %x", FACTORY_SUBNODE_BAND0_RC_10M_FRQ_ID);
        return;
    }

   /* if(p_frq_2g->u32_rfChCount > 0 && p_frq_2g->u16_rfChFrqList[0] < 100)//real freq value must > 100, e.g 2400,5800
    {
        //DLOG_Warning("2g auto generate rc freq");
        p_frq_vt = (STRU_RF_CHANNEL *)FCT_GetNodeAndData(nodeid_band0, &node_vt);
        if (NULL == p_frq_vt)
        {
           // DLOG_Error("Fail Node = %x", nodeid_band0);
            return;
        }
		pItFreqlist_2g = p_frq_vt->u16_rfChFrqList;
        freq_cnt = p_frq_2g->u32_rfChCount;
        if(p_frq_2g->u32_rfChCount * p_frq_vt->u32_rfChCount > 240)
        {
            freq_cnt = 240 / p_frq_vt->u32_rfChCount;
           // DLOG_Warning("2g force sub rc %d->%d",p_frq_2g->u32_rfChCount,freq_cnt);
        }

        for(i=0;i<p_frq_vt->u32_rfChCount;i++)
        {
            for(j=0;j<freq_cnt;j++)
            {
                Gen2GrcFreq[i*freq_cnt+j] = p_frq_vt->u16_rfChFrqList  [i] + p_frq_2g->u16_rfChFrqList[j];
                RF8003xCalcFrq2Register(p_frq_vt->u16_rfChFrqList  [i] + p_frq_2g->u16_rfChFrqList[j], (pstru_rcFreq_2g + i*freq_cnt+j));
            }
        }
        rcFreqCnt_2g = freq_cnt * p_frq_vt->u32_rfChCount;
        u8_subBand10MrcFreqNum = freq_cnt;
        //DLOG_Warning("2g rc frq %d %d",u8_subBand10MrcFreqNum,rcFreqCnt_2g);
    }
  */
  //  else  
    {
        RF8003s_CalcFctNode2RegTable(FACTORY_SUBNODE_BAND0_RC_10M_FRQ_ID, pstru_rcFreq_2g, &rcFreqCnt_2g);
		p_frq_2g = (STRU_RF_CHANNEL *)FCT_GetNodeAndData(FACTORY_SUBNODE_BAND0_RC_10M_FRQ_ID, &node);
		pRcFreqlist_2g = p_frq_2g->u16_rfChFrqList;
    }
    
    p_frq_5g = (STRU_RF_CHANNEL *)FCT_GetNodeAndData(FACTORY_SUBNODE_BAND1_RC_10M_FRQ_ID, &node);
    if (NULL == p_frq_5g)
    {
       // DLOG_Error("Fail Node = %x", FACTORY_SUBNODE_BAND1_RC_10M_FRQ_ID);
        return;
    }
/*
    if(p_frq_5g->u32_rfChCount > 0 && p_frq_5g->u16_rfChFrqList[0] < 100)
    {
        //DLOG_Warning("5g auto generate rc freq");
        p_frq_vt = (STRU_RF_CHANNEL *)FCT_GetNodeAndData(nodeid_band1, &node_vt);
        if (NULL == p_frq_vt)
        {
            //DLOG_Error("Fail Node = %x", nodeid_band1);
            return;
        }
		pItFreqlist_5g = p_frq_vt->u16_rfChFrqList;
        freq_cnt = p_frq_5g->u32_rfChCount;
        if(p_frq_5g->u32_rfChCount * p_frq_vt->u32_rfChCount > 240)
        {
            freq_cnt = 240 / p_frq_vt->u32_rfChCount;
            //DLOG_Warning("5g force sub rc %d->%d",p_frq_5g->u32_rfChCount,freq_cnt);
        }

        for(i=0;i<p_frq_vt->u32_rfChCount;i++)
        {
            for(j=0;j<freq_cnt;j++)
            {
                Gen5GrcFreq[i*freq_cnt+j] = p_frq_vt->u16_rfChFrqList  [i] + p_frq_5g->u16_rfChFrqList[j];
                RF8003xCalcFrq2Register(p_frq_vt->u16_rfChFrqList  [i] + p_frq_5g->u16_rfChFrqList[j], (pstru_rcFreq_5g + i*freq_cnt+j));
            }
        }
        u8_rcFreqCnt_5g = freq_cnt * p_frq_vt->u32_rfChCount;
        u8_subBand20MrcFreqNum = freq_cnt;
       // DLOG_Warning("5g rc frq %d %d",u8_subBand20MrcFreqNum,u8_rcFreqCnt_5g);

    }
    */
   // else
    {
        RF8003s_CalcFctNode2RegTable(FACTORY_SUBNODE_BAND1_RC_10M_FRQ_ID, pstru_rcFreq_5g, &u8_rcFreqCnt_5g);
		p_frq_5g = (STRU_RF_CHANNEL *)FCT_GetNodeAndData(FACTORY_SUBNODE_BAND1_RC_10M_FRQ_ID, &node);
    	pRcFreqlist_5g = p_frq_5g->u16_rfChFrqList;
    }

	if (e_bw == BW_10M)
    {
	
	    p_frq_vt_5g_10M = FCT_GetNodeAndData(FACTORY_SUBNODE_BAND1_VT_10M_FRQ_ID, &node_vt);
	    pItFreqlist_5g = p_frq_vt_5g_10M->u16_rfChFrqList;
		p_frq_vt_2g_10M = FCT_GetNodeAndData(FACTORY_SUBNODE_BAND0_VT_10M_FRQ_ID, &node_vt);
		pItFreqlist_2g = p_frq_vt_2g_10M->u16_rfChFrqList;
		
    }
    else
    {
		
	    p_frq_vt_5g_20M = FCT_GetNodeAndData(FACTORY_SUBNODE_BAND1_VT_20M_FRQ_ID, &node_vt);
	    pItFreqlist_5g = p_frq_vt_5g_20M->u16_rfChFrqList;
		p_frq_vt_2g_20M = FCT_GetNodeAndData(FACTORY_SUBNODE_BAND0_VT_20M_FRQ_ID, &node_vt);
		 pItFreqlist_2g = p_frq_vt_2g_20M->u16_rfChFrqList;
    }
	//DLOG_Critical("cnt = %d", itp_frq->u32_rfChCount);
	//for(i=0;i<itFreqCnt_2g;i++)
	//	DLOG_Critical("it2g[%d]=%d",i,pItFreqlist_2g[i]);
	//for(i=0;i<u8_rcFreqCnt_5g;i++)
	//	DLOG_Critical("it5g[%d]=%d",i,pRcFreqlist_5g[i]);
	
	//DLOG_Critical("itFreqCnt_2g=%d,u8_rcFreqCnt_5g=%d",itFreqCnt_2g,u8_rcFreqCnt_5g);

}

#define  RF8003S_RF_CLOCKRATE    (1)    //1MHz clockrate

static int RF8003s_SPI_WriteReg_internal(uint8_t u8_addr, uint8_t u8_data)
{
    int ret = 0;
    uint8_t wdata[] = {0x80, (u8_addr <<1), u8_data};   //RF_8003S_SPI: wr: 0x80 ; 

    //SPI_master_init(BB_SPI_BASE_IDX, &init);
    SPI_write_read(BB_SPI_BASE_IDX, wdata, sizeof(wdata), 0, 0); 
    ret =  SPI_WaitIdle(BB_SPI_BASE_IDX, BB_SPI_MAX_DELAY);

    return ret;
}


static int RF8003s_SPI_ReadReg_internal(uint8_t u8_addr)
{
    uint8_t wdata[2] = {0x00, (u8_addr<<1)};      //RF_8003S_SPI:  rd: 0x00
    uint8_t rdata;
    
    //use low speed for the RF8003 read, from test, read fail if use the same clockrate as baseband
    STRU_SPI_InitTypes init = {
        .ctrl0   = SPI_CTRL0_DEF_VALUE,
        .clk_Mhz = RF8003S_RF_CLOCKRATE,
        .Tx_Fthr = SPI_TXFTLR_DEF_VALUE,
        .Rx_Ftlr = SPI_RXFTLR_DEF_VALUE,
        .SER     = SPI_SSIENR_DEF_VALUE
    };

    SPI_master_init(BB_SPI_BASE_IDX, &init);
 
    SPI_write_read(BB_SPI_BASE_IDX, wdata, sizeof(wdata), &rdata, 1);
    SPI_WaitIdle(BB_SPI_BASE_IDX, BB_SPI_MAX_DELAY);

    BB_SPI_init();

    return rdata;
}

/**
  * @brief : Write 8003 RF register by SPI 
  * @param : addr: 8003 SPI address
  * @param : data: data for 8003
  * @retval  0: sucess   1: FAIL
  */
int RF_SPI_WriteReg(uint16_t u16_addr, uint8_t u8_data)
{
    return RF8003s_SPI_WriteReg_internal((uint8_t)u16_addr, u8_data);
}

/**
  * @brief : Read 8003 RF register by SPI 
  * @param : addr: 8003 SPI address
  * @retval  0: sucess   1: FAIL
  */
int RF_SPI_ReadReg(uint16_t u16_addr, uint8_t *pu8_rxValue)
{
    uint8_t tmp = RF8003s_SPI_ReadReg_internal( (uint8_t)u16_addr);
    if( pu8_rxValue)
    {
        *pu8_rxValue = tmp;
    }

    return 0;
}

/**
  * @brief : init RF8003s register
  * @param : addr: 8003 SPI address
  * @retval  None
  */
void RF_init(ENUM_BB_MODE en_mode)
{
    uint8_t idx;
    uint8_t cnt;
    uint8_t num;

    STRU_RF_REG * pstru_rfReg = NULL;

    RF8003s_GetFctFreqTable(context.st_bandMcsOpt.e_bandwidth);

    BB_gen_open_power_makeup_value(en_mode, context.st_bandMcsOpt.e_bandwidth);

	if(en_mode == BB_SKY_MODE)
	{
		pst_close_power = FCT_GetNodeAndData(FACTORY_SUBNODE_POWERCLOSE_VT_SET_ID, NULL);
	}
	else
	{
		pst_close_power = FCT_GetNodeAndData(FACTORY_SUBNODE_POWERCLOSE_RC_SET_ID, NULL);
	}
    RF8003s_getCfgData(en_mode, (STRU_cfgBin *)SRAM_CONFIGURE_MEMORY_ST_ADDR);

    //RF 0
    {
        BB_SPI_curPageWriteByte(0x01,0x01);             //bypass: SPI change into 8003

        if ( pstru_rf_boardcfg != NULL )
        {        
            if ( en_mode == BB_SKY_MODE )               //sky mode register replace
            {
                num = pstru_rf_boardcfg->u8_rf0SkyRegsCnt;
                pstru_rfReg = pstru_rf0_regBeforeCali;
            }
            else                                        //ground mode register replace
            {
                num = pstru_rf_boardcfg->u8_rf0GrdRegsCnt;
                pstru_rfReg = (STRU_RF_REG * )pstru_rf0_regBeforeCali;
            }

            for (cnt = 0; (pstru_rfReg != NULL) && (cnt < num); cnt++)
            {
                pu8_rf0_regs[pstru_rfReg[cnt].addr_l] = pstru_rfReg[cnt].value;
            } 
        }
        
        for(idx = 0; idx < 128; idx++)
        {
            RF8003s_SPI_WriteReg_internal(idx, pu8_rf0_regs[idx]);
        }

        {
            //add patch, reset 8003
            RF8003s_SPI_WriteReg_internal(0x15, 0x51);
            RF8003s_SPI_WriteReg_internal(0x15, 0x50);
        }
        
        BB_SPI_curPageWriteByte(0x01,0x02);             //SPI change into 8020
    }

    //RF 1 only used in ground   
    if (pstru_rf_boardcfg != NULL && pstru_rf_boardcfg->u8_rfCnt > 1 && en_mode == BB_GRD_MODE )
    {
        num = pstru_rf_boardcfg->u8_rf1GrdRegsCnt;
        pstru_rfReg = pstru_rf1_regBeforeCali;
        
        BB_SPI_curPageWriteByte(0x01,0x03);             //bypass: SPI change into 2rd 8003s
        
        for (cnt = 0; (pstru_rfReg != NULL) && (cnt < num); cnt++)
        {
            pu8_rf1_regs[pstru_rfReg[cnt].addr_l] = pstru_rfReg[cnt].value;
        }

        for(idx = 0; idx < 128; idx++)
        {
            RF8003s_SPI_WriteReg_internal( idx, pu8_rf1_regs[idx]);
        }

        {
            //add patch, reset 8003
            RF8003s_SPI_WriteReg_internal(0x15, 0x51);
            RF8003s_SPI_WriteReg_internal(0x15, 0x50);
        } 
        
        BB_SPI_curPageWriteByte(0x01,0x02);             //SPI change into 8020
    }
}





static void RF8003s_afterCali(ENUM_BB_MODE en_mode, STRU_BOARD_RF_PARA *pstru_rf_boardcfg)
{
    STRU_RF_REG * pu8_rf1_regs, * rf2_regs;
    uint8_t cnt;
    uint8_t rf_regcnt1, rf_regcnt2;


    if( NULL == pstru_rf_boardcfg)
    {
        return;
    }

    if (en_mode == BB_SKY_MODE)
    {
        rf_regcnt1 = pstru_rf_boardcfg->u8_rf0SkyRegsCntAfterCali;
        rf_regcnt2 = 0;

        pu8_rf1_regs   = pstru_rf0_regAfterCali;
        rf2_regs   = NULL;
    }
    else
    {
        rf_regcnt1 = pstru_rf_boardcfg->u8_rf0GrdRegsCntAfterCali;
        rf_regcnt2 = pstru_rf_boardcfg->u8_rf1GrdRegsCntAfterCali;

        pu8_rf1_regs   = (STRU_RF_REG * )pstru_rf0_regAfterCali;
        rf2_regs   = (STRU_RF_REG * )pstru_rf1_regAfterCali;
    }


    if ( rf_regcnt1 > 0 && pu8_rf1_regs != NULL)
    {
        BB_SPI_curPageWriteByte(0x01,0x01);             //bypass: SPI change into 1st 8003s
        
        for(cnt = 0; cnt < rf_regcnt1; cnt++)
        {
            RF8003s_SPI_WriteReg_internal(pu8_rf1_regs[cnt].addr_l, pu8_rf1_regs[cnt].value);
        }


        {
            //add patch, reset 8003
            uint16_t delay = 0;
            RF8003s_SPI_WriteReg_internal(0x15, 0x51);
            while(delay ++ < 1000);
            RF8003s_SPI_WriteReg_internal(0x15, 0x50);
        } 
        
        BB_SPI_curPageWriteByte(0x01,0x02);             //SPI change into 8020    
    }

    if (pstru_rf_boardcfg->u8_rfCnt > 1 && rf_regcnt2 > 0 && rf2_regs != NULL)
    {
        BB_SPI_curPageWriteByte(0x01,0x03);             //bypass: SPI change into 2rd 8003s
        
        for(cnt = 0; cnt < rf_regcnt2; cnt++)
        {
            RF8003s_SPI_WriteReg_internal( rf2_regs[cnt].addr_l, rf2_regs[cnt].value);
        }

        {
            //add patch, reset 8003
            uint16_t delay = 0;
            RF8003s_SPI_WriteReg_internal(0x15, 0x51);
            while(delay ++ < 1000);            
            RF8003s_SPI_WriteReg_internal(0x15, 0x50);
        } 
        
        BB_SPI_curPageWriteByte(0x01,0x02);             //SPI change into 8020    
    }

    BB_SPI_init();
}



static int BB_before_RF_cali(void)
{
    BB_WriteRegMask(PAGE0, 0x20, 0x00, 0x0c);
}


void read_cali_register(uint8_t *buf)
{
    buf[0] = BB_ReadReg(PAGE0, 0xd0);
    buf[1] = BB_ReadReg(PAGE0, 0xd1);
    buf[2] = BB_ReadReg(PAGE0, 0xd2);
    buf[3] = BB_ReadReg(PAGE0, 0xd3);
    buf[4] = BB_ReadReg(PAGE0, 0xd4);
    buf[5] = BB_ReadReg(PAGE0, 0xd5);
    buf[6] = BB_ReadReg(PAGE0, 0xd6);
    buf[7] = BB_ReadReg(PAGE0, 0xd7);
    buf[8] = BB_ReadReg(PAGE0, 0xd8);
    buf[9] = BB_ReadReg(PAGE0, 0xd9);
}


static void BB_RF_start_cali( void )
{
    uint8_t data;

    //step 1
    //1.1 Enable RF calibration 0x61= 0x0F
    BB_WriteReg(PAGE0, TX_CALI_ENABLE, 0x0F);

    //1.2: soft reset0
    //page0 0x64[4] = 1'b1
    data = BB_ReadReg(PAGE0, 0x64);
    data = data | 0x10;
    BB_WriteReg(PAGE0, 0x64, data);
    //page0 0x64[4] = 1'b0
    data = data & 0xEF;
    BB_WriteReg(PAGE0, 0x64, data);

    data = BB_ReadReg(PAGE0, 0x00);
    data |= 0x01;
    BB_WriteReg(PAGE0, 0x00, data);
    //page0 0x64[4] = 1'b0
    data &= 0xFE;
    BB_WriteReg(PAGE0, 0x00, data);

    //1.3: wait 1s
    SysTicks_DelayMS(1000);

    //select the 2G,  Read RF calibration register values
    data = BB_ReadReg(PAGE0, 0x64);
    BB_WriteReg(PAGE0, 0x64, (data&0x7F));
    read_cali_register(cali_reg[0]);

    //select the 5G,  Read RF calibration register values
    BB_WriteReg(PAGE0, 0x64, (data | 0x80));
    read_cali_register(cali_reg[1]);
}



void BB_RF_band_switch(ENUM_RF_BAND rf_band)
{
    uint8_t data, data1;
    uint8_t tmpdata[2] = {rf_band, rf_band};

    char *regbuf = ((rf_band==RF_2G) ? cali_reg[0]:cali_reg[1]); 

    //write 0xd0[7] -> 0x67[7]
    data = (BB_ReadReg(PAGE0, 0x67) & 0x7f) | (regbuf[0] & 0x80);
    BB_WriteReg(PAGE0, 0x67, data);

    //write 0xd0[5:2] -> 0x68[7:4]
    data1 = ((regbuf[0] & 0x3c) << 2);
    data = (BB_ReadReg(PAGE0, 0x68) & 0x0f) | data1;
    BB_WriteReg(PAGE0, 0x68, data);

    //write 0xd1[7] -> 0x67[6]
    data = (BB_ReadReg(PAGE0, 0x67) & 0xbf) |  ((regbuf[1] & 0x80) >> 1);
    BB_WriteReg(PAGE0, 0x67, data);

    //write 0xd1[5:2] -> 0x68[3:0]
    data1 = ((regbuf[1] & 0x3c) >> 2);
    data = (BB_ReadReg(PAGE0, 0x68) & 0xf0) | data1;
    BB_WriteReg(PAGE0, 0x68, data);
    
    //write 0xd4[7] -> 0x67[5]
    data = (BB_ReadReg(PAGE0, 0x67) &  0xdf) | ((regbuf[4] & 0x80) >> 2);
    BB_WriteReg(PAGE0, 0x67, data);

    //write 0xd4[6:3] -> 0x6a[7:4]
    data1 = ((regbuf[4] & 0x78) << 1);
    data = (BB_ReadReg(PAGE0, 0x6A) & 0x0f) | data1;
    BB_WriteReg(PAGE0, 0x6A, data);
    
    //0xd3[3:0]->0x6a[3:0]
    data = (BB_ReadReg(PAGE0, 0x6A) & 0xf0) | (regbuf[3] & 0x0f);;
    BB_WriteReg(PAGE0, 0x6A, data);
    
    //0xd2[7:0] -> 0x6b[7:0]
    BB_WriteReg(PAGE0, 0x6b, regbuf[2]);
    
    uint16_t tmp = (((uint16_t)regbuf[3] & 0x0f) << 8 ) | regbuf[2]; //abs(tmp[11:0])
    if(tmp & 0x800) //tmp[11]
    {
        tmp = (~tmp) + 1;
        tmp &= 0x0fff;
    }

    typedef struct _STRU_thresh_regvalue
    {
        uint16_t thresh;
        uint8_t value;
    }STRU_thresh_regvalue;

    STRU_thresh_regvalue thresh_regvalue[] = 
    {
        {0x41,  0xFF}, {0x60,  0xFE}, {0x70,  0xFD}, {0x80,  0xFC}, 
        {0x8F,  0xFB}, {0x9F,  0xFA}, {0xAF,  0xF8}, {0xBE,  0xF7}, 
        {0xCE,  0xF6}, {0xDD,  0xF4}, {0xED,  0xF2}, {0xFD,  0xF0}, 
        {0x10C, 0xEE}, {0x11C, 0xEC}, {0x12B, 0xEA}, {0x13B, 0xE7}, 
        {0x14A, 0xE5}, {0x15A, 0xE2}, {0x169, 0xE0}, {0x179, 0xDD}, 
        {0x188, 0xDA}, {0x198, 0xD7}, {0x1A7, 0xD4}, {0x1B6, 0xD0},
        {0x1C6, 0xCD}, {0x1D5, 0xC9},
    };

    uint8_t regvalue = 0xc6;
    uint8_t idx = 0;
    for(idx = 0; idx < sizeof(thresh_regvalue)/sizeof(thresh_regvalue[0]); idx++)
    {
        if(tmp <= thresh_regvalue[idx].thresh)
        {
            regvalue = thresh_regvalue[idx].value;
            break;
        }
    }

    BB_WriteReg(PAGE0, 0x69, regvalue);

    //write 0xd5[7] -> 0x67[3]
    data1 = (regbuf[5] & 0x80) >> 4;
    data = BB_ReadReg(PAGE0, 0x67) & 0xf7 | data1;
    BB_WriteReg(PAGE0, 0x67, data);
    
    //0xd5[5:2]->0x67[7:4]
    data1 = (regbuf[5] & 0x3c) << 2;
    data = (BB_ReadReg(PAGE0, 0x6c) & 0x0f) | data1;
    BB_WriteReg(PAGE0, 0x6c, data);

    //write 0xd6[7] -> 0x67[2]
    data1 = (regbuf[6] & 0x80) >> 5;
    data = (BB_ReadReg(PAGE0, 0x67) & 0xfb) | data1;
    BB_WriteReg(PAGE0, 0x67, data);

    //0xd6[5:2]->0x6c[3:0]
    data1 = (regbuf[6] >> 2) & 0x0f ;
    data = (BB_ReadReg(PAGE0, 0x6c) & 0xf0) | data1;
    BB_WriteReg(PAGE0, 0x6c, data);

    //write 0xd9[7] -> 0x67[1]
    data1 = (regbuf[9] & 0x80) >> 6;
    data = (BB_ReadReg(PAGE0, 0x67) & 0xfd) | data1;
    BB_WriteReg(PAGE0, 0x67, data);
    
    //write 0xd9[6:3] -> 0x6E[7:4]
    data1 = (regbuf[9]<<1) & 0xf0;
    data = (BB_ReadReg(PAGE0, 0x6e) & 0x0f) | data1;
    BB_WriteReg(PAGE0, 0x6e, data); 

    //0xd8[3:0] -> 0x6E[3:0]
    data1 = regbuf[8] & 0x0f;
    data = (BB_ReadReg(PAGE0, 0x6e) & 0xf0) | data1;
    BB_WriteReg(PAGE0, 0x6e, data);

    //0xd7[7:0] -> 0x6F[7:0]
    BB_WriteReg(PAGE0, 0x6f, regbuf[7]);

    tmp = (((uint16_t)regbuf[8] & 0x0f)<<8) | regbuf[7];
    if(tmp & 0x800) //tmp[11]
    {
        tmp = (~tmp) + 1;
        tmp &= 0x0fff;
    }

    regvalue = 0xc6;
    for(idx = 0; idx < sizeof(thresh_regvalue)/sizeof(thresh_regvalue[0]); idx++)
    {
        if(tmp <= thresh_regvalue[idx].thresh)
        {
            regvalue = thresh_regvalue[idx].value;
            break;
        }
    }

    BB_WriteReg(PAGE0, 0x6d, regvalue);
    BB_WriteRegMask(PAGE0, 0x60, 0x02, 0x02);   //fix calibration result.

#if 0
    data = BB_ReadReg(PAGE0, 0x00);
    data |= 0x01;
    BB_WriteReg(PAGE0, 0x00, data);
    
    data &= 0xfe;
    BB_WriteReg(PAGE0, 0x00, data);  
#endif

    //BB_WriteReg(PAGE2, RF_BAND_CHANGE_0, rf_band);
    //BB_WriteReg(PAGE2, RF_BAND_CHANGE_1, rf_band);  
    if ((rf_band != context.e_curBand) && (BB_GRD_MODE == context.en_bbmode))
    {
        //BB_DtSendToBuf(DT_NUM_RF_BAND_CHANGE, tmpdata);
    }
}


void RF_CaliProcess(ENUM_BB_MODE en_mode)
{
    BB_before_RF_cali();

    BB_RF_start_cali();

    BB_WriteReg(PAGE0, TX_CALI_ENABLE, 0x00);   //disable calibration

    RF8003s_afterCali(en_mode, pstru_rf_boardcfg);
}

void BB_write_ItRegs(uint8_t *u8_it)
{
    context.stru_itRegs.frq1 = u8_it[0];
    context.stru_itRegs.frq2 = u8_it[1];
    context.stru_itRegs.frq3 = u8_it[2];
    context.stru_itRegs.frq4 = u8_it[3];
    context.stru_itRegs.frq5 = u8_it[4];

    BB_WriteReg(PAGE2, AGC3_0, context.stru_itRegs.frq1);
    BB_WriteReg(PAGE2, AGC3_1, context.stru_itRegs.frq2);
    BB_WriteReg(PAGE2, AGC3_2, context.stru_itRegs.frq3);
    BB_WriteReg(PAGE2, AGC3_3, context.stru_itRegs.frq4);
}

uint8_t BB_set_ItFrqByCh(ENUM_RF_BAND band, uint8_t ch)
{
    STRU_FRQ_CHANNEL *it_ch_ptr = ((band == RF_2G)?pstru_itFreq_2g:pstru_itFreq_5g);

    context.stru_itRegs.frq1 = it_ch_ptr[ch].frq1;
    context.stru_itRegs.frq2 = it_ch_ptr[ch].frq2;
    context.stru_itRegs.frq3 = it_ch_ptr[ch].frq3;
    context.stru_itRegs.frq4 = it_ch_ptr[ch].frq4;

    BB_WriteReg(PAGE2, AGC3_0, it_ch_ptr[ch].frq1);
    BB_WriteReg(PAGE2, AGC3_1, it_ch_ptr[ch].frq2);
    BB_WriteReg(PAGE2, AGC3_2, it_ch_ptr[ch].frq3);
    BB_WriteReg(PAGE2, AGC3_3, it_ch_ptr[ch].frq4);

	if(!pst_bbPowerOpenRefValue->u8_powerOpenMakeupEnable)
	{
		return 0;
	}
    //set channel power
    if (context.en_bbmode == BB_SKY_MODE)
    {
		if(context.e_powerMode == RF_POWER_CLOSE)
		{
			if((band == RF_2G))
			{
				BB_WriteReg(PAGE0, 0x09, u8_2g_bb_power_a[ch]);
				BB_WriteReg(PAGE0, 0x16, u8_2g_bb_power_b[ch]);
				//DLOG_Warning("VT 2g open power -> ch %d, 0x09=%2x,0x16=%2x",ch,u8_2g_bb_power_a[ch],u8_2g_bb_power_b[ch]);
			}
			else
			{
				BB_WriteReg(PAGE0, 0x09, u8_5g_bb_power_a[ch]);
				BB_WriteReg(PAGE0, 0x16, u8_5g_bb_power_b[ch]);


			}
		}
		else
		{
			if((band == RF_2G))
			{
				BB_WriteReg(PAGE0, 0x09, u8_2g_bb_power_a[ch]);
				BB_WriteReg(PAGE0, 0x16, u8_2g_bb_power_b[ch]);

			}
			else
			{
				BB_WriteReg(PAGE0, 0x09, u8_5g_bb_power_a[ch]);
				BB_WriteReg(PAGE0, 0x16, u8_5g_bb_power_b[ch]);
				//DLOG_Warning("VT 5g open power -> ch %d, 0x09=%2x,0x16=%2x",ch,u8_5g_bb_power_a[ch],u8_5g_bb_power_b[ch]);

			}

		}
	}
}


uint8_t BB_write_RcRegs(uint8_t *u8_rc)
{
    context.stru_rcRegs.frq1 = u8_rc[0];
    context.stru_rcRegs.frq2 = u8_rc[1];
    context.stru_rcRegs.frq3 = u8_rc[2];
    context.stru_rcRegs.frq4 = u8_rc[3];
    context.stru_rcRegs.frq5 = u8_rc[4];

    BB_WriteReg(PAGE2, AGC3_a, context.stru_rcRegs.frq1);
    BB_WriteReg(PAGE2, AGC3_b, context.stru_rcRegs.frq2);
    BB_WriteReg(PAGE2, AGC3_c, context.stru_rcRegs.frq3);
    BB_WriteReg(PAGE2, AGC3_d, context.stru_rcRegs.frq4);
}
uint8_t BB_set_skySweepVtfrq(ENUM_RF_BAND band, uint8_t ch)
{
    STRU_FRQ_CHANNEL *it_ch_ptr = ((band == RF_2G)?pstru_itFreq_2g : pstru_itFreq_5g);

    //set sweep frequency
    if (band == RF_2G )
    {
        BB_WriteRegMask(PAGE2, 0x20, 0x00, 0x04);
    }
    else
    {
        // P2 0x20 [2]=1,sweep frequency,5G
        BB_WriteRegMask(PAGE2, 0x20, 0x04, 0x04);   
    }

    BB_WriteReg(PAGE2, 0x14, it_ch_ptr[ch].frq1);
    BB_WriteReg(PAGE2, 0x15, it_ch_ptr[ch].frq2);
    BB_WriteReg(PAGE2, 0x16, it_ch_ptr[ch].frq3);
    BB_WriteReg(PAGE2, 0x17, it_ch_ptr[ch].frq4);

    return 0;
}

uint8_t BB_set_skySweepfrq(ENUM_RF_BAND band, uint8_t ch)
{
    STRU_FRQ_CHANNEL *pu8_rcRegs = ((band == RF_2G)?pstru_skySweeFreq_2g:pstru_skySweeFreq_5g);

    //set sweep frequency
    if (band == RF_2G )
    {
        BB_WriteRegMask(PAGE2, 0x20, 0x00, 0x04);
    }
    else
    {
        // P2 0x20 [2]=1,sweep frequency,5G
        BB_WriteRegMask(PAGE2, 0x20, 0x04, 0x04);   
    }

    BB_WriteReg(PAGE2, 0x14, pu8_rcRegs[ch].frq1);
    BB_WriteReg(PAGE2, 0x15, pu8_rcRegs[ch].frq2);
    BB_WriteReg(PAGE2, 0x16, pu8_rcRegs[ch].frq3);
    BB_WriteReg(PAGE2, 0x17, pu8_rcRegs[ch].frq4);

    return 0;
}


uint8_t BB_set_Rcfrq(ENUM_RF_BAND band, uint8_t ch)
{

    STRU_FRQ_CHANNEL *pu8_rcRegs = ((band == RF_2G)?pstru_rcFreq_2g:pstru_rcFreq_5g);

    context.stru_rcRegs.frq1 = pu8_rcRegs[ch].frq1;
    context.stru_rcRegs.frq2 = pu8_rcRegs[ch].frq2;
    context.stru_rcRegs.frq3 = pu8_rcRegs[ch].frq3;
    context.stru_rcRegs.frq4 = pu8_rcRegs[ch].frq4;

    BB_WriteReg(PAGE2, AGC3_a, pu8_rcRegs[ch].frq1);
    BB_WriteReg(PAGE2, AGC3_b, pu8_rcRegs[ch].frq2);
    BB_WriteReg(PAGE2, AGC3_c, pu8_rcRegs[ch].frq3);
    BB_WriteReg(PAGE2, AGC3_d, pu8_rcRegs[ch].frq4); 
	if(!pst_bbPowerOpenRefValue->u8_powerOpenMakeupEnable)
	{
		return 0;
	}

	if (context.en_bbmode == BB_GRD_MODE)
	{		
		if((band == RF_2G))
		{
			BB_WriteReg(PAGE0, 0x09, u8_2g_bb_power_a[ch]);
		}
		else
		{
			BB_WriteReg(PAGE0, 0x09, u8_5g_bb_power_a[ch]);
		}
	}
}



uint8_t BB_set_SweepFrq(ENUM_RF_BAND band, ENUM_CH_BW e_bw, uint8_t ch)
{
    STRU_FRQ_CHANNEL *ch_ptr;

    if (BW_10M == e_bw)
    {
        ch_ptr = ((band == RF_2G)?pstru_itFreq_2g:pstru_itFreq_5g);
    }
    else
    {
        ch_ptr = ((band == RF_2G)?pstru_itFreq_2g:pstru_itFreq_5g);
    }

    //set sweep frequency
    if ( band == RF_2G )
    {
        BB_WriteRegMask(PAGE2, 0x20, 0x00, 0x04);
    }
    else
    {
        // P2 0x20 [2]=1,sweep frequency,5G
        BB_WriteRegMask(PAGE2, 0x20, 0x04, 0x04);   
    }

    BB_WriteReg(PAGE2, SWEEP_FREQ_0, ch_ptr[ch].frq1);
    BB_WriteReg(PAGE2, SWEEP_FREQ_1, ch_ptr[ch].frq2);
    BB_WriteReg(PAGE2, SWEEP_FREQ_2, ch_ptr[ch].frq3);
    BB_WriteReg(PAGE2, SWEEP_FREQ_3, ch_ptr[ch].frq4);
}


uint8_t BB_GetRcFrqNum(ENUM_RF_BAND e_rfBand)
{
    if (e_rfBand == RF_2G)
    {
        return rcFreqCnt_2g;
    }
    else if (e_rfBand == RF_5G)
    {
        return u8_rcFreqCnt_5g;
    }
    return 0;
}

uint8_t BB_GetItFrqNum(ENUM_RF_BAND e_rfBand)
{
    if (e_rfBand == RF_2G)
    {
        return itFreqCnt_2g;
    }
    else if (e_rfBand == RF_5G)
    {
        return u8_itFreqCnt_5g;
    }
    return 0;
}

uint8_t BB_GetSkySweepFrqNum(ENUM_RF_BAND e_rfBand)
{
    if (e_rfBand == RF_2G)
    {
        return u8_skySweeFreqCnt_2g;
    }
    else if (e_rfBand == RF_5G)
    {
        return u8_skySweeFreqCnt_5g;
    }
    return 0;
}

uint8_t BB_SetAgcGain(ENUM_RF_BAND e_rfBand, uint8_t gain)
{
    if(AAGC_GAIN_FAR == gain)
    {
        if(RF_2G == e_rfBand)
        {
            BB_WriteRegMask(PAGE0, 0x91, 0, 0x01);
            BB_WriteReg(PAGE0, AGC_2, AAGC_GAIN_FAR_8003S);
            BB_WriteReg(PAGE0, AGC_3, AAGC_GAIN_FAR_8003S);
        }
        else if(RF_5G == e_rfBand)
        {
            BB_WriteRegMask(PAGE0, 0x91, 1, 0x01);
            BB_WriteReg(PAGE0, AGC_5G_GAIN1, AAGC_GAIN_FAR_8003S); //add 5G agc setting
            BB_WriteReg(PAGE0, AGC_5G_GAIN2, AAGC_GAIN_FAR_8003S);
        }
    }
    else if(AAGC_GAIN_NEAR == gain)
    {
        if(RF_2G == e_rfBand)
        {
            BB_WriteRegMask(PAGE0, 0x91, 0, 0x01);
            BB_WriteReg(PAGE0, AGC_2, AAGC_GAIN_NEAR_8003S);
            BB_WriteReg(PAGE0, AGC_3, AAGC_GAIN_NEAR_8003S);
        }
        else if(RF_5G == e_rfBand)
        {
            BB_WriteRegMask(PAGE0, 0x91, 1, 0x01);
            BB_WriteReg(PAGE0, AGC_5G_GAIN1, AAGC_GAIN_NEAR_8003S); //add 5G agc setting
            BB_WriteReg(PAGE0, AGC_5G_GAIN2, AAGC_GAIN_NEAR_8003S);
        }
    }
    else
    {
        ;
    }

    return 0;
}


int32_t BB_SweepEnergyCompensation(int32_t data)
{
    return ( data += ( (data > 30) ? (-120) : (-122) ) );
}


int32_t BB_SweepEnergy()
{
    int32_t  power_db;
    uint32_t power_td = (((uint32_t)(BB_ReadReg(PAGE2, SWEEP_ENERGY_HIGH)) << 16) |
                                     (BB_ReadReg(PAGE2, SWEEP_ENERGY_MID) << 8)  |
                                     (BB_ReadReg(PAGE2, SWEEP_ENERGY_LOW)));
    
    if (power_td == 0)
    {
        return 0;
    }

    power_db = BB_SweepEnergyCompensation(get_10log10(power_td));
    //DLOG_Info("%x -> %d",power_td,power_db);

    return power_db;
}

int RF_PcWriteReg(uint8_t ch, uint16_t u16_addr, uint8_t u8_data)
{
    //DLOG_Warning("%d %x %x", ch, u16_addr, u8_data);
    if (0 == ch)
    {
        BB_SPI_curPageWriteByte(0x01,0x01);             //bypass: SPI change into 1st 
    }
    else
    {
        BB_SPI_curPageWriteByte(0x01,0x03);             //bypass: SPI change into 2rd 
    }
    
    RF_SPI_WriteReg(u16_addr, u8_data);
    
    {
        //add patch, reset 8003s
        uint16_t delay = 0;
        RF8003s_SPI_WriteReg_internal(0x15, 0x51);
        while(delay ++ < 1000);
        RF8003s_SPI_WriteReg_internal(0x15, 0x50);
    } 
    
    BB_SPI_curPageWriteByte(0x01,0x02);             //SPI change into 8020    
}

int RF_PcReadReg(uint8_t ch, uint16_t u16_addr, uint8_t *pu8_rxValue)
{
    //DLOG_Warning("%d %x", ch, u16_addr);
    if (0 == ch)
    {
        BB_SPI_curPageWriteByte(0x01,0x01);             //bypass: SPI change into 1st 
    }
    else
    {
        BB_SPI_curPageWriteByte(0x01,0x03);             //bypass: SPI change into 2rd 
    }
    
    RF_SPI_ReadReg(u16_addr, pu8_rxValue);
    
    {
        //add patch, reset 8003s
        uint16_t delay = 0;
        RF8003s_SPI_WriteReg_internal(0x15, 0x51);
        while(delay ++ < 1000);
        RF8003s_SPI_WriteReg_internal(0x15, 0x50);
    } 
    
    BB_SPI_curPageWriteByte(0x01,0x02);             //SPI change into 8020    
}

int AR_round(float x)
{
    if (x > 0x0)
        return ((int)(x + 0.5));
    else
        return ((int)(x - 0.5));
}

uint8_t BB_calu_power_reg(uint16_t frq,uint16_t refPower,uint8_t refValueBase,uint8_t refValueCnt, STRU_OPEN_POWER_MEAS_VALUE *pst_PowerOpenRefValue)
{
	uint8_t i;
	uint8_t bFind;
	float pwr,delta_pwr,delta_freq;
	int makeup_value,obj_value;

	if(refValueCnt <= 1)
	{
		return refValueBase;
	}

	bFind = FALSE;
	for(i=0;i<refValueCnt-1;i++)
	{
		if((frq >= pst_PowerOpenRefValue[i].freq) &&
			(frq <= pst_PowerOpenRefValue[i+1].freq))
		{
			bFind = TRUE;
			break;
		}
	}

	if(!bFind)
	{
		DLOG_Warning("%d Not Find Open Power Ref Freq Value",frq);
		if(frq > pst_PowerOpenRefValue[refValueCnt-1].freq)
		{
			pwr = (float)(pst_PowerOpenRefValue[refValueCnt-1].real_power);
		}
		else
		{
			pwr = (float)(pst_PowerOpenRefValue[0].real_power);
		}
		goto skip_interpolation;
	}

    delta_freq = (float)(pst_PowerOpenRefValue[i+1].freq - pst_PowerOpenRefValue[i].freq);

    #define EPSINON  (1e-6)
    if(delta_freq >= -EPSINON && delta_freq <= EPSINON)
    {
        DLOG_Warning("delta_freq == 0");
        return refValueBase;
    }
    
    delta_pwr = (float)(pst_PowerOpenRefValue[i+1].real_power - pst_PowerOpenRefValue[i].real_power);

	pwr = pst_PowerOpenRefValue[i].real_power + delta_pwr / delta_freq * (frq - pst_PowerOpenRefValue[i].freq);

skip_interpolation:

	makeup_value = AR_round(((pwr - (float)refPower*10)/ 10) * 4);
	obj_value = (int)refValueBase - makeup_value;

	#define BB_MIN_PWR (0x01)
	#define BB_MAX_PWR (0x3f)
	if(obj_value <= 0)
	{
		return BB_MIN_PWR;
	}
	else if(obj_value > BB_MAX_PWR)
	{
		return BB_MAX_PWR;
	}
	else
	{
		return (uint8_t)obj_value;
	}
	
}
void BB_freq_power_table_byGenRcFreq(ENUM_RF_BAND band,STRU_BB_CH_OPEN_POWER_REF_VALUE *pst_PowerOpenRefValue)
{
	uint8_t i;

	if(band == RF_2G)
	{
		for(i=0;i<rcFreqCnt_2g;i++)
		{
			u8_2g_bb_power_a[i] = BB_calu_power_reg(Gen2GrcFreq[i],
									pst_PowerOpenRefValue->u8_2p4g_ref_power,
									pst_PowerOpenRefValue->u8_bbPwr[0],
									pst_PowerOpenRefValue->u8_2p4g_ref_point_num_a,
									pst_PowerOpenRefValue->st_2p4g_real_power_value_a);
			u8_2g_bb_power_b[i] = BB_calu_power_reg(Gen2GrcFreq[i],
									pst_PowerOpenRefValue->u8_2p4g_ref_power,
									pst_PowerOpenRefValue->u8_bbPwr[1],
									pst_PowerOpenRefValue->u8_2p4g_ref_point_num_b,
									pst_PowerOpenRefValue->st_2p4g_real_power_value_b);

		}
	}
	else if(band == RF_5G)
	{
		for(i=0;i<u8_rcFreqCnt_5g;i++)
		{
			u8_5g_bb_power_a[i] = BB_calu_power_reg(Gen5GrcFreq[i],
									pst_PowerOpenRefValue->u8_5p8g_ref_power,
									pst_PowerOpenRefValue->u8_bbPwr[0],
									pst_PowerOpenRefValue->u8_5p8g_ref_point_num_a,
									pst_PowerOpenRefValue->st_5p8g_real_power_value_a);
			u8_5g_bb_power_b[i] = BB_calu_power_reg(Gen5GrcFreq[i],
									pst_PowerOpenRefValue->u8_5p8g_ref_power,
									pst_PowerOpenRefValue->u8_bbPwr[1],
									pst_PowerOpenRefValue->u8_5p8g_ref_point_num_b,
									pst_PowerOpenRefValue->st_5p8g_real_power_value_b);

		}

	}
}

void BB_freq_power_table(ENUM_RF_BAND band,STRU_RF_CHANNEL *p_frq,STRU_BB_CH_OPEN_POWER_REF_VALUE *pst_PowerOpenRefValue)
{
	uint8_t i;

	if(band == RF_2G)
	{
		for(i=0;i<p_frq->u32_rfChCount;i++)
		{
			u8_2g_bb_power_a[i] = BB_calu_power_reg(p_frq->u16_rfChFrqList[i],
									pst_PowerOpenRefValue->u8_2p4g_ref_power,
									pst_PowerOpenRefValue->u8_bbPwr[0],
									pst_PowerOpenRefValue->u8_2p4g_ref_point_num_a,
									pst_PowerOpenRefValue->st_2p4g_real_power_value_a);
			u8_2g_bb_power_b[i] = BB_calu_power_reg(p_frq->u16_rfChFrqList[i],
									pst_PowerOpenRefValue->u8_2p4g_ref_power,
									pst_PowerOpenRefValue->u8_bbPwr[1],
									pst_PowerOpenRefValue->u8_2p4g_ref_point_num_b,
									pst_PowerOpenRefValue->st_2p4g_real_power_value_b);

		}
	}
	else if(band == RF_5G)
	{
		for(i=0;i<p_frq->u32_rfChCount;i++)
		{
			u8_5g_bb_power_a[i] = BB_calu_power_reg(p_frq->u16_rfChFrqList[i],
									pst_PowerOpenRefValue->u8_5p8g_ref_power,
									pst_PowerOpenRefValue->u8_bbPwr[0],
									pst_PowerOpenRefValue->u8_5p8g_ref_point_num_a,
									pst_PowerOpenRefValue->st_5p8g_real_power_value_a);
			u8_5g_bb_power_b[i] = BB_calu_power_reg(p_frq->u16_rfChFrqList[i],
									pst_PowerOpenRefValue->u8_5p8g_ref_power,
									pst_PowerOpenRefValue->u8_bbPwr[1],
									pst_PowerOpenRefValue->u8_5p8g_ref_point_num_b,
									pst_PowerOpenRefValue->st_5p8g_real_power_value_b);

		}

	}
}

static void BB_gen_open_power_makeup_value(ENUM_BB_MODE en_mode, ENUM_CH_BW bw)
{
    STRU_cfgNode *node;
    STRU_RF_CHANNEL *p_2g_frq,*p_5g_frq;

    if(en_mode == BB_SKY_MODE)
    {
        if (bw == BW_10M)
        {
            p_2g_frq = FCT_GetNodeAndData(FACTORY_SUBNODE_BAND0_VT_10M_FRQ_ID, &node);
            p_5g_frq = FCT_GetNodeAndData(FACTORY_SUBNODE_BAND1_VT_10M_FRQ_ID, &node);
        }
        else
        {
            p_2g_frq = FCT_GetNodeAndData(FACTORY_SUBNODE_BAND0_VT_20M_FRQ_ID, &node);
            p_5g_frq = FCT_GetNodeAndData(FACTORY_SUBNODE_BAND1_VT_20M_FRQ_ID, &node);
        }
        
        pst_bbPowerOpenRefValue = FCT_GetNodeAndData(FACTORY_SUBNODE_POWEROPEN_VT_SET_ID, NULL);
    }
    else if(en_mode == BB_GRD_MODE)
    {
        p_2g_frq = FCT_GetNodeAndData(FACTORY_SUBNODE_BAND0_RC_10M_FRQ_ID, &node);
        p_5g_frq = FCT_GetNodeAndData(FACTORY_SUBNODE_BAND1_RC_10M_FRQ_ID, &node);
        pst_bbPowerOpenRefValue = FCT_GetNodeAndData(FACTORY_SUBNODE_POWEROPEN_RC_SET_ID, NULL);
        if (NULL == p_2g_frq || NULL == p_5g_frq || NULL == pst_bbPowerOpenRefValue)
        {
            DLOG_Error("Fail Get Node");
            return;
        }

        if(p_2g_frq->u32_rfChCount > 0 && p_2g_frq->u16_rfChFrqList[0] < 100)
        {
            BB_freq_power_table_byGenRcFreq(RF_2G,pst_bbPowerOpenRefValue);
        }
        else
        {
            BB_freq_power_table(RF_2G,p_2g_frq,pst_bbPowerOpenRefValue);
        }
        if(p_5g_frq->u32_rfChCount > 0 && p_5g_frq->u16_rfChFrqList[0] < 100)
        {
            BB_freq_power_table_byGenRcFreq(RF_5G,pst_bbPowerOpenRefValue);
        }
        else
        {
            BB_freq_power_table(RF_5G,p_5g_frq,pst_bbPowerOpenRefValue);
        }
        return;
    }
    else
    {
        DLOG_Error("Unknow device type");
        return;
	}

    if (NULL == p_2g_frq || NULL == p_5g_frq || NULL == pst_bbPowerOpenRefValue)
    {
        DLOG_Error("Fail Get Node");
        return;
    }

	BB_freq_power_table(RF_2G,p_2g_frq,pst_bbPowerOpenRefValue);
	BB_freq_power_table(RF_5G,p_5g_frq,pst_bbPowerOpenRefValue);
}

void BB_set_power_close(ENUM_RF_BAND band,uint8_t power)
{
	if(pst_close_power == NULL)
	{
		DLOG_Error("pst_close_power == NULL");
		return;
	}
	
	if (band == RF_2G)
	{
		BB_WriteReg(PAGE0, 0xbe, pst_close_power->st_2g_chPowerClose[power].u8_centralGain);
		BB_WriteReg(PAGE0, 0xbf, (pst_close_power->st_2g_chPowerClose[power].u16_regPwr >> 8));
		BB_WriteRegMask(PAGE0, 0xcb, (pst_close_power->st_2g_chPowerClose[power].u16_regPwr & 0xf0), 0xf0);
	}
	else
	{
		BB_WriteReg(PAGE0, 0xce, pst_close_power->st_5g_chPowerClose[power].u8_centralGain);
		BB_WriteReg(PAGE0, 0xcf, (pst_close_power->st_5g_chPowerClose[power].u16_regPwr >> 8));
		BB_WriteRegMask(PAGE0, 0xcb, (pst_close_power->st_5g_chPowerClose[power].u16_regPwr & 0x0f), 0x0f);
	}

}

void  BB_set_power_open(ENUM_RF_BAND band,uint8_t power)
{
	int8_t pwr_a,pwr_b;
	if(pst_bbPowerOpenRefValue == NULL)
	{
		DLOG_Error("pst_bbPowerOpenRefValue NULL");
		return;
	}
	#define RF_2G_MIN_PWR (0x1f)
	#define RF_2G_MAX_PWR (0x02)
	#define RF_5G_MIN_PWR (0x1f)
	#define RF_5G_MAX_PWR (0x02)

	if (band == RF_2G)
	{
		pwr_a = (int8_t)(pst_bbPowerOpenRefValue->u8_2p4g_rfPwr[0]) - ((int8_t)power - (int8_t)(pst_bbPowerOpenRefValue->u8_2p4g_ref_power));
		pwr_b = (int8_t)(pst_bbPowerOpenRefValue->u8_2p4g_rfPwr[1]) - ((int8_t)power - (int8_t)(pst_bbPowerOpenRefValue->u8_2p4g_ref_power));
		if(pwr_a <= 0)
		{
			pwr_a = RF_2G_MAX_PWR;
		}
		else if(pwr_a >= RF_2G_MIN_PWR)
		{
			pwr_a = RF_2G_MIN_PWR;
		}
		
		if(pwr_b <= 0)
		{
			pwr_b = RF_2G_MAX_PWR;
		}
		else if(pwr_b >= RF_2G_MIN_PWR)
		{
			pwr_b = RF_2G_MIN_PWR;
		}

		RF8003s_SPI_WriteReg_internal(0x2a, pwr_a);
        RF8003s_SPI_WriteReg_internal(0x2e, pwr_b);
	}
	else
	{
		pwr_a = (int8_t)(pst_bbPowerOpenRefValue->u8_5p8g_rfPwr[0]) - ((int8_t)power - (int8_t)(pst_bbPowerOpenRefValue->u8_5p8g_ref_power));
		pwr_b = (int8_t)(pst_bbPowerOpenRefValue->u8_5p8g_rfPwr[1]) - ((int8_t)power - (int8_t)(pst_bbPowerOpenRefValue->u8_5p8g_ref_power));
		if(pwr_a <= 0)
		{
			pwr_a = RF_5G_MAX_PWR;
		}
		else if(pwr_a >= RF_5G_MIN_PWR)
		{
			pwr_a = RF_5G_MIN_PWR;
		}
		
		if(pwr_b <= 0)
		{
			pwr_b = RF_5G_MAX_PWR;
		}
		else if(pwr_b >= RF_5G_MIN_PWR)
		{
			pwr_b = RF_5G_MIN_PWR;
		}

        RF8003s_SPI_WriteReg_internal(0x66, pwr_a);
        RF8003s_SPI_WriteReg_internal(0x67, pwr_b);
	}

}

void  BB_set_bbpower_open(ENUM_RF_BAND band,uint8_t power)
{
	int8_t pwr_a,pwr_b;
	if(pst_bbPowerOpenRefValue == NULL)
	{
		DLOG_Error("pst_bbPowerOpenRefValue NULL");
		return;
	}
	#define RF_2G_MIN_PWR (0x1f)
	#define RF_2G_MAX_PWR (0x02)
	#define RF_5G_MIN_PWR (0x1f)
	#define RF_5G_MAX_PWR (0x02)

	if (band == RF_2G)
	{
		pwr_a = (int8_t)(pst_bbPowerOpenRefValue->u8_2p4g_rfPwr[0]) - ((int8_t)power - (int8_t)(pst_bbPowerOpenRefValue->u8_2p4g_ref_power));
		pwr_b = (int8_t)(pst_bbPowerOpenRefValue->u8_2p4g_rfPwr[1]) - ((int8_t)power - (int8_t)(pst_bbPowerOpenRefValue->u8_2p4g_ref_power));
		if(pwr_a <= 0)
		{
			pwr_a = RF_2G_MAX_PWR;
		}
		else if(pwr_a >= RF_2G_MIN_PWR)
		{
			pwr_a = RF_2G_MIN_PWR;
		}
		
		if(pwr_b <= 0)
		{
			pwr_b = RF_2G_MAX_PWR;
		}
		else if(pwr_b >= RF_2G_MIN_PWR)
		{
			pwr_b = RF_2G_MIN_PWR;
		}

		RF8003s_SPI_WriteReg_internal(0x2a, pwr_a);
        RF8003s_SPI_WriteReg_internal(0x2e, pwr_b);
	}
	else
	{
		pwr_a = (int8_t)(pst_bbPowerOpenRefValue->u8_5p8g_rfPwr[0]) - ((int8_t)power - (int8_t)(pst_bbPowerOpenRefValue->u8_5p8g_ref_power));
		pwr_b = (int8_t)(pst_bbPowerOpenRefValue->u8_5p8g_rfPwr[1]) - ((int8_t)power - (int8_t)(pst_bbPowerOpenRefValue->u8_5p8g_ref_power));
		if(pwr_a <= 0)
		{
			pwr_a = RF_5G_MAX_PWR;
		}
		else if(pwr_a >= RF_5G_MIN_PWR)
		{
			pwr_a = RF_5G_MIN_PWR;
		}
		
		if(pwr_b <= 0)
		{
			pwr_b = RF_5G_MAX_PWR;
		}
		else if(pwr_b >= RF_5G_MIN_PWR)
		{
			pwr_b = RF_5G_MIN_PWR;
		}

        RF8003s_SPI_WriteReg_internal(0x66, pwr_a);
        RF8003s_SPI_WriteReg_internal(0x67, pwr_b);
	}

}

void BB_set_RF_mimo_mode(ENUM_MIMO_MODE e_mimo_mode)
{
	BB_SPI_curPageWriteByte(0x01,0x01);
	if(e_mimo_mode == MIMO_1T1R)
	{
		RF_SPI_WriteReg(0x00,0x50);
		RF_SPI_WriteReg(0x40,0x50);
	}
	else if(e_mimo_mode == MIMO_1T2R)
	{
		RF_SPI_WriteReg(0x00,0x74);
		RF_SPI_WriteReg(0x40,0x74);
	}
	else
	{
		RF_SPI_WriteReg(0x00,0x7c);
		RF_SPI_WriteReg(0x40,0x7c);
	}
	BB_SPI_curPageWriteByte(0x01,0x02);
}

uint8_t BB_GetRcFrqNumPerFilter(void)
{
    if(RF_2G == context.e_curBand)
    {
        return rcFreqCnt_2g;
    }
    else
    {
        return u8_rcFreqCnt_5g;
    }
}

uint8_t BB_GetItFrqNumPerFilter(void)
{
}

uint16_t BB_GetRcFrqByCh(uint8_t ch)
{
  //DLOG_Warning("2g_5g rc size %d",rcFreqCnt_2g);

  if(RF_2G == context.e_curBand)
  {
	 if(pRcFreqlist_2g ==NULL)return 0;
	 if(ch > rcFreqCnt_2g )return 0;
	return pRcFreqlist_2g[ch];
  }
 else if(RF_5G == context.e_curBand)
  {
  	 if(ch > u8_rcFreqCnt_5g )return 0;
	 if(pRcFreqlist_5g ==NULL)return 0;
	return pRcFreqlist_5g[ch];
  }

}

uint16_t BB_GetItFrqByCh(uint8_t ch)
{
   if(RF_2G == context.e_curBand)
  {
	if(pItFreqlist_2g ==NULL)return 0;
	if(ch > itFreqCnt_2g )return 0;
    return pItFreqlist_2g[ch];
  }
  else if(RF_5G == context.e_curBand)
  {
	if(pItFreqlist_5g ==NULL)return 0;
	if(ch > u8_itFreqCnt_5g )return 0;
   	return pItFreqlist_5g[ch];
   }
}

uint8_t BB_GetItChInOneFilter(uint8_t ch)
{
    return ch;
}

int RF_RegValue2Frq(uint8_t value[], uint16_t *frq)
{
    return 0;
}

uint8_t BB_GetItStarEndByItCh(uint8_t *start, uint8_t *end, uint8_t it_ch)
{
    return 0;
}

uint8_t BB_GetRcStarEndByItCh(uint8_t *start, uint8_t *end, uint8_t it_ch)
{
    return 0;
}

void BB_InitSunBand_RcFreqNum(uint8_t bw_10M_Rc_freq_num, uint8_t bw_20M_Rc_freq_num)
{
    if(u8_subBand10MrcFreqNum > 0 || u8_subBand20MrcFreqNum > 0)
    {
        DLOG_Warning("setting invalid");
        return;
    }

    u8_subBand10MrcFreqNum = bw_10M_Rc_freq_num;
    u8_subBand20MrcFreqNum = bw_20M_Rc_freq_num;
}
uint8_t BB_GetSubBandStartCH(ENUM_RF_BAND rf_band, ENUM_CH_BW bw, uint8_t it_ch)
{
    if(rf_band == RF_2G)
    {
        if (bw == BW_10M)
        {
            return it_ch*u8_subBand10MrcFreqNum;
        }
        else
        {
            return it_ch*u8_subBand20MrcFreqNum;
        }
    }
    else if(rf_band == RF_5G)
    {
        if (bw == BW_10M)
        {
            return it_ch*u8_subBand10MrcFreqNum;
        }
        else
        {
            return it_ch*u8_subBand20MrcFreqNum;
        }

    }
    else
    {
        //DLOG_Error("Unknow device type");
        return 0;
	}
}

uint8_t BB_GetSubBandEndCH(ENUM_RF_BAND rf_band, ENUM_CH_BW bw, uint8_t it_ch)
{
    uint8_t value;
    
    value = BB_GetSubBandStartCH(rf_band,bw,it_ch);
    if(rf_band == RF_2G)
    {
        if (bw == BW_10M)
        {
            return value+u8_subBand10MrcFreqNum;
        }
        else
        {
            return value+u8_subBand20MrcFreqNum;
        }
    }
    else if(rf_band == RF_5G)
    {
        if (bw == BW_10M)
        {
            return value+u8_subBand10MrcFreqNum;
        }
        else
        {
            return value+u8_subBand20MrcFreqNum;
        }

    }
    else
    {
        DLOG_Error("Unknow device type");
        return 0;
	}
}

