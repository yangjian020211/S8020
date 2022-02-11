/*****************************************************************************
Copyright: 2016-2020, Artosyn. Co., Ltd.
File name: RF_8003.c
Description: The internal APIs to control the RF 8003
Author: Artosy Software Team
Version: 0.0.1
Date: 2016/12/20
History: 
        0.0.1    2016/12/20    The initial version of rf8003.c
*****************************************************************************/
#include "debuglog.h"
#include "bb_spi.h"
#include "rf_if.h"
#include "systicks.h"
#include "bb_ctrl_internal.h"
#include "cfg_parser.h"
#include "factory.h"
#include "log10.h"

#define     AAGC_GAIN_FAR_8003                   (0x12)
#define     AAGC_GAIN_NEAR_8003                  (0x3F)

extern uint8_t RF_8003_regs0[64];
extern uint8_t RF8003_regs1[64];


static uint8_t  u8_caliRegArray[10];

static uint8_t *pu8_rf0_regs;
static uint8_t *pu8_rf1_regs;

static STRU_FRQ_CHANNEL pstru_rcFreq_2g[240];//40*6
static STRU_FRQ_CHANNEL pstru_itFreq_2g[40];
static uint16_t *pRcFreqlist = NULL;
static uint16_t *pItFreqlist = NULL;

static int16_t Gen2GrcFreq[240];

static uint8_t u8_2g_bb_power_a[240];
static uint8_t u8_2g_bb_power_b[240];

static uint8_t u8_rcFreqCnt_2g;
static uint8_t u8_itFreqCnt_2g;

static STRU_RF_REG *pstru_rf0_regBeforeCali;
static STRU_RF_REG *pstru_rf1_regBeforeCali;

static STRU_RF_REG *pstru_rf0_regAfterCali;
static STRU_RF_REG *pstru_rf1_regAfterCali;

static STRU_BOARD_RF_PARA *pstru_rf_boardcfg;
static STRU_BB_POWER_CLOSE *pst_close_power = NULL; 
static STRU_BB_CH_OPEN_POWER_REF_VALUE *pst_bbPowerOpenRefValue = NULL;

static uint8_t u8_skySweeFreqCnt_2g;
static uint8_t u8_subBand10MrcFreqNum;
static uint8_t u8_subBand20MrcFreqNum;

static void BB_gen_open_power_makeup_value(ENUM_BB_MODE en_mode, ENUM_CH_BW bw);



void RF8003_getCfgData(ENUM_BB_MODE en_mode, STRU_cfgBin *cfg)
{
    STRU_cfgNode  *rfcfg_node;

    pu8_rf0_regs = RF_8003_regs0;
    pu8_rf1_regs = RF8003_regs1;

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

    STRU_RF_REG * bb_reg = CFGBIN_GetNodeData(cfg, RF_BOARDCFG_DATA_ID);
    if (NULL != bb_reg)
    {
        if (en_mode == BB_SKY_MODE)
        {
            pstru_rf0_regBeforeCali = bb_reg;
            pstru_rf1_regBeforeCali = bb_reg; 

            pstru_rf0_regAfterCali = pstru_rf0_regBeforeCali + pstru_rf_boardcfg->u8_rf0SkyRegsCnt + pstru_rf_boardcfg->u8_rf0GrdRegsCnt;
            pstru_rf1_regAfterCali = pstru_rf0_regAfterCali;
        }
        else
        {
            pstru_rf0_regBeforeCali =  bb_reg + pstru_rf_boardcfg->u8_rf0SkyRegsCnt;
            pstru_rf1_regBeforeCali =  pstru_rf0_regBeforeCali + pstru_rf_boardcfg->u8_rf0GrdRegsCnt;
        
            pstru_rf0_regAfterCali = pstru_rf1_regBeforeCali + pstru_rf_boardcfg->u8_rf0SkyRegsCntAfterCali;
            pstru_rf1_regAfterCali = pstru_rf0_regAfterCali  + pstru_rf_boardcfg->u8_rf0GrdRegsCntAfterCali;
        }
    }

}

void RF8003xCalcFrq2Register(uint16_t u16_frq, STRU_FRQ_CHANNEL *frq_regvalue)
{
    uint8_t  integer;
    uint32_t fraction;

    integer  = (uint32_t)u16_frq * 2/ 39;
    fraction = ((double) u16_frq * 2/ 39 - integer) * (1 << 24);

    frq_regvalue->frq1 = (fraction & 0xff);
    frq_regvalue->frq2 = ((fraction >>  8) & 0xff);
    frq_regvalue->frq3 = ((fraction >> 16) & 0xff);
    frq_regvalue->frq4 = integer;

    DLOG_Info("%d %x %x %x %x", u16_frq, frq_regvalue->frq1, frq_regvalue->frq2, frq_regvalue->frq3, frq_regvalue->frq4);
}

uint16_t RF8003xCalcRegister2Frq(uint32_t u32_frq_reg)
{
    uint32_t tmp;
    
    tmp = u32_frq_reg >> 8;
    tmp = ((tmp & 0xff ) << 16) | ((tmp >> 16) & 0xff) | (tmp & 0x00ff00);
    DLOG_Warning("%x",tmp);
    return (((double)(tmp )) / (1 << 24) +((double) ((u32_frq_reg) & 0x000000ff))) * 39 / 2 + 0.5;
}

void RF8003_CalcFctNode2RegTable(uint32_t nodeid, STRU_FRQ_CHANNEL *frq_regvalue, uint8_t * frq_cnt)
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

    *frq_cnt = p_frq->u32_rfChCount;
    for (i = 0; i < p_frq->u32_rfChCount; i++)
    {
        RF8003xCalcFrq2Register(p_frq->u16_rfChFrqList[i], (frq_regvalue + i));
    }
	
	

}



void RF8003_GetFctFreqTable(ENUM_CH_BW e_bw)
{
    uint8_t i = 0,j=0;
    uint32_t nodeid_band0,freq_cnt;
    STRU_cfgNode *node,*node_vt;
    STRU_RF_CHANNEL *p_frq,*p_frq_vt;
    STRU_cfgNode *itnode;
	STRU_RF_CHANNEL * itp_frq;
	STRU_cfgNode *rcnode;
	STRU_RF_CHANNEL * rcp_frq ;
    if (e_bw == BW_10M)
    {
        RF8003_CalcFctNode2RegTable(FACTORY_SUBNODE_BAND0_VT_10M_FRQ_ID, pstru_itFreq_2g, &u8_itFreqCnt_2g);
        nodeid_band0 = FACTORY_SUBNODE_BAND0_VT_10M_FRQ_ID;
		itp_frq = FCT_GetNodeAndData(nodeid_band0, &itnode);
		pItFreqlist = itp_frq->u16_rfChFrqList;
		
		DLOG_Critical("cnt = %d", itp_frq->u32_rfChCount);
		for(i=0;i<itp_frq->u32_rfChCount;i++)
			DLOG_Critical("it[%d]=%d",i,pItFreqlist[i]);
    }
    else
    {
        RF8003_CalcFctNode2RegTable(FACTORY_SUBNODE_BAND0_VT_20M_FRQ_ID, pstru_itFreq_2g, &u8_itFreqCnt_2g);
        nodeid_band0 = FACTORY_SUBNODE_BAND0_VT_20M_FRQ_ID;
		itp_frq = FCT_GetNodeAndData(nodeid_band0, &itnode);
		pItFreqlist = itp_frq->u16_rfChFrqList;
    }
    
    p_frq = (STRU_RF_CHANNEL *)FCT_GetNodeAndData(FACTORY_SUBNODE_BAND0_RC_10M_FRQ_ID, &node);
    if (NULL == p_frq)
    {
        DLOG_Error("Fail Node = %x", FACTORY_SUBNODE_BAND0_RC_10M_FRQ_ID);
        return;
    }
	
	#ifdef RFSUB_BAND
    if(p_frq->u32_rfChCount > 0 && p_frq->u16_rfChFrqList[0] < 100)//real freq value must > 100, e.g 2400,5800
    {
        DLOG_Warning("2g auto generate rc freq");
        p_frq_vt = (STRU_RF_CHANNEL *)FCT_GetNodeAndData(nodeid_band0, &node_vt);
        if (NULL == p_frq_vt)
        {
            DLOG_Error("Fail Node = %x", nodeid_band0);
            return;
        }
		
		pItFreqlist = p_frq_vt->u16_rfChFrqList;
		
        freq_cnt = p_frq->u32_rfChCount;
        if(p_frq->u32_rfChCount * p_frq_vt->u32_rfChCount > 240)
        {
            freq_cnt = 240 / p_frq_vt->u32_rfChCount;
            DLOG_Warning("2g force sub rc %d->%d",p_frq->u32_rfChCount,freq_cnt);
        }

        for(i=0;i<p_frq_vt->u32_rfChCount;i++)
        {
            for(j=0;j<freq_cnt;j++)
            {
                Gen2GrcFreq[i*freq_cnt+j] = p_frq_vt->u16_rfChFrqList  [i] + p_frq->u16_rfChFrqList[j];
                RF8003xCalcFrq2Register(p_frq_vt->u16_rfChFrqList  [i] + p_frq->u16_rfChFrqList[j], (pstru_rcFreq_2g + i*freq_cnt+j));
            }
        }
        u8_rcFreqCnt_2g = freq_cnt * p_frq_vt->u32_rfChCount;
        u8_subBand10MrcFreqNum = freq_cnt;
        DLOG_Warning("2g rc frq %d %d",u8_subBand10MrcFreqNum,u8_rcFreqCnt_2g);
    }
    else
	#endif
    {
        RF8003_CalcFctNode2RegTable(FACTORY_SUBNODE_BAND0_RC_10M_FRQ_ID, pstru_rcFreq_2g, &u8_rcFreqCnt_2g);
		rcp_frq = FCT_GetNodeAndData(FACTORY_SUBNODE_BAND0_RC_10M_FRQ_ID, &rcnode);
		pRcFreqlist = rcp_frq->u16_rfChFrqList;
    }

    DLOG_Warning("e_bw = %d,rc_cnt = %d", e_bw,u8_rcFreqCnt_2g);
}

void RF_GetFctFreqTable(ENUM_CH_BW e_bw){
	RF8003_GetFctFreqTable(e_bw);
}

#define  RF8003_RF_CLOCKRATE    (1)    //1MHz clockrate

static int RF8003_SPI_WriteReg_internal(uint8_t u8_addr, uint8_t u8_data)
{
    int ret = 0;
    uint8_t wdata[] = {0x80, (u8_addr <<1), u8_data};   //RF_8003S_SPI: wr: 0x80 ;

    //SPI_master_init(BB_SPI_BASE_IDX, &init);
    SPI_write_read(BB_SPI_BASE_IDX, wdata, sizeof(wdata), 0, 0); 
    ret =  SPI_WaitIdle(BB_SPI_BASE_IDX, BB_SPI_MAX_DELAY);

    return ret;
}


static int RF8003_SPI_ReadReg_internal(uint8_t u8_addr)
{
    uint8_t wdata[2] = {0x00, (u8_addr<<1)};      //RF_8003_SPI:  rd: 0x00
    uint8_t rdata;
    
    //use low speed for the RF8003 read, from test, read fail if use the same clockrate as baseband
    STRU_SPI_InitTypes init = {
        .ctrl0   = SPI_CTRL0_DEF_VALUE,
        .clk_Mhz = RF8003_RF_CLOCKRATE,
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
    return RF8003_SPI_WriteReg_internal((uint8_t)u16_addr, u8_data);
}

/**
  * @brief : Read 8003 RF register by SPI 
  * @param : addr: 8003 SPI address
  * @retval  0: sucess   1: FAIL
  */
int RF_SPI_ReadReg(uint16_t u16_addr, uint8_t *pu8_rxValue)
{
    uint8_t tmp = RF8003_SPI_ReadReg_internal( (uint8_t)u16_addr);
    if( pu8_rxValue)
    {
        *pu8_rxValue = tmp;
    }
    else
    {
        DLOG_Error("pu8_rxValue == NULL");
    }

    return 0;
}

/**
  * @brief : init RF8003 register
  * @param : addr: 8003 SPI address
  * @retval  None
  */
void RF_init(ENUM_BB_MODE en_mode)
{
    uint8_t idx;
    uint8_t cnt;
    uint8_t num;

    STRU_RF_REG * pstru_rfReg = NULL;

    RF8003_GetFctFreqTable(context.st_bandMcsOpt.e_bandwidth);

    BB_gen_open_power_makeup_value(en_mode, context.st_bandMcsOpt.e_bandwidth);

	if(en_mode == BB_SKY_MODE)
	{
		pst_close_power = FCT_GetNodeAndData(FACTORY_SUBNODE_POWERCLOSE_VT_SET_ID, NULL);
	}
	else
	{
		pst_close_power = FCT_GetNodeAndData(FACTORY_SUBNODE_POWERCLOSE_RC_SET_ID, NULL);
	}

    RF8003_getCfgData(en_mode, (STRU_cfgBin *)SRAM_CONFIGURE_MEMORY_ST_ADDR);

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

        for(idx = 0; idx < 64; idx++)
        {
            RF8003_SPI_WriteReg_internal(idx, pu8_rf0_regs[idx]);
        }

        {
            //add patch, reset 8003
            RF8003_SPI_WriteReg_internal(0x15, 0x51);
            RF8003_SPI_WriteReg_internal(0x15, 0x50);
        }
        
        BB_SPI_curPageWriteByte(0x01,0x02);             //SPI change into 8020
    }
}

int RF8003_SetPowerModeAfterCali(STRU_RF_POWER_CTRL * pst_power)
{
    return 0;
}

static void RF8003_afterCali(ENUM_BB_MODE en_mode, STRU_BOARD_RF_PARA *pstru_rf_boardcfg)
{
    STRU_RF_REG * pu8_rf1_regs, * rf2_regs;
    uint8_t cnt;
    uint8_t rf_regcnt1, rf_regcnt2;
    uint8_t value;

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
        BB_SPI_curPageWriteByte(0x01,0x01);             //bypass: SPI change into 1st 8003
        
        for(cnt = 0; cnt < rf_regcnt1; cnt++)
        {
            RF8003_SPI_WriteReg_internal( pu8_rf1_regs[cnt].addr_l, pu8_rf1_regs[cnt].value);
        }

        //pst_powercfg
        {
            STRU_cfgNode *cfgnode;
            STRU_RF_POWER_CTRL *pst_powercfg = NULL;
            pst_powercfg = (STRU_RF_POWER_CTRL *)FCT_GetNodeAndData(FACTORY_SUBNODE_POWER_NODE_ID, &cfgnode);
            RF8003_SetPowerModeAfterCali(pst_powercfg);

        }

        {
            //add patch, reset 8003
            uint16_t delay = 0;
            RF8003_SPI_WriteReg_internal(0x15, 0x51);
            while(delay ++ < 1000);
            RF8003_SPI_WriteReg_internal(0x15, 0x50);
        } 
        
        BB_SPI_curPageWriteByte(0x01,0x02);             //SPI change into 8020    
    }

    if (pstru_rf_boardcfg->u8_rfCnt > 1 && rf_regcnt2 > 0 && rf2_regs != NULL)
    {
        BB_SPI_curPageWriteByte(0x01,0x03);             //bypass: SPI change into 2rd 8003
        
        for(cnt = 0; cnt < rf_regcnt2; cnt++)
        {
            RF8003_SPI_WriteReg_internal( rf2_regs[cnt].addr_l, rf2_regs[cnt].value);
        }

        {
            //add patch, reset 8003
            uint16_t delay = 0;
            RF8003_SPI_WriteReg_internal(0x15, 0x51);
            while(delay ++ < 1000);            
            RF8003_SPI_WriteReg_internal(0x15, 0x50);
        } 
        
        BB_SPI_curPageWriteByte(0x01,0x02);             //SPI change into 8020    
    }

    RF_PcReadReg(0, 0x35, &value);
    value &= ~0x07; //[2:0]=000
    RF_PcWriteReg(0, 0x35, value);
    
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
    read_cali_register(u8_caliRegArray);
}



void BB_RF_band_switch(ENUM_RF_BAND rf_band)
{

}


void RF_CaliProcess(ENUM_BB_MODE en_mode)
{
    BB_before_RF_cali();

    BB_RF_start_cali();

    BB_WriteReg(PAGE0, TX_CALI_ENABLE, 0x00);   //disable calibration

    RF8003_afterCali(en_mode, pstru_rf_boardcfg);
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
    context.stru_itRegs.frq1 = pstru_itFreq_2g[ch].frq1;
    context.stru_itRegs.frq2 = pstru_itFreq_2g[ch].frq2;
    context.stru_itRegs.frq3 = pstru_itFreq_2g[ch].frq3;
    context.stru_itRegs.frq4 = pstru_itFreq_2g[ch].frq4;

    BB_WriteReg(PAGE2, AGC3_0, pstru_itFreq_2g[ch].frq1);
    BB_WriteReg(PAGE2, AGC3_1, pstru_itFreq_2g[ch].frq2);
    BB_WriteReg(PAGE2, AGC3_2, pstru_itFreq_2g[ch].frq3);
    BB_WriteReg(PAGE2, AGC3_3, pstru_itFreq_2g[ch].frq4);
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

    BB_WriteReg(PAGE2, 0x14, pstru_itFreq_2g[ch].frq1);
    BB_WriteReg(PAGE2, 0x15, pstru_itFreq_2g[ch].frq2);
    BB_WriteReg(PAGE2, 0x16, pstru_itFreq_2g[ch].frq3);
    BB_WriteReg(PAGE2, 0x17, pstru_itFreq_2g[ch].frq4);

    return 0;
}

uint8_t BB_set_skySweepfrq(ENUM_RF_BAND band, uint8_t ch)
{
    BB_WriteReg(PAGE2, 0x14, pstru_rcFreq_2g[ch].frq1);
    BB_WriteReg(PAGE2, 0x15, pstru_rcFreq_2g[ch].frq2);
    BB_WriteReg(PAGE2, 0x16, pstru_rcFreq_2g[ch].frq3);
    BB_WriteReg(PAGE2, 0x17, pstru_rcFreq_2g[ch].frq4); 
    
    return 0;
}


uint8_t BB_set_Rcfrq(ENUM_RF_BAND band, uint8_t ch)
{
    context.stru_rcRegs.frq1 = pstru_rcFreq_2g[ch].frq1;
    context.stru_rcRegs.frq2 = pstru_rcFreq_2g[ch].frq2;
    context.stru_rcRegs.frq3 = pstru_rcFreq_2g[ch].frq3;
    context.stru_rcRegs.frq4 = pstru_rcFreq_2g[ch].frq4;

    BB_WriteReg(PAGE2, AGC3_a, pstru_rcFreq_2g[ch].frq1);
    BB_WriteReg(PAGE2, AGC3_b, pstru_rcFreq_2g[ch].frq2);
    BB_WriteReg(PAGE2, AGC3_c, pstru_rcFreq_2g[ch].frq3);
    BB_WriteReg(PAGE2, AGC3_d, pstru_rcFreq_2g[ch].frq4); 
}



uint8_t BB_set_SweepFrq(ENUM_RF_BAND band, ENUM_CH_BW e_bw, uint8_t ch)
{
    STRU_FRQ_CHANNEL *ch_ptr = ((BW_10M == e_bw) ? pstru_itFreq_2g : pstru_itFreq_2g);

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
	
    return u8_rcFreqCnt_2g;
	DLOG_Warning("2g rc size %d",u8_rcFreqCnt_2g);
}


uint8_t BB_GetItFrqNum(ENUM_RF_BAND e_rfBand)
{
    return u8_itFreqCnt_2g;
}

uint8_t BB_GetSkySweepFrqNum(ENUM_RF_BAND e_rfBand)
{
    if (e_rfBand == RF_2G)
    {
       // return u8_skySweeFreqCnt_2g;
       return BB_GetRcFrqNum(e_rfBand);
    }
    
    return 0;
}


uint8_t BB_SetAgcGain(ENUM_RF_BAND e_rfBand, uint8_t gain)
{
    if(AAGC_GAIN_FAR == gain)
    {
        BB_WriteReg(PAGE0, AGC_2, AAGC_GAIN_FAR_8003);
        BB_WriteReg(PAGE0, AGC_3, AAGC_GAIN_FAR_8003);
    }
    else if(AAGC_GAIN_NEAR == gain)
    {
        BB_WriteReg(PAGE0, AGC_2, AAGC_GAIN_NEAR_8003);
        BB_WriteReg(PAGE0, AGC_3, AAGC_GAIN_NEAR_8003);
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
        RF8003_SPI_WriteReg_internal(0x15, 0x51);
        while(delay ++ < 1000);
        RF8003_SPI_WriteReg_internal(0x15, 0x50);
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
        RF8003_SPI_WriteReg_internal(0x15, 0x51);
        while(delay ++ < 1000);
        RF8003_SPI_WriteReg_internal(0x15, 0x50);
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
void dump_freq(uint8_t offset , uint8_t cnt, uint8_t val)
{
    uint8_t i;
    if(val==0)
    {
        DLOG_Warning("cnt = %d",u8_rcFreqCnt_2g);
        for(i=offset;i<cnt;i++)
            DLOG_Warning("%d",Gen2GrcFreq[i]);
    }
    else if(val == 2)
    {
        DLOG_Warning("cnt = %d",u8_rcFreqCnt_2g);
        for(i=offset;i<cnt;i++)
            DLOG_Warning("%x",u8_2g_bb_power_a[i]);

    }
}

void BB_freq_power_table_byGenRcFreq(ENUM_RF_BAND band,STRU_BB_CH_OPEN_POWER_REF_VALUE *pst_PowerOpenRefValue)
{
	uint8_t i;

	if(band == RF_2G)
	{
		for(i=0;i<u8_rcFreqCnt_2g;i++)
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
}


static void BB_gen_open_power_makeup_value(ENUM_BB_MODE en_mode, ENUM_CH_BW bw)
{
    STRU_cfgNode *node;
    STRU_RF_CHANNEL *p_2g_frq;

    if(en_mode == BB_SKY_MODE)
    {
        if (bw == BW_10M)
        {
            p_2g_frq = FCT_GetNodeAndData(FACTORY_SUBNODE_BAND0_VT_10M_FRQ_ID, &node);
        }
        else
        {
            p_2g_frq = FCT_GetNodeAndData(FACTORY_SUBNODE_BAND0_VT_20M_FRQ_ID, &node);
        }
        
        pst_bbPowerOpenRefValue = FCT_GetNodeAndData(FACTORY_SUBNODE_POWEROPEN_VT_SET_ID, NULL);
    }
    else if(en_mode == BB_GRD_MODE)
    {
        p_2g_frq = FCT_GetNodeAndData(FACTORY_SUBNODE_BAND0_RC_10M_FRQ_ID, &node);
        pst_bbPowerOpenRefValue = FCT_GetNodeAndData(FACTORY_SUBNODE_POWEROPEN_RC_SET_ID, NULL);
        if (NULL == p_2g_frq || NULL == pst_bbPowerOpenRefValue)
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
        return;

    }
    else
    {
        DLOG_Error("Unknow device type");
        return;
	}

    if (NULL == p_2g_frq || NULL == pst_bbPowerOpenRefValue)
    {
        DLOG_Error("Fail Get Node");
        return;
    }

	BB_freq_power_table(RF_2G,p_2g_frq,pst_bbPowerOpenRefValue);
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

		RF8003_SPI_WriteReg_internal(0x2a, pwr_a);
        RF8003_SPI_WriteReg_internal(0x2e, pwr_b);
	}
	else
	{
		
	}

}

void BB_set_RF_mimo_mode(ENUM_MIMO_MODE e_mimo_mode)
{
    //todo
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
    return u8_rcFreqCnt_2g;
}

uint8_t BB_GetItFrqNumPerFilter(void)
{
    return u8_itFreqCnt_2g;
}

uint16_t BB_GetRcFrqByCh(uint8_t ch)
{
	if(pRcFreqlist ==NULL)return 0;
	if(ch>u8_rcFreqCnt_2g)return 0;
	return pRcFreqlist[ch];
}

uint16_t BB_GetItFrqByCh(uint8_t ch)
{
	if(pItFreqlist==NULL) return 0;
	if(ch > u8_itFreqCnt_2g )return 0;
	return pItFreqlist[ch];
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
    if(NULL != start)
    {
        *start = 0;
    }
    if(NULL != end)
    {
        *end = u8_itFreqCnt_2g;
    }

    return 0;
}

uint8_t BB_GetRcStarEndByItCh(uint8_t *start, uint8_t *end, uint8_t it_ch)
{
    if(NULL != start)
    {
        *start = 0;
    }
    if(NULL != end)
    {
        *end = u8_rcFreqCnt_2g;
    }

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

