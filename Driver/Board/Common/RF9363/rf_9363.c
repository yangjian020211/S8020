/*****************************************************************************
Copyright: 2016-2020, Artosyn. Co., Ltd.
File name: rf_9363.c
Description: The internal APIs to control the RF 9363
Author: Artosy Software Team
Version: 0.0.1
Date: 2017/07/04
History: 
        0.0.1    2016/12/20    The initial version of RF9363.c
*****************************************************************************/
#include "debuglog.h"
#include "bb_spi.h"
#include "rf_if.h"
#include "systicks.h"
#include "bb_ctrl_internal.h"
#include "cfg_parser.h"
#include "factory.h"
#include "log10.h"
#include "boardParameters.h"

#define     IT_FRQ_NUM_PER_FILTER               (2)
#define     RC_FRQ_NUM_PER_FILTER               (8)

#define     RF9363_RF_CLOCKRATE                  (9)    //9MHz clockrate

#define     AAGC_GAIN_FAR_9363                   (0x4C)
#define     AAGC_GAIN_NEAR_9363                  (0x18)


static uint8_t *RF9363_common_regs;
static uint32_t u32_common_regscnt;

static uint8_t *RF9363_sky_regs;
static uint32_t u32_sky_regscnt;

static uint8_t *RF9363_grd_regs;
static uint32_t u32_grd_regscnt;

static STRU_FRQ_CHANNEL pstru_rcFreq[40];
static STRU_FRQ_CHANNEL pstru_itFreq[40];

static uint8_t u8_rcFreqCnt;
static uint8_t u8_itFreqCnt;

static uint16_t *pRcFreq = NULL;
static uint16_t *pItFreq = NULL;

static STRU_BB_CH_OPEN_POWER_REF_VALUE *pst_bbPowerOpenRefValue = NULL;
static uint8_t u8_2g_bb_power_a[40];
static uint8_t u8_2g_bb_power_b[40];
static uint8_t u8_subBand10MrcFreqNum;
static uint8_t u8_subBand20MrcFreqNum;

static int AR_round(float x)
{
    if (x > 0x0)
        return ((int)(x + 0.5));
    else
        return ((int)(x - 0.5));
}

static uint8_t BB_calu_power_reg(uint16_t frq,uint16_t refPower,uint8_t refValueBase,uint8_t refValueCnt, STRU_OPEN_POWER_MEAS_VALUE *pst_PowerOpenRefValue)
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

static void BB_freq_power_table(ENUM_RF_BAND band,STRU_RF_CHANNEL *p_frq,STRU_BB_CH_OPEN_POWER_REF_VALUE *pst_PowerOpenRefValue)
{
    uint8_t i;

    //if(band == RF_2G)
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

static void BB_gen_open_power_makeup_value(ENUM_BB_MODE en_mode)
{
    STRU_cfgNode *node;
    STRU_RF_CHANNEL *p_2g_frq;

    if(en_mode == BB_SKY_MODE)
    {
        p_2g_frq = FCT_GetNodeAndData(FACTORY_SUBNODE_BAND0_VT_10M_FRQ_ID, &node);
        pst_bbPowerOpenRefValue = FCT_GetNodeAndData(FACTORY_SUBNODE_POWEROPEN_VT_SET_ID, NULL);
    }
    else if(en_mode == BB_GRD_MODE)
    {
        p_2g_frq = FCT_GetNodeAndData(FACTORY_SUBNODE_BAND0_RC_10M_FRQ_ID, &node);
        pst_bbPowerOpenRefValue = FCT_GetNodeAndData(FACTORY_SUBNODE_POWEROPEN_RC_SET_ID, NULL);
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


static int RF9363_SPI_WriteReg_internal(uint8_t u8_data[3])
{
    int ret = 0;

    SPI_write_read(BB_SPI_BASE_IDX, u8_data, 3, 0, 0); 
    ret =  SPI_WaitIdle(BB_SPI_BASE_IDX, BB_SPI_MAX_DELAY);

    return ret;
}

static int RF9363_SPI_ReadReg_internal(uint8_t u8_data[2], uint8_t *pvalue)
{
    uint8_t ret;
 
    SPI_write_read(BB_SPI_BASE_IDX, u8_data, 2, pvalue, 1);
    ret = SPI_WaitIdle(BB_SPI_BASE_IDX, BB_SPI_MAX_DELAY);

    return ret;
}


/**
  * @brief : Write 9363 RF register by SPI 
  * @param : addr: 9363 SPI address
  * @param : data: data for 9363
  * @retval  0: sucess   1: FAIL
  */
int RF_SPI_WriteReg(uint16_t u16_addr, uint8_t u8_value)
{
    uint8_t ret;
    uint16_t addr = (u16_addr & 0x0FFF) ;
    uint8_t data[3] = { ((u16_addr>>8) | 0x80) & 0xff,  u16_addr&0xff, u8_value};

    ret = RF9363_SPI_WriteReg_internal(data);    

    return ret;
}


/**
  * @brief : Read 9363 RF register by SPI 
  * @param : addr: 9363 SPI address
  * @retval  0: sucess   1: FAIL
  */
int RF_SPI_ReadReg(uint16_t u16_addr, uint8_t *pu8_rxValue)
{
    int ret = 0;
    uint8_t data[2] = {(u16_addr>>8)&0xff,  u16_addr&0xff};

    ret = RF9363_SPI_ReadReg_internal(data, pu8_rxValue);
    return ret;
}


void config_9363_regs(STRU_RF_REG *RF1_9363_regs, uint16_t idx)
{
    uint8_t data[3];
    uint16_t addr;    
    int ret;

    if ( RF1_9363_regs->addr_h == 0xFF) //wait timeout
    {
       uint16_t u16_delay = ((RF1_9363_regs->addr_l << 8) | RF1_9363_regs->value);
       SysTicks_DelayMS(u16_delay);

       DLOG_Warning("waittimeout: 0x%x %d", u16_delay, idx);
    }
    else if ( ((RF1_9363_regs->addr_h) & 0xF0) == 0xF0) //wait regiters done
    {
        uint16_t count = 0;
        uint8_t waitbit   = (RF1_9363_regs->value & 0x0F);
        uint8_t donevalue = (RF1_9363_regs->value >> 4);
        addr = ((RF1_9363_regs->addr_h & 0x0F) << 8) | (RF1_9363_regs->addr_l);

        do
        {
            ret = RF_SPI_ReadReg(addr, data+2);
            SysTicks_DelayMS(2);
            count ++;
        }while( ( (data[2] >> waitbit) & 0x01) != donevalue && count <= 2000);

        if( count >= 2000)
        {
            DLOG_Error("9363: cali TIMOUT");
        }
    }
    else
    {
        uint8_t flag = (RF1_9363_regs->addr_h & 0xF0);
        if ( flag == 0x80 ) //write
        {
            if (RF1_9363_regs->addr_l == 0xA3)
            {
                //RF1_9363_regs->value |= (reg_0xa3 & 0xC0); //update bit[7:6]
                RF_SPI_WriteReg(0xa3, RF1_9363_regs->value);
            }
            else
            {
                addr = (RF1_9363_regs->addr_h <<8) | RF1_9363_regs->addr_l;
                ret = RF_SPI_WriteReg(addr, RF1_9363_regs->value);
            }
        }
        else if(flag == 0xe0)
        {
            uint8_t regout;
            RF_SPI_ReadReg(addr, &regout);
        }
        else
        {
            DLOG_Error("reg map: %d 0x%x 0x%x 0x%x", idx, RF1_9363_regs->addr_h , RF1_9363_regs->addr_l, RF1_9363_regs->value);
        }
    }
}


static void RF9363_CalcFrq2Register(uint16_t u16_frq, STRU_FRQ_CHANNEL *frq_regvalue)
{
    uint8_t  integer;
    uint32_t fraction;

    integer  = u16_frq / 50;
    fraction = ((double)u16_frq / 50 - integer) * 8388593;

    frq_regvalue->frq1 = integer;
    frq_regvalue->frq2 = 0;
    frq_regvalue->frq3 = (fraction & 0xff);
    frq_regvalue->frq4 = ((fraction >>  8) & 0xff);
    frq_regvalue->frq5 = ((fraction >> 16) & 0xff);

    DLOG_Info("%d %x %x %x %x %x", u16_frq, frq_regvalue->frq1, frq_regvalue->frq2, frq_regvalue->frq3, frq_regvalue->frq4, frq_regvalue->frq5);
}


void RF9363_CalcFctNode2RegTable(uint32_t nodeid, STRU_FRQ_CHANNEL *frq_regvalue, uint8_t * frq_cnt)
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
        RF9363_CalcFrq2Register(p_frq->u16_rfChFrqList  [i], (frq_regvalue + i));
    }
    if(FACTORY_SUBNODE_BAND0_RC_10M_FRQ_ID == nodeid)
    {
        pRcFreq = p_frq->u16_rfChFrqList;
    }
    else if(FACTORY_SUBNODE_BAND0_VT_10M_FRQ_ID == nodeid)
    {
        pItFreq = p_frq->u16_rfChFrqList;
    }
}

void RF9363_GetFctFreqTable(void)
{
    RF9363_CalcFctNode2RegTable(FACTORY_SUBNODE_BAND0_RC_10M_FRQ_ID, pstru_rcFreq, &u8_rcFreqCnt);
    RF9363_CalcFctNode2RegTable(FACTORY_SUBNODE_BAND0_VT_10M_FRQ_ID, pstru_itFreq, &u8_itFreqCnt);
}

void rf9363_checkDeviceId(void)
{
    uint8_t deviceid;

    RF_SPI_ReadReg(0x37, &deviceid);
    if(deviceid != 0x0a)
    {
        DLOG_Error("Error: deviceiD =0x%x", deviceid);
    }
}

static void RF9363_getCfgData(ENUM_BB_MODE en_mode, STRU_cfgBin *cfg)
{
    STRU_cfgNode *pcfg_node;

    RF9363_common_regs = CFGBIN_GetNodeAndData(cfg, RF_INIT_REG_NODE_ID_0, &pcfg_node);
    if (NULL!=pcfg_node && NULL!=RF9363_common_regs)
    {
        u32_common_regscnt  = pcfg_node->nodeElemCnt;
    }

    RF9363_sky_regs = CFGBIN_GetNodeAndData(cfg, RF_SKY_INIT_REG_ID_0, &pcfg_node);
    if (NULL!=pcfg_node && NULL != RF9363_sky_regs)
    {
        u32_sky_regscnt  = pcfg_node->nodeElemCnt;
    }

    RF9363_grd_regs = CFGBIN_GetNodeAndData(cfg, RF_GRD_INIT_REG_ID_0, &pcfg_node);
    if (NULL!=pcfg_node && NULL != RF9363_grd_regs)
    {
        u32_grd_regscnt  = pcfg_node->nodeElemCnt;
    }
}


/**
  * @brief : init RF9363 register
  * @param : addr: 9363 SPI address
  * @retval  None
  */
void RF_init(ENUM_BB_MODE en_mode)
{
    uint16_t idx;
    uint16_t cnt;

    RF9363_GetFctFreqTable();

    BB_gen_open_power_makeup_value(en_mode);

    RF9363_getCfgData(en_mode, (STRU_cfgBin *)SRAM_CONFIGURE_MEMORY_ST_ADDR);
    STRU_RF_REG *RF1_9363_regs = (STRU_RF_REG *)RF9363_common_regs;

    BB_WriteRegMask(PAGE2, 0x00, 0x01, 0x01);        //hold BB sw reset

    BB_SPI_curPageWriteByte(0x01, 0x01);             //bypass: SPI change into 9363
    rf9363_checkDeviceId();

    cnt = u32_common_regscnt;
    RF1_9363_regs = (STRU_RF_REG *)RF9363_common_regs;
    for(idx = 0; idx < cnt-1; idx++)
    {
        config_9363_regs(RF1_9363_regs+idx, idx);
    }

    if (en_mode == BB_GRD_MODE)
    {
        cnt = u32_grd_regscnt;
        RF1_9363_regs = (STRU_RF_REG *)RF9363_grd_regs;
    }
    else
    {
        cnt = u32_sky_regscnt;
        RF1_9363_regs = (STRU_RF_REG *)RF9363_sky_regs;
    }

    for(idx = 0; idx < cnt ; idx++)
    {
        config_9363_regs(RF1_9363_regs+idx, idx);
    }

    RF_SPI_WriteReg(0x14, 0x09);
    RF_SPI_WriteReg(0x14, 0x29);

    BB_SPI_curPageWriteByte(0x01,0x02);             //SPI change into 8020
    BB_WriteRegMask(PAGE2, 0x00, 0x01, 0x00);       //release BB sw reset
}


void BB_RF_band_switch(ENUM_RF_BAND rf_band)
{
    return;
}

int RF9363_SetPowerModeAfterCali(STRU_RF_POWER_CTRL * pst_power)
{
    BB_SPI_curPageWriteByte(0x01, 0x01);            //bypass: SPI change into 9363

    BB_SPI_curPageWriteByte(0x01,0x02);             //SPI change into 8020
    return 0;
}



void BB_set_power_close(ENUM_RF_BAND band,uint8_t power)
{

}

void  BB_set_power_open(ENUM_RF_BAND band,uint8_t power)
{
    RF_SPI_WriteReg(0x073, power);
    RF_SPI_WriteReg(0x075, power);
}


uint8_t BB_GetSkySweepFrqNum(ENUM_RF_BAND e_rfBand)
{
    return 0;
}

void RF_CaliProcess(ENUM_BB_MODE en_mode)
{
    //before calibration: do nothing

    // set RF after calibration
    STRU_cfgNode *cfgnode;
    STRU_RF_POWER_CTRL *pst_powercfg = NULL;
    pst_powercfg = (STRU_RF_POWER_CTRL *)FCT_GetNodeAndData(FACTORY_SUBNODE_POWER_NODE_ID, &cfgnode);
    RF9363_SetPowerModeAfterCali(pst_powercfg);

    return;
}

void BB_write_ItRegs(uint8_t *u8_it)
{
    context.stru_itRegs.frq1 = u8_it[0];
    context.stru_itRegs.frq2 = 0;
    context.stru_itRegs.frq3 = u8_it[1];
    context.stru_itRegs.frq4 = u8_it[2];
    context.stru_itRegs.frq5 = u8_it[3];

    BB_WriteReg(PAGE2, 0x10, context.stru_itRegs.frq1);
    BB_WriteReg(PAGE2, 0x11, context.stru_itRegs.frq2);
    BB_WriteReg(PAGE2, 0x12, context.stru_itRegs.frq3);
    BB_WriteReg(PAGE2, 0x13, context.stru_itRegs.frq4);
    BB_WriteReg(PAGE2, 0x14, context.stru_itRegs.frq5);
}


uint8_t BB_set_ItFrqByCh(ENUM_RF_BAND band, uint8_t ch)
{
    STRU_FRQ_CHANNEL *it_ch_ptr = pstru_itFreq;

    context.stru_itRegs.frq1 = it_ch_ptr[ch].frq1;
    context.stru_itRegs.frq2 = it_ch_ptr[ch].frq2;
    context.stru_itRegs.frq3 = it_ch_ptr[ch].frq3;
    context.stru_itRegs.frq4 = it_ch_ptr[ch].frq4;
    context.stru_itRegs.frq5 = it_ch_ptr[ch].frq5;

    BB_WriteReg(PAGE2, 0x10, it_ch_ptr[ch].frq1);
    BB_WriteReg(PAGE2, 0x11, it_ch_ptr[ch].frq2);
    BB_WriteReg(PAGE2, 0x12, it_ch_ptr[ch].frq3);
    BB_WriteReg(PAGE2, 0x13, it_ch_ptr[ch].frq4);
    BB_WriteReg(PAGE2, 0x14, it_ch_ptr[ch].frq5);

    if(!pst_bbPowerOpenRefValue->u8_powerOpenMakeupEnable)
    {
        return 0;
    }
    //set channel power
    if (context.en_bbmode == BB_SKY_MODE)
    {
        //if((band == RF_2G))
        {
            BB_WriteReg(PAGE0, 0x09, u8_2g_bb_power_a[ch]);
            BB_WriteReg(PAGE0, 0x16, u8_2g_bb_power_b[ch]);
            //DLOG_Warning("VT 2g open power -> ch %d, 0x09=%2x,0x16=%2x",ch,u8_2g_bb_power_a[ch],u8_2g_bb_power_b[ch]);
        }
    }

    return 0;
}
void dump_freq(uint8_t offset , uint8_t cnt, uint8_t val)
{

}

uint8_t BB_write_RcRegs(uint8_t *u8_rc)
{
    context.stru_rcRegs.frq1 = u8_rc[0];
    context.stru_rcRegs.frq2 = 0;
    context.stru_rcRegs.frq3 = u8_rc[1];
    context.stru_rcRegs.frq4 = u8_rc[2];
    context.stru_rcRegs.frq5 = u8_rc[3];

    BB_WriteReg(PAGE2, 0x1A, context.stru_rcRegs.frq1);
    BB_WriteReg(PAGE2, 0x1B, context.stru_rcRegs.frq2);
    BB_WriteReg(PAGE2, 0x1C, context.stru_rcRegs.frq3);
    BB_WriteReg(PAGE2, 0x1D, context.stru_rcRegs.frq4);
    BB_WriteReg(PAGE2, 0x1E, context.stru_rcRegs.frq5);
}
uint8_t BB_set_skySweepVtfrq(ENUM_RF_BAND band, uint8_t ch)
{
    return 0;
}

uint8_t BB_set_skySweepfrq(ENUM_RF_BAND band, uint8_t ch)
{
    STRU_FRQ_CHANNEL *pu8_rcRegs = pstru_rcFreq;

    BB_WriteReg(PAGE2, 0x15, pu8_rcRegs[ch].frq1);
    BB_WriteReg(PAGE2, 0x16, pu8_rcRegs[ch].frq2);
    BB_WriteReg(PAGE2, 0x17, pu8_rcRegs[ch].frq3);
    BB_WriteReg(PAGE2, 0x18, pu8_rcRegs[ch].frq4); 
    BB_WriteReg(PAGE2, 0x19, pu8_rcRegs[ch].frq5); 
    
    return 0;
}


uint8_t BB_set_Rcfrq(ENUM_RF_BAND band, uint8_t ch)
{
    STRU_FRQ_CHANNEL *pu8_rcRegs = pstru_rcFreq;

    context.stru_rcRegs.frq1 = pu8_rcRegs[ch].frq1;
    context.stru_rcRegs.frq2 = pu8_rcRegs[ch].frq2;
    context.stru_rcRegs.frq3 = pu8_rcRegs[ch].frq3;
    context.stru_rcRegs.frq4 = pu8_rcRegs[ch].frq4;
    context.stru_rcRegs.frq5 = pu8_rcRegs[ch].frq5;

    BB_WriteReg(PAGE2, 0x1A, pu8_rcRegs[ch].frq1);
    BB_WriteReg(PAGE2, 0x1B, pu8_rcRegs[ch].frq2);
    BB_WriteReg(PAGE2, 0x1C, pu8_rcRegs[ch].frq3);
    BB_WriteReg(PAGE2, 0x1D, pu8_rcRegs[ch].frq4); 
    BB_WriteReg(PAGE2, 0x1E, pu8_rcRegs[ch].frq5); 

    if(!pst_bbPowerOpenRefValue->u8_powerOpenMakeupEnable)
    {
        return 0;
    }

    if (context.en_bbmode == BB_GRD_MODE)
    {        
        //if((band == RF_2G))
        {
            BB_WriteReg(PAGE0, 0x09, u8_2g_bb_power_a[ch]);
        }
    }

    return 0;
}


uint8_t BB_set_SweepFrq(ENUM_RF_BAND band, ENUM_CH_BW e_bw, uint8_t ch)
{
    STRU_FRQ_CHANNEL *ch_ptr = pstru_itFreq;

    BB_WriteReg(PAGE2, 0x15, ch_ptr[ch].frq1);
    BB_WriteReg(PAGE2, 0x16, ch_ptr[ch].frq2);
    BB_WriteReg(PAGE2, 0x17, ch_ptr[ch].frq3);
    BB_WriteReg(PAGE2, 0x18, ch_ptr[ch].frq4);
    BB_WriteReg(PAGE2, 0x19, ch_ptr[ch].frq5);
    
    return 0;
}



uint8_t BB_GetRcFrqNum(ENUM_RF_BAND e_rfBand)
{
    return u8_rcFreqCnt;
}

uint8_t BB_GetItFrqNum(ENUM_RF_BAND e_rfBand)
{
    return u8_itFreqCnt;
}

uint16_t BB_GetRcFrqByCh(uint8_t ch)
{
    if( (pRcFreq != NULL) && (ch < u8_rcFreqCnt) )
    {
        return pRcFreq[ch];
    }
    else
    {
        return 0;
    }
}

uint16_t BB_GetItFrqByCh(uint8_t ch)
{
    if( (pItFreq != NULL) && (ch < u8_itFreqCnt) )
    {
        return pItFreq[ch];
    }
    else
    {
        return 0;
    }
}

uint8_t BB_GetItChInOneFilter(uint8_t ch)
{
    if(0 == (ch % IT_FRQ_NUM_PER_FILTER))
    {
        return (ch + 1);
    }
    else
    {
        return (ch - 1);
    }
}

uint8_t BB_GetItStarEndByItCh(uint8_t *start, uint8_t *end, uint8_t it_ch)
{
    if(0 == (it_ch % IT_FRQ_NUM_PER_FILTER))
    {
        if(NULL != start)
        {
            *start = it_ch;
        }
        if(NULL != end)
        {
            *end = it_ch + IT_FRQ_NUM_PER_FILTER;
        }
    }
    else
    {
        if(NULL != start)
        {
            *start = it_ch - 1;
        }
        if(NULL != end)
        {
            *end = it_ch + 1;
        }
    }

    return 0;
}

uint8_t BB_GetRcStarEndByItCh(uint8_t *start, uint8_t *end, uint8_t it_ch)
{
    uint8_t filter = (it_ch / IT_FRQ_NUM_PER_FILTER);

    
    if(NULL != start)
    {
        *start = filter * RC_FRQ_NUM_PER_FILTER;
    }
    if(NULL != end)
    {
        *end = (filter + 1) * RC_FRQ_NUM_PER_FILTER;
    }

    return 0;
}


uint8_t BB_SetAgcGain(ENUM_RF_BAND e_rfBand, uint8_t gain)
{
    if(AAGC_GAIN_FAR == gain)
    {
        BB_WriteReg(PAGE1, 0x03, 0x30);
        BB_WriteReg(PAGE1, 0x0c, 0x08);

        BB_WriteReg(PAGE0, AGC_2, AAGC_GAIN_FAR_9363);
        BB_WriteReg(PAGE0, AGC_3, AAGC_GAIN_FAR_9363);
    }
    else if(AAGC_GAIN_NEAR == gain)
    {
        BB_WriteReg(PAGE1, 0x03, 0x30);
        BB_WriteReg(PAGE1, 0x0c, 0x08);

        BB_WriteReg(PAGE0, AGC_2, AAGC_GAIN_NEAR_9363);
        BB_WriteReg(PAGE0, AGC_3, AAGC_GAIN_NEAR_9363);
    }
    else
    {
        ;
    }

    return 0;
}

int32_t BB_SweepEnergyCompensation(int32_t data)
{
    return ( data += (-136) );
}


int32_t BB_SweepEnergy()
{
    int32_t power_db;
    uint32_t power_td = (((uint32_t)(BB_ReadReg(PAGE2, SWEEP_ENERGY_HIGH)) << 16) |
                                     (BB_ReadReg(PAGE2, SWEEP_ENERGY_MID) << 8)  |
                                     (BB_ReadReg(PAGE2, SWEEP_ENERGY_LOW)));
    
    if (power_td == 0)
    {
        return 0;
    }

    return BB_SweepEnergyCompensation(get_10log10(power_td));
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

    RF_SPI_WriteReg(u16_addr & 0x0FFF, u8_data);
    
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

    RF_SPI_ReadReg(u16_addr & 0x0FFF, pu8_rxValue);
    
    BB_SPI_curPageWriteByte(0x01,0x02);             //SPI change into 8020    
}


void BB_set_RF_mimo_mode(ENUM_MIMO_MODE e_mimo_mode)
{
    //todo
}

int RF_RegValue2Frq(uint8_t value[], uint16_t *frq)
{
    uint32_t integer = (uint32_t)value[0];
    uint32_t fraction = (value[3] << 16) | (value[2] << 8) | value[1];

    *frq = (uint16_t)((fraction / 8388593.0 + integer) * 5.0 + 0.5);

    return 0;
}

uint8_t BB_GetRcFrqNumPerFilter(void)
{
    return (uint8_t)RC_FRQ_NUM_PER_FILTER;
}

uint8_t BB_GetItFrqNumPerFilter(void)
{
    return (uint8_t)IT_FRQ_NUM_PER_FILTER;
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

