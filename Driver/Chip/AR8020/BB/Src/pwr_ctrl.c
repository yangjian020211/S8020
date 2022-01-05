#include <stdlib.h>
#include "debuglog.h"
#include "systicks.h"
#include "pwr_ctrl.h"
#include "memory_config.h"
#include "bb_ctrl_internal.h"
#include "boardParameters.h"
#include "bb_ctrl.h"
#include "rf_if.h"

#define     PWR_CTRL_ON        (0x55)
#define     PWR_CTRL_OFF       (0xAA)

extern volatile CONTEXT context;
extern STRU_BOARD_BB_PARA *pstru_bb_boardcfg;
extern uint8_t wake_up_flag;
extern STRU_PWR_CTRL *pstru_pwr_ctrl_cfg;
//extern uint8_t cpu2_sleep_flag;

// page1 0x90 register define
#define		EN_ABB				(1 << 7) //	EN_ABB
#define		EN_ADC_A			(1 << 6) //	Rx A enable
#define		EN_ADC_B			(1 << 5) //	Rx B enable
#define		EN_DAC_A			(1 << 4) //	Tx A enable
#define		EN_DAC_B			(1 << 3) //	Tx B enable
#define		EN_SAR8_1			(1 << 2) //	SAR8_1 enable
#define		EN_SAR10			(1 << 1) //	SAR10 enable
#define		EN_PLL_ADDA_LDO		(1 << 0) //	PLL_ADDA LDO enable

// page1 0x91 register define
#define		EN_ADC_C_D			(1 << 7) //	ADC C&D enable
#define		EN_PLL_DSP1_LDO		(1 << 6) //	PLL_DSP1 LDO enable
#define		EN_PLL_DSP2_LDO		(1 << 5) //	PLL_DSP2 LDO enable
#define		EN_SAR8_2			(1 << 4) //	SAR8 2 enable
#define		EN_PLL_DSP3_LDO		(1 << 3) //	PLL DSP3 LDO enable 


int PWR_CTRL_Init(STRU_PWR_CTRL *ctrl)
{
    if (PWR_LEVEL0 != ctrl->pwr_ctrl)
    {
        //if(BB_SKY_MODE == context.en_bbmode)
        {
            PWR_CTRL_ModeSet(ctrl);
            DLOG_Warning("power-on2sleep %d",ctrl->pwr_ctrl);
        }
        
        return 0;
    }

    return -1;
}

static int PWR_CTRL_ModeSet2t2rSingle(uint8_t ch, uint8_t flag)
{
#define     EN_RX_A         (1 << 6)
#define     EN_RX_B         (1 << 5)
#define     EN_TX_A         (1 << 4)
#define     EN_TX_B         (1 << 3)

    uint8_t value = 0;
    uint8_t rf_org = 0;
    uint8_t bb_org = 0;

    if(0 == ch)
    {
        value = (EN_RX_A | EN_TX_A);
    }
    else if(1 ==  ch)
    {
        value = (EN_RX_B | EN_TX_B);
    }
    else
    {
        DLOG_Warning("err ch:%d", ch);
        return -1;
    }

    if(PWR_CTRL_OFF == flag)
    {
        RF_PcReadReg(0, 0x00, &rf_org);   //
        RF_PcWriteReg(0, 0x00, rf_org & (~value));     //
        
        bb_org = BB_ReadReg(PAGE1, 0x90);
        BB_WriteReg(PAGE1, 0x90, bb_org & (~value));
    }
    else
    {
        RF_PcReadReg(0, 0x00, &rf_org);   //
        RF_PcWriteReg(0, 0x00, rf_org | value);     //
        
        bb_org = BB_ReadReg(PAGE1, 0x90);
        BB_WriteReg(PAGE1, 0x90, bb_org | value);
    }

    return 0;
}

static int PWR_CTRL_ModeSetBbReset(void)
{
    uint8_t data = BB_ReadReg(PAGE2, 0);
    BB_WriteReg(PAGE2, 0, data | 1);

    return 0;
}

static int PWR_CTRL_ModeSetRfChip(uint8_t flag)
{
    uint8_t data,rf_org=0,rf_org1=0,cnt=0;
    
    if(PWR_CTRL_OFF == flag)
    {
        //if(context.st_bandMcsOpt.e_bandsupport & RF_2G)
            RF_PcWriteReg(0, 0x00, 0x00);
        if(context.st_bandMcsOpt.e_bandsupport & RF_5G)
            RF_PcWriteReg(0, 0x40, 0x00);
        RF_PcWriteReg(0, 0x1B, 0xC0);
        BB_WriteRegMask(PAGE1, 0xB6, 0, 0xC0); // 2g 9v 10ma
    }
    else
    {
        do{
            //if(context.st_bandMcsOpt.e_bandsupport & RF_2G)
                RF_PcWriteReg(0, 0x00, 0x78);
            if(context.st_bandMcsOpt.e_bandsupport & RF_5G)
                RF_PcWriteReg(0, 0x40, 0x78);
            
            //if(context.st_bandMcsOpt.e_bandsupport & RF_2G)
                RF_PcReadReg(0, 0x00, &rf_org);
            //else
            //    rf_org = 0x78;
            if(context.st_bandMcsOpt.e_bandsupport & RF_5G)
                RF_PcReadReg(0, 0x40, &rf_org1);
            else
                rf_org1 = 0x78;
            cnt++;
            if(cnt > 100){
                DLOG_Warning("%d",cnt);
                cnt = 0;
            }
        }while(rf_org != 0x78 || rf_org1 != 0x78);
        DLOG_Warning("%d",cnt);
        RF_PcWriteReg(0, 0x1B, 0x00);
        BB_WriteRegMask(PAGE1, 0xB6, 0xC0, 0xC0);
    }
    
    return 0;
}

static int PWR_CTRL_ModeSetAdDa(uint8_t flag)
{
    uint8_t data = 0;
    
    if(PWR_CTRL_OFF == flag)
    {
        data = EN_ABB | EN_PLL_ADDA_LDO | EN_SAR10;
        BB_WriteReg(PAGE1, 0x90, data);
        data = EN_PLL_DSP1_LDO | EN_PLL_DSP2_LDO | EN_PLL_DSP3_LDO;
        BB_WriteReg(PAGE1, 0x91, data);
    }
    else
    {
        BB_WriteReg(PAGE1, 0x90, 0xFF);
        BB_WriteReg(PAGE1, 0x91, 0xF8);
    }
    
    return 0;
}

// page0 0x20 register define
#define        PA_0                (1 << 3) //    
#define        PA_1                (1 << 2) //    

static int PWR_CTRL_ModeSetPa(uint8_t flag)
{
    uint8_t data;
    
    if(PWR_CTRL_OFF == flag)
    {
        data = BB_ReadReg(PAGE0, 0x20); // p0 0x20[3]/0x20[2] 2G/5G external PA off
        data &= (~(PA_0 | PA_1 | 0x03));       // 2 channel off
        BB_WriteReg(PAGE0, 0x20, data); //
    }
    else
    {
        data = BB_ReadReg(PAGE0, 0x20); // p0 0x20[3]/0x20[2] 2G/5G external PA
        if(RF_5G == context.e_curBand) // 5G
        {
            data |= PA_1; // just power on work channel
        }
        else
        {
            data |= PA_0;
        }
        BB_WriteReg(PAGE0, 0x20, data | 0x02);
    }
    
    return 0;
}

static void set_rf_mode(int mode)
{
	uint8_t v;
	BB_SPI_curPageWriteByte(0x01, 0x01); // switch to AR8003
	RF_SPI_ReadReg(0x33,&v);//read 0x33 
	if(mode)
	{
		RF_SPI_WriteReg(0x33,v | 0x0c );//set 0x33[3:0]= 2'b11
	}
	else
	{
		RF_SPI_WriteReg(0x33,v & 0xf3 );//set 0x33[3:0]= 2'b00
	}
	BB_SPI_curPageWriteByte(0x01,0x02); //switch back to baseband
}


int PWR_CTRL_ModeSet(STRU_PWR_CTRL *ctrl)
{
	uint8_t v;
    STRU_DEVICE_INFO *pst_devInfo = (STRU_DEVICE_INFO *)(DEVICE_INFO_SHM_ADDR);
    
    if((ctrl->pwr_ctrl) <= PWR_LEVEL3)
    {
        pst_devInfo->sleep_level= ctrl->pwr_ctrl;
    }
    
    switch(ctrl->pwr_ctrl)
    {
        case PWR_LEVEL0:
        {
            //PWR_CTRL_ModeSetPa(PWR_CTRL_ON);
            PWR_CTRL_ModeSetRfChip(PWR_CTRL_ON);
            PWR_CTRL_ModeSetAdDa(PWR_CTRL_ON);
            //PWR_CTRL_ModeSet2t2rSingle(0, PWR_CTRL_ON);
            //PWR_CTRL_ModeSet2t2rSingle(1, PWR_CTRL_ON);
            //context.stru_bandSwitchParam.u8_bandSwitchAfterUnlock = (pstru_pwr_ctrl_cfg->pwr_ctrl == PWR_LEVEL0);

			//for uav project and others 
			// set RF to be ppa mode,0x33[3:0]= 2'b11: pa mode; and 0x33[3:0]=2'b00:ppa mode
			set_rf_mode(0);
			BB_SetPowerCloseMode(RF_2G);// 3 SetPowerOpenMode	
			context.sleep_level = PWR_LEVEL0;   
			break;
        }
        case PWR_LEVEL1: // force to 2g
        {
            PWR_CTRL_ModeSetRfChip(PWR_CTRL_ON);
            PWR_CTRL_ModeSetAdDa(PWR_CTRL_ON);
            PWR_CTRL_ModeSet2t2rSingle(0, PWR_CTRL_ON);
            PWR_CTRL_ModeSet2t2rSingle(1, PWR_CTRL_OFF);
            /*if((RF_5G == context.e_curBand) && (RF_2G_5G == context.st_bandMcsOpt.e_bandsupport))
            {
                sky_requestRfBandSwitch(RF_2G);
            }
            context.stru_bandSwitchParam.u8_bandSwitchAfterUnlock = 0;*/
            context.sleep_level = PWR_LEVEL1;           
            break;
        }
        case PWR_LEVEL2:
        {
            PWR_CTRL_ModeSet2t2rSingle(0, PWR_CTRL_OFF);
            PWR_CTRL_ModeSet2t2rSingle(1, PWR_CTRL_OFF);
            PWR_CTRL_ModeSetBbReset();
            wake_up_flag = 0x55;
            context.sleep_level = PWR_LEVEL2;           
            break;
        }
        case PWR_LEVEL3:
        {
            PWR_CTRL_ModeSet2t2rSingle(0, PWR_CTRL_OFF);
            PWR_CTRL_ModeSet2t2rSingle(1, PWR_CTRL_OFF);
            PWR_CTRL_ModeSetAdDa(PWR_CTRL_OFF);
            PWR_CTRL_ModeSetRfChip(PWR_CTRL_OFF);
            //PWR_CTRL_ModeSetPa(PWR_CTRL_OFF);
            PWR_CTRL_ModeSetBbReset();
            wake_up_flag = 0xAA;
            context.sleep_level = PWR_LEVEL3;           
            break;
        }
        case PWR_LEVEL4: // force to 2g
        {
            PWR_CTRL_ModeSetRfChip(PWR_CTRL_ON);
            PWR_CTRL_ModeSetAdDa(PWR_CTRL_ON);
            PWR_CTRL_ModeSet2t2rSingle(0, PWR_CTRL_ON);
            PWR_CTRL_ModeSet2t2rSingle(1, PWR_CTRL_OFF);

			BB_SetPowerOpenMode(RF_2G);
			set_rf_mode(1);
            context.sleep_level = PWR_LEVEL4;
            break;
        }
        case 0x10: 
        {
            PWR_CTRL_ModeSet2t2rSingle(0, PWR_CTRL_OFF);
            PWR_CTRL_ModeSet2t2rSingle(1, PWR_CTRL_OFF);
            break;
        }
        case 0x11:
        {
            PWR_CTRL_ModeSetAdDa(PWR_CTRL_OFF);
            break;
        }
        case 0x12:
        {
            PWR_CTRL_ModeSetRfChip(PWR_CTRL_OFF);
            break;
        }
        case 0x13:
        {
            PWR_CTRL_ModeSetPa(PWR_CTRL_OFF);
            break;
        }
        case 0x14:
        {
            PWR_CTRL_ModeSetBbReset();
            break;
        }
        case 0x20: 
        {
            PWR_CTRL_ModeSet2t2rSingle(0, PWR_CTRL_ON);
            PWR_CTRL_ModeSet2t2rSingle(1, PWR_CTRL_ON);
            break;
        }
        case 0x21:
        {
            PWR_CTRL_ModeSetAdDa(PWR_CTRL_ON);
            break;
        }
        case 0x22:
        {
            PWR_CTRL_ModeSetRfChip(PWR_CTRL_ON);
            break;
        }
        case 0x23:
        {
            PWR_CTRL_ModeSetPa(PWR_CTRL_ON);
            break;
        }
        default:
        {
            
            break;
        }
    }
    DLOG_Warning("%d",context.sleep_level);
}

