/*====================================================*/
/*              Include         Files                 */
/*====================================================*/
#include "enc_internal.h"
#include "vsoc_enc.h"
#include "debuglog.h"
#include <string.h>
#include "reg_rw.h"
#include "data_type.h"
#include "brc.h"

extern RC_DATA rca[];

static STRU_ENC_CUSTOMER_CFG g_stEncUsrCfg[2] = {
                                    {
                                        .u8_encHdmiDvpPolarity  = 0,
                                        .u8_encMipiPolarity     = 4,
                                    },
                                    {
                                        .u8_encHdmiDvpPolarity  = 0,
                                        .u8_encMipiPolarity     = 4,
                                    }};
                                        
static STRU_VIEW_CFG g_stEncViewCfg[2] = { 0 };

static int view_cfg_refresh(uint8_t ch);


/*====================================================*/
/*              Encoder   Functions                   */
/*====================================================*/

void __attribute__ ((section(".h264"))) vsoc_enc_enable() 
{
    unsigned int rd_enc_reg = 0 ;
    //ENC View0 Enable 
    encoder_delay(2000);

    //
    READ_WORD(VSOC_ENC_REG_BASE + (0<<2), rd_enc_reg) ;

    rd_enc_reg = rd_enc_reg | 0x01000000 ;

    encoder_delay(2000) ;
   
    WRITE_WORD(VSOC_ENC_REG_BASE + (0<<2), rd_enc_reg) ;
}

const int bridx2br[19] = { // update brindex_2_br look-up table, lhu, 2017/06/06
    8000000 ,
    600000  ,
    1200000 ,
    2400000 ,
    3000000 ,
    3500000 ,
    4000000 ,
    4800000 ,
    5000000 ,
    6000000 ,
    7000000 ,
    7500000 ,
    9000000 ,
    10000000,
    11000000,
    12000000,
    13000000,
    14000000,
    15000000
};
void __attribute__ ((section(".h264"))) init_view0(unsigned int width, unsigned int height, unsigned int gop, unsigned int fps, unsigned int br, ENUM_ENCODER_INPUT_SRC src) 
{
    unsigned int value;
	unsigned int bu, height_in_mb, width_in_mb;
    
    if (ENCODER_INPUT_SRC_MIPI == src)
    {
        value = 0x00D00736 | ((((unsigned int)(g_stEncUsrCfg[0].u8_encMipiPolarity)) << 16) & 0x00070000);
        Reg_Write32_Mask((ENC_REG_ADDR+(0x00<<2)), value, (~(BIT(21))));
    }
    else
    {
        value = 0x00900736 | ((((unsigned int)(g_stEncUsrCfg[0].u8_encHdmiDvpPolarity)) << 16) & 0x00070000);
        Reg_Write32_Mask((ENC_REG_ADDR+(0x00<<2)), value, (~(BIT(21))));
    }
    
    WRITE_WORD((ENC_REG_ADDR+(0x01<<2)),((width<<16)+(height&0xffff)));
    Reg_Write32_Mask((ENC_REG_ADDR+(0x02<<2)), ((gop<<24)|(fps<<16)), 0xFFFF0000);

    if ((width == 720) && (height == 480))
    {
        WRITE_WORD((ENC_REG_ADDR+(0x03<<2)),0x003C000F);
        
        value = ((ENCODER_INPUT_SRC_MIPI == src)?(0xF01E4008):(0xF01E8008));
        Reg_Write32_Mask((ENC_REG_ADDR+(0x04<<2)), value, (~(BIT(13))));
    }
    else
    {
        WRITE_WORD((ENC_REG_ADDR+(0x03<<2)),0x00000000);
        
        value = ((ENCODER_INPUT_SRC_MIPI == src)?(0xF0004000):(0xF0008000)); 
        Reg_Write32_Mask((ENC_REG_ADDR+(0x04<<2)), value, (~(BIT(13))));
    }

    if(ENCODER_INPUT_SRC_MIPI == src)
    {
        value = BIT(13);
        Reg_Write32_Mask((ENC_REG_ADDR+(0x04<<2)), value, BIT(13));
    }

	height_in_mb = ((height + 15) >> 4);
	width_in_mb  = ((width  + 15) >> 4);
	if( height_in_mb % 2 == 0 )
		bu = 2;
	else if( height_in_mb % 3 == 0 )
		bu = 3;
	else if( height_in_mb % 5 == 0 )
		bu = 5;
	else 
		bu = 1;
	rca[0].row_cnt_in_bu 		= bu;
	rca[0].FrameHeightInMbs 	= height_in_mb;
	
	bu = bu * width_in_mb;	
    Reg_Write32_Mask((ENC_REG_ADDR+(0x05<<2)), bu, 0x0000FFFF); //bu

    //if( br == 8 || br == 1 || br == 2)  // <= 2Mbps 
    //    WRITE_WORD((ENC_REG_ADDR+(0x06<<2)),0x20330806);
    //else
        WRITE_WORD((ENC_REG_ADDR+(0x06<<2)),0x202a0806);

    WRITE_WORD((ENC_REG_ADDR+(0x07<<2)),bridx2br[br]);
    if (ENCODER_INPUT_SRC_MIPI == src)
    {
        WRITE_WORD((ENC_REG_ADDR+(0x0a<<2)),(0x0302589E | ((br & 0x1F)<<26) ));
    }
    else
    {
        WRITE_WORD((ENC_REG_ADDR+(0x0a<<2)),(0x030258FE | ((br & 0x1F)<<26) ));
    }

    if( height <= 720 ) {
        //DLOG_Info("v0 <= 720p\n");
        WRITE_WORD((ENC_REG_ADDR+(0x0d<<2)),0x00002E00);
        WRITE_WORD((ENC_REG_ADDR+(0x0e<<2)),0x18004600);
        WRITE_WORD((ENC_REG_ADDR+(0x0f<<2)),0x20004E00);
        WRITE_WORD((ENC_REG_ADDR+(0x10<<2)),0x22005000);
        WRITE_WORD((ENC_REG_ADDR+(0x11<<2)),0x28005600);
    }
    else {
        //DLOG_Info("v0 == 1080p\n");
        WRITE_WORD((ENC_REG_ADDR+(0x0d<<2)),0x00004700);
        WRITE_WORD((ENC_REG_ADDR+(0x0e<<2)),0x22006900);
        WRITE_WORD((ENC_REG_ADDR+(0x0f<<2)),0x32007900);
        WRITE_WORD((ENC_REG_ADDR+(0x10<<2)),0x36007d00);
        WRITE_WORD((ENC_REG_ADDR+(0x11<<2)),0x3e808580);
    }


    WRITE_WORD((ENC_REG_ADDR+(0x18<<2)),0x0005AA60); // HBitsRatioAbits_level=5, AbitsRatioTargetBits_level=170, psnr_drop_level=3db, insertOneIFrame mode disable, wireless_screen mode disable, lhu
	Reg_Write32_Mask(ENC_REG_ADDR + 0xDC, (unsigned int)(0x21 << 24), BIT(31)|BIT(30)|BIT(29)|BIT(28)|BIT(27)|BIT(26)|BIT(25)|BIT(24));

    view_cfg_refresh(0);
}

void __attribute__ ((section(".h264"))) init_view1(unsigned int width, unsigned int height, unsigned int gop, unsigned int fps, unsigned int br, ENUM_ENCODER_INPUT_SRC src) 
{
    unsigned int value;
	unsigned int bu, height_in_mb, width_in_mb;

    if (ENCODER_INPUT_SRC_MIPI == src)
    {
        value = 0x00D00736 | ((((unsigned int)(g_stEncUsrCfg[1].u8_encMipiPolarity)) << 16) & 0x00070000);
        Reg_Write32_Mask((ENC_REG_ADDR+(0x19<<2)), value, (~(BIT(21))));
    }
    else
    {
        value = 0x00900736 | ((((unsigned int)(g_stEncUsrCfg[1].u8_encHdmiDvpPolarity)) << 16) & 0x00070000);
        Reg_Write32_Mask((ENC_REG_ADDR+(0x19<<2)), value, (~(BIT(21))));
    }

    WRITE_WORD((ENC_REG_ADDR+(0x1a<<2)),((width<<16)+(height&0xffff)));
    Reg_Write32_Mask((ENC_REG_ADDR+(0x1b<<2)), ((gop<<24)|(fps<<16)), 0xFFFF0000);

    if ((width == 720) && (height == 480))
    {
        WRITE_WORD((ENC_REG_ADDR+(0x1c<<2)),0x003C000F);
        
        value = ((ENCODER_INPUT_SRC_MIPI == src)?(0xF01E4008):(0xF01E8008));
        Reg_Write32_Mask((ENC_REG_ADDR+(0x1d<<2)), value, (~(BIT(13))));
    }
    else
    {
        WRITE_WORD((ENC_REG_ADDR+(0x1c<<2)),0x00000000);
        
        value = ((ENCODER_INPUT_SRC_MIPI == src)?(0xF0004000):(0xF0008000));
        Reg_Write32_Mask((ENC_REG_ADDR+(0x1d<<2)), value, (~(BIT(13))));
    }

    if(ENCODER_INPUT_SRC_MIPI == src)
    {
        value = BIT(13);
        Reg_Write32_Mask((ENC_REG_ADDR+(0x1d<<2)), value, BIT(13));
    }

	height_in_mb = ((height + 15) >> 4);
	width_in_mb  = ((width  + 15) >> 4);
	if( height_in_mb % 2 == 0 )
		bu = 2;
	else if( height_in_mb % 3 == 0 )
		bu = 3;
	else if( height_in_mb % 5 == 0 )
		bu = 5;
	else 
		bu = 1;
	rca[1].row_cnt_in_bu 	= bu;
	rca[1].FrameHeightInMbs = height_in_mb;
	bu = bu * width_in_mb;	
    Reg_Write32_Mask((ENC_REG_ADDR+(0x1e<<2)), bu, 0x0000FFFF); //bu

    //if( br == 8 || br == 1 || br == 2)  // <= 2Mbps 
    //    WRITE_WORD((ENC_REG_ADDR+(0x1f<<2)),0x20330806);
    //else
        WRITE_WORD((ENC_REG_ADDR+(0x1f<<2)),0x202a0806);

    WRITE_WORD((ENC_REG_ADDR+(0x20<<2)),bridx2br[br]);
    if (ENCODER_INPUT_SRC_MIPI == src)
    {
        WRITE_WORD((ENC_REG_ADDR+(0x23<<2)),(0x0302589E | ((br & 0x1F)<<26) ));
    }
    else
    {
        WRITE_WORD((ENC_REG_ADDR+(0x23<<2)),(0x030258FE | ((br & 0x1F)<<26) ));
    }

    if( height <= 720 ) {
        DLOG_Info("v1 <= 720p\n");

        WRITE_WORD((ENC_REG_ADDR+(0x26<<2)),0x5C008A00);
        WRITE_WORD((ENC_REG_ADDR+(0x27<<2)),0x7400A200);
        WRITE_WORD((ENC_REG_ADDR+(0x28<<2)),0x7C00AA00);
        WRITE_WORD((ENC_REG_ADDR+(0x29<<2)),0x7E00AC00);
        WRITE_WORD((ENC_REG_ADDR+(0x2a<<2)),0x8400B200);
    }
    else {
        DLOG_Info("v1 == 1080p\n");

        WRITE_WORD((ENC_REG_ADDR+(0x26<<2)),0x00004700);
        WRITE_WORD((ENC_REG_ADDR+(0x27<<2)),0x22006900);
        WRITE_WORD((ENC_REG_ADDR+(0x28<<2)),0x32007900);
        WRITE_WORD((ENC_REG_ADDR+(0x29<<2)),0x36007d00);
        WRITE_WORD((ENC_REG_ADDR+(0x2a<<2)),0x3e808580);
    }
    
    WRITE_WORD((ENC_REG_ADDR+(0x31<<2)),0x0005AA60); 


    // Patch: view0 settings must be same as view1 when 1080P input. 
    if (height > 720)
    {
        init_view0(width, height, gop, fps, br, src);
    }

    view_cfg_refresh(1);
}
void __attribute__ ((section(".h264"))) open_view0( unsigned int rc_en ) 
{ 
    // hold until SDRAM initial done
    unsigned int i,t0,t1;
    while(1){
        for(i=0;i<2000;i++);
        READ_WORD(0xa0030024,t1);
        if((t1&0x1)==1){
            READ_WORD(0xa0010000,t0);
            t0=t0|0x01000000;
            WRITE_WORD(0xa0010000,t0);
            break;
        }
    }

    if( rc_en ) {
        READ_WORD((ENC_REG_ADDR+(0x05<<2)),t0);
        t0=t0|0x01000000;
        WRITE_WORD((ENC_REG_ADDR+(0x05<<2)),t0);
    }

}

void __attribute__ ((section(".h264"))) close_view0(void) 
{
    unsigned int t0;

    READ_WORD(0xa0010000,t0);
    t0 &= ~0x01000000;
    WRITE_WORD(0xa0010000,t0);

    READ_WORD((ENC_REG_ADDR+(0x05<<2)),t0);
    t0 &= ~0x01000000;
    WRITE_WORD((ENC_REG_ADDR+(0x05<<2)),t0);


	READ_WORD((ENC_REG_ADDR+(0x33<<2)), t0);
	t0 &= ~(0xF<<4);
    WRITE_WORD((ENC_REG_ADDR+(0x33<<2)),t0);// clear watch dog
}

void __attribute__ ((section(".h264"))) open_view1( unsigned int rc_en ) 
{ 
    // hold until SDRAM initial done
    unsigned int i,t0,t1;
    while(1){
        for(i=0;i<2000;i++);
        READ_WORD(0xa0030024,t1);
        if((t1&0x1)==1){
            READ_WORD(0xa0010064,t0);
            t0=t0|0x01000000;
            WRITE_WORD(0xa0010064,t0);
            break;
        }
    }

    if( rc_en ) {
        READ_WORD((ENC_REG_ADDR+(0x1e<<2)),t0);
        t0=t0|0x01000000;
        WRITE_WORD((ENC_REG_ADDR+(0x1e<<2)),t0);
    }
}

void __attribute__ ((section(".h264"))) close_view1(void) 
{
    unsigned int t0;

    READ_WORD(0xa0010064,t0);
    t0 &= ~0x01000000;
    WRITE_WORD(0xa0010064,t0);

    READ_WORD((ENC_REG_ADDR+(0x1e<<2)),t0);
    t0 &= ~0x01000000;
    WRITE_WORD((ENC_REG_ADDR+(0x1e<<2)),t0);

	READ_WORD((ENC_REG_ADDR+(0x33<<2)), t0);
	t0 &= ~(0xF<<4);
    WRITE_WORD((ENC_REG_ADDR+(0x33<<2)),t0);// clear watch dog

}

int __attribute__ ((section(".h264"))) usr_cfg(uint8_t ch, STRU_ENC_CUSTOMER_CFG *pst_h264UsrCfg) 
{
    if (ch > 1)
    {
        return 0;
    }
    else
    {
        memcpy((uint8_t *)(&g_stEncUsrCfg[ch]), (uint8_t *)pst_h264UsrCfg, sizeof(STRU_ENC_CUSTOMER_CFG));
    }

    return 1;
}

int __attribute__ ((section(".h264"))) view_cfg(uint8_t ch, STRU_REG *pst_viewCfg, uint32_t num) 
{
    if (ch > 1)
    {
        return 0;
    }
    else
    {
        g_stEncViewCfg[ch].pst_reg = pst_viewCfg;
        g_stEncViewCfg[ch].u32_num = num;
    }

    return 1;
}

static int __attribute__ ((section(".h264"))) view_cfg_refresh(uint8_t ch) 
{
    uint32_t i;
    STRU_REG * pst_tmp;

    if (ch > 1)
    {
        return 0;
    }
    else
    {
        pst_tmp = g_stEncViewCfg[ch].pst_reg;
        for(i=0; i<(g_stEncViewCfg[ch].u32_num); i++)
        {
            Reg_Write32_Mask(pst_tmp->u32_regAddr, pst_tmp->u32_regValue, pst_tmp->u32_regdataMask);
            pst_tmp += 1;
        }
    }

    return 1;
}



