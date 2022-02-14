#include <stddef.h>
#include <stdint.h>
#include "debuglog.h"
#include "systicks.h"
#include "data_type.h"
#include "enc_internal.h"
#include "brc.h"
#include "h264_encoder.h"
#include "interrupt.h"
#include "ar8020.h"
#include "sys_event.h"
#include "reg_rw.h"
#include "bb_types.h"
#include "rtc.h"
#include "memory_config.h"
#include "cfg_parser.h"
#include "sram_sky.h"

extern RC_DATA rca[];
#define H264_ENCODER_BUFFER_HIGH_LEVEL    (1<<19)
#define H264_ENCODER_BUFFER_LOW_LEVEL     (1<<17)

static STRU_EncoderStatus g_stEncoderStatus[2] = { 0 };
extern RC_SETTING rc_setting[];



static int H264_Encoder_UpdateGop(unsigned char view, unsigned char gop);

static int H264_getCfgData(STRU_cfgBin *cfg, uint8_t encbit);

static int __attribute__ ((section(".h264"))) H264_Encoder_StartView(unsigned char view, unsigned int resW, unsigned int resH, unsigned int gop, unsigned int framerate, unsigned int bitrate, ENUM_ENCODER_INPUT_SRC src) 
{
    if (view >= 2)
    {
        return 0;
    }

    NVIC_DisableIRQ(VIDEO_ARMCM7_IRQ_VECTOR_NUM);

#ifdef ARCAST
	Reg_Write32(SRAM_SKY_MASTER_ID_ADDR, SRAM_SKY_MASTER_ID_VALUE);
	Reg_Write32(SRAM_SKY_MASTER_ID_MASK_ADDR, SRAM_SKY_MASTER_ID_MASK_VALUE);
#endif

    if (view == 0)
    {
        init_view0(resW, resH, gop, framerate, bitrate, src);
        open_view0(g_stEncoderStatus[view].brc_enable);
    }
    else if (view == 1)
    {
        init_view1(resW, resH, gop, framerate, bitrate, src);
        open_view1(g_stEncoderStatus[view].brc_enable);
    }

    g_stEncoderStatus[view].resW = resW;
    g_stEncoderStatus[view].resH = resH;
    g_stEncoderStatus[view].framerate = framerate;
    g_stEncoderStatus[view].running = 1;
    g_stEncoderStatus[view].src = src;

    H264_Encoder_UpdateBitrate(view, bitrate);
    H264_Encoder_UpdateGop(view, framerate * 2);

	my_initial_all( view );
        
    if ((g_stEncoderStatus[0].brc_enable && g_stEncoderStatus[0].running) || 
        (g_stEncoderStatus[1].brc_enable && g_stEncoderStatus[1].running)) // view0 and view1 share the same BRC interrupt
    {
        NVIC_EnableIRQ(VIDEO_ARMCM7_IRQ_VECTOR_NUM);
    }

    return 1;
}

static int __attribute__ ((section(".h264"))) H264_Encoder_RestartView(unsigned char view, unsigned int resW, unsigned int resH, unsigned int gop, unsigned int framerate, unsigned int bitrate, ENUM_ENCODER_INPUT_SRC src) 
{
    if (view >= 2)
    {
        return 0;
    }
    
    NVIC_DisableIRQ(VIDEO_ARMCM7_IRQ_VECTOR_NUM);

    if (view == 0)
    {
        close_view0();
        H264_Encoder_StartView(view, resW, resH, gop, framerate, bitrate, src);
    }
    else if (view == 1)
    {
        close_view1();
        H264_Encoder_StartView(view, resW, resH, gop, framerate, bitrate, src);
    }

    return 1;
}

static int __attribute__ ((section(".h264"))) H264_Encoder_CloseView(unsigned char view) 
{
    if (view >= 2)
    {
        return 0;
    }

    NVIC_DisableIRQ(VIDEO_ARMCM7_IRQ_VECTOR_NUM);
    
    if (view == 0)
    {
        close_view0();
    }
    else if (view == 1)
    {
        close_view1();
    }

    g_stEncoderStatus[view].resW = 0;
    g_stEncoderStatus[view].resH = 0;
    g_stEncoderStatus[view].framerate = 0;
    g_stEncoderStatus[view].running = 0;

    if ((g_stEncoderStatus[0].brc_enable && g_stEncoderStatus[0].running) || 
        (g_stEncoderStatus[1].brc_enable && g_stEncoderStatus[1].running)) // view0 and view1 share the same BRC interrupt
    {
        NVIC_EnableIRQ(VIDEO_ARMCM7_IRQ_VECTOR_NUM);
    }

    return 1;
}

static int __attribute__ ((section(".h264"))) H264_Encoder_UpdateVideoInfo(unsigned char view, unsigned int resW, unsigned int resH, unsigned int framerate, ENUM_ENCODER_INPUT_SRC src) 
{
    unsigned int u32_data; 
    unsigned int tmp_resW;
    unsigned int tmp_resH;
    STRU_WIRELESS_INFO_DISPLAY *osdptr = (STRU_WIRELESS_INFO_DISPLAY *)(SRAM_BB_STATUS_SHARE_MEMORY_ST_ADDR);

    if (view >= 2)
    {
        return 0;
    }
    
    if(g_stEncoderStatus[view].resW != resW ||
       g_stEncoderStatus[view].resH != resH ||
       g_stEncoderStatus[view].framerate != framerate || 
       g_stEncoderStatus[view].src != src)
    {
        if ((resW == 0) || (resH == 0) || (framerate == 0) || (src == 0))
        {
            H264_Encoder_CloseView(view);

			STRU_SysEvent_H264RptParameter s_RptParam;
			if( view == 0 ) {
				s_RptParam.v0_width = 0;
				s_RptParam.v0_hight = 0;
				s_RptParam.v1_width = rca[1].ret_width;
				s_RptParam.v1_hight = rca[1].ret_hight;

			}
			else {
				s_RptParam.v0_width = rca[0].ret_width;
				s_RptParam.v0_hight = rca[0].ret_hight;
				s_RptParam.v1_width = 0;
				s_RptParam.v1_hight = 0;
			}
			SYS_EVENT_NotifyInterCore(SYS_EVENT_ID_H264_RPT_EVENT, (&s_RptParam) );	
        }
        else
        {            
            if (10 > framerate)
            {
                H264_Encoder_CloseView(view);
                Reg_Write32_Mask(0xA0010000+(0x04<<2), (uint32_t)(0 << 28), (uint32_t)(0xf << 28));
                Reg_Write32_Mask(0xA0010000+(0x1D<<2), (uint32_t)(0 << 28), (uint32_t)(0xf << 28));
            }
            if (g_stEncoderStatus[view].over_flow == 0)
            {
            H264_Encoder_RestartView(view, resW, resH, g_stEncoderStatus[view].gop, 
                                     framerate, g_stEncoderStatus[view].bitrate, src);
			}
            else
            {
                g_stEncoderStatus[view].resW = resW;
                g_stEncoderStatus[view].resH = resH;
                g_stEncoderStatus[view].framerate = framerate;
                g_stEncoderStatus[view].src = src;
            }
        }
        
        //DLOG_Critical("Video format change: %d, %d, %d, %d\n", view, resW, resH, framerate);
    }

    osdptr->video_width[view] = resW;
    osdptr->video_height[view] = resH;
    osdptr->frameRate[view] = framerate;
    
    READ_WORD((ENC_REG_ADDR+(0x01<<2)), u32_data);
    tmp_resW = (u32_data >> 16) & 0xFFFF;
    tmp_resH = (u32_data >> 0) & 0xFFFF;
    if ((tmp_resW == (g_stEncoderStatus[0].resW)) && (tmp_resH == (g_stEncoderStatus[0].resH)))
    {
        osdptr->encoder_status |= 0x01;
    }
    else
    {
        osdptr->encoder_status &= ~0x01;
    }
    
    READ_WORD((ENC_REG_ADDR+(0x1a<<2)), u32_data);
    tmp_resW = (u32_data >> 16) & 0xFFFF;
    tmp_resH = (u32_data >> 0) & 0xFFFF;
    if ((tmp_resW == (g_stEncoderStatus[1].resW)) && (tmp_resH == (g_stEncoderStatus[1].resH)))
    {
        osdptr->encoder_status |= 0x02;
    }
    else
    {
        osdptr->encoder_status &= ~0x02;
    }

    return 1;
}

uint32_t __attribute__ ((section(".h264"))) H264_Encoder_GetBufferLevel(unsigned char view) 
{
    uint32_t buf_level = 0;

    Reg_Write32_Mask(ENC_REG_ADDR + 0xDC, (unsigned int)(0x21 << 24), BIT(31)|BIT(30)|BIT(29)|BIT(28)|BIT(27)|BIT(26)|BIT(25)|BIT(24));
    
    //read buffer counter

    if (view == 0)
    {
        Reg_Write32_Mask(ENC_REG_ADDR + 0xD8, (unsigned int)(0x04 <<  8), BIT(11)|BIT(10)|BIT(9)|BIT(8));       // Switch to vdb debug register
    }
    else
    {
        Reg_Write32_Mask(ENC_REG_ADDR + 0xD8, (unsigned int)(0x05 <<  8), BIT(11)|BIT(10)|BIT(9)|BIT(8));       // Switch to vdb debug register
    }
    
    buf_level = Reg_Read32(ENC_REG_ADDR + 0xF8);
    Reg_Write32_Mask(ENC_REG_ADDR + 0xDC, (unsigned int)(0x00 << 24), BIT(31)|BIT(30)|BIT(29)|BIT(28)|BIT(27)|BIT(26)|BIT(25)|BIT(24));


    return buf_level;
}

static void __attribute__ ((section(".h264"))) H264_Encoder_IdleCallback(void* p) 
{
    uint32_t buf_level;
    if ((g_stEncoderStatus[0].over_flow == 1) && (g_stEncoderStatus[0].running == 0))
    {
        buf_level = H264_Encoder_GetBufferLevel(0);
		uint32_t target_level = ((rca[0].bit_rate*4) >> 8);
        if ( buf_level <= target_level )
        {
            Reg_Write32_Mask( 0xa003008c, 0x0, 0x01);
            H264_Encoder_StartView(0, g_stEncoderStatus[0].resW, g_stEncoderStatus[0].resH, 
                                      g_stEncoderStatus[0].gop, g_stEncoderStatus[0].framerate, 
                                      g_stEncoderStatus[0].bitrate,
                                      g_stEncoderStatus[0].src
                                      );

            g_stEncoderStatus[0].over_flow = 0;
            
            //DLOG_Error("Buffer level %d, open view 0", buf_level);

        }
    }
    else if ((g_stEncoderStatus[1].over_flow == 1) && (g_stEncoderStatus[1].running == 0))
    {
        buf_level = H264_Encoder_GetBufferLevel(1);
		uint32_t target_level = ((rca[1].bit_rate*4) >> 8);
        if (buf_level <= target_level)
        {
            Reg_Write32_Mask(0xa003004c, 0x0, 0x04);
            H264_Encoder_StartView(1, g_stEncoderStatus[1].resW, g_stEncoderStatus[1].resH, 
                                      g_stEncoderStatus[1].gop,  g_stEncoderStatus[1].framerate, 
                                      g_stEncoderStatus[1].bitrate,
                                      g_stEncoderStatus[1].src
                                      );

            g_stEncoderStatus[1].over_flow = 0;
            
            //DLOG_Error("Buffer level %d, open view 1", buf_level);
        }        
    }
}

uint8_t flip_flag;
static void __attribute__ ((section(".h264"))) H264_Encoder_InputVideoFormatChangeCallback(void* p) 
{
    if((((STRU_SysEvent_H264InputFormatChangeParameter*)p)->reserve)[0] == 0x5A && \
       (((STRU_SysEvent_H264InputFormatChangeParameter*)p)->reserve)[1] == 0xA5)
    {
        ++flip_flag;
        flip_flag = flip_flag % 2;
    }
       
    DLOG_Critical("flip_flag :%d %x", flip_flag, (((STRU_SysEvent_H264InputFormatChangeParameter*)p)->reserve)[0]);

    if(0 == flip_flag)
    {
        uint8_t index  = ((STRU_SysEvent_H264InputFormatChangeParameter*)p)->index;
        uint16_t width = ((STRU_SysEvent_H264InputFormatChangeParameter*)p)->width;
        uint16_t hight = ((STRU_SysEvent_H264InputFormatChangeParameter*)p)->hight;
        uint8_t framerate = ((STRU_SysEvent_H264InputFormatChangeParameter*)p)->framerate;
        ENUM_ENCODER_INPUT_SRC src = ((STRU_SysEvent_H264InputFormatChangeParameter*)p)->e_h264InputSrc;

        // ADV7611 0,1 is connected to H264 encoder 1,0
        H264_Encoder_UpdateVideoInfo(index, width, hight, framerate, src);
    }
}

static int __attribute__ ((section(".h264"))) H264_Encoder_UpdateGop(unsigned char view, unsigned char gop) 
{
    uint32_t addr;

    if(view == 0 )
    {
        addr = ENC_REG_ADDR + (0x02 << 2);
    }
    else
    {
        addr = ENC_REG_ADDR + (0x1b << 2);
    }

    Reg_Write32_Mask(addr, (unsigned int)(gop << 24), BIT(31)|BIT(30)|BIT(29)|BIT(28)|BIT(27)|BIT(26)|BIT(25)|BIT(24));
}


int __attribute__ ((section(".h264"))) H264_Encoder_UpdateIpRatioAndIpRatioMax(unsigned char view, unsigned char ipratio, unsigned char ipratiomax) 
{
    uint32_t addr;

    if(view == 0 )
    {
        addr = ENC_REG_ADDR + (0x08 << 2);
    }
    else
    {
        addr = ENC_REG_ADDR + (0x21 << 2);
    }
    
    Reg_Write32_Mask(addr, (unsigned int)(ipratio << 24), BIT(27)|BIT(26)|BIT(25)|BIT(24));
    Reg_Write32_Mask(addr, (unsigned int)(ipratiomax << 8), BIT(15)|BIT(14)|BIT(13)|BIT(12)|BIT(11)|BIT(10)|BIT(9)|BIT(8));
    rca[view].RCISliceBitRatioMax = ipratiomax;
    rca[view].RCISliceBitRatio    = ipratio;
}

static int __attribute__ ((section(".h264"))) H264_Encoder_UpdateMinAndMaxQP(unsigned char view, unsigned char br) 
{
    unsigned int addr, addr_arcast, minqp, maxqp;
    if(view==0) {
    	addr = ENC_REG_ADDR+(0x06<<2);
    	addr_arcast = ENC_REG_ADDR+(0x18<<2);
    } else {
        addr = ENC_REG_ADDR+(0x1F<<2);
    	addr_arcast = ENC_REG_ADDR+(0x31<<2);
    }

	minqp = rc_setting[br].MinQP;
	
    Reg_Write32_Mask(addr, (unsigned int)(minqp<<8), BIT(15)|BIT(14)|BIT(13)|BIT(12)|BIT(11)|BIT(10)|BIT(9)|BIT(8)); // set minqp, lhu
    // set maxqp in arcast mode (maxqp-minqp:0x13) and non-arcast mode (maxqp-minqp:0x19) when bit-rate changed, lhu, 2017/05/05
    {
    	//maxqp = minqp + ((((Reg_Read32(addr_arcast))>>2)&0x01)? 0x13:0x19);
    	maxqp = rc_setting[br].MaxQP;
        Reg_Write32_Mask(addr, (unsigned int)(maxqp<<16), BIT(23)|BIT(22)|BIT(21)|BIT(20)|BIT(19)|BIT(18)|BIT(17)|BIT(16));
    }
}

unsigned char IPRatio[2][14][2] = 
{
	{	// <40fps, 3frame *1/fps, 6frame * 1/fps
		{ 4,  7	}, // 8Mbps
    	{10, 20	}, // 600kbps
    	{ 5,  8 }, // 1.2Mbps
    	{ 5,  8	}, // 2.4Mbps
    	{ 4,  7	}, // 3Mbps
    	{ 3,  6	}, // 3.5Mbps
    	{ 3,  5	}, // 4Mbps
    	{ 2,  4	}, // 4.8Mbps
    	{ 4,  8	}, // 5Mbps
    	{ 4,  7	}, // 6Mbps
    	{ 3,  6 }, // 7Mbps
    	{ 4,  8	}, // 7.5Mbps
    	{ 5,  9	}, // 9Mbps
    	{ 4,  8	}  // 10Mbps
	},
	//  Buf 2Frame for realtime, Buf 4Frames for fluent mode
	{	// >40fps, 3frame * 1/fps, 6 frame * 1/fps
		{ 4,  11 }, //{ 9, 17	}, // 8Mbps
	/**/{10,  24},	//{24, 59	}, // 600kbps
	/**/{ 5,  16},	//{10, 20	}, // 1.2Mbps
	/**/{ 5,  16},	//{10, 20	}, // 2.4Mbps
		{ 4,  12 }, //{ 8, 15	}, // 3Mbps
		{ 3,  12 }, //{ 6, 12	}, // 3.5Mbps
		{ 3,  12 }, //{ 6, 10	}, // 4Mbps
		{ 2,  12 }, //{ 5,	8	}, // 4.8Mbps
	/**/{ 4,  12 }, //{ 9, 19	}, // 5Mbps
		{ 4,  12 }, //{ 8, 15	}, // 6Mbps
		{ 3,  12 }, //{ 6, 12	}, // 7Mbps
	/**/{ 4,  12 }, //{ 9, 19	}, // 7.5Mbps
		{ 5,  12 }, //{11, 22	}, // 9Mbps
	/**/{ 4,  12 }	//{ 9, 19	}  // 10Mbps
	}

};

int __attribute__ ((section(".h264"))) H264_Encoder_UpdateBitrate(unsigned char view, unsigned char br_idx) 
{
    uint8_t ratio = 0, ratiomax = 0;
    unsigned int frame_rate;

    if (view >= 2)
    {
        return 0;
    }

    frame_rate = (view == 0) ? g_stEncoderStatus[0].framerate: g_stEncoderStatus[1].framerate;

	//if( frame_rate >= 40 ) 
	{
		ratio 		= IPRatio[1][br_idx][0];
		ratiomax	= IPRatio[1][br_idx][1];
	}
	//else {
	//	ratio		= IPRatio[0][br_idx][0];
	//	ratiomax	= IPRatio[0][br_idx][1];
	//}

    //DLOG_Info("%d %d %d %d %d %d\n", g_stEncoderStatus[0].running, g_stEncoderStatus[1].running, 
   //                                  g_stEncoderStatus[0].brc_enable, g_stEncoderStatus[1].brc_enable, 
   //                                  g_stEncoderStatus[0].bitrate, g_stEncoderStatus[1].bitrate);
    if (g_stEncoderStatus[view].running && g_stEncoderStatus[view].brc_enable /*&& (g_stEncoderStatus[view].bitrate != br_idx)*/)
    {
        if (view == 0)
        {
            Reg_Write32_Mask(ENC_REG_ADDR+(0xA<<2), (unsigned int)(br_idx << 26), BIT(26) | BIT(27) | BIT(28) | BIT(29) | BIT(30));
            //H264_Encoder_UpdateGop(0, g_stEncoderStatus[0].framerate); // comment out by lhu, 2017/04/21
            H264_Encoder_UpdateIpRatioAndIpRatioMax(0, ratio, ratiomax);
            H264_Encoder_UpdateMinAndMaxQP(0, br_idx);
        }
        else
        {
            Reg_Write32_Mask(ENC_REG_ADDR+(0x23<<2), (unsigned int)(br_idx << 26), BIT(26) | BIT(27) | BIT(28) | BIT(29) | BIT(30));
            //H264_Encoder_UpdateGop(1, g_stEncoderStatus[1].framerate); // comment out by lhu, 2017/04/21
            H264_Encoder_UpdateIpRatioAndIpRatioMax(1, ratio, ratiomax);
            H264_Encoder_UpdateMinAndMaxQP(1, br_idx);
        }

    //    DLOG_Info("Encoder bitrate change: %d, %d, %d, %d\n", view, br_idx, ratio, ratiomax);
    }
    
    g_stEncoderStatus[view].bitrate = br_idx;
    return 1;
}

static void __attribute__ ((section(".h264"))) H264_Encoder_BBModulationChangeCallback(void* p) 
{
    uint8_t br_idx = ((STRU_SysEvent_BB_ModulationChange *)p)->encoder_brcidx;
    uint8_t ch = ((STRU_SysEvent_BB_ModulationChange *)p)->u8_bbCh;
    
    if (0 == ch)
    {
        H264_Encoder_UpdateBitrate(0, br_idx);
        DLOG_Warning("H264 bitidx ch1: %d \r\n", br_idx);
    }
    else if (1 == ch)
    {
        H264_Encoder_UpdateBitrate(1, br_idx);
        DLOG_Warning("H264 bitidx ch2: %d \r\n", br_idx);
    }
    else
    {
    }
}

static void __attribute__ ((section(".h264"))) H264_Encoder_CmdProcess(void* p) 
{
    uint8_t *par = (uint8_t *)p;

    switch(par[0])
    {
        case 0: // take picture
        {
            //DLOG_Warning("take %d", par[1]);
			rca[0].photography_enable = par[1];
			rca[1].photography_enable = par[1];
            break;
        }
        case 1: // picture quality set
        {
            //DLOG_Error("%d", par[1]);
            rca[0].realtime_mode = par[1];
            rca[1].realtime_mode = par[1];
            break;
        }
        default:
        {
            break;
        }
    }
}


static void __attribute__ ((section(".h264"))) VEBRC_IRQ_Wrap_Handler(uint32_t u32_vectorNum) 
{
    uint32_t v0_last_row;
	uint32_t v1_last_row;
    uint32_t view0_feedback = Reg_Read32(ENC_REG_ADDR+(0x09<<2));
    uint32_t view1_feedback = Reg_Read32(ENC_REG_ADDR+(0x22<<2));
    STRU_WIRELESS_INFO_DISPLAY *osdptr = (STRU_WIRELESS_INFO_DISPLAY *)(SRAM_BB_STATUS_SHARE_MEMORY_ST_ADDR);

    if(g_stEncoderStatus[0].running == 1)  // View0 is opened
    {
        // check if this is the last row
        v0_last_row = ((view0_feedback >> 8) & 0x01);

        if(v0_last_row)
        {
            //DLOG_Info("%d,%d\n", rca.v0_frame_cnt, rca.v0_intra_period );
            uint32_t buf_level = H264_Encoder_GetBufferLevel(0);
			uint32_t target_level = ((rca[0].bit_rate) >> 3);
			target_level = (target_level > H264_ENCODER_BUFFER_HIGH_LEVEL)? H264_ENCODER_BUFFER_HIGH_LEVEL :
						   (target_level < H264_ENCODER_BUFFER_LOW_LEVEL )? H264_ENCODER_BUFFER_LOW_LEVEL  : target_level;
            //if(buf_level >= H264_ENCODER_BUFFER_HIGH_LEVEL)
            if(buf_level >= target_level)
            {
                //Close Encoder
                Reg_Write32_Mask( (unsigned int) 0xa003008c, 0x01, 0x01);
                g_stEncoderStatus[0].over_flow = 1;
                close_view0();
                g_stEncoderStatus[0].running = 0;
				//DLOG_Error("Buffer level %d, close view 0.", buf_level);
            }
			DLOG_Info("V0 BLvl %d", buf_level);


			//	STRU_SysEvent_H264RptParameter s_RptParam;
			//	s_RptParam.v0_width = rca[0].ret_width;
			//	s_RptParam.v0_hight = rca[0].ret_hight;
			//	s_RptParam.v1_width = rca[1].ret_width;
			//	s_RptParam.v1_hight = rca[1].ret_width;
			//	SYS_EVENT_NotifyInterCore(SYS_EVENT_ID_H264_RPT_EVENT, (&s_RptParam) );	
			rca[0].enc_frame_cnt ++;
			if( ( rca[0].enc_frame_cnt % 10 ) == 0 )
			{
				osdptr->enc_running_cnt ++;
			}
        }
    }

    if(g_stEncoderStatus[1].running == 1)  // View1 is opened
    {
        // check if this is the last row
        v1_last_row = ((view1_feedback >> 8) & 0x01);

        if(v1_last_row)
        {
            //DLOG_Info("%d %d\n", rca.v1_frame_cnt,rca.v1_intra_period );
            uint32_t buf_level = H264_Encoder_GetBufferLevel(1);
			uint32_t target_level = ((rca[1].bit_rate) >> 3);
			target_level = (target_level > H264_ENCODER_BUFFER_HIGH_LEVEL)? H264_ENCODER_BUFFER_HIGH_LEVEL :
						   (target_level < H264_ENCODER_BUFFER_LOW_LEVEL )? H264_ENCODER_BUFFER_LOW_LEVEL  : target_level;
            //if(buf_level >= H264_ENCODER_BUFFER_HIGH_LEVEL)
			if(buf_level >= target_level)
            {
                //Close Encoder
                Reg_Write32_Mask( (unsigned int) 0xa003004c, 0x04, 0x04);

                g_stEncoderStatus[1].over_flow = 1;
                close_view1();
                g_stEncoderStatus[1].running = 0;
                DLOG_Error("Buffer level %d, close view 1.", buf_level);
            }
			//DLOG_Info("V1 BLvl %d", buf_level);

#ifdef ARCAST
            // Insert timestamp for audio sync
            if( ( (view1_feedback >> 1) & 0x01) == 1)   // Check if it is one I frame.
            {
                // Switch to DVP1 input debug register
                //  WRITE_WORD( (ENC_REG_ADDR+ (0x37<<2)), 0x12000000);
                //  unsigned int lb_freeblk = Reg_Read32( (unsigned int)(ENC_REG_ADDR + 0xE4) );

                WRITE_WORD( (ENC_REG_ADDR+ (0x37<<2)), 0x0a000000);
                unsigned int freespace = Reg_Read32( (unsigned int)(ENC_REG_ADDR + 0xFC) );

                //dlog_error("xxx%08x %08x\n", freespace, rca[1].dvp_lb_freesize);
                if( (freespace & 0xFFFF) == rca[1].dvp_lb_freesize ) 
                {
                    // insert timestamp data, not only consider view1.
                    // close encoder channel
                    volatile uint32_t tick;
                    uint32_t tmp;
                    uint8_t  sum = 0;

                    Reg_Write32_Mask( (unsigned int) 0xa003004c, 0x04, 0x04);
                    
                    tick =  *((volatile uint32_t *)(SRAM_MODULE_SHARE_AVSYNC_TICK));
                    //head: 0x35 + 0x53 + 0x55 + sum
                    sum += (0x35+0x53+0x55+((tick>>24)&0xff) + ((tick>>16) & 0xff) + ((tick>>8)& 0xff) + (tick& 0xff));
                    tmp = (sum << 24) + (0x55 << 16) + (0x53 <<8) + 0x35;

                    Reg_Write32( (unsigned int) 0xb1800000, 0x7f010000);
                    Reg_Write32( (unsigned int) 0xb1800000, tmp);
                    tmp = ( ((tick & 0xff)<<24) + ((tick & 0xff00)<<8) + ((tick & 0xff0000)>>8) + ((tick & 0xff000000)<<24));
                    Reg_Write32( (unsigned int) 0xb1800000, tmp);

                    //
                    Reg_Write32_Mask( (unsigned int) 0xa003004c, 0x00, 0x04); // back to encoder channel
                    //dlog_error("TS tick = 0x%08x 0x%02x\n", tick, sum);
                }

                WRITE_WORD( (ENC_REG_ADDR+ (0x37<<2)), 0x00000000);
           }
#endif
		
			//	STRU_SysEvent_H264RptParameter s_RptParam;
			//	s_RptParam.v0_width = rca[0].ret_width;
			//	s_RptParam.v0_hight = rca[0].ret_hight;
			//	s_RptParam.v1_width = rca[1].ret_width;
			//	s_RptParam.v1_hight = rca[1].ret_width;
			//	SYS_EVENT_NotifyInterCore(SYS_EVENT_ID_H264_RPT_EVENT, (&s_RptParam) );	
			rca[1].enc_frame_cnt ++;
			if( ( rca[1].enc_frame_cnt % 10 ) == 0 )
			{
				osdptr->enc_running_cnt ++;
			}

        }
    }

    if( view0_feedback & BIT(6) )
	//if( 0 )
    {
        // Encoder hang happens, then do reset.
        Reg_Write32(VSOC_SOFT_RESET, (~(BIT(3)))); // can go back
        //Reg_Write32(VSOC_SOFT_RESET, 0xFF);

        if (g_stEncoderStatus[0].running == 1)
        {
            //DLOG_Critical("Enc Hang, Reset view0");
            H264_Encoder_RestartView(0, g_stEncoderStatus[0].resW, g_stEncoderStatus[0].resH, 
                                     g_stEncoderStatus[0].gop, g_stEncoderStatus[0].framerate, 
                                     g_stEncoderStatus[0].bitrate, g_stEncoderStatus[0].src);
        }

        if (g_stEncoderStatus[1].running == 1)
        {
            DLOG_Critical("Enc Hang, Reset view1");
			#ifdef ARCAST 
			uint8_t channel = 1;
			SYS_EVENT_NotifyInterCore(SYS_EVENT_ID_HDMI_RESET_EVENT, (&channel) );
			#endif
            H264_Encoder_RestartView(1, g_stEncoderStatus[1].resW, g_stEncoderStatus[1].resH, 
                                     g_stEncoderStatus[1].gop, g_stEncoderStatus[1].framerate, 
                                     g_stEncoderStatus[1].bitrate, g_stEncoderStatus[1].src);
        }
    }
    else
    {
        uint8_t chReset = 0;
        
        if ((g_stEncoderStatus[0].running == 1) && (Reg_Read32(ENC_REG_ADDR+(0x34<<2)) & (BIT(24)|BIT(25)|BIT(26))))
        {
            DLOG_Critical("PicSize Mismatch, Reset channel 0");
			uint8_t channel = 0;
			//SYS_EVENT_NotifyInterCore(SYS_EVENT_ID_HDMI_RESET_EVENT, (&channel) );
            // View0 encoder error, then do channel reset.
            Reg_Write32_Mask(ENC_REG_ADDR+(0x00<<2), 0, BIT(24));
            my_initial_all( 0 );
            Reg_Write32_Mask(ENC_REG_ADDR+(0x00<<2), BIT(24), BIT(24));
            Reg_Write32_Mask(ENC_REG_ADDR+(0x34<<2), 0, (BIT(24)|BIT(25)|BIT(26)));
            chReset = 1;
        }

        if ((g_stEncoderStatus[1].running == 1) && (Reg_Read32(ENC_REG_ADDR+(0x34<<2)) & (BIT(28)|BIT(29)|BIT(30))))
        {
            // View1 encoder error, then do channel reset.
            DLOG_Critical("PicSize Mismatch, Reset channel 1");
			#ifdef ARCAST
			uint8_t channel = 1;
			SYS_EVENT_NotifyInterCore(SYS_EVENT_ID_HDMI_RESET_EVENT, (&channel) );
			#endif

            // View1 encoder error, then do channel reset.
            Reg_Write32_Mask(ENC_REG_ADDR+(0x19<<2), 0, BIT(24));
            my_initial_all( 1 );
            Reg_Write32_Mask(ENC_REG_ADDR+(0x19<<2), BIT(24), BIT(24));
            Reg_Write32_Mask(ENC_REG_ADDR+(0x34<<2), 0, (BIT(28)|BIT(29)|BIT(30)));
            chReset = 1;
        }
        if (chReset == 0)
        {
            // Normal status, then do bitrate control.
            VEBRC_IRQ_Handler(view0_feedback, view1_feedback);
        }
		
    }

    Reg_Write32_Mask(ENC_REG_ADDR + 0xDC, (unsigned int)(0x01 << 24), BIT(31)|BIT(30)|BIT(29)|BIT(28)|BIT(27)|BIT(26)|BIT(25)|BIT(24));
    uint32_t regData = Reg_Read32( ENC_REG_ADDR + 0xE4 );
	uint16_t v0_width  =	(regData & 0xFFFF);
	uint16_t v0_height =	((regData >> 16)& 0xFFFF);
    Reg_Write32_Mask(ENC_REG_ADDR + 0xDC, (unsigned int)(0x11 << 24), BIT(31)|BIT(30)|BIT(29)|BIT(28)|BIT(27)|BIT(26)|BIT(25)|BIT(24));
    regData = Reg_Read32( ENC_REG_ADDR + 0xE4 );
	uint16_t v1_width  =	(regData & 0xFFFF);
	uint16_t v1_height =	((regData >> 16) & 0xFFFF);

	if( rca[0].ret_width != v0_width || rca[0].ret_hight != v0_height ||
		rca[1].ret_width != v1_width || rca[1].ret_hight != v1_height ) {
		STRU_SysEvent_H264RptParameter s_RptParam;
		s_RptParam.v0_width = v0_width;
		s_RptParam.v0_hight = v0_height;
		s_RptParam.v1_width = v1_width;
		s_RptParam.v1_hight = v1_height;
		SYS_EVENT_NotifyInterCore(SYS_EVENT_ID_H264_RPT_EVENT, (&s_RptParam) );	
		//DLOG_Critical("PicSize Change: V0 %d, %d, V1 %d, %d\n", v0_width, v0_height, v1_width, v1_height);
	}

	rca[0].ret_width = v0_width;
	rca[0].ret_hight = v0_height;
	rca[1].ret_width = v1_width;
	rca[1].ret_hight = v1_height;
}

static int __attribute__ ((section(".h264"))) H264_getCfgData(STRU_cfgBin *cfg, uint8_t encbit) 
{
    uint32_t cnt;
    STRU_cfgNode  *pst_node;
    STRU_REG *pst_regCfg;

    pst_regCfg = CFGBIN_GetNodeAndData(cfg, VSOC_ENC_INIT_ID, &pst_node);
    if (NULL != pst_node && NULL != pst_regCfg)
    {
        for (cnt = 0; cnt < (pst_node->nodeElemCnt); cnt++)
        {
            Reg_Write32_Mask(pst_regCfg->u32_regAddr, pst_regCfg->u32_regValue, pst_regCfg->u32_regdataMask);
            pst_regCfg += 1;
        }
    }

    pst_regCfg = CFGBIN_GetNodeAndData(cfg, VSOC_ENC_VIEW_ID_0, &pst_node);
    if (NULL != pst_node && NULL != pst_regCfg)
    {
        view_cfg(0, pst_regCfg, pst_node->nodeElemCnt);
    }
    
    pst_regCfg = CFGBIN_GetNodeAndData(cfg, VSOC_ENC_VIEW_ID_1, &pst_node);
    if (NULL != pst_node && NULL != pst_regCfg)
    {
        view_cfg(1, pst_regCfg, pst_node->nodeElemCnt);
    }

    //enable 8bit
    if (encbit > 0)
    {
        Reg_Write32_Mask((ENC_REG_ADDR+(0x00 << 2)), (1 << 21), BIT(21));
        Reg_Write32_Mask((ENC_REG_ADDR+(0x19 << 2)), (1 << 21), BIT(21));
    }

    if (2 == encbit)
    {
        Reg_Write32_Mask((ENC_REG_ADDR+(0x04 << 2)), (1 << 13), BIT(13));
        Reg_Write32_Mask((ENC_REG_ADDR+(0x1D << 2)), (1 << 13), BIT(13));
    }
    return 0;
}

int __attribute__ ((section(".h264"))) H264_Encoder_Init(uint8_t gop0, uint8_t br0, uint8_t brc0_e, uint8_t gop1, uint8_t br1, uint8_t brc1_e, uint8_t encbit) 
{
    // variable Declaraton 
    char spi_rd_dat, i2c_rd_dat;
    unsigned int wait_cnt, i;
    unsigned char read_cnt ;

    // Video_Soc Wait SDRAM INIT_DONE
    sdram_init_check(); 

    reg_IrqHandle(VIDEO_ARMCM7_IRQ_VECTOR_NUM, VEBRC_IRQ_Wrap_Handler, NULL);
    NVIC_SetPriority(VIDEO_ARMCM7_IRQ_VECTOR_NUM,NVIC_EncodePriority(NVIC_PRIORITYGROUP_5,INTR_NVIC_PRIORITY_VIDEO_ARMCM7,0));
    NVIC_DisableIRQ(VIDEO_ARMCM7_IRQ_VECTOR_NUM);

    g_stEncoderStatus[0].gop = gop0;
    g_stEncoderStatus[0].bitrate = br0;
    g_stEncoderStatus[0].brc_enable = brc0_e;
    rca[0].poweron_rc_params_set = 1;
	rca[0].photography_enable    = 0;
    rca[0].realtime_mode         = 0;	// fluquent mode, in default.
    #ifdef ARCAST
	rca[0].drop_scene_change_frame_en	= 1;
	#else
	rca[0].drop_scene_change_frame_en	= 0;
	#endif

    g_stEncoderStatus[1].gop = gop1;
    g_stEncoderStatus[1].bitrate = br1;
    g_stEncoderStatus[1].brc_enable = brc1_e;
    rca[1].poweron_rc_params_set = 1;
	rca[1].photography_enable    = 0;
    rca[1].realtime_mode         = 0;	// fluquent mode, in default.
    #ifdef ARCAST
    rca[1].drop_scene_change_frame_en = 1;
	#else
    rca[1].drop_scene_change_frame_en = 0;
	#endif
    
    //load from cfg.bin and write register
    H264_getCfgData((STRU_cfgBin *)SRAM_CONFIGURE_MEMORY_ST_ADDR, encbit);

    SYS_EVENT_RegisterHandler(SYS_EVENT_ID_IDLE, H264_Encoder_IdleCallback);
    SYS_EVENT_RegisterHandler(SYS_EVENT_ID_H264_INPUT_FORMAT_CHANGE, H264_Encoder_InputVideoFormatChangeCallback);
    SYS_EVENT_RegisterHandler(SYS_EVENT_ID_BB_SUPPORT_BR_CHANGE, H264_Encoder_BBModulationChangeCallback);
    SYS_EVENT_RegisterHandler(SYS_EVENT_ID_ENCODER_CMD, H264_Encoder_CmdProcess);

    //DLOG_Info("h264 encoder init OK\n");

    return 1;
}

int __attribute__ ((section(".h264"))) H264_Encoder_UsrCfg(uint8_t ch, STRU_ENC_CUSTOMER_CFG *pst_h264UsrCfg) 
{
    if (ch > 1)
    {
        return 0;
    }
    else
    {
        return usr_cfg(ch, pst_h264UsrCfg);
    }
}

