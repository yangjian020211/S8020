#include <stdint.h>

#include "h264_encoder.h"
#include "cfg_parser.h"


extern STRU_REG h264_init_reg[10];

STRU_cfgNode vsoc_enc_nodeInfo =
{
    .nodeId       = VSOC_ENC_INIT_ID,
    .nodeElemCnt  = 10,
    .nodeDataSize = sizeof(h264_init_reg)
};


STRU_REG h264_init_reg[10] __attribute__ ((aligned (4)))= 
{
    // view_0
    //opne 8bit mode. & VS Positive Polarity & HS Negative Polarity
    //*(0x01)|0x24    
    {ENC_REG_ADDR, (uint32_t)((1 << 21) | (4<<16)), (uint32_t)((1 << 21) | (7 << 16))},
    //{ENC_REG_ADDR, 0x240000, 0x270000},
    //Porch:
    {(ENC_REG_ADDR+(0x3<<2)), 0x00010000, 0xFFFFFFFF},
    {(ENC_REG_ADDR+(0x4<<2)), 0, (((uint32_t)0xFFF << 16) | 0xFFF)},
    //{ENC_REG_ADDR, 0x240000, 0x270000},
    //close auto check
    {(ENC_REG_ADDR+(0x0a<<2)), (uint32_t)(0<<5), (uint32_t)(3 << 5)},
    //DE mode & valid data in low 8bit:
    {(ENC_REG_ADDR+(0x04<<2)), (uint32_t)(1<<13), (uint32_t)(7 << 13)},
	
    // view_1
    //open 8bit mode. & VS Positive Polarity & HS Negative Polarity
    //*(0x65)|0x24
    {(ENC_REG_ADDR+(0x19<<2)), (uint32_t)((1 << 21) | (4<<16)), (uint32_t)((1 << 21) | (7 << 16))},
    //Porch:
    {(ENC_REG_ADDR+(0x1c<<2)), 0x00010000, 0xFFFFFFFF},
    {(ENC_REG_ADDR+(0x1d<<2)), 0, (((uint32_t)0xFFF << 16) | 0xFFF)},
    //close auto check
    {(ENC_REG_ADDR+(0x23<<2)), (uint32_t)(0<<5), (uint32_t)(3 << 5)},
    //DE mode & valid data in low 8bit:
    {(ENC_REG_ADDR+(0x1D<<2)), (uint32_t)(1<<13), (uint32_t)(7 << 13)},
};


extern STRU_REG h264_view_0[0];

STRU_cfgNode vsoc_enc_view_0 =
{
    .nodeId       = VSOC_ENC_VIEW_ID_0,
    .nodeElemCnt  = 0,
    .nodeDataSize = sizeof(h264_view_0)
};


STRU_REG h264_view_0[0] __attribute__ ((aligned (4)))= 
{
    // view_0
};



extern STRU_REG h264_view_1[0];

STRU_cfgNode vsoc_enc_view_1 =
{
    .nodeId       = VSOC_ENC_VIEW_ID_1,
    .nodeElemCnt  = 0,
    .nodeDataSize = sizeof(h264_view_1)
};


STRU_REG h264_view_1[0] __attribute__ ((aligned (4)))= 
{
	// view_1
};

