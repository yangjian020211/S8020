#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include "debuglog.h"
#include "bb_spi.h"
#include "bb_ctrl_internal.h"
#include "bb_grd_sweep.h"
#include "bb_grd_ctrl.h"
#include "bb_sky_ctrl.h"
#include "bb_uart_com.h"
#include "reg_rw.h"
#include "systicks.h"
#include "bb_regs.h"
#include "sys_event.h"
#include "rf_if.h"
#include "memory_config.h"
#include "boardParameters.h"
#include "factory.h"
#include "cfg_parser.h"
#include "pwr_ctrl.h"
#include "filter.h"

#define     BB_SPI_TEST         (0)
#define     RF_SPI_TEST         (0)

#define     VSOC_GLOBAL2_BASE   (0xA0030000)
#define     BB_SPI_UART_SEL     (0x9c)




#define 	SKY_PATTEN_SIZE_5G	4
static uint8_t vector_pwr_avrg_time_r[6]={1,1,1,2,2,3};
static uint8_t vector_snr_avrg_time_r[6]={1,1,1,2,2,3};
#define PRECIESE	10


pfun pfun_lna_open=NULL,pfun_lna_bypass=NULL;
pfun_fem pfun_fem_open = NULL,pfun_fem_close = NULL;
pfun pfun_f2g_open=NULL,pfun_f5g_open=NULL;

static int BB_before_RF_cali(void);
static void BB_GetRcIdVtIdFromFlash(uint8_t *pu8_rcid, uint8_t *pu8_vtid);
static void BB_after_RF_cali(ENUM_BB_MODE en_mode);
static void BB_RF_start_cali( void );
static void BB_HandleEventsCallback(void *p);
static void BB_SendRmoteEvent(void *p);
static void BB_AocInit(void);
static void BB_AesInit(ENUM_BB_MODE en_mode);
 

typedef struct _STRU_cmds
{
    uint8_t avail;                      /*command is using*/
    STRU_WIRELESS_CONFIG_CHANGE config;
}STRU_cmds;

typedef struct _STRU_AGC2LNA
{
    uint8_t u8_agcaBuf[1000];
    uint8_t u8_agcbBuf[1000];
    uint32_t sum_a;
    uint32_t sum_b;
    uint16_t mindex;//move avg index
    uint8_t  isFull;
}STRU_AGC2LNA;

volatile CONTEXT context;
static volatile ENUM_REG_PAGES en_curPage;

uint8_t *p_bbRegs = NULL;

STRU_BOARD_BB_PARA *pstru_bb_boardcfg= NULL;

STRU_BB_REG *p_bb_reg_beforeCali = NULL;
STRU_BB_REG *p_bb_reg_afterCali  = NULL;

STRU_BB_AOC *pstru_bb_aoc_boardcfg = NULL;
STRU_BB_AES *pstru_bb_aes_cfg = NULL;
STRU_AGC_SET *pstru_agc_set_cfg = NULL;
STRU_PWR_CTRL *pstru_pwr_ctrl_cfg = NULL;
STRU_RF_BAND_MCS_OPT *pstru_mcs_opt = NULL;
STRU_BB_LAN_AUTO *pstru_lna_auto = NULL;

STRU_RF_POWER_CTRL *pst_powercfg = NULL;

STRU_DT_HEAD g_dtLen[] = {  {0, 0, 1, 2, DT_NUM_RC_CH_MODE},
                            {0, 0, 1, 1, DT_NUM_IT_CH_MODE},
                            {0, 0, 1, 8, DT_NUM_RF_BAND_CHANGE},
                            {0, 0, 1, 1, DT_NUM_RF_CH_BW_CHANGE},
                            {0, 0, 1, 1, DT_NUM_MCS_MODE_SELECT},
                            {0, 0, 1, 1, DT_NUM_ENCODER_BRC_CHAGE_CH1},
                            {0, 0, 1, 1, DT_NUM_ENCODER_BRC_CHAGE_CH2},
                            {0, 0, 1, 1, DT_NUM_RF_CH_LDPC_CHANGE},
                            {0, 0, 1, 1, DT_NUM_RF_CH_QAM_CHANGE},
                            {0, 0, 0, 1, DT_NUM_GRD_MASK_CODE},
                            {0, 0, 1, 6, DT_NUM_RC_FRQ},
                            {0, 0, 1, 5, DT_NUM_IT_FRQ},
                            {0, 0, 1, 2, DT_NUM_RC_RATE},
                            {0, 0, 1, 7, DT_NUM_GRD_SEND_VT_ID},
//                            {0, 0, 1, 5, DT_NUM_GROUND_SYNC_RC_ID},
                            {0, 0, 1, 1, DT_NUM_RF_BAND_MODE},
//                            {0, 0, 1, SYS_EVENT_HANDLER_PARAMETER_LENGTH+1, DT_NUM_REMOTE_EVENT},
                            {0, 0, 0, sizeof(int), DT_NUM_AOC_ADJUST},
                            {0, 0, 1, 2, DT_NUM_FILTER_CHG},
                            {0, 0, 1, 2, DT_NUM_TAKE_PICTURE},
                            {0, 0, 1, 2, DT_NUM_PICTURE_QUALITY_SET},
                            {0, 0, 1, 2, DT_NUM_BAND_MODE_CHG},
                            {0, 0, 1, 1, DT_NUM_LNA_STATUS_CHG},
                            //{0, 0, 1, 1, DT_NUM_NON_LBT_ENABLE},
                            {0, 0, 1, 1, DT_NUM_NON_LBT_NOTICE_OPTCH},
                            {0, 0, 1, 1, DT_NUM_SKY_RC_PATTEN},
							{0, 0, 1, 1, DT_NUM_GRD_RC_CHPATTEN},
                            };

static uint8_t u8_session0RecvBuf[256] = {0};
static uint16_t u16_session0DataSize = 0;

static STRU_cmds cmds_poll[16];
static STRU_cmds rf_cmds_poll[16];
STRU_PURE_VT_INFO vt_info = {0, 0, AUTO};
uint8_t wake_up_flag = 0;
static STRU_AGC2LNA stru_agc2lna;

#if (defined RF_9363) && (defined RF_8003X)
#error "RF_9363 and RF_8003X only choose one"
#elif (!defined RF_9363) && (!defined RF_8003X)
#error "RF_9363 and RF_8003X must choose one"
#endif

static const uint16_t crc16tab[256]= {
    0x0000,0x1021,0x2042,0x3063,0x4084,0x50a5,0x60c6,0x70e7,
    0x8108,0x9129,0xa14a,0xb16b,0xc18c,0xd1ad,0xe1ce,0xf1ef,
    0x1231,0x0210,0x3273,0x2252,0x52b5,0x4294,0x72f7,0x62d6,
    0x9339,0x8318,0xb37b,0xa35a,0xd3bd,0xc39c,0xf3ff,0xe3de,
    0x2462,0x3443,0x0420,0x1401,0x64e6,0x74c7,0x44a4,0x5485,
    0xa56a,0xb54b,0x8528,0x9509,0xe5ee,0xf5cf,0xc5ac,0xd58d,
    0x3653,0x2672,0x1611,0x0630,0x76d7,0x66f6,0x5695,0x46b4,
    0xb75b,0xa77a,0x9719,0x8738,0xf7df,0xe7fe,0xd79d,0xc7bc,
    0x48c4,0x58e5,0x6886,0x78a7,0x0840,0x1861,0x2802,0x3823,
    0xc9cc,0xd9ed,0xe98e,0xf9af,0x8948,0x9969,0xa90a,0xb92b,
    0x5af5,0x4ad4,0x7ab7,0x6a96,0x1a71,0x0a50,0x3a33,0x2a12,
    0xdbfd,0xcbdc,0xfbbf,0xeb9e,0x9b79,0x8b58,0xbb3b,0xab1a,
    0x6ca6,0x7c87,0x4ce4,0x5cc5,0x2c22,0x3c03,0x0c60,0x1c41,
    0xedae,0xfd8f,0xcdec,0xddcd,0xad2a,0xbd0b,0x8d68,0x9d49,
    0x7e97,0x6eb6,0x5ed5,0x4ef4,0x3e13,0x2e32,0x1e51,0x0e70,
    0xff9f,0xefbe,0xdfdd,0xcffc,0xbf1b,0xaf3a,0x9f59,0x8f78,
    0x9188,0x81a9,0xb1ca,0xa1eb,0xd10c,0xc12d,0xf14e,0xe16f,
    0x1080,0x00a1,0x30c2,0x20e3,0x5004,0x4025,0x7046,0x6067,
    0x83b9,0x9398,0xa3fb,0xb3da,0xc33d,0xd31c,0xe37f,0xf35e,
    0x02b1,0x1290,0x22f3,0x32d2,0x4235,0x5214,0x6277,0x7256,
    0xb5ea,0xa5cb,0x95a8,0x8589,0xf56e,0xe54f,0xd52c,0xc50d,
    0x34e2,0x24c3,0x14a0,0x0481,0x7466,0x6447,0x5424,0x4405,
    0xa7db,0xb7fa,0x8799,0x97b8,0xe75f,0xf77e,0xc71d,0xd73c,
    0x26d3,0x36f2,0x0691,0x16b0,0x6657,0x7676,0x4615,0x5634,
    0xd94c,0xc96d,0xf90e,0xe92f,0x99c8,0x89e9,0xb98a,0xa9ab,
    0x5844,0x4865,0x7806,0x6827,0x18c0,0x08e1,0x3882,0x28a3,
    0xcb7d,0xdb5c,0xeb3f,0xfb1e,0x8bf9,0x9bd8,0xabbb,0xbb9a,
    0x4a75,0x5a54,0x6a37,0x7a16,0x0af1,0x1ad0,0x2ab3,0x3a92,
    0xfd2e,0xed0f,0xdd6c,0xcd4d,0xbdaa,0xad8b,0x9de8,0x8dc9,
    0x7c26,0x6c07,0x5c64,0x4c45,0x3ca2,0x2c83,0x1ce0,0x0cc1,
    0xef1f,0xff3e,0xcf5d,0xdf7c,0xaf9b,0xbfba,0x8fd9,0x9ff8,
    0x6e17,0x7e36,0x4e55,0x5e74,0x2e93,0x3eb2,0x0ed1,0x1ef0
};

uint16_t crc16(const char *buf, int len) {
    int counter;
    uint16_t crc = 0;
    for (counter = 0; counter < len; counter++)
            crc = (crc<<8) ^ crc16tab[((crc>>8) ^ *buf++)&0x00FF];
    return crc;
}
static uint32_t crc32_tab[] =
{
    0x00000000L, 0x77073096L, 0xee0e612cL, 0x990951baL, 0x076dc419L,
    0x706af48fL, 0xe963a535L, 0x9e6495a3L, 0x0edb8832L, 0x79dcb8a4L,
    0xe0d5e91eL, 0x97d2d988L, 0x09b64c2bL, 0x7eb17cbdL, 0xe7b82d07L,
    0x90bf1d91L, 0x1db71064L, 0x6ab020f2L, 0xf3b97148L, 0x84be41deL,
    0x1adad47dL, 0x6ddde4ebL, 0xf4d4b551L, 0x83d385c7L, 0x136c9856L,
    0x646ba8c0L, 0xfd62f97aL, 0x8a65c9ecL, 0x14015c4fL, 0x63066cd9L,
    0xfa0f3d63L, 0x8d080df5L, 0x3b6e20c8L, 0x4c69105eL, 0xd56041e4L,
    0xa2677172L, 0x3c03e4d1L, 0x4b04d447L, 0xd20d85fdL, 0xa50ab56bL,
    0x35b5a8faL, 0x42b2986cL, 0xdbbbc9d6L, 0xacbcf940L, 0x32d86ce3L,
    0x45df5c75L, 0xdcd60dcfL, 0xabd13d59L, 0x26d930acL, 0x51de003aL,
    0xc8d75180L, 0xbfd06116L, 0x21b4f4b5L, 0x56b3c423L, 0xcfba9599L,
    0xb8bda50fL, 0x2802b89eL, 0x5f058808L, 0xc60cd9b2L, 0xb10be924L,
    0x2f6f7c87L, 0x58684c11L, 0xc1611dabL, 0xb6662d3dL, 0x76dc4190L,
    0x01db7106L, 0x98d220bcL, 0xefd5102aL, 0x71b18589L, 0x06b6b51fL,
    0x9fbfe4a5L, 0xe8b8d433L, 0x7807c9a2L, 0x0f00f934L, 0x9609a88eL,
    0xe10e9818L, 0x7f6a0dbbL, 0x086d3d2dL, 0x91646c97L, 0xe6635c01L,
    0x6b6b51f4L, 0x1c6c6162L, 0x856530d8L, 0xf262004eL, 0x6c0695edL,
    0x1b01a57bL, 0x8208f4c1L, 0xf50fc457L, 0x65b0d9c6L, 0x12b7e950L,
    0x8bbeb8eaL, 0xfcb9887cL, 0x62dd1ddfL, 0x15da2d49L, 0x8cd37cf3L,
    0xfbd44c65L, 0x4db26158L, 0x3ab551ceL, 0xa3bc0074L, 0xd4bb30e2L,
    0x4adfa541L, 0x3dd895d7L, 0xa4d1c46dL, 0xd3d6f4fbL, 0x4369e96aL,
    0x346ed9fcL, 0xad678846L, 0xda60b8d0L, 0x44042d73L, 0x33031de5L,
    0xaa0a4c5fL, 0xdd0d7cc9L, 0x5005713cL, 0x270241aaL, 0xbe0b1010L,
    0xc90c2086L, 0x5768b525L, 0x206f85b3L, 0xb966d409L, 0xce61e49fL,
    0x5edef90eL, 0x29d9c998L, 0xb0d09822L, 0xc7d7a8b4L, 0x59b33d17L,
    0x2eb40d81L, 0xb7bd5c3bL, 0xc0ba6cadL, 0xedb88320L, 0x9abfb3b6L,
    0x03b6e20cL, 0x74b1d29aL, 0xead54739L, 0x9dd277afL, 0x04db2615L,
    0x73dc1683L, 0xe3630b12L, 0x94643b84L, 0x0d6d6a3eL, 0x7a6a5aa8L,
    0xe40ecf0bL, 0x9309ff9dL, 0x0a00ae27L, 0x7d079eb1L, 0xf00f9344L,
    0x8708a3d2L, 0x1e01f268L, 0x6906c2feL, 0xf762575dL, 0x806567cbL,
    0x196c3671L, 0x6e6b06e7L, 0xfed41b76L, 0x89d32be0L, 0x10da7a5aL,
    0x67dd4accL, 0xf9b9df6fL, 0x8ebeeff9L, 0x17b7be43L, 0x60b08ed5L,
    0xd6d6a3e8L, 0xa1d1937eL, 0x38d8c2c4L, 0x4fdff252L, 0xd1bb67f1L,
    0xa6bc5767L, 0x3fb506ddL, 0x48b2364bL, 0xd80d2bdaL, 0xaf0a1b4cL,
    0x36034af6L, 0x41047a60L, 0xdf60efc3L, 0xa867df55L, 0x316e8eefL,
    0x4669be79L, 0xcb61b38cL, 0xbc66831aL, 0x256fd2a0L, 0x5268e236L,
    0xcc0c7795L, 0xbb0b4703L, 0x220216b9L, 0x5505262fL, 0xc5ba3bbeL,
    0xb2bd0b28L, 0x2bb45a92L, 0x5cb36a04L, 0xc2d7ffa7L, 0xb5d0cf31L,
    0x2cd99e8bL, 0x5bdeae1dL, 0x9b64c2b0L, 0xec63f226L, 0x756aa39cL,
    0x026d930aL, 0x9c0906a9L, 0xeb0e363fL, 0x72076785L, 0x05005713L,
    0x95bf4a82L, 0xe2b87a14L, 0x7bb12baeL, 0x0cb61b38L, 0x92d28e9bL,
    0xe5d5be0dL, 0x7cdcefb7L, 0x0bdbdf21L, 0x86d3d2d4L, 0xf1d4e242L,
    0x68ddb3f8L, 0x1fda836eL, 0x81be16cdL, 0xf6b9265bL, 0x6fb077e1L,
    0x18b74777L, 0x88085ae6L, 0xff0f6a70L, 0x66063bcaL, 0x11010b5cL,
    0x8f659effL, 0xf862ae69L, 0x616bffd3L, 0x166ccf45L, 0xa00ae278L,
    0xd70dd2eeL, 0x4e048354L, 0x3903b3c2L, 0xa7672661L, 0xd06016f7L,
    0x4969474dL, 0x3e6e77dbL, 0xaed16a4aL, 0xd9d65adcL, 0x40df0b66L,
    0x37d83bf0L, 0xa9bcae53L, 0xdebb9ec5L, 0x47b2cf7fL, 0x30b5ffe9L,
    0xbdbdf21cL, 0xcabac28aL, 0x53b39330L, 0x24b4a3a6L, 0xbad03605L,
    0xcdd70693L, 0x54de5729L, 0x23d967bfL, 0xb3667a2eL, 0xc4614ab8L,
    0x5d681b02L, 0x2a6f2b94L, 0xb40bbe37L, 0xc30c8ea1L, 0x5a05df1bL,
    0x2d02ef8dL
};


signed char int2char(int data){
	signed char value=0;
	if(data<0){
		uint8_t temp = abs(data);
		if(temp >127) temp = 127;
		value = 0-temp; 
	}
	else 
		value = (data & 0x7F);
}
/* crc32 hash */
uint32_t crc32(const char* s, int len)
{
    int i;
    uint32_t crc32val = 0;
    crc32val ^= 0xFFFFFFFF;

    for (i = 0;  i < len;  i++) {
        crc32val = crc32_tab[(crc32val ^ s[i]) & 0xFF] ^ ((crc32val >> 8) & 0x00FFFFFF);
    }

    return labs(crc32val ^ 0xFFFFFFFF);
}

/*
 * set BB register to start close power mode base on factory setting, After calibration
*/
void BB_SetPowerCloseMode(ENUM_RF_BAND band)
{
    //if (band == RF_POWER_CLOSE)
    {
        //enable close mode register setting
        //page2:
        //0x1E[3]:reg_aagc_tx_agc_en_24g  1:enable 2.4G tssi  0:disable 2.4G tssi
        //0x1E[2]:reg_aagc_tx_agc_en_58g  1:enable 5.8G tssi  0:disable 5.8G tssi
        if(band == RF_2G)
        {
            BB_WriteRegMask(PAGE2, 0x1e, 0x08, 0x0c);
        }
        else
        {
            BB_WriteRegMask(PAGE2, 0x1e, 0x04, 0x0c);
        }

        //0x1F[3] : reg_tssi_only_a     1: ab path adjust by pathA   0: ab adjust seperately
        //0x1F[5]:  reg_tssi_diff       1: adjust 8db      0: adjust 4db
        //0x1F[7:6]: reg_tssi_time_option  tssi fresh time:  00:14ms  01:112ms   10:896m    11:7s
        BB_WriteRegMask(PAGE2, 0x1f, pst_powercfg->tssi_update_time << 6, 0xc0);

        //page0:
        //0xBD[7:0] per_db1  per_db2 for 2.4g, set to 0x11
        BB_WriteReg(PAGE0, 0xbd, 0x11);

        //0xCD[7:0] per_db1  per_db2 for 5.8g,  set to 0x11
        BB_WriteReg(PAGE0, 0xcd, 0x11);
    }
}

void   BB_SetPowerOpenMode(ENUM_RF_BAND band)
{
    BB_WriteRegMask(PAGE2, 0x1e, 0x00, 0x0c);
}

static void  __attribute__ ((section(".h264"))) BB_getCfgData(ENUM_BB_MODE en_mode, STRU_cfgBin *cfg)
{
    pstru_bb_boardcfg = (STRU_BOARD_BB_PARA *)CFGBIN_GetNodeData(cfg, BB_BOARDCFG_PARA_ID);
    STRU_RF_POWER_CTRL *power_data;
    STRU_cfgNode *cfgnode;

    STRU_BB_REG * bb_reg = (STRU_BB_REG *)CFGBIN_GetNodeData(cfg, BB_BOARDCFG_DATA_ID);

    if (en_mode == BB_SKY_MODE)
    {
        p_bb_reg_beforeCali = bb_reg;
        p_bb_reg_afterCali  = p_bb_reg_beforeCali + pstru_bb_boardcfg->u8_bbSkyRegsCnt + pstru_bb_boardcfg->u8_bbGrdRegsCnt;
    }
    else
    {
        p_bb_reg_beforeCali = bb_reg + pstru_bb_boardcfg->u8_bbSkyRegsCnt;
        p_bb_reg_afterCali  = p_bb_reg_beforeCali + pstru_bb_boardcfg->u8_bbGrdRegsCnt + pstru_bb_boardcfg->u8_bbSkyRegsCntAfterCali;
    }

    pst_powercfg = (STRU_RF_POWER_CTRL *)FCT_GetNodeAndData(FACTORY_SUBNODE_POWER_NODE_ID, &cfgnode);

    pstru_bb_aoc_boardcfg = CFGBIN_GetNodeData(cfg, BB_AOC_BOARDCFG_ID);
    pstru_lna_auto = CFGBIN_GetNodeData(cfg, BB_LNA_AUTO_CFG_ID);

    pstru_bb_aes_cfg   = FCT_GetNodeAndData(FACTORY_SUBNODE_BB_AES_CFG_NODE_ID, NULL);
    pstru_agc_set_cfg  = FCT_GetNodeAndData(FACTORY_SUBNODE_AGC_SET, NULL);
    pstru_pwr_ctrl_cfg = FCT_GetNodeAndData(FACTORY_SUBNODE_PWR_CTRL, NULL);
    
    STRU_BB_RC_CH_SELECTION *p_rc_sel = CFGBIN_GetNodeData(cfg, BB_RC_CH_SELECTION);
    memcpy((void *)&context.st_chSelectionParam, (void *)p_rc_sel, sizeof(context.st_chSelectionParam));

    pstru_mcs_opt = FCT_GetNodeAndData(FACTORY_SUBNODE_BAND_MCS_ID, NULL);
    if (pstru_mcs_opt != NULL)
    {
        memcpy((void *)(&context.st_bandMcsOpt), (void *)pstru_mcs_opt, sizeof(STRU_RF_BAND_MCS_OPT));
        //DLOG_Warning("%d %d %d  %d  %d", context.st_bandMcsOpt.e_bandsupport, context.st_bandMcsOpt.e_bandwidth, context.st_bandMcsOpt.e_rfbandMode, 
        //                                context.st_bandMcsOpt.u8_bbStartMcs10M, context.st_bandMcsOpt.u8_bbStartMcs20M);
    }
    STRU_MIMO_MODE *pst_mimo_cfg = FCT_GetNodeAndData(FACTORY_SUBNODE_MIMO_MODE_ID, NULL);
    if(pst_mimo_cfg != NULL)
    {
        context.st_mimo_mode.st_skyMimoMode = (ENUM_MIMO_MODE)pst_mimo_cfg->st_skyMimoMode;
        context.st_mimo_mode.st_grdMimoMode = (ENUM_MIMO_MODE)pst_mimo_cfg->st_grdMimoMode;
        context.st_mimo_mode.enum_lna_mode = pst_mimo_cfg->enum_lna_mode;
        if(context.st_mimo_mode.st_grdMimoMode >= MIMO_2T2R)
        {
            context.st_mimo_mode.st_grdMimoMode = MIMO_1T2R;
            //DLOG_Warning("FORCE 2T->1T");
        }
//      DLOG_Warning("mimo g=%d,s=%d,lna=%d",context.st_mimo_mode.st_grdMimoMode,context.st_mimo_mode.st_skyMimoMode,context.st_mimo_mode.enum_lna_mode);
    }
}


static void  __attribute__ ((section(".h264"))) BB_regs_init(ENUM_BB_MODE en_mode)
{
    uint32_t page_cnt = 0;
    uint8_t  regsize;

    if (NULL != pstru_bb_boardcfg)
    {
        regsize = (en_mode == BB_SKY_MODE) ? pstru_bb_boardcfg->u8_bbSkyRegsCnt : pstru_bb_boardcfg->u8_bbGrdRegsCnt;
    }

    //update the board registers
    {
        uint8_t num;
        for(num = 0; num < regsize; num++)
        {
            uint16_t addr  = ((uint16_t)p_bb_reg_beforeCali[num].page << 8) + p_bb_reg_beforeCali[num].addr;
            uint8_t  value = p_bb_reg_beforeCali[num].value;
            p_bbRegs[addr] = value;
        }
    }

    //wrtie power setting related register to p_bbRegs base on factory settting
    //BB_SetStartPower(p_bbRegs, pst_powercfg);

    for(page_cnt = 0 ; page_cnt < 4; page_cnt ++)
    {
        uint32_t addr_cnt=0;
        ENUM_REG_PAGES page = (ENUM_REG_PAGES)(page_cnt << 6);
        /*
         * PAGE setting included in the regs array.
         */
        en_curPage = page;

        for(addr_cnt = 0; addr_cnt < 256; addr_cnt++)
        {
            //PAGE1 reg[0xa1] reg[0xa2] reg[0xa4] reg[0xa5] are PLL setting for cpu0, cpu1, cpu2, set in the sysctrl.c when system init
            if(page==PAGE1 && (addr_cnt==0xa1||addr_cnt==0xa2||addr_cnt==0xa4||addr_cnt==0xa5))
            {}
            else
            {
                BB_SPI_curPageWriteByte((uint8_t)addr_cnt, *p_bbRegs);
            }
            
            p_bbRegs++;
        }
    }
}

void  BB_set_power(ENUM_RF_BAND band,uint8_t power)
{
    if(pst_powercfg == NULL)
    {
        DLOG_Error("NULL");
        BB_set_power_open(band,power);
        return;
    }

    if(context.e_powerMode == RF_POWER_OPEN)
    {
        BB_SPI_curPageWriteByte(0x01, 0x01);                //value2==0: write RF8003-0
                                                            //value2==1: write RF8003-1
        BB_set_power_open(band, power);
        BB_SPI_curPageWriteByte(0x01,0x02);
    }
    else
    {
        BB_set_power_close(band,power);
    }
}

uint8_t  __attribute__ ((section(".h264"))) BB_get_fct_power(ENUM_RF_BAND band)
{
    #define DEFAULT_FCT_POWER (14)
    if(pst_powercfg == NULL)
    {
//      DLOG_Error("pst_powercfg == NULL");
        return DEFAULT_FCT_POWER;
    }

#ifdef AR8020_MP_INFO_RECORD
    uint32_t power_offset = *((uint32_t *)SRAM_MODULE_SHARE_POWER_OFFSET);
    uint8_t pwr_2G_vt_offset = (power_offset >> 24) & 0x0FF;
    uint8_t pwr_2G_rc_offset = (power_offset >> 16) & 0x0FF;
    uint8_t pwr_5G_vt_offset = (power_offset >> 8) & 0x0FF;
    uint8_t pwr_5G_rc_offset = power_offset & 0x0FF;
    
    #define POWER_NEGATIVE_MARK     0x80
    #define POWER_POSITIVE_MARK     0x00
    int8_t signed_2G_vt_offset = 0;
    int8_t signed_2G_rc_offset = 0;
    int8_t signed_5G_vt_offset = 0;
    int8_t signed_5G_rc_offset = 0;

    if ((pwr_2G_vt_offset & 0x80) == POWER_NEGATIVE_MARK)
        signed_2G_vt_offset = (pwr_2G_vt_offset & 0x7F) * (-1);
    else
        signed_2G_vt_offset = (pwr_2G_vt_offset & 0x7F);

    if ((pwr_2G_rc_offset & 0x80) == POWER_NEGATIVE_MARK)
        signed_2G_rc_offset = (pwr_2G_rc_offset & 0x7F) * (-1);
    else
        signed_2G_rc_offset = (pwr_2G_rc_offset & 0x7F);

    if ((pwr_5G_vt_offset & 0x80) == POWER_NEGATIVE_MARK)
        signed_5G_vt_offset = (pwr_5G_vt_offset & 0x7F) * (-1);
    else
        signed_5G_vt_offset = (pwr_5G_vt_offset & 0x7F);

    if ((pwr_5G_rc_offset & 0x80) == POWER_NEGATIVE_MARK)
        signed_5G_rc_offset = (pwr_5G_rc_offset & 0x7F) * (-1);
    else
        signed_5G_rc_offset = (pwr_5G_rc_offset & 0x7F);
#else
    int8_t signed_2G_vt_offset = 0;
    int8_t signed_2G_rc_offset = 0;
    int8_t signed_5G_vt_offset = 0;
    int8_t signed_5G_rc_offset = 0;
#endif


    if(context.e_powerWorkMode == FCC)
    {
        if (band == RF_5G)
        {
            if(context.en_bbmode == BB_SKY_MODE)
            {
                return pst_powercfg->vtPowerFcc[1] + signed_5G_vt_offset;
            }
            else
            {
                return pst_powercfg->rcPowerFcc[1] + signed_5G_rc_offset;
            }
        }
        else
        {
            if(context.en_bbmode == BB_SKY_MODE)
            {
                return pst_powercfg->vtPowerFcc[0] + signed_2G_vt_offset;
            }
            else
            {
                return pst_powercfg->rcPowerFcc[0] + signed_2G_rc_offset;
            }

        }
    }
    else if(context.e_powerWorkMode == CE)
    {
        if (band == RF_5G)
        {
            if(context.en_bbmode == BB_SKY_MODE)
            {
                return pst_powercfg->vtPowerCe[1] + signed_5G_vt_offset;
            }
            else
            {
                return pst_powercfg->rcPowerCe[1] + signed_5G_rc_offset;
            }

        }
        else
        {
            if(context.en_bbmode == BB_SKY_MODE)
            {
                return pst_powercfg->vtPowerCe[0] + signed_2G_vt_offset;
            }
            else
            {
                return pst_powercfg->rcPowerCe[0] + signed_2G_rc_offset;
            }

        }
    }
    else if(context.e_powerWorkMode == SRRC)
    {
        if (band == RF_5G)
        {
            if(context.en_bbmode == BB_SKY_MODE)
            {
                return pst_powercfg->vtPowerSrrc[1] + signed_5G_vt_offset;
            }
            else
            {
                return pst_powercfg->rcPowerSrrc[1] + signed_5G_rc_offset;
            }

        }
        else
        {
            if(context.en_bbmode == BB_SKY_MODE)
            {
                return pst_powercfg->vtPowerSrrc[0] + signed_2G_vt_offset;
            }
            else
            {
                return pst_powercfg->rcPowerSrrc[0] + signed_2G_rc_offset;
            }

        }
    }
    else if(context.e_powerWorkMode == OTHER)
    {
        if (band == RF_5G)
        {
            if(context.en_bbmode == BB_SKY_MODE)
            {
                return pst_powercfg->vtPowerOther[1] + signed_5G_vt_offset;
            }
            else
            {
                return pst_powercfg->rcPowerOther[1] + signed_5G_rc_offset;
            }

        }
        else
        {
            if(context.en_bbmode == BB_SKY_MODE)
            {
                return pst_powercfg->vtPowerOther[0] + signed_2G_vt_offset;
            }
            else
            {
                return pst_powercfg->rcPowerOther[0] + signed_2G_rc_offset;
            }

        }

    }
    else
    {
        DLOG_Error("%d",context.e_powerWorkMode);
    }

    return DEFAULT_FCT_POWER;
}

void  __attribute__ ((section(".h264"))) BB_load_fct_power(void)
{
    context.u8_TargetPower[0] = BB_get_fct_power(RF_2G);
    context.u8_TargetPower[1] = BB_get_fct_power(RF_5G);

    context.stru_bandSwitchParam.i8_skyPower0 = context.u8_TargetPower[0];
    context.stru_bandSwitchParam.i8_skyPower1 = context.u8_TargetPower[1];
}

uint8_t  __attribute__ ((section(".h264"))) BB_get_band_power(ENUM_RF_BAND band)
{
    if(context.e_curBand == RF_5G)
    {
        return context.u8_TargetPower[1];
    }
    else
    {
        return context.u8_TargetPower[0];
    }
}

static void  __attribute__ ((section(".h264"))) BB_set_TxMIMO(uint8_t value)
{
    #define TX_MIMO_REG_ADDR (0x03)
    BB_WriteRegMask(PAGE2,TX_MIMO_REG_ADDR,(uint8_t)value<<7,0x80);
}

static void  __attribute__ ((section(".h264"))) BB_set_RxMIMO(ENUM_BB_MODE e_bb_mode,ENUM_MIMO_MODE e_rx_mimo)
{
    #define RX_MIMO_REG_ADDR (0x08)
    if(e_rx_mimo == MIMO_1T1R)
    {
        BB_WriteRegMask(PAGE0,0x90,0x01<<7,0x80);
        BB_WriteReg(PAGE0,0xC0,0xff);
        BB_WriteReg(PAGE0,0xC1,0xf0);
        BB_WriteReg(PAGE0,0xC2,0x00);
        BB_WriteReg(PAGE0,0xC3,0x00);
        BB_WriteReg(PAGE0,0xC4,0x00);
        BB_WriteReg(PAGE0,0xC5,0x00);
        BB_WriteReg(PAGE1,0x90,0xd7);//for low power
        if(e_bb_mode == BB_GRD_MODE)
        {
            BB_WriteRegMask(PAGE0,0x91,0,0x08);
        }
        
        e_rx_mimo = MIMO_1T2R;//BB 1t1r not real support,false support,so force to 1t2r
    }
    
    BB_WriteRegMask(PAGE2,RX_MIMO_REG_ADDR,(uint8_t)e_rx_mimo<<4,0x30);

}

static void  __attribute__ ((section(".h264"))) BB_setMimoMode(ENUM_MIMO_MODE e_mimo_mode)
{
    if(context.en_bbmode == BB_SKY_MODE)
    {
        if(e_mimo_mode == MIMO_2T2R)
        {
            BB_set_TxMIMO(1);
        }
        else
        {
            BB_set_TxMIMO(0);
        }
        BB_set_RxMIMO(context.en_bbmode,context.st_mimo_mode.st_grdMimoMode);
        BB_set_RF_mimo_mode(e_mimo_mode);
    }
    else
    {
        BB_set_RxMIMO(context.en_bbmode,context.st_mimo_mode.st_skyMimoMode);
        BB_set_RF_mimo_mode(e_mimo_mode);
    }
    
    DLOG_Warning("%d",e_mimo_mode);
}
int  __attribute__ ((section(".h264"))) BB_softTxReset(ENUM_BB_MODE en_mode)
{
    uint8_t reg_after_reset;
    if(en_mode == BB_GRD_MODE)
    {
        BB_SPI_curPageWriteByte(0x00,0xB4);
        BB_SPI_curPageWriteByte(0x00,0xB0);
        reg_after_reset = 0xB0;
    }
    else
    {        
        BB_SPI_curPageWriteByte(0x00, 0x84);
        BB_SPI_curPageWriteByte(0x00, 0x80);
        reg_after_reset = 0x80;
    }

    //bug fix: write reset register may fail. 
    int count = 0;
    while(count++ < 5)
    {
        uint8_t rst = BB_SPI_curPageReadByte(0x00);
        if(rst != reg_after_reset)
        {
            BB_SPI_curPageWriteByte(0x00, reg_after_reset);
            count ++;
        }
        else
        {
            break;
        }
    }

    if (count >= 5)
    {
        DLOG_Warning("Error");
    }
    en_curPage = PAGE2;
    DLOG_Warning("");
    return 0;
}
int BB_softRxReset(ENUM_BB_MODE en_mode)
{
    uint8_t reg_after_reset;
    if(en_mode == BB_GRD_MODE)
    {
        BB_SPI_curPageWriteByte(0x00,0xB2);
        BB_SPI_curPageWriteByte(0x00,0xB0);
        reg_after_reset = 0xB0;
    }
    else
    {        
        BB_SPI_curPageWriteByte(0x00, 0x82);
        BB_SPI_curPageWriteByte(0x00, 0x80);
        reg_after_reset = 0x80;
    }

    //bug fix: write reset register may fail. 
    int count = 0;
    while(count++ < 5)
    {
        uint8_t rst = BB_SPI_curPageReadByte(0x00);
        if(rst != reg_after_reset)
        {
            BB_SPI_curPageWriteByte(0x00, reg_after_reset);
            count ++;
        }
        else
        {
            break;
        }
    }

    if (count >= 5)
    {
        DLOG_Warning("Error");
    }
    en_curPage = PAGE2;
    DLOG_Warning("");
    return 0;
}

int  BB_softReset(ENUM_BB_MODE en_mode)
{
    uint8_t reg_after_reset;
    if(en_mode == BB_GRD_MODE)
    {
        BB_SPI_curPageWriteByte(0x00,0xB1);
        BB_SPI_curPageWriteByte(0x00,0xB0);
        reg_after_reset = 0xB0;
    }
    else
    {        
        BB_SPI_curPageWriteByte(0x00, 0x81);
        BB_SPI_curPageWriteByte(0x00, 0x80);
        reg_after_reset = 0x80;
    }

    //bug fix: write reset register may fail. 
    int count = 0;
    while(count++ < 5)
    {
        uint8_t rst = BB_SPI_curPageReadByte(0x00);
        if(rst != reg_after_reset)
        {
            BB_SPI_curPageWriteByte(0x00, reg_after_reset);
            count ++;
        }
        else
        {
            break;
        }
    }

    if (count >= 5)
    {
        DLOG_Warning("Error");
    }
    en_curPage = PAGE2;
    //DLOG_Warning("");
    return 0;
}


STRU_CUSTOMER_CFG stru_defualtCfg = 
{
    .itHopMode        = AUTO,
    .rcHopMode        = AUTO,
    .qam_skip_mode    = AUTO,
};


void  __attribute__ ((section(".h264"))) BB_ResetRcMap(void)
{
    uint8_t i;

    for (i = 0; i < sizeof(context.rc_ch_map); ++i)
    {
        context.rc_ch_map[i]    = i;
    }
}


void  __attribute__ ((section(".h264"))) BB_init(ENUM_BB_MODE en_mode, STRU_CUSTOMER_CFG *pstru_customerCfg, uint8_t *bb_reg)
{
    p_bbRegs = bb_reg;
    if (NULL==pstru_customerCfg)
    {
        pstru_customerCfg = &stru_defualtCfg;
    }

    BB_getCfgData(en_mode, (STRU_cfgBin *)SRAM_CONFIGURE_MEMORY_ST_ADDR);

    context.en_bbmode = en_mode;

    context.itHopMode = pstru_customerCfg->itHopMode;
    context.rcHopMode = pstru_customerCfg->rcHopMode;
    context.qam_skip_mode = pstru_customerCfg->qam_skip_mode;


    BB_GetRcIdVtIdFromFlash((uint8_t *)context.rcid, (uint8_t *)context.vtid);
    BB_GenHashId((uint8_t *)context.rcid, (uint8_t *)context.vtid,BB_SKY_MODE);
    BB_GenHashId((uint8_t *)context.rcid, (uint8_t *)context.vtid,BB_GRD_MODE);

    BB_uart10_spi_sel(0x00000003);
    BB_SPI_init();

    BB_regs_init(context.en_bbmode);
    RF_init(en_mode);
    BB_InitSunBand_RcFreqNum(FREQ_SUB_BAND_10M_RC_NUM, FREQ_SUB_BAND_20M_RC_NUM);

#ifdef JYDS
    context.st_bandMcsOpt.e_rfbandMode = MANUAL;
    context.e_curBand = RF_2G;
#else
    context.e_curBand     = (context.st_bandMcsOpt.e_bandsupport & RF_2G) ? RF_2G : context.st_bandMcsOpt.e_bandsupport;
#endif
    context.u8_bbStartMcs = (context.st_bandMcsOpt.e_bandwidth == BW_20M) ? context.st_bandMcsOpt.u8_bbStartMcs20M : context.st_bandMcsOpt.u8_bbStartMcs10M;

    BB_set_ItFrqByCh(context.e_curBand, BB_GetItFrqNum(context.e_curBand)/2);
    BB_set_Rcfrq(context.e_curBand, BB_GetRcFrqNum(context.e_curBand)/2);

	
    
    RF_CaliProcess(en_mode);

    BB_after_RF_cali(en_mode);
    BB_softReset(en_mode);
#ifdef RF_9363
	if(RF_600M == context.e_curBand)
	{
		BB_FilterInit();
	}
#endif
    BB_set_RF_Band(en_mode, context.e_curBand);
    BB_set_RF_bandwitdh(en_mode, context.st_bandMcsOpt.e_bandwidth);

    context.e_powerMode = pst_powercfg->mode;
    context.e_powerWorkMode = pst_powercfg->power_work_mode;
    context.aoc.u8_aocEnable = pst_powercfg->powerAutoCtrl;
    BB_load_fct_power();
    context.pwr = BB_get_band_power(context.e_curBand);
    context.u8_aocAdjustPwr = context.pwr;

    if(context.e_powerMode == RF_POWER_CLOSE)
    {
        BB_SetPowerCloseMode(context.e_curBand);
    }
    BB_set_power(context.e_curBand,context.pwr);
//  DLOG_Warning("pwr mode %d,auto %d std %d, val %d",context.e_powerMode,context.aoc.u8_aocEnable,context.e_powerWorkMode,context.pwr);
    BB_AocInit();

    BB_setMimoMode(context.en_bbmode == BB_SKY_MODE ? context.st_mimo_mode.st_skyMimoMode : context.st_mimo_mode.st_grdMimoMode);
    BB_softReset(en_mode);
    BB_AesInit(en_mode);
    BB_Lna_reset();

    context.factory_lna_mode = context.st_mimo_mode.enum_lna_mode;
    if(context.st_mimo_mode.enum_lna_mode == LNA_BYPASS)
    {
        BB_bypass_lna();
        context.lna_status = BYPASS_LNA;
    }
    else
    {
        BB_open_lna();
        context.lna_status = OPEN_LNA;
    }

    SYS_EVENT_RegisterHandler(SYS_EVENT_ID_USER_CFG_CHANGE, BB_HandleEventsCallback);
    SYS_EVENT_RegisterHandler(SYS_EVENT_ID_REMOTE_EVENT, BB_SendRmoteEvent);
    //BB_ResetRcMap();

//  DLOG_Warning("cfg:%d %d %d", context.st_bandMcsOpt.e_bandwidth, context.e_curBand, context.st_bandMcsOpt.e_bandsupport);
    context.freq_band_mode = context.st_bandMcsOpt.frq_band_mode;
	#ifdef RFSUB_BAND
    if(context.freq_band_mode == SUB_BAND)
    {
        context.sub_band_value = INVALIDE_SUB_BAND_VALUE;
        context.st_chSelectionParam.u8_rcChSelectionEnable = 0;
        DLOG_Warning("subBB no rcChSel");
    }
	#endif
	/*
    if(BB_GetRcFrqNum(context.e_curBand) != RC_SKIP_CNT ||
        context.freq_band_mode == SUB_BAND || 
        RF_600M == context.e_curBand || context.enable_rc_random_skip_patten == 0)
    {
        //context.enable_rc_skip_patten = 0;
        DLOG_Warning("unen rcRandom");
    }
    else
    {
        //context.enable_rc_skip_patten = 0;
        //context.rc_skip_patten = 0xff;
        DLOG_Warning("en rcRandom");
    }
	*/
    memset(cmds_poll, 0x00, sizeof(cmds_poll));
    memset(rf_cmds_poll, 0x00, sizeof(rf_cmds_poll));
    context.rc_start = 0;
    context.rc_end = BB_GetRcFrqNumPerFilter();
    context.vt_start = 0;
    context.vt_end = BB_GetItFrqNumPerFilter();
    BB_WriteReg(PAGE2, SPI_DT_END_ADDR, 0);
	
	context.rf_info.rc_patten_nextchg_delay = SysTicks_GetTickCount();
	bb_update_rc_patten_size();
	
	rc_set_unlock_patten();
	context.rf_info.rc_patten_set_by_usr=0;
	//DLOG_Warning("eband=%d,rc_size=%d",context.e_curBand,BB_GetRcFrqNum(context.e_curBand));

	
}


void  __attribute__ ((section(".h264"))) BB_uart10_spi_sel(uint32_t sel_dat)
{
    write_reg32( (uint32_t *)(VSOC_GLOBAL2_BASE + BB_SPI_UART_SEL), sel_dat);
}

void  __attribute__ ((section(".h264"))) BB_rc_hope_mode_set(ENUM_RC_HOPE_PATTEN_MODE mode)
{
    context.rc_hoping_patten_mode=mode;
	/*
	if(mode==RANDOM_HOPING){
		//context.enable_rc_skip_patten = 1;
        context.rc_skip_patten = 0xff;
        DLOG_Warning("en rcRandom");
	}
	*/
}


uint8_t  __attribute__ ((section(".h264"))) BB_WriteReg(ENUM_REG_PAGES page, uint8_t addr, uint8_t data)
{
    if(en_curPage != page)
    {
        BB_SPI_WriteByte(page, addr, data);
        en_curPage = page;
    }
    else
    {
        BB_SPI_curPageWriteByte(addr, data);
    }
}

uint8_t  __attribute__ ((section(".h264"))) BB_ReadReg(ENUM_REG_PAGES page, uint8_t addr)
{
    uint8_t reg;
    if(en_curPage != page)
    {
        reg = BB_SPI_ReadByte(page, addr);
        en_curPage = page;
    }
    else
    {
        reg = BB_SPI_curPageReadByte(addr);
    }
    return reg;
}

int  __attribute__ ((section(".h264"))) BB_WriteRegMask(ENUM_REG_PAGES page, uint8_t addr, uint8_t data, uint8_t mask)
{
    uint8_t ori = BB_ReadReg(page, addr);
    data = (ori & (~mask)) | data;
    return BB_WriteReg(page, addr, data);
}


int  __attribute__ ((section(".h264"))) BB_ReadRegMask(ENUM_REG_PAGES page, uint8_t addr, uint8_t mask)
{
    return BB_ReadReg(page, addr) & mask;
}



const uint8_t mcs_idx_bitrate_map_10m[] = 
{
    1,      //0.6Mbps BPSK 1/2
    2,      //1.2     BPSK 1/2
    3,      //2.4     QPSK 1/2
    8,      //5.0     16QAM 1/2
    11,     //7.5     64QAM 1/2
    13,     //10      64QAM 2/3
};

const uint8_t mcs_idx_bitrate_map_20m[] = 
{
    2,      //1.2Mbps BPSK 1/2
    3,      //2.4     BPSK 1/2
    8,      //5.0     QPSK 1/2
    11,     //7.5     16QAM 1/2
    13,     //10      16QAM 2/3
};

uint8_t  __attribute__ ((section(".h264"))) BB_get_bitrateByMcs(ENUM_CH_BW bw, uint8_t u8_mcs)
{
    if (BW_10M == bw)
    {
        return mcs_idx_bitrate_map_10m[u8_mcs];
    }
    else
    {
        return mcs_idx_bitrate_map_20m[u8_mcs];
    }
}



void  __attribute__ ((section(".h264"))) BB_saveRcid(uint8_t *u8_rcIdArray, uint8_t *u8_vtIdArray)
{
    STRU_SysEvent_NvMsg st_nvMsg;

    // src:cpu0 dst:cpu2
    st_nvMsg.u8_nvSrc = INTER_CORE_CPU2_ID;
    st_nvMsg.u8_nvDst = INTER_CORE_CPU0_ID;

    // parament number
    st_nvMsg.e_nvNum = NV_NUM_RCID;

    // parament set
    st_nvMsg.u8_nvPar[0] = 1;
    memcpy(&(st_nvMsg.u8_nvPar[1]), u8_rcIdArray, RC_ID_SIZE);
    memcpy(&(st_nvMsg.u8_nvPar[1 + RC_ID_SIZE]), u8_vtIdArray, VT_ID_SIZE);

    // send msg
    SYS_EVENT_NotifyInterCore(SYS_EVENT_ID_NV_MSG, (void *)(&(st_nvMsg)));
}

void  __attribute__ ((section(".h264")))  BB_set_QAM(ENUM_BB_QAM mod)
{
    uint8_t data = BB_ReadReg(PAGE2, TX_2);
    BB_WriteReg(PAGE2, TX_2, (data & 0x3f) | ((uint8_t)mod << 6));
}

void  __attribute__ ((section(".h264")))  BB_set_LDPC(ENUM_BB_LDPC ldpc)
{
    uint8_t data = BB_ReadReg(PAGE2, TX_2);
    BB_WriteReg(PAGE2, TX_2, (data & 0xF8) | (uint8_t)ldpc);
}


/************************************************************
PAGE2    0x20[2]    rf_freq_sel_rx_sweep    RW    sweep frequency selection for the 2G frequency band o or 5G frequency band,
        0'b0: 2G frequency band
        1'b1: 5G frequency band

PAGE2    0x21[7]    rf_freq_sel_tx              RW     The frequency band selection for the transmitter
        1'b0: 2G frequency band
        1'b1 for 5G frequency band

PAGE2    0x21[4]    rf_freq_sel_rx_work       RW    The frequency band selection for the receiver
        1'b0: 2G frequency band
        1'b1 for 5G frequency band
*****************************************************************/
/*
 * set RF baseband to 2.4G or 5G
 */
void  __attribute__ ((section(".h264")))  BB_set_RF_Band(ENUM_BB_MODE sky_ground, ENUM_RF_BAND rf_band)
{
    if(rf_band == RF_2G)
    {
        BB_WriteRegMask(PAGE0, 0x20, 0x08, 0x0c);
        BB_WriteRegMask(PAGE2, 0x21, 0x00, 0x90);
        BB_WriteRegMask(PAGE2, 0x20, 0x00, 0x04);
        if(pfun_f2g_open){
            pfun_f2g_open();
        }
    }
    else if(rf_band == RF_5G)
    {
        // P0 0x20 [3]=0, [2]=1,2G PA off,5G PA on
        BB_WriteRegMask(PAGE0, 0x20, 0x04, 0x0C); 
        // P2 0x21 [7]=1, [4]=1,rf_freq_sel_tx,rf_freq_sel_rx,5G
        BB_WriteRegMask(PAGE2, 0x21, 0x90, 0x90); 
        // P2 0x20 [2]=1,sweep frequency,5G
        BB_WriteRegMask(PAGE2, 0x20, 0x04, 0x04);

        //softreset
        //BB_softReset(sky_ground);
        if(pfun_f5g_open){
            pfun_f5g_open();
        }
    }

    //calibration and reset
    BB_RF_band_switch(rf_band);
	//reset_sweep_table(rf_band);
}


/*
 * set RF bandwidth = 10M or 20M
*/
void  __attribute__ ((section(".h264"))) BB_set_RF_bandwitdh(ENUM_BB_MODE sky_ground, ENUM_CH_BW rf_bw)
{
    if (sky_ground == BB_SKY_MODE){
        BB_WriteRegMask(PAGE2, TX_2, (rf_bw << 3), 0x38); /*bit[5:3]*/
        if (BW_20M == (context.st_bandMcsOpt.e_bandwidth)){
            BB_WriteRegMask(PAGE2, 0x05, 0x80, 0xC0);
        }
    }
    else{
        BB_WriteRegMask(PAGE2, RX_MODULATION, (rf_bw << 0), 0x07); /*bit[2:0]*/
    }
   
    DLOG_Info("%d", context.st_bandMcsOpt.e_bandwidth);   

	//RF8003s_GetFctFreqTable(context.st_bandMcsOpt.e_bandwidth);
	//rc_set_unlock_patten();
    //softreset
    //BB_softReset(sky_ground);
}

static void  __attribute__ ((section(".h264"))) BB_after_RF_cali(ENUM_BB_MODE en_mode)
{
    //BB_WriteRegMask(PAGE0, 0x20, 0x80, 0x80);
    // enalbe RXTX
    //BB_WriteRegMask(PAGE1, 0x94, 0x10, 0xFF);    //remove to fix usb problem

    uint8_t bb_regcnt = (en_mode == BB_SKY_MODE) ? pstru_bb_boardcfg->u8_bbSkyRegsCntAfterCali : pstru_bb_boardcfg->u8_bbGrdRegsCntAfterCali;

    if ( bb_regcnt > 0)
    {
        uint8_t cnt;
        for(cnt = 0; cnt < bb_regcnt; cnt ++)
        {
            ENUM_REG_PAGES page = (ENUM_REG_PAGES )(p_bb_reg_afterCali[cnt].page << 6);
            BB_WriteReg(page, p_bb_reg_afterCali[cnt].addr, p_bb_reg_afterCali[cnt].value);
        }
    }

}

/*
 * BB_Getcmd: get command from command buffer pool and free the buffer
*/
int  __attribute__ ((section(".h264"))) BB_GetCmd(uint8_t type, STRU_WIRELESS_CONFIG_CHANGE *pconfig)
{
    uint8_t found = 0;
    uint8_t i;
    uint8_t loop = 0;
    STRU_cmds *p_cmd = NULL;

    if(0 == type)
    {
        p_cmd = cmds_poll;
        loop = sizeof(cmds_poll) / sizeof(cmds_poll[0]);
    }
    else if(1 == type)
    {
        p_cmd = rf_cmds_poll;
        loop = sizeof(rf_cmds_poll) / sizeof(rf_cmds_poll[0]);
    }
    else
    {
        return FALSE;
    }
    
    for(i = 0; i < loop; i++)
    {
        if(p_cmd[i].avail == 1)
        {
            memcpy(pconfig, &(p_cmd[i].config), sizeof(STRU_WIRELESS_CONFIG_CHANGE));
            p_cmd[i].avail = 0;
            found = 1;
            //DLOG_Warning("%d %08x", i, p_cmd[i].config.u32_configValue);
            break;
        }
    }

    return (found) ? TRUE:FALSE;
}


int  __attribute__ ((section(".h264"))) BB_InsertCmd(uint8_t type, STRU_WIRELESS_CONFIG_CHANGE *p)
{
    uint8_t i;
    uint8_t found = 0;
    uint8_t loop = 0;
    STRU_cmds *p_cmd = NULL;

    if(0 == type)
    {
        p_cmd = cmds_poll;
        loop = sizeof(cmds_poll) / sizeof(cmds_poll[0]);
    }
    else if(1 == type)
    {
        p_cmd = rf_cmds_poll;
        loop = sizeof(rf_cmds_poll) / sizeof(rf_cmds_poll[0]);
    }
    else
    {
        return FALSE;
    }

    for(i = 0; i < loop; i++)
    {
        if(p_cmd[i].avail == 0)
        {
            memcpy((void *)(&p_cmd[i].config), p, sizeof(p_cmd[0].config));
            p_cmd[i].avail = 1;
            found = 1;
            //DLOG_Warning("%d %08x", i, p_cmd[i].config.u32_configValue);
            break;
        }
    }

    if(!found)
    {
        DLOG_Warning("error");
    }

    return (found? TRUE:FALSE);
}

int  __attribute__ ((section(".h264"))) BB_add_cmds(uint8_t type, uint32_t param0, uint32_t param1, uint32_t param2, uint32_t param3)
{
    STRU_WIRELESS_CONFIG_CHANGE cmd;
    int ret = 1;

    DLOG_Warning("type:%d:%d %d %d %d", type, param0, param1, param2, param3);
    switch(type)
    {
#if 0
        case 0:
        {        
            cmd.u8_configClass  = WIRELESS_FREQ_CHANGE;
            cmd.u8_configItem   = FREQ_BAND_WIDTH_SELECT;
            cmd.u32_configValue  = param0;
            break;
        }

        case 1:
        {
            cmd.u8_configClass  = WIRELESS_FREQ_CHANGE;
            cmd.u8_configItem   = FREQ_BAND_MODE;
            cmd.u32_configValue  = param0;
            break;            
        }
#endif

        case 2:
        {
            cmd.u8_configClass  = WIRELESS_FREQ_CHANGE;
            cmd.u8_configItem   = FREQ_BAND_SELECT;
            cmd.u32_configValue  = param0;
            break;
        }
#if 0
        case 3:
        {
            cmd.u8_configClass  = WIRELESS_FREQ_CHANGE;
            cmd.u8_configItem   = FREQ_CHANNEL_MODE;
            cmd.u32_configValue  = param0;
            break;
        }
    
        case 4:
        {
            cmd.u8_configClass  = WIRELESS_FREQ_CHANGE;
            cmd.u8_configItem   = FREQ_CHANNEL_SELECT;
            cmd.u32_configValue  = param0;
            break;
        }
#endif

        case 5:        
        {
            cmd.u8_configClass  = WIRELESS_MCS_CHANGE;
            cmd.u8_configItem   = MCS_MODE_SELECT;
            cmd.u32_configValue  = param0;
            break;
        }

        case 6:
        {
            cmd.u8_configClass  = WIRELESS_MCS_CHANGE;
            cmd.u8_configItem   = MCS_MODULATION_SELECT;
            cmd.u32_configValue  = param0;
            break;            
        }
#if 0
        case 7:
        {
            cmd.u8_configClass  = WIRELESS_MCS_CHANGE;
            cmd.u8_configItem   = MCS_CODE_RATE_SELECT;
            cmd.u32_configValue  = param0;
            break;
        }


        case 9:
        {
            cmd.u8_configClass  = WIRELESS_ENCODER_CHANGE;
            cmd.u8_configItem   = ENCODER_DYNAMIC_BIT_RATE_SELECT_CH1;
            cmd.u32_configValue  = param0;
            break;
        }

        case 10:
        {
            cmd.u8_configClass  = WIRELESS_ENCODER_CHANGE;
            cmd.u8_configItem   = ENCODER_DYNAMIC_BIT_RATE_SELECT_CH2;
            cmd.u32_configValue = param0;
            break;
        }

        case 11:
        {
            cmd.u8_configClass  = WIRELESS_MISC;
            cmd.u8_configItem   = MISC_READ_RF_REG;
            cmd.u32_configValue = (param0) | (param1 << 8) | (param2 << 16);

            DLOG_Info("1:%d 2:%d 3:%d 4:%d", type, param0, param1, param2);
            break;
        }

        case 12:
        {
            cmd.u8_configClass  = WIRELESS_MISC;
            cmd.u8_configItem   = MISC_WRITE_RF_REG;
                               //8003s num: addr: value
            cmd.u32_configValue  = (param0) | (param1 << 8) | (param2 << 16) | (param3 << 24);
            break;
        }

        case 13:
        {
            cmd.u8_configClass  = WIRELESS_MISC;
            cmd.u8_configItem   = MISC_READ_BB_REG;
                               //page, addr
            cmd.u32_configValue  = param0 | (param1 << 8);
            break;
        }
        
        case 14:
        {
            cmd.u8_configClass  = WIRELESS_MISC;
            cmd.u8_configItem   = MISC_WRITE_BB_REG;
                               //page, addr, value
            cmd.u32_configValue  = (param0) | (param1<<8) | (param2<<16);
            break;
        }

        case 15:
        {
            cmd.u8_configClass  = WIRELESS_DEBUG_CHANGE;
            cmd.u8_configItem   = 1;
            break;
        }

        case 16:
        {
            cmd.u8_configClass  = WIRELESS_AUTO_SEARCH_ID;
            cmd.u8_configItem   = 0;
            break;
        }
        
        case 17:
        {
            cmd.u8_configClass  = WIRELESS_MCS_CHANGE;
            cmd.u8_configItem   = MCS_IT_QAM_SELECT;
            cmd.u32_configValue  = (param0);
            break;
        }
        
        case 18:
        {
            cmd.u8_configClass  = WIRELESS_MCS_CHANGE;
            cmd.u8_configItem   = MCS_IT_CODE_RATE_SELECT;
            cmd.u32_configValue  = (param0);
            break;
        }

        case 19:
        {
            cmd.u8_configClass  = WIRELESS_MCS_CHANGE;
            cmd.u8_configItem   = MCS_RC_QAM_SELECT;
            cmd.u32_configValue  = (param0);
            break;
        }
        
        case 20:
        {
            cmd.u8_configClass  = WIRELESS_MCS_CHANGE;
            cmd.u8_configItem   = MCS_RC_CODE_RATE_SELECT;
            cmd.u32_configValue  = (param0);
            break;
        }
        
        case 21:
        {
            cmd.u8_configClass  = WIRELESS_OTHER;
            cmd.u8_configItem   = CALC_DIST_ZERO_CALI;
            cmd.u32_configValue  = (param0);
            break;
        }

        case 22:
        {
            cmd.u8_configClass  = WIRELESS_OTHER;
            cmd.u8_configItem   = SET_CALC_DIST_ZERO_POINT;
            cmd.u32_configValue  = (param0);
            break;
        }

        case 23:
        {
            cmd.u8_configClass  = WIRELESS_OTHER;
            cmd.u8_configItem   = SET_RC_FRQ_MASK;
            cmd.u32_configValue  = (param0);
            break;
        }
        
        case 24:
        {
            cmd.u8_configClass  = WIRELESS_FREQ_CHANGE;
            cmd.u8_configItem   = IT_CHANNEL_FREQ;
            cmd.u32_configValue  = (param0);
            break;
        }
#endif
        case 25:
        {
            cmd.u8_configClass  = WIRELESS_MCS_CHANGE;
            cmd.u8_configItem   = MCS_CHG_RC_RATE;
            cmd.u32_configValue  = (param0);
            break;
        }
        
        case 26:
        {
            DLOG_Warning("rc rate:%d",BB_GetRcRate(param0));
            ret = 0;
            break;
        }
#if 0  
        case 27:
        {
            cmd.u8_configClass  = WIRELESS_OTHER;
            cmd.u8_configItem   = SET_PURE_VT_MODE;
            cmd.u32_configValue  = (param0);
            break;
        }
        
        case 28:
        {
            cmd.u8_configClass  = WIRELESS_OTHER;
            cmd.u8_configItem   = RW_BB_RF_REG;
            cmd.u32_configValue  = ((param3&0xFF)<<24) | ((param2&0xFF)<<16) | \
                                    ((param1&0xFF)<<8) | ((param0&0xFF)<<0);
            DLOG_Warning("%02x %02x %02x %02x %08x", param0,param1,param2,param3,cmd.u32_configValue);
            break;
        }
        
        case 29:
        {
            cmd.u8_configClass  = WIRELESS_OTHER;
            cmd.u8_configItem   = AGC_GEAR_CHG;
            cmd.u32_configValue  = (param0);
            break;
        }
#endif

        case 30:
        {
            cmd.u8_configClass  = WIRELESS_FREQ_CHANGE;
            cmd.u8_configItem   = RC_CHANNEL_SELECT;
            cmd.u32_configValue  = (param0);
            break;
        }
        
        case 31:
        {
            cmd.u8_configClass  = WIRELESS_FREQ_CHANGE;
            cmd.u8_configItem   = IT_CHANNEL_SELECT;
            cmd.u32_configValue  = (param0);
            break;
        }

        case 32:
        {
            cmd.u8_configClass  = WIRELESS_OTHER;
            cmd.u8_configItem   = SET_SWEEP_MODE;
            cmd.u32_configValue  = (param0);
            break;
        }

        case 34:
        {
            cmd.u8_configClass  = WIRELESS_ENCODER_CHANGE;
            cmd.u8_configItem   = ENCODER_TAKE_PICTURE;
            cmd.u32_configValue  = (param0);
            break;
        }

        case 35:
        {
            cmd.u8_configClass  = WIRELESS_ENCODER_CHANGE;
            cmd.u8_configItem   = ENCODER_PICTURE_QUALITY_SET;
            cmd.u32_configValue  = (param0);
            break;
        }

        case 36:
        {
            cmd.u8_configClass  = WIRELESS_OTHER;
            cmd.u8_configItem   = MISC_RC_FILTER_FUNC;
            cmd.u32_configValue = (param0);

			context.st_chSelectionParam.u8_rcChSelectionEnable = param0;
            //DLOG_Warning("filter enable: %d", param0);

            break;
        }

        case 37:
        {
            cmd.u8_configClass  = WIRELESS_OTHER;
            cmd.u8_configItem   = MISC_CHECK_LOCK_TIME;
            cmd.u32_configValue = (param0);			//reset times

			//DLOG_Warning("reset times: %d", param0);

            break;
        }
        
        case 38:
        {
            cmd.u8_configClass  = WIRELESS_OTHER;
            cmd.u8_configItem   = SET_NON_LBT;
            cmd.u32_configValue = (param0);			//reset times

			DLOG_Warning("non-lbt :  %d", param0);

            break;
        }

        default:
        {
            ret = 0;
            break;
        }
    }

    if(ret)
    {
       ret = BB_InsertCmd(0, &cmd);
    }

    return ret;
}

static void  __attribute__ ((section(".h264"))) BB_ExitSleepProc(void)
{
    if (wake_up_flag == 0x55)
    {
        BB_WriteRegMask(PAGE2, 0, 0, 1);
    }
    else if (wake_up_flag == 0xAA)
    {
        BB_WriteRegMask(PAGE1, 0xB6, 0xC0, 0xC0);
        RF_init(context.en_bbmode);
        RF_CaliProcess(context.en_bbmode);
        BB_after_RF_cali(context.en_bbmode);
        BB_set_power(context.e_curBand,context.pwr);
        context.u8_aocAdjustPwr = context.pwr;
        BB_softReset(context.en_bbmode);
        BB_set_RF_Band(context.en_bbmode, context.e_curBand);
    }
    wake_up_flag = 0;
}
static void __attribute__ ((section(".h264"))) BB_HandleEventsCallback(void *p)
{
    STRU_WIRELESS_CONFIG_CHANGE* pcmd = (STRU_WIRELESS_CONFIG_CHANGE* )p;
    uint8_t  class  = pcmd->u8_configClass;
    uint8_t  item   = pcmd->u8_configItem;
    uint32_t value  = pcmd->u32_configValue;
	uint32_t value1  = pcmd->u32_configValue1;
	uint8_t value2  = pcmd->u32_configValue2;
    if (class == WIRELESS_RF_POWER_CTRL)
    {
        if (item ==RF_POWER_CTRL_OUTPUT_LEVEL)
        {
            BB_SetTargetPower((value >>8)&0xff, value & 0xff);
        }
        else if (item ==RF_POWER_CTRL_STANDARD)
        {
            BB_SetTargetPowerWorkMode((ENUM_RF_POWER_WORK_MODE)(value & 0xff));
        }

        //in sky mode, update rf power to ground
        if ( context.en_bbmode == BB_SKY_MODE)
        {
            context.stru_bandSwitchParam.i8_skyPower0 = ((value >>8)&0xff);
            context.stru_bandSwitchParam.i8_skyPower1 = (value & 0xff);
            BB_Session0SendMsg(DT_NUM_BANDSWITCH_ENABLE, (uint8_t *)(&context.stru_bandSwitchParam), sizeof(context.stru_bandSwitchParam));
        }
    }
	else if(class == WIRELESS_RF_REMOTE_UPGRADE)
	{
		context.flag_in_upgrade = value;
		DLOG_Critical("flag_in_upgrade = %d", context.flag_in_upgrade);
	}
    else if (class == WIRELESS_DEBUG_CHANGE && item == 0 && (value == 0 || value == 1))
    {    
        uint8_t u8_debugMode = ((value == 0) ? TRUE:FALSE);

        if( context.u8_debugMode != u8_debugMode )
        {
            context.u8_flagdebugRequest = u8_debugMode | 0x80;
            BB_SPI_curPageWriteByte(0x01, 0x02);
            en_curPage = (BB_SPI_curPageReadByte(0x0) & 0xc0);
        }
        //DLOG_Info("Debug: %d \n", u8_debugMode);         
    }
    else
    {
        if((class == WIRELESS_OTHER) && (item == PWR_CTRL_SET))// && (BB_SKY_MODE == context.en_bbmode))
        {
            if((value == 0) || (value == 1))
            {
                BB_ExitSleepProc();
                //DLOG_Info("value:%x flag:%x", value, wake_up_flag);
            }
        }
		else if((class == WIRELESS_OTHER) && (item == WIRELESS_RF_PA_MODE))
		{
				context.e_powerMode = value;
				DLOG_Critical("e_powerMode: %d \n", context.e_powerMode); 
		}
		else if((class == WIRELESS_OTHER) && (item == SET_RC_PATTEN)){
				uint8_t msg[10]={0};
				int i=0;
				context.rf_info.rc_patten_set_by_usr=value;
				msg[0]=value1;
				msg[1]=value1>>8;
				msg[2]=value1>>16;
				msg[3]=value1>>24;
				msg[4]=value2;
				if(context.en_bbmode==BB_SKY_MODE)
				{
					if(value==1) sky_setRcpatten(msg);
				}
				else 
				{
					for(i=0;i<context.rf_info.rc_ch_patten_need_id_size;i++) 
					{
						context.rcChgPatten.patten_grd[i]=msg[i];
					}
					context.rcChgPatten.en_flag_grd=1;
					context.rcChgPatten.valid_grd=1;
					context.rcChgPatten.timeout_cnt_grd=context.sync_cnt+STATUS_CHG_DELAY;
				}
				DLOG_Critical("is_manul=%d,set rc patten %d:%d:%d:%d:%d",value,msg[0],msg[1],msg[2],msg[3],msg[4]); 
		}
		else if(class==WIRELESS_OTHER && item==AUTTO_BW_CHANGE){
				context.rf_bw.en_flag=1;
				context.rf_bw.valid=1;
				context.rf_bw.bw=value;
				context.rf_bw.timeout_cnt=context.sync_cnt+STATUS_CHG_DELAY;
		}
	
        int ret = BB_InsertCmd(0, (STRU_WIRELESS_CONFIG_CHANGE * )p);
    }
}

static void BB_SendRmoteEvent(void *p)
{
    BB_Session0SendMsg(DT_NUM_REMOTE_EVENT, (uint8_t *)p, SYS_EVENT_HANDLER_PARAMETER_LENGTH);
}


void BB_handle_misc_cmds(STRU_WIRELESS_CONFIG_CHANGE* pcmd)
{
    uint8_t class = pcmd->u8_configClass;
    uint8_t item  = pcmd->u8_configItem;

    uint8_t value  = (uint8_t)(pcmd->u32_configValue);
    uint8_t value1 = (uint8_t)(pcmd->u32_configValue >> 8);
    uint8_t value2 = (uint8_t)(pcmd->u32_configValue >> 16);
    uint8_t value3 = (uint8_t)(pcmd->u32_configValue >> 24);

    if(class == WIRELESS_MISC)
    {
        switch(item)
        {
            case MISC_READ_RF_REG:
            {
                uint8_t v;
                BB_SPI_curPageWriteByte(0x01, (value == 0)? 0x01 : 0x03);               //value2==0: write RF8003-0
                                                                                        //value2==1: write RF8003-1
                RF_SPI_ReadReg( (value1<<8 )|value2, &v);
                BB_SPI_curPageWriteByte(0x01,0x02);
                //DLOG_Info("RF read %d addrH=0x%0.2x addrL:%0.2x out:0x%0.2x", value, value1, value2, v);
                break;
            }

            case MISC_WRITE_RF_REG:
            {
                BB_SPI_curPageWriteByte(0x01, (value == 0)? 0x01 : 0x03);              //value2==0: write RF8003-0
                                                                                       //value2==1: write RF8003-1
                RF_SPI_WriteReg( (value1 << 8) | value2, value3);
                BB_SPI_curPageWriteByte(0x01,0x02);

                //DLOG_Info("RF write %d addr=0x%0.2x value=0x%0.2x", value, (value1 << 8) | value2, value3);
                break;
            }

            case MISC_READ_BB_REG:
            {
                uint8_t v = BB_ReadReg( (ENUM_REG_PAGES)value, (uint8_t)value1);
                //DLOG_Info("BB read PAGE=0x%0.2x addr=0x%0.2x value=0x%0.2x", value, value1, v);
                break;
            }

            case MISC_WRITE_BB_REG:
            {
                BB_WriteReg((ENUM_REG_PAGES)value, (uint8_t)value1, (uint8_t)value2);
                //DLOG_Info("BB write PAGE=0x%0.2x addr=0x%0.2x value=0x%0.2x", value, value1, value2);
                break;
            }

            case MICS_IT_ONLY_MODE:
            {
                BB_WriteReg(PAGE2, 0x02, 0x06);
                break;
            }
        }
    }
}


static void BB_GetRcIdVtIdFromFlash(uint8_t *pu8_rcid, uint8_t *pu8_vtid)
{
    uint32_t loop = 0;
    uint8_t flag_found = 0;
    volatile STRU_NV *pst_nv = (volatile STRU_NV *)SRAM_NV_MEMORY_ST_ADDR;

    while( loop++ < 500 && 0 == flag_found )
    {
        if ( 0x23178546 != pst_nv->st_nvMng.u32_nvInitFlag)
        {
            SysTicks_DelayMS(20);
        }
        else
        {
            flag_found = 1;
        }
    }

    if (flag_found)
    {
        memcpy((void *)pu8_rcid, (void *)(pst_nv->st_nvDataUpd.u8_nvBbRcId), RC_ID_SIZE);
        memcpy((void *)pu8_vtid, (void *)(pst_nv->st_nvDataUpd.u8_nvBbVtId), VT_ID_SIZE);
        memcpy((void *)context.chipid, (void *)(pst_nv->st_nvDataUpd.u8_nvChipId), RC_ID_SIZE);

        if (pst_nv->st_nvMng.u8_nvVld != TRUE)
        {
            DLOG_Warning("rcid null");
        }

        /*if(BB_SKY_MODE == context.en_bbmode)
        {
            memcpy((void *)pu8_rcid, (void *)context.chipid, RC_ID_SIZE);
        }*/

        DLOG_Critical("rc: 0x%x 0x%x 0x%x 0x%x 0x%x vt: 0x%x 0x%x", pu8_rcid[0], pu8_rcid[1], pu8_rcid[2], pu8_rcid[3], pu8_rcid[4], pu8_vtid[0], pu8_vtid[1]);
    }
}

/** 
 * @brief       
 * @param   
 * @retval      
 * @note      
 */
int BB_GetDevInfo(void)
{ 
    uint8_t u8_data;
    STRU_DEVICE_INFO *pst_devInfo = (STRU_DEVICE_INFO *)(DEVICE_INFO_SHM_ADDR);

    pst_devInfo->msg_id = 0x19;
    pst_devInfo->msg_len = sizeof(STRU_DEVICE_INFO) - sizeof(STRU_WIRELESS_MSG_HEADER);

    pst_devInfo->skyGround = context.en_bbmode;
    pst_devInfo->band = context.e_curBand;
    
    //pst_devInfo->bandWidth = context.CH_bandwidth;
    pst_devInfo->itHopMode = context.itHopMode;
    pst_devInfo->rcHopping = context.rcHopMode;
    pst_devInfo->adapterBitrate = context.qam_skip_mode;
    u8_data = BB_ReadReg(PAGE1, 0x8D);
    pst_devInfo->channel1_on = (u8_data >> 6) & 0x01;
    pst_devInfo->channel2_on = (u8_data >> 7) & 0x01;
    pst_devInfo->isDebug = context.u8_debugMode;
    if (BB_GRD_MODE == context.en_bbmode )
    {
        u8_data = BB_ReadReg(PAGE2, GRD_FEC_QAM_CR_TLV);
        pst_devInfo->itQAM = u8_data & 0x03;
        pst_devInfo->itCodeRate  = ((u8_data >>2) & 0x07);
        u8_data = BB_ReadReg(PAGE2, RX_MODULATION);
        pst_devInfo->bandWidth = (u8_data >> 0) & 0x07;
       
        u8_data = BB_ReadReg(PAGE2, TX_2);
        pst_devInfo->rcQAM = (u8_data >> 6) & 0x01;
        pst_devInfo->rcCodeRate = (u8_data >> 0) & 0x01;
    }
    else
    {
        u8_data = BB_ReadReg(PAGE2, TX_2);
        pst_devInfo->itQAM = (u8_data >> 6) & 0x03;
        pst_devInfo->bandWidth = (u8_data >> 3) & 0x07;
        pst_devInfo->itCodeRate  = ((u8_data >> 0) & 0x07);
        
        u8_data = BB_ReadReg(PAGE2, 0x09);
        pst_devInfo->rcQAM = (u8_data >> 0) & 0x01;
        pst_devInfo->rcCodeRate = (u8_data >> 2) & 0x01;
    }

    if(context.brc_mode == AUTO)
    {
        pst_devInfo->ch1Bitrates = context.qam_ldpc;
        pst_devInfo->ch2Bitrates = context.qam_ldpc;
    }
    else
    {
        pst_devInfo->ch1Bitrates = context.brc_bps[0];
        pst_devInfo->ch2Bitrates = context.brc_bps[1];
    }

    #ifdef RF_9363
    //if (context.e_curBand == RF_600M)
    {
        pst_devInfo->u8_itRegs[0] = context.stru_itRegs.frq1;
        pst_devInfo->u8_itRegs[1] = context.stru_itRegs.frq3;
        pst_devInfo->u8_itRegs[2] = context.stru_itRegs.frq4;
        pst_devInfo->u8_itRegs[3] = context.stru_itRegs.frq5;
        
        pst_devInfo->u8_rcRegs[0] = context.stru_rcRegs.frq1;
        pst_devInfo->u8_rcRegs[1] = context.stru_rcRegs.frq3;
        pst_devInfo->u8_rcRegs[2] = context.stru_rcRegs.frq4;        
        pst_devInfo->u8_rcRegs[3] = context.stru_rcRegs.frq5;   
    }
    #endif

    #ifdef RF_8003X
    //else
    {
        pst_devInfo->u8_itRegs[0] = context.stru_itRegs.frq4;
        pst_devInfo->u8_itRegs[1] = context.stru_itRegs.frq3;
        pst_devInfo->u8_itRegs[2] = context.stru_itRegs.frq2;
        pst_devInfo->u8_itRegs[3] = context.stru_itRegs.frq1;
        
        pst_devInfo->u8_rcRegs[0] = context.stru_rcRegs.frq4;
        pst_devInfo->u8_rcRegs[1] = context.stru_rcRegs.frq3;
        pst_devInfo->u8_rcRegs[2] = context.stru_rcRegs.frq2;
        pst_devInfo->u8_rcRegs[3] = context.stru_rcRegs.frq1; 
    }
    #endif
    
    pst_devInfo->switch_mode_2g_5g = context.st_bandMcsOpt.e_rfbandMode;
    pst_devInfo->pure_vt_valid = vt_info.valid;

    /*STRU_NV *pst_nv = (STRU_NV *)SRAM_NV_MEMORY_ST_ADDR;    
    memcpy((void *)pst_devInfo->rcId, (void *)(pst_nv->st_nvDataUpd.u8_nvBbRcId), RC_ID_SIZE);
    memcpy((void *)pst_devInfo->vtid, (void *)(pst_nv->st_nvDataUpd.u8_nvBbVtId), VT_ID_SIZE);
    memcpy((void *)pst_devInfo->chipId, (void *)(pst_nv->st_nvDataUpd.u8_nvChipId), RC_ID_SIZE);*/
    
    memcpy((void *)pst_devInfo->rcId, (void *)(context.rcid), RC_ID_SIZE);
    memcpy((void *)pst_devInfo->vtid, (void *)(context.vtid), VT_ID_SIZE);
    memcpy((void *)pst_devInfo->chipId, (void *)(context.chipid), RC_ID_SIZE);
    pst_devInfo->inSearching = context.inSearching;
    pst_devInfo->en_realtime = context.realtime_mode | (context.enable_non_lbt << 1);
    pst_devInfo->lna_status = context.lna_status | (context.st_mimo_mode.enum_lna_mode << 1) \
        | (context.bandedge_enable << 7) | (context.low_power_db << 3);  
    pst_devInfo->reserved[0] = context.pwr;
    pst_devInfo->sleep_level = context.sleep_level;

    //pst_devInfo->u8_startWrite = 0;
    //pst_devInfo->u8_endWrite = 1;    
}

/** 
 * @brief       
 * @param   
 * @retval      
 * @note      
 */
int BB_SwtichOnOffCh(uint8_t u8_ch, uint8_t u8_data)
{
    uint8_t u8_regVal;

    u8_regVal = BB_ReadReg(PAGE1, 0x8D);
    if ((0 == u8_ch) && (0 == u8_data))
    {
        u8_regVal &= ~(0x40); // channel1 
        BB_WriteReg(PAGE1, 0x8D, u8_regVal); 
    }
    else if ((0 == u8_ch) && (1 == u8_data))
    {
        u8_regVal |= (0x40); // channel1 
        BB_WriteReg(PAGE1, 0x8D, u8_regVal); 
    }
    else if ((1 == u8_ch) && (0 == u8_data))
    {
        u8_regVal &= ~(0x80); // channel2 
        BB_WriteReg(PAGE1, 0x8D, u8_regVal); 
    }
    else if ((1 == u8_ch) && (1 == u8_data))
    {
        u8_regVal |= (0x80); // channel2 
        BB_WriteReg(PAGE1, 0x8D, u8_regVal); 
    }
    else
    {
    }
}

int BB_GetRcId(uint8_t *pu8_rcId, uint8_t bufsize)
{
    if ( bufsize < RC_ID_SIZE)
    {
        return 1;
    }
    else
    {
        memcpy((void *)pu8_rcId, (uint8_t *)(SRAM_SHARE_FLAG_ST_ADDR + SHARE_FLAG_RC_ID_OFFSET), RC_ID_SIZE);
    }

    return 0;

}

/** 
 * @brief       get rc rate
 * @param       none
 * @retval      1: BPSK 1/2, uart max 32bytes
 *              2: QPSK 2/3, uart max 208bytes
 *              0: unknow qam/code_rate
 * @note        None
 */
uint32_t BB_GetRcRate(ENUM_BB_MODE en_mode)
{
    uint32_t ret = 0;
    uint8_t rate = 0;
    
    if (BB_SKY_MODE == en_mode)
    {
        rate = BB_ReadReg(PAGE2, 0x09) & 0x05;

        if (0 == rate)
        {
            ret = 1; // BPSK 1/2
        }
        else if(0x05 == rate)
        {
            ret = 2; // QPSK 2/3
        }
        else if (0x01 == rate)
        {
            ret = 3; // QPSK 1/2
        }
        else if(0x04 == rate)
        {
            ret = 4; // BPSK 2/3
        }
    }
    else if (BB_GRD_MODE == en_mode)
    {
        rate = BB_ReadReg(PAGE2, 0x04) & 0x41;

        if (0 == rate)
        {
            ret = 1; // BPSK 1/2
        }
        else if(0x41 == rate)
        {
            ret = 2; // QPSK 2/3
        }
        else if (0x40 == rate)
        {
            ret = 3; // QPSK 1/2
        }
        else if(0x1 == rate)
        {
            ret = 4; // BPSK 2/3
        }
    }

    return ret;
}

static void BB_AocInit(void)
{
    if (NULL != pstru_bb_aoc_boardcfg)
    {
        context.aoc.u16_snrThdL = pstru_bb_aoc_boardcfg->u16_snrThdL;
        context.aoc.u16_snrThdH = pstru_bb_aoc_boardcfg->u16_snrThdH;
        context.aoc.u8_agcThdL = pstru_bb_aoc_boardcfg->u8_agcThdL;
        context.aoc.u8_agcThdH = pstru_bb_aoc_boardcfg->u8_agcThdH;
        context.aoc.u8_PwrThdMin = pstru_bb_aoc_boardcfg->u8_PwrThdMin;
        context.aoc.u8_agcAvgCnt = pstru_bb_aoc_boardcfg->u8_agcAvgCnt;
        context.aoc.u8_snrAvgCnt = pstru_bb_aoc_boardcfg->u8_snrAvgCnt;
        context.aoc.u16_ldpcStacCnt = pstru_bb_aoc_boardcfg->u16_ldpcStacCnt;
        context.aoc.u16_ldpcThd = pstru_bb_aoc_boardcfg->u16_ldpcThd;
    }
    else
    {
        context.aoc.u16_snrThdL = 0x1900;      // 20db
        context.aoc.u16_snrThdH = 0x279F;      // 22db
        context.aoc.u8_agcThdL = 60;          //
        context.aoc.u8_agcThdH = 70;          //
        context.aoc.u8_PwrThdMin = 15;      // -3.25dB
        context.aoc.u8_agcAvgCnt = 64;
        context.aoc.u8_snrAvgCnt = 16;
        context.aoc.u16_ldpcStacCnt = 100;
        context.aoc.u16_ldpcThd = 100;
    }
}

static void BB_AesInit(ENUM_BB_MODE en_mode)
{
    uint8_t i;

    if (NULL != pstru_bb_aes_cfg)
    {
        if(BB_SKY_MODE == en_mode)
        {
            for (i = 0; i < 32; i++)
            {
                BB_WriteReg(PAGE3, 0x20 + i, pstru_bb_aes_cfg->u8_upLinkKeyArray[i]);
            }
            BB_WriteRegMask(PAGE1, 0x8E, ((!pstru_bb_aes_cfg->u8_upLinkSwitch) << 6), (1 << 6));
            context.aes_off = pstru_bb_aes_cfg->u8_upLinkSwitch == 0 ? 1 : 0;
            for (i = 0; i < 32; i++)
            {
                BB_WriteReg(PAGE2, 0x30 + i, pstru_bb_aes_cfg->u8_downLinkKeyArray[i]);
            }
            BB_WriteRegMask(PAGE0, 0x1F, (pstru_bb_aes_cfg->u8_downLinkSwitch << 1), (1 << 1));
        }
        else
        {
            for (i = 0; i < 32; i++)
            {
                BB_WriteReg(PAGE2, 0x30 + i, pstru_bb_aes_cfg->u8_upLinkKeyArray[i]);
            }
            BB_WriteRegMask(PAGE0, 0x1F, (pstru_bb_aes_cfg->u8_upLinkSwitch << 1), (1 << 1));

            for (i = 0; i < 32; i++)
            {
                BB_WriteReg(PAGE3, 0x20 + i, pstru_bb_aes_cfg->u8_downLinkKeyArray[i]);
            }
            BB_WriteRegMask(PAGE1, 0x8E, ((!pstru_bb_aes_cfg->u8_downLinkSwitch) << 6), (1 << 6));
        }
    }
    else
    {
        if(BB_SKY_MODE == en_mode)
        {
            BB_WriteRegMask(PAGE1, 0x8E, 1, (1 << 6)); // 0: aes decryption enable

            BB_WriteRegMask(PAGE0, 0x1F, 0, (1 << 1));
        }
        else
        {
            BB_WriteRegMask(PAGE0, 0x1F, 0, (1 << 1)); // 1: aes encryption enable

            BB_WriteRegMask(PAGE1, 0x8E, 1, (1 << 6));
        }
    }
}


int BB_GetDtInfo(ENUM_DT_NUM e_num, uint8_t *len, uint32_t *addr, uint8_t *idx)
{
    int ret = -1;
    uint8_t i = 0;
    uint8_t index = 0;
    uint8_t offset = 0;
    uint8_t size = sizeof(g_dtLen) / sizeof(g_dtLen[0]);

    while(index < size)
    {
        if (e_num == (g_dtLen[index].num))
        {
            if (NULL != idx)
            {
                *idx = index;
            }
            if (NULL != len)
            {
                *len = g_dtLen[index].len;
            }

            while(i < index)
            {
                offset += g_dtLen[i].len;
                i += 1;
            }

            if (NULL != addr)
            {
                *addr = (DT_ADDR + offset);
            }

            ret = 0;
        }
        index++;
    }

    return ret;
}

int BB_DtSendToBuf(ENUM_DT_NUM e_num, uint8_t *arg)
{
    int ret = -1;
    uint32_t addr;
    uint8_t len;
    uint8_t idx;

    if((BB_GRD_MODE == context.en_bbmode) && (1 == vt_info.valid))
    {
        return ret;
    }

    if(0 == BB_GetDtInfo(e_num, &len, &addr, &idx))
    {
        g_dtLen[idx].flag = 1;
        memcpy((uint32_t *)addr, arg, len);
        ret = 0;
    }
    return ret;
}

int BB_DtStopSend(ENUM_DT_NUM e_num)
{
    int ret = -1;
    uint8_t index = 0;
    uint8_t size = sizeof(g_dtLen) / sizeof(g_dtLen[0]);

    while(index < size)
    {
        if (e_num == (g_dtLen[index].num))
        {
            g_dtLen[index].flag = 0;
            ret = 0;
        }
        index++;
    }

    return ret;
}

int BB_Session0SendMsg(uint8_t id, uint8_t* data_buf, uint16_t len)
{
    uint8_t  data[128];
    uint8_t  i = 0;

    if (len + 3 > sizeof(data))
    {
        DLOG_Error("size flow");
    }

    data[0] = 0xa5;
    data[1] = len+1;                //len = id + size(data)
    data[2] = id;

    if (len > 0)
    {
        memcpy(data+3, data_buf, len);
    }

    //return 0;
    BB_ComSendMsg(BB_COM_SESSION_0, data, len + 3);
}


int BB_Session0RecvMsg(uint8_t* data_buf, uint16_t buflen)
{
    uint8_t i = 0;

    //find the head and len
    if (u16_session0DataSize < 3)
    {
        //read more data, packet is not complete
        u16_session0DataSize += BB_ComReceiveMsg(BB_COM_SESSION_0, u8_session0RecvBuf+u16_session0DataSize, sizeof(u8_session0RecvBuf) - u16_session0DataSize);        
    }

    if (u16_session0DataSize < 3)
    {
        return 0;
    }

    uint8_t lable    = u8_session0RecvBuf[0];
    uint8_t data_len = u8_session0RecvBuf[1];

    if (lable != 0xa5 || data_len + 2 > u16_session0DataSize)  //2: head, len
    {
        DLOG_Error("-1: %d %d %d %d", lable, u16_session0DataSize, data_len, buflen);
        u16_session0DataSize = 0;
        return -1;
    }

    //copy only valid data
    memcpy(data_buf, u8_session0RecvBuf+2, data_len);
    u16_session0DataSize -= (data_len +2);              //data_len = sizeof(data) + id
    if (u16_session0DataSize > 0)
    {
        //DLOG_Warning("copy left %d %d", u8_session0RecvBuf[data_len +2], u16_session0DataSize);
        memcpy(u8_session0RecvBuf, u8_session0RecvBuf + (data_len +2), u16_session0DataSize);
    }

    return data_len;
}

void BB_DtSentToSession(void)
{
    uint32_t addr;
    uint8_t len;
    uint8_t index = 0;
    uint8_t size = sizeof(g_dtLen) / sizeof(g_dtLen[0]);
    while(index < size) // high priority
    {
        if ((1 == (g_dtLen[index].priority)) && (1 == (g_dtLen[index].flag)))
        {
            if(0 == BB_GetDtInfo(g_dtLen[index].num, &len, &addr, NULL))
            {
                if (0 == BB_Session0SendMsg(g_dtLen[index].num, (uint8_t *)addr, len))
                {
                    return;
                }
                if(0 == g_dtLen[index].ack)
                {
                    g_dtLen[index].flag = 0;
                }
            }
        }
        index += 1;
    }

    index = 0;
    while(index < size) // low priority
    {
        if ((0 == (g_dtLen[index].priority)) && (1 == (g_dtLen[index].flag)))
        {
            if(0 == BB_GetDtInfo(g_dtLen[index].num, &len, &addr, NULL))
            {
                if (0 == BB_Session0SendMsg(g_dtLen[index].num, (uint8_t *)addr, len))
                {
                    return;
                }
                if(0 == g_dtLen[index].ack)
                {
                    g_dtLen[index].flag = 0;
                }
            }
        }
        index += 1;
    }
}

void  BB_SetTrxMode(ENUM_BB_TRX_MODE mode)
{
    BB_WriteRegMask(PAGE0, 0x20, (uint8_t)mode, 0x03);
}
/*
 * 
*/
int BB_SetTargetPower(uint8_t u8_2g_power,uint8_t u8_5g_power)
{
    context.u8_TargetPower[0] = u8_2g_power;
    context.u8_TargetPower[1] = u8_5g_power;
    context.pwr = BB_get_band_power(context.e_curBand);
    BB_set_power(context.e_curBand, context.pwr);
    context.u8_aocAdjustPwr = context.pwr;

    DLOG_Warning("%d", context.pwr);
    return 0;
}

int BB_SetTargetPowerWorkMode(ENUM_RF_POWER_WORK_MODE e_power_work_mode)
{
    if(e_power_work_mode == context.e_powerWorkMode)
    {
        return 0;
    }
    context.e_powerWorkMode = e_power_work_mode;
    BB_load_fct_power();
    
    context.pwr = BB_get_band_power(context.e_curBand);
    BB_set_power(context.e_curBand, context.pwr);
    context.u8_aocAdjustPwr = context.pwr;

    DLOG_Warning("%d", context.pwr);
    return 0;
}

int BB_NormalModePcRwReg(void *p)
{
    uint16_t addr = 0;
    STRU_REG_FLAG *pflag = (STRU_REG_FLAG *)(p);
    STRU_BB_REG_CMD *pBbCmd = (STRU_BB_REG_CMD *)(p);
    STRU_RF_REG_CMD *pRfCmd = (STRU_RF_REG_CMD *)(p);
    STRU_BB_REG_CMD rBbCmd = {
        
        .flag.subType = 2,
        .flag.rsv = 0,
        .flag.rfCh = 0,
        .flag.type = 0,
        .flag.dir = 1,
        .page = pBbCmd->page,
        .addr = pBbCmd->addr,
        .value = 0,
    };
    STRU_RF_REG_CMD rRfCmd = {
        
        .flag.subType = 2,
        .flag.rsv = 0,
        .flag.rfCh = pRfCmd->flag.rfCh,
        .flag.type = 1,
        .flag.dir = 1,
        .addr_h = pRfCmd->addr_h,
        .addr_l = pRfCmd->addr_l,
        .value = 0,
    };

    //DLOG_Warning("flag:%x sizeof STRU_BB_REG_CMD:%d", *pflag, sizeof(STRU_BB_REG_CMD));

    if(pflag->dir)
    {
        return -1;
    }

    if(pflag->type) // rf
    {
        /*DLOG_Warning("flag:%x subType:%d rsv:%d rfCh:%d type:%d dir:%d addr_h:%x addr_l:%x value:%x",\
                    pRfCmd->flag,
                    pRfCmd->flag.subType,
                    pRfCmd->flag.rsv,
                    pRfCmd->flag.rfCh,
                    pRfCmd->flag.type,
                    pRfCmd->flag.dir,
                    pRfCmd->addr_h,
                    pRfCmd->addr_l,
                    pRfCmd->value);*/
        
        addr = ((pRfCmd->addr_h << 8) | (pRfCmd->addr_l));
        if (pflag->subType) // rf read
        {
            //DLOG_Warning("1*value:%x", rRfCmd.value);
            RF_PcReadReg((uint8_t)(pRfCmd->flag.rfCh), addr, &(rRfCmd.value));
            //DLOG_Warning("2*value:%x", rRfCmd.value);
            SYS_EVENT_NotifyInterCore(SYS_EVENT_ID_USER_CFG_CHANGE, (void *)(&rRfCmd));
        }
        else // rf write
        {
            RF_PcWriteReg((uint8_t)(pRfCmd->flag.rfCh), addr, pRfCmd->value);
        }
    }
    else // bb
    {
        /*DLOG_Warning("flag:%x subType:%d rsv:%d rfCh:%d type:%d dir:%d page:%x addr:%x value:%x",
                                pBbCmd->flag,
                                pBbCmd->flag.subType,
                                pBbCmd->flag.rsv,
                                pBbCmd->flag.rfCh,
                                pBbCmd->flag.type,
                                pBbCmd->flag.dir,
                                pBbCmd->page,
                                pBbCmd->addr,
                                pBbCmd->value);*/
        
        if (pflag->subType) // rf read
        {
            //DLOG_Warning("1*value:%x", rBbCmd.value);
            rBbCmd.value = BB_ReadReg((ENUM_REG_PAGES)(pBbCmd->page), pBbCmd->addr);
            //DLOG_Warning("2*value:%x", rBbCmd.value);
            SYS_EVENT_NotifyInterCore(SYS_EVENT_ID_USER_CFG_CHANGE, (void *)(&rBbCmd));
        }
        else // rf write
        {
            BB_WriteReg((ENUM_REG_PAGES)(pBbCmd->page), pBbCmd->addr, pBbCmd->value);
        }
    }

    return 0;
}
void BB_Register_lna(pfun open_lna_function, pfun bypass_lna_function)
{
    pfun_lna_open = open_lna_function;
    pfun_lna_bypass = bypass_lna_function;
}
void BB_Register_freq2g5g_switch(pfun open_2g_function, pfun open_5g_function)
{
    pfun_f2g_open = open_2g_function;
    pfun_f5g_open = open_5g_function;
}

void BB_Register_fem_cb(pfun_fem fem_cb_open,pfun_fem fem_cb_close)
{
    pfun_fem_open = fem_cb_open;
    pfun_fem_close = fem_cb_close;
}
void BB_fem_open(uint8_t ch)
{
    if(NULL == pfun_fem_open)
    {
//        DLOG_Warning("null");
        return;
    }
    pfun_fem_open(ch);
}
void BB_fem_close(uint8_t ch)
{
    if(NULL == pfun_fem_close)
    {
 //       DLOG_Warning("null");
        return;
    }
    pfun_fem_close(ch);
}

void BB_Enable_RcRandomSkipPatten(void)
{
    context.enable_rc_random_skip_patten = 1;
}

void BB_config_uplink_qam_mode(uint8_t val)
{
    context.uplink_qam_mode = val;
}
void BB_open_lna(void)
{
    if(NULL == pfun_lna_open)
    {
 //       DLOG_Warning("null");
        return;
    }

    #ifdef USE_EXTERN_LNA
    BB_WriteReg(PAGE0, 0xa0, 0x33);//lna open, makeup loss by adjust 8003x gain
    pfun_lna_open();
    DLOG_Warning("");
    #endif
}
void BB_bypass_lna(void)
{
    if(NULL == pfun_lna_bypass)
    {
 //       DLOG_Warning("null");
        return;
    }

    #ifdef USE_EXTERN_LNA
    BB_WriteReg(PAGE0, 0xa0, 0x24);//lna close, makeup loss by adjust 8003x gain
    pfun_lna_bypass();
    DLOG_Warning("ok");
    #endif
}
uint8_t BB_RssiOffset(uint8_t rssi)
{
    if(context.e_curBand == RF_2G)
    {
        return context.lna_status == BYPASS_LNA ? (rssi > pstru_lna_auto->u8_2g_lna ? rssi - pstru_lna_auto->u8_2g_lna : 0) : rssi;
    }
    else if(context.e_curBand == RF_5G)
    {
        return context.lna_status == BYPASS_LNA ? (rssi > pstru_lna_auto->u8_5g_lna ? rssi - pstru_lna_auto->u8_5g_lna : 0) : rssi;
    }
}
void BB_Lna_AddAgc(uint8_t agca, uint8_t agcb)
{
    if(NULL == pstru_lna_auto)
    {
        DLOG_Warning("null");
        return;
    }
    
    if(stru_agc2lna.isFull)//move agc avg
    {
        stru_agc2lna.sum_a -= stru_agc2lna.u8_agcaBuf[stru_agc2lna.mindex];
        stru_agc2lna.sum_b -= stru_agc2lna.u8_agcbBuf[stru_agc2lna.mindex];
    }

    agca = BB_RssiOffset(agca);
    agcb = BB_RssiOffset(agcb);
    
    stru_agc2lna.sum_a += agca;
    stru_agc2lna.sum_b += agcb;
    stru_agc2lna.u8_agcaBuf[stru_agc2lna.mindex] = agca;
    stru_agc2lna.u8_agcbBuf[stru_agc2lna.mindex] = agcb;
    stru_agc2lna.mindex++;
    if(stru_agc2lna.mindex >= pstru_lna_auto->u16_avgCnt)
    {
        stru_agc2lna.mindex = 0;
        stru_agc2lna.isFull = 1;
        DLOG_Warning("a=%d,b=%d",stru_agc2lna.sum_a/pstru_lna_auto->u16_avgCnt,stru_agc2lna.sum_b/pstru_lna_auto->u16_avgCnt);
    }

}

void BB_Lna_reset(void)
{
    memset(&stru_agc2lna,0,sizeof(stru_agc2lna));
    DLOG_Warning("");
}

ENUM_LNA_STATUS BB_Lna_isNeedSwitch(ENUM_RF_BAND band)
{
    uint8_t avg_a,avg_b;
	
    if(context.swp_bypass == 1)
    {
        return BYPASS_LNA;
    }
	
    if(!stru_agc2lna.isFull)
    {
        return INVALID_LNA;
    }

    avg_a = (stru_agc2lna.sum_a / pstru_lna_auto->u16_avgCnt) ;
    avg_b = (stru_agc2lna.sum_b / pstru_lna_auto->u16_avgCnt) ;
    avg_a = avg_a > avg_b ? avg_b : avg_a;
    if(band == RF_2G)
    {
        if(avg_a < pstru_lna_auto->u8_2g_agcThdL)
        {
            return BYPASS_LNA;
        }
        else if(avg_a > pstru_lna_auto->u8_2g_agcThdH)
        {
            return OPEN_LNA;
        }
        else
        {
            return INVALID_LNA;
        }
    }
    else if(band == RF_5G)
    {
        if(avg_a < pstru_lna_auto->u8_5g_agcThdL)
        {
            return BYPASS_LNA;
        }
        else if(avg_a > pstru_lna_auto->u8_5g_agcThdH)
        {
            return OPEN_LNA;
        }
        else
        {
            return INVALID_LNA;
        }
    }
    else
    {
        return INVALID_LNA;
    }
}

void BB_GenHashId(uint8_t *rcid, uint8_t *vtid,ENUM_BB_MODE en_mode)
{
    uint32_t crc;
    uint8_t id[RC_ID_SIZE+VT_ID_SIZE];

    memcpy((uint8_t *)id,(uint8_t *)rcid,RC_ID_SIZE);
    memcpy((uint8_t *)(id+RC_ID_SIZE),(uint8_t *)vtid,VT_ID_SIZE);
    if(en_mode == BB_SKY_MODE)
    {
        crc = crc16(id,RC_ID_SIZE+VT_ID_SIZE);
        context.hashVtid[0] = crc >> 8;
        context.hashVtid[1] = crc;
        DLOG_Warning("vtid %x:%x",context.hashVtid[0],context.hashVtid[1]);
    }
    else
    {
        crc = crc32(id,RC_ID_SIZE+VT_ID_SIZE);
        context.hashRcid[0] = crc >> 24;
        context.hashRcid[1] = crc >> 16;
        context.hashRcid[2] = crc >> 8;
        context.hashRcid[3] = crc;
        context.hashRcid[4] = rcid[0];
        DLOG_Warning("rcid %x:%x:%x:%x:%x",context.hashRcid[0],context.hashRcid[1],context.hashRcid[2],context.hashRcid[3],context.hashRcid[4]);
    }
}

void __attribute__ ((section(".h264")))sky_setRcpatten(uint8_t *msg)
{
	int i=0;
	for(i=0;i<context.rf_info.rc_ch_patten_need_id_size;i++) 
	{
		context.rcChgPatten.patten[i]=msg[i];
	}
	context.rcChgPatten.en_flag=1;
	context.rcChgPatten.valid=1;
	context.rcChgPatten.timeout_cnt=context.sync_cnt+STATUS_CHG_DELAY;
	DLOG_Critical("manul set patten,cnt=%d,aim_cnt=%d",context.sync_cnt,context.rcChgPatten.timeout_cnt);
}

void __attribute__ ((section(".h264")))selectionSortBy(STRU_RF_DATA *a,int len,STRU_RF_DATA *b,int by)//b is the org list,and a is the result list
{

	int min_Index,i,j;
	STRU_RF_DATA temp={0};
	for(i=0;i<len;i++){
		a[i].value=b[i].value;
		a[i].id=b[i].id;
	}
	for(i=0;i<len;i++){
		min_Index = i;
		for(j=i+1;j<len;j++){
			if(by==1){
				if(a[j].value<a[min_Index].value){
					min_Index=j;
				}
			}
			else{
				if(a[j].id<a[min_Index].id){
					min_Index=j;
				}
			}
		}
		if(min_Index !=i){
			temp.value =a[i].value;
			temp.id = a[i].id;
			a[i].value=a[min_Index].value;
			a[i].id=a[min_Index].id;
			a[min_Index].value=temp.value;
			a[min_Index].id=temp.id;
		}
	}
	
}

signed char __attribute__ ((section(".h264")))vector_1xn_nx1_caculate(uint8_t *a,signed char *b,uint8_t len,uint8_t precision){
	int i=0;
	int r=0;
	for(i=0;i<len;i++)
	{
		r+=a[i]*b[i];
	}
	int v1 = r/precision;
	if(v1<0){
		int v2 = (abs(r)%precision)>5 ? -1 : 0;
		return (v1+v2);
	}else{
		int v3 = (abs(r)%precision)>5 ? 1 : 0;
		return (v1+v3);
	}

	
}

uint8_t __attribute__ ((section(".h264")))dec2bit_index(uint8_t d)
{
	uint8_t r;
	switch(d){
		case 0: r=0x01;break;
		case 1: r=0x02;break;
		case 2: r=0x04;break;
		case 3: r=0x08;break;
		case 4: r=0x10;break;
		case 5: r=0x20;break;
		case 6: r=0x40;break;
		case 7: r=0x80;break;
		default: r=0x00;break;
		
	}
	return (r);
}

static void reset_working_table_statistics()
{
	uint8_t row=0;
	uint8_t ch=0;
	for(ch=0;ch<MAX_RC_FRQ_SIZE;ch++){
		//context.rf_info.work_snr_avrg_value[ch].value=0;
		context.rf_info.work_rc_error_value[ch].value=0;
		//context.rf_info.work_snr_fluct_value[ch].value=0;
	    for (row = 0; row < SWEEP_FREQ_BLOCK_ROWS; row++){
	        context.rf_info.work_rc_unlock_table[row][ch].value=0;
			//context.rf_info.work_snr_table[row][ch].value=0;
	    }
	}
}

void __attribute__ ((section(".h264")))rc_set_unlock_patten(void)
{
	context.rcChgPatten.patten[0]=0x81;
	context.rcChgPatten.patten[1]=0x41;
	context.rcChgPatten.patten[2]=0x00;
	context.rcChgPatten.patten[3]=0x00;
	context.rcChgPatten.patten[4]=0x02;
	DLOG_Error("go to common patten,cnt=%d",context.sync_cnt);
	
	if(context.st_bandMcsOpt.e_bandwidth ==BW_20M){
		context.st_bandMcsOpt.e_bandwidth = BW_10M;
		reset_sweep_table(context.e_curBand);
		RF8003s_GetFctFreqTable(context.st_bandMcsOpt.e_bandwidth);
		if(context.en_bbmode==BB_SKY_MODE){
			
			BB_set_RF_bandwitdh(BB_SKY_MODE, BW_10M);
		}else{
			BB_set_RF_bandwitdh(BB_GRD_MODE, BW_10M);
		}
		DLOG_Error("e_bandwidth=%d",context.st_bandMcsOpt.e_bandwidth);
	}
	
	rc_update_working_patten();
}
void __attribute__ ((section(".h264")))rc_update_working_patten(void)
{
	uint8_t i=0,j=0,k=0;
	uint8_t id=1;
	
	for(i=0;i<context.rf_info.rc_ch_patten_need_id_size;i++)
	{
		id =0x01;
		for(j=0;j<8;j++)
		{
			if((id & context.rcChgPatten.patten[i]) == id)
			{
				context.rf_info.rc_ch_working_patten[k] = i*8+j;
				DLOG_Warning("syncnt=%d,new_patten[%d]=%d,freq=%d,frq_size=%d",context.sync_cnt, k,context.rf_info.rc_ch_working_patten[k],BB_GetRcFrqByCh(context.rf_info.rc_ch_working_patten[k]),BB_GetRcFrqNum(context.e_curBand));
				k++;
			}
			id = id <<1;
		}
	}
	reset_working_table_statistics();
	context.rf_info.rc_ch_working_patten_size = k;
	context.rc_have_update_list = 1;
}
void __attribute__ ((section(".h264")))bb_get_rc_channel()
{
	uint8_t id = 1,channel=0;
	uint8_t i=0,offset=0,compare_segment;
	context.rc_eq_channel++;
	if(context.rc_eq_channel >= context.rf_info.rc_ch_working_patten_size)
    {
        context.rc_eq_channel = 0;
    }
	channel = context.rf_info.rc_ch_working_patten[context.rc_eq_channel];
	#if 0
	if(context.rc_have_update_list==1)
	{
		context.rc_have_update_list=0;
		DLOG_Warning("patten_size=%d,freqch[%d]=%d",
			context.rf_info.rc_ch_working_patten_size,
			channel,
			BB_GetRcFrqByCh(channel)
			);
	}
	#endif
	context.sky_rc_channel = channel;
	context.grd_rc_channel = channel;
	
}
void __attribute__ ((section(".h264")))bb_update_rc_patten_size()
{
	//int mod = BB_GetRcFrqNum(context.e_curBand)%8;
	//context.rf_info.rc_ch_patten_need_id_size= BB_GetRcFrqNum(context.e_curBand)/8+(mod >0);
	context.rf_info.rc_ch_patten_need_id_size=5;
	DLOG_Critical("rc_ch_patten_need_id_size=%d",context.rf_info.rc_ch_patten_need_id_size);
	
}

void __attribute__ ((section(".h264")))CalcAverageSweepPower(uint8_t ch){
    uint8_t row;
	int temp = 0;
	int tempfluct = 0;
    //get average power
	signed char buffer[SWEEP_FREQ_BLOCK_ROWS]={0};
	for(row=0;row<SWEEP_FREQ_BLOCK_ROWS;row++){
		
		buffer[row]=context.rf_info.sweep_pwr_table[row][ch].value;
		if(row < SWEEP_FREQ_BLOCK_ROWS-1){
			tempfluct +=abs(context.rf_info.sweep_pwr_table[row+1][ch].value-context.rf_info.sweep_pwr_table[row][ch].value);
		}  
	}
	int v1 = tempfluct/(SWEEP_FREQ_BLOCK_ROWS-1);
	int v2 =  (tempfluct%(SWEEP_FREQ_BLOCK_ROWS-1) > 5) ? 1:0;
	context.rf_info.sweep_pwr_fluct_value[ch].value=v1+v2;
	context.rf_info.sweep_pwr_avrg_value[ch].value=vector_1xn_nx1_caculate(vector_pwr_avrg_time_r,buffer,SWEEP_FREQ_BLOCK_ROWS,PRECIESE);
}
void  reset_sweep_table(ENUM_RF_BAND cur_band){
	int i=0,j=0;
    DLOG_Warning("cur_band=%d,en_bbmode=%d",cur_band,context.en_bbmode);
	if(context.en_bbmode==BB_SKY_MODE){
		if(cur_band==RF_2G){
			context.rf_info.sweepBand[0] = RF_2G;
			context.rf_info.sweep_freqsize = BB_GetSkySweepFrqNum(RF_2G);
	        context.rf_info.bandCnt = 1;
			context.rf_info.rc_avr_sweep_result_size =  context.rf_info.sweep_freqsize;
		}else if(cur_band==RF_5G){
			context.rf_info.sweepBand[0] = RF_5G;
	        context.rf_info.sweep_freqsize = BB_GetSkySweepFrqNum(RF_5G);
	        context.rf_info.bandCnt = 1;
			context.rf_info.rc_avr_sweep_result_size = context.rf_info.sweep_freqsize;
		}else if(cur_band==RF_2G_5G){
			context.rf_info.sweepBand[0] = RF_2G;
	        context.rf_info.sweepBand[1] = RF_5G;
			context.rf_info.sweep_freqsize = BB_GetSkySweepFrqNum(RF_2G);
	        context.rf_info.bandCnt = 2;
			context.rf_info.rc_avr_sweep_result_size =  context.rf_info.sweep_freqsize;
		}
		else{
			context.rf_info.bandCnt = 1;
	        context.rf_info.sweepBand[0]    = context.st_bandMcsOpt.e_bandsupport;
	        context.rf_info.sweep_freqsize = BB_GetSkySweepFrqNum(cur_band);
			context.rf_info.rc_avr_sweep_result_size = context.rf_info.sweep_freqsize;
		}
	}
	else if(context.en_bbmode==BB_GRD_MODE){
		context.rf_info.fine_sweep_size=4;
		if (cur_band == RF_2G_5G)
	    {
	        context.rf_info.u8_bb1ItFrqSize = BB_GetItFrqNum(RF_2G);
	        context.rf_info.u8_bb2ItFrqSize = BB_GetItFrqNum(RF_5G);
			context.rf_info.sweep_freqsize = context.rf_info.u8_bb1ItFrqSize;
	        context.e_curBand = RF_2G;
	    }
	    else if (RF_5G == cur_band)
	    {
	        context.rf_info.u8_bb1ItFrqSize = BB_GetItFrqNum(RF_5G);
			context.rf_info.sweep_freqsize = context.rf_info.u8_bb1ItFrqSize;
	        context.e_curBand = RF_5G;
	    }
	    else if (RF_2G == cur_band)
	    {
	        context.rf_info.u8_bb1ItFrqSize = BB_GetItFrqNum(RF_2G);
			context.rf_info.sweep_freqsize = context.rf_info.u8_bb1ItFrqSize;
	        context.e_curBand = RF_2G;
	    }
	}

	for(i=0;i<context.rf_info.sweep_freqsize;i++)
	{
		context.rf_info.prelist[i].id=i;context.rf_info.prelist[i].value=0;
		context.rf_info.pre_selection_list[i].id=i;context.rf_info.pre_selection_list[i].value=0;
		context.rf_info.sweep_pwr_avrg_value[i].id=i;context.rf_info.sweep_pwr_avrg_value[i].value=0;
		context.rf_info.sweep_pwr_fluct_value[i].id=i;context.rf_info.sweep_pwr_fluct_value[i].value=0;
		context.rf_info.work_rc_error_value[i].id=i;context.rf_info.work_rc_error_value[i].value=0;
		//context.rf_info.work_snr_avrg_value[i].id=i;context.rf_info.work_snr_avrg_value[i].value=0;
		//context.rf_info.work_snr_fluct_value[i].id=i;context.rf_info.work_snr_fluct_value[i].value=0;
		context.rf_info.error_ch_record[i]=0xff;
		for(j=0;j<SWEEP_FREQ_BLOCK_ROWS;j++)
		{
			context.rf_info.sweep_pwr_table[j][i].id=i;context.rf_info.sweep_pwr_table[j][i].value=0;
			context.rf_info.work_rc_unlock_table[j][i].id=i;context.rf_info.work_rc_unlock_table[j][i].value=0;
			//context.rf_info.work_snr_table[j][i].id=i;context.rf_info.work_snr_table[j][i].value=0;
		}
	}
	context.rf_info.fine_sweep_id=0;
	context.rf_info.curBandIdx = 0;
	context.rf_info.curSweepCh=0;
	context.rf_info.curRowCnt  = 0;
	context.rf_info.currc_statistics_Row=0;
	context.rf_info.sweep_finished=1;
	context.rf_info.lock_sweep=0;
	context.rf_info.fine_sweep_size=8;
	context.rf_info.fine_sweep_id=0;
	context.rf_info.fine_sweep_row=0;
	context.rf_info.isFull = 0;
	context.rf_info.sweep_cycle=0;
	context.rf_info.rc_ch_working_patten_len=SKY_PATTEN_SIZE_2G;
	context.rf_info.rc_ch_dynamic_working_patten_max_len=SKY_PATTEN_MAX_Dynamic_SIZE_2G;
	bb_update_rc_patten_size();

}
  
