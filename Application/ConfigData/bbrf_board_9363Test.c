#include <string.h>
#include "boardParameters.h"
#include "cfg_parser.h"
#include "bb_types.h"
#include "factory.h"
#include "hal_uart.h"

/////////////////////////////////////////////
extern STRU_BOARD_BB_PARA stru_9363Test_bb_boardCfg;

STRU_cfgNode AR8020_9363Test_bbcfg_nodeInfo= 
{
    .nodeId       = BB_BOARDCFG_PARA_ID,
    .nodeElemCnt  = 1,
    .nodeDataSize = sizeof(stru_9363Test_bb_boardCfg)
};

STRU_BOARD_BB_PARA stru_9363Test_bb_boardCfg __attribute__ ((aligned (4)))= 
{
    .u8_bbSkyRegsCnt    = 46,
    
    .u8_bbGrdRegsCnt    = 51,

     //after calibration
    .u8_bbSkyRegsCntAfterCali    = 3,

    .u8_bbGrdRegsCntAfterCali    = 2,
};



/////////////////////////////////////////////

extern STRU_BB_REG Test9363_bb_regCfg[102];

STRU_cfgNode Test9363_nodeInfo =
{
    .nodeId       = BB_BOARDCFG_DATA_ID,
    .nodeElemCnt  = 102,
    .nodeDataSize = sizeof(Test9363_bb_regCfg)
};


STRU_BB_REG Test9363_bb_regCfg[102] __attribute__ ((aligned (4)))= 
{
    //AR8020TEST9363_bb_skyregs
    {0, 0x90, 0x34},
    {0, 0x91, 0x40},
    {0, 0x92, 0x30},
    {0, 0x93, 0x30},
    {0, 0x96, 0x28},
    {0, 0x97, 0xc0},
    {0, 0x98, 0x14},
    {0, 0x9c, 0x1c},
    {0, 0x9d, 0x30},

    {0, 0xa0, 0x36},
    {0, 0xa1, 0xc4},    
    {0, 0xa2, 0x00},
    {0, 0x9d, 0x00},
    {0, 0x9e, 0x00},
    {0, 0x9f, 0x00},

    {0, 0xb0, 0x80},
    {0, 0xb1, 0x08},    
    {0, 0xb3, 0x80},
    {0, 0xb4, 0x08},
    {0, 0xbc, 0x00},
    {0, 0xbd, 0x00},    
    {0, 0xbe, 0x00},
    {0, 0xbf, 0x00},

    {0, 0xc6, 0x00},
    {0, 0xc7, 0x00},
    {0, 0xc8, 0x00},
    {0, 0xc9, 0x00},
    {0, 0xca, 0x00},
    {0, 0xcd, 0x00},
    {0, 0xce, 0x00},
    {0, 0xcf, 0x00},

    {1, 0x89, 0x99},
    
    {2, 0x10, 0x8c},
    {2, 0x13, 0x00}, 
    {2, 0x1a, 0x8c},
    {2, 0x1b, 0x00},  
    {2, 0x1c, 0x00},
    {2, 0x1d, 0x00},  
    {2, 0x1f, 0x20},
    {2, 0x20, 0x00},
    {2, 0x21, 0x20},
    {2, 0x2d, 0x00},
    {2, 0x2e, 0x00},
    {2, 0x2f, 0x00},
	{0, 0x09, 0x30}, // baseband TX power 
    {0, 0x16, 0x30},

    //AR8020TEST9363_bb_groundregs
    {0, 0x90, 0x38},
    {0, 0x91, 0x40},
    {0, 0x92, 0x20},
    {0, 0x93, 0x20},
    {0, 0x97, 0x08},
    {0, 0x98, 0x14},
    {0, 0x9d, 0x0c},

    {0, 0xa0, 0x38},
    {0, 0xa2, 0xc0}, 
    {0, 0xa3, 0x98},
    {0, 0xa4, 0x8d},
    {0, 0xa5, 0x7a},
    {0, 0xa6, 0x73},
    
    {0, 0xa7, 0x65},
    {0, 0xa8, 0x7c},
    {0, 0xa9, 0x70},
    {0, 0xaa, 0x60},
    {0, 0xab, 0x50},
    {0, 0xac, 0x48},

    {0, 0xb0, 0x00},
    {0, 0xb1, 0x00}, 
    {0, 0xb2, 0x00}, 
    {0, 0xb3, 0x00},
    {0, 0xb4, 0x00},
    {0, 0xb5, 0x00},
    {0, 0xb6, 0x00}, 
    {0, 0xb7, 0x00},
    {0, 0xb8, 0x00},
    {0, 0xb9, 0x00},
    {0, 0xba, 0x00},
    {0, 0xbb, 0x00},
    {0, 0xbc, 0x00},
    {0, 0xbe, 0x00},

    {0, 0xc0, 0x00},
    {0, 0xc1, 0x00},
    {0, 0xc6, 0x00},
    {0, 0xc7, 0x00},
    {0, 0xca, 0x00},
    {0, 0xce, 0x00},
    
    {2, 0x10, 0x8c},
    {2, 0x13, 0x00}, 
    {2, 0x1a, 0x8c},
    {2, 0x1b, 0x00}, 
    {2, 0x1c, 0x00},
    {2, 0x1d, 0x00}, 
    {2, 0x1f, 0x20},
    {2, 0x20, 0x00},
    {2, 0x21, 0x20},
    {2, 0x2f, 0x00},
    {0, 0x09, 0x30}, // baseband TX power 
    {0, 0x16, 0x30},

    //AR8020TEST9363_bb_skyregsAfterCali
    {0, 0xa1, 0xc4},
    {1, 0x90, 0x81}, //power down ADC.DAC
    {1, 0x91, 0x68}, //power down ADC.DAC

    //AR8020TEST9363_bb_grdregsAfterCali
    {1, 0x90, 0x81}, //power down ADC.DAC
    {1, 0x91, 0x68}, //power down ADC.DAC
};



////////////////////////////////////////////////////////////
extern STRU_BOARD_RF_PARA stru_9363Test_rf_boardCfg;

STRU_cfgNode AR8020_9363Test_nodeInfo= 
{
    .nodeId       = RF_BOARDCFG_PARA_ID,
    .nodeElemCnt  = 1,
    .nodeDataSize = sizeof(stru_9363Test_rf_boardCfg)
};

STRU_BOARD_RF_PARA stru_9363Test_rf_boardCfg __attribute__ ((aligned (4)))= 
{
    .u8_rfCnt           = 1,

    .u8_rf0SkyRegsCnt   = 0,
    
    .u8_rf0GrdRegsCnt   = 0,
    
    .u8_rf1GrdRegsCnt   = 0,

     //after calibration
    .u8_rf0SkyRegsCntAfterCali   = 0,

    .u8_rf0GrdRegsCntAfterCali   = 0,
    
    .u8_rf1GrdRegsCntAfterCali   = 0,

    .boardCfgName                = "P201-WS"
};

extern STRU_BOARD_RF_BW_CHG stru_rf_bw_chg__boardCfg;

STRU_cfgNode rf_bw_chg_nodeInfo= 
{
    .nodeId       = BB_BW_AUTO_CHG_ID,
    .nodeElemCnt  = 1,    
    .nodeDataSize = sizeof(stru_rf_bw_chg__boardCfg)
};

STRU_BOARD_RF_BW_CHG stru_rf_bw_chg__boardCfg __attribute__ ((aligned (4)))= 
{
   .en_auto  = 0,    	   // enbale
    .thd_10   = 78,        // > u8_agcThdL, select_10MHz
    .thd_20   = 73,        // < u8_agcThdH, select_20MHz
    .en_it_hoping_quickly=1,
    .max_rc_len = 6,
	.sweep_noise_thd = 5,
	.sweep_patten_size = 4,
	.rc_fine_sweep_size=8,
	.it_fine_sweep_size=4,
	.rc_common_ch_size=4,
	.rc_common_ch={0,4,30,37},
	.it_unlock_timeout_cnt=64,
	.rc_unlock_timeout_cnt=335,
	.rc_sweep_log_open=0,

};


extern STRU_RF_REG stru_9363Test_rf_regCfg[0];

STRU_cfgNode stru_9363Test_bbrfdata_nodeInfo =
{
    .nodeId       = RF_BOARDCFG_DATA_ID,
    .nodeElemCnt  = 0,
    .nodeDataSize = sizeof(stru_9363Test_rf_regCfg)
};

STRU_RF_REG stru_9363Test_rf_regCfg[0] __attribute__ ((aligned (4)))= 
{
    //pstru_rf1SkyRegs
    //pstru_rf2SkyRegs
    //pstru_rf1GrdRegs
    //pstru_rf2GrdRegs
    
    //rf1_skyregs_afterCali

    //rf1_grdregs_afterCali 
};

extern STRU_BB_AOC stru_9363Test_bb_aoc_boardCfg;

STRU_cfgNode Test9363_bb_aoc_cfg_nodeInfo= 
{
    .nodeId       = BB_AOC_BOARDCFG_ID,
    .nodeElemCnt  = 1,    
    .nodeDataSize = sizeof(stru_9363Test_bb_aoc_boardCfg)
};


STRU_BB_AOC stru_9363Test_bb_aoc_boardCfg __attribute__ ((aligned (4)))= 
{
    //10*lg10(x/64)
    //13dbm->0x4fc,14dbm->0x647,15dbm->0x7e7,16dbm->0x9f3
    //17dbm->0xc87,18dbm->0xfc6,19dbm->0x13db,20db->0x1900,
    //21->0x1f79,22dbm->0x279F
    .u8_aocEnable   = 0,            // default enable
    .u16_snrThdL     = 0x7e7,    // 
    .u16_snrThdH     = 0xc87,    //
    .u8_agcThdL      = 60,        //
    .u8_agcThdH      = 70,        //
    .u8_PwrThdMin    = 15,      //min adjust power 15dbm
    .u8_snrAvgCnt    = 16,//snr average num, MAX Average Count value is 64
    .u8_agcAvgCnt    = 64,//rssi average num,MAX Average Count value is 128
    .u16_ldpcStacCnt = 100,// statistics ldpc err count
    .u16_ldpcThd     = 100,//ldpc err threshold
};

STRU_FACTORY_SETTING stru_Factory_9363Test_cfg __attribute__ ((aligned (4)))= 
{
    //FACTORY_SETTING NODE
    .st_factoryNode =
    {
        .nodeId       = FACTORY_SETTING_NODE_ID,
        .nodeElemCnt  = 1,
        .nodeDataSize = sizeof(STRU_FACTORY_SETTING_DATA)
    },

    //FACTORY_SETTING DATA
    .st_factorySetting =
    {
        //AES node and data
        .st_aesNode =
        {   
            .nodeId       = FACTORY_SUBNODE_BB_AES_CFG_NODE_ID,
            .nodeElemCnt  = 1,
            .nodeDataSize = sizeof(STRU_BB_AES)
        },

        .st_aesData = 
        {
            .u8_upLinkSwitch            = 1,
            .u8_downLinkSwitch          = 1,

            .u8_upLinkKeyArray          = {0x5B,0xA1,0x23,0x70,0x69,0x07,0x7E,0xAF,0x4D,0xD9,0x8C,0x77,0x0E,0x2A,0x38,0xCB,\
                                           0xB6,0x54,0x85,0x31,0xBD,0x46,0x3F,0xE0,0xA8,0x1C,0x15,0x9A,0xC4,0x93,0xD2,0x62},
            .u8_downLinkKeyArray        = {0x5B,0xA1,0x23,0x70,0x69,0x07,0x7E,0xAF,0x4D,0xD9,0x8C,0x77,0x0E,0x2A,0x38,0xCB,\
                                           0xB6,0x54,0x85,0x31,0xBD,0x46,0x3F,0xE0,0xA8,0x1C,0x15,0x9A,0xC4,0x93,0xD2,0x62},
        },

        //RF power node and data
        .st_rfPowerNode =
        {
            .nodeId                     = FACTORY_SUBNODE_POWER_NODE_ID,
            .nodeElemCnt                = 1,
            .nodeDataSize               = sizeof(STRU_RF_POWER_CTRL)
        },

        .st_rfPowerData =
        {
            .mode                       = RF_POWER_OPEN,
            .powerAutoCtrl              = 0,
            .tssi_update_time           = 0,//0:14ms, 1:112ms, 2:896ms, 3:7S
            .power_work_mode            = OTHER,

                                         //2.4G, 5.8G
            .vtPowerFcc                  = {24,  20},
            .vtPowerCe                   = {14,  14},
            .vtPowerSrrc                 = {17,  20},
            .vtPowerOther                = {0x00,  0x00},
/*
            .vtPowerOther                = {0x24,  0x24},
**/
                                         //2.4G, 5.8G
            .rcPowerFcc                  = {24,  20},
            .rcPowerCe                   = {14,  14},
            .rcPowerSrrc                 = {17,  18},
            .rcPowerOther                = {0x00,  0x00},
/*
            .rcPowerOther                = {0x24,  0x24},
**/
        },

        /////////////////////////////////////////////////
        //2.4G 10M rc channel, max 40 channel
        .st_band0_rcChNode = 
        {
            .nodeId                     = FACTORY_SUBNODE_BAND0_RC_10M_FRQ_ID,
            .nodeElemCnt                = 1,
            .nodeDataSize               = sizeof(STRU_RF_CHANNEL)
        },

        .st_band0_rcChData = 
        {
            .u32_rfChCount              = 40,
            .u16_rfChFrqList            = {6350, 6350, 6350, 6350, 6350, 6350, 6350, 6350,
                                           6740, 6740, 6740, 6740, 6740, 6740, 6740, 6740, 
										   6980, 6980, 6980, 6980, 6980, 6980, 6980, 6980,
										   7355, 7355, 7355, 7355, 7355, 7355, 7355, 7355,
										   7710, 7710, 7710, 7710, 7710, 7710, 7710, 7710},
            /*
            .u32_rfChCount              = 40,
            .u16_rfChFrqList            = {6260, 6290, 6320, 6350, 6380, 6410, 6440, 6470,
                                           6650, 6680, 6710, 6740, 6760, 6780, 6830, 6860,
                                           6890, 6920, 6950, 6980, 7010, 7040, 7070, 7100,
                                           7310, 7325, 7340, 7355, 7370, 7390, 7410, 7450,
                                           7620, 7650, 7680, 7710, 7740, 7770, 7800, 7830},
            **/
        },

        //5.8G rc channel, max 40 channel
        .st_band0_10M_vtChNode = 
        {
            .nodeId                     = FACTORY_SUBNODE_BAND0_VT_10M_FRQ_ID,
            .nodeElemCnt                = 1,
            .nodeDataSize               = sizeof(STRU_RF_CHANNEL)
        },
        
        .st_band0_10M_vtChData = 
        {
            .u32_rfChCount              = 10,
            .u16_rfChFrqList            = {6300, 6450, 6700, 6850, 6900, 7040, 7340, 7370, 7640, 7720},
        },

        //do not change or delete
        .st_band1_10M_rcChNode = 
        {
            .nodeId                     = FACTORY_SUBNODE_BAND1_RC_10M_FRQ_ID,
            .nodeElemCnt                = 1,
            .nodeDataSize               = sizeof(STRU_RF_CHANNEL)
        },
        
        //do not change or delete
        .st_band1_10M_vtChNode = 
        {
            .nodeId                     = FACTORY_SUBNODE_BAND1_VT_10M_FRQ_ID,
            .nodeElemCnt                = 1,
            .nodeDataSize               = sizeof(STRU_RF_CHANNEL)
        },

        //do not change or delete
        .st_band0_20M_vtChNode = 
        {
            .nodeId                     = FACTORY_SUBNODE_BAND0_VT_20M_FRQ_ID,
            .nodeElemCnt                = 1,
            .nodeDataSize               = sizeof(STRU_RF_CHANNEL)
        },

        //do not change or delete
        .st_band1_20M_vtChNode = 
        {
            .nodeId                     = FACTORY_SUBNODE_BAND1_VT_20M_FRQ_ID,
            .nodeElemCnt                = 1,
            .nodeDataSize               = sizeof(STRU_RF_CHANNEL)
        },

        // agc set
        .st_agcSetNode = 
        {
            .nodeId                     = FACTORY_SUBNODE_AGC_SET,
            .nodeElemCnt                = 1,
            .nodeDataSize               = sizeof(STRU_AGC_SET)
        },

        .st_agcSetData = 
        {
            .e_gear                     = AGC_GEAR_2,
            .u8_agcSwitchThresh         = 0x50,
        },

        //vt open power makeup value 
        .st_powerOpen_vtNode= 
        {
            .nodeId                     = FACTORY_SUBNODE_POWEROPEN_VT_SET_ID,
            .nodeElemCnt                = 1,
            .nodeDataSize               = sizeof(STRU_BB_CH_OPEN_POWER_REF_VALUE)
        },

        .st_powerOpen_vtData = 
        {
            .u8_powerOpenMakeupEnable   =   1,
            .u8_bbPwr                   =   {0x30, 0x30},
            .u8_2p4g_rfPwr              =   {0x1A, 0x1A},
            .u8_5p8g_rfPwr              =   {0x1A, 0x1A},

            .u8_2p4g_ref_power          =   21,
            .u8_5p8g_ref_power          =   21,

            .u8_2p4g_ref_point_num_a = 8,
            .st_2p4g_real_power_value_a = {{6330, 182}, {6410, 189}, {6750, 210}, {6980, 207}, {7340, 193}, {7370, 192}, {7640, 180}, {7720, 176}},
            .u8_2p4g_ref_point_num_b = 8,
            .st_2p4g_real_power_value_b = {{6330, 180}, {6410, 187}, {6750, 210}, {6980, 210}, {7340, 196}, {7370, 195}, {7640, 180}, {7720, 174}},
        },

        //rc open power makeup value 
        .st_powerOpen_rcNode= 
        {
            .nodeId                     = FACTORY_SUBNODE_POWEROPEN_RC_SET_ID,
            .nodeElemCnt                = 1,
            .nodeDataSize               = sizeof(STRU_BB_CH_OPEN_POWER_REF_VALUE)
        },

        .st_powerOpen_rcData = 
        {
            .u8_powerOpenMakeupEnable   = 1,
            .u8_bbPwr                   = {0x30, 0x00},

            .u8_2p4g_rfPwr              = {0x19, 0x19},
            .u8_5p8g_rfPwr              = {0x19, 0x19},

            .u8_2p4g_ref_power          = 24,
            .u8_5p8g_ref_power          = 24,

            .u8_2p4g_ref_point_num_a = 9,
            .st_2p4g_real_power_value_a = {{6260, 207}, {6440, 214}, {6650, 228}, {6830, 240},  {7100, 238}, {7310, 230}, {7450, 225}, {7620, 215}, {7830, 203}},

            .u8_5p8g_ref_point_num_b = 0,
        },

        //vt close power makeup value 
        .st_powerClose_vtNode= 
        {
            .nodeId                     = FACTORY_SUBNODE_POWERCLOSE_VT_SET_ID,
            .nodeElemCnt                = 1,
            .nodeDataSize               = sizeof(STRU_BB_POWER_CLOSE)
        },

        //rc close power makeup value 
        .st_powerClose_rcNode= 
        {
            .nodeId                     = FACTORY_SUBNODE_POWERCLOSE_RC_SET_ID,
            .nodeElemCnt                = 1,
            .nodeDataSize                = sizeof(STRU_BB_POWER_CLOSE)
        },

        // pwr ctrl
        .st_pwrCtrlNode = 
        {
            .nodeId                     = FACTORY_SUBNODE_PWR_CTRL,
            .nodeElemCnt                = 1,
            .nodeDataSize               = sizeof(STRU_PWR_CTRL)
        },

        .st_pwrCtrlData = 
        {
            .pwr_ctrl = PWR_LEVEL0,
        },

        //
        .st_rfBandMcsNode =
        {
            .nodeId                     = FACTORY_SUBNODE_BAND_MCS_ID,
            .nodeElemCnt                = 1,
            .nodeDataSize               = sizeof(STRU_RF_BAND_MCS_OPT)
        },

        .st_rfBandMcsData =
        {
            .e_bandsupport              = RF_600M,
            .u8_bbStartMcs10M           = 0,
            .u8_bbStartMcs20M           = 0,
            .e_rfbandMode               = MANUAL, //0
            .e_bandwidth                = BW_10M,
        },

        .st_mimoModeNode =
        {
            .nodeId                     = FACTORY_SUBNODE_MIMO_MODE_ID,
            .nodeElemCnt                = 1,
            .nodeDataSize               = sizeof(STRU_MIMO_MODE)
        },

        .st_mimoModeData =
        {
            .st_skyMimoMode             = MIMO_2T2R,
            .st_grdMimoMode             = MIMO_1T2R,
            .enum_lna_mode              = LNA_OPEN,
            .spi_num                    = 1,
        },
             
        .st_uartBaundNode =     
        {
            .nodeId                     = FACTORY_SUBNODE_UART_BAUDR_ID,
            .nodeElemCnt                = 1,
            .nodeDataSize               = sizeof(STRU_UART_BAUDR)
        },
        
        .st_uartBaudData =
        {
            .st_uartBaudr               = {HAL_UART_BAUDR_115200, HAL_UART_BAUDR_115200, HAL_UART_BAUDR_115200, 0x01},
        },
    },
};
