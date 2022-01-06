#include "serial.h"
#include "debuglog.h"
#include "hal_h264.h"
#include "hal_bb.h"
#include "sys_event.h"
#include "hal_sys_ctl.h"
#include "hal_bb.h"
#include "hal_gpio.h"
#include "hal.h"
#include "test_bb.h"
#include "bb_customerctx.h"
#include "enc_customerctx.h"
#include "cmd_line.h"
#include "board_watchdog.h"
#include "test_bb_led_ctrl_2.h"

void console_init(uint32_t uart_num, uint32_t baut_rate)
{
    DLOG_Init(CMD_exec_cmd, NULL, DLOG_CLIENT_PROCESSOR);
}

static STRU_ENC_CUSTOMER_CFG st_enc_usr_cfg = 
{
    .u8_encHdmiDvpPolarity  = 0,
    .u8_encMipiPolarity     = 4,
};

void fem_only_rx(uint8_t ch)
{
    HAL_GPIO_OutPut(HAL_GPIO_NUM47);
    HAL_GPIO_OutPut(HAL_GPIO_NUM49);
    HAL_GPIO_SetPin(HAL_GPIO_NUM47,HAL_GPIO_PIN_RESET);
    HAL_GPIO_SetPin(HAL_GPIO_NUM49,HAL_GPIO_PIN_RESET);
}

void fem_normal_trx(uint8_t ch)
{
	HAL_GPIO_InPut(HAL_GPIO_NUM46);
	HAL_GPIO_InPut(HAL_GPIO_NUM47);
	HAL_GPIO_InPut(HAL_GPIO_NUM48);
	HAL_GPIO_InPut(HAL_GPIO_NUM49);
}

/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
int main(void)
{
    STRU_HAL_SYS_CTL_CONFIG *pst_cfg;
    HAL_SYS_CTL_GetConfig(&pst_cfg);
    HAL_SYS_CTL_Init(pst_cfg);

    WATCHDOG_InitCpu2();
    /* initialize the uart */
    console_init(2, 115200);   
    dlog_set_output_level(LOG_LEVEL_WARNING);
    DLOG_Critical("cpu2 start!!! \n");
	#if 0
    STRU_HAL_H264_CONFIG st_h264Cfg;
    st_h264Cfg.u8_view0En = 1;
    st_h264Cfg.u8_view0Gop = 10;
    st_h264Cfg.e_view0Br = HAL_H264_BITRATE_500K;
    st_h264Cfg.u8_view0BrEn = 1;
    st_h264Cfg.u8_view1En = 1;
    st_h264Cfg.u8_view1Gop = 10;
    st_h264Cfg.e_view1Br = HAL_H264_BITRATE_500K;
    st_h264Cfg.u8_view1BrEn = 1;
    st_h264Cfg.e_bit = HAL_H264_0_7BIT;
    HAL_H264_Init(st_h264Cfg);
    HAL_H264_UsrCfg(0, &st_enc_usr_cfg);
    HAL_H264_UsrCfg(1, &st_enc_usr_cfg);
	#endif
	
    HAL_BB_RegisterLnaFuntion(lna_open,c201s_lna_bypass);
    HAL_BB_Enable_RcRandomSkipFreq();
    HAL_BB_RegisterFemFuntion(fem_normal_trx,fem_only_rx);
	//normal_rf();
    HAL_BB_InitSky(NULL);
    HAL_BB_SetUnlockCnt(64);

    for( ;; )
    {
        SYS_EVENT_Process();
        DLOG_Process(NULL);
    }
} 

