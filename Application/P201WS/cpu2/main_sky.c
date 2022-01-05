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

void console_init(uint32_t uart_num, uint32_t baut_rate)
{
    DLOG_Init(CMD_exec_cmd, NULL, DLOG_CLIENT_PROCESSOR);
}
static void slna_open(void)
{
       HAL_GPIO_InPut(HAL_GPIO_NUM92);
       HAL_GPIO_InPut(HAL_GPIO_NUM96);
       HAL_GPIO_InPut(HAL_GPIO_NUM100);
}

static void slna_bypass(void)
{
       HAL_GPIO_OutPut(HAL_GPIO_NUM100);
       HAL_GPIO_SetPin(HAL_GPIO_NUM100,HAL_GPIO_PIN_RESET);
       HAL_GPIO_InPut(HAL_GPIO_NUM92);
       HAL_GPIO_InPut(HAL_GPIO_NUM96);
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

    STRU_HAL_H264_CONFIG st_h264Cfg;
    st_h264Cfg.u8_view0En = 1;
    st_h264Cfg.u8_view0Gop = 10;
    st_h264Cfg.e_view0Br = HAL_H264_BITRATE_500K;
    st_h264Cfg.u8_view0BrEn = 1;
    st_h264Cfg.u8_view1En = 1;
    st_h264Cfg.u8_view1Gop = 10;
    st_h264Cfg.e_view1Br = HAL_H264_BITRATE_500K;
    st_h264Cfg.u8_view1BrEn = 1;
    st_h264Cfg.e_bit = HAL_H264_16BIT;
    HAL_H264_Init(st_h264Cfg);

//  HAL_BB_RegisterLnaFuntion(slna_open,slna_bypass);

    HAL_BB_InitSky(NULL);

    for( ;; )
    {
        SYS_EVENT_Process();
        DLOG_Process(NULL);
    }
} 

