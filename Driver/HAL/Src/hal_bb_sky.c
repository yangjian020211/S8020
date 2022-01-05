/*****************************************************************************
Copyright: 2016-2020, Artosyn. Co., Ltd.
File name: hal_bb.c
Description: The external HAL APIs to use the I2C controller.
Author: Artosy Software Team
Version: 0.0.1
Date: 2016/12/20
History:
        0.0.1    2017/02/06    The initial version of hal_bb_sky.c
*****************************************************************************/


#include <stdio.h>
#include <stdint.h>
#include "sys_event.h"
#include "bb_spi.h"
#include "bb_ctrl.h"
#include "bb_sky_ctrl.h"
#include "hal_bb.h"

#include "debuglog.h"
#include "bb_customerctx.h"


/** 
 * @brief   init baseband to sky mode
 * @param   NONE
 * @return  HAL_OK:                         means init baseband 
 *          HAL_BB_ERR_INIT:                means some error happens in init session 
 */
HAL_RET_T HAL_BB_InitSky(STRU_CUSTOMER_CFG *stru_customerCfg )
{
    BB_init( BB_SKY_MODE, stru_customerCfg, (uint8_t *)BB_sky_regs);
	
	BB_rc_hope_mode_set(SELECTION_BAND_HOPING);

    BB_SKY_start();

    return HAL_OK;
}
/** 
 * @brief   set Sky BaseBand continous error packet threshold, if over the threshold, sky unlock
 * @param   error packet threshlod
 * @return  void
 */
void HAL_BB_SetUnlockCnt(uint8_t u8_lock2unlockCnt)
{
    sky_set_unlockCnt(u8_lock2unlockCnt);
}
#if 0
/** 
 * @brief   set sky BaseBand start to find the same type device in besides
 * @param   null
 * @return  void
 */
void HAL_BB_StartFindBesideDev(void)
{
    sky_start_find_beside_dev();
}
/** 
 * @brief   finish find beside dev, call this functions to stop find
 * @param   null
 * @return  void
 */
void HAL_BB_StopFindBesideDev(void)
{
    sky_stop_find_beside_dev();
}
#endif
