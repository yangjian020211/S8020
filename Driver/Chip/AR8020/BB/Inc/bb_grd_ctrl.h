/**
  ******************************************************************************
  * @file    ground_controller.h
  * @author  Artosyn AE/FAE Team
  * @version V1.0
  * @date    03-21-2016
  * @brief
  *
  *
  ******************************************************************************
  */
#ifndef __GRD_CONTROLLER_H
#define __GRD_CONTROLLER_H

#include <stdint.h>
#include "debuglog.h"


void BB_GRD_start(void);

void grd_SetRCId(uint8_t *pu8_id);

uint8_t grd_is_bb_fec_lock(void);

uint8_t snr_static_for_qam_change(uint16_t threshod_left_section,uint16_t threshold_right_section);

static void grd_handle_RC_mode_cmd(ENUM_RUN_MODE mode);

void BB_grd_NotifyItFreqByCh(ENUM_RF_BAND band, uint8_t u8_ch);

#endif
