#ifndef __RF9363_H__
#define __RF9363_H__

#define     AAGC_GAIN_FAR                   (0)
#define     AAGC_GAIN_NEAR                  (1)


#include "boardParameters.h"
#include "bb_ctrl_internal.h"

int RF_SPI_WriteReg(uint16_t u16_addr, uint8_t u8_data);

int RF_SPI_ReadReg(uint16_t u16_addr, uint8_t *pu8_rxValue);

int RF_PcWriteReg(uint8_t ch, uint16_t u16_addr, uint8_t u8_data);

int RF_PcReadReg(uint8_t ch, uint16_t u16_addr, uint8_t *pu8_rxValue);

void RF_init(ENUM_BB_MODE en_mode);

void RF_CaliProcess(ENUM_BB_MODE en_mode);

void BB_RF_band_switch(ENUM_RF_BAND rf_band);


void BB_grd_notify_it_skip_freq(ENUM_RF_BAND band, uint8_t u8_ch);

uint8_t BB_set_ItFrqByCh(ENUM_RF_BAND band, uint8_t ch);

uint8_t BB_set_Rcfrq(ENUM_RF_BAND band, uint8_t ch);

uint8_t BB_set_skySweepfrq(ENUM_RF_BAND band, uint8_t ch);

uint8_t BB_write_RcRegs(uint8_t *u8_rc);

uint8_t BB_set_SweepFrq(ENUM_RF_BAND band, ENUM_CH_BW e_bw, uint8_t ch);


uint8_t BB_GetRcFrqNum(ENUM_RF_BAND e_rfBand);

uint8_t BB_GetItFrqNum(ENUM_RF_BAND e_rfBand);

uint8_t BB_SetAgcGain(ENUM_RF_BAND e_rfBand, uint8_t gain);

int32_t BB_SweepEnergyCompensation(int32_t data);

int32_t BB_SweepEnergy();

void  BB_set_power_open(ENUM_RF_BAND band,uint8_t power);

void BB_set_power_close(ENUM_RF_BAND band,uint8_t power);

uint8_t BB_GetSkySweepFrqNum(ENUM_RF_BAND e_rfBand);

void BB_set_RF_mimo_mode(ENUM_MIMO_MODE e_mimo_mode);

int RF_RegValue2Frq(uint8_t value[], uint16_t *frq);

uint16_t BB_GetRcFrqByCh(uint8_t ch);


uint16_t BB_GetItFrqByCh(uint8_t ch);


uint8_t BB_GetItChInOneFilter(uint8_t ch);

uint8_t BB_GetItStarEndByItCh(uint8_t *start, uint8_t *end, uint8_t it_ch);

uint8_t BB_GetRcStarEndByItCh(uint8_t *start, uint8_t *end, uint8_t it_ch);

uint8_t BB_GetRcFrqNumPerFilter(void);
    
uint8_t BB_GetItFrqNumPerFilter(void);

void BB_InitSunBand_RcFreqNum(uint8_t bw_10M_Rc_freq_num, uint8_t bw_20M_Rc_freq_num);

uint8_t BB_GetSubBandStartCH(ENUM_RF_BAND rf_band, ENUM_CH_BW bw, uint8_t it_ch);

uint8_t BB_GetSubBandEndCH(ENUM_RF_BAND rf_band, ENUM_CH_BW bw, uint8_t it_ch);

uint16_t RF8003xCalcRegister2Frq(uint32_t u32_frq_reg);

void RF8003xCalcFrq2Register(uint16_t u16_frq, STRU_FRQ_CHANNEL *frq_regvalue);

uint8_t BB_set_skySweepVtfrq(ENUM_RF_BAND band, uint8_t ch);

void RF8003s_GetFctFreqTable(ENUM_CH_BW e_bw);

void RF_GetFctFreqTable(ENUM_CH_BW e_bw);


#endif
