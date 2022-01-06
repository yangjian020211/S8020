#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <stdint.h>

#include "systicks.h"
#include "debuglog.h"
#include "bb_ctrl_internal.h"
#include "bb_regs.h"
#include "log10.h"
#include "bb_sky_ctrl.h"
#include "rf_if.h"


#define RC_SWEEP_TIMES (80)

uint8_t  sky_curSweepCh = 0;
int16_t  sky_NoisePower[MAX_RC_FRQ_SIZE][RC_SWEEP_TIMES] = {0};
int16_t  sky_averNoisePower[MAX_RC_FRQ_SIZE] = {0};
int16_t  sky_curNoisePower[MAX_RC_FRQ_SIZE] = {0};

uint32_t sky_sweepcnt = 0;
uint8_t  sky_loopcnt  = 0;

static uint8_t sky_subBandFrqIdx[5] = {0, 0, 0, 0, 0};
static uint8_t sky_subBandSweepFlag[5] = {0, 0, 0, 0, 0};

static uint8_t sky_getSeepFrqChByUseFrqCh(uint8_t useFrqCh)
{
    uint8_t frq_num = BB_GetRcFrqNumPerFilter();
    uint8_t sub_band = useFrqCh / frq_num;
    uint8_t sweep_ch = sub_band * frq_num + sky_subBandFrqIdx[sub_band];
    
    sky_subBandFrqIdx[sub_band] += 1;
    sky_subBandFrqIdx[sub_band] %= frq_num;

    return sweep_ch;
}

static void sky_ResetSweepFlag(void)
{
    uint8_t len = sizeof(sky_subBandSweepFlag)/sizeof(sky_subBandSweepFlag[0]);
    
    memset(sky_subBandSweepFlag, 0x00, len);
}

static void sky_SetSweepFlag(uint8_t ch)
{
    uint8_t frq_num = BB_GetRcFrqNumPerFilter();
    uint8_t band = ch / frq_num;
    uint8_t offset = ch % frq_num;
    
    sky_subBandSweepFlag[band] |= (1 << offset);
}

static uint8_t sky_CompleteOneSweep(void)
{
    uint8_t ret = 1;
    uint8_t frq_num = BB_GetRcFrqNumPerFilter();
    uint8_t band = context.rc_start/ frq_num;

    if(0xFF != sky_subBandSweepFlag[band])
    {
        ret = 0;
    }

    return ret;
}

void sky_startSweep(ENUM_RF_BAND band)
{
    BB_set_RF_Band(BB_SKY_MODE, band);
    memset(sky_subBandFrqIdx, 0x00, sizeof(sky_subBandFrqIdx)/sizeof(sky_subBandFrqIdx[0]));
    sky_loopcnt = 0;
    sky_ResetSweepFlag();
}

uint8_t sky_SetSweepCh(ENUM_RF_BAND band, uint8_t ch)
{
    sky_curSweepCh = sky_getSeepFrqChByUseFrqCh(ch);
    
    BB_set_skySweepfrq(band, sky_curSweepCh);
    sky_SetSweepFlag(sky_curSweepCh);

    if (1 == sky_CompleteOneSweep())
    {
        sky_ResetSweepFlag();
        sky_loopcnt ++;
        if (sky_loopcnt >= RC_SWEEP_TIMES)
        {
            sky_loopcnt = 0;
        }
    }
    
    return 0;
}

uint32_t sky_GetSweepResult(void)
{
    sky_curNoisePower[sky_curSweepCh] = BB_SweepEnergy();

    sky_NoisePower[sky_curSweepCh][sky_loopcnt] = sky_curNoisePower[sky_curSweepCh];

    //cal average sweep power
    sky_CalAverSweepResult(sky_curSweepCh, sky_curSweepCh+1);

    return 0;
}

uint32_t sky_GetSweepResultbych(uint8_t row,uint8_t col)
{
	return sky_NoisePower[row][col];
}

int sky_GetAverSweepResult(uint8_t ch)
{
    return sky_averNoisePower[ch];
}

void sky_CalAverSweepResult(uint8_t start, uint8_t end)
{
    uint8_t ch;
    uint8_t i = 0;
    
    for (ch = start ; ch < end; ch++)
    {
        int32_t aver = 0;
	    sky_averNoisePower[ch] = 0;
        for (i =0; i< RC_SWEEP_TIMES; i++)
        {
            aver += sky_NoisePower[ch][i];
        }
        sky_averNoisePower[ch] = aver / RC_SWEEP_TIMES;
        //DLOG_Warning("ch:%d value:%d", ch, sky_averNoisePower[ch]);
    }
}


void sky_GetSweepNoise(int16_t *ptr_noise_power, uint32_t max)

{
    uint16_t i = 0;
    uint16_t j = 0;
    int16_t value;

    for (i=context.rc_start; i<context.rc_end; i++)
    {
        //value = (int16_t)sky_averNoisePower[i];
        value = (int16_t)sky_curNoisePower[i];
        for(j = 0; j < 8; j++)
        {
            ptr_noise_power[(i - context.rc_start) * 8 + j] = value;
        }
        //DLOG_Warning("ch:%d value:%d", i, value);
    }
}

void __attribute__ ((section(".h264"))) sky_SweepProcess(void)
{
}

void __attribute__ ((section(".h264"))) sky_requestRfBandSwitch(ENUM_RF_BAND e_band)
{
}

