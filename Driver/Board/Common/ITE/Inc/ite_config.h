///*****************************************
//  Copyright (C) 2009-2017
//  ITE Tech. Inc. All Rights Reserved
//  Proprietary and Confidential
///*****************************************
//   @file   <config.h>
//   @author Jau-Chih.Tseng@ite.com.tw
//   @date   2017/03/29
//   @fileversion: ITE_MHLRX_SAMPLE_V1.26
//******************************************/
#ifndef _CONFIG_H_
#define _CONFIG_H_

//#define _MCU_8051_

#ifndef _MCU_8051_

#endif

#define _IT6801_

#if (!defined(_IT6801_))&&(!defined(_IT6802_))&&(!defined(_IT6803_))
    #define _IT6802_
#endif

#ifdef _IT6803_
    #pragma message("defined _IT6803_")
    //#define ENABLE_IT6803	// only for IT6803 Usage
#endif


#ifdef _IT6802_
#pragma message("defined _IT6802_")
#endif

#ifdef _IT6801_
#pragma message("defined _IT6801_")
#endif

//#define SUPPORT_I2C_SLAVE
#ifdef SUPPORT_I2C_SLAVE
#pragma message ("SUPPORT_I2C_SLAVE defined")
#endif

#define _EN_DUAL_PIXEL_CTRL_
#define _EN_BLOCK_PWRDN_
#define SUPPORT_OUTPUTRGB
#define SUPPORT_INPUTYUV


#define IT6802A0_HDMI_ADDR 0x94 //Hardware Fixed I2C address of IT6602 HDMI
#define IT6802B0_HDMI_ADDR 0x92 //Hardware Fixed I2C address of IT6602 HDMI
#define MHL_ADDR 0xE0   //Software programmable I2C address of IT6602 MHL
#define EDID_ADDR 0xA8  //Software programmable I2C address of IT6602 EDID RAM
#define CEC_ADDR 0xC8   //Software programmable I2C address of IT6602 CEC


#define DP_ADDR 0x90
#define ADC_ADDR 0x90



#define HDMI_DEV  0
#define DP_DEV    0

#define RXDEV           0
#define MAXRXDEV        1
/****************************************************************************/

/****************************************************************************/
#define HDMI_DEV        0
#define SWADR       0x96
#define EDID_HEAD   0xA0

#define DELAY_TIME        1
#define IDLE_TIME        100

#define HIGH            1
#define LOW                0
#define ACTIVE          1
#define DISABLE         0

#endif // _CONFIG_H_
