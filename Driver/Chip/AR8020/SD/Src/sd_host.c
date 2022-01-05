/**
  * @file    sd_host.c
  * @author  Minzhao & wu.min
  * @version V1.0.0
  * @date    7-10-2016
  * @brief   Header file of sd card.
  *          This file contains:
  *           + Initialization and de-initialization functions
  *           + IO operation functions
  *           + Peripheral Control functions
  *           + Peripheral State functions
  *           + bug fix for status check after erase (wumin) 2017-4-21
  *           + some for stability improvement (wumin) 2017-5-21
  */

/*
 * updated 2017-6-3
1. improve SD driver PowerOFF stability, seperated into two stage
2. according to the specification, added DAT0 checking before SD read, program, erase
3. take away Core_SDMMC_SetCTYPE from SD_DMAConfig to init
*/


#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>
#include "reg_rw.h"
#include "sd_host.h"
#include "sd_core.h"
#include "debuglog.h"
#include "systicks.h"
#include "sys_event.h"
#include <string.h>
#include "cpu_info.h"
#include "memory.h"


#define BUFFER_CHIP_SURPPORTED 1

//SDMMC_DMATransTypeDef dma;
SD_HandleTypeDef sdhandle;
SD_CardInfoTypedef cardinfo;

/** @defgroup SD_Private_Functions SD Private Functions
  * @{
  */
static EMU_SD_RTN IdentificateCard(SD_HandleTypeDef *hsd,SD_CardInfoTypedef *pCardInfo);
static EMU_SD_RTN PowerOnCard(SD_HandleTypeDef *hsd);
static EMU_SD_RTN InitializeCard(SD_HandleTypeDef *hsd);
static EMU_SD_RTN PowerOffCard(SD_HandleTypeDef *hsd);
static EMU_SD_RTN SD_CmdError(SD_HandleTypeDef *hsd);
static EMU_SD_RTN SD_CmdResp1Error(SD_HandleTypeDef *hsd, uint32_t SD_CMD);
static EMU_SD_RTN SD_CmdResp7(SD_HandleTypeDef *hsd);
static EMU_SD_RTN SD_DMAConfig(SD_HandleTypeDef *hsd, SDMMC_DMATransTypeDef *dma);
static EMU_SD_RTN SD_ENUM(SD_HandleTypeDef *hsd);
static EMU_SD_RTN SD_IsCardProgramming(SD_HandleTypeDef *hsd, uint8_t *status);
static SD_STATUS sd_getState(SD_HandleTypeDef *hsd);
static EMU_SD_RTN wait_cmd_busy(void);
static EMU_SD_RTN wait_cmd_done(void);
static EMU_SD_RTN convert_to_transfer(SD_HandleTypeDef *hsd);
static EMU_SD_RTN Card_SD_CMD6_check_patten(SD_HandleTypeDef *hsd);
static EMU_SD_RTN SD_Tuning(SD_HandleTypeDef *hsd);
static EMU_SD_RTN SD_DMAConfig_test(SD_HandleTypeDef * hsd, SDMMC_DMATransTypeDef * dma);

#define SD_READ_SINGLE_BLOCK 0 
#define	SD_READ_MULTIPLE_BLOCK 1
#define SD_WRITE_SINGLE_BLOCK 2
#define SD_WRITE_MULTIPLE_BLOCK 3

static void sd_delay_ms(uint32_t num)
{
    volatile int i;
    for (i = 0; i < num * 100; i++);
}

static void host_setting_clock(SD_HandleTypeDef *hsd);

EMU_SD_RTN Card_SD_Init(SD_HandleTypeDef *hsd, SD_CardInfoTypedef *SDCardInfo)
{
  __IO EMU_SD_RTN errorstate = SD_OK;
  uint32_t get_val;
  DLOG_Info("1\n");
  errorstate = PowerOnCard(hsd);
  if (errorstate != SD_OK)
  {
    DLOG_Error("Power on Card Error!\n");
    return errorstate;
  }
  DLOG_Info("2\n");
  /* Identify card operating voltage */
  errorstate = InitializeCard(hsd);
  if (errorstate != SD_OK)
  {
    if (errorstate == 42)
    {
      DLOG_Info("No SD Card! Quit Initialization!\n");
    }
    else
    {
      DLOG_Error("Initialize Card Error!\n");  
    }
    return errorstate;
  }
  DLOG_Info("3\n");
  /* Initialize the present SDMMC card(s) and put them in idle state */
  errorstate = IdentificateCard(hsd, SDCardInfo);
  if (errorstate != SD_OK)
  {
    return errorstate;
  }

  /* Read CSD/CID MSD registers */
  errorstate = Card_SD_Get_CardInfo(hsd, SDCardInfo);
  return errorstate;
}



/**
  * @brief  De-Initializes the SD card.
  * @param  hsd: SD handle
  * @retval HAL status
  */
EMU_SD_RTN Card_SD_DeInit(SD_HandleTypeDef *hsd)
{
  EMU_SD_RTN errorstate = SD_OK;
  /* Set SD power state to off */
  errorstate = PowerOffCard(hsd);

  return errorstate;
}

void Card_SwtichCLK(SD_HandleTypeDef *hsd)
{
    SDMMC_CmdInitTypeDef sdmmc_cmdinitstructure;

    Core_SDMMC_SetCLKSRC(hsd->Instance, 0x00000000);
    Core_SDMMC_SetCLKENA(hsd->Instance, 0x00000000);

    sdmmc_cmdinitstructure.Argument  = 0x00000000;
    sdmmc_cmdinitstructure.CmdIndex  = SD_CMD_GO_IDLE_STATE;
    sdmmc_cmdinitstructure.Response  = SDMMC_RESPONSE_NO;
    sdmmc_cmdinitstructure.Attribute = SDMMC_CMD_START_CMD | 
                                       SDMMC_CMD_USE_HOLD_REG | 
                                       SDMMC_CMD_PRV_DAT_WAIT | 
                                       SDMMC_CMD_UPDATE_CLK;
    Core_SDMMC_SendCommand(hsd->Instance, &sdmmc_cmdinitstructure);
    Core_SDMMC_WaiteCmdStart(hsd->Instance);
    //sd_delay_ms(500);
    SysTicks_DelayMS(50);
    host_setting_clock(hsd);  

    Core_SDMMC_SetCLKSRC(hsd->Instance, 0x00000000);
    Core_SDMMC_SetCLKENA(hsd->Instance, 0x00000001); 

    sdmmc_cmdinitstructure.Argument  = 0x00000000;
    sdmmc_cmdinitstructure.CmdIndex  = SD_CMD_GO_IDLE_STATE;
    sdmmc_cmdinitstructure.Response  = SDMMC_RESPONSE_NO;
    sdmmc_cmdinitstructure.Attribute = SDMMC_CMD_START_CMD | 
                                       SDMMC_CMD_USE_HOLD_REG | 
                                       SDMMC_CMD_PRV_DAT_WAIT | 
                                       SDMMC_CMD_UPDATE_CLK;
    Core_SDMMC_SendCommand(hsd->Instance, &sdmmc_cmdinitstructure);
    Core_SDMMC_WaiteCmdStart(hsd->Instance);
    Core_SDMMC_SetCTYPE(hsd->Instance, SDMMC_CTYPE_4BIT);
}

/*
CPU0: Card_SD_CMD6      0x440ba7f8: 0x01809001 0x0f800180 0x01800f80 0x02001f80 0x00000002 0x00000000 0x00000000 0x00000000 
                                                                                                                       287:272 
CPU0: Card_SD_CMD6      0x440ba818: 0x00000000 0x00000000 0x00000000 0x00000000 0x00000000 0x00000000 0x00000000 0x00000000 
CPU0: Card_SD_CMD19     0x440ba7e0: 0x00ff0fff 0xccc3ccff 0xffcc3cc3 0xeffefffe 0xddffdfff 0xfbfffbff 0xff7fffbf 0xefbdf777 
CPU0: Card_SD_CMD19     0x440ba800: 0xf0fff0ff 0x3cccfc0f 0xcfcc33cc 0xeeffefff 0xfdfffdff 0xffbfffdf 0x00000000 0x00000000 
*/
EMU_SD_RTN Card_SD_CMD6(SD_HandleTypeDef *hsd)
{    
    SDMMC_DMATransTypeDef *dma = malloc_safe(sizeof(SDMMC_DMATransTypeDef));
    SDMMC_CmdInitTypeDef sdmmc_cmdinitstructure;
    IDMAC_DescTypeDef desc = {0};
    uint8_t *CMD6Data = malloc_safe(64);
    EMU_SD_RTN ret = SD_OK;

    DLOG_Info("Send CMD6");

    if (CMD6Data)
    {
        memset(CMD6Data, 0, 64);
    }
    else
    {
        DLOG_Error("malloc error");
    }
    uint32_t dma_Dst = peripheralAddrConvert((uint32_t)CMD6Data);    
    hsd->SdTransferCplt  = 0;
    hsd->SdTransferErr   = SD_OK;
    if (hsd->CardType == HIGH_CAPACITY_SD_CARD)
    {
      dma->BlockSize = 64;
    }
    /* Configure the SD DPSM (Data Path State Machine) */
    SD_DMAConfig(hsd, dma);
    Core_SDMMC_SetDBADDR(hsd->Instance, peripheralAddrConvert((uint32_t)&desc));
    Core_SDMMC_SetBYCTNT(hsd->Instance, 0x40);
    desc.des0 = SDMMC_DES0_OWN | SDMMC_DES0_FS |  SDMMC_DES0_CH | SDMMC_DES0_LD;
    desc.des1 = 64;
    desc.des2 = dma_Dst;
    desc.des3 = 0x0;
    sdmmc_cmdinitstructure.Argument         = (0x80ffff00 | hsd->SpeedMode);
/*     sdmmc_cmdinitstructure.Argument         = (0x00ffff00 | hsd->SpeedMode); */
/*     sdmmc_cmdinitstructure.Argument         = (0x00fffff1); */
    // DLOG_Info("argumen = %x", sdmmc_cmdinitstructure.Argument);
    sdmmc_cmdinitstructure.CmdIndex         = SD_CMD_HS_SWITCH;
    sdmmc_cmdinitstructure.Response         = SDMMC_RESPONSE_R1;
    sdmmc_cmdinitstructure.Attribute        = SDMMC_CMD_START_CMD | \
                                              SDMMC_CMD_USE_HOLD_REG | \
                                              SDMMC_CMD_PRV_DAT_WAIT | \
                                              SDMMC_CMD_DAT_EXP      | \
                                              SDMMC_CMD_RESP_CRC     | \
                                              SDMMC_CMD_RESP_EXP;
    Core_SDMMC_SetRINTSTS(hsd->Instance, 0xFFFE);
/*     Core_SDMMC_SetRINTSTS(hsd->Instance, 0xFFFFFFFF); */
    Core_SDMMC_SendCommand(hsd->Instance, &sdmmc_cmdinitstructure);
    Core_SDMMC_WaiteCmdDone(hsd->Instance);
/*     Core_SDMMC_WaiteCardBusy(hsd->Instance);   */


    //sd_delay_ms(50000);
    SysTicks_DelayMS(100);
    unsigned int readAddress = dma_Dst;

    unsigned char row;
    readAddress -= (readAddress % 4);

    for (row = 0; row < 2; row++)
    {
      /* new line */
      DLOG_Info("0x%08x: 0x%08x 0x%08x 0x%08x 0x%08x 0x%08x 0x%08x 0x%08x 0x%08x ", 
      readAddress,
      *(uint32_t *)readAddress,
      *(uint32_t *)(readAddress + 4),
      *(uint32_t *)(readAddress + 8),
      *(uint32_t *)(readAddress + 12),
      *(uint32_t *)(readAddress + 16),
      *(uint32_t *)(readAddress + 20),
      *(uint32_t *)(readAddress + 24),
      *(uint32_t *)(readAddress + 28));
      readAddress += 32;
    }

    if ((CMD6Data[16] & 0x0F) == hsd->SpeedMode)
    {
    DLOG_Info("speed = %d supported", hsd->SpeedMode);
    ret = SD_OK;
    }
    else
    {
    DLOG_Error("speed = %d not supported", hsd->SpeedMode);
    ret = SD_FAIL;
    }
     
    free_safe(CMD6Data);
    free_safe(dma);
    return ret;
}

static EMU_SD_RTN Card_SD_CMD6_check_patten(SD_HandleTypeDef *hsd)
{    
    SDMMC_DMATransTypeDef *dma = malloc_safe(sizeof(SDMMC_DMATransTypeDef));
    SDMMC_CmdInitTypeDef sdmmc_cmdinitstructure;
    IDMAC_DescTypeDef desc = {0};
    uint8_t *CMD6Data = malloc_safe(64);
    EMU_SD_RTN ret = SD_OK;

    DLOG_Info("Send CMD6 check patten");

    if (CMD6Data)
    {
        DLOG_Info("CMD6Data = 0x%08x", CMD6Data);
        memset(CMD6Data, 0, 64);
    }
    else
    {
        DLOG_Error("malloc error");
    }
    uint32_t dma_Dst = peripheralAddrConvert((uint32_t)CMD6Data);    
    DLOG_Info("CMD6Dst = 0x%08x", dma_Dst);
    hsd->SdTransferCplt  = 0;
    hsd->SdTransferErr   = SD_OK;
    if (hsd->CardType == HIGH_CAPACITY_SD_CARD)
    {
      dma->BlockSize = 64;
    }
    /* Configure the SD DPSM (Data Path State Machine) */
    SD_DMAConfig(hsd, dma);
    Core_SDMMC_SetDBADDR(hsd->Instance, peripheralAddrConvert((uint32_t)&desc));
    Core_SDMMC_SetBYCTNT(hsd->Instance, 0x40);
    desc.des0 = SDMMC_DES0_OWN | SDMMC_DES0_FS |  SDMMC_DES0_CH | SDMMC_DES0_LD;
    desc.des1 = 64;
    desc.des2 = dma_Dst;
    desc.des3 = 0x0;
    sdmmc_cmdinitstructure.Argument         = (0x00ff1200 | hsd->SpeedMode);

    
/*     sdmmc_cmdinitstructure.Argument         = (0x00ffff00 | hsd->SpeedMode); */
/*     sdmmc_cmdinitstructure.Argument         = (0x00fffff1); */
    // DLOG_Info("argumen = %x", sdmmc_cmdinitstructure.Argument);
    sdmmc_cmdinitstructure.CmdIndex         = SD_CMD_HS_SWITCH;
    sdmmc_cmdinitstructure.Response         = SDMMC_RESPONSE_R1;
    sdmmc_cmdinitstructure.Attribute        = SDMMC_CMD_START_CMD | \
                                              SDMMC_CMD_USE_HOLD_REG | \
                                              SDMMC_CMD_PRV_DAT_WAIT | \
                                              SDMMC_CMD_DAT_EXP      | \
                                              SDMMC_CMD_RESP_CRC     | \
                                              SDMMC_CMD_RESP_EXP;
    Core_SDMMC_SetRINTSTS(hsd->Instance, 0xFFFE);
/*     Core_SDMMC_SetRINTSTS(hsd->Instance, 0xFFFFFFFF); */
    Core_SDMMC_SendCommand(hsd->Instance, &sdmmc_cmdinitstructure);
    Core_SDMMC_WaiteCmdDone(hsd->Instance);
/*     Core_SDMMC_WaiteCardBusy(hsd->Instance);   */


    //sd_delay_ms(50000);
    SysTicks_DelayMS(100);
    unsigned int readAddress = dma_Dst;

     unsigned char row;
     readAddress -= (readAddress % 4);

     for (row = 0; row < 2; row++)
     {
         /* new line */
         DLOG_Info("0x%08x: 0x%08x 0x%08x 0x%08x 0x%08x 0x%08x 0x%08x 0x%08x 0x%08x ", 
                   readAddress,
                   *(uint32_t *)readAddress,
                   *(uint32_t *)(readAddress + 4),
                   *(uint32_t *)(readAddress + 8),
                   *(uint32_t *)(readAddress + 12),
                   *(uint32_t *)(readAddress + 16),
                   *(uint32_t *)(readAddress + 20),
                   *(uint32_t *)(readAddress + 24),
                   *(uint32_t *)(readAddress + 28));

         readAddress += 32;
     }

     if ((CMD6Data[16] & 0x0F) == hsd->SpeedMode)
     {
        DLOG_Info("speed = %d supported %x", hsd->SpeedMode, CMD6Data[16]);
        ret = SD_OK;
     }
     else
     {
        DLOG_Error("speed = %d not supported", hsd->SpeedMode);
        ret = SD_FAIL;
     }
     
    free_safe(CMD6Data);
    free_safe(dma);
    return ret ;
}

EMU_SD_RTN Card_SD_CMD19(SD_HandleTypeDef *hsd)
{
    SDMMC_DMATransTypeDef *dma = malloc_safe(sizeof(SDMMC_DMATransTypeDef));
    SDMMC_CmdInitTypeDef sdmmc_cmdinitstructure;
    EMU_SD_RTN errorstate = SD_OK;
    IDMAC_DescTypeDef desc = {0};
    uint8_t *tuning = malloc_safe(64);
    memset(tuning, 0, 64);
    uint32_t tuningdst = peripheralAddrConvert((uint32_t)tuning);
    hsd->SdTransferCplt  = 0;
    hsd->SdTransferErr   = SD_OK;

    if (hsd->CardType == HIGH_CAPACITY_SD_CARD)
    {
        dma->BlockSize = 64;
    }

    errorstate = SD_DMAConfig(hsd, dma);
    if (errorstate != SD_OK) {
        return errorstate;
    }
    Core_SDMMC_SetDBADDR(hsd->Instance, peripheralAddrConvert((uint32_t)&desc));
    Core_SDMMC_SetBYCTNT(hsd->Instance, 0x40);

    desc.des0 = SDMMC_DES0_OWN | SDMMC_DES0_FS |  SDMMC_DES0_CH | SDMMC_DES0_LD;
    desc.des1 = 64;
    desc.des2 = tuningdst;
    desc.des3 = 0x0;

    uint32_t RINTSTS_val, STATUS_val, RESP0_val;
    DLOG_Info("Send CMD19");
    sdmmc_cmdinitstructure.Argument         = 0x0;
    sdmmc_cmdinitstructure.CmdIndex         = SD_CMD_SEND_TUNING_PATTERN;
    sdmmc_cmdinitstructure.Response         = SDMMC_RESPONSE_R1;
    sdmmc_cmdinitstructure.Attribute        = SDMMC_CMD_START_CMD | 
                                              SDMMC_CMD_USE_HOLD_REG | 
                                              SDMMC_CMD_PRV_DAT_WAIT | 
                                              SDMMC_CMD_DAT_EXP      | 
                                              SDMMC_CMD_RESP_CRC     | 
                                              SDMMC_CMD_RESP_EXP;
    Core_SDMMC_SetRINTSTS(hsd->Instance, 0xFFFE);
    Core_SDMMC_SendCommand(hsd->Instance, &sdmmc_cmdinitstructure);
    Core_SDMMC_WaiteCmdDone(hsd->Instance);  

    RESP0_val = Core_SDMMC_GetRESP0(hsd->Instance);
    DLOG_Info("RESP0 = %x", RESP0_val);
    RESP0_val = Core_SDMMC_GetRINTSTS(hsd->Instance);
    DLOG_Info("RINTSTS = %x", RESP0_val);
    RESP0_val = Core_SDMMC_GetSTATUS(hsd->Instance);
    DLOG_Info("STATUS = %x", RESP0_val);

/*     sd_delay_ms(500); */
    unsigned int readAddress = tuningdst;
    unsigned char row;
    readAddress -= (readAddress % 4);
    for (row = 0; row < 2; row++)
    {
        DLOG_Info("0x%08x: 0x%08x 0x%08x 0x%08x 0x%08x 0x%08x 0x%08x 0x%08x 0x%08x ", 
                  readAddress,
                  *(uint32_t *)readAddress,
                  *(uint32_t *)(readAddress + 4),
                  *(uint32_t *)(readAddress + 8),
                  *(uint32_t *)(readAddress + 12),
                  *(uint32_t *)(readAddress + 16),
                  *(uint32_t *)(readAddress + 20),
                  *(uint32_t *)(readAddress + 24),
                  *(uint32_t *)(readAddress + 28));

        readAddress += 32;
    }
    DLOG_Output(1000);
    free_safe(tuning);
    free_safe(dma);
    hsd->SdTransferErr = errorstate;
    return errorstate;
}

EMU_SD_RTN Card_SD_ReadBlock_DMA(SD_HandleTypeDef *hsd, SDMMC_DMATransTypeDef *dma)
{
  SDMMC_CmdInitTypeDef sdmmc_cmdinitstructure;
  EMU_SD_RTN errorstate = SD_OK;
  IDMAC_DescTypeDef desc = {0};

    Core_SDMMC_WaiteCardBusy(hsd->Instance);  
/*   convert_to_transfer(hsd); */
  /* Initialize handle flags */
/*   SD_STATUS current_state; */
/*   current_state = sd_getState(hsd); */
/*   DLOG_Info("%d state = %d", __LINE__, current_state); */
    sdmmc_cmdinitstructure.CmdIndex         = SD_CMD_STOP_TRANSMISSION;
    sdmmc_cmdinitstructure.Response         = SDMMC_RESPONSE_R1;
    sdmmc_cmdinitstructure.Attribute        = SDMMC_CMD_START_CMD | 
                                              SDMMC_CMD_USE_HOLD_REG | 
                                              SDMMC_CMD_PRV_DAT_WAIT | 
                                              SDMMC_CMD_RESP_CRC | 
                                              SDMMC_CMD_RESP_EXP;
    Core_SDMMC_SetRINTSTS(hsd->Instance, SDMMC_RINTSTS_CMD_DONE);
    Core_SDMMC_SendCommand(hsd->Instance, &sdmmc_cmdinitstructure);
    Core_SDMMC_WaiteCmdDone(hsd->Instance);
#if 0
  if (current_state == SD_CARD_PROGRAMMING)
  {
    DLOG_Error("error");
    return SD_FAIL;
  }
#endif
  
  hsd->SdTransferCplt  = 0;
  hsd->SdTransferErr   = SD_OK;

  if (hsd->CardType == HIGH_CAPACITY_SD_CARD)
  {
    dma->BlockSize = 512;
  }

  /* Configure the SD DPSM (Data Path State Machine) */
  errorstate = SD_DMAConfig(hsd, dma);
  Core_SDMMC_SetDBADDR(hsd->Instance, peripheralAddrConvert((uint32_t)&desc));
  Core_SDMMC_SetBYCTNT(hsd->Instance, dma->BlockSize);
  if (errorstate != SD_OK) {
    return errorstate;
  }

  desc.des0 = SDMMC_DES0_OWN | SDMMC_DES0_FS |  SDMMC_DES0_CH | SDMMC_DES0_LD;
  desc.des1 = dma->BlockSize;
  desc.des2 = dma->DstAddr;
  desc.des3 = 0x0;

  /* send CMD17*/
  sdmmc_cmdinitstructure.Argument         = dma->SrcAddr;
  sdmmc_cmdinitstructure.CmdIndex         = SD_CMD_READ_SINGLE_BLOCK;
  sdmmc_cmdinitstructure.Response         = SDMMC_RESPONSE_R1;
  sdmmc_cmdinitstructure.Attribute        = SDMMC_CMD_START_CMD | 
                                            SDMMC_CMD_USE_HOLD_REG | 
                                            SDMMC_CMD_PRV_DAT_WAIT | 
                                            SDMMC_CMD_DAT_EXP      | 
                                            SDMMC_CMD_RESP_CRC     | 
                                            SDMMC_CMD_RESP_EXP;
/*   Core_SDMMC_SetRINTSTS(hsd->Instance, SDMMC_RINTSTS_CMD_DONE | SDMMC_RINTSTS_DATA_OVER); */
  Core_SDMMC_SetRINTSTS(hsd->Instance, 0xFFFE);
  Core_SDMMC_SendCommand(hsd->Instance, &sdmmc_cmdinitstructure);

  /* Check for error conditions */
  Core_SDMMC_WaiteCmdDone(hsd->Instance);
  Core_SDMMC_WaiteDataOver(hsd->Instance);
//  Core_SDMMC_WaiteCardBusy(hsd->Instance);
  
  sdmmc_cmdinitstructure.CmdIndex         = SD_CMD_STOP_TRANSMISSION;
  sdmmc_cmdinitstructure.Response         = SDMMC_RESPONSE_R1;
  sdmmc_cmdinitstructure.Attribute        = SDMMC_CMD_START_CMD | 
                                            SDMMC_CMD_USE_HOLD_REG | 
                                            SDMMC_CMD_PRV_DAT_WAIT | 
                                            SDMMC_CMD_RESP_CRC | 
                                            SDMMC_CMD_RESP_EXP;
  Core_SDMMC_SetRINTSTS(hsd->Instance, SDMMC_RINTSTS_CMD_DONE);
  Core_SDMMC_SendCommand(hsd->Instance, &sdmmc_cmdinitstructure);
  Core_SDMMC_WaiteCmdDone(hsd->Instance);

  /* Update the SD transfer error in SD handle */
  hsd->SdTransferErr = errorstate;
  return errorstate;
}


EMU_SD_RTN Card_SD_ReadMultiBlocks_DMA(SD_HandleTypeDef *hsd, SDMMC_DMATransTypeDef *dma)
{
  SDMMC_CmdInitTypeDef sdmmc_cmdinitstructure;
  EMU_SD_RTN errorstate = SD_OK;
  uint32_t BlockIndex, TmpAddr = dma->DstAddr, DstAddr = dma->DstAddr;
  uint32_t BuffSize = BUFFSIZE8;
  uint32_t SectorDivid = dma->SectorNum / BuffSize;
  uint32_t SectorRmd = dma->SectorNum % BuffSize;

  SD_STATUS current_state;
  Core_SDMMC_WaiteCardBusy(hsd->Instance);  
  current_state = sd_getState(hsd);
/*   DLOG_Info("%d state = %d", __LINE__, current_state); */
  if (current_state == SD_CARD_PROGRAMMING)
  {
    DLOG_Error("error");
    return SD_FAIL;
  }

  convert_to_transfer(hsd);

  /* malloc the space for descriptor */
  IDMAC_DescTypeDef *desc = (IDMAC_DescTypeDef *)malloc_safe(sizeof(IDMAC_DescTypeDef) * (SectorDivid + SectorRmd));
  if (!desc){
    DLOG_Info("Malloc Failed! Exit Read\n");
    errorstate = SD_ERROR;
    return errorstate;
  }

  memset(desc, 0, sizeof(IDMAC_DescTypeDef) * (SectorDivid + SectorRmd));

  /* Initialize handle flags */
  hsd->SdTransferCplt  = 0;
  hsd->SdTransferErr   = SD_OK;

  if (hsd->CardType == HIGH_CAPACITY_SD_CARD)
  {
    dma->BlockSize = 512;
  }

  /* Configure the SD DPSM (Data Path State Machine) */
  errorstate = SD_DMAConfig(hsd, dma);
  Core_SDMMC_SetDBADDR(hsd->Instance, peripheralAddrConvert((uint32_t)&desc[0]));
  if (errorstate != SD_OK) {
    free_safe(desc);
    return errorstate;
  }

  if (SectorDivid)
  {
    Core_SDMMC_SetBYCTNT(hsd->Instance, SectorDivid * BuffSize * dma->BlockSize);
    for (BlockIndex = 0; BlockIndex < SectorDivid; BlockIndex++)
    {

      DstAddr = dma->DstAddr + dma->BlockSize * BuffSize * BlockIndex;

      if (BlockIndex == 0 && (SectorDivid != 1))
      {

        desc[BlockIndex].des0 = SDMMC_DES0_OWN | SDMMC_DES0_FS |  SDMMC_DES0_CH;
        desc[BlockIndex].des1 = dma->BlockSize * BuffSize;
        desc[BlockIndex].des2 = DstAddr;
        desc[BlockIndex].des3 = peripheralAddrConvert((uint32_t)&desc[BlockIndex+1]);

      }
      else if ((BlockIndex == 0) && (SectorDivid == 1))
      {

        desc[BlockIndex].des0 = SDMMC_DES0_OWN | SDMMC_DES0_CH | SDMMC_DES0_LD | SDMMC_DES0_FS;
        desc[BlockIndex].des1 = dma->BlockSize * BuffSize;
        desc[BlockIndex].des2 = DstAddr;
        desc[BlockIndex].des3 = 0x0;
        TmpAddr = DstAddr + dma->BlockSize * BuffSize;
      }
      else if (BlockIndex == SectorDivid - 1)
      {

        desc[BlockIndex].des0 = SDMMC_DES0_OWN | SDMMC_DES0_CH | SDMMC_DES0_LD;
        desc[BlockIndex].des1 = dma->BlockSize * BuffSize;
        desc[BlockIndex].des2 = DstAddr;
        desc[BlockIndex].des3 = 0x0;
        TmpAddr = DstAddr + dma->BlockSize * BuffSize;
      }
      else
      {
        desc[BlockIndex].des0 = SDMMC_DES0_OWN | SDMMC_DES0_CH;
        desc[BlockIndex].des1 = dma->BlockSize * BuffSize;
        desc[BlockIndex].des2 = DstAddr;
        desc[BlockIndex].des3 = peripheralAddrConvert((uint32_t)&desc[BlockIndex+1]);
      }
    }

/*
    uint32_t i,j;
    for (i = 0; i < BuffSize; i++)
    {
        DLOG_Info("desc[%d] = 0x%08x", i, desc[i].des0);        
        DLOG_Info("desc[%d] = 0x%08x", i, desc[i].des1);
        DLOG_Info("desc[%d] = 0x%08x", i, desc[i].des2);
        DLOG_Info("desc[%d] = 0x%08x", i, desc[i].des3);
        sd_delay_ms(100);
    }
*/

    /* send CMD18 */
    sdmmc_cmdinitstructure.Argument         = dma->SrcAddr;
    sdmmc_cmdinitstructure.CmdIndex         = SD_CMD_READ_MULTIPLE_BLOCK;
    sdmmc_cmdinitstructure.Response         = SDMMC_RESPONSE_R1;
    sdmmc_cmdinitstructure.Attribute        = SDMMC_CMD_START_CMD | 
                                              SDMMC_CMD_USE_HOLD_REG | 
                                              SDMMC_CMD_PRV_DAT_WAIT | 
                                              SDMMC_CMD_SEND_STOP    | 
                                              SDMMC_CMD_DAT_EXP      | 
                                              SDMMC_CMD_RESP_CRC     | 
                                              SDMMC_CMD_RESP_EXP;
    Core_SDMMC_SetRINTSTS(hsd->Instance, SDMMC_RINTSTS_CMD_DONE | SDMMC_RINTSTS_DATA_OVER);
    Core_SDMMC_SendCommand(hsd->Instance, &sdmmc_cmdinitstructure);
    /* Check for error conditions */
    Core_SDMMC_WaiteCmdDone(hsd->Instance);
    Core_SDMMC_WaiteDataOver(hsd->Instance);
    //Core_SDMMC_WaiteCardBusy(hsd->Instance);
  }
  
  if (SectorRmd)
  {

    if (SectorDivid)
    {
        TmpAddr = dma->DstAddr + SectorDivid * dma->BlockSize * BUFFSIZE8;

        hsd->SdTransferCplt  = 0;
        hsd->SdTransferErr   = SD_OK;

        if (hsd->CardType == HIGH_CAPACITY_SD_CARD)
        {
          dma->BlockSize = 512;
        }
    
        /* Configure the SD DPSM (Data Path State Machine) */
        errorstate = SD_DMAConfig(hsd, dma);
        Core_SDMMC_SetDBADDR(hsd->Instance, peripheralAddrConvert((uint32_t)&desc[0]));
        //memset(desc, 0, sizeof(IDMAC_DescTypeDef) * (SectorDivid + SectorRmd));    
        //Core_SDMMC_SetBYCTNT(hsd->Instance, SectorRmd * dma->BlockSize);
    }
    memset(desc, 0, sizeof(IDMAC_DescTypeDef) * (SectorDivid + SectorRmd));    
    Core_SDMMC_SetBYCTNT(hsd->Instance, SectorRmd * dma->BlockSize);
    
    // DLOG_Info("------------read rmd-----------------\n");
    for (BlockIndex = 0; BlockIndex < SectorRmd; BlockIndex++)
    {

      DstAddr = TmpAddr + dma->BlockSize * BlockIndex;

      if (BlockIndex == 0 && SectorRmd == 1)
      {
          desc[BlockIndex].des0 = SDMMC_DES0_OWN | SDMMC_DES0_FS |  SDMMC_DES0_CH | SDMMC_DES0_LD;
          desc[BlockIndex].des1 = dma->BlockSize;
          desc[BlockIndex].des2 = DstAddr;
          desc[BlockIndex].des3 = peripheralAddrConvert((uint32_t)&desc[BlockIndex]);
      }
      else if (BlockIndex == 0)
      {

        desc[BlockIndex].des0 = SDMMC_DES0_OWN | SDMMC_DES0_FS |  SDMMC_DES0_CH;
        desc[BlockIndex].des1 = dma->BlockSize;
        desc[BlockIndex].des2 = DstAddr;
        desc[BlockIndex].des3 = peripheralAddrConvert((uint32_t)&desc[BlockIndex+1]);

      }
      else if (BlockIndex == SectorRmd - 1)
      {

        desc[BlockIndex].des0 = SDMMC_DES0_OWN | SDMMC_DES0_CH | SDMMC_DES0_LD;
        desc[BlockIndex].des1 = dma->BlockSize;
        desc[BlockIndex].des2 = DstAddr;
        desc[BlockIndex].des3 = 0x0;
      }
      else
      {
        desc[BlockIndex].des0 = SDMMC_DES0_OWN | SDMMC_DES0_CH;
        desc[BlockIndex].des1 = dma->BlockSize;
        desc[BlockIndex].des2 = DstAddr;
        desc[BlockIndex].des3 = peripheralAddrConvert((uint32_t)&desc[BlockIndex+1]);
      }
    }
    // DLOG_Info("Rmd Addr = %x", dma->SrcAddr + SectorDivid * BuffSize * dma->BlockSize);
    /* send CMD18 */
    sdmmc_cmdinitstructure.Argument         = (dma->SrcAddr + SectorDivid * BuffSize);
    sdmmc_cmdinitstructure.CmdIndex         = SD_CMD_READ_MULTIPLE_BLOCK;
    sdmmc_cmdinitstructure.Response         = SDMMC_RESPONSE_R1;
    sdmmc_cmdinitstructure.Attribute        = SDMMC_CMD_START_CMD | 
                                              SDMMC_CMD_USE_HOLD_REG | 
                                              SDMMC_CMD_PRV_DAT_WAIT | 
                                              SDMMC_CMD_SEND_STOP    | 
                                              SDMMC_CMD_DAT_EXP      | 
                                              SDMMC_CMD_RESP_CRC     | 
                                              SDMMC_CMD_RESP_EXP;
    Core_SDMMC_SetRINTSTS(hsd->Instance, SDMMC_RINTSTS_CMD_DONE | SDMMC_RINTSTS_DATA_OVER);
    Core_SDMMC_SendCommand(hsd->Instance, &sdmmc_cmdinitstructure);
    Core_SDMMC_WaiteCmdDone(hsd->Instance);
    Core_SDMMC_WaiteDataOver(hsd->Instance);
/*     Core_SDMMC_WaiteCardBusy(hsd->Instance); */
  }

  free_safe(desc);

  sdmmc_cmdinitstructure.Argument         = (uint32_t)(hsd->RCA << 16);
  sdmmc_cmdinitstructure.CmdIndex         = SD_CMD_STOP_TRANSMISSION;
  sdmmc_cmdinitstructure.Response         = SDMMC_RESPONSE_R1;
  sdmmc_cmdinitstructure.Attribute        = SDMMC_CMD_START_CMD | 
                                            SDMMC_CMD_USE_HOLD_REG | 
                                            SDMMC_CMD_PRV_DAT_WAIT | 
                                            SDMMC_CMD_RESP_CRC | 
                                            SDMMC_CMD_RESP_EXP;
  Core_SDMMC_SetRINTSTS(hsd->Instance, SDMMC_RINTSTS_CMD_DONE);
  Core_SDMMC_SendCommand(hsd->Instance, &sdmmc_cmdinitstructure);
  Core_SDMMC_WaiteCmdDone(hsd->Instance);
  
  /* Update the SD transfer error in SD handle */
  hsd->SdTransferErr = errorstate;
  return errorstate;
}

EMU_SD_RTN Card_SD_WriteBlock_DMA(SD_HandleTypeDef *hsd, SDMMC_DMATransTypeDef *dma)
{
  SDMMC_CmdInitTypeDef sdmmc_cmdinitstructure;
  EMU_SD_RTN errorstate = SD_OK;
  IDMAC_DescTypeDef desc = {0};
  
/*   DLOG_Info("%d state = %d", __LINE__, sd_getState(hsd)); */

  Core_SDMMC_WaiteCardBusy(hsd->Instance);  
  convert_to_transfer(hsd);
  
  /* Initialize handle flags */
  hsd->SdTransferCplt  = 0;
  hsd->SdTransferErr   = SD_OK;
  /* Initialize SD Read operation */
  hsd->SdOperation = SD_WRITE_SINGLE_BLOCK;

  if (hsd->CardType == HIGH_CAPACITY_SD_CARD)
  {
    dma->BlockSize = 512;
  }

  /* Configure the SD DPSM (Data Path State Machine) */
  errorstate = SD_DMAConfig(hsd, dma);
  Core_SDMMC_SetDBADDR(hsd->Instance, peripheralAddrConvert((uint32_t)&desc));
  Core_SDMMC_SetBYCTNT(hsd->Instance, dma->BlockSize);
  if (errorstate != SD_OK) {
    DLOG_Error("SD_DMAConfig Fail\n");
    return errorstate;
  }

  desc.des0 = SDMMC_DES0_OWN | SDMMC_DES0_FS |  SDMMC_DES0_CH | SDMMC_DES0_LD;
  desc.des1 = 0x200;
  desc.des2 = dma->SrcAddr;
  desc.des3 = 0x0;

  /* send CMD24 */
  sdmmc_cmdinitstructure.Argument         = dma->DstAddr;
  sdmmc_cmdinitstructure.CmdIndex         = SD_CMD_WRITE_SINGLE_BLOCK;
  sdmmc_cmdinitstructure.Response         = SDMMC_RESPONSE_R1;
  sdmmc_cmdinitstructure.Attribute        = SDMMC_CMD_START_CMD | 
                                            SDMMC_CMD_USE_HOLD_REG | 
                                            SDMMC_CMD_PRV_DAT_WAIT | 
                                            SDMMC_CMD_DAT_WRITE | 
                                            SDMMC_CMD_DAT_EXP      | 
                                            SDMMC_CMD_RESP_CRC     | 
                                            SDMMC_CMD_RESP_EXP;
  Core_SDMMC_SetRINTSTS(hsd->Instance, SDMMC_RINTSTS_CMD_DONE | SDMMC_RINTSTS_DATA_OVER);
  Core_SDMMC_SendCommand(hsd->Instance, &sdmmc_cmdinitstructure);
   /* Check for error conditions */
  Core_SDMMC_WaiteCmdDone(hsd->Instance);
  Core_SDMMC_WaiteDataOver(hsd->Instance);
  
  sdmmc_cmdinitstructure.CmdIndex         = SD_CMD_STOP_TRANSMISSION;
  sdmmc_cmdinitstructure.Response         = SDMMC_RESPONSE_R1;
  sdmmc_cmdinitstructure.Attribute        = SDMMC_CMD_START_CMD | 
                                            SDMMC_CMD_USE_HOLD_REG | 
                                            SDMMC_CMD_PRV_DAT_WAIT | 
                                            SDMMC_CMD_RESP_CRC | 
                                            SDMMC_CMD_RESP_EXP;
  Core_SDMMC_SetRINTSTS(hsd->Instance, SDMMC_RINTSTS_CMD_DONE);
  Core_SDMMC_SendCommand(hsd->Instance, &sdmmc_cmdinitstructure);
  Core_SDMMC_WaiteCmdDone(hsd->Instance);
  /* Update the SD transfer error in SD handle */
  hsd->SdTransferErr = errorstate;
  return errorstate;
}

EMU_SD_RTN Card_SD_WriteMultiBlocks_DMA(SD_HandleTypeDef *hsd, SDMMC_DMATransTypeDef *dma)
{
    SDMMC_CmdInitTypeDef sdmmc_cmdinitstructure;
    EMU_SD_RTN errorstate = SD_OK;
    uint32_t BlockIndex, TmpAddr = dma->SrcAddr, SrcAddr = dma->SrcAddr;
    uint32_t BuffSize = BUFFSIZE8;
    uint32_t SectorDivid = dma->SectorNum / BuffSize;
    uint32_t SectorRmd = dma->SectorNum % BuffSize;    

/*     DLOG_Info("%d state = %d", __LINE__, sd_getState(hsd)); */

    Core_SDMMC_WaiteCardBusy(hsd->Instance);  
    convert_to_transfer(hsd);

  IDMAC_DescTypeDef *desc = (IDMAC_DescTypeDef *)malloc_safe(sizeof(IDMAC_DescTypeDef) * (SectorDivid + SectorRmd));
  if (!desc){
    DLOG_Error("malloc failed! Exit writing\n");
    errorstate = SD_ERROR;
    return errorstate;
  }
    memset(desc, 0, sizeof(IDMAC_DescTypeDef) * (SectorDivid + SectorRmd));


  /* Initialize handle flags */
  hsd->SdTransferCplt  = 0;
  hsd->SdTransferErr   = SD_OK;
  /* Initialize SD Read operation */
  hsd->SdOperation = SD_WRITE_MULTIPLE_BLOCK;

#if 0
  if (hsd->CardType == HIGH_CAPACITY_SD_CARD)
  {
    dma->BlockSize = 512;
  }
#endif
  dma->BlockSize = 512;
  /* Configure the SD DPSM (Data Path State Machine) */
  errorstate = SD_DMAConfig(hsd, dma);
  Core_SDMMC_SetDBADDR(hsd->Instance, peripheralAddrConvert((uint32_t)&desc[0]));
  if (errorstate != SD_OK) {
    DLOG_Error("SD_DMAConfig Fail\n");
    free_safe(desc);
    return errorstate;
  }

  if (SectorDivid)
  { 
    Core_SDMMC_SetBYCTNT(hsd->Instance, SectorDivid * BuffSize * dma->BlockSize);
    for (BlockIndex = 0; BlockIndex < SectorDivid; BlockIndex++)
    {
      SrcAddr = dma->SrcAddr + dma->BlockSize * BuffSize * BlockIndex;
      if ((BlockIndex == 0) && (SectorDivid == 1))
      {
        desc[BlockIndex].des0 = SDMMC_DES0_OWN | SDMMC_DES0_FS | SDMMC_DES0_LD | SDMMC_DES0_CH;
        desc[BlockIndex].des1 = dma->BlockSize * BuffSize;
        desc[BlockIndex].des2 = SrcAddr;
        desc[BlockIndex].des3 = 0x0;
        TmpAddr = SrcAddr + dma->BlockSize * BuffSize;
      }
      else if ((BlockIndex == 0) && (SectorDivid != 1))
      {
        desc[BlockIndex].des0 = SDMMC_DES0_OWN | SDMMC_DES0_FS | SDMMC_DES0_CH;
        desc[BlockIndex].des1 = dma->BlockSize * BuffSize;
        desc[BlockIndex].des2 = SrcAddr;
        desc[BlockIndex].des3 = peripheralAddrConvert((uint32_t)&desc[BlockIndex+1]);

      }
      else if (BlockIndex == SectorDivid - 1)
      {
        desc[BlockIndex].des0 = SDMMC_DES0_OWN | SDMMC_DES0_LD | SDMMC_DES0_CH;
        desc[BlockIndex].des1 = dma->BlockSize * BuffSize;
        desc[BlockIndex].des2 = SrcAddr;
        desc[BlockIndex].des3 = 0x0;
        TmpAddr = SrcAddr + dma->BlockSize * BuffSize;
      }
      else
      {
        desc[BlockIndex].des0 = SDMMC_DES0_OWN | SDMMC_DES0_CH;
        desc[BlockIndex].des1 = dma->BlockSize * BuffSize;
        desc[BlockIndex].des2 = SrcAddr;
        desc[BlockIndex].des3 = peripheralAddrConvert((uint32_t)&desc[BlockIndex+1]);
      }
    }

    sdmmc_cmdinitstructure.Argument         = dma->DstAddr;
    sdmmc_cmdinitstructure.CmdIndex         = SD_CMD_WRITE_MULTIPLE_BLOCK;
    sdmmc_cmdinitstructure.Response         = SDMMC_RESPONSE_R1;
    sdmmc_cmdinitstructure.Attribute        = SDMMC_CMD_START_CMD | 
                                              SDMMC_CMD_USE_HOLD_REG | 
                                              SDMMC_CMD_PRV_DAT_WAIT | 
                                              SDMMC_CMD_SEND_STOP    | 
                                              SDMMC_CMD_DAT_WRITE | 
                                              SDMMC_CMD_DAT_EXP      | 
                                              SDMMC_CMD_RESP_CRC     | 
                                              SDMMC_CMD_RESP_EXP;
    
    Core_SDMMC_SetRINTSTS(hsd->Instance, SDMMC_RINTSTS_CMD_DONE | SDMMC_RINTSTS_DATA_OVER);
    Core_SDMMC_SendCommand(hsd->Instance, &sdmmc_cmdinitstructure);
    Core_SDMMC_WaiteCmdDone(hsd->Instance);
    Core_SDMMC_WaiteDataOver(hsd->Instance);
    //Core_SDMMC_WaiteCardBusy(hsd->Instance);

    sdmmc_cmdinitstructure.Argument         = (uint32_t)(hsd->RCA << 16);
    sdmmc_cmdinitstructure.CmdIndex         = SD_CMD_STOP_TRANSMISSION;
    sdmmc_cmdinitstructure.Response         = SDMMC_RESPONSE_R1;
    sdmmc_cmdinitstructure.Attribute        = SDMMC_CMD_START_CMD | 
                                              SDMMC_CMD_USE_HOLD_REG | 
                                              SDMMC_CMD_PRV_DAT_WAIT | 
                                              SDMMC_CMD_RESP_CRC | 
                                              SDMMC_CMD_RESP_EXP;
    Core_SDMMC_SetRINTSTS(hsd->Instance, SDMMC_RINTSTS_CMD_DONE);
    Core_SDMMC_SendCommand(hsd->Instance, &sdmmc_cmdinitstructure);
    Core_SDMMC_WaiteCmdDone(hsd->Instance);
  }


  if (SectorRmd)
  {
    if (SectorDivid)
    {
        Core_SDMMC_WaiteCardBusy(hsd->Instance);  
        convert_to_transfer(hsd);

        hsd->SdTransferCplt  = 0;
        hsd->SdTransferErr   = SD_OK;
        hsd->SdOperation = SD_WRITE_MULTIPLE_BLOCK;

        if (hsd->CardType == HIGH_CAPACITY_SD_CARD)
        {
            dma->BlockSize = 512;
        }

        Core_SDMMC_SetDBADDR(hsd->Instance, peripheralAddrConvert((uint32_t)&desc[0]));
        errorstate = SD_DMAConfig(hsd, dma);
        if (errorstate != SD_OK) {
            DLOG_Error("SD_DMAConfig Fail\n");
            free_safe(desc);
            return errorstate;
        }
    }
    
    memset(desc, 0, sizeof(IDMAC_DescTypeDef) * (SectorDivid + SectorRmd));    
    Core_SDMMC_SetBYCTNT(hsd->Instance, SectorRmd * dma->BlockSize);
    
    for (BlockIndex = 0; BlockIndex < SectorRmd; BlockIndex++)
    {
      SrcAddr = TmpAddr + dma->BlockSize * BlockIndex;
      if ((BlockIndex == 0) && (SectorRmd == 1))
      {
        // first and last
        desc[BlockIndex].des0 = SDMMC_DES0_OWN | SDMMC_DES0_FS |  SDMMC_DES0_CH | SDMMC_DES0_LD;
        desc[BlockIndex].des1 = dma->BlockSize;
        desc[BlockIndex].des2 = SrcAddr;
        desc[BlockIndex].des3 = peripheralAddrConvert((uint32_t)&desc[BlockIndex]);
      }
      else if (BlockIndex == 0)
      {
        // first
        desc[BlockIndex].des0 = SDMMC_DES0_OWN | SDMMC_DES0_FS |  SDMMC_DES0_CH;
        desc[BlockIndex].des1 = dma->BlockSize;
        desc[BlockIndex].des2 = SrcAddr;
        desc[BlockIndex].des3 = peripheralAddrConvert((uint32_t)&desc[BlockIndex+1]);

      }
      else if (BlockIndex ==  SectorRmd - 1)
      {
        // last
        desc[BlockIndex].des0 = SDMMC_DES0_OWN | SDMMC_DES0_CH | SDMMC_DES0_LD;
        desc[BlockIndex].des1 = dma->BlockSize;
        desc[BlockIndex].des2 = SrcAddr;
        desc[BlockIndex].des3 = 0x0;
      }
      else
      {
        // middle
        desc[BlockIndex].des0 = SDMMC_DES0_OWN | SDMMC_DES0_CH;
        desc[BlockIndex].des1 = dma->BlockSize;
        desc[BlockIndex].des2 = SrcAddr;
        desc[BlockIndex].des3 = peripheralAddrConvert((uint32_t)&desc[BlockIndex+1]);
      }
    }

    /* send CMD25 */
    sdmmc_cmdinitstructure.Argument         = dma->DstAddr + SectorDivid * BuffSize;
    sdmmc_cmdinitstructure.CmdIndex         = SD_CMD_WRITE_MULTIPLE_BLOCK;
    sdmmc_cmdinitstructure.Response         = SDMMC_RESPONSE_R1;
    sdmmc_cmdinitstructure.Attribute        = SDMMC_CMD_START_CMD | 
                                              SDMMC_CMD_USE_HOLD_REG | 
                                              SDMMC_CMD_PRV_DAT_WAIT | 
                                              SDMMC_CMD_SEND_STOP    | 
                                              SDMMC_CMD_DAT_WRITE | 
                                              SDMMC_CMD_DAT_EXP      | 
                                              SDMMC_CMD_RESP_CRC     | 
                                              SDMMC_CMD_RESP_EXP;
    Core_SDMMC_SetRINTSTS(hsd->Instance, SDMMC_RINTSTS_CMD_DONE | SDMMC_RINTSTS_DATA_OVER);    
    Core_SDMMC_SendCommand(hsd->Instance, &sdmmc_cmdinitstructure);
    Core_SDMMC_WaiteCmdDone(hsd->Instance);
    Core_SDMMC_WaiteDataOver(hsd->Instance);

    sdmmc_cmdinitstructure.Argument         = (uint32_t)(hsd->RCA << 16);
    sdmmc_cmdinitstructure.CmdIndex         = SD_CMD_STOP_TRANSMISSION;
    sdmmc_cmdinitstructure.Response         = SDMMC_RESPONSE_R1;
    sdmmc_cmdinitstructure.Attribute        = SDMMC_CMD_START_CMD | 
                                              SDMMC_CMD_USE_HOLD_REG | 
                                              SDMMC_CMD_PRV_DAT_WAIT | 
                                              SDMMC_CMD_RESP_CRC | 
                                              SDMMC_CMD_RESP_EXP;
    Core_SDMMC_SetRINTSTS(hsd->Instance, SDMMC_RINTSTS_CMD_DONE);
    Core_SDMMC_SendCommand(hsd->Instance, &sdmmc_cmdinitstructure);
    Core_SDMMC_WaiteCmdDone(hsd->Instance);
  }

  free_safe(desc);

  /* Update the SD transfer error in SD handle */
  hsd->SdTransferErr = errorstate;
  return errorstate;
}

/**
  * @brief  Erases the specified memory area of the given SD card.
  * @param  hsd: SD handle
  * @param  startaddr: Start byte address
  * @param  blocknum: End byte address
  * @retval SD Card error state
  */
EMU_SD_RTN Card_SD_Erase(SD_HandleTypeDef *hsd, uint32_t startBlkAddr, uint32_t blocknum)
{
    EMU_SD_RTN errorstate = SD_OK;
    SDMMC_CmdInitTypeDef sdmmc_cmdinitstructure;
    uint32_t get_val, cmd_done, data_over, card_busy;
    uint8_t cardstate = 0;

    Core_SDMMC_WaiteCardBusy(hsd->Instance);  
    convert_to_transfer(hsd);

  /* According to sd-card spec 3.0 ERASE_GROUP_START (CMD32) and erase_group_end(CMD33) */
    if ((hsd->CardType == STD_CAPACITY_SD_CARD)||(hsd->CardType == HIGH_CAPACITY_SD_CARD))
    {
        /* Send CMD32 SD_ERASE_GRP_START with argument as addr  */
        sdmmc_cmdinitstructure.Argument = (uint32_t)startBlkAddr;
        sdmmc_cmdinitstructure.CmdIndex = SD_CMD_SD_ERASE_WR_BLK_START;
        sdmmc_cmdinitstructure.Response = SDMMC_RESPONSE_R1;
        sdmmc_cmdinitstructure.Attribute = SDMMC_CMD_START_CMD | 
                                           SDMMC_CMD_USE_HOLD_REG | 
                                           SDMMC_CMD_PRV_DAT_WAIT | 
                                           SDMMC_CMD_RESP_CRC     | 
                                           SDMMC_CMD_RESP_EXP;
        Core_SDMMC_SetRINTSTS(hsd->Instance, SDMMC_RINTSTS_CMD_DONE);
        Core_SDMMC_SendCommand(hsd->Instance, &sdmmc_cmdinitstructure);
        /* Check for error conditions */
        Core_SDMMC_WaiteCmdDone(hsd->Instance);
//        Core_SDMMC_WaiteCardBusy(hsd->Instance);


        /* Send CMD33 SD_ERASE_GRP_END with argument as addr  */
        sdmmc_cmdinitstructure.Argument = (uint32_t)(startBlkAddr + blocknum - 1);
        sdmmc_cmdinitstructure.CmdIndex = SD_CMD_SD_ERASE_WR_BLK_END;
        sdmmc_cmdinitstructure.Response = SDMMC_RESPONSE_R1;
        sdmmc_cmdinitstructure.Attribute = SDMMC_CMD_START_CMD | 
                                           SDMMC_CMD_USE_HOLD_REG | 
                                           SDMMC_CMD_PRV_DAT_WAIT | 
                                           SDMMC_CMD_RESP_CRC     | 
                                           SDMMC_CMD_RESP_EXP;
        Core_SDMMC_SetRINTSTS(hsd->Instance, SDMMC_RINTSTS_CMD_DONE);
        Core_SDMMC_SendCommand(hsd->Instance, &sdmmc_cmdinitstructure);
        /* Check for error conditions */
        Core_SDMMC_WaiteCmdDone(hsd->Instance);
//        Core_SDMMC_WaiteCardBusy(hsd->Instance);
    }

    /* Send CMD38 ERASE */
    sdmmc_cmdinitstructure.Argument = 0;
    sdmmc_cmdinitstructure.CmdIndex = SD_CMD_ERASE;
    sdmmc_cmdinitstructure.Response = SDMMC_RESPONSE_R1;
    sdmmc_cmdinitstructure.Attribute = SDMMC_CMD_START_CMD    | 
                                       SDMMC_CMD_USE_HOLD_REG | 
                                       SDMMC_CMD_PRV_DAT_WAIT | 
                                       SDMMC_CMD_SEND_STOP    |
                                       SDMMC_CMD_RESP_CRC     | 
                                       SDMMC_CMD_RESP_EXP;
    Core_SDMMC_SetRINTSTS(hsd->Instance, SDMMC_RINTSTS_CMD_DONE);
    Core_SDMMC_SendCommand(hsd->Instance, &sdmmc_cmdinitstructure);
    Core_SDMMC_WaiteCmdDone(hsd->Instance);
    Core_SDMMC_WaiteCardBusy(hsd->Instance);

	/* Wait until the card is in programming state */

#if 1
    errorstate = SD_IsCardProgramming(hsd, &cardstate);
    uint8_t tmp_state = cardstate;
    while ( cardstate == SD_CARD_TRANSFER ||  cardstate == SD_CARD_PROGRAMMING)
    {
        errorstate = SD_IsCardProgramming(hsd, &cardstate);
        if (tmp_state != cardstate)
            break;
    }
        
    while ( cardstate == SD_CARD_PROGRAMMING )
    {
    	errorstate = SD_IsCardProgramming(hsd, &cardstate);

    	if ( cardstate == SD_CARD_TRANSFER)
    	{
    		break;
    	}
    }
#endif

    return errorstate;
}

/** 
  * @brief  Returns information about specific card.
  * @param  hsd: SD handle
  * @param  pCardInfo: Pointer to a SD_CardInfoTypedef structure that
  *         contains all SD cardinformation
  * @retval SD Card error state
  */
EMU_SD_RTN Card_SD_Get_CardInfo(SD_HandleTypeDef *hsd, SD_CardInfoTypedef *pCardInfo)
{
  EMU_SD_RTN errorstate = SD_OK;

  uint32_t tmp = 0;
  pCardInfo->CardType = (uint32_t)(hsd->CardType);
  pCardInfo->RCA      = (uint32_t)(hsd->RCA);

  /* Byte 0 */
  tmp = (hsd->CSD[3] & 0xFF000000) >> 24;
  pCardInfo->SD_csd.CSD_STRUCTURE      = (uint8_t)((tmp & 0xC0) >> 6);
  //DLOG_Info("CSD_STRUCTURE = %d\n", pCardInfo->SD_csd.CSD_STRUCTURE);
  pCardInfo->SD_csd.SysSpecVersion = (uint8_t)((tmp & 0x3C) >> 2);
  pCardInfo->SD_csd.Reserved1      = tmp & 0x03;
  /* Byte 1 */
  tmp = (hsd->CSD[3] & 0x00FF0000) >> 16;
  pCardInfo->SD_csd.TAAC = (uint8_t)tmp;
  //DLOG_Info("TAAC = %d\n", pCardInfo->SD_csd.TAAC);
  /* Byte 2 */
  tmp = (hsd->CSD[3] & 0x0000FF00) >> 8;
  pCardInfo->SD_csd.NSAC = (uint8_t)tmp;
  /* Byte 3 */
  //hsd->CardType = HIGH_CAPACITY_SD_CARD;
  tmp = hsd->CSD[3] & 0x000000FF;
  pCardInfo->SD_csd.TRAN_SPEED = (uint8_t)tmp;
  //DLOG_Info("TRAN_SPEED = %d\n", pCardInfo->SD_csd.TRAN_SPEED);
  /* Byte 4 */
  tmp = (hsd->CSD[1] & 0xFF000000) >> 24;
  pCardInfo->SD_csd.CCC = (uint16_t)(tmp << 4);

  /* Byte 5 */
  tmp = (hsd->CSD[2] & 0x00FF0000) >> 16;
  pCardInfo->SD_csd.CCC |= (uint16_t)((tmp & 0xF0) >> 4);
  //DLOG_Info("CCC = %d\n", pCardInfo->SD_csd.CCC);

  pCardInfo->SD_csd.READ_BL_LEN       = (uint8_t)(tmp & 0x0F);
  //DLOG_Info("READ_BL_LEN = %d\n", pCardInfo->SD_csd.READ_BL_LEN);

  /* Byte 6 */
  tmp = (hsd->CSD[2] & 0x0000FF00) >> 8;
  pCardInfo->SD_csd.READ_BL_PARTIAL   = (uint8_t)((tmp & 0x80) >> 7);
  //DLOG_Info("READ_BL_PARTIAL = %d\n", pCardInfo->SD_csd.READ_BL_PARTIAL);

  pCardInfo->SD_csd.WRITE_BLK_MISALIGN = (uint8_t)((tmp & 0x40) >> 6);
  //DLOG_Info("WRITE_BLK_MISALIGN = %d\n", pCardInfo->SD_csd.WRITE_BLK_MISALIGN);  

  pCardInfo->SD_csd.READ_BLK_MISALIGN = (uint8_t)((tmp & 0x20) >> 5);
  //DLOG_Info("READ_BLK_MISALIGN = %d\n", pCardInfo->SD_csd.READ_BLK_MISALIGN);  

  pCardInfo->SD_csd.DSP_IMP         = (uint8_t)((tmp & 0x10) >> 4);
  pCardInfo->SD_csd.Reserved2       = 0; /*!< Reserved */

  if (hsd->CardType == STD_CAPACITY_SD_CARD)
  {
    /* Byte 7 */
    pCardInfo->SD_csd.C_SIZE = (tmp & 0x03) << 10;
    tmp = (uint8_t)(hsd->CSD[2] & 0x000000FF);
    pCardInfo->SD_csd.C_SIZE |= (tmp) << 2;

    /* Byte 8 */
    tmp = (uint8_t)((hsd->CSD[1] & 0xFF000000) >> 24);
    pCardInfo->SD_csd.C_SIZE |= (tmp & 0xC0) >> 6;
    // DLOG_Info("C_SIZE = %d\n", pCardInfo->SD_csd.C_SIZE);  
    // DLOG_Output(200);

    pCardInfo->SD_csd.MaxRdCurrentVDDMin = (tmp & 0x38) >> 3;
    pCardInfo->SD_csd.MaxRdCurrentVDDMax = (tmp & 0x07);

    /* Byte 9 */
    tmp = (uint8_t)((hsd->CSD[1] & 0x00FF0000) >> 16);
    pCardInfo->SD_csd.MaxWrCurrentVDDMin = (tmp & 0xE0) >> 5;
    pCardInfo->SD_csd.MaxWrCurrentVDDMax = (tmp & 0x1C) >> 2;
    pCardInfo->SD_csd.C_SIZEMul      = (tmp & 0x03) << 1;
    
    /* Byte 10 */
    tmp = (uint8_t)((hsd->CSD[1] & 0x0000FF00) >> 8);
    pCardInfo->SD_csd.C_SIZEMul |= (tmp & 0x80) >> 7;
    //DLOG_Info("C_SIZEMul = %d\n", pCardInfo->SD_csd.C_SIZEMul);  

    pCardInfo->CardCapacity  = (pCardInfo->SD_csd.C_SIZE + 1) ;
    pCardInfo->CardCapacity *= (1 << (pCardInfo->SD_csd.C_SIZEMul + 2));
    pCardInfo->CardBlockSize = 1 << (pCardInfo->SD_csd.READ_BL_LEN);
    pCardInfo->CardCapacity *= pCardInfo->CardBlockSize;
    //DLOG_Info("CardCapacity = %d\n", pCardInfo->CardCapacity);
  }
  else if (hsd->CardType == HIGH_CAPACITY_SD_CARD)
  {
    /* Byte 7 */
    tmp = (uint8_t)(hsd->CSD[2] & 0x000000FF);
    pCardInfo->SD_csd.C_SIZE = (tmp & 0x3F) << 16;
    /* Byte 8 */
    tmp = (uint8_t)((hsd->CSD[1] & 0xFF000000) >> 24);
    pCardInfo->SD_csd.C_SIZE |= (tmp << 8);
    /* Byte 9 */
    tmp = (uint8_t)((hsd->CSD[1] & 0x00FF0000) >> 16);
    pCardInfo->SD_csd.C_SIZE |= (tmp);
    /* Byte 10 */
    tmp = (uint8_t)((hsd->CSD[1] & 0x0000FF00) >> 8);

    pCardInfo->CardCapacity  = ((pCardInfo->SD_csd.C_SIZE + 1)) * 1024;
    pCardInfo->CardBlockSize = 512;

    // DLOG_Info("pCardInfo->CardCapacity = %llu\n", pCardInfo->CardCapacity);  
    // DLOG_Info("C_SIZE = %d\n", pCardInfo->SD_csd.C_SIZE);  
    // DLOG_Output(200);
  }
  else
  {
    DLOG_Info("Error: Not supported card type\n");
    /* Not supported card type */
    errorstate = SD_ERROR;
  }

  pCardInfo->SD_csd.ERASE_BLK_EN = (tmp & 0x40) >> 6;
  pCardInfo->SD_csd.SECTOR_SIZE  = (tmp & 0x3F) << 1;

  /* Byte 11 */
  tmp = (uint8_t)(hsd->CSD[1] & 0x000000FF);
  pCardInfo->SD_csd.SECTOR_SIZE     |= (tmp & 0x80) >> 7;
  pCardInfo->SD_csd.WP_GRP_SIZE = (tmp & 0x7F);
  //DLOG_Info("sector size = %d\n", pCardInfo->SD_csd.SECTOR_SIZE);
  /* Byte 12 */
  tmp = (uint8_t)((hsd->CSD[0] & 0xFF000000) >> 24);
  pCardInfo->SD_csd.WP_GRP_ENABLE = (tmp & 0x80) >> 7;
  pCardInfo->SD_csd.ManDeflECC        = (tmp & 0x60) >> 5;
  pCardInfo->SD_csd.R2W_FACTOR       = (tmp & 0x1C) >> 2;
  pCardInfo->SD_csd.WRITE_BL_LEN     = (tmp & 0x03) << 2;

  /* Byte 13 */
  tmp = (uint8_t)((hsd->CSD[0] & 0x00FF0000) >> 16);
  pCardInfo->SD_csd.WRITE_BL_LEN      |= (tmp & 0xC0) >> 6;
  pCardInfo->SD_csd.WRITE_BL_PARTIAL = (tmp & 0x20) >> 5;
  pCardInfo->SD_csd.Reserved3           = 0;

  /* Byte 14 */
  tmp = (uint8_t)((hsd->CSD[0] & 0x0000FF00) >> 8);
  pCardInfo->SD_csd.FILE_FORMAT_GRP = (tmp & 0x80) >> 7;
  pCardInfo->SD_csd.COPY = (tmp & 0x40) >> 6;
  pCardInfo->SD_csd.PERM_WRITE_PROTECT    = (tmp & 0x20) >> 5;
  pCardInfo->SD_csd.TMP_WRITE_PROTECT    = (tmp & 0x10) >> 4;
  pCardInfo->SD_csd.FILE_FORMAT       = (tmp & 0x0C) >> 2;

  /* Byte 15 */
  tmp = (uint8_t)(hsd->CSD[0] & 0x000000FF);
  pCardInfo->SD_csd.CSD_CRC = (tmp & 0xFE) >> 1;
  pCardInfo->SD_csd.Reserved4 = 1;

  /* Byte 0 */
  tmp = (uint8_t)((hsd->CID[0] & 0xFF000000) >> 24);
  pCardInfo->SD_cid.ManufacturerID = tmp;
  /* Byte 1 */
  tmp = (uint8_t)((hsd->CID[0] & 0x00FF0000) >> 16);
  pCardInfo->SD_cid.OEM_AppliID = tmp << 8;

  /* Byte 2 */
  tmp = (uint8_t)((hsd->CID[0] & 0x000000FF00) >> 8);
  pCardInfo->SD_cid.OEM_AppliID |= tmp;

  /* Byte 3 */
  tmp = (uint8_t)(hsd->CID[0] & 0x000000FF);
  pCardInfo->SD_cid.ProdName1 = tmp << 24;

  /* Byte 4 */
  tmp = (uint8_t)((hsd->CID[1] & 0xFF000000) >> 24);
  pCardInfo->SD_cid.ProdName1 |= tmp << 16;

  /* Byte 5 */
  tmp = (uint8_t)((hsd->CID[1] & 0x00FF0000) >> 16);
  pCardInfo->SD_cid.ProdName1 |= tmp << 8;

  /* Byte 6 */
  tmp = (uint8_t)((hsd->CID[1] & 0x0000FF00) >> 8);
  pCardInfo->SD_cid.ProdName1 |= tmp;

  /* Byte 7 */
  tmp = (uint8_t)(hsd->CID[1] & 0x000000FF);
  pCardInfo->SD_cid.ProdName2 = tmp;

  /* Byte 8 */
  tmp = (uint8_t)((hsd->CID[2] & 0xFF000000) >> 24);
  pCardInfo->SD_cid.ProdRev = tmp;

  /* Byte 9 */
  tmp = (uint8_t)((hsd->CID[2] & 0x00FF0000) >> 16);
  pCardInfo->SD_cid.ProdSN = tmp << 24;

  /* Byte 10 */
  tmp = (uint8_t)((hsd->CID[2] & 0x0000FF00) >> 8);
  pCardInfo->SD_cid.ProdSN |= tmp << 16;

  /* Byte 11 */
  tmp = (uint8_t)(hsd->CID[2] & 0x000000FF);
  pCardInfo->SD_cid.ProdSN |= tmp << 8;

  /* Byte 12 */
  tmp = (uint8_t)((hsd->CID[3] & 0xFF000000) >> 24);
  pCardInfo->SD_cid.ProdSN |= tmp;

  /* Byte 13 */
  tmp = (uint8_t)((hsd->CID[3] & 0x00FF0000) >> 16);
  pCardInfo->SD_cid.Reserved1   |= (tmp & 0xF0) >> 4;
  pCardInfo->SD_cid.ManufactDate = (tmp & 0x0F) << 8;

  /* Byte 14 */
  tmp = (uint8_t)((hsd->CID[3] & 0x0000FF00) >> 8);
  pCardInfo->SD_cid.ManufactDate |= tmp;

  /* Byte 15 */
  tmp = (uint8_t)(hsd->CID[3] & 0x000000FF);
  pCardInfo->SD_cid.CID_CRC   = (tmp & 0xFE) >> 1;
  pCardInfo->SD_cid.Reserved2 = 1;

  return errorstate;
}

static void host_setting_clock(SD_HandleTypeDef *hsd)
{
    switch(hsd->SpeedMode)
    {
        case CARD_SDR12:
            Core_SDMMC_SetCLKDIV(hsd->Instance, 0x00000004);  //switch frequency from 400Khz to 25Mhz,SDR12
            DLOG_Info("Now in SDR12 Mode");
        break;
        case CARD_SDR25:
            Core_SDMMC_SetCLKDIV(hsd->Instance, 0x00000002);  //switch frequency from 400Khz to 50Mhz,SDR25
            DLOG_Info("Now in SDR25 Mode");
        break;
        case CARD_SDR50:
            Core_SDMMC_SetCLKDIV(hsd->Instance, 0x00000001);  //switch frequency from 400Khz to 100Mhz,SDR50
            DLOG_Info("Now in SDR50 Mode");
        break;
        case CARD_SDR104:
            Core_SDMMC_SetCLKDIV(hsd->Instance, 0x00000000);  //switch frequency from 400Khz to 100Mhz,SDR104
            DLOG_Info("Now in SDR104 Mode");
        break;
        default:
            Core_SDMMC_SetCLKDIV(hsd->Instance, 0x00000002);  //switch frequency from 400Khz to 100Mhz,SDR25
            DLOG_Info("Default in SDR25 Mode");
        break;
    }
}

static EMU_SD_RTN InitializeCard(SD_HandleTypeDef *hsd)
{

    SDMMC_CmdInitTypeDef sdmmc_cmdinitstructure;
    __IO EMU_SD_RTN errorstate = SD_OK;
    uint32_t get_val = 0, vs_busy = 0, loop = 0;
    uint32_t response;

    hsd->CardType = STD_CAPACITY_SD_CARD;

    Core_SDMMC_SetTMOUT(hsd->Instance, SDMMC_TMOUT_DEFAULT);
    Core_SDMMC_SetCTYPE(hsd->Instance, SDMMC_CTYPE_1BIT);
    /* FIFO_DEPTH = 16 words */
    Core_SDMMC_SetFIFOTH(hsd->Instance, 0x00070008);

    /* send CMD0 first*/
    sdmmc_cmdinitstructure.Argument         = 0;
    sdmmc_cmdinitstructure.CmdIndex         = SD_CMD_GO_IDLE_STATE;
    sdmmc_cmdinitstructure.Response         = SDMMC_RESPONSE_NO;
    sdmmc_cmdinitstructure.Attribute        = (SDMMC_CMD_START_CMD | 
                                              SDMMC_CMD_USE_HOLD_REG | 
                                              SDMMC_CMD_UPDATE_CLK | 
                                              SDMMC_CMD_PRV_DAT_WAIT);
    Core_SDMMC_SendCommand(hsd->Instance, &sdmmc_cmdinitstructure);
    Core_SDMMC_WaiteCmdStart(hsd->Instance);

    /* CMD0: GO_IDLE_STATE -----------------------------------------------------*/
    DLOG_Info("Send CMD0");
    //sd_delay_ms(500);  /*add the delay to fix the initialize fail when print to sram */
    SysTicks_DelayMS(500);
    sdmmc_cmdinitstructure.Argument         = 0;
    sdmmc_cmdinitstructure.CmdIndex         = SD_CMD_GO_IDLE_STATE;
    sdmmc_cmdinitstructure.Response         = SDMMC_RESPONSE_NO;
    sdmmc_cmdinitstructure.Attribute        = SDMMC_CMD_START_CMD | 
                                              SDMMC_CMD_USE_HOLD_REG | 
                                              SDMMC_CMD_PRV_DAT_WAIT |
                                              SDMMC_CMD_SEND_INIT;
    /* clear intreq status */
    Core_SDMMC_SetRINTSTS(hsd->Instance, SDMMC_RINTSTS_CMD_DONE);
    Core_SDMMC_SendCommand(hsd->Instance, &sdmmc_cmdinitstructure);
    Core_SDMMC_WaiteCmdDone(hsd->Instance);

    DLOG_Info("Send CMD8");
    sdmmc_cmdinitstructure.Argument         = SD_CHECK_PATTERN;
    sdmmc_cmdinitstructure.CmdIndex         = SD_CMD_SEND_IF_COND;
    sdmmc_cmdinitstructure.Response         = SDMMC_RESPONSE_R7;
    sdmmc_cmdinitstructure.Attribute        = SDMMC_CMD_START_CMD | 
                                              SDMMC_CMD_USE_HOLD_REG | 
                                              SDMMC_CMD_PRV_DAT_WAIT | 
                                              SDMMC_CMD_RESP_CRC | 
                                              SDMMC_CMD_RESP_EXP;
    Core_SDMMC_SetRINTSTS(hsd->Instance, SDMMC_RINTSTS_CMD_DONE);
    Core_SDMMC_SendCommand(hsd->Instance, &sdmmc_cmdinitstructure);
    Core_SDMMC_WaiteCmdDone(hsd->Instance);

    errorstate = SD_CmdResp7(hsd);
    if (errorstate != SD_OK)
    {
        DLOG_Error("CMD8: SD_UNSUPPORTED_VOLTAGE");
        return errorstate;
    }

    DLOG_Info("Send CMD55");
    sdmmc_cmdinitstructure.Argument         = 0;
    sdmmc_cmdinitstructure.CmdIndex         = SD_CMD_APP_CMD;
    sdmmc_cmdinitstructure.Response         = SDMMC_RESPONSE_R1;
    sdmmc_cmdinitstructure.Attribute        = SDMMC_CMD_START_CMD | 
                                              SDMMC_CMD_USE_HOLD_REG | 
                                              SDMMC_CMD_PRV_DAT_WAIT | 
                                              SDMMC_CMD_RESP_CRC | 
                                              SDMMC_CMD_RESP_EXP;
    Core_SDMMC_SetRINTSTS(hsd->Instance, SDMMC_RINTSTS_CMD_DONE);
    Core_SDMMC_SendCommand(hsd->Instance, &sdmmc_cmdinitstructure);
    Core_SDMMC_WaiteCmdDone(hsd->Instance);

    DLOG_Info("Send ACMD41");
    //sdmmc_cmdinitstructure.Argument         = SD_ACMD41_HCS | 
    //                                          SD_ACMD41_XPC | 
    //                                          SD_ACMD41_S18R;
    sdmmc_cmdinitstructure.Argument         = 0x51FF8000;
    sdmmc_cmdinitstructure.CmdIndex         = SD_CMD_APP_SD_SEND_OP_COND;
    sdmmc_cmdinitstructure.Response         = SDMMC_RESPONSE_R3;
    sdmmc_cmdinitstructure.Attribute        = SDMMC_CMD_START_CMD | 
                                              SDMMC_CMD_USE_HOLD_REG | 
                                              SDMMC_CMD_PRV_DAT_WAIT | 
                                              SDMMC_CMD_RESP_EXP;
    Core_SDMMC_SetRINTSTS(hsd->Instance, SDMMC_RINTSTS_CMD_DONE);
    Core_SDMMC_SendCommand(hsd->Instance, &sdmmc_cmdinitstructure);
    Core_SDMMC_WaiteCmdDone(hsd->Instance);
    response = Core_SDMMC_GetRESP0(hsd->Instance);
    DLOG_Info("ACMD41 RESP0 = 0x%x", response);

    vs_busy = (response & SD_ACMD41_BUSY);
    if (vs_busy == 0) 
    {
        DLOG_Info("ACMD41 Loop Start:");
        do 
        {
            //sd_delay_ms(200);
            SysTicks_DelayMS(20);
            /* Send CMD55 */
            sdmmc_cmdinitstructure.Argument         = 0;
            sdmmc_cmdinitstructure.CmdIndex         = SD_CMD_APP_CMD;
            sdmmc_cmdinitstructure.Response         = SDMMC_RESPONSE_R1;
            sdmmc_cmdinitstructure.Attribute        = SDMMC_CMD_START_CMD | 
                                                    SDMMC_CMD_USE_HOLD_REG | 
                                                    SDMMC_CMD_PRV_DAT_WAIT | 
                                                    SDMMC_CMD_RESP_CRC | 
                                                    SDMMC_CMD_RESP_EXP;
            Core_SDMMC_SetRINTSTS(hsd->Instance, SDMMC_RINTSTS_CMD_DONE);
            Core_SDMMC_SendCommand(hsd->Instance, &sdmmc_cmdinitstructure);
            Core_SDMMC_WaiteCmdDone(hsd->Instance);

            sdmmc_cmdinitstructure.Argument         = 0x51FF8000;
            sdmmc_cmdinitstructure.CmdIndex         = SD_CMD_APP_SD_SEND_OP_COND;
            sdmmc_cmdinitstructure.Response         = SDMMC_RESPONSE_R3;
            sdmmc_cmdinitstructure.Attribute        = SDMMC_CMD_START_CMD | 
                                                    SDMMC_CMD_USE_HOLD_REG | 
                                                    SDMMC_CMD_PRV_DAT_WAIT | 
                                                    SDMMC_CMD_RESP_EXP;
            Core_SDMMC_SetRINTSTS(hsd->Instance, SDMMC_RINTSTS_CMD_DONE);
            Core_SDMMC_SendCommand(hsd->Instance, &sdmmc_cmdinitstructure);

            response = Core_SDMMC_GetResponse(hsd->Instance, SDMMC_RESP0);
            vs_busy = (response & SD_ACMD41_BUSY);

            response = Core_SDMMC_GetRESP0(hsd->Instance);
            if (loop == 1000) 
            {
                DLOG_Info("ACMD41 Loop Fail");
            }
            vs_busy = (response & SD_ACMD41_BUSY);

            loop++;
        } while ((vs_busy == 0) && loop <= 1000);

        if (vs_busy != 0) 
        {
            sd_delay_ms(10);
            //SysTicks_DelayMS(1);
            DLOG_Info("ACMD41 Loop Done, loop = %d", loop);
        }
    }

    if (response & SD_ACMD41_HCS) 
    {
        DLOG_Info("This is SDHC/SDXC card!");
    }
    else
    {
        DLOG_Info("This is SDSC card!");
    }

#if BUFFER_CHIP_SURPPORTED
    if (response & SD_ACMD41_S18R) 
    {
        //sd_delay_ms(10);
        SysTicks_DelayMS(1);
        DLOG_Info("1.8V Support");

        DLOG_Info("Send CMD11");
        //sd_delay_ms(500);
        SysTicks_DelayMS(10);
        /* send CMD11 to switch 1.8V bus signaling level */
        Core_SDMMC_SetCLKENA(hsd->Instance, 0x00001);
        sdmmc_cmdinitstructure.Argument  = 0x41ffffff;
        sdmmc_cmdinitstructure.CmdIndex  = SD_CMD_VOLTAGE_SWITCH;
        sdmmc_cmdinitstructure.Response  = SDMMC_RESPONSE_R1;
        sdmmc_cmdinitstructure.Attribute = SDMMC_CMD_START_CMD | 
                                           SDMMC_CMD_USE_HOLD_REG | 
                                           SDMMC_CMD_VOLT_SWITCH  | 
                                           SDMMC_CMD_PRV_DAT_WAIT | 
                                           SDMMC_CMD_RESP_EXP  | 
                                           SDMMC_CMD_RESP_CRC;
        Core_SDMMC_SetRINTSTS(hsd->Instance, SDMMC_RINTSTS_CMD_DONE | SDMMC_RINTSTS_HTO);
        Core_SDMMC_SendCommand(hsd->Instance, &sdmmc_cmdinitstructure);
        DLOG_Info("1.8v Switch Start");
        DLOG_Output(100);
/*
    31 start_cmd 0 Start command. Once command is taken by CIU, bit is cleared.
    When bit is set, host should not attempt to write to any command
    registers. If write is attempted, hardware lock error is set in raw
    interrupt register.
    Once command is sent and response is received from
    SD_MMC_CEATA cards, Command Done bit is set in raw interrupt
    register.
*/
        // Core_SDMMC_WaiteCmdDone(hsd->Instance);

        Core_SDMMC_WaiteVoltSwitchInt(hsd->Instance);
        // delay_ms(1);
        DLOG_Info("CMD11 RESP 1.8v Switch Success");

        /* disable all the clock */
        Core_SDMMC_SetCLKENA(hsd->Instance, 0x00000000);
        Core_SDMMC_SetUHSREG(hsd->Instance, 0x00000001); 
        //sd_delay_ms(10);
        SysTicks_DelayMS(1);
        sdmmc_cmdinitstructure.Argument  = 0x00000000;
        sdmmc_cmdinitstructure.CmdIndex  = SD_CMD_GO_IDLE_STATE;
        sdmmc_cmdinitstructure.Response  = SDMMC_RESPONSE_NO;
        sdmmc_cmdinitstructure.Attribute = SDMMC_CMD_START_CMD | 
                                           SDMMC_CMD_USE_HOLD_REG | 
                                           SDMMC_CMD_VOLT_SWITCH  | 
                                           SDMMC_CMD_PRV_DAT_WAIT | 
                                           SDMMC_CMD_UPDATE_CLK;
        Core_SDMMC_SendCommand(hsd->Instance, &sdmmc_cmdinitstructure);
        //sd_delay_ms(500);
        SysTicks_DelayMS(100);
        DLOG_Info("Host Supply 1.8v Clock");

        Core_SDMMC_SetCLKENA(hsd->Instance, 0x00001);

        sdmmc_cmdinitstructure.Argument  = 0x00000000;
        sdmmc_cmdinitstructure.CmdIndex  = SD_CMD_GO_IDLE_STATE;
        sdmmc_cmdinitstructure.Response  = SDMMC_RESPONSE_NO;
        sdmmc_cmdinitstructure.Attribute = SDMMC_CMD_START_CMD | 
                                           SDMMC_CMD_USE_HOLD_REG | 
                                           SDMMC_CMD_VOLT_SWITCH  | 
                                           SDMMC_CMD_PRV_DAT_WAIT | 
                                           SDMMC_CMD_UPDATE_CLK;
        Core_SDMMC_SetRINTSTS(hsd->Instance, SDMMC_RINTSTS_CMD_DONE | SDMMC_RINTSTS_HTO);
        Core_SDMMC_SendCommand(hsd->Instance, &sdmmc_cmdinitstructure);
        Core_SDMMC_WaiteCmdDone(hsd->Instance);
        Core_SDMMC_WaiteVoltSwitchInt(hsd->Instance);
        DLOG_Info("Voltage Switching Success!");
        //sd_delay_ms(50000);
        SysTicks_DelayMS(100);
    }
    else 
    {
        DLOG_Info("not support 1.8V card");
        hsd->CardType = STD_CAPACITY_SD_CARD;
        hsd->SpeedMode = CARD_SDR12;
        errorstate = SD_OK;
    }
#endif

    return errorstate;
}
/**
  * @brief  Initializes all cards or single card as the case may be Card(s) come
  *         into standby state.
  * @param  hsd: SD handle
  * @retval SD Card error state
  */
static EMU_SD_RTN IdentificateCard(SD_HandleTypeDef *hsd,SD_CardInfoTypedef *SDCardInfo)
{
    SDMMC_CmdInitTypeDef sdmmc_cmdinitstructure;
    EMU_SD_RTN errorstate = SD_OK;
    uint16_t sd_rca = 1;
    uint32_t cmd_done, response, get_val;
    EMU_SD_RTN ret;
  
    if (hsd->CardType != SECURE_DIGITAL_IO_CARD)
    {
        DLOG_Info("Send CMD2");
        sdmmc_cmdinitstructure.Argument = 0;
        sdmmc_cmdinitstructure.CmdIndex = SD_CMD_ALL_SEND_CID;
        sdmmc_cmdinitstructure.Response = SDMMC_RESPONSE_R2;
        sdmmc_cmdinitstructure.Attribute  = SDMMC_CMD_START_CMD | 
                                            SDMMC_CMD_USE_HOLD_REG | 
                                            SDMMC_CMD_PRV_DAT_WAIT | 
                                            SDMMC_CMD_RESP_CRC     | 
                                            SDMMC_CMD_RESP_LONG    | 
                                            SDMMC_CMD_RESP_EXP;
        Core_SDMMC_SetRINTSTS(hsd->Instance, SDMMC_RINTSTS_CMD_DONE);
        Core_SDMMC_SendCommand(hsd->Instance, &sdmmc_cmdinitstructure);
        Core_SDMMC_WaiteCmdDone(hsd->Instance);
        Core_SDMMC_WaiteCardBusy(hsd->Instance);
        hsd->CID[0] = Core_SDMMC_GetResponse(hsd->Instance, SDMMC_RESP0);
        hsd->CID[1] = Core_SDMMC_GetResponse(hsd->Instance, SDMMC_RESP1);
        hsd->CID[2] = Core_SDMMC_GetResponse(hsd->Instance, SDMMC_RESP2);
        hsd->CID[3] = Core_SDMMC_GetResponse(hsd->Instance, SDMMC_RESP3);
        //sd_delay_ms(1000);
        SysTicks_DelayMS(10);
    }

    if ((hsd->CardType == STD_CAPACITY_SD_CARD)    ||  
       (hsd->CardType == SECURE_DIGITAL_IO_COMBO_CARD) || 
       (hsd->CardType == HIGH_CAPACITY_SD_CARD))
    {
        DLOG_Info("Send CMD3");
        sdmmc_cmdinitstructure.Argument         = 0;
        sdmmc_cmdinitstructure.CmdIndex         = SD_CMD_SEND_RELATIVE_ADDR;
        sdmmc_cmdinitstructure.Response         = SDMMC_RESPONSE_R6;
        sdmmc_cmdinitstructure.Attribute        = SDMMC_CMD_START_CMD | 
                                                  SDMMC_CMD_USE_HOLD_REG | 
                                                  SDMMC_CMD_PRV_DAT_WAIT | 
                                                  SDMMC_CMD_RESP_CRC | 
                                                  SDMMC_CMD_RESP_EXP;
        Core_SDMMC_SetRINTSTS(hsd->Instance, SDMMC_RINTSTS_CMD_DONE);
        Core_SDMMC_SendCommand(hsd->Instance, &sdmmc_cmdinitstructure);
        //sd_delay_ms(1000);
        SysTicks_DelayMS(10);
        Core_SDMMC_WaiteCmdDone(hsd->Instance);
        response = Core_SDMMC_GetResponse(hsd->Instance, SDMMC_RESP0);
        sd_rca = (uint32_t)(response >> 16) & 0x0000FFFF;

        DLOG_Info("Card Relative Address = 0x%x", sd_rca);
        hsd->RCA = sd_rca;
    }

    if (hsd->CardType != SECURE_DIGITAL_IO_CARD)
    {
        hsd->RCA = sd_rca;
        DLOG_Info("Send CMD10");
        sdmmc_cmdinitstructure.Argument         = (uint32_t)(hsd->RCA << 16);
        sdmmc_cmdinitstructure.CmdIndex         = SD_CMD_SEND_CID;
        sdmmc_cmdinitstructure.Response         = SDMMC_RESPONSE_R2;
        sdmmc_cmdinitstructure.Attribute        = SDMMC_CMD_START_CMD | 
                                                  SDMMC_CMD_USE_HOLD_REG | 
                                                  SDMMC_CMD_PRV_DAT_WAIT | 
                                                  SDMMC_CMD_RESP_CRC     | 
                                                  SDMMC_CMD_RESP_LONG    | 
                                                  SDMMC_CMD_RESP_EXP;
        Core_SDMMC_SetRINTSTS(hsd->Instance, SDMMC_RINTSTS_CMD_DONE);
        Core_SDMMC_SendCommand(hsd->Instance, &sdmmc_cmdinitstructure);
        Core_SDMMC_WaiteCmdDone(hsd->Instance);
        hsd->CID[3] = Core_SDMMC_GetResponse(hsd->Instance, SDMMC_RESP0);
        hsd->CID[2] = Core_SDMMC_GetResponse(hsd->Instance, SDMMC_RESP1);
        hsd->CID[1] = Core_SDMMC_GetResponse(hsd->Instance, SDMMC_RESP2);
        hsd->CID[0] = Core_SDMMC_GetResponse(hsd->Instance, SDMMC_RESP3);
    }
    //sd_delay_ms(1000);
    SysTicks_DelayMS(10);
    DLOG_Info("Send CMD9");
    sdmmc_cmdinitstructure.Argument         = (uint32_t)(hsd->RCA << 16);
    sdmmc_cmdinitstructure.CmdIndex         = SD_CMD_SEND_CSD;
    sdmmc_cmdinitstructure.Response         = SDMMC_RESPONSE_R2;
    sdmmc_cmdinitstructure.Attribute        = SDMMC_CMD_START_CMD | 
                                              SDMMC_CMD_USE_HOLD_REG | 
                                              SDMMC_CMD_PRV_DAT_WAIT | 
                                              SDMMC_CMD_RESP_CRC     | 
                                              SDMMC_CMD_RESP_LONG    | 
                                              SDMMC_CMD_RESP_EXP;
    Core_SDMMC_SetRINTSTS(hsd->Instance, SDMMC_RINTSTS_CMD_DONE);
    Core_SDMMC_SendCommand(hsd->Instance, &sdmmc_cmdinitstructure);
    Core_SDMMC_WaiteCmdDone(hsd->Instance);
    hsd->CSD[0] = Core_SDMMC_GetResponse(hsd->Instance, SDMMC_RESP0);
    hsd->CSD[1] = Core_SDMMC_GetResponse(hsd->Instance, SDMMC_RESP1);
    hsd->CSD[2] = Core_SDMMC_GetResponse(hsd->Instance, SDMMC_RESP2);
    hsd->CSD[3] = Core_SDMMC_GetResponse(hsd->Instance, SDMMC_RESP3);
    //sd_delay_ms(1000);
    SysTicks_DelayMS(10);
    DLOG_Info("Send CMD7");
    sdmmc_cmdinitstructure.Argument         = (uint32_t)(hsd->RCA << 16);
    sdmmc_cmdinitstructure.CmdIndex         = SD_CMD_SEL_DESEL_CARD;
    sdmmc_cmdinitstructure.Response         = SDMMC_RESPONSE_R1;
    sdmmc_cmdinitstructure.Attribute        = SDMMC_CMD_START_CMD | 
                                              SDMMC_CMD_USE_HOLD_REG | 
                                              SDMMC_CMD_PRV_DAT_WAIT | 
                                              SDMMC_CMD_RESP_CRC    | 
                                              SDMMC_CMD_RESP_EXP;
    Core_SDMMC_SetRINTSTS(hsd->Instance, SDMMC_RINTSTS_CMD_DONE);
    Core_SDMMC_SendCommand(hsd->Instance, &sdmmc_cmdinitstructure);
    Core_SDMMC_WaiteCmdDone(hsd->Instance);
    //sd_delay_ms(1000);
    SysTicks_DelayMS(10);
    DLOG_Info("Send CMD55");
    sdmmc_cmdinitstructure.Argument         = (uint32_t)(hsd->RCA << 16);
    sdmmc_cmdinitstructure.CmdIndex         = SD_CMD_APP_CMD;
    sdmmc_cmdinitstructure.Response         = SDMMC_RESPONSE_R1;
    sdmmc_cmdinitstructure.Attribute        = SDMMC_CMD_START_CMD | 
                                              SDMMC_CMD_USE_HOLD_REG | 
                                              SDMMC_CMD_PRV_DAT_WAIT | 
                                              SDMMC_CMD_RESP_CRC | 
                                              SDMMC_CMD_RESP_EXP;
    Core_SDMMC_SetRINTSTS(hsd->Instance, SDMMC_RINTSTS_CMD_DONE);
    Core_SDMMC_SendCommand(hsd->Instance, &sdmmc_cmdinitstructure);
    Core_SDMMC_WaiteCmdDone(hsd->Instance);
    //sd_delay_ms(1000);
    SysTicks_DelayMS(10);
    DLOG_Info("Send ACMD6");
    sdmmc_cmdinitstructure.Argument         = SDMMC_BUS_WIDE_4B;
    sdmmc_cmdinitstructure.CmdIndex         = SD_CMD_APP_SD_SET_BUSWIDTH;
    sdmmc_cmdinitstructure.Response         = SDMMC_RESPONSE_R1;
    sdmmc_cmdinitstructure.Attribute        = SDMMC_CMD_START_CMD | 
                                              SDMMC_CMD_USE_HOLD_REG | 
                                              SDMMC_CMD_PRV_DAT_WAIT | 
                                              SDMMC_CMD_RESP_CRC | 
                                              SDMMC_CMD_RESP_EXP;
    Core_SDMMC_SetRINTSTS(hsd->Instance, SDMMC_RINTSTS_CMD_DONE);
    Core_SDMMC_SendCommand(hsd->Instance, &sdmmc_cmdinitstructure);
    Core_SDMMC_WaiteCmdDone(hsd->Instance);
    DLOG_Info("Send ACMD6 response %x", Core_SDMMC_GetRESP0(hsd->Instance));

    if (hsd->SpeedMode == CARD_SDR25 ||
        hsd->SpeedMode == CARD_SDR50 || 
        hsd->SpeedMode == CARD_SDR104)
    {             
      Card_SwtichCLK(hsd);
      //ret = Card_SD_CMD6(hsd);
      ret = Card_SD_CMD6_check_patten(hsd);
      if (SD_FAIL == ret)
      {
          hsd->SpeedMode = CARD_SDR25;
          Card_SwtichCLK(hsd);
          ret = Card_SD_CMD6_check_patten(hsd);
          if (SD_OK == ret)
          {
            ret = Card_SD_CMD6(hsd);
          }
      }
      else
      {    
          ret = Card_SD_CMD6(hsd);
          Card_SwtichCLK(hsd);            
      }

      if (hsd->SpeedMode == CARD_SDR50 || 
          hsd->SpeedMode == CARD_SDR104)
      {
          errorstate = SD_Tuning(hsd);
          DLOG_Info("tuning status = %d", errorstate);
      }

    }
    else
    {
        Card_SwtichCLK(hsd);
    }

    return errorstate;
}

/**
  * @brief  Enquires cards about their operating voltage and configures clock
  *         controls and stores SD information that will be needed in future
  *         in the SD handle.
  * @param  hsd: SD handle
  * @retval SD Card error state
  */
static EMU_SD_RTN PowerOnCard(SD_HandleTypeDef *hsd)
{
    Core_SDMMC_SetPWREN(hsd->Instance, SDMMC_PWREN_0);

    Core_SDMMC_SetRST_N(hsd->Instance, 0x0);
    Core_SDMMC_SetRST_N(hsd->Instance, 0x1);

    // sd_delay_ms(100);
    //sd_delay_ms(500);
    SysTicks_DelayMS(20);
    
    Core_SDMMC_SetCTRL(hsd->Instance, SDMMC_CTRL_CONTROLLER_RESET |
                                    SDMMC_CTRL_FIFO_RESET |
                                    SDMMC_CTRL_DMA_RESET);
    Core_SDMMC_SetINTMASK(hsd->Instance, SDMMC_INTMASK_CARD_DETECT);
    Core_SDMMC_SetRINTSTS(hsd->Instance, 0xFFFFFFFF);
  Core_SDMMC_SetCTRL(hsd->Instance, SDMMC_CTRL_INT_ENABLE );
  // SD clock = 50MHz
  // Core_SDMMC_SetCLKDIV(hsd->Instance, SDMMC_CLKDIV_BIT1);
  //SD clock = 1MHz
  //Core_SDMMC_SetCLKDIV(hsd->Instance, SDMMC_CLKDIV_BIT2 | \
  //                     SDMMC_CLKDIV_BIT5 | \
  //                     SDMMC_CLKDIV_BIT6 );
  //SD clock = 25MHz
  // Core_SDMMC_SetCLKDIV(hsd->Instance, SDMMC_CLKDIV_BIT2);
  //SD clock = 100MHz
  // Core_SDMMC_SetCLKDIV(hsd->Instance, SDMMC_CLKDIV_BIT0);
  // Core_SDMMC_SetCLKSRC(hsd->Instance, SDMMC_CLKSRC_CLKDIV0);
  // SD clock = 400KHz
  Core_SDMMC_SetCLKDIV(hsd->Instance, 0xFA);
  Core_SDMMC_SetCLKSRC(hsd->Instance, SDMMC_CLKSRC_CLKDIV0);
  Core_SDMMC_SetCLKENA(hsd->Instance, 0x00000001);

  
  Core_SDMMC_SetUHSREG(hsd->Instance, 0x0);
  return 0;
}
/**
  * @brief  deselect the card.
  * @param  hsd: SD handle
  * @retval SD Card error state
  */
static EMU_SD_RTN PowerOffCard(SD_HandleTypeDef *hsd)
{
    EMU_SD_RTN errorstate = SD_OK;
    SDMMC_CmdInitTypeDef sdmmc_cmdinitstructure;

    if(SD_CARD_TRANSFER == sd_getState(hsd))
    {
        sdmmc_cmdinitstructure.Argument         = hsd->RCA << 16;
        sdmmc_cmdinitstructure.CmdIndex         = SD_CMD_SEL_DESEL_CARD;
        sdmmc_cmdinitstructure.Response         = SDMMC_RESPONSE_R1;
        sdmmc_cmdinitstructure.Attribute        = SDMMC_CMD_START_CMD | 
                                                  SDMMC_CMD_USE_HOLD_REG | 
                                                  SDMMC_CMD_PRV_DAT_WAIT | 
                                                  SDMMC_CMD_RESP_CRC | 
                                                  SDMMC_CMD_RESP_EXP;
        Core_SDMMC_SetRINTSTS(hsd->Instance, SDMMC_RINTSTS_CMD_DONE);
        Core_SDMMC_SendCommand(hsd->Instance, &sdmmc_cmdinitstructure);
        /* waite for command finish*/
        Core_SDMMC_WaiteCmdDone(hsd->Instance);

        //     /* send CMD0 first*/
        // sdmmc_cmdinitstructure.Argument         = 0;
        // sdmmc_cmdinitstructure.CmdIndex         = SD_CMD_GO_IDLE_STATE;
        // sdmmc_cmdinitstructure.Response         = SDMMC_RESPONSE_NO;
        // sdmmc_cmdinitstructure.Attribute        = SDMMC_CMD_START_CMD | 
        //                                         SDMMC_CMD_USE_HOLD_REG | 
        //                                         SDMMC_CMD_UPDATE_CLK | 
        //                                         SDMMC_CMD_PRV_DAT_WAIT;
        // Core_SDMMC_SendCommand(hsd->Instance, &sdmmc_cmdinitstructure);
        // Core_SDMMC_WaiteCmdStart(hsd->Instance);



        DLOG_Info("deselect card");
    }
    
    Core_SDMMC_SetCLKENA(hsd->Instance, 0x0);
    sdmmc_cmdinitstructure.Argument         = 0;
    sdmmc_cmdinitstructure.CmdIndex         = SD_CMD_GO_IDLE_STATE;
    sdmmc_cmdinitstructure.Response         = SDMMC_RESPONSE_NO;
    sdmmc_cmdinitstructure.Attribute        = SDMMC_CMD_START_CMD | 
                                            SDMMC_CMD_USE_HOLD_REG | 
                                            SDMMC_CMD_UPDATE_CLK | 
                                            SDMMC_CMD_PRV_DAT_WAIT;
    Core_SDMMC_SendCommand(hsd->Instance, &sdmmc_cmdinitstructure);
    Core_SDMMC_WaiteCmdStart(hsd->Instance);

    Core_SDMMC_SetPWREN(hsd->Instance, 0x0);
    //sd_delay_ms(1);
    SysTicks_DelayMS(1);
    Core_SDMMC_SetINTMASK(hsd->Instance, SDMMC_INTMASK_CARD_DETECT);
    Core_SDMMC_SetCTRL(hsd->Instance, SDMMC_CTRL_INT_ENABLE );
    Core_SDMMC_SetRINTSTS(hsd->Instance, SDMMC_RINTSTS_CARD_DETECT);
    return errorstate;
}

/**
  * @brief  Returns the current card's status.
  * @param  hsd: SD handle
  * @param  pCardStatus: pointer to the buffer that will contain the SD card
  *         status (Card Status register)
  * @retval SD Card error state
  */
EMU_SD_RTN SD_GetState(SD_HandleTypeDef *hsd, uint32_t *CardStatus)
{
  SDMMC_CmdInitTypeDef sdmmc_cmdinitstructure;
  EMU_SD_RTN errorstate = SD_OK;

  /* Send Status command */
  sdmmc_cmdinitstructure.Argument         = (uint32_t)(hsd->RCA << 16);
  sdmmc_cmdinitstructure.CmdIndex         = SD_CMD_SEND_STATUS;
  sdmmc_cmdinitstructure.Response         = SDMMC_RESPONSE_R1;
  sdmmc_cmdinitstructure.Attribute        = SDMMC_CMD_START_CMD | 
      SDMMC_CMD_USE_HOLD_REG | 
      SDMMC_CMD_PRV_DAT_WAIT | 
      SDMMC_CMD_RESP_CRC | 
      SDMMC_CMD_RESP_EXP;
  Core_SDMMC_SendCommand(hsd->Instance, &sdmmc_cmdinitstructure);
  Core_SDMMC_WaiteCmdDone(hsd->Instance);
  *CardStatus = Core_SDMMC_GetRESP0(hsd->Instance);
  // DLOG_Info("Card State = %x\n", *CardStatus);

  return errorstate;
}

/**
  * @brief  Checks for error conditions for CMD.
  * @param  hsd: SD handle
  * @retval SD Card error state
  */
static EMU_SD_RTN SD_CmdError(SD_HandleTypeDef *hsd)
{
  EMU_SD_RTN errorstate = SD_OK;
  uint32_t RINTSTS_val, cmd_done, endBitErr, startBitErr, hdLockErr, datStarvTo, resp_to;
  uint32_t datCrcErr, resp_crc_err, datTranOver, resp_err, errHappen;
    uint32_t start = SysTicks_GetTickCount();
  do {
    RINTSTS_val = Core_SDMMC_GetRINTSTS(hsd->Instance);
    cmd_done     = (RINTSTS_val & SDMMC_RINTSTS_CMD_DONE);
    endBitErr    = (RINTSTS_val & SDMMC_RINTSTS_EBE); //[15]
    startBitErr  = (RINTSTS_val & SDMMC_RINTSTS_SBE); //[13]
    hdLockErr    = (RINTSTS_val & SDMMC_RINTSTS_HLE); //[12]
    datStarvTo   = (RINTSTS_val & SDMMC_RINTSTS_HTO); //[10]
    resp_to      = (RINTSTS_val & SDMMC_RINTSTS_RTO); //[8]
    datCrcErr    = (RINTSTS_val & SDMMC_RINTSTS_DCRC); //[7]
    resp_crc_err = (RINTSTS_val & SDMMC_RINTSTS_RCRC); //[6]
    datTranOver  = (RINTSTS_val & SDMMC_RINTSTS_DATA_OVER); //[3]
    resp_err     = (RINTSTS_val & SDMMC_RINTSTS_RESP_ERR); //[1]
    
    if((SysTicks_GetDiff(start, SysTicks_GetTickCount())) > 100)
    {
        DLOG_Error("time out");
    }
  } while (!cmd_done);

  // errHappen = endBitErr | startBitErr | hdLockErr | resp_to | datCrcErr | resp_crc_err | resp_err;
  errHappen = endBitErr | startBitErr | hdLockErr | datCrcErr | resp_crc_err | resp_err;
  if (errHappen) {
#ifdef ECHO
    DLOG_Info("CMD ERROR\n");
#endif
    return SD_ERROR;
  }
  return errorstate;
}

/**
  * @brief  Checks for error conditions for R7 response.
  * @param  hsd: SD handle
  * @retval SD Card error state
*/
static EMU_SD_RTN SD_CmdResp7(SD_HandleTypeDef *hsd)
{
    EMU_SD_RTN errorstate = SD_OK;
    uint32_t RESP_val;
    RESP_val = Core_SDMMC_GetResponse(hsd->Instance, SDMMC_RESP0);
    if (RESP_val == SD_CHECK_PATTERN) 
    {
        // DLOG_Info("CMD8 Resp Right: v2");
    }
    else 
    {
        errorstate = SD_UNSUPPORTED_VOLTAGE;
    }
    return errorstate;
}

/**
  * @brief  Checks for error conditions for R1 response.
  * @param  hsd: SD handle
  * @param  SD_CMD: The sent command index
  * @retval SD Card error state
  */
static EMU_SD_RTN SD_CmdResp1Error(SD_HandleTypeDef *hsd, uint32_t SD_CMD)
{
  EMU_SD_RTN errorstate = SD_OK;
  uint32_t response_r1;

  /* We have received response, retrieve it for analysis  */
  response_r1 = Core_SDMMC_GetResponse(hsd->Instance, SDMMC_RESP0);


  if ((response_r1 & SDMMC_RESP1_OUT_OF_RANGE) == SDMMC_RESP1_OUT_OF_RANGE)
  {
    return (SD_OUT_OF_RANGE);
  }

  if ((response_r1 & SDMMC_RESP1_ADDRESS_ERROR) == SDMMC_RESP1_ADDRESS_ERROR)
  {
    return (SD_ADDRESS_ERROR);
  }

  if ((response_r1 & SDMMC_RESP1_BLOCK_LEN_ERROR) == SDMMC_RESP1_BLOCK_LEN_ERROR)
  {
    return (SD_BLOCK_LEN_ERROR);
  }

  if ((response_r1 & SDMMC_RESP1_ERASE_SEQ_ERROR) == SDMMC_RESP1_ERASE_SEQ_ERROR)
  {
    return (SD_ERASE_SEQ_ERROR);
  }

  if ((response_r1 & SDMMC_RESP1_ERASE_PARAM) == SDMMC_RESP1_ERASE_PARAM)
  {
    return (SD_ERASE_PARAM);
  }

  if ((response_r1 & SDMMC_RESP1_WP_VIOLATION) == SDMMC_RESP1_WP_VIOLATION)
  {
    return (SD_WP_VIOLATION);
  }

  if ((response_r1 & SDMMC_RESP1_CARD_IS_LOCKED) == SDMMC_RESP1_CARD_IS_LOCKED)
  {
    return (SD_CARD_IS_LOCKED);
  }

  if ((response_r1 & SDMMC_RESP1_LOCK_UNLOCK_FAILED) == SDMMC_RESP1_LOCK_UNLOCK_FAILED)
  {
    return (SD_LOCK_UNLOCK_FAILED);
  }

  if ((response_r1 & SDMMC_RESP1_COM_CRC_ERROR) == SDMMC_RESP1_COM_CRC_ERROR)
  {
    return (SD_COM_CRC_ERROR);
  }

  if ((response_r1 & SDMMC_RESP1_ILLEGAL_COMMAND) == SDMMC_RESP1_ILLEGAL_COMMAND)
  {
    return (SD_ILLEGAL_COMMAND);
  }

  if ((response_r1 & SDMMC_RESP1_CARD_ECC_FAILED) == SDMMC_RESP1_CARD_ECC_FAILED)
  {
    return (SD_CARD_ECC_FAILED);
  }

  if ((response_r1 & SDMMC_RESP1_CC_ERROR) == SDMMC_RESP1_CC_ERROR)
  {
    return (SD_CC_ERROR);
  }

  if ((response_r1 & SDMMC_RESP1_ERROR) == SDMMC_RESP1_ERROR)
  {
    return (SD_GENERAL_UNKNOWN_ERROR);
  }

  if ((response_r1 & SDMMC_RESP1_CSD_OVERWRITE) == SDMMC_RESP1_CSD_OVERWRITE)
  {
    return (SD_CSD_OVERWRITE);
  }

  if ((response_r1 & SDMMC_RESP1_WP_ERASE_SKIP) == SDMMC_RESP1_WP_ERASE_SKIP)
  {
    return (SD_WP_ERASE_SKIP);
  }

  if ((response_r1 & SDMMC_RESP1_CARD_ECC_DISABLED) == SDMMC_RESP1_CARD_ECC_DISABLED)
  {
    return (SD_CARD_ECC_DISABLED);
  }

  if ((response_r1 & SDMMC_RESP1_ERASE_RESET) == SDMMC_RESP1_ERASE_RESET)
  {
    return (SD_ERASE_RESET);
  }

  if ((response_r1 & SDMMC_RESP1_AKE_SEQ_ERROR) == SDMMC_RESP1_AKE_SEQ_ERROR)
  {
    return (SD_AKE_SEQ_ERROR);
  }

  return errorstate;
}

/**
  * @brief  Checks if the SD card is in programming state.
  * @param  hsd: SD handle
  * @param  pStatus: pointer to the variable that will contain the SD card state
  * @retval SD Card error state
  */
static EMU_SD_RTN SD_IsCardProgramming(SD_HandleTypeDef * hsd, uint8_t *status)
{
  SDMMC_CmdInitTypeDef sdmmc_cmdinitstructure;
  uint32_t responseR1 = 0;
  EMU_SD_RTN errorstate = SD_OK;
  sdmmc_cmdinitstructure.Argument         = (uint32_t)(hsd->RCA << 16);
  sdmmc_cmdinitstructure.CmdIndex         = SD_CMD_SEND_STATUS;
  sdmmc_cmdinitstructure.Response         = SDMMC_RESPONSE_R1;
  sdmmc_cmdinitstructure.Attribute        = SDMMC_CMD_START_CMD | 
                                            SDMMC_CMD_USE_HOLD_REG | 
                                            SDMMC_CMD_PRV_DAT_WAIT | 
                                            SDMMC_CMD_RESP_CRC     | 
                                            SDMMC_CMD_RESP_EXP;
  Core_SDMMC_SetRINTSTS(hsd->Instance, 0xFFFFFFFF);
  Core_SDMMC_SendCommand(hsd->Instance, &sdmmc_cmdinitstructure);

  /* We have received response, retrieve it for analysis */
  responseR1 = Core_SDMMC_GetResponse(hsd->Instance, SDMMC_RESP0);

  if ((responseR1 & SDMMC_RESP1_OUT_OF_RANGE) == SDMMC_RESP1_OUT_OF_RANGE)
  {
    return (SD_OUT_OF_RANGE);
  }

  if ((responseR1 & SDMMC_RESP1_ADDRESS_ERROR) == SDMMC_RESP1_ADDRESS_ERROR)
  {
    return (SD_ADDRESS_ERROR);
  }

  if ((responseR1 & SDMMC_RESP1_CARD_IS_LOCKED) == SDMMC_RESP1_CARD_IS_LOCKED)
  {
    return (SD_CARD_IS_LOCKED);
  }

  if ((responseR1 & SDMMC_RESP1_LOCK_UNLOCK_FAILED) == SDMMC_RESP1_LOCK_UNLOCK_FAILED)
  {
    return (SD_LOCK_UNLOCK_FAILED);
  }

  if ((responseR1 & SDMMC_RESP1_COM_CRC_ERROR) == SDMMC_RESP1_COM_CRC_ERROR)
  {
    return (SD_COM_CRC_ERROR);
  }

  if ((responseR1 & SDMMC_RESP1_ILLEGAL_COMMAND) == SDMMC_RESP1_ILLEGAL_COMMAND)
  {
    return (SD_ILLEGAL_COMMAND);
  }

  if ((responseR1 & SDMMC_RESP1_CARD_ECC_FAILED) == SDMMC_RESP1_CARD_ECC_FAILED)
  {
    return (SD_CARD_ECC_FAILED);
  }

  if ((responseR1 & SDMMC_RESP1_CC_ERROR) == SDMMC_RESP1_CC_ERROR)
  {
    return (SD_CC_ERROR);
  }

  if ((responseR1 & SDMMC_RESP1_ERROR) == SDMMC_RESP1_ERROR)
  {
    return (SD_GENERAL_UNKNOWN_ERROR);
  }

  if ((responseR1 & SDMMC_RESP1_CARD_ECC_DISABLED) == SDMMC_RESP1_CARD_ECC_DISABLED)
  {
    return (SD_CARD_ECC_DISABLED);
  }

  if ((responseR1 & SDMMC_RESP1_AKE_SEQ_ERROR) == SDMMC_RESP1_AKE_SEQ_ERROR)
  {
    return (SD_AKE_SEQ_ERROR);
  }

  /* Find out card status */
  // *status = responseR1 & SDMMC_RESP1_CURRENT_STATE;
  *status = responseR1 >> 9;

  return errorstate;
}

/**
  * @brief: identify if it is a sd card
  */
static EMU_SD_RTN SD_ENUM(SD_HandleTypeDef * hsd)
{
  SDMMC_CmdInitTypeDef sdmmc_cmdinitstructure;
  __IO EMU_SD_RTN errorstate = SD_OK;
  uint32_t cmd_done, RINTSTS_val, RINTSTS_to, RINTSTS_crc_err, RINTSTS_err, RINTSTS_fail, RINTSTS_rto;
  uint32_t RESP_val, cmd_illegal, sdio_mem, sd_mem;
  sdmmc_cmdinitstructure.Argument         = 0;
  sdmmc_cmdinitstructure.CmdIndex         = SD_CMD_IO_SEND_OP_COND;
  sdmmc_cmdinitstructure.Response         = SDMMC_RESPONSE_NO;
  sdmmc_cmdinitstructure.Attribute        = SDMMC_CMD_START_CMD | 
                                            SDMMC_CMD_USE_HOLD_REG | 
                                            SDMMC_CMD_SEND_INIT | 
                                            SDMMC_CMD_PRV_DAT_WAIT | 
                                            SDMMC_CMD_RESP_CRC | 
                                            SDMMC_CMD_RESP_EXP;
  Core_SDMMC_SetRINTSTS(hsd->Instance, 0xFFFFFFFF);
  Core_SDMMC_SendCommand(hsd->Instance, &sdmmc_cmdinitstructure);
  RINTSTS_val = Core_SDMMC_GetRINTSTS(hsd->Instance);
#ifdef ECHO
  DLOG_Info("CMD5 RINTSTS_val = %x\n", RINTSTS_val);
#endif
    uint32_t start = SysTicks_GetTickCount();
  do {
    RINTSTS_val = Core_SDMMC_GetRINTSTS(hsd->Instance);
    cmd_done = (RINTSTS_val & SDMMC_RINTSTS_CMD_DONE);
    if((SysTicks_GetDiff(start, SysTicks_GetTickCount())) > 100)
    {
        DLOG_Error("time out");
        return SD_ERROR;
    }
  } while (!cmd_done);

  RINTSTS_val = Core_SDMMC_GetRINTSTS(hsd->Instance);
#ifdef ECHO
  DLOG_Info("SD_ENUM: RINTSTS = %x\n", RINTSTS_val);
#endif
  RINTSTS_rto = (RINTSTS_val & SDMMC_RINTSTS_RTO);
  RINTSTS_crc_err = (RINTSTS_val & SDMMC_RINTSTS_RCRC);
  RINTSTS_err = (RINTSTS_val & SDMMC_RINTSTS_RESP_ERR);
  RINTSTS_fail = RINTSTS_rto & RINTSTS_crc_err & RINTSTS_err;
  if (RINTSTS_fail)
  {
    if (RINTSTS_rto)
    {
      DLOG_Info("RTO\n");
    }
    if (RINTSTS_crc_err)
    {
      DLOG_Info("RCRC\n");
    }
    if (RINTSTS_err)
    {
      DLOG_Info("RESP_ERR\n");
    }
  }
  RESP_val = Core_SDMMC_GetResponse(hsd->Instance, SDMMC_RESP0);
  cmd_illegal = (RESP_val & 0x00400000);
  sdio_mem = (RESP_val & 0x08000000);
  sd_mem = RINTSTS_rto || sdio_mem || cmd_illegal;
  if (!sd_mem) {
    return SD_NOTCARD;
  }
  return errorstate;
}

/**
  * @ configuration the register for dma
  */
static EMU_SD_RTN SD_DMAConfig(SD_HandleTypeDef * hsd, SDMMC_DMATransTypeDef * dma)
{
  EMU_SD_RTN errorstate = SD_OK;
  SDMMC_CmdInitTypeDef sdmmc_cmdinitstructure;

/*   Core_SDMMC_SetUHSREG(hsd->Instance, 0x0000FFFF); */
  
  // Core_SDMMC_SetRINTSTS(hsd->Instance, 0xFFFFFFFF);
  Core_SDMMC_SetBLKSIZ(hsd->Instance, dma->BlockSize);
  // Core_SDMMC_SetBYCTNT(hsd->Instance, dma->SectorNum * dma->BlockSize);
  /* Set Block Size for Card */
  sdmmc_cmdinitstructure.Argument         = dma->BlockSize;
  sdmmc_cmdinitstructure.CmdIndex         = SD_CMD_SET_BLOCKLEN;
  sdmmc_cmdinitstructure.Response         = SDMMC_RESPONSE_R1;
  sdmmc_cmdinitstructure.Attribute        = SDMMC_CMD_START_CMD | 
                                            SDMMC_CMD_USE_HOLD_REG | 
                                            SDMMC_CMD_PRV_DAT_WAIT | 
                                            SDMMC_CMD_RESP_CRC     | 
                                            SDMMC_CMD_RESP_EXP;

  
/*   DLOG_Info("RINTSTS = 0x%08x", read_reg32((uint32_t *)(0x42000000 + 0x44))); */
    Core_SDMMC_SetRINTSTS(hsd->Instance, SDMMC_RINTSTS_CMD_DONE);
  Core_SDMMC_SendCommand(hsd->Instance, &sdmmc_cmdinitstructure);
  /* Check for error conditions */
  Core_SDMMC_WaiteCmdDone(hsd->Instance);
// #ifdef ECHO
//   get_val = Core_SDMMC_GetRESP0(hsd->Instance);
//   DLOG_Info("CMD16 RESP0 = 0x%x\n", get_val);
// #endif

  /* set up idma descriptor */
  Core_SDMMC_SetBMOD(hsd->Instance, SDMMC_BMOD_ENABLE | SDMMC_BMOD_FB);
  Core_SDMMC_SetIDINTEN(hsd->Instance, 0x0);
/*   Core_SDMMC_SetCTYPE(hsd->Instance, SDMMC_CTYPE_4BIT); */
  Core_SDMMC_SetCTRL(hsd->Instance, SDMMC_CTRL_USE_INTERNAL_IDMAC |
                                    SDMMC_CTRL_INT_ENABLE | 
                                    SDMMC_CTRL_FIFO_RESET);
  return errorstate;
}

void SD_IRQHandler(uint32_t vectorNum)
{

    uint32_t status, pending, cdetect;

    static uint32_t flag_plug_out = 0;

    status  = read_reg32((uint32_t *)(SDMMC_BASE + 0x44));  /* RINTSTS */
    // pending = read_reg32((uint32_t *)(SDMMC_BASE + 0x40));  /* MINTSTS */
    cdetect = read_reg32((uint32_t *)(SDMMC_BASE + 0x50));  /* CDETECT*/

    write_reg32((uint32_t *)(SDMMC_BASE + 0x44), 0xFFFFFFFF);  /* RINTSTS */
    DLOG_Info("sd_irq\n");
    if ((read_reg32((uint32_t *)(SDMMC_BASE + 0x50)) & (1<<0)) == 0) // card in
    {
        flag_plug_out = 0;
        SYS_EVENT_Notify_From_ISR(SYS_EVENT_ID_SD_CARD_CHANGE, (void*)(&flag_plug_out));
        #if 0
        if (sdhandle.inited == 0)
        {
            sdhandle.SpeedMode = 2;
            sdhandle.Instance = SDMMC_ADDR;
            DLOG_Info("speedMode = %d", sdhandle.SpeedMode);
            // sd_delay_ms(100);
            sd_delay_ms(10);
            /*if (SD_OK == Card_SD_Init(&sdhandle, &cardinfo))
            {
                sdhandle.inited = 1;
                DLOG_Info("Inited SD Card...\n");                    
            }*/
            Card_SD_Init(&sdhandle, &cardinfo);
            sdhandle.inited = 1;
            DLOG_Info("Inited SD Card...\n");                    
            
         }

        DLOG_Info("event: SYS_EVENT_ID_SD_CARD_CHANGE state 0 notify succeed");
        #endif
    }
    else    // card out
    {
        
        flag_plug_out = 1;
        SYS_EVENT_Notify_From_ISR(SYS_EVENT_ID_SD_CARD_CHANGE, (void*)(&flag_plug_out));
        #if 0
        if (sdhandle.inited == 1)
        {
            DLOG_Info("Removing the SD Card...\n");
            EMU_SD_RTN errorstate = SD_OK;
            if (SD_OK == Card_SD_DeInit(&sdhandle))
            {
                sdhandle.inited = 0;
                DLOG_Info("Remove SD Success!\n");
            }
            else
            {
                DLOG_Warning("Remove SD fail!\n");
            }
        }

        DLOG_Info("event: SYS_EVENT_ID_SD_CARD_CHANGE state 1 notify succeed");
        #endif
    }

#if 0
  if(pending)
  {
    if (pending & SDMMC_RINTSTS_RESP_ERR)
    {
      if (status & SDMMC_RINTSTS_RESP_ERR)
      {
          DLOG_Info("SDMMC_RINTSTS_RESP_ERR\n");
          status &= ~SDMMC_RINTSTS_RESP_ERR;
          write_reg32((uint32_t *)(SDMMC_BASE + 0x44), status);
          return;
        
      }
    }
    if (pending & SDMMC_RINTSTS_CMD_DONE)
    {
      if (status & SDMMC_RINTSTS_CMD_DONE)
      {
          DLOG_Info("SDMMC_RINTSTS_CMD_DONE\n");
          status &= ~SDMMC_RINTSTS_CMD_DONE;
          write_reg32((uint32_t *)(SDMMC_BASE + 0x44), status);
          return;
        
      }
    }
    if (pending & SDMMC_RINTSTS_DATA_OVER)
    {
      if (status & SDMMC_RINTSTS_DATA_OVER)
      {
          DLOG_Info("SDMMC_RINTSTS_DATA_OVER\n");
          status &= ~SDMMC_RINTSTS_DATA_OVER;
          write_reg32((uint32_t *)(SDMMC_BASE + 0x44), status);
          return;
        
      }
    }
    if (pending & SDMMC_RINTSTS_TXDR)
    {
      if (status & SDMMC_RINTSTS_TXDR)
      {
          DLOG_Info("SDMMC_RINTSTS_TXDR\n");
          status &= ~SDMMC_RINTSTS_TXDR;
          write_reg32((uint32_t *)(SDMMC_BASE + 0x44), status);
          return;
        
      }
    }
    if (pending & SDMMC_RINTSTS_RXDR)
    {
      if (status & SDMMC_RINTSTS_RXDR)
      {
          DLOG_Info("SDMMC_RINTSTS_RXDR\n");
          status &= ~SDMMC_RINTSTS_RXDR;
          write_reg32((uint32_t *)(SDMMC_BASE + 0x44), status);
          return;
        
      }
    }
    if (pending & SDMMC_RINTSTS_RCRC)
    {
      if (status & SDMMC_RINTSTS_RCRC)
      {
          DLOG_Info("SDMMC_RINTSTS_RCRC\n");
          status &= ~SDMMC_RINTSTS_RCRC;
          write_reg32((uint32_t *)(SDMMC_BASE + 0x44), status);
          return;
        
      }
    }
    if (pending & SDMMC_RINTSTS_RTO)
    {
      if (status & SDMMC_RINTSTS_RTO)
      {
          DLOG_Info("SDMMC_RINTSTS_RTO\n");
          status &= ~SDMMC_RINTSTS_RTO;
          write_reg32((uint32_t *)(SDMMC_BASE + 0x44), status);
          return;
        
      }
    }
    if (pending & SDMMC_RINTSTS_DRTO)
    {
      if (status & SDMMC_RINTSTS_DRTO)
      {
          DLOG_Info("SDMMC_RINTSTS_DRTO\n");
          status &= ~SDMMC_RINTSTS_DRTO;
          write_reg32((uint32_t *)(SDMMC_BASE + 0x44), status);
          return;
        
      }
    }
    if (pending & SDMMC_RINTSTS_HTO)
    {
      if (status & SDMMC_RINTSTS_HTO)
      {
          DLOG_Info("SDMMC_RINTSTS_HTO\n");
          status &= ~SDMMC_RINTSTS_HTO;
          write_reg32((uint32_t *)(SDMMC_BASE + 0x44), status);
          return;
        
      }
    }

    if (pending & SDMMC_INTMASK_FRUN)
    {
      if (status & SDMMC_INTMASK_FRUN)
      {
          DLOG_Info("SDMMC_INTMASK_FRUN\n");
          status &= ~SDMMC_INTMASK_FRUN;
          write_reg32((uint32_t *)(SDMMC_BASE + 0x44), status);
          return;
        
      }
    }
    if (pending & SDMMC_INTMASK_HLE)
    {
      if (status & SDMMC_INTMASK_HLE)
      {
          DLOG_Info("SDMMC_INTMASK_HLE\n");
          status &= ~SDMMC_INTMASK_HLE;
          write_reg32((uint32_t *)(SDMMC_BASE + 0x44), status);
          return;
        
      }
    }
    if (pending & SDMMC_INTMASK_SBE)
    {
      if (status & SDMMC_INTMASK_SBE)
      {
          DLOG_Info("SDMMC_INTMASK_SBE\n");
          status &= ~SDMMC_INTMASK_SBE;
          write_reg32((uint32_t *)(SDMMC_BASE + 0x44), status);
          return;
        
      }
    }
    if (pending & SDMMC_INTMASK_ACD)
    {
      if (status & SDMMC_INTMASK_ACD)
      {
          DLOG_Info("SDMMC_INTMASK_ACD\n");
          status &= ~SDMMC_INTMASK_ACD;
          write_reg32((uint32_t *)(SDMMC_BASE + 0x44), status);
          return;
        
      }
    }
  }
#endif
}


SD_TRANSFER_STATUS SD_CardStatus(SD_STATUS *e_cardStatus)
{
  //DLOG_Info("Get Card Status\n");
  uint32_t RespState = 0;
  SD_STATUS e_cardStatusTmp = SD_CARD_IDLE;
  EMU_SD_RTN errorstate = SD_OK;
  errorstate = SD_GetState(&sdhandle, (uint32_t *)&RespState);
  if (errorstate != SD_OK)
  {
    DLOG_Error("Get SD Status Failed!\n");
    return SD_CARD_ERROR;
  }

  /* Find SD status according to card state*/
  e_cardStatusTmp = (SD_STATUS)((RespState >> 9) & 0x0F);
  *e_cardStatus = e_cardStatusTmp;
  if (e_cardStatusTmp == SD_CARD_TRANSFER)
  {
    return SD_TRANSFER_READY;
  }
  else if(e_cardStatusTmp == SD_CARD_ERROR)
  {
    return SD_TRANSFER_ERROR;
  }
  else
  {
    return SD_TRANSFER_BUSY;
  }
}

void SD_init_deInit_Callback(void *p)
{
    uint32_t status, pending, cdetect;
    static uint8_t flag_card_state = 0;

    DLOG_Info("into SD_init_deInit_Callback");

    uint8_t first_state;
    uint8_t sec_state;

    //sd_delay_ms(10);
    SysTicks_DelayMS(2);

    for (int i = 0;i < 10;i++)
    {
        first_state = getCardPresence;
        //sd_delay_ms(1);
        SysTicks_DelayMS(1);
        sec_state = getCardPresence;
        if (sec_state != first_state)
        {
            DLOG_Info("not stable trigger");            
            return;
        }
    }
    
    if (first_state == CARD_IN)
    {
        if (sdhandle.inited == 0)
        {
            sdhandle.SpeedMode = 2;
            sdhandle.Instance = SDMMC_ADDR;
            DLOG_Info("speedMode = %d", sdhandle.SpeedMode);
            //sd_delay_ms(100);
            SysTicks_DelayMS(50);
            if (SD_OK == Card_SD_Init(&sdhandle, &cardinfo))
            {
                sdhandle.inited = 1;
                DLOG_Info("Inited SD Card...\n");                    
            }
        }
    }
    else
    {
        if (sdhandle.inited == 1)
        {
            DLOG_Info("Removing the SD Card...\n");
            EMU_SD_RTN errorstate = SD_OK;
            if (SD_OK == Card_SD_DeInit(&sdhandle))
            {
                sdhandle.inited = 0;
                DLOG_Info("Remove SD Success!\n");
            }
            else
            {
                DLOG_Warning("Remove SD fail!\n");
            }
        }
    }
#if 0
    if (getCardPresence == CARD_IN && flag_card_state == 0)
    {
        DLOG_Info("Initializing the SD Card...\n");
        sdhandle.Instance = SDMMC_ADDR;
        Card_SD_Init(&sdhandle, &cardinfo);
        flag_card_state = 1;
    }
    else
    {
        if (flag_card_state == 1)
        {
            DLOG_Info("Removing the SD Card...\n");
            EMU_SD_RTN errorstate = SD_OK;
            if (SD_OK == Card_SD_DeInit(&sdhandle))
            {
                DLOG_Info("Remove SD Success!\n");
                flag_card_state = 0;
            }
            else
            {
                DLOG_Info("Remove SD fail!\n");
            }
        }
    }
#endif
}
/* SYS_EVENT_RegisterHandler(SYS_EVENT_ID_IDLE, H264_Encoder_IdleCallback); */

static SD_STATUS sd_getState(SD_HandleTypeDef *hsd)
{
    SDMMC_CmdInitTypeDef sdmmc_cmdinitstructure;
    uint32_t responseR1 = 0;
    EMU_SD_RTN errorstate = SD_OK;
    sdmmc_cmdinitstructure.Argument         = (uint32_t)(hsd->RCA << 16);
    sdmmc_cmdinitstructure.CmdIndex         = SD_CMD_SEND_STATUS;
    sdmmc_cmdinitstructure.Response         = SDMMC_RESPONSE_R1;
    sdmmc_cmdinitstructure.Attribute        = SDMMC_CMD_START_CMD | 
                                              SDMMC_CMD_USE_HOLD_REG | 
                                              SDMMC_CMD_PRV_DAT_WAIT | 
                                              SDMMC_CMD_RESP_CRC     | 
                                              SDMMC_CMD_RESP_EXP;
    Core_SDMMC_SetRINTSTS(hsd->Instance, 0xFFFFFFFF);
    Core_SDMMC_SendCommand(hsd->Instance, &sdmmc_cmdinitstructure);
    
    responseR1 = Core_SDMMC_GetResponse(hsd->Instance, SDMMC_RESP0);
/*     DLOG_Info("%d, responseR1 = 0x%08x", __LINE__, responseR1); */
    return responseR1>>9;
}

static EMU_SD_RTN wait_cmd_busy(void)
{
    uint32_t start = SysTicks_GetTickCount();

    do
    {
        if((SysTicks_GetDiff(start, SysTicks_GetTickCount())) > 100)
        {
            DLOG_Error("time out");
            return SD_FAIL;
        }
    } while (read_reg32((uint32_t *)(SDMMC_BASE + 0x44)) & (1<<13));

    return SD_OK;
}

static EMU_SD_RTN wait_cmd_done(void)
{   
    uint32_t start = SysTicks_GetTickCount();

    do
    {
        if((SysTicks_GetDiff(start, SysTicks_GetTickCount())) > 100)
        {
            DLOG_Error("time out");
            return SD_FAIL;
        }
    } while ((read_reg32((uint32_t *)(SDMMC_BASE + 0x44)) & (1<<2)) == 0);

    return SD_OK;
}

static EMU_SD_RTN convert_to_transfer(SD_HandleTypeDef *hsd)
{
    SDMMC_CmdInitTypeDef sdmmc_cmdinitstructure;
    EMU_SD_RTN errorstate = SD_OK;
    SD_STATUS cardstate;    
    uint32_t start;

    start = SysTicks_GetTickCount();

    do{
        cardstate = sd_getState(hsd);
        
        if((SysTicks_GetDiff(start, SysTicks_GetTickCount())) > 1000)
        {
            DLOG_Error("time out");
            errorstate = SD_FAIL;
            return errorstate;
        }
    }while(cardstate == SD_CARD_RECEIVE || cardstate == SD_CARD_PROGRAMMING);  

    return errorstate;
}

static EMU_SD_RTN SD_Tuning(SD_HandleTypeDef *hsd)
{
    uint32_t tuningPatten[16] =
    {
        0x00ff0fff,0xccc3ccff,0xffcc3cc3,0xeffefffe,0xddffdfff,0xfbfffbff,0xff7fffbf,0xefbdf777,
        0xf0fff0ff,0x3cccfc0f,0xcfcc33cc,0xeeffefff,0xfdfffdff,0xffbfffdf,0xfff7ffbb,0xde7b7ff7 
    };
    
    SDMMC_DMATransTypeDef *dma = malloc_safe(sizeof(SDMMC_DMATransTypeDef));
    SDMMC_CmdInitTypeDef sdmmc_cmdinitstructure;
    EMU_SD_RTN errorstate = SD_OK;
    IDMAC_DescTypeDef desc = {0};
    uint8_t *tuning = malloc_safe(64);
    memset(tuning, 0, 64);
    uint32_t tuningdst = peripheralAddrConvert((uint32_t)tuning);
    hsd->SdTransferCplt  = 0;
    hsd->SdTransferErr   = SD_OK;

    if (hsd->CardType == HIGH_CAPACITY_SD_CARD)
    {
        dma->BlockSize = 64;
    }

    errorstate = SD_DMAConfig(hsd, dma);
    if (errorstate != SD_OK) {
        return errorstate;
    }
    Core_SDMMC_SetDBADDR(hsd->Instance, peripheralAddrConvert((uint32_t)&desc));
    Core_SDMMC_SetBYCTNT(hsd->Instance, 0x40);

    desc.des0 = SDMMC_DES0_OWN | SDMMC_DES0_FS |  SDMMC_DES0_CH | SDMMC_DES0_LD;
    desc.des1 = 64;
    desc.des2 = tuningdst;
    desc.des3 = 0x0;

    uint8_t flag_dismattch = 0;
    uint8_t tuningTime = 0;

    do
    {
        DLOG_Info("tuning send time %d", tuningTime);

        memset(tuning, 0, 64);
        hsd->SdTransferCplt  = 0;
        hsd->SdTransferErr   = SD_OK;

        if (hsd->CardType == HIGH_CAPACITY_SD_CARD)
        {
            dma->BlockSize = 64;
        }

        errorstate = SD_DMAConfig(hsd, dma);
        if (errorstate != SD_OK) {
            return errorstate;
        }
        Core_SDMMC_SetDBADDR(hsd->Instance, peripheralAddrConvert((uint32_t)&desc));
        Core_SDMMC_SetBYCTNT(hsd->Instance, 0x40);

        desc.des0 = SDMMC_DES0_OWN | SDMMC_DES0_FS |  SDMMC_DES0_CH | SDMMC_DES0_LD;
        desc.des1 = 64;
        desc.des2 = tuningdst;
        desc.des3 = 0x0;
        
        sdmmc_cmdinitstructure.Argument         = 0x0;
        sdmmc_cmdinitstructure.CmdIndex         = SD_CMD_SEND_TUNING_PATTERN;
        sdmmc_cmdinitstructure.Response         = SDMMC_RESPONSE_R1;
        sdmmc_cmdinitstructure.Attribute        = SDMMC_CMD_START_CMD | 
                                                  SDMMC_CMD_USE_HOLD_REG | 
                                                  SDMMC_CMD_PRV_DAT_WAIT | 
                                                  SDMMC_CMD_DAT_EXP      | 
                                                  SDMMC_CMD_RESP_CRC     | 
                                                  SDMMC_CMD_RESP_EXP;
        Core_SDMMC_SetRINTSTS(hsd->Instance, 0xFFFE);
        Core_SDMMC_SendCommand(hsd->Instance, &sdmmc_cmdinitstructure);
        Core_SDMMC_WaiteCmdDone(hsd->Instance);  
        
        // sd_delay_ms(100);
        
        unsigned int readAddress = tuningdst;
        unsigned char row;
        readAddress -= (readAddress % 4);
        for (row = 0; row < 2; row++)
        {
            DLOG_Info("0x%08x: 0x%08x 0x%08x 0x%08x 0x%08x 0x%08x 0x%08x 0x%08x 0x%08x ", 
                      readAddress,
                      *(uint32_t *)readAddress,
                      *(uint32_t *)(readAddress + 4),
                      *(uint32_t *)(readAddress + 8),
                      *(uint32_t *)(readAddress + 12),
                      *(uint32_t *)(readAddress + 16),
                      *(uint32_t *)(readAddress + 20),
                      *(uint32_t *)(readAddress + 24),
                      *(uint32_t *)(readAddress + 28));
        
            readAddress += 32;
        }

        flag_dismattch = 0;
        readAddress = tuningdst;
        for (int i = 0; i < 16; i++)
        {
            if ((*(uint32_t *)(readAddress + i*4)) != tuningPatten[i])
            {
                flag_dismattch = 1;
                break;
            }
        }

        if (flag_dismattch == 0)
        {
            errorstate = SD_OK;
            break;
        }

        tuningTime++;
    }while(tuningTime < 40 );
    
    free_safe(tuning);
    free_safe(dma);
    hsd->SdTransferErr = errorstate;

    if (flag_dismattch == 0)
    {
        return SD_OK;
    }
    else
    {
        return SD_FAIL;
    }
}

EMU_SD_RTN Card_SD_ReadMultiBlocks_DMA_test(SD_HandleTypeDef *hsd, SDMMC_DMATransTypeDef *dma)
{
    SDMMC_CmdInitTypeDef sdmmc_cmdinitstructure;
    EMU_SD_RTN errorstate = SD_OK;
    uint32_t BlockIndex, TmpAddr = dma->DstAddr, DstAddr = dma->DstAddr;
    uint32_t BuffSize = BUFFSIZE8;
    uint32_t SectorDivid = dma->SectorNum / BuffSize;
    uint32_t SectorRmd = dma->SectorNum % BuffSize;
    uint32_t i = 0;
    uint32_t j;

    SD_STATUS current_state;
    Core_SDMMC_WaiteCardBusy(hsd->Instance);  
    current_state = sd_getState(hsd);
    if (current_state == SD_CARD_PROGRAMMING)
    {
        DLOG_Error("error");
        return SD_FAIL;
    }

    convert_to_transfer(hsd);

    uint32_t u32_start = SysTicks_GetTickCount();
    uint32_t u32_diff;
    uint32_t loop_times = 1000;
    uint32_t total_SectorDivid = SectorDivid*loop_times;
    IDMAC_DescTypeDef *desc = (IDMAC_DescTypeDef *)malloc_safe(sizeof(IDMAC_DescTypeDef) * (total_SectorDivid));
    if (!desc){
        DLOG_Error("Malloc Failed! Exit Read\n");
        errorstate = SD_ERROR;
        return errorstate;
    }

    memset(desc, 0, sizeof(IDMAC_DescTypeDef) * (total_SectorDivid));

    hsd->SdTransferCplt  = 0;
    hsd->SdTransferErr   = SD_OK;

    if (hsd->CardType == HIGH_CAPACITY_SD_CARD)
    {
        dma->BlockSize = 512;
    }

    errorstate = SD_DMAConfig(hsd, dma);
    Core_SDMMC_SetDBADDR(hsd->Instance, peripheralAddrConvert((uint32_t)&desc[0]));
    if (errorstate != SD_OK) {
        free_safe(desc);
        return errorstate;
    }

  if (SectorDivid)
  {
    Core_SDMMC_SetBYCTNT(hsd->Instance, total_SectorDivid * BuffSize * dma->BlockSize);
    DLOG_Info("Core_SDMMC_SetBYCTNT = %d Bytes", total_SectorDivid * BuffSize * dma->BlockSize);

    for (i = 0; i < loop_times; i++)
    {
        for (BlockIndex = 0; BlockIndex < SectorDivid; BlockIndex++)
        {

          DstAddr = dma->DstAddr + dma->BlockSize * BuffSize * BlockIndex;

          if (BlockIndex == 0 && (SectorDivid != 1))           // first
          {
            
            if (i == (loop_times - 1))
            {
                desc[i*SectorDivid + BlockIndex].des0 = SDMMC_DES0_OWN | SDMMC_DES0_CH;
                desc[i*SectorDivid + BlockIndex].des1 = dma->BlockSize * BuffSize;
                desc[i*SectorDivid + BlockIndex].des2 = DstAddr;
                desc[i*SectorDivid + BlockIndex].des3 = peripheralAddrConvert((uint32_t)&desc[i*SectorDivid+BlockIndex+1]);

            }
            else
            {
                // first
                desc[i*SectorDivid + BlockIndex].des0 = SDMMC_DES0_OWN | SDMMC_DES0_FS |  SDMMC_DES0_CH;
                desc[i*SectorDivid + BlockIndex].des1 = dma->BlockSize * BuffSize;
                desc[i*SectorDivid + BlockIndex].des2 = DstAddr;
                desc[i*SectorDivid + BlockIndex].des3 = peripheralAddrConvert((uint32_t)&desc[i*SectorDivid+BlockIndex+1]);

            }
          }
          else if ((BlockIndex == 0) && (SectorDivid == 1))  // first and last
          {

            desc[i*SectorDivid + BlockIndex].des0 = SDMMC_DES0_OWN | SDMMC_DES0_CH | SDMMC_DES0_LD | SDMMC_DES0_FS;
            desc[i*SectorDivid + BlockIndex].des1 = dma->BlockSize * BuffSize;
            desc[i*SectorDivid + BlockIndex].des2 = DstAddr;
            desc[i*SectorDivid + BlockIndex].des3 = 0x0;
            TmpAddr = DstAddr + dma->BlockSize * BuffSize;
          }
          else if (BlockIndex == SectorDivid - 1)           // last
          {
            desc[i*SectorDivid + BlockIndex].des1 = dma->BlockSize * BuffSize;
            desc[i*SectorDivid + BlockIndex].des2 = DstAddr;

            if (i == (loop_times - 1))
            {
                desc[i*SectorDivid + BlockIndex].des3 = 0x0;
                desc[i*SectorDivid + BlockIndex].des0 = SDMMC_DES0_OWN | SDMMC_DES0_CH | SDMMC_DES0_LD;
            }
            else
            {
                desc[i*SectorDivid + BlockIndex].des3 = peripheralAddrConvert((uint32_t)&desc[i*SectorDivid + BlockIndex+1]);
                desc[i*SectorDivid + BlockIndex].des0 = SDMMC_DES0_OWN | SDMMC_DES0_CH;
            }
            
            TmpAddr = DstAddr + dma->BlockSize * BuffSize;
          }
          else                                              // middle
          {
            desc[i*SectorDivid + BlockIndex].des0 = SDMMC_DES0_OWN | SDMMC_DES0_CH;
            desc[i*SectorDivid + BlockIndex].des1 = dma->BlockSize * BuffSize;
            desc[i*SectorDivid + BlockIndex].des2 = DstAddr;
            desc[i*SectorDivid + BlockIndex].des3 = peripheralAddrConvert((uint32_t)&desc[i*SectorDivid+BlockIndex+1]);
          }
        }
    }

    
/*
    for (i = 0; i < loop_times; i++)
    {
        for (j = 0; j < BuffSize; j++)
        {
            DLOG_Info("desc[%d][%d][0] = 0x%08x", i, j, desc[i*BuffSize + j].des0);        
            DLOG_Info("desc[%d][%d][1] = 0x%08x", i, j, desc[i*BuffSize + j].des1);
            DLOG_Info("desc[%d][%d][2] = 0x%08x", i, j, desc[i*BuffSize + j].des2);
            DLOG_Info("desc[%d][%d][3] = 0x%08x", i, j, desc[i*BuffSize + j].des3);
            sd_delay_ms(1);
        }
    }
*/

    u32_start = SysTicks_GetTickCount();

    /* send CMD18 */
    sdmmc_cmdinitstructure.Argument         = dma->SrcAddr;
    sdmmc_cmdinitstructure.CmdIndex         = SD_CMD_READ_MULTIPLE_BLOCK;
    sdmmc_cmdinitstructure.Response         = SDMMC_RESPONSE_R1;
    sdmmc_cmdinitstructure.Attribute        = SDMMC_CMD_START_CMD | 
                                              SDMMC_CMD_USE_HOLD_REG | 
                                              SDMMC_CMD_PRV_DAT_WAIT | 
                                              SDMMC_CMD_SEND_STOP    | 
                                              SDMMC_CMD_DAT_EXP      | 
                                              SDMMC_CMD_RESP_CRC     | 
                                              SDMMC_CMD_RESP_EXP;
    Core_SDMMC_SetRINTSTS(hsd->Instance, SDMMC_RINTSTS_CMD_DONE | SDMMC_RINTSTS_DATA_OVER);
    Core_SDMMC_SendCommand(hsd->Instance, &sdmmc_cmdinitstructure);
    Core_SDMMC_WaiteCmdDone(hsd->Instance);
    Core_SDMMC_WaiteDataOver(hsd->Instance);
    //Core_SDMMC_WaiteCardBusy(hsd->Instance);

    u32_diff = SysTicks_GetTickCount() - u32_start;

    DLOG_Info("loop_times = %d, used %d ms", loop_times, u32_diff);
    DLOG_Info("speed = %d kB/s", loop_times*4096*8/u32_diff);
  }
  
  free_safe(desc);

  sdmmc_cmdinitstructure.Argument         = (uint32_t)(hsd->RCA << 16);
  sdmmc_cmdinitstructure.CmdIndex         = SD_CMD_STOP_TRANSMISSION;
  sdmmc_cmdinitstructure.Response         = SDMMC_RESPONSE_R1;
  sdmmc_cmdinitstructure.Attribute        = SDMMC_CMD_START_CMD | 
                                            SDMMC_CMD_USE_HOLD_REG | 
                                            SDMMC_CMD_PRV_DAT_WAIT | 
                                            SDMMC_CMD_RESP_CRC | 
                                            SDMMC_CMD_RESP_EXP;
  Core_SDMMC_SetRINTSTS(hsd->Instance, SDMMC_RINTSTS_CMD_DONE);
  Core_SDMMC_SendCommand(hsd->Instance, &sdmmc_cmdinitstructure);
  Core_SDMMC_WaiteCmdDone(hsd->Instance);
  
  hsd->SdTransferErr = errorstate;
  return errorstate;
}

#if 0
EMU_SD_RTN Card_SD_ReadMultiBlocks_DMA_test(SD_HandleTypeDef *hsd, SDMMC_DMATransTypeDef *dma)
{
    SDMMC_CmdInitTypeDef sdmmc_cmdinitstructure;
    EMU_SD_RTN errorstate = SD_OK;
    uint32_t BlockIndex, TmpAddr = dma->DstAddr, DstAddr = dma->DstAddr;
    uint32_t BuffSize = BUFFSIZE32;
    uint32_t SectorDivid = dma->SectorNum / BuffSize;
    uint32_t SectorRmd = dma->SectorNum % BuffSize;
    uint32_t i = 0;
    uint32_t j;

    SD_STATUS current_state;
    Core_SDMMC_WaiteCardBusy(hsd->Instance);  
    current_state = sd_getState(hsd);
/*   DLOG_Info("%d state = %d", __LINE__, current_state); */
    if (current_state == SD_CARD_PROGRAMMING)
    {
        DLOG_Error("error");
        return SD_FAIL;
    }

    convert_to_transfer(hsd);

    /* malloc the space for descriptor */
    uint32_t u32_start = SysTicks_GetTickCount();
    uint32_t u32_diff;
    uint32_t loop_times = 20;
    uint32_t total_SectorDivid = SectorDivid*loop_times;
    // IDMAC_DescTypeDef *desc = (IDMAC_DescTypeDef *)malloc(sizeof(IDMAC_DescTypeDef) * (SectorDivid + SectorRmd));
    IDMAC_DescTypeDef *desc = (IDMAC_DescTypeDef *)malloc_safe(sizeof(IDMAC_DescTypeDef) * (total_SectorDivid));
    if (!desc){
        DLOG_Info("Malloc Failed! Exit Read\n");
        errorstate = SD_ERROR;
        return errorstate;
    }

    memset(desc, 0, sizeof(IDMAC_DescTypeDef) * (total_SectorDivid));

    /* Initialize handle flags */
    hsd->SdTransferCplt  = 0;
    hsd->SdTransferErr   = SD_OK;

    if (hsd->CardType == HIGH_CAPACITY_SD_CARD)
    {
        dma->BlockSize = 512;
    }

    /* Configure the SD DPSM (Data Path State Machine) */
    errorstate = SD_DMAConfig(hsd, dma);
    Core_SDMMC_SetDBADDR(hsd->Instance, peripheralAddrConvert((uint32_t)&desc[0]));
    if (errorstate != SD_OK) {
        free(desc);
        return errorstate;
    }
    
    DstAddr = dma->DstAddr;
    for (BlockIndex = 0; BlockIndex < total_SectorDivid; BlockIndex++)
    {
        if (BlockIndex != 0)
        {
            DstAddr += dma->BlockSize * BuffSize;
        }

        if (DstAddr > 0x2100A000)
        {
            DstAddr = dma->DstAddr;
        }

        if (BlockIndex == 0)    // first
        {
            desc[BlockIndex].des0 = SDMMC_DES0_OWN | SDMMC_DES0_FS |  SDMMC_DES0_CH;
            desc[BlockIndex].des1 = dma->BlockSize * BuffSize;
            desc[BlockIndex].des2 = DstAddr;
            desc[BlockIndex].des3 = peripheralAddrConvert((uint32_t)&desc[BlockIndex+1]);
        }
        else if (BlockIndex == total_SectorDivid - 1) // last
        {
            desc[BlockIndex].des0 = SDMMC_DES0_OWN | SDMMC_DES0_CH | SDMMC_DES0_LD;
            desc[BlockIndex].des1 = dma->BlockSize * BuffSize;
            desc[BlockIndex].des2 = DstAddr;
            desc[BlockIndex].des3 = 0x0;
        }
        else // middle
        {
            desc[BlockIndex].des0 = SDMMC_DES0_OWN | SDMMC_DES0_CH;
            desc[BlockIndex].des1 = dma->BlockSize * BuffSize;
            desc[BlockIndex].des2 = DstAddr;
            desc[BlockIndex].des3 = peripheralAddrConvert((uint32_t)&desc[BlockIndex+1]);
        }
    }

    for (i = 0; i < total_SectorDivid; i++)
    {
        DLOG_Info("desc[%d][0] = 0x%08x", i, desc[i].des0);        
        DLOG_Info("desc[%d][1] = 0x%08x", i, desc[i].des1);
        DLOG_Info("desc[%d][2] = 0x%08x", i, desc[i].des2);
        DLOG_Info("desc[%d][3] = 0x%08x", i, desc[i].des3);
        sd_delay_ms(100);
    }

/*     Core_SDMMC_SetFIFOTH(hsd->Instance, 0x00070008 | (1<<28) | (8) | (3<<16)); */
    Core_SDMMC_SetFIFOTH(hsd->Instance, (1<<28) | (8) | (7<<16));
    Core_SDMMC_SetBYCTNT(hsd->Instance, total_SectorDivid * BuffSize*dma->BlockSize);
    DLOG_Info("Core_SDMMC_SetBYCTNT = %d Bytes", total_SectorDivid * BuffSize*dma->BlockSize);

    u32_start = SysTicks_GetTickCount();

    /* send CMD18 */
    sdmmc_cmdinitstructure.Argument         = dma->SrcAddr;
    sdmmc_cmdinitstructure.CmdIndex         = SD_CMD_READ_MULTIPLE_BLOCK;
    sdmmc_cmdinitstructure.Response         = SDMMC_RESPONSE_R1;
    sdmmc_cmdinitstructure.Attribute        = SDMMC_CMD_START_CMD | 
                                              SDMMC_CMD_USE_HOLD_REG | 
                                              SDMMC_CMD_PRV_DAT_WAIT | 
                                              SDMMC_CMD_SEND_STOP    | 
                                              SDMMC_CMD_DAT_EXP      | 
                                              SDMMC_CMD_RESP_CRC     | 
                                              SDMMC_CMD_RESP_EXP;
    Core_SDMMC_SetRINTSTS(hsd->Instance, SDMMC_RINTSTS_CMD_DONE | SDMMC_RINTSTS_DATA_OVER);
    Core_SDMMC_SendCommand(hsd->Instance, &sdmmc_cmdinitstructure);
    /* Check for error conditions */
    Core_SDMMC_WaiteCmdDone(hsd->Instance);
    Core_SDMMC_WaiteDataOver(hsd->Instance);
    //Core_SDMMC_WaiteCardBusy(hsd->Instance);

    u32_diff = SysTicks_GetTickCount() - u32_start;

    DLOG_Info("loop_times = %d, used %d ms", loop_times, u32_diff);
    DLOG_Info("speed = %d kB/s", loop_times*4096*8/u32_diff);
  
    free(desc);    
    Core_SDMMC_SetFIFOTH(hsd->Instance, 0x00070008);

    sdmmc_cmdinitstructure.Argument         = (uint32_t)(hsd->RCA << 16);
    sdmmc_cmdinitstructure.CmdIndex         = SD_CMD_STOP_TRANSMISSION;
    sdmmc_cmdinitstructure.Response         = SDMMC_RESPONSE_R1;
    sdmmc_cmdinitstructure.Attribute        = SDMMC_CMD_START_CMD | 
                                              SDMMC_CMD_USE_HOLD_REG | 
                                              SDMMC_CMD_PRV_DAT_WAIT | 
                                              SDMMC_CMD_RESP_CRC | 
                                              SDMMC_CMD_RESP_EXP;
    Core_SDMMC_SetRINTSTS(hsd->Instance, SDMMC_RINTSTS_CMD_DONE);
    Core_SDMMC_SendCommand(hsd->Instance, &sdmmc_cmdinitstructure);
    Core_SDMMC_WaiteCmdDone(hsd->Instance);

    hsd->SdTransferErr = errorstate;
    return errorstate;
}
#endif


EMU_SD_RTN Card_SD_WriteMultiBlocks_DMA_test(SD_HandleTypeDef *hsd, SDMMC_DMATransTypeDef *dma)
{
    SDMMC_CmdInitTypeDef sdmmc_cmdinitstructure;
    EMU_SD_RTN errorstate = SD_OK;
    uint32_t BlockIndex, TmpAddr = dma->SrcAddr, SrcAddr = dma->SrcAddr;
    uint32_t BuffSize = BUFFSIZE8;
    uint32_t SectorDivid = dma->SectorNum / BuffSize;
    uint32_t SectorRmd = dma->SectorNum % BuffSize;    
    uint32_t i = 0;
    uint32_t j;

    Core_SDMMC_WaiteCardBusy(hsd->Instance);  
    convert_to_transfer(hsd);

    uint32_t u32_start = SysTicks_GetTickCount();
    uint32_t u32_diff;
    uint32_t loop_times = 1000;
    uint32_t total_SectorDivid = SectorDivid*loop_times;
    IDMAC_DescTypeDef *desc = (IDMAC_DescTypeDef *)malloc_safe(sizeof(IDMAC_DescTypeDef) * (total_SectorDivid));

    if (!desc){
        DLOG_Error("malloc failed! Exit writing\n");
        errorstate = SD_ERROR;
        return errorstate;
    }

     memset(desc, 0, sizeof(IDMAC_DescTypeDef) * (total_SectorDivid));

    hsd->SdTransferCplt  = 0;
    hsd->SdTransferErr   = SD_OK;
    hsd->SdOperation = SD_WRITE_MULTIPLE_BLOCK;

#if 0
  if (hsd->CardType == HIGH_CAPACITY_SD_CARD)
  {
    dma->BlockSize = 512;
  }
#endif
  dma->BlockSize = 512;
  errorstate = SD_DMAConfig(hsd, dma);
  Core_SDMMC_SetDBADDR(hsd->Instance, peripheralAddrConvert((uint32_t)&desc[0]));
  if (errorstate != SD_OK) {
    DLOG_Error("SD_DMAConfig Fail\n");
    free_safe(desc);
    return errorstate;
  }

    if (SectorDivid)
    { 
        Core_SDMMC_SetBYCTNT(hsd->Instance, total_SectorDivid * BuffSize * dma->BlockSize);
        for (i = 0; i < loop_times; i++)
        {
            for (BlockIndex = 0; BlockIndex < SectorDivid; BlockIndex++)
            {
                SrcAddr = dma->SrcAddr + dma->BlockSize * BuffSize * BlockIndex;
                if ((BlockIndex == 0) && (SectorDivid != 1))   // first
                {
                    if (i == (loop_times - 1))
                    {
                        desc[i*SectorDivid+BlockIndex].des0 = SDMMC_DES0_OWN | SDMMC_DES0_CH;
                        desc[i*SectorDivid+BlockIndex].des1 = dma->BlockSize * BuffSize;
                        desc[i*SectorDivid+BlockIndex].des2 = SrcAddr;
                        desc[i*SectorDivid+BlockIndex].des3 = peripheralAddrConvert((uint32_t)&desc[i*SectorDivid+BlockIndex+1]);
                    }
                    else
                    {
                        desc[i*SectorDivid+BlockIndex].des0 = SDMMC_DES0_OWN | SDMMC_DES0_FS | SDMMC_DES0_CH;
                        desc[i*SectorDivid+BlockIndex].des1 = dma->BlockSize * BuffSize;
                        desc[i*SectorDivid+BlockIndex].des2 = SrcAddr;
                        desc[i*SectorDivid+BlockIndex].des3 = peripheralAddrConvert((uint32_t)&desc[i*SectorDivid+BlockIndex+1]);
                    }
                }
                else if ((BlockIndex == 0) && (SectorDivid == 1))        // first and last
                {
                    desc[i*SectorDivid+BlockIndex].des0 = SDMMC_DES0_OWN | SDMMC_DES0_FS |  SDMMC_DES0_CH | SDMMC_DES0_LD;
                    desc[i*SectorDivid+BlockIndex].des1 = dma->BlockSize;
                    desc[i*SectorDivid+BlockIndex].des2 = SrcAddr;
                    desc[i*SectorDivid+BlockIndex].des3 = 0;
                }
                else if (BlockIndex == SectorDivid - 1)             // last
                {
                    desc[i*SectorDivid+BlockIndex].des1 = dma->BlockSize * BuffSize;
                    desc[i*SectorDivid+BlockIndex].des2 = SrcAddr;
                    if (i == (loop_times - 1))
                    {
                        desc[i*SectorDivid+BlockIndex].des3 = 0x0;
                        desc[i*SectorDivid+BlockIndex].des0 = SDMMC_DES0_OWN | SDMMC_DES0_LD | SDMMC_DES0_CH;
                    }
                    else
                    {
                        desc[i*SectorDivid+BlockIndex].des3 = peripheralAddrConvert((uint32_t)&desc[i*SectorDivid+BlockIndex+1]);
                        desc[i*SectorDivid+BlockIndex].des0 = SDMMC_DES0_OWN | SDMMC_DES0_CH;
                    }
                }
                else                                                // middle
                {
                    desc[i*SectorDivid+BlockIndex].des0 = SDMMC_DES0_OWN | SDMMC_DES0_CH;
                    desc[i*SectorDivid+BlockIndex].des1 = dma->BlockSize * BuffSize;
                    desc[i*SectorDivid+BlockIndex].des2 = SrcAddr;
                    desc[i*SectorDivid+BlockIndex].des3 = peripheralAddrConvert((uint32_t)&desc[i*SectorDivid+BlockIndex+1]);
                }
            }
        }

/*
        for (i = 0; i < loop_times; i++)
        {
            for (j = 0; j < BuffSize; j++)
            {
                DLOG_Info("desc[%d][%d][0] = 0x%08x", i, j, desc[i*BuffSize + j].des0);        
                DLOG_Info("desc[%d][%d][1] = 0x%08x", i, j, desc[i*BuffSize + j].des1);
                DLOG_Info("desc[%d][%d][2] = 0x%08x", i, j, desc[i*BuffSize + j].des2);
                DLOG_Info("desc[%d][%d][3] = 0x%08x", i, j, desc[i*BuffSize + j].des3);
                sd_delay_ms(20);
            }
        }
*/

        u32_start = SysTicks_GetTickCount();

        sdmmc_cmdinitstructure.Argument         = dma->DstAddr;
        sdmmc_cmdinitstructure.CmdIndex         = SD_CMD_WRITE_MULTIPLE_BLOCK;
        sdmmc_cmdinitstructure.Response         = SDMMC_RESPONSE_R1;
        sdmmc_cmdinitstructure.Attribute        = SDMMC_CMD_START_CMD | 
                                                  SDMMC_CMD_USE_HOLD_REG | 
                                                  SDMMC_CMD_PRV_DAT_WAIT | 
                                                  SDMMC_CMD_SEND_STOP    | 
                                                  SDMMC_CMD_DAT_WRITE | 
                                                  SDMMC_CMD_DAT_EXP      | 
                                                  SDMMC_CMD_RESP_CRC     | 
                                                  SDMMC_CMD_RESP_EXP;

        Core_SDMMC_SetRINTSTS(hsd->Instance, SDMMC_RINTSTS_CMD_DONE | SDMMC_RINTSTS_DATA_OVER);
        Core_SDMMC_SendCommand(hsd->Instance, &sdmmc_cmdinitstructure);
        Core_SDMMC_WaiteCmdDone(hsd->Instance);
        Core_SDMMC_WaiteDataOver(hsd->Instance);
        //Core_SDMMC_WaiteCardBusy(hsd->Instance);

        sdmmc_cmdinitstructure.Argument         = (uint32_t)(hsd->RCA << 16);
        sdmmc_cmdinitstructure.CmdIndex         = SD_CMD_STOP_TRANSMISSION;
        sdmmc_cmdinitstructure.Response         = SDMMC_RESPONSE_R1;
        sdmmc_cmdinitstructure.Attribute        = SDMMC_CMD_START_CMD | 
                                                  SDMMC_CMD_USE_HOLD_REG | 
                                                  SDMMC_CMD_PRV_DAT_WAIT | 
                                                  SDMMC_CMD_RESP_CRC | 
                                                  SDMMC_CMD_RESP_EXP;
        Core_SDMMC_SetRINTSTS(hsd->Instance, SDMMC_RINTSTS_CMD_DONE);
        Core_SDMMC_SendCommand(hsd->Instance, &sdmmc_cmdinitstructure);
        Core_SDMMC_WaiteCmdDone(hsd->Instance);

        u32_diff = SysTicks_GetTickCount() - u32_start;
        
        DLOG_Info("loop_times = %d, used %d ms", loop_times, u32_diff);
        DLOG_Info("speed = %d kB/s", loop_times*4096*8/u32_diff);
    }

    free_safe(desc);
    hsd->SdTransferErr = errorstate;
    return errorstate;
}

static EMU_SD_RTN SD_DMAConfig_test(SD_HandleTypeDef * hsd, SDMMC_DMATransTypeDef * dma)
{
    EMU_SD_RTN errorstate = SD_OK;
    SDMMC_CmdInitTypeDef sdmmc_cmdinitstructure;

    Core_SDMMC_SetBLKSIZ(hsd->Instance, dma->BlockSize);
    sdmmc_cmdinitstructure.Argument         = dma->BlockSize;
    sdmmc_cmdinitstructure.CmdIndex         = SD_CMD_SET_BLOCKLEN;
    sdmmc_cmdinitstructure.Response         = SDMMC_RESPONSE_R1;
    sdmmc_cmdinitstructure.Attribute        = SDMMC_CMD_START_CMD | 
                                              SDMMC_CMD_USE_HOLD_REG | 
                                              SDMMC_CMD_PRV_DAT_WAIT | 
                                              SDMMC_CMD_RESP_CRC     | 
                                              SDMMC_CMD_RESP_EXP;  
    Core_SDMMC_SetRINTSTS(hsd->Instance, SDMMC_RINTSTS_CMD_DONE);
    Core_SDMMC_SendCommand(hsd->Instance, &sdmmc_cmdinitstructure);
    Core_SDMMC_WaiteCmdDone(hsd->Instance);

    Core_SDMMC_SetBMOD(hsd->Instance, SDMMC_BMOD_ENABLE | SDMMC_BMOD_FB | (7<<8));
    Core_SDMMC_SetIDINTEN(hsd->Instance, 0x0);
    Core_SDMMC_SetCTRL(hsd->Instance, SDMMC_CTRL_USE_INTERNAL_IDMAC |
                                      SDMMC_CTRL_INT_ENABLE | 
                                      SDMMC_CTRL_FIFO_RESET);
    return errorstate;
}

