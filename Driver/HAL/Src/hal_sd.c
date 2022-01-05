/*****************************************************************************
Copyright: 2016-2020, Artosyn. Co., Ltd.
File name: hal_sd.h
Description: The external HAL APIs to use the SDMMC controller.
Author: Artosyn Software Team
Version: 0.0.1
Date: 2016/12/26
History: 
        0.0.1    2016/12/26    The initial version of hal_sd.h
*****************************************************************************/
#include <stdint.h>
#include "memory_config.h"
#include "hal_sd.h"
#include "hal_nvic.h"
#include "sd_core.h"
#include "sd_host.h"
#include "debuglog.h"
#include "reg_rw.h"
#include "interrupt.h"
#include "sys_event.h"
#include "systicks.h"
#include "sd_diskio.h"
#include "ff_gen_drv.h"
#include "hal.h"
#include "cpu_info.h"

extern SDMMC_DMATransTypeDef dma;
extern SD_HandleTypeDef sdhandle;
extern SD_CardInfoTypedef cardinfo;

static uint32_t addrConvert(uint32_t addr);

static void speed_disp(SpeedModeTypedef speed)
{
    switch (speed)
    {
        case SDR12:
            DLOG_Info("speedMode = SDR12");
        break;

        case SDR25:
            DLOG_Info("speedMode = SDR25");
        break;

        case SDR50:
            DLOG_Info("speedMode = SDR50");
        break;

        case SDR104:
            DLOG_Info("speedMode = SDR104");
        break;
    }
}

/**
* @brief  Initializes the SD card according to the specified parameters in the 
            SD_HandleTypeDef and create the associated handle
* @param  none
* @retval HAL_OK            means the initializtion is well done
*         HAL_SD_ERR_ERROR  means some error happens in the initializtion
*/
HAL_RET_T HAL_SD_Init(void)
{
    EMU_SD_RTN e_errorState = SD_OK;
    if ( FALSE == SYS_EVENT_RegisterHandler(SYS_EVENT_ID_SD_CARD_CHANGE, SD_init_deInit_Callback))
    {
        DLOG_Error("SYS_EVENT_Register fail");
        return HAL_SD_ERR_ERROR;            
    }
    else
    {
        DLOG_Info("SYS_EVENT_Register success");
    }


    if (getCardPresence == CARD_IN)
    {
        sdhandle.Instance = SDMMC_ADDR;
        sdhandle.SpeedMode = SDR50;
        speed_disp(sdhandle.SpeedMode);
        SysTicks_DelayMS(100);
        e_errorState = Card_SD_Init(&sdhandle, &cardinfo);
        if (e_errorState != SD_OK)
        {
            sdhandle.inited = 0;
            return HAL_SD_ERR_ERROR;
        }
        else
        {
            sdhandle.inited = 1;
        }
        uint32_t status = read_reg32((uint32_t *)(SDMMC_BASE + 0x24));
        write_reg32((uint32_t *)(SDMMC_BASE + 0x24), status | 0x1);
        status = read_reg32((uint32_t *)(SDMMC_BASE));
        write_reg32((uint32_t *)(SDMMC_BASE), 0x00000010 | status);
    }
    else
    {
        sdhandle.inited = 0;
        // write_reg32((uint32_t *)(SDMMC_BASE + 0x50), 0x00000001);
        write_reg32((uint32_t *)(SDMMC_BASE + 0x24), 0x00000001);
        write_reg32((uint32_t *)(SDMMC_BASE), 0x00000010);
    }
    NVIC_ClearPendingIRQ(SD_INTR_VECTOR_NUM);
    write_reg32((uint32_t *)(SDMMC_BASE + 0x44), 0xffff);
    reg_IrqHandle(SD_INTR_VECTOR_NUM, SD_IRQHandler, NULL);
    NVIC_SetPriority(SD_INTR_VECTOR_NUM,NVIC_EncodePriority(NVIC_PRIORITYGROUP_5,INTR_NVIC_PRIORITY_SD,0));
    NVIC_EnableIRQ(SD_INTR_VECTOR_NUM);
    
	DLOG_Info("Initialize SD Success!");
	
    SysTicks_DelayMS(500);  // wait for stable

	return HAL_OK;
}

/**
* @brief  Writes block(s) to a specified address in a card. The Data transfer 
  *         is managed by DMA mode
* @param  u32_dstStartAddr   pointer to the buffer that will contain the data to transmit
*         u32_srcStartAddr   Address from where data is to be read   
*         u32_sectorNum      the SD card Data block size 
* @retval HAL_OK            write to sd card succeed
*         HAL_SD_ERR_ERROR  means some error happens during the writing
*/
HAL_RET_T HAL_SD_Write(uint32_t u32_dstBlkAddr, uint32_t u32_srcStartAddr, uint32_t u32_sectorNum)
{
	EMU_SD_RTN e_errorState = SD_OK;
	SDMMC_DMATransTypeDef st_dma;
	st_dma.BlockSize = 512;
    st_dma.SrcAddr = peripheralAddrConvert(u32_srcStartAddr);
	st_dma.DstAddr = (uint32_t )u32_dstBlkAddr;                        /* [block units] */
	st_dma.SectorNum = u32_sectorNum;

    if (getCardPresence != CARD_IN)
    {
        DLOG_Error("card not pressent");
        return SD_FAIL;
    }


	if (u32_sectorNum == 1)
	{
		e_errorState = Card_SD_WriteBlock_DMA(&sdhandle, &st_dma);	
	}
	else
	{
		e_errorState = Card_SD_WriteMultiBlocks_DMA(&sdhandle, &st_dma);
	}
	
	if (e_errorState != SD_OK) {
		DLOG_Error("Write SD Failed!\n");
		return HAL_SD_ERR_ERROR;
	}
	// DLOG_Info("Write SD %d Sectors Done. From Sector %d to Sector %d\n", 
	//	         st_dma.SectorNum, st_dma.DstAddr, (st_dma.DstAddr + st_dma.SectorNum - 1));
	return HAL_OK;
}

/**
* @brief  Reads block(s) from a specified address in a card. The Data transfer 
  *         is managed by DMA mode. 
* @param  u32_dstStartAddr   Pointer to the buffer that will contain the received data
*         u32_srcStartAddr   Address from where data is to be read  
*         u32_sectorNum      SD card Data block size
* @retval HAL_OK             read from sd card succeed
*         HAL_SD_ERR_ERROR   means some error happens during the reading
*/
HAL_RET_T HAL_SD_Read(uint32_t u32_dstStartAddr, uint32_t u32_srcBlkAddr, uint32_t u32_sectorNum)
{
	EMU_SD_RTN e_errorState = SD_OK;
	SDMMC_DMATransTypeDef st_dma;
	st_dma.BlockSize = 512;
	st_dma.SrcAddr = u32_srcBlkAddr;                     /* [block units] */
    st_dma.DstAddr = peripheralAddrConvert(u32_dstStartAddr);
	st_dma.SectorNum = u32_sectorNum;
	
    if (getCardPresence != CARD_IN)
    {
        DLOG_Error("card not pressent");
        return SD_FAIL;
    }


	if (u32_sectorNum == 1)
	{
		e_errorState = Card_SD_ReadBlock_DMA(&sdhandle, &st_dma);
	}
	else
	{
		e_errorState = Card_SD_ReadMultiBlocks_DMA(&sdhandle, &st_dma);
	}
	
	if (e_errorState != SD_OK) {
		DLOG_Error("Read SD Failed!");
		return HAL_SD_ERR_ERROR;
	}
	// DLOG_Info("Read SD %d Sectors Done. From Sector %d to Sector %d\n", 
	//	         st_dma.SectorNum, st_dma.SrcAddr, (st_dma.SrcAddr + st_dma.SectorNum - 1));
	return HAL_OK;
}

/**
* @brief  Erases the specified memory area of the given SD card
* @param  u32_startBlock    start byte address
*         u32_sectorNum     the SD card data block size
* @retval HAL_OK            erase SD card succeed
*         HAL_SD_ERR_ERROR  means some error happens during the erasing
*/
HAL_RET_T HAL_SD_Erase(uint32_t u32_startBlkAddr, uint32_t u32_sectorNum)
{
    if (getCardPresence != CARD_IN)
    {
        DLOG_Error("card not pressent");
        return SD_FAIL;
    }

	EMU_SD_RTN e_errorState = SD_OK;
	e_errorState = Card_SD_Erase(&sdhandle, u32_startBlkAddr, u32_sectorNum);  /* [block units] */
	if (e_errorState != SD_OK) {
		DLOG_Error("Erase SD failed!\n");
		return HAL_SD_ERR_ERROR;
	}
	// DLOG_Info("Erase %d Sectors Done. From Sector %d to Sector %d\n", 
	//	         u32_sectorNum, u32_startAddr, (u32_startAddr + u32_sectorNum - 1));
	return HAL_OK;
}

/**
* @brief  De-Initializes the SD card
* @param  none
* @retval HAL_OK      means the de-initialization is well done
      HAL_SD_ERR_ERROR means de-initialization
*/
HAL_RET_T HAL_SD_Deinit()
{
	
	EMU_SD_RTN e_errorState = SD_OK;
	e_errorState = Card_SD_DeInit(&sdhandle);
	if (e_errorState != SD_OK) {
		DLOG_Error("Deinit SD Failed!\n");
		return HAL_SD_ERR_ERROR;
	}
	DLOG_Info("Remove SD Success!\n");
	return HAL_OK;
}

/*
 * @brief Get the time value according to SD spec
 * @param raw value in CSD
 * @return float value will mutiple with speed
*/
static float SD_SpeedTimesConvert(unsigned char times)
{
	float result_convert = 0;
	switch (times)
	{
		case 0:
			result_convert = 0;
			DLOG_Info("SD Speed Times reserved\n");
			break;
		case 1:
			result_convert = 1.0;
			break;
		case 2:
			result_convert = 1.2;
			break;
		case 3:
			result_convert = 1.3;
			break;
		case 4:
			result_convert = 1.5;
			break;
		case 5:
			result_convert = 2.0;
			break;	
		case 6:
			result_convert = 2.5;
			break;
		case 7:
			result_convert = 3.0;
			break;
		case 8:
			result_convert = 3.5;
			break;
		case 9:
			result_convert = 4.0;
			break;
		case 10:
			result_convert = 4.5;
			break;
		case 11:
			result_convert = 5.0;
			break;
		case 12:
			result_convert = 5.5;
			break;
		case 13:
			result_convert = 6.0;
			break;
		case 14:
			result_convert = 7.0;
			break;
		case 15:
			result_convert = 8.0;
			break;
		default: break;
	}

	return result_convert;
}

/**
* @brief  IO ctrl function to acquire SD card related information
* @param  e_sdCtrl          the SD IO ctrl enumetation variable to specify the corresponding function
*         pu32_info         IO ctrl function output 
* @retval HAL_OK            get the information succeed
*         HAL_SD_ERR_ERROR  means some error happens in the function
*/
HAL_RET_T HAL_SD_Ioctl(ENUM_HAL_SD_CTRL e_sdCtrl, uint32_t *pu32_info)
{
    EMU_SD_RTN e_errorState = SD_OK;
    uint32_t u32_tmpValue = 0;
    float tran_speed = 0;
/*     long long int CardCapacity = 0; */
	e_errorState = Card_SD_Get_CardInfo(&sdhandle, &cardinfo);
    
	if (e_errorState != SD_OK) 
	{
		DLOG_Info("Get SD Info failed!\n");
		return HAL_SD_ERR_ERROR;
	}
	switch(e_sdCtrl)
	{
		case HAL_SD_GET_SECTOR_COUNT:
                    DLOG_Info("cardinfo.SD_csd.C_SIZE = %d\n", cardinfo.SD_csd.C_SIZE);
                    DLOG_Info("cardinfo.CardBlockSize = %d\n", cardinfo.CardBlockSize);
                   break;

            case HAL_SD_GET_SECTOR_SIZE:
            u32_tmpValue = cardinfo.SD_csd.READ_BL_LEN;
        switch(u32_tmpValue)
        {
          case 9:
            DLOG_Info("Maxi data block length: 512bytes\n");
            *pu32_info = 512;
            break;
          case 10:
            DLOG_Info("Maxi data block length: 1024bytes\n");
            *pu32_info = 1024;
            break;
          case 11:
            DLOG_Info("Maxi data block length: 2048bytes\n");
            *pu32_info = 2048;
            break;
          default:
            break;
        }
		    break;
		case HAL_SD_GET_CSD_VERSION:
		    *pu32_info = (uint32_t)cardinfo.SD_csd.CSD_STRUCTURE;
        if (cardinfo.SD_csd.CSD_STRUCTURE)
        {
          DLOG_Info("CSD Version 2.0\n");
        }
        else
        {
          DLOG_Info("CSD Version 1.0\n"); 
        }
		    break;
	case HAL_SD_GET_TRAN_SPEED:
            if (cardinfo.SD_csd.CSD_STRUCTURE == 0)
            {
                *pu32_info = (uint32_t)(cardinfo.SD_csd.TRAN_SPEED & 0x7);
                switch(*pu32_info)
                {
                  case 0:
                    tran_speed = 100;
                    tran_speed *= SD_SpeedTimesConvert( (cardinfo.SD_csd.TRAN_SPEED >> 3) & 0x7);
                    DLOG_Info("Maxi Data Transfer Rate: %fkbit/s\n", tran_speed);
                    break;
                  case 1:
                    tran_speed = 1;
                    tran_speed *= SD_SpeedTimesConvert( (cardinfo.SD_csd.TRAN_SPEED >> 3) & 0x7);
                    DLOG_Info("Maxi Data Transfer Rate: %fMbit/s\n", tran_speed);
                    break;
                  case 2:
                    tran_speed = 10;
                    tran_speed *= SD_SpeedTimesConvert( (cardinfo.SD_csd.TRAN_SPEED >> 3) & 0x7);
                    DLOG_Info("Maxi Data Transfer Rate: %fMbit/s\n", tran_speed);
                    break;
                  case 3:
                    tran_speed = 100;
                    tran_speed *= SD_SpeedTimesConvert( (cardinfo.SD_csd.TRAN_SPEED >> 3) & 0x7);
                    DLOG_Info("Maxi Data Transfer Rate: %fMbit/s\n", tran_speed);
                    break;
                  default:
                    break;
                }
            }
            else
            {
                // if UHS50
                DLOG_Info("Maxi Data Transfer Rate: 100Mbit/s\n");
                // if UHS104
               //  DLOG_Info("Maxi Data Transfer Rate: 200Mbit/s\n");
            }
        break;
    case HAL_SD_GET_CARD_STATUS:
        {
          SD_STATUS e_cardStatus;
          SD_CardStatus(&e_cardStatus);
          switch(e_cardStatus)
          {
            case SD_CARD_IDLE:
              DLOG_Info("Current Card State: IDLE\n");
              break;
            case SD_CARD_READY:
              DLOG_Info("Current Card State: READY\n");
              break;
            case SD_CARD_IDENTIFICATION:
              DLOG_Info("Current Card State: IDENTIFICATION\n");
              break;
            case SD_CARD_STANDBY:
              DLOG_Info("Current Card State: STANDBY\n");
              break;
            case SD_CARD_TRANSFER:
              DLOG_Info("Current Card State: TRANSFER\n");
              break;
            case SD_CARD_DATA:
              DLOG_Info("Current Card State: DATA\n");
              break;
            case SD_CARD_RECEIVE:
              DLOG_Info("Current Card State: RECEIVE\n");
              break;
            case SD_CARD_PROGRAMMING:
              DLOG_Info("Current Card State: PROGRAMMING\n");
              break;
            case SD_CARD_DISCONNECTED:
              DLOG_Info("Current Card State: DISCONNECTED\n");
              break;
            case SD_CARD_ERROR:
              DLOG_Info("Current Card State: ERROR\n");
              break;
            default:
              break;
          }
          break;
        }
		case HAL_SD_GET_MANUID:
		    *pu32_info = (uint32_t)cardinfo.SD_cid.ManufacturerID;
		    break;
	  case HAL_SD_GET_OEMID:
		    *pu32_info = (uint32_t)cardinfo.SD_cid.OEM_AppliID;
		    break;
		default:
			  break;
	}

	return HAL_OK;
}

HAL_RET_T HAL_SD_Fatfs_Init(void)
{
	FRESULT res;
    char *path = "SD:/";
    static FATFS fatfs;

    if (HAL_OK != HAL_SD_GetPresent())
    {
        return -1;
    }


    if( sdhandle.inited == 0 )
    {
        DLOG_Error("SD card not initilized");
        return HAL_SD_ERR_ERROR;
    }
    
	if (FATFS_LinkDriver(&SD_Driver, path) != 0)
	{
		DLOG_Error("Link error!");
		return HAL_SD_ERR_ERROR;
	}
	
	if ((res = f_mount(&fatfs, (TCHAR const*)path, 1)) != FR_OK)
	{
		DLOG_Error("f_mount = %d", res);
		return HAL_SD_ERR_ERROR;
	}
	else
	{
        sd_mountStatus = 1;
        DLOG_Info("mount to fat32 success, sd_mountStatus = %d", sd_mountStatus);	
    }
    
    return HAL_OK;
}

static uint32_t addrConvert(uint32_t addr)
{
    if (ENUM_CPU0_ID == CPUINFO_GetLocalCpuId())
    {
        if((addr >= 0 && addr <= 0x7FFFF) ||                // ITCM
           (addr >= 0x20000000 && addr <= 0x2007FFFF))      // DTCM
        {
            return addr + DTCM_CPU0_DMA_ADDR_OFFSET;
        }
        else
        {
            return addr;
        }
    }
    else if (ENUM_CPU1_ID == CPUINFO_GetLocalCpuId())
    {    
        if((addr >= 0 && addr <= 0x3FFFF) ||                // ITCM
           (addr >= 0x20000000 && addr <= 0x2003FFFF))      // DTCM
        {
            return addr + DTCM_CPU1_DMA_ADDR_OFFSET;
        }
        else
        {
            return addr;
        }
    }
    else
    {
        return HAL_SD_ERR_ERROR;
    }
}

HAL_RET_T HAL_SD_Write_test(uint32_t u32_dstBlkAddr, uint32_t u32_srcStartAddr, uint32_t u32_sectorNum)
{
	EMU_SD_RTN e_errorState = SD_OK;
	SDMMC_DMATransTypeDef st_dma;
	st_dma.BlockSize = 512;
    st_dma.SrcAddr = peripheralAddrConvert(u32_srcStartAddr);
	st_dma.DstAddr = (uint32_t )u32_dstBlkAddr;                        /* [block units] */
	st_dma.SectorNum = u32_sectorNum;
    
    if (getCardPresence != CARD_IN)
    {
        DLOG_Error("card not pressent");
        return SD_FAIL;
    }

    e_errorState = Card_SD_WriteMultiBlocks_DMA_test(&sdhandle, &st_dma);
	
	if (e_errorState != SD_OK) {
		DLOG_Error("Write SD Failed!\n");
		return HAL_SD_ERR_ERROR;
	}

	return HAL_OK;
}

HAL_RET_T HAL_SD_Read_test(uint32_t u32_dstStartAddr, uint32_t u32_srcBlkAddr, uint32_t u32_sectorNum)
{
	EMU_SD_RTN e_errorState = SD_OK;
	SDMMC_DMATransTypeDef st_dma;
	st_dma.BlockSize = 512;
	st_dma.SrcAddr = u32_srcBlkAddr;                     /* [block units] */
    st_dma.DstAddr = peripheralAddrConvert(u32_dstStartAddr);
	st_dma.SectorNum = u32_sectorNum;
	
    if (getCardPresence != CARD_IN)
    {
        DLOG_Error("card not pressent");
        return SD_FAIL;
    }

	if (u32_sectorNum == 1)
	{
		e_errorState = Card_SD_ReadBlock_DMA(&sdhandle, &st_dma);
	}
	else
	{
		e_errorState = Card_SD_ReadMultiBlocks_DMA_test(&sdhandle, &st_dma);
	}
	
	if (e_errorState != SD_OK) {
		DLOG_Error("Read SD Failed!");
		return HAL_SD_ERR_ERROR;
	}
	return HAL_OK;
}

HAL_RET_T HAL_SD_GetPresent(void)
{    
    if (getCardPresence != CARD_IN)
    {
        // DLOG_Error("card not pressent");
        return HAL_SD_ERR_ERROR;
    }

    return HAL_OK;
}
