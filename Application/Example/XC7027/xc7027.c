#include <stdlib.h>
#include <systicks.h>
#include "debuglog.h"
#include "serial.h"
#include "cmsis_os.h"
#include "sys_event.h"
#include "bb_spi.h"
#include "hal.h"
#include "hal_gpio.h"
#include "hal_bb.h"
#include "hal_hdmi_rx.h"
#include "hal_usb_otg.h"
#include "hal_sys_ctl.h"
#include "wireless_interface.h"
#include "hal_nv.h"
#include "cmd_line.h"
#include "hal_dma.h"
#include "hal_sd.h"
#include "hal_uart.h"
#include "bb_reply_pc.h"
#include "test_search_id_2.h"
#include "app_sys_event_process.h"
#include "hal_i2c.h"
#include "usr_usb_task.h"
#include "board_watchdog.h"
#include "hal_norflash.h"
#include "memory_config.h"
#include "sleep_mode.h"
#include "factory.h"
#include "hal_dvp.h"

#include "spcd_camera_config_1080p_30fps_ahead.h"
#include "spcd_camera_config_720p_30fps_ahead.h"
#include "spcd_camera_config_720p_50fps_ahead.h"
#include "spcd_camera_config_set.h"
#include "xc7027.h"

#define XC7027_DVP_CHANNEL		1

static volatile int is_Inited = 0;

void Reg_Write32(uint32_t regAddr, uint32_t regData);
void Reg_Write32_Mask(uint32_t regAddr, uint32_t regData, uint32_t regDataMask);
uint32_t Reg_Read32(uint32_t regAddr);

void XC7027_Reset(void)
{
	HAL_GPIO_OutPut(HAL_GPIO_NUM66);
    HAL_GPIO_SetPin(HAL_GPIO_NUM66, HAL_GPIO_PIN_RESET);
    HAL_Delay(15);
    HAL_GPIO_SetPin(HAL_GPIO_NUM66, HAL_GPIO_PIN_SET);
    HAL_Delay(50);
}

void XC7027_SENSOR_Config(ENUM_XC7027_MODE SensorFormat)
{
    uint32_t iRevalue = 0;
    uint32_t iI2cRevalue = 0;
    uint8_t iWrBuf[3];
    uint8_t iRdBuf = 0;
    uint32_t i = 0;
    const uint16_t* piISPRegister = NULL;
    uint32_t iISPRegisterLenght = 0;
    const uint16_t* piSensorRegister = NULL;
    uint32_t iSensorRegisterLenght = 0;
    uint8_t iRetry = 0;
	uint8_t mRetry = 0;

    HAL_I2C_MasterInit(HAL_I2C_COMPONENT_2, 0x001B, 1);
    HAL_Delay(3);
    HAL_I2C_MasterInit(HAL_I2C_COMPONENT_2, 0x0030, 1);
    HAL_Delay(3);

	switch(SensorFormat)
	{
		case XC7027_MODE_1080P_30FPS:
			HAL_DVP_Init(XC7027_DVP_CHANNEL, XC7027_DVP_CHANNEL, 1920, 1080, 30);
		    piISPRegister = XC7027_1080P_30FPS_SET_AHEAD;
			iISPRegisterLenght = sizeof(XC7027_1080P_30FPS_SET_AHEAD);
			piSensorRegister = SC2310_1080P_30FPS_SET_AHEAD;
		    iSensorRegisterLenght = sizeof(SC2310_1080P_30FPS_SET_AHEAD);
			break;

		case XC7027_MODE_720P_50FPS:
			HAL_DVP_Init(XC7027_DVP_CHANNEL, XC7027_DVP_CHANNEL, 1280, 720, 50);
		    piISPRegister = XC7027_720P_50FPS_SET_AHEAD;
			iISPRegisterLenght = sizeof(XC7027_720P_50FPS_SET_AHEAD);
			piSensorRegister = SC2310_720P_50FPS_SET_AHEAD;
		    iSensorRegisterLenght = sizeof(SC2310_720P_50FPS_SET_AHEAD);
			break;

		case XC7027_MODE_720P_30FPS:
			HAL_DVP_Init(XC7027_DVP_CHANNEL, XC7027_DVP_CHANNEL, 1280, 720, 30);
		    piISPRegister = XC7027_720P_30FPS_SET_AHEAD;
		    iISPRegisterLenght = sizeof(XC7027_720P_30FPS_SET_AHEAD);
			piSensorRegister = SC2310_720P_30FPS_SET_AHEAD;
		    iSensorRegisterLenght = sizeof(SC2310_720P_30FPS_SET_AHEAD);
			break;

		default: 
			HAL_DVP_Init(XC7027_DVP_CHANNEL, XC7027_DVP_CHANNEL, 1920, 1080, 30);
            piISPRegister = XC7027_1080P_30FPS_SET_AHEAD;
            iISPRegisterLenght = sizeof(XC7027_1080P_30FPS_SET_AHEAD);
			piSensorRegister = SC2310_1080P_30FPS_SET_AHEAD;
            iSensorRegisterLenght = sizeof(SC2310_1080P_30FPS_SET_AHEAD);
			break;
	}

	for(i = 0; i < (iISPRegisterLenght / 2); )
    {   
        iWrBuf[0] = (uint8_t)(piISPRegister[i] >> 8);
        iWrBuf[1] = (uint8_t)(piISPRegister[i]);
        iWrBuf[2] = (uint8_t)(piISPRegister[i + 1]);
        while(1)
		{
			mRetry = 0;
            iI2cRevalue = HAL_I2C_MasterWriteData(HAL_I2C_COMPONENT_2, 0x001B,  &iWrBuf[0], 3, 1000);
            if(iI2cRevalue == 0)
            {
                break;
            }
            else
            {
                DLOG_Critical("Step 0: ISP %ust Addr%04X=%02X,err=%08X\r\n", i, piISPRegister[i], iWrBuf[2], iI2cRevalue);
                if(iI2cRevalue == HAL_BUSY)
                {
                    HAL_Delay(4);
					mRetry++;
					if(mRetry >= 5)
						break;
                }
                else
                {
                    DLOG_Critical("Step 1: ISP %ust Addr%04X=%02X,err=%08X\r\n", i, piISPRegister[i], iWrBuf[2], iI2cRevalue);
                    iRevalue = 3;
                    break;
                }
            }
        }

        if(iRevalue != 0)
        {
            DLOG_Critical("iRevalue != 0, Loop Breaking..."); 
            break;
        }

		iWrBuf[0] = (uint8_t)(piISPRegister[i] >> 8);
        iWrBuf[1] = (uint8_t)(piISPRegister[i]);

        iI2cRevalue = HAL_I2C_MasterReadData(HAL_I2C_COMPONENT_2, 0x001B, &iWrBuf[0], 2, &iRdBuf, 1, 1000);

        if(iRdBuf != iWrBuf[2])
        {
            iRetry++;
            if(iRetry >= 3)
            {
                iRetry = 0;
                i += 2;
                DLOG_Critical("Step 2: ISP %ust Addr%04X,%02X!=iWrBuf[2],err=%08X\r\n", i, piISPRegister[i], iRdBuf, iI2cRevalue);
            }
            HAL_Delay(4);
        }
        else
        {
            iRetry = 0;
            i += 2;
        }
    }

	HAL_Delay(4);

	if(iRevalue == 0)
	{
		//旁路ISP
		for(i = 0; i < ((sizeof(BYPASS_ON))/2); )
		{
			iWrBuf[0] = (uint8_t)(BYPASS_ON[i]>>8);
			iWrBuf[1] = (uint8_t)(BYPASS_ON[i]);
			iWrBuf[2] = (uint8_t)(BYPASS_ON[i+1]);
			while(1)
			{
				iI2cRevalue = HAL_I2C_MasterWriteData(HAL_I2C_COMPONENT_2, 0x001B,  &iWrBuf[0], 3, 1000);
				if(iI2cRevalue == 0)
				{
					break;
				}
				else
				{
					DLOG_Critical( "Step 3: ISP %ust Addr%04X=%02X,err=%08X\r\n", i, piISPRegister[i], iWrBuf[2], iI2cRevalue);
					if(iI2cRevalue == HAL_BUSY)
					{
						HAL_Delay(4);
					}
					else
					{
						iRevalue = 3;
						break;
					}
				}
			}
			if(iRevalue != 0)
			{
				break;
			}
			HAL_Delay(2);
			iWrBuf[0] = (uint8_t)(BYPASS_ON[i]>>8);
			iWrBuf[1] = (uint8_t)(BYPASS_ON[i]);
			iI2cRevalue = HAL_I2C_MasterWriteData(HAL_I2C_COMPONENT_2, 0x001B,  &iWrBuf[0], 3, 1000);
			if(iRdBuf != iWrBuf[2])
			{
				iRetry++;
				if(iRetry >= 3)
				{
					iRetry = 0;
					i += 2;
					DLOG_Critical( "Step 4: ISP %ust Addr%04X,%02X!=iWrBuf[2],err=%08X\r\n", i, BYPASS_ON[i], iRdBuf, iI2cRevalue);
				}
				HAL_Delay(4);
			}
			else
			{
				iRetry = 0;
				i += 2;
			}
		}
	}
	HAL_Delay(4);

	if(iRevalue == 0)
	{
		//配置Sensor
		for(i = 0; i < (iSensorRegisterLenght/2); )
		{
			iWrBuf[0] = (uint8_t)(piSensorRegister[i]>>8);
			iWrBuf[1] = (uint8_t)(piSensorRegister[i]);
			iWrBuf[2] = (uint8_t)(piSensorRegister[i+1]);
			while(1)
			{
				iI2cRevalue = HAL_I2C_MasterWriteData(HAL_I2C_COMPONENT_2, 0x0030,  &iWrBuf[0], 3, 1000);
				if(iI2cRevalue == 0)
				{
					break;
				}
				else
				{
					DLOG_Critical( "Sensor %ust Addr%04X=%02X,err=%08X\r\n", i, piSensorRegister[i], iWrBuf[2], iI2cRevalue);
					if(iI2cRevalue == HAL_BUSY)
					{
						HAL_Delay(4);
					}
					else
					{
						iRevalue = 3;
						break;
					}
				}
			}
			if(iRevalue != 0)
			{
				break;
			}
			//HAL_Delay(2);
			iWrBuf[0] = (uint8_t)(piSensorRegister[i]>>8);
			iWrBuf[1] = (uint8_t)(piSensorRegister[i]);
			iI2cRevalue = HAL_I2C_MasterReadData(HAL_I2C_COMPONENT_2, 0x0030, &iWrBuf[0], 2, &iRdBuf, 1, 1000);
			if(iRdBuf != iWrBuf[2])
			{
				iRetry++;
				if(iRetry >= 3)
				{
					iRetry = 0;
					i += 2;
					DLOG_Critical( "Sensor %ust Addr%04X,%02X!=iWrBuf[2],err=%08X\r\n", i, piSensorRegister[i], iRdBuf, iI2cRevalue);
				}
					HAL_Delay(4);
			}
			else
			{
				iRetry = 0;
				i += 2;
			}
#if 0
			if((i % 50) == 0)
			{
				HAL_Delay(14);
			}
#endif
		}
	}
	HAL_Delay(4);
	if(iRevalue == 0)
	{
		//去旁路ISP
		for(i = 0; i < ((sizeof(BYPASS_OFF))/2); )
		{
			iWrBuf[0] = (uint8_t)(BYPASS_OFF[i]>>8);
			iWrBuf[1] = (uint8_t)(BYPASS_OFF[i]);
			iWrBuf[2] = (uint8_t)(BYPASS_OFF[i+1]);
			while(1)
			{
				iI2cRevalue = HAL_I2C_MasterWriteData(HAL_I2C_COMPONENT_2, 0x001B,  &iWrBuf[0], 3, 1000);
				if(iI2cRevalue == 0)
				{
					break;
				}
				else
				{
					DLOG_Critical( "ISP-DeBp %ust Addr%04X=%02X,err=%08X\r\n", i, piISPRegister[i], iWrBuf[2], iI2cRevalue);
					if(iI2cRevalue == HAL_BUSY)
					{
						HAL_Delay(4);
					}
					else
					{
						iRevalue = 3;
						break;
					}
				}
			}
			if(iRevalue != 0)
			{
				break;
			}
			HAL_Delay(2);
			iWrBuf[0] = (uint8_t)(BYPASS_OFF[i]>>8);
			iWrBuf[1] = (uint8_t)(BYPASS_OFF[i]);
			iI2cRevalue = HAL_I2C_MasterReadData(HAL_I2C_COMPONENT_2, 0x001B, &iWrBuf[0], 2, &iRdBuf, 1, 1000);
			if(iRdBuf != iWrBuf[2])
			{
				iRetry++;
				if(iRetry >= 3)
				{
					iRetry = 0;
					i += 2;
					DLOG_Critical( "ISP-DeBP %ust Addr%04X,%02X!=iWrBuf[2],err=%08X\r\n", i, BYPASS_OFF[i], iRdBuf, iI2cRevalue);
				}
				HAL_Delay(4);
			}
			else
			{
				iRetry = 0;
				i += 2;
			}
		}
	} 
}

int XC7027_ENC_Config()
{
	uint8_t miDVPChannel = XC7027_DVP_CHANNEL;
	HAL_DVP_SendInfoToEncoder(XC7027_DVP_CHANNEL);

	HAL_Delay(100);	//延时一段时间让CPU2操作完DVP
	/*//开启8bit mode.
	Reg_Write32_Mask(0xA0010000, (uint32_t)1 << 21, (uint32_t)1 << 21);
	Reg_Write32_Mask(0xA0010000+(0x19<<2), (uint32_t)1 << 21, (uint32_t)1 << 21);
	//Porch:
	//View0:  
	Reg_Write32(0xA0010000+(0x3<<2), 0);
	Reg_Write32_Mask(0xA0010000+(0x4<<2), 0, (((uint32_t)0xFFF << 16) | 0xFFF));
	//View1:  
	Reg_Write32(0xA0010000+(0x1c<<2), 0);
	Reg_Write32_Mask(0xA0010000+(0x1d<<2), 0, (((uint32_t)0xFFF << 16) | 0xFFF));*/
	if(miDVPChannel == 0)
	{
		//要在配置这些配置之前，最好先把encoder 关掉，配置完在打开。
		//关掉：
		//View0:0x10写0x00
		Reg_Write32_Mask(0xA0010000+(0x00<<2), (uint32_t)(0 << 24), (uint32_t)(1 << 24));
		//开启8bit mode. & VS Positive Polarity & HS Negative Polarity
		//*(0x01)|0x24
		Reg_Write32_Mask(0xA0010000, (uint32_t)((1 << 21) | (4<<16)), (uint32_t)((1 << 21) | (7 << 16)));
		//Porch:
		//View0: 
		Reg_Write32(0xA0010000+(0x3<<2), 0x00010000);
		Reg_Write32_Mask(0xA0010000+(0x4<<2), 0, (((uint32_t)0xFFF << 16) | 0xFFF));
		//关掉自动检测机制
		Reg_Write32_Mask(0xA0010000+(0x0a<<2), (uint32_t)(0<<5), (uint32_t)(3 << 5));
		//DE mode & valid data in low 8bit:
		Reg_Write32_Mask(0xA0010000+(0x04<<2), (uint32_t)(1<<13), (uint32_t)(7 << 13));
#if 0
		//先把watch dog关掉， 因为现在帧率有点低。
		Reg_Write32_Mask(0xA0010000+(0x04<<2), (uint32_t)(0 << 28), (uint32_t)(0xf << 28));
		Reg_Write32_Mask(0xA0010000+(0x33<<2), (uint32_t)(0 << 4), (uint32_t)(0xf << 4));
#endif
		//同时也把码流控制也关掉吧，因为帧率不正确码流也不正确。
		//Reg_Write32_Mask(0xA0010000+(0x05<<2), (uint32_t)(0 << 24), (uint32_t)(1 << 24));
		//同时也把码流控制打开
		Reg_Write32_Mask(0xA0010000+(0x05<<2), (uint32_t)(1 << 24), (uint32_t)(1 << 24));
		//打开：建议用工具手动打开，因为你现在还在调试阶段。
		//View0:
		Reg_Write32_Mask(0xA0010000+(0x00<<2), (uint32_t)(1<<24), (uint32_t)(1<<24));
	}
	else
	{
		//要在配置这些配置之前，最好先把encoder 关掉，配置完在打开。
		//关掉：
		//View1:0x64写0x00   
		Reg_Write32_Mask(0xA0010000+(0x19<<2), (uint32_t)(0 << 24), (uint32_t)(1 << 24));
		//开启8bit mode. & VS Positive Polarity & HS Negative Polarity
		//*(0x65)|0x24
		Reg_Write32_Mask(0xA0010000+(0x19<<2), (uint32_t)((1 << 21) | (4<<16)), (uint32_t)((1 << 21) | (7 << 16)));
		//Porch:
		//View1: 
		Reg_Write32(0xA0010000+(0x1c<<2), 0x00010000);
		Reg_Write32_Mask(0xA0010000+(0x1d<<2), 0, (((uint32_t)0xFFF << 16) | 0xFFF));
		//关掉自动检测机制
		Reg_Write32_Mask(0xA0010000+(0x23<<2), (uint32_t)(0<<5), (uint32_t)(3 << 5));
		//DE mode & valid data in low 8bit:
		Reg_Write32_Mask(0xA0010000+(0x1D<<2), (uint32_t)(1<<13), (uint32_t)(7 << 13));
	
#if 0
		//先把watch dog关掉， 因为现在帧率有点低。
		Reg_Write32_Mask(0xA0010000+(0x1D<<2), (uint32_t)(0 << 28), (uint32_t)(0xf << 28));
		Reg_Write32_Mask(0xA0010000+(0x33<<2), (uint32_t)(0 << 4), (uint32_t)(0xf << 4));
#endif
		//同时也把码流控制也关掉吧，因为帧率不正确码流也不正确。
		//Reg_Write32_Mask(0xA0010000+(0x1e<<2), (uint32_t)(0 << 24), (uint32_t)(1 << 24));
		//同时也把码流控制打开
		Reg_Write32_Mask(0xA0010000+(0x1e<<2), (uint32_t)(1 << 24), (uint32_t)(1 << 24));
		//打开：建议用工具手动打开，因为你现在还在调试阶段。
		//View1:
		Reg_Write32_Mask(0xA0010000+(0x19<<2), (uint32_t)(1<<24), (uint32_t)(1<<24));
	}

	return 0;
}


void XC7027_SENSOR_LoopCallBack(void)
{
	uint8_t miDVPChannel = XC7027_DVP_CHANNEL;

	if (is_Inited == 0)
		return;

	if(miDVPChannel == 0)
	{
		if((Reg_Read32(0xA0010000+(0x3<<2))&0x00FF0000) != 0x00010000)
		{
			DLOG_Critical("DVP0 Call Patch Start!\n\n");
				
			//要在配置这些配置之前，最好先把encoder 关掉，配置完在打开。
			//关掉：
			//View0:0x10写0x00
			Reg_Write32_Mask(0xA0010000+(0x00<<2), (uint32_t)(0 << 24), (uint32_t)(1 << 24));
			//开启8bit mode. & VS Positive Polarity & HS Negative Polarity
			//*(0x01)|0x24
			Reg_Write32_Mask(0xA0010000, (uint32_t)((1 << 21) | (4<<16)), (uint32_t)((1 << 21) | (7 << 16)));
			//Porch:
			//View0: 
			Reg_Write32(0xA0010000+(0x3<<2), 0x00010000);
			Reg_Write32_Mask(0xA0010000+(0x4<<2), 0, (((uint32_t)0xFFF << 16) | 0xFFF));
			//关掉自动检测机制
			Reg_Write32_Mask(0xA0010000+(0x0a<<2), (uint32_t)(0<<5), (uint32_t)(3 << 5));
			//DE mode & valid data in low 8bit:
			Reg_Write32_Mask(0xA0010000+(0x04<<2), (uint32_t)(1<<13), (uint32_t)(7 << 13));
#if 0
			//先把watch dog关掉， 因为现在帧率有点低。
			Reg_Write32_Mask(0xA0010000+(0x04<<2), (uint32_t)(0 << 28), (uint32_t)(0xf << 28));
			Reg_Write32_Mask(0xA0010000+(0x33<<2), (uint32_t)(0 << 4), (uint32_t)(0xf << 4));
#endif
			//同时也把码流控制也关掉吧，因为帧率不正确码流也不正确。
			//Reg_Write32_Mask(0xA0010000+(0x05<<2), (uint32_t)(0 << 24), (uint32_t)(1 << 24));
			//同时也把码流控制打开
			Reg_Write32_Mask(0xA0010000+(0x05<<2), (uint32_t)(1 << 24), (uint32_t)(1 << 24));
			//打开：建议用工具手动打开，因为你现在还在调试阶段。
			//View0:
			Reg_Write32_Mask(0xA0010000+(0x00<<2), (uint32_t)(1<<24), (uint32_t)(1<<24));

			/*//View0:0x10写0x00
			Reg_Write32_Mask(0xA0010000+(0x00<<2), (uint32_t)(0 << 24), (uint32_t)(1 << 24));
			Reg_Write32(0xA0010000+(0x3<<2), 0x00010000);
			//View0:
			Reg_Write32_Mask(0xA0010000+(0x00<<2), (uint32_t)(1<<24), (uint32_t)(1<<24));*/

			DLOG_Critical("DVP0 Call Patch End!\n\n");
		}
	}
	else
	{
		if((Reg_Read32(0xA0010000+(0x1c<<2))&0x00FF0000) != 0x00010000)
		{
			DLOG_Critical("DVP1 Call Patch Start!\n\n");	
			//要在配置这些配置之前，最好先把encoder 关掉，配置完在打开。
			//关掉：
			//View1:0x64写0x00   
			Reg_Write32_Mask(0xA0010000+(0x19<<2), (uint32_t)(0 << 24), (uint32_t)(1 << 24));
			//开启8bit mode. & VS Positive Polarity & HS Negative Polarity
			//*(0x65)|0x24
			Reg_Write32_Mask(0xA0010000+(0x19<<2), (uint32_t)((1 << 21) | (4<<16)), (uint32_t)((1 << 21) | (7 << 16)));
			//Porch:
			//View1: 
			Reg_Write32(0xA0010000+(0x1c<<2), 0x00010000);
			Reg_Write32_Mask(0xA0010000+(0x1d<<2), 0, (((uint32_t)0xFFF << 16) | 0xFFF));
			//关掉自动检测机制
			Reg_Write32_Mask(0xA0010000+(0x23<<2), (uint32_t)(0<<5), (uint32_t)(3 << 5));
			//DE mode & valid data in low 8bit:
			Reg_Write32_Mask(0xA0010000+(0x1D<<2), (uint32_t)(1<<13), (uint32_t)(7 << 13));
				
#if 0
			//先把watch dog关掉， 因为现在帧率有点低。
			Reg_Write32_Mask(0xA0010000+(0x1D<<2), (uint32_t)(0 << 28), (uint32_t)(0xf << 28));
			Reg_Write32_Mask(0xA0010000+(0x33<<2), (uint32_t)(0 << 4), (uint32_t)(0xf << 4));
#endif
			//同时也把码流控制也关掉吧，因为帧率不正确码流也不正确。
			//Reg_Write32_Mask(0xA0010000+(0x1e<<2), (uint32_t)(0 << 24), (uint32_t)(1 << 24));
			//同时也把码流控制打开
			Reg_Write32_Mask(0xA0010000+(0x1e<<2), (uint32_t)(1 << 24), (uint32_t)(1 << 24));
			//打开：建议用工具手动打开，因为你现在还在调试阶段。
			//View1:
			Reg_Write32_Mask(0xA0010000+(0x19<<2), (uint32_t)(1<<24), (uint32_t)(1<<24));

			/*//View0:0x10写0x00
			Reg_Write32_Mask(0xA0010000+(0x19<<2), (uint32_t)(0 << 24), (uint32_t)(1 << 24));
			Reg_Write32(0xA0010000+(0x1c<<2), 0x00010000);
			//View1:
			Reg_Write32_Mask(0xA0010000+(0x19<<2), (uint32_t)(1<<24), (uint32_t)(1<<24));*/

			DLOG_Critical("DVP1 Call Patch End!\n\n");
		}
	}
}

void XC7027_Sensor_Start(void)
{
	XC7027_Reset();
	XC7027_SENSOR_Config(XC7027_MODE_720P_50FPS);
//	XC7027_SENSOR_Config(XC7027_MODE_1080P_30FPS);
	HAL_Delay(20);
	XC7027_ENC_Config();

	HAL_Delay(20);
	is_Inited = 1;
/*
	osTimerDef(SensorLoopTask, Sensor_Task_Loop);
    osTimerId SensorLoopTimer = osTimerCreate(osTimer(SensorLoopTask), osTimerPeriodic, NULL);
    osTimerStart(SensorLoopTimer, 100);
**/
}

