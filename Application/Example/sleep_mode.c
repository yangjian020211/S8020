#include <stdlib.h>
#include "debuglog.h"
#include "hal.h"
#include "sleep_mode.h"
#include "hal_bb.h"
#include "hal_gpio.h"
#include "test_bb_led_ctrl_2.h"

static uint8_t FemPin[] = {92, 96};

static void EnterSleep(uint8_t level);

void tx_bypass(int en){
	if(en==1)
	{
		HAL_GPIO_OutPut(HAL_GPIO_NUM50);//SPI_SS_0_N5
		HAL_GPIO_InPut(HAL_GPIO_NUM51);//SLI_SCLK_OUT5
		HAL_GPIO_OutPut(HAL_GPIO_NUM52);//SPI_TXD5
		HAL_GPIO_InPut(HAL_GPIO_NUM53);//SPI_RXD5
		
		HAL_GPIO_SetPin(HAL_GPIO_NUM50,HAL_GPIO_PIN_RESET);
		HAL_GPIO_SetPin(HAL_GPIO_NUM52,HAL_GPIO_PIN_SET);
#if 1
		HAL_GPIO_OutPut(HAL_GPIO_NUM108);//UART_RXD5 for debug test 
		HAL_GPIO_OutPut(HAL_GPIO_NUM115);//UART_TXD5 for debug test 
		HAL_GPIO_SetPin(HAL_GPIO_NUM108,HAL_GPIO_PIN_RESET);	
		HAL_GPIO_SetPin(HAL_GPIO_NUM54,HAL_GPIO_PIN_RESET);	
		DLOG_Critical("set gpio_num_108 0");
		DLOG_Critical("set gpio_num_115 0");
#endif
		//close B path fem
		HAL_GPIO_OutPut(HAL_GPIO_NUM46);//spi ss6
		HAL_GPIO_OutPut(HAL_GPIO_NUM47);//spi clk6
		HAL_GPIO_SetPin(HAL_GPIO_NUM46,HAL_GPIO_PIN_RESET);//C0=0
    	HAL_GPIO_SetPin(HAL_GPIO_NUM47,HAL_GPIO_PIN_RESET);//C1=0
		
		
	}
	else
	{
		HAL_GPIO_OutPut(HAL_GPIO_NUM50);//SPI_SS_0_N5: AR8003S1_SW_A2_2G
		HAL_GPIO_OutPut(HAL_GPIO_NUM51);//SLI_SCLK_OUT5:AR8003S1_SW_B1_2G
		HAL_GPIO_OutPut(HAL_GPIO_NUM52);//SPI_TXD5:AR8003S1_SW_A1_2G
		HAL_GPIO_OutPut(HAL_GPIO_NUM53);//SLI_RXD5:AR8003S1_SW_B2_2G
		
		HAL_GPIO_InPut(HAL_GPIO_NUM108);//UART_RXD5 for debug test
		HAL_GPIO_InPut(HAL_GPIO_NUM115);//UART_TXD5 for debug test
		
		DLOG_Critical("set gpio_num_108 as input pin");
		DLOG_Critical("set gpio_num_115 as input pin");

		HAL_GPIO_SetPin(HAL_GPIO_NUM50,HAL_GPIO_PIN_SET);
		HAL_GPIO_SetPin(HAL_GPIO_NUM51,HAL_GPIO_PIN_RESET);
		HAL_GPIO_SetPin(HAL_GPIO_NUM52,HAL_GPIO_PIN_RESET);
		HAL_GPIO_SetPin(HAL_GPIO_NUM53,HAL_GPIO_PIN_SET);

		//open B path fem
		HAL_GPIO_InPut(HAL_GPIO_NUM46);//spi ss6 :C0=input
		HAL_GPIO_InPut(HAL_GPIO_NUM47);//spi clk6:C1=input
	}
}



void FemOn(uint8_t ch)
{
    if(ch > 1)
    {
        return;
    }
    
    HAL_GPIO_SetMode(FemPin[ch], HAL_GPIO_PIN_MODE2);    
    HAL_GPIO_OutPut(FemPin[ch]);
    HAL_GPIO_SetPin(FemPin[ch],  HAL_GPIO_PIN_SET);
}

void FemOff(uint8_t ch)
{
    if(ch > 1)
    {
        return;
    }
    
    HAL_GPIO_SetMode(FemPin[ch], HAL_GPIO_PIN_MODE2);    
    HAL_GPIO_OutPut(FemPin[ch]);
    HAL_GPIO_SetPin(FemPin[ch],  HAL_GPIO_PIN_RESET);
}

static void EnterSleep(uint8_t level)
{
    if(0 == level)
    {
        FemOn(0);
        FemOn(1);
		HAL_PwrCtrlSet(level);//pwr_level 0
		HAL_Delay(200);
		tx_bypass(0);//for uav project
		HAL_PA_modeCtrlSet(0);//set the pa work in close mode when switch to the fem work path
    }
    if(1 == level)
    {
        FemOn(0);
        FemOff(1);
		HAL_PwrCtrlSet(level);
    }
    else if(3 == level || 2 == level)
    {
        FemOff(0);
        FemOff(1);
        led_enter_sleep();
		HAL_PwrCtrlSet(level);
    }
	else if(4==level)
	{	
		HAL_PA_modeCtrlSet(1);//set the pa work in open mode before switch out of fem path
		tx_bypass(1);//for uav project
		HAL_PwrCtrlSet(level);//pwr_level 4
	}
    DLOG_Warning("enter sleep: %d",level);
}
    
void command_EnterSleep(uint8_t *level)
{
    uint8_t tmpLevel = strtoul(level, NULL, 0);

    EnterSleep(tmpLevel);
    DLOG_Warning("sleep level:%x", HAL_GetPwrCtrlLevel());
}

void command_ExitSleep(void)
{
    EnterSleep(0);
    DLOG_Warning("exit sleep");
}


void SleepModeProcess(void *p)
{
    uint8_t *value = (uint8_t *)p;

    EnterSleep(value[1]);
}

void Mod_Pin_SleepTask(void const *argument)
{
    uint8_t cnt = 0;
    uint32_t pin_value;
    uint8_t isSleeped=0;
    uint8_t sleep_level;

    #define EXTERN_SLEEP_PIN (HAL_GPIO_NUM97)

    HAL_GPIO_SetMode(EXTERN_SLEEP_PIN, HAL_GPIO_PIN_MODE2);  
    HAL_GPIO_InPut(EXTERN_SLEEP_PIN);
    while (1)
    {
        HAL_GPIO_GetPin(EXTERN_SLEEP_PIN,&pin_value);
        if(pin_value == 0)
        {
            cnt++;
            if(cnt >= 3)
            {
                cnt = 0;
                if(!isSleeped)
                {
                    sleep_level = HAL_GetPwrCtrlLevel();
                    EnterSleep(3);
                    isSleeped = 1;
                    DLOG_Warning("enter sleep: %d->3",sleep_level);
                }
            }
        }
        else
        {
            cnt = 0;
            if(isSleeped)
            {
                isSleeped = 0;
                EnterSleep(sleep_level);
                DLOG_Warning("exit sleep: 3->%d",sleep_level);
            }
        }

        HAL_Delay(200);
    }
}

