#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "debuglog.h"
#include "hal_uart.h"
#include "hal_gpio.h"
#include "cmsis_os.h"
#include "factory.h"
#include "hal.h"
#include "hal_dvp.h"
#include "test_bb_led_ctrl.h"

#define HDMI_CMD_UART HAL_UART_COMPONENT_6
#define HDMI_CMD_GPIO HAL_GPIO_NUM66
#define HDMI_CMD_UART_BUF_LEN 64

//static uint8_t hdmi_status = 0;
static uint8_t hdmi_gpio_irq_flag;
static uint8_t hdmi_cmd_uart_data[HDMI_CMD_UART_BUF_LEN];
static uint8_t hdmi_cmd_uart_data_len;

struct hdmi_timing{
    uint16_t u16_width;
    uint16_t u16_hight;
    uint8_t u8_framerate;
};

struct hdmi_timing cur_hdmi_timing,last_hdmi_timing;

const struct hdmi_timing support_hdmi_timing[] =
{
    {1280,720,25},
    {1280,720,30},
    {1280,720,50},
    {1280,720,60},
    {1920,1080,25},
    {1920,1080,30},
};

extern uint8_t hdmi_status;

ENUM_HAL_UART_BAUDR get_fac_uart6_baudrate(void)
{
    STRU_UART_BAUDR *pst_uart_baudr = NULL;
    ENUM_HAL_UART_BAUDR enum_baud = HAL_UART_BAUDR_115200;
    
    pst_uart_baudr = (STRU_UART_BAUDR *)FCT_GetNodeAndData(FACTORY_SUBNODE_UART_BAUDR_ID,NULL);
    if(pst_uart_baudr != NULL)
    {
        if(pst_uart_baudr->st_uartBaudr[2] <= HAL_UART_BAUDR_460800)
        {
            enum_baud = (ENUM_HAL_UART_BAUDR)pst_uart_baudr->st_uartBaudr[2];
            DLOG_Warning("bypass uart6 baud = %d",enum_baud);
        }
        else
        {
            DLOG_Warning("bypass uart6 baud %d ,force to  = %d",pst_uart_baudr->st_uartBaudr[2],enum_baud);
        }
    }
    else
    {
        DLOG_Warning("get pst_uart_baudr null");
    }
    return enum_baud;

}
static void HDMI_cmdGPIOhal_IRQHandler(uint32_t u32_vectorNum)
{
    hdmi_gpio_irq_flag = 1;
}

void HDMI_cmdGpioInterrupt_Init(void)
{	
    if(0==HAL_GPIO_RegisterInterrupt(HDMI_CMD_GPIO, HAL_GPIO_LEVEL_SENUMSITIVE, HAL_GPIO_ACTIVE_LOW, HDMI_cmdGPIOhal_IRQHandler))
    {
        DLOG_Warning("ok %d",HDMI_CMD_GPIO);
    }
    else
    {
        DLOG_Error("fail %d",HDMI_CMD_GPIO);
    }

}
void dump_data(uint8_t *pu8_rxBuf, uint8_t u8_len)
{
    uint8_t i;
    
    for(i=0;i<u8_len;i++)
        DLOG_Warning("%x",pu8_rxBuf[i]);
}
static uint32_t bypass_hal_uart6_irq_handler(uint8_t *pu8_rxBuf, uint8_t u8_len)
{
    uint8_t i;
    
    for(i=0;i<u8_len;i++)
    {
        hdmi_cmd_uart_data[hdmi_cmd_uart_data_len] = pu8_rxBuf[i];
        hdmi_cmd_uart_data_len++;
        if(hdmi_cmd_uart_data_len >= HDMI_CMD_UART_BUF_LEN)
        {
            hdmi_cmd_uart_data_len = 0;
            DLOG_Error("buf over");
        }
    }

    dump_data(pu8_rxBuf,u8_len);

    
}

void cmd_ch7038(uint8_t cmd,uint8_t param)
{
    int ret;
    uint8_t cmd_buf[4];
    
    cmd_buf[0] = 0xfe;
    cmd_buf[1] = cmd;
    cmd_buf[2] = param;
    cmd_buf[3] = 0xff;
    ret = HAL_UART_TxData(HDMI_CMD_UART, cmd_buf, 4, HAL_UART_DEFAULT_TIMEOUTMS * 1000);
    if(ret != HAL_OK)
    {
        DLOG_Error("failed ret %d",ret);
    }

}

uint8_t parse_cmdHdmi_timing(void)
{
    uint8_t i;

    for(i=0;i<hdmi_cmd_uart_data_len;i++)
    {
        if((hdmi_cmd_uart_data[i] != 0xa5) && 
            ((i + 6) <= hdmi_cmd_uart_data_len) && 
            (hdmi_cmd_uart_data[i+5] == 0xa5))
        {
            cur_hdmi_timing.u16_width = hdmi_cmd_uart_data[i] << 8 | hdmi_cmd_uart_data[i+1]; 
            cur_hdmi_timing.u16_hight= hdmi_cmd_uart_data[i+2] << 8 | hdmi_cmd_uart_data[i+3]; 
            cur_hdmi_timing.u8_framerate = hdmi_cmd_uart_data[i+4] ; 
            return 1;
        }
    }

    return 0;
}

uint8_t diff_value(uint16_t val1,uint16_t val2)
{
    if(val1 >= val2)
    {
        return val1 - val2;
    }
    else
    {
        return val2 - val1;
    }
}

int check_cmdHdmi_timing(void)
{
    int i;
    int size = sizeof(support_hdmi_timing)/sizeof(support_hdmi_timing[0]);

    if(cur_hdmi_timing.u16_width == 0 && cur_hdmi_timing.u16_hight == 0 && cur_hdmi_timing.u8_framerate == 0)
    {
        return 2;
    }

    //return 1;

    for(i=0;i<size;i++)
    {
        if(diff_value(cur_hdmi_timing.u16_width,support_hdmi_timing[i].u16_width) <= 2 &&
            diff_value(cur_hdmi_timing.u16_hight,support_hdmi_timing[i].u16_hight) <= 2 &&
            diff_value(cur_hdmi_timing.u8_framerate,support_hdmi_timing[i].u8_framerate) <= 2)
        {
            return 1;
        }
    }
    
    return 0;
}

void command_hdmi_ch7038_edid(char* str_edid)
{
    unsigned char edid = strtoul(str_edid, NULL, 0);
    cmd_ch7038(0x02,edid);//set hdmi edie
    DLOG_Warning("set edid %d",edid);

}
static void hdmi_ch7038_task(void const *argument)
{
    int ret,cnt = 0,state=1,tcnt;
    uint32_t p_retGpioState;
    uint8_t timing = 5;//default 1080p60
    int hdmi_output_timing_flag;

    hdmi_output_timing_flag = 0;
    hdmi_gpio_irq_flag = 0;
    hdmi_cmd_uart_data_len = 0;
    tcnt = 0;

    while(1)
    {
        HAL_GPIO_GetPin(HDMI_CMD_GPIO,&p_retGpioState);
        if(!p_retGpioState)
        {
            if(++cnt > 3)
                hdmi_gpio_irq_flag = 1;
        }
        else
        {
            hdmi_gpio_irq_flag = 0;
            cnt = 0;
        }

        switch(state)
        {
            case 0 :
                if(hdmi_gpio_irq_flag)
                {
                    hdmi_gpio_irq_flag = 0;
                    cmd_ch7038(0x01,0x00); //clear gpio5 level
                    state = 1;
                    DLOG_Warning("reset gpio5 level");
                }

                break;
            case 1 :
                cmd_ch7038(0x03,0x01);//get hdmi output format cmd
                state = 2;
                break;
            case 2 :
                hdmi_output_timing_flag =parse_cmdHdmi_timing();
                if(hdmi_output_timing_flag)
                {
                    hdmi_output_timing_flag = 0;
                    hdmi_cmd_uart_data_len = 0;
                    state = 3;
                }
                else if(tcnt++ > 3)//failed to get hdmi timing, re-sent cmd
                {
                    tcnt = 0;
                    state = 1;
                    DLOG_Warning("get hdmi timing timeout");
                }
                break;
            case 3 :
                hdmi_output_timing_flag = check_cmdHdmi_timing();
                DLOG_Warning("W=%d,H=%d,F=%d",cur_hdmi_timing.u16_width,cur_hdmi_timing.u16_hight,cur_hdmi_timing.u8_framerate);
                if(hdmi_output_timing_flag)
                {
                    //if(diff_value(cur_hdmi_timing.u16_width,last_hdmi_timing.u16_width) > 2 ||
                    //    diff_value(cur_hdmi_timing.u16_hight,last_hdmi_timing.u16_hight) > 2 ||
                    //    diff_value(cur_hdmi_timing.u8_framerate,last_hdmi_timing.u8_framerate) > 2)
                    {
                        
                        HAL_DVP_Init(1,1,cur_hdmi_timing.u16_width,cur_hdmi_timing.u16_hight,cur_hdmi_timing.u8_framerate);
                        HAL_DVP_SendInfoToEncoder(1);
                        hdmi_status = hdmi_output_timing_flag == 2 ? 0 : 1;
                        //last_hdmi_timing.u16_width = support_hdmi_timing[hdmi_output_timing_flag].u16_width;
                        //last_hdmi_timing.u16_hight = support_hdmi_timing[hdmi_output_timing_flag].u16_hight;
                        //last_hdmi_timing.u8_framerate = support_hdmi_timing[hdmi_output_timing_flag].u8_framerate;

                    }

                }
                else
                {
                    /*timing--;
                    cmd_ch7038(0x02,timing);//set hdmi edie
                    DLOG_Warning("set edid %d",timing);
                    if(timing == 0)
                    {
                        timing = 6;
                    }*/
                    
                    hdmi_status = 0;
                }
                state = 0;
                break;
            default:
                DLOG_Warning("undef state");
                break;
        }

        HAL_Delay(200);
    }

}

void test_hdmi_ch7038_init(void)
{
    ENUM_HAL_UART_BAUDR enum_baud;

    hdmi_cmd_uart_data_len = 0;
    memset(hdmi_cmd_uart_data,0,HDMI_CMD_UART_BUF_LEN);
    
    enum_baud = get_fac_uart6_baudrate();
    HAL_UART_Init(HDMI_CMD_UART,enum_baud,bypass_hal_uart6_irq_handler);
    DLOG_Warning("uart6 baudrate %d",enum_baud);

    HAL_GPIO_SetMode(HDMI_CMD_GPIO,HAL_GPIO_PIN_MODE2);
    HAL_GPIO_InPut(HDMI_CMD_GPIO);
    
    osThreadDef(test_hdmi_ch7038_task, hdmi_ch7038_task, osPriorityHigh, 0, 4 * 128);
    osThreadCreate(osThread(test_hdmi_ch7038_task), NULL);

    //HDMI_cmdGpioInterrupt_Init();

    
}
