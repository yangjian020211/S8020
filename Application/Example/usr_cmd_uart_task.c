#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include "cmsis_os.h"
#include "hal_nvic.h"
#include "hal_bb.h"
#include "hal_uart.h"
#include "debuglog.h"
#include "systicks.h"
#include "hal.h"
#include "ringbuf.h"
#include "hal_gpio.h"
#include "interrupt.h"
#include "ar8020.h"
#include "hal_gpio.h"
#include "hal_timer.h"
#include "memory_config.h"
#include "factory.h"
#include "queue.h"
#include "test_bb_led_ctrl_2.h"
#include "test_search_id_2.h"
#include "data_type.h"
#include "usr_protocol.h"
#include "debuglog.h"

#define CMD_UART_NUM HAL_UART_COMPONENT_7
static osMutexId cmd_uart_mutex_id;
static uint8_t dev_type;

uint16_t get_fac_update_time(uint8_t *enable)
{
    STRU_PWR_CTRL *pst_pwr_ctrl = NULL;
    #define MIN_UPDATE_INTERVAL 14 
    uint16_t update_time = MIN_UPDATE_INTERVAL;

    *enable = TRUE;  
    pst_pwr_ctrl = (STRU_PWR_CTRL *)FCT_GetNodeAndData(FACTORY_SUBNODE_PWR_CTRL,NULL);
    if(pst_pwr_ctrl != NULL)
    {
        update_time = pst_pwr_ctrl->mod_status_update_time;
        *enable = pst_pwr_ctrl->mod_status_update_enable;
    }
    else
    {
        DLOG_Warning("get pwr ctrl null");
    }

    if(update_time < MIN_UPDATE_INTERVAL)
    {
        DLOG_Warning("force to %d",MIN_UPDATE_INTERVAL);
        update_time = MIN_UPDATE_INTERVAL;
    }

    return update_time;
}

static uint32_t hal_cmd_uart_irq_handler(uint8_t *pu8_rxBuf, uint8_t u8_len)
{
    BQForceQueueBytes(pu8_rxBuf,u8_len);
    //DLOG_Warning("get data %d",u8_len);
}
static bool find_sync_head(uint8 *data, uint32_t len, uint32_t *skip_offset)
{
	uint32_t i;

    #define SYNC_HEAD1 0xff
    #define SYNC_HEAD2 0x5a

	for(i=0;i<len-1;i++)
	{
		if(data[i] == SYNC_HEAD1 && data[i+1] == SYNC_HEAD2)
		{
			*skip_offset  = i;
			return TRUE;
		}
	}

	*skip_offset = len;
	return FALSE;
}

ENUM_HAL_UART_BAUDR get_fac_uart7_baudrate(void)
{
    STRU_UART_BAUDR *pst_uart_baudr = NULL;
    ENUM_HAL_UART_BAUDR enum_baud = HAL_UART_BAUDR_115200;
        
    pst_uart_baudr = (STRU_UART_BAUDR *)FCT_GetNodeAndData(FACTORY_SUBNODE_UART_BAUDR_ID,NULL);
    if(pst_uart_baudr != NULL)
    {
        if(pst_uart_baudr->st_uartBaudr[3] <= HAL_UART_BAUDR_460800)
        {
            enum_baud = (ENUM_HAL_UART_BAUDR)pst_uart_baudr->st_uartBaudr[3];
            DLOG_Warning("cmd-uart7 baud = %d",enum_baud);
        }
        else
        {
            DLOG_Warning("cmd-uart7 baud %d ,force to  = %d",pst_uart_baudr->st_uartBaudr[3],enum_baud);
        }

    }
    else
    {
        DLOG_Warning("get pst_uart_baudr null");
    }
    
    return enum_baud;

}

uint8_t get_ground_mod_status_info(uint8_t *buf)
{
    int len;
    uint16_t ret;
    
    buf[0] = 0xff;
    buf[1] = 0x5a;
    buf[2] = 0x82;
    buf[3] = 0x00;
    buf[4] = 0x00;
    buf[5] = 0x00;
    buf[6] = 0x02;
    buf[7] = 0x00;

    buf[10] = get_grd_status_info(buf+11,&len);

    len += 1;//data is ready len

    buf[6] = len;
    buf[7] = len >> 8;


    ret = data_check(&buf[10],len);

    buf[8] = (char)ret;
    buf[9] = (char)(ret >> 8);

    return len + 10;
}
    
uint8_t get_sky_mod_status_info(uint8_t *buf)
{
    int len;
    uint16_t ret;

    
    buf[0] = 0xff;
    buf[1] = 0x5a;
    buf[2] = 0x83;
    buf[3] = 0x00;
    buf[4] = 0x00;
    buf[5] = 0x00;
    buf[6] = 0x02;
    buf[7] = 0x00;

    buf[10] = get_sky_status_info(buf+11,&len);

    len += 1;//data is ready len

    buf[6] = len;
    buf[7] = len >> 8;


    ret = data_check(&buf[10],len);

    buf[8] = (char)ret;
    buf[9] = (char)(ret >> 8);

    return len + 10;
}

void cmd_execute(uint16 msg_id,uint8_t *pu8_rxBuf, uint16_t len)
{
    uint8_t buf[64],msg_len;
    
    switch(msg_id)
    {
        case 0x005b:
            DLOG_Warning("uart search id start");

            search_id(dev_type);

            set_link_led_status(LINK_SEARCH_ID);
            break;
        case 0x0082:
            msg_len = get_ground_mod_status_info(buf);
            osMutexWait(cmd_uart_mutex_id,0);
            HAL_UART_TxData(CMD_UART_NUM,buf,msg_len,1000);
            osMutexRelease(cmd_uart_mutex_id);

            break;
        case 0x0083:
            msg_len = get_sky_mod_status_info(buf);
            osMutexWait(cmd_uart_mutex_id,0);
            HAL_UART_TxData(CMD_UART_NUM,buf,msg_len,1000);
            osMutexRelease(cmd_uart_mutex_id);

            break;
        default:
            DLOG_Warning("undef cmd");
            break;
    }
}

static void cmd_mod_status_update_task(void const *argument)
{
    uint16_t update_time;
    uint8_t enable;
    uint8_t buf[64],len;

    update_time = get_fac_update_time(&enable);

    DLOG_Warning("cmd uart %d %d",enable,update_time);
    
    while (1)
    {
        if(!enable)
        {
           HAL_Delay(10000); 
           continue;
        }

        if(dev_type)
        {
            len = get_ground_mod_status_info(buf);
        }
        else
        {
            len = get_sky_mod_status_info(buf);
        }

        //print_msg(buf);

        osMutexWait(cmd_uart_mutex_id,0);
        HAL_UART_TxData(CMD_UART_NUM,buf,len,1000);
        osMutexRelease(cmd_uart_mutex_id);
        HAL_Delay(update_time);
    }
}

static bool parse_cmd_data(void)
{
	uint16 len;
	bool find_head;
	#define FRAME_MIN_LEN_THRESHOLD (10)
	uint8 buf[64];
	uint8 ret;
	
	len = BQGetDataSize();
	
	if( len < FRAME_MIN_LEN_THRESHOLD)
	{
		return FALSE;
	}
	
	BQPeekBytes(buf,FRAME_MIN_LEN_THRESHOLD);

	uint32_t offset=0;
	find_head = find_sync_head(buf,FRAME_MIN_LEN_THRESHOLD,&offset);

	if(!find_head)
	{
		BQPopBytes(buf,offset);
        DLOG_Warning("not find head");
		return FALSE;
	}

	if(offset > 0)
	{
		BQPopBytes(buf,offset);
		return FALSE;
	}

	uint16 msg_len,payload_len;

	payload_len = (buf[7] << 8 | buf[6]);
    msg_len =  payload_len  + FRAME_MIN_LEN_THRESHOLD;
	if(len < msg_len)
	{
		return FALSE;
	}

	BQPeekBytes(buf,msg_len);

	uint16 check;
	check = data_check(buf+FRAME_MIN_LEN_THRESHOLD,payload_len);
	if(check != (buf[9] << 8 | buf[8]))
	{
		BQPopBytes(buf,1);//to find next head
		DLOG_Warning("sum check error");
		return FALSE;
	}

    uint16 msg_id;
    msg_id = (buf[3] << 8 | buf[2]);

    cmd_execute(msg_id,buf+FRAME_MIN_LEN_THRESHOLD,payload_len);

	BQCommitLastPeek();
	
	return TRUE;

}

static void cmd_parse_task(void const *argument)
{
    bool isOk;

    while (1)
    {
        isOk = parse_cmd_data();
        if(isOk)
        {
            HAL_Delay(10);
        }
        else
        {
            HAL_Delay(50);
        }
    }
}

void usr_cmd_uart_task(uint8_t dev_type_class)
{
    HAL_RET_T ret;
    
    ENUM_HAL_UART_BAUDR enum_baud = get_fac_uart7_baudrate();
    dev_type = dev_type_class;

    ret = HAL_UART_Init(CMD_UART_NUM,enum_baud,hal_cmd_uart_irq_handler);
    if(ret != HAL_OK)
    {
        DLOG_Error("uart7 init failed");
        return;
    }

    osMutexDef(cmd_uart_mutex);
    cmd_uart_mutex_id = osMutexCreate(osMutex(cmd_uart_mutex));

    osThreadDef(MOD_STATUS_TASK, cmd_mod_status_update_task, osPriorityIdle, 0, 2 * 128);
    osThreadCreate(osThread(MOD_STATUS_TASK), NULL);

    osThreadDef(PARSE_CMD_TASK, cmd_parse_task, osPriorityIdle, 0, 2 * 128);
    osThreadCreate(osThread(PARSE_CMD_TASK), NULL);

}

