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
#include "factory.h"
#include "ringbuf_p201.h"

//extern osSemaphoreId uart_semaphore_id;

#define RS422_DEFAULT_BB_SESSION_PORT		BB_COM_SESSION_4

#define FCT_PARAM_ERROR						-1

static ENUM_HAL_UART_BAUDR default_baudr = HAL_UART_BAUDR_115200;

static ENUM_BB_COM_SESSION_ID GLOBAL_BB_SESSION = BB_COM_SESSION_4;

static ringbuf_t ring_hal_uart_rx = NULL;

static ringbuf_t ring_bb_uart_rx = NULL;

static int hal_uart_rx_cnt = 0, bb_uart_rx_cnt = 0;

static int hal_uart_tx_cnt = 0, bb_uart_tx_cnt = 0;

static int bb_dev_type;

static int check_rs422_factory_setting(void)
{
	int fct_param = -1;
	int param0 = 0, param1 = 0, param2 = 0;

    STRU_cfgNode *node;
    STRU_UART_BAUDR *p_uart_baudr = NULL;

	p_uart_baudr = FCT_GetNodeAndData(FACTORY_SUBNODE_UART_BAUDR_ID, NULL);
	if (p_uart_baudr != NULL)
	{
		param0 = p_uart_baudr->st_uartBaudr[0];
		param0 = (param0 & 0xF0) >> 4;

		param1 = p_uart_baudr->st_uartBaudr[1];
		param1 = (param1 & 0xF0) >> 4;

		fct_param = param2 = p_uart_baudr->st_uartBaudr[2];
		param2 = (param2 & 0xF0) >> 4;

		DLOG_Critical("Param0 %04x, Param1 %04x, Param2 %04x, FCT Param %04x\n", param0, param1, param2, fct_param);
		if ((param2 != 0) && ((param2 == param1) || (param2 == param0)))  		//Factory Setting Conflict
			return (fct_param & 0x0F);

		 if ((param2 == 0) && ((0 != param1) || (0 != param0)))
            return -2;

		return fct_param;
	}

	return -1;
}

static int get_factory_buadrate(void)
{
    int baudr = -1;
    STRU_cfgNode *node;
    STRU_UART_BAUDR *p_uart_baudr;

    p_uart_baudr = FCT_GetNodeAndData(FACTORY_SUBNODE_UART_BAUDR_ID, NULL);
    baudr = p_uart_baudr->st_uartBaudr[2];
    if (baudr >= HAL_UART_BAUDR_9600 && baudr <= HAL_UART_BAUDR_460800) {
        switch(baudr) {
            case HAL_UART_BAUDR_9600:
                DLOG_Critical("UART HAL_UART_BAUDR_9600\n");
                break;
            case HAL_UART_BAUDR_19200:
                DLOG_Critical("UART HAL_UART_BAUDR_19200\n");
                break;
            case HAL_UART_BAUDR_38400:
                DLOG_Critical("UART HAL_UART_BAUDR_38400\n");
                break;
            case HAL_UART_BAUDR_57600:
                DLOG_Critical("UART HAL_UART_BAUDR_57600\n");
                break;
            case HAL_UART_BAUDR_115200:
                DLOG_Critical("UART HAL_UART_BAUDR_115200\n");
                break;
            case HAL_UART_BAUDR_230400:
                DLOG_Critical("UART HAL_UART_BAUDR_230400\n");
                break;
            case HAL_UART_BAUDR_256000:
                DLOG_Critical("UART HAL_UART_BAUDR_256000\n");
                break;
            case HAL_UART_BAUDR_380400:
                DLOG_Critical("UART HAL_UART_BAUDR_380400\n");
                break;
            case HAL_UART_BAUDR_460800:
                DLOG_Critical("UART HAL_UART_BAUDR_460800\n");
                break;

        }

        return baudr;
    }

    return -1;
}

static void RS422_Data_Handler(void *p)
{
	#define BB_READ_MAX_LEN 200
    uint32_t cnt;
    uint8_t buffer[BB_READ_MAX_LEN];
    HAL_RET_T ret;

    ret = HAL_BB_ComReceiveMsg(GLOBAL_BB_SESSION, buffer, BB_READ_MAX_LEN, &cnt);
    if(ret != HAL_OK)
    {
        DLOG_Error("failed read bbcom");
        return;
    }

    if(cnt > 0 && cnt <= BB_READ_MAX_LEN)
    {
        ringbuf_memcpy_into(ring_bb_uart_rx, buffer, cnt);
        bb_uart_rx_cnt += cnt;
    }
    else
    {
        DLOG_Info("bb read %d",cnt);
    }
}

static uint32_t Uart6_Irq_Handler(uint8_t *pu8_rxBuf, uint8_t u8_len)
{
	ringbuf_memcpy_into(ring_hal_uart_rx, pu8_rxBuf, u8_len);
    hal_uart_rx_cnt += u8_len;
}

static void COMTASK_DebugFunction(void const *argument)
{
    while (1)
    {
        DLOG_Warning("RS422------> hal_uart_rx = %d",hal_uart_rx_cnt);
        DLOG_Warning("RS422------> bb_rx = %d",bb_uart_rx_cnt);
        DLOG_Warning("RS422------> hal_uart_tx = %d",hal_uart_tx_cnt);
        DLOG_Warning("RS422------> bb_tx = %d",bb_uart_tx_cnt);
        HAL_Delay(5000);
    }
}

static void hal_rs422_tx2bb_task(void const *argument)
{
    #define MAX_BB_UPLINK_SEND_SIZE 70
    #define MAX_BB_DOWNLINK_SEND_SIZE 200
    #define MAX(a,b) (((a) > (b)) ?  (a) :  (b) )
    HAL_RET_T ret;
    char buffer[MAX(MAX_BB_UPLINK_SEND_SIZE, MAX_BB_DOWNLINK_SEND_SIZE)];
    int len;
    int max_send_len;

    if(bb_dev_type)
    {
        max_send_len = MAX_BB_UPLINK_SEND_SIZE;
        DLOG_Warning("uplink pack len %d",max_send_len);

    }
    else
    {
        max_send_len = MAX_BB_DOWNLINK_SEND_SIZE;
    }

    while (1)
    {
        len = ringbuf_bytes_used(ring_hal_uart_rx);
        if(len == 0)
        {
            HAL_Delay(8);
            continue;
        }

        if(len > 1024)
        {
            DLOG_Error("len error %d,%08x,%08x", len, ringbuf_head(ring_hal_uart_rx), ringbuf_tail(ring_hal_uart_rx));
        }

        if(len > max_send_len)
        {
            len = max_send_len;
        }

        ringbuf_memcpy_from(buffer,ring_hal_uart_rx,len);
        bb_uart_tx_cnt += len;

//      osSemaphoreWait(uart_semaphore_id, 0);
        ret = HAL_BB_ComSendMsg(GLOBAL_BB_SESSION, buffer, len);
//      osSemaphoreRelease(uart_semaphore_id);
        if(ret != HAL_OK)
        {
            DLOG_Error("failed ret %d",ret);
        }

        HAL_Delay(4);
    }
}

static void bb_rs422_tx2hal_task(void const *argument)
{
    HAL_RET_T ret;
    char buffer[1024];
    int len;

    while (1)
    {
        len = ringbuf_bytes_used(ring_bb_uart_rx);
        if(len == 0)
        {
            HAL_Delay(14);
            continue;
        }

        if(len > 1024)
        {
            DLOG_Error("len error %d,%08x,%08x",len,ringbuf_head(ring_bb_uart_rx),ringbuf_tail(ring_bb_uart_rx));
            len = 1024;
        }

        ringbuf_memcpy_from(buffer,ring_bb_uart_rx,len);

        if (len > 0)
            hal_uart_tx_cnt += len;

        ret = HAL_UART_TxData(HAL_UART_COMPONENT_1, buffer, len, HAL_UART_DEFAULT_TIMEOUTMS * 1000);
        if(ret != HAL_OK)
        {
            DLOG_Error("failed ret %d",ret);
        }

        HAL_Delay(4);
    }
}

void Usr_Rs422_Task(int dev_type)
{
	int param = check_rs422_factory_setting();
	int rs422_session = 0;
	ENUM_HAL_UART_BAUDR rs422_baudr = default_baudr;

	bb_dev_type = dev_type;

	if (param < 0)
	{
		rs422_baudr = default_baudr;
		GLOBAL_BB_SESSION = RS422_DEFAULT_BB_SESSION_PORT;		
	}
	else
	{
		rs422_session = (param & 0xF0) >> 4;
		rs422_baudr = param & 0x0F;

		if (rs422_session == 0)
		{
			GLOBAL_BB_SESSION = RS422_DEFAULT_BB_SESSION_PORT;
		}
		else
		{
			rs422_session++;
			if (rs422_session < BB_COM_SESSION_2 || rs422_session > BB_COM_SESSION_4)
				GLOBAL_BB_SESSION = RS422_DEFAULT_BB_SESSION_PORT;
			else
				GLOBAL_BB_SESSION = rs422_session;
		}

		if (rs422_baudr < HAL_UART_BAUDR_9600 || rs422_baudr > HAL_UART_BAUDR_460800)
	        rs422_baudr = default_baudr;
	}

	DLOG_Critical("USR_RS422_Task (UART1) ENUM_HAL_UART_BAUDR is %d, ENUM_BB_COM_SESSION_ID is %d, GLOBAL_BB_SESSION is: %d \n", rs422_baudr, rs422_session, GLOBAL_BB_SESSION);

	ring_hal_uart_rx = ringbuf_new(4);
	if(!ring_hal_uart_rx)
    {
        DLOG_Critical("malloced failed");
    }

    ring_bb_uart_rx = ringbuf_new(5);
    if(!ring_bb_uart_rx)
    {
        DLOG_Critical("malloced failed");
    }

	if (rs422_baudr >= HAL_UART_BAUDR_9600 && rs422_baudr <= HAL_UART_BAUDR_460800)
        HAL_UART_Init(HAL_UART_COMPONENT_1, rs422_baudr, Uart6_Irq_Handler);

	HAL_BB_ComRegisterSession(GLOBAL_BB_SESSION,
                              BB_COM_SESSION_PRIORITY_LOW,
                              BB_COM_SESSION_DATA_NORMAL,
                              RS422_Data_Handler);

    osThreadDef(HAL_RS422_TX2BB_TASK, hal_rs422_tx2bb_task, osPriorityHigh, 0, 8 * 128);
    osThreadCreate(osThread(HAL_RS422_TX2BB_TASK), NULL);

    osThreadDef(BB_RS422_TX2HAL_TASK, bb_rs422_tx2hal_task, osPriorityHigh, 0, 8 * 128);
    osThreadCreate(osThread(BB_RS422_TX2HAL_TASK), NULL);

    osThreadDef(COMTASK_Debug, COMTASK_DebugFunction, osPriorityIdle, 0, 8 * 128);
    osThreadCreate(osThread(COMTASK_Debug), NULL);
}


