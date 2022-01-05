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
#include "ringbuf_p201.h"

//extern osSemaphoreId uart_semaphore_id;

static ENUM_BB_COM_SESSION_ID GLOBAL_BB_SESSION = BB_COM_SESSION_1;

static ringbuf_t ring_hal_uart_rx = NULL;

static ringbuf_t ring_bb_uart_rx = NULL;

static int hal_uart_rx_cnt = 0, bb_uart_rx_cnt = 0;

static int hal_uart_tx_cnt = 0, bb_uart_tx_cnt = 0;

static int bb_dev_type;

static void Rcv1_Data_Handler(void *p)
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

static uint32_t Uart1_Irq_Handler(uint8_t *pu8_rxBuf, uint8_t u8_len)
{
	ringbuf_memcpy_into(ring_hal_uart_rx, pu8_rxBuf, u8_len);
    hal_uart_rx_cnt += u8_len;
}

static void COMTASK_DebugFunction(void const *argument)
{
    while (1)
    {
        DLOG_Warning("SBUS------> hal_uart_rx = %d",hal_uart_rx_cnt);
        DLOG_Warning("SBUS------> bb_rx = %d",bb_uart_rx_cnt);
        DLOG_Warning("SBUS------> hal_uart_tx = %d",hal_uart_tx_cnt);
        DLOG_Warning("SBUS------> bb_tx = %d",bb_uart_tx_cnt);
        HAL_Delay(5000);
    }
}

static void hal_sbus_tx2bb_task(void const *argument)
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

static void bb_sbus_tx2hal_task(void const *argument)
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

        ret = HAL_UART_TxData(HAL_UART_COMPONENT_7, buffer, len, HAL_UART_DEFAULT_TIMEOUTMS * 1000);
        if(ret != HAL_OK)
        {
            DLOG_Error("failed ret %d",ret);
        }

        HAL_Delay(4);
    }
}

static void Uart1_Init(void)
{
    HAL_SBUS_Init(HAL_UART_COMPONENT_7, 100000, Uart1_Irq_Handler);
}

void Usr_SBUS_Task(int dev_type)
{
	bb_dev_type = dev_type;
	ring_hal_uart_rx = ringbuf_new(6);
    if(!ring_hal_uart_rx)
    {
        DLOG_Critical("malloced failed");
    }

    ring_bb_uart_rx = ringbuf_new(7);
    if(!ring_bb_uart_rx)
    {
        DLOG_Critical("malloced failed");
    }

	HAL_SBUS_Init(HAL_UART_COMPONENT_7, 100000, Uart1_Irq_Handler);

	HAL_BB_ComRegisterSession(GLOBAL_BB_SESSION,
                              BB_COM_SESSION_PRIORITY_LOW,
                              BB_COM_SESSION_DATA_NORMAL,
                              Rcv1_Data_Handler);

	osThreadDef(HAL_SBUS_TX2BB_TASK, hal_sbus_tx2bb_task, osPriorityHigh, 0, 8 * 128);
    osThreadCreate(osThread(HAL_SBUS_TX2BB_TASK), NULL);

    osThreadDef(BB_SBUS_TX2HAL_TASK, bb_sbus_tx2hal_task, osPriorityHigh, 0, 8 * 128);
    osThreadCreate(osThread(BB_SBUS_TX2HAL_TASK), NULL);

    osThreadDef(COMTASK_Debug, COMTASK_DebugFunction, osPriorityIdle, 0, 8 * 128);
    osThreadCreate(osThread(COMTASK_Debug), NULL);

}


