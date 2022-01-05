/*****************************************************************************
Copyright: 2016-2020, Artosyn. Co., Ltd.
File name: hal_rots.h
Description: The external HAL APIs to use the SDMMC controller.
Author: Wumin @Artosyn Software Team
Version: 0.1.1
Date: 2017/4/5
History: 
        0.1.1    2017/4/5    The initial version of ar_freertos_specific.c
                             added top command
*****************************************************************************/

#include <stdint.h>
#include <string.h>
#include "ar_freertos_specific.h"
#include "hal_timer.h"
#include "FreeRTOSConfig.h"
#include "portmacro.h"
#include "FreeRTOS.h"
#include "task.h"
#include "debuglog.h"
#include "hal_ret_type.h"
#include "cmsis_os.h"
#include "systicks.h"

#define LOCAL_MAX_TASK_NUM 20
#define LOCAL_HAL_TIMER 21
#define LOCAL_HAL_TIMER_INTERVEL 100

static osMessageQId s_sys_event_msg_queue_id = 0;
static osMessageQId s_usbh_event_msg_queue_id = 0;

uint32_t g_rtos_feature_task_traceability_cnt = 0;

static void rtos_feature_task_traceability_handler(void)
{
    g_rtos_feature_task_traceability_cnt++;
    // DLOG_Info("running...\n");
}

void rtos_feature_task_traceability_init(void)
{
    HAL_TIMER_RegisterTimer(LOCAL_HAL_TIMER, LOCAL_HAL_TIMER_INTERVEL, rtos_feature_task_traceability_handler);
    // DLOG_Info("rtos_feature_task_traceability_init called\n");
}

void ar_top(void) 
{
    const char task_state[]={'r','R','B','S','D'};  
    volatile UBaseType_t uxArraySize, x;
    uint32_t ulTotalRunTime;
    unsigned int ulStatsAsPercentage;
    TaskStatus_t pxTaskStatusArray[LOCAL_MAX_TASK_NUM];

    /* require the total tasks */
    uxArraySize = uxTaskGetNumberOfTasks();  
    
    if(uxArraySize>LOCAL_MAX_TASK_NUM)  
    {  
        DLOG_Warning("too many tasks\n");
        return;
    }
    /* require each task struct information */
    uxArraySize = uxTaskGetSystemState(pxTaskStatusArray, uxArraySize, &ulTotalRunTime);  

    DLOG_Info("\n");
    DLOG_Info("name                            status ID Priority   stack   used");

    if( ulTotalRunTime > 0 )  
    {  
        for( x = 0; x < uxArraySize; x++ )  
        {  
            char tmp[128];  
             
            /* caculate the total run time and total percent time */
            ulStatsAsPercentage =(uint64_t)(pxTaskStatusArray[x].ulRunTimeCounter)*100 / 
            ulTotalRunTime;  
   
            if( ulStatsAsPercentage > 0UL )  
            {
                sprintf(tmp,"%-32s%-7c%-6d%-8d%-8d%d%%",pxTaskStatusArray[x].pcTaskName,
                    task_state[pxTaskStatusArray[x].eCurrentState],  
                    pxTaskStatusArray[x].xTaskNumber,pxTaskStatusArray[x].uxCurrentPriority,  
                    pxTaskStatusArray[x].usStackHighWaterMark,ulStatsAsPercentage);  
            }  
            else  
            {  
                /* cpu used not more than 1 */
                sprintf(tmp,"%-32s%-7c%-6d%-8d%-8dt<1%%",pxTaskStatusArray[x].pcTaskName,
                    task_state[pxTaskStatusArray[x].eCurrentState],  
                    pxTaskStatusArray[x].xTaskNumber,pxTaskStatusArray[x].uxCurrentPriority,
                    pxTaskStatusArray[x].usStackHighWaterMark); 
            }
            DLOG_Info("%s",tmp);  
        }  
    }  
    return;
}

void ar_osDelay(uint32_t u32_ms)
{
    if (osKernelRunning())
    {
        osDelay(u32_ms);
    }
    else
    {
        SysTicks_DelayMS(u32_ms);
    }
}

static void createSysEventMsgQ(void)
{
    if (s_sys_event_msg_queue_id == 0)
    {
        osMessageQDef(sys_event_msg_queue, 64, uint8_t);
        s_sys_event_msg_queue_id  = osMessageCreate(osMessageQ(sys_event_msg_queue), NULL);
    }
}

static void getSysEventMsgQ(void)
{
    if (s_sys_event_msg_queue_id != 0)
    {
        osMessageGet(s_sys_event_msg_queue_id, 50);
    }
}

static void putSysEventMsgQ(void)
{
    if (s_sys_event_msg_queue_id != 0)
    {
        uint8_t u8_Data;
        osMessagePut(s_sys_event_msg_queue_id, u8_Data, 0);
    }
}

void ar_osSysEventMsgQGet(void)
{
    if (osKernelRunning())
    {
        createSysEventMsgQ();

        getSysEventMsgQ();
    }
}

void ar_osSysEventMsgQPut(void)
{
    if (osKernelRunning())
    {
        createSysEventMsgQ();

        putSysEventMsgQ();
    }
}

static void createUSBHEventMsgQ(void)
{
    if (s_usbh_event_msg_queue_id == 0)
    {
        osMessageQDef(usbh_event_msg_queue, 20, uint32_t);
        s_usbh_event_msg_queue_id  = osMessageCreate(osMessageQ(usbh_event_msg_queue), NULL);
    }
}

static void getUSBHEventMsgQ(void *p)
{
    osEvent event;

    if (s_usbh_event_msg_queue_id != 0)
    {
        event = osMessageGet(s_usbh_event_msg_queue_id, osWaitForever);

        memcpy(p, (void *)event.value.v, sizeof(uint32_t));
    }
}

static void putUSBHEventMsgQ(void *p)
{
    if (s_usbh_event_msg_queue_id != 0)
    {
        osMessagePut(s_usbh_event_msg_queue_id, (uint32_t)p, 0);
    }
}

void ar_osUSBHEventMsgQGet(void *p)
{
    if (osKernelRunning())
    {
        createUSBHEventMsgQ();

        getUSBHEventMsgQ(p);
    }
}

void ar_osUSBHEventMsgQPut(void *p)
{
    if (osKernelRunning())
    {
        createUSBHEventMsgQ();

        putUSBHEventMsgQ(p);
    }
}


void ar_osWirelessTaskInit(void TaskHandler(void const *argument))
{
    osThreadDef(WIRELESS_TASK, TaskHandler, osPriorityNormal, 0, 4 * 128);
    osThreadCreate(osThread(WIRELESS_TASK), NULL);
}

#if configCHECK_FOR_STACK_OVERFLOW > 0
void vApplicationStackOverflowHook( TaskHandle_t xTask, char *pcTaskName )
{
    DLOG_Info("%s stack overflow!\n", pcTaskName);
   
}
#endif
