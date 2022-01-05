/*****************************************************************************
Copyright: 2016-2020, Artosyn. Co., Ltd.
File name: jnc_hal_sys.c
Description: 
Author: Artosyn Software Team
Version: 0.0.1
Date: 2017/12/04
History: 
        0.0.1    2017/12/04    The initial version of hal.c
*****************************************************************************/
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include "systicks.h"
#include "hal_ret_type.h"
#include "hal.h"
//#include "dma.h"
#include "jnc_hal_sys_cpu0.h"
#include "hal_sys_ctl.h"
#include "hal_nv.h"
#include "memory_config.h"
#include "debuglog.h"
#include "bb_types.h"
#include "hal_bb.h"
//#include "bb_ctrl_internal.h"
#include "sys_event.h"
#include "ff.h"
#include "hal_usb_host.h"


#define USR_NV_SRAM_ADDR     (SRAM_USR_NV_MEMORY_ST_ADDR)
#define USR_NV_MAX_LEN       (SRAM_USR_NV_MEMORY_SIZE)

static DIR      g_udisk_dir;
static FIL      g_log_file;
static uint8_t  g_log_file_exist = 0;
static uint8_t  g_log_file_name[LOG_FILE_NAME_MAX_LENGTH + 1];

static void ss_log_write_handler(void *p);
static void ss_log_set_name(uint8_t *file_name);

static uint32_t NV_UsrDataUnitWrite(uint32_t max)
{
    static uint32_t step = 0;
    static uint32_t write_cnt = 0;
    uint32_t cnt = 0;
    uint32_t loop = 0;
    uint32_t u_size = sizeof(STRU_USR_NV_UNIT);
    uint32_t max_unit = USR_NV_MAX_LEN / u_size;
    STRU_USR_NV_UNIT *p_unit;

    while (loop < max_unit)
    {
        p_unit = (STRU_USR_NV_UNIT *)(USR_NV_SRAM_ADDR + step * u_size);
        if (1 == (p_unit->valid))
        {
            HAL_NV_UsrDataWrite(p_unit->addr, p_unit->data, p_unit->len);
            write_cnt += 1;
            DLOG_Critical("write_cnt %d", write_cnt);
            p_unit->valid = 0;
            cnt += 1;
            if (cnt >= max)
            {
                step += 1;
                step %= max_unit;
                break;
            }
        }
        step += 1;
        step %= max_unit;
        loop += 1;
    }

    return cnt;
}

void HAL_NV_TaskWriteData(void *p)
{
    NV_UsrDataUnitWrite(1);
}


static void ss_log_write_handler(void *p)
{
    STRU_SysEvent_FLUSH_UDISK_BUFFER    *pst_flush_udisk_buffer;
    void                                *write_buffer;
    uint32_t                             write_length;
    static uint8_t                       msc_port_id;
    FRESULT                              f_result;
    uint32_t                             write_real;
    static uint32_t                      data_index_record = 0;
    static uint8_t                       timeout = 0;

    pst_flush_udisk_buffer = (STRU_SysEvent_FLUSH_UDISK_BUFFER *)p;

    write_buffer = (void *)(pst_flush_udisk_buffer->data_address + 0x24180000);
    write_length = pst_flush_udisk_buffer->data_length;

    if ((write_buffer == NULL)||(write_length == 0))
    {
        DLOG_Error("NULL pointer or 0 length to write: 0x%08x, %d", write_buffer, write_length);

        return;
    }

    if ((pst_flush_udisk_buffer->data_index - data_index_record) != 1)
    {
        DLOG_Error("data not continue: %d, %d", data_index_record, pst_flush_udisk_buffer->data_index);

        return;
    }

    data_index_record = pst_flush_udisk_buffer->data_index;

    if (g_log_file_exist)
    {
        f_write(&g_log_file, write_buffer, write_length, (void *)&write_real);

        if (f_size(&g_log_file) >= LOG_FILE_MAX_SIZE)
        {
            f_close(&g_log_file);

            g_log_file_exist = 0;
        }
        else
        {
            f_sync(&g_log_file);
        }
    }
    else
    {
        msc_port_id = HAL_USB_GetMSCPort();

        while (HAL_USB_GetHostAppState(msc_port_id) != HAL_USB_HOST_STATE_READY)
        {
            HAL_Delay(10);

            timeout++;

            if (timeout >= 50)
            {
                DLOG_Error("udisk-sd not connect");
                return;
            }
        }

        ss_log_set_name(g_log_file_name);

        f_result = f_open(&g_log_file, g_log_file_name, FA_CREATE_NEW | FA_WRITE);

        if (FR_OK == f_result)
        {
            g_log_file_exist = 1;

            f_write(&g_log_file, write_buffer, write_length, (void *)&write_real);

            f_sync(&g_log_file);
        }
    }

}


void ss_log_write_init(void)
{
    memset(g_log_file_name, '-', LOG_FILE_NAME_MAX_LENGTH);
    memcpy(g_log_file_name, LOG_FILE_NAME_PREFIX, (sizeof(LOG_FILE_NAME_PREFIX) - 1));

    SYS_EVENT_RegisterHandler(SYS_EVENT_ID_FLUSH_UDISK_BUFFER, ss_log_write_handler);
}


static void ss_log_set_name(uint8_t *file_name)
{
    int32_t         max_number = 0;
    int32_t         current_number;
    FRESULT         f_result = FR_OK;
    FILINFO         f_info;
    char            file_number_part[LOG_FILE_NAME_NUMBER_LENGTH];

    f_result = f_opendir(&g_udisk_dir, "0:/");

    if (f_result != FR_OK)
    {
        DLOG_Error("open root dir fail");
        return;
    }

    f_result = f_readdir(&g_udisk_dir, &f_info);

    /* check the existed log files, find the last log file */
    while ((f_result == FR_OK)&&(0 != f_info.fname[0]))
    {
        if (0 == strncmp(file_name + 2, f_info.fname, (sizeof(LOG_FILE_NAME_PREFIX) - 3)))
        {
            memcpy(file_number_part,
                   (f_info.fname + sizeof(LOG_FILE_NAME_PREFIX) - 1),
                   LOG_FILE_NAME_NUMBER_LENGTH);

            current_number = atoi(file_number_part);

            if (max_number <= current_number)
            {
                max_number = current_number;
            }
        }

        f_result = f_readdir(&g_udisk_dir, &f_info);
    }

    /* add one number to the file name */
    max_number++;

    sprintf(&file_name[sizeof(LOG_FILE_NAME_PREFIX) - 1], "%08d", max_number);

    /* add surfix of the file name */
    memcpy(g_log_file_name + (sizeof(LOG_FILE_NAME_PREFIX) - 1) + LOG_FILE_NAME_NUMBER_LENGTH,
           LOG_FILE_NAME_SURFIX,
           (sizeof(LOG_FILE_NAME_SURFIX) - 1));

    f_closedir(&g_udisk_dir);

    return;
}




