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
#include <string.h>
#include <stdint.h>
#include "systicks.h"
#include "hal_ret_type.h"
#include "hal.h"
//#include "dma.h"
#include "jnc_hal_sys.h"
#include "hal_sys_ctl.h"
#include "hal_nv.h"
#include "memory_config.h"
#include "debuglog.h"
#include "bb_types.h"
#include "hal_bb.h"
//#include "bb_ctrl_internal.h"
#include "hal_usb_host.h"
#include "sys_event.h"
#include "lock.h"

#include "hal_uart.h"
#include "cmd_line.h"
#include "test_hal_uart.h"


#define USR_NV_SRAM_ADDR     (SRAM_USR_NV_MEMORY_ST_ADDR)
#define USR_NV_MAX_LEN       (SRAM_USR_NV_MEMORY_SIZE)

#define RC_CHANNEL_CNT          (8)
#define RC_MSG_HEAD_ID          (0x5A)

static uint8_t lock_state = 0; 

static uint16_t rc_value_ppm[RC_CHANNEL_CNT] = {0};

#define RC_SESSION_DATA     (1024)




#define DATA_LOG_FILE_NAME_LENGTH       128
#define DATA_LOG_BUFFER_NUMBER          4
#define DATA_LOG_BUFFER_LENGTH          4096

/* AUTO Flush cache every 100ms */
#define DATA_LOG_BUFFER_FLUSH_PERIOD    100



typedef struct _session_ringbuf
{
    uint8_t     data[RC_SESSION_DATA];
    uint16_t    r_pos;          //r_pos: data read position
    uint16_t    w_pos;          //w_pos: data write position
    uint32_t    data_len;       //available data length
} session_ringbuf_t;

typedef struct _rc_status_ppm
{
    uint8_t  head;
    uint8_t  rc_index;
    uint16_t value_ppm;
} rc_status_ppm_t;


static session_ringbuf_t rb_session2 = {0};
static session_ringbuf_t rb_session3 = {0};
static uint cpu0_in_upgrade = 0;

SS_E_Inf error_info[] = 
{
    "telem_unconnected",
    "failed",
};


static uint8_t      g_data_log_buffer[DATA_LOG_BUFFER_NUMBER][DATA_LOG_BUFFER_LENGTH];
static uint8_t      g_data_log_write_index = 0;
static uint16_t     g_data_log_buffer_pos = 0;
static uint32_t     g_data_log_tick_record = 0;
static uint32_t     g_data_log_lock;


static void ss_hal_flush_udisk(void *data_buffer, uint16_t data_size);
static void ss_hal_datalog_autosave(void *p);
static void ss_rc_com_handler( void * param );
static void ss_telem_session2_rcv_handler( void * param );
static void ss_telem_session3_rcv_handler( void * param );
static uint16_t ss_get_rb_data_len(session_ringbuf_t *rb);
static uint16_t ss_get_rb_space_len(session_ringbuf_t *rb);
static uint16_t ss_rb_write(session_ringbuf_t *rb, uint8_t *data, uint16_t len);
static uint16_t ss_rb_read(session_ringbuf_t *rb, uint8_t *data, uint16_t len);
static uint16_t ss_get_rb_data_len(session_ringbuf_t *rb);
static uint16_t ss_get_rb_space_len(session_ringbuf_t *rb);
static uint16_t ss_rb_write(session_ringbuf_t *rb, uint8_t *data, uint16_t len);
static uint16_t ss_rb_read(session_ringbuf_t *rb, uint8_t *data, uint16_t len);
static uint32_t NV_UsrDataUnitWrite(uint32_t max);
static void cpu0_upgrade_handler(void * arg);


/**
* @brief    all board-system init here
* @param    none.
* @retval   error information/SS_SUCCESS
* @note     none.
*       
*/
SS_E_Inf ss_hal_board_init()
{
    HAL_SYS_CTL_Init(NULL);
    DLOG_Init(CMD_exec_cmd, NULL, DLOG_CLIENT_PROCESSOR);
    dlog_set_output_level(LOG_LEVEL_CRITICAL);
    DLOG_Critical("cpu1 start!!! \n");    


    HAL_BB_ComRegisterSession(BB_COM_SESSION_1, BB_COM_SESSION_PRIORITY_HIGH, BB_COM_SESSION_DATA_NORMAL, ss_rc_com_handler);
    HAL_BB_ComRegisterSession(BB_COM_SESSION_2, BB_COM_SESSION_PRIORITY_HIGH, BB_COM_SESSION_DATA_NORMAL, ss_telem_session2_rcv_handler);
    HAL_BB_ComRegisterSession(BB_COM_SESSION_3, BB_COM_SESSION_PRIORITY_HIGH, BB_COM_SESSION_DATA_NORMAL, ss_telem_session3_rcv_handler);

	SYS_EVENT_RegisterHandler(SYS_EVENT_ID_CPU0_INTO_UPGRADE, cpu0_upgrade_handler);
    return SS_SUCCESS;
}

/**
* @brief    do nothing
* @param    
* @retval   
* @note     
*       
*/
void ss_hal_board_deinit()
{

}

/**
* @brief  do nothing
* @param  
* @retval 
* @note   
*       
*/
SS_E_Inf ss_hal_create_task( const char* name, void (*func)(void*), uint8_t priority )
{
    return NULL;
}

/**
* @brief  do nothing
* @param  
* @retval 
* @note   
*       
*/
SS_E_Inf ss_hal_start_task()
{
    return NULL;
}

/**
* @brief  get us since app started
* @param  none
* @retval us since app started
* @note   none
*       
*/
uint64_t ss_hal_micros()
{
    return HAL_GetSysUsTick();
}

/**
* @brief  delay current thread/task
* @param  ms: the delay time value in millisecond unit.  
* @retval none
* @note   none
*       
*/
void ss_hal_delay( uint16_t ms )
{
    HAL_Delay((uint32_t)ms);
}

/**
* @brief  delay current thread/task
* @param  us: the delay time value in microseconds unit.  
* @retval none
* @note   none
*       
*/
void ss_hal_delay_micros( uint16_t us )
{
    SysTicks_DelayUS((uint64_t)us);
}

/**
* @brief  delay current thread/task
* @param  us: the delay time value in microseconds unit.  
* @retval none
* @note   none
*       
*/
void ss_hal_delay_micros_boost( uint16_t us )
{ 
    return ss_hal_delay_micros(us); 
}

/**
* @brief
* @param  
* @retval 
* @note   
*       
*/
void ss_hal_idle(uint16_t us)
{
    uint64_t start;
    uint64_t end;

    start = HAL_GetSysUsTick();
    
    SYS_EVENT_Process();

    end = HAL_GetSysUsTick();
    if ((end - start) > (uint64_t)us)
    {
        DLOG_Critical("timer out");
    }

    DLOG_Process(NULL);
}

/**
* @brief  do nothing
* @param  
* @retval 
* @note   
*       
*/
void ss_hal_reboot( bool bl)
{

}


/**
* @brief  init flash for persistant data storage
* @param  none
* @retval 
* @note   error information/SS_SUCCESS
*       
*/
SS_E_Inf ss_hal_data_persist_init()
{
    return SS_SUCCESS;
}


static void cpu0_upgrade_handler(void * arg)
{
	cpu0_in_upgrade = 1;
}



/**
* @brief  read a data block
* @param  dst:    destination buffer pointer
          offset: read from offset since storage start addr.
          size:   data block size in bytes

* @retval ok:true, or false
* @note   none
*       
*/
bool ss_hal_data_persist_read( uint8_t* dst, uint32_t offset, uint32_t size )
{
    if (size > 16000 || cpu0_in_upgrade == 1)
    {
        return false;
    }

    if (HAL_OK == HAL_NV_UsrDataRead(offset, dst, size))
    {
        return true;
    }
    else
    {
        return false;
    }
}





/**
* @brief  write a data block
* @param  src    source buffer pointer
          offset where to write since storage start address
          size   in bytes
* @retval ok true, or false
* @note   
*       
*/
bool ss_hal_data_persist_write( const uint8_t* src, uint32_t offset, uint32_t size )
{
#if 0
    uint32_t i = 0;
    uint32_t u_size = sizeof(STRU_USR_NV_UNIT);
    uint32_t max_unit = USR_NV_MAX_LEN / u_size;
    STRU_USR_NV_UNIT *p_unit;

    for(i = 0; i < max_unit; i++)
    {
        p_unit = (STRU_USR_NV_UNIT *)(USR_NV_SRAM_ADDR + i * u_size);
        if (0 == (p_unit->valid))
        {
            break;
        }
    }

    if (i == max_unit)
    {
        DLOG_Warning("bufer full !");
        return false;
    }
    else
    {
        if (size > USR_NV_UNIT_MAX_DATA_LEN)
        {
            DLOG_Warning("size over limit %d", USR_NV_UNIT_MAX_DATA_LEN);
            return false;
        }
        p_unit->addr = offset;
        p_unit->len = size;
        memcpy(p_unit->data, src, p_unit->len); 
        p_unit->valid = 1;
    }
#endif

	if (cpu0_in_upgrade == 1)
	{
		return false;
	}

    if (HAL_OK == HAL_NV_UsrDataWrite(offset, (uint8_t *)src, size))
    {
        return true;
    }

    return false;
}

static uint16_t ss_get_rb_data_len(session_ringbuf_t *rb)
{
    if (rb->w_pos >= rb->r_pos)
    {
        return (rb->w_pos - rb->r_pos);
    }
    else
    {
        return (rb->w_pos + RC_SESSION_DATA - rb->r_pos);
    }
}

static uint16_t ss_get_rb_space_len(session_ringbuf_t *rb)
{
    if (rb->w_pos >= rb->r_pos)
    {
        return (rb->r_pos + RC_SESSION_DATA - rb->w_pos);
    }
    else
    {
        return (rb->r_pos - rb->w_pos);
    }
}

static uint16_t ss_rb_write(session_ringbuf_t *rb, uint8_t *data, uint16_t len)
{
    uint16_t i = 0;

    while(i < len)
    {
        rb->data[rb->w_pos] = data[i];
        rb->w_pos += 1;
        rb->w_pos %= RC_SESSION_DATA;
        i += 1;
    }
}

static uint16_t ss_rb_read(session_ringbuf_t *rb, uint8_t *data, uint16_t len)
{
    uint16_t i = 0;

    while(i < len)
    {
        data[i] = rb->data[rb->r_pos];
        rb->r_pos += 1;
        rb->r_pos %= RC_SESSION_DATA;
        i += 1;
    }
}

/**
* @brief  init rc control
* @param  none
* @retval 0 if ok, or return error text
* @note   none
*       
*/
SS_E_Inf ss_hal_rc_init()
{
    return 0;
}

/**
* @brief  write remote controller info
* @param  index: channel
          value: rc ppm value
* @retval 0 if ok, or return error text
* @note   none
*       
*/
SS_E_Inf ss_hal_rc_write( uint8_t index, uint16_t value )
{
    rc_status_ppm_t rc = 
    {
        .head       = RC_MSG_HEAD_ID,
        .rc_index   = index,
        .value_ppm  = value
    };

    if (HAL_OK == HAL_BB_ComSendMsg(BB_COM_SESSION_1, (uint8_t *)&rc, sizeof(rc_status_ppm_t)))
    {
        return 0;
    }
    else
    {
        return error_info[1];
    }
}


static void ss_rc_com_handler( void * param )
{
    HAL_RET_T     ret;
    rc_status_ppm_t rc;
    uint32_t      data_size = 0;

    ret = HAL_BB_ComReceiveMsg(BB_COM_SESSION_1, (uint8_t *)&rc, sizeof(rc_status_ppm_t), &data_size);
    if ((ret == HAL_OK) && (data_size > 0))
    {
        if (data_size >= sizeof(rc_status_ppm_t))
        {
            if (rc.head == RC_MSG_HEAD_ID && rc.rc_index < RC_CHANNEL_CNT)
            {
                rc_value_ppm[rc.rc_index] = rc.value_ppm;
            }
            else
            {
                //should not come here
                uint8_t   data[128];
                HAL_BB_ComReceiveMsg(BB_COM_SESSION_1, data, sizeof(data), &data_size);
                DLOG_Error("drop rc session Error data size=%d", data_size);
            }
        }
    }
}

/**
* @brief  read remote controller info
* @param  index: channel
* @retval rc ppm value
* @note   none
*       
*/
uint16_t ss_hal_rc_read( uint8_t index )
{
    return rc_value_ppm[index];
}

/**
* @brief  call in sky return rc lock status, call in grd return vt lock status.
* @param  none
* @retval lock status
* @note   none
*       
*/
int16_t ss_hal_rc_connected()
{
#define DEVICE_INFO_SHM_SIZE             (sizeof(STRU_DEVICE_INFO))
#define DEVICE_INFO_SHM_ADDR             ((SRAM_BB_STATUS_SHARE_MEMORY_ST_ADDR + SRAM_BB_STATUS_SHARE_MEMORY_SIZE) - DEVICE_INFO_SHM_SIZE)

    STRU_WIRELESS_INFO_DISPLAY *osdptr = (STRU_WIRELESS_INFO_DISPLAY *)(SRAM_BB_STATUS_SHARE_MEMORY_ST_ADDR);
    STRU_DEVICE_INFO *pst_devInfo = (STRU_DEVICE_INFO *)(DEVICE_INFO_SHM_ADDR);

    if (BB_SKY_MODE == (pst_devInfo->skyGround))
    {
        lock_state = osdptr->u8_rclock;
    }
    else
    {
        lock_state = osdptr->lock_status;
    }

    return lock_state;
}

/**
* @brief  telem-data tranfer init.
* @param  none
* @retval 0 if success, or return error text
* @note   none
*       
*/
SS_E_Inf ss_hal_telem_init()
{
    uint32_t start_ms;
    int16_t  lock = 0;

    memset((uint8_t *)(&rb_session2), 0x00, sizeof(session_ringbuf_t));
    memset((uint8_t *)(&rb_session3), 0x00, sizeof(session_ringbuf_t));

    start_ms = HAL_GetSysMsTick();

    while ( HAL_GetSysMsTick() - start_ms < 5000)
    {
        lock = (ss_hal_rc_connected() > 0) ? 1: 0;
        if (lock)
        {
            break;
        }
        else
        {
            HAL_Delay(200);
        }
    }

    //return (lock ? SS_SUCCESS:error_info[0]);
    return SS_SUCCESS;
}


static void ss_telem_session_rcv_handler(session_ringbuf_t *rb, ENUM_BB_COM_SESSION_ID session)
{
    uint8_t buffer[RC_SESSION_DATA];
    uint32_t sapce_len = ss_get_rb_space_len(rb);
    uint32_t avail_len = 0;
    uint32_t i= 0;

    if (sapce_len > 0)
    {
        //DLOG_Critical("sapce_len %d", sapce_len);
        HAL_RET_T ret = HAL_BB_ComReceiveMsg(session, buffer, sapce_len, &avail_len);
        //DLOG_Critical("%d %d", ret, avail_len);
        if (HAL_OK == ret && avail_len > 0)
        {
            ss_rb_write(rb, buffer, avail_len);
        }
    }
}


static void ss_telem_session2_rcv_handler( void *param )
{
    ss_telem_session_rcv_handler(&rb_session2, BB_COM_SESSION_2);
}


static void ss_telem_session3_rcv_handler( void *param )
{
    ss_telem_session_rcv_handler(&rb_session3, BB_COM_SESSION_3);
}

/**
* @brief  read telem data.
* @param  channel:  2 or 3
          dst: read data buffer
          max: read max num in bytes
* @retval real read data length
* @note   none
*       
*/
int16_t ss_hal_telem_read(uint8_t channel, uint8_t* dst, uint16_t max)
{
    uint16_t copy_len = 0;

    // re-map channel
    if(channel == 0){
        channel = BB_COM_SESSION_2;
    }else if(channel == 1){
        channel = BB_COM_SESSION_3;
    }else{
        return 0;
    }

    if (!lock_state)
    {
        DLOG_Critical("unlock status");
        return 0;
    }
    
    if (channel != BB_COM_SESSION_2 && channel != BB_COM_SESSION_3)
    {
        DLOG_Error("Error channel=%d", channel);
        return 0;
    }

    if (channel == BB_COM_SESSION_2)
    {
        copy_len =  ss_get_rb_data_len(&rb_session2);
        copy_len = (copy_len < max) ? copy_len : max;
        ss_rb_read(&rb_session2, dst, copy_len);
    }
    else if(channel == BB_COM_SESSION_3)
    {
        copy_len =  ss_get_rb_data_len(&rb_session3);
        copy_len = (copy_len < max) ? copy_len : max;
        ss_rb_read(&rb_session3, dst, copy_len);
    }
    else
    {
        return -1;
    }

    return copy_len;
}

/**
* @brief  write telem data.
* @param  channel:  2 or 3
          src: write data buffer
          lenght: write data length
* @retval actual write data length
* @note   none
*       
*/
int16_t ss_hal_telem_write(uint8_t channel, const uint8_t* src, uint16_t length)
{
    // re-map channel
    if(channel == 0){
        channel = BB_COM_SESSION_2;
    }else if(channel == 1){
        channel = BB_COM_SESSION_3;
    }else{
        return 0;
    }

    
    HAL_RET_T ret = -1;

    if (0 == lock_state)
    {
        DLOG_Critical("unlock status");
        return 0;
    }

    if (0 == length)
    {
        DLOG_Critical("length : 0");
        return 0;
    }

    if (channel == BB_COM_SESSION_2)
    {
        ret = HAL_BB_ComSendMsg(BB_COM_SESSION_2, (uint8_t*)src, length);
        if (ret == HAL_OK)
        {
            return length;
        }
        else
        {
            return 0;
        }
    }
    else if (channel == BB_COM_SESSION_3)
    {
        ret = HAL_BB_ComSendMsg(BB_COM_SESSION_3, (uint8_t*)src, length);
        if (ret == HAL_OK)
        {
            return length;
        }
        else
        {
            return 0;
        }
    }
    else
    {
        return -1;
    }
}

/**
* @brief  call in sky return rc lock status(0: unlock, 1~100 locked), 
            call in grd return vt lock status.
* @param  none
* @retval lock status
* @note   none
*       
*/
int16_t ss_hal_telem_connected()
{
    return ss_hal_rc_connected();
}


/**
* @brief  init the log save function in CPU1
* @param  none
* @retval none
* @note   none
*       
*/
SS_E_Inf ss_hal_datalog_init(void)
{
    g_data_log_write_index = 0;
    g_data_log_buffer_pos  = 0;

    SYS_EVENT_RegisterHandler(SYS_EVENT_ID_IDLE, ss_hal_datalog_autosave);

    return SS_SUCCESS;
}


/**
* @brief  no use this api
* @param  none
* @retval none
* @note   none
*       
*/
uint32_t ss_hal_datalog_start(void)
{
    
}


/**
* @brief    write log into SD buffer in CPU1
* @param  none
* @retval none
* @note   none
*       
*/
void ss_hal_datalog_write( const void* pBuffer, uint16_t size )
{
    uint8_t                 *data_buffer;
    uint16_t                 data_size;

    data_buffer   = (uint8_t *)pBuffer;
    data_size     = size;

    Lock(&g_data_log_lock);

    while (data_size > 0)
    {
        if (data_size >= (DATA_LOG_BUFFER_LENGTH - g_data_log_buffer_pos))
        {
            memcpy((void *)&g_data_log_buffer[g_data_log_write_index][g_data_log_buffer_pos],
                   (void *)data_buffer,
                   (DATA_LOG_BUFFER_LENGTH - g_data_log_buffer_pos));

            data_size   -= (DATA_LOG_BUFFER_LENGTH - g_data_log_buffer_pos);
            data_buffer += (DATA_LOG_BUFFER_LENGTH - g_data_log_buffer_pos);

            /* Notify to cpu0 */
            ss_hal_flush_udisk((void *)g_data_log_buffer[g_data_log_write_index],
                                DATA_LOG_BUFFER_LENGTH);

            /* record the time when save log */
            g_data_log_tick_record = HAL_GetSysMsTick();

            //DLOG_Error("systicks: %d", g_data_log_tick_record);

            g_data_log_buffer_pos = 0;

            g_data_log_write_index++;

            if (g_data_log_write_index >= DATA_LOG_BUFFER_NUMBER)
            {
                g_data_log_write_index = 0;
            }
        }
        else
        {
            memcpy((void *)&g_data_log_buffer[g_data_log_write_index][g_data_log_buffer_pos],
                   (void *)data_buffer,
                   data_size);

            g_data_log_buffer_pos += data_size;

            data_size              = 0;
        }
    }

    UnLock(&g_data_log_lock);
}


/**
* @brief    once SD buffer is full, notify CPU0 to write into SD card
* @param  none
* @retval none
* @note   none
*       
*/
static void ss_hal_flush_udisk(void *data_buffer, uint16_t data_size)
{
    STRU_SysEvent_FLUSH_UDISK_BUFFER     st_flush_udisk_buffer;
    static uint32_t                      index = 0;

    index++;

    st_flush_udisk_buffer.data_address  = (uint32_t)data_buffer;
    st_flush_udisk_buffer.data_length   = data_size;
    st_flush_udisk_buffer.data_index    = index;

    SYS_EVENT_NotifyInterCore(SYS_EVENT_ID_FLUSH_UDISK_BUFFER, (void *)&st_flush_udisk_buffer);
}


/**
* @brief   if 100 ms, no data write into SD buffer, auto save current data to SD card
* @param  none
* @retval none
* @note   none
*       
*/
static void ss_hal_datalog_autosave(void *p)
{
    uint32_t     current_systicks = 0;
    uint32_t     systicks_diff = 0;

    current_systicks = HAL_GetSysMsTick();

    if (current_systicks >= g_data_log_tick_record)
    {
        systicks_diff = current_systicks - g_data_log_tick_record;
    }
    else
    {
        systicks_diff = (0xFFFFFFFFUL - g_data_log_tick_record) + current_systicks;
    }

    /* timeout, auto save logfile */
    if (systicks_diff >= DATA_LOG_BUFFER_FLUSH_PERIOD)
    {
        if (g_data_log_buffer_pos > 0)
        {
            ss_hal_flush_udisk((void *)g_data_log_buffer[g_data_log_write_index],
                                g_data_log_buffer_pos);

            g_data_log_buffer_pos = 0;

            g_data_log_write_index++;

            if (g_data_log_write_index >= DATA_LOG_BUFFER_NUMBER)
            {
                g_data_log_write_index = 0;
            }
        }

        g_data_log_tick_record = HAL_GetSysMsTick();
    }
}


// move from ss_hal_core.cpp
static const uint16_t DEBUGLOG_FLUSH_SIZE = 128;
static bool _debuglog_is_block = true;

static void ss_hal_debuglog_flush(){
    DLOG_Output(DEBUGLOG_FLUSH_SIZE);
}

void ss_hal_debuglog(const char* fmt, ...){
    va_list vlist;
    va_start(vlist, fmt);
    vprintf(fmt, vlist);
    va_end(vlist);
    printf("\n");
    if(ss_hal_debuglog_get_block()){
        DLOG_Output(DEBUGLOG_FLUSH_SIZE);
    }
}

void ss_hal_debuglog_set_block( bool is_block ){
    _debuglog_is_block = is_block;
}

bool ss_hal_debuglog_get_block(){
    return _debuglog_is_block;
}

