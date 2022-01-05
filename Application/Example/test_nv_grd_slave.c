#include "hal_nv.h"
#include "hal_bb_debug.h"
#include "wireless_interface.h"
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "debuglog.h"
#include "nor_flash.h"



#define MAGIC_ID 0xa1b2c5d6
#define CMD_GRD_ENTER_SLAVE  0x8a
#define CMD_GRD_CLEAR_NV_SLAVE_ID  0x8b

typedef struct
{
    uint32_t magic_id;
    uint8_t rc_id[5];
    uint8_t vt_id[2];
    uint8_t rsv[17];
}SLAVE_ID;

#define SAVLE_ID_LEN (sizeof(SLAVE_ID))

void clear_nv(void)
{
    uint8_t data[SAVLE_ID_LEN];

    memset(data, 0xff, SAVLE_ID_LEN);
    
    HAL_NV_UsrDataWrite(0, data, SAVLE_ID_LEN);
}

uint32_t read_nv_slave_id(SLAVE_ID *pSlave_id)
{
    uint8_t rData[SAVLE_ID_LEN];
    SLAVE_ID *pSlave_id_tmp;

    if (HAL_NV_ERR == HAL_NV_UsrDataRead(0, rData, SAVLE_ID_LEN))
    {
        DLOG_Warning("read err");
        return HAL_NV_ERR;
    }

    pSlave_id_tmp = (SLAVE_ID *)rData;
    if(pSlave_id_tmp->magic_id == MAGIC_ID)
    {
        pSlave_id->magic_id = pSlave_id_tmp->magic_id;
        memcpy(pSlave_id->rc_id,pSlave_id_tmp->rc_id,5);
        memcpy(pSlave_id->vt_id,pSlave_id_tmp->vt_id,2);
        return 0;
    }
    else
    {
        DLOG_Warning("not get slave id");
        return 1;
    }

    return 0;
}


uint32_t write_nv_slave_id(uint8_t *rc_id, uint8_t *vt_id)
{
    uint8_t rData[SAVLE_ID_LEN];
    SLAVE_ID *pSlave_id_tmp;
    HAL_RET_T ret;

    pSlave_id_tmp = (SLAVE_ID *)rData;

    pSlave_id_tmp->magic_id = MAGIC_ID;
    memcpy(pSlave_id_tmp->rc_id,rc_id,5);
    memcpy(pSlave_id_tmp->vt_id,vt_id,2);
    //clear_nv();
    ret=HAL_NV_UsrDataWrite(0, rData, SAVLE_ID_LEN);
    if(ret != HAL_OK)
    {
        DLOG_Warning("write slave id failed %d",ret);
        return ret;
    }

    return 0;

}


/** 
 * @brief USB send command to enter slave mode      
 * @param msg head 10 byte is USB protocol head, payload = msg + 10
          payload[0], 0,not save nv flash, 1 save to nv flash
          payload[1]-payload[7], rc id(5byte) + vt id (2byte)
 * @retval      
 * @retval      
 * @note        
 */
uint8_t cmd_usb_enter_slave(void *msg, uint8_t port_id)
{
    uint8_t *pmsg = (uint8_t *)msg;

    if(pmsg[10])
    {
        if(write_nv_slave_id(pmsg+11,pmsg+11+5) == 0)
        {
            DLOG_Warning("write nv slave id successful");
        }
    }
    
    HAL_BB_SetGrdSlaveMode(pmsg+11,pmsg+11+5);
    DLOG_Warning("enter slave mode %x:%x:%x:%x:%x,%x:%x:",pmsg[11],pmsg[12],pmsg[13],pmsg[14],pmsg[15],pmsg[16],pmsg[17]);
    return 0;
}
/** 
 * @brief USB send command to clear slave mode      
 * @param 
 * @retval      
 * @retval      
 * @note        
 */
uint8_t cmd_usb_clear_nv_slave_id(void *msg, uint8_t port_id)
{
    clear_nv();
    DLOG_Warning("clear nv slave id");
    return 0;
}

void grd_slave_mode_init(void)
{
    SLAVE_ID slave_id;
    
    STRU_WIRELESS_USER_HANDLER   Cmd_Handler[2] = {
        {CMD_GRD_ENTER_SLAVE, cmd_usb_enter_slave},
        {CMD_GRD_CLEAR_NV_SLAVE_ID, cmd_usb_clear_nv_slave_id},
    };
    
    WIRELESS_INTERFACE_RegisterUserHandler(Cmd_Handler,2);

    if(read_nv_slave_id(&slave_id) == 0)
    {
        DLOG_Warning("enter slave mode %x:%x:%x:%x:%x,%x:%x:",slave_id.rc_id[0],slave_id.rc_id[1],slave_id.rc_id[2],slave_id.rc_id[3],
            slave_id.rc_id[4],slave_id.vt_id[0],slave_id.vt_id[1]);
        HAL_BB_SetGrdSlaveMode(slave_id.rc_id,slave_id.vt_id);
    }
}
