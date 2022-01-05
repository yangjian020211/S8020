#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "memory_config.h"
#include "debuglog.h"
#include "quad_spi_ctrl.h"
#include "nor_flash.h"
#include "cfg_parser.h"
#include "factory.h"
#include "hal_gpio.h"


STRU_NODE_LIST st_node_list_default  = {0};
STRU_NODE_LIST st_node_list_user = {0};


STRU_FACTORY_SETTING *pst_factory_defaultcfgnode = NULL;

#define FLASH_APB_FCT_START_ADDR_0  (NV_FLASH_FCT_START_ADDR_0 + FLASH_APB_BASE_ADDR)
#define FLASH_APB_FCT_START_ADDR_1  (NV_FLASH_FCT_START_ADDR_1 + FLASH_APB_BASE_ADDR)


/*
 * 1: valid
 * 0: unvalid
*/
static uint8_t FCT_CheckFlashSettingValid(STRU_FACTORY_SETTING *fct)
{
    uint8_t *pdata         = (uint8_t *)fct;
    STRU_cfgNode *fct_node = &(fct->st_factoryNode);

    uint8_t  valid = 0;
    uint32_t fct_len = sizeof(STRU_cfgNode) + fct_node->nodeDataSize;
    uint32_t i;
    uint32_t checksum = 0;

    if (fct_len > 4 * 1024) //sector size: 4 * 1024 
    {
        DLOG_Error("Error: factory len = 0x%x 0x%x", fct, fct_len);
        return 0;
    }

    //checksum
    for (i = 0; i < fct_len; i++)
    {
        checksum += pdata[i];
    }

    valid = (checksum == *(uint32_t *)(pdata + fct_len));
    if (valid == 0)
    {
        DLOG_Info("fct checksum error: %p %d %d ", pdata, checksum, *(uint32_t *)(pdata + fct_len));
    }

    return valid;
}


/*
 * return: factory node count
*/
int FCT_ListAllNode(STRU_FACTORY_SETTING *fct, STRU_NODE_LIST *p_nodelist)
{
    STRU_FACTORY_SETTING_DATA *fct_data = (STRU_FACTORY_SETTING_DATA *)&fct->st_factorySetting;
    uint32_t fct_datasize = fct->st_factoryNode.nodeDataSize;
    uint32_t cur_datasize = 0;
    p_nodelist->nodecnt   = 0;

    if (FACTORY_SETTING_NODE_ID != fct->st_factoryNode.nodeId)
    {
        DLOG_Error("No factorySetting data addr=0x%x", (uint32_t)fct);
        return 0;
    }
    DLOG_Info("fct data addr=0x%x 0x%x size=%d", (uint32_t)fct, (uint32_t)fct_data, fct_datasize);

    //because node parse is 4bytes aligned
    if (fct_datasize & 0x03)
    {
        fct_datasize += (fct_datasize + 4) & (~0x03);
    }

    //parse factory data to get sub node 
    while (cur_datasize < fct_datasize)
    {
        STRU_cfgNode * node = (STRU_cfgNode *)((uint8_t *)fct_data + cur_datasize);
        DLOG_Info("node ptr=0x%x 0x%x %d", node, node->nodeId, node->nodeDataSize);

        if ( (node->nodeId & FACTORY_NODE_ID_MASK) != FACTORY_NODE_ID_MASK)
        {
            p_nodelist->nodecnt = 0;
            DLOG_Error("fct node id=%d", node->nodeId);
            return 0;
        }

        if (p_nodelist->nodecnt < MAX_FCT_NODE)
        {
            p_nodelist->offset_to_fct_node[p_nodelist->nodecnt] = ((uint32_t)node - (uint32_t)fct);
            p_nodelist->nodeid[p_nodelist->nodecnt] = node->nodeId;
            p_nodelist->nodecnt ++;
        }

        cur_datasize += sizeof(STRU_cfgNode);

        if (node->nodeDataSize & 0x03)
        {
            cur_datasize += (node->nodeDataSize + 4) & (~0x03);  //STRU_cfgNode is 4bytes-align alignment
        }
        else
        {
            cur_datasize += node->nodeDataSize;
        }
    }

    //check if some error happen when node parse
    if (cur_datasize != fct_datasize)
    {
        p_nodelist->nodecnt = 0;
        DLOG_Error("Fct setting parser error %d %d", cur_datasize, fct_datasize);
    }

    return (p_nodelist->nodecnt);
}


/*
 * 0: Fail to get factory setting
 * 1: suceess
*/
int FCT_Load2Sram(void)
{
    uint8_t  valid0 = 0, valid1 = 0;
    uint32_t flash_factoryAddr;

    // get sram config data & node
    void *p_sramcfgdata = CFGBIN_GetNodeAndData((STRU_cfgBin *)SRAM_CONFIGURE_MEMORY_ST_ADDR, FACTORY_SETTING_NODE_ID, (STRU_cfgNode **)&pst_factory_defaultcfgnode);
    if (p_sramcfgdata == NULL || pst_factory_defaultcfgnode == NULL)
    {
        DLOG_Critical("Fail get sram %p %p", p_sramcfgdata, pst_factory_defaultcfgnode);
        return -1;
    }

    DLOG_Info("get sram %p %p", p_sramcfgdata, pst_factory_defaultcfgnode);

    int node_cnt0 = FCT_ListAllNode((STRU_FACTORY_SETTING *)pst_factory_defaultcfgnode, &st_node_list_default);

#ifdef RF_9363
    uint32_t u32_Gpio87_val = 0;
    uint32_t u32_Gpio88_val = 0;
    uint32_t u32_Gpio89_val = 0;

    HAL_GPIO_InPut(HAL_GPIO_NUM87);
    HAL_GPIO_InPut(HAL_GPIO_NUM88);
    HAL_GPIO_InPut(HAL_GPIO_NUM89);

    HAL_GPIO_GetPin(HAL_GPIO_NUM87, &u32_Gpio87_val);
    HAL_GPIO_GetPin(HAL_GPIO_NUM88, &u32_Gpio88_val);
    HAL_GPIO_GetPin(HAL_GPIO_NUM89, &u32_Gpio89_val);

    if (!(u32_Gpio87_val == 0 && u32_Gpio88_val == 1 && u32_Gpio89_val == 1))  //Board V2
    {
        STRU_RF_CHANNEL *p_frq;
        STRU_cfgNode *p_node;

        STRU_RF_POWER_CTRL *pst_powercfg = NULL;
        STRU_cfgNode *cfgnode;

        p_frq = FCT_GetNodeAndData(FACTORY_SUBNODE_BAND0_VT_10M_FRQ_ID, &p_node);

        if (p_frq != NULL)
        {
            p_frq->u32_rfChCount = 10;
            p_frq->u16_rfChFrqList[0] = 6330;
            p_frq->u16_rfChFrqList[1] = 6410;
            p_frq->u16_rfChFrqList[2] = 6730;
            p_frq->u16_rfChFrqList[3] = 6820;
            p_frq->u16_rfChFrqList[4] = 6930;
            p_frq->u16_rfChFrqList[5] = 6980;
            p_frq->u16_rfChFrqList[6] = 7340;
            p_frq->u16_rfChFrqList[7] = 7370;
            p_frq->u16_rfChFrqList[8] = 7580;
            p_frq->u16_rfChFrqList[9] = 7720;
        }

        pst_powercfg = (STRU_RF_POWER_CTRL *)FCT_GetNodeAndData(FACTORY_SUBNODE_POWER_NODE_ID, &cfgnode);
        if (pst_powercfg != NULL)
        {
            pst_powercfg->vtPowerOther[0] = 0x1A;
            pst_powercfg->vtPowerOther[1] = 0x1A;

            pst_powercfg->rcPowerOther[0] = 0x19;
            pst_powercfg->rcPowerOther[1] = 0x19;
        }

        DLOG_Info("********* Hardware Version 2.0. *********\n");
    }
    else
    {
        DLOG_Info("********* Hardware Version 3.0. *********\n");
    }

#endif

    //check FLASH_APB_FCT_START_ADDR_0 factory valid
    valid0 = FCT_CheckFlashSettingValid((STRU_FACTORY_SETTING *)FLASH_APB_FCT_START_ADDR_0);
    if (0 == valid0)
    {
        valid1 = FCT_CheckFlashSettingValid((STRU_FACTORY_SETTING *)FLASH_APB_FCT_START_ADDR_1);        
    }

    if (1 == valid0)
    {
        flash_factoryAddr = FLASH_APB_FCT_START_ADDR_0;
    }
    else if (1 == valid1)
    {
        flash_factoryAddr = FLASH_APB_FCT_START_ADDR_1;
    }
    else
    {
        DLOG_Critical("No valid flash factory setting");
        return 1;
    }

    //do replacement, list sram node, list flash node
    int node_cnt1 = FCT_ListAllNode((STRU_FACTORY_SETTING *)flash_factoryAddr,   &st_node_list_user);

    if (node_cnt0 > 0 && node_cnt1 > 0)
    {
        uint8_t i, j;
        uint32_t flash_cfgdata = flash_factoryAddr + sizeof(STRU_cfgNode);

        for (i = 0 ; i < node_cnt0; i++)
        {
            for (j = 0 ; j < node_cnt1; j++)
            {
                STRU_cfgNode * node0 = (STRU_cfgNode *)((uint8_t *)pst_factory_defaultcfgnode + st_node_list_default.offset_to_fct_node[i]);
                STRU_cfgNode * node1 = (STRU_cfgNode *)(flash_factoryAddr + st_node_list_user.offset_to_fct_node[j]);

                if (st_node_list_default.nodeid[i] == st_node_list_user.nodeid[j])
                {
                    //do replacement, copy to sram
                    if ( node0->nodeDataSize == node1->nodeDataSize)
                    {
                        memcpy((void *)node0, (void *)node1, node0->nodeDataSize + sizeof(STRU_cfgNode));
                    }
                    else
                    {
                        DLOG_Error("node size error %d %d", node0->nodeDataSize, node1->nodeDataSize);
                    }

                    break;
                }
            }
        }
    }
    
    return 1;
}


/*
 * copy factory setting to buffer
*/
int FCT_CopySetting(void *pdata, uint32_t size)
{
    uint32_t totalsize = sizeof(STRU_FACTORY_SETTING);
    if (size < totalsize)
    {
        return -1;
    }

    memcpy(pdata, (void *)pst_factory_defaultcfgnode, sizeof(STRU_FACTORY_SETTING));

    return totalsize;
}

void * FCT_GetNodeAndData(uint32_t u32_nodeId, STRU_cfgNode **pp_node)
{
    int i;

    if (pst_factory_defaultcfgnode == NULL)
    {
        void * pdata = CFGBIN_GetNodeAndData((STRU_cfgBin *)SRAM_CONFIGURE_MEMORY_ST_ADDR, FACTORY_SETTING_NODE_ID, (STRU_cfgNode **)&pst_factory_defaultcfgnode);
        if (pdata != NULL && pst_factory_defaultcfgnode != NULL)
        {
            FCT_ListAllNode((STRU_FACTORY_SETTING *)pst_factory_defaultcfgnode, &st_node_list_default);
        }
    }

    if (pst_factory_defaultcfgnode == NULL || st_node_list_default.nodecnt == 0)
    {
        DLOG_Error("No factory node: %p %d", pst_factory_defaultcfgnode, st_node_list_default.nodecnt);
        return NULL;
    }

    for (i= 0; i < st_node_list_default.nodecnt; i++)
    {
        if (u32_nodeId == st_node_list_default.nodeid[i])
        {
            DLOG_Info("find nod 0x%x", u32_nodeId);
            break;
        }
    }

    if (i < st_node_list_default.nodecnt)
    {
        uint8_t *pdata = (uint8_t *)pst_factory_defaultcfgnode;
        STRU_cfgNode *node = (STRU_cfgNode *)(pdata + st_node_list_default.offset_to_fct_node[i]);
        if (pp_node != NULL)
        {
            *pp_node = node;
        }

        return ((void *)(node + 1));
    }
    else
    {
        if (pp_node != NULL)
        {
            *pp_node = NULL;
        }

        return NULL;
    }
}
