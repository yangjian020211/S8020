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


#define     MAX_FCT_NODE    (32)
typedef struct
{
    uint32_t    nodecnt;
    uint32_t    nodeid[MAX_FCT_NODE];
    uint32_t    offset_to_fct_node[MAX_FCT_NODE];
} STRU_NODE_LIST;


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
static int FCT_ListAllNode(STRU_FACTORY_SETTING *fct, STRU_NODE_LIST *p_nodelist)
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
    int node_cnt0 = FCT_ListAllNode((STRU_FACTORY_SETTING *)pst_factory_defaultcfgnode, &st_node_list_default);
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
 * write data & checksum to flash
*/
int FCT_Save2Flash(STRU_FACTORY_SETTING * fct)
{
    uint32_t i         = 0;
    uint32_t checksum  = 0;
    uint8_t *pdata     = (uint8_t *)fct;
    uint32_t totalsize = sizeof(STRU_FACTORY_SETTING);
    uint32_t writesize = 0;
    STRU_NODE_LIST tmpnode;

    //check node valid
    if (FCT_ListAllNode(fct, &tmpnode) <= 0)
    {
        return 0;
    }

    for (i = 0; i < totalsize; i++)
    {
        checksum += pdata[i];
    }

    DLOG_Info("save to flash: 0x%x 0x%x", NV_FLASH_FCT_START_ADDR_0, NV_FLASH_FCT_START_ADDR_1);

    //erase flash ADDR_0
    writesize = 0;
    do
    {
        NOR_FLASH_EraseSector(NV_FLASH_FCT_START_ADDR_0 + writesize);
        writesize += (4 * 1024);
    }while(writesize < totalsize);

    //write data
    NOR_FLASH_WriteByteBuffer(NV_FLASH_FCT_START_ADDR_0, pdata, totalsize);
    //write checksum
    NOR_FLASH_WriteByteBuffer(NV_FLASH_FCT_START_ADDR_0 + totalsize, (uint8_t *)&checksum, sizeof(checksum));

    //erase flash ADDR_1
    writesize = 0;
    do
    {
        NOR_FLASH_EraseSector(NV_FLASH_FCT_START_ADDR_1 + writesize);
        writesize += (4 * 1024);
    }while(writesize < totalsize);

    NOR_FLASH_WriteByteBuffer(NV_FLASH_FCT_START_ADDR_1, pdata, totalsize);
    NOR_FLASH_WriteByteBuffer(NV_FLASH_FCT_START_ADDR_1 + totalsize, (uint8_t *)&checksum, sizeof(checksum));

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


void FCT_SaveToFlashTest(void)
{
    DLOG_Info("pst_factory_defaultcfgnode = 0x%x", pst_factory_defaultcfgnode);

    if (NULL == pst_factory_defaultcfgnode)
    {
        return;
    }

    FCT_Save2Flash((STRU_FACTORY_SETTING *)pst_factory_defaultcfgnode);
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


/*
 * reset factory setting
*/
void FCT_Reset(void)
{
    uint32_t writesize = 0;
    uint32_t totalsize = NV_FLASH_FCT_SIZE;

    do
    {
        NOR_FLASH_EraseSector(NV_FLASH_FCT_START_ADDR_0 + writesize);
        writesize += (4 * 1024);
    }while(writesize < totalsize);

    DLOG_Warning("finished");
}
