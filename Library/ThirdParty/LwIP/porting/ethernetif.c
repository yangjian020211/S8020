/**
 * @file
 * Ethernet Interface Skeleton
 */

/*
 * Copyright (c) 2001-2004 Swedish Institute of Computer Science.
 * All rights reserved. 
 * 
 * Redistribution and use in source and binary forms, with or without modification, 
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. The name of the author may not be used to endorse or promote products
 *    derived from this software without specific prior written permission. 
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR IMPLIED 
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF 
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT 
 * SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, 
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT 
 * OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS 
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING 
 * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY 
 * OF SUCH DAMAGE.
 *
 * This file is part of the lwIP TCP/IP stack.
 * 
 * Author: Adam Dunkels <adam@sics.se>
 *
 */

#include <string.h>
#include "lwip/mem.h"
#include "lwip/pbuf.h"
#include <lwip/stats.h>
#include <lwip/snmp.h>
#include "lwip/opt.h"
#include "netif/etharp.h"
#include "netif/ppp_oe.h"

/* Kernel includes. */
#include "hal_usb_host.h"
#include <string.h>


#define configENET_RX_BUFFER_SIZE               (1520)
#define configENET_TX_BUFFER_SIZE               (1520)
#define configUSE_PROMISCUOUS_MODE              ( 0 )
#define configETHERNET_INPUT_TASK_STACK_SIZE    ( 256 )
#define configETHERNET_INPUT_TASK_PRIORITY      ( configMAX_PRIORITIES - 1 )


/* Define those to better describe your network interface. */
#define IFNAME0 'e'
#define IFNAME1 'n'

struct netif *s_netif = NULL;

/* Standard lwIP netif handlers. */
static err_t low_level_init( struct netif *netif );
static err_t low_level_output(struct netif *netif, struct pbuf *p);
static struct pbuf *low_level_input(struct netif *netif, char *data, int size);

static err_t low_level_init(struct netif *netif)
{
  /* set MAC hardware address */
  HAL_USB_NetDeviceGetMacAddr(netif->hwaddr);
  if (netif->hwaddr[0]==0 && netif->hwaddr[1]==0 && netif->hwaddr[2]==0 && 
      netif->hwaddr[3]==0 && netif->hwaddr[4]==0 && netif->hwaddr[5]==0)
  {
    return ERR_IF;
  }

  /* set MAC hardware address length */
  netif->hwaddr_len = ETHARP_HWADDR_LEN;

  /* maximum transfer unit */
  netif->mtu = configENET_TX_BUFFER_SIZE;

  /* device capabilities */
  /* don't set NETIF_FLAG_ETHARP if this device is not an ethernet one */
  netif->flags = NETIF_FLAG_BROADCAST | NETIF_FLAG_ETHARP | NETIF_FLAG_LINK_UP;

  return ERR_OK;
}

/**
 * This function should do the actual transmission of the packet. The packet is
 * contained in the pbuf that is passed to the function. This pbuf
 * might be chained.
 *
 * @param netif the lwip network interface structure for this ethernetif
 * @param p the MAC packet to send (e.g. IP packet including MAC addresses and type)
 * @return ERR_OK if the packet could be sent
 *         an err_t value if the packet couldn't be sent
 *
 * @note Returning ERR_MEM here if a DMA queue of your MAC is full can lead to
 *       strange results. You might consider waiting for space in the DMA queue
 *       to become availale since the stack doesn't retry to send a packet
 *       dropped because of memory failure (except for the TCP timers).
 */

static err_t low_level_output(struct netif *netif, struct pbuf *p)
{
  u16_t l = 0;
  struct pbuf *q;
  portBASE_TYPE i;


  for(q = p; q != NULL; q = q->next) 
  {
     unsigned char *tmp = (unsigned char *)(q->payload); 
     HAL_USB_NetDeviceSend(q->len, q->payload);
  }

  return ERR_OK;
}

/**
 * Should allocate a pbuf and transfer the bytes of the incoming
 * packet from the interface into the pbuf.
 *
 * @param netif the lwip network interface structure for this ethernetif
 * @return a pbuf filled with the received packet (including MAC header)
 *         NULL on memory error
 */
static struct pbuf *low_level_input(struct netif *netif, char *data, int size)
{
  struct pbuf *head;

  if (size <= 0 || size > configENET_TX_BUFFER_SIZE * 4)
  {
    return NULL;
  }

  head = pbuf_alloc(PBUF_RAW, size, PBUF_POOL);
  if(head == NULL)
  {
    return NULL;
  }
  else
  {
    /////////////wkwk/////////////
    struct pbuf * pnow = head;
    int32_t nowlen = 0;

    for(nowlen = 0; nowlen < head->tot_len && pnow != NULL; nowlen += pnow->len)
    {
        memcpy((uint8_t*)(pnow->payload), data + nowlen, pnow->len);
        pnow = pnow->next;
    }
    pnow = NULL;
  }
  
  return head;  
}


/**
 * This function should be called when a packet is ready to be read
 * from the interface. It uses the function low_level_input() that
 * should handle the actual reception of bytes from the network
 * interface. Then the type of the received packet is determined and
 * the appropriate input function is called.
 *
 */
void ethernetif_input(void *data, uint32_t size)
{
   struct eth_hdr *ethhdr;
   struct pbuf *p;
   struct netif *netif = s_netif;
   char ret;
   p = low_level_input(netif, data, size);

   if (p == NULL) 
   {
     DLOG_Error("0 %s", "Malloc error");
     return;
   }

   /* points to packet payload, which starts with an Ethernet header */
   ethhdr = p->payload;  
   
   /* IP or ARP packet? */
   switch (htons(ethhdr->type)) 
   {
       case ETHTYPE_IP:
       case ETHTYPE_ARP:

#if PPPOE_SUPPORT
       case ETHTYPE_PPPOEDISC:
       case ETHTYPE_PPPOE:
#endif
           /* full packet send to tcpip_thread to process */
           if (NULL == netif->input)
           {
               pbuf_free(p);
               p = NULL;
               DLOG_Error("%s", "NULL == netif->input");
               return;
           }
           else
           {
               ret = netif->input(p, netif);
               if( ret!=ERR_OK)
               {
                   pbuf_free(p);
                   DLOG_Error("%s %d %p", "ethernetif_input: IP input error", ret, netif->input);
                   p = NULL;
                   return;
               }       
           }

           break;

      default:
        pbuf_free(p);
        p = NULL;
        DLOG_Error("Error type: 0x%x", htons(ethhdr->type));
        break;
  }

  return;
}

/**
 * Should be called at the beginning of the program to set up the
 * network interface. It calls the function low_level_init() to do the
 * actual setup of the hardware.
 *
 * This function should be passed as a parameter to netif_add().
 *
 * @param netif the lwip network interface structure for this ethernetif
 * @return ERR_OK if the loopif is initialized
 *         ERR_MEM if private data couldn't be allocated
 *         any other err_t on error
 */
err_t
ethernetif_init(struct netif *netif)
{
  err_t result;

  LWIP_ASSERT("netif != NULL", (netif != NULL));

#if LWIP_NETIF_HOSTNAME
  /* Initialize interface hostname */
  netif->hostname = "lwip";
#endif /* LWIP_NETIF_HOSTNAME */

  /*
   * Initialize the snmp variables and counters inside the struct netif.
   * The last argument should be replaced with your link speed, in units
   * of bits per second.
   */
  NETIF_INIT_SNMP(netif, snmp_ifType_ethernet_csmacd, LINK_SPEED_OF_YOUR_NETIF_IN_BPS);

  netif->name[0] = IFNAME0;
  netif->name[1] = IFNAME1;
  /* We directly use etharp_output() here to save a function call.
   * You can instead declare your own function an call etharp_output()
   * from it if you have to do some checks before sending (e.g. if link
   * is available...) 
   */
  netif->output = etharp_output;
  netif->linkoutput = low_level_output;
  s_netif = netif;

  /* initialize the hardware */
  if (low_level_init(netif) != ERR_OK)
  {
    return ERR_IF;
  }

  return ERR_OK;
}
