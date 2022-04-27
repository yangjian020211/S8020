///////////////////////////////////////////////////////////////////////////////
// Includes
///////////////////////////////////////////////////////////////////////////////
// Standard C Included Files
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
// SDK Included Files
#include "debuglog.h"
#include "hal_bb.h"
#include "hal_gpio.h"
#include "memory_config.h"
#include "debuglog.h"
#include "cmsis_os.h"
#include "hal_usb_host.h"
#include "hal.h"
#include "test_net_repeater_ground.h"
#include "hal_sram_ground.h"
#include "memory.h"
//#include "rle.h"


#define MAX_BUFFER (128)
#define NET_COM BB_COM_SESSION_3
#define MAX_IP_NUMBER 20
#define ARP_PACKAGE_SIZE 44
#define IP_ACK_LEN 60
#define IFRAME_SPACE 20
#define CHECK_NUM 10
#define CHECKTIMEOUTMS 200
#define CHECKTIMEOUTMS_WAIT 200

typedef enum
{
   ORG_FRAME=0x10,
   ORG_REF_FRAME,
   REF_ENCODE_NORMAL,
   REF_DATA,
   REF_ENCODE_PLUS,
   ACK_DATA= 0X80, 
   CLR_TABLE,  
   CLR_ALL_TABLE,
} ENUM_NET_CMD;


typedef struct _ring_buf
{
    void  *buffer[MAX_BUFFER];
    uint16_t  buffer_size[MAX_BUFFER];    
    uint16_t  rd_ptr;
    uint16_t  wr_ptr;
}ring_buf_t;

uint8_t filter_ip_collection[MAX_IP_NUMBER][4];
volatile ring_buf_t session_ring;

uint32_t start_send_tick = 0;
uint32_t pre_log_tick = 0;
uint32_t pre_tick = 0;

uint32_t total_size = 0;
uint8_t  ipcamera_mac_address[6];
uint8_t  ipcamera_mac_address_valid = 0;
uint32_t time_space;

#define MAX_FRAME 4
#define MAX_TX_BUF_DEEP 10
#define MAX_ENCODE_PACKET_THD 62
#define MAX_NET_LEN 1600
#define MAX_REF_FRAME_LEN 1600
#define UPDATE_REFFRAME_TIME 40000

static unsigned char dec_net_packets[MAX_FRAME][MAX_NET_LEN]={0};
static unsigned char ip_protocol_xor[MAX_REF_FRAME_LEN];
static unsigned char net_ref_packets[MAX_TX_BUF_DEEP][MAX_REF_FRAME_LEN];
unsigned char temp_net_ref_packets[MAX_REF_FRAME_LEN];
unsigned char update_net_ref_packets[MAX_REF_FRAME_LEN];


static unsigned char net_packets[MAX_FRAME][MAX_NET_LEN];
static unsigned char net_xor_packets[MAX_FRAME][MAX_REF_FRAME_LEN];
static unsigned char net_xor_rm_zero_encode_packets[MAX_FRAME][MAX_REF_FRAME_LEN];
static unsigned char net_send_buffer[MAX_TX_BUF_DEEP][MAX_NET_LEN];
static unsigned char net_id=0;
static unsigned char net_send_buffer_id=0;
static uint32_t net_send_buffer_len=0;
static uint32_t net_packet_pre_time=0;
static unsigned char net_refframe_id=0;
static osSemaphoreId net_semaphore_id;
static unsigned char get_upddate_rfid=0; 
static unsigned char cmdid_buf[MAX_TX_BUF_DEEP]; 

static void plot_msg(uint8_t *gdata,int size);
static void repeater_ground_input(void *data, uint32_t size);

static void plot_msg(uint8_t *gdata,int size)
{

	DLOG_Critical("tp=%d s=%d: %2x %2x %2x %2x %2x %2x %2x %2x %2x %2x %2x %2x %2x %2x %2x %2x %2x %2x %2x %2x %2x %2x %2x %2x %2x %2x %2x %2x %2x %2x %2x %2x %2x %2x %2x %2x %2x %2x %2x %2x %2x %2x %2x %2x %2x %2x %2x %2x %2x %2x %2x %2x %2x %2x %2x %2x %2x %2x %2x %2x",
			time_space,size,gdata[0],gdata[1],gdata[2],gdata[3],gdata[4],gdata[5],gdata[6],gdata[7],gdata[8],gdata[9],
			gdata[10],gdata[11],gdata[12],gdata[13],gdata[14],gdata[15],gdata[16],gdata[17],gdata[18],gdata[19],
			gdata[20],gdata[21],gdata[22],gdata[23],gdata[24],gdata[25],gdata[26],gdata[27],gdata[28],gdata[29],
			gdata[30],gdata[31],gdata[32],gdata[33],gdata[34],gdata[35],gdata[36],gdata[37],gdata[38],gdata[39],
			gdata[40],gdata[41],gdata[42],gdata[43],gdata[44],gdata[45],gdata[46],gdata[47],gdata[48],gdata[49],
			gdata[50],gdata[51],gdata[52],gdata[53],gdata[54],gdata[55],gdata[56],gdata[57],gdata[58],gdata[59]
			);
}
static void reset_reftable(unsigned char *si)
{

	int i=0;
	for(i=0;i<MAX_TX_BUF_DEEP;i++)
	{
		if(memcmp(&net_ref_packets[i][5], &si[5], 12)==0)
			memset(net_ref_packets[i], 0, MAX_REF_FRAME_LEN);
	}
}
static void reset_all_table()
{

	int i=0;
	for(i=0;i<MAX_TX_BUF_DEEP;i++)
	{
		memset(net_ref_packets[i], 0, MAX_REF_FRAME_LEN);
		memset(net_send_buffer[i], 0, MAX_NET_LEN);
	}
	for(i=0;i<MAX_FRAME;i++)
	{
		memset(net_packets[i], 0, MAX_NET_LEN);
		memset(net_xor_packets[i], 0, MAX_REF_FRAME_LEN);
		memset(net_xor_rm_zero_encode_packets[i], 0, MAX_REF_FRAME_LEN);
	}
	//net_id=0;
}

static void serial_xor(uint8_t *s1,uint8_t *s2,uint8_t *so,int len ){
	int i=0;
	for(i=0;i<len;i++) so[i] = s1[i] ^ s2[i];
}

static void encode_find_non_zero_plus(unsigned char *si,int in_len ,unsigned char *so,int *len_out)
{
	#define debug1 0
	int j=-1;
	int i=0;
	int n=0;
	unsigned char *s = malloc(in_len+100);
	memset(s, 0, in_len+100);
	int begin=0;
	s[0]=0x00;
	int addr=0;
	int zero_cnt=0;
	int state=0;
	while(1){
		
		if(state==0)
		{
			if(si[i])
			{
				if(i==0)
				{
					addr=0;
					j=0;
					s[j]=0;
					n=0;
				}
				else 
				{
					
					if(zero_cnt>14)
					{
						j++;
						addr=j;
						s[addr]= 0xf0 | (zero_cnt%15);	
						j++;
						addr=j;
						s[addr]= 0x00;
						
					}
					else if(zero_cnt>0)
					{
						j++;
						addr=j;
						s[addr] = (zero_cnt)<<4;
					}
					
				}
				zero_cnt=0;
				j++;
				state = 1;
			}
			else
			{
				state =4;
			}
		#if debug1
					printf("0 n=%d i=%d si[%d]=%x,j=%d,sj[%d]=%2x addr=%d sj[%d]=%2x zero_cnt=%d\n",n,i,i,si[i],j,j,s[j],addr,addr,s[addr],zero_cnt);
		#endif
		}
		else if(state==1)
		{
			n++;
			s[j]=si[i];
			if(n==15)
			{
				n=0;
				state =2;
				j++;
			}
			else
			{
				i++;
				if(i>=in_len)
				{
					state=6;
				}
				else
				{
					state=0;
				}
			}
		#if debug1
				 printf("1 n=%d i=%d si[%d]=%x,j=%d,sj[%d]=%2x addr=%d sj[%d]=%2x zero_cnt=%d\n",n,i,i,si[i],j,j,s[j],addr,addr,s[addr],zero_cnt);
		#endif
		}
		else if(state==2)
		{
			//addr=j;
			s[addr] |= 0x0f;
		#if debug1
				printf("2.1 n=%d i=%d si[%d]=%x,j=%d,sj[%d]=%2x addr=%d sj[%d]=%2x zero_cnt=%d\n",n,i,i,si[i],j,j,s[j],addr,addr,s[addr],zero_cnt);
		#endif
			//j++;
			addr =j;
			s[addr] = 0x00;
			i++;
			if(i>=in_len){
				state=6;
			}
			else{
				state=0;
			}
		#if debug1
				printf("2 n=%d i=%d si[%d]=%x,j=%d,sj[%d]=%2x addr=%d sj[%d]=%2x zero_cnt=%d\n",n,i,i,si[i],j,j,s[j],addr,addr,s[addr],zero_cnt);
		#endif
		}
		else if(state==4)
		{
			zero_cnt++; 		
			if(zero_cnt>30)
			{
				state=5;
				zero_cnt=0;
				j++;
				n=0;
				
			}
			else
			{
				if(n>0)
				{
					s[addr] +=n;
				}
				i++;
				if(i>=in_len){
					state=6;
				}
				else
				{
					state=0;
				}
				n=0;
			}
		#if debug1
				printf("4 n=%d i=%d si[%d]=%x,j=%d,sj[%d]=%2x addr=%d sj[%d]=%2x zero_cnt=%d\n",n,i,i,si[i],j,j,s[j],addr,addr,s[addr],zero_cnt);
		#endif
		}
		else if(state==5)
		{
			addr=j; 
			s[addr]=0xFF;	
			i++;
			if(i>=in_len){
				state=6;
			}
			else{
				
				state=0;
			}
		#if debug1
				printf("5 n=%d i=%d si[%d]=%x,j=%d,sj[%d]=%2x addr=%d sj[%d]=%2x zero_cnt=%d\n",n,i,i,si[i],j,j,s[j],addr,addr,s[addr],zero_cnt);
		#endif
		}
		else if(state==6)
		{
			j++;
			
			if(n>0)s[addr]+=n;	
			else if(zero_cnt>0){
				addr=j;
				s[addr]=zero_cnt<<4;
				j++;
			}				
		#if debug1
			printf("6 n=%d i=%d si[%d]=%x,j=%d,sj[%d]=%2x addr=%d sj[%d]=%2x zero_cnt=%d\n",n,i,i,si[i],j,j,s[j],addr,addr,s[addr],zero_cnt);
		#endif
			break;	
		}
	}
	//if(zero_cnt>0)j++;
	//s[j-1] |= 0xf0|(zero_cnt-1);
	*len_out = j;
	so[0]=j;
	for(i=0;i<j;i++) {
		so[i] = s[i];
	}
	
	if(s)
	free(s);

}

static int decode_find_non_zero_plus(unsigned char *si,int in_len,unsigned char *so,int *len_out){
	#define debug 0
	int i=0;
	unsigned char key;
	int j=0;
	int m=0;
	int n=0;
	int l=0;

	for(i=0;i<in_len;)
	{

		unsigned char key = si[i] & 0xf0;
		unsigned char zero_len=(si[i] & 0x0f);
		if(si[i]==0xff){
			zero_len = (key>>4)  + zero_len;
			memset(&so[j],0,zero_len);
			
		}
		else if(key == 0xf0)
		{
			zero_len = (key>>4)  + zero_len;
			memset(&so[j],0,zero_len);
			//j++;
			j+=zero_len;
	#if debug
			 printf("1 n=%2x i=%2d j=%2d,m=%2d	zero_len=%2d l=%2d\n",n,i,j,m,zero_len,l);
	#endif 
			i++;
			//n=((0xf0 & si[i])>>4);
			l=(0x0f & si[i]);
	#if debug
			printf("2 n=%2x i=%2d j=%2d,m=%2d  zero_len=%2d l=%2d\n",n,i,j,m,zero_len,l);
	#endif 
			//if(n==0x0f) continue;
		
			//printf("2.1 n=%2x i=%2d j=%2d,m=%2d  zero_len=%2d l=%2d\n",n,i,j,m,zero_len,l);
			//memset(&so[j],0,n);
			//j+=n;
			
			i++;
	#if debug
			printf("3 n=%2x i=%2d j=%2d,m=%2d  zero_len=%2d l=%2d\n",n,i,j,m,zero_len,l);
	#endif 
			for(m=0;m<l;m++) so[j+m] = si[i+m];
			j+=l;
			i+=l;
	#if debug
			printf("4 i=%2d j=%2d,m=%2d l=%2d\n",i,j,m,l);
	#endif
		}
		else
		{
			
			n=((0xf0 & si[i])>>4);
			l=(0x0f & si[i]);
			//if(i==0 && n==0 && l==0x0f) j=0;
	#if debug
			printf("5 n=%2d i=%2d j=%2d,m=%2d l=%2d\n",n,i,j,m,l);
	#endif 
			//if(l==0)n++;
			memset(&so[j],0,n);
			j+=n;
	#if debug
			 printf("6 n=%2x i=%2d j=%2d,m=%2d l=%2d\n",n,i,j,m,l);
	#endif
			//if(n==0)j++;
			//if(n==0)
			//	for(m=0;m<l;m++) so[j+m] = si[i+m+1];
			//else 
			i++;
			for(m=0;m<l;m++) so[j+m] = si[i+m];
			j+=l;
			i+=l;
	#if debug
			 printf("7 i=%2d j=%2d,m=%2d l=%2d\n",i,j,m,l);
	#endif
		}
	
	}
	*len_out = j;

}

static void clear_ref_frame(unsigned char *si){
	int i=0;
	int j=0;

	for(i=0;i<MAX_TX_BUF_DEEP;i++)
	{
		if(memcmp(&net_ref_packets[i][5], &si[5], 12)==0){
			for(j=0;j<20;j++) net_ref_packets[i][j]=0;
			DLOG_Critical("clear refid:%d",i);
		}
	}

}

static void netrcvDataHandler(void *p)
{
	uint8_t data_buf_proc[100];
	uint8_t temp_data[100];
    uint32_t u32_rcvLen = 0;
    HAL_RET_T ret=HAL_BB_ComReceiveMsg(NET_COM, data_buf_proc, sizeof(data_buf_proc), &u32_rcvLen);
	if(ret == HAL_OK){
		uint8_t cmd = data_buf_proc[0];
		uint8_t id = data_buf_proc[1];
		uint8_t net_ref_id = data_buf_proc[2];
		DLOG_Critical("ack cmd=%2x id=%d,net_ref_id=%d",cmd,id,net_ref_id);
		if(net_ref_id >=MAX_TX_BUF_DEEP) return;
		if(cmd==ORG_REF_FRAME)
		{
			DLOG_Critical("get org ref frame ack ");
			memcpy(&update_net_ref_packets[2], &data_buf_proc,u32_rcvLen);
			//osSemaphoreWait(net_semaphore_id,0);
			update_net_ref_packets[0]=(u32_rcvLen-3);
			update_net_ref_packets[1]=(u32_rcvLen-3)>>8;
			
			//osSemaphoreRelease(net_semaphore_id);
		}
		else if(cmd==CLR_TABLE) 
		{
		
			DLOG_Critical("get clear ref frame ask ");
			
			//osSemaphoreWait(net_semaphore_id,0);
			cmdid_buf[id]=CLR_TABLE;
			//osSemaphoreRelease(net_semaphore_id);
		}
		else if(cmd==CLR_ALL_TABLE)
		{
			DLOG_Critical("get clear all ref frame ask ");
			//osSemaphoreWait(net_semaphore_id,0);
			cmdid_buf[id]=CLR_ALL_TABLE;
			//osSemaphoreRelease(net_semaphore_id);
		}
	}
}
static HAL_RET_T send_ref_frame(){
	
	int rd_idx = session_ring.rd_ptr;
	int wr_idx = session_ring.wr_ptr;
	uint16_t size = session_ring.buffer_size[rd_idx];
	HAL_RET_T ret;
	memcpy(&temp_net_ref_packets[5], session_ring.buffer[rd_idx],size);
	temp_net_ref_packets[0] = size;
	temp_net_ref_packets[1] = size<<8;
	temp_net_ref_packets[2] = ORG_REF_FRAME;
	temp_net_ref_packets[3] = net_refframe_id;
	temp_net_ref_packets[4] = net_refframe_id;
	size +=3;
	ret = HAL_BB_ComSendMsg(NET_COM, &temp_net_ref_packets[2], size);
	if (HAL_OK == ret)
	{
    	rd_idx = (rd_idx + 1) % (MAX_BUFFER);
		//net_refframe_id=(net_refframe_id+1)%MAX_TX_BUF_DEEP;
		session_ring.rd_ptr = rd_idx;
    }
	else
	{
		HAL_Delay(size * 14*4 / 50);
	}
	return ret;

}

static HAL_RET_T send_encode_data(uint8_t id){
	//1 check the net_xor_rm_zero_encode_packets len to decide the plus encode
	int i=0;
	HAL_RET_T ret;
	int len = 0;
	int j = 0;
	net_send_buffer_len=0;
	memset(net_send_buffer[net_send_buffer_id], 0, MAX_NET_LEN);
	for(i=0;i<net_id;i++)
	{
		for(len=0;len<256;){
			if( net_xor_rm_zero_encode_packets[i][len]){
				len++;
			}
			else break;
		}
		memcpy(&net_send_buffer[net_send_buffer_id][5+net_send_buffer_len], net_xor_rm_zero_encode_packets[i],len);
		net_send_buffer[net_send_buffer_id][5+net_send_buffer_len+len]=0x00;
		net_send_buffer_len+=len+1;
	}
	net_send_buffer[net_send_buffer_id][0] = net_send_buffer_len;
	net_send_buffer[net_send_buffer_id][1] = net_send_buffer_len<<8;
	net_send_buffer[net_send_buffer_id][2] = REF_ENCODE_NORMAL;
	net_send_buffer[net_send_buffer_id][3] = net_send_buffer_id;
	net_send_buffer[net_send_buffer_id][4] = id;
	net_send_buffer_len +=3;
	//plot_msg(&net_send_buffer[net_send_buffer_id][2],net_send_buffer_len);

	ret = HAL_BB_ComSendMsg(NET_COM, &net_send_buffer[net_send_buffer_id][2], net_send_buffer_len);
#if 1
	static int totals = 0;
	static int n =0;

	totals +=net_send_buffer_len;
	n+=net_id;
	if(n>=100)
	{
		//plot_msg(&net_send_buffer[net_send_buffer_id][2],net_send_buffer_len);
		DLOG_Critical("total num=%d avrg=%d id=%d", totals,totals/n,id);
		totals = 0;
		n=0;
	}
#endif
	
	if (HAL_OK == ret)
	{
		net_send_buffer_id=(net_send_buffer_id+1)%MAX_TX_BUF_DEEP;
		net_id =0;
		net_send_buffer_len=0;
	}
	else
	{
		 HAL_Delay(net_send_buffer_len * 14 / 50);
	}
	return ret;

}

static HAL_RET_T send_org_packet(){
	
	int rd_idx = session_ring.rd_ptr;
	int wr_idx = session_ring.wr_ptr;
	uint16_t size = session_ring.buffer_size[rd_idx];
	net_send_buffer_len=0;
	HAL_RET_T ret;
	net_send_buffer_len = size;
	memcpy(&net_send_buffer[net_send_buffer_id][5], session_ring.buffer[rd_idx],size);
	
	net_send_buffer[net_send_buffer_id][0] = net_send_buffer_len;
	net_send_buffer[net_send_buffer_id][1] = net_send_buffer_len<<8;
	net_send_buffer[net_send_buffer_id][2] = ORG_FRAME;
	net_send_buffer[net_send_buffer_id][3] = net_send_buffer_id;
	net_send_buffer[net_send_buffer_id][4] = net_refframe_id;
	net_send_buffer_len +=3;
	ret = HAL_BB_ComSendMsg(NET_COM, &net_send_buffer[net_send_buffer_id][2], net_send_buffer_len);
	if (HAL_OK == ret)
    {
    	net_send_buffer_id=(net_send_buffer_id+1)%MAX_TX_BUF_DEEP;
    	rd_idx = (rd_idx + 1) % (MAX_BUFFER);
		session_ring.rd_ptr = rd_idx;
    }
	else
	{
		HAL_Delay(net_send_buffer_len * 14 / 50);
	}
	return ret;
}

static HAL_RET_T send_ref_packet(){
	HAL_RET_T ret;
	int rd_idx = session_ring.rd_ptr;
	int wr_idx = session_ring.wr_ptr;
	uint16_t size = session_ring.buffer_size[rd_idx];
	net_send_buffer_len=0;
	
	net_send_buffer[net_send_buffer_id][0] = net_send_buffer_len;
	net_send_buffer[net_send_buffer_id][1] = net_send_buffer_len<<8;
	net_send_buffer[net_send_buffer_id][2] = REF_DATA;
	net_send_buffer[net_send_buffer_id][3] = net_send_buffer_id;
	net_send_buffer[net_send_buffer_id][4] = net_refframe_id;
	net_send_buffer_len +=3;
	DLOG_Error("sent ref frame id=%d",net_send_buffer_id);
	ret = HAL_BB_ComSendMsg(NET_COM, &net_send_buffer[net_send_buffer_id][2], net_send_buffer_len);
	if (HAL_OK == ret)
    {
    	net_send_buffer_id=(net_send_buffer_id+1)%MAX_TX_BUF_DEEP;
    	rd_idx = (rd_idx + 1) % (MAX_BUFFER);
		session_ring.rd_ptr = rd_idx;
		net_id =0;
    }
	else
	{
		HAL_Delay(net_send_buffer_len * 14 / 50);
	}
	return ret;
}

static uint8_t get_ref_id(unsigned char *si,int in_len,int *id){
	int i=0;
	uint8_t cur_type=1;
	static uint8_t before_type=0;
	static uint32_t net_ref_packet_pre_time=0;
	static uint32_t update_net_ref_packet_time=0;
	int have =0;
	for(i=0;i<MAX_TX_BUF_DEEP;i++)
	{
		uint32_t size = net_ref_packets[i][1]<<8 ||net_ref_packets[i][0];
		if(memcmp(si, &net_ref_packets[i][5], 12)==0 && size >0)
		{
			*id= net_ref_packets[i][4];
			cur_type =2;
			before_type=2;
			have=1;
			uint32_t cur_time =  HAL_GetSysMsTick();
			if(cur_time-update_net_ref_packet_time>=UPDATE_REFFRAME_TIME){
				update_net_ref_packet_time = cur_time;
				cur_type=0;	
				before_type=0;
				return cur_type;
			}
			break;
			
		}
	}
	if(have)return cur_type;
	if(cur_type != 2)
	{
					
		if(net_packets[net_id][3] == 0xff)
		{
			before_type =1;
			cur_type=1;
		}
		
		else if(before_type ==0 )
		{
			uint32_t cur_time =  HAL_GetSysMsTick();
			if(cur_time-net_ref_packet_pre_time >400000){
				net_ref_packet_pre_time = cur_time;
				before_type =0;
				cur_type =0;
				
			}else{
				before_type =1;
				cur_type=1;
			}
		}
		else 
		{
			
			before_type =0;
			cur_type=0;
		}
	}

	return cur_type;

}

static void get_mac_address(){
   if (ipcamera_mac_address_valid == 0)
   {
	   while (HAL_USB_GetIPCameraMacAddress(ipcamera_mac_address) != HAL_OK)
	   {
		   HAL_Delay(20);
	   }

	   DLOG_Critical("get ipaddress: %02x, %02x, %02x, %02x, %02x, %02x",
									 ipcamera_mac_address[0],
									 ipcamera_mac_address[1],
									 ipcamera_mac_address[2],
									 ipcamera_mac_address[3],
									 ipcamera_mac_address[4],
									 ipcamera_mac_address[5]);

	   HAL_USB_NetDeviceUp(ENUM_USB_NETCARD_PROMISCUOUS_MODE);
	   HAL_USB_NetDeviceRecv(repeater_ground_input);
	   HAL_USB_NetDeviceSetMacAddr(ipcamera_mac_address);
	   ipcamera_mac_address_valid = 1;
   }

}
static void sessionSend_task(void const *argument)
{
	
	int i=0,j=0;
	static int k=0;
	int len=0;
	HAL_RET_T ret;
	uint8_t ref_frame_data[1600];
	uint8_t cur_type=1;
	int id=0;
    while(1)
    {
        
		get_mac_address();
        int rd_idx = session_ring.rd_ptr;
        int wr_idx = session_ring.wr_ptr;
        if (rd_idx >= MAX_BUFFER || wr_idx >=MAX_BUFFER)
        {
            DLOG_Error("race condition");
            HAL_Delay(5);
            continue;
        }
        if (rd_idx != wr_idx) //not empty
        {
        	uint16_t size = session_ring.buffer_size[rd_idx];
			uint8_t must_trans=0;
			static int  waittime=0;
			//osSemaphoreWait(net_semaphore_id,0);
			net_packets[net_id][0]=net_id;
			net_packets[net_id][1]=size;
			net_packets[net_id][2]=size>>8;
			memcpy(&net_packets[net_id][3], session_ring.buffer[rd_idx], size);

			int getsize = update_net_ref_packets[1]<<8 | update_net_ref_packets[0];
			cur_type =get_ref_id(session_ring.buffer[rd_idx],size,&id);
			
			if(net_id==0)
			{
				memcpy(ref_frame_data, &net_ref_packets[id][5],size);
				if(getsize>0)
				{
					for(i=0;i<10;i++)
					{
						if(cmdid_buf[i]==CLR_TABLE)
						{
							reset_reftable(net_ref_packets[i]);
							//osSemaphoreWait(net_semaphore_id,0);
							cmdid_buf[i]=0;
							//osSemaphoreRelease(net_semaphore_id);
						}
					}
					int getid = update_net_ref_packets[4];
					clear_ref_frame(update_net_ref_packets);
					memcpy(net_ref_packets[getid], update_net_ref_packets, getsize+5);
					plot_msg(net_ref_packets[getid],getsize+5);
					//osSemaphoreWait(net_semaphore_id,0);
					update_net_ref_packets[0]=0;
					update_net_ref_packets[1]=0;
					net_refframe_id=(net_refframe_id+1)%MAX_TX_BUF_DEEP;
					cur_type =get_ref_id(session_ring.buffer[rd_idx],size,&id);
					//waittime = 5;
					if(cur_type==2){
						DLOG_Error("net_send_buffer_id=%d",net_send_buffer_id);
						memcpy(ref_frame_data, &net_ref_packets[id][5], getsize);
						plot_msg(ref_frame_data, getsize);
						must_trans=1;
						plot_msg(session_ring.buffer[rd_idx], size);
					}
					
				}
			}
			
		
#if 1
			static int total =0;
			static int ids=0;
			static uint32_t timesa=0;
			uint32_t now=0;
			if(ids !=rd_idx){
				total +=size;
				ids = rd_idx;
			}
			now =  HAL_GetSysMsTick();
			if(now-timesa >=10000){
				
				DLOG_Critical("br=%d",total*1000/(now-timesa));
				timesa = now;
				total =0;
			}
#endif		
			
			
			//send org packet
			if(size > MAX_ENCODE_PACKET_THD || cur_type==1)
			{
				//if(waittime)waittime--;
				if(net_id > 0)//begin trans the encode data first
				{
					//DLOG_Critical("1 before_type = %d size=%d",before_type,size);
					//1 finished the encode and trans the encode data
					ret =  send_encode_data(id);	
					//2 transmit the org packet
					if(HAL_OK == ret)
					{
						net_id=0;
						ret =  send_org_packet();
					}
				}
				else
				{
					//1 send the org frame
					ret =  send_org_packet();
				}
			}
			else if(cur_type==0)
			{
				 send_ref_frame();
				//DLOG_Critical("0 before_type = %d size=%d",before_type,size);
			}
			else  if(cur_type==2)
			{
			
				if(net_id ==0)
				{
					//plot_msg(&net_ref_packets[net_refframe_id][2],size);
					
					serial_xor(ref_frame_data,&net_packets[net_id][3],ip_protocol_xor,size);
					int same=1;
					for(i=0;i<size;i++){
						if(ip_protocol_xor[i]){
								same=0;
								break;
							}
					}
					if(same){
						ret = send_ref_packet();
						//osSemaphoreRelease(net_semaphore_id);
						continue;
					}
					memcpy(&net_xor_packets[net_id][3],ip_protocol_xor, size);
				}
				
				//2 if net_id >0 and push the xor result to table
				else if(net_id>0)
				{
					serial_xor(&net_packets[net_id-1][3],&net_packets[net_id][3],ip_protocol_xor,size);
					memcpy(&net_xor_packets[net_id][3],ip_protocol_xor, size);
				}
				net_xor_packets[net_id][0]=net_id;
				net_xor_packets[net_id][1]=size;
				net_xor_packets[net_id][2]=size>>8;
				//3 encode the xor result
				memset(net_xor_rm_zero_encode_packets[net_id], 0, MAX_REF_FRAME_LEN);
				encode_find_non_zero_plus(ip_protocol_xor,size,net_xor_rm_zero_encode_packets[net_id],&len);
				#if 0
				unsigned char temp_protocol_xor[MAX_NET_LEN];
				static  int cnyt=0;
				cnyt++;
				int outsize=0;

			 	memset(temp_protocol_xor, 0, MAX_NET_LEN);
				decode_find_non_zero_plus(net_xor_rm_zero_encode_packets[net_id],len,temp_protocol_xor,&outsize);
				for(i=0;i<size;i++)
				{
					if(ip_protocol_xor[i] !=temp_protocol_xor[i])
					{
						DLOG_Error("error=%d,net_id=%d",net_send_buffer_id,net_id);
						if(net_id>0)
						{
							DLOG_Error("---0");
							plot_msg(&net_packets[net_id-1][3],size);
							HAL_Delay(10);
							DLOG_Error("---1");
							plot_msg(&net_packets[net_id][3],size);
							HAL_Delay(10);
						}else{
							DLOG_Error("---2");
							plot_msg(ref_frame_data,size);
							HAL_Delay(10);
							DLOG_Error("---3");
							plot_msg(&net_packets[net_id][3],size);
							HAL_Delay(10);
						}
						HAL_Delay(10);
						DLOG_Error("---4");
						plot_msg(ip_protocol_xor,size);
						HAL_Delay(10);
						DLOG_Error("---5");
						plot_msg(temp_protocol_xor,size);
						HAL_Delay(10);
						DLOG_Error("---6");
						plot_msg(net_xor_rm_zero_encode_packets[net_id],len);
					}
				}
	
					
				if(cnyt>100) cnyt=0;
				
				#endif
				
				net_id++;
				uint32_t curtime = HAL_GetSysMsTick();
				uint32_t spacetime= curtime-net_packet_pre_time;
				if(net_id >= MAX_FRAME || must_trans ) //org data
				{
					net_packet_pre_time = curtime;
					ret =  send_encode_data(id);
					if (HAL_OK == ret){
		                rd_idx = (rd_idx + 1) % (MAX_BUFFER);
		                session_ring.rd_ptr = rd_idx;
						net_id=0;
		            }
				}
				else{
					 rd_idx = (rd_idx + 1) % (MAX_BUFFER);
		             session_ring.rd_ptr = rd_idx;
				}
				
			}
			//osSemaphoreRelease(net_semaphore_id);
		}
		
        else
        {
            HAL_Delay(1);
        }
    }
}

static void create_send_thread(void)
{
    uint8_t i;
    session_ring.rd_ptr = 0;
    session_ring.wr_ptr = 0;
    for(i = 0; i < MAX_BUFFER; i++)
    {
        session_ring.buffer_size[i] = 0;
    }
    osThreadDef(sessionSend_Task, sessionSend_task, osPriorityHigh, 0, 16 * 128);
    osThreadCreate(osThread(sessionSend_Task), NULL);
}

//filter src ip
static uint8_t filter_ip(uint8_t *data){

	int i=0;
	//not set ip filter
	if(filter_ip_collection[0][0]==0x00 && filter_ip_collection[0][1]==0x00 && filter_ip_collection[0][2]==0x00 && filter_ip_collection[0][3]==0x00) return 1;
	for(i=0;i<MAX_IP_NUMBER;i++)
	{
		if(memcmp(filter_ip_collection[i], data, 4)== 0)  return 1;
	}

	return 0;
}

static void repeater_ground_input(void *data, uint32_t size)
{
    uint8_t rd_idx = session_ring.rd_ptr;
    uint8_t wr_idx = session_ring.wr_ptr;
    uint8_t cnt;
    uint8_t broadcast_mac_addr[6] = {0xff,0xff,0xff,0xff,0xff,0xff};
	static int getsum=0;
	uint8_t *gdata = (uint8_t *)data;
	uint8_t type[2] = {gdata[12],gdata[13]};
	uint8_t is_arp=0;
    total_size += size;
    if (start_send_tick == 0)
    {
        start_send_tick = HAL_GetSysMsTick();
    }

    if (wr_idx >= rd_idx)
    {
        cnt = wr_idx - rd_idx;
    }
    else
    {
        cnt = MAX_BUFFER - (rd_idx - wr_idx);
    }
	
    if (memcmp(ipcamera_mac_address, data, 6) != 0 && memcmp(broadcast_mac_addr, data, 6) != 0)
    {
        return;
    }
	
	
	if(type[0]==0x08 && type[1]==0x06)//arp
	{
		is_arp=1;
		if(filter_ip(&gdata[28])==0)  return;
		
	}
	else if(type[0]==0x08 && type[1]==0x00)//ip
	{
		is_arp=0;
		if(filter_ip(&gdata[26])==0)  return;
	}
	
	
    uint32_t cur_tick = HAL_GetSysMsTick();
  	time_space = cur_tick - pre_tick;
	pre_tick = cur_tick;
    if (time_space > 5000 || cnt > 40)
    {
        //pre_log_tick = cur_tick;
        DLOG_Error("send=%d tick=%d cnt=%d %d %d", total_size, (cur_tick-start_send_tick), cnt, rd_idx, wr_idx);
    }

    if (rd_idx >= MAX_BUFFER || wr_idx >=MAX_BUFFER )
    {
        DLOG_Error("race condition");
        return;
    }

    if ( (wr_idx + 1) % MAX_BUFFER == rd_idx)
    {
        DLOG_Error("No ring buffer !!!");
        return;
    }

    if (size > 2000)
    {
        DLOG_Error("size =%d overflow !!!", size);
        return;
    }
	//plot_msg(gdata,size);

    if (size > session_ring.buffer_size[wr_idx])
    {
        if (session_ring.buffer[wr_idx] == NULL)
        {
            session_ring.buffer[wr_idx] = malloc_safe(size);
        }
        else
        {
            session_ring.buffer[wr_idx] = realloc_safe(session_ring.buffer[wr_idx], size);
        }
    }

    if (session_ring.buffer[wr_idx] != NULL)
    {
        memcpy(session_ring.buffer[wr_idx], data, size);
        session_ring.buffer_size[wr_idx] = size;
        wr_idx = (wr_idx + 1) % MAX_BUFFER;
        session_ring.wr_ptr = wr_idx;
    }
    else
    {
        DLOG_Error("Buff malloc failed!");
    }

    return;
}

void set_ip_filter(uint8_t *data,uint8_t index)
{
	if(index>MAX_IP_NUMBER) return;
	filter_ip_collection[index][0]=data[0];
	filter_ip_collection[index][1]=data[1];
	filter_ip_collection[index][2]=data[2];
	filter_ip_collection[index][3]=data[3];
}

void command_TestNetRepeaterGnd( void )
{
    if (HAL_OK != HAL_BB_ComRegisterSession(NET_COM, BB_COM_SESSION_PRIORITY_LOW, BB_COM_SESSION_DATA_NORMAL, netrcvDataHandler))
    {
        DLOG_Error("Fail register session %d", NET_COM);
		return;
    }
	//HAL_USB_NetDeviceUp(ENUM_USB_NETCARD_PROMISCUOUS_MODE);
	int i=0;
	for(i=0;i<MAX_IP_NUMBER;i++)
	{
		filter_ip_collection[i][0]=0;
		filter_ip_collection[i][1]=0;
		filter_ip_collection[i][2]=0;
		filter_ip_collection[i][3]=0;
	}
	
    ipcamera_mac_address_valid = 0;
	net_id =0;
	osSemaphoreDef(net_send_semaphore);
    net_semaphore_id = osSemaphoreCreate(osSemaphore(net_send_semaphore),1);
	reset_all_table();
    create_send_thread();

    HAL_BB_UpgradeMode(0);

    HAL_SRAM_ChannelConfig(ENUM_HAL_SRAM_CHANNEL_TYPE_RTSP_BYPASS,  1);
                         
	DLOG_Error("ok register session %d", NET_COM);
}

/**
  * This function will be call when receive data form Baseband,
  * if you want send to USB automatically return HAL_OK, else return others.
  */
HAL_RET_T RTSPBufferCallBak(uint32_t dataLen, uint8_t *data)
{
    //uint8_t buffer[1600];
    
    //DLOG_Critical("dataLen:%u dataAddr:%d", dataLen, data);

    //memcpy(buffer, data, dataLen);// must copy

    //do something
    //HAL_USB_NetDeviceSend(dataLen, buffer);
    
	return HAL_OK;

    static int gl=0;
	gl++;
	if(gl==100){
		gl=0;
		DLOG_Critical("size=%d %2x %2x %2x %2x %2x %2x %2x %2x %2x %2x %2x %2x %2x %2x %2x %2x %2x %2x %2x %2x ", \
		dataLen, data[0],data[1],data[2],data[3],data[4],data[5],data[6],data[7],data[8],data[9],data[10], \
		 data[11],data[12],data[13],data[14],data[15],data[16],data[17],data[18],data[19] \
		);
	}
	
    return HAL_OK;
}
