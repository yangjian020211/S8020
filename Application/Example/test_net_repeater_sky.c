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
#include "test_net_repeater_sky.h"
#include "hal_sram_sky.h"
//#include "rle.h"


#define NET_COM BB_COM_SESSION_3
#define MAX_IP_NUMBER 20

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

static unsigned char rx_id=0;
static unsigned char tx_id=0;

#define MAX_FRAME 10
#define MAX_TX_BUF_DEEP 10
#define MAX_NET_LEN 1600
#define MAX_REF_FRAME_LEN 1600

static unsigned char dec_net_packets[MAX_FRAME][MAX_NET_LEN]={0};
static unsigned char net_ref_packets[MAX_TX_BUF_DEEP][MAX_REF_FRAME_LEN];
static unsigned char net_packets[MAX_TX_BUF_DEEP][MAX_NET_LEN];
unsigned char protocol_xor[MAX_TX_BUF_DEEP][MAX_REF_FRAME_LEN]={0};
unsigned char so[MAX_NET_LEN]={0};

static osSemaphoreId net_semaphore_id;

static void process_push2net();

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


static void plot_msg(uint8_t *gdata,int size)
{

	DLOG_Critical("s=%d: %2x %2x %2x %2x %2x %2x %2x %2x %2x %2x %2x %2x %2x %2x %2x %2x %2x %2x %2x %2x %2x %2x %2x %2x %2x %2x %2x %2x %2x %2x %2x %2x %2x %2x %2x %2x %2x %2x %2x %2x %2x %2x %2x %2x %2x %2x %2x %2x %2x %2x %2x %2x %2x %2x %2x %2x %2x %2x %2x %2x",
			size,gdata[0],gdata[1],gdata[2],gdata[3],gdata[4],gdata[5],gdata[6],gdata[7],gdata[8],gdata[9],
			gdata[10],gdata[11],gdata[12],gdata[13],gdata[14],gdata[15],gdata[16],gdata[17],gdata[18],gdata[19],
			gdata[20],gdata[21],gdata[22],gdata[23],gdata[24],gdata[25],gdata[26],gdata[27],gdata[28],gdata[29],
			gdata[30],gdata[31],gdata[32],gdata[33],gdata[34],gdata[35],gdata[36],gdata[37],gdata[38],gdata[39],
			gdata[40],gdata[41],gdata[42],gdata[43],gdata[44],gdata[45],gdata[46],gdata[47],gdata[48],gdata[49],
			gdata[50],gdata[51],gdata[52],gdata[53],gdata[54],gdata[55],gdata[56],gdata[57],gdata[58],gdata[59]
	);
}

static void process_push2net(){
/*
	uint8_t outbuf[1600]={0};
	HAL_RET_T  send_ret=0;
	if(ip_protocol[tx_id][4]==1)
	{
	    int size = ip_protocol[tx_id][1] <<8 | ip_protocol[tx_id][0];
		 osSemaphoreWait(net_semaphore_id,0);
		if(ip_protocol[tx_id][2]==ORG_IREF_NORMAL)
		{
			int decompsize=Rle_Decode(&ip_protocol[tx_id][5],size,outbuf,2000);
			if(tx_id==0)  serial_xor(&ip_protocol[MAX_IP_NUMBER-1][5],outbuf,ip_protocol_xor,decompsize);
			else serial_xor(&ip_protocol[tx_id-1][5],outbuf,ip_protocol_xor,decompsize);
			ip_protocol[tx_id][0]=decompsize ;
			ip_protocol[tx_id][1]=decompsize>>8 ;
			ip_protocol[tx_id][2]=ORG_IREF_NORMAL;
			ip_protocol[tx_id][3]=tx_id;
			ip_protocol[tx_id][4]=0;
			memcpy(&ip_protocol[rx_id][5], ip_protocol_xor, decompsize);
			size = decompsize;
		}
		ip_protocol[tx_id][4]=0;
		osSemaphoreRelease(net_semaphore_id);
	    send_ret = HAL_USB_NetDeviceSend(size,&ip_protocol[tx_id][5]);
		DLOG_Critical("rxid=%d txid=%d",rx_id,tx_id);
	    if(HAL_OK != send_ret)
	    {
	         DLOG_Error("NetDeviceSend fail");
	    }
		tx_id = (tx_id +1)% MAX_IP_NUMBER;
		
	}
	*/
}
static void netsessionSend_task(void const *argument)
{
	unsigned char temp_data[MAX_NET_LEN]={0};
	int i=0;
	static int k=0;
    while(1)
    {
        process_push2net();
        HAL_Delay(5);
    }
}

static void clear_all_ref_frame(){
	uint8_t   data_buf_proc[10];
	data_buf_proc[0]=CLR_ALL_TABLE;
	data_buf_proc[1]=0;
	HAL_BB_ComSendMsg(NET_COM,data_buf_proc,2);
}

static int check_ref_frame_exist(int id){
	int i=0;
	int have =0;
	for(i=0;i<MAX_TX_BUF_DEEP;i++)
	{
		uint32_t size = net_ref_packets[i][1]<<8 ||net_ref_packets[i][0];
		if(net_ref_packets[i][4]==id && size>0){
			have =1;
			break;
		}
	}
	return have;
}

static void decode_frame( uint8_t *si,uint32_t len,uint8_t is_plus,uint8_t refid){

	uint8_t i=0;
	uint8_t j=0;
	int k=0;
	int m=0;
	int n=0;
	int size =0;
	int size2 =0;
	// DLOG_Error("1 ");
	//plot_msg(si,len);

	//1 split frame
	for(i=0;i<MAX_FRAME;)
	{
		memset(dec_net_packets[i], 0, MAX_NET_LEN);
		for(n=0;n<len;)
		{
			if(si[k+n])
			{
				dec_net_packets[i][n]=si[k+n];
				n++;
			}
			else
			{
				n++;
				break;
			}
		}
		k+=n;
		i++;
		if(k >= len) break;
	}
decode:
		
		n=0;
		memset(so, 0, MAX_NET_LEN);
		for(n=0;n<i;n++)
		{
			int lin=0;
			while(1){
				if(dec_net_packets[n][lin]) lin++;
				else break;
			}
			//DLOG_Error("2 ");
			if(lin==0) continue;
			//plot_msg(dec_net_packets[n],lin);
			decode_find_non_zero_plus(dec_net_packets[n],lin,so,&size);
			//size = net_ref_packets[refid][1]<<8 | net_ref_packets[refid][0];
			
			if(n==0){
				serial_xor(&net_ref_packets[refid][5],so,protocol_xor[n],size);
			}else{
				serial_xor(protocol_xor[n-1],so,protocol_xor[n],size);
			}
			//DLOG_Error("3 ");
			//plot_msg(protocol_xor[n],size);
			if(size<60) {
				 DLOG_Error("--0 i=%d rxid=%d",i,is_plus);
				 plot_msg(si,len);
			     HAL_Delay(10);
				 DLOG_Error("--1 i=%d",i);
				 plot_msg(dec_net_packets[n],lin);
				 HAL_Delay(10);
				 DLOG_Error("--2");
				 plot_msg(so,lin);
				 HAL_Delay(10);
				 DLOG_Error("--3 rfid=%d",refid);
				 plot_msg(&net_ref_packets[refid][5],size);
				 HAL_Delay(10);
				 DLOG_Error("--4 n=%d",n);
				 plot_msg(protocol_xor[n],size);
				// continue;
			}
			HAL_RET_T ret = HAL_USB_NetDeviceSend(size,protocol_xor[n]);
			if(HAL_OK != ret)
			{
				 DLOG_Error("NetDeviceSend fail");
			}
		}
}

static void rcvDataHandler(void *p)
{
    /* receive from repeater ground SPI send to IP Camera */
    uint8_t   data_buf_proc[MAX_NET_LEN];// 1500 + 6 + 6 + 2 + 4 = 1518
    uint32_t  u32_rcvLen = 0;
	HAL_RET_T ret = HAL_BB_ComReceiveMsg(NET_COM, data_buf_proc, MAX_NET_LEN, &u32_rcvLen);
	if(ret == HAL_OK && u32_rcvLen >0)
	{
		uint8_t cmd = data_buf_proc[0];
		rx_id = data_buf_proc[1];
		uint8_t net_ref_id = data_buf_proc[2];
		
		//if(rx_id>MAX_IP_NUMBER) return;
		//if(ip_protocol[rx_id][4]==1) return;
		 //osSemaphoreWait(net_semaphore_id,0);
		//ip_protocol[rx_id][0]=u32_rcvLen ;
		//ip_protocol[rx_id][1]=u32_rcvLen>>8 ;
		//memcpy(&ip_protocol[rx_id][2], data_buf_proc, u32_rcvLen);
		//ip_protocol[rx_id][4]=1;
		//osSemaphoreRelease(net_semaphore_id);
		//HAL_BB_ComSendMsg(NET_COM,data_buf_proc,2);
		//DLOG_Critical("sky get rxid=%d txid=%d",rx_id,tx_id);
		//DLOG_Critical("sky get rxid=%d cmd=%2x size=%d,net_ref_id=%d",rx_id,cmd,u32_rcvLen-3,net_ref_id);
		if(ORG_FRAME==cmd)
		{
			ret = HAL_USB_NetDeviceSend(u32_rcvLen-3,&data_buf_proc[3]);
		    if(HAL_OK != ret)
		    {
		         DLOG_Error("NetDeviceSend fail");
		    }
		}
		else if(ORG_REF_FRAME==cmd){
			memcpy(&net_ref_packets[net_ref_id][2], &data_buf_proc,u32_rcvLen);
			HAL_BB_ComSendMsg(NET_COM,data_buf_proc,u32_rcvLen);
			net_ref_packets[net_ref_id][0] =u32_rcvLen-3;
			net_ref_packets[net_ref_id][1] =(u32_rcvLen-3)<<8;
			plot_msg(net_ref_packets[net_ref_id],u32_rcvLen);
			ret = HAL_USB_NetDeviceSend(u32_rcvLen-3,&data_buf_proc[3]);
		    if(HAL_OK != ret){
		         DLOG_Error("NetDeviceSend fail");
		    }
			
		}
		else if(REF_ENCODE_NORMAL==cmd)
		{
			int check=check_ref_frame_exist(net_ref_id);
			if(check)
				decode_frame(&data_buf_proc[3],u32_rcvLen-3,rx_id,net_ref_id);
			else {
				data_buf_proc[0]=CLR_TABLE;
				HAL_BB_ComSendMsg(NET_COM,data_buf_proc,u32_rcvLen);
			}
		}
		else if(REF_DATA==cmd)
		{
			uint8_t len = net_ref_packets[net_ref_id][1]<<8 | net_ref_packets[net_ref_id][0];
			ret = HAL_USB_NetDeviceSend(len,&net_ref_packets[net_ref_id][5]);
		    if(HAL_OK != ret){
		         DLOG_Error("NetDeviceSend fail");
		    }
		}
	}
}

void repeater_sky_input(void *data, uint32_t size)
{
    HAL_SRAM_IPCameraPassThrough(data, size, HAL_SRAM_VIDEO_CHANNEL_1);
	#if 0
	static int ti =0;
	static uint8_t buffer1[1600]={0};
	static uint8_t bufferout[1600]={0};
	int sizer=0;
	int sizerorg=0;
	int i=0;
	ti++;
	if(ti==100){
		serial_xor((uint8_t*)data,buffer1,bufferout,size);
		memcpy(buffer1,data, size);
		for(i=0;i<size;i++) if(((uint8_t*)data)[i]) sizerorg++;
		for(i=0;i<size;i++) if(bufferout[i]) sizer++;
		 DLOG_Error("sizerorg=%d sizer=%d,",sizerorg,sizer);
		//plot_msg(data,size);
		ti=0;
	}
	#endif
}


void command_TestNetRepeaterSky( void )
{

    HAL_USB_NetDeviceUp(ENUM_USB_NETCARD_PROMISCUOUS_MODE);
    HAL_USB_NetDeviceRecv(repeater_sky_input);

    HAL_SRAM_EnableSkyBypassVideo(HAL_SRAM_VIDEO_CHANNEL_1);

	 osThreadDef(netsessionSend_Task, netsessionSend_task, osPriorityHigh, 0, 16 * 128);
    osThreadCreate(osThread(netsessionSend_Task), NULL);

	osSemaphoreDef(net_send_semaphore);
    net_semaphore_id = osSemaphoreCreate(osSemaphore(net_send_semaphore),1);

    if (HAL_OK != HAL_BB_ComRegisterSession(NET_COM, BB_COM_SESSION_PRIORITY_LOW, BB_COM_SESSION_DATA_NORMAL, rcvDataHandler))
    {
        DLOG_Error("Fail register session %d", NET_COM);
    }
	  DLOG_Critical("Ok register session %d", NET_COM);
	  clear_all_ref_frame();
}



