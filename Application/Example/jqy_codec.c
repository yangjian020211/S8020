#include "jqy_codec.h"
//-----------------------------------codec------------------------------------------------
#define MAX_BUF_SIZE_TEMP 2000
static  unsigned char dec_need_buf[MAX_BUF_SIZE_TEMP]={0};
static  unsigned char dec_so[MAX_BUF_SIZE_TEMP]={0};
static  unsigned char dec_frame[MAX_BUF_SIZE_TEMP]={0};
static  unsigned char dec_preframe[MAX_BUF_SIZE_TEMP]={0};
static  unsigned char dec_xor_frame[MAX_BUF_SIZE_TEMP]={0};
static  unsigned char pre_dec_union_frame[MAX_BUF_SIZE_TEMP]={0};

static  unsigned char enc_encout_frame[MAX_BUF_SIZE_TEMP]={0};
static  unsigned char enc_xor_frame[MAX_BUF_SIZE_TEMP]={0};
static  unsigned char enc_frame[MAX_BUF_SIZE_TEMP]={0};
static  unsigned char enc_union_frame[MAX_BUF_SIZE_TEMP]={0};
static  unsigned char preenc_union_frame[MAX_BUF_SIZE_TEMP]={0};

void codec_init(void* arg){
	jqy_codec_t* codec = (jqy_codec_t*)arg;
	context_t* trcontext = codec->trcontext;
	int ok=0,ok1=0,ok2=0,ok3=0;
	int i=0,j=0;
	if(trcontext->max_buffer_cnt>0){
		
		for(i=0;i<MAX_BUF_CNT;i++){
			//memset(&trcontext->buf[i], 0, sizeof(struct _jqyring_buf));
			trcontext->buf[i].max_buffer_cnt=trcontext->max_buffer_cnt;
			trcontext->buf[i].bufferid=i;
			trcontext->buf[i].buffer_offset=0;
			for(j=0;j<MAX_BUFFER;j++){
				trcontext->buf[i].buffer_size[j]=0;
			}
			trcontext->buf[i].rd_ptr=trcontext->buf[i].buffer_offset;
			trcontext->buf[i].wr_ptr=trcontext->buf[i].buffer_offset;
		}
	}
	//initi spec
	trcontext->buf[ENC_REF_BUF].buffer_offset=2;
	trcontext->buf[ENC_REF_BUF].rd_ptr=trcontext->buf[ENC_REF_BUF].buffer_offset;
	trcontext->buf[ENC_REF_BUF].wr_ptr=trcontext->buf[ENC_REF_BUF].buffer_offset;

	trcontext->buf[DEC_REF_BUF].buffer_offset=2;
	trcontext->buf[DEC_REF_BUF].rd_ptr=trcontext->buf[DEC_REF_BUF].buffer_offset;
	trcontext->buf[DEC_REF_BUF].wr_ptr=trcontext->buf[DEC_REF_BUF].buffer_offset;

	trcontext->buf[ENC_INPUT_BUF].buffer_offset=2;
	trcontext->buf[ENC_INPUT_BUF].rd_ptr=trcontext->buf[ENC_INPUT_BUF].buffer_offset;
	trcontext->buf[ENC_INPUT_BUF].wr_ptr=trcontext->buf[ENC_INPUT_BUF].buffer_offset;

	trcontext->buf[DEC_INPUT_BUF].buffer_offset=2;
	trcontext->buf[DEC_INPUT_BUF].rd_ptr=trcontext->buf[DEC_INPUT_BUF].buffer_offset;
	trcontext->buf[DEC_INPUT_BUF].wr_ptr=trcontext->buf[DEC_INPUT_BUF].buffer_offset;
}
void codec_start(void    * arg){
	jqy_codec_t* codec = (jqy_codec_t*)arg;
	codec->trcontext->can_run=1;
}
void codec_stop(void   * arg){
	jqy_codec_t* codec = (jqy_codec_t*)arg;
	codec->trcontext->is_stop=1;
}
void codec_pause(void    * arg){
	jqy_codec_t* codec = (jqy_codec_t*)arg;
	codec->trcontext->can_run=0;
}
static void codec_modify_buf_rd(void const *arg){
	jqyring_buf_t* buf = (jqyring_buf_t*)arg;
	buf->rd_ptr = (buf->rd_ptr+1)%buf->max_buffer_cnt;
	if(buf->rd_ptr==0) buf->rd_ptr = buf->buffer_offset;
}
static void codec_modify_buf_wr(void const *arg){
	jqyring_buf_t* buf = (jqyring_buf_t*)arg;
	buf->wr_ptr = (buf->wr_ptr+1)%buf->max_buffer_cnt;
	if(buf->wr_ptr==0) buf->wr_ptr = buf->buffer_offset;
}
static int codec_check_buf_bussy(void const *arg){
	jqyring_buf_t* buf = (jqyring_buf_t*)arg;
	 if (buf->rd_ptr >=buf->max_buffer_cnt  || buf->wr_ptr >=buf->max_buffer_cnt){
	       printf("codec_session_send_task:race condition\n");
		   return 1;
	    }
	 return 0;
}

unsigned int codec_get_buf_cnt(void* arg1,void* arg2)
{
	jqy_codec_t* codec = (jqy_codec_t*)arg1;
	jqyring_buf_t* buf = (jqyring_buf_t*)arg2;
	
	unsigned int rd_idx = buf->rd_ptr;
	unsigned int wr_idx =  buf->wr_ptr;
	unsigned int cnt=0;
	 if (wr_idx >= rd_idx)
	 {
		 cnt = wr_idx - rd_idx;
	 }
	 else
	 {
		 cnt =  buf->max_buffer_cnt - (rd_idx - wr_idx)-buf->buffer_offset;
	 }
	return cnt;
}
void codec_push_data(void* arg1,void* arg2,void* datai,unsigned int size)
{
	jqy_codec_t* codec = (jqy_codec_t*)arg1;
	jqyring_buf_t* buf = (jqyring_buf_t*)arg2;
	unsigned char *data = (unsigned char *)datai;
	unsigned int rd_idx = buf->rd_ptr;
	unsigned int wr_idx = buf->wr_ptr;
	unsigned int cnt;
	if(buf==NULL) return;
	if(size==0) return;
	if(datai==NULL)return;
	//printf("rd_idx=%2d,wr_idx=%d max_buffer_cnt=%d id=%d\n",rd_idx,wr_idx,buf->max_buffer_cnt,buf->bufferid);
	 if (rd_idx >= buf->max_buffer_cnt || wr_idx >=buf->max_buffer_cnt)
	 {
		 printf("\n codec_push_data:race condition\n");
		 return ;
	 }
	 if ( (wr_idx + 1) % buf->max_buffer_cnt == rd_idx)
	 {
		 printf("\n codec_push_data:No ring buffer id=%d!!!\n",buf->bufferid);
		 return ;
	 }
	 if (size > codec->trcontext->max_frame_size)
	 {
		 printf("\n codec_push_data:size =%d overflow !!!\n", size);
		 return ;
	 }
	
	 if (size > buf->buffer_size[wr_idx])
	 {
		 if (buf->buffer[wr_idx] == NULL)
		 {
			#if Linuxsimulation
			buf->buffer[wr_idx] = malloc(size);
			#else
			buf->buffer[wr_idx] = malloc_safe(size);
			#endif
		 }
		 else
		 {
			#if Linuxsimulation
				free(buf->buffer[wr_idx]);
				buf->buffer[wr_idx] = malloc(size);
			#else
				buf->buffer[wr_idx] = realloc_safe(buf->buffer[wr_idx], size);
			#endif
			
		 }
	 }
	
	 if (buf->buffer[wr_idx] != NULL)
	 {
	 	 
		 memcpy(buf->buffer[wr_idx], data, size);
		 buf->buffer_size[wr_idx] = size;
		 codec_modify_buf_wr(buf);
	 }
	 else
	 {
		 printf("Buff malloc failed! bufferid=%d\n",buf->bufferid);
		 #if Linuxsimulation
		 buf->buffer[wr_idx] = malloc(size);	
		 #else
		 //buf->buffer[wr_idx] = malloc_safe(size);
		 #endif
	 }

}
int codec_set_data_head_tag(void* arg1,unsigned char tag,unsigned char len){
	jqy_codec_t* codec = (jqy_codec_t*)arg1;
	if(tag>0x0f)
	{
		 printf("the tag 0x00~0x0f is used for codec itself please make sure the tag more than 0x00 to 0x0f\n");
		 int error = ERROR_TAG_VALUE_INVALID;
		 codec->codec_error_callback(&error);
		 return 0;
	}
	else 
	{
		codec->trcontext->data_tag.tag[tag][1]=len;
		codec->trcontext->data_tag.size++;
		return 1;
	}
}
void codec_set_max_bag_thd(void* arg1,unsigned int thd){
	jqy_codec_t* codec = (jqy_codec_t*) arg1;
	codec->trcontext->max_bag_thd=thd;
}

void codec_set_max_buf_cnt(void* arg1,unsigned int arg2){
	jqy_codec_t* codec = (jqy_codec_t*) arg1;
	codec->trcontext->max_buffer_cnt=arg2;
}
void codec_set_union_max_cnt(void* arg1,unsigned char arg2){
	jqy_codec_t* codec = (jqy_codec_t*) arg1;
	codec->trcontext->max_union_packege=arg2;
}

void codec_set_buf_max_cnt(void* arg1,void* arg2,unsigned int arg3){
	jqy_codec_t* codec = (jqy_codec_t*) arg1;
	jqyring_buf_t* buf = (jqyring_buf_t*)arg2;
	buf->max_buffer_cnt=arg3;
}

void codec_set_max_buf_size(void* arg1,unsigned int arg2){
	jqy_codec_t* codec = (jqy_codec_t*) arg1;
	codec->trcontext->max_frame_size=arg2;
}
void codec_set_max_refbuf_thd(void* arg1,unsigned int argl,unsigned int argh){
	jqy_codec_t* codec = (jqy_codec_t*) arg1;
	codec->trcontext->max_ref_frame_thd[0]=argl;
	codec->trcontext->max_ref_frame_thd[1]=argh;
}
void codec_set_update_refbuf_time(void* arg1,unsigned int arg2){
	jqy_codec_t* codec = (jqy_codec_t*) arg1;
	codec->trcontext->update_ref_time = arg2;
}
//*******************************core********************************************************************8
static void codec_serial_xor(unsigned char *s1,unsigned char *s2,unsigned char *so,int len ){
	int i=0;
	for(i=0;i<len;i++) so[i] = s1[i] ^ s2[i];
}
 void codec_enc_find_non_zero_plus(unsigned char *si,unsigned int in_len ,unsigned char *so,unsigned int *len_out)
{
		#define debug1 0
		int j=-1;
		unsigned int i=0;
		unsigned int n=0;
		unsigned int addr=0;
		unsigned int zero_cnt=0;
		int n_is_15=0;
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
						so[j]=0;
						n=0;
					}
					else 
					{
						
						if(zero_cnt>14)
						{
							j++;
							addr=j;
							so[addr]= 0xf0 | (zero_cnt%15); 
							j++;
							addr=j;
							so[addr]= 0x00;
							
						}
						else if(zero_cnt>0)
						{
							if(n_is_15==0)j++;
							addr=j;
							so[addr] = (zero_cnt)<<4;
						}
						
					}
					zero_cnt=0;
					j++;
					state = 1;
				}
				else
				{
					//n=0;
					state =4;
				}
		#if debug1
				printf("0 n=%d i=%d si[%d]=%x,j=%d,sj[%d]=%2x addr=%d sj[%d]=%2x zero_cnt=%d\n",n,i,i,si[i],j,j,so[j],addr,addr,so[addr],zero_cnt);
		#endif
			}
			else if(state==1)
			{
				n++;
				so[j]=si[i];
				if(n==15)
				{
					n=0;
					state =2;
					n_is_15=1;
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
					n_is_15=0;
				}
		#if debug1
					 printf("1 n=%d i=%d si[%d]=%x,j=%d,sj[%d]=%2x addr=%d sj[%d]=%2x zero_cnt=%d\n",n,i,i,si[i],j,j,so[j],addr,addr,so[addr],zero_cnt);
		#endif
	
			}
			else if(state==2)
			{
				so[addr] |= 0x0f;
		#if debug1
					printf("2.1 n=%d i=%d si[%d]=%x,j=%d,sj[%d]=%2x addr=%d sj[%d]=%2x zero_cnt=%d\n",n,i,i,si[i],j,j,so[j],addr,addr,so[addr],zero_cnt);
		#endif
				i++;
				if(i>=in_len){
					state=6;
				}
				else{
					//if(n_is_15 !=1)
					j++;
					addr =j;
					//j++;
					state=0;
				}
		#if debug1
					printf("2 n=%d i=%d si[%d]=%x,j=%d,sj[%d]=%2x addr=%d sj[%d]=%2x zero_cnt=%d\n",n,i,i,si[i],j,j,so[j],addr,addr,so[addr],zero_cnt);
		#endif
			}
			else if(state==4)
			{
				zero_cnt++; 		
				if(zero_cnt==30)
				{
					state=5;
					zero_cnt=0;
					n=0;
					
				}
				else
				{
					if(n>0)
					{
						//addr=j; 
						so[addr] +=n;
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
					printf("4 n=%d i=%d si[%d]=%x,j=%d,sj[%d]=%2x addr=%d sj[%d]=%2x zero_cnt=%d\n",n,i,i,si[i],j,j,so[j],addr,addr,so[addr],zero_cnt);
		#endif
			}
			else if(state==5)
			{
				if(n_is_15 ==1){
					addr=j; 
					so[addr]=0xFF;
					n_is_15=0;
				}else{
					j++;
					addr=j; 
					so[addr]=0xFF;
				}
				i++;
				if(i>=in_len){
					state=6;
				}
				else{
					state=0;
				}
		#if debug1
					printf("5 n=%d i=%d si[%d]=%x,j=%d,sj[%d]=%2x addr=%d sj[%d]=%2x zero_cnt=%d\n",n,i,i,si[i],j,j,so[j],addr,addr,so[addr],zero_cnt);
		#endif
			}
			else if(state==6)
			{
				if(n>0)
				{
					if(so[addr]!=0x0f)j++;
					else if(addr!=0 && zero_cnt!=0){j++;zero_cnt=0;}
					so[addr]+=n; 
					if(addr==0 && zero_cnt!=0){j++;zero_cnt=0;}
				}
				else if(zero_cnt>=15){
					if(n_is_15 !=1){j++;n_is_15=0;}
					//if(addr!=0)j++;
					addr=j;
					so[addr]=0xf0 | (zero_cnt-15);
					j++;
				}
				else if(zero_cnt>0){
					if(n_is_15 !=1){j++;n_is_15=0;}
					//else if(addr!=0)j++;
					addr=j;
					so[addr]=zero_cnt<<4;
					j++;
				}				
		#if debug1
				printf("6 n=%d i=%d si[%d]=%x,j=%d,sj[%d]=%2x addr=%d sj[%d]=%2x zero_cnt=%d\n",n,i,i,si[i],j,j,so[j],addr,addr,so[addr],zero_cnt);
		#endif
				break;	
			}
		}
		*len_out = j;

}
int codec_dec_find_non_zero_plus(unsigned char *si,int in_len,unsigned char *so,int *len_out,int max_lout){
	#define debug 0
	int i=0;
	unsigned char key;
	unsigned int j=0;
	unsigned int m=0;
	unsigned int n=0;
	unsigned int l=0;

	for(i=0;i<in_len;)
	{

		unsigned char key = si[i] & 0xf0;
		unsigned char zero_len=(si[i] & 0x0f);
		if(si[i]==0xff){
			
			memset(&so[j],0,30);
			j+=30;
			i++;
			if(j>=max_lout) return 0;
		}
		else if(key == 0xf0)
		{
			zero_len = (key>>4)  + zero_len;
			memset(&so[j],0,zero_len);
			j+=zero_len;
			if(j>=max_lout) return 0;
	#if debug
			 printf("1 n=%2x i=%2d j=%2d,m=%2d	zero_len=%2d l=%2d\n",n,i,j,m,zero_len,l);
	#endif 
			i++;
			//n=((0xf0 & si[i])>>4);
			l=(0x0f & si[i]);
	#if debug
			printf("2 n=%2x i=%2d j=%2d,m=%2d  zero_len=%2d l=%2d\n",n,i,j,m,zero_len,l);
	#endif 
			
			i++;
	#if debug
			printf("3 n=%2x i=%2d j=%2d,m=%2d  zero_len=%2d l=%2d\n",n,i,j,m,zero_len,l);
	#endif 
			for(m=0;m<l;m++) so[j+m] = si[i+m];
			j+=l;
			i+=l;
			if(j>=max_lout) return 0;
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
			if(j>=max_lout) return 0;
	#if debug
			 printf("6 n=%2x i=%2d j=%2d,m=%2d l=%2d\n",n,i,j,m,l);
	#endif

			i++;
			for(m=0;m<l;m++) so[j+m] = si[i+m];
			j+=l;
			i+=l;
			if(j>=max_lout) return 0;
	#if debug
			 printf("7 i=%2d j=%2d,m=%2d l=%2d\n",i,j,m,l);
	#endif
		}
	
	}
	*len_out = j;

	return 1;

}
static int codec_devide2_sort(unsigned char *si,int len,unsigned char *so,unsigned int* lout,int maxlen){

	int i=0;
	int j=0;
	unsigned char* sortH =(unsigned char*)malloc(len);
	unsigned char* sortL =(unsigned char*)malloc(len);
	if(len==2)
	{
		if(si[0]>si[1]){ so[* lout]=si[0];so[*lout+1]=si[1];}
		else { so[*lout]=si[1];so[*lout+1]=si[0];}
		*lout+=2;
		/*
		for(i=0;i<*lout;i++)
		{
			printf("%2x ",so[i]);
		}
		printf("\n");
		*/
		return 1;
	}
	if(*lout >=maxlen) return 1;
	for(i=0;i<len-1;)
	{
		if(si[i]>si[i+1]) { sortH[j]=si[i];sortL[j]=si[i+1]; }
		else { sortH[j]=si[i+1];sortL[j]=si[i];}
		i+=2;
		j++;
	}
	codec_devide2_sort(sortH,len/2,so,lout,maxlen);
	codec_devide2_sort(sortL,len/2,so,lout,maxlen);
	
	if(sortH!=NULL)free(sortH);
	if(sortL!=NULL)free(sortL);
	return 1;
}
static void codec_devide2_desort(unsigned char *si,int len,unsigned char *so){
	

}
static int check_is_sort(unsigned char *si,int len){
	int i=0;
	for(i=0;i<len-1;i++){
		if(si[i]<si[i+1]) return 0;
	}
	return 1;
}
static void codec_do_finanlly_encode(unsigned char *si,int len,unsigned char *sot,unsigned char *times,unsigned char *so){
	static int lout=0;
	int i=0;
	unsigned char sit[128]={0};
	static unsigned char soo[128]={0};
	codec_devide2_sort(si,len,sot,&lout,len);
	if(check_is_sort(si,len-*times))
	{
		//printf("------------0\n");
		memcpy(&so[*times], sot, len-*times);
		return;
	}
	else
	{
		soo[*times]=sot[0];
		*times=*times+1;
		lout=0;
		memset(sit, 0, len);
		
		printf("---sot\n");
		for(i=0;i<len;i++){
			printf("%2x ",sot[i]);
		}
		printf("\n---so\n");
		for(i=0;i<len;i++){
			printf("%2x ",soo[i]);
		}
		printf("\n");
		
		memcpy(sit,&sot[1],len-*times);
		memset(sot, 0, len);
		
		if(check_is_sort(sit,len-*times))
		{
			memcpy(so, soo, *times);
			memcpy(&so[*times], sit, len-*times);
			memset(soo, 0, 128);
			printf("do loop again times=%d\n",*times);
			printf("--so\n");
			for(i=0;i<len;i++){
				printf("%2x ",so[i]);
			}
			printf("\n");
			
			return;
		}
		codec_do_finanlly_encode(sit,len,sot,times,so);
	}
}
static void codec_do_finanlly_decode(unsigned char *si,int len,unsigned char times,unsigned char *so){
	
	int i=0;
	unsigned char sit[128]={0};
	static unsigned char soo[128]={0};
	memset(sit, 0, len);
	memcpy(so, si, len-times);
	for(i=0;i<times;i++)
	{
		sit[0] = si[i];
		memcpy(&sit[1], so,len-1);
		codec_devide2_desort(sit,len,so);
	}
}
void codec_sort_core(unsigned char *si,int len,unsigned char *so,unsigned char *htimes,unsigned char *ltimes){
	if(len !=4 && len!=8 && len!=16 && len!=32 && len !=64 && len !=128  && len !=256) return;
	int i=0;
	static unsigned char siH[64]={0};
	static unsigned char siL[64]={0};
	static unsigned char soH[64]={0};
	static unsigned char soL[64]={0};
	static unsigned char soHR[64]={0};
	static unsigned char soLR[64]={0};
	
	for(i=0;i<len;i++) siH[i] = (si[i] & 0xf0)>>4;
	for(i=0;i<len;i++) siL[i] = si[i] & 0x0f;

	//
	codec_do_finanlly_encode(siH,len,soH,htimes,soHR);
	printf("---htimes=%d\n",*htimes);
	codec_do_finanlly_encode(siL,len,soL,ltimes,soLR);
	printf("---ltimes=%d\n",*ltimes);

	for(i=0;i<len;i++){
		so[i]=soHR[i]<<4;
		so[i] |=soLR[i];
	}
	
	return;
}
void codec_desort_core(unsigned char *si,int len,unsigned char *so,unsigned char htimes,unsigned char ltimes){
	if(len !=4 && len!=8 && len!=16 && len!=32 && len !=64 && len !=128  && len !=256) return;
	int i=0;
	static unsigned char siH[64]={0};
	static unsigned char siL[64]={0};
	static unsigned char soH[64]={0};
	static unsigned char soL[64]={0};
	static unsigned char soHR[64]={0};
	static unsigned char soLR[64]={0};
	memset(soHR, 0, 64);
	memset(soLR, 0, 64);
	for(i=0;i<len;i++) siH[i] = (si[i] & 0xf0)>>4;
	for(i=0;i<len;i++) siL[i] = si[i] & 0x0f;

	codec_do_finanlly_decode(siH,len,htimes,soHR);
	printf("---Hbits\n");
	codec_do_finanlly_decode(siL,len,ltimes,soLR);
	printf("---Lbits\n");

	for(i=0;i<len;i++){
		so[i]=soHR[i]<<4;
		so[i] |=soLR[i];
	}
	return;
}

//*********************************end core***************************************************************

//*********************************encode*****************************************************************
static void codec_nec_update_ref_frameid(jqy_codec_t* codec , unsigned char *si,unsigned int size)
{
	jqyring_buf_t *enc_refbuf = &codec->trcontext->buf[ENC_REF_BUF];
	memset(dec_preframe, 0, MAX_BUF_SIZE_TEMP);
	struct _ref_frame_head refhead={0};
	refhead.frequency=1;
	refhead.last_time=codec->codec_get_curtime_ms();
	memcpy(dec_preframe, &refhead,8);
	memcpy(&dec_preframe[8], si,size);
	codec_push_data(codec,enc_refbuf,dec_preframe,size+8);
	codec->codec_data_dump_callback(dec_preframe,size+8,DATA_ENC_7);
}
static int find_data_head_len(jqy_codec_t* codec,unsigned char tag){
	for(int i=0;i<codec->trcontext->data_tag.size;i++)
	{
		if(codec->trcontext->data_tag.tag[tag][0]==tag) return codec->trcontext->data_tag.tag[tag][1];
	}
	return -1;
}
static void codec_update_table_exist_ref_frame(jqy_codec_t* codec,jqyring_buf_t *inbuf,jqyring_buf_t *nec_refbuf){

	unsigned int rd_idx1 = nec_refbuf->rd_ptr;
	unsigned int wr_idx1 = nec_refbuf->wr_ptr;
	unsigned int size1 =  nec_refbuf->buffer_size[rd_idx1];

	unsigned int rd_idx2 = inbuf->rd_ptr;
	unsigned int size2 =  inbuf->buffer_size[rd_idx2];
	sync_head_t *head2 = (sync_head_t *)inbuf->buffer[rd_idx2];
	
	int cnt=0;
	int precnt=20000;
	int len = find_data_head_len(codec,head2->usser_type);
	
	//1 update the ref_buf to find the have ,eq, rid param
	memset(enc_frame, 0, MAX_BUF_SIZE_TEMP);
	int tid=rd_idx1;
	while(tid !=wr_idx1)
	{
		//find input data type with the len
		if(len <0)
		{
			int error = ERROR_NO_DATA_TAG;
			codec->codec_error_callback(&error);
			codec->trcontext->ref_state.best_rid=0;
			codec->trcontext->ref_state.not_as_ref=1;
			return;
		}
		ref_frame_head_t *head4 = (ref_frame_head_t *)nec_refbuf->buffer[tid];
		if(memcmp(&nec_refbuf->buffer[tid][11], &inbuf->buffer[rd_idx2][3], len)==0 && head4->sync.id !=0)			
		{
			codec->trcontext->ref_state.have=1;
			memset(enc_frame, 0, MAX_BUF_SIZE_TEMP);
			codec_serial_xor(&nec_refbuf->buffer[tid][11],&inbuf->buffer[rd_idx2][3],enc_frame,size2-3);
			int i=0;
			cnt=0;
			for(i=0;i<size2-3;i++)if(enc_frame[i] > 0)cnt++;
			if(cnt==0)
			{
				codec->trcontext->ref_state.have_eq=0x01;
				codec->trcontext->ref_state.best_rid=head4->sync.id;
				codec->trcontext->ref_state.best_rid_in_table=tid;
				head4->frequency++;
				return;
			}
			else if(cnt<precnt)
			{
				codec->trcontext->ref_state.best_rid_in_table=tid;
				codec->trcontext->ref_state.best_rid=head4->sync.id;
				head4->frequency++;
				precnt = cnt;
			}

		}
		if(head4->sync.id==head2->id) codec->trcontext->ref_state.id_in_ref=1;
		tid = (tid+1)% (nec_refbuf->max_buffer_cnt);
		if(tid==0)tid=nec_refbuf->buffer_offset;
	}
}
static void codec_enc_get_ref_frame_id(jqy_codec_t* codec,jqyring_buf_t *enc_input_buf,sync_head_t *head)
{
	int size=enc_input_buf->buffer_size[enc_input_buf->rd_ptr];
	if(size >codec->trcontext->max_bag_thd){
		head->codec_type=RM_ZERO_HUGE;
		head->rid=1;
		#if Linuxsimulation 
		printf("\n----1----\n");
		#else
		//DLOG_Error("----1----");
		#endif
		return;
	}
	else if(codec->trcontext->ref_state.not_as_ref==1) {
		head->codec_type=RM_ZERO_REF;
		head->rid=1;
		#if Linuxsimulation 
		printf("\n----2----\n");
		#else
		//DLOG_Error("----2----");
		#endif
		return ;
	}
	else if(codec->trcontext->ref_state.have_eq==1) 
	{
		#if Linuxsimulation 
		printf("\n----3----\n");
		#else
		//DLOG_Error("----3----");
		#endif
		head->codec_type=RM_ZERO_REF_EQ;
		head->rid=codec->trcontext->ref_state.best_rid;
		return ;
	}
	static unsigned int  lasttime=0;
	if(codec->trcontext->ref_state.have==1)
		head->rid=codec->trcontext->ref_state.best_rid;
	else
		head->rid=1;
		unsigned int now_time = codec->codec_get_curtime_ms();
	if(now_time- lasttime > codec->trcontext->update_ref_time && codec->trcontext->ref_state.id_in_ref==0) 
	{
		codec->trcontext->ref_state.reqtimeout=1;
		head->codec_type=RM_ZERO_REF_UPDATE;
		head->rid=1;
		lasttime = now_time;
		codec->trcontext->ref_state.last_time=now_time;
	}
	else
	{
		codec->trcontext->ref_state.reqtimeout=0;
		head->codec_type=RM_ZERO_REF;
	}
	
}
static void send_eq_id(jqy_codec_t* codec,sync_head_t* head){
	
	memcpy(enc_encout_frame, head, 3);
	enc_encout_frame[3]=0;
	codec->codec_data_dump_callback(enc_encout_frame,4,DATA_ENC_6);
	codec_push_data(codec,&codec->trcontext->buf[ENC_OUTPUT_BUF],enc_encout_frame,4);
}

static void pop_union_buf(jqy_codec_t* codec,jqyring_buf_t *enc_union_buf,jqyring_buf_t *enc_refbuf)
{
	memset(enc_union_frame, 0, MAX_BUF_SIZE_TEMP);
	memset(enc_frame, 0, MAX_BUF_SIZE_TEMP);
	unsigned int rd_idx1 = enc_union_buf->rd_ptr;
	unsigned int wr_idx1 = enc_union_buf->wr_ptr;
	unsigned int size =  enc_union_buf->buffer_size[rd_idx1];
	unsigned int tid;
	unsigned char head_len=sizeof(struct _sync_head);
	unsigned int len=0;
	unsigned int currntlen=0;
	tid = rd_idx1;
	while(tid !=wr_idx1)
	{
		size =  enc_union_buf->buffer_size[tid];
		if(tid==rd_idx1)
		{
			sync_head_t *head = (sync_head_t *)enc_union_buf->buffer[tid];
			head->codec_type=RM_ZERO_REF_ORDER2;
			memcpy(enc_union_frame, head,3);
			enc_union_frame[3]=size-4;
			currntlen = 4;
			memcpy(&enc_union_frame[currntlen], &enc_union_buf->buffer[tid][3], size-3);
			currntlen=size;
			codec->codec_data_dump_callback(enc_union_frame,currntlen,94);
			memset(preenc_union_frame, 0, MAX_BUF_SIZE_TEMP);
			memcpy(preenc_union_frame, &enc_union_frame[4], size-4);
			codec->codec_data_dump_callback(preenc_union_frame,size-4,95);
		}
		else
		{
			
			codec->codec_data_dump_callback(&enc_union_buf->buffer[tid][3],size-4,96);
			codec_serial_xor(preenc_union_frame,&enc_union_buf->buffer[tid][3],enc_xor_frame,size-4);
			codec->codec_data_dump_callback(enc_xor_frame,size-4,97);
			codec_enc_find_non_zero_plus(enc_xor_frame,size-4,enc_frame,&len);
			codec->codec_data_dump_callback(enc_frame,len,98);
			enc_union_frame[currntlen]=len;
			currntlen+=1;
			memcpy(&enc_union_frame[currntlen], enc_frame, len);
			currntlen+=len;
			codec->codec_data_dump_callback(enc_frame,len,99);// 
			memset(preenc_union_frame, 0, MAX_BUF_SIZE_TEMP);
			memcpy(preenc_union_frame, &enc_union_buf->buffer[tid][3], size-4);
			codec->codec_data_dump_callback(preenc_union_frame,size-4,100);
		}
		
		
		tid = (tid+1) % (enc_union_buf->max_buffer_cnt);
		if(tid==0)tid=enc_union_buf->buffer_offset;
	}
	enc_union_frame[currntlen]=0;
	currntlen+=1;
	codec_push_data(codec,&codec->trcontext->buf[ENC_OUTPUT_BUF],enc_union_frame,currntlen);
	codec->codec_data_dump_callback(enc_union_frame,currntlen,122);// 
	enc_union_buf->rd_ptr = wr_idx1;
	codec->trcontext->enc_frame_cnt=0;
}
static int codec_xor_rm_zero( jqy_codec_t* codec,jqyring_buf_t *enc_input_buf,jqyring_buf_t *enc_union_buf,jqyring_buf_t *enc_refbuf,unsigned int enc_rd,unsigned int size)
{
	unsigned int len=0;
	int ref_cnt =0;
	int count_in_union_buf=0;
	memset(enc_encout_frame, 0, MAX_BUF_SIZE_TEMP);
	memset(enc_xor_frame, 0, MAX_BUF_SIZE_TEMP);
	memset(enc_frame, 0, MAX_BUF_SIZE_TEMP);
	if(size>MAX_BUF_SIZE_TEMP-3){
		codec_modify_buf_rd(enc_input_buf);
		codec->trcontext->pre_ref_frame_id=0;
		return 1;
	}
	int frame_len= size-3;
	struct _sync_head newh={0};
	memset(&newh, 0,3);
	memcpy(&newh,enc_input_buf->buffer[enc_rd], 3);
	newh.id= enc_rd;
	newh.rid= enc_rd;
	ref_cnt=codec_get_buf_cnt(codec,enc_refbuf);
	
	memset(&codec->trcontext->ref_state, 0 ,sizeof(struct _update_state));
	codec_update_table_exist_ref_frame(codec,enc_input_buf,enc_refbuf);
	codec_enc_get_ref_frame_id(codec,enc_input_buf,&newh);
	#if Linuxsimulation 
		printf("refbuf_cnt=%d,headr.rfid=%d,headr.id=%d,headr.ctype=%d,headr.utype=%d size=%d\n",ref_cnt,newh.rid,newh.id,newh.codec_type,newh.usser_type,size);
	#else
		//DLOG_Error("have=%d refbuf_cnt=%d,headr.rfid=%d,headr.id=%d,headr.ctype=%d,headr.utype=%d size=%d",codec->trcontext->ref_state.have,ref_cnt,newh.rid,newh.id,newh.codec_type,newh.usser_type,size);
	#endif
	if(newh.codec_type==RM_ZERO_REF_EQ)
	{
		count_in_union_buf=codec_get_buf_cnt(codec,enc_union_buf);
		if(count_in_union_buf>0) pop_union_buf(codec,enc_union_buf,enc_refbuf);
		send_eq_id(codec,&newh);
		codec_modify_buf_rd(enc_input_buf);
		codec->trcontext->pre_ref_frame_id=0;
		return 1;
	}
	else if(newh.codec_type==RM_ZERO_HUGE)
	{
		count_in_union_buf=codec_get_buf_cnt(codec,enc_union_buf);
		if(count_in_union_buf>0) pop_union_buf(codec,enc_union_buf,enc_refbuf);
		memcpy(enc_encout_frame, &newh, 3);
		memcpy(&enc_encout_frame[3],&enc_input_buf->buffer[enc_rd][3], frame_len);
		codec_push_data(codec,&codec->trcontext->buf[ENC_OUTPUT_BUF],enc_encout_frame,size);
		codec->codec_data_dump_callback(enc_encout_frame,size,DATA_ENC_5);
		codec_modify_buf_rd(enc_input_buf);
		codec->trcontext->pre_ref_frame_id=0;
		return 1;
	}
	//1 pre encode ,do xor to find diff
	//else if(ref_cnt>0 && newh.rid>1 && enc_refbuf->buffer[codec->trcontext->ref_state.best_rid_in_table]!=NULL)
	else if(ref_cnt>0 && newh.rid>1)
	{
		if(enc_refbuf->buffer_size[codec->trcontext->ref_state.best_rid_in_table] >= frame_len)
		{
			
			codec->codec_data_dump_callback(enc_refbuf->buffer[codec->trcontext->ref_state.best_rid_in_table],size+8,DATA_ENC_1);// 
			codec_serial_xor(&enc_refbuf->buffer[codec->trcontext->ref_state.best_rid_in_table][11],&enc_input_buf->buffer[enc_rd][3],enc_xor_frame,frame_len);
		}
		else
		{
			newh.codec_type=RM_ZERO_REF;
			memcpy(enc_xor_frame,&enc_input_buf->buffer[enc_rd][3], frame_len);
		}
	}
	else 
	{
		//newh.codec_type=RM_ZERO_REF;
		memcpy(enc_xor_frame,&enc_input_buf->buffer[enc_rd][3], frame_len);
	}
	//2 do enc
	codec->codec_data_dump_callback(enc_xor_frame,frame_len,DATA_ENC_2);// 
	if(size-3 >MAX_BUF_SIZE_TEMP)
	{
		codec_modify_buf_rd(enc_input_buf);
		codec->trcontext->pre_ref_frame_id=0;
		return 1;
	}
	
	codec_enc_find_non_zero_plus(enc_xor_frame,frame_len,enc_frame,&len);

	#if 1
	unsigned char buf[2000]={0};
	int ltest=0;
	int j=0;
	
	//codec_dec_find_non_zero_plus(enc_frame,len,buf,&ltest,MAX_BUF_SIZE_TEMP);

	for(j=0;j<len;j++) if(enc_frame[j]==0 && j<len){
		codec->codec_data_dump_callback(enc_xor_frame,frame_len,500);// 
		codec->codec_data_dump_callback(enc_frame,len,501);// 
	}
	#endif
	
	codec->codec_data_dump_callback(enc_frame,len,DATA_ENC_3);// 
	//3 add head
	memcpy(enc_encout_frame, &newh, 3);
	//4 copy head to buffer
	memcpy(&enc_encout_frame[3],enc_frame,len);
	
	//5 add end tag to frame end
	enc_encout_frame[len+3]=0;
	//6 push to enc_output_buffer
	if(len+4 >MAX_BUF_SIZE_TEMP)
	{
		//printf("\n----len=%d----\n",len);
		codec_modify_buf_rd(enc_input_buf);
		codec->trcontext->pre_ref_frame_id=0;
		return 1;
	}
	if(codec->trcontext->pre_ref_frame_id==newh.rid)
	{
		codec_push_data(codec,&codec->trcontext->buf[ENC_UNION_BUF],enc_encout_frame,len+4);
		codec->codec_data_dump_callback(enc_encout_frame,len+4,DATA_ENC_4);// 
		codec->trcontext->enc_frame_cnt++;
		codec_modify_buf_rd(enc_input_buf);
		if(codec->trcontext->enc_frame_cnt >=codec->trcontext->max_union_packege)
		{
			pop_union_buf(codec,enc_union_buf,enc_refbuf);
		}
	}
	else
	{
		count_in_union_buf=codec_get_buf_cnt(codec,enc_union_buf);
		if(count_in_union_buf>0) pop_union_buf(codec,enc_union_buf,enc_refbuf);
		//printf("\n--rid=%d--\n",newh.rid);
		if(newh.rid ==1)
		{
			
			codec_push_data(codec,&codec->trcontext->buf[ENC_OUTPUT_BUF],enc_encout_frame,len+4);
			codec->codec_data_dump_callback(enc_encout_frame,len+4,DATA_ENC_4);// 
			codec->trcontext->pre_ref_frame_id=0;
			codec->trcontext->enc_frame_cnt=0;
		}
		else if(newh.rid >1)
		{
			codec_push_data(codec,&codec->trcontext->buf[ENC_UNION_BUF],enc_encout_frame,len+4);
			codec->trcontext->pre_ref_frame_id=newh.rid;
			codec->codec_data_dump_callback(enc_encout_frame,len+4,DATA_ENC_4);// 
			codec->trcontext->enc_frame_cnt++;
		}
		codec_modify_buf_rd(enc_input_buf);
	}
	return 1;	
}
void codec_enc_task(void const *argument)
{
	jqy_codec_t* codec = (jqy_codec_t*) argument;
	jqyring_buf_t *enc_input_buf = &codec->trcontext->buf[ENC_INPUT_BUF];
	jqyring_buf_t *enc_refbuf = &codec->trcontext->buf[ENC_REF_BUF];
	jqyring_buf_t *enc_union_buf = &codec->trcontext->buf[ENC_UNION_BUF];
	while(1)
    {
    	if(0==codec->trcontext->can_run)
		{
			codec->codec_delayms(1000);
			if(codec->trcontext->is_stop)break;
			continue;
    	}
		if(codec_check_buf_bussy(enc_input_buf))
		{
			codec->codec_delayms(5);
            continue;
		}
		else if(enc_input_buf->rd_ptr !=enc_input_buf->wr_ptr)
		{
			unsigned int size =  enc_input_buf->buffer_size[enc_input_buf->rd_ptr];
			sync_head_t *head = (sync_head_t *)enc_input_buf->buffer[enc_input_buf->rd_ptr];
			head->id=enc_input_buf->rd_ptr;
			codec->codec_data_dump_callback(enc_input_buf->buffer[enc_input_buf->rd_ptr],size,DATA_ENC_0);//
			codec_xor_rm_zero(codec,enc_input_buf,enc_union_buf,enc_refbuf,enc_input_buf->rd_ptr,size);
			int ref_buf_cnt = codec_get_buf_cnt(codec,enc_refbuf);
			if(ref_buf_cnt>=enc_refbuf->max_buffer_cnt){
				enc_refbuf->rd_ptr = (enc_refbuf->rd_ptr+1)%enc_refbuf->max_buffer_cnt;
				if(enc_refbuf->rd_ptr==0) enc_refbuf->rd_ptr=enc_refbuf->buffer_offset;
			}
		}
        else codec->codec_delayms(5);
    }
}
//***********************************end encode************************************************************

//************************************decode***************************************************************
static int codec_find_sync_head(unsigned char *si,unsigned int *size){
	for(int i=0;i<*size;i++){
		if(si[i]){
			*size -=i;
			return i;
		}
	}
	return -1;
}
static int find_ref_id_by_rid(jqy_codec_t* codec,int rid){
	jqyring_buf_t *decrefbuf = &codec->trcontext->buf[DEC_REF_BUF];
	int tid=decrefbuf->rd_ptr;
	int ref_cnt = codec_get_buf_cnt(codec,decrefbuf);	
	while(tid != decrefbuf->wr_ptr){
		ref_frame_head_t *head2 = (ref_frame_head_t *)decrefbuf->buffer[tid];
		codec->codec_data_dump_callback(decrefbuf->buffer[tid],decrefbuf->buffer_size[tid],101);//
		//printf("%d  ----  %d tid=%d \n",head2->sync.id,rid,tid);
		if(head2->sync.id==rid && head2->sync.id>0){
			return tid;
		}
		tid = (tid + 1) % (decrefbuf->max_buffer_cnt);
		if(tid==0)tid=decrefbuf->buffer_offset;
	}
	 return 0;
}
static void codec_dec_update_ref_frameid(jqy_codec_t* codec ,unsigned char *si,unsigned int size)
{
	jqyring_buf_t *decrefbuf =&codec->trcontext->buf[DEC_REF_BUF];
	int ref_buf_cnt = codec_get_buf_cnt(codec, decrefbuf);
	if(ref_buf_cnt>=decrefbuf->max_buffer_cnt){
		decrefbuf->rd_ptr = (decrefbuf->rd_ptr+1)%decrefbuf->max_buffer_cnt;
		if(decrefbuf->rd_ptr==0) decrefbuf->rd_ptr=decrefbuf->buffer_offset;
	}

	sync_head_t *head_new = (sync_head_t *)si;
	memset(dec_preframe, 0, MAX_BUF_SIZE_TEMP);
	memcpy(&dec_preframe[8], si, size);
	struct _ref_frame_head refhead={0};
	refhead.frequency=1;
	refhead.last_time=codec->codec_get_curtime_ms();
	refhead.sync.codec_type= head_new->codec_type;
	refhead.sync.id=head_new->id;
	refhead.sync.rid=head_new->rid;
	refhead.sync.usser_type=head_new->usser_type;
	memcpy(dec_preframe, &refhead,11);
	codec_push_data(codec,decrefbuf,dec_preframe,size+8);
	codec->codec_data_dump_callback(dec_preframe,size+8,DATA_DEC_28);
}
static void do_union_frame_decode(jqy_codec_t* codec,jqyring_buf_t *dec_refbuf)
{
	jqyring_buf_t *dec_union_buf= &codec->trcontext->buf[DEC_UNION_BUF];
	unsigned int rd_idx1 = dec_union_buf->rd_ptr;
	unsigned int wr_idx1 = dec_union_buf->wr_ptr;
	unsigned int size =  dec_union_buf->buffer_size[rd_idx1];
	int tid = rd_idx1;
	int len=0;
	while(tid !=wr_idx1)
	{
		memset(dec_frame, 0, MAX_BUF_SIZE_TEMP);
		size =  dec_union_buf->buffer_size[tid];
		sync_head_t *head = (sync_head_t *)dec_union_buf->buffer[tid];
		codec->codec_data_dump_callback(&dec_union_buf->buffer[tid][3],size-3,152);
		codec_dec_find_non_zero_plus(&dec_union_buf->buffer[tid][3],size-3,dec_preframe,&len,MAX_BUF_SIZE_TEMP);
		codec->codec_data_dump_callback(dec_preframe,len,151);
		if(tid==rd_idx1)
		{
			memset(pre_dec_union_frame, 0, MAX_BUF_SIZE_TEMP);
			memcpy(pre_dec_union_frame, &dec_union_buf->buffer[tid][3], size-3);
			codec->codec_data_dump_callback(pre_dec_union_frame,size-3,153);
			int id_inrefbuf = find_ref_id_by_rid(codec,head->rid);
			if(id_inrefbuf==0)
			{
				//have not the rid frame in the ref_buf,neet to update table
				//printf("\n 2---id=%d rid=%d \n ",head->id,head->rid);
				int error = ERROR_FRAME_NOT_IN_REFBUF;
				codec->codec_error_callback(&error);
				struct _sync_head head2 ={0};
				memcpy(&head2, head, 3);
				head2.codec_type=ACK_RM_ZERO_REF_HAVE_NOT;
				unsigned char buf[10]={0};
				memcpy(buf, &head2, 3);
				buf[3]=0;
				codec_push_data(codec,&codec->trcontext->buf[DEC_OUTPUT_BUF],buf,4);
				codec->codec_data_dump_callback(buf,4,DATA_DEC_35);//
				return ;
			}
			else
			{
				//if(len > dec_refbuf->buffer_size[id_inrefbuf]) return ;
				codec_serial_xor(&dec_refbuf->buffer[id_inrefbuf][11],dec_preframe,dec_xor_frame,len);
				codec->codec_data_dump_callback(dec_refbuf->buffer[id_inrefbuf],len,DATA_DEC_36);
			}
			memcpy(dec_frame,head,3);
			memcpy(&dec_frame[3],dec_xor_frame,len);
			codec_push_data(codec,&codec->trcontext->buf[DEC_OUTPUT_BUF],dec_frame,len+3);
			codec->codec_data_dump_callback(dec_frame,3+len,DATA_DEC_40);
		}
		else
		{
			codec->codec_data_dump_callback(dec_union_buf->buffer[tid],size,149);
			memset(dec_xor_frame, 0, MAX_BUF_SIZE_TEMP);
			codec_serial_xor(pre_dec_union_frame,dec_preframe,dec_xor_frame,len);
			codec->codec_data_dump_callback(dec_xor_frame,len,154);
			memset(pre_dec_union_frame, 0, MAX_BUF_SIZE_TEMP);
			memcpy(pre_dec_union_frame, dec_xor_frame, len);
			codec->codec_data_dump_callback(pre_dec_union_frame,len,155);
			int len2 = len;
			int len3=0;
			memset(dec_preframe, 0, MAX_BUF_SIZE_TEMP);
			codec_dec_find_non_zero_plus(dec_xor_frame,len2,dec_preframe,&len3,MAX_BUF_SIZE_TEMP);
			codec->codec_data_dump_callback(dec_preframe,len3,156);
			int id_inrefbuf = find_ref_id_by_rid(codec,head->rid);
			if(id_inrefbuf==0)
			{
				//have not the rid frame in the ref_buf,neet to update table
				//printf("\n 3---id=%d rid=%d \n ",head->id,head->rid);
				int error = ERROR_FRAME_NOT_IN_REFBUF;
				codec->codec_error_callback(&error);	
				struct _sync_head head2 ={0};
				memcpy(&head2, head, 3);
				head2.codec_type=ACK_RM_ZERO_REF_HAVE_NOT;
				unsigned char buf[10]={0};
				memcpy(buf, &head2, 3);
				buf[3]=0;
				codec_push_data(codec,&codec->trcontext->buf[DEC_OUTPUT_BUF],buf,4);
				codec->codec_data_dump_callback(buf,4,DATA_DEC_35);//
				return ;
			}
			else
			{
				//if(len > dec_refbuf->buffer_size[id_inrefbuf]) return 0;
				memset(dec_xor_frame, 0, MAX_BUF_SIZE_TEMP);
				codec_serial_xor(&dec_refbuf->buffer[id_inrefbuf][11],dec_preframe,dec_xor_frame,len3);
				codec->codec_data_dump_callback(dec_refbuf->buffer[id_inrefbuf],len3,DATA_DEC_42);
			}
			memcpy(dec_frame,head,3);
			memcpy(&dec_frame[3],dec_xor_frame,len3);
			codec_push_data(codec,&codec->trcontext->buf[DEC_OUTPUT_BUF],dec_frame,len3+3);
			codec->codec_data_dump_callback(dec_frame,3+len3,DATA_DEC_43);
		}
		tid = (tid+1) % (dec_union_buf->max_buffer_cnt);
		if(tid==0)tid=dec_union_buf->buffer_offset;
		
	}
	dec_union_buf->rd_ptr = wr_idx1;
}
static int decode_one_frame(jqy_codec_t* codec,unsigned char *si,jqyring_buf_t *dec_refbuf,unsigned int size){
	sync_head_t *head = (sync_head_t *)si;
	int insize = size-3;
	int len=0;
	memset(dec_frame, 0, MAX_BUF_SIZE_TEMP);
	memset(dec_preframe, 0, MAX_BUF_SIZE_TEMP);
	memset(dec_xor_frame, 0, MAX_BUF_SIZE_TEMP);
	
 	if(head->codec_type==RM_ZERO_REF_EQ)
	{
		int tid=dec_refbuf->rd_ptr;
		while(tid != dec_refbuf->wr_ptr)
		{
			ref_frame_head_t *head2 = (ref_frame_head_t *)dec_refbuf->buffer[tid];
			if(head->rid == head2->sync.id)
			{
				//1 copy the head to tempframe to replace the head infomation
				memcpy(dec_frame, head,3);
				//2 copy the refframe data to tempframe
				memcpy(&dec_frame[3],&dec_refbuf->buffer[tid][11],dec_refbuf->buffer_size[tid]-11);
				//3 push to output_buffer
				codec_push_data(codec,&codec->trcontext->buf[DEC_OUTPUT_BUF],dec_frame,dec_refbuf->buffer_size[tid]-8);
				//4 
				codec->codec_data_dump_callback(dec_frame,dec_refbuf->buffer_size[tid]-(8),DATA_DEC_33);//
				return 1;
			}
			tid = (tid + 1) % (dec_refbuf->max_buffer_cnt);
			if(tid==0)tid=dec_refbuf->buffer_offset;
		}
	}
	else if(head->codec_type==ACK_RM_ZERO_REF_HAVE_NOT){
		jqyring_buf_t *enc_refbuf =&codec->trcontext->buf[ENC_REF_BUF];
		enc_refbuf->rd_ptr=enc_refbuf->wr_ptr;
		//printf("ack have not ref \n");
		return 1;
	}
	else if(head->codec_type==ACK_RM_ZERO_REF){
		codec_dec_find_non_zero_plus(&si[3],insize,dec_preframe,&len,MAX_BUF_SIZE_TEMP);
		memcpy(dec_frame,head,3);
		memcpy(&dec_frame[3],dec_preframe,len);
		codec_nec_update_ref_frameid(codec,dec_frame,3+len);
		//printf("get ack rm zero ref \n");
		return 1;
	}
	else if(head->codec_type==RM_ZERO_REF_UPDATE || head->codec_type==RM_ZERO_REF)
	{	
		if(insize<0){
			int error = ERROR_FRAME_IN_FRAME_LEN_LESS_0;
			codec->codec_error_callback(&error);
			return 0;
		}
		if(insize>=MAX_BUF_SIZE_TEMP){
			
			int error = ERROR_FRAME_DEC_LENGTH_LONG_THAN2000;
			codec->codec_error_callback(&error);
			return 0;
		}
		codec->codec_data_dump_callback(si,size,DATA_DEC_36);//
		codec_dec_find_non_zero_plus(&si[3],insize,dec_preframe,&len,MAX_BUF_SIZE_TEMP);
		//codec->codec_data_dump_callback(dec_preframe,len,DATA_DEC_36);//
		if(len>=MAX_BUF_SIZE_TEMP){
			int error = ERROR_FRAME_DEC_LENGTH_LONG_THAN2000;
			codec->codec_error_callback(&error);
			return 0;
		}
		int ref_cnt = codec_get_buf_cnt(codec,dec_refbuf);
		codec->codec_data_dump_callback(dec_preframe,len,DATA_DEC_21);
		if(head->rid > 1 && ref_cnt>0)
		{
			int id_inrefbuf = find_ref_id_by_rid(codec,head->rid);
			if(id_inrefbuf==0)
			{
				//have not the rid frame in the ref_buf,neet to update table
				//printf("\n 1---id=%d rid=%d \n ",head->id,head->rid);
				int error = ERROR_FRAME_NOT_IN_REFBUF;
				codec->codec_error_callback(&error);
				
				struct _sync_head head2 ={0};
				memcpy(&head2, head, 3);
				head2.codec_type=ACK_RM_ZERO_REF_HAVE_NOT;
				unsigned char buf[100]={0};
				memcpy(buf, &head2, 3);
				buf[3]=0;
				codec_push_data(codec,&codec->trcontext->buf[DEC_OUTPUT_BUF],buf,3+1);
				codec->codec_data_dump_callback(buf,3+1,DATA_DEC_30);//
				return 0;
			}
			else
			{
				//if(len > dec_refbuf->buffer_size[id_inrefbuf]) return 0;
				codec_serial_xor(&dec_refbuf->buffer[id_inrefbuf][11],dec_preframe,dec_xor_frame,len);
				codec->codec_data_dump_callback(dec_refbuf->buffer[id_inrefbuf],len,DATA_DEC_22);
			}
		}
		else{
			memcpy(dec_xor_frame,dec_preframe,len);
		}
		
		memcpy(dec_frame,head,3);
		memcpy(&dec_frame[3],dec_xor_frame,len);
		codec->codec_data_dump_callback(dec_frame,3+len,DATA_DEC_23);
		codec_push_data(codec,&codec->trcontext->buf[DEC_OUTPUT_BUF],dec_frame,3+len);
		if(head->codec_type==RM_ZERO_REF_UPDATE)
		{
			codec->codec_data_dump_callback(dec_frame,3+len,DATA_DEC_24);//
			codec_dec_update_ref_frameid(codec,dec_frame,3+len);
			memset(dec_frame, 0, MAX_BUF_SIZE_TEMP);
			memcpy(dec_frame, si, size);
			sync_head_t* head2 =(sync_head_t*)dec_frame;
			head2->codec_type=ACK_RM_ZERO_REF;
			codec_push_data(codec,&codec->trcontext->buf[DEC_OUTPUT_BUF],dec_frame,size+1);
			codec->codec_data_dump_callback(dec_frame,size+1,DATA_DEC_25);//
		}
	}
	else if(head->codec_type==RM_ZERO_REF_ORDER2_UPDATE || head->codec_type==RM_ZERO_REF_ORDER2)
	{
		if(insize<0){
			int error = ERROR_FRAME_IN_FRAME_LEN_LESS_0;
			codec->codec_error_callback(&error);
			return 0;
		}
		if(insize>=MAX_BUF_SIZE_TEMP){
			
			int error = ERROR_FRAME_DEC_LENGTH_LONG_THAN2000;
			codec->codec_error_callback(&error);
			return 0;
		}
		codec->codec_data_dump_callback(si,size,DATA_DEC_37);//
		int firstlen = si[3];
		//printf("1 firstlen=%d\n",firstlen);
		int id=0;
		memcpy(dec_frame,head,3);
		if(firstlen<size){
			memcpy(&dec_frame[3],&si[4],firstlen);
			codec_push_data(codec,&codec->trcontext->buf[DEC_UNION_BUF],dec_frame,firstlen+3);
			id = firstlen+4;
			codec->codec_data_dump_callback(dec_frame,firstlen+3,DATA_DEC_38);//
		}
		int nextframe_size = si[id];
		//printf("2 nextframe_size=%d id=%d\n",nextframe_size,id);
		id++;
		while(id<size){
			
			head->id++;
			memcpy(dec_frame,head,3);
			memcpy(&dec_frame[3],&si[id],nextframe_size);
			codec_push_data(codec,&codec->trcontext->buf[DEC_UNION_BUF],dec_frame,nextframe_size+3);
			codec->codec_data_dump_callback(dec_frame,nextframe_size+3,DATA_DEC_39);//
			id +=nextframe_size;
			if(id<size)
			nextframe_size = si[id];
			id++;
			//printf("3 nextframe_size=%d id=%d\n",nextframe_size,id);
		}
		do_union_frame_decode(codec,dec_refbuf);
	}
	else
	{
		int error = ERROR_FRAME_TYPE;
		codec->codec_error_callback(&error);
		codec->codec_data_dump_callback(si,size,100);//
		return 0;
	}
	return 1;
}
static  void do_dec_process(jqy_codec_t* codec,jqyring_buf_t *dec_refbuf,unsigned char *si,unsigned int *size,unsigned char *so){
	unsigned char head_len = sizeof(struct _sync_head);
	int begin = head_len;
	int index = codec_find_sync_head(si,size);
	if(index<0){
		*size=0;
		return;
	}
	else if(head_len > *size){
		memcpy(so, si,*size);
		return;
	}
	if(*size==0)return;//not find the sync head
	//this a single main frame
	int more_than_one=0;
	for(int i=begin;i<*size;i++)
	{
		//this frames have more than one frame or contains next frame
	   if(si[i]==0)
	   {
			//get the frame end,then decode the frame
			//return;
			int ok=decode_one_frame(codec,si,dec_refbuf,i);
			if(ok==0){
				*size=0;
				return;
			}
			codec->codec_data_dump_callback(si,i,DATA_DEC_26);//
			i++;
			*size = *size-i;
			if(*size==0){
				// the is a entire frame,arrived the end
				return;
			}
			else if(*size < head_len){ 
				//return the leave data and compile to next frame to decoded
				memcpy(so, &si[i],*size);
				return;
			}
			si=si+i;
			codec->codec_data_dump_callback(si,*size,DATA_DEC_27);//
			do_dec_process(codec,dec_refbuf,si,size,so);
			more_than_one=1;
		}
	}
	//this is a big frame,need to find end tag with the next frame
	if(more_than_one==0){
		memcpy(so, si,*size);
		return;
	}
}
void codec_dec_task(void const *argument)
{
	jqy_codec_t* codec = (jqy_codec_t*) argument;
	jqyring_buf_t *buf = &codec->trcontext->buf[DEC_INPUT_BUF];
	jqyring_buf_t *dec_refbuf = &codec->trcontext->buf[DEC_REF_BUF];
	while(1)
   	{
	   if(0==codec->trcontext->can_run)
	   {
		   codec->codec_delayms(1000);
		   if(codec->trcontext->is_stop)break;
		   continue;
	   }
	   if(codec_check_buf_bussy(buf))
	   {
		   codec->codec_delayms(5);
		   continue;
	   }
	   else if(buf->rd_ptr !=buf->wr_ptr)
	   {
	   	   int ok=0;
		   static unsigned int leavesize=0;
		   unsigned int size =	buf->buffer_size[buf->rd_ptr];
		   codec->codec_data_dump_callback(buf->buffer[buf->rd_ptr],size,DATA_DEC_20);// 
		   int size2=size+leavesize;

		   if(size2 > MAX_BUF_SIZE_TEMP)
		   	{
				leavesize=0;
		   		codec_modify_buf_rd(buf);
				int error = ERROR_FRAME_NCNT_NOT_MATCH_FRAME_LEN;
				codec->codec_error_callback(&error);
				continue;
		   	}
		 
		   memcpy(dec_need_buf+leavesize,buf->buffer[buf->rd_ptr], size);
		   unsigned int index=codec_find_sync_head(dec_need_buf,&size2);
		   if(index<0) 
		   {
		   		leavesize=0;
		   		codec_modify_buf_rd(buf);
				int error = ERROR_FRAME_NCNT_NOT_MATCH_FRAME_LEN;
				 codec->codec_error_callback(&error);
				 continue;
		   }
		   int size3 = size2;
		   if(size2==0)
		   {
		   		leavesize=0;
				codec_modify_buf_rd(buf);
				int error = ERROR_FRAME_NCNT_NOT_MATCH_FRAME_LEN;
				codec->codec_error_callback(&error);
				continue;
		   }
		   sync_head_t *head = (sync_head_t *)(&dec_need_buf[index]);
		   if(head->codec_type==RM_ZERO_HUGE)
		   {
				int ok=codec->codec_dec_pop_data(&dec_need_buf[index],size2);
				if(ok)
				{
					codec->codec_data_dump_callback(&dec_need_buf[index],size2,DATA_DEC_35);// 
					codec_modify_buf_rd(buf);
				}
				else  codec->codec_delayms(size2 *14/50);
		   		continue;
		   }
		   else if(head->codec_type>0x00)
		   {
				do_dec_process(codec,dec_refbuf,&dec_need_buf[index],&size2,dec_so);
				if(size2>0)
				{
					leavesize = size2;
					printf("leavesize=%d\n",leavesize);
					memcpy(dec_need_buf,dec_so,leavesize);
				}
			}
			else
			{
			  	codec->codec_data_dump_callback(&dec_need_buf[index],size2,DATA_DEC_20);// 
				printf("codec_dec_task:codec_type error head->codec_type=%d\n",head->codec_type);
				leavesize=0;
			}
		   codec_modify_buf_rd(buf);
	   }
	   else codec->codec_delayms(5);
   }
}
//************************************end decode************************************************************

void codec_enc_pop_task(void const *argument)
{
	jqy_codec_t* codec = (jqy_codec_t*) argument;
	jqyring_buf_t *enc_output_buf = &codec->trcontext->buf[ENC_OUTPUT_BUF];
	while(1)
    {
    	if(0==codec->trcontext->can_run)
		{
			codec->codec_delayms(1000);
			if(codec->trcontext->is_stop)break;
			continue;
    	}
		if(codec_check_buf_bussy(enc_output_buf))
		{
			codec->codec_delayms(5);
            continue;
		}
		else if(enc_output_buf->rd_ptr !=enc_output_buf->wr_ptr)
		{
			unsigned int size =  enc_output_buf->buffer_size[enc_output_buf->rd_ptr];
			int ok=codec->codec_enc_pop_data( enc_output_buf->buffer[enc_output_buf->rd_ptr],size);
			if(ok)
			{
				codec->codec_data_dump_callback(enc_output_buf->buffer[enc_output_buf->rd_ptr],size,DATA_ENC_9);
				codec_modify_buf_rd(enc_output_buf);
			}
			 else codec->codec_delayms(size*14/50);
		}
        else codec->codec_delayms(5);
    }	

}
void codec_dec_pop_task(void const *argument)
{
	jqy_codec_t* codec = (jqy_codec_t*) argument;
	jqyring_buf_t *dec_output_buf = &codec->trcontext->buf[DEC_OUTPUT_BUF];
	while(1)
	{
		if(0==codec->trcontext->can_run)
		{
			codec->codec_delayms(1000);
			if(codec->trcontext->is_stop)break;
			continue;
		}
		if(codec_check_buf_bussy(dec_output_buf))
		{
			codec->codec_delayms(5);
			continue;
		}
		else if(dec_output_buf->rd_ptr !=dec_output_buf->wr_ptr)
		{
			unsigned int size =  dec_output_buf->buffer_size[dec_output_buf->rd_ptr];
			sync_head_t *head = (sync_head_t *)dec_output_buf->buffer[dec_output_buf->rd_ptr];
			int ok=0;
			if(head->codec_type==ACK_RM_ZERO_REF || head->codec_type==ACK_RM_ZERO_REF_HAVE_NOT)
			{
				ok=codec->codec_dec_ack_data( dec_output_buf->buffer[dec_output_buf->rd_ptr],size);
				codec->codec_data_dump_callback(dec_output_buf->buffer[dec_output_buf->rd_ptr],size,DATA_DEC_31);
			}
			else
			{
				ok=codec->codec_dec_pop_data( dec_output_buf->buffer[dec_output_buf->rd_ptr],size);
				codec->codec_data_dump_callback(dec_output_buf->buffer[dec_output_buf->rd_ptr],size,DATA_DEC_32);
			}
			
			if(ok) codec_modify_buf_rd(dec_output_buf);
			else codec->codec_delayms(size*14/50);
		}
		else codec->codec_delayms(5);
	}	
}


