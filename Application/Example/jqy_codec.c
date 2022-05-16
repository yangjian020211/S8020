#include "jqy_codec.h"
//-----------------------------------codec------------------------------------------------
#define MAX_BUF_SIZE_TEMP 2000
static  unsigned char dec_need_buf[MAX_BUF_SIZE_TEMP]={0};
static  unsigned char dec_so[MAX_BUF_SIZE_TEMP]={0};
static  unsigned char dec_frame[MAX_BUF_SIZE_TEMP]={0};
static  unsigned char dec_preframe[MAX_BUF_SIZE_TEMP]={0};
static  unsigned char enc_encout_frame[MAX_BUF_SIZE_TEMP]={0};
static  unsigned char enc_xor_frame[MAX_BUF_SIZE_TEMP]={0};
static  unsigned char enc_frame[MAX_BUF_SIZE_TEMP]={0};


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
	trcontext->buf[REF_BUF].buffer_offset=2;
	trcontext->buf[REF_BUF].rd_ptr=trcontext->buf[REF_BUF].buffer_offset;
	trcontext->buf[REF_BUF].wr_ptr=trcontext->buf[REF_BUF].buffer_offset;
	
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
		 printf("\n codec_push_data:No ring buffer !!!\n");
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
						j++;
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
				 printf("1 n=%d i=%d si[%d]=%x,j=%d,sj[%d]=%2x addr=%d sj[%d]=%2x zero_cnt=%d\n",n,i,i,si[i],j,j,so[j],addr,addr,so[addr],zero_cnt);
		#endif

		}
		else if(state==2)
		{
			so[addr] |= 0x0f;
		#if debug1
				printf("2.1 n=%d i=%d si[%d]=%x,j=%d,sj[%d]=%2x addr=%d sj[%d]=%2x zero_cnt=%d\n",n,i,i,si[i],j,j,so[j],addr,addr,so[addr],zero_cnt);
		#endif
			addr =j;
			so[addr] = 0x00;
			i++;
			if(i>=in_len){
				state=6;
			}
			else{
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
				//zero_cnt=0;
				j++;
				n=0;
				
			}
			else
			{
				if(n>0)
				{
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
			addr=j; 
			so[addr]=0xFF;	
			//j++;
			//addr=j;
			//so[addr]=0xFF;
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
			if(so[addr] !=0)
			j++;
			
			if(n>0){
				so[addr]+=n;
				//j++;
			}
			else if(zero_cnt>0)
			{
				{
					int dn = zero_cnt/15;
					addr=j;
					#if debug1
					printf("addr=%d,zero_cnt=%d dn=%d  \n",addr,zero_cnt,dn);
					#endif
					for(int c=0;c<dn;c++){
						so[addr+c]=0xff;
					}
					addr+=dn;
					
					if(dn>0)
					{
						 int ls = zero_cnt%15;
						 if(ls>0)
						 {
							 j++;
							 addr =j;
							 so[addr]=ls<<4;
							 j++;
						 }
					}
					else
					{
					 int ls = zero_cnt%16;
					 if(ls>0)
					 {
					 	addr =j;
					 	so[addr]=ls<<4;	
					 	j++;
					  }
					}
					#if debug1
					printf("addr=%d,zero_cnt=%d\n",addr,zero_cnt);
					#endif
				}
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
	#if debug
				 printf("in_len=%2d\n",in_len);
	#endif 

	for(i=0;i<in_len;)
	{

		unsigned char key = si[i] & 0xf0;
		unsigned char zero_len=(si[i] & 0x0f);
		if(si[i]==0xff)
		{
			zero_len = (key>>4)  + zero_len;
			memset(&so[j],0,zero_len);
			j+=15;
			i++;
		}
		else if(key == 0xf0)
		{
			zero_len = (key>>4)  + zero_len;
			memset(&so[j],0,zero_len);
			//j++;
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
//*********************************end core***************************************************************

//*********************************encode*****************************************************************
static void codec_nec_update_ref_frameid(jqy_codec_t* codec , unsigned char *si,unsigned int size)
{
	unsigned char head_len = sizeof(struct _sync_head);
	jqyring_buf_t *refbuf = &codec->trcontext->buf[REF_BUF];
	jqyring_buf_t *refbuf_temp = &codec->trcontext->buf[REF_BUF_TEMP];
	//codec->codec_data_dump_callback( si,size,81);//
	int ref_cnt = codec_get_buf_cnt(codec,refbuf);	
	int ref_cnt_tmp = codec_get_buf_cnt(codec,refbuf_temp);	
	//printf("refbuf cnt=%d wrid=%d ref_cnt_tmp=%d refbuf_temp->rd_ptr=%d \n",ref_cnt,refbuf->wr_ptr,ref_cnt_tmp,refbuf_temp->rd_ptr);
	//return;
	if(ref_cnt_tmp==0){
		//printf("--ref_cnt_tmp--%d-----\n",ref_cnt_tmp);
		return;	
	}
	int id_in_temp_ref_buf=-1;
	sync_head_t *head_ack = (sync_head_t *)si;
	int tid=refbuf_temp->rd_ptr;
	while(tid != refbuf_temp->wr_ptr)
	{
		ref_frame_head_t *head_ref = (ref_frame_head_t *)refbuf_temp->buffer[tid];
		if(head_ref->sync.id==head_ack->id)
		{
			id_in_temp_ref_buf = tid;
			codec->codec_data_dump_callback(refbuf_temp->buffer[tid],refbuf_temp->buffer_size[tid],DATA_ENC_8);//
		}
		tid = (tid + 1) % (refbuf->max_buffer_cnt);
		if(tid==0)tid=refbuf->buffer_offset;
	}
	if(id_in_temp_ref_buf<0){
		printf("get error ack,the ref_temp_buf has not the id : %d \n",head_ack->id);
		return;
	}
	//1 show 
	codec->codec_data_dump_callback(refbuf_temp->buffer[id_in_temp_ref_buf],refbuf_temp->buffer_size[id_in_temp_ref_buf],DATA_ENC_7);//
	//2 push to ref_buf
	codec_push_data(codec,refbuf,refbuf_temp->buffer[id_in_temp_ref_buf],refbuf_temp->buffer_size[id_in_temp_ref_buf]);
	//3 rem the tid from the temp_ref_buf
	if(id_in_temp_ref_buf==refbuf_temp->rd_ptr){
		memset(refbuf_temp->buffer[id_in_temp_ref_buf], 0, refbuf_temp->buffer_size[id_in_temp_ref_buf]);
		codec_modify_buf_rd(refbuf_temp);
	}		
	//4 show ref_buf cnt
	ref_cnt = codec_get_buf_cnt(codec,refbuf);	
	//printf("------refbuf cnt=%d --------\n",ref_cnt);
}
static int find_data_head_len(jqy_codec_t* codec,unsigned char tag){
	for(int i=0;i<codec->trcontext->data_tag.size;i++)
	{
		if(codec->trcontext->data_tag.tag[tag][0]==tag) return codec->trcontext->data_tag.tag[tag][1];
	}
	return -1;
}
static void codec_update_table_exist_ref_frame(jqy_codec_t* codec,jqyring_buf_t *inbuf,jqyring_buf_t *refbuf,jqyring_buf_t *refbuf_temp){

	unsigned char head_len = sizeof(struct _sync_head);
	int len_refhead=sizeof(struct _ref_frame_head);

	unsigned int rd_idx1 = refbuf->rd_ptr;
	unsigned int wr_idx1 = refbuf->wr_ptr;
	unsigned int size1 =  refbuf->buffer_size[rd_idx1];

	unsigned int rd_idx2 = inbuf->rd_ptr;
	unsigned int size2 =  inbuf->buffer_size[rd_idx2];
	sync_head_t *head2 = (sync_head_t *)inbuf->buffer[rd_idx2];

	unsigned int rd_idx3 = refbuf_temp->rd_ptr;
	unsigned int wr_idx3 = refbuf_temp->wr_ptr;
	unsigned int size3=  refbuf_temp->buffer_size[rd_idx3];
	int cnt=0;
	int precnt=20000;
	int len = find_data_head_len(codec,head2->usser_type);

	//1 update the ref_buf to find the have ,eq, rid param
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
		//codec->codec_data_dump_callback(&refbuf->buffer[tid][len_refhead+head_len],len,66);
		//codec->codec_data_dump_callback(&inbuf->buffer[rd_idx2][head_len],len,67);
		//sync_head_t *head_in = (sync_head_t *)inbuf->buffer[rd_idx2];
		//ref_frame_head_t *head_ref = (ref_frame_head_t *)&refbuf->buffer[tid];
		//printf("\nhead_in->id=%d,head_ref->id=%d\n",head_in->id,head_ref->sync.id);
		if(memcmp(&refbuf->buffer[tid][len_refhead+head_len], &inbuf->buffer[rd_idx2][head_len], len)==0)			
		{
			codec->trcontext->ref_state.have=1;
			ref_frame_head_t *head4 = (ref_frame_head_t *)refbuf->buffer[tid];
			codec->trcontext->ref_state.id_in_table=tid;
			codec_serial_xor(&refbuf->buffer[tid][len_refhead+head_len],&inbuf->buffer[rd_idx2][head_len],enc_frame,size2-head_len);
			for(int i=0;i<size2-head_len;i++)if(enc_frame[i] !=0)cnt++;
			//printf("1---tid=%d-rd_idx2=%d rfid=%d-have =%d frequency=%d \n",tid,rd_idx2,head4->sync.id,1,head4->frequency);
			if(cnt==0)
			{
				codec->trcontext->ref_state.have_eq=0x01;
				codec->trcontext->ref_state.best_rid=head4->sync.id;
				codec->trcontext->ref_state.id_in_table=tid;
				head4->frequency++;
				//printf("2---tid=%d-rd_idx2=%d rfid=%d-have =%d frequency=%d \n",tid,rd_idx2,head4->sync.id,1,head4->frequency);
				return;
			}
			else if(cnt<precnt)
			{
				codec->trcontext->ref_state.id_in_table=tid;
				codec->trcontext->ref_state.best_rid=head4->sync.id;
				head4->frequency++;
				precnt = cnt;
				//printf("3---tid=%d-rd_idx2=%d rfid=%d-have =%d frequency=%d \n",tid,rd_idx2,head4->sync.id,1,head4->frequency);
			}
		}
		tid = (tid+1)% (refbuf->max_buffer_cnt);
		if(tid==0)tid=refbuf->buffer_offset;
	}

	//2 to update the ref_buf_temp to decided the resend or not.it is to decide the request is sent but get the ack or not
	unsigned char havesent=0;
	tid=rd_idx3;
	while(tid !=wr_idx3)
	{
		//sync_head_t *head_in = (sync_head_t *)inbuf->buffer[rd_idx2];
		//ref_frame_head_t *head_ref_temp = (ref_frame_head_t *)&refbuf_temp->buffer[tid];
		//find input data type with the len
		if(memcmp(&refbuf_temp->buffer[tid][len_refhead+head_len], &inbuf->buffer[rd_idx2][head_len], len)==0)			
		{		
			havesent=1;
			
			ref_frame_head_t *refhead = (ref_frame_head_t *) refbuf_temp->buffer;
			unsigned int now_time = codec->codec_get_curtime_ms();

			if(now_time-refhead->last_time > codec->trcontext->update_ref_time) 
				codec->trcontext->ref_state.timeout=1;
			else 
				codec->trcontext->ref_state.timeout=0;
			break;
		}
		tid = (tid+1)% (refbuf_temp->max_buffer_cnt);
		if(tid==0)tid=refbuf_temp->buffer_offset;
	}
	codec->trcontext->ref_state.sent = havesent;

}
static void codec_enc_get_ref_frame_id(jqy_codec_t* codec,jqyring_buf_t *enc_input_buf,jqyring_buf_t *refbuf_temp,sync_head_t *head)
{
	int size=enc_input_buf->buffer_size[enc_input_buf->rd_ptr];
	unsigned char head_len = sizeof(struct _sync_head);
	//head = (sync_head_t *)inbuf;
	
	if(size >codec->trcontext->max_bag_thd){
		head->codec_type=RM_ZERO_HUGE;
		head->rid=1;
		return;
	}
	//3 judge the result
	if(codec->trcontext->ref_state.not_as_ref==1) {
		head->codec_type=RM_ZERO_REF;
		head->rid=1;
		#if Linuxsimulation 
		printf("\n----2----\n");
		#else
		DLOG_Error("----2----");
		#endif
		return ;
	}
	else if(codec->trcontext->ref_state.have_eq==1) {
		#if Linuxsimulation 
		printf("\n----3----\n");
		#else
		DLOG_Error("----3----");
		#endif
		head->codec_type=RM_ZERO_REF_EQ;
		head->rid=codec->trcontext->ref_state.best_rid;
		return ;
	}
	else if(codec->trcontext->ref_state.have==0 && codec->trcontext->ref_state.sent==0 ) {
		#if Linuxsimulation 
		printf("\n----4----\n");
		#else
		DLOG_Error("----4----");
		#endif
		head->codec_type=RM_ZERO_REF_UPDATE;
		head->rid=1;
		//get now time and push to the temp_buffer
		struct _ref_frame_head refhead={0};
		int len_refhead=sizeof(struct _ref_frame_head);
		memcpy(&enc_frame[len_refhead], enc_input_buf->buffer[enc_input_buf->rd_ptr], size);
		refhead.frequency=1;
		refhead.sync.codec_type=head->codec_type;
		refhead.sync.rid=head->rid;
		refhead.sync.id=head->id;
		refhead.last_time=codec->codec_get_curtime_ms();
		memcpy(enc_frame, &refhead,len_refhead);
		//overlap the sync head
		codec_push_data(codec,refbuf_temp,enc_frame,size+len_refhead);
		//codec->codec_data_dump_callback(temp_frame,size+len_refhead-head_len,66);// 
		//codec->codec_data_dump_callback(refbuf_temp->buffer[refbuf_temp->rd_ptr],refbuf_temp->buffer_size[refbuf_temp->rd_ptr],67);// 
		return;
	}
	else if(codec->trcontext->ref_state.have==1 && codec->trcontext->ref_state.timeout==0 ) {
		#if Linuxsimulation 
		printf("\n----5----\n");
		#else
		DLOG_Error("----5----");
		#endif
		head->codec_type=RM_ZERO_REF;
		head->rid=codec->trcontext->ref_state.best_rid;
		return;
	}//tale have the ref_frame but can not get the ack signal
	else if(codec->trcontext->ref_state.have==1 && codec->trcontext->ref_state.timeout==1) {
		#if Linuxsimulation 
		printf("\n----6----\n");
		#else
		DLOG_Error("----6----");
		#endif
		head->codec_type=RM_ZERO_REF_UPDATE;
		head->rid=codec->trcontext->ref_state.best_rid;
		struct _ref_frame_head refhead={0};
		int len_refhead=sizeof(struct _ref_frame_head);
		memcpy(&enc_frame[len_refhead], enc_input_buf->buffer[enc_input_buf->rd_ptr], size);
		refhead.frequency=1;
		refhead.sync.codec_type=head->codec_type;
		refhead.sync.rid=head->rid;
		refhead.sync.id=head->id;
		refhead.last_time=codec->codec_get_curtime_ms();
		memcpy(enc_frame, &refhead,len_refhead);
		codec_push_data(codec,refbuf_temp,enc_frame,size+len_refhead);
		return ;
	}//table have not the ref frame ,have sent but not the ack
	else if(codec->trcontext->ref_state.have==0 && codec->trcontext->ref_state.timeout==1) {
		#if Linuxsimulation 
		printf("\n----7----\n");
		#else
		DLOG_Error("----7----");
		#endif
		head->codec_type=RM_ZERO_REF;
		head->rid=1;
		return;
	}
	else if(codec->trcontext->ref_state.have==0 && codec->trcontext->ref_state.timeout==0) {
		head->codec_type=RM_ZERO_REF_UPDATE;
		head->rid=1;
		#if Linuxsimulation 
		printf("\n----8----\n");
		#else
		DLOG_Error("----8----");
		#endif
		struct _ref_frame_head refhead={0};
		int len_refhead=sizeof(struct _ref_frame_head);
		memcpy(&enc_frame[len_refhead], enc_input_buf->buffer[enc_input_buf->rd_ptr], size);
		refhead.frequency=1;
		refhead.sync.codec_type=head->codec_type;
		refhead.sync.rid=head->rid;
		refhead.sync.id=head->id;
		refhead.last_time=codec->codec_get_curtime_ms();
		memcpy(enc_frame, &refhead,len_refhead);
		codec_push_data(codec,refbuf_temp,enc_frame,size+len_refhead);
		return ;
	}
	else
	{
		#if Linuxsimulation 
		printf("\n----9----\n");
		#else
		DLOG_Error("\n----9----\n");
		#endif
		head->codec_type=RM_ZERO_REF_UPDATE;
		head->rid=1;
		return ;
	}
	
}
static int codec_xor_rm_zero( jqy_codec_t* codec,jqyring_buf_t *enc_input_buf,jqyring_buf_t *enc_refbuf,jqyring_buf_t *enc_refbuf_temp,unsigned int enc_rd,unsigned int size)
{
	unsigned int len=0;
	int ref_cnt =0;

	unsigned char head_len=sizeof(struct _sync_head);
	int ref_head_len = sizeof(struct _ref_frame_head);
	if(size>2000-head_len){
		return 1;
	}
	struct _sync_head newh={0};
	memset(&newh, 0,head_len);
	memcpy(&newh,enc_input_buf->buffer[enc_rd], head_len);
	newh.id= enc_rd;
	newh.rid= enc_rd;
	ref_cnt=codec_get_buf_cnt(codec,enc_refbuf);	
	memset(&codec->trcontext->ref_state, 0 ,sizeof(struct _update_state));
	codec_update_table_exist_ref_frame(codec,enc_input_buf,enc_refbuf,enc_refbuf_temp);
	codec_enc_get_ref_frame_id(codec,enc_input_buf,enc_refbuf_temp,&newh);
	#if Linuxsimulation 
		printf("---refbuf_cnt=%d,headr.rfid=%d,headr.id=%d,headr.ctype=%d,headr.utype=%d size=%d\n",ref_cnt,newh.rid,newh.id,newh.codec_type,newh.usser_type,size);
	#else
		//DLOG_Error("refbuf_cnt=%d,headr.rfid=%d,headr.id=%d,headr.ctype=%d,headr.utype=%d size=%d",ref_cnt,newh.rid,newh.id,newh.codec_type,newh.usser_type,size);
	#endif
	if(newh.codec_type==RM_ZERO_REF_EQ)
	{
		memcpy(enc_encout_frame, &newh, head_len);
		enc_encout_frame[head_len]=0;
		codec->codec_data_dump_callback(enc_encout_frame,head_len+1,DATA_ENC_6);
		//codec->codec_enc_pop_data(encout_frame, head_len+1);
		codec_push_data(codec,&codec->trcontext->buf[ENC_OUTPUT_BUF],enc_encout_frame,head_len+1);
		return 1;
	}
	else if(newh.codec_type==RM_ZERO_HUGE)
	{
		memcpy(enc_encout_frame, &newh, head_len);
		memcpy(&enc_encout_frame[head_len],&enc_input_buf->buffer[enc_rd][head_len], size-head_len);
		//codec->codec_enc_pop_data(encout_frame, size);
		codec_push_data(codec,&codec->trcontext->buf[ENC_OUTPUT_BUF],enc_encout_frame,size);
		codec->codec_data_dump_callback(enc_encout_frame,size,DATA_ENC_5);
		return 1;
	}
	//1 pre encode ,do xor to find diff
	else if(ref_cnt>0 && newh.rid>1)
	{
		codec->codec_data_dump_callback(enc_refbuf->buffer[codec->trcontext->ref_state.id_in_table],size+ref_head_len,DATA_ENC_1);// 
		codec_serial_xor(&enc_refbuf->buffer[codec->trcontext->ref_state.id_in_table][ref_head_len+head_len],&enc_input_buf->buffer[enc_rd][head_len],enc_xor_frame,size-head_len);
	}
	else 
	{
		memcpy(enc_xor_frame,&enc_input_buf->buffer[enc_rd][head_len], size-head_len);
	}
	//2 do enc
	codec->codec_data_dump_callback(enc_xor_frame,size-head_len,DATA_ENC_2);// 
	if(size-head_len >2000) return 1;
	
	codec_enc_find_non_zero_plus(enc_xor_frame,size-head_len,enc_frame,&len);
	codec->codec_data_dump_callback(enc_frame,len,DATA_ENC_3);// 
	//3 add head
	memcpy(enc_encout_frame, &newh, head_len);
	//4 copy head to buffer
	memcpy(&enc_encout_frame[head_len],enc_frame,len);
	
	//5 add end tag to frame end
	enc_encout_frame[len+head_len]=0;
	//6 push to enc_output_buffer
	if(len+head_len+1 >2000){
		printf("\n----len=%d----\n",len);
		return 1;
	}
	//codec->codec_enc_pop_data(encout_frame, len+head_len+1);
	codec_push_data(codec,&codec->trcontext->buf[ENC_OUTPUT_BUF],enc_encout_frame,len+head_len+1);
	codec->codec_data_dump_callback(enc_encout_frame,len+head_len+1,DATA_ENC_4);// 
	return 1;	
}
void codec_enc_task(void const *argument)
{
	jqy_codec_t* codec = (jqy_codec_t*) argument;
	jqyring_buf_t *enc_input_buf = &codec->trcontext->buf[ENC_INPUT_BUF];
	jqyring_buf_t *enc_refbuf = &codec->trcontext->buf[REF_BUF];
	jqyring_buf_t *enc_refbuf_temp = &codec->trcontext->buf[REF_BUF_TEMP];
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
			codec_xor_rm_zero(codec,enc_input_buf,enc_refbuf,enc_refbuf_temp,enc_input_buf->rd_ptr,size);
			codec_modify_buf_rd(enc_input_buf);
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
	jqyring_buf_t *refbuf = &codec->trcontext->buf[REF_BUF];
	int tid=refbuf->rd_ptr;
	int ref_cnt = codec_get_buf_cnt(codec,refbuf);	
	while(tid != refbuf->wr_ptr){
		ref_frame_head_t *head2 = (ref_frame_head_t *)refbuf->buffer[tid];
		if(head2->sync.id==rid){
			return tid;
		}
		tid = (tid + 1) % (refbuf->max_buffer_cnt);
		if(tid==0)tid=refbuf->buffer_offset;
	}
	 return 0;
}
static void codec_dec_update_ref_frameid(jqy_codec_t* codec ,unsigned char *si,unsigned int size)
{
	
	jqyring_buf_t *refbuf =&codec->trcontext->buf[REF_BUF];
	int sync_head_len = sizeof(struct _sync_head);
	int ref_head_len = sizeof(struct _ref_frame_head);
	int tid=refbuf->rd_ptr;
	while(tid != refbuf->wr_ptr)
	{
		ref_frame_head_t *head = (ref_frame_head_t *)refbuf->buffer[tid];
		sync_head_t *head_new = (sync_head_t *)si;
		if(head->sync.id==head_new->id)
		{
			memcpy(&refbuf->buffer[tid][ref_head_len-sync_head_len-1], si,size);
			head->frequency++;
			codec->codec_data_dump_callback(refbuf->buffer[tid],size+ref_head_len-sync_head_len,DATA_DEC_29);//
			return;
		}
		tid = (tid + 1) % (refbuf->max_buffer_cnt);
		if(tid==0)tid=refbuf->buffer_offset;
	}
	struct _ref_frame_head refhead={0};
	refhead.frequency=1;
	refhead.last_time=codec->codec_get_curtime_ms();
	memcpy(dec_frame, &refhead,ref_head_len);
	//overlap the sync head
	memcpy(&dec_frame[ref_head_len-sync_head_len-1], si, size);
	codec_push_data(codec,refbuf,dec_frame,size+ref_head_len-sync_head_len);
	codec->codec_data_dump_callback(dec_frame,size+ref_head_len-sync_head_len,DATA_DEC_28);
	
}
static void decode_one_frame(jqy_codec_t* codec,unsigned char *si,jqyring_buf_t *refbuf,unsigned int size){
	sync_head_t *head = (sync_head_t *)si;
	unsigned char head_len = sizeof(struct _sync_head);
	int ref_head_len = sizeof(struct _ref_frame_head);
	int insize = size-head_len;
	int len=0;
	
 	if(head->codec_type==RM_ZERO_REF_EQ)
	{
		int tid=refbuf->rd_ptr;
		while(tid != refbuf->wr_ptr)
		{
			ref_frame_head_t *head2 = (ref_frame_head_t *)refbuf->buffer[tid];
			if(head->rid == head2->sync.id)
			{
				//1 copy the head to tempframe to replace the head infomation
				memcpy(dec_frame, head,head_len);

				//2 copy the refframe data to tempframe
				int ref_head_len=sizeof(struct _ref_frame_head);
				memcpy(&dec_frame[head_len],&refbuf->buffer[tid][ref_head_len-1],refbuf->buffer_size[tid]-ref_head_len);
				//3 push to output_buffer
				//codec->codec_dec_pop_data(temp_frame,refbuf->buffer_size[tid]-(ref_head_len-head_len));
				codec_push_data(codec,&codec->trcontext->buf[DEC_OUTPUT_BUF],dec_frame,refbuf->buffer_size[tid]-(ref_head_len-head_len));
				//4 
				codec->codec_data_dump_callback(dec_frame,refbuf->buffer_size[tid]-(ref_head_len-head_len),DATA_DEC_33);//
				return;
			}
			tid = (tid + 1) % (refbuf->max_buffer_cnt);
			if(tid==0)tid=refbuf->buffer_offset;
		}
	}
	else if(head->codec_type==ACK_RM_ZERO_REF){

		codec_nec_update_ref_frameid(codec,si,size);
		return;
	}
	else if(head->codec_type==ACK_RM_ZERO_REF_HAVE_NOT){

		refbuf->rd_ptr=refbuf->wr_ptr;
		return;
	}
	else if(head->codec_type==RM_ZERO_REF_UPDATE || head->codec_type==RM_ZERO_REF){	
		if(insize<0){
			int error = ERROR_FRAME_IN_FRAME_LEN_LESS_0;
			codec->codec_error_callback(&error);
			return;
		}
		if(insize>=2000){
			
			int error = ERROR_FRAME_DEC_LENGTH_LONG_THAN2000;
			codec->codec_error_callback(&error);
			return;
		}
		
		codec_dec_find_non_zero_plus(&si[head_len],insize,dec_preframe,&len,2000);
		
		if(len>=2000){
			int error = ERROR_FRAME_DEC_LENGTH_LONG_THAN2000;
			codec->codec_error_callback(&error);
			return;
		}
		int ref_cnt = codec_get_buf_cnt(codec,refbuf);
		codec->codec_data_dump_callback(dec_preframe,len,DATA_DEC_21);
		if(head->rid > 1 && ref_cnt>0)
		{
			int id_inrefbuf = find_ref_id_by_rid(codec,head->rid);
			if(id_inrefbuf==0 && head->codec_type==RM_ZERO_REF)
			{
				//have not the rid frame in the ref_buf,neet to update table
				int error = ERROR_FRAME_NOT_IN_REFBUF;
				codec->codec_error_callback(&error);
				
				struct _sync_head head2 ={0};
				head_len = sizeof(struct _sync_head);
				memcpy(&head2, head, head_len);
				head2.codec_type=ACK_RM_ZERO_REF_HAVE_NOT;
				unsigned char buf[100]={0};
				memcpy(buf, &head2, head_len);
				buf[head_len]=0;
				codec_push_data(codec,&codec->trcontext->buf[DEC_OUTPUT_BUF],buf,head_len+1);
				codec->codec_data_dump_callback(buf,head_len+1,DATA_DEC_30);//
				return;
			}
			else{
				codec_serial_xor(&refbuf->buffer[id_inrefbuf][ref_head_len-1],dec_preframe,dec_frame,len);
				codec->codec_data_dump_callback(refbuf->buffer[id_inrefbuf],refbuf->buffer_size[id_inrefbuf],DATA_DEC_22);
			}
		}
		else{
			memcpy(dec_frame,dec_preframe,len);
		}
		
		memcpy(dec_preframe,head,head_len);
		memcpy(&dec_preframe[head_len],dec_frame,len);
		codec->codec_data_dump_callback(dec_preframe,head_len+len,DATA_DEC_23);

		codec_push_data(codec,&codec->trcontext->buf[DEC_OUTPUT_BUF],dec_preframe,head_len+len);
		codec->codec_data_dump_callback(dec_preframe,head_len+len,DATA_DEC_23);
		if(head->codec_type==RM_ZERO_REF_UPDATE)
		{

			codec_dec_update_ref_frameid(codec,dec_preframe,head_len+len);
			codec->codec_data_dump_callback(head,head_len,DATA_DEC_24);//
		
			struct _sync_head head2 ={0};
			memcpy(&head2, dec_preframe, head_len);
			head2.codec_type=ACK_RM_ZERO_REF;
			unsigned char buf[100]={0};
			memcpy(buf, &head2, head_len);
			buf[head_len]=0;
			codec_push_data(codec,&codec->trcontext->buf[DEC_OUTPUT_BUF],buf,head_len+1);
			codec->codec_data_dump_callback(buf,head_len+1,DATA_DEC_25);//
		}
	}
	else {
		int error = ERROR_FRAME_TYPE;
		codec->codec_error_callback(&error);
	}
}
static  void do_dec_process(jqy_codec_t* codec,jqyring_buf_t *refbuf,unsigned char *si,unsigned int *size,unsigned char *so){
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
			decode_one_frame(codec,si,refbuf,i);
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
			do_dec_process(codec,refbuf,si,size,so);
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
	jqyring_buf_t *refbuf = &codec->trcontext->buf[REF_BUF];
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
				if(ok){
					codec->codec_data_dump_callback(&dec_need_buf[index],size2,DATA_DEC_35);// 
					codec_modify_buf_rd(buf);
				}
				else  codec->codec_delayms(size2 *14/50);
				
		   		continue;
		   }
		   else if(head->codec_type>0x00)
		   {
				do_dec_process(codec,refbuf,&dec_need_buf[index],&size2,dec_so);
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


