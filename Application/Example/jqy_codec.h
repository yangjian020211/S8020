#ifndef __JQY_CODEC_H__
#define __JQY_CODEC_H__

#define Linuxsimulation 0

#define MAX_BUFFER (128)

#if Linuxsimulation
#include <assert.h>
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <sys/time.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <stdio.h>
#include <fcntl.h>
#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <string.h>
#include <pthread.h>
#include <sys/wait.h>
#include <errno.h>  
#include <string.h> 
#include <stdint.h> 

#else
#include <assert.h>
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "debuglog.h"
#include "hal_bb.h"
#include "hal_gpio.h"
#include "memory_config.h"
#include "debuglog.h"
#include "cmsis_os.h"
#include "hal_usb_host.h"
#include "hal.h"
#include "tranceiver.h"
#include "hal_sram_ground.h"
#include "hal_sram_sky.h"
#include "hal_usb_otg.h"
#include "hal_usb_device.h"
#include "hal_usb_host.h"
#include "memory.h"

#endif

#ifdef __cplusplus
extern "C"
{
#endif

typedef enum{

   RM_ZERO_REF=0x01,
   RM_ZERO_REF_UPDATE,//2
   RM_ZERO_REF_EQ,
   RM_ZERO_HUGE,
   ACK_RM_ZERO_REF,
   ACK_RM_ZERO_REF_HAVE_NOT,
   CMD_MAX_CNT,
} ENUM_ENC_TYPE;
typedef enum{
	
   ERROR_TAG_VALUE_INVALID=0,
   ERROR_NO_DATA_TAG,
   ERROR_FRAME_LEN,
   ERROR_FRAME_FORMAT,
   ERROR_FRAME_TYPE,
   ERROR_FRAME_NCNT_NOT_MATCH_FRAME_LEN,
   ERROR_FRAME_NOT_IN_REFBUF,
   ERROR_FRAME_DEC_LENGTH_LONG_THAN2000,
   ERROR_FRAME_IN_FRAME_LEN_LESS_0,
   ERROR_MAX,
} ENUM_CODEC_ERROR;
typedef enum{
	
   DATA_ENC_0=0,
   DATA_ENC_1,
   DATA_ENC_2,
   DATA_ENC_3,
   DATA_ENC_4,
   DATA_ENC_5,
   DATA_ENC_6,
   DATA_ENC_7,
   DATA_ENC_8,
   DATA_ENC_9,
   DATA_ENC_10,
   DATA_ENC_11,
   DATA_ENC_12,
   DATA_ENC_13,
   DATA_ENC_14,
   DATA_ENC_15,
   DATA_ENC_16,
   DATA_ENC_17,
   DATA_ENC_18,
   DATA_ENC_19,
   DATA_DEC_20,
   DATA_DEC_21,
   DATA_DEC_22,
   DATA_DEC_23,
   DATA_DEC_24,
   DATA_DEC_25,
   DATA_DEC_26,
   DATA_DEC_27,
   DATA_DEC_28,
   DATA_DEC_29,
   DATA_DEC_30,
   DATA_DEC_31,
   DATA_DEC_32,
   DATA_DEC_33,
   DATA_DEC_34,
   DATA_DEC_35,
   DATA_DEC_36,
   DATA_DEC_37,
   DATA_DEC_38,
   DATA_DEC_39,
   DATA_DEC_40,
   DATA_DEC_41,
   DATA_DEC_42,
   DATA_DEC_43,
   DATA_DEC_44,
   DATA_MAX,
} ENUM_CODEC_BUFFER_DATA_ID;   
typedef enum{
   ENC_INPUT_BUF=0,
   DEC_INPUT_BUF,
   ENC_OUTPUT_BUF,
   DEC_OUTPUT_BUF,
   ENC_REF_BUF,
   DEC_REF_BUF,
   MAX_BUF_CNT,

} ENUM_BUF_T;

typedef struct _update_state
{	
	unsigned char have_eq 			: 1;
	unsigned char have 				: 1;
	unsigned char reqtimeout 		: 1; 
	unsigned char no_ref 			: 1; 
	unsigned char not_as_ref		: 1;
	unsigned char id_in_ref			: 1;
	unsigned char reserve			: 2;
	unsigned int  best_rid_in_table    ;
	unsigned int  best_rid	   		   ;
	unsigned int  last_time	   		   ;

} update_state_t;  
typedef struct _sync_head
{	
	unsigned char usser_type : 4;
	unsigned char codec_type : 4; 
	unsigned char id  ;
	unsigned char rid ; 
} sync_head_t;  
typedef struct _ref_frame_head
{	
	unsigned int frequency 	;
	unsigned int last_time 	;
	struct _sync_head sync	;
} ref_frame_head_t		  	;  

typedef struct _jqyring_buf{
    unsigned char  *buffer[MAX_BUFFER];
    unsigned int   buffer_size[MAX_BUFFER];    
    unsigned int  rd_ptr;
    unsigned int  wr_ptr;
	unsigned int max_buffer_cnt;
	unsigned char bufferid;
	unsigned int buffer_offset;
}jqyring_buf_t;
typedef struct _jqyuser_data_tag
{
    unsigned char  size;
    unsigned char  tag[256][2];    
}jqyuser_data_tag_t;
typedef struct _context{
	
	struct _jqyring_buf buf[MAX_BUF_CNT];
	struct _jqyuser_data_tag data_tag;
	struct _update_state ref_state;
	unsigned int max_buffer_cnt;
	unsigned char pframe_cnt;
	unsigned char can_run;
	unsigned char is_stop;
	unsigned int max_frame_size;
	unsigned int update_ref_time;
	unsigned int buffer_offset;
	unsigned int max_ref_frame_thd[2];
	unsigned int max_bag_thd;
}context_t;
typedef struct _jqy_codec
{
	context_t* trcontext;
	void (*codec_delayms)(unsigned int arg1);
	unsigned int (*codec_get_curtime_ms)(void);
	int (*codec_enc_pop_data)(void* data,unsigned int size);
	int (*codec_dec_pop_data)(void* data,unsigned int size);
	int (*codec_dec_ack_data)(void* data,unsigned int size);
	void (*codec_error_callback)(int *error);
	int (*codec_register_event)(void);
	void (*codec_data_dump_callback)(void* data,unsigned int size,int id);
}jqy_codec_t;
void codec_init(void* arg);
void codec_start(void   * arg);
void codec_pause(void   * arg);
void codec_stop(void   * arg);
void codec_push_data(void* arg1,void* arg2,void* data,unsigned int size);
unsigned int codec_get_buf_cnt(void* arg1,void* arg2);
void codec_set_max_bag_thd(void* arg1,unsigned int thd);
int codec_set_data_head_tag(void* arg1,unsigned char tag,unsigned char len);
void codec_set_max_buf_cnt(void* arg1,unsigned int arg2);
void codec_set_buf_max_cnt(void* arg1,void* arg2,unsigned int arg3);

void codec_set_max_buf_size(void* arg1,unsigned int arg2);
void codec_set_max_refbuf_thd(void* arg1,unsigned int argl,unsigned int argh);
void codec_set_update_refbuf_time(void* arg1,unsigned int arg2);
void codec_enc_task(void const *argument);
void codec_dec_task(void const *argument);
void codec_enc_pop_task(void const *argument);
void codec_dec_pop_task(void const *argument);
void codec_log_task(void const *argument);
void codec_enc_find_non_zero_plus(unsigned char *si,unsigned int in_len ,unsigned char *so,unsigned int *len_out);
int codec_dec_find_non_zero_plus(unsigned char *si,int in_len,unsigned char *so,int *len_out,int max_lout);

#ifdef __cplusplus
}
#endif 

#endif

