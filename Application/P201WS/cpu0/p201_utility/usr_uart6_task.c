#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include "cmsis_os.h"
#include "hal_nvic.h"
#include "hal_bb.h"
#include "hal_uart.h"
#include "debuglog.h"
#include "systicks.h"
#include "hal.h"
#include "factory.h"
#include "ringbuf_p201.h"

/*
1.uart retrans 

A.frame format
    frame head fixed : 0xc2 0xa3 0xd6
    frame payload len + frame seq + frame type == 2 byte
    frame payload len : bit15:8
    frame seq : bit7:1 
    frame type : bit0  0x00->frame data, 0x01->frame ack, 0x10->reset seq
    frame data : N byte 
    frame check crc8 : 1 byte,check from frame payload len to frame data


    notes: A frame head + paylaod  <= 256
B.retran mode
    NO_RETRANS : no retrans, there is no frame format, only exist user data
    LIMIT_RETRANS : max retrans number is cur_default_limit_retrans_number, over the cur_default_limit_retrans_number, 
                    if frame not recv ack ,drop the frame. cur_default_limit_retrans_number <= 5
    UNLIMIT_RETRANS : after frame send ok, must recv ack, if not , retrans frame until recv ack.

    
    notes: cur_limit_retrans_number and cur_default_retrans_mode value got from factory setting
*/

typedef uint8_t uint8;
typedef uint16_t uint16;
typedef uint32_t uint32;

#define FRAME_SYNC_HEAD0 0xc2 
#define FRAME_SYNC_HEAD1 0xa3 
#define FRAME_SYNC_HEAD2 0xd6
#define MAX_FRAME_SEQ 64
#define MAX_FRAME_QUEUE_LEN 6
#define FRAME_ACK_TIMEOUT_MS 100
#define FRAME_ACK_TIMEOUT_MS_RETRAN3 20
#define FRAME_MAX_LEN 256
#define ACK_SEQ_MAX_LEN 64
#define MAX_RECV_FRAME_QUEUE_LEN (MAX_FRAME_SEQ)
#define WAIT_NEXT_RETRAN (0xff)

typedef enum 
{
    NO_RETRANS=0,
    UNLIMIT_RETRANS=1,
    LIMIT_RETRANS=2,
}RETRANS_MODE;

typedef enum
{
    FRAME_EMPTY=0,
    FRAME_WAIT_SEND,
    FRAME_WAIT_ACK,
    FRAME_WAIT_OUTPUT,
}FRAME_STATE;

typedef enum
{
    TYPE_FRAME_DATA,
    TYPE_FRAME_ACK,
    TYPE_FRAME_RESET_SEQ,
    TYPE_FRAME_RESET_SEQ_ACK,
}FRAME_TYPE;

typedef union
{
    uint8 frame_info[2];
    struct {
        uint16 frame_payload_len : 8;
        uint16 frame_seq : 6;
        uint16 frame_type : 2;
    };
}FRAME_INFO;

typedef struct
{
    uint8 frame_data[FRAME_MAX_LEN];
    uint16 frame_data_len;
}FRAME_DATA;

typedef struct
{
    FRAME_STATE f_state;
    FRAME_INFO f_info;
    FRAME_DATA f_data;
    uint32 retran_cnt;
    uint32 frame_time;
}FRAME;

typedef struct
{
    uint8 seq[ACK_SEQ_MAX_LEN];
    uint8 r_i;
    uint8 w_i;
}ACK_SEQ_Q;

static volatile uint8 cur_default_limit_retrans_number  =  3;

static volatile RETRANS_MODE cur_default_retrans_mode = UNLIMIT_RETRANS;

static volatile uint8 cur_send_frame_seq = 0;

static volatile uint8 cur_recv_frame_seq = 0;

static FRAME g_send_frame_q[MAX_FRAME_QUEUE_LEN];

static FRAME g_recv_frame_q[MAX_RECV_FRAME_QUEUE_LEN];

static ACK_SEQ_Q ack_seq_q;

static ACK_SEQ_Q send_ack_seq_q;

static volatile uint8 tx_q_init = 0;

static volatile uint8 rx_q_init = 0;

static volatile uint8 drop_cnt = 0;

static volatile uint32 drop_bb_read_cnt = 0;

static volatile uint8 sync_seq_ok = 0;

static volatile uint32 uart_rx_drop = 0;

const char CRC8Table[]={
  0, 94, 188, 226, 97, 63, 221, 131, 194, 156, 126, 32, 163, 253, 31, 65,
  157, 195, 33, 127, 252, 162, 64, 30, 95, 1, 227, 189, 62, 96, 130, 220,
  35, 125, 159, 193, 66, 28, 254, 160, 225, 191, 93, 3, 128, 222, 60, 98,
  190, 224, 2, 92, 223, 129, 99, 61, 124, 34, 192, 158, 29, 67, 161, 255,
  70, 24, 250, 164, 39, 121, 155, 197, 132, 218, 56, 102, 229, 187, 89, 7,
  219, 133, 103, 57, 186, 228, 6, 88, 25, 71, 165, 251, 120, 38, 196, 154,
  101, 59, 217, 135, 4, 90, 184, 230, 167, 249, 27, 69, 198, 152, 122, 36,
  248, 166, 68, 26, 153, 199, 37, 123, 58, 100, 134, 216, 91, 5, 231, 185,
  140, 210, 48, 110, 237, 179, 81, 15, 78, 16, 242, 172, 47, 113, 147, 205,
  17, 79, 173, 243, 112, 46, 204, 146, 211, 141, 111, 49, 178, 236, 14, 80,
  175, 241, 19, 77, 206, 144, 114, 44, 109, 51, 209, 143, 12, 82, 176, 238,
  50, 108, 142, 208, 83, 13, 239, 177, 240, 174, 76, 18, 145, 207, 45, 115,
  202, 148, 118, 40, 171, 245, 23, 73, 8, 86, 180, 234, 105, 55, 213, 139,
  87, 9, 235, 181, 54, 104, 138, 212, 149, 203, 41, 119, 244, 170, 72, 22,
  233, 183, 85, 11, 136, 214, 52, 106, 43, 117, 151, 201, 74, 20, 246, 168,
  116, 42, 200, 150, 21, 75, 169, 247, 182, 232, 10, 84, 215, 137, 107, 53
};
unsigned char CRC8(unsigned char *p, char counter)
{
    unsigned char crc8 = 0;

    for( ; counter > 0; counter--){
        crc8 = CRC8Table[crc8^*p];
        p++;
    }
    return(crc8);

}

uint8 get_empty_frame_index(void)
{
    uint8 i;

    for(i = 0; i < MAX_FRAME_QUEUE_LEN; i++)
    {
        if(g_send_frame_q[i].f_state == FRAME_EMPTY){
            return i;
        }
    }

    return MAX_FRAME_QUEUE_LEN;
}
uint8 get_recv_empty_frame_index(void)
{
    uint8 i;

    for(i = 0; i < MAX_FRAME_QUEUE_LEN; i++)
    {
        if(g_recv_frame_q[i].f_state == FRAME_EMPTY){
            return i;
        }
    }

    return MAX_FRAME_QUEUE_LEN;
}

uint8 is_send_q_reset_seq(void)
{
    #define RETRANS_SEND_THRESHOLD 30

    uint8 i,cnt;

    cnt = 0;
    for(i = 0; i < MAX_FRAME_QUEUE_LEN; i++)
    {
        if(g_send_frame_q[i].f_state == FRAME_WAIT_ACK &&
            g_send_frame_q[i].retran_cnt > RETRANS_SEND_THRESHOLD)
        {
            cnt++;
        }
    }

    return cnt == MAX_FRAME_QUEUE_LEN;
}

uint8 is_recv_q_reset_seq(void)
{
    #define RETRANS_RECV_THRESHOLD 10

    uint8 i,cnt;

    cnt = 0;
    for(i = 0; i < MAX_FRAME_QUEUE_LEN; i++)
    {
        if(g_send_frame_q[i].f_state == FRAME_WAIT_OUTPUT &&
            g_send_frame_q[i].retran_cnt > RETRANS_RECV_THRESHOLD)
        {
            cnt++;
        }
    }

    return cnt == MAX_FRAME_QUEUE_LEN;
}

int seq_exception_process(void)
{
    uint8 send_excp,recv_excp;
    /*
        1,grd unlock or sky unlock, will reset seq
    */
    if(cur_default_retrans_mode == UNLIMIT_RETRANS)
    {
        /*recv_excp = is_recv_q_reset_seq();
        if(recv_excp)
        {
            drop_cnt = 0;
            trx_q_init = 0;
            DLOG_Warning("recv reset seq");
            return 0;
        }*/

        if(drop_cnt > MAX_FRAME_QUEUE_LEN * 10)
        {
            drop_cnt = 0;
            tx_q_init = 0;
            rx_q_init = 0;
            DLOG_Warning("drop too much reset seq");
            return 0;
        }
        
        send_excp = is_send_q_reset_seq();
        if(send_excp)
        {
            drop_cnt = 0;
            tx_q_init = 0;
            rx_q_init = 0;
            DLOG_Warning("send reset seq");
            return 0;
        }

        return 0;
    }
    else
    {
        cur_recv_frame_seq++;
        if(cur_recv_frame_seq >= MAX_FRAME_SEQ){
            cur_recv_frame_seq = 0;
        }

        return 1;
    }
}
/*
*/
uint8 is_new_seq(uint8 cur_seq,uint8 seq)
{
    if(cur_seq < seq )
    {
        if(seq - cur_seq < MAX_RECV_FRAME_QUEUE_LEN/2)
        {
            return 1;
        }
    }
    else if(cur_seq > seq )
    {
        if(seq + MAX_FRAME_SEQ - cur_seq <  MAX_RECV_FRAME_QUEUE_LEN/2)
        {
            return 1;
        }
    }

    return 0;
}
uint8 is_old_seq(uint8 cur_seq,uint8 seq)
{
    if(cur_seq < seq )
    {
        if(MAX_FRAME_SEQ + cur_seq - seq < MAX_FRAME_QUEUE_LEN*2)//+2,recv
        {
            return 1;
        }

    }
    else if(cur_seq > seq )
    {
        if(cur_seq - seq < MAX_FRAME_QUEUE_LEN*2)
        {
            return 1;
        }

    }

    return 0;
}

uint8 check_seq_is_need_cache(uint8 cur_seq,uint8 seq,uint8 *is_need_ack)
{
    uint8 ret;
    
    *is_need_ack = 0;

    ret = is_new_seq(cur_seq,seq);
    if(ret)
    {
        return ret;
    }

    ret = is_old_seq(cur_seq,seq);
    if(ret)
    {
        *is_need_ack = 1;
        return 0;
    }

    DLOG_Warning("not new or old ,is bug cur %d recv %d",cur_seq,seq);

    /*
    if(cur_seq < seq ){
        if(seq - cur_seq < MAX_FRAME_QUEUE_LEN){
            return 1;
        }
        else{
            if(( MAX_FRAME_SEQ + cur_seq - seq) < MAX_FRAME_QUEUE_LEN){
                DLOG_Info("seq is old, cur %d recv %d",cur_seq,seq);
                *is_need_ack = 1;
            }
            else{
                DLOG_Warning("over cache max %d , cur seq %d recv seq %d",MAX_FRAME_QUEUE_LEN,cur_seq,seq);
            }
            return 0;
        }
    }
    else if(cur_seq > seq){
        if((seq + MAX_FRAME_SEQ - cur_seq ) < MAX_FRAME_QUEUE_LEN){
            return 1;
        }
        else{
            if(cur_seq - seq < MAX_FRAME_QUEUE_LEN ){
                DLOG_Info("rollback seq is old, cur %d recv %d",cur_seq,seq);
                *is_need_ack = 1;
            }
            else
            {
                DLOG_Warning("rollback over cache max %d , cur seq %d recv seq %d",MAX_FRAME_QUEUE_LEN,cur_seq,seq);
            }
            return 0;
        }
    }
    else{
        DLOG_Error("please not here");
    }*/
    
    return 0;
}

/*
*/
uint8 check_ack(uint8 cur_seq,uint8 seq,uint8 *is_need_ack)
{
    *is_need_ack = 0;
    
    if(cur_seq < seq ){
        if(seq - cur_seq < MAX_FRAME_QUEUE_LEN){
            return 1;
        }
        else{
            if(( MAX_FRAME_SEQ + cur_seq - seq) <= MAX_FRAME_QUEUE_LEN){
                //DLOG_Warning("seq is old, cur %d recv %d",cur_seq,seq);
                *is_need_ack = 1;
            }
            else{
                DLOG_Warning("ack over max %d , cur seq %d send seq %d",MAX_FRAME_QUEUE_LEN,cur_seq,seq);
            }
            return 0;
        }
    }
    else if(cur_seq > seq){
        if((seq + MAX_FRAME_SEQ - cur_seq ) <= MAX_FRAME_QUEUE_LEN){
            return 1;
        }
        else{
            if(cur_seq - seq <= MAX_FRAME_QUEUE_LEN ){
                //DLOG_Warning("rollback seq is old, cur %d recv %d",cur_seq,seq);
                *is_need_ack = 1;
            }
            else
            {
                DLOG_Warning("ack over max %d , cur seq %d recv seq %d",MAX_FRAME_QUEUE_LEN,cur_seq,seq);
            }
            return 0;
        }
    }
    else{
        //DLOG_Error("please not here");
        *is_need_ack = 1;
    }
    
    return 0;
}

uint8 get_recv_output_frame_index(void)
{
    uint8 i;

    for(i = 0; i < MAX_FRAME_QUEUE_LEN; i++)
    {
        if(g_recv_frame_q[i].f_state == FRAME_WAIT_OUTPUT){
            return i;
        }
    }

    return MAX_FRAME_QUEUE_LEN;
}
uint8 is_recv_output_frame_seq_match(uint8 seq)
{
    uint8 i;

    for(i = 0; i < MAX_FRAME_QUEUE_LEN; i++)
    {
        if(g_recv_frame_q[i].f_state == FRAME_WAIT_OUTPUT && g_recv_frame_q[i].f_info.frame_seq == seq){
            g_recv_frame_q[i].retran_cnt += 1;
            return i;
        }
    }

    return MAX_FRAME_QUEUE_LEN;
}
uint8 is_frame_cached(uint8 seq)
{
    return is_recv_output_frame_seq_match(seq);
}

uint8 get_send_frame_index(void)
{
    uint8 i;

    for(i = 0; i < MAX_FRAME_QUEUE_LEN; i++)
    {
        if(g_send_frame_q[i].f_state == FRAME_WAIT_SEND){
            return i;
        }
    }

    return MAX_FRAME_QUEUE_LEN;
}

uint8 set_send_frame2empty_by_seq(uint8 seq)
{
    uint8 i,is_old;

    for(i = 0; i < MAX_FRAME_QUEUE_LEN; i++)
    {
        if(g_send_frame_q[i].f_state == FRAME_WAIT_ACK)
        {
            if(seq == g_send_frame_q[i].f_info.frame_seq)
            {
                g_send_frame_q[i].f_state = FRAME_EMPTY;
                return i;
            }
            /*else
            {
                is_old = is_old_seq(seq,g_send_frame_q[i].f_info.frame_seq);
                if(is_old)
                {
                    g_send_frame_q[i].f_state = FRAME_EMPTY;
                }
            }*/
        }
    }

    return MAX_FRAME_QUEUE_LEN;

}
uint8 set_recv_frame2empty_by_seq(uint8 seq)
{
    uint8 i;

    for(i = 0; i < MAX_FRAME_QUEUE_LEN; i++)
    {
        if(g_recv_frame_q[i].f_state == FRAME_WAIT_OUTPUT && g_recv_frame_q[i].f_info.frame_seq == seq){
            g_recv_frame_q[i].retran_cnt = 0;
            g_recv_frame_q[i].f_state = FRAME_EMPTY;
            return i;
        }
    }

    return MAX_FRAME_QUEUE_LEN;

}

uint8 check_frame2ack_timeout(void)
{
    uint8 i,need_wait,got_it_flag,got_it_index;
    uint32 time;

    need_wait = 0;
    got_it_flag = 0;
    time = 0xffffffff;
    for(i = 0; i < MAX_FRAME_QUEUE_LEN; i++)
    {
        if(cur_default_retrans_mode == UNLIMIT_RETRANS)
        {
            if(g_send_frame_q[i].f_state == FRAME_WAIT_ACK)
            {
                //retran cnt > 3 and timeout and to get the old msg to retry
                if((g_send_frame_q[i].retran_cnt > 3))
                {
                    if((SysTicks_GetDiff(g_send_frame_q[i].frame_time,HAL_GetSysMsTick()) > FRAME_ACK_TIMEOUT_MS))
                    {
                        got_it_flag = 1;
                        if(g_send_frame_q[i].frame_time < time)
                        {
                            got_it_index = i;
                            time = g_send_frame_q[i].frame_time;
                        }
                        //return i;
                    }

                    need_wait = 1;
                }

            }
        }
    }
    
    if(got_it_flag)
    {
        return got_it_index;
    }

    if(need_wait)
    {
        return WAIT_NEXT_RETRAN;
    }

    got_it_flag = 0;
    time = 0xffffffff;

    for(i = 0; i < MAX_FRAME_QUEUE_LEN; i++)
    {
        if(cur_default_retrans_mode == UNLIMIT_RETRANS)
        {
            //timeout and to get the old msg to retry
            if(g_send_frame_q[i].f_state == FRAME_WAIT_ACK)
            {
                if((SysTicks_GetDiff(g_send_frame_q[i].frame_time,HAL_GetSysMsTick()) > FRAME_ACK_TIMEOUT_MS))
                {
                    got_it_flag = 1;
                    if(g_send_frame_q[i].frame_time < time)
                    {
                        got_it_index = i;
                        time = g_send_frame_q[i].frame_time;
                    }
                    //return i;
                }
            }
        }
    }

    if(got_it_flag)
    {
        return got_it_index;
    }

    return MAX_FRAME_QUEUE_LEN;

}

int find_frame_sync_head(ringbuf_t rb, int len, int *skip_offset)
{
	int i;

	for(i=0;i<len-2;i++)
	{
		if(ringbuf_watch(rb,i) == FRAME_SYNC_HEAD0 && ringbuf_watch(rb,i+1) == FRAME_SYNC_HEAD1 
            && ringbuf_watch(rb,i+2) == FRAME_SYNC_HEAD2)
		{
			*skip_offset  = i;
			return 1;
		}
	}

	*skip_offset = len-2;
	return 0;
}

void add_ack_seq(uint8 seq)
{
    ack_seq_q.seq[ack_seq_q.w_i] = seq;
    ack_seq_q.w_i++;
    if(ack_seq_q.w_i >= ACK_SEQ_MAX_LEN){
        ack_seq_q.w_i = 0;
    }
}
void add_send_ack_seq(uint8 seq)
{
    send_ack_seq_q.seq[send_ack_seq_q.w_i] = seq;
    send_ack_seq_q.w_i++;
    if(send_ack_seq_q.w_i >= ACK_SEQ_MAX_LEN){
        send_ack_seq_q.w_i = 0;
    }
}

uint8 get_ack_seq(void)
{
    uint8 seq;
    
    if(ack_seq_q.w_i == ack_seq_q.r_i){
        return MAX_FRAME_SEQ;
    }
    seq = ack_seq_q.seq[ack_seq_q.r_i];
    ack_seq_q.r_i++;
    if(ack_seq_q.r_i >= ACK_SEQ_MAX_LEN){
        ack_seq_q.r_i = 0;
    }
    return seq;
    
}
uint8 get_send_ack_seq(void)
{
    uint8 seq;
    
    if(send_ack_seq_q.w_i == send_ack_seq_q.r_i){
        return MAX_FRAME_SEQ;
    }
    seq = send_ack_seq_q.seq[send_ack_seq_q.r_i];
    send_ack_seq_q.r_i++;
    if(send_ack_seq_q.r_i >= ACK_SEQ_MAX_LEN){
        send_ack_seq_q.r_i = 0;
    }
    return seq;
    
}

void init_send_queue(void)
{
    memset(&g_send_frame_q,0,sizeof(g_send_frame_q));
    cur_send_frame_seq = 0;
}
void init_recv_queue(void)
{
    memset(&g_recv_frame_q,0,sizeof(g_recv_frame_q));
    memset(&ack_seq_q,0,sizeof(ack_seq_q));
    memset(&send_ack_seq_q,0,sizeof(send_ack_seq_q));
    cur_recv_frame_seq = 0;
}

int get_retran_mode_factsetting(void)
{
    STRU_MIMO_MODE *p_mimo = NULL;

    p_mimo = FCT_GetNodeAndData(FACTORY_SUBNODE_MIMO_MODE_ID, NULL);
    if (p_mimo != NULL)
    {
        if(p_mimo->spi_num < 2){
            cur_default_retrans_mode = (RETRANS_MODE)p_mimo->spi_num;
        }
        else{
            cur_default_retrans_mode = LIMIT_RETRANS;
            cur_default_limit_retrans_number = p_mimo->spi_num;
        }
    }
    else{
        DLOG_Warning("failed");
    }

    DLOG_Warning("retran mode %d, num %d",cur_default_retrans_mode,cur_default_limit_retrans_number);

    return 0;
}

extern uint8_t lock_flag;

extern osSemaphoreId uart_semaphore_id;

#define TTL_DEFAULT_BB_SESSION_PORT       BB_COM_SESSION_2

static ENUM_HAL_UART_BAUDR default_baudr = HAL_UART_BAUDR_115200;

static ENUM_BB_COM_SESSION_ID GLOBAL_BB_SESSION = TTL_DEFAULT_BB_SESSION_PORT;

static ringbuf_t ring_hal_uart_rx = NULL;

static ringbuf_t ring_bb_uart_rx = NULL;

static int hal_uart_rx_cnt = 0, bb_uart_rx_cnt = 0;
static int hal_uart_tx_cnt = 0, bb_uart_tx_cnt = 0;

static int bb_dev_type;

static int check_ttl_factory_setting(void)
{
    int fct_param = -1;
    int param0 = 0, param1 = 0, param2 = 0;

    STRU_cfgNode *node;
    STRU_UART_BAUDR *p_uart_baudr = NULL;

    p_uart_baudr = FCT_GetNodeAndData(FACTORY_SUBNODE_UART_BAUDR_ID, NULL);
    if (p_uart_baudr != NULL)
    {
        param0 = p_uart_baudr->st_uartBaudr[0];
        param0 = (param0 & 0xF0) >> 4;

        fct_param = param1 = p_uart_baudr->st_uartBaudr[1];
        param1 = (param1 & 0xF0) >> 4;

        param2 = p_uart_baudr->st_uartBaudr[2];
        param2 = (param2 & 0xF0) >> 4;

		DLOG_Critical("Param0 %04x, Param1 %04x, Param2 %04x, FCT Param %04x\n", param0, param1, param2, fct_param);
        if ((param1 != 0) && ((param1 == param0) || (param1 == param2)))        //Factory Setting Conflict
            return (fct_param & 0x0F);

        if ((param1 == 0) && ((0 != param0) || (0 != param2)))
            return -2;

        return fct_param;
    }

    return -1;
}

static int get_factory_buadrate(void)
{
    int baudr = -1;
    STRU_cfgNode *node;
    STRU_UART_BAUDR *p_uart_baudr;

    p_uart_baudr = FCT_GetNodeAndData(FACTORY_SUBNODE_UART_BAUDR_ID, NULL);
    baudr = p_uart_baudr->st_uartBaudr[1];
    if (baudr >= HAL_UART_BAUDR_9600 && baudr <= HAL_UART_BAUDR_460800) {
        switch(baudr) {
            case HAL_UART_BAUDR_9600:
                DLOG_Critical("UART HAL_UART_BAUDR_9600\n");
                break;
            case HAL_UART_BAUDR_19200:
                DLOG_Critical("UART HAL_UART_BAUDR_19200\n");
                break;
            case HAL_UART_BAUDR_38400:
                DLOG_Critical("UART HAL_UART_BAUDR_38400\n");
                break;
            case HAL_UART_BAUDR_57600:
                DLOG_Critical("UART HAL_UART_BAUDR_57600\n");
                break;
            case HAL_UART_BAUDR_115200:
                DLOG_Critical("UART HAL_UART_BAUDR_115200\n");
                break;
            case HAL_UART_BAUDR_230400:
                DLOG_Critical("UART HAL_UART_BAUDR_230400\n");
                break;
            case HAL_UART_BAUDR_256000:
                DLOG_Critical("UART HAL_UART_BAUDR_256000\n");
                break;
            case HAL_UART_BAUDR_380400:
                DLOG_Critical("UART HAL_UART_BAUDR_380400\n");
                break;
            case HAL_UART_BAUDR_460800:
                DLOG_Critical("UART HAL_UART_BAUDR_460800\n");
                break;

        }

        return baudr;
    }

    return -1;
}

static void Rcv6_Data_Handler(void *p)
{
    #define BB_READ_MAX_LEN 200
    uint32_t cnt;
    uint8_t buffer[BB_READ_MAX_LEN];
    HAL_RET_T ret;

    ret = HAL_BB_ComReceiveMsg(GLOBAL_BB_SESSION, buffer, BB_READ_MAX_LEN, &cnt);
    if(ret != HAL_OK)
    {
        DLOG_Error("failed read bbcom");
        return;
    }

    if(cnt > 0 && cnt <= BB_READ_MAX_LEN)
    {
        if(ringbuf_memcpy_into_1(ring_bb_uart_rx, buffer, cnt) < 0)
        {
            //DLOG_Warning("overflow, drop cnt=%d",cnt);
            drop_bb_read_cnt += cnt;
        }
        bb_uart_rx_cnt += cnt;
    }
    else
    {
        DLOG_Info("bb read %d",cnt);
    }
}

static uint32_t Uart6_Irq_Handler(uint8_t *pu8_rxBuf, uint8_t u8_len)
{
    if(ringbuf_memcpy_into_1(ring_hal_uart_rx, pu8_rxBuf, u8_len) < 0)
    {
        uart_rx_drop += u8_len;
    }
    hal_uart_rx_cnt += u8_len;
}

void send_frame_acks(uint8 *seq, uint8 seq_cnt)
{
    uint8 ack_frame[6+ACK_SEQ_MAX_LEN];
    FRAME_INFO ack_frame_info;
    HAL_RET_T ret;

    ack_frame[0] = FRAME_SYNC_HEAD0;
    ack_frame[1] = FRAME_SYNC_HEAD1;
    ack_frame[2] = FRAME_SYNC_HEAD2;
    ack_frame_info.frame_payload_len = seq_cnt;
    ack_frame_info.frame_seq = 0;
    ack_frame_info.frame_type = TYPE_FRAME_ACK;
    ack_frame[3] = ack_frame_info.frame_info[0];
    ack_frame[4] = ack_frame_info.frame_info[1];
    memcpy(ack_frame+5,seq,seq_cnt);
    ack_frame[seq_cnt+5] = CRC8(ack_frame+5,seq_cnt);
    
    osSemaphoreWait(uart_semaphore_id, 0);
    ret = HAL_BB_ComSendMsg(GLOBAL_BB_SESSION,ack_frame, seq_cnt + 6);
    osSemaphoreRelease(uart_semaphore_id);
    if(ret != HAL_OK){
        DLOG_Error("failed ret %d",ret);
    }

}

void send_frame_ack(uint8 seq)
{
    uint8 ack_frame[6];
    FRAME_INFO ack_frame_info;
    HAL_RET_T ret;

    ack_frame[0] = FRAME_SYNC_HEAD0;
    ack_frame[1] = FRAME_SYNC_HEAD1;
    ack_frame[2] = FRAME_SYNC_HEAD2;
    ack_frame_info.frame_payload_len = 0;
    ack_frame_info.frame_seq = seq;
    ack_frame_info.frame_type = TYPE_FRAME_ACK;
    ack_frame[3] = ack_frame_info.frame_info[0];
    ack_frame[4] = ack_frame_info.frame_info[1];
    //memcpy(ack_frame+5,seq,seq_cnt);
    ack_frame[5] = 0;//CRC8(ack_frame+5,seq_cnt);
    
    osSemaphoreWait(uart_semaphore_id, 0);
    ret = HAL_BB_ComSendMsg(GLOBAL_BB_SESSION,ack_frame, 6);
    osSemaphoreRelease(uart_semaphore_id);
    if(ret != HAL_OK){
        DLOG_Error("failed ret %d",ret);
    }

}


void send_frame_seq_sync(uint8 seq)
{
    uint8 ack_frame[6];
    FRAME_INFO ack_frame_info;
    HAL_RET_T ret;

    ack_frame[0] = FRAME_SYNC_HEAD0;
    ack_frame[1] = FRAME_SYNC_HEAD1;
    ack_frame[2] = FRAME_SYNC_HEAD2;
    ack_frame_info.frame_payload_len = 0;
    ack_frame_info.frame_seq = seq;
    ack_frame_info.frame_type = TYPE_FRAME_RESET_SEQ;
    ack_frame[3] = ack_frame_info.frame_info[0];
    ack_frame[4] = ack_frame_info.frame_info[1];
    //memcpy(ack_frame+5,seq,seq_cnt);
    ack_frame[5] = 0;//CRC8(ack_frame+5,seq_cnt);
    
    osSemaphoreWait(uart_semaphore_id, 0);
    ret = HAL_BB_ComSendMsg(GLOBAL_BB_SESSION,ack_frame, 6);
    osSemaphoreRelease(uart_semaphore_id);
    if(ret != HAL_OK){
        DLOG_Error("failed ret %d",ret);
    }

}



void send_frame_seq_sync_ack(uint8 seq)
{
    uint8 ack_frame[6];
    FRAME_INFO ack_frame_info;
    HAL_RET_T ret;

    ack_frame[0] = FRAME_SYNC_HEAD0;
    ack_frame[1] = FRAME_SYNC_HEAD1;
    ack_frame[2] = FRAME_SYNC_HEAD2;
    ack_frame_info.frame_payload_len = 0;
    ack_frame_info.frame_seq = seq;
    ack_frame_info.frame_type = TYPE_FRAME_RESET_SEQ_ACK;
    ack_frame[3] = ack_frame_info.frame_info[0];
    ack_frame[4] = ack_frame_info.frame_info[1];
    //memcpy(ack_frame+5,seq,seq_cnt);
    ack_frame[5] = 0;//CRC8(ack_frame+5,seq_cnt);
    
    osSemaphoreWait(uart_semaphore_id, 0);
    ret = HAL_BB_ComSendMsg(GLOBAL_BB_SESSION,ack_frame, 6);
    osSemaphoreRelease(uart_semaphore_id);
    if(ret != HAL_OK){
        DLOG_Error("failed ret %d",ret);
    }

}

static void bypass_ttl_task_excp_proc(void const *argument)
{
    while(1)
    {
        HAL_Delay(100);
        seq_exception_process();
    }
}

static void bb_uart_tx6hal_task_byparse(void const *argument)
{
    HAL_RET_T ret;
    char buffer[1024];
    int len,pos,i;
    uint8 index,crc,need_scan_recv_q;
    FRAME_INFO rx_frame_info;

    while (1)
    {
        if(!lock_flag)
        {
            HAL_Delay(14);
            continue;
        }

        len = ringbuf_bytes_used(ring_bb_uart_rx);
        if(len == 0)
        {
            HAL_Delay(8);
            continue;
        }

        if(len > 1024)
        {
            DLOG_Error("len error %d,%08x,%08x", len, ringbuf_head(ring_bb_uart_rx), ringbuf_tail(ring_bb_uart_rx));
            len = 1024;
        }

        pos = 0;
        find_frame_sync_head(ring_bb_uart_rx,len,&pos);
        if(pos > 0){
            ringbuf_move(ring_bb_uart_rx,pos);
            DLOG_Warning("skip %d",pos);
            goto wait_a_while;
        }

        if(len < 6){
            goto wait_a_while;
        }

        rx_frame_info.frame_info[0] = ringbuf_watch(ring_bb_uart_rx,3);
        rx_frame_info.frame_info[1] = ringbuf_watch(ring_bb_uart_rx,4);
        if(len < rx_frame_info.frame_payload_len + 6){
            goto wait_a_while;
        }

        ringbuf_memcpy_from(buffer,ring_bb_uart_rx,rx_frame_info.frame_payload_len + 6);

        crc = CRC8(buffer+5,rx_frame_info.frame_payload_len);
        if(crc != buffer[rx_frame_info.frame_payload_len+5])
        {
            DLOG_Warning("crc error %x-%x",crc,buffer[rx_frame_info.frame_payload_len+5]);
            goto wait_a_while;
        }

        if(rx_frame_info.frame_type == TYPE_FRAME_ACK){
            for(i=0;i<rx_frame_info.frame_payload_len;i++){
                //add_ack_seq(rx_frame_info.frame_seq);
                add_ack_seq(buffer[5+i]);
            }
            goto wait_a_while;
        }
        else if(rx_frame_info.frame_type == TYPE_FRAME_DATA)
        {
            need_scan_recv_q = 0;
            if(rx_frame_info.frame_seq == cur_recv_frame_seq)
            {
                ret = HAL_UART_TxData(HAL_UART_COMPONENT_6, buffer+5, rx_frame_info.frame_payload_len, HAL_UART_DEFAULT_TIMEOUTMS * 1000);
                if(ret != HAL_OK)
                {
                    DLOG_Error("failed ret %d",ret);
                }
                drop_cnt = 0;
                cur_recv_frame_seq++;
                if(cur_recv_frame_seq >= MAX_FRAME_SEQ){
                    cur_recv_frame_seq = 0;
                }
                need_scan_recv_q = 1;
                if (rx_frame_info.frame_payload_len > 0)
                    hal_uart_tx_cnt += rx_frame_info.frame_payload_len;
            }
            else
            {
                if(is_new_seq(cur_recv_frame_seq,rx_frame_info.frame_seq))
                {
                    if(g_recv_frame_q[rx_frame_info.frame_seq].f_state == FRAME_EMPTY)
                    {
                        drop_cnt = 0;
                        index = rx_frame_info.frame_seq;
                        memcpy(g_recv_frame_q[index].f_data.frame_data,buffer+5,rx_frame_info.frame_payload_len);
                        g_recv_frame_q[index].f_data.frame_data_len = rx_frame_info.frame_payload_len;
                        g_recv_frame_q[index].f_info.frame_info[0] = buffer[3];
                        g_recv_frame_q[index].f_info.frame_info[1] = buffer[4];
                        g_recv_frame_q[index].frame_time = HAL_GetSysMsTick();
                        g_recv_frame_q[index].retran_cnt += 1;
                        g_recv_frame_q[index].f_state = FRAME_WAIT_OUTPUT;
                        DLOG_Warning("cached seq %d , cur %d",rx_frame_info.frame_seq,cur_recv_frame_seq);
                    }
                    else
                    {
                        g_recv_frame_q[rx_frame_info.frame_seq].retran_cnt += 1;
                        DLOG_Warning("cache again seq=%d cnt=%d",rx_frame_info.frame_seq,g_recv_frame_q[rx_frame_info.frame_seq].retran_cnt);
                    }
                }
                else
                {
                    DLOG_Warning("drop %d - %d",cur_recv_frame_seq,rx_frame_info.frame_seq);
                    drop_cnt++;
                }
            }

            if(need_scan_recv_q)
            {
                while(1)
                {
                    index = (cur_recv_frame_seq);
                    if(g_recv_frame_q[index].f_state == FRAME_WAIT_OUTPUT)
                    {
                        ret = HAL_UART_TxData(HAL_UART_COMPONENT_6, g_recv_frame_q[index].f_data.frame_data, 
                            g_recv_frame_q[index].f_data.frame_data_len, HAL_UART_DEFAULT_TIMEOUTMS * 1000);
                        if(ret != HAL_OK)
                        {
                            DLOG_Error("failed ret %d",ret);
                        }
                        g_recv_frame_q[index].f_state = FRAME_EMPTY;
                        g_recv_frame_q[index].retran_cnt = 0;
                        if (g_recv_frame_q[index].f_data.frame_data_len > 0)
                            hal_uart_tx_cnt += g_recv_frame_q[index].f_data.frame_data_len;
                        cur_recv_frame_seq++;
                        if(cur_recv_frame_seq >= MAX_FRAME_SEQ){
                            cur_recv_frame_seq = 0;
                        }
                    }
                    else
                    {
                        break;
                    }
                
                }
                
            }
            
            //send_frame_ack(rx_frame_info.frame_seq);
            add_send_ack_seq(rx_frame_info.frame_seq);

            
        }
        else if(rx_frame_info.frame_type == TYPE_FRAME_RESET_SEQ)
        {
            init_recv_queue();
            send_frame_seq_sync_ack(0);
            DLOG_Warning("recv sync seq");
        }
        else if(rx_frame_info.frame_type == TYPE_FRAME_RESET_SEQ_ACK)
        {
            sync_seq_ok = 1;
            DLOG_Warning("send sync seq");
        }
        else{
            DLOG_Error("frame type error %d",rx_frame_info.frame_type);
        }
wait_a_while:
        HAL_Delay(4);
    }

}

static void hal_uart_tx6bb_task_byretry(void const *argument)
{
#define MAX_BB_UPLINK_SEND_SIZE 70
#define MAX_BB_DOWNLINK_SEND_SIZE 200
#define MAX(a,b) (((a) > (b)) ?  (a) :  (b) )
    HAL_RET_T ret;
    char buffer[MAX(MAX_BB_UPLINK_SEND_SIZE, MAX_BB_DOWNLINK_SEND_SIZE)];
    int len;
    int max_send_len;
    uint8 index,check_sum,recv_ack_seq;
    uint8 send_ack[ACK_SEQ_MAX_LEN],send_ack_cnt;

    if(bb_dev_type)
    {
        max_send_len = MAX_BB_UPLINK_SEND_SIZE;
        DLOG_Warning("uplink pack len %d",max_send_len);

    }
    else
    {
        max_send_len = MAX_BB_DOWNLINK_SEND_SIZE;
    }

    while (1)
    {
        if(!lock_flag)
        {
            tx_q_init = 0;
            sync_seq_ok = 0;
            HAL_Delay(14);
            continue;
        }

        if(!tx_q_init)
        {
            init_send_queue();
            tx_q_init = 1;
        }

        if(!sync_seq_ok)
        {
            send_frame_seq_sync(0);
            HAL_Delay(120);
            continue;
        }

        do
        {
            recv_ack_seq = get_ack_seq();
            if(recv_ack_seq < MAX_FRAME_SEQ)
            {
                //DLOG_Warning("%d",recv_ack_seq);
                set_send_frame2empty_by_seq(recv_ack_seq);
            }
        }while(recv_ack_seq < MAX_FRAME_SEQ);


        send_ack_cnt = 0;
        do
        {
            recv_ack_seq = get_send_ack_seq();
            if(recv_ack_seq < MAX_FRAME_SEQ)
            {
                //send_frame_ack(recv_ack_seq);
                send_ack[send_ack_cnt] = recv_ack_seq;
                send_ack_cnt++;
            }
        }while(recv_ack_seq < MAX_FRAME_SEQ);
        if(send_ack_cnt > 0)
        {
            send_frame_acks(send_ack,send_ack_cnt);
        }

        index = get_send_frame_index();
        if(index < MAX_FRAME_QUEUE_LEN){
            DLOG_Warning("goto send again");
            HAL_Delay(20);
            goto send_frame;
        }

        index = check_frame2ack_timeout();
        if(index == WAIT_NEXT_RETRAN)
        {
            HAL_Delay(20);
            continue;
        }
        else if(index < MAX_FRAME_QUEUE_LEN)
        {
            if(cur_default_retrans_mode == UNLIMIT_RETRANS){
                g_send_frame_q[index].f_state = FRAME_WAIT_SEND;
                DLOG_Warning("retry seq=%d, cnt=%d",g_send_frame_q[index].f_info.frame_seq,g_send_frame_q[index].retran_cnt);
                goto send_frame;
            }
            else{
                if(g_send_frame_q[index].retran_cnt >= cur_default_limit_retrans_number){
                    g_send_frame_q[index].f_state = FRAME_EMPTY;
                    g_send_frame_q[index].retran_cnt = 0;
                    DLOG_Info("drop frame seq=%d len=%d",g_send_frame_q[index].f_info.frame_seq,g_send_frame_q[index].f_info.frame_payload_len);
                }
                else{
                    g_send_frame_q[index].f_state = FRAME_WAIT_SEND;
                    DLOG_Info("retry seq=%d, cnt=%d",g_send_frame_q[index].f_info.frame_seq,g_send_frame_q[index].retran_cnt);
                    goto send_frame;
                }
            }
        }

        len = ringbuf_bytes_used(ring_hal_uart_rx);
        if(len == 0)
        {
            HAL_Delay(8);
            continue;
        }

        if(len > 1024)
        {
            DLOG_Error("len error %d,%08x,%08x", len, ringbuf_head(ring_hal_uart_rx), ringbuf_tail(ring_hal_uart_rx));
        }

        if(len > max_send_len)
        {
            len = max_send_len;
        }

        index = get_empty_frame_index();
        if(index >= MAX_FRAME_QUEUE_LEN){
            DLOG_Info("send frame queue is full");
            HAL_Delay(8);
            continue;
        }

		if (len > 0)
			bb_uart_tx_cnt += len;

        //make frame format data
        g_send_frame_q[index].f_data.frame_data[0] = FRAME_SYNC_HEAD0;
        g_send_frame_q[index].f_data.frame_data[1] = FRAME_SYNC_HEAD1;
        g_send_frame_q[index].f_data.frame_data[2] = FRAME_SYNC_HEAD2;
        g_send_frame_q[index].f_info.frame_payload_len = len;
        g_send_frame_q[index].f_info.frame_seq = cur_send_frame_seq;
        g_send_frame_q[index].f_info.frame_type = TYPE_FRAME_DATA;
        g_send_frame_q[index].f_data.frame_data[3] = g_send_frame_q[index].f_info.frame_info[0];
        g_send_frame_q[index].f_data.frame_data[4] = g_send_frame_q[index].f_info.frame_info[1];
        ringbuf_memcpy_from(g_send_frame_q[index].f_data.frame_data+5,ring_hal_uart_rx,len);
        g_send_frame_q[index].f_data.frame_data[5+len] = CRC8(g_send_frame_q[index].f_data.frame_data+5,len);
        g_send_frame_q[index].f_data.frame_data_len = 6 + len;

        g_send_frame_q[index].f_state = FRAME_WAIT_SEND;
        g_send_frame_q[index].retran_cnt = 0;
        cur_send_frame_seq++;
        if(cur_send_frame_seq >= MAX_FRAME_SEQ){
            cur_send_frame_seq = 0;
        }

send_frame:
        osSemaphoreWait(uart_semaphore_id, 0);
        ret = HAL_BB_ComSendMsg(GLOBAL_BB_SESSION, g_send_frame_q[index].f_data.frame_data, g_send_frame_q[index].f_data.frame_data_len);
        osSemaphoreRelease(uart_semaphore_id);
        if(ret == HAL_OK){
            g_send_frame_q[index].f_state = FRAME_WAIT_ACK;
            g_send_frame_q[index].frame_time = HAL_GetSysMsTick();
            g_send_frame_q[index].retran_cnt += 1;
        }
        else
        {
            DLOG_Error("failed ret %d",ret);
        }
        HAL_Delay(4);
    }

}

static void hal_uart_tx6bb_task(void const *argument)
{
    #define MAX_BB_UPLINK_SEND_SIZE 70
    #define MAX_BB_DOWNLINK_SEND_SIZE 200
    #define MAX(a,b) (((a) > (b)) ?  (a) :  (b) )
    HAL_RET_T ret;
    char buffer[MAX(MAX_BB_UPLINK_SEND_SIZE, MAX_BB_DOWNLINK_SEND_SIZE)];
    int len;
    int max_send_len;

    if(bb_dev_type)
    {
        max_send_len = MAX_BB_UPLINK_SEND_SIZE;
        DLOG_Warning("uplink pack len %d",max_send_len);

    }
    else
    {
        max_send_len = MAX_BB_DOWNLINK_SEND_SIZE;
    }

    while (1)
    {
        len = ringbuf_bytes_used(ring_hal_uart_rx);
        if(len == 0)
        {
            HAL_Delay(8);
            continue;
        }

        if(len > 1024)
        {
            DLOG_Error("len error %d,%08x,%08x", len, ringbuf_head(ring_hal_uart_rx), ringbuf_tail(ring_hal_uart_rx));
        }

        if(len > max_send_len)
        {
            len = max_send_len;
        }

        ringbuf_memcpy_from(buffer,ring_hal_uart_rx,len);
		if (len > 0)
			bb_uart_tx_cnt += len;

        osSemaphoreWait(uart_semaphore_id, 0);
        ret = HAL_BB_ComSendMsg(GLOBAL_BB_SESSION, buffer, len);
        osSemaphoreRelease(uart_semaphore_id);
        if(ret != HAL_OK)
        {
            DLOG_Error("failed ret %d",ret);
        }

        HAL_Delay(4);
    }
}

static void bb_uart_tx6hal_task(void const *argument)
{
    HAL_RET_T ret;
    char buffer[1024];
    int len;

    while (1)
    {
        len = ringbuf_bytes_used(ring_bb_uart_rx);
        if(len == 0)
        {
            HAL_Delay(14);
            continue;
        }

        if(len > 1024)
        {
            DLOG_Error("len error %d,%08x,%08x", len, ringbuf_head(ring_bb_uart_rx), ringbuf_tail(ring_bb_uart_rx));
            len = 1024;
        }

        ringbuf_memcpy_from(buffer,ring_bb_uart_rx,len);
		if (len > 0)
			hal_uart_tx_cnt += len;

        ret = HAL_UART_TxData(HAL_UART_COMPONENT_6, buffer, len, HAL_UART_DEFAULT_TIMEOUTMS * 1000);
        if(ret != HAL_OK)
        {
            DLOG_Error("failed ret %d",ret);
        }

        HAL_Delay(4);
    }
}

static void COMTASK_DebugFunction(void const *argument)
{
    while (1)
    {
        DLOG_Warning("TTL ------> hal_uart_rx = %d",hal_uart_rx_cnt);
        DLOG_Warning("TTL ------> bb_rx = %d",bb_uart_rx_cnt);
        DLOG_Warning("TTL ------> hal_uart_tx = %d",hal_uart_tx_cnt);
        DLOG_Warning("TTL ------> bb_tx = %d",bb_uart_tx_cnt);
        DLOG_Warning("TTL ------> s-seq = %d r-seq %d, u-rx %d b-rd = %d",cur_send_frame_seq,cur_recv_frame_seq,uart_rx_drop,drop_bb_read_cnt);
        HAL_Delay(5000);
    }
}

void Usr_TTL_Task(int dev_type)
{
	int param = check_ttl_factory_setting();
    int ttl_session = 0;
    ENUM_HAL_UART_BAUDR ttl_baudr = default_baudr;

	bb_dev_type = dev_type;

    if (param == -2)
    {
        DLOG_Critical("Usr_Uart6_Task Discard!!!\n");
        return;
    }

    if (param == -1)
    {
        ttl_baudr = default_baudr;
        GLOBAL_BB_SESSION = TTL_DEFAULT_BB_SESSION_PORT;
    }
    else
    {
        ttl_session = (param & 0xF0) >> 4;
        ttl_baudr = param & 0x0F;

        if (ttl_session == 0)
		{
            GLOBAL_BB_SESSION = TTL_DEFAULT_BB_SESSION_PORT;
		}
        else
		{
			ttl_session++;
	        if (ttl_session < BB_COM_SESSION_2 || ttl_session > BB_COM_SESSION_3)
    	        GLOBAL_BB_SESSION = TTL_DEFAULT_BB_SESSION_PORT;
			else
		        GLOBAL_BB_SESSION = ttl_session;

	        if (ttl_baudr < HAL_UART_BAUDR_9600 || ttl_baudr > HAL_UART_BAUDR_460800)
    	        ttl_baudr = default_baudr;
		}
    }

    DLOG_Critical("USR_TTL_Task (UART6) ENUM_HAL_UART_BAUDR is %d, ENUM_BB_COM_SESSION_ID is %d, GLOBAL_BB_SESSION is %d \n", ttl_baudr, ttl_session, GLOBAL_BB_SESSION);

	ring_hal_uart_rx = ringbuf_new(2);
    if(!ring_hal_uart_rx)
    {
        DLOG_Critical("malloced failed");
    }

    ring_bb_uart_rx = ringbuf_new(3);
    if(!ring_bb_uart_rx)
    {
        DLOG_Critical("malloced failed");
    }

    if (ttl_baudr >= HAL_UART_BAUDR_9600 && ttl_baudr <= HAL_UART_BAUDR_460800)
        HAL_UART_Init(HAL_UART_COMPONENT_6, ttl_baudr, Uart6_Irq_Handler);

    HAL_BB_ComRegisterSession(GLOBAL_BB_SESSION,
                              BB_COM_SESSION_PRIORITY_LOW,
                              BB_COM_SESSION_DATA_NORMAL,
                              Rcv6_Data_Handler);
    
    get_retran_mode_factsetting();
    
    if(cur_default_retrans_mode == LIMIT_RETRANS || cur_default_retrans_mode == UNLIMIT_RETRANS)
    {
        
        osThreadDef(HAL_UART_TX6BB_TASK, hal_uart_tx6bb_task_byretry, osPriorityHigh, 0, 8 * 128);
        osThreadCreate(osThread(HAL_UART_TX6BB_TASK), NULL);

        osThreadDef(BB_UART_TX6HAL_TASK, bb_uart_tx6hal_task_byparse, osPriorityHigh, 0, 8 * 128);
        osThreadCreate(osThread(BB_UART_TX6HAL_TASK), NULL);
        
        //osThreadDef(BYPASS_TTL_TASK_EXCP_TASK, bypass_ttl_task_excp_proc, osPriorityNormal, 0, 8 * 128);
        //osThreadCreate(osThread(BYPASS_TTL_TASK_EXCP_TASK), NULL);

    }
    else
    {
        osThreadDef(HAL_UART_TX6BB_TASK, hal_uart_tx6bb_task, osPriorityHigh, 0, 8 * 128);
        osThreadCreate(osThread(HAL_UART_TX6BB_TASK), NULL);

        osThreadDef(BB_UART_TX6HAL_TASK, bb_uart_tx6hal_task, osPriorityHigh, 0, 8 * 128);
        osThreadCreate(osThread(BB_UART_TX6HAL_TASK), NULL);
    }

    osThreadDef(COMTASK_UART6_Debug, COMTASK_DebugFunction, osPriorityIdle, 0, 8 * 128);
    osThreadCreate(osThread(COMTASK_UART6_Debug), NULL);
}


