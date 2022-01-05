#include "command.h"
#include "debuglog.h"
#include "debuglog.h"
#include <string.h>
#include <stdlib.h>
#include "cmsis_os.h"
#include "test_i2c_adv7611.h"
#include "test_hal_camera.h"
#include "hal_dma.h"
#include "test_upgrade.h"
#include "memory_config.h"
#include "hal_gpio.h"
#include "hal_ret_type.h"
#include "test_hal_mipi.h"
#include "test_usbh.h"
#include "test_hal_nv.h"
#include "ar_freertos_specific.h"
#include "test_bb.h"
#include "cmd_line.h"
#include "test_hal_spi.h"
#include "factory.h"
#include "test_common.h"
#include "sys_event.h"
#include "memory.h"
#include "test_hal_i2c.h"
#include "hal.h"
#include "hal_uart.h"
#include "hal_gpio.h"
#include "hal_pwm.h"
#include "systicks.h"
#include "cmsis_os.h"

typedef int (*fun)(char);
typedef void (*fun2)(char);

#define P201_BLUE		(67)
#define P201_RED		(71)
#define LED_ON     		 0
#define LED_OFF    		 1

int command_test_acon(void);

void command_upgrade(void);
void command_malloc(char *size);
static void command_LNA_bypass(char *bypass);
static void command_LED_CTRL(char *led_name);

STRU_CMD_ENTRY_T g_cmdArray[] =
{
    {1, (f_cmdline)command_readMemory, "read", "<address>"},// 
    {2, (f_cmdline)command_writeMemory, "write", "<address> <data>"},
    {1, (f_cmdline)upgrade,  "upgrade", "<filename>"},
    {1, (f_cmdline)gndforskyupgrade, "gndforskyupgrade", "<filename>"},
    {2, (f_cmdline)command_TestHalCameraInit, "test_camera_init",  "<rate 0~1> <mode 0~8>"},
    {2, (f_cmdline)command_TestCameraWrite, "test_write_camera", "<subAddr(hex)> <value>(hex)"},
    {1, (f_cmdline)command_TestCameraRead, "test_read_camera", "<subAddr(hex)>"},
    {4, (f_cmdline)command_TestHalMipiInit, "test_hal_mipi_init", "<toEncoderCh 0~1> <width>, <hight> <frameRate>"},
    {1, (f_cmdline)command_startBypassVideo, "startbypassvideo", "channel"},
    {0, (f_cmdline)command_stopBypassVideo, "stopbypassvideo",  ""},// 
    {0, (f_cmdline)command_TestNvResetBbRcId, "NvResetBbRcId",  ""},
    {7, (f_cmdline)command_TestNvSetBbRcId, "NvSetBbRcId",  "<rc id1~5> <vt id0~1>"},
    {5, (f_cmdline)command_TestNvSetChipId, "NvSetChipId",  "<chip id1~5>"},
    {1, (f_cmdline)command_test_BB_uart, "command_test_BB_uart", "<param>"},
    {1, (f_cmdline)command_malloc, "malloc", "size"},
    {0, (f_cmdline)ar_top, "top", ""},
    {2, (f_cmdline)command_set_loglevel, "set_loglevel", "<cpuid> <loglevel>"},
    {0, (f_cmdline)FCT_Reset, "FCT_Reset", ""},
    {1, (f_cmdline)command_LNA_bypass, "command_LNA_bypass", "<on/off>"},
    {4, (f_cmdline)command_TestHalSpiInit, "command_TestHalSpiInit", "<ch> <baudr> <polarity> <phase>"},
    {4, (f_cmdline)command_TestHalSpiTxRx, "command_TestHalSpiTxRx", "<ch> <startData>, <txLen> <rxLen>"},
    {3, (f_cmdline)command_TestHalI2cInit, "command_TestHalI2cInit", "<ch> <i2c addr>, <speed>"},
    {5, (f_cmdline)command_TestHalI2cWrite, "command_TestHalI2cWrite", "<ch> <subAddr>, <subAddrLen>, <data>, <dataLen>"},
    {4, (f_cmdline)command_TestHalI2cRead, "command_TestHalI2cRead", "<ch> <subAddr>, <subAddrLen>, <dataLen>"},
    {0, (f_cmdline)SYS_EVENT_MallocFreeCntCheck, "event_mem", ""},
    {0, (f_cmdline)memory_malloc_free_check, "pmem", ""},
    {0, (f_cmdline)command_LED_CTRL, "command_LED_CTRL", "<Off / R1 / G1 / R2 / G2>"},
	{0, (f_cmdline)command_test_acon,"command_test_aviationconnector",""},

    END_OF_CMD_ENTRY
};

void delay_ms(uint32_t num)
{
    volatile int i;
    for (i = 0; i < num * 100; i++);
}

unsigned int command_str2uint(char *str)
{
    return strtoul(str, NULL, 0); 
}

void command_malloc(char *size)
{
    unsigned int mallocSize;
	char *malloc_addr;
	
    mallocSize = command_str2uint(size);
	malloc_addr = malloc_safe(mallocSize);

	if (malloc_addr != 0)
	{
		DLOG_Info("0x%08x\n", malloc_addr);
	}

	return;
}

static void command_LNA_bypass(char *bypass)
{
    if (memcmp(bypass, "on", strlen("on")) == 0)
    {
	HAL_GPIO_OutPut(HAL_GPIO_NUM49);
        HAL_GPIO_SetPin(HAL_GPIO_NUM49, HAL_GPIO_PIN_SET);
	DLOG_Warning("command_LNA_bypass: GPIO-49 output high!\n");
    }
    else if (memcmp(bypass, "off", strlen("off")) == 0)
    {
        HAL_GPIO_InPut(HAL_GPIO_NUM49);
	DLOG_Warning("command_LNA_bypass: GPIO-49 input!\n");
    }
}

static void command_LED_CTRL(char *led_name)
{
	HAL_GPIO_SetMode(HAL_GPIO_NUM67, HAL_GPIO_PIN_MODE2);	//Green 1
    HAL_GPIO_OutPut(HAL_GPIO_NUM67);


	HAL_GPIO_SetMode(HAL_GPIO_NUM71, HAL_GPIO_PIN_MODE2);	//RED 1
    HAL_GPIO_OutPut(HAL_GPIO_NUM71);

	HAL_GPIO_SetMode(HAL_GPIO_NUM82, HAL_GPIO_PIN_MODE2);	//RED 2
    HAL_GPIO_OutPut(HAL_GPIO_NUM82);

	HAL_GPIO_SetMode(HAL_GPIO_NUM83, HAL_GPIO_PIN_MODE2);	//Green 2
    HAL_GPIO_OutPut(HAL_GPIO_NUM83);

	if (memcmp(led_name, "Off", strlen("Off")) == 0)
	{
		HAL_GPIO_SetPin(HAL_GPIO_NUM67, HAL_GPIO_PIN_SET);
		HAL_GPIO_SetPin(HAL_GPIO_NUM71, HAL_GPIO_PIN_SET);
		HAL_GPIO_SetPin(HAL_GPIO_NUM82, HAL_GPIO_PIN_SET);
		HAL_GPIO_SetPin(HAL_GPIO_NUM83, HAL_GPIO_PIN_SET);

	}
	else if (memcmp(led_name, "R1", strlen("R1")) == 0)
	{
		 HAL_GPIO_SetPin(HAL_GPIO_NUM71, HAL_GPIO_PIN_RESET);

	}
	else if (memcmp(led_name, "G1", strlen("G1")) == 0)
    {
		HAL_GPIO_SetPin(HAL_GPIO_NUM67, HAL_GPIO_PIN_RESET);

    }
	else if (memcmp(led_name, "R2", strlen("R2")) == 0)
    {
		HAL_GPIO_SetPin(HAL_GPIO_NUM82, HAL_GPIO_PIN_RESET);		
    }
	else if (memcmp(led_name, "G2", strlen("G2")) == 0)
    {
		HAL_GPIO_SetPin(HAL_GPIO_NUM83, HAL_GPIO_PIN_RESET);
    }
}

void set_p201_led(unsigned char pin,unsigned char state)
{
    HAL_GPIO_SetMode(pin, HAL_GPIO_PIN_MODE2);
    HAL_GPIO_OutPut(pin);
    HAL_GPIO_SetPin(pin, state);
}

#define ENABLE_USB_GOLDEN 1

#define MAX_USB_RECVLEN 20
typedef struct _USB_RECV_BUFFER{

    uint8_t  data[20];   
    uint32_t cnt;
}USB_RECV_BUFFER;

USB_RECV_BUFFER usb_recv_buffer;

USB_RECV_BUFFER usb_getdata_cmd =
{{0xFF,0xAA,0xCC},3};
USB_RECV_BUFFER usb_test_ok =
{{0xAA,0xCC,0xFF},3};
USB_RECV_BUFFER usb_test_err =
{{0xFF,0xCC,0xAA},3};



void usb_recvcalback(void *buff,uint32_t data_len, uint8_t port_id)
{
  // clear;
  memcpy(usb_recv_buffer.data,0,MAX_USB_RECVLEN);
  usb_recv_buffer.cnt =0;
  // copy
  memcpy(usb_recv_buffer.data,buff,data_len);
  usb_recv_buffer.cnt = data_len;
}


#define PPM_INPUT_PIN   HAL_GPIO_NUM95
#define PPM_OUTPUT_PIN  HAL_GPIO_NUM99

#define ACON_RS232  3    
#define ACON_RS422  1
#define ACON_TTL    6
#define ACON_SBUS   7



#define PIN_OK     1
#define PIN_ERR    0


typedef struct _TEST_GPIO{

    unsigned char pin_a;   
    unsigned char pin_b;
    unsigned char flag;


}TEST_GPIO;


TEST_GPIO test_gpio[] = {
    {99,     95,     PIN_ERR},  //PPM
};


typedef struct _TEST_INFO{

	char testnum;
	fun testfun;
	char *name;
	int flag;
	int result;

}TEST_INFO;

int p201acon_test_init(char num);
int p201acon_test_ttl(char num);
int p201acon_test_rs422(char num);
int p201acon_test_rs232(char num);
int p201acon_test_sbus(char num);
int p201acon_test_gpio(char num);
int p201acon_test_usb(char num);

int p201acon_test_finish(char num);


TEST_INFO p201_test_info[] = {
    
    {1, p201acon_test_init,     "test init",    0,  0},
    {2, p201acon_test_ttl,      "ttl test",     0,  0},
    {3, p201acon_test_rs232,    "rs232 test",   0,  0},
    {4, p201acon_test_rs422,    "rs422 test",   0,  0}, 
    {5, p201acon_test_sbus,     "subs test",    0,  0},    
    {6, p201acon_test_gpio,     "ppm_test",     0,  0},
   // {7, p201acon_test_usb,      "usb_test",     0,  0},  
    {7, p201acon_test_finish,   "test finish",  0,  0},   
    {0, 0,                      "finished",     0,  0}

};



char test_uart_buffer[256];
char p201_pt_uart_cnt = 0;


static uint32_t Uart1_Irq_Handler(uint8_t *pu8_rxBuf, uint8_t u8_len)
{
    char u8_cnt = u8_len;
    char pbuf[256];
    
    if(u8_len > 0)
    {
        memcpy(&test_uart_buffer[p201_pt_uart_cnt],pu8_rxBuf,u8_len);
        p201_pt_uart_cnt += u8_len;
    }
}
static uint32_t Uart7_Irq_Handler(uint8_t *pu8_rxBuf, uint8_t u8_len)
{
    char u8_cnt = u8_len;
    char pbuf[256];
    
    if(u8_len > 0)
    {
        memcpy(&test_uart_buffer[p201_pt_uart_cnt],pu8_rxBuf,u8_len);
        p201_pt_uart_cnt += u8_len;
    }
}


static uint32_t Uart3_Irq_Handler(uint8_t *pu8_rxBuf, uint8_t u8_len)
{
    char u8_cnt = u8_len;
    char pbuf[256];
    
    char i;

    if(u8_len > 0)
    {
        memcpy(&test_uart_buffer[p201_pt_uart_cnt],pu8_rxBuf,u8_len);
        p201_pt_uart_cnt += u8_len;
    }
}

static uint32_t Uart6_Irq_Handler(uint8_t *pu8_rxBuf, uint8_t u8_len)
{
    char u8_cnt = u8_len;
    char pbuf[256];
    
    if(u8_len > 0)
    {
        memcpy(&test_uart_buffer[p201_pt_uart_cnt],pu8_rxBuf,u8_len);
        p201_pt_uart_cnt += u8_len;
    }
}

int p201acon_test_init(char num)
{
  int i;
   for(i = 0; i < (sizeof(test_gpio)/sizeof(TEST_GPIO)); i ++)
     {

     HAL_GPIO_SetMode(test_gpio[i].pin_a, HAL_GPIO_PIN_MODE2);
     HAL_GPIO_InPut(test_gpio[i].pin_a);
     HAL_GPIO_SetMode(test_gpio[i].pin_b, HAL_GPIO_PIN_MODE2);
     HAL_GPIO_InPut(test_gpio[i].pin_b);
    }

     HAL_UART_Init(HAL_UART_COMPONENT_1,HAL_UART_BAUDR_115200,Uart1_Irq_Handler);
     HAL_UART_Init(HAL_UART_COMPONENT_7,HAL_UART_BAUDR_115200,Uart7_Irq_Handler);
     HAL_UART_Init(HAL_UART_COMPONENT_3,HAL_UART_BAUDR_115200,Uart3_Irq_Handler);
     HAL_UART_Init(HAL_UART_COMPONENT_6,HAL_UART_BAUDR_115200,Uart6_Irq_Handler);
     return 0;
}

int p201acon_test(char test_item)
{
   char state =0;
   char rx_buffer[256];
   char tx_buffer[256];
   int i;
   int timeout = 0;
   int err_cnt = 0;

   
   switch (test_item)
   {
        case ACON_TTL:
            state = 6;
          
            for(i = 0;i < 50;i ++)
            tx_buffer[i] = 150+i;
        break;
        case ACON_RS232:
             state = 3;
       
             for(i = 0;i < 50;i ++)
             tx_buffer[i] = 100+i;
        break;
        case ACON_RS422:
             state = 1;
           
             for(i = 0;i < 50;i ++)
             tx_buffer[i] = 50+i;
        break;
        case ACON_SBUS:
             state = 7;
    
             for(i = 0;i < 50;i ++)
             tx_buffer[i] = i;
        break;

   }
   memset(test_uart_buffer,0,50);
   p201_pt_uart_cnt = 0;

   HAL_UART_TxData(state, tx_buffer, 50, HAL_UART_DEFAULT_TIMEOUTMS * 10);
   while(1)
    {
          if(p201_pt_uart_cnt >= 50)
           {    
                  memcpy(rx_buffer,test_uart_buffer,50);
                  memset(test_uart_buffer,0,50);
                  p201_pt_uart_cnt = 0;
                
                   for(i = 0;i < 10;i ++)
                    {
                        if(rx_buffer[i] != tx_buffer[i])
                         {
                            err_cnt = err_cnt + 1;
                         }
                    }
                   if(err_cnt >= 10)
                    {
                        DLOG_Critical("###test failed !!!###");
                        return -1;
                    }
                   else
                    {    
                        DLOG_Critical("###test pass !!!###");
                        return 0;
                    }
                            
             }
             
             SysTicks_DelayMS(20);
             timeout ++;
             if(timeout > 50)
                {
                  DLOG_Critical("###timeout !!!###");
                  return -1;
                  break;
                }

    }
}

int p201acon_test_rs232(char num)
{
 DLOG_Critical("now is in rs232_test");
 if(p201acon_test(ACON_RS232)>=0)
        return  0;
 else
    return -1;

}
int p201acon_test_rs422(char num)
{
 DLOG_Critical("now is in rs422_test");
 if(p201acon_test(ACON_RS422)>=0)
    return  0;
 else
    return -1;
}
int p201acon_test_ttl(char num)
{
 DLOG_Critical("now is in ttl_test");
 if(p201acon_test(ACON_TTL)>=0)
    return  0;
 else
    return -1;
}
int p201acon_test_sbus(char num)
{
 DLOG_Critical("now is in sbus_test");
 if(p201acon_test(ACON_SBUS)>=0)
    return  0;
 else
   return -1 ;
}


int p201acon_test_gpio(char num)
{
    uint8_t i;
    uint32_t val;

  DLOG_Critical("now is in ppm_test");


  for(i = 0; i < (sizeof(test_gpio)/sizeof(TEST_GPIO)); i ++)
    {
       if(test_gpio[i].flag != PIN_ERR)
        {
           test_gpio[i].flag = PIN_ERR;
        }
       HAL_GPIO_SetMode(test_gpio[i].pin_a,HAL_GPIO_PIN_MODE2);
       HAL_GPIO_SetMode(test_gpio[i].pin_b,HAL_GPIO_PIN_MODE2);
      //pin_a input     pin_b output
       HAL_GPIO_InPut(test_gpio[i].pin_a);
       SysTicks_DelayMS(10);
       HAL_GPIO_OutPut(test_gpio[i].pin_b);
      //pin_a  set 1
       HAL_GPIO_SetPin(test_gpio[i].pin_b, 1);
       SysTicks_DelayMS(10);
       HAL_GPIO_GetPin(test_gpio[i].pin_a,&val);
       if(val != 1)
       {
         HAL_GPIO_InPut(test_gpio[i].pin_a);
         HAL_GPIO_InPut(test_gpio[i].pin_b); 
         continue;
        }
        //pin_a set 0 
       HAL_GPIO_SetPin(test_gpio[i].pin_b, 0);
       SysTicks_DelayMS(10);
       HAL_GPIO_GetPin(test_gpio[i].pin_a,&val);
       if(val != 0)
       {
         HAL_GPIO_InPut(test_gpio[i].pin_a);
         HAL_GPIO_InPut(test_gpio[i].pin_b); 
         continue;
        }
       test_gpio[i].flag = PIN_OK;
    }

   for(i = 0; i < (sizeof(test_gpio)/sizeof(TEST_GPIO)); i ++)
    {
    if(test_gpio[i].flag == PIN_ERR)
     {
         DLOG_Critical("###test failed !!!###");
         return -1;  
     }
    }
 
   DLOG_Critical("###test pass !!!###");
   return 0;   
}


/*
int p201acon_test_usb(char num)
{
     DLOG_Critical("now is in usb_test");
     uint8_t buffer[MAX_USB_RECVLEN];
     uint8_t i=0;

     for(i=0;i<MAX_USB_RECVLEN;i++)
       {
           buffer[i] = i;
       }
    //set usb device
     HAL_USB_Init(1,0);
     HAL_USB_RegisterCustomerRecvData(usb_recvcalback);

    if(0==memcmp(usb_getdata_cmd.data, usb_recv_buffer.data, usb_getdata_cmd.cnt))
    {
      
      HAL_USB_CustomerSendData(buffer,MAX_USB_RECVLEN,1);

    }
    if(0==memcmp(usb_test_ok.data, usb_recv_buffer.data, usb_test_ok.cnt))
    {
       DLOG_Critical("###test pass !!!###");
       return 0;
    }
    if(0==memcmp(usb_test_err.data, usb_recv_buffer.data, usb_test_err.cnt))
     {
        DLOG_Critical("###test failed !!!###");
        return 0;
     }
    DLOG_Critical("###test failed !!!###");
    return  -1;
}
*/
int p201acon_test_finish(char num)
{
    int i;

    DLOG_Critical("now is in test_finish");

    for(i = 0; p201_test_info[i].testnum; i++)
    {
        if(p201_test_info[i].result < 0)
        { 
           DLOG_Critical("test failed ");
        }
    }
    return 0;    
}

int command_test_acon(void)
{

  uint8_t i=0;
  DLOG_Critical("###test start!!!###");  
       
   for(i = 0; p201_test_info[i].testnum; i++)
       p201_test_info[i].result = p201_test_info[i].testfun(i);
       
   for(i = 0; p201_test_info[i].testnum; i++)
       if(p201_test_info[i].result < 0)
       {
            DLOG_Critical("###test failed !!!###");
            set_p201_led(P201_RED,LED_ON);
            set_p201_led(P201_BLUE,LED_OFF);
            break;
        }   
       
	if(p201_test_info[i].testnum == 0)
	{
            DLOG_Critical("###test pass !!!###");
            set_p201_led(P201_RED,LED_OFF);
            set_p201_led(P201_BLUE,LED_ON);  
	}
    
	SysTicks_DelayMS(50);

}
