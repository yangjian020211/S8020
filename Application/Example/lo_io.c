
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "sys_event.h"
#include "hal_gpio.h"
#include "debuglog.h"
#include "hal.h"
#include "lo_io.h"

#define Maxband 14

static void SET_LE(){HAL_GPIO_SetPin(HAL_GPIO_NUM53,HAL_GPIO_PIN_SET);}
static void CLR_LE(){HAL_GPIO_SetPin(HAL_GPIO_NUM53,HAL_GPIO_PIN_RESET);}
static void SET_SCL(){HAL_GPIO_SetPin(HAL_GPIO_NUM52,HAL_GPIO_PIN_SET);}
static void CLR_SCL(){HAL_GPIO_SetPin(HAL_GPIO_NUM52,HAL_GPIO_PIN_RESET);}
static void SET_DATA(){HAL_GPIO_SetPin(HAL_GPIO_NUM50,HAL_GPIO_PIN_SET);}
static void CLR_DATA(){HAL_GPIO_SetPin(HAL_GPIO_NUM50,HAL_GPIO_PIN_RESET);}
static void SET_DATA_OUT(){ HAL_GPIO_OutPut(HAL_GPIO_NUM50);}//SPI_SS5----DATA
static void SET_DATA_IN(){ HAL_GPIO_InPut(HAL_GPIO_NUM50);}//SPI_SS5----DATA
void write_lo(unsigned char count, unsigned char *buf);
void read_lo(unsigned char count, unsigned char *buf);

static unsigned char GET_DATA()
{
	uint32_t data=0;
	HAL_GPIO_GetPin(HAL_GPIO_NUM50,&data);
	return (unsigned char)data;
}
typedef struct _rfband{
  int band;
   unsigned	int R0;
   unsigned	int R1;
   unsigned	int R2;
   unsigned	int R3;
   unsigned	int R4;
   unsigned	int R5;  
} rfband;

rfband myrfband[Maxband]=
{
	
	{200,0x6B8000,0x8008011,0x8E42,0x4B3,0x9A003C,0x580005},//2350-200=2150
	{300,0x668000,0x8008011,0x8E42,0x4B3,0x9A003C,0x580005},//2350-300=2050
	{400,0x618000,0x8008011,0x8E42,0x4B3,0x9A003C,0x580005},//2350-400=1950
	{500,0x5C8000,0x8008011,0x8E42,0x4B3,0x9A003C,0x580005},//2350-500=1850
	{600,0x578000,0x8008011,0x8E42,0x4B3,0x9A003C,0x580005},//2350-600=1750
	{700,0x528000,0x8008011,0x8E42,0x4B3,0x9A003C,0x580005},//2350-700=1650
	{800,0x4D8000,0x8008011,0x8E42,0x4B3,0x9A003C,0x580005},//2350-800=1550
	{900,0x488000,0x8008011,0x8E42,0x4B3,0x9A003C,0x580005},//2350-900=1450
	{1000,0x438000,0x8008011,0x8E42,0x4B3,0x9A003C,0x580005},//2350-1000=1350
	{1100,0x3E8000,0x8008011,0x8E42,0x4B3,0x9A003C,0x580005},//2350-1100=1250
	{1200,0x398000,0x8008011,0x8E42,0x4B3,0x9A003C,0x580005},//2350-1200=1150
	{1300,0x690000,0x8008011,0x8E42,0x4B3,0xAA003C,0x580005},//2350-1300=1050
	{1400,0x5F0000,0x8008011,0x8E42,0x4B3,0xAA003C,0x580005},//2350-1400=950
	{1500,0x550000,0x8008011,0x8E42,0x4B3,0xAA003C,0x580005},//2350-1500=850
};

void Initial_Lo_GPIO()
{
	HAL_GPIO_InPut(HAL_GPIO_NUM50);//SPI_SS5----DATA
	HAL_GPIO_InPut(HAL_GPIO_NUM51);//SPI_CLK----LD
	HAL_GPIO_OutPut(HAL_GPIO_NUM52);//SPI_TXD5---CLK
	HAL_GPIO_OutPut(HAL_GPIO_NUM53);//SPI_RXD5---LE
}


void write_lo(unsigned char count, unsigned char *buf)
{
	unsigned	char	ValueToWrite = 0;
    unsigned	char	i = 0;
	unsigned	char	j = 0;
	
	SET_DATA_OUT();
	HAL_Delay(10);
	CLR_SCL();
	CLR_LE();
	HAL_Delay(10);
	for(i=count;i>0;i--)
 	{
	 	ValueToWrite = *(buf + i - 1);
		for(j=0; j<8; j++)
		{
			if(0x80 == (ValueToWrite & 0x80)){
				SET_DATA();	  //Send one to SDO pin
			}
			else{
				CLR_DATA();	  //Send zero to SDO pin
			}
			HAL_Delay(10);
			SET_SCL();
			HAL_Delay(10);
			ValueToWrite <<= 1;	//Rotate data
			CLR_SCL();
		}
	}
	CLR_DATA();
	HAL_Delay(10);
	SET_LE();
	HAL_Delay(10);
	CLR_LE();
}

void read_lo(unsigned char count, unsigned char *buf)
{
	unsigned	char	i = 0;
	unsigned	char	j = 0;
	unsigned	int  	iTemp = 0;
	unsigned	char  	RotateData = 0;

	SET_DATA_IN();
	
	HAL_Delay(10);
	CLR_SCL();
	CLR_LE();
	HAL_Delay(10);

	for(j=count; j>0; j--)
	{
		for(i=0; i<8; i++)
		{
			RotateData <<= 1;		//Rotate data
			HAL_Delay(10);
			iTemp = GET_DATA();		//Read DATA of ADF4350
			SET_SCL();	
			if(iTemp)
			{
				RotateData |= 1;	
			}
			HAL_Delay(10);
			CLR_SCL();
		}
		*(buf + j - 1)= RotateData;
	}	 
	HAL_Delay(10);
	SET_LE();
	HAL_Delay(10);
	CLR_LE();
} 

void Set_Lo(int band){

if(band <200 || band >1500) return;
int i=0;
unsigned char buf[4]={0,0,0,0};

for(i=0;i<Maxband;i++)
	{
		if(myrfband[i].band==band)
		{
			buf[0]=myrfband[i].R5;
			buf[1]=myrfband[i].R5>>8;
			buf[2]=myrfband[i].R5>>16;
			buf[3]=myrfband[i].R5>>24;
			write_lo(4,buf);
			buf[0]=myrfband[i].R4;
			buf[1]=myrfband[i].R4>>8;
			buf[2]=myrfband[i].R4>>16;
			buf[3]=myrfband[i].R4>>24;
			write_lo(4,buf);
			buf[0]=myrfband[i].R3;
			buf[1]=myrfband[i].R3>>8;
			buf[2]=myrfband[i].R3>>16;
			buf[3]=myrfband[i].R3>>24;
			write_lo(4,buf);
			buf[0]=myrfband[i].R2;
			buf[1]=myrfband[i].R2>>8;
			buf[2]=myrfband[i].R2>>16;
			buf[3]=myrfband[i].R2>>24;
			write_lo(4,buf);
			buf[0]=myrfband[i].R1;
			buf[1]=myrfband[i].R1>>8;
			buf[2]=myrfband[i].R1>>16;
			buf[3]=myrfband[i].R1>>24;
			write_lo(4,buf);
			buf[0]=myrfband[i].R0;
			buf[1]=myrfband[i].R0>>8;
			buf[2]=myrfband[i].R0>>16;
			buf[3]=myrfband[i].R0>>24;
			write_lo(4,buf);
		}
	}

}

