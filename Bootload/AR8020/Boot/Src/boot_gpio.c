#include <stdint.h>
#include <stdlib.h>
#include "boot_gpio.h"
#include "boot_regrw.h"

#define GPIO_WATCHDOG               (42)

void BOOT_GPIOSetMode(uint32_t gpioNum, uint32_t mode)
{

    uint32_t u32_GroupNoAddr = 0;
    uint8_t u8_GroupNo = 0;
    uint32_t u32_GpioRegVal = 0;
    
    u8_GroupNo = (gpioNum>>4);
   
    u32_GroupNoAddr = u8_GroupNo*0x04 + GPIO_MODE0_ADDR;

    u32_GpioRegVal = Reg_Read32(u32_GroupNoAddr);
    u32_GpioRegVal |= (mode << ((gpioNum % 16)<<1));
    Reg_Write32(GPIO_MODE0_ADDR + (0x04)*(gpioNum>>4), u32_GpioRegVal);

}

void BOOT_GPIOSetPinDirect(uint32_t gpioNum, uint32_t mode)
{
    uint32_t u32_GroupNoAddr = 0;
    uint32_t u32_RegNoAddr = 0;
    uint8_t  u8_RegNo = 0;
    uint8_t  u8_PinNo = 0;
    uint8_t  u8_GroupNo = 0;
    uint32_t u32_GpioRegVal = 0;
    
    u8_GroupNo = (gpioNum>>5);
    u8_RegNo = (gpioNum%32)>>3;
    u8_PinNo = (gpioNum%32)%8;

    u32_GroupNoAddr = u8_GroupNo*0x40000 + GPIO0_BASE_ADDR;
    u32_RegNoAddr = u8_RegNo*0x0C + GPIO_DATA_DIRECT_A_OFFSET;   
    u32_GpioRegVal = Reg_Read32(u32_GroupNoAddr + u32_RegNoAddr);
    if(GPIO_DATA_DIRECT_OUTPUT == mode)
    {        
        u32_GpioRegVal |= (mode << u8_PinNo);
    }
    else
    {
        u32_GpioRegVal &= ~(1 << u8_PinNo);
    }
    
    Reg_Write32(u32_GroupNoAddr + u32_RegNoAddr, u32_GpioRegVal);
}

void BOOT_GPIOSetPinCtrl(uint32_t gpioNum, uint32_t mode)
{
    uint32_t u32_GroupNoAddr = 0;
    uint32_t u32_RegNoAddr = 0;
    uint8_t  u8_RegNo = 0;
    uint8_t  u8_PinNo = 0;
    uint8_t  u8_GroupNo = 0;
    uint32_t u32_GpioRegVal = 0;
    

    u8_GroupNo = (gpioNum>>5);
    u8_RegNo = (gpioNum%32)>>3;
    u8_PinNo = (gpioNum%32)%8;

    u32_GroupNoAddr = u8_GroupNo*0x40000 + GPIO0_BASE_ADDR;
    u32_RegNoAddr = u8_RegNo*0x0C + GPIO_CTRL_A_OFFSET;   
    u32_GpioRegVal = Reg_Read32(u32_GroupNoAddr + u32_RegNoAddr);
    if(GPIO_CTRL_SOFTWARE == mode)
    {                
        u32_GpioRegVal &= ~(1 << u8_PinNo);
    }
    else
    {
        u32_GpioRegVal |= (mode << u8_PinNo);
    }
    
    Reg_Write32(u32_GroupNoAddr + u32_RegNoAddr, u32_GpioRegVal);
}

uint32_t BOOT_GPIOGet(uint32_t gpioNum)
{
    uint32_t u32_RegNoAddr = 0;
    uint32_t u32_GroupNoAddr = 0;
    uint8_t u8_RegNo = 0;
    uint8_t u8_PinNo = 0;
    uint8_t u8_GroupNo = 0;
    uint32_t u32_GpioRegVal = 0;

    
    u8_GroupNo = (gpioNum>>5);
    u8_RegNo = (gpioNum%32)>>3;
    u8_PinNo = (gpioNum%32)%8;
    
    u32_GroupNoAddr = u8_GroupNo*0x40000 + GPIO0_BASE_ADDR;
    u32_RegNoAddr = u8_RegNo*0x04 + GPIO_EXT_PORT_A_OFFSET;
    
    u32_GpioRegVal = Reg_Read32(u32_GroupNoAddr + u32_RegNoAddr);
    return ((u32_GpioRegVal >> u8_PinNo) & 1);
}

void BOOT_GPIOInPut(uint32_t u32_gpioPin)
{

    BOOT_GPIOSetMode(u32_gpioPin, GPIO_MODE_2);
    BOOT_GPIOSetPinDirect(u32_gpioPin, GPIO_DATA_DIRECT_INPUT);
    BOOT_GPIOSetPinCtrl(u32_gpioPin, GPIO_CTRL_SOFTWARE);
}

void GPIO_SetPin(uint32_t gpioNum, uint32_t value)
{
    
    uint32_t u32_RegNoAddr = 0;
    uint32_t u32_GroupNoAddr = 0;
    uint8_t u8_PinNo = ((gpioNum%32)%8);
    
    switch((gpioNum>>5))
    {
        case 0:
        {
            u32_GroupNoAddr = GPIO0_BASE_ADDR;
            break;
        }
        case 1:
        {
            u32_GroupNoAddr = GPIO1_BASE_ADDR;
            break;
        }
        case 2:
        {
            u32_GroupNoAddr = GPIO2_BASE_ADDR;
            break;
        }
        case 3:
        {
            u32_GroupNoAddr = GPIO3_BASE_ADDR;
            break;
        }
        default:
            return ;
    }
    switch(((gpioNum%32)>>3))
    {
        case 0:
        {
            u32_RegNoAddr = GPIO_DATA_A_OFFSET;
            break;
        }
        case 1:
        {
            u32_RegNoAddr = GPIO_DATA_B_OFFSET;
            break;
        }
        case 2:
        {
            u32_RegNoAddr = GPIO_DATA_C_OFFSET;
            break;
        }
        case 3:
        {
            u32_RegNoAddr = GPIO_DATA_D_OFFSET;
            break;
        }
        default:
            return ;
    }

    if (value == 0)
    {
        Reg_Write32_Mask(u32_GroupNoAddr + u32_RegNoAddr, 0, (1 << u8_PinNo));
    }
    else
    {
        Reg_Write32_Mask(u32_GroupNoAddr + u32_RegNoAddr, (1 << u8_PinNo), (1 << u8_PinNo));
    }

}

void BOOT_WatchDogInit(void)
{
    BOOT_GPIOSetMode(GPIO_WATCHDOG, GPIO_MODE_2);
    BOOT_GPIOSetPinDirect(GPIO_WATCHDOG, GPIO_DATA_DIRECT_OUTPUT);
    BOOT_GPIOSetPinCtrl(GPIO_WATCHDOG, GPIO_CTRL_SOFTWARE);
}
static uint32_t u32_gpioState = 0;
void BOOT_WatchDogFeed(void)
{
    u32_gpioState++;
    u32_gpioState &= 1;
    GPIO_SetPin(GPIO_WATCHDOG, u32_gpioState);
}