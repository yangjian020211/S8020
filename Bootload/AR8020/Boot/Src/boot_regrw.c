#include "stddef.h"
#include "stdint.h"

void Reg_Write32_Mask(uint32_t regAddr, uint32_t regData, uint32_t regDataMask)
{
    uint32_t u32_regDataTmp;
    volatile uint32_t* ptr_regAddr = (uint32_t*)regAddr;
    uint32_t u32_regDataWithMask = regData & regDataMask;

    u32_regDataTmp = *ptr_regAddr;
    u32_regDataTmp &= ~regDataMask;
    u32_regDataTmp |= u32_regDataWithMask;
     
    *ptr_regAddr = u32_regDataTmp;
}

uint32_t Reg_Read32(uint32_t regAddr)
{
    return *((volatile uint32_t*)regAddr);
}

void Reg_Write32(uint32_t regAddr, uint32_t regData)
{
    *((volatile uint32_t*)regAddr) = regData;
}