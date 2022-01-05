#include <stdint.h>
#include "arcast_appcommon.h"

STRU_ARCAST_AVSTAUTS st_ARCastStatus;
const unsigned int s_st_ARCastSupportedOutputFormat[9][4] =
{
    {1920, 1080, 30, 34},//16:9
    {1280, 720,  60,  4},//16:9
    {1280, 720,  50, 19},//16:9
    { 720, 480,  60,  3},//16:9
    { 720, 576,  60, 18},//16:9
    {1280, 768, 60, 0xff},
    {1024, 768, 60, 0xff},
    {800, 600, 60, 0xff},
    {640, 480, 60, 0xff},
//    { 720, 480,  50,  2},//4:3
//    { 720, 576,  50, 17},//4:3
};

void insertion_sort(uint8_t *buff, uint8_t Length)
{
    for (int i = 1; i < Length; i++)
    {
        if (buff[i - 1] > buff[i])
        {
            int temp = buff[i];
            int j = i;
            while (j > 0 && buff[j - 1] > temp)
            {
                buff[j] = buff[j - 1];
                j--;
            }
            buff[j] = temp;
        }
    }
}


int8_t Common_ARStatus_Set(uint8_t status)
{
    st_ARCastStatus.u8_ARStatus |= 1 << status;
}

int8_t Common_ARStatus_Clear(uint8_t status)
{
    st_ARCastStatus.u8_ARStatus &= (~(1 << status));
}

int8_t Common_ARStatus_Get(uint8_t status)
{
    return ((st_ARCastStatus.u8_ARStatus >> status) & 1);
}

int8_t Common_ARStatus_Reset(void)
{
    st_ARCastStatus.u8_ARStatus = 0;
}
