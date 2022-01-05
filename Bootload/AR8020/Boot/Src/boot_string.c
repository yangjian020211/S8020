#include <string.h>
#include <stdint.h>
#include <stdlib.h>
#include "boot_gpio.h"
#include "boot_core.h"


void * memcpy(void * __dest, void const * __src, size_t len)
{
    long * plDst = (long *) __dest;
    long const * plSrc = (long const *) __src;
   
    if((0 == ((unsigned int)__src & (unsigned int)0x03)) && (0 == ((unsigned int)__dest & (unsigned int)0x03)))
    {
        while (len >= 4)
        {
            *plDst++ = *plSrc++;
            len -= 4;
        }
    }

    char * pcDst = (char *) plDst;
    char const * pcSrc = (char const *) plSrc;

    while (len--)
    {
        *pcDst++ = *pcSrc++;
    }
    
    return (__dest);
}

void * imgcpy(void * __dest, void const * __src, size_t len)
{
    long * plDst = (long *) __dest;
    long const * plSrc = (long const *) __src;
    char * pcDst = NULL;
    char const * pcSrc = NULL;
   
    if((0 == ((unsigned int)__src & (unsigned int)0x03)) && (0 == ((unsigned int)__dest & (unsigned int)0x03)))
    {
        while (len >= 4)
        {
            *plDst++ = *plSrc++;
            len -= 4;
        }

        if(len > 0)
        {
            long tmp = 0;
            
            tmp = *plSrc;
            pcSrc = (char const *) &tmp;
            pcDst = (char *) plDst;

            while (len--)
            {
                *pcDst++ = *pcSrc++;                
            }
        }
    }
    else
    {
        pcDst = (char *) plDst;
        pcSrc = (char const *) plSrc;

        while (len--)
        {
            *pcDst++ = *pcSrc++;
        }
    }

    return (__dest);
}



void *memset(void *s, int c, size_t count)
{
    char *xs = s;
    while (count--)
        *xs++ = c;
    return s;
}

size_t strnlen(const char *s, size_t maxlen)
{
        const char *es = s;
        while (*es && maxlen) {
                es++;
                maxlen--;
        }

        return (es - s);
}
