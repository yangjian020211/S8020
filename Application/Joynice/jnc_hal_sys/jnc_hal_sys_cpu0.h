/*****************************************************************************
Copyright: 2016-2020, Artosyn. Co., Ltd.
File name: jnc_hal_sys.h
Description: 
Author: Artosyn Software Team
Version: 0.0.1
Date: 2017/12/04
History: 
        0.0.1    2017/12/04    The initial version of hal.h
*****************************************************************************/

#ifndef __JNC_HAL_SYS_CPU0_H__
#define __JNC_HAL_SYS_CPU0_H__


#ifdef __cplusplus
extern "C"
{
#endif

/**
 for handy, if you do not have them, define yours
 */
#include <stdint.h>     // uint8_t/int16_t/uint64_t...
#include <stdbool.h>    // bool/true/false
#include <string.h>     // use memcpy, strlen, strncpy
#include <stdlib.h>     // malloc/free
#include <stdio.h>      // sprintf, will not use printf if unsupported.
#include <math.h>       // basic math calculation
#include <stdarg.h>     // for ... arg
//#include <typeinfo>     // for typeid(type).name()
//#include <memory>       // memory for c++ uniq_ptr just test in copter class only



/**
 for some struct, we DO NOT want bytes aligned
 usage:
 1. typedef struct {...} SS_PACKED test_t;
 2. struct SS_PACKED test_s { ... };
 */
#define SS_PACKED __attribute__((__packed__))


/**
 hal firmware type
 
 since this header file should be the same,
 SS_HAL_TYPE_COMMON is for commonly usage when there is unique .cpp versions.
 the others are for several .cpp files exists in the same complie environment.
 */
#define SS_HAL_TYPE_COMMON      1
#define SS_HAL_TYPE_MAC_DEBUG   2
#define SS_HAL_TYPE_SIL_U3D     3
#define SS_HAL_TYPE_HIL_U3D     4
#define SS_HAL_TYPE_ST32        5
#define SS_HAL_TYPE_ASY8020     6
#define SS_HAL_TYPE SS_HAL_TYPE_COMMON


/**
 system scheduler time
 */
#define SS_MAIN_LOOP_HZ     400
#define SS_MAIN_LOOP_US     (1000000L/SS_MAIN_LOOP_HZ)
#define SS_MAIN_LOOP_SEC    (1.0f/SS_MAIN_LOOP_HZ)


/**
 usually as SS_E_Inf's return value
 */
#define SS_SUCCESS 0


/* Log File Name Example: datafile_00000001.log */
#define LOG_FILE_NAME_PREFIX            "0:datafile_"
#define LOG_FILE_NAME_SURFIX            ".log"
#define LOG_FILE_NAME_ALPHA_LEGNTH      (sizeof(LOG_FILE_NAME_PREFIX) + sizeof(LOG_FILE_NAME_SURFIX) - 2)
#define LOG_FILE_NAME_NUMBER_LENGTH     (8)    // uint32 type
#define LOG_FILE_NAME_MAX_LENGTH        (LOG_FILE_NAME_ALPHA_LEGNTH + LOG_FILE_NAME_NUMBER_LENGTH)


/* Every Log File should not exceed 8MB, can be modified by requirement */
#define LOG_FILE_MAX_SIZE               (256 * 1024 * 1024)


/**
 do not modify if you're not sure
 */
typedef const char* SS_E_Inf;

/**
* @brief      
* @param    
* @retval   
* @note   
*/
void HAL_NV_TaskWriteData(void *p);


/**
* @brief      init the sd write in CPU0
* @param    
* @retval   
* @note   
*/
void ss_log_write_init(void);


#ifdef __cplusplus
}
#endif

#endif

