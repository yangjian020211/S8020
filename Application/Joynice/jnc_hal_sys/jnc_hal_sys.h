//
//  jnc_hal_sys.h
//  AirX
//
//  Created by joynices on 2017/11/25.
//  Copyright © 2017年 joynices. All rights reserved.
//

/**
 说明：
 如果不希望将接口函数改写成宏定义，这个文件不应该有任何改动（增删改）
 除非某些<xxx.h>标准库在对应平台上不被支持，此时可以修改标准库的代码部分。
 若希望将（某些）接口函数改写成宏，则在user hook部分添加自己需要的“必要的”依赖项。
 关于错误返回：1.尽量短，不加换行符。2.不要返回临时变量，返回常量字符串。
 
 RTOS请特别注意：
 至少留出 8000bytes 的heap空间，大致有3~5kb用来存储动态参数。如果使用hash，至少留出15kb。
 */

#ifndef __JNC_HAL_SYS_CPU1_H__
#define __JNC_HAL_SYS_CPU1_H__



/**
 user hook. you should put all your own "essential!" defination/include_files here
 If you are not sure, do not modify anything
 */
// TODO...


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


/**
 do not modify if you're not sure
 */
typedef const char* SS_E_Inf;


// TODO: versions, ss_report_version()


/****************************************************
 interfaces for system
 ****************************************************/
#ifdef __cplusplus
extern "C"
{
#endif


/**
 all board-system init here

 @return error information/SS_SUCCESS
 */
SS_E_Inf ss_hal_board_init();


/**
 release and deinit all resources
 目前没有使用，可实现也可不用实现
 */
void ss_hal_board_deinit();



/**
 create a task/thread. with default stack as you like, 500 for instance.
 if your hardware system does not support tasks, just ignore it.
 
 @param name name
 @param func callback function
 
 @return error information text. return 0 if success.
 */
SS_E_Inf ss_hal_create_task( const char* name, void (*func)(void*), uint8_t priority );
SS_E_Inf ss_hal_start_task();


/**
 return us since app started
 
 @return us( micro-seconds )
 */
uint64_t ss_hal_micros();


/**
 delay current thread/task
 
 @param ms milli seconds( ms )
 */
void ss_hal_delay( uint16_t ms );

/**
 delay current thread/task
 
 @param us micro seconds( us )
 */
void ss_hal_delay_micros( uint16_t us );


/**
 delay current thread/task, but raise priority within this period of time.
 if platform does not support this, just wrap as ss_hal_delay_micros.
 eg: #define ss_hal_delay_micros_boost( us ) ss_hal_delay_micros( us )
 or  { return ss_hal_delay_micros(us); } in .cpp/c
 
 @param us micro seconds( us )
 */
void ss_hal_delay_micros_boost( uint16_t us );//{ return ss_hal_delay_micros(us); }


/**
 delay to spare time for other threads/tasks/system_tasks
 为系统事件等其它非控制算法在此处理。参数为允许的最大处理时间（default 500us）
 
 @param us micro sec. maybe only 100*x us supported
 */
void ss_hal_idle(uint16_t us);


/**
 reboot board
 
 @param bool reboot or not
 */
//void ss_hal_reboot( bool ); // ?? Do we need this


/**
 Datalog(sdCard) init. like card inserted?, and etc..
 you should also do clear if logfiles are too many.
 
 !!Note: when sdcard not mounted, following datalog functions could be called but no effect.
 
 @return return 0 if ok, or return error text
 */
SS_E_Inf ss_hal_datalog_init();

/**
 start and ready to write datalogs into (sdcard).
 logfile NUMBER is 1,2,3... or timestamp in seconds since 1970-01-01 00:00:00
 
 @return 0x0000ffff(65535) if error or return logfile NUMBER.
 */
uint32_t ss_hal_datalog_start();

/**
 write log data ！！！！需要背后处理缓冲（超出缓冲块和超时再一次性写入）
 不可阻塞
 此函数需要支持高频高速调用。每帧会调用多次。
 按默认频率400Hz（不排除后续提高的可能性）每帧需可写入100bytes起，越高越好。
 TODO: 测试出对应平台的单帧最高数据量（最好能做到500bytes以上）调整机器人黑盒子写入频率。
 
 @param pBuffer buffer head pointer
 @param size    buffer size in bytes
 */
void ss_hal_datalog_write( const void* pBuffer, uint16_t size );


/**
 init eeprom/xflash... for persistant data storage
 
 @return 0 if ok, or return error text
 */
SS_E_Inf ss_hal_data_persist_init();


/**
 read a data block
 参数区预设大小最大为16000bytes。此函数可以阻塞，保证函数执行完毕数据读取完成。
 此函数会反复调用多次，每次读取偏移和大小不等。（一般单次读取不会超过50bytes）
 
 @param dst    destination buffer pointer
 @param offset read from offset since storage start addr.
 @param size   data block size in bytes
 
 @return ok:true, or false
 */
bool ss_hal_data_persist_read( uint8_t* dst, uint32_t offset, uint32_t size );


/**
 write a data block
 不可阻塞（或者 <100us 级别的阻塞）
 参数区预设大小最大为16000bytes。
 此函数会反复调用多次，每次写入偏移和大小不等。（一般单次写入不会超过50bytes）
 
 @param src    source buffer pointer
 @param offset where to write since storage start address
 @param size   in bytes
 
 @return ok true, or false
 */
bool ss_hal_data_persist_write( const uint8_t* src, uint32_t offset, uint32_t size );


/**
 初始化遥控器，未连接也返回成功，其余错误返回错误字串。
 是否失联由状态获取完成。
 
 @return 成功返回0，否则返回错误字符串
 */
SS_E_Inf ss_hal_rc_init();

/**
 read remote controller 遥控器
 如果任意通道返回大概1000至2000左右的值，那么所有通道都应该返回在大致这个区间。
 不可以有不同的尺度表现。
 
 @param index [0, x] for channel number.
 
 @return 大约1000-2000（遗留自ppm），如果返回为0、远低于1000或大于2000，则认为发生错误。
 */
uint16_t ss_hal_rc_read( uint8_t index );

// not used for now
SS_E_Inf ss_hal_rc_write( uint8_t index, uint16_t value );


/**
 返回遥控器是否连接（和read可能有功能重复）
 如果连接，但硬件平台不支持强度信息，返回100。但需保证连接务必可用，否则返回0.
 
 @return 0：未连接 负数：其它连接错误 [1, 100]：连接的信号强度
 */
int16_t ss_hal_rc_connected();



/**
 telem-data tranfer init.
 未连接则阻塞直到连接或超时（超时需在0-5秒以内，越短越好。如太久需商讨解决方案）
 如果数传通道连接超时，仍然返回成功。通过状态函数获取连接状态。
 其它硬件性错误，仍然返回错误字串。
 如果未连接，下面的读写函数不起作用，且返回0
 
 @return 0 if success, or return error text
 */
SS_E_Inf ss_hal_telem_init();


/// new API, available/read/write above are deprecated.
// !! should not block main thread and
// !! keep cpu cost as low as possible using DMA or sth like that
// -: error 0: nothing >0 bytes received.
int16_t ss_hal_telem_read(uint8_t channel, uint8_t* dst, uint16_t max);


/**
 无线数据传输写入（下行）
 不可阻塞（us级别）。
 目前控制算法每帧会调用一次（每帧根据SS_MAIN_LOOP_XX决定），每次数据量不定。
 但总体数据带宽维持在 2000bytes/秒 以内（eg：一次写入1000bytes，其余此函数调用空或不调用）
 
 @param channel 通道ID号
 @param src 源数据指针，传入0直接返回
 @param length 数据程度（bytes），传入0直接返回
 @return 成功写入的长度（一般等于length或者0），0为未连接，负数为错误码。
 */
int16_t ss_hal_telem_write(uint8_t channel, const uint8_t* src, uint16_t length);


/**
 无线数据通讯连接状态
 如果连接，但硬件平台不支持强度信息，返回100。但需保证连接务必可用，否则返回0.
 
 @return 0：未连接 负数：其它连接错误 [1, 100]：信号强度
 */
int16_t ss_hal_telem_connected();


/**
 set block as default
 debuglog interface, TODO: put all together to jnc_hal_sys(cpu1).h/c
 */
void ss_hal_debuglog(const char* fmt, ...);
void ss_hal_debuglog_set_block( bool is_block );
bool ss_hal_debuglog_get_block();



#ifdef __cplusplus
}
#endif

#endif /* jnc_hal_sys_h */
