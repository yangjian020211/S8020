//
//  ss_test_hal_sys.cpp
//  AirX
//
//  Created by uncle on 2017/12/14.
//  Copyright © 2017年 uncle. All rights reserved.
//

#include "ss_test_hal_sys.h"
#include "jnc_hal_sys.h"


#define INIT_FUNC(info, method) \
ss_hal_debuglog(info);\
if ((e = method) != SS_SUCCESS) hold_when_failure(e)


// call at first in loop
static uint32_t _overtime_count = 0;
static uint64_t fixed_fps1(){
    static uint64_t _start_time = 0;
    //++_loop_count;
    
    // 帧率锁定
    int64_t cost_us = ss_hal_micros() - _start_time;
    if ( cost_us < 0 ) {
        ss_hal_delay_micros_boost( SS_MAIN_LOOP_US );
    }else if ( cost_us <= SS_MAIN_LOOP_US ){
        ss_hal_delay_micros_boost( SS_MAIN_LOOP_US - cost_us );
    }else{ // 超时
        ++_overtime_count;
        ss_hal_debuglog("loop timeout");
    }
    
    // 记录、更新时间戳
    _start_time = ss_hal_micros();
    return _start_time;
}

//static uint64_t fixed_fps2(){
//    static uint64_t _start_time = 0;
//    // while (ss_hal_micros() - _start_time < SS_MAIN_LOOP_US) {}
//    uint64_t now = ss_hal_micros();
//    while (now - _start_time < SS_MAIN_LOOP_US){
//        now = ss_hal_micros();
//    }
//    _start_time = ss_hal_micros();
//    return _start_time;
//}


static void hold_when_failure(const char* info){
    while (1) {
        // even following methods may crash, but try it anyway.
        ss_hal_debuglog("jncerr %s", info);
        ss_hal_delay(1000);
    }
}

void test_hal_sys_task(void*){
    ss_hal_debuglog("test begin");
    
    // init all here together, even that feature will not be tested.
    SS_E_Inf e = SS_SUCCESS;
    INIT_FUNC("init board", ss_hal_board_init());
    INIT_FUNC("init datalog", ss_hal_datalog_init());
    INIT_FUNC("init data_persist", ss_hal_data_persist_init());
    INIT_FUNC("init rc", ss_hal_rc_init());
    INIT_FUNC("init telem2", ss_hal_telem_init());

    ss_hal_debuglog_set_block(false);
    while(1){
        fixed_fps1();
        static int _count = 0;
        if(++_count >= 400){
            ss_hal_debuglog("hello");
            _count = 0;
        }
        //ss_hal_delay(2);
        ss_hal_idle(500);
    }
}
