//
//  ss_test_datalog.cpp
//  AirX
//
//  Created by uncle on 2017/12/18.
//  Copyright © 2017年 uncle. All rights reserved.
//

#include "ss_test_datalog.h"
#include "jnc_hal_sys.h"

/**
 测试程序，写成公用格式，以借助第三方工具来查看。
 */

#define SS_TEST_BYTE1   163
#define SS_TEST_BYTE2   149
#define SS_TEST_BYTE3   128
#define SS_TEST_HEAD(id)    SS_TEST_BYTE1, SS_TEST_BYTE2, SS_TEST_BYTE3, id

struct SS_PACKED ss_log_Format {
    uint8_t head1;
    uint8_t head2;
    uint8_t msgid;
    uint8_t type;
    uint8_t length;
    char name[4];
    char format[16];
    char labels[64];
};

struct SS_PACKED ss_test_log {
    uint8_t head1, head2, msgid;
    uint32_t timestamp;
    float v1, v2, v3, v4, v5, v6, v7, v8, v9, v10, v11, v12;
}; // 55 bytes

static ss_log_Format _test_list[] = {
    {
        SS_TEST_HEAD(1), sizeof(struct ss_test_log),
        "TS1", "Iffffffffffff",
        "t,v1,v2,v3,v4,v5,v6,v7,v8,v9,v10,v11,v12",
    },
    
    {
        SS_TEST_HEAD(2), sizeof(struct ss_test_log),
        "TS2", "Iffffffffffff",
        "t,v1,v2,v3,v4,v5,v6,v7,v8,v9,v10,v11,v12",
    }, // 110 bytes/frame
    
//    {
//        SS_TEST_HEAD(3), sizeof(struct ss_test_log),
//        "TS3", "Iffffffffffff",
//        "t,v1,v2,v3,v4,v5,v6,v7,v8,v9,v10,v11,v12",
//    },
//
//    {
//        SS_TEST_HEAD(4), sizeof(struct ss_test_log),
//        "TS4", "Iffffffffffff",
//        "t,v1,v2,v3,v4,v5,v6,v7,v8,v9,v10,v11,v12",
//    },
//
//    {
//        SS_TEST_HEAD(5), sizeof(struct ss_test_log),
//        "TS5", "Iffffffffffff",
//        "t,v1,v2,v3,v4,v5,v6,v7,v8,v9,v10,v11,v12",
//    }, // 275 bytes/frame
};

static int _test_count = sizeof(_test_list) / sizeof(_test_list[0]);
static bool _init_ok = true;

void ss_test_datalog_init(){
    
    ss_hal_debuglog(" testing datalog...");
    ss_hal_debuglog(" reference: init cost <= 5us runtime <= 65us(sinf)");
    ss_hal_debuglog(" logfile number should increase time by time");
    
    /* move init-methods together outside
    SS_E_Inf e = SS_SUCCESS;
    if((e = ss_hal_datalog_init()) != SS_SUCCESS){
        ss_hal_debuglog("error %s", e);
        _init_ok = false; return;
    }*/
    int num = ss_hal_datalog_start();
    if (num < 0 || num > 65535) {
        ss_hal_debuglog("file number error %d", num);
        _init_ok = false; return;
    }
    ss_hal_debuglog("logfile number is %d", num);
    _init_ok = true;
    
    // init head
    for (int i = 0; i < _test_count; ++i) {
        ss_hal_datalog_write(&_test_list[i], sizeof(ss_log_Format));
        ss_hal_delay(10);
    }
    
    // 初始化测试
    ss_hal_debuglog("writing pkts, pls wait...");
    uint32_t avg_micros = 0;
    uint32_t max_micros = 0;
    static const uint32_t NUM_LOGS = 500;
    for (int i = 0; i < NUM_LOGS; ++i) {
        uint64_t t0 = ss_hal_micros();
        struct ss_test_log log = {
            SS_TEST_BYTE1,
            SS_TEST_BYTE2,
            1,
            uint32_t(ss_hal_micros()),
            float(2000 + i),
            float(2010 + i),
            float(2020 + i),
            float(2030 + i),
            float(2040 + i),
            float(2050 + i),
            float(2060 + i),
            float(2070 + i),
            float(2080 + i),
            float(2090 + i),
            float(2010 + i),
            float(2110 + i),
        };
        uint32_t dt = uint32_t(ss_hal_micros() - t0);
        if (dt > max_micros) {
            max_micros = dt;
        }
        avg_micros += dt;
        ss_hal_datalog_write(&log, sizeof(log));
        ss_hal_delay(3);
    }
    avg_micros /= NUM_LOGS;
    ss_hal_debuglog("test datalog init total %d pkts, cost max %d avg %d",
                        NUM_LOGS,
                        max_micros,
                        avg_micros);
}

void ss_test_datalog_update(){
    if(!_init_ok) return;
    
    // 运行时测试（稍后需要读取sd卡查验错误率等）
    uint64_t t0 = ss_hal_micros();
    for (int i = 0; i < _test_count; ++i) {
        struct ss_test_log log;
        log.head1 = SS_TEST_BYTE1;
        log.head2 = SS_TEST_BYTE2;
        log.msgid = i + 1;
        log.timestamp = (uint32_t)ss_hal_micros();
        log.v1 = 1000 * sinf((30.f * 0 + t0) / 57.f);
        log.v2 = 1000 * sinf((30.f * 1 + t0) / 57.f);
        log.v3 = 1000 * sinf((30.f * 2 + t0) / 57.f);
        log.v4 = 1000 * sinf((30.f * 3 + t0) / 57.f);
        log.v5 = 1000 * sinf((30.f * 4 + t0) / 57.f);
        log.v6 = 1000 * sinf((30.f * 5 + t0) / 57.f);
        log.v7 = 1000 * sinf((30.f * 6 + t0) / 57.f);
        log.v8 = 1000 * sinf((30.f * 7 + t0) / 57.f);
        log.v9 = 1000 * sinf((30.f * 8 + t0) / 57.f);
        log.v10 = 1000 * sinf((30.f * 9 + t0) / 57.f);
        log.v11 = 1000 * sinf((30.f * 10 + t0) / 57.f);
        log.v12 = 1000 * sinf((30.f * 11 + t0) / 57.f);
        ss_hal_datalog_write(&log, sizeof(log));
    }
    int32_t dt = int32_t(ss_hal_micros() - t0);
    
    // print out cost
    static int32_t _max_dt = 0;
    static int32_t _avg_dt = 0;
    if (dt > _max_dt) {
        _max_dt = dt;
    }
    _avg_dt += dt;
    static uint32_t _loop_count = 0;
    if (++_loop_count >= SS_MAIN_LOOP_HZ) {
        _avg_dt /= SS_MAIN_LOOP_HZ;
        ss_hal_debuglog("test datalog runtime: max %d avg %d", _max_dt, _avg_dt);
        _loop_count = _max_dt = _avg_dt = 0;
    }
}
