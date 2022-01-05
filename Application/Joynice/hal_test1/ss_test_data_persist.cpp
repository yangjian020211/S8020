//
//  ss_test_data_persist.cpp
//  AirX
//
//  Created by uncle on 2017/12/18.
//  Copyright © 2017年 uncle. All rights reserved.
//

#include "ss_test_data_persist.h"
#include "jnc_hal_sys.h"
#include "hal.h"
#include "debuglog.h"

// 需要反复测试
void ss_test_data_persist_init(){
    
    ss_hal_debuglog(" testing data-persist...");
    ss_hal_debuglog(" reference: error = 0, cost ~= 80us, the less the better");
    
    /* move init-method outside
    ss_hal_data_persist_init();*/
    
    static const char* H1 = "joynices data persist test pingpang1";
    static const char* H2 = "joynices data persist test pingpang2";
    static const uint8_t H_N = 64;
    static const uint8_t BATCH_N = 67; // 测试一个奇异batch数目
    static const uint16_t IO_COUNT = 230; // 不能等于0
    
    uint32_t read_error_count = 0;
    uint8_t read_state = 0;
    char hbuf[H_N] = {0};
	uint32_t timeOut = 0;

    if (!ss_hal_data_persist_read((uint8_t*)hbuf, 0, 64)) {
        ++read_error_count;
    }
    if (strcmp(hbuf, H1) == 0) {
        read_state = 1;
    }else if (strcmp(hbuf, H2) == 0){
        read_state = 2;
    }
    
    // set seed
    if (read_state == 1) {
        srand(2);
    }else if (read_state == 2){
        srand(1);
    }
    uint8_t tmp[BATCH_N] = {0};
    
    // read persist-data
    int32_t max_dt = 0;
    int32_t avg_dt = 0;
    if (read_state != 0) {
        for (int i = 0; i < IO_COUNT; ++i) {
            // read block
            memset(tmp, 0x00, sizeof(tmp));
            uint64_t t0 = ss_hal_micros();
            if (!ss_hal_data_persist_read(tmp, H_N + i * BATCH_N, BATCH_N)) {
                ++read_error_count;
            }
            int32_t dt = int32_t(ss_hal_micros() - t0);
            if (dt > max_dt) {
                max_dt = dt;
            }
            avg_dt += dt;
            
            // check
            for (int j = 0; j < BATCH_N; ++j) {
				if (tmp[j] != (j % 256)) {
                    ++read_error_count;
                }
            }
        }
        // 输出结果
        avg_dt /= IO_COUNT;
        ss_hal_debuglog("test data persist read error %d cost max %d avg %d",
                            read_error_count,
                            max_dt,
                            avg_dt);
    }else{
        ss_hal_debuglog("first-time reading, no checking");
    }
    
    // set seed
    if (read_state == 0 || read_state == 2) {
        srand(2);
        strcpy(hbuf, H1);
    }else if (read_state == 1){
        srand(1);
        strcpy(hbuf, H2);
    }
    
    // write blocks
    max_dt = 0;
    avg_dt = 0;
    uint32_t write_error_count = 0;
    if(!ss_hal_data_persist_write((const uint8_t*)hbuf, 0, H_N)){
        ++write_error_count;
    }
    for (int i = 0; i < IO_COUNT; ++i) {
        // generate data
        for (int j = 0; j < BATCH_N; ++j) {
            tmp[j] = j % 256;
        }
        // write
        uint64_t t0 = ss_hal_micros();
#if 1
        if(!ss_hal_data_persist_write(tmp, H_N + i * BATCH_N, BATCH_N)){
            ++write_error_count;
        }
		DLOG_Critical("%d", i);
#endif

#if 0
		timeOut = 0;
		while(true != ss_hal_data_persist_write(tmp, H_N + i * BATCH_N, BATCH_N))
		{	
			HAL_Delay(10);
			timeOut += 1;
			if(timeOut > 200)
			{
				++write_error_count;
				break;
			}
		}

		DLOG_Critical("%d", i);
#endif		
        int32_t dt = int32_t(ss_hal_micros() - t0);
        if (dt > max_dt) {
            max_dt = dt;
        }
        avg_dt += dt;
    }
    
    // output result
    avg_dt /= IO_COUNT;
    ss_hal_debuglog("test data persist write error %d cost max %d avg %d",
                        write_error_count,
                        max_dt,
                        avg_dt);
}

void ss_test_data_persist_update(){
    // not neccesary now...
}
