
#include "jnc_hal_sys.h"
#include "ss_test_hal_sys.h"


/**
 1. datalog return value
 2. rename to jnc_hal_sys.h
 3. #include "jnc_hal_sys.h"
 4. move all init logic to ss_hal_datalog_init();
 5. move DLOG_Process(NULL); to ss_hal_idle
 6. remap telem channel-id
 7. DATA_LOG_BUFFER_NUMBER, ... move to cpp
 8. .eh_frame lds error fix
 9. ss_hal_debuglog move to jnc_hal_sys(_cpu1).h/cpp
 10. cpu2 log: XXX_CRITICAL
 11. add/modify return value for all init methods in jnc_hal_sys_cpu1.cpp
 12. ss_hal_telem_init return success when unconnected
*/

int main(void){
    test_hal_sys_task(0);
    return 0;
} 

