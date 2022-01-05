#ifndef __TEST_SBUS_UART__
#define __TEST_SBUS_UART__

#ifdef __cplusplus
extern "C"
{
#endif

#include "hal_uart.h"

void usr_bypass_sbus_uart_task(uint8_t dev_type);


uint8_t get_fac_spi_num(void);

ENUM_HAL_UART_BAUDR get_fac_uart4_baudrate(void);


#ifdef __cplusplus
}
#endif

#endif

