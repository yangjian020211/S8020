#ifndef __TEST_BB_LED_CTRL_H__
#define __TEST_BB_LED_CTRL_H__

#ifdef __cplusplus
extern "C"
{
#endif

void BB_ledGpioInit(void);

void BB_ledLock(void);

void BB_ledUnlock(void);

void BB_EventHandler(void *p);

void lna_open(void);

void lna_bypass(void);


uint8_t get_link_status(void);
uint8_t get_video_state(void);
void led_enter_sleep(void);
#ifdef __cplusplus
}
#endif

#endif
