#ifndef __TEST_BB_LED_CTRL_2_H__
#define __TEST_BB_LED_CTRL_2_H__

#ifdef __cplusplus
extern "C"
{
#endif

typedef enum
{
    LINK_UNLOCK = 0,
    LINK_LOCK,
    LINK_SEARCH_ID,
    LINK_ID_NO_MATCH,
    LINK_INVALID
}LINK_LED_STATUS;

void BB_ledGpioInit(void);

void BB_ledLock(void);

void BB_ledUnlock(void);

void BB_EventHandler(void *p);

void lna_bypass(void);

void c201s_lna_bypass(void);

void lna_open(void);

void normal_rf(void);

void BB_ledSearchId(void);

void set_link_led_status(LINK_LED_STATUS e_link_led_status);

void sky_led_Task(void const *argument);

void grd_led_Task(void const *argument);

void led_enter_sleep(void);

LINK_LED_STATUS get_link_status(void);

typedef enum
{
    V_EMPTY = 0,
    V_NORMAL,
    V_FULL,
    V_INVALID
}LED_VIDEO_STATUS;

LED_VIDEO_STATUS get_video_state(void);

void ENCODE_EventHandler(void *p);
#ifdef __cplusplus
}
#endif

#endif
