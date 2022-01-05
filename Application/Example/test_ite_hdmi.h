#ifndef __TEST_HDMI_H__
#define __TEST_HDMI_H__

#ifdef __cplusplus
extern "C"
{
#endif


void command_hdmiHandler(uint8_t *index, uint8_t *value);

void command_hdmiRegRW(uint8_t *bank,uint8_t *rw,uint8_t *reg,uint8_t *value );

#ifdef __cplusplus
}
#endif

#endif

