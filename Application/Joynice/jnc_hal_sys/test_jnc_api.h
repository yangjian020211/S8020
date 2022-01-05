#ifndef __TEST_JNC_API_H__
#define __TEST_JNC_API_H__

#ifdef __cplusplus
extern "C"
{
#endif


void command_JncUsrDataWrite(void);

void command_JncUsrDataRead(void);

void command_JncUsrDataDestroy(uint8_t *addr);

void command_JncInit(void);

void command_JncRcWrite(void);

void command_JncRcRead(void);

void command_JncTelemWrite(void);

void command_JncTelemRead(void);

void JncTelemPrint(uint8_t *buf, uint16_t len);

#ifdef __cplusplus
}
#endif 

#endif
