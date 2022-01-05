#ifndef BOOT_CORE_H
#define BOOT_CORE_H

void BOOT_BootApp(unsigned int address);
void BOOT_StartBoot(uint8_t u8_flag);
void BOOT_StartApp(uint8_t u8_flag);
void BOOT_PrintInfo(uint32_t u32_addr);
uint8_t BOOT_CheckCode(uint32_t u32_BaseAddr);
int BOOT_Printf(const char *fmt, ...);
#endif