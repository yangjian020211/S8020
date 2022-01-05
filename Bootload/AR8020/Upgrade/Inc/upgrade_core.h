#ifndef UPGRADE_CORE_H
#define UPGRADE_CORE_H


#define SDRAM_INIT_DONE                     (*(volatile uint32_t *)0xA0030024)
#define SFR_TRX_MODE_SEL                    (*(volatile uint32_t *)0x40B00068)

#define GET_WORD_BOOT_INOF(any_addr) ((((uint32_t)(*any_addr)) << 24) | \
                                         (((uint32_t)(*(any_addr+1))) << 16) | \
                                         (((uint32_t)(*(any_addr+2))) << 8) | \
                                         ((uint32_t)((*(any_addr+3)))))


void UPGRADE_CopyFromNorToITCM(void);


#endif
