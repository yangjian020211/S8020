#ifndef IT_68021_H_
#define IT_68021_H_


#define IT_68021_REG_BLOCK_SEL          (0x0F)  

/*                        
#define REG_RX_P0_SYS_STATUS            (0x0A)
#define B_P0_PWR5V_DET                  (0x01)
*/
#define ARRAY_COUNT_OF(x)                   ((sizeof(x)/sizeof(0[x])) / ((!(sizeof(x) % sizeof(0[x])))))

#define IT_68021_V_LOCKED_SET_INTERRUPT_POS                    (0x1)
#define IT_68021_V_LOCKED_SET_INTERRUPT_MASK                   (0x1)
#define IT_68021_V_LOCKED_CLEAR_INTERRUPT_POS                  (0x1)
#define IT_68021_V_LOCKED_CLEAR_INTERRUPT_MASK                 (0x1)

#define IT_68021_DE_REGEN_LCK_SET_INTERRUPT_POS                (0x0)
#define IT_68021_DE_REGEN_LCK_SET_INTERRUPT_MASK               (0x1)
#define IT_68021_DE_REGEN_LCK_CLEAR_INTERRUPT_POS              (0x0)
#define IT_68021_DE_REGEN_LCK_CLEAR_INTERRUPT_MASK             (0x1)


uint8_t IT_68021_IrqHandler0(void);
uint8_t IT_68021_IrqHandler1(void);
void IT_68021_Initial(uint8_t index);
uint8_t IT_68021_GetVideoFormat(uint8_t index, uint16_t* widthPtr, uint16_t* hightPtr, uint8_t* framteratePtr, uint8_t* vic);
uint8_t IT_68021_GetAudioSampleRate(uint8_t index, uint32_t* sampleRate);
uint8_t ITE_I2C_WriteByte(uint8_t slv_addr, uint8_t sub_addr, uint8_t val);
uint8_t ITE_I2C_ReadByte(uint8_t slv_addr, uint8_t sub_addr);
uint8_t ITE_I2C_WriteBytes(uint8_t slv_addr, uint8_t sub_addr, uint8_t byteno, uint8_t *p_data);
void IT_68021_Set(unsigned char slv_addr, unsigned char sub_addr, unsigned char mask, unsigned char val);
void ITE_Delay(uint32_t delay);
uint8_t IT_68021_GetAndClearIrq(void);
#endif

