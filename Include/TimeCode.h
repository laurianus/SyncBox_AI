/**************************************************************************//**
 * @file     TimeCode.h
 * @brief    Time code decode encode header file
 ******************************************************************************/
#include "stm32f4xx_hal.h"
 



void SMPTE_IRQHandler (uint32_t bit_timer);
void UartFSK_PD_set(uint8_t tmp_rcv_trans);
void UartFSK_F1_set(void);
void UartFSK_ATX_set(uint8_t tmp_time_code);
uint32_t get_SMPTE_Time(void);

void init_time_code(void);
uint8_t get_TC_fps_setX(void);

long int get_time_codeOffset(void);
void test_bit_time(void);

uint8_t CharToHex(char c);
uint8_t XOR_chks_FSK(void);
void read_gen_FSK_Time(void);
void set_TimeCode(uint8_t timeCC);
uint8_t get_Time_CodeX(void);
void set_time_code_read_SMPTE(void);
void UartFSK_CMD_Mode(void);
void UartFSK_PD_sent(uint16_t snt_time_code);
uint16_t get_time_codeBrut(void); 
uint16_t get_FSK_err(void);
void UartFSK_F1_sent(int snt_time_code);
void UartFSK_CMD_ModeX(void);
void UartFSK_F1_setNew(void);
void UartFSK_PD_setNew(uint8_t tmp_rcv_trans);
void MX_TIM9_Init(void);
uint8_t get_time_code_Stat(void);
uint8_t get_is_TC_Active(void);

