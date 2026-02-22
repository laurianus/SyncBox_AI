
/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "stdio.h"
#include "options.h"

typedef struct
{
  uint8_t ButtonsP[MAX_BUT_NR];
  uint8_t is_Valid_But[MAX_BUT_NR];
  uint16_t ButPress_T[MAX_BUT_NR];
  uint16_t ButRelease_T[MAX_BUT_NR];

} Buttons_Struct;



void init_buttons_read(void);
void init_buttons_write(void);

void init_buttons_rst(void);

void read_XXX_But(uint8_t but_pos, GPIO_TypeDef* but_port, uint16_t but_pin);
void read_Buttons_stat(void);


void write_XXX_But(uint8_t but_pos, GPIO_TypeDef* but_port, uint16_t but_pin);
void write_button_stat(void);

void Buttons_fncts(void);

void R1_B_Func(void);
void R2_B_Func(void);
void R3_B_Func(void);
void R4_B_Func(void);

void L01_B_Func(void);
void L02_B_Func(void);
void L03_B_Func(void);
void L04_B_Func(void);
void L05_B_Func(void);
void L06_B_Func(void);
void L07_B_Func(void);
void L08_B_Func(void);
void L09_B_Func(void);
void L10_B_Func(void);
void L11_B_Func(void);
void L12_B_Func(void);
void L13_B_Func(void);
void L14_B_Func(void);
void L15_B_Func(void);
void L16_B_Func(void);

void MM_B_Func(void);
void MP_B_Func(void);
void SCRIPT_B_Func(void);
void AP_B_Func(void);
void SET_B_Func(void);
void MF_B_Func(void);
void SZ_B_Func(void);
void START_B_Func(void);
void STEP_B_Func(void);
void SEQ_B_Func(void);
void SHORT_B_Func(void);

void read_arm_key(void);

void rst_buttons_press(uint8_t rst_buton_id);
void set_buttons_press(uint8_t rst_bt_Idx);
uint8_t getC_Pressed(void);
uint8_t getL_Pressed(void);
void set_last_fired(uint8_t fired_ch_pos);
uint8_t get_arm_key(void);

void SetReset_Buttons_stats(void);
void reset_all_fired(void);
void update_buttons_MF(void);
void set_ButtonsP(uint8_t Pos, uint8_t Value);


void OnOff_Red_Start(void);
void OnOff_Green_Start(void);
void OnOff_Blue_Start(void);

void OnOff_Red_Panel(void);
void OnOff_Green_Panel(void);
void OnOff_Blue_Panel(void);
uint8_t return_disp_area(void);
void set_but_led_val(uint8_t but_pos, uint8_t but_val);
void reset_Script_screen(void);
void reset_man_fire(void);
void reset_AP_screen(void);
uint8_t get_Func_man(void);
void reset_seq_fire(void);
uint8_t get_R_pressed(void);
void reset_seq_screen(void);
void set_R_seq(void);
void reset_sz_fire(void);
uint8_t getC_Pressed_but(void);
void set_rail_press(uint8_t tmp_val_cds);
void set_but_light(uint8_t post_but, uint8_t val_to_set);
uint8_t get_R_pressed_now(void);
void rst_screen_Line_stat(void);
void reset_SET_screen(void);
void update_AB_But(void);
void rst_AB_But(void);
uint8_t get_firing_stat(void);
void reset_man_firex(void);
uint8_t get_is_Time_code_run(void);
void exit_chg_channel(void);
void load_script_auto(void);
void read_DMS_stat(void);
void set_Wake_button(void);
uint8_t get_but_light(uint8_t post_but);
void testing_buttons(void);
void enable_disbale_AP(uint8_t ena_AP);

