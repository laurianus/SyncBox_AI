/**************************************************************************//**
* @file     Buttons.c
* @brief    Buttons functions
******************************************************************************/
#include <string.h>
#include <stdio.h>

#include "stm32f4xx_hal.h"
#include "waveplayer.h"
#include "TimeCode.h"

#include "SystemTimer.h"
#include "main.h"
#include "Device.h"
#include "Display.h"
#include "options.h"
#include "Wireless.h"
#include "Buttons.h"
#include "uart.h"
#include "USBHostMain.h"
#include "flash_if.h"
#include "stm32f4xx_hal_flash.h"


#ifndef VS_READABLE_CODE
#include "System1.h"
#include "System1Action.h"
#include "System1Data.h"
#endif

#include "simpleEventHandler.h"         		// Event Queue
#include "System1SEMLibB.h"                    	// visualSTATE essentials


uint16_t Menu_Set = 0;

//0 Main Menu
//10 Time
//30 LCD Backlight

extern uint8_t was_Pause_byM;
extern int Adjusted_time;

int saved_arming_time = 0;

extern uint8_t is_Play_Drive;

extern uint8_t is_433Mhz_en;

uint8_t is_arm_valid = 0;

uint8_t DMS_1_Stat = 0;
uint8_t DMS_2_Stat = 0;
uint8_t DMS_3_Stat = 0;

uint16_t DMS_2_time_0 = 0;
uint16_t DMS_2_time_1 = 0;


uint16_t DMS_3_time_0 = 0;
uint16_t DMS_3_time_1 = 0;

uint8_t DMS_2_event = 0;
uint8_t DMS_3_event = 0;


uint8_t Power_off_mods_enabled = 0;
//uint8_t Standby_enabled = 0;

extern uint8_t arm_key_valid;
extern uint16_t arm_key_time;
uint16_t last_arm_timerXA = 0;

extern TIM_HandleTypeDef htim7;

extern char Network_Name[6];
uint8_t is_time_code_run = 0;
extern uint8_t is_dev_test;
extern uint8_t Area_state;
uint8_t was_screen_change = 0;
uint8_t was_screen_changeM = 0;
uint8_t is_pause_firing = 0;
extern uint8_t was_LTC_Started;

extern uint8_t was_skip_used;

uint8_t Man_Fire_was_chg = 0;

extern uint8_t current_AB_For_D;

//uint8_t Saved_ID_AP = 0;

extern uint8_t was_SET_Disp;

uint8_t was_button_armed = 0;
uint8_t was_not_display_arm = 0;

extern uint8_t was_AP_Disp;
extern uint8_t Current_Seq;
extern uint8_t Current_Seq_D;

extern uint8_t Rail_Stat[4][25];

extern uint8_t current_Slave_For_D;
extern uint8_t current_Slave_For_D_Prg;
extern uint8_t is_UpFailed[MAX_NUMBER_OF_DEVICES];

extern uint8_t file_read_error;

extern uint8_t was_last_seq;

uint8_t was_LCD_SZ_Changed = 0;

uint8_t was_but_Rxread = 0;


extern int Is_Short_Pressed;

uint8_t was_LCD_Seq_Changed = 0;
extern uint8_t was_state_up;

extern uint8_t was_LCD_Script_Changed;
extern uint8_t was_lcd_name_load;

uint8_t is_Start_but_dis = 0;
uint16_t is_Start_but_dis_timer = 0;

extern uint8_t was_filed_open;

Buttons_Struct Buttons_Stat;

uint8_t Touch_int_pin = 0;

uint16_t Blink_last_but[MAX_BUT_NR + 1] = {0};

//Arm key status
uint8_t is_key_armed = 0;

extern uint8_t is_init_done;
extern uint8_t load_script;
extern uint8_t USB_recconnect;
extern uint8_t state_wave_player;

uint8_t Function_But = DEFAULT_FUNC;


uint32_t time_paased_here = 0;
uint32_t time_was_here = 0;

extern uint8_t USB_Stick_menu;

uint8_t is_manual_multiple_mods = 0;




void read_Buttons_stat(void)
{
//        read_XXX_But(MM_BUT_POS, MM_BUT_PORT, MM_BUT_PIN);
        
   //     read_XXX_But(MP_BUT_POS, MP_BUT_PORT, MP_BUT_PIN);
}



void init_buttons_rst(void)
{
        for(int i = 0; i < MAX_BUT_NR; i++)
        {
                Buttons_Stat.ButtonsP[i] = 0;
                Buttons_Stat.ButPress_T[i] = 0;
                Buttons_Stat.ButRelease_T[i] = 0;
                Buttons_Stat.is_Valid_But[i] = 0;
        }

}


void init_buttons_read(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  
  
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
        


//M-
  GPIO_InitStruct.Pin = MM_BUT_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(MM_BUT_PORT, &GPIO_InitStruct);


//M+
  GPIO_InitStruct.Pin = MP_BUT_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(MP_BUT_PORT, &GPIO_InitStruct);

//EXTERNAL TRIGGER
/*
  GPIO_InitStruct.Pin = EXT_TRIG_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(EXT_TRIG_PORT, &GPIO_InitStruct);
  */

}


void load_script_auto(void){
if(was_filed_open == 0 && device_Status < Status_PowerEnable)
                                {
                                        Main_Copy();
                                }
}
