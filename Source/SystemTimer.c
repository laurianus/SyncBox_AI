/***************************************************************************//**
* @file     SystemTimer.c
* @brief    System periodic timer functions and variables
******************************************************************************/
#include "SystemTimer.h"
#include "waveplayer.h"
#include "Buttons.h"
#include "Display.h"
#include "Options.h"
#include "Device.h"
#include "sht3x.h"
#include "GlobalPositioning.h"
#include "USBHostMain.h"
#include "ADC.h"
#include "uart.h"
#include "Wireless.h"
#include "TimeCode.h"



#include <stdio.h>
/*
#include "Device.h"
#include "GPIO.h"
#include "wdt.h"
#include "ADC.h"
#include "UART.h"


//Netw 01.07.2017
#include "Display.h"

#include "Wireless.h"
*/

#ifndef VS_READABLE_CODE
#include "System1.h"
#include "System1Action.h"
#include "System1Data.h"

#endif

#include "simpleEventHandler.h"         // Event Queue
#include "System1SEMLibB.h"                    // visualSTATE essentials


//for tests
//#define TEST_DBG 1

#define PRG_DBG_X 1

uint8_t was_WirConfigRead = 0;

uint8_t Rot_enc_push = 0;        
uint32_t Rot_enc_push_tim = 0;  
uint8_t Rot_enc_push_V = 0;        

uint8_t Ext_trig_val = 0;
uint8_t Ext_trig_Valid = 0;
uint16_t Ext_trig_timer = 0;
uint16_t Ext_trig_timerX = 0;


extern uint8_t Time_code_Started;
uint16_t time_Code_TST = 0;

uint16_t PPS_run_XXX = 0;

uint8_t read_mpr_stat_AB_XXX = 0;
extern uint8_t current_AB_For_D;
extern volatile uint8_t DMX_stat;

extern uint8_t SB_AsMaster;

uint32_t Timer_forGenerate_time_code = 0;

extern uint32_t timer_last_msg_ping;

uint8_t External_trig = 0;
extern uint8_t is_audio_skip;
extern uint8_t GPS_Fixed;
uint8_t isGPSTimerCTRL = 0;
extern uint32_t real_time_ctrl;

uint8_t is_DMX_Pass = 0;
uint8_t is_DMX_R_valid = 0;
extern long int DMX_Ramp_Vect[DMX_CH_MAX][3];
extern uint8_t buf_DMX[DMX_CH_MAX];

uint32_t FSK_frame = 0;
uint8_t FSK_Frame_rdy = 0;

uint64_t DMX_Timer = 0;

uint8_t is_pps_valid = 0;

uint8_t was_ButStoped = 0;

volatile uint32_t PPS_Time_Received = 0;
uint8_t need_GPS_update = 0;
uint16_t failed_gps_up = 0;


extern RTC_HandleTypeDef hrtc;
extern RTC_TimeTypeDef sTime;
extern RTC_DateTypeDef sDate;


#if PRG_DBG_X
        uint16_t total_msg_sent = 0;
#endif

//for tests
#if TEST_DBG
uint8_t pwr_stat = 0;
uint8_t pwr_ctrl = 0;
uint8_t pwr_chg = 0;
uint8_t pwrVBUS_ctrl = 0;
uint8_t pwrVBUS = 0;

#endif


#if TST_DEBUG
int Meesage_received_lost = 0;
int Meesage_transmit = 0;


extern uint64_t Start_433Mhz_timer;
extern uint16_t MSG_433Mhz_timer;

uint64_t Start_2400Mhz_timer = 0;
uint16_t MSG_2400Mhz_timer = 0;

int Meesage_received_lost2400 = 0;
uint16_t Meesage_received2400 = 0;


uint8_t mode_trans_wir = 0;

extern int Meesage_received;
#endif


volatile unsigned char hourCount = 0;
volatile unsigned char minuteCount = 0;
volatile unsigned char secondCount = 0;
//volatile unsigned char frameCount = 0;
volatile int frameCount = 0;

volatile unsigned char bitCount = 0;
volatile unsigned char updateCnt = 0;
volatile unsigned char currentBit = 0;

volatile unsigned char lastLevel = 0;
volatile unsigned char polarBit = 0;

uint8_t frameCount_XXX = 0;

float tmp_mil = 0;
float tmp_mil2 = 0;

extern uint8_t fps_gen;
extern uint8_t Can_be_Slave;

uint16_t ping_noise = 0;

uint8_t pwr_stat = 1;
uint16_t pwr_but_timer = 0;
uint8_t is_power_on_enable = 0;


uint8_t SMPTE_RF = 0;
uint16_t min_time_toPause = 0;

uint8_t frameCountXXX = 0;
uint8_t err_freame0 = 0;
uint8_t err_freame1 = 0;
uint8_t err_freame2 = 0;
uint8_t err_freame3 = 0;
uint8_t err_freame4 = 0;

uint16_t saved_mills = 0;

extern TIM_HandleTypeDef htim7;

uint8_t must_reply_with_stat = 0;
uint8_t is_need_sent = 0;
extern uint8_t is_first_mod_msg;



//Blue button — volatile: written in EXTI ISR callback, read in timer polling
volatile uint8_t isValidBlueT = 0u;
uint8_t isValidBlueX = 0u;
volatile uint32_t BlueTimerPress = 0u;
volatile uint32_t BlueTimerUnpress = 0u;

uint16_t BlueGreenPress = 0;

//GreenButton — volatile: written in EXTI ISR callback, read in timer polling
volatile uint8_t isValidGreenT = 0u;
uint8_t isValidGreenX = 0u;
volatile uint32_t GreenTimerPress = 0u;
volatile uint32_t GreenTimerUnpress = 0u;


uint32_t is_wireless_OK_Timer = 0;


uint32_t isLongBPress = 0;
uint16_t ID_autoset_timer = 0;
extern uint8_t set_AD_ID;


uint8_t is_new_433_BC_msg = 0;
uint32_t timer_433_MsgValue = 0;
uint8_t is_433_NC = 0;

extern uint8_t  USBH_USR_ApplicationState;


uint32_t is_sd_error = 0;
uint32_t is_sd_errorMore = 0;
uint8_t was_find_error = 0;

extern uint8_t msg_to_be_snt_433Mhz;

uint32_t check_audio = 0;
int Time_offset_player = 0; 

uint16_t SHOULD_NOT_BE_HERE = 0;// BUGs debug

uint8_t is_final_go_back = 0;
uint32_t time_to_reset = 0;

uint16_t time_to_wait_msg = MAX_TIME_WAIT_FINISH;
uint8_t retry_msg_times = 6;

uint8_t even_to_prg_ext = 0;
uint16_t was_433_msg_sent = 0;

int time_ofsset_AB = 0;

uint16_t was_sync_by_AB = 0;
uint8_t was_just_startP = 0;
uint8_t was_sync_by_433MHz = 0;

uint8_t is_433Mhz_BC_wait = 0;

long int dif_xgfd_rds = 0;

uint8_t DMS_Enabled = 1;

//int time_for_calibration = 0;

uint16_t was_time_synncedAB = 0;
extern uint8_t is_433Mhz_en;
uint64_t last_ping_433msg_sent = 0;

uint8_t is_TC_MOD_Detect = 1;
uint8_t is_FSK_setup = 0;
uint16_t FSK_Timer_SET = 0;
extern uint8_t TC_fps_setX;
extern int smpte_time;


uint64_t last_msg_snt_timer = 0;
uint64_t Time_when_asked_last_msg = 0;

extern uint8_t scan_wifi_energ;
extern uint8_t Power_off_mods_enabled;
uint8_t Power_off_valid = 0;
uint32_t Power_off_timer = 0;


uint8_t was_canceled_script = 0;
uint16_t last_mod_delete = 0;

uint16_t Timer_Wir_learn = 0;
uint8_t wir_msg_snt_learn = 0;
extern uint8_t disable_ping;
extern uint8_t wir_to_learn;


extern uint32_t GPS_SetValue;
extern uint8_t isGPSFTValid;


extern uint8_t was_LTC_Started;
extern uint8_t SMPTE_R;
uint32_t SMPTE_Pause_Time = 0;
uint32_t SMPTE_Pause_TimeX = 0u;
extern uint8_t SMPTE_Valid_PT;

uint32_t ping_timer_calc = 0;

int time_difference = 0;
uint8_t was_updated_time_dev = 0;

//if 1 means script was manual skiped and will ignore Audioplayer time - not sync anymore
uint8_t was_skip_used = 0;

uint16_t was_here_time_plus = 0;
uint8_t need_Time_up = 0;
int tmp_was_skip = 0;

//for tests
//uint16_t is_UpFailedRetry[100] = {0};

uint8_t was_msg_prg_sent = 0;

/**
@var systemTimerValue
@brief Variable holding the current system time (in timer ticks)
*/

extern uint8_t isGPSTimerValid;
extern uint32_t GPS_timerValue;



extern uint8_t was_not_display_arm;
int Is_Short_Pressed = 0;

uint8_t was_finished_prg_id = 100;

int is_time_start_start = 0;

uint16_t Buz_time = 0;
    
uint8_t is_prg_wait = 0;// wait for module to respond- 1 respond on prg, 2 respond on finish prg
uint16_t ev_mod_rem = 0; // events to program for current module
uint16_t ev_mod_rem_ALL = 0;
uint8_t start_here = 0;
uint8_t Del_or_prg = 0;
uint8_t mod_prg_ALL = 0;
uint8_t mod_prg_left = 0;
uint8_t All_or_ID = 0;

uint8_t mod_con_ALL = 0;

extern uint8_t is_open_file_up;



extern uint8_t current_Slave_For_D;
extern uint8_t current_Slave_For_D_Prg;

extern uint8_t fire_type;
extern int id_to_prg;
extern uint8_t start_prg;

uint8_t was_screen_filledA = 0;
uint8_t was_screen_filledB = 0;


uint32_t tmp_time_up = 0;
uint32_t TimeTillEnd = 0;
uint32_t StartUpTime = 0;




uint16_t ev_to_prgT = 0;
uint16_t ev_to_prg = 0;
uint16_t Aev_tmp_prg = 0;

uint8_t prg_all = 0;
uint8_t cur_mfp = 0;
                                                
uint8_t percent_upload = 0;
uint16_t up_events;
uint16_t tot_events = 0;


uint8_t was_Comp_checked = 0;
uint8_t it_isMod_ok = 0;




uint8_t is_UpFailed[MAX_NUMBER_OF_DEVICES] = {0};




uint32_t timerUpValue = 0;

uint16_t is_lcd_chg_wait = 0;
uint16_t is_lcd_chg_timer = 0;

volatile uint64_t systemTimerValue = 0ul;

int Adjusted_time = 255;




uint32_t real_time = 0;
uint8_t real_time_valid = 0;

/*
uint32_t Time_pass = 0;
uint64_t was_here_time_saveA = 0;
uint16_t high_time1 = 0;
uint16_t high_time2 = 0;
uint16_t high_time3 = 0;
uint16_t high_time4 = 0;
uint16_t high_time5 = 0;
*/

extern uint8_t is_Start_but_dis;
extern uint16_t is_Start_but_dis_timer;

uint32_t device_time = 0ul;
uint8_t isValidDevTimer = 0u;
uint8_t sync = 0u;

uint8_t was_offset = 0;

uint32_t Temp_Hum_refresh = 0;

uint8_t PPS_Mp3_AB = 0;

extern uint8_t state_wave_player;

//delay Timer
uint32_t delay_tim_TH = 0;
uint8_t is_delayTim_valid = 0;
uint32_t delayTim_Val = 0;

uint32_t Timer_audio_test = 0;
extern uint8_t is_init_done;

uint32_t player_mil_test_start = 0;

uint32_t millis_AP_Tst = 0;

uint32_t ping_timer = 0;



uint16_t is_timerGO_value = 0;
uint8_t is_timerGO_valid = 0;



uint32_t AP_Start_time_For_Drift = 0;
int AP_Drift_Time = 0;

uint8_t but_read_timers = 0;

uint32_t beep_Timer = 0;
extern uint8_t isFiring;
extern TIM_HandleTypeDef htim1;
uint32_t Test_time = 0;
uint8_t isValidTestTimer = 0;


uint8_t isValidPingTimer = 0;
uint32_t Ping_time = 0;

uint8_t is_dev_test = 0;

int max_time_up_val = 0;


uint8_t is_host_off = 0;
uint32_t Timer_USB_test = 0;
extern uint8_t is_Play_Drive;
extern __IO HOST_StateTypeDef USB_stats;
extern uint8_t is_CopyMenu;


extern uint8_t isGPSTimerValid;


int drift_time_gps = 0;
int drift_time_gps2 = 0;

uint8_t is_not_all_arm_confirmed = 0;
uint16_t arm_timer_check = 0;


extern uint8_t is_smpte_active;
uint32_t timer_smpte_active = 0;

uint32_t device_timeSYNC = 0;

uint8_t arm_key_valid = 0;
uint16_t arm_key_time = 0;

void SysTimer(void)        
{
  systemTimerValue++;
       if(isValidDevTimer && sync == 0) 
        {
                device_time++;
                
                if(device_time % 1000 == 0){
                        sync_time_code(device_time);
                }
                
                if(get_Time_CodeX() == 6){ //generate FSK PD
                        if(device_time % 100 == 0){  
                                FSK_frame = device_time/100 + 1;
                                FSK_Frame_rdy = 1;
                        }
                }
                else  if(get_Time_CodeX() == 7){ //generate FSK F1
                        if(device_time % 1000 == 0){  
                                FSK_frame = device_time/1000 + 1;
                                FSK_Frame_rdy = 2;
                        }
                }

        }

        if(adress_interface() == 10){
                Ext_trig_val = HAL_GPIO_ReadPin(EXT_TRIG_PORT, EXT_TRIG_PIN);
                
                if(Ext_trig_val == 0){
                        if(Ext_trig_timerX > 200){
                                
                                Ext_trig_timer++;
                                
                                if(Ext_trig_timer > 50){
                                        Ext_trig_timerX = 0;
                                        Ext_trig_timer = 0;
                                        
                                        Ext_trig_Valid = 1;
                                }
                        }
                }else{
                        if(Ext_trig_timerX < 220){
                                Ext_trig_timerX++;
                        }else{
                                Ext_trig_timer = 0;
                        }
                }
        }


        
        Blue_button();
        Green_button();
        

        if(pwr_stat == 0){
                pwr_but_timer++;
        }

        
        if(real_time_valid != 0)
        {
                real_time++;
                if(real_time == 86400001){
                        real_time = 0;
                }
        }
        
       
        /*
         if(device_Status < Status_PowerEnable)
        {
                gps_pps_pin();
        }
        */
      
        
       if(device_Status < Status_PowerEnable){
                if(time_Code_TST > TIME_FOR_TC_START || Time_code_Started > 0){
                        Timer_forGenerate_time_code++;
                        if(Timer_forGenerate_time_code % 1000 == 0){
                                sync_time_code(Timer_forGenerate_time_code);
                        }
                }else if(time_Code_TST < TIME_FOR_TC_START && Timer_forGenerate_time_code != 0){
                        Timer_forGenerate_time_code = 0;
                }else{
                        //do nothing
                }
                
                read_encoder_push();
        }
        
        
        if(is_new_433_BC_msg != 0) timer_433_MsgValue++; //433Mhz message offset
        
        if(device_Status != Status_PowerEnable)
        {
                ping_timer++;
        }
        
        if(is_init_done == 1 || is_init_done == 2)
        {
               Timer_audio_test++; 
        }
        
        

        
       if(is_timerGO_valid == 1)
       {
               is_timerGO_value++;
               if(is_timerGO_value == 1500)
               {
                        is_timerGO_valid = 3;
               }
       }
       
               
        if(USB_stats == HOST_IDLE && is_Play_Drive == 0 &&  (is_init_done == 0 || is_init_done > 9)  && (PPS_Mp3_AB == 2 || PPS_Mp3_AB == 0) && systemTimerValue > 30000)
        {
                Timer_USB_test++;
        }
        else
        {

        }

        if(is_smpte_active == 1)
        {
           is_smpte_active = 2;
           timer_smpte_active = 0;
           smpte_time++;
        }
        else if(is_smpte_active == 2)
        {
           timer_smpte_active++;
                if(timer_smpte_active == MAX_TC_WAIT)
                {
                      is_smpte_active = 0;
                      timer_smpte_active = 0;
                }
        }
        
        
        // upload script wait timings
        if (is_prg_wait > 0){
                timerUpValue++;
                if(timerUpValue > max_time_up_val){
                        max_time_up_val = timerUpValue;
                }
        }
        
        if(is_lcd_chg_wait == 1){
                        is_lcd_chg_timer++;
                }
        
                
                //buzzer task
        if(Buz_time > 1)
        {
                Buz_time--;
        }
        else if(Buz_time == 1)
        {
                stop_Buz_PWM();  
        }
        else{
                //do nothing
        }
        
        
        
        if(Power_off_valid == 1){
                Power_off_timer++;
        }
        
        
        if(device_Status == Status_PowerEnable || device_Status == Status_Pause || device_Status == Status_Play){
                DMX_Special_func();
                if(device_Status != Status_Play){
                        DMX_Timer++;
                }
        }
        
        
}
        
void updateTimers(void)
{
        
                //for tests
        #if TEST_DBG
                tests_only();
        #endif
        
        
        read_encoder_pins();
        
        
        if(device_Status < Status_PowerEnable){
                check_push_X();
        }
        
        
        if(adress_interface() == 10){
                if(Ext_trig_Valid == 1){
                     
                        if(device_Status == Status_Idle){
                                
                                Audio_Play_Pause_Idle();
                                
                        }else if(device_Status == Status_PowerEnable){
                                SEQ_AddEvent(ev_GB_Short_Press);
                        }
                        else if(device_Status == Status_Play){
                                SEQ_AddEvent(ev_GB_Short_Press);
                                
                        }
                        else if(device_Status == Status_Pause){
                                SEQ_AddEvent(ev_GB_Short_Press);
                        }
                        
                        Ext_trig_Valid = 0;
                }
        }

        
        
        run_DMX();
        if(DMX_stat == 4){
                update_buffer_DMX();
        }
        
        if (isValidDevTimer){
                generateTimeMatrixEvent(device_time);
        }
        else if(device_Status == Status_PowerEnable || device_Status == Status_Pause){
               // generateDMXTimeMatrixEvent(DMX_Timer);
                generateTimeMatrixEvent(DMX_Timer);
        }
        else if(DMX_Timer != 0 && device_Status < Status_PowerEnable){
                DMX_Timer = 0;
        }
        
        
//        gps_timer_Update();

        send_433_wireless_msg_on_uart();
        
        if(is_final_go_back == 1){
                time_to_reset++;
                if(time_to_reset == 1000 && retry_msg_times == RETRY_TIMES_433MHZ){
                                start_433_Mhz();
                }else if(time_to_reset > 1100){
                        is_final_go_back = 0;
                }
        }
        
       if(is_first_mod_msg == 0){
                do_delay_timer();
       }
       
       pwr_but_read();
       
       if(must_reply_with_stat == 1){
                must_reply_with_stat = 0;
                do_delay_timer();
        }
        
       if(is_need_sent > 1){
               init_delay_timer(adress_interface() * 80);
               is_need_sent = 0;
       }
       
        if(systemTimerValue > 6000){
                delay_timer();
        }
                  
        read_mpr_stat_AB_XXX = read_mp3_AB_pstatus(current_AB_For_D);
          
      if(Can_be_Slave == 2){//it is Master
                Ping_Slaves();
      }
      
      /*
      if(ping_noise > 10000){
        send_noise_msg();

        ping_noise = 0;
      }
      
      ping_noise++;
      */

        if(is_init_done == 10){
                if(device_Status < Status_PowerEnable + 1 || device_Status == Status_Pause){
                        if(is_CopyMenu == 0 || is_CopyMenu == 10){
                                is_sd_error++;
                        }
                }
                
                if(is_sd_error > 5000){
                        if(check_player_stat(was_find_error) != FR_OK){
                                was_find_error = 1;
                                is_sd_errorMore++;
                        }else{
                                was_find_error = 2;
                        }
                        is_sd_error = 0;
                }
        }
        

        
       // External_trig = HAL_GPIO_ReadPin(EXT_TRIG_PORT, EXT_TRIG_PIN);
        
        check_USB_Con();//check if the USB it is not on charge
        
        PPS_Mp3_AB = read_mp3_Pstatus(); 
        
        
        //TEST AUDIO
/*        
         if(device_Status == Status_Play){
                        if(PPS_Mp3_AB == 1){
                                uint32_t mils_to_set = 0;
                                 if(check_audio == 0){
                                        check_audio = device_time;
                                 }else{
                                        wave_player_current_miliseconds_get(&mils_to_set);
                                        Time_offset_player = device_time - mils_to_set - check_audio;
                                 }
                        }else{
                                check_audio = 0;
                        }
         }
         */
        
        Audio_Player_Start_Test();
        
        
          
       if(device_Status == Status_PowerEnable || device_Status == Status_Play || device_Status == Status_Pause){
                generateDMXTimeMatrixEvent(device_time);
        }
        
        
          if(device_Status == Status_Play && was_LTC_Started == 0){
                update_device_time_audio();
          }
          else if(device_Status == Status_Play && was_LTC_Started == 1){
                check_timeCode_IntPlayer();
          }
          
          if(device_Status == Status_PowerEnable){
                GPS_Time_start();
          }
          
          if(device_Status < Status_PowerEnable + 1){
                get_tus();
          }
          

          check_LTC_Pause();
          LTC_Pause();
          
          Wir_learn();
          
          if(is_new_433_BC_msg == 1){ //433Mhz message offset
                
                uint32_t pin_timer_off = 0;
                
                pin_timer_off = PING_433_OFFSET * get_AB_Adre();
        
                if (timer_433_MsgValue >= pin_timer_off)		
                {
                        if(device_Status == Status_Idle)
                        {
                                if(is_433_NC == 1 || is_433_NC > 4){
                                        send_433_mod_ping_reply();
                                        is_433_NC = 2;
                                }
                        }
                        
                        timer_433_MsgValue = 0;
                        is_new_433_BC_msg = 0;                    
                }
        
        
        }
          
  
}


void fsk_gen_Timer(void){
        if(FSK_Frame_rdy == 1){
                     UartFSK_PD_sent(FSK_frame);
                     FSK_Frame_rdy = 0;
        }
        else if(FSK_Frame_rdy == 2){
                     UartFSK_F1_sent(FSK_frame);
                     FSK_Frame_rdy = 0;
        }
}

void GPS_Time_start(void){
        if(isGPSFTValid == 1 && real_time > GPS_SetValue - 1 && real_time < GPS_SetValue + 500)
        {
              SEQ_AddEvent(ev_GB_Short_Press);//GPS Start
              isGPSFTValid = 0; 
        }
}





void check_USB_Con(void)
{
        uint16_t usb_timer_tmp = 0;
        
        if(USB_stats == HOST_IDLE && is_Play_Drive == 0 && (is_init_done == 0 || is_init_done > 9)  && (PPS_Mp3_AB == 2 || PPS_Mp3_AB == 0))
        {
                //do nothing
        }
        else
        {
                Timer_USB_test = 0;
        }
        

       usb_timer_tmp = MAX_USB_TIME;

        
        
                        if(is_CopyMenu != 0 && Timer_USB_test > 1000)
                        {
                                is_CopyMenu = 0;
                        }
                        
                        if(Timer_USB_test > usb_timer_tmp && is_host_off == 0)
                        {
                                //USBH_LL_DriverVBUS(&hUSB_Host, 0);
                                HAL_GPIO_WritePin(HOST_POWERSW_PORT, HOST_POWERSW_VBUS, GPIO_PIN_RESET);
                                DeInit_USB_Drive();
                                //turn off USB host
                                Timer_USB_test = 0;
                                is_host_off = 1;
                        }
                        else if(is_host_off == 1 && Timer_USB_test > 500)
                        {
                                 HAL_GPIO_WritePin(HOST_POWERSW_PORT, HOST_POWERSW_VBUS, GPIO_PIN_RESET);
                                
                                
                                 //if(HAL_GPIO_ReadPin(USB_VBUS_Port, USB_VBUS_Pin) == GPIO_PIN_RESET)
                                if(get_ADC_value(VBUS_ADC_CH) < USB_VAL_VBUS)
                                 {
                                        //USBH_LL_DriverVBUS(&hUSB_Host, 1);
                                        HAL_GPIO_WritePin(STP_CHG_PORT, STP_CHG_PIN, GPIO_PIN_RESET);//disable charge
                                        //HAL_GPIO_WritePin(HOST_POWERSW_PORT, HOST_POWERSW_VBUS, GPIO_PIN_SET);
                                         Init_USB_Drive(); 
                                         
                                        Timer_USB_test = 0;
                                        is_host_off = 0;
                                 }
                                 else
                                 {
                                        DeInit_USB_Drive();
                                        HAL_GPIO_WritePin(STP_CHG_PORT, STP_CHG_PIN, GPIO_PIN_SET);//enable charge
                                        Timer_USB_test = 0;
                                        is_host_off = 3;
                                 }
                               
                        }
                        else if(Timer_USB_test > usb_timer_tmp && is_host_off == 3)
                        {
                                HAL_GPIO_WritePin(HOST_POWERSW_PORT, HOST_POWERSW_VBUS, GPIO_PIN_RESET);
                                
                                
                                //if(HAL_GPIO_ReadPin(USB_VBUS_Port, USB_VBUS_Pin) == GPIO_PIN_RESET)
                                if(get_ADC_value(VBUS_ADC_CH) < USB_VAL_VBUS)
                                 {
                                       // USBH_LL_DriverVBUS(&hUSB_Host, 1);
                                         HAL_GPIO_WritePin(STP_CHG_PORT, STP_CHG_PIN, GPIO_PIN_RESET);//disable charge
                                         //HAL_GPIO_WritePin(HOST_POWERSW_PORT, HOST_POWERSW_VBUS, GPIO_PIN_SET);
                                         Init_USB_Drive(); 
                                         
                                        Timer_USB_test = 0;
                                        is_host_off = 0;
                                 }
                                 else
                                 {
                                        HAL_GPIO_WritePin(STP_CHG_PORT, STP_CHG_PIN, GPIO_PIN_SET);//enable charge
                                        DeInit_USB_Drive();
                                         
                                        Timer_USB_test = 0;
                                        is_host_off = 3;
                                 }
                                
                        }
                        else
                        {
                                //do nothing
                        }
}


void Start_button_disabled(void)
{
        if(is_Start_but_dis == 1)
        {
                is_Start_but_dis_timer++;
                if(is_Start_but_dis_timer > DISABLE_START_TIME)
                {
                        is_Start_but_dis_timer = 0;
                        is_Start_but_dis = 0;
                }
        }
}




void is_fire_tst(void)
{

        if(isFiring != 0)
        {
                if(beep_Timer == 0)
                {
                   HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);     
                   isFiring = 2;
                   beep_Timer++;
                }
                else if(beep_Timer == 10)
                {
                   HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_4);    
                   isFiring = 0;
                   beep_Timer = 0;
               }
                else
               {
                     beep_Timer++;
               }
                
        }
}


void init_test_timer(void)
{
                Test_time = 0;
                isValidTestTimer = 1;
                beep_Timer = 0;
        
  
        
                if(PPS_Mp3_AB == 1)
                        {
                                WavePlayerPauseResume(PAUSE_STATUS);
                        }
                        else if(PPS_Mp3_AB == 3)
                        {
                                WavePlayerPauseResume(RESUME_STATUS);
                        }
                        else if(PPS_Mp3_AB == 2)
                        {
                                wave_player_state_set(STATE_WAVE_PLAYER_START);
                        }
}



void tests_only(void)
{
        #if TEST_DBG
                pwr_stat = HAL_GPIO_ReadPin(PWR_STAT_Port, PWR_STAT_Pin);
                pwr_ctrl = HAL_GPIO_ReadPin(PWR_CTRL_Port, PWR_CTRL_Pin);
                pwr_chg = HAL_GPIO_ReadPin(STP_CHG_PORT, STP_CHG_PIN);
                pwrVBUS_ctrl = HAL_GPIO_ReadPin(HOST_POWERSW_PORT, HOST_POWERSW_VBUS);
                pwrVBUS = HAL_GPIO_ReadPin(USB_VBUS_Port, USB_VBUS_Pin);
        #endif
}


void read_temp_hum(void)
{
                        if(Temp_Hum_refresh > TEM_HUM_REFRESH)
                        {
                                /*
                                Temp_Hum_refresh = 0;
                                read_T_H();
                                */
                        }

}

void init_ping_timer(void){
        ping_timer = DEVICE_PING_PERIOD - 3000;
}

void Ping_Slaves(void)
{

        if(disable_ping == 1){
                ping_timer = 0;
                return;
        }
        
        
        switch (device_Status)
                            {
                            case Status_Idle:
                                         
                                        /* //2.4Ghz
                                             dif_xgfd_rds = systemTimerValue - last_ping_24msg_sent;  
                                             if(is_24Mhz_BC_wait == 0){
                                                                         if((dif_xgfd_rds > PING_24_OFFSET) && Is_Short_Pressed == 0){
                                                                                setTOsalve();
                                                                                sendMSPingBC();
                                                                                ping_timer = 0;
                                                                                last_ping_24msg_sent = systemTimerValue;
                                                                                is_24Mhz_BC_wait = 1;
                                                                        }
                                                                }
                                                                else if(is_24Mhz_BC_wait == 1){
                                                                        if((dif_xgfd_rds > TIME_WAIT_BC_24GHZ) && Is_Short_Pressed == 0){ // check every module one by one
                                                                             is_24Mhz_BC_wait = 2;
                                                                        }else{
                                                                                //do nothing
                                                                        }
                                                                }
                                                                else if(is_24Mhz_BC_wait == 2){
                                                                        if((dif_xgfd_rds > PING_24_OFFSET) && Is_Short_Pressed == 0){ // check every module one by one
                                                                                send_Ping_24_to_nextmod();
                                                                                last_ping_24msg_sent = systemTimerValue;
                                                                        }
                                                                        else{
                                                                                //do nothing
                                                                        }
                                                       }
                                                       */
                                         
                                            if(is_433Mhz_en == 2){
                                                    
                                                   dif_xgfd_rds = systemTimerValue - last_ping_433msg_sent;                                                    
                                                    
                                                    if(dif_xgfd_rds > PING_433_OFFSET){
                                                            //send_uCast_433Mhz_msg(1,0);
                                                            send_4332_msg_to_nextmod_ALL();//only to connected module ping
                                                            last_ping_433msg_sent = systemTimerValue;
                                                    }
                                                    
                                                    
                                                    
                                                    /*

                                                dif_xgfd_rds = systemTimerValue - last_ping_433msg_sent;                                                    

                                                                if(is_433Mhz_BC_wait == 0){
                                                                                 if((dif_xgfd_rds > PING_433_OFFSET) && Is_Short_Pressed == 0){
                                                                                        send_noise_msg();
                                                                                        //last_msg_snt_timer = systemTimerValue; //disabled
                                                                                        send_433_BC_msg();
                                                                                        last_ping_433msg_sent = systemTimerValue;
                                                                                        is_433Mhz_BC_wait = 1;
                                                                                }
                                                                }
                                                                else if(is_433Mhz_BC_wait == 1){
                                                                                if((dif_xgfd_rds > PING_433_OFFSET) && Is_Short_Pressed == 0){ // check every module one by one
                                                                                     is_433Mhz_BC_wait = 2;
                                                                                     last_ping_433msg_sent = systemTimerValue;
                                                                                }else{
                                                                                        //do nothing
                                                                                }

                                                                }
                                                                else if(is_433Mhz_BC_wait == 2){
                                                                        if((dif_xgfd_rds > PING_433_OFFSET) && Is_Short_Pressed == 0){ // check every module one by one
                                                                                send_4332_msg_to_nextmod_ALL();//only to connected module ping
                                                                                last_ping_433msg_sent = systemTimerValue;
                                                                        }
                                                                        else{
                                                                                //do nothing
                                                                        }
                                                                }
                                                                */
                                                        }

                                    break;
                            case Status_PowerEnable:
                                      //DO NO PING
                                        if(is_433Mhz_en == 2){
                                                if(systemTimerValue - last_ping_433msg_sent >  PING_433_OFFSET){ // check every module one by one if are armed
                                                        send_433_arm_check_msg_to_nextmod(); //check if mods are armed on 433Mhz wireless
                                                        last_ping_433msg_sent = systemTimerValue;
                                                }
                                                else{
                                                        //do nothing
                                                }
                                            
                                            } 
                                    break;
                            case Status_Play:
                                     if(ping_timer > DEVICE_SYNC_PERIOD)
                                    {
                                           send_sync_msg(0);
                                           //ping_timer = 0;
                                    }
                                    break;
                            case Status_Pause:
                                     if(ping_timer > DEVICE_PAUSE_PERIOD)
                                    {
                                           //send_sync_msg();
                                           sendMSPause();
                                           ping_timer = 0;
                                    }
                                    break;
                    }

}



uint32_t calculate_ping(void){
        ping_timer_calc = DEVICE_PING_PERIOD + get_slave_con() * PING_OFFSET;
        return ping_timer_calc;
}

VS_VOID RESET_PING_TIMER()
{
        ping_timer = 0;
}

uint32_t Get_PingTimerValue(void)
{
        return ping_timer;
}



int get_is_time_start_start(void){
        return is_time_start_start;
}

void set_is_time_start_start(void){
        is_time_start_start = 0;
}


void Button_Task(void)
{

}

void Button_Run(void)
{
}


void check_LTC_Pause(void){
        if(device_Status == Status_Play){
                if(get_Time_CodeX() == 0 || get_Time_CodeX() == 1){
                        min_time_toPause = 5 * MIN_TIME_TC_WAIT;
                        if(SMPTE_R == 0){
                                if (was_LTC_Started == 1) 
                                {
                                        SMPTE_Pause_Time++;
                                }
                        }
                        else
                        {
                                SMPTE_Pause_Time = 0;
                        }
                        SMPTE_R = 0u; 
                }else if(get_Time_CodeX() == 2 || get_Time_CodeX() == 3){
                        if(get_Time_CodeX() == 2){
                                min_time_toPause = MIN_TIME_TC_WAIT_FSK;
                        }else{
                                min_time_toPause = 6 * MIN_TIME_TC_WAIT_FSK;
                        }
                        if(SMPTE_RF == 0){
                                if (was_LTC_Started == 1) 
                                {
                                        SMPTE_Pause_Time++;
                                }
                        }
                        else
                        {
                                SMPTE_Pause_Time = 0;
                        }
                        SMPTE_RF = 0u; 
                }
        }
}


// Pause/Resume 
void LTC_Pause(void){
        //if(TIME_CODE_BACKUP == 1) 
      //  if(get_TC_BackUp() == 1) 
        if(TIME_CODE_BACKUP == 1)         
        {
                if(was_LTC_Started == 0) return;

                if (SMPTE_Pause_Time > min_time_toPause && SMPTE_Valid_PT && was_LTC_Started == 1 && device_Status == Status_Play && SMPTE_Pause_TimeX == 0){
                        was_LTC_Started = 2;
                }else{
                        SMPTE_Pause_TimeX++;
                        if(SMPTE_Pause_TimeX > 2000){
                                SMPTE_Pause_TimeX = 0;
                        }
                }
        }else{
                if (SMPTE_Pause_Time > min_time_toPause && SMPTE_Valid_PT && was_LTC_Started == 1 && device_Status == Status_Play && SMPTE_Pause_TimeX == 0)                     
                {
                        SMPTE_Pause_Time = 0;
                        SMPTE_Pause_TimeX = 1;
                        // Time code pause
                        SMPTE_Valid_PT = 0;
                        
                        is_Start_but_dis = 1;
                        SEQ_AddEvent(ev_Pause);
                }
                
               
                if(SMPTE_Pause_TimeX > 0){
                        SMPTE_Pause_TimeX++;
                        if(SMPTE_Pause_TimeX > 2000){
                                SMPTE_Pause_TimeX = 0;
                        }
                }
               
        }
}
                


void check_timeCode_IntPlayer(void){
        /*
          if(PPS_Mp3_AB == 1){
                        wave_player_current_miliseconds_get(&millis_AP_Tst);
                        time_difference = millis_AP_Tst - device_time;
          }
        */
}

void update_device_time_audio(){
        
        if(PPS_Mp3_AB == 1){
                  wave_player_current_miliseconds_get(&millis_AP_Tst);
                  time_difference = millis_AP_Tst - device_time; 
        }
        
        
        if(was_sync_by_AB > 0){
                was_sync_by_AB--;
                return;
        }
        if(was_skip_used == 1) return;
        if(get_was_SeqMusic() != 0) return;
        if(PPS_Mp3_AB != 1) return;
        

                  
        if(time_difference < -30 || time_difference > 30){
                device_time = millis_AP_Tst;
                was_updated_time_dev++;
         }
        
}

void Audio_Player_Start_Test(void)
{

        if(is_init_done == 1)
        {
               //Timer_audio_test++; 
                
               if(Timer_audio_test > 1000)
               {
                       wave_player_volume_set(0);
                       
  
                        if(PPS_Mp3_AB == 1)
                        {
                                WavePlayerPauseResume(PAUSE_STATUS);
                        }
                        else if(PPS_Mp3_AB == 3)
                        {
                                WavePlayerPauseResume(RESUME_STATUS);
                        }
                        else if(PPS_Mp3_AB == 2)
                        {
                                wave_player_state_set(STATE_WAVE_PLAYER_START);
                                is_timerGO_valid = 0;
                                is_timerGO_value = 0;                                
                        }
                        Timer_audio_test = 0;
                        is_init_done = 2;
                        
                        wave_player_current_miliseconds_get(&millis_AP_Tst);
                        player_mil_test_start = millis_AP_Tst;
               }
        }
        else if(is_init_done == 2)
        {
               //Timer_audio_test++; 
               if(Timer_audio_test > AP_TEST_TIME)
               {

                       
                    wave_player_current_miliseconds_get(&millis_AP_Tst);
                    if(millis_AP_Tst - player_mil_test_start < AP_TEST_TIME + 100 
                            && millis_AP_Tst - player_mil_test_start > AP_TEST_TIME - 100)
                    {
                            WavePlayerStop();
                            is_init_done = 10; 
                            is_timerGO_value = 0;
                            is_timerGO_valid = 1;
                            Timer_audio_test = 0;
                            wave_player_volume_set(60);
                            
                            rst_player();
                            
                    }
                     else
                     {
                            WavePlayerStop();
                            Timer_audio_test = 0;
                            is_init_done = 100; //Player error please restart
                            wave_player_volume_set(60);
                             
                            rst_player();
                     }
                     
                        if(is_Play_Drive == 1){
                                Test_USB_File();// for test
                                delete_script_int();
                                load_script_auto();
                        }else{
                                Test_SD_File();// FOR TESTS
                                if(get_is_file_exists() == 1){
                                        load_script_auto();
                                }
                        }
               }
        }
        else if(is_init_done == 10)
        {
//                test_music_file();

              /*
                if(PPS_Mp3_AB == 2)
                {
                        Timer_audio_test++; 
                }

                
                if(Timer_audio_test > AP_TEST_TIME)
                {
                      if(PPS_Mp3_AB == 2)
                        {
                                wave_player_state_set(STATE_WAVE_PLAYER_START);
                        }
                        Timer_audio_test = 0;
                }
              */
        }

        else
        {
                        //do nothing
        }
}




void set_AP_Start_time(void)
{
       AP_Start_time_For_Drift = systemTimerValue; 
}

uint64_t GetCurrentSystemTime(void){
      return (uint64_t)(systemTimerValue);
}

uint64_t CheckTimeSpan(uint64_t timestamp){
      uint64_t bufValue=(uint64_t)(systemTimerValue);		
      return (uint64_t)(bufValue-timestamp);
}


uint64_t getDevicetime(void)
 {
       return device_time;
 }


 /*
 void setDevicetime(uint64_t SMPTEtimeRead)
 {
       device_time = SMPTEtimeRead;
}
 */
 
void setDevicetime(uint64_t SMPTEtimeRead)
{
        
        tmp_was_skip = SMPTEtimeRead - device_time - 25;

       
        if((tmp_was_skip > 50 || tmp_was_skip < -50) && device_Status == Status_Play && device_time > 100)
        {
                device_time = SMPTEtimeRead - 25;
                need_Time_up = 1;
                was_here_time_plus++;
        }
        else
        {
                device_time = SMPTEtimeRead -25;
                
        }
}



void set_dev_time(uint32_t dev_timer_x)
{
        device_time = dev_timer_x;
}


VS_VOID RST_DEV_Time()
{
        device_time = 0;
        //time_for_calibration = 0;
        device_timeSYNC = 0;
        isValidDevTimer = 0;
        is_audio_skip = 0;
}

VS_VOID Dev_Valid_Timer_Set()
{
        isValidDevTimer = 1;
}

VS_VOID Dev_Valid_Timer_Rst()
{
        isValidDevTimer = 0;
}




void init_delay_timer(uint32_t delay_ms)
{
        delay_tim_TH = delay_ms;
        is_delayTim_valid = 1;
        //delayTim_Val = 0;

}

void rst_delay_timer(void)
{
        delay_tim_TH = 0;
        is_delayTim_valid = 0;
        delayTim_Val = 0;
}


void delay_timer(void)
{
        if(is_delayTim_valid == 1)
        {
                delayTim_Val++;
                if(delayTim_Val > delay_tim_TH)
                {
                        //do_delay_timer();
                        must_reply_with_stat = 1;
                        is_delayTim_valid = 0;
                        delayTim_Val = 0;
                        delay_tim_TH = 0;
                }
        }
}

void do_delay_timer(void)
{
        sendSMStatusInterface();
}

VS_VOID DELAY_STAT_MSG()
{
        init_delay_timer(100);
}



void syncMSTimerAB(MS_Message *msg)
        {
       // int devicetime_MS = 0;
        //int devicetime_MSX = 0;
        
         if(was_sync_by_433MHz > 0){
                was_sync_by_433MHz--;
                return;
         }
        
        sync = 1;
        
      //  devicetime_MS = ((msg->data[6] & 0xF0) << 20) | (msg->data[2] << 16) | (msg->data[1] << 8) | (msg->data[0] << 0);
                
        if (msg->dataLength == 12){//message is received form an Audiobox
                was_sync_by_AB = 7000;
                time_ofsset_AB = (((msg->data[8] << 0) & 0xFF) | ((msg->data[9] << 8) & 0xFF00) | ((msg->data[10] << 16) & 0xFF0000) | ((msg->data[11] << 24) & 0xFF000000));
                device_time = device_time + time_ofsset_AB;
        }
        

        sync = 0;
}


//add 07.12.2015
void syncMSTimerS(MS_Message *msg){
        
        
        uint32_t devicetime_MS = 0;
        
        sync = 1;
        was_offset = 1;
        devicetime_MS = (msg->data[3] << 24) | (msg->data[2] << 16) | (msg->data[1] << 8) | (msg->data[0] << 0);
        device_time = devicetime_MS + 20;
        sync = 0;
}


void change_dev_time(int adjust_time){
        if(device_Status > Status_PowerEnable)
        {
                //if(device_time < 9000) return;
                
                if(device_time + (100 * (Adjusted_time - 100)) < 10) return;
                
                
                if(Adjusted_time == 255) {
                        Adjusted_time = 100;
                }
                
                if((Adjusted_time < 1 && adjust_time < 0) || (Adjusted_time > 199 && adjust_time > 0))
                {
                        // do nothing
                }
                else
                {
                        Adjusted_time = Adjusted_time + adjust_time;
                        
                       // reset_TimeAdj_line();
                }
                
                if(Adjusted_time < 0)
                {
                        Adjusted_time = 0;
                }
                else if(Adjusted_time > 200)
                {
                        Adjusted_time = 200;
                }
        }
}


void rst_prg_param(uint8_t exit_param)
{
        cur_mfp = 0;
        start_prg = 0;
        ev_to_prg = 0;
        
        if(exit_param == 1)
        {
                fire_type = 0;
        }
}


uint8_t was_mod_Check_comp(uint8_t tmp_cur_mfp) // TO ADD
{
        
        return 1;

}

void initTimer4X(void)
{
        timerUpValue = 0; 
}

void reset_prg_time(void)
{
        start_prg = 1; 

}

uint8_t Del_or_Prg = 0; // 1 to 99 programing specific; 100 - Programing all, 101 to 199 delete specific; 200 - Delete all




void start_buz(uint16_t msTime_up){
        if(get_buz_stat() == 1){
                Buz_time = msTime_up;
                start_Buz_PWM();
        }
}


void rst_arm_timings(void){
        is_not_all_arm_confirmed = 0;
        arm_timer_check = 0;
}

void set_arm_timings(void){
        is_not_all_arm_confirmed = 1;
        arm_timer_check = 0;
}


void Wir_learn(void){
        if(disable_ping == 0) return;
        if(device_Status > Status_Idle) return;
        if(Timer_Wir_learn > FIRST_MSG_SENT){
                Timer_Wir_learn = 0;
                wir_msg_snt_learn++;
        }
        
        if(wir_msg_snt_learn == 1){
                        sendMSJoinBroadcastWX(wir_to_learn);
                        wir_msg_snt_learn = 2;
        }
        else if(wir_msg_snt_learn == 3){
                        sendMSJoinBroadcastWX(wir_to_learn);
                        wir_msg_snt_learn = 4;
        }
        else if(wir_msg_snt_learn == 5){
                Timer_Wir_learn = 0;
                //wir_msg_snt_learn = 0;
                       Config_Wir(wir_to_learn); 
                       //save_network_name(); 
                       HAL_NVIC_SystemReset();
        }
        
}


uint8_t get_timer_smpte_active(void){
        return timer_smpte_active;
}


uint8_t get_was_canceled(void){
        return was_canceled_script;
}

void rst_was_canceled_script(void){
        was_canceled_script = 0;
}




void update_TC_timer(int FSK_timeX)
 {
       
       smpte_time = FSK_timeX;
         
        if(device_Status == Status_Play && smpte_time != 0 && was_LTC_Started > 0)
        {
              setDevicetime(smpte_time);
        }
       
         /*
       if(device_Status == Status_Pause)
        {
              SMPTE_R = 0u;
              SMPTE_Valid_PT = 1;
              SEQ_AddEvent(ev_SMPTEstartP);
              
        }
       else if(device_Status == Status_Idle )
        {
              SMPTE_Valid_XT = 1;
        }
       else if(device_Status == Status_PowerEnable && smpte_time != 0 && get_TC_val() == 0)
        {
              setDevicetime(smpte_time);
              SMPTE_Valid_PT = 1;
              was_LTC_Started = 1;
              SEQ_AddEvent(ev_SMPTEstart);
        }
       
       else if(device_Status == Status_Play && smpte_time != 0 && was_LTC_Started == 1 && get_TC_val() == 0)
        {
              setDevicetime(smpte_time);
        }
        */
 }

 
 void set_last_msg_with_reply_wait(void){
             Time_when_asked_last_msg = GetCurrentSystemTime();
}


void send_433_wireless_msg_on_uart(void){
        //if((systemTimerValue - last_msg_snt_timer > MSG_433_TIME_OFFSET) && (systemTimerValue - Time_when_asked_last_msg > PING_433_OFFSET)){
        if((systemTimerValue - last_msg_snt_timer > MSG_433_TIME_OFFSET) && msg_to_be_snt_433Mhz > 0){
                
                if(SB_AsMaster == 1){
                        if(device_Status == Status_PowerEnable){
                                if((systemTimerValue - timer_last_msg_ping > 150 && systemTimerValue - timer_last_msg_ping < 29850) || systemTimerValue - timer_last_msg_ping > 30150){
                                        wireless_433Mhz_send();
                                }
                        }else{
                                wireless_433Mhz_send();
                        }
                }else{
                        wireless_433Mhz_send();// send 433Mhz wireless messages TO ADD to know how many messages remain to be sent
                        last_msg_snt_timer = systemTimerValue;
                }
                
                /*
                wireless_433Mhz_send();// send 433Mhz wireless messages TO ADD to know how many messages remain to be sent
                last_msg_snt_timer = systemTimerValue;
                */
        }
}

void syncMSTimer433X(uint32_t new_AB_time){
        device_time = new_AB_time +  TIME_433_OFFSET;
        was_sync_by_AB = 7000;
        was_sync_by_433MHz = 5;
        
}




void Blue_button(void)
{
//Blue button control    
        if(isValidBlueT == 1)
        {
                BlueTimerPress++;
                if(BlueTimerPress > MIN_LONG_PRESS_TIME)
                {
                        //do long time press
                        isValidBlueT = 3;
                        isValidBlueX = 1;
                        BlueTimerPress = 0;
                        if(is_CopyMenu == 0)
                        {
                                SEQ_AddEvent(ev_BB_Long_Press);
                        }
                }
        }
        else if(isValidBlueT == 0)
        {
                BlueTimerUnpress++;
                
                
                 if(isValidBlueX == 1)
                {
                        BlueTimerUnpress = 0;
                        BlueTimerPress = 0;
                        isValidBlueX = 0;
                }
                
                
                if(BlueTimerPress > MIN_SHORT_PRESS_TIME && BlueTimerUnpress > MIN_UNPRESSED_TIME)
                {
                        //do short time press
                        isValidBlueT = 4;
                        BlueTimerPress = 0;
                        BlueTimerUnpress = 0;
                        isLongBPress = 0;
                        
                        if(set_AD_ID == 1){
                                if(get_menu_item() == 22){
                                        timer_saving(0);
                                }else if(get_menu_item() == 23){
                                        timer_Start_saving(0);
                                }else if(get_menu_item() == 50){
                                        go_Back_main();
                                }
                        }
                        else if(is_CopyMenu == 1)
                        {
                                is_CopyMenu = 2;
                                USBH_USR_ApplicationState = USBH_USR_FS_INIT;
                        }
                        else if(is_CopyMenu == 3)
                        {
                                is_CopyMenu = 10;
                                USBH_USR_ApplicationState = USBH_USR_FS_INIT;
                        }
                        else if(PPS_Mp3_AB == 2 && device_Status < 3)
                        {
                               // Stop_DMX();
                                
                                //set_rev_display(); //MoveOut from IRQ
                                //load_script_auto();
                        }
                        else
                        {
                                if(device_Status == Status_Play && was_LTC_Started == 1){
                                        was_ButStoped = 1;
                                 }
                                SEQ_AddEvent(ev_BB_Short_Press);
                               // load_script_auto();
                        }
                }
        }
        else if(isValidBlueT == 3)
        {
                BlueTimerPress++;
                if(BlueTimerPress > MIN_LONG_PRESS_TIME_X)
                {
                        isValidBlueX = 1;
                        BlueTimerPress = 0;
                        if(is_CopyMenu == 0 || is_CopyMenu == 10)//do long time fast
                        {
                                SEQ_AddEvent(ev_BB_Long_Press);
                        }
                }
                
                if(get_volume() == 20)
                {
                       isLongBPress++;
                }
                       if(isValidGreenT == 4 && isLongBPress > (3 * MIN_LONG_PRESS_TIME))
                       {
                                //ENABLE_PWR_BUTTON();
                                isLongBPress = 0;
                       }
        }
        else
        {
                //do nothing_ button event was made
        }


}

void Green_button(void)
{
//Green button control     
        if(isValidGreenT == 1)
        {
                GreenTimerPress++;
                if(GreenTimerPress > MIN_LONG_PRESS_TIME)
                {
                        //do long time press
                        isValidGreenT = 3;
                        isValidGreenX = 1;
                        GreenTimerPress = 0;
                        if(is_CopyMenu == 0)
                        {
                                SEQ_AddEvent(ev_GB_Long_Press);
                        }
                }
        }       
        else if(isValidGreenT == 0)
        {
                GreenTimerUnpress++;
                
                if(isValidGreenX == 1)
                {
                        GreenTimerUnpress = 0;
                        GreenTimerPress = 0;
                        isValidGreenX = 0;
                }
                
                if(GreenTimerPress > MIN_SHORT_PRESS_TIME && GreenTimerUnpress > MIN_UNPRESSED_TIME)
                {
                        //do short time press
                        isValidGreenT = 4;
                        GreenTimerPress = 0;
                        GreenTimerUnpress = 0;
                        if(set_AD_ID == 1){
                                if(get_menu_item() == 22){
                                        timer_saving(1);
                                }else if(get_menu_item() == 23){
                                        timer_Start_saving(1);
                                }else if(get_menu_item() == 50){
                                        set_default_settings();
                                        go_Back_main();
                                }
                        }
                        else if(is_CopyMenu == 1)
                        {
                                is_CopyMenu = 10;
                        }
                        else if(is_CopyMenu == 3)
                        {
                                is_CopyMenu = 10;
                                USBH_USR_ApplicationState = USBH_USR_FS_INIT;
                        }
                        else{
                                 if(device_Status == Status_Play && was_LTC_Started == 1){
                                        was_ButStoped = 1;
                                 }
                                 else if(device_Status < Status_PowerEnable){
                                        was_ButStoped = 0;
                                 }
                                 
                                 if(SB_AsMaster == 1 && device_Status == Status_PowerEnable){
                                         SEQ_AddEvent(ev_GB_Short_PressX); 
                                 }else{
                                        SEQ_AddEvent(ev_GB_Short_Press); 
                                 }
                                //run_DMX();
                        }
                }
        }
        else if(isValidGreenT == 3)
        {
                GreenTimerPress++;
                if(GreenTimerPress > MIN_LONG_PRESS_TIME_X)
                {
                        isValidGreenX = 1;
                        GreenTimerPress = 0;
                        if(is_CopyMenu == 0 || is_CopyMenu == 10)//do long time fast
                        {
                                SEQ_AddEvent(ev_GB_Long_Press);
                        }
                }
        }
        else
        {
                //do nothing_ button event was made
        }

}


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
        uint32_t pin_val = 0;
        
        if(GPIO_Pin == GreenButton_GPIO_Pin)//Green Button
        {
                pin_val = HAL_GPIO_ReadPin(GreenButton_GPIO_Port, GreenButton_GPIO_Pin);
                if(pin_val == 0)
                {
                        isValidGreenT = 1u;
                        //GreenTimerPress = 0u;
                        GreenTimerUnpress = 0u;
                }
                else
                {
                        isValidGreenT = 0u;
                        GreenTimerUnpress = 0u;
                        //GreenTimerPress = 0u;

                }
        
        }
        else if(GPIO_Pin == BlueButton_GPIO_Pin)//Blue Button
        {
                pin_val = HAL_GPIO_ReadPin(BlueButton_GPIO_Port, BlueButton_GPIO_Pin);
                
                if(pin_val == 0)
                {
                        isValidBlueT = 1u;
                        //BlueTimerPress = 0u;
                        BlueTimerUnpress = 0u;
                }
                else
                {
                        isValidBlueT = 0u;
                        //BlueTimerPress = 0u;
                        BlueTimerUnpress = 0u;

                }
        }
}


void gps_pps_pin(void){

        uint8_t pin_val = 0;
        static uint8_t pin_val_timer = 0;
        static uint8_t pin_val_timer_valid = 0;
        pin_val = HAL_GPIO_ReadPin(GPS_PPS_PORT, GPS_PPS_PIN);
        if(pin_val == 1){
                if(pin_val_timer_valid == 0){
                        pin_val_timer++;
                        if(pin_val_timer > 3){
                                pin_val_timer = 0;
                                pin_val_timer_valid = 1;
                                run_pps_function();
                        }
                }
        }else{
                if(pin_val_timer_valid == 1){
                        pin_val_timer++;
                        if(pin_val_timer > 100){
                                pin_val_timer = 0;
                                pin_val_timer_valid = 0;
                        }
                }
        
        
        }
        
}



void run_pps_function(void){
        uint32_t tmp_xxstimer = systemTimerValue;
                                if(tmp_xxstimer - PPS_Time_Received > 950 && tmp_xxstimer - PPS_Time_Received < 1050){
                                        set_GPS_time();
                                       // need_GPS_update = 1;
                                }
                                PPS_Time_Received = tmp_xxstimer;
}



void gps_timer_Update(void){
        if(need_GPS_update == 1){
                need_GPS_update = 0;
                if((systemTimerValue - PPS_Time_Received) < 10){
                        set_GPS_time();
                }else{
                        failed_gps_up++;
                }
        }
}



uint8_t ret_Green_Pin(void)
{
        return isValidGreenT;
}

uint8_t ret_Blue_Pin(void)
{
        return isValidBlueT;
}


void sync_time_code(uint32_t time_to_use){
        
        secondCount = (uint8_t) (time_to_use / 1000) % 60;
        minuteCount = (uint8_t) ((time_to_use / (1000 * 60)) % 60);
        hourCount = (uint8_t) ((time_to_use / (1000 * 60 * 60)) % 24);
        frameCount = 0;
        
}

/*
void sync_time_code(void){
        uint32_t time_to_use = device_time;
        
        secondCount = (uint8_t) (time_to_use / 1000) % 60;
        minuteCount = (uint8_t) ((time_to_use / (1000 * 60)) % 60);
        hourCount = (uint8_t) ((time_to_use / (1000 * 60 * 60)) % 24);
        frameCount = 0;
        
}
*/


void setLevel(void)
{
  switch(bitCount)
  {
    case 0:
      polarBit = 0;
      currentBit = (((frameCount % 10) >> (1 - 1)) & 1);
    
      TC_pin_ctrl();
    
      if((updateCnt)&(!currentBit)){polarBit=!polarBit;};
      break;
    case 1:
      currentBit = (((frameCount % 10) >> (2 - 1)) & 1);
      
      TC_pin_ctrl();
    
      if((updateCnt)&(!currentBit)){polarBit=!polarBit;};
      break;
    case 2:
      currentBit = (((frameCount % 10) >> (3 - 1)) & 1);
      
      TC_pin_ctrl();
    
      if((updateCnt)&(!currentBit)){polarBit=!polarBit;};
      break;
    case 3:
      currentBit = (((frameCount % 10) >> (4 - 1)) & 1);
      TC_pin_ctrl();
      if((updateCnt)&(!currentBit)){polarBit=!polarBit;};
      break;
    case 4:
      currentBit = 0;
      TC_pin_ctrl();
      if((updateCnt)&(!currentBit)){polarBit=!polarBit;};
      break;
    case 5:
      currentBit = 0;
      TC_pin_ctrl();
      if((updateCnt)&(!currentBit)){polarBit=!polarBit;};
      break;
    case 6:
      currentBit = 0;
      TC_pin_ctrl();
      if((updateCnt)&(!currentBit)){polarBit=!polarBit;};
      break;
    case 7:
      currentBit = 0;
      TC_pin_ctrl();
      if((updateCnt)&(!currentBit)){polarBit=!polarBit;};
      break;
    case 8:
      currentBit = (((frameCount / 10 % 10) >> (1 - 1)) & 1);
      TC_pin_ctrl();
      if((updateCnt)&(!currentBit)){polarBit=!polarBit;};
      break;  
    case 9:
      currentBit = (((frameCount / 10 % 10) >> (2 - 1)) & 1);
      TC_pin_ctrl();
      if((updateCnt)&(!currentBit)){polarBit=!polarBit;};
      break;  
    case 10:
      currentBit = 0;
      TC_pin_ctrl();
      if((updateCnt)&(!currentBit)){polarBit=!polarBit;};
      break;
    case 11:
      currentBit = 0;
      TC_pin_ctrl();
      if((updateCnt)&(!currentBit)){polarBit=!polarBit;};
      break;
    case 12:
      currentBit = 0;
      TC_pin_ctrl();
      if((updateCnt)&(!currentBit)){polarBit=!polarBit;};
      break;
    case 13:
      currentBit = 0;
      TC_pin_ctrl();
      if((updateCnt)&(!currentBit)){polarBit=!polarBit;};
      break;
    case 14:
      currentBit = 0;
      TC_pin_ctrl();
      if((updateCnt)&(!currentBit)){polarBit=!polarBit;};
      break;
    case 15:
      currentBit = 0;
      TC_pin_ctrl();
      if((updateCnt)&(!currentBit)){polarBit=!polarBit;};
      break;
    case 16:
      currentBit = (((secondCount % 10) >> (1 - 1)) & 1);
      TC_pin_ctrl();
      if((updateCnt)&(!currentBit)){polarBit=!polarBit;};
      break;
    case 17:
      currentBit = (((secondCount % 10) >> (2 - 1)) & 1);
      TC_pin_ctrl();
      if((updateCnt)&(!currentBit)){polarBit=!polarBit;};
      break;
    case 18:
      currentBit = (((secondCount % 10) >> (3 - 1)) & 1);
      TC_pin_ctrl();
      if((updateCnt)&(!currentBit)){polarBit=!polarBit;};
      break;
    case 19:
      currentBit = (((secondCount % 10) >> (4 - 1)) & 1);
      TC_pin_ctrl();
      if((updateCnt)&(!currentBit)){polarBit=!polarBit;};
      break;
    case 20:
      currentBit = 0;
      TC_pin_ctrl();
      if((updateCnt)&(!currentBit)){polarBit=!polarBit;};
      break;
    case 21:
      currentBit = 0;
      TC_pin_ctrl();
      if((updateCnt)&(!currentBit)){polarBit=!polarBit;};
      break;
    case 22:
      currentBit = 0;
      TC_pin_ctrl();
      if((updateCnt)&(!currentBit)){polarBit=!polarBit;};
      break;
    case 23:
      currentBit = 0;
      TC_pin_ctrl();
      if((updateCnt)&(!currentBit)){polarBit=!polarBit;};
      break;
    case 24:
      currentBit = (((secondCount /10 % 10) >> (1 - 1)) & 1);
      TC_pin_ctrl();
      if((updateCnt)&(!currentBit)){polarBit=!polarBit;};
      break;
    case 25:
      currentBit = (((secondCount /10 % 10) >> (2 - 1)) & 1);
      TC_pin_ctrl();
      if((updateCnt)&(!currentBit)){polarBit=!polarBit;};
      break; 
    case 26:
      currentBit = (((secondCount /10 % 10) >> (3 - 1)) & 1);
      TC_pin_ctrl();
      if((updateCnt)&(!currentBit)){polarBit=!polarBit;};
      break;
    case 27:
      currentBit = 0;
      TC_pin_ctrl();
      if((updateCnt)&(!currentBit)){polarBit=!polarBit;};
      break;
    case 28:
      currentBit = 0;
      TC_pin_ctrl();
      if((updateCnt)&(!currentBit)){polarBit=!polarBit;};
      break;
    case 29:
      currentBit = 0;
      TC_pin_ctrl();
      if((updateCnt)&(!currentBit)){polarBit=!polarBit;};
      break;
    case 30:
      currentBit = 0;
      TC_pin_ctrl();
      if((updateCnt)&(!currentBit)){polarBit=!polarBit;};
      break;
    case 31:
      currentBit = 0;
      TC_pin_ctrl();
      if((updateCnt)&(!currentBit)){polarBit=!polarBit;};
      break;
    case 32:
      currentBit = (((minuteCount % 10) >> (1 - 1)) & 1);
      TC_pin_ctrl();
      if((updateCnt)&(!currentBit)){polarBit=!polarBit;};
      break;
    case 33:
      currentBit = (((minuteCount % 10) >> (2 - 1)) & 1);
      TC_pin_ctrl();
      if((updateCnt)&(!currentBit)){polarBit=!polarBit;};
      break;
    case 34:
      currentBit = (((minuteCount % 10) >> (3 - 1)) & 1);
      TC_pin_ctrl();
      if((updateCnt)&(!currentBit)){polarBit=!polarBit;};
      break;
    case 35:
      currentBit = (((minuteCount % 10) >> (4 - 1)) & 1);
      TC_pin_ctrl();
      if((updateCnt)&(!currentBit)){polarBit=!polarBit;};
      break;
    case 36:
      currentBit = 0;
      TC_pin_ctrl();
      if((updateCnt)&(!currentBit)){polarBit=!polarBit;};
      break;
    case 37:
      currentBit = 0;
      TC_pin_ctrl();
      if((updateCnt)&(!currentBit)){polarBit=!polarBit;};
      break;
    case 38:
      currentBit = 0;
      TC_pin_ctrl();
      if((updateCnt)&(!currentBit)){polarBit=!polarBit;};
      break;
    case 39:
      currentBit = 0;
      TC_pin_ctrl();
      if((updateCnt)&(!currentBit)){polarBit=!polarBit;};
      break;
    case 40:
      currentBit = (((minuteCount / 10 % 10) >> (1 - 1)) & 1);
      TC_pin_ctrl();
      if((updateCnt)&(!currentBit)){polarBit=!polarBit;};
      break;
    case 41:
      currentBit = (((minuteCount / 10 % 10) >> (2 - 1)) & 1);
      TC_pin_ctrl();
      if((updateCnt)&(!currentBit)){polarBit=!polarBit;};
      break;
    case 42:
      currentBit = (((minuteCount / 10 % 10) >> (3 - 1)) & 1);
      TC_pin_ctrl();
      if((updateCnt)&(!currentBit)){polarBit=!polarBit;};
      break;
    case 43:
      currentBit = 0;
      TC_pin_ctrl();
      if((updateCnt)&(!currentBit)){polarBit=!polarBit;};
      break;
    case 44:
      currentBit = 0;
      TC_pin_ctrl();
      if((updateCnt)&(!currentBit)){polarBit=!polarBit;};
      break; 
    case 45:
      currentBit = 0;
      TC_pin_ctrl();
      if((updateCnt)&(!currentBit)){polarBit=!polarBit;};
      break;
    case 46:
      currentBit = 0;
      TC_pin_ctrl();
      if((updateCnt)&(!currentBit)){polarBit=!polarBit;};
      break;
    case 47:
      currentBit = 0;
      TC_pin_ctrl();
      if((updateCnt)&(!currentBit)){polarBit=!polarBit;};
      break; 
    case 48:
      currentBit = (((hourCount % 10) >> (1 - 1)) & 1);
      TC_pin_ctrl();
      if((updateCnt)&(!currentBit)){polarBit=!polarBit;};
      break;
    case 49:
      currentBit = (((hourCount % 10) >> (2 - 1)) & 1);
      TC_pin_ctrl();
      if((updateCnt)&(!currentBit)){polarBit=!polarBit;};
      break;
    case 50:
      currentBit = (((hourCount % 10) >> (3 - 1)) & 1);
      TC_pin_ctrl();
      if((updateCnt)&(!currentBit)){polarBit=!polarBit;};
      break;
    case 51:
      currentBit = (((hourCount % 10) >> (4 - 1)) & 1);
      TC_pin_ctrl();
      if((updateCnt)&(!currentBit)){polarBit=!polarBit;};
      break;
    case 52:
      currentBit = 0;
      TC_pin_ctrl();
      if((updateCnt)&(!currentBit)){polarBit=!polarBit;};
      break;
    case 53:
      currentBit = 0;
      TC_pin_ctrl();
      if((updateCnt)&(!currentBit)){polarBit=!polarBit;};
      break;
    case 54:
      currentBit = 0;
      TC_pin_ctrl();
      if((updateCnt)&(!currentBit)){polarBit=!polarBit;};
      break;
    case 55:
      currentBit = 0;
      TC_pin_ctrl();
      if((updateCnt)&(!currentBit)){polarBit=!polarBit;};
      break;
    case 56:
      currentBit = (((hourCount / 10 % 10) >> (1 - 1)) & 1);
      TC_pin_ctrl();
      if((updateCnt)&(!currentBit)){polarBit=!polarBit;};
      break;
    case 57:
      currentBit = (((hourCount / 10 % 10) >> (2 - 1)) & 1);
      TC_pin_ctrl();
      if((updateCnt)&(!currentBit)){polarBit=!polarBit;};
      break; 
    case 58:
      currentBit = 0;
      TC_pin_ctrl();
      if((updateCnt)&(!currentBit)){polarBit=!polarBit;};
      break;
    case 59:
      currentBit = (polarBit);
      TC_pin_ctrl();
      if((updateCnt)&(!currentBit)){polarBit=!polarBit;};
      break;
    case 60:
      currentBit = 0;
      TC_pin_ctrl();
      if((updateCnt)&(!currentBit)){polarBit=!polarBit;};
      break;
    case 61:
      currentBit = 0;
      TC_pin_ctrl();
      if((updateCnt)&(!currentBit)){polarBit=!polarBit;};
      break;
    case 62:
      currentBit = 0;
      TC_pin_ctrl();
      if((updateCnt)&(!currentBit)){polarBit=!polarBit;};
      break;
    case 63:
      currentBit = 0;
      TC_pin_ctrl();
      if((updateCnt)&(!currentBit)){polarBit=!polarBit;};
      break;
    case 64:
      currentBit = 0;
      TC_pin_ctrl();
      if((updateCnt)&(!currentBit)){polarBit=!polarBit;};
      break;
    case 65:
      currentBit = 0;
      TC_pin_ctrl();
      if((updateCnt)&(!currentBit)){polarBit=!polarBit;};
      break;
    case 66:
      currentBit = 1;
      TC_pin_ctrl();
      if((updateCnt)&(!currentBit)){polarBit=!polarBit;};
      break;
    case 67:
      currentBit = 1;
      TC_pin_ctrl();
      if((updateCnt)&(!currentBit)){polarBit=!polarBit;};
      break;
    case 68:
      currentBit = 1;
      TC_pin_ctrl();
      if((updateCnt)&(!currentBit)){polarBit=!polarBit;};
      break;
    case 69:
      currentBit = 1;
      TC_pin_ctrl();
      if((updateCnt)&(!currentBit)){polarBit=!polarBit;};
      break;
    case 70:
      currentBit = 1;
      TC_pin_ctrl();
      if((updateCnt)&(!currentBit)){polarBit=!polarBit;};
      break;
    case 71:
      currentBit = 1;
      TC_pin_ctrl();
      if((updateCnt)&(!currentBit)){polarBit=!polarBit;};
      break;
    case 72:
      currentBit = 1;
      TC_pin_ctrl();
      if((updateCnt)&(!currentBit)){polarBit=!polarBit;};
      break;
    case 73:
      currentBit = 1;
      TC_pin_ctrl();
      if((updateCnt)&(!currentBit)){polarBit=!polarBit;};
      break;
    case 74:
      currentBit = 1;
      TC_pin_ctrl();
      if((updateCnt)&(!currentBit)){polarBit=!polarBit;};
      break;
    case 75:
      currentBit = 1;
      TC_pin_ctrl();
      if((updateCnt)&(!currentBit)){polarBit=!polarBit;};
      break;
    case 76:
      currentBit = 1;
      TC_pin_ctrl();
      if((updateCnt)&(!currentBit)){polarBit=!polarBit;};
      break;
    case 77:
      currentBit = 1;
      TC_pin_ctrl();
      if((updateCnt)&(!currentBit)){polarBit=!polarBit;};
      break;
    case 78:
      currentBit = 0;
      TC_pin_ctrl();
      if((updateCnt)&(!currentBit)){polarBit=!polarBit;};
      break;
    case 79:
      currentBit = 1;
      TC_pin_ctrl();
      if((updateCnt)&(!currentBit)){polarBit=!polarBit;};
      
      //for tests
    //  time_difference222 = systemTimerValue - time_code_start_saved - smpte_time;
      

      
      break;
    default:
      HAL_GPIO_TogglePin(CTR_TC_GPIO_Port, CTR_TC_Pin);
  }
}


void timeRealUpdate(uint32_t time_to_X)
{
    if(bitCount < 79)
    {
      bitCount ++;
      lastLevel = 0;
    }
    else
    {
            
           
      bitCount = 0;
      lastLevel = 0;
       static uint32_t milis = 0;
        
        hourCount = time_to_X / 3600000;
        time_to_X = time_to_X - 3600000 * hourCount;
        //60000 milliseconds in a minute
        minuteCount = time_to_X / 60000;
        time_to_X = time_to_X - 60000 * minuteCount;

        //1000 milliseconds in a second
        secondCount = time_to_X / 1000;
        milis = time_to_X - 1000 * secondCount;
       
       frameCount_XXX = frameCount;    
            
       if(fps_gen == 25){
                frameCount = milis/40;
       }else{
               tmp_mil = (float)(milis);
               tmp_mil2 = milis/34.35f;

              frameCount = (int)(tmp_mil2);
               
               // frameCount = milis/34;
       }
            
        if(frameCount > 29){
                        
                        err_freame2++;
       }
        

       int frame_stats = frameCount - frameCount_XXX;
       
       if(frame_stats != 1 && frame_stats != -29){
               
               if(frameCount == frameCount_XXX){
                        err_freame0++;
                        if(frameCount == 29){
                                frameCount = 1;
                        }else{
                                frameCount++;
                        }
               
               }else{
                        err_freame1++;
               }
       }
        
       saved_mills = milis; 
            
          
           
    }
                
}

void timeUpdate(void)
{
  
  if(bitCount < 79)
    {
      bitCount ++;
      lastLevel = 0;
    }
    else
    {
            
         
            
      bitCount = 0;
      lastLevel = 0;
      if(frameCount < fps_gen)
      {
        frameCount ++;
      }
      else
      {
        frameCount = 0;
        if(secondCount < 59)
        {
          secondCount ++;
        }
        else
        {
          secondCount = 0;
          if(minuteCount < 59)
          {
            minuteCount ++;
          }
          else
          {
            minuteCount = 0;
            if(hourCount < 23)
            {
              hourCount ++;
            }
            else
            {
              hourCount = 0;
            }
          }
        }
      }      
    }
    
}



/**
  * @brief This function handles TIM7 global interrupt.
  */
void TIM7_IRQHandler(void)
{
  /* USER CODE BEGIN TIM7_IRQn 0 */

  /* USER CODE END TIM7_IRQn 0 */
  HAL_TIM_IRQHandler(&htim7);
  /* USER CODE BEGIN TIM7_IRQn 1 */
  timer_TC_task();

//      HAL_GPIO_TogglePin(CTR_TC_GPIO_Port, CTR_TC_Pin);
  /* USER CODE END TIM7_IRQn 1 */
}


void timer_TC_task(void)
{
  setLevel();

  if(updateCnt == 0)
  {
    updateCnt ++;
  }
  else
  {
    updateCnt = 0;
 //   timeUpdate();
    //timeRealUpdate(device_time);
          
            if(device_Status < Status_PowerEnable){
                timeRealUpdate(Timer_forGenerate_time_code + get_time_Code_ofset()); 
            }else{
                timeRealUpdate(device_time + get_time_Code_ofset()); 
            }
  }          
}

void TC_pin_ctrl(void){
            if(currentBit == 0){
                      if(lastLevel == 0){
                              lastLevel = 1;
                              HAL_GPIO_TogglePin(CTR_TC_GPIO_Port, CTR_TC_Pin);
                      }else{
                              lastLevel = 0;
                      }
              }
              else if(currentBit == 1){
                      lastLevel = !lastLevel;
                      HAL_GPIO_TogglePin(CTR_TC_GPIO_Port, CTR_TC_Pin);
              }
}


uint16_t get_Timer_autoset(void){
        return ID_autoset_timer;
}

void read_encoder_pins(void){
        
uint8_t Rot_enc_A = 0;
uint8_t Rot_enc_B = 0;
//uint8_t Rot_enc_D = 0;

static uint8_t encoder0PinALast = 0;

int encoder0Pos = 0;


uint8_t _currValueAB = 0;
static uint8_t _prevValueAB = 0;
int _counter = 0;
        
        Rot_enc_A = HAL_GPIO_ReadPin(ROT_ENCODE_A_PORT, ROT_ENCODE_A_PIN);
        Rot_enc_B = HAL_GPIO_ReadPin(ROT_ENCODE_B_PORT, ROT_ENCODE_B_PIN);
     //   Rot_enc_D = HAL_GPIO_ReadPin(ROT_ENCODE_D_PORT, ROT_ENCODE_D_PIN);
        
 
  if ((encoder0PinALast == 0) && (Rot_enc_A == 1)) {
    if (Rot_enc_B == 0) {
      encoder0Pos++;
    } else {
      encoder0Pos--;
    }
  }
  encoder0PinALast = Rot_enc_A;
  
  _currValueAB = (uint8_t)Rot_enc_A << 1;
  _currValueAB |= (uint8_t)Rot_enc_B;
  
  switch((_prevValueAB | _currValueAB))
  {
 
        case 1: //0b0001:
        _counter--;
        break;
        case 14: //0b1110:
        _counter--;
        break;

        case 4: //0b0100: 
        _counter++;
        break;
        case 11://0b1011:                                
        _counter++;
        break;
  }

  _prevValueAB = _currValueAB << 2;                             //update previouse state

}

void set_time_code_Gen_SMPTE_30(void){
        fps_gen = 29;
        Enable_TC_SMPTE_Gen_Pin();
        MX_TIM7_Init();
        HAL_TIM_Base_Start_IT(&htim7);
}

void set_time_code_Gen_SMPTE_25(void){
        fps_gen = 25;
        Enable_TC_SMPTE_Gen_Pin();
        MX_TIM7_Init();
        //HAL_TIM_Base_Start_IT(&htim7);
}

void start_time_code_Gen_SMPTE(void){
        HAL_TIM_Base_Start_IT(&htim7);
}

void stop_time_code_Gen_SMPTE(void){
        HAL_TIM_Base_Stop_IT(&htim7);
}




/**
  * @brief TIM7 Initialization Function
  * @param None
  * @retval None
  */
void MX_TIM7_Init(void)// Time code generator
{

  /* USER CODE BEGIN TIM7_Init 0 */

  /* USER CODE END TIM7_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM7_Init 1 */

  /* USER CODE END TIM7_Init 1 */
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 1;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  
  if(fps_gen == 25){
        //htim7.Init.Period = 10655;//25fps
         htim7.Init.Period = 10499;//25fps OK
  }else{
        //htim7.Init.Period = 8655;//30fps
         // htim7.Init.Period = 8747;//30fps
           htim7.Init.Period = 8747;//30fps
  }
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM7_Init 2 */

  /* USER CODE END TIM7_Init 2 */


}


   
void pwr_but_read(void){
        pwr_stat = HAL_GPIO_ReadPin(PWR_STAT_PORT, PWR_STAT_PIN);
        
        if(pwr_stat == 0){
                if(pwr_but_timer > TIME_PWR_OFF_BUT)
                {
                        pwr_but_timer = 0;
                        lock_pwr_button(0);//power off
                }
        }else{
                if(pwr_but_timer > 100){
                        pwr_but_timer =  pwr_but_timer - 100;
                }else{
                        pwr_but_timer = 0;
                }
                
        }
}


uint16_t get_pwr_but_timer(void){
        return pwr_but_timer;
}

void lock_pwr_button(uint8_t lock){
        GPIO_InitTypeDef GPIO_InitStruct = {0};
        if(lock == 0){//power off
                  GPIO_InitStruct.Pin = PWR_BUT_PIN;
                  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
                  GPIO_InitStruct.Pull = GPIO_NOPULL;
                  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
                
                
                  HAL_GPIO_WritePin(PWR_BUT_PORT, PWR_BUT_PIN, GPIO_PIN_RESET);
                
                  HAL_GPIO_Init(PWR_BUT_PORT, &GPIO_InitStruct);
                
                 
        }else if(lock == 1){//lock button
                  GPIO_InitStruct.Pin = PWR_BUT_PIN;
                  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
                  GPIO_InitStruct.Pull = GPIO_NOPULL;
                  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
                
                  HAL_GPIO_WritePin(PWR_BUT_PORT, PWR_BUT_PIN, GPIO_PIN_SET);
                
                  HAL_GPIO_Init(PWR_BUT_PORT, &GPIO_InitStruct);
                
                //  HAL_GPIO_WritePin(PWR_BUT_PORT, PWR_BUT_PIN, GPIO_PIN_SET);
        }else if(lock == 10){// unlock buttom
                  GPIO_InitStruct.Pin = PWR_BUT_PIN;
                  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
                  GPIO_InitStruct.Pull = GPIO_PULLUP;
                  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
                  HAL_GPIO_Init(PWR_BUT_PORT, &GPIO_InitStruct);
        
        }
}


VS_VOID ENABLE_PWR_BUTTON()
{
        
        lock_pwr_button(10);

        is_power_on_enable = 0;
}

VS_VOID DISABLE_PWR_BUTTON()
{
        lock_pwr_button(1);
                       
        is_power_on_enable = 1;

}


void set_real_time(uint8_t tmp_fix, uint32_t tmp_val_time){ 
        if(GPS_Fixed == 1 && tmp_fix == 1){
                real_time = tmp_val_time;
                isGPSTimerCTRL = 0;
        }else if(tmp_fix == 0){
                isGPSTimerCTRL = 1;
                real_time = tmp_val_time + 60;
        }else{
                //do nothing
        }
}

void Get_MCU_time(void){
        HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN);
        HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
        real_time = 1000 * 60 * 60 * sTime.Hours + 1000 * 60 * sTime.Minutes + 1000 * sTime.Seconds + 4 * sTime.SubSeconds;
        real_time_valid = 1;
}


uint32_t get_real_time(void){
        return real_time;
}

void load_key_config(void){
                if(device_Status < Status_PowerEnable + 1){//read wireless config
                       if(was_WirConfigRead == 0 && USB_stats == HOST_CLASS){
                                was_WirConfigRead =  10;
                                Wireless_config_Update();
                                was_WirConfigRead = ReadWirKey();
                       }
                }
        }




void DMX_Special_func(void){
        //if(device_Status > Status_Idle && is_DMX_Pass == 1 && is_DMX_R_valid == 1)// for DMX ramp
        if(device_Status > Status_Idle && is_DMX_R_valid == 1)// for DMX ramp
        {
                is_DMX_R_valid = 0;
                for(int i = 0 ; i < DMX_CH_MAX; i++)
                {
                        if(DMX_Ramp_Vect[i][1] == 0)  
                        {
                                continue;
                        }
                        else
                        {
                                is_DMX_R_valid = 1;
                                if(DMX_Ramp_Vect[i][0] >= 1000 && DMX_Ramp_Vect[i][0] < 2000)
                                {
                                        if(DMX_Ramp_Vect[i][1] == 1)
                                        {
                                                buf_DMX[i] = DMX_Ramp_Vect[i][0] - 1000;
                                                DMX_Ramp_Vect[i][0] = 0;
                                                DMX_Ramp_Vect[i][1] = 0;
                                                DMX_Ramp_Vect[i][2] = 0;
                                        }
                                        else
                                        {
                                                DMX_Ramp_Vect[i][1]--;
                                        }
                                }
                                else if(DMX_Ramp_Vect[i][0] >= 2000 && DMX_Ramp_Vect[i][0] < 3000)
                                {
                                        if(DMX_Ramp_Vect[i][1] < 0)
                                        {
                                                DMX_Ramp_Vect[i][1] *= (-1);
                                        }
                                        
                                        if(DMX_Ramp_Vect[i][1] == 1)
                                        {
                                                if(DMX_Ramp_Vect[i][2] < 0)
                                                {
                                                        buf_DMX[i] = buf_DMX[i] - 1;
                                                }
                                                else
                                                {
                                                        buf_DMX[i] = buf_DMX[i] + 1;
                                                }
                                                
                                                if(buf_DMX[i] == DMX_Ramp_Vect[i][0] - 2000) // DMX value it is reached
                                                {
                                                        DMX_Ramp_Vect[i][0] = 0;
                                                        DMX_Ramp_Vect[i][1] = 0;
                                                        DMX_Ramp_Vect[i][2] = 0;
                                                }
                                                else
                                                {
                                                        DMX_Ramp_Vect[i][1]  =  DMX_Ramp_Vect[i][2];
                                                        DMX_Ramp_Vect[i][1]--;
                                                }
                                        }
                                        else
                                        {
                                                DMX_Ramp_Vect[i][1]--;
                                        }
                                }
                        }
                        
                }
                
                
        }
}

   


uint64_t getDMXtime(){
        return DMX_Timer;
}


void rst_DMX_Timer(void){
        DMX_Timer = 0;
}


void read_encoder_push(void){
       Rot_enc_push = HAL_GPIO_ReadPin(ROT_ENCODE_D_PORT, ROT_ENCODE_D_PIN); 
                if(Rot_enc_push == 1){
                        Rot_enc_push_tim = 0;
                        Rot_enc_push_V = 0;
                }else{
                        Rot_enc_push_tim++;
                }
}




void check_push_X(void){
        if(Rot_enc_push_tim > 50 && Rot_enc_push_V == 0){
                if(Time_code_Started == 0){
                      Rot_enc_push_tim = 0;
                      Rot_enc_push_V = 1;  
                      wave_player_state_set(STATE_WAVE_PLAYER_START);
                      Start_TimeCode();
                      Time_code_Started = 1;
                }else{
                       Rot_enc_push_V = 1;  
                       Rot_enc_push_tim = 0;
                       Time_code_Started = 0;
                       Stop_TimeCode();
                       WavePlayerStop();
                
                }
                
        }

}


uint32_t get_time_code_timer(void){
        return Timer_forGenerate_time_code + get_time_Code_ofset();
        }
