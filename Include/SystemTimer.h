/**************************************************************************//**
 * @file     SystemTimer.h
 * @brief    System periodic timer header file
 ******************************************************************************/

#include "Device.h"


uint64_t GetCurrentSystemTime(void);
void SysTimer(void);
uint64_t CheckTimeSpan(uint64_t timestamp);
uint64_t getDevicetime(void);
void setDevicetime(uint64_t SMPTEtimeRead);
void set_dev_time(uint32_t dev_timer_x);
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);



void init_delay_timer(uint32_t delay_ms);
void rst_delay_timer(void);
void do_delay_timer(void);
void delay_timer(void);
void Audio_Player_Start_Test(void);
void set_AP_Start_time(void);

void Button_Task(void);

void Ping_Slaves(void);
void is_fire_tst(void);
void updateTimers(void);
void init_test_timer(void);
uint32_t Get_PingTimerValue(void);
void syncMSTimerAB(MS_Message *msg);
void syncMSTimerS(MS_Message *msg);
void Button_Run(void);
void read_temp_hum(void);
void Start_button_disabled(void);
void check_USB_Con(void);
void tests_only(void);
void update_GPS_time(void);

void change_dev_time(int adjust_time);
void rst_prg_param(uint8_t exit_param);
uint8_t was_mod_Check_comp(uint8_t tmp_cur_mfp);
void initTimer4X(void);
void reset_prg_time(void);
void reset_all_programing(void);
void start_prg_upload(uint8_t del_o_prg, uint8_t all_o_ID);
void mod_prg_del(uint8_t mod_id, uint8_t del_prg);
void do_upload_mod_task(void);

uint8_t return_prg_stat(void);
void init_ping_timer(void);
uint8_t get_mod_id_u(void);
void calc_upload_progress(uint8_t del_prg);
void set_prg_stat(void);
void reset_all_other(void);
void skip_to_next_mod_up(void);
uint8_t get_is_wait_prg(void);
void init_lcd_del_chg(void);
void lcd_chg_del(void);
void rst_lcd_del_chg(void);
void start_buz(uint16_t msTime_up);
int get_arm_but_time(void);
void rst_arm_timings(void);
void set_arm_timings(void);
void update_device_time_audio(void);

void check_timeCode_IntPlayer(void);
void LTC_Pause(void);
void check_LTC_Pause(void);
uint32_t calculate_ping(void);
void GPS_Time_start(void);
void Wir_learn(void);
uint8_t get_was_canceled(void);
void rst_was_canceled_script(void);
void arm_from_key_delayed(void);
void update_TC_timer(int FSK_timeX);
void set_last_msg_with_reply_wait(void);
void send_433_wireless_msg_on_uart(void);
int get_is_time_start_start(void);
void set_is_time_start_start(void);
void syncMSTimer433X(uint32_t new_AB_time);
void Green_button(void);
void Blue_button(void);
void set_rev_display(void);
uint8_t ret_Green_Pin(void);
uint8_t ret_Blue_Pin(void);
void TC_pin_ctrl(void);
void timer_TC_task(void);
void timeUpdate(void);
void setLevel(void);

void read_encoder_pins(void);
uint16_t get_Timer_autoset(void);
void set_time_code_Gen_SMPTE_25(void);
void set_time_code_Gen_SMPTE_30(void);
void MX_TIM7_Init(void);

void start_time_code_Gen_SMPTE(void);
void stop_time_code_Gen_SMPTE(void);
void fsk_gen_Timer(void);
void run_pps_function(void);

uint16_t get_pwr_but_timer(void);
void lock_pwr_button(uint8_t lock);
void pwr_but_read(void);
void gps_timer_Update(void);
uint32_t get_real_time(void);
void gps_pps_pin(void);
void load_key_config(void);
void DMX_Special_func(void);
uint64_t getDMXtime(void);
void rst_DMX_Timer(void);
void sync_time_code(uint32_t time_to_use);
        
void read_encoder_push(void);
void check_push_X(void);
uint32_t get_time_code_timer(void);

