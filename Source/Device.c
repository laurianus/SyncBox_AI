/**************************************************************************//**
* @file     Device.c
* @brief    Core device operation functions
******************************************************************************/
#include <string.h>
#include <stdio.h>

#include "stm32f4xx_hal.h"
#include "waveplayer.h"
#include "USBHostMain.h"
#include "TimeCode.h"
#include "GlobalPositioning.h"
#include "flash_if.h"

#include "SystemTimer.h"
#include "main.h"
#include "Device.h"
#include "Display.h"
#include "options.h"
#include "Wireless.h"
#include "Buttons.h"
#include "Uart.h"

#ifndef VS_READABLE_CODE
#include "System1.h"
#include "System1Action.h"
#include "System1Data.h"
#endif

#include "simpleEventHandler.h"         		// Event Queue
#include "System1SEMLibB.h"                    	// visualSTATE essentials



//Saved_Time_prog savedDatas __ALIGNED(4) __attribute__((at(0x20000BFC)));
Saved_Time_prog savedDatas __ALIGNED(4) __attribute__((at(0x2001C000)));
Saved_Cfg savedDatasCfg;

extern uint8_t was_screen_changeM;
extern uint8_t set_AD_ID;



extern uint8_t is_DMX_Pass;
extern uint8_t is_DMX_R_valid;

uint8_t Time_code_AB = 0;
uint8_t Time_code_Started = 0;


uint32_t timer_last_msg_ping = 0;

int Time_until_start = 0;

extern uint32_t ID_MCU2;

uint8_t SB_AsMaster = 0;
uint32_t AB_master_433_was_set = 0;


uint8_t cur_channel_433Mhz = 0;
uint8_t saved_433_channel = 0;

uint8_t tmp_crc_fds = 0;

uint32_t master_433_was_setC = 0;
uint32_t master_433_was_set = 0;
uint32_t Backup_master_433_was_set = 0;

uint8_t waslastMsg_val = 0;

extern uint8_t defaultX_set3_433C[];

uint8_t was_full_check = 0;
uint8_t was_last_mod_check = 0;
uint8_t was_last_AB_check = 0;

int Ping_433_Slave_Time[MAX_NUMBER_OF_DEVICES];
int Ping_24_Slave_Time[MAX_NUMBER_OF_DEVICES];
uint16_t Miss_mod_ping[MAX_NUMBER_OF_DEVICES] = {0}; /// time from last pinged
uint8_t retry_times_on_433_xx = 0;
uint64_t time_of_LastMod_ping = 0;
uint8_t last_ping_mod_on433Mhz = 0;


uint8_t  was_started_config = 0;

uint8_t msg_433_tmpA[MAXIMUM_433_MESSAGE_SIZE] = {0};

uint8_t is_868_sync = 0;

uint8_t AB_connected = 0;
extern uint8_t is_CopyMenu;
uint16_t ping_interval = 0;

extern uint16_t ID_autoset_timer;

uint8_t LANGx = ENGLISH;



uint8_t was_here_X = 0;

uint8_t is_first_mod_msg = 100;
extern uint8_t is_need_sent;



int difference_last433_time = 0;
uint32_t last_time_here_433_X = 0;
uint16_t not_able_to433Sync = 0;

extern uint8_t is_new_433_BC_msg;
extern uint8_t is_433_NC;


int difference_last24_time = 0;
uint32_t last_time_here_24_X = 0;
//uint16_t not_able_to24Sync = 0;


uint8_t buf_DMX_DMA[DMX_CH_MAX] = {0x00};//DMA DMX buffer

uint8_t buf_DMX[DMX_CH_MAX] = {0x00};//Main DMX buffer
uint8_t buf_DMX_ARM[DMX_CH_MAX] = {0x00};//Arm DMX buffer

uint64_t seq_timers[MAX_SEQ] = {0};
uint32_t seq_timersX[MAX_SEQ] = {0};

uint8_t seq_validT[MAX_SEQ];



extern long int DMX_Ramp_Vect[DMX_CH_MAX][3];

uint32_t ping_timer_current_mod = 0;
uint32_t ping_timer_current_start = 0;

extern char wir_network_rem_name[6];

uint8_t system_wake_up = 0;

uint16_t msg_stat1_received = 0;
uint16_t msg_stat2_received = 0;
uint16_t msg_stat_req = 0;

int tmp_433_mhz_channel = -1;
uint8_t wire_433_mod_det = 1;

uint32_t disc_time_asdf = 0;

uint8_t PING_OFFSET_x = 0; // Offset ping received from controller
uint32_t real_time_ctrl = 0;

uint32_t is_pinged_433_for_TEST[6] = {0};
uint32_t is_start_pinged_433_for_TEST[6] = {0};
int is_msg_pinged_433_for_TEST[6] = {0};
int is_msg_pinged_433_for_WIR_SGN[6] = {0};

uint16_t msg_sent_on_433_was = 0;

uint8_t Can_be_Slave = 0;


uint8_t is_audio_skip = 0;

extern uint64_t last_msg_snt_timer;
uint16_t msg_lost_req_rcvd = 0;
extern uint8_t msg_number;


extern uint32_t device_time;

extern uint8_t is_manual_multiple_mods;

int device_update_timer = 0;

uint8_t tmp_adre_received = 0;

//extern int time_for_calibration;

uint8_t msg_433_tmp[MAXIMUM_433_MESSAGE_SIZE] = {0};

uint8_t last_mod_433_pinged = 0;
uint8_t last_AB_433_pinged = 0;
uint8_t last_ABNC_433_pinged = 0;
uint8_t last_modNC_433_pinged = 0;
uint8_t was_433_pinged_first = 0;
uint8_t was_433_pinged_firstX = 0;

extern uint8_t is_433Mhz_BC_wait;

uint8_t last_con_mod_pinged = 0;
uint8_t last_con_AB_pinged = 0;


uint8_t last_con_mod_pinged_sleep = 0;
uint8_t last_con_AB_pinged_sleep = 0;

extern uint32_t Seq_Firs_Time[MAX_SEQ];

extern uint8_t Power_off_mods_enabled;
//extern uint8_t Standby_enabled;
extern UART_HandleTypeDef huart6;


extern uint8_t is_433Mhz_en;
uint8_t is_433Mhz_prg_mode = 0;
uint8_t Prg_msg_433_ext[MAXIMUM_433_MESSAGE_SIZE_EXT] = {0};

uint8_t msg_433_tmpX[MAXIMUM_433_MESSAGE_SIZE] = {0};


/*
uint32_t msg_snt_timer_AAS = 0;
uint64_t last_msg_timer_sntttt = 0;
uint64_t last_msg_timer_snttttAAA = 0;

uint64_t last_msg_timer_snttttAAAB[10] = {0};
uint16_t msg_missed_test[10] = {0};
uint32_t msg_snt_timer_AASB[10] = {0};
*/


uint32_t long_433_reply = 0;

extern uint8_t rasb_433DataBuffer[256];

extern uint8_t is_UpFailed[MAX_NUMBER_OF_DEVICES];
extern char Network_Name[6];

extern uint8_t was_screen_change;
extern uint8_t was_screen_changeM;

extern uint64_t coordinatorAddressX;

uint32_t time_code_offset = 0;

uint8_t disable_ping = 0;

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim5;
extern TIM_HandleTypeDef htim6;

uint64_t refresh_rate = 0;
uint8_t ERR_STAT = 0;
uint8_t wir_to_learn = 0;

char NewNetwork_Name[4] = {0};

uint32_t GPS_SetValue = 86500000;

uint8_t Hours_tmp = 0;
uint8_t Min_tmp = 0;
uint8_t Sec_tmp = 0;


//START FIRE TIME
uint8_t isGPSFTValid = 0;
uint8_t Hours_Start = 255;
uint8_t Min_Start = 255;
uint8_t Sec_Start = 255;

extern uint8_t signal_flag;

extern uint16_t error_load_script;

extern uint8_t was_LTC_Started;
extern uint8_t script_is_end;

extern uint8_t was_finished_prg_id;

extern uint8_t was_state_up;

extern uint8_t was_LCD_SZ_Changed;

extern uint8_t was_skip_used;

uint8_t armchecks = 1;

uint8_t current_AB_For_D = MIN_NUMBER_OF_AUDIO_BOX;

uint8_t Current_Seq = 0;
extern uint8_t Current_Seq_D;

extern dev_progXXX ScriptUpLoad;
extern dev_progXXX ScriptUpLoadEXT[MAX_NEWEVENTS_EXT];

uint8_t Current_Slave_step = 0;
uint8_t Current_Chan_step = 0;

extern uint16_t tmp_pyro_ev;
extern uint16_t tmp_DMX_ev;

uint8_t current_Slave_For_D_Prg = 0;

uint8_t statusMatrixA_tmp[24] = {0};
uint8_t last_mod_stat_A = 0;

uint8_t tmp_cur_mod_prg = 0;

uint8_t tmpprg_msg[16] = {0};//for programing answer to check
uint8_t tstOK_prg = 0;
extern uint8_t is_prg_wait;// wait for module to respond- 1 respond on prg, 2 respond on finish prg

int id_to_prg = -1;
extern uint8_t is_UpFailed[MAX_NUMBER_OF_DEVICES];

extern uint8_t Function_But;
extern uint8_t  was_AP_Disp;

extern int Adjusted_time;

uint8_t Area_state = 0;
extern uint8_t was_filed_open;
uint8_t Err_found_mods = 0;

uint32_t milisecondsX = 0;
extern uint32_t ping_timer;
uint8_t Display_timer = 0;

//extern uint8_t TC_fps_setX;
extern uint8_t SMPTE_Valid_PT;


uint8_t Mods_Err[4] = {0};
uint8_t Mods_Wngs[4] = {0};

extern uint8_t isGPSTimerValid;

extern uint32_t real_time;

uint8_t is_SF_OK = 0;
uint8_t is_F1_check = 0;
uint8_t start_prg = 0;

//uint8_t Rail_Stat[MAXIM_RAILS][CHAN_STAT]= {{0}};
//extern uint16_t Mod_Stat[MAXIM_STATS];

uint16_t Sys_info[MAX_SYS_STATS] = {0};

//extern uint32_t Rem_Stat[MAXIM_STATS];


char tempTextTime[20] = {0};




extern lcd_disp_script Script_to_LCD[MAX_SCRIPT_LINES];

char tempTextTimeF[20] = {0};
char tempTextTimetoEND[20] = {0};
char tempTextTimetoPass[20] = {0};



uint8_t fire_type = 0;
uint8_t is_step_EN = 0;
uint8_t Rai_ID_stp = 0;

uint8_t Hours = 0;
uint8_t Min = 0;
uint8_t Sec = 0;

uint8_t Hours_TCo = 0;
uint8_t Min_TCo = 0;
uint8_t Sec_TCo = 0;


uint8_t was_sl_test = 0;
uint8_t is_F1_checkA = 0;

uint8_t Seq_Time_prg = 0;
uint32_t RTS_Time_prg = 0;
uint8_t prg_ch_tmp;

uint8_t S_PRGxA = 0;

uint8_t Slaves_connected = 0;
uint8_t S_PRGx = 0;
        
uint8_t crit_err = 0;
uint8_t ncrit_err = 0; 
        

extern uint8_t ID_duplicated[MAX_NUMBER_OF_DEVICES];

extern uint32_t LastPingTime[MAX_NUMBER_OF_DEVICES]; // last ping time
extern int Ping_Slave_Time[MAX_NUMBER_OF_DEVICES];
extern uint8_t Pinged_Slave[MAX_NUMBER_OF_DEVICES];



uint8_t current_BroadcastSlave = 0;





uint32_t LastPingTimeAB[MAX_NUMBER_OF_AB + 1] = {0}; // last ping time

int Ping_AB_Time[MAX_NUMBER_OF_AB + 1] = {0};
uint8_t Pinged_AB[MAX_NUMBER_OF_AB + 1] = {0};


uint16_t ERR_found = 0;

uint8_t matrixBStatusX[12] = {0};


uint8_t isFiring = 0;
uint8_t is_Pause_Script = 0; // choose if show only Pyro and/or DMX events. 0 - ALL; 1 - Pyro; 2 - DMX
// for sequence fire
uint64_t seq_timers[MAX_SEQ];
uint8_t seq_validT[MAX_SEQ];

uint16_t mark_pos = 0;

uint32_t F_NextTime = 0;
uint8_t F_NextChannel = 0;

uint8_t SafeZone_Disabled[16] = {0};//Disable SafeZones 1 - means disable


uint16_t sync_times = 0;
extern uint8_t PPS_Mp3_AB; 

extern uint8_t file_read_error;

extern uint8_t main_volume;
//uint8_t is_audio_skip = 0;
extern uint8_t is_init_done;
int time_offset = 0;


uint8_t crit_errA = 0;


extern SlaveInfo slaves[MAX_NUMBER_OF_DEVICES];

extern audioBoxA AudioBoxes[MAX_NUMBER_OF_AB + 1];


extern int is_remain_Pyro;
extern int is_remain_DMX;

uint8_t mod_ping_snt[MAX_NUMBER_OF_DEVICES] = {0};
uint8_t AB_ping_snt[MAX_NUMBER_OF_AB + 1] = {0};


uint8_t current_Slave_For_D = 0;

uint8_t Safe_ZoneZ[MAX_SZ] = {0};
uint8_t is_PyroDMX_Dis[MAX_NUMBER_OF_DEVICES] = {0};


uint32_t devicetime_MSx = 0;


uint8_t is_newDMXsoft = 0;

uint8_t is_solo_valid = 0;

uint8_t selected_Channel;


int8_t TimeZone = 0;

uint8_t is_old_firmware = 0;

Device_Data device_datas;

dev_FIFO device_FIFO;


extern uint8_t Mod_Err_Stats[MAX_NUMBER_OF_DEVICES];




#if DEBUG_X
uint8_t valid_script_pos[TimeMatrixDepth] = {0};
#endif

MS_Message currentMsg;

uint8_t is_Pause = 0;
uint8_t Network_found = 0;


Device_Data* get_device_datas()
 {
       return &device_datas;
 }

Device_Time_prog* get_device_Time_Program()
 {
       return &savedDatas.time_prog;
 }

Device_Time_prog get_device_Time_ProgramX()
 {
       return savedDatas.time_prog;
 }






 
uint8_t check_pwr(void)
{
        if(device_Status> 2)
        {
                return 1;
        }
return 0;

}
 
/******************************************************************************
//jcm-Complete
@fn void generateStatusResponseMessage(PC_Message *message)
@param[out] message Pointer to the structure that will contain the message data
@brief Generates device status response message.
@details Used in all device operation modes.
******************************************************************************/
void getStatusResponseMessage(PC_Message *message)
 {
     device_datas.power_Enable = check_pwr();
       
       
       message->address=device_datas.device_Address;
       message->dataLength=(uint8_t)0x08;
       
       message->command = (uint8_t)PC_Master_Msg_Status_Interface;
       
       message->data[0] = 0;


        //BYTE 0
       //BIT 7 JE - NOT USED
         
        //BIT 6
       //Programmed interface bit
       if(device_datas.programmed_interface)
             message->data[0] |= (uint8_t)(1 << 6);
       else
             message->data[0] &= (uint8_t)(~(1 << 6));
       
       //BIT 5
       //PWEN bit whether matrix is enabled - for key ones
       if (device_datas.power_Enable)
             message->data[0] |= (uint8_t)(1 << 5);
       else
             message->data[0] &= (uint8_t)(~(1 << 5));
       
       //BIT 4
       // IST bit
       if(device_datas.isMaster){
             message->data[0]|=(uint8_t)(1<<4);
       }else{
             message->data[0] &= (uint8_t)(~(1 << 4));
       }	
       
       
       //BIT 3 - NOT USED
       //for fireone router 
       //message->data[0]&=(uint8_t)(~(1<<3));// not used
       
       
       //BIT 2
       //CAN network is created??(can connection is ok?)
       if(device_datas.isCANConnected){ 
             message->data[0]|=(uint8_t)(1<<2);		
       }else{
             message->data[0]&=(uint8_t)(~(1<<2));
       }
       
       
       //BIT 0 and 1
       //Device Status
       if(device_Status > 1)
        {
              switch(device_Status){
                case 2:// idle 0
                    message->data[0]&=(uint8_t)(~(1<<1)); //x0
                    message->data[0]&=(uint8_t)(~(1<<0)); //y0
                    break;
                case 3://armed 1
                    message->data[0]&=(uint8_t)(~(1<<1)); //x0
                    message->data[0]|=(uint8_t)(1<<0); //y1
                    break;
                case 4://play 2
                    message->data[0]|=(uint8_t)(1<<1);  //x1
                    message->data[0]&=(uint8_t)(~(1<<0)); //y0
                    break;
                case 5://pause 3
                    message->data[0]|=(uint8_t)(1<<1);  //x1
                    message->data[0]|=(uint8_t)(1<<0); //y1	
                    break;
                    
              }
              
        }
       else
        {
              message->data[0]&=(uint8_t)(~(1<<1)); //x0
              message->data[0]&=(uint8_t)(~(1<<0)); //y0
        }
       
       
       
       
       
       //BYTE 1
       //Status byte 
        //AudioBox Internal player status        
             message->data[1] = 0x00;
        

              // internal player status: 0 - ID; 1-UD; 2-TC;
              uint8_t mp3_sts_tmp = read_mp3_status();
              mp3_sts_tmp <<= 3;
              message->data[1] = message->data[1] | mp3_sts_tmp;
              
              
              // internal player work status;
              uint8_t mp3_psts_tmp = read_mp3_Pstatus();
              mp3_psts_tmp <<= 1;
              message->data[1] = message->data[1] | mp3_psts_tmp;

       
       //BYTE 2 and 3
       // Internal Battery: Byte 2 with 4 bits and external power bytes(12 bits)
       message->data[2] = 0x00;
       
       message->data[2] = (uint8_t) device_datas.internal_battery_status;
       message->data[2] <<= 4;
       
       message->data[3] = 0x00;
       
       
       //BYTE 4
       //Wireless signal status byte
       if(IsWirelessDetected()==SUCCESS){
             message->data[4]=(uint8_t)device_datas.wireless_power_status;		
       }
       else {
             message->data[4] = (uint8_t)0x00;
       }
       
       
       //BYTE 5
       //Self test Status(5 bits) and Firmware version(3 bits)
       message->data[5] = (uint8_t)0x00; // for Modules status errors 
       // 0: Script Warning
       // 1: Script Fault 
       // 2: Hardware Warning
       // 3: Hardware Fault       
       // 4: Reserved
       // 5: Firmware Version
       // 6: Firmware Version
       // 7: Firmware Version

    
       
       
       //BYTE 6
       message->data[6] = (uint8_t)0x00;//Used for rail report
       
       // for rail errors, each 2 bits per rail - Total 1 byte (8 bits)
       // 0 - Rail not connected
       // 1 - Rail connected no igniters
       // 2 - Rail connected with igniters
       // 3 - Rail Error
       
       //for audio box
       // 0 - Audio Player init
       // 1 - Audio Player starting
       // 2 - NO USB Drive
       // 3 - No Music File
       // 4 - Error ion music file
       //10 - Audio player ready
       
       /*
       if(read_mp3_status() == 3)
       {
            message->data[6] = 11;//no usb drive
       }
       else if(file_read_error == 1)
       {
            message->data[6] = 12;//no valid music file
       }
       else
       {
            message->data[6] = 10;// Audio player ready
       }
*/
       
       //BYTE 7
       //Report module
       message->data[7] = 0x00; 
       
       //3 BITs for module kind- BIT 0, 1 and 2 - 8 Values
       // - 0: FTH-48Fx
       // - 1: FTQ-16x64 64 channel 4 rails
       // - 2: FTQ-4x16
       // - 3: SyncBox Audiobox
       
       //Options: 5 BITs for options - 3 options
       //- DMX BIT b3
       //- 2W BIT b4
       //- TC BIT b5
       //- GPS b6
       //- Reserverd b7
       
       message->data[7] = 4; //Audiobox
      
       
}







dev_FIFO* getDevFIFO()
 {
       return &device_FIFO;
 }




VS_VOID sendSMStatusInterface() 
 {
         


       MS_Message outputF;
       PC_Message tmp;
       //int flag = 0;
       memset(&outputF, 0x00, sizeof(MS_Message));
       memset(&tmp, 0x00, sizeof(PC_Message));
       
       getStatusResponseMessage(&tmp);
       
       outputF.address = AUDIO_BOX_ADR_SET + MASTER_SLAVE_NewStatus;
       outputF.dataLength = 24;
       for (int i = 0; i < 8; i++)
        {
              outputF.data[i] = tmp.data[i];
              
        }
        
        outputF.data[8] = device_datas.device_Address + 200;
       
       Wireless_Send(&outputF);
     
 }




void set_Master(void)
{
        device_datas.isMaster = 1;
}


void set_Last_ping_time(uint8_t adrr_xxx)
{
        if(adrr_xxx < MAX_NUMBER_OF_DEVICES && adrr_xxx > 0)
        {
                LastPingTime[adrr_xxx] = GetCurrentSystemTime()/PING_433_GRANULARITY;
        }
        else if(adrr_xxx > AUDIO_BOX_OFFSET && adrr_xxx < AUDIO_BOX_OFFSET + 10)
        {
                LastPingTimeAB[adrr_xxx - AUDIO_BOX_OFFSET] = GetCurrentSystemTime()/PING_433_GRANULARITY;
        }
        else if(adrr_xxx == 255)
        {
                for (int i = 1; i< MAX_NUMBER_OF_DEVICES; i ++)
                {       
                        if(slaves[i].isConnected == 1)
                        {
                                LastPingTime[i] = GetCurrentSystemTime()/PING_433_GRANULARITY;
                        }
                }
                
                for (int i = 1; i < MAX_NUMBER_OF_AB + 1; i ++)
                {       
                        if(AudioBoxes[i].isConnected == 1)
                        {
                                LastPingTime[i] = GetCurrentSystemTime()/PING_433_GRANULARITY;
                        }
                }
        }
        
}

uint8_t get_mod_stat_sleep(uint8_t mot_modss){

                return savedDatasCfg.sleep_mods[mot_modss];
}



void getMS_433_Event(void)
{
if(Can_be_Slave == 0) return;

        /* Guard: validate minimum message length before accessing fixed indices.
         * Message accesses indices up to MSG_LOST_POS (61), so need at least 62 bytes. */
        extern uint8_t Bytes_433_rcvd;
        if(Bytes_433_rcvd < 3) return;  /* Need at least header + cmd + address */

        if (device_datas.isMaster) {

                 device_datas.isWireless433Connected = 1;
                 if(rasb_433DataBuffer[0] == START_CHAR_433MHZ_AB){// received sync from Audiobox

                                if(Bytes_433_rcvd < 8) return;  /* Need indices 0-7 for sync message */
                                if((rasb_433DataBuffer[1] == M433_PING_AB_SYNC) && (rasb_433DataBuffer[2] == M433_BROADCAST)){

                                        uint32_t new_device_timeX = (rasb_433DataBuffer[7] << 24) | (rasb_433DataBuffer[6] << 16) | (rasb_433DataBuffer[5] << 8) | (rasb_433DataBuffer[4] << 0);
                                        syncMSTimer433X(new_device_timeX);
                                }

                }
                else if(rasb_433DataBuffer[0] == START_CHAR_433MHZ_SM){
                        tmp_adre_received = rasb_433DataBuffer[2];

                        /* Bounds check: reject invalid device addresses from radio data */
                        if(rasb_433DataBuffer[1] == M433_PING_AB_REPLY){
                                /* AudioBox addresses must be in range [AUDIO_BOX_OFFSET+1 .. AUDIO_BOX_OFFSET+MAX_NUMBER_OF_AB] */
                                if(tmp_adre_received <= AUDIO_BOX_OFFSET || tmp_adre_received >= (AUDIO_BOX_OFFSET + MAX_NUMBER_OF_AB + 1)){
                                        return;
                                }
                        } else {
                                /* Module addresses must be in range [1 .. MAX_NUMBER_OF_DEVICES-1] */
                                if(tmp_adre_received == 0 || tmp_adre_received >= MAX_NUMBER_OF_DEVICES){
                                        return;
                                }
                        }

                        if(rasb_433DataBuffer[1] == M433_PING_1_REPLY){ //received status 1 of a module

                                        /* Guard: PING_1_REPLY accesses up to index 61 (MSG_LOST_POS) */
                                        if(Bytes_433_rcvd < (MSG_LOST_POS + 1)) return;

                                        //set_Last_ping_433_time(tmp_adre_received);//set ping time
                                        //update_ping_times();
                                       // Set_Ping_TimeX_received(tmp_adre_received);



                                        current_BroadcastSlave =  tmp_adre_received;
                                        setSlavePingedX(tmp_adre_received);
                                        Set_Ping_TimeX(tmp_adre_received);
                                
                                        slaves[tmp_adre_received].missedCount = rasb_433DataBuffer[61];
                                        slaves[tmp_adre_received].isConnected = 1;
                                        slaves[tmp_adre_received].is433WirelessConnected = 1;
                                
                                        Pinged_Slave[tmp_adre_received] = 1;
                                
                                         for (int i = 0; i < 8; i++)
                                         {
                                                        slaves[tmp_adre_received].interfaceStatus[i] = rasb_433DataBuffer[i + 3];

                                                         if(slaves[tmp_adre_received].statusMatrixA[i] != rasb_433DataBuffer[i + 8 + 3])
                                                         {
                                                                 slaves[tmp_adre_received].matrixAStatusX = 0;                                                                                                                           
                                                         }
                                                         if(slaves[tmp_adre_received].statusMatrixA[i+8] != rasb_433DataBuffer[i + 16 + 3])
                                                         {
                                                                 slaves[tmp_adre_received].matrixAStatusX = 0;                                                                                                                           
                                                         }
                                                         
                                                         slaves[tmp_adre_received].statusMatrixA[i] = rasb_433DataBuffer[i + 8 + 3];
                                                         slaves[tmp_adre_received].statusMatrixA[i+8] = rasb_433DataBuffer[i + 16 + 3];
                                                         
                                                         slaves[tmp_adre_received].statusMatrixB[i] = rasb_433DataBuffer[i + 24 + 3];
                                                         slaves[tmp_adre_received].statusMatrixP[i] = rasb_433DataBuffer[i + 32 + 3];

                                         }    
                                         
                                         slaves[tmp_adre_received].msg_lost = rasb_433DataBuffer[MSG_LOST_POS];
                                         slaves[tmp_adre_received].signal_433SM = rasb_433DataBuffer[60];
                                         


                                 if(device_Status == Status_PowerEnable)
                                 {
                                         check_arm_st();
                                 }
                                 
                                 mod_ping_snt[tmp_adre_received] = 0;
                                 getSlaves_Errors(tmp_adre_received);
                                 
                                  if(get_mod_kind(tmp_adre_received, 0) == 20 || get_mod_kind(tmp_adre_received, 0) == 21){
                                        was_screen_change = 5;
                                 }
                                  
                                 if((tmp_adre_received) == 99){
                                        was_AP_Disp = 0;
                                 }       

                                 
                                 //SLEEPING PART
                                 if(rasb_433DataBuffer[50] < 2){
                                        savedDatasCfg.sleep_mods[tmp_adre_received] = rasb_433DataBuffer[50];
                                 }else if(rasb_433DataBuffer[50] == 100){// module is awaking
                                        savedDatasCfg.sleep_mods[tmp_adre_received] = 0;
                                        was_screen_changeM = 6;
                                 }else if(rasb_433DataBuffer[50] == 0xAA){// module is prepared for sleep
                                        send_uCast_433Mhz_msg(tmp_adre_received, 2);
                                        savedDatasCfg.sleep_mods[tmp_adre_received] = 1;
                                 }
                                 else{
                                 
                                 }
                                 
                                 /*
                                 if(get_sleep_stateX() == 11){
                                        if(get_mod_sleep() == 0){
                                                savedDatasCfg.is_sleep_stat = 0;
                                                save_config(0);
                                                was_screen_changeM = 5;
                                        }
                                 }
                                 if(get_mod_sleep() > 0){
                                        was_screen_changeM = 6;
                                 }
                                         
                                 if(get_sleep_state() == 1 || get_sleep_stateX() == 1 || get_sleep_stateX() == 11){//send a message to force fast sleep or awake a module in Sleeping stage
                                        send_uCast_433Mhz_msg(tmp_adre_received, 2);
                                        was_screen_changeM = 6;
                                 }
                                 */
                       }
                       else if(rasb_433DataBuffer[1] == M433_PING_2_REPLY){ //received status 2 of a module
                               
                               
                               if(tmp_adre_received == TEST_MOD_ID){
                                                msg_stat2_received++;
                                        }
                               
                                        slaves[tmp_adre_received].missedCount = 0x00;
                                        slaves[tmp_adre_received].isConnected = 1;
                                        Pinged_Slave[tmp_adre_received] = 1;
                               
//                                        set_Last_ping_433_time(tmp_adre_received);//set ping time
                                 
                                        current_BroadcastSlave =  tmp_adre_received;  
                                        setSlavePingedX(tmp_adre_received);
                                        Set_Ping_TimeX(tmp_adre_received);
                               
                                         for (int i = 0; i < 8; i++)
                                         {
                                                 slaves[tmp_adre_received].statusMatrixB[i] = rasb_433DataBuffer[i + 0 + 3];
                                                 slaves[tmp_adre_received].statusMatrixP[i] = rasb_433DataBuffer[i + 8 + 3];
                                         }
                                 }
                       
                       else if(rasb_433DataBuffer[1] == M433_PING_AB_REPLY){ //received status of an audiobox

                                        AudioBoxes[tmp_adre_received - AUDIO_BOX_OFFSET].isConnected = 1;
                                        AudioBoxes[tmp_adre_received - AUDIO_BOX_OFFSET].isWir433_Connected = 1;
                               
//                                        set_Last_ping_433_time(tmp_adre_received);//set ping time 
                               
                                   for (int i = 0; i < 8; i++)
                                         {
                                                        AudioBoxes[tmp_adre_received - AUDIO_BOX_OFFSET].interfaceStatus[i] = rasb_433DataBuffer[i + 3];

                                         }
                                        get_ab_con();
                               
                                        AudioBoxes[tmp_adre_received - AUDIO_BOX_OFFSET].msg_lost = rasb_433DataBuffer[MSG_LOST_POS];
                                        

                                 }
                       
                    else if (rasb_433DataBuffer[1] == M433_PING_MIR_PRG) {

                                set_Last_ping_time(255);//set ping time on all connected mods
                                
                                is_prg_wait = 0;

                        }
                }
                
        }else{
                 if (device_datas.join_status == 0) 
                        {
                                
                                if(rasb_433DataBuffer[0] == START_CHAR_433MHZ_MS){

                                        if((rasb_433DataBuffer[1] == M433_BACKUP_MASTER_REPLY || rasb_433DataBuffer[22] == 1) && master_433_was_set == 0){ 
                                                return;
                                        }

                                        
                                        master_433_was_setC = (((rasb_433DataBuffer[28] << 0) & 0xFF) | ((rasb_433DataBuffer[27] << 8) & 0xFF00) | ((rasb_433DataBuffer[26] << 16) & 0xFF0000) | ((rasb_433DataBuffer[25] << 24) & 0xFF000000));

                                        if(master_433_was_set == 0){
                                                master_433_was_set = master_433_was_setC;
                                                savedDatasCfg.Mast_ID_433Mhz = master_433_was_set;
                                        
                                        }
                                        else{ 
                                                if(rasb_433DataBuffer[22] == 1){//if messsage is from backup controller
                                                        if(Backup_master_433_was_set != master_433_was_setC){// not the correct backup controller
                                                                return;
                                                        }
                                                }else{
                                                        if(master_433_was_set != master_433_was_setC){
                                                                return;
                                                        }
                                                }
                                        }
                                
                                
                                        
                                       if(rasb_433DataBuffer[1] == M433_BACKUP_MASTER_REPLY){
                                                if(rasb_433DataBuffer[8] == 1 && rasb_433DataBuffer[7] == 200){
                                                        Backup_master_433_was_set = (((rasb_433DataBuffer[12] << 0) & 0xFF) | ((rasb_433DataBuffer[11] << 8) & 0xFF00) | ((rasb_433DataBuffer[10] << 16) & 0xFF0000) | ((rasb_433DataBuffer[9] << 24) & 0xFF000000));
                                                        savedDatasCfg.BMast_ID_433Mhz = Backup_master_433_was_set;
                                                 }
                                                return;
                                        }
                                        else if(rasb_433DataBuffer[1] == M433_SETTINGS && rasb_433DataBuffer[2] == M433_BROADCAST && rasb_433DataBuffer[23] != waslastMsg_val){ //set custom 433Mhz wireless
                                                waslastMsg_val = rasb_433DataBuffer[23];
                                                set_custom_433_wireless();
                                                Change_wireless_settings();
                                        
                                                save_new_433_config(1);
                                                config_PAN_EPAN_new();
                                                
                                                rst_433Rx_buffer();
                                                
                                                //send_noise_msg();     
                                        
                                        
                                        }                                              
                                        
                                               
                                        device_datas.join_status = 1;//make module as slave
                                        device_datas.isWireless433Connected = 1;
                                        device_datas.isMaster = 0;
                                        is_solo_valid = 0;
                                        SEQ_AddEvent(ev_NetworkJoin);
                                        
                                        // update_controller_ping(rasb_433DataBuffer);

                                         if(rasb_433DataBuffer[2] == 200 + device_datas.device_Address){
                                                send_433_mod_ping_reply();
                                                is_new_433_BC_msg = 0;
                                                is_433_NC = 2;
                                        }
                                        //send data back to controller with status
                                        
                                        if(rasb_433DataBuffer[2] == M433_BROADCAST){
                                                is_new_433_BC_msg = 1;
                                                is_433_NC++;
                                                
                                                 if(rasb_433DataBuffer[2] == 1){
                                                        is_433_NC = 1;
                                                }
                                        }
                                        
                                }else{
                                        // wait 2 times to see if only this module does not get signal
                                        // if fails after 3 times just send a message to get a reply
                                }
                                
                        }else{ 
                                        if(rasb_433DataBuffer[0] == START_CHAR_433MHZ_AB){// received sync from Audiobox
                                                if((rasb_433DataBuffer[1] == M433_PING_AB_SYNC) && (rasb_433DataBuffer[2] == M433_BROADCAST)){
                                                        /*                                        
                                                                device_update_timer = (rasb_433DataBuffer[7] << 24) | (rasb_433DataBuffer[6] << 16) | (rasb_433DataBuffer[5] << 8) | (rasb_433DataBuffer[4] << 0);
                                                                if(rasb_433DataBuffer[3] == 10){//negative
                                                                        update_dev_time_sync(device_update_timer);
                                                                        
                                                                }
                                                                else {
                                                                        //do nothing
                                                                }
                                                        */
                                                
                                                }
                                        }
                                        else if(rasb_433DataBuffer[0] == START_CHAR_433MHZ_MS){
                                                
                                                
                                                if((rasb_433DataBuffer[1] == M433_BACKUP_MASTER_REPLY || rasb_433DataBuffer[22] == 1) && master_433_was_set == 0){ 
                                                        return;
                                                }

                                                master_433_was_setC = (((rasb_433DataBuffer[28] << 0) & 0xFF) | ((rasb_433DataBuffer[27] << 8) & 0xFF00) | ((rasb_433DataBuffer[26] << 16) & 0xFF0000) | ((rasb_433DataBuffer[25] << 24) & 0xFF000000));
                                                
                                                if(master_433_was_set == 0){
                                                        master_433_was_set = master_433_was_setC;
                                                        savedDatasCfg.Mast_ID_433Mhz = master_433_was_set;
                                                }else{
                                                        if(rasb_433DataBuffer[22] == 1){//if messsage is from backup controller
                                                                        if(Backup_master_433_was_set != master_433_was_setC){// not the correct backup controller
                                                                                return;
                                                                        }
                                                         }else{
                                                                        if(master_433_was_set != master_433_was_setC){
                                                                                return;
                                                                        }
                                                        }
                                                }

                                                
                                                
                                                if(rasb_433DataBuffer[1] == M433_BACKUP_MASTER_REPLY){
                                                                if(rasb_433DataBuffer[8] == 1 && rasb_433DataBuffer[7] == 200){
                                                                        Backup_master_433_was_set = (((rasb_433DataBuffer[12] << 0) & 0xFF) | ((rasb_433DataBuffer[11] << 8) & 0xFF00) | ((rasb_433DataBuffer[10] << 16) & 0xFF0000) | ((rasb_433DataBuffer[9] << 24) & 0xFF000000));
                                                                        savedDatasCfg.BMast_ID_433Mhz = Backup_master_433_was_set;
                                                                 }
                                                                return;
                                                        }
                                                else if(rasb_433DataBuffer[1] == M433_SETTINGS && rasb_433DataBuffer[2] == M433_BROADCAST && rasb_433DataBuffer[23] != waslastMsg_val){ //set custom 433Mhz wireless
                                                                waslastMsg_val = rasb_433DataBuffer[23];
                                                                set_custom_433_wireless();
                                                                Change_wireless_settings();
                                                        
                                                                save_new_433_config(1);
                                                                config_PAN_EPAN_new();
                                                                
                                                                rst_433Rx_buffer();
                                                        
                                                                //send_noise_msg();     
                                                        }                                           
                                                
                                                device_datas.isWireless433Connected = 1;

                                                if(rasb_433DataBuffer[1] == M433_ARM_SYNC_FIRE){ //arming the system
                                                                 timer_last_msg_ping = GetCurrentSystemTime();  
                                                                 SEQ_AddEvent(ev_Arm);
                                                                 if(rasb_433DataBuffer[2] == 200 + device_datas.device_Address){
                                                                        send_433_mod_ping_reply();
                                                                 }
                                                                 
                                                }
                                                else if(rasb_433DataBuffer[1] == M433_BCK_MASTER && rasb_433DataBuffer[2] == M433_BROADCAST){ //set custom 433Mhz wireless
                                                         if(rasb_433DataBuffer[8] == 1){
                                                                Backup_master_433_was_set = (((rasb_433DataBuffer[12] << 0) & 0xFF) | ((rasb_433DataBuffer[11] << 8) & 0xFF00) | ((rasb_433DataBuffer[10] << 16) & 0xFF0000) | ((rasb_433DataBuffer[9] << 24) & 0xFF000000));
                                                                savedDatasCfg.BMast_ID_433Mhz = Backup_master_433_was_set;
                                                         }
                                                }
                                                else if(rasb_433DataBuffer[1] == M433_AB_MASTER && rasb_433DataBuffer[2] == M433_BROADCAST){ //set custom 433Mhz wireless
                                                        
                                                        AB_master_433_was_set = (((rasb_433DataBuffer[6] << 0) & 0xFF) | ((rasb_433DataBuffer[7] << 8) & 0xFF00) | ((rasb_433DataBuffer[8] << 16) & 0xFF0000) | ((rasb_433DataBuffer[9] << 24) & 0xFF000000));
                                                        
                                                         if(rasb_433DataBuffer[3] == 0xAA && rasb_433DataBuffer[4] == device_datas.device_Address && AB_master_433_was_set == ID_MCU2){
                                                                SB_AsMaster = 1;
                                                                send_433_mod_ping_reply();
                                                         }else{
                                                                SB_AsMaster = 0;
                                                         }
                                                }
                                                else if(rasb_433DataBuffer[1] == M433_AB_TIMECODE && rasb_433DataBuffer[2] == M433_BROADCAST){ //Start Time code generate in TEST
                                                        
                                                         Time_code_AB = rasb_433DataBuffer[4];
                                                        
                                                        if(Time_code_Started == 1){

                                                                        Time_code_Started = 0;
                                                                        Stop_TimeCode();
                                                                        WavePlayerStop();
                                                                
                                                                        time_code_offset =  ((rasb_433DataBuffer[8] << 16) & 0x00FF0000) | ((rasb_433DataBuffer[7] << 8) & 0x0000FF00) | ((rasb_433DataBuffer[6] << 0) & 0x000000FF);
                                                                
                                                        }else{
                                                        
                                                                if(Time_code_AB == 1 || Time_code_AB == 10){
                                                                         wave_player_state_set(STATE_WAVE_PLAYER_START);
                                                                         Start_TimeCode();
                                                                         Time_code_Started = 1;
                                                                        
                                                                        
                                                                        time_code_offset =  ((rasb_433DataBuffer[8] << 16) & 0x00FF0000) | ((rasb_433DataBuffer[7] << 8) & 0x0000FF00) | ((rasb_433DataBuffer[6] << 0) & 0x000000FF);
                                                                        
                                                                }else{
                                                                        Time_code_Started = 0;
                                                                        Stop_TimeCode();
                                                                        WavePlayerStop();
                                                                        
                                                                        time_code_offset =  ((rasb_433DataBuffer[8] << 16) & 0x00FF0000) | ((rasb_433DataBuffer[7] << 8) & 0x0000FF00) | ((rasb_433DataBuffer[6] << 0) & 0x000000FF);
                                                                
                                                                }
                                                        }
                                                        
                                                }
                                                else if(rasb_433DataBuffer[1] == M433_SETTINGS && rasb_433DataBuffer[2] == M433_BROADCAST){ //set custom 433Mhz wireless
                                                              //  set_custom_433_wireless();
                                                              //  save_new_433_config(1);
                                                        
                                                                waslastMsg_val = rasb_433DataBuffer[23];
                                                                set_custom_433_wireless();
                                                                Change_wireless_settings();
                                                        
                                                                save_new_433_config(1);
                                                                config_PAN_EPAN_new();
                                                        
                                                                rst_433Rx_buffer();

                                                }
                                                else if(rasb_433DataBuffer[1] == M433_STOP_PING_SYNC_FIRE){ //Stop if play and reply to ping is to ID
                                                        timer_last_msg_ping = GetCurrentSystemTime();  
                                                        if(device_Status == Status_Pause){
                                                                SEQ_AddEvent(ev_Stop);
                                                        }
                                                        else if(device_Status == Status_PowerEnable || device_Status == Status_Play){
                                                                SEQ_AddEvent(ev_MS_ToTEST);
                                                        }
                                                       
                                                        real_time_ctrl =  ((rasb_433DataBuffer[18] << 24) & 0xFF000000) | ((rasb_433DataBuffer[17] << 16) & 0x00FF0000) | ((rasb_433DataBuffer[16] << 8) & 0x0000FF00) | ((rasb_433DataBuffer[15] << 0) & 0x000000FF);
                                                        if(real_time_ctrl > 0){
                                                                set_real_time(0, real_time_ctrl);
                                                        }
                                                        
                                                        //update_controller_ping(rasb_433DataBuffer); safety zones no need 
                                                        /*
                                                        if(rasb_433DataBuffer[2] > 100){
                                                                is_433_NC++;
                                                        
                                                        }


                */                                        if(rasb_433DataBuffer[2] == 200 + device_datas.device_Address){
                                                                        send_433_mod_ping_reply();
                                                                        is_new_433_BC_msg = 0;
                                                                        is_433_NC = 2;
                                                                       // update_controller_ping(rasb_433DataBuffer);
                                                                }else if(rasb_433DataBuffer[2] == M433_BROADCAST){
                                                                        is_new_433_BC_msg = 1;
                                                                        is_433_NC++;
                                                                        
                                                                        if(rasb_433DataBuffer[2] == 1){
                                                                                is_433_NC = 1;
                                                                        }
                                                                }
                                                      
                                                        if(device_Status <= Status_PowerEnable){
                                                                if((rasb_433DataBuffer[19] >> 7) & 0x01){//power off
                                                                                pwr_off_mod();
                                                                }
                                                        }
                                                                            
                                                }
                                                else if(rasb_433DataBuffer[1] == M433_SEQ_FIRE){ //sequence fire
                                                        /*
                                                        if(device_Status > Status_Idle && (rasb_433DataBuffer[3] == 0xAA || rasb_433DataBuffer[3] == 0x55))
                                                                {
                                                                        set_433_seq_V(rasb_433DataBuffer);
                                                                }
                                                        */
                                                }
                                                else if(rasb_433DataBuffer[1] == M433_PLAY_SYNC_FIRE){ //Start and sync the show
                                                        timer_last_msg_ping = GetCurrentSystemTime();  
                                                        
                                                        if(device_Status == Status_PowerEnable || device_Status == Status_Pause){
                                                                SEQ_AddEvent(ev_Start);
                                                                 
                                                        }
                                                        
                                                        
                                                         if(is_init_done == 10)  
                                                         {
                                                                if (is_CopyMenu == 1)
                                                                {
                                                                      is_CopyMenu = 10;  
                                                                }
                                                                if(device_Status == Status_Play && PPS_Mp3_AB == 1)
                                                                {
                                                                       if(device_datas.device_Address == 1){
                                                                               
                                                                               uint32_t time_mil = 0;
                                                   
                                                                               devicetime_MSx = ((rasb_433DataBuffer[9] & 0xF0) << 20) | (rasb_433DataBuffer[5] << 16) | (rasb_433DataBuffer[4] << 8) | (rasb_433DataBuffer[3] << 0);
                                                                               devicetime_MSx = devicetime_MSx + 45;
                                                                                                 
                                                                               wave_player_current_miliseconds_get(&time_mil);

                                                                               time_offset = time_mil - devicetime_MSx;
                                                                               
                                                                               if(time_offset < -35 || time_offset > 35){
                                                                                 send_sync_433_msg();
                                                                               }
                                                                               
                                                                               is_868_sync = 3;
                                                                        }
                                                                }else{
                                                                        devicetime_MSx = ((rasb_433DataBuffer[9] & 0xF0) << 20) | (rasb_433DataBuffer[5] << 16) | (rasb_433DataBuffer[4] << 8) | (rasb_433DataBuffer[3] << 0);
                                                                        //start audiobox synced
                                                                        Start_music_Sync();
                                                                        set_dev_time(devicetime_MSx);
                                                                        SEQ_AddEvent(ev_SyncMsg);//TOADD 
                                                                }
                                                         }
                                                }
                                                else if(rasb_433DataBuffer[1] == M433_PAUSE_SYNC_FIRE){ //pause the system
                                                        SEQ_AddEvent(ev_Pause);
                                                        timer_last_msg_ping = GetCurrentSystemTime();  
                                                        

                                                       // update_controller_sync(rasb_433DataBuffer);
                                                }
                                                 if(rasb_433DataBuffer[1] == M433_AB_CMDS){ //Audiobox copmmands
                                                        if(device_Status == Status_Idle && (rasb_433DataBuffer[2] == device_datas.device_Address) && rasb_433DataBuffer[3] == AB_CMD)
                                                                {
                                                                              uint8_t tmp_mp3_cmd = rasb_433DataBuffer[8];
                                                                              switch(tmp_mp3_cmd)
                                                                               {
                                                                                 case 0: //Disbale Player
                                                                                     //Mp3Disfunc();//Not available
                                                                                     //sendSMStatusInterface();
                                                                                     break;
                                                                                 case 1://Stop
                                                                                     Audio_Stop();
                                                                                     init_delay_timer(100);
                                                                                     break;
                                                                                 case 2: //Enable player
                                                                                     //Mp3Enfunc(); //Not available
                                                                                     break;
                                                                                 case 3: //Play//Pause
                                                                                      Audio_Play_Pause_Idle();
                                                                                      init_delay_timer(100);
                                                                                     //UART_MP3_TXPlay();
                                                                                 break;
                                                                               }
                                                                }
                                                }
                        }
                }
        }
}




void getMSEvent(void)
 {
       
      
       
       if (device_FIFO.rdPtr == device_FIFO.wrPtr) return;
       
       MS_Message* tmpMSG = &(device_FIFO.buffer[device_FIFO.rdPtr]);
       memcpy(&currentMsg, tmpMSG, sizeof(MS_Message));
       selected_Channel = device_FIFO.revChannel[device_FIFO.rdPtr];
       device_FIFO.revChannel[device_FIFO.rdPtr] = Channel_None;
         
//       last_time_wasHere_XXX = GetCurrentSystemTime();
         
//       lastMessage_adr =  tmpMSG->address;
       
         
         
       if (device_datas.isMaster) {

               
             
       }
       else 
        {
             if (device_datas.join_status) 
              {
                  is_Pause = 0;
                
                   Network_found = 1;
                      
                   if (tmpMSG->address == MASTER_SLAVE_Stop) {
                         SEQ_AddEvent(ev_Stop); //TOADD
                        //   wave_player_stop();
                   }
                   

                   else if (tmpMSG->address == MASTER_SLAVE_Start) {
                         SEQ_AddEvent(ev_Start); //TOADD
                           
                   }
                   
                   else if (tmpMSG->address == MASTER_SLAVE_Pause) {
                           SEQ_AddEvent(ev_Pause); //TOADD
                   }
                   
                   else if (tmpMSG->address == MASTER_SLAVE_Error) {
                         
                   }
                   
                   else if (tmpMSG->address == MASTER_SLAVE_Seq) {

                   }
                   
                   else if (tmpMSG->address == MASTER_SLAVE_Sync) {
                           
                             if (tmpMSG->dataLength == 16)
                                {
                                        updateDisable(tmpMSG);//update disables
                                }
                         
                           ping_interval = 100 * tmpMSG->data[11];
                                         if(ping_interval < 1000){
                                                ping_interval = DEVICE_SYNC_PERIOD;
                                         }

                           
                         uint32_t time_mil = 0;
                           
                         devicetime_MSx = ((tmpMSG->data[6] & 0xF0) << 20) | (tmpMSG->data[2] << 16) | (tmpMSG->data[1] << 8) | (tmpMSG->data[0] << 0);
                         devicetime_MSx = devicetime_MSx + 35;
                                         
                         wave_player_current_miliseconds_get(&time_mil);

                         time_offset = time_mil - devicetime_MSx;
                           
                         
                                                     
                         if(is_init_done == 10)  
                         {
                                if (is_CopyMenu == 1)
                                {
                                      is_CopyMenu = 10;  
                                }
                                
                             
                                if(device_Status == Status_Play && PPS_Mp3_AB == 1)
                                {
                                                difference_last24_time = getDevicetime() - last_time_here_24_X;
                                                last_time_here_24_X = getDevicetime();
                                         
                                                 //if(difference_last24_time > (ping_interval - 50) && difference_last24_time < (ping_interval + 50)) {
                                                if(difference_last24_time > (ping_interval - 35) && difference_last24_time < (ping_interval + 35) 
                                                        && (time_offset < -35 || time_offset > 35)){
                                                                if(is_868_sync == 0){
                                                                                if(device_datas.device_Address == 1){
                                                                                        send_sync_msg(time_offset);
                                                                                }
                                                                                sync_times++;
                                                                }else{
                                                                        is_868_sync--;
                                                                }
                                                }else{
                                                        //not_able_to24Sync++;
                                                }
                                }else{
                                                        Start_music_Sync();
                                                        set_dev_time(devicetime_MSx);
                                                        SEQ_AddEvent(ev_SyncMsg);//TOADD 
                                                        //start audiobox synced
                                }
                                
                             
                                

                         }
                           
                   }
                   
                   else if (tmpMSG->address < MASTER_SLAVE_Ping + MAX_NUMBER_OF_DEVICES) {
                           
                           if (tmpMSG->dataLength == 16)
                                {
                                        updateDisable(tmpMSG);//update disables
                                }
                           
                           if(tmpMSG->dataLength != 16) return;
                                        is_old_firmware = 1;
                                        
                                        switch(tmpMSG->data[0]){
                                        case PING_X://0 - ping a module for interface status and matrix status; Also can contain pyro and safety zones control
                                                
                                                if(tmpMSG->data[15] != 0)
                                                {
                                                        if(tmpMSG->data[15] == Status_PowerEnable && device_Status == Status_Idle){
                                                                SEQ_AddEvent(ev_Arm);
                                                        }
                                                }
                                                sendSMStatusInterface();  
                                                //updateDisable(tmpMSG);//update disables
                                                break;  
                                        case AB_CMD://AB COmmands
                                                if ((tmpMSG->address % 100) == device_datas.device_Address) {
                                                              uint8_t tmp_mp3_cmd = tmpMSG->data[5];
                                                              switch(tmp_mp3_cmd)
                                                               {
                                                                 case 0: //Disbale Player
                                                                     //Mp3Disfunc();//Not available
                                                                     sendSMStatusInterface();
                                                                     break;
                                                                 case 1://Stop
                                                                     Audio_Stop();
                                                                     init_delay_timer(100);
                                                                     break;
                                                                 case 2: //Enable player
                                                                     //Mp3Enfunc(); //Not available
                                                                     break;
                                                                 case 3: //Play//Pause
                                                                      Audio_Play_Pause_Idle();
                                                                      init_delay_timer(100);
                                                                     //UART_MP3_TXPlay();
                                                                 break;
                                                                 case 100: //Ask status 
                                                                     is_need_sent = 0;
                                                                     sendSMStatusInterface();
                                                                 break;
                                                               }
                                                }
                                                break;  
                                        }
                         
                   } 
                   
                   else if (tmpMSG->address == MASTER_SLAVE_PowerEnable) {
                         
                         if (tmpMSG->data[0] == 0) SEQ_AddEvent(ev_Disarm);
                         else if (tmpMSG->data[0] == 1) SEQ_AddEvent(ev_Arm);

                   }

                   else if (tmpMSG->address == MASTER_SLAVE_AudioBOXadr) {
                         
                         saveAB_adr(tmpMSG);
                   }
                   
                   else if (MASTER_SLAVE_FinishProgramming <= tmpMSG->address
                            && tmpMSG->address < MASTER_SLAVE_FinishProgramming + MAX_NUMBER_OF_DEVICES) {
                                  
                                  
                                  
                                  if ((tmpMSG->address % 100) == device_datas.device_Address) {
                                        if(1)
                                         {
                                               if (tmpMSG->data[0] == 0xAA)
                                                {
                                                      
                                                      uint8_t tmp_mp3_cmd = tmpMSG->data[5];
                                                      switch(tmp_mp3_cmd)
                                                       {
                                                         case 0: //Disbale Player
                                                             //Mp3Disfunc();//Not available
                                                             sendSMStatusInterface();
                                                             break;
                                                         case 1://Stop
                                                             Audio_Stop();
                                                             init_delay_timer(100);
                                                             break;
                                                         case 2: //Enable player
                                                             //Mp3Enfunc(); //Not available
                                                             break;
                                                         case 3: //Play//Pause
                                                              Audio_Play_Pause_Idle();
                                                              init_delay_timer(100);
                                                             //UART_MP3_TXPlay();
                                                         break;
                                                       }
                                                      
                                                }
                                               
                                         }
                                 }
                         }
                   
                   
                   else if (tmpMSG->address == MASTER_SLAVE_PingBC) {
                           
                           
                           if(tmpMSG->dataLength == 19 && tmpMSG->data[0] == MASTER_SLAVE_WIRELESS_SET){//wireless settings
                                        if(tmpMSG->data[1] != waslastMsg_val){
                                                waslastMsg_val = tmpMSG->data[1];
                                                set_433Mhz_configNew(tmpMSG);
                                                config_PAN_EPAN_new();
                                                save_script_flash();
                                        }
                                        
                        }else{
                           
                           if (tmpMSG->dataLength == 16 || tmpMSG->dataLength == 24)
                                {
                                        updateDisable(tmpMSG);//update disables
                                }
                           
                           if(device_Status == Status_Idle && tmpMSG->dataLength == 12 && tmpMSG->data[0] == AB_CMD){ //start play all audioboxes
                                                      uint8_t tmp_mp3_cmd = tmpMSG->data[5];
                                                      switch(tmp_mp3_cmd)
                                                       {
                                                         case 0: //Disbale Player
                                                             //Mp3Disfunc();//Not available
                                                             sendSMStatusInterface();
                                                             break;
                                                         case 1://Stop
                                                             Audio_Stop();
                                                             init_delay_timer(device_datas.device_Address * 80);
                                                             break;
                                                         case 2: //Enable player
                                                             //Mp3Enfunc(); //Not available
                                                             break;
                                                         case 3: //Play//Pause
                                                              Audio_Play_Pause_Idle();
                                                              init_delay_timer(device_datas.device_Address * 80);
                                                             //UART_MP3_TXPlay();
                                                         break;
                                                       }
                           }else{
                           if(device_Status > Status_Idle){
                                
                                        was_here_X++;
                                        if(was_here_X > 3){
                                                SEQ_AddEvent(ev_MS_ToTEST);
                                                was_here_X = 0;
                                        }
                                }
                         is_old_firmware = tmpMSG->data[7];
                         //SEQ_AddEvent(ev_MS_Ping);//TOADD
                        sendSMStatusInterface();
                        //init_delay_timer(100);
                        //response_x_ping++;
                                
                                
                                if(tmpMSG->dataLength == 24){
                                        set_433Mhz_config(tmpMSG);
                                        is_first_mod_msg = (tmpMSG->data[17] >> 7) & 0x01;
                                        is_need_sent++;

                                        PING_OFFSET_x = tmpMSG->data[11];
                                        real_time_ctrl = (tmpMSG->data[15] << 24) | (tmpMSG->data[14] << 16) | (tmpMSG->data[13] << 8) | (tmpMSG->data[12] << 0);
                                        
                                        set_real_time(0, real_time_ctrl);
                                }
                                if(device_Status == Status_Idle)
                                        {
                                                uint8_t tmp_vdata = tmpMSG->data[3] & 0x20;
                                                
                                                if(tmp_vdata == 0x20)
                                                {
                                                        /*
                                                        Hours = tmpMSG->data[0];
                                                        Min = tmpMSG->data[1];
                                                        Sec = tmpMSG->data[2];
                                                        */
                                                        GPS_SetValue = tmpMSG->data[0]*3600000 + tmpMSG->data[1] * 60000 + tmpMSG->data[2]*1000;
                                                        isGPSFTValid = 1;
                                                }
                                                else
                                                {
                                                        isGPSFTValid = 0;
                                                }
                                                
                                                tmp_vdata = tmpMSG->data[3] & 0x1F;
                                                
                                                // TOADD Save time zone
                                                if((TimeZone + 12) != tmp_vdata)
                                                {
                                                        TimeZone = tmp_vdata - 12;
                                                        /*
                                                        savedDatasCFG.T_Zone = tmp_vdata;
                                                        __disable_irq();
                                                        saveFlashDataCFG();
                                                        __enable_irq();
                                                        */
                                                }
                                                
                                        }
                                }
                        }
                   }
                   
                   else if (tmpMSG->address == MASTER_SLAVE_Broadcast) {
                                        if(was_started_config == 0)
                                        {
                                                if (tmpMSG->data[1] == 0xAA)
                                                {
                                                        was_started_config = 1;
                                                        Wireless_RST_Wait(1);
                                                        Config_Wir(1);// set only EPAN
                                                }
                                                else if (tmpMSG->data[1] == 0xAB)
                                                {
                                                        was_started_config = 1;
                                                        Wireless_RST_Wait(0);
                                                        Config_Wir(0); //reset to standard
                                                }
                                                else if (tmpMSG->data[1] == 0xAC)
                                                {
                                                        was_started_config = 1;
                                                        Wireless_RST_Wait(1);
                                                        Config_Wir(2);//set PAN and epan
                                                }
                                                else if (tmpMSG->data[1] == 0xAD)
                                                {
                                                        if (tmpMSG->dataLength == 16){
                                                                wir_network_rem_name[0] = tmpMSG->data[3];
                                                                wir_network_rem_name[1] = tmpMSG->data[4];
                                                                wir_network_rem_name[2] = tmpMSG->data[5];
                                                                wir_network_rem_name[3] = tmpMSG->data[6];
                                                        
                                                                was_started_config = 1;
                                                                Wireless_RST_Wait(1);
                                                                Config_Wir(3);//set New PAN and epan
                                                        }
                                                }
                                                else
                                                {
                                                        //do nothing
                                                }
                                        }
                                        if (tmpMSG->data[2] == 0xAA)
                                        {
                                                is_newDMXsoft = 1;
                                        }
                                        else
                                        {
                                                is_newDMXsoft = 0;
                                        }
                                        
                                           sendSMStatusInterface();
                                }
                         
                        }else{ //if (Can_be_Slave == 1)

                   device_datas.join_status = 1;
                   SEQ_AddEvent(ev_NetworkJoin);//TOADD 
                     
                     if (tmpMSG->address == MASTER_SLAVE_Sync) {
                         devicetime_MSx = ((tmpMSG->data[6] & 0xF0) << 20) | (tmpMSG->data[2] << 16) | (tmpMSG->data[1] << 8) | (tmpMSG->data[0] << 0);
                         
                             ping_interval = 100 * tmpMSG->data[11];
                                         if(ping_interval < 1000){
                                                ping_interval = DEVICE_SYNC_PERIOD;
                                         }
                                                     
                         if(is_init_done == 10)  
                         {
                                if (is_CopyMenu == 1)
                                {
                                      is_CopyMenu = 10;  
                                }
                         
                                 
                                if(device_Status == Status_Play && PPS_Mp3_AB == 1)
                                {

                                }else{
                                        //SEQ_AddEvent(ev_SyncMsg);//TOADD 
                                        Start_music_Sync();
                                        set_dev_time(devicetime_MSx);
                                        SEQ_AddEvent(ev_SyncMsg);//TOADD 
                                }
                                
                         }
                           
                   }
                   else if (tmpMSG->address < MASTER_SLAVE_Ping + MAX_NUMBER_OF_DEVICES) {
                           
                           if (tmpMSG->dataLength == 16)
                                {
                                        updateDisable(tmpMSG);//update disables
                                }
                           
                           if(tmpMSG->dataLength != 16) return;
                                        is_old_firmware = 1;
                                        
                                        switch(tmpMSG->data[0]){
                                        case PING_X://0 - ping a module for interface status and matrix status; Also can contain pyro and safety zones control
                                                
                                                if(tmpMSG->data[15] != 0)
                                                {
                                                        if(tmpMSG->data[15] == Status_PowerEnable && device_Status == Status_Idle){
                                                                SEQ_AddEvent(ev_Arm);
                                                        }
                                                }
                                                sendSMStatusInterface();  
                                                //updateDisable(tmpMSG);//update disables
                                                break;  
                                        case AB_CMD://AB COmmands
                                                if ((tmpMSG->address % 100) == device_datas.device_Address) {
                                                              uint8_t tmp_mp3_cmd = tmpMSG->data[5];
                                                              switch(tmp_mp3_cmd)
                                                               {
                                                                 case 0: //Disbale Player
                                                                     //Mp3Disfunc();//Not available
                                                                     sendSMStatusInterface();
                                                                     break;
                                                                 case 1://Stop
                                                                     Audio_Stop();
                                                                     init_delay_timer(100);
                                                                     break;
                                                                 case 2: //Enable player
                                                                     //Mp3Enfunc(); //Not available
                                                                     break;
                                                                 case 3: //Play//Pause
                                                                      Audio_Play_Pause_Idle();
                                                                      init_delay_timer(100);
                                                                     //UART_MP3_TXPlay();
                                                                 break;
                                                                 case 100: //Ask status 
                                                                     is_need_sent = 0;
                                                                     sendSMStatusInterface();
                                                                 break;
                                                               }
                                                }
                                                break;  
                                        }
                         
                   }
                   else if (tmpMSG->address == MASTER_SLAVE_Broadcast) {
                          
                          //rprg 12.07.2017
                           
                           if (tmpMSG->data[2] == 0xAA)
                           {
                                 is_newDMXsoft = 1;//TOADD
                                 is_old_firmware = 1;
                           }
                          else
                           {
                                 is_newDMXsoft = 0;//TOADD
                           }
                           
                          
                          if (tmpMSG->data[0] != adress_interface())
                           {
                                 device_datas.join_status = 1;
                                 SEQ_AddEvent(ev_NetworkJoin);//TOADD
                                 

                           }
                          else
                           {
                                 //SEQ_AddEvent(ev_ST_Error);TOADD
                           }
                          
                          
                            sendSMStatusInterface();
                          
                          
                    }
                    else if (tmpMSG->address == MASTER_SLAVE_PingBC) {


                       if(tmpMSG->dataLength == 19 && tmpMSG->data[0] == MASTER_SLAVE_WIRELESS_SET){//wireless settings
                                        if(tmpMSG->data[1] != waslastMsg_val){
                                                waslastMsg_val = tmpMSG->data[1];
                                                set_433Mhz_configNew(tmpMSG);
                                                config_PAN_EPAN_new();
                                                save_script_flash();
                                        }
                                        
                        }else{
                                
                          is_old_firmware = tmpMSG->data[7];
                            
                            if (tmpMSG->dataLength == 16 || tmpMSG->dataLength == 24)
                                {
                                        updateDisable(tmpMSG);//update disables
                                }
                           
                            
                          //SEQ_AddEvent(ev_MS_Ping);TOADD
                           if (tmpMSG->data[0] != adress_interface())
                           {
                                 device_datas.join_status = 1;
                                 SEQ_AddEvent(ev_NetworkJoin);//TOADD
                                 
                           }
                          else
                           {
                                 //SEQ_AddEvent(ev_ST_Error);TOADD
                           }
                           
                           if(tmpMSG->dataLength == 24){
                                        
                                        is_first_mod_msg = (tmpMSG->data[17] >> 7) & 0x01;
                                        is_need_sent++;

                                        PING_OFFSET_x = tmpMSG->data[11];
                                        real_time_ctrl = (tmpMSG->data[15] << 24) | (tmpMSG->data[14] << 16) | (tmpMSG->data[13] << 8) | (tmpMSG->data[12] << 0);
                                        
                                        set_real_time(0, real_time_ctrl);
                                }
                                if(device_Status == Status_Idle)
                                        {
                                                uint8_t tmp_vdata = tmpMSG->data[3] & 0x20;
                                                
                                                if(tmp_vdata == 0x20)
                                                {
                                                        Hours = tmpMSG->data[0];
                                                        Min = tmpMSG->data[1];
                                                        Sec = tmpMSG->data[2];
                                                        GPS_SetValue = Hours*3600000 + Min * 60000 + Sec*1000;
                                                        isGPSFTValid = 1;
                                                }
                                                else
                                                {
                                                        isGPSFTValid = 0;
                                                }
                                                
                                                tmp_vdata = tmpMSG->data[3] & 0x1F;
                                                
                                                // TOADD Save time zone
                                                if((TimeZone + 12) != tmp_vdata)
                                                {
                                                        TimeZone = tmp_vdata - 12;
                                                        /*
                                                        savedDatasCFG.T_Zone = tmp_vdata;
                                                        __disable_irq();
                                                        saveFlashDataCFG();
                                                        __enable_irq();
                                                        */
                                                }
                                                
                                        }
                                }
                        }
              }
       }
       device_FIFO.rdPtr++;
       if (device_FIFO.rdPtr == MAX_DEV_BUFFER) device_FIFO.rdPtr = 0;
}

 
 
void Start_music_Sync(void){

        //if(device_Status < Status_Play && PPS_Mp3_AB == 1 && devicetime_MSx > 2500){
        if(device_Status < Status_Play && devicetime_MSx > 2500){
                wave_player_current_seconds_set(devicetime_MSx/1000);
        //        wave_player_current_miliseconds_set(devicetime_MSx);
                
                 if(getWaveDataLength() < getCur_pos())
                                                {
                                                        wave_player_current_position_set(0);
                                                        is_audio_skip = 10;
                                                        set_dev_time(devicetime_MSx);
                                                      //  wave_player_stop();
                                                }else{
            
                          wave_player_state_set(STATE_WAVE_PLAYER_START);
                                                                
                                                        
                          wave_player_volume_set(60);
                          is_audio_skip = 1;
                  }
          }
        
}
 
 VS_VOID Audio_Play_Resume()
 {
         if(is_audio_skip != 10){
                         if(device_Status == Status_Pause)
                         {
                                wave_player_pause_resume_set(RESUME_STATUS);
                         }
                         else
                         {
                                wave_player_state_set(STATE_WAVE_PLAYER_START); //de vazut sa fie STOP si Play
                                 
  
/*                                 
                                if(devicetime_MSx > 0)
                                {
                                        is_audio_skip = 1;
                                }
                                 */
     
                         }
                 }
         
}
 

VS_VOID Audio_Pause()
 {
       wave_player_pause_resume_set(PAUSE_STATUS);
 }
 
VS_VOID Audio_Play_Pause_Idle()
{ 
        /*
                               case 1: sprintf(tempText, "%s", "\xF8\xF9"); break; //Play
                       case 2: sprintf(tempText, "%s", "\xFE\xFA"); break; //Stop
                       case 3: sprintf(tempText, "%s", "\xF7\xF6"); break; //Pause
                       case 10: sprintf(tempText, "%s", "xx"); break; //no File
        */
        
        
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

VS_VOID Audio_Stop()
 {
        WavePlayerStop();
//        wave_player_stop();
        devicetime_MSx = 0;
 }
 



uint8_t get_checkpwr(void)
 {
       return device_datas.power_Enable;
 }
 



void set_Time_Zone (uint16_t TZ_val)
 {
       if(TZ_val == 0)
        {
              TimeZone = 0;
        }
       else if(TZ_val < 12)
        {
              TimeZone =  - TZ_val;
        }
       else if(TZ_val  == 12)
        {
              TimeZone = 14;
        }
       else if(TZ_val  == 13)
        {
              TimeZone = 13;
        }
       else if(TZ_val  == 14)
        {
              TimeZone = 12;
        }
       else if(TZ_val  == 15)
        {
              TimeZone = 11;
        }
       else if(TZ_val  == 16)
        {
              TimeZone = 10;
        }
       else if(TZ_val  == 17)
        {
              TimeZone = 9;
        }
       else if(TZ_val  == 18)
        {
              TimeZone = 8;
        }
       else if(TZ_val  == 19)
        {
              TimeZone = 7;
        }
       else if(TZ_val  == 20)
        {
              TimeZone = 6;
        }
       else if(TZ_val  == 21)
        {
              TimeZone = 5;
        }
       else if(TZ_val  == 22)
        {
              TimeZone = 4;
        }
       else if(TZ_val  == 23)
        {
              TimeZone = 3;
        }
       else if(TZ_val  == 24)
        {
              TimeZone = 2;
        }
       else if(TZ_val  == 25)
        {
              TimeZone = 1;
        }
 }





void set_dev_address(uint8_t adr_ID)
{
        device_datas.device_Address = adr_ID;

}

VS_VOID clearPWEN()
 {
       // for Arm state report
       device_datas.power_Enable = 0;
 }

VS_VOID setPWEN()
 {
       device_datas.power_Enable = 1;
 }
 

uint8_t adress_interface()
 {
       return device_datas.device_Address;
 }

void Delay(__IO uint32_t nCount)
{
       __IO uint32_t index = 0; 
       for (index = (100000 * nCount); index != 0; index--);
}





VS_VOID Vol_Up()
 {
       if(main_volume < 96)  
       {
            main_volume = main_volume + 5;
       }
       else
       {
            main_volume = 100;
       }
       wave_player_volume_set(main_volume);
}

VS_VOID Vol_Down()
 {
       if(main_volume > 20)  
       {
            main_volume = main_volume - 5;
       }
       else
       {
            main_volume = 20;
       }
       wave_player_volume_set(main_volume);
 } 

 




 
 void check_arm_st(void)
 {
       crit_errA = 0;

       for (int i = 1; i < MAX_NUMBER_OF_DEVICES; i++)
        {
              if(slaves[i].isConnected == 1)
               {
                       
                      uint8_t temp_a = 0;
                        temp_a = slaves[i].interfaceStatus[0] & 0x20;      
                        
                        uint8_t temp_x = 0;
                        temp_x = slaves[i].interfaceStatus[0] & 0x03;    

                      if(temp_a == 0 || temp_x != 1)
                      {
                            crit_errA++;
                            slaves[i].slave_err = 1;
                      }
                      else{
                            mod_ping_snt[i] = 0;
                            slaves[i].slave_err = 0;
                      }

               }
        }
        
        if(crit_errA > 0){
                set_arm_timings();
        }
        else{
                rst_arm_timings();
        }
 }

 
 void rst_mod_stat(uint8_t mod){
        slaves[mod].interfaceStatus[0] = slaves[mod].interfaceStatus[0] & 0x3F;
        slaves[mod].interfaceStatus[0] = slaves[mod].interfaceStatus[0] & 0xDF;
 }
 
 
void check_arm_st_X(void)
{
        uint16_t x = getSlaves_connected();
        check_arm_st();
        
        if(crit_errA == 0) return;

        for (; armchecks < MAX_NUMBER_OF_DEVICES;)
        {
                if(slaves[armchecks].isConnected == 1 && (slaves[armchecks].slave_err & 0x01))
                {
                                if(get_mod_kind(armchecks, 0) == 0){
                                        sendMSPingX(armchecks);
                                }
                                else{
                                        sendMSPingA(armchecks);
                                }
                }
                armchecks++;
        }
        
        if(armchecks == MAX_NUMBER_OF_DEVICES){
                armchecks = 1;
        }
}




void sendMSPingX(uint8_t S_ID)
{
        MS_Message outputSPx;
        
        
        memset(&outputSPx, 0x00, sizeof(outputSPx));        
        
        outputSPx.address = S_ID + MASTER_SLAVE_Ping;
        outputSPx.dataLength = 8;

        

        outputSPx.data[0] = 0;	
        outputSPx.data[1] = 0;	
        outputSPx.data[2] = 0;	
        outputSPx.data[3] = 0;	      
        outputSPx.data[4] = 0;
        outputSPx.data[5] = 0;
        outputSPx.data[6] = 0;
        outputSPx.data[7] = 0;
        
     
        /*
        if(is_433Mhz_en == 1){
                
                        uint8_t msg_433_tmp[MAXIMUM_433_MESSAGE_SIZE] = {0};
                        
                        memset(msg_433_tmp, 0x00, sizeof(msg_433_tmp));        
                        
                        msg_433_tmp[0] = START_CHAR_433MHZ_MS;
                        msg_433_tmp[1] = M433_STOP_PING_SYNC_FIRE;
                        msg_433_tmp[2] = S_ID;
                        
                        Wireless_433_Add_msg_to_sent(msg_433_tmp);
        }
        */
        
        
        Wireless_Send(&outputSPx);
        //TODO-jcm ***** error handling******
}
 
 void sendMSPingA(uint8_t mod_id_ping)
 {
       MS_Message outputSPA1;
       memset(&outputSPA1, 0x00, sizeof(MS_Message));
       outputSPA1.address = mod_id_ping + MASTER_SLAVE_Ping;
       outputSPA1.dataLength = 16;
       
       outputSPA1.data[0] = 0x00;	
       outputSPA1.data[1] = 0x00;	
       outputSPA1.data[2] = 0x00;	
       outputSPA1.data[3] = 0x00;	      
       outputSPA1.data[4] = 0x00;
       outputSPA1.data[5] = 0x00;
       outputSPA1.data[6] = 0x00;
       outputSPA1.data[7] = 0x00;
       outputSPA1.data[8] = 0x00;
       outputSPA1.data[9] = 0x00;
       outputSPA1.data[10] = 0x00;
       
       for(int i = 0; i < 8; i++)
        {
              if(Safe_ZoneZ[i] == 1)
               {
                     outputSPA1.data[8] = outputSPA1.data[8] | (0x01 << i);
               }
              if(Safe_ZoneZ[i + 8] == 1)
               {
                     outputSPA1.data[9] = outputSPA1.data[9] | (0x01 << i);
               }
        }
       
       if(is_PyroDMX_Dis[0] > 0)
        {
              outputSPA1.data[10] = is_PyroDMX_Dis[0]; // 0 - no; 1- Pyro, 2 - DMX, 3 - PYRO + DMX;
        }
       else
        {
              outputSPA1.data[10] = is_PyroDMX_Dis[mod_id_ping]; // 0 - no; 1- Pyro, 2 - DMX, 3 - PYRO + DMX;
        }  
       
        
         if(is_433Mhz_en == 2){
                 
                 
                 send_uCast_433Mhz_msg(mod_id_ping,0);
                 
        }
        
        
        
       Wireless_Send(&outputSPA1);
 }

 /*
void sendMSPingBC(void)
 {
       
       if(device_Status > Status_Idle && current_Slave_For_D != 0)
        {
              sendMSPingA(current_Slave_For_D);
        }
       else
        {
               MS_Message output2;
                                   
                                   memset(&output2, 0x00, sizeof(MS_Message));
                                   output2.address = MASTER_SLAVE_PingBC;
                                   output2.dataLength = 16;

                                   
                                   Wireless_Send(&output2);
   
               }
}
*/

void sendMSPingBC(void)
{

        
        if(device_Status > Status_Idle && current_Slave_For_D != 0 && is_SF_OK == 0)
        {
                sendMSPingA(current_Slave_For_D);
        }
        else
        {
                is_SF_OK = 0;
                if(is_F1_check == 0)        
                {
                        /*
                        if(was_sl_test == 0)
                        {
                                was_sl_test = 1;
                        }
                        */
                        
                        if(start_prg == 0)
                        {
                                if (device_Status != Status_PowerEnable && device_Status != Status_Pause && device_Status != Status_Play)  //prg 13.07.2017
                                {
                                        
                                        MS_Message outputPC;
                                        
                                        
                                        memset(&outputPC, 0x00, sizeof(outputPC)); 
                                        outputPC.address = MASTER_SLAVE_PingBC;
                                        outputPC.dataLength = 24;
                                        uint8_t tmp_ft = 0;
                                        
                                        if(isGPSFTValid == 1 && device_Status == Status_Idle)
                                        {
                                                //        add valid seted fire hour
                                                outputPC.data[0] = Hours_Start;	
                                                outputPC.data[1] = Min_Start;	
                                                outputPC.data[2] = Sec_Start;	
                                                tmp_ft = 0x01;	
                                        }
                                        else
                                        {
                                                //        add invalid seted fire hour
                                                outputPC.data[0] = 0x00;	
                                                outputPC.data[1] = 0x00;	
                                                outputPC.data[2] = 0x00;	
                                                tmp_ft = 0x00;	
                                                
                                        }
                                        

                                        uint8_t tmp_gpsTZ = (uint8_t) TimeZone + 12;
                                        tmp_gpsTZ = tmp_gpsTZ & 0x1F;
                                        uint8_t tmp_gpsT = isGPSTimerValid;
                                        tmp_gpsT = tmp_gpsT << 6;
                                        tmp_ft = tmp_ft << 5;
                                        tmp_gpsT = tmp_gpsT | tmp_ft;
                                        tmp_gpsT = tmp_gpsT | tmp_gpsTZ;
                                        
                                        outputPC.data[3] = tmp_gpsT;	      
                                      //  outputPC.data[4] = (GPS_ModtimerValue >> 0)  & 0xFF;
                                     //   outputPC.data[5] = (GPS_ModtimerValue >> 8)  & 0xFF;

                                        outputPC.data[6] = 0; // reserved
                                        outputPC.data[7] = 1; // firmware version 
                                        
                                        
                                        
                                        outputPC.data[8] = 0x00;
                                        outputPC.data[9] = 0x00;
                                        outputPC.data[10] = 0x00;
                                        
                                        for(int i = 0; i < 8; i++)
                                        {
                                                if(Safe_ZoneZ[i] == 1)
                                                {
                                                        outputPC.data[8] = outputPC.data[8] | (0x01 << i);
                                                }
                                                if(Safe_ZoneZ[i+ 8] == 1)
                                                {
                                                        outputPC.data[9] = outputPC.data[9] | (0x01 << i);
                                                }
                                        }
                                        
                                        if(is_PyroDMX_Dis[0] > 0)
                                        {
                                                outputPC.data[10] = is_PyroDMX_Dis[0]; // 0 - no; 1- Pyro, 2 - DMX, 3 - PYRO + DMX;
                                        }
                                        else
                                        {
                                                outputPC.data[10] = is_PyroDMX_Dis[current_Slave_For_D]; // 0 - no; 1- Pyro, 2 - DMX, 3 - PYRO + DMX;
                                        }                                
                                        
                                        
                                        
                                        outputPC.data[11] = PING_OFFSET/10; // PING_OFSET in 10 of ms;
                                        
                                        outputPC.data[12] = (real_time >> 0)  & 0xFF;
                                        outputPC.data[13] = (real_time >> 8)  & 0xFF;
                                        outputPC.data[14] = (real_time >> 16)  & 0xFF;
                                        outputPC.data[15] = (real_time >> 24)  & 0xFF;
                                        
                                        outputPC.data[16] = get_Lang() & 0x0F; //Language 
                                        outputPC.data[16] = outputPC.data[16] | (Power_off_mods_enabled << 7); //Power off
                                        
                                        
                                                outputPC.data[16] = outputPC.data[16] | (savedDatasCfg.is_last_sleep << 6); //Sleep
                                        
                                        
                                        outputPC.data[17] = 0x00;
                                        outputPC.data[18] = 0x00;
                                        outputPC.data[19] = 0x00;
                                        outputPC.data[20] = 0x00;
                                        outputPC.data[21] = 0x00;
                                        outputPC.data[22] = 0x00;
                                        outputPC.data[23] = 0x00;

                                        if(is_433Mhz_en == 2){
                                                        uint8_t msg_433_tmp[MAXIMUM_433_MESSAGE_SIZE] = {0};
                                                        memset(msg_433_tmp, 0x00, sizeof(msg_433_tmp));        
                                                        
                                                        msg_433_tmp[0] = START_CHAR_433MHZ_MS;
                                                        msg_433_tmp[1] = M433_STOP_PING_SYNC_FIRE;
                                                        msg_433_tmp[2] = M433_BROADCAST;
                                                        
                                                        msg_433_tmp[3] = outputPC.data[0];
                                                        msg_433_tmp[4] = outputPC.data[1];
                                                        msg_433_tmp[5] = outputPC.data[2];
                                                        msg_433_tmp[6] = outputPC.data[3];
                                                        msg_433_tmp[7] = outputPC.data[4];
                                                        msg_433_tmp[8] = outputPC.data[5];
                                                        msg_433_tmp[9] = outputPC.data[6];
                                                        msg_433_tmp[10] = outputPC.data[7];
                                                        msg_433_tmp[11] = outputPC.data[8];
                                                        msg_433_tmp[12] = outputPC.data[9];
                                                        msg_433_tmp[13] = outputPC.data[10];
                                                        msg_433_tmp[14] = outputPC.data[11];
                                                        msg_433_tmp[15] = outputPC.data[12];
                                                        msg_433_tmp[16] = outputPC.data[13];
                                                        msg_433_tmp[17] = outputPC.data[14];
                                                        msg_433_tmp[18] = outputPC.data[15];
                                                        msg_433_tmp[19] = outputPC.data[16];
                                                        msg_433_tmp[20] = outputPC.data[17];
                                                        msg_433_tmp[21] = outputPC.data[18];
                                                        msg_433_tmp[22] = outputPC.data[19];
                                                        msg_433_tmp[23] = outputPC.data[20];
                                                        msg_433_tmp[24] = outputPC.data[21];
                                                        msg_433_tmp[25] = 0x00;
                                                        msg_433_tmp[26] = 0x00;
                                                        
                                                       
                                                        Wireless_433_Add_msg_to_sent(msg_433_tmp);

                                        }
                                        
                                        
                                        
                                        Wireless_Send(&outputPC);
                                        is_F1_checkA = 0;
                                }
                        }
                        
                        /*
                        cur_f1_mr = check_f1Mods();
                        if(cur_f1_mr > 0)
                        {
                                is_F1_check = 1;
                        }
                        else
                        {
                                is_F1_check = 0;            
                        }
                        */
                }
                else
                {
                        /*
                        if(cur_f1_mr > 0)
                        {
                                sendMSPingF1x(cur_f1_mr);
                                is_F1_checkA = 1;
                        }
                        
                        cur_f1_mr = check_f1Mods();
                        if(cur_f1_mr > 0)
                        {
                                is_F1_check = 1;
                        }
                        else
                        {
                                is_F1_check = 0;            
                        }
                        */
                }
        }
}



void send_sync_msg(int t_ossfet)
 {
       if(get_AB_Adre() != MIN_AB_ID) return;
        
       uint32_t devicetimeinms = 0;
       wave_player_current_miliseconds_get(&devicetimeinms);
         
       devicetimeinms = devicetimeinms + 30;
       uint32_t real_time_MSx = 0;
       uint32_t tmp_TimeX = 0;        

       
     
       
       MS_Message output2;
       memset(&output2,0x00, sizeof(MS_Message));
       output2.address = MASTER_SLAVE_Sync;
       output2.dataLength = 12;
       
       
       output2.data[0] = (devicetimeinms >> 0)  & 0xFF;
       output2.data[1] = (devicetimeinms >> 8)  & 0xFF;
       output2.data[2] = (devicetimeinms >> 16) & 0xFF;
       tmp_TimeX = (devicetimeinms >> 24) & 0x0F;
       tmp_TimeX = tmp_TimeX << 4;
       tmp_TimeX = tmp_TimeX & 0xF0;        
       output2.data[6] = tmp_TimeX;        
       
       
       output2.data[3] = (real_time_MSx >> 0)  & 0xFF;
       output2.data[4] = (real_time_MSx >> 8)  & 0xFF;
       output2.data[5] = (real_time_MSx >> 16) & 0xFF;
       
       tmp_TimeX = (real_time_MSx >> 24) & 0x0F;
       output2.data[6] = output2.data[6] | tmp_TimeX;
       
       
       output2.data[7] = isGPSTimerValid | 0xA0;

       output2.data[8] = (t_ossfet >> 0)  & 0xFF;
       output2.data[9] = (t_ossfet >> 8)  & 0xFF;
       output2.data[10] = (t_ossfet >> 16) & 0xFF;
       output2.data[11] = (t_ossfet >> 24) & 0xFF;



       Wireless_Send(&output2);
       
}


void set_dis_pyro_DMX(uint8_t mod, uint8_t state)
{
        is_PyroDMX_Dis[mod] = state;
        ping_timer = DEVICE_SYNC_PERIOD;
}


VS_VOID initSeqTimer()
{
        
        for(int i = 0; i < MAX_SEQ; i++)
        {
                seq_validT[i] = 0;
                seq_timers[i] = 0;
                reset_DMX_Seq(i);
        }
        
}


VS_VOID RST_DMX_Timer(){
        rst_DMX_Timer();
        initSeqTimer();
}





void set_433_seq_V(uint8_t *seq_msg){
        
        uint32_t tmp_timer_fire = 0;
        
                
        if(device_Status == Status_Play)
                tmp_timer_fire = getDevicetime();
        else{
                tmp_timer_fire = getDMXtime();
        }
        
        /* Bounds check: seq_msg[4] is used as index into arrays of size MAX_SEQ */
        if(seq_msg[4] >= MAX_SEQ) return;

        if(seq_timers[seq_msg[4]] != 0 && seq_timers[seq_msg[4]] > tmp_timer_fire + 300) return;


        if(seq_msg[3] == 0xAA)
        {

                /*
                uint32_t tmp_time_calc = ((seq_msg[12] & 0xFF) << 24) | ((seq_msg[13] & 0xFF) << 16) | ((seq_msg[14] & 0xFF) << 8) | ((seq_msg[15] & 0xFF) << 0);

                if(tmp_time_calc == 0xFFFFFFFF){
                        seq_timersX[seq_msg[4]] = 0;
                }else{
                        seq_timersX[seq_msg[4]] = tmp_time_calc;
                }
                */

                if(seq_msg[4] > 0)
                {
                        if(seq_validT[seq_msg[4]] == 0)
                        {
                                reset_DMX_Seq(seq_msg[4]);
                                seq_validT[seq_msg[4]] = 1;
                        }
                        else
                        {
                                reset_DMX_Seq(seq_msg[4]);
                        }
                        if(device_Status == Status_Play){
                                    if(seq_msg[0] == START_CHAR_433MHZ_SS){//message was forward it
                                                seq_timers[seq_msg[4]] = getDevicetime() + TIME_433_OFFSET;
                                    }else{
                                                seq_timers[seq_msg[4]] = getDevicetime();
                                    }
                        }
                        else if(device_Status == Status_PowerEnable || device_Status == Status_Pause){
                                    if(seq_msg[0] == START_CHAR_433MHZ_SS){//message was forward it
                                                seq_timers[seq_msg[4]] = getDMXtime() + TIME_433_OFFSET;
                                    }else{
                                                seq_timers[seq_msg[4]] = getDMXtime();
                                    }
                                
                        }
                }
        }
        else if(seq_msg[0] == 0x55)
        {
                
                if(seq_msg[4] > 0)
                {                        
                        if(seq_validT[seq_msg[4]] == 1)
                        {
                                
                                seq_validT[seq_msg[4]] = 0;
                                reset_DMX_Seq(seq_msg[4]);
                        }
                }
        }  
        
        
}



void Calc_Ramp_Step(uint8_t CH_ID_tmp, uint8_t Cur_DMX_CH_value, uint8_t New_DMX_CH_value, uint8_t DMX_Dur, uint32_t DMX_rmp_tmp)
{
        if(New_DMX_CH_value != Cur_DMX_CH_value)
        {
                long int tmp_value_XXX = 0;
                long int tmp_value_yyy = 0;
                
                is_DMX_R_valid = 1;
                
                CH_ID_tmp = CH_ID_tmp - 100;
                
                if(DMX_Dur == 1)// instant change dmx value and back to 0
                {
                        buf_DMX[CH_ID_tmp] = New_DMX_CH_value; //change value
                        DMX_Ramp_Vect[CH_ID_tmp][0] = 1000;  
                        DMX_Ramp_Vect[CH_ID_tmp][1] = DMX_rmp_tmp;  
                        DMX_Ramp_Vect[CH_ID_tmp][2] = 0;
                }
                else if(DMX_Dur == 2)// instant change dmx value and back to previous value
                {
                        buf_DMX[CH_ID_tmp] = New_DMX_CH_value; //change value
                        DMX_Ramp_Vect[CH_ID_tmp][0] = Cur_DMX_CH_value + 1000;  
                        DMX_Ramp_Vect[CH_ID_tmp][1] = DMX_rmp_tmp;  
                        DMX_Ramp_Vect[CH_ID_tmp][2] = 0;
                }           
                else if(DMX_Dur == 3)// ramp change dmx value in a specific time
                {
                        DMX_Ramp_Vect[CH_ID_tmp][0] = New_DMX_CH_value + 2000; //Value at the end
                        
                        tmp_value_yyy = New_DMX_CH_value - Cur_DMX_CH_value;
                        
                        
                        tmp_value_XXX = (int)(DMX_rmp_tmp) / tmp_value_yyy;
                        if(tmp_value_XXX == 0)
                        {
                                tmp_value_XXX = 1;
                        }
                        DMX_Ramp_Vect[CH_ID_tmp][1] = tmp_value_XXX;  //How much time need for 1 increase
                        DMX_Ramp_Vect[CH_ID_tmp][2] = tmp_value_XXX;  //How much time need for 1 increase
                        
                }
                else if(DMX_Dur == 4)// ramp change dmx value with a specific speed
                {
                        DMX_Ramp_Vect[CH_ID_tmp][0] = New_DMX_CH_value + 2000; //Value at the end
                        
                        tmp_value_yyy = New_DMX_CH_value - Cur_DMX_CH_value;
                        
                        if(DMX_rmp_tmp > 1000)
                        {
                                DMX_rmp_tmp = 1000;
                        }
                        if(DMX_rmp_tmp > 0)
                        {
                                tmp_value_XXX = 1000/ DMX_rmp_tmp;
                        }
                        
                        if(tmp_value_yyy < 0)
                        {
                                tmp_value_XXX *= (-1);
                        }
                        
                        DMX_Ramp_Vect[CH_ID_tmp][1] = tmp_value_XXX;  //How much time need for 1 increase
                        DMX_Ramp_Vect[CH_ID_tmp][2] = tmp_value_XXX;  //How much time need for 1 increase
                        
                }           
                else
                {
                        buf_DMX[CH_ID_tmp] = 0;// error DMX channel resets
                }
        }
}


VS_VOID ARM_DMX()
{
        
        dev_prog *tmp;
        
        if (!(device_Status == Status_PowerEnable)) return;
        if(is_Pause_Script == 3) return;
        if(is_Pause == 1) return;
       
       
        if (device_datas.programmed_interface == 0) return;
        if (savedDatas.time_prog.count == 0) return;
        
        for (uint32_t i = 0; i < TimeMatrixDepth; i++) {
                tmp = (dev_prog*)(&(savedDatas.time_prog.programme[i]));
                
                if (tmp->valid == 0 || tmp->valid == 2) continue;
                if(tmp->lineID < 100) continue; //Pyro event, ignore line
                if(tmp->DMX_Ramp != 100) continue; //Not Arming event, Ignore line
                if(tmp->Rmp_Cfg != 100) continue; //Not Arming event, Ignore line
                
                buf_DMX_ARM[tmp->lineID - 100] = tmp->DMX_value;
        }
        
        memcpy(buf_DMX, buf_DMX_ARM, sizeof(buf_DMX));//copy buffer DMX from ARM to Play DMX buffer
}




// New generate Time matrix events
void generateTimeMatrixEvent(uint64_t currentTimeX)
{
        
//        uint8_t tmp_LID_NX = 0;
        
        if(device_Status < Status_PowerEnable) return;
         
        if(is_Pause == 1) return;

        if (device_datas.programmed_interface == 0) return;
        if (savedDatas.time_prog.count == 0) return;
        if (currentTimeX == 0) return;
        
        
        clear_mark();
        
        dev_prog *tmp;
//        uint8_t flag = 0;
        static uint64_t currentTime = 0;
        
        for (uint32_t i = 0; i < TimeMatrixDepth; i++) {
                tmp = (dev_prog*)(&(savedDatas.time_prog.programme[i]));
                

                
                 if((device_Status == Status_PowerEnable || device_Status == Status_Pause) && tmp->seqID == 0) continue;
                         
                // if(isFiring == 1 && tmp->lineID < 100) continue;
                
                if(tmp->lineID < 100) continue; //Pyro channel, ignore it
                
                if(tmp->DMX_Ramp == 100 && tmp->Rmp_Cfg == 100 && tmp->lineID > 100) continue; //Arming event, Ignore line in Play
                
                
                
                if(mark_pos == MAX_CH_SAME_TIME - 1) break;
                
                if (tmp->valid == 0 || tmp->valid == 2) continue;
                
                if (tmp->seqID != 0 && seq_validT[tmp->seqID] == 0) continue;
                
                if(tmp->Safe_Zone > 0){
                        if(SafeZone_Disabled[tmp->Safe_Zone - 1] == 1){
                                if(tmp->lineID > 100)
                                {
                                        buf_DMX[tmp->lineID - 100] = 0;
                                }
                                continue;
                        }
                }
                
                
                if (tmp->seqID == 0)
                {
                        currentTime = currentTimeX;
                }
                else
                {
                        currentTime = currentTimeX - seq_timers[tmp->seqID];
                }
                
                
                
                
                if(tmp->lineID > 100)
                {
                        currentTime = currentTime + 200;
                }
                
                
                if (currentTime < 1000) {
                        if (tmp->time < currentTime) {
                                if(tmp->lineID > 100)
                                {
                                        
                                        if (device_Status != Status_Manual)
                                        {
                                                if (tmp->valid == 1)
                                                {
                                                        if(tmp->DMX_Ramp == 0)
                                                        {
                                                                buf_DMX[tmp->lineID - 100] = tmp->DMX_value;
                                                                DMX_Ramp_Vect[tmp->lineID - 100][0] = 0; 
                                                                DMX_Ramp_Vect[tmp->lineID - 100][1] = 0; 
                                                                DMX_Ramp_Vect[tmp->lineID - 100][2] = 0; 
                                                                
                                                        }
                                                        else
                                                        {
                                                                if(tmp->Rmp_Cfg != 0 && tmp->Rmp_Cfg != 100)
                                                                {
                                                                        Calc_Ramp_Step(tmp->lineID, buf_DMX[tmp->lineID - 100], tmp->DMX_value, tmp->Rmp_Cfg, tmp->DMX_Ramp);
                                                                }
                                                        }
                                                }
                                                else if (tmp->valid == 3)
                                                {
                                                        buf_DMX[tmp->lineID - 100] = 0;
                                                }
                                                savedDatas.time_prog.mark[mark_pos] = i;
                                                mark_pos++;
                                                //flag_DMX = 1;
                                                
                                        }
                                }
                        }
                }
                else
                {
                        if (tmp->time > (currentTime - 500)) {
                                
                                if (tmp->time < currentTime) {
                                        
                                        
                                        if(tmp->lineID > 100)
                                        {
                                                
                                                if (device_Status != Status_Manual)
                                                {
                                                        if (tmp->valid == 1)
                                                        {
                                                                if(tmp->DMX_Ramp == 0)
                                                                {
                                                                        buf_DMX[tmp->lineID - 100] = tmp->DMX_value;
                                                                        DMX_Ramp_Vect[tmp->lineID - 100][0] = 0; 
                                                                        DMX_Ramp_Vect[tmp->lineID - 100][1] = 0; 
                                                                        DMX_Ramp_Vect[tmp->lineID - 100][2] = 0; 
                                                                        
                                                                }
                                                                else
                                                                {
                                                                        if(tmp->Rmp_Cfg != 0 && tmp->Rmp_Cfg != 100)
                                                                        {
                                                                                Calc_Ramp_Step(tmp->lineID, buf_DMX[tmp->lineID - 100], tmp->DMX_value, tmp->Rmp_Cfg, tmp->DMX_Ramp);
                                                                        }
                                                                }
                                                        }
                                                        else if (tmp->valid == 3)
                                                        {
                                                                buf_DMX[tmp->lineID - 100] = 0;
                                                        }
                                                        savedDatas.time_prog.mark[mark_pos] = i;
                                                        mark_pos++;
                                                        //flag_DMX = 1;
                                                        
                                                }
                                        }
                                        
                                }
                                
                        }
                }
                
                
        }
        
        
        if(mark_pos != 0)
        {
                clearActivedTimeMartix();
        }
        

}





VS_VOID clearActivedTimeMartix()
{
        
    //    uint8_t tmp_seq_ID = 0;
        
        
        // if(mark_pos != 0)
        // {
        for (int i = 0; i < mark_pos; i++) {
                
                if (savedDatas.time_prog.programme[savedDatas.time_prog.mark[i]].valid == 3)
                {
                        savedDatas.time_prog.programme[savedDatas.time_prog.mark[i]].time = savedDatas.time_prog.programme[savedDatas.time_prog.mark[i]].time - 100 * savedDatas.time_prog.programme[savedDatas.time_prog.mark[i]].Rmp_Cfg;
                        
                        savedDatas.time_prog.programme[savedDatas.time_prog.mark[i]].valid = 0;
                        savedDatas.time_prog.count--;
                        
                       // tmp_seq_ID = savedDatas.time_prog.programme[savedDatas.time_prog.mark[i]].seqID;
                       // savedDatas.time_prog.count_seq[tmp_seq_ID]--;
                        
                }
                else
                {
                        if (savedDatas.time_prog.programme[savedDatas.time_prog.mark[i]].valid == 1)
                        {
                                if(savedDatas.time_prog.programme[savedDatas.time_prog.mark[i]].lineID < 100)// if are pyro
                                {
                                        savedDatas.time_prog.programme[savedDatas.time_prog.mark[i]].valid = 0;
                                        savedDatas.time_prog.count--;
                                        
                                     //   tmp_seq_ID = savedDatas.time_prog.programme[savedDatas.time_prog.mark[i]].seqID;
                                       // savedDatas.time_prog.count_seq[tmp_seq_ID]--;
                                }
                                else// if are DMX
                                {
                                        if(savedDatas.time_prog.programme[savedDatas.time_prog.mark[i]].Rmp_Cfg != 0 && savedDatas.time_prog.programme[savedDatas.time_prog.mark[i]].DMX_Ramp == 0)// if Column 7 (Duration) is used in old format
                                        {
                                                savedDatas.time_prog.programme[savedDatas.time_prog.mark[i]].valid = 3;
                                                savedDatas.time_prog.programme[savedDatas.time_prog.mark[i]].time = savedDatas.time_prog.programme[savedDatas.time_prog.mark[i]].time 
                                                        + 100 * savedDatas.time_prog.programme[savedDatas.time_prog.mark[i]].Rmp_Cfg;
                                                
                                                savedDatas.time_prog.count--;
                                                
                                         //       tmp_seq_ID = savedDatas.time_prog.programme[savedDatas.time_prog.mark[i]].seqID;
                                              //  savedDatas.time_prog.count_seq[tmp_seq_ID]--;
                                        }
                                        else 
                                        {
                                                savedDatas.time_prog.programme[savedDatas.time_prog.mark[i]].valid = 0;
                                                savedDatas.time_prog.count--;
                                                
                                          //      tmp_seq_ID = savedDatas.time_prog.programme[savedDatas.time_prog.mark[i]].seqID;
                                                //savedDatas.time_prog.count_seq[tmp_seq_ID]--;
                                        }
                                }
                                
                        }
                }    
                
                
        }
        //}
        clear_mark();
}

void reset_DMX_Seq(uint8_t sequenceRes)
{
        //savedDatas.time_prog.count = savedDatas.time_prog.count - savedDatas.time_prog.count_seq[sequenceRes];;
        //savedDatas.time_prog.count_seq[sequenceRes]  = 0;
        
        for (int i = 0; i < TimeMatrixDepth; i++) {
                //if(savedDatas.time_prog.programme[i].time  == 0) break;
                //if(savedDatas.time_prog.programme[i].Mod_ID != device_datas.device_Address) continue;
                if (savedDatas.time_prog.programme[i].seqID == sequenceRes && savedDatas.time_prog.programme[i].lineID > 100) 
                {
                        uint8_t LID_tmp = savedDatas.time_prog.programme[i].lineID;
                        if (savedDatas.time_prog.programme[i].valid == 3)
                        {
                                savedDatas.time_prog.programme[i].time = savedDatas.time_prog.programme[i].time - 100 * savedDatas.time_prog.programme[i].Rmp_Cfg;
                                savedDatas.time_prog.programme[i].valid = 1;
                                savedDatas.time_prog.count++;
                                
                            //    uint8_t tmp_seq_ID = savedDatas.time_prog.programme[i].seqID;
                              //  savedDatas.time_prog.count_seq[tmp_seq_ID]++;
                                
                                
                                buf_DMX[LID_tmp - 100] = 0;
                                
                        }
                        else
                        {
                                savedDatas.time_prog.programme[i].valid = 1;
                                savedDatas.time_prog.count++;
                            //    savedDatas.time_prog.count_seq[sequenceRes]++;
                                
                                //reset DMX buffer
                                
                                buf_DMX[LID_tmp - 100] = 0;
                        }
                        
                }
        }
}


VS_VOID reset_DMX_Buf()
{
        for (int i = 0; i < DMX_CH_MAX; i++)
                buf_DMX[i] = 0;
}



// New generate Time matrix events
void generateTimeMatrixEventOLD(uint64_t currentTimeX)
 {
       
       #if DEBUG_Y
       if (currentTimeX == 0) return;
       if (savedDatas.is_R_Prg == 0) return;
       if (savedDatas.time_prog.count == 0) return;
       
       if(isFiring != 0) return;
       

       dev_prog *tmp;
       uint64_t currentTime = 0;

       
       
       for (uint32_t i = 0; i < TimeMatrixDepth; i++) {
             tmp = (dev_prog*)(&(savedDatas.time_prog.programme[i]));
             

             
             if(tmp->lineID < 100 && is_Pause_Script == 1) continue; //Pyro disabled
             if(tmp->lineID > 100 && is_Pause_Script == 2) continue;//DMX disabled
             

             
          //   if(mark_pos == MAX_CH_SAME_TIME - 1) break;
             
             if(tmp->time == 0) break;
               
             if (valid_script_pos[i] == 0) break;
             if (valid_script_pos[i] == 10) continue;
             
               
             if(savedDatas.time_prog.count == 0) break;
               
             if (tmp->seqID != 0 && seq_validT[tmp->seqID] == 0) continue;
             
             if(SafeZone_Disabled[savedDatas.time_prog.Safe_Zone[tmp->lineID]] == 1) continue;
              
             if (tmp->seqID == 0)
              {
                    currentTime = currentTimeX;
              }
             else
              {
                    currentTime = currentTimeX - seq_timers[tmp->seqID];
              }
             


                   if (tmp->time < currentTime) {
                                       savedDatas.time_prog.count--;
                           
                                       savedDatas.time_prog.mark[mark_pos] = i;
                                       valid_script_pos[i] = 10; //fired
                                       isFiring = 1;
                   }
                   else
                   {
                       F_NextTime = tmp->time;
                       F_NextChannel = tmp->lineID;
                       break;
                   }

             
             
       }
       #endif
       
}

void set_valid_script_pos(void)
{
        #if DEBUG_Y
        for(int i = 0; i < TimeMatrixDepth; i++)
        {
                 if(savedDatas.time_prog.programme[i].time != 0)
                 {
                        valid_script_pos[i] = 1;
                 }
                 else
                 {
                        break;
                 }
        }
        #endif

}


void save_custom_Wir(uint64_t net_IDx)
{
        savedDatasCfg.CW_Address = net_IDx;
        if(net_IDx == 0)
        {
                savedDatasCfg.is_Custom_Wir = 0;
        }
        else
        {
                savedDatasCfg.is_Custom_Wir = 1;
        }
        
}



void setSlavePingedX(uint8_t pinged_slaveB)
{
}

void Set_Ping_TimeX(uint8_t pinged_slaveA)
{
        if(pinged_slaveA >= MAX_NUMBER_OF_DEVICES) return;  /* Bounds guard */
        if(Pinged_Slave[pinged_slaveA] == 0)
        {
                uint32_t timerPingValue = Get_PingTimerValue();                
                Ping_Slave_Time [pinged_slaveA] = timerPingValue - pinged_slaveA * PING_OFFSET;
                Pinged_Slave[pinged_slaveA] = 1;
                getSlaves_Errors(pinged_slaveA);
        }
}


void getSlaves_Errors(uint8_t pinged_slave_X)
{
        if(pinged_slave_X >= MAX_NUMBER_OF_DEVICES) return;  /* Bounds guard */

        uint8_t tmp = 0;
        uint8_t tmpy = 0;
        uint8_t ERR_tmpy = 0;
        uint8_t WNG_tmpy = 0;
        
        uint8_t T_MatrixAA[4][12];
        
        
        
        if(pinged_slave_X == current_Slave_For_D)
        {
                update_rail_for_LCD(current_Slave_For_D);// for testing
                ERR_found = 0x00;
        }
        
        int i;
        int j;
        
        extract_mtxB(pinged_slave_X);
        
                /*
                //Slave Errors
                        //Slave Errors
                        #define DISC_NOT_CON_F          0 // disocnnected or not connected
                        #define SCRIPT_F                1 // Script, MD5 or module not programmed and rem programmed Faults
                        #define BATTERY_F               2 // Battery fault as is too low
                        #define OTHERS_F                3 //others


                        #define LS_NOTS_W               0 // Low signal or not in script
                        #define BATTERY_W               1 // battery warning as it is low
                        #define SCRIPT_W                2 //connected but not in script
                        #define OTHERS_W                3 // Other warnings
                */

        
        get_mod_kind(pinged_slave_X, 0);
        
        slaves[pinged_slave_X].slave_err = 0x00;
        slaves[pinged_slave_X].slave_wng = 0x00;
        
        uint8_t tdevicePrg = getSlaves_isProgrammed(pinged_slave_X);
        uint8_t tdevicePrgRem = getSlaves_isProgrammedRem(pinged_slave_X);
        
        if(tdevicePrgRem > 0 && slaves[pinged_slave_X].isConnected == 0) // not connected but programmed 
        {
                //slaves[pinged_slave_X].slave_err = slaves[pinged_slave_X].slave_err | 0x10;
                ERR_tmpy = ERR_tmpy | (0x01 << DISC_NOT_CON_F);
        }
        
//       #if DBG_TST_X 
        if(tdevicePrgRem == 0 && slaves[pinged_slave_X].isConnected == 1 && savedDatas.is_R_Prg == 1)
        {
                //slaves[pinged_slave_X].slave_err = slaves[pinged_slave_X].slave_err | 0x10;
                if(pinged_slave_X == 99 && (get_mod_kind(pinged_slave_X , 0) == 20 || get_mod_kind(pinged_slave_X, 0) == 21)){
                        //do nothing as it is Audiobox
                }
                else{
                         WNG_tmpy = WNG_tmpy | (0x01 << SCRIPT_W);
                         if(pinged_slave_X == current_Slave_For_D){
                                ERR_found = ERR_found | (0x01 << 7);
                         }
                        
                }
        }
        
        if(getSlaves_isProgrammedV(pinged_slave_X) == 3 && savedDatas.is_R_Prg == 1){ // Script in module but not into controller
                        ERR_tmpy = ERR_tmpy | (0x01 << SCRIPT_F);
                        if(pinged_slave_X == current_Slave_For_D){
                                ERR_found = ERR_found | (0x01 << 3);
                        }
        
        }
   //     #endif

        if(slaves[pinged_slave_X].isConnected == 1)
        {
        if(tdevicePrg == 1 && slaves[pinged_slave_X].matrixAStatusX == 0 && get_mod_kind(pinged_slave_X, 0) == 0)// igniters matrix checking
      //  if(tdevicePrg == 1 && slaves[pinged_slave_X].matrixAStatusX == 0)// igniters matrix checking
        {
                if(check_if_F1_M(pinged_slave_X) == 0)                
                {
                        for(i = 0; i < 24; i += 2)
                        {
                                for(j = 0; j < 4; j++)
                                {
                                        tmpy = 2*(3-j);
                                        tmp = slaves[pinged_slave_X].statusMatrixA[i];
                                        tmp = tmp >> tmpy;
                                        tmp = tmp & 0x03;
                                        T_MatrixAA[j][i/2] = tmp;		
                                }
                        }
                        
                        for(i=0; i<12; i++)
                        {
                                for(j=4; j<8; j++)
                                {
                                        tmp = matrixBStatusX[i];
                                        tmp = tmp >> j;
                                        tmp = tmp & 0x01;
                                        int jxxx = 7-j;

                                        if(tmp == 1 && T_MatrixAA[jxxx][i] < 2) //script Fault
                                        {
                                                ERR_tmpy = ERR_tmpy | (0x01 << SCRIPT_F);
                                                if(pinged_slave_X == current_Slave_For_D){
                                                        ERR_found = ERR_found | 0x01;
                                                }
                                                /*
                                                slaves[pinged_slave_X].slave_err = slaves[pinged_slave_X].slave_err | 0x01;                                                        
                                                if(pinged_slave_X == current_Slave_For_D)
                                                {
                                                        ERR_found = ERR_found | 0x01;
                                                }
                                                */
                                        }
                                        else if(tmp == 0 && T_MatrixAA[jxxx][i] > 1)
                                        {
                                                WNG_tmpy = WNG_tmpy | (0x01 << SCRIPT_W);
                                                
                                                if(pinged_slave_X == current_Slave_For_D){
                                                        ERR_found = ERR_found | 0x01;
                                                }
                                                /*
                                                slaves[pinged_slave_X].slave_err = slaves[pinged_slave_X].slave_err | 0x02;
                                                if(pinged_slave_X == current_Slave_For_D)
                                                {
                                                        ERR_found = ERR_found | 0x01;
                                                } 
                                                */                                                
                                        }
                                        else
                                        {
                                        }
                                        
                                }
                        }
                }
        }
        
        if(get_mod_kind(pinged_slave_X, 0) != 0)
        {
                if(((slaves[pinged_slave_X].interfaceStatus[5] >> 0) & 0x01) == 1 && slaves[pinged_slave_X].matrixAStatusX == 0)//Script Fault
                {
                        WNG_tmpy = WNG_tmpy | (0x01 << SCRIPT_W); //Error
                        if(pinged_slave_X == current_Slave_For_D){
                                ERR_found = ERR_found | 0x01;
                        }
                        
                        /*
                        slaves[pinged_slave_X].slave_err = slaves[pinged_slave_X].slave_err | 0x02;
                        if(pinged_slave_X == current_Slave_For_D)
                        {
                                ERR_found = ERR_found | 0x01;
                        } 
                        */                        
                }
                if(((slaves[pinged_slave_X].interfaceStatus[5] >> 1) & 0x01) == 1 && slaves[pinged_slave_X].matrixAStatusX == 0)//Script Fault
                {
                        ERR_tmpy = ERR_tmpy | (0x01 << SCRIPT_F);
                        if(pinged_slave_X == current_Slave_For_D){
                                ERR_found = ERR_found | 0x01;
                        }
                        
                        /*
                        slaves[pinged_slave_X].slave_err = slaves[pinged_slave_X].slave_err | 0x01;
                        if(pinged_slave_X == current_Slave_For_D)
                        {
                                ERR_found = ERR_found | 0x01;
                        }  
                        */
                }
                
                if(getSlaves_isProgrammed(pinged_slave_X) == 1 && get_Slave_prgX(pinged_slave_X) == 1 //MD5 Fault
                   && getSlaves_isProgrammedV(pinged_slave_X) == 0)//MD5 Fault
                {
                        ERR_tmpy = ERR_tmpy | (0x01 << SCRIPT_F);
                        if(pinged_slave_X == current_Slave_For_D){
                                ERR_found = ERR_found | 0x01;
                        }
                        
                        /*
                        slaves[pinged_slave_X].slave_err = slaves[pinged_slave_X].slave_err | 0x01;
                        if(pinged_slave_X == current_Slave_For_D)
                        {
                                ERR_found = ERR_found | 0x01;
                        } 
                        */                        
                }
                
                if(((slaves[pinged_slave_X].interfaceStatus[5] >> 6) & 0x01) == 1)//Hardware Warning
                {
                        WNG_tmpy = WNG_tmpy | (0x01 << OTHERS_W);
                        if(pinged_slave_X == current_Slave_For_D){
                                ERR_found = ERR_found | 0x10;
                        }
                        /*
                        slaves[pinged_slave_X].slave_err = slaves[pinged_slave_X].slave_err | 0x02;
                        if(pinged_slave_X == current_Slave_For_D)
                        {
                                ERR_found = ERR_found | 0x10;
                        } 
                        */                        
                }
                
                if(((slaves[pinged_slave_X].interfaceStatus[5] >> 7) & 0x01) == 1)//Hardware Fault
                {
                        ERR_tmpy = ERR_tmpy | (0x01 << OTHERS_F);
                        if(pinged_slave_X == current_Slave_For_D){
                                ERR_found = ERR_found | 0x20;
                        }
                        /*
                        slaves[pinged_slave_X].slave_err = slaves[pinged_slave_X].slave_err | 0x01;
                        if(pinged_slave_X == current_Slave_For_D)
                        {
                                ERR_found = ERR_found | 0x20;
                        }  
                        */
                }               
                
                for(int m = 0; m < 4; m++)
                {
                        if(((slaves[pinged_slave_X].interfaceStatus[6] >> 2 * m) & 0x03) == 3)//Rail Fault
                        {
                                ERR_tmpy = ERR_tmpy | (0x01 << OTHERS_F);
                                if(pinged_slave_X == current_Slave_For_D){
                                        ERR_found = ERR_found | 0x40;
                                }
                                /*
                                slaves[pinged_slave_X].slave_err = slaves[pinged_slave_X].slave_err | 0x01;
                                if(pinged_slave_X == current_Slave_For_D)
                                {
                                        ERR_found = ERR_found | 0x40;
                                }  
                                break;
                                */
                        }
                }
                
        }
        
        disc_time_asdf = (GetCurrentSystemTime()/PING_433_GRANULARITY) - LastPingTime[pinged_slave_X];
        
        uint32_t module_timer_ping_tmpsNR = 0;
        uint32_t module_timer_ping_tmpsD = 0;
        
        if(savedDatasCfg.sleep_mods[pinged_slave_X] == 1){
                module_timer_ping_tmpsNR = TIME_TO_SLEEP * 22000;
                module_timer_ping_tmpsD = TIME_TO_SLEEP * 42000;
        }else{
                
                module_timer_ping_tmpsNR = 25000 + Slaves_connected * MAX_NOT_RESPOND_TIME_PER_MOD;
                module_timer_ping_tmpsD = 25000 + Slaves_connected * MAX_DISC_TIME_PER_MOD;
                /*
                module_timer_ping_tmpsNR = MAX_NOT_RESPOND_TIME;
                module_timer_ping_tmpsD = MAX_DISC_TIME;
                */
        }
        
        if(ID_duplicated[pinged_slave_X] != 0)
        {
                ERR_tmpy = ERR_tmpy | (0x01 << OTHERS_F);
                if(pinged_slave_X == current_Slave_For_D){
                                        ERR_found = ERR_found | (0x01 << 8);
                }
                /*
                slaves[pinged_slave_X].slave_err = slaves[pinged_slave_X].slave_err | 0x01;
                */
        }
        //else if(LastPingTimeX[pinged_slave_X] > (get_ping_timeInterval() * MAX_DISC_TIME))//module disconnected
        else if(disc_time_asdf > module_timer_ping_tmpsD/PING_433_GRANULARITY)//module disconnected
        {
                ERR_tmpy = ERR_tmpy | (0x01 << DISC_NOT_CON_F);
                if(pinged_slave_X == current_Slave_For_D){
                        ERR_found = ERR_found | 0x10;
                }
                
                /*
                slaves[pinged_slave_X].slave_err = slaves[pinged_slave_X].slave_err | 0x01;
                if(pinged_slave_X == current_Slave_For_D)
                {
                        ERR_found = ERR_found | 0x10;
                } 
                */                
        }
        //else if (LastPingTimeX[pinged_slave_X] > (get_ping_timeInterval() * MAX_NOT_RESPOND_TIME))// module not responding
        else if(disc_time_asdf > module_timer_ping_tmpsNR/PING_433_GRANULARITY)//module not respond
        {
                WNG_tmpy = WNG_tmpy | (0x01 << LS_NOTS_W);
                if(pinged_slave_X == current_Slave_For_D){
                        ERR_found = ERR_found | 0x02;
                }
                /*
                slaves[pinged_slave_X].slave_err = slaves[pinged_slave_X].slave_err | 0x02;
                if(pinged_slave_X == current_Slave_For_D)
                {
                        ERR_found = ERR_found | 0x02;
                } 
                */                
        }
        else
        {
        }
        
        
        uint16_t tempA = slaves[pinged_slave_X].interfaceStatus[2];
        tempA = tempA & 0xC;
        tempA = tempA >> 2;
        
        if(tempA != 3)
        {
                uint8_t tmp_int_B = getSlaves_Intern_Bat(pinged_slave_X);
                if(get_mod_kind(pinged_slave_X, 0) != 0)
                {
                        
                        if(tmp_int_B < 2) // Too Low Battery
                        {
                                ERR_tmpy = ERR_tmpy | (0x01 << BATTERY_F);
                                if(pinged_slave_X == current_Slave_For_D){
                                        ERR_found = ERR_found | 0x04;
                                }
                                
                                /*
                                slaves[pinged_slave_X].slave_err = slaves[pinged_slave_X].slave_err | 0x01;
                                if(pinged_slave_X == current_Slave_For_D)
                                {
                                        ERR_found = ERR_found | 0x04;
                                }
                                */
                        }
                        else if (tmp_int_B < 3) //LOW BATTERY
                        {
                                WNG_tmpy = WNG_tmpy | (0x01 << BATTERY_W);
                                if(pinged_slave_X == current_Slave_For_D){
                                        ERR_found = ERR_found | 0x04;
                                }
                                /*
                                slaves[pinged_slave_X].slave_err = slaves[pinged_slave_X].slave_err | 0x02;
                                if(pinged_slave_X == current_Slave_For_D)
                                {
                                        ERR_found = ERR_found | 0x04;
                                } 
                                */                                
                        }
                        else
                        {
                        }
                        
                }
                else
                {
                        if(tmp_int_B < 1)//Too low battery
                        {
                                ERR_tmpy = ERR_tmpy | (0x01 << BATTERY_F);
                                if(pinged_slave_X == current_Slave_For_D){
                                        ERR_found = ERR_found | 0x04;
                                }
                                /*
                                slaves[pinged_slave_X].slave_err = slaves[pinged_slave_X].slave_err | 0x01;
                                if(pinged_slave_X == current_Slave_For_D)
                                {
                                        ERR_found = ERR_found | 0x04;
                                }
                                */
                        }
                        else if (tmp_int_B < 3)//Low battery
                        {
                                WNG_tmpy = WNG_tmpy | (0x01 << BATTERY_W);
                                if(pinged_slave_X == current_Slave_For_D){
                                        ERR_found = ERR_found | 0x04;
                                }
                                /*
                                slaves[pinged_slave_X].slave_err = slaves[pinged_slave_X].slave_err | 0x02;
                                
                                if(pinged_slave_X == current_Slave_For_D)
                                {
                                        ERR_found = ERR_found | 0x04;
                                }
                                */
                        }
                        else
                        {
                        }
                }
        }
        }
        
        
        
        slaves[pinged_slave_X].slave_err = ERR_tmpy;
        slaves[pinged_slave_X].slave_wng = WNG_tmpy;
        
        
        //rprg 31.07.2017
        
        /*
        if(slaves[pinged_slave_X].is_PRG_X == 1 && getSlaves_isProgrammed(pinged_slave_X) == 0)
        {
               // ERR_tmpy = ERR_tmpy | (0x01 << PRG_FAULT_MS_F);

                slaves[pinged_slave_X].slave_err = slaves[pinged_slave_X].slave_err | 0x01;
                
                if(pinged_slave_X == current_Slave_For_D)
                {
                        ERR_found = ERR_found | 0x08;
                }

        }
        else
        {
                
        }
        
        
        
        if(S_PRGxA > 0)
        {
                for(i = 1; i < (MAX_NUMBER_OF_DEVICES - 1); i++)
                {
                        if(slaves[i].is_PRG_X == 1 && slaves[i].isConnected == 0)
                        {
                               // ERR_tmpy = ERR_tmpy | (0x01 << PRG_FAULT_MS_F);

                                slaves[i].slave_err = slaves[i].slave_err | 0x01;

                        }
                        else
                        {
                        }
                        
                }
        }
        */
        
        getSlaves_connected();
        
        
}


void getAB_Errors(uint8_t pinged_AB_X){
        
        uint8_t ERR_tmpy = 0;
        uint8_t WNG_tmpy = 0;
        
        if(pinged_AB_X > 0 && pinged_AB_X < 10){  // it is AB
                AudioBoxes[pinged_AB_X].slave_err = 0x00;
                AudioBoxes[pinged_AB_X].slave_wng = 0x00;
        }
        
        
        if(AudioBoxes[pinged_AB_X].isConnected == 1){
        
                disc_time_asdf = (GetCurrentSystemTime()/PING_433_GRANULARITY) - LastPingTimeAB[pinged_AB_X];
                
                uint32_t module_timer_ping_tmpsNR = 0;
                uint32_t module_timer_ping_tmpsD = 0;
                

                        
                module_timer_ping_tmpsNR = 25000 + Slaves_connected * MAX_NOT_RESPOND_TIME_PER_MOD;
                module_timer_ping_tmpsD = 25000 + Slaves_connected * MAX_DISC_TIME_PER_MOD;



                if(disc_time_asdf > module_timer_ping_tmpsD/PING_433_GRANULARITY)//module disconnected
                {
                        ERR_tmpy = ERR_tmpy | (0x01 << DISC_NOT_CON_F);
                        if(pinged_AB_X == current_Slave_For_D){
                                ERR_found = ERR_found | 0x10;
                        }
                       
                }
                else if(disc_time_asdf > module_timer_ping_tmpsNR/PING_433_GRANULARITY)//module not respond
                {
                        WNG_tmpy = WNG_tmpy | (0x01 << LS_NOTS_W);
                        if(pinged_AB_X == current_Slave_For_D){
                                ERR_found = ERR_found | 0x02;
                        }
                }
                else
                {
                }
                
                
                uint16_t tempA = slaves[pinged_AB_X].interfaceStatus[2];
                tempA = tempA & 0xC;
                tempA = tempA >> 2;
                
                if(tempA != 3)
                {
                        uint8_t tmp_int_B = getAB_Intern_Bat(pinged_AB_X);
                                if(tmp_int_B < 2) // Too Low Battery
                                {
                                        ERR_tmpy = ERR_tmpy | (0x01 << BATTERY_F);
                                        if(pinged_AB_X == current_Slave_For_D){
                                                ERR_found = ERR_found | 0x04;
                                        }
                                        
                                }
                                else if (tmp_int_B < 3) //LOW BATTERY
                                {
                                        WNG_tmpy = WNG_tmpy | (0x01 << BATTERY_W);
                                        if(pinged_AB_X == current_Slave_For_D){
                                                ERR_found = ERR_found | 0x04;
                                        }
                                }
                                else
                                {
                                }
                                

                }
        }
        
        
        
        AudioBoxes[pinged_AB_X].slave_err = ERR_tmpy;
        AudioBoxes[pinged_AB_X].slave_wng = WNG_tmpy;
        
        getSlaves_connected();

}



        //5 bits for module kind
        // - 0: Not used/ old modules old firmware
        // - 1: 48Fx - new firmware
        // - 2: 48E - new firmware
        // - 3: Reserved
        // - 4: Reserved
        // - 5: Reserved
        // - 6: Reserved
        // - 7: Reserved
        // - 8: Reserved
        // - 9: Reserved
        // - 10: FTQ-4x16 4 channels
        // - 11: FTQ-4x16 16 channel 1 rail
        // - 12: FTQ-16x64 64 ch normal 4 rails
        // - 13: Reserved
        // - 14: Reserved
        // - 15: Reserved
        // - 16: Reserved
        // - 17: Reserved
        // - 18: Reserved
        // - 19: Reserved
        // - 20: FTM-99Sx - AB New firmware
        // - 21: AB Special
        // - 22: Reserved
        // - 23: Reserved
        // - 24: Reserved
        // - 25: Reserved
        // - 26: Reserved
        // - 27: Reserved
        // - 28: Reserved
        // - 29: Reserved
        // - 30: Reserved
        // - 31: Reserved

uint8_t get_mod_kind(uint8_t Mod_ID_tmp, uint8_t DMX_Option)
{
        
        slaves[Mod_ID_tmp].Mod_Kind = slaves[Mod_ID_tmp].interfaceStatus[7];
        
        
        if(DMX_Option == 1)
        {
                return slaves[Mod_ID_tmp].Mod_Kind & 0xE0;//return option as DMX, bit 8:DMX
        }
        else
        {
                //return slaves[Mod_ID_tmp].Mod_Kind & 0x1F; // return only module Kind, 0 - 48Fx, 1 - 64Qx, 2 - 16Qx, 3 - 4Qx
                
                if((slaves[Mod_ID_tmp].Mod_Kind & 0x1F) == 10) // small mdoule 4 channels
                {
                        return 3;
                }
                else if((slaves[Mod_ID_tmp].Mod_Kind & 0x1F) == 11) // Small module 16 channels
                {
                        return 2;
                }
                else if((slaves[Mod_ID_tmp].Mod_Kind & 0x1F) == 12) // FTQ-16x64 64 channels
                {
                        return 1;
                }
                else if((slaves[Mod_ID_tmp].Mod_Kind & 0x1F) == 1) // 48Fx new firmware
                {
                        return 0;
                }
                else if((slaves[Mod_ID_tmp].Mod_Kind & 0x1F) == 2) //48E new firmware
                {
                        return 0;
                }
                else if((slaves[Mod_ID_tmp].Mod_Kind & 0x1F) == 20) // 99Sx as Audiobox
                {
                        return 20;
                }
                else if((slaves[Mod_ID_tmp].Mod_Kind & 0x1F) == 21) // firetek Audiobox
                {
                        return 21;
                }
                else //unknow or others old firmware
                {
                        return 0;
                }
        }
}




uint8_t getSlaves_isProgrammed(uint8_t slv_disp)
{
        uint8_t temp_a = 0;
        uint8_t temp_b = 0;
        temp_a = slaves[slv_disp].interfaceStatus[0] & 0x40;
        
        if (temp_a > 0)
        {
                temp_b = 1;		
        }
        return temp_b;
        
}

uint8_t getSlaves_isProgrammedRem(uint8_t slv_disp)
{
//        #if DBG_TST_X 
                return savedDatas.time_prog.PRG_ID[slv_disp];
   //     #else
    //            return 0;
   //     #endif
}

uint8_t check_if_F1_M(uint8_t mod_tmp) // check if the slave it is an F1 firetek controller
{
        
        return 0;          
        //return ((slaves[mod_tmp].interfaceStatus[0] >> 3) & 0x01);        
        
}

uint8_t getSlaves_isProgrammedV(uint8_t slv_disp)//MD5 verification
{
        if(slaves[slv_disp].interfaceStatus[0] >> 6 && 0x01) //module has a script
                {
//                        #if DBG_TST_X 
                                 if(savedDatas.time_prog.PRG_ID[slv_disp] == 1){//Remote has a script for the module
                                        return 1;// In module and on controller
                                 }
                                 else{
                                        return 2;//in module not in controller
                                 }
             //           #else
               //                 return 0;
                //        #endif
                }
        else{ // module does not have a script
//                #if DBG_TST_X 
                if(savedDatas.time_prog.PRG_ID[slv_disp] == 1){//Remote has a script for the module
                                return 3;//in controller not in module
                         }
                         else{
                                return 0; // not in module not in controller
                         }
      //          #else
     //                   return 0;
      //          #endif
        }
        /*
                if(slaves[slv_disp].is_PRG_MD5 == 100)// script in remote and module and ok
                {
                        return 1;
                }
                else if(slaves[slv_disp].is_PRG_MD5 == 1)// script in both but different
                {
                        return 0;
                }
                else if(slaves[slv_disp].is_PRG_MD5 == 2)// no script into module, no script into remote
                {
                        return 0;
                }
                else if(slaves[slv_disp].is_PRG_MD5 == 3)// script in remote only
                {
                        return 0;
                }
                else if(slaves[slv_disp].is_PRG_MD5 == 4)// script in module only
                {
                        return 0;
                }
                else
                {
                        return 0;
                }
        */
}








uint8_t get_Slave_prgX(uint8_t isadrX)
{
        return slaves[isadrX].is_PRG_X;
}

uint32_t get_ping_timeInterval(void)
{
        return (calculate_ping()/PING_433_GRANULARITY);// to check
}

uint8_t getSlaves_Intern_Bat(uint8_t slave_bat)
{
        uint8_t temp = slaves[slave_bat].interfaceStatus[2];
        temp = (temp >> 4);
        
         if(get_mod_kind(slave_bat, 0) != 0){
                if(temp == 7)
                {
                        return 1;
                }
                else
                {
                        return  temp + 1;
                }
        }
         else{
                 //return temp;
                 /*
                
                 temp 11 -> 0 // no battery
                 temp 0 -> 1 // 0 lines
                 temp 2 -> 2 // 1 lines
                 temp 4 -> 3 // 2 lines
                 temp 6 -> 4 // 3 lines
                 temp 8 -> 5 // 4 lines
                 temp 10 -> 6 // 5 lines
                 
                 */
                 
                 
                if(temp == 0){
                        return  1;
                 }
                 else if(temp == 1){
                        return  2;
                 }
                 else if(temp == 4){
                        return  3;
                 }
                 else if(temp == 6){
                        return  4;
                 }
                 else if(temp == 8){
                        return  5;
                 }
                 else if(temp == 10){
                        return  6;
                 }
                 else if(temp == 11){
                         return 0;
                 }
                 else{
                        return 0;
                 }
                 
               
         }
}


uint8_t getAB_Intern_Bat(uint8_t slave_bat)
{
        uint8_t temp = AudioBoxes[slave_bat].interfaceStatus[2];
        temp = (temp >> 4);
        

               //  return temp;
              
              /*
                 temp 11 -> 0 // no battery
                 temp 0 -> 1 // 0 lines
                 temp 2 -> 2 // 1 lines
                 temp 4 -> 3 // 2 lines
                 temp 6 -> 4 // 3 lines
                 temp 8 -> 5 // 4 lines
                 temp 10 -> 6 // 5 lines
                 
           */
                 
                 
                if(temp == 0){
                        return 1;
                 }
                 else if(temp == 1){
                        return  2;
                 }
                 else if(temp == 2){
                        return  3;
                 }
                 else if(temp == 3){
                        return  4;
                 }
                 else if(temp == 4){
                        return  5;
                 }
                 else if(temp == 5){
                        return  6;
                 }
                 else if(temp == 7){
                         return 0;
                 }
                 else{
                        return 0;
                 }
}



void rst_slaves_script(void){
        for (uint8_t i = 1; i < MAX_NUMBER_OF_DEVICES; i++) {
                
                slaves[i].slave_err = 0x00;
        }
}


uint16_t getSlaves_connected(void)
{
        
        //rprg 31.07.2017
        
        Slaves_connected = 0;
        AB_connected = 0;
        S_PRGx = 0;
        S_PRGxA = 0;
        
        crit_err = 0;
        ncrit_err = 0; 
        
        memset(Mods_Err,0x00 , sizeof(Mods_Err));
        memset(Mods_Wngs,0x00 , sizeof(Mods_Wngs));
        
        signal_flag = 0;
        
        for (uint8_t i = 1; i < MAX_NUMBER_OF_DEVICES; i++) {
                
              //  slaves[i].slave_err = 0x00;
                
                // connected modules
                if (slaves[i].isConnected == 1)
                {
                        if(i < MAX_NUMBER_OF_DEVICES - 1){
                                Slaves_connected = Slaves_connected + 1;
                        }
                }
                
                //PRG remote vs PRG connected/not connected modules
                
//                #if DBG_TST_X 
                        if (savedDatas.time_prog.PRG_ID[i] > 1)
                        {
                                S_PRGx = S_PRGx + 1;
                                
                                if (slaves[i].isConnected == 0)
                                {
                                        S_PRGxA = S_PRGxA + 1;
                                }
                        }
  //              #endif
                
                /*
                uint8_t Mods_Err[4] = {0};
                uint8_t Mods_Wngs[4] = {0};
                
                #define DISC_NOT_CON_F          0 // disocnnected or not connected
                #define SCRIPT_F                1 // Script, MD5 or module not programmed and rem programmed Faults
                #define BATTERY_F               2 // Battery fault as is too low
                #define OTHERS_F                3 //others
                

                #define LS_NOTS_W               0 // Low signal or not in script
                #define BATTERY_W               1 // battery warning as it is low
                #define SCRIPT_W                2 //connected but not in script
                #define OTHERS_W                3 // Other warnings                
                */
                


                
                if((slaves[i].slave_err >> DISC_NOT_CON_F) & 0x01)
                {
                        Mods_Err[DISC_NOT_CON_F]++; //line 1
                        signal_flag = 1;
                }
                if((slaves[i].slave_err >> SCRIPT_F) & 0x01)
                {
                        Mods_Err[SCRIPT_F]++; //line 2
                }
                if((slaves[i].slave_err >> BATTERY_F) & 0x01)
                {
                        Mods_Err[BATTERY_F]++; //line 3
                }
                if((slaves[i].slave_err >> OTHERS_F) & 0x01)
                {
                        Mods_Err[OTHERS_F]++; // //line 4
                }
                
                
                
                                
                if((slaves[i].slave_wng >> LS_NOTS_W) & 0x01)
                {
                        Mods_Wngs[LS_NOTS_W]++; //Line 1
                        signal_flag = 1;
                }
                if((slaves[i].slave_wng >> SCRIPT_W) & 0x01)
                {
                        Mods_Wngs[SCRIPT_W]++; //Line 2
                }
                
                if((slaves[i].slave_wng >> BATTERY_W) & 0x01)
                {
                        Mods_Wngs[BATTERY_W]++; // Line 3
                }
                if((slaves[i].slave_wng >> OTHERS_W) & 0x01)
                {
                        Mods_Wngs[OTHERS_W]++; //Line 4
                }
                
                
                
                // error read                
                if(slaves[i].slave_err > 0)
                {
                        crit_err++;
                }
                
                if (slaves[i].slave_wng > 0)
                {
                        ncrit_err++;              
                }

/*                
                if(check_if_F1_M(i) == 1)  
                {
                        ncrit_err = ncrit_err + slaves[i].statusMatrixA[3];                        
                        crit_err = crit_err + slaves[i].statusMatrixA[4];
                        
                }
                */
                
                
        }
        
       
                for (uint8_t i = 1; i < MAX_NUMBER_OF_AB + 1; i++) {//for AB
                
                        /*
                if (AudioBoxes[i].isConnected == 1)
                {
                        if(i < MAX_NUMBER_OF_AB){
                                AB_connected = AB_connected + 1;
                        }
                }
                */
                
                /*
                uint8_t Mods_Err[4] = {0};
                uint8_t Mods_Wngs[4] = {0};
                
                #define DISC_NOT_CON_F          0 // disocnnected or not connected
                #define SCRIPT_F                1 // Script, MD5 or module not programmed and rem programmed Faults
                #define BATTERY_F               2 // Battery fault as is too low
                #define OTHERS_F                3 //others
                

                #define LS_NOTS_W               0 // Low signal or not in script
                #define BATTERY_W               1 // battery warning as it is low
                #define SCRIPT_W                2 //connected but not in script
                #define OTHERS_W                3 // Other warnings                
                */
                


                
                if((AudioBoxes[i].slave_err >> DISC_NOT_CON_F) & 0x01)
                {
                        Mods_Err[DISC_NOT_CON_F]++; //line 1
                        signal_flag = 1;
                }
                if((AudioBoxes[i].slave_err >> BATTERY_F) & 0x01)
                {
                        Mods_Err[BATTERY_F]++; //line 3
                }
                if((AudioBoxes[i].slave_err >> OTHERS_F) & 0x01)
                {
                        Mods_Err[OTHERS_F]++; // //line 4
                }
                
                
                
                                
                if((AudioBoxes[i].slave_wng >> LS_NOTS_W) & 0x01)
                {
                        Mods_Wngs[LS_NOTS_W]++; //Line 1
                        signal_flag = 1;
                }
                
                if((AudioBoxes[i].slave_wng >> BATTERY_W) & 0x01)
                {
                        Mods_Wngs[BATTERY_W]++; // Line 3
                }
                if((AudioBoxes[i].slave_wng >> OTHERS_W) & 0x01)
                {
                        Mods_Wngs[OTHERS_W]++; //Line 4
                }
                
                
                
                // error read                
                if(AudioBoxes[i].slave_err > 0)
                {
                        crit_err++;
                }
                
                if (AudioBoxes[i].slave_wng > 0)
                {
                        ncrit_err++;              
                }
               
                
        }
        
        return Slaves_connected;
}


uint8_t get_slave_con(void){
        return Slaves_connected;
}

uint8_t get_AB_con(void){
        uint8_t tmp_AB_conect = 0;
         for (uint8_t i = 1; i < MAX_NUMBER_OF_AB; i++) {//for AB
                
                if (AudioBoxes[i].isConnected == 1)
                {
                                tmp_AB_conect++;
                }
        }
        AB_connected = tmp_AB_conect;
        return AB_connected;
}

uint8_t get_433_slave_con(void){
        uint8_t slaves_con_433_mhz = 0;
        for (int m = 0; m < MAX_NUMBER_OF_DEVICES; m++)
                {
                        if(slaves[m].is433WirelessConnected == 1){
                                slaves_con_433_mhz++;
                        }
                }
                
         return slaves_con_433_mhz;
}

void read_prg_time(MS_Message *msg){
        
        
        if(msg->data[0] == prg_ch_tmp)
        {
                RTS_Time_prg = (msg->data[5] << 24) | (msg->data[4] << 16) | (msg->data[3] << 8) | (msg->data[2] << 0);
                Seq_Time_prg = msg->data[1];
        }
        
}

//exctract matrix B from Matrix A

void extract_mtxB(uint8_t slave_req_id)
{
        if(get_mod_kind(slave_req_id, 0) == 0)
        {
                for (int m = 0; m < 23; m += 2)
                {
                        matrixBStatusX[m/2] = slaves[slave_req_id].statusMatrixA[m+1];
                }
        }
        
        
}


/******************************************************************************
//jcm-Complete
@fn SlaveInfo* getCurrentSlaveInfos(uint8_t slavenum)
@param[in] slave number to be returned SlaveInfo
@return data struct of selected slave.
@details Used in all device operation modes for master only.
******************************************************************************/
SlaveInfo* getCurrentSlaveInfo(uint8_t slavenum)
{
        if (!device_datas.isMaster) return NULL;
        if (slavenum == device_datas.device_Address) return NULL;
        if (slavenum >= MAX_NUMBER_OF_DEVICES) return NULL;
        return &slaves[slavenum];
}


void setTOsalve(void)
{
        if(Power_off_mods_enabled == 1){
                      for (int i = 1; i < MAX_NUMBER_OF_DEVICES; i++)
                        {
                                if(slaves[i].isConnected == 1){
                                        if(((slaves[i].interfaceStatus[5] >> 2) & 0x01) == 1){//power off
                                             slaves[i].isConnected = 0;
                                                
                                        }
                                                
                                        //if(((slaves[pinged_slave_X].interfaceStatus[5] >> 3) & 0x01) == 1)//stadby
                                }
                        }                                
        }
        
        if(is_F1_checkA == 0)
        {
                if (was_sl_test == 1)
                {
                        was_sl_test = 2;
                }
                
                
                        for (int i = 1; i < MAX_NUMBER_OF_DEVICES; i++)
                        {
                                if(slaves[i].isConnected == 1 && Pinged_Slave[i] == 0)
                                {
                                        
                                        Ping_Slave_Time [i] = WAIT_MS_MESSAGE_PERIOD + PING_OFFSET;
                                        
                                        
                                        if (slaves[i].missedCount > MAX_Fail_Ping && device_Status == Status_Create) {
                                                slaves[i].missedCount = 0;
                                                
                                                slaves[i].isConnected = 0;
                                                clearDeviceAddressForWireless(i);
                                        }
                                        
                                        
                                        if (slaves[i].missedCount > 5) {
                                                slaves[i].missedCount = 5;
                                        }
                                        
                                        
                                        getSlaves_Errors(i);    
                                }
                                else
                                {
                                        if(slaves[i].is_PRG_X)
                                        {
                                                getSlaves_Errors(i);                  
                                        }
//                                        #if DBG_TST_X 
                                                if(savedDatas.is_R_Prg > 0 && savedDatas.time_prog.PRG_ID[i] > 0)
                                                {
                                                        getSlaves_Errors(i);                  
                                                }
  //                                      #endif
                                }
                                
                        }
                        
                        for (int i = 1; i < MAX_NUMBER_OF_AB + 1; i++)
                        {
                                getAB_Errors (i);
                        }
                        
                        
        }
        
        Slaves_connected = 0;
        
        for (int i = 1; i < MAX_NUMBER_OF_DEVICES; i++)
        {
                Pinged_Slave[i] = 0;
                if(slaves[i].isConnected == 1)
                {
                        if(i < MAX_NUMBER_OF_DEVICES - 1){
                                Slaves_connected++;
                        }
                }
        }

}


void update_ping_times(void)
{
        
//      ping_timer_current_mod = GetCurrentSystemTime() - ping_timer_current_start;
      /*
                if(slaves[mod_pinged].isConnected == 1)
                {
                        LastPingTimeX[mod_pinged] = GetCurrentSystemTime()/PING_433_GRANULARITY - LastPingTime[mod_pinged];
                }
        */
}





void rail_stat_reset(uint8_t rst_rail)
{
        /*
                if(Rail_Stat[rst_rail][23] == 100)
                {
                        Rail_Stat[rst_rail][23] = 1;
                }
                if(Rail_Stat[rst_rail][23] == 0)
                {
                        Rail_Stat[rst_rail][23] = 1;
                }
*/
}

void update_rail_for_LCD(uint8_t mod_ID_up)
{

}

uint8_t get_rail_conStat(uint8_t mod_D_chk)
{
                //RailConSt
        // 0 - Rail not connected
        // 1 - Rail connected no igniters
        // 2 - Rail connected with igniters
        // 3 - Rail Error
        uint8_t stat_rail_tmp = 0;
        
        if(slaves[mod_D_chk].Mod_Kind == 0){
                for(int i = 0; i < 4; i++){
                       if(slaves[mod_D_chk].statusMatrixA[i] != 0){
                                stat_rail_tmp = stat_rail_tmp | 0x02;
                       }
                       if(slaves[mod_D_chk].statusMatrixA[i + 4] != 0){
                                stat_rail_tmp = stat_rail_tmp | 0x08;
                       }
                       if(slaves[mod_D_chk].statusMatrixA[i + 8] != 0){
                                stat_rail_tmp = stat_rail_tmp | 0x20;
                       }
                       if(slaves[mod_D_chk].statusMatrixA[i + 12] != 0){
                                stat_rail_tmp = stat_rail_tmp | 0x80;
                       }
                }
                return stat_rail_tmp;
        }
        else{
                return slaves[mod_D_chk].interfaceStatus[6];
        }
}

uint8_t check_R_S (uint8_t tmp_mod_chk, uint8_t tmp_rls)
{
        
        if(slaves[tmp_mod_chk].statusMatrixB[tmp_rls] == 0 && slaves[tmp_mod_chk].statusMatrixB[tmp_rls + 1] == 0)
        {
                return 0;
        }
        return 1;
        
}


uint8_t get_ch_stat (uint8_t mod_tmp_chk, int t_line, int t_col, int need_NX)
{
        uint8_t tmp_stat, bit, tmp_prg;
        
        bit =  2 * (3 - t_col);
        tmp_stat = (slaves[mod_tmp_chk].statusMatrixA[t_line] >> bit) & 0x03;
        
        if(getSlaves_isProgrammed(mod_tmp_chk) == 1)
        {
                if(t_line % 2 == 0)
                {
                        bit = 7 - t_col;
                }
                else
                {
                        bit = 3 - t_col;
                }
                
                
                tmp_prg = (slaves[mod_tmp_chk].statusMatrixB[t_line/2] >> bit) & 0x01;
                
                /* for special 16 ch module
                if(RailConNX != 0x0F || need_NX == 1)
                {
                tmp_prg = (slaves[mod_tmp_chk].statusMatrixB[t_line/2] >> bit) & 0x01;
        }
              else// special 16ch rail
                {
                if(t_line < 8)
                {
                bit = 7 - t_line;
                tmp_prg = (slaves[mod_tmp_chk].statusMatrixB[0] >> bit) & 0x01;
        }
                     else
                {
                bit = 15 - t_line;
                tmp_prg = (slaves[mod_tmp_chk].statusMatrixB[1] >> bit) & 0x01;
        }
        }
                */
                
                if(tmp_stat < 2 && tmp_prg == 1)
                {
                        tmp_stat = 4;
                }
                else if(tmp_stat > 1 && tmp_prg == 0)
                {
                        tmp_stat = 5;
                }
        }
        return tmp_stat;
}



void sendMSManualActivationX(uint8_t Line_fired, uint8_t fire_typeX)
{

}


void sendMSSequenceX(uint8_t Line_fired){

}

void sendMSSequenceUCAST(uint8_t Line_fired){

}


void sendMSSequenceBCAST(uint8_t Line_fired){

}




/******************************************************************************
@fn VS_VOID sendMSPowerEnable()
@brief broadcase power enable message to slaves
@note it will be used by master only.
******************************************************************************/
VS_VOID sendMSPowerEnable()
{
        MS_Message outputPWREN;
        
        memset(&outputPWREN, 0x00, sizeof(outputPWREN)); 
        
        
        outputPWREN.address = MASTER_SLAVE_PowerEnable;
        outputPWREN.dataLength = 0x01;
        outputPWREN.data[0] = 0x01;
        
        
                        if(is_433Mhz_en == 2){
                                                        uint8_t msg_433_tmp[MAXIMUM_433_MESSAGE_SIZE] = {0};
                                                        memset(msg_433_tmp, 0x00, sizeof(msg_433_tmp));        
                                                        
                                                        msg_433_tmp[0] = START_CHAR_433MHZ_MS;
                                                        msg_433_tmp[1] = M433_ARM_SYNC_FIRE;
                                                        msg_433_tmp[2] = M433_BROADCAST;
                                                        
                                                        msg_433_tmp[3] = outputPWREN.data[0];
                                                        
                                                        
                                                        Wireless_433_Add_msg_to_sent(msg_433_tmp);
                                        }
        
        Wireless_Send(&outputPWREN);
        
}


/******************************************************************************
@fn VS_VOID sendMSStart()
@brief broadcase start message to slaves
@note it will be used by master only.
******************************************************************************/
VS_VOID sendMSStart()
{
        MS_Message outputMStart;
        
      
        memset(&outputMStart, 0x00, sizeof(outputMStart));        
        
        outputMStart.address = MASTER_SLAVE_Start;
        outputMStart.dataLength = 0x05;
        
        
        uint32_t devicetimeinms;
        devicetimeinms = getDevicetime();
        
        outputMStart.data[0] = (devicetimeinms >> 0)  & 0xFF;
        outputMStart.data[1] = (devicetimeinms >> 8)  & 0xFF;
        outputMStart.data[2] = (devicetimeinms >> 16) & 0xFF;
        outputMStart.data[3] = (devicetimeinms >> 24) & 0xFF;
        
        outputMStart.data[4] = SMPTE_Valid_PT;
        //outputMStart.data[4] = 0;
        
        if(is_433Mhz_en == 2){
                
                send_433_start_sync_msg();
        }
        
        
        Wireless_Send(&outputMStart);
}


/******************************************************************************
@fn VS_VOID sendMSPause()
@brief broadcast Pause message to slaves.
@note it will be used by master only.
******************************************************************************/
VS_VOID sendMSPause()
{
        MS_Message outputPause;
        
        
        memset(&outputPause, 0x00, sizeof(outputPause));          
        
        outputPause.address = MASTER_SLAVE_Pause;
        outputPause.dataLength = 0x00;
        
       if(is_433Mhz_en == 2){
                uint8_t msg_433_tmp[MAXIMUM_433_MESSAGE_SIZE] = {0};
                memset(msg_433_tmp, 0x00, sizeof(msg_433_tmp));        
                
                uint32_t devicetimeinms;
                devicetimeinms = getDevicetime();
                
                msg_433_tmp[0] = START_CHAR_433MHZ_MS;
                msg_433_tmp[1] = M433_PAUSE_SYNC_FIRE;
                msg_433_tmp[2] = M433_BROADCAST;
                
                msg_433_tmp[3] = (devicetimeinms >> 0)  & 0xFF;
                msg_433_tmp[4] = (devicetimeinms >> 8)  & 0xFF;
                msg_433_tmp[5] = (devicetimeinms >> 16)  & 0xFF;
                msg_433_tmp[6] = (devicetimeinms >> 24)  & 0xFF;
                
                msg_433_tmp[7] = SMPTE_Valid_PT;
                
                Wireless_433_Add_msg_to_sent(msg_433_tmp);
        }
        
        Wireless_Send(&outputPause);
        
}

/******************************************************************************
@fn VS_VOID sendMSStop()
@brief broadcase stop message to slaves
@note it will be used by master only.
******************************************************************************/
VS_VOID sendMSStop()
{
        MS_Message outputMStop;
        
        memset(&outputMStop, 0x00, sizeof(outputMStop));          
        
        outputMStop.address = MASTER_SLAVE_Stop;
        outputMStop.dataLength = 0x00;
        
      if(is_433Mhz_en == 2){
                uint8_t msg_433_tmp[MAXIMUM_433_MESSAGE_SIZE] = {0};
                memset(msg_433_tmp, 0x00, sizeof(msg_433_tmp));        
                
                msg_433_tmp[0] = START_CHAR_433MHZ_MS;
                msg_433_tmp[1] = M433_STOP_PING_SYNC_FIRE;
                msg_433_tmp[2] = M433_BROADCAST;
                
                Wireless_433_Add_msg_to_sent(msg_433_tmp);
        }
        
        
        Wireless_Send(&outputMStop);
        
}

/******************************************************************************
@fn VS_VOID sendMSPowerDisable()
@brief broadcase power disable message to slaves
@note it will be used by master only.
******************************************************************************/
VS_VOID sendMSPowerDisable()
{
        MS_Message outputPD1;
        
        memset(&outputPD1, 0x00, sizeof(outputPD1));      
        
        outputPD1.address = MASTER_SLAVE_PowerEnable;
        outputPD1.dataLength = 0x01;
        outputPD1.data[0] = 0x00;
        
        if(is_433Mhz_en == 2){
                uint8_t msg_433_tmp[MAXIMUM_433_MESSAGE_SIZE] = {0};
                memset(msg_433_tmp, 0x00, sizeof(msg_433_tmp));        
                
                msg_433_tmp[0] = START_CHAR_433MHZ_MS;
                msg_433_tmp[1] = M433_STOP_PING_SYNC_FIRE;
                msg_433_tmp[2] = M433_BROADCAST;
                
                msg_433_tmp[3] = outputPD1.data[0];
                
                Wireless_433_Add_msg_to_sent(msg_433_tmp);
        }
        
        Wireless_Send(&outputPD1);
        
}

void send_ping_arm(uint8_t mod_ID_Ping)
{
        if(mod_ID_Ping != 0 && slaves[mod_ID_Ping].isConnected == 1)
        {
                mod_ping_snt[mod_ID_Ping] = 1;     
                
                sendMSPingA(mod_ID_Ping);
        }
        
}

uint8_t is_online(uint8_t mod_to_resp){
        return mod_ping_snt[mod_to_resp];
}


uint8_t is_ABonline(uint8_t mod_to_resp){
        return AB_ping_snt[mod_to_resp];
}




uint8_t get_mod_prg_con_stats(uint8_t mod_tmp)
{
        
        
                if(slaves[mod_tmp].slave_err != 0) //fault
                {
                        if(slaves[mod_tmp].isConnected == 1)
                        {
                                return 10; 
                        }
                        else
                        {
                                return 11; 
                        }
                }
                else if(slaves[mod_tmp].slave_wng != 0) //warning
                {

                        if(slaves[mod_tmp].isConnected == 1)
                        {
                                return 20; 
                        }
                        else
                        {
                                return 21; 
                        }
                        

                }
                else//no errors
                {
                        if(slaves[mod_tmp].isConnected == 1)
                        {
                                return 1;
                        }
                        else
                        {
                                return 0;
                        }
                }
        
        
       
        /*
        if(slaves[mod_tmp].isConnected == 1)
        {
               if(savedDatas.is_R_Prg == 1)
               {
                       if(savedDatas.time_prog.PRG_ID[mod_tmp] == 1)//is connected and programmed
                       {
                                return 101;
                       }
                       else // is connected but not programmed
                       {
                                return 1;

                       }
               }
               else // is connected but not programmed into remote
                       {
                                return 101;

                       }
        }
        else
        {
                if(savedDatas.is_R_Prg == 1)
                {
                       if(savedDatas.time_prog.PRG_ID[mod_tmp] == 1)// is programmed but not connected
                       {
                                return 201;
                       }  
                }
        }
                return 0;
        */
       

}

uint8_t update_modStats_prg(uint8_t check_x, uint8_t check_y)
{
        /*
        uint8_t need_up = 0;
        //PRG status
        //1 - connected but not programmed
        //101 - connected and programmed
        //201 - progframmed but not connected
        
       
        for(int i = check_x; i < check_y + 1; i++)
        {
               
                if(Mod_Err_Stats[i] != get_mod_prg_con_stats(i))
                {
                        Mod_Err_Stats[i] = get_mod_prg_con_stats(i);
                        need_up = 1;
                }
        }
     
        return need_up;
        */
        return 0;
}





uint8_t get_err_stat(void){
        return ERR_STAT;
}

uint16_t getSlaves_Extern_Bat(uint8_t mod_to_up)
 {
       uint16_t temp = slaves[mod_to_up].interfaceStatus[2];
       temp = temp << 14;
       temp = temp >> 6;
       temp = temp | slaves[mod_to_up].interfaceStatus[3];
       return  temp;
 }
 
 
 
uint8_t getAB_Wireless433Signal(uint8_t mod_to_up){
            uint8_t tmp_wir_sign = 0;
        
            tmp_wir_sign = AudioBoxes[mod_to_up].signal_433SM;
  
                
                if(tmp_wir_sign > 98) 
                {
                         tmp_wir_sign = 98;
                }

                tmp_wir_sign = (tmp_wir_sign + 15)/20;
                 
                return tmp_wir_sign;    

}


uint8_t getAB_WirelessCon(uint8_t mod_to_up){
            uint8_t tmp_wir_sign = 0;
        
            tmp_wir_sign = AudioBoxes[mod_to_up].isWir433_Connected;
  
             return tmp_wir_sign;    
}
 

uint8_t getSlaves_Wireless433Con(uint8_t mod_to_up){
         
        uint8_t tmp_wir_sign = 0;
         
        tmp_wir_sign = slaves[mod_to_up].is433WirelessConnected;

       return tmp_wir_sign;
}

 uint8_t getAB_WirelessSignal(uint8_t mod_to_up)
 {
         
        uint8_t tmp_wir_sign = 0;
         

             tmp_wir_sign = AudioBoxes[mod_to_up].interfaceStatus[4];
  
                
                if(tmp_wir_sign > 98) 
                {
                         tmp_wir_sign = 98;
                }

                tmp_wir_sign = (tmp_wir_sign + 15)/20;
                 
                return tmp_wir_sign;
 }
 
 
 
uint8_t getSlaves_Wireless433Signal(uint8_t mod_to_up)
 {
         
        uint8_t tmp_wir_sign = 0;
         
       if(mod_to_up == 0)  
       {
                //TO ADD wireless sginal for controll;er
       }
       else
       {

                tmp_wir_sign = slaves[mod_to_up].signal_433SM;

       }                 


                
                
                if(tmp_wir_sign > 98) 
                {
                         tmp_wir_sign = 98;
                }

                tmp_wir_sign = (tmp_wir_sign + 15)/20;
                 
                return tmp_wir_sign;
 }
 
 
 uint8_t getSlaves_WirelessSignal(uint8_t mod_to_up)
 {
         
        uint8_t tmp_wir_sign = 0;
         
       if(mod_to_up == 0)  
       {
              tmp_wir_sign = get_device_datas()->wireless_power_status;
                if (tmp_wir_sign ==  0)
                {
                        tmp_wir_sign = 98;
                } 
       }
       else
       {
             tmp_wir_sign = slaves[mod_to_up].interfaceStatus[4];
       }                 


                
                
                if(tmp_wir_sign > 98) 
                {
                         tmp_wir_sign = 98;
                }

                tmp_wir_sign = (tmp_wir_sign + 15)/20;
                 
                return tmp_wir_sign;
 }
 
 
uint8_t get_slaves_need(void)
{
        uint8_t tmp_slaves_tmp_X = 0;
        
        for(int i = 1; i < MAX_NUMBER_OF_DEVICES; i++)
        {
//                #if DBG_TST_X 
                        if(savedDatas.time_prog.PRG_ID[i] != 0)
                        {
                                tmp_slaves_tmp_X++;
                        }
  //              #endif
        
        }
        
        return tmp_slaves_tmp_X;
}

uint8_t get_slaves_remain(void)
{
        uint8_t tmp_slaves_tmp_X = 0;
        
        for(int i = 1; i < MAX_NUMBER_OF_DEVICES; i++)
        {
//                #if DBG_TST_X 
                if(savedDatas.time_prog.PRG_ID[i] == 1)
                {
                        tmp_slaves_tmp_X++;
                }
  //              #endif
        
        }
        
        return tmp_slaves_tmp_X;
}



uint8_t Errors_found(void)
{
        uint8_t errF_up = 0;
      //  getSlaves_connected();
        
        for (int i = 0; i < 4; i++)
        {
                if(Mods_Wngs[i] != 0 || Mods_Err[i])
                {
                        errF_up++;
                }
        }
        
        if(errF_up != 0 && Err_found_mods == 0)
        {
                Err_found_mods = 1;
                return 1;
        }
        else if(errF_up == 0 && Err_found_mods != 0)
        {
                Err_found_mods = 0;
                return 1;
        }
        else
        {
                return 0;
        }
}


 

uint8_t get_AP_Stat(void)
{
             uint8_t UDC_Mp3_AB = 0;
             uint8_t tmp_return = 0;
             
             UDC_Mp3_AB = read_mp3_status();
             
         
             if(UDC_Mp3_AB == 0)// USB Drive
              {
                     
                    switch(PPS_Mp3_AB)
                     {
                       case 1: tmp_return = 0x10; break; //Play
                       case 2: tmp_return = 0x11; break; //Stop
                       case 3: tmp_return = 0x0F; break; //Pause
                       case 10: tmp_return = 0x0D; break; //no File
                     }
              }
             else if(UDC_Mp3_AB == 1) // Internal Drive
              {

                    switch(PPS_Mp3_AB)
                     {
                       case 1: tmp_return = 0x03; break; //Play
                       case 2: tmp_return = 0x04; break; //Stop
                       case 3: tmp_return = 0x02; break; //Pause
                       case 10: tmp_return = 0x00; break; //no File
                     }
              }
             else // NO USB Connnected, NO Internal Drive
              {
                      tmp_return = 0x08;
              }
              
              return tmp_return;

}



void rst_prg_script(void)
{
//        #if DBG_TST_X 
        if(savedDatas.is_R_Prg == 1 && get_is_internal_drive() == 0)
        {
                //lcd update
                was_filed_open = 0;
                savedDatas.is_R_Prg = 0;
                memset(&savedDatas,  0x00 , sizeof(savedDatas));
                is_remain_Pyro =  0;
                is_remain_DMX = 0;
                tmp_DMX_ev = 0;
                tmp_pyro_ev = 0;
        }
  //      #endif
        
        /*
        else if(savedDatas.is_R_Prg == 1 && get_is_internal_drive() == 1){
                Main_Copy();
        }
        */
}


uint8_t get_is_rem_programmed(void)
{
//        #if DBG_TST_X 
                return savedDatas.is_R_Prg;
 //       #else
 //               return 0;
 //       #endif
}


uint8_t get_device_Status(void)
{
        return device_Status;
}

VS_VOID State_update()
{
             
}


void update_buffer_DMX(void)
{
        
        if (device_Status == Status_Play && is_Pause == 0 && is_Pause_Script < 2)
        {
                memcpy(buf_DMX_DMA, buf_DMX, sizeof(buf_DMX));
        }
        else if (device_Status == Status_PowerEnable && is_Pause == 0 && is_Pause_Script < 2){
                memcpy(buf_DMX_DMA, buf_DMX, sizeof(buf_DMX));
        }
        else if (device_Status == Status_Pause && is_Pause == 0 && is_Pause_Script < 2){
                memcpy(buf_DMX_DMA, buf_DMX, sizeof(buf_DMX));
        }
        else
        {
                memset(buf_DMX_DMA, 0x00, sizeof(buf_DMX_DMA));
        }
        
        
}

uint8_t get_mod_errors(void)
{
        if(slaves[current_Slave_For_D].slave_err == 0 && slaves[current_Slave_For_D].slave_wng == 0)
        {
                return 0;
        }
        else
        {
                return 1;
        }
}

uint8_t set_first_connected_slave(void)
{
        if(get_mod_kind(current_Slave_For_D, 0) != 20 || get_mod_kind(current_Slave_For_D, 0) != 21){
                current_Slave_For_D = 0;
        }
        
        if(current_Slave_For_D == 0)
        {
                for(int i = 1; i< MAX_NUMBER_OF_DEVICES; i++)
                {
                        if((get_mod_kind(i, 0) != 20 || get_mod_kind(i, 0) != 21) && Function_But != MANUAL_FIRE_FUNC)
                        {
                                if(slaves[i].isConnected > 0)
                                {
                                        current_Slave_For_D = i;
                                        was_screen_change = 5; 
                                        getSlaves_Errors(current_Slave_For_D);
                                        break;
                                }
                        }
                }
        }
        else
        {
                return 0;//remote already in a module screen
        }
        
         if(current_Slave_For_D == 0)
         {
                return 1; //error... no modules connected
         }
         return 0;
}


uint8_t get_next_connected_slave(uint8_t mod_con)
{

                for(int i = mod_con + 1; i< MAX_NUMBER_OF_DEVICES; i++)
                {
                        if(slaves[i].isConnected > 0)
                        {
                                return i;
                        }
                }
        return 0;
}


uint8_t getSlaves_isConnectedX(uint8_t adr_ID)
{
        return slaves[adr_ID].isConnected;
}




void sendMSJoinBroadcastD(void)
{
        if(device_Status > Status_Create)
        {
                MS_Message outputJBD;
                
                memset(&outputJBD, 0x00, sizeof(outputJBD));    
                
                outputJBD.address = MASTER_SLAVE_Broadcast;
                outputJBD.dataLength = 3;
                outputJBD.data[0] = adress_interface();	//This field is reserved for future using.
                outputJBD.data[1] = 0x00;	//This field is reserved for future using.
                
                outputJBD.data[2] = 0xAA;	//new dmx soft
                
                Wireless_Send(&outputJBD);
        }
        //TODO-jcm ***** error handling******
        
}



uint8_t get_mods_prg(void)// return hoe many modules are programmed
{
//        #if DBG_TST_X 
        if(savedDatas.is_R_Prg == 0) return 0;
 //       #endif
        
        uint8_t mods_prg_X = 0;
        
        for(int i = 0 + 1; i < MAX_NUMBER_OF_DEVICES; i++)
        {
         //       #if DBG_TST_X 
                if(savedDatas.time_prog.PRG_ID[i] == 1)
                {
                     mods_prg_X++;   
                }
         //       #endif
        }
        
        return mods_prg_X;
}



uint8_t get_Next_mod_Err(uint8_t id_tmp){
        
        for(int i = id_tmp + 1; i < MAX_NUMBER_OF_DEVICES - 1; i++)
        {
                if(is_UpFailed[i] > 0)
                        return i;
        }
        return 0;
}



uint8_t get_first_mod_to_prg(uint8_t last_mod)// return 0 if no modules left to prg, return ID mod for next one need to be prg
{
//        #if DBG_TST_X 
        for(int i = last_mod + 1; i < MAX_NUMBER_OF_DEVICES; i++)
        {
                if(savedDatas.time_prog.PRG_ID[i] == 1)
                {
                        if(get_ev_mod(i) > 0)
                        {
                                if(slaves[i].isConnected > 0)
                                {

                                        if(get_mod_kind(i, 0) == 0){
                                                is_UpFailed[i] = 3;//not compatible
                                        }
                                        else{
                                                return i;
                                        }

                                }
                                else{
                                         is_UpFailed[i] = 1; //NOT CONNECTED
                                }
                        }
                }
        }
  //      #endif
        return 0;
}


uint8_t get_mods_up_error(void){
        
        uint8_t reply_X = 0;
        for(int i = 1; i < MAX_NUMBER_OF_DEVICES; i++)
        {
                if(is_UpFailed[i] > 0){
                        reply_X++;
                }
                
        }
        return reply_X;
}

uint8_t get_first_mod_con(uint8_t last_mod){
        for(int i = last_mod + 1; i < MAX_NUMBER_OF_DEVICES - 1; i++)
        {
                if(slaves[i].isConnected > 0)
                                {
                                                if(get_mod_kind(i, 0) == 0){
                                                        is_UpFailed[i] = 3;
                                                }
                                                else{
                                                        return i;
                                                }
                                }
        }
        return 0;
}

void SetNextER_Up_Slave(void){
        for(int i = current_Slave_For_D_Prg + 1; i < MAX_NUMBER_OF_DEVICES; i++)
        {
                if(is_UpFailed[i] > 0){
                       current_Slave_For_D_Prg = i;
                        return;
                }
        }
        
        for(int i = 1; i < current_Slave_For_D_Prg; i++)
        {
                if(is_UpFailed[i] > 0){
                       current_Slave_For_D_Prg = i;
                        return;
                }
        }
}

void SetBackER_Up_Slave(void){
        for(int i = current_Slave_For_D_Prg - 1; i > 0; i--)
        {
                if(is_UpFailed[i] > 0){
                       current_Slave_For_D_Prg = i;
                        return;
                }
        }
        
        for(int i = MAX_NUMBER_OF_DEVICES; i > current_Slave_For_D_Prg; i--)
        {
                if(is_UpFailed[i] > 0){
                       current_Slave_For_D_Prg = i;
                        return;
                }
        }
}



//return events from a specific module
uint16_t get_ev_mod (uint8_t mod_id){
//        #if DBG_TST_X 
        return savedDatas.time_prog.count_mod_ev[mod_id];
   //     #else
   //     return 0;
    //    #endif
}

//return events from a specific module
uint16_t get_ev_Allmod (void){
//        #if DBG_TST_X 
                return savedDatas.time_prog.count;
  //      #else
  //              return 0;
 //       #endif
}


void send_delete_prg(uint8_t mod_id){

                MS_Message outputF_PRG;
        
                memset(&outputF_PRG, 0x00, sizeof(outputF_PRG));         
                
                outputF_PRG.address = mod_id + MASTER_SLAVE_Ping;
                outputF_PRG.dataLength = 16;
                
                outputF_PRG.data[0] = PING_DS;
                outputF_PRG.data[1] = 0x00;
                outputF_PRG.data[2] = 0x00;
                outputF_PRG.data[3] = 0x00;
                outputF_PRG.data[4] = 0x00;
                outputF_PRG.data[5] = 0x00;
                

                for (int i = 0; i < outputF_PRG.dataLength; i++)
                {
                        tmpprg_msg[i] = outputF_PRG.data[i];
                }
                
                
                               uint8_t msg_433_tmp[MAXIMUM_433_MESSAGE_SIZE] = {0};
                                memset(msg_433_tmp, 0x00, sizeof(msg_433_tmp));                         
                        
                        Wireless_Send(&outputF_PRG); 
                                        
}


void send_delete_prg433Mhz(uint8_t mod_id){
        if(is_433Mhz_prg_mode == 0){
                        is_433Mhz_prg_mode =1;
                        init_433Mhz_prg();
                }
        
                memset(Prg_msg_433_ext, 0x00, sizeof(Prg_msg_433_ext));
                
                     Prg_msg_433_ext[0] = START_CHAR_433MHZ_SP;
                     Prg_msg_433_ext[1] = M433_PING_DS;
                     Prg_msg_433_ext[2] = mod_id;
                
                     uint32_t hw_111 = ID_MCU2;
                     Prg_msg_433_ext[233] = 0xFF & (hw_111 >> 16);  
                     Prg_msg_433_ext[234] = 0xFF & (hw_111 >> 16);        
                     Prg_msg_433_ext[235] = 0xFF & (hw_111 >> 8);
                     Prg_msg_433_ext[236] = 0xFF & (hw_111 >> 0);
                     Prg_msg_433_ext[237] = msg_number;
                
                     uint8_t Xor_CS = 0;
                     for(int i = 0; i < MAXIMUM_433_MESSAGE_SIZE_EXT - 2; i++)
                              {
                                        Xor_CS = Xor_CS ^ Prg_msg_433_ext[i];
                              }
                     Prg_msg_433_ext[238] = Xor_CS;
                     Prg_msg_433_ext[239] = END_CHAR_433MHZ_SS;
                              
                     msg_number++;
                     if(msg_number == 255){
                            msg_number = 1;
                     }
                              
                     HAL_UART_Transmit_IT(&huart6,(uint8_t*)Prg_msg_433_ext, MAXIMUM_433_MESSAGE_SIZE_EXT);

}

void send_prg_request_433Ext(uint8_t prg_mode, uint8_t pro_mod, uint8_t prg_lineID, uint16_t prg_pos, uint8_t events_load_it){

                if(is_433Mhz_prg_mode == 0){
                        is_433Mhz_prg_mode =1;
                        init_433Mhz_prg();
                }
                
                 memset(Prg_msg_433_ext, 0x00, sizeof(Prg_msg_433_ext));
                
                 Prg_msg_433_ext[0] = START_CHAR_433MHZ_SP;
                 Prg_msg_433_ext[1] = prg_mode;
                 Prg_msg_433_ext[2] = pro_mod;
                 Prg_msg_433_ext[3] = (prg_pos >> 8)  & 0xFF;
                 Prg_msg_433_ext[4] = (prg_pos >> 0)  & 0xFF;
                 Prg_msg_433_ext[5] = events_load_it; // how many program lines are in this message
        
                if(events_load_it > 0){
                        uint32_t tmp_time = 0;
                        
                        uint16_t tmp_pos_ytr = 0;
                
                        for (int i = 0 ; i < events_load_it; i++){
                                
                                                        tmp_pos_ytr = (i * PRG_MSG_LENGTH) + 6;

                                                        Prg_msg_433_ext[tmp_pos_ytr + 0] = ScriptUpLoadEXT[i].lineID; 
                                
                                                        tmp_time = ScriptUpLoadEXT[i].time;
                                                        
                                                        Prg_msg_433_ext[tmp_pos_ytr + 1] = (tmp_time >> 24)  & 0xFF;
                                                        Prg_msg_433_ext[tmp_pos_ytr + 2] = (tmp_time >> 16)  & 0xFF;
                                                        Prg_msg_433_ext[tmp_pos_ytr + 3] = (tmp_time >> 8)  & 0xFF;
                                                        Prg_msg_433_ext[tmp_pos_ytr + 4] = (tmp_time >> 0)  & 0xFF;
                                                        
                                                        Prg_msg_433_ext[tmp_pos_ytr + 5] = ScriptUpLoadEXT[i].seqID;
                                                        
                                                        
                                                        Prg_msg_433_ext[tmp_pos_ytr + 6] = ScriptUpLoadEXT[i].DMX_value;//DMX Value
                                                        Prg_msg_433_ext[tmp_pos_ytr + 7] = ScriptUpLoadEXT[i].Rmp_Cfg; //Ramp_CFG
                                                        
                                                        Prg_msg_433_ext[tmp_pos_ytr + 8] = (ScriptUpLoadEXT[i].DMX_Ramp >> 8)  & 0xFF; //DMX Ramp Rmp_Cfg Byte 1 >> 8
                                                        Prg_msg_433_ext[tmp_pos_ytr + 9] = (ScriptUpLoadEXT[i].DMX_Ramp >> 0) & 0xFF; //DMX Ramp Rmp_Cfg Byte 0 >> 0
                                                        Prg_msg_433_ext[tmp_pos_ytr + 10] = ScriptUpLoadEXT[i].Position; // Position
                                                        Prg_msg_433_ext[tmp_pos_ytr + 11] = ScriptUpLoadEXT[i].SZone; // Safety Zones
                        }
                }
                     uint32_t hw_111 = ID_MCU2;
                     Prg_msg_433_ext[233] = 0xFF & (hw_111 >> 16);  
                     Prg_msg_433_ext[234] = 0xFF & (hw_111 >> 16);        
                     Prg_msg_433_ext[235] = 0xFF & (hw_111 >> 8);
                     Prg_msg_433_ext[236] = 0xFF & (hw_111 >> 0);
                     Prg_msg_433_ext[237] = msg_number;
                
                     uint8_t Xor_CS = 0;
                     for(int i = 0; i < MAXIMUM_433_MESSAGE_SIZE_EXT - 2; i++)
                              {
                                        Xor_CS = Xor_CS ^ Prg_msg_433_ext[i];
                              }
                     Prg_msg_433_ext[238] = Xor_CS;
                     Prg_msg_433_ext[239] = END_CHAR_433MHZ_SS;
                              
                     msg_number++;
                     if(msg_number == 255){
                            msg_number = 1;
                     }
                              
                     HAL_UART_Transmit_IT(&huart6,(uint8_t*)Prg_msg_433_ext, MAXIMUM_433_MESSAGE_SIZE_EXT);


}


void send_prg_request(uint8_t prg_mode, uint8_t pro_mod, uint8_t prg_lineID, uint16_t prg_pos)
{
    
        initTimer4X();

        if(get_mod_kind(pro_mod, 0) != 0)
        {
                
                MS_Message outputRS_prg;
                
                memset(&outputRS_prg, 0x00, sizeof(outputRS_prg));         
                
                if(prg_lineID == 255)//upload script
                {
                        if(prg_mode == PING_FP)//finish programing
                        {
                                outputRS_prg.address = pro_mod + MASTER_SLAVE_Ping;
                                outputRS_prg.dataLength = 16;
                                outputRS_prg.data[0] = prg_mode;
                                outputRS_prg.data[1] = 0;
                                outputRS_prg.data[2] = 0;
                                outputRS_prg.data[3] = 0;
                                outputRS_prg.data[4] = 0;
                                outputRS_prg.data[5] = 0;
                                outputRS_prg.data[6] = 0;
                                outputRS_prg.data[7] = 0;
                                outputRS_prg.data[8] = 0;
                                outputRS_prg.data[9] = 0;
                                outputRS_prg.data[10] = 0;
                                outputRS_prg.data[11] = 0;
                                outputRS_prg.data[12] = 0;
                                outputRS_prg.data[13] = 0;
                                outputRS_prg.data[14] = (prg_pos >> 8)  & 0xFF;
                                outputRS_prg.data[15] = (prg_pos >> 0)  & 0xFF;
                        }
                        else //programing
                        {

                                        outputRS_prg.address = pro_mod + MASTER_SLAVE_Ping;
                                        outputRS_prg.dataLength = 16;
                                        
                                        outputRS_prg.data[0] = prg_mode;
                                        //0 - ping a module for interface status and matrix status; Also can contain pyro and safet zones control
                                        //1 - ping a module for interface status;
                                        //2 - ping a module for Matrix statusA;
                                        //3 - ping a module for Matrix statusBP;
                                        //4 - Send a manual activation message
                                        //5 - request from module specific channel programing details; 
                                        //6 - Delete a pyro channel - the other channels are not affected
                                        //7 - respond to Master with programing details for a specific mode/line ID; 
                                        //8 - programing only a specific channel from a module (the other channels will not be affected);
                                        //9 - programing a channel on a module - if the module was programmed before entire script will be deleted
                                        //10 - finish programing
                                        
                                        
                                        
                                        outputRS_prg.data[1] = ScriptUpLoad.lineID; 
                                        
                                        
                                        
                                        uint32_t tmp_time = ScriptUpLoad.time;
                                        
                                        outputRS_prg.data[2] = (tmp_time >> 24)  & 0xFF;
                                        outputRS_prg.data[3] = (tmp_time >> 16)  & 0xFF;
                                        outputRS_prg.data[4] = (tmp_time >> 8)  & 0xFF;
                                        outputRS_prg.data[5] = (tmp_time >> 0)  & 0xFF;
                                        
                                        outputRS_prg.data[6] = ScriptUpLoad.seqID;
                                        
                                        
                                        outputRS_prg.data[7] = ScriptUpLoad.DMX_value;//DMX Value
                                        outputRS_prg.data[8] = ScriptUpLoad.Rmp_Cfg; //Ramp_CFG
                                        
                                        outputRS_prg.data[9] = (ScriptUpLoad.DMX_Ramp >> 8)  & 0xFF; //DMX Ramp Rmp_Cfg Byte 1 >> 8
                                        outputRS_prg.data[10] = (ScriptUpLoad.DMX_Ramp >> 0) & 0xFF; //DMX Ramp Rmp_Cfg Byte 0 >> 0
                                        outputRS_prg.data[11] = ScriptUpLoad.Position; // Position
                                        outputRS_prg.data[12] = ScriptUpLoad.SZone; // Safety Zones
                                        
                                        outputRS_prg.data[13] = 0x00;
                                        outputRS_prg.data[14] = (prg_pos >> 8)  & 0xFF;
                                        outputRS_prg.data[15] = (prg_pos >> 0)  & 0xFF;
                        }
                        
                                for (int i = 0; i < outputRS_prg.dataLength; i++)
                                {
                                        tmpprg_msg[i] = outputRS_prg.data[i];
                                }
                                
                                uint8_t msg_433_tmp[MAXIMUM_433_MESSAGE_SIZE] = {0};
                                memset(msg_433_tmp, 0x00, sizeof(msg_433_tmp));                         
                                
                                Wireless_Send(&outputRS_prg);
  
                }
                else
                {
                                
                }
                
                
                
        }
        
   
}


uint8_t get_is_seq_dis(void){
  for(int i = 0; i < MAX_SZ; i++){
          if(Safe_ZoneZ[i] > 0){
                return 1;
          }
  }
  return 0;
}

uint8_t get_PD_Stat(void){
        return is_PyroDMX_Dis[0];
}




uint8_t get_number_of_seq(void){
        uint8_t seq_tmp_s = 0;
        
        for(int i = 1; i < MAX_SEQ; i++)
        {
//                #if DBG_TST_X 
                  if(savedDatas.time_prog.count_seq[i] != 0 && savedDatas.time_prog.seq_valid[i] > 0){
                        seq_tmp_s++;
                  }
 //                 #endif
        }
        
        return seq_tmp_s;
}


/*
uint8_t get_number_of_sz(void){
        uint8_t seq_tmp_s = 0;
        
        for(int i = 1; i < MAX_SZ; i++)
        {
                  if(savedDatas.time_prog.Safe_Zone[i]){
                        seq_tmp_s++;
                  }
        }
        
        return seq_tmp_s;
}
*/


uint8_t get_SZ_stat(uint8_t sz_id){
//        #if DBG_TST_X 
                return savedDatas.time_prog.Safe_Zone[sz_id];
//        #else
//                return 0;
 //       #endif
}

uint8_t get_SZ_En_Dis(uint8_t sz_id){
        return Safe_ZoneZ[sz_id];
}



void fire_next_channel(uint8_t mod_tofire, uint8_t chan_tofire){

        MS_Message outputX1;

        memset(&outputX1, 0x00, sizeof(outputX1));         
        uint8_t tmp_sads = 7 - chan_tofire/16;
        uint8_t tmpcol_p = 0x01 << tmp_sads;
        
        
        if(is_manual_multiple_mods == 0){
        
                        outputX1.address = mod_tofire + MASTER_SLAVE_ManualActivation;
                        outputX1.dataLength = 0x08;
                
                        outputX1.data[0] = tmpcol_p;        
                        
                        outputX1.data[1] = chan_tofire % 16;

                        outputX1.data[2] =  0;
                        outputX1.data[3] =  0;
                        outputX1.data[4] =  0;
                        outputX1.data[5] =  0;                                
                        outputX1.data[6] =  0;
                        outputX1.data[7] =  get_msg_number();
        }else{
                outputX1.address = MASTER_SLAVE_PingBC;
                outputX1.dataLength = 12;
                
                outputX1.data[0] = PING_MA;        

                outputX1.data[1] = tmpcol_p;        
                        
                outputX1.data[2] = chan_tofire % 16;

                outputX1.data[3] =  0;
                outputX1.data[4] =  0;
                outputX1.data[5] =  0;
                outputX1.data[6] =  is_manual_multiple_mods;                                
                outputX1.data[7] =  get_msg_number();
                
                RESET_PING_TIMER();
        }
        

                if(outputX1.data[1] < 16 && outputX1.data[0] != 0)
                {
                        
                        if(is_433Mhz_en == 2){
                                                        uint8_t msg_433_tmp[MAXIMUM_433_MESSAGE_SIZE] = {0};
                                                        memset(msg_433_tmp, 0x00, sizeof(msg_433_tmp));        
                                                        
                                                        msg_433_tmp[0] = START_CHAR_433MHZ_MS;
                                                        msg_433_tmp[1] = M433_PING_MA;
                                                        msg_433_tmp[2] = mod_tofire;
                                                        
                                                        msg_433_tmp[3] = tmpcol_p;
                                                        msg_433_tmp[4] = chan_tofire % 16;
                                                        msg_433_tmp[5] = 0;
                                                        msg_433_tmp[6] = 0;
                                                        msg_433_tmp[7] = 0;
                                                        msg_433_tmp[8] = 0;
                                                        msg_433_tmp[9] = 0;
                                                        msg_433_tmp[10] = get_msg_number();
                                                        
                                                        msg_433_tmp[15] = is_manual_multiple_mods;
                                                     
                                                        
                                                        Wireless_433_Add_msg_to_sent(msg_433_tmp);
                                        }
                        
                        //CAN_Send(&outputX1);
                                        
                }
                         if(current_Slave_For_D != 0){                
                                Wireless_Send(&outputX1);    
                        } 
}



void extractAold(uint8_t tmp_id){
        
                slaves[tmp_id].statusMatrixA[0] = (statusMatrixA_tmp[0] & 0xC0) >> 0 | (statusMatrixA_tmp[2] & 0xC0) >> 2 | (statusMatrixA_tmp[4] & 0xC0) >> 4 | (statusMatrixA_tmp[6] & 0xC0) >> 6;
                slaves[tmp_id].statusMatrixA[1] = (statusMatrixA_tmp[8] & 0xC0) >> 0 | (statusMatrixA_tmp[10] & 0xC0) >> 2 | (statusMatrixA_tmp[12] & 0xC0) >> 4 | (statusMatrixA_tmp[14] & 0xC0) >> 6;
                slaves[tmp_id].statusMatrixA[2] = (statusMatrixA_tmp[16] & 0xC0) >> 0 | (statusMatrixA_tmp[18] & 0xC0) >> 2 | (statusMatrixA_tmp[20] & 0xC0) >> 4 | (statusMatrixA_tmp[22] & 0xC0) >> 6;
                slaves[tmp_id].statusMatrixA[3] = 0x00;
        
                slaves[tmp_id].statusMatrixA[4] = (statusMatrixA_tmp[0] & 0x30) << 2 | (statusMatrixA_tmp[2] & 0x30) >> 0 | (statusMatrixA_tmp[4] & 0x30) >> 2 | (statusMatrixA_tmp[6] & 0x30) >> 4;
                slaves[tmp_id].statusMatrixA[5] = (statusMatrixA_tmp[8] & 0x30) << 2 | (statusMatrixA_tmp[10] & 0x30) >> 0 | (statusMatrixA_tmp[12] & 0x30) >> 2 | (statusMatrixA_tmp[14] & 0x30) >> 4;
                slaves[tmp_id].statusMatrixA[6] = (statusMatrixA_tmp[16] & 0x30) << 2 | (statusMatrixA_tmp[18] & 0x30) >> 0 | (statusMatrixA_tmp[20] & 0x30) >> 2 | (statusMatrixA_tmp[22] & 0x30) >> 4;
                slaves[tmp_id].statusMatrixA[7] = 0x00;
        
        
                slaves[tmp_id].statusMatrixA[8] = (statusMatrixA_tmp[0] & 0x0C) << 4 | (statusMatrixA_tmp[2] & 0x0C) << 2 | (statusMatrixA_tmp[4] & 0x0C) >> 0 | (statusMatrixA_tmp[6] & 0x0C) >> 2;
                slaves[tmp_id].statusMatrixA[9] = (statusMatrixA_tmp[8] & 0x0C) << 4 | (statusMatrixA_tmp[10] & 0x0C) << 2 | (statusMatrixA_tmp[12] & 0x0C) >> 0 | (statusMatrixA_tmp[14] & 0x0C) >> 2;
                slaves[tmp_id].statusMatrixA[10] = (statusMatrixA_tmp[16] & 0x0C) << 4 | (statusMatrixA_tmp[18] & 0x0C) << 2 | (statusMatrixA_tmp[20] & 0x0C) >> 0 | (statusMatrixA_tmp[22] & 0x0C) >> 2;
                slaves[tmp_id].statusMatrixA[11] = 0x00;
        
                slaves[tmp_id].statusMatrixA[12] = (statusMatrixA_tmp[0] & 0x03) << 6 | (statusMatrixA_tmp[2] & 0x03) << 4 | (statusMatrixA_tmp[4] & 0x03) << 2 | (statusMatrixA_tmp[6] & 0x03) >> 0;
                slaves[tmp_id].statusMatrixA[13] = (statusMatrixA_tmp[8] & 0x03) << 6 | (statusMatrixA_tmp[10] & 0x03) << 4 | (statusMatrixA_tmp[12] & 0x03) << 2 | (statusMatrixA_tmp[14] & 0x03) >> 0;
                slaves[tmp_id].statusMatrixA[14] = (statusMatrixA_tmp[16] & 0x03) << 6 | (statusMatrixA_tmp[18] & 0x03) << 4 | (statusMatrixA_tmp[20] & 0x03) << 2 | (statusMatrixA_tmp[22] & 0x03) >> 0;
                slaves[tmp_id].statusMatrixA[15] = 0x00;
}


// send message to audiobox for Audiobox
void snd_AB_IP_msg(uint8_t mp3_command, uint8_t AB_ID)
{
        // mp3_command: 1 - STOP 
        // mp3_command: 3 - PLAY/PAUSE
        // mp3_command: 10 - Vol Down
        // mp3_command: 11 - Vol Up
        
        MS_Message outputAB;
        
        
       if(AB_ID == 100){
                outputAB.address = MASTER_SLAVE_PingBC;
                outputAB.dataLength = 12;
                
                outputAB.data[0] = AB_CMD;        

                outputAB.data[1] = 0xAA;        
                        
                outputAB.data[2] = 0;

                outputAB.data[3] =  0;
                outputAB.data[4] =  0;
                outputAB.data[5] =  mp3_command;
                outputAB.data[6] =  0;                                
                outputAB.data[7] =  get_msg_number();
       }else{
                memset(&outputAB, 0x00, sizeof(MS_Message));
                outputAB.address = AB_ID + MASTER_SLAVE_Ping;
                outputAB.dataLength = 16;
                outputAB.data[0] = AB_CMD;// means it is a AB command
                outputAB.data[1] = 0;
                outputAB.data[2] = 0;
                outputAB.data[3] = 0;
                outputAB.data[4] = 0;
                outputAB.data[5] = mp3_command; // command 
                outputAB.data[6] = 0;
                outputAB.data[7] = get_msg_number();
       }
        
        
                                if(is_433Mhz_en == 2){
                                                        uint8_t msg_433_tmp[MAXIMUM_433_MESSAGE_SIZE] = {0};
                                                        memset(msg_433_tmp, 0x00, sizeof(msg_433_tmp));        
                                                        
                                                        msg_433_tmp[0] = START_CHAR_433MHZ_MS;
                                                        msg_433_tmp[1] = M433_AB_CMDS;
                                                        msg_433_tmp[2] = AB_ID;
                                                        
                                                        msg_433_tmp[3] = outputAB.data[0];
                                                        msg_433_tmp[4] = outputAB.data[1];
                                                        msg_433_tmp[5] = outputAB.data[2];
                                                        msg_433_tmp[6] = outputAB.data[3];
                                                        msg_433_tmp[7] = outputAB.data[4];
                                                        msg_433_tmp[8] = outputAB.data[5];
                                                        msg_433_tmp[9] = outputAB.data[6];
                                                        msg_433_tmp[10] = outputAB.data[7];

                                                       
                                                        
                                                        Wireless_433_Add_msg_to_sent(msg_433_tmp);
                                        }
        
        Wireless_Send(&outputAB);
        
      
        
}

//read Audiobox player status (Enabled/Disabled)
uint8_t read_mp3AB_status(uint8_t AB_ID)
{
        uint8_t temp_a = 0;
        temp_a = slaves[AB_ID].interfaceStatus[1] & 0x18;
        
        temp_a >>= 3;
        
        return temp_a;  
}


//READ audiobox PLAY_BEEP_TIME status(Play/Pause/Stop)
uint8_t read_mp3_AB_pstatus(uint8_t AB_ID)
{
        if(AudioBoxes[AB_ID].isConnected == 1)
        {
                uint8_t temp_a = 0;
                temp_a = AudioBoxes[AB_ID].interfaceStatus[1] & 0x06;
                
                temp_a >>= 1;
                
                return temp_a;  
        }
        
        return 0;
}


VS_VOID Set_crit_errA()
{
       // crit_errA = 100;
        armchecks = 1;
        
        for (int i = 1; i < MAX_NUMBER_OF_DEVICES; i++)
        {
              if(slaves[i].isConnected == 1){
                rst_mod_stat(i);
              }
        }
        
        check_arm_st();
}



uint8_t get_slave_Fault(uint8_t slave_to_check){
        return slaves[slave_to_check].slave_err;
}

uint8_t get_slave_Wng(uint8_t slave_to_check){
        return slaves[slave_to_check].slave_wng;
}


uint8_t get_AB_Fault(uint8_t slave_to_check){
        return AudioBoxes[slave_to_check].slave_err;
}

uint8_t get_AB_Wng(uint8_t slave_to_check){
        return AudioBoxes[slave_to_check].slave_wng;
}


int get_time_zone(void){
        if(TimeZone > 14 || TimeZone < -12){
                        TimeZone = 0;
                }
        return TimeZone;
}

void set_lang_x(uint8_t lang_x){
        savedDatasCfg.Lang = lang_x;
     //   set_lang(savedDatasCfg.Lang);
}
        
void update_Config(void){
        TimeZone = savedDatasCfg.TimeZoneX;
        
        if(savedDatasCfg.AB_address == 0 || savedDatasCfg.AB_address > 10){
                device_datas.device_Address = 1;
        }else{
                device_datas.device_Address = savedDatasCfg.AB_address;
        }
        GPS_SetValue = savedDatasCfg.Start_Time;
        isGPSFTValid = savedDatasCfg.isGPSFTValid;
        
//        set_lang(savedDatasCfg.Lang);
        
        if(savedDatasCfg.Key_433Mhz != 0 && savedDatasCfg.Key_433Mhz != 0xABCD){
                tmp_433_mhz_channel = savedDatasCfg.Channel_433_Wir;
        }else{
                tmp_433_mhz_channel = DEFAULT_433_CHANNEL;
        }
        
        
        /*
        for(int i = 0; i < 4; i++){
                Network_Name[i] = savedDatasCfg.Network_NameS[i];
        }
        */
}

void inc_Time_Zone(void){
        if(TimeZone == 14){
                //do nothing
        }
        else{
                TimeZone++;
        }
        if(TimeZone > 14){
                TimeZone = 14;
        }
        savedDatasCfg.TimeZoneX = TimeZone;
}

void dec_Time_Zone(void){
        if(TimeZone == -12){
                //do nothing
        }
        else{
                TimeZone--;
        }
        
        if(TimeZone < -12){
                TimeZone = -12;
        }
        
        savedDatasCfg.TimeZoneX = TimeZone;
}


void set_sleep_state(uint8_t tmp_state){
       savedDatasCfg.is_last_sleep = tmp_state;
        //0 - full awake, 1: full sleep,          
}



void set_sleep_stateX(uint8_t tmp_state){
       savedDatasCfg.is_sleep_stat = tmp_state;
       //0 - All modules are sleep or awake, 1: Modules start go to sleep, 11: Modules start awake
}


uint8_t get_sleep_stateX(void){
        return savedDatasCfg.is_sleep_stat;
}


void save_config(uint8_t display_NO){
        savedDatasCfg.TimeZoneX = TimeZone;
        was_state_up = 0x07;

        if(display_NO == 1)
        {
               //lcd update
        }
        
        save_script_flash();
        
        was_state_up = 0x07;
        was_screen_changeM = 5;

}


void get_time_to_offset(void){
        Hours_tmp = Hours_TCo;
        Min_tmp = Min_TCo;
        Sec_tmp = Sec_TCo;
        
}

void set_Time_Code_offset(void){
        Hours_TCo = Hours_tmp;
        Min_TCo = Min_tmp;
        Sec_TCo = Sec_tmp;
        time_code_offset = 1000 * 60 * 60 * Hours_tmp + 1000 * 60 * Min_tmp + 1000 * Sec_tmp;
        
        
}

void rst_Time_Code_offset(void){
        Hours_TCo = 0;
        Min_TCo = 0;
        Sec_TCo = 0;
        time_code_offset = 0;
}

uint32_t get_time_Code_ofset(void){
        return time_code_offset;
}



void get_time_to_set(void){
        Hours_tmp = Hours;
        Min_tmp = Min;
        Sec_tmp = Sec;
}
void inc_Time_H(void){
        if(Hours_tmp < 23){
                Hours_tmp++;
        }
        else if(Hours_tmp == 23){
                Hours_tmp = 0;
        }
}
void inc_Time_M(void){
        if(Min_tmp < 59){
                Min_tmp++;
        }
        else if(Min_tmp == 59){
                Min_tmp = 0;
        }
}
void inc_Time_S(void){
        if(Sec_tmp < 59){
                Sec_tmp++;
        }
        else if(Sec_tmp == 59){
                Sec_tmp = 0;
        }
}

void dec_Time_H(void){
        if(Hours_tmp > 0){
                Hours_tmp--;
        }
        else if(Hours_tmp == 0){
                Hours_tmp = 23;
        }
}
void dec_Time_M(void){
        if(Min_tmp > 0){
                Min_tmp--;
        }
        else if(Min_tmp == 0){
                Min_tmp = 59;
        }
}
void dec_Time_S(void){
        if(Sec_tmp > 0){
                Sec_tmp--;
        }
        else if(Sec_tmp == 0){
                Sec_tmp = 59;
        }
}

void save_time_new(uint8_t tmp_hh, uint8_t tmp_mm, uint8_t tmp_ss){ //save new time

        RTC_Set(22, 01, 01, tmp_hh, tmp_mm, tmp_ss); //year, month, day, hour, min, sec)
        real_time = 1000 * 60 * 60 * tmp_hh + 1000 * 60 * tmp_mm + 1000 * tmp_ss;

}


void save_time_start_new(void){//save new time start
        isGPSFTValid = 1;
        Hours_Start = Hours_tmp;
        Min_Start = Min_tmp;
        Sec_Start = Sec_tmp;
        GPS_SetValue = 1000 * 60 * 60 * Hours_tmp + 1000 * 60 * Min_tmp + 1000 * Sec_tmp;
}

void del_time_start_new(void){//delete time start
        isGPSFTValid = 0;
        
        Hours_Start = 255;
        Min_Start = 255;
        Sec_Start = 255;
        
        GPS_SetValue = 86500000;
        
        sendMSPingBC();
}

uint8_t get_is_gps_fire_time(void){
        return isGPSFTValid;
}

uint32_t get_tus(void){
        
        if(isGPSFTValid == 0){
            return 86400000;
        }

        
        if(GPS_SetValue > real_time){
                Time_until_start = GPS_SetValue - real_time;
        }
        else{
                Time_until_start = GPS_SetValue + 86400000;
                Time_until_start = Time_until_start - real_time;
        }
        
        return Time_until_start;

}


uint8_t get_buz_stat(void){
        return !savedDatasCfg.BuzzerEnableX;
}

void set_rst_buz_stat(void){
        savedDatasCfg.BuzzerEnableX = !savedDatasCfg.BuzzerEnableX;
}


uint8_t get_pwBut_stat(void){
        return !savedDatasCfg.PB_EN;
}

void set_rst_pwBut_stat(void){
        savedDatasCfg.PB_EN = !savedDatasCfg.PB_EN;
}


uint8_t  get_BL_LCD_val(void){
        if(savedDatasCfg.LCD_Backlight < MIN_LCD_BLIGHT){
                savedDatasCfg.LCD_Backlight = DEFAULT_LCD_BLIGHT;
                return DEFAULT_LCD_BLIGHT;
        }
        else{
                return savedDatasCfg.LCD_Backlight;
        }
}

void inc_LCD_BackLight(void){
        if(savedDatasCfg.LCD_Backlight < MIN_LCD_BLIGHT){
                savedDatasCfg.LCD_Backlight = DEFAULT_LCD_BLIGHT;
        }
        else if (savedDatasCfg.LCD_Backlight < 96){
                savedDatasCfg.LCD_Backlight = savedDatasCfg.LCD_Backlight + 5;
        }
        set_lcd_BL(savedDatasCfg.LCD_Backlight);
}

void dec_LCD_BackLight(void){
        if(savedDatasCfg.LCD_Backlight < MIN_LCD_BLIGHT){
                savedDatasCfg.LCD_Backlight = DEFAULT_LCD_BLIGHT;
        }
        else if (savedDatasCfg.LCD_Backlight > MIN_LCD_BLIGHT){
                savedDatasCfg.LCD_Backlight = savedDatasCfg.LCD_Backlight - 5;
        }
        set_lcd_BL(savedDatasCfg.LCD_Backlight);
}


void refresh_mod(void){
        
        memset(slaves,0x00 , sizeof(slaves));
        memset(AudioBoxes,0x00 , sizeof(AudioBoxes));
        
        rst_wirelessDeviceTableAB();
        rst_wirelessDeviceTableMod();
        
        
        for(int i = 1; i< MAX_NUMBER_OF_DEVICES; i++)
        {
                
                slaves[i].missedCount = 0;
                slaves[i].isConnected = 0;
                slaves[i].slave_err = 0;
                slaves[i].is_PRG_MD5 = 0;
                slaves[i].isWirelessConnected = 0;
                clearDeviceAddressForWireless(i);
                reset_Duplicate_ID(i);
                
        }
        Slaves_connected = 0;
}



void wireless_config(uint8_t config_wir){
        

        //lcd update
        
        if(config_wir == 0){
                //lcd update
        }
        else{
                //lcd update
        }

        //lcd update
        
        wir_to_learn = config_wir;
        disable_ping = 1;
        
}


void set_New_Net_name(uint8_t set_rest){
        if(set_rest == 0)//Wireless standard
                {
                       NewNetwork_Name[0] = 'A'; 
                       NewNetwork_Name[1] = 'A'; 
                       NewNetwork_Name[2] = 'A'; 
                       NewNetwork_Name[3] = 'A'; 
                }
        else if(set_rest == 2)//Wireless standard
        {
                uint16_t coordinatorAddressXA = (uint16_t)(coordinatorAddressX);
                if(coordinatorAddressXA == 0){
                        coordinatorAddressXA = 0xABCD;
                }
                        
                sprintf(NewNetwork_Name, "%04X", coordinatorAddressXA);
        
        }

}


void sendMSJoinBroadcastWX(uint8_t set_rst)
{
        MS_Message outputJBWX;
        
        memset(&outputJBWX, 0x00, sizeof(outputJBWX));          
        
        outputJBWX.address = MASTER_SLAVE_Broadcast;
        outputJBWX.dataLength = 16;
        outputJBWX.data[0] = adress_interface();	//This field is reserved for future using.
        
        set_New_Net_name(set_rst);

        if(set_rst == 0)
        {
                outputJBWX.data[1] = 0xAB;// reset to standard
                outputJBWX.data[3] = NewNetwork_Name[0];//Network Name
                outputJBWX.data[4] = NewNetwork_Name[1];//Network Name
                outputJBWX.data[5] = NewNetwork_Name[2];//Network Name
                outputJBWX.data[6] = NewNetwork_Name[3];//Network Name
        }
        else if(set_rst == 1)
        {
                outputJBWX.data[1] = 0xAA;//set only EPAN
                outputJBWX.data[3] = NewNetwork_Name[0];//Network Name
                outputJBWX.data[4] = NewNetwork_Name[1];//Network Name
                outputJBWX.data[5] = NewNetwork_Name[2];//Network Name
                outputJBWX.data[6] = NewNetwork_Name[3];//Network Name
        }
        else if(set_rst == 2)
        {
                outputJBWX.data[1] = 0xAD;//set EPAN and PAN
                outputJBWX.data[3] = NewNetwork_Name[0];//Network Name
                outputJBWX.data[4] = NewNetwork_Name[1];//Network Name
                outputJBWX.data[5] = NewNetwork_Name[2];//Network Name
                outputJBWX.data[6] = NewNetwork_Name[3];//Network Name
        }
        else
        {
                outputJBWX.data[1] = 0x00;// do nothing
                outputJBWX.data[3] = NewNetwork_Name[0];//Network Name
                outputJBWX.data[4] = NewNetwork_Name[1];//Network Name
                outputJBWX.data[5] = NewNetwork_Name[2];//Network Name
                outputJBWX.data[6] = NewNetwork_Name[3];//Network Name
        }
        
        if(is_newDMXsoft == 1) outputJBWX.data[2] = 0xAA;	//new dmx soft
        else outputJBWX.data[2] = 0x00;	//new dmx soft
        
        Wireless_Send(&outputJBWX);

        //TODO-jcm ***** error handling******
        
}


uint8_t get_Display_timer(void){
        return Display_timer;
}


void save_network_name(void){
        for(int i = 0; i < 4; i++){
                savedDatasCfg.Network_NameS[i] = NewNetwork_Name[i];
        }
        save_script_flash();
}

uint8_t get_ab_con(void)
{
    
        uint8_t tmp_audi_nmb = 0;
        
        for (int i = 0; i < MAX_NUMBER_OF_AB + 1; i++){
                if(AudioBoxes[i].isConnected == 1){
                        tmp_audi_nmb++;
                }
        }
        return tmp_audi_nmb;

}


void rst_last_mod_ping(void){
        last_mod_433_pinged = 0;
        last_modNC_433_pinged = 0;
        was_433_pinged_first = 1;
}

void send_433_arm_check_msg_to_nextmod(void){
        
         check_arm_st();
        
         if(crit_errA == 0) return;
        
         uint8_t tmp_last_433_pinged = last_mod_433_pinged;
        
         for(int i = last_mod_433_pinged + 1; i < MAX_NUMBER_OF_DEVICES; i++)
                {
                        if(slaves[i].isConnected == 1 && (slaves[i].slave_err & 0x01)){
                                send_uCast_433Mhz_msg(i,0);
                                last_mod_433_pinged++;
                                if(last_mod_433_pinged == MAX_NUMBER_OF_DEVICES - 1){
                                        last_mod_433_pinged = 0;
                                }
                                break;
                        }
                } 

         if(last_mod_433_pinged == tmp_last_433_pinged){
                                 last_mod_433_pinged = 0;
         }         
}


void rst_ping_433_stat(uint8_t stantss){
        was_433_pinged_firstX = 0;
        if(stantss == 1){
                for (int i = 0; i < MAX_NUMBER_OF_DEVICES + MAX_NUMBER_OF_AB; i++)
                savedDatasCfg.sleep_mods[i] = 0;
        }
}

void send_433_msg_to_nextmod_Sleep(void){

      uint8_t is_not_all_sleep = 0;
      uint8_t tmp_last_con_433_pinged = last_con_mod_pinged_sleep;
//      uint8_t tmp_last_con_433AB_pinged = last_con_AB_pinged_sleep;
        
      
      if(was_433_pinged_firstX == 1){
              was_433_pinged_firstX = 2;
      }
      else if(was_433_pinged_firstX == 0){
                for(int i = last_con_mod_pinged_sleep + 1; i < (MAX_NUMBER_OF_DEVICES_TST - 1); i++)
                {
                         if(slaves[i].is433WirelessConnected == 1 && savedDatasCfg.sleep_mods[i] == 0){
                                        send_uCast_433Mhz_msg(i,0);
                                        last_con_mod_pinged_sleep++;
                                        break;
                                 }
                               
                 }
                if(last_con_mod_pinged_sleep == tmp_last_con_433_pinged){
                         last_con_mod_pinged_sleep = 0;
                         was_433_pinged_firstX = 1;
                }
        }
        else if(was_433_pinged_firstX == 2){
                /*
                for(int i = last_con_AB_pinged_sleep + 1; i < MAX_NUMBER_OF_AB; i++)
                {
                         if(AudioBoxes[i - 1].isWir433_Connected == 1){
                                        send_uCast_433Mhz_msg(200 + i,0);
                                        last_con_AB_pinged_sleep++;
                                 }
                                break;
                        }
                }
                if(last_con_AB_pinged_sleep == tmp_last_con_433AB_pinged){
                         last_con_AB_pinged_sleep = 0;
                         was_433_pinged_first = 0;
                         savedDatasCfg.is_sleep_stat = 0;
                         save_config(0);
                         
                       // is_433Mhz_BC_wait = 0;
                }
                */
                
               
                       last_con_AB_pinged_sleep = 0;
                       was_433_pinged_firstX = 0;
                       for(int i = 0 + 1; i < (MAX_NUMBER_OF_DEVICES_TST - 1); i++)
                       {
                                if(slaves[i].is433WirelessConnected == 1 && savedDatasCfg.sleep_mods[i] == 0){
                                        is_not_all_sleep = 1;
                                        break;
                                }
                        }
                       
                        if(is_not_all_sleep == 0){
                                savedDatasCfg.is_sleep_stat = 0;
                        }
                        
                       save_config(0);
                
        }
}



void send_TEST_msg_to_nextmod(void){
        
        uint8_t tmp_last_con_433_pinged = last_con_mod_pinged;
        
        for(int i = last_con_mod_pinged + 1; i < (5); i++)
                {
                                if(i == 1){
                                        msg_sent_on_433_was++;
                                }
                                send_uCast_433Mhz_msg(i,0);
                                last_con_mod_pinged = i;
                                is_start_pinged_433_for_TEST[i] = GetCurrentSystemTime();
                                break;
                }
                
                if(last_con_mod_pinged == tmp_last_con_433_pinged){
                         last_con_mod_pinged = 0;
                }
}

uint16_t get_ping_time(uint8_t tmp_id_AAAAA){
        return is_pinged_433_for_TEST[tmp_id_AAAAA];
}


uint16_t get_ping_nmr(uint8_t tmp_id_AAAAA){
        return (1 + msg_sent_on_433_was - is_msg_pinged_433_for_TEST[tmp_id_AAAAA]);
}


int get_signal_strength(uint8_t tmpsdff){
        return is_msg_pinged_433_for_WIR_SGN[tmpsdff];
}


void set_time_out_mod(void){
        if(slaves[last_con_mod_pinged - 1].is433WirelessConnected == 1){
              if(Ping_433_Slave_Time[last_con_mod_pinged - 1] == 0){
                        Miss_mod_ping[last_con_mod_pinged]++;
              }
        }

}



void send_4332_msg_to_nextmod_ALL(void){
      if(was_433_pinged_first == 1){
              was_433_pinged_first = 2;
      }
      else if(was_433_pinged_first == 0){
              if(last_con_mod_pinged == 0){
                        last_con_mod_pinged = 1;
              }
              
              set_time_out_mod();
              
              Ping_433_Slave_Time[last_con_mod_pinged] = 0;
              if(Ping_433_Slave_Time[last_con_mod_pinged - 1] == 0 && slaves[last_con_mod_pinged - 1].is433WirelessConnected == 1){
                        if(retry_times_on_433_xx < 3){
                                last_con_mod_pinged = last_con_mod_pinged - 1;
                        }else if(retry_times_on_433_xx == 3){
                                //module doesnot respond;
                                retry_times_on_433_xx = 0;
                        }else{
                                retry_times_on_433_xx = 0;
                        }
                        retry_times_on_433_xx++;
                
              }else{
                        retry_times_on_433_xx = 0;
              }
              
              send_uCast_433Mhz_msg(last_con_mod_pinged,0);
              time_of_LastMod_ping = GetCurrentSystemTime();
              last_ping_mod_on433Mhz = last_con_mod_pinged;
              last_con_mod_pinged++;
              
              if(last_con_mod_pinged == MAX_NUMBER_OF_DEVICES_TST - 1){
                         last_con_mod_pinged = 1;
                         was_433_pinged_first = 1;
              }
              
              if(was_full_check == 1){
                      if(slaves[last_con_mod_pinged].is433WirelessConnected == 1){
                                was_last_mod_check = 0;
                      }else{
                                was_last_mod_check++;
                              if(was_last_mod_check > MAX_MOD_NC){
                                        was_last_mod_check = 0;
                                        last_con_mod_pinged = MAX_NUMBER_OF_DEVICES - 2;
                              }
                      }
                }
        }
        else if(was_433_pinged_first == 2){
                if(last_con_AB_pinged == 0){
                        last_con_AB_pinged = 1;
                }
                
                send_uCast_433Mhz_msg(200 + last_con_AB_pinged,0);
                last_con_AB_pinged++;
                
                if(last_con_AB_pinged == MAX_NUMBER_OF_AB + 1){
                         last_con_AB_pinged = 1;
                         was_433_pinged_first = 0;
                         is_433Mhz_BC_wait = 0;
                         was_full_check = 1;
                }
                
                
                if(was_full_check == 1){
                               if(AudioBoxes[last_con_AB_pinged].isWir433_Connected == 1){
                                        was_last_AB_check = 0;
                              }else{
                                        was_last_AB_check++;
                                      if(was_last_AB_check > MAX_AB_NC){
                                                was_last_AB_check = 0;
                                                last_con_AB_pinged = MAX_NUMBER_OF_AB - 1;
                                      }
                              }
                }
                
        }
                
}




/*
void send_4332_msg_to_nextmod_ALL(void){
        

      if(was_433_pinged_first == 1){
              was_433_pinged_first = 2;
      }
      else if(was_433_pinged_first == 0){
              if(last_con_mod_pinged == 0){
                        last_con_mod_pinged = 1;
              }
              
              set_time_out_mod();
              
              Ping_433_Slave_Time[last_con_mod_pinged] = 0;
              if(Ping_433_Slave_Time[last_con_mod_pinged - 1] == 0 && slaves[last_con_mod_pinged - 1].is433WirelessConnected == 1){
                        if(retry_times_on_433_xx < 3){
                                last_con_mod_pinged = last_con_mod_pinged - 1;
                        }else if(retry_times_on_433_xx == 3){
                                //module doesnot respond;
                                retry_times_on_433_xx = 0;
                        }else{
                                retry_times_on_433_xx = 0;
                        }
                        retry_times_on_433_xx++;
                
              }else{
                        retry_times_on_433_xx = 0;
              }
              
              send_uCast_433Mhz_msg(last_con_mod_pinged,0);
              time_of_LastMod_ping = GetCurrentSystemTime();
              last_ping_mod_on433Mhz = last_con_mod_pinged;
              last_con_mod_pinged++;
              
              if(last_con_mod_pinged == MAX_NUMBER_OF_DEVICES_TST - 1){
                         last_con_mod_pinged = 1;
                         was_433_pinged_first = 1;
              }
        }
        else if(was_433_pinged_first == 2){
                if(last_con_AB_pinged == 0){
                        last_con_AB_pinged = 1;
                }
                
                send_uCast_433Mhz_msg(200 + last_con_AB_pinged,0);
                last_con_AB_pinged++;
                
                if(last_con_AB_pinged == MAX_NUMBER_OF_AB + 1){
                         last_con_AB_pinged = 1;
                         was_433_pinged_first = 0;
                         is_433Mhz_BC_wait = 0;
                }
        }
                
}
*/

void set_433_wir_strength(uint8_t mod_to, uint8_t signal_str){
        
        if(mod_to < 100){
                slaves[mod_to].signal_433SM = signal_str;
        }
        else if(mod_to > 200 && mod_to < 210){
                AudioBoxes[mod_to - AUDIO_BOX_OFFSET].signal_433SM = signal_str;
        }
        
}



void send_433_BC_msg(void){
        
        uint8_t ping_type = 0;
        
        static uint8_t is_first_pingBC = 0;
        
        if(device_Status == Status_Idle){
                ping_type = M433_STOP_PING_SYNC_FIRE;
        
        }else if(device_Status > Status_Idle){
                ping_type = M433_ARM_SYNC_FIRE;
        }
        
        
        
                        if(is_433Mhz_en == 2){
                        
                        memset(msg_433_tmpX, 0x00, sizeof(msg_433_tmpX));        
                                                        
                        uint8_t tmp_ft = 0;
                 
                                        
                        
                        msg_433_tmpX[0] = START_CHAR_433MHZ_MS;
                        msg_433_tmpX[1] = ping_type;
                        msg_433_tmpX[2] = M433_BROADCAST;
                        
                                              

                                        if(isGPSFTValid == 1 && device_Status == Status_Idle)
                                        {
                                                //        add valid seted fire hour
                                                msg_433_tmpX[3] = Hours_Start;	
                                                msg_433_tmpX[4] = Min_Start;	
                                                msg_433_tmpX[5] = Sec_Start;	
                                                tmp_ft = 0x01;	
                                        }
                                        else
                                        {
                                                //        add invalid seted fire hour
                                                msg_433_tmpX[3] = 0x00;	
                                                msg_433_tmpX[4] = 0x00;	
                                                msg_433_tmpX[5] = 0x00;	
                                                tmp_ft = 0x00;	
                                                
                                        }

                        
                        uint8_t tmp_gpsTZ = (uint8_t) TimeZone + 12;
                        tmp_gpsTZ = tmp_gpsTZ & 0x1F;
                        uint8_t tmp_gpsT = isGPSTimerValid;
                        tmp_gpsT = tmp_gpsT << 6;
                        tmp_ft = tmp_ft << 5;
                        tmp_gpsT = tmp_gpsT | tmp_ft;
                        tmp_gpsT = tmp_gpsT | tmp_gpsTZ;
                        
                        msg_433_tmpX[6] = tmp_gpsT;
                     //   msg_433_tmpX[7] = (GPS_ModtimerValue >> 0)  & 0xFF;
                     //   msg_433_tmpX[8] = (GPS_ModtimerValue >> 8)  & 0xFF;                        
                        
                        
                        msg_433_tmpX[9] = 0; //0 - Send a full status request (module reply with 2 messages), 1 send a basic status report module reply with 1 message
                        
                        msg_433_tmpX[10] = 0x01; //FIRMWARE VERSION

                        
                        
                        
                        for(int i = 0; i < 8; i++)
                                        {
                                                if(Safe_ZoneZ[i] == 1)
                                                {
                                                        msg_433_tmpX[11] = msg_433_tmpX[11] | (0x01 << i);
                                                }
                                                if(Safe_ZoneZ[i+ 8] == 1)
                                                {
                                                         msg_433_tmpX[12] =  msg_433_tmpX[12] | (0x01 << i);
                                                }
                                        }
                        
                        
                        
                        if(is_PyroDMX_Dis[0] > 0)
                                        {
                                               msg_433_tmpX[13] = is_PyroDMX_Dis[0]; // 0 - no; 1- Pyro, 2 - DMX, 3 - PYRO + DMX;
                                        }
                                        else
                                        {
                                               msg_433_tmpX[13] = is_PyroDMX_Dis[current_Slave_For_D]; // 0 - no; 1- Pyro, 2 - DMX, 3 - PYRO + DMX;
                                        }                                
                        
                        
                        msg_433_tmpX[14] = PING_OFFSET/10; // PING_OFSET in 10 of ms;
                        
                                        
                        //Controller real time
                        msg_433_tmpX[15] = (real_time >> 0)  & 0xFF;
                        msg_433_tmpX[16] = (real_time >> 8)  & 0xFF;
                        msg_433_tmpX[17] = (real_time >> 16)  & 0xFF;
                        msg_433_tmpX[18] = (real_time >> 24)  & 0xFF;
 
                        uint8_t settings_tmp = get_Lang() & 0x0F; //Language 
                        settings_tmp = settings_tmp | (Power_off_mods_enabled << 7); //Power off
                        

                        settings_tmp = settings_tmp | (savedDatasCfg.is_last_sleep  << 6); //Sleep

                                        
                        msg_433_tmpX[19] = settings_tmp; // Settings as lang, power off mods, standby
                                        //Other 
                        if(is_first_pingBC == 0){// to let modules know need to reply faster
                                msg_433_tmpX[20] = 1; // RESERVED
                                is_first_pingBC = 1;
                        }else{
                                msg_433_tmpX[20] = 0; // RESERVED
                        }
                        
                        msg_433_tmpX[21] = 0x00; // RESERVED
                        msg_433_tmpX[22] = 0x00; // RESERVED
                        msg_433_tmpX[23] = 0x00; // RESERVED
                        msg_433_tmpX[24] = 0x00; // RESERVED
                        msg_433_tmpX[25] = 0x00;
                        msg_433_tmpX[26] = 0x00;
                        
                        
                        Wireless_433_Add_msg_to_sent(msg_433_tmpX);
                }

}

void send_uCast_433Mhz_msg(uint8_t mod_to_send, uint8_t stat_req){ //mod_to_send: module ID to send ping; ping_type:M433_STOP_PING_SYNC_FIRE or M433_ARM_SYNC_FIRE stat_req: 0 - invalid, ping to mods 1 to 99, 100 - broadcast ping, // ping AB on 201 to 210
        
          if(mod_to_send > 0 && mod_to_send < 100){
                 set_last_msg_with_reply_wait();
         }                 
        uint8_t ping_type = 0;
        
        if(device_Status == Status_Idle){
                ping_type = M433_STOP_PING_SYNC_FIRE;
        
        }else if(device_Status > Status_Idle){
                ping_type = M433_ARM_SYNC_FIRE;
        }
        
                if(is_433Mhz_en == 2){
                        
                        memset(msg_433_tmpX, 0x00, sizeof(msg_433_tmpX));        
                                                        
                        uint8_t tmp_ft = 0;
                 
                                        
                        
                        msg_433_tmpX[0] = START_CHAR_433MHZ_MS;
                        msg_433_tmpX[1] = ping_type;
                        msg_433_tmpX[2] = mod_to_send;
                        
                        /*
                        if(mod_to_send == TEST_MOD_ID){
                                msg_stat_req++;
                                last_msg_timer_sntttt = GetCurrentSystemTime();
                        }
                        */
                                              

                                        if(isGPSFTValid == 1 && device_Status == Status_Idle)
                                        {
                                                //        add valid seted fire hour
                                                msg_433_tmpX[3] = Hours_Start;	
                                                msg_433_tmpX[4] = Min_Start;	
                                                msg_433_tmpX[5] = Sec_Start;	
                                                tmp_ft = 0x01;	
                                        }
                                        else
                                        {
                                                //        add invalid seted fire hour
                                                msg_433_tmpX[3] = 0x00;	
                                                msg_433_tmpX[4] = 0x00;	
                                                msg_433_tmpX[5] = 0x00;	
                                                tmp_ft = 0x00;	
                                                
                                        }

                        
                        uint8_t tmp_gpsTZ = (uint8_t) TimeZone + 12;
                        tmp_gpsTZ = tmp_gpsTZ & 0x1F;
                        uint8_t tmp_gpsT = isGPSTimerValid;
                        tmp_gpsT = tmp_gpsT << 6;
                        tmp_ft = tmp_ft << 5;
                        tmp_gpsT = tmp_gpsT | tmp_ft;
                        tmp_gpsT = tmp_gpsT | tmp_gpsTZ;
                        
                        msg_433_tmpX[6] = tmp_gpsT;
                     //   msg_433_tmpX[7] = (GPS_ModtimerValue >> 0)  & 0xFF;
                     //   msg_433_tmpX[8] = (GPS_ModtimerValue >> 8)  & 0xFF;                        
                        
                        
                        msg_433_tmpX[9] = stat_req; //0 - Send a full status request (module reply with 2 messages), 1 send a basic status report module reply with 1 message, 2 - fast sleep
                        
                        msg_433_tmpX[10] = 0x01; //FIRMWARE VERSION

                        
                        
                        
                        for(int i = 0; i < 8; i++)
                                        {
                                                if(Safe_ZoneZ[i] == 1)
                                                {
                                                        msg_433_tmpX[11] = msg_433_tmpX[11] | (0x01 << i);
                                                }
                                                if(Safe_ZoneZ[i+ 8] == 1)
                                                {
                                                         msg_433_tmpX[12] =  msg_433_tmpX[12] | (0x01 << i);
                                                }
                                        }
                        
                        
                        
                        if(is_PyroDMX_Dis[0] > 0)
                                        {
                                               msg_433_tmpX[13] = is_PyroDMX_Dis[0]; // 0 - no; 1- Pyro, 2 - DMX, 3 - PYRO + DMX;
                                        }
                                        else
                                        {
                                               msg_433_tmpX[13] = is_PyroDMX_Dis[current_Slave_For_D]; // 0 - no; 1- Pyro, 2 - DMX, 3 - PYRO + DMX;
                                        }                                
                        
                        
                        msg_433_tmpX[14] = PING_OFFSET/10; // PING_OFSET in 10 of ms;
                        
                                        
                        //Controller real time
                        msg_433_tmpX[15] = (real_time >> 0)  & 0xFF;
                        msg_433_tmpX[16] = (real_time >> 8)  & 0xFF;
                        msg_433_tmpX[17] = (real_time >> 16)  & 0xFF;
                        msg_433_tmpX[18] = (real_time >> 24)  & 0xFF;
 
                        uint8_t settings_tmp = get_Lang() & 0x0F; //Language 
                        settings_tmp = settings_tmp | (Power_off_mods_enabled << 7); //Power off


                        settings_tmp = settings_tmp | (savedDatasCfg.is_last_sleep << 6); //Sleep


                        msg_433_tmpX[19] = settings_tmp; // Settings as lang, power off mods, standby
                        msg_433_tmpX[20] = TIME_TO_SLEEP; //Time to sleep * 10000
                        msg_433_tmpX[21] = 0x00; // RESERVED
                        msg_433_tmpX[22] = 0x00; // RESERVED
                        msg_433_tmpX[23] = 0x00; // RESERVED
                        msg_433_tmpX[24] = 0x00; // RESERVED
                        msg_433_tmpX[25] = 0x00; // RESERVED
                        msg_433_tmpX[26] = 0x00; // RESERVED
                        
                        
                        Wireless_433_Add_msg_to_sent(msg_433_tmpX);
                }
}



VS_VOID send_433_start_Master_msg()
 {
       uint32_t devicetimeinms;

       devicetimeinms = getDevicetime() + 100;

         
       uint32_t real_time_MSx = 0;
       uint32_t tmp_TimeX = 0;        
       

      
       uint8_t tmp_msg_to433_snt[16] = {0};
       
       memset(tmp_msg_to433_snt, 0x00 ,sizeof(tmp_msg_to433_snt));
       
       
         
       tmp_msg_to433_snt[0] = (devicetimeinms >> 0)  & 0xFF;
       tmp_msg_to433_snt[1] = (devicetimeinms >> 8)  & 0xFF;
       tmp_msg_to433_snt[2] = (devicetimeinms >> 16) & 0xFF;
       tmp_TimeX = (devicetimeinms >> 24) & 0x0F;
       tmp_TimeX = tmp_TimeX << 4;
       tmp_TimeX = tmp_TimeX & 0xF0;        
       tmp_msg_to433_snt[6] = tmp_TimeX;        
       
       
       tmp_msg_to433_snt[3] = (real_time_MSx >> 0)  & 0xFF;
       tmp_msg_to433_snt[4] = (real_time_MSx >> 8)  & 0xFF;
       tmp_msg_to433_snt[5] = (real_time_MSx >> 16) & 0xFF;
       
       tmp_TimeX = (real_time_MSx >> 24) & 0x0F;
       tmp_msg_to433_snt[6] = tmp_msg_to433_snt[6] | tmp_TimeX;
       

                                                        uint8_t msg_433_tmp[MAXIMUM_433_MESSAGE_SIZE] = {0};
                                                        memset(msg_433_tmp, 0x00, sizeof(msg_433_tmp));        
                                                        
                                                        msg_433_tmp[0] = START_CHAR_433MHZ_MS;
                                                        msg_433_tmp[1] = M433_PLAY_ABM_FIRE;
                                                        msg_433_tmp[2] = M433_BROADCAST;
                                                        
                                                        msg_433_tmp[3] = tmp_msg_to433_snt[0];
                                                        msg_433_tmp[4] = tmp_msg_to433_snt[1];
                                                        msg_433_tmp[5] = tmp_msg_to433_snt[2];
                                                        msg_433_tmp[6] = tmp_msg_to433_snt[3];
                                                        msg_433_tmp[7] = tmp_msg_to433_snt[4];
                                                        msg_433_tmp[8] = tmp_msg_to433_snt[5];
                                                        msg_433_tmp[9] = tmp_msg_to433_snt[6];
                                                        
                                                        
                                                        Wireless_433_Add_msg_to_sent(msg_433_tmp);
}

void send_433_start_sync_msg(void)
 {
       uint32_t devicetimeinms;

       devicetimeinms = getDevicetime();

         
       uint32_t real_time_MSx = 0;
       uint32_t tmp_TimeX = 0;        
       

      
       uint8_t tmp_msg_to433_snt[16] = {0};
       
       memset(tmp_msg_to433_snt, 0x00 ,sizeof(tmp_msg_to433_snt));
       
       
         
       tmp_msg_to433_snt[0] = (devicetimeinms >> 0)  & 0xFF;
       tmp_msg_to433_snt[1] = (devicetimeinms >> 8)  & 0xFF;
       tmp_msg_to433_snt[2] = (devicetimeinms >> 16) & 0xFF;
       tmp_TimeX = (devicetimeinms >> 24) & 0x0F;
       tmp_TimeX = tmp_TimeX << 4;
       tmp_TimeX = tmp_TimeX & 0xF0;        
       tmp_msg_to433_snt[6] = tmp_TimeX;        
       
       
       tmp_msg_to433_snt[3] = (real_time_MSx >> 0)  & 0xFF;
       tmp_msg_to433_snt[4] = (real_time_MSx >> 8)  & 0xFF;
       tmp_msg_to433_snt[5] = (real_time_MSx >> 16) & 0xFF;
       
       tmp_TimeX = (real_time_MSx >> 24) & 0x0F;
       tmp_msg_to433_snt[6] = tmp_msg_to433_snt[6] | tmp_TimeX;
       
       
       tmp_msg_to433_snt[7] = 0x00;
       
       
       for(int i = 0; i < 8; i++) // Safety zones control
                {
                        if(Safe_ZoneZ[i] == 1)
                        {
                                tmp_msg_to433_snt[8] = tmp_msg_to433_snt[8] | (0x01 << i);
                        }
                        if(Safe_ZoneZ[i+ 8] == 1)
                        {
                                tmp_msg_to433_snt[9] = tmp_msg_to433_snt[9] | (0x01 << i);
                        }
                }
                
      if(is_PyroDMX_Dis[0] > 0)// Pyro DMX control
                {
                        tmp_msg_to433_snt[10] = is_PyroDMX_Dis[0]; // 0 - no; 1- Pyro, 2 - DMX, 3 - PYRO + DMX;
                }
                else
                {
                        tmp_msg_to433_snt[10] = is_PyroDMX_Dis[current_Slave_For_D]; // 0 - no; 1- Pyro, 2 - DMX, 3 - PYRO + DMX;
                }
       
       

       tmp_msg_to433_snt[11] = (DEVICE_SYNC_PERIOD/100);
       tmp_msg_to433_snt[12] = 0x00; // USed to anounce jump
                
             
       tmp_msg_to433_snt[13] = Adjusted_time;//NOT USED
     //  tmp_msg_to433_snt[14] = 0x00;//NOT USED
               
                
                
       tmp_msg_to433_snt[15] = device_Status; // for encoding system state
       
       

                                                        uint8_t msg_433_tmp[MAXIMUM_433_MESSAGE_SIZE] = {0};
                                                        memset(msg_433_tmp, 0x00, sizeof(msg_433_tmp));        
                                                        
                                                        msg_433_tmp[0] = START_CHAR_433MHZ_MS;
                                                        msg_433_tmp[1] = M433_PLAY_SYNC_FIRE;
                                                        msg_433_tmp[2] = M433_BROADCAST;
                                                        
                                                        msg_433_tmp[3] = tmp_msg_to433_snt[0];
                                                        msg_433_tmp[4] = tmp_msg_to433_snt[1];
                                                        msg_433_tmp[5] = tmp_msg_to433_snt[2];
                                                        msg_433_tmp[6] = tmp_msg_to433_snt[3];
                                                        msg_433_tmp[7] = tmp_msg_to433_snt[4];
                                                        msg_433_tmp[8] = tmp_msg_to433_snt[5];
                                                        msg_433_tmp[9] = tmp_msg_to433_snt[6];
                                                        msg_433_tmp[10] = tmp_msg_to433_snt[7];
                                                        msg_433_tmp[11] = tmp_msg_to433_snt[8];
                                                        msg_433_tmp[12] = tmp_msg_to433_snt[9];
                                                        msg_433_tmp[13] = tmp_msg_to433_snt[10];
                                                        msg_433_tmp[14] = tmp_msg_to433_snt[11];
                                                        msg_433_tmp[15] = tmp_msg_to433_snt[12];
                                                        msg_433_tmp[16] = tmp_msg_to433_snt[13];
                                                        msg_433_tmp[17] = tmp_msg_to433_snt[14];
                                                        msg_433_tmp[18] = tmp_msg_to433_snt[15];
                                                        
                                                        
                                                        Wireless_433_Add_msg_to_sent(msg_433_tmp);
                                                        
                                                       tmp_msg_to433_snt[0] = ((devicetimeinms + MSG_433_TIME_OFFSET) >> 0)  & 0xFF;
                                                       tmp_msg_to433_snt[1] = ((devicetimeinms + MSG_433_TIME_OFFSET) >> 8)  & 0xFF;
                                                       tmp_msg_to433_snt[2] = ((devicetimeinms + MSG_433_TIME_OFFSET) >> 16) & 0xFF;
                                                       tmp_TimeX = ((devicetimeinms + MSG_433_TIME_OFFSET) >> 24) & 0x0F;
                                                       tmp_TimeX = tmp_TimeX << 4;
                                                       tmp_TimeX = tmp_TimeX & 0xF0;        
                                                       tmp_msg_to433_snt[6] = tmp_TimeX;        
                                                       
                                                       
                                                       tmp_msg_to433_snt[3] = (real_time_MSx >> 0)  & 0xFF;
                                                       tmp_msg_to433_snt[4] = (real_time_MSx >> 8)  & 0xFF;
                                                       tmp_msg_to433_snt[5] = (real_time_MSx >> 16) & 0xFF;
                                                       
                                                       tmp_TimeX = (real_time_MSx >> 24) & 0x0F;
                                                       tmp_msg_to433_snt[6] = tmp_msg_to433_snt[6] | tmp_TimeX;
                                                       
                                                        msg_433_tmp[3] = tmp_msg_to433_snt[0];
                                                        msg_433_tmp[4] = tmp_msg_to433_snt[1];
                                                        msg_433_tmp[5] = tmp_msg_to433_snt[2];
                                                        msg_433_tmp[6] = tmp_msg_to433_snt[3];
                                                        msg_433_tmp[7] = tmp_msg_to433_snt[4];
                                                        msg_433_tmp[8] = tmp_msg_to433_snt[5];
                                                        msg_433_tmp[9] = tmp_msg_to433_snt[6];
                                                        
                                                        Wireless_433_Add_msg_to_sent(msg_433_tmp);
                                                        
                                                         
                                                       tmp_msg_to433_snt[0] = ((devicetimeinms + MSG_433_TIME_OFFSET) >> 0)  & 0xFF;
                                                       tmp_msg_to433_snt[1] = ((devicetimeinms + MSG_433_TIME_OFFSET) >> 8)  & 0xFF;
                                                       tmp_msg_to433_snt[2] = ((devicetimeinms + MSG_433_TIME_OFFSET) >> 16) & 0xFF;
                                                       tmp_TimeX = ((devicetimeinms + 2 * MSG_433_TIME_OFFSET) >> 24) & 0x0F;
                                                       tmp_TimeX = tmp_TimeX << 4;
                                                       tmp_TimeX = tmp_TimeX & 0xF0;        
                                                       tmp_msg_to433_snt[6] = tmp_TimeX;        
                                                       
                                                       
                                                       tmp_msg_to433_snt[3] = (real_time_MSx >> 0)  & 0xFF;
                                                       tmp_msg_to433_snt[4] = (real_time_MSx >> 8)  & 0xFF;
                                                       tmp_msg_to433_snt[5] = (real_time_MSx >> 16) & 0xFF;
                                                       
                                                       tmp_TimeX = (real_time_MSx >> 24) & 0x0F;
                                                       tmp_msg_to433_snt[6] = tmp_msg_to433_snt[6] | tmp_TimeX;
                                                       
                                                        msg_433_tmp[3] = tmp_msg_to433_snt[0];
                                                        msg_433_tmp[4] = tmp_msg_to433_snt[1];
                                                        msg_433_tmp[5] = tmp_msg_to433_snt[2];
                                                        msg_433_tmp[6] = tmp_msg_to433_snt[3];
                                                        msg_433_tmp[7] = tmp_msg_to433_snt[4];
                                                        msg_433_tmp[8] = tmp_msg_to433_snt[5];
                                                        msg_433_tmp[9] = tmp_msg_to433_snt[6];
                                                        
                                                        Wireless_433_Add_msg_to_sent(msg_433_tmp);
                                                        
                                                        
}


void send_433_Sequnce_ctrl(uint8_t line_seq_frd){

}






void send_433_chg_chan(uint8_t chan_XXX, uint8_t net_name, uint16_t key_x,  uint8_t is_custom_wir){
        
        uint8_t msg_433_tmp[MAXIMUM_433_MESSAGE_SIZE] = {0};
        memset(msg_433_tmp, 0x00, sizeof(msg_433_tmp));        
                                                  

        if(is_custom_wir == 0){
                //do nothing as are set default settings
                msg_433_tmp[4] = DEFAULT_433_NET_ID;
                msg_433_tmp[3] = DEFAULT_433_CHANNEL;
                msg_433_tmp[6] = DEFAULT_433_KEY_H;
                msg_433_tmp[5] = DEFAULT_433_KEY_L;
        }
        else if(is_custom_wir == 100){//change only channel
                //do nothing as are set default settings
                msg_433_tmp[4] = DEFAULT_433_NET_ID;
                
                msg_433_tmp[3] = chan_XXX;
                
                msg_433_tmp[6] = DEFAULT_433_KEY_H;
                msg_433_tmp[5] = DEFAULT_433_KEY_L;
                
                net_name = DEFAULT_433_NET_ID;
                key_x = DEFAULT_433_KEY_LH;
        }
        else{
                if(chan_XXX < MAXIMUM_433_CHANNEL + 1){
                        msg_433_tmp[4] = net_name;
                        msg_433_tmp[3] = chan_XXX;
                }
                else{
                        msg_433_tmp[4] = DEFAULT_433_NET_ID;
                        msg_433_tmp[3] = DEFAULT_433_CHANNEL;
                }
                
                if(key_x != 0 && key_x != 0xFFFF){
                        msg_433_tmp[6] = (key_x >> 8) & 0xFF;
                        msg_433_tmp[5] = (key_x >> 0) & 0xFF;
                }else{
                        msg_433_tmp[6] = DEFAULT_433_KEY_H;
                        msg_433_tmp[5] = DEFAULT_433_KEY_L;
                }
        }

        
        msg_433_tmp[0] = START_CHAR_433MHZ_MS;
        msg_433_tmp[1] = M433_SETTINGS;
        msg_433_tmp[2] = M433_BROADCAST;
                                                        
        msg_433_tmp[3] = chan_XXX;//channel
        msg_433_tmp[4] = net_name;//Network name
        msg_433_tmp[5] = 0xFF & (key_x >> 0);//Key low
        msg_433_tmp[6] = 0xFF & (key_x >> 8);//Key high
        

        Wireless_433_Add_msg_to_sent(msg_433_tmp);
}




uint8_t get_433_chane(void){
        return cur_channel_433Mhz;
}

void load_433_Chan(void){
        if(savedDatasCfg.Channel_433_Wir > 0 && savedDatasCfg.Channel_433_Wir < 81){
                cur_channel_433Mhz = savedDatasCfg.Channel_433_Wir;
        }else{
                cur_channel_433Mhz = DEFAULT_433_CHANNEL;
        }
        
        saved_433_channel = cur_channel_433Mhz;
}


void set_new_chan(uint8_t chna_start){
        uint8_t channel_433Mhz = 0;
        if(chna_start == 0){
                channel_433Mhz = cur_channel_433Mhz;
        }else{
                channel_433Mhz = chna_start;
                cur_channel_433Mhz = channel_433Mhz;
        }
        
        uint8_t netID_433Mhz = DEFAULT_433_NET_ID; 
        uint16_t key_433Mhz = DEFAULT_433_KEY_LH;
        
        savedDatasCfg.Net_ID_433_Wir = netID_433Mhz;
        savedDatasCfg.Channel_433_Wir = channel_433Mhz;
        savedDatasCfg.Key_433Mhz = key_433Mhz;

        
        uint8_t key_low_tmp = 0xFF & (savedDatasCfg.Key_433Mhz >> 8);
        uint8_t key_high_tmp = 0xFF & (savedDatasCfg.Key_433Mhz >> 0);
        savedDatasCfg.crc_wireless = savedDatasCfg.Channel_433_Wir ^ savedDatasCfg.Net_ID_433_Wir ^ key_low_tmp  ^ key_high_tmp;
        
        //savedDatasCfg.crc_wireless = savedDatasCfg.Net_ID_433_Wir ^ savedDatasCfg.Channel_433_Wir ^ (savedDatasCfg.Key_433Mhz << 8) ^ (savedDatasCfg.Key_433Mhz << 0);
        
        set_433_config_param(channel_433Mhz, netID_433Mhz, key_433Mhz, 1);

}

void start_433_Mhz(void){
        if(wire_433_mod_det == 0) return;
        uint8_t channel_433Mhz = savedDatasCfg.Channel_433_Wir;
        uint8_t netID_433Mhz = savedDatasCfg.Net_ID_433_Wir; 
        uint16_t key_433Mhz = savedDatasCfg.Key_433Mhz;
        uint8_t tmp_csrc_gfd = 0;
        
        
        uint8_t key_low_tmp = 0xFF & (savedDatasCfg.Key_433Mhz >> 8);
        uint8_t key_high_tmp = 0xFF & (savedDatasCfg.Key_433Mhz >> 0);
        tmp_csrc_gfd = savedDatasCfg.Channel_433_Wir ^ savedDatasCfg.Net_ID_433_Wir ^ key_low_tmp  ^ key_high_tmp;
        
        //uint8_t tmp_csrc_gfd = channel_433Mhz ^ netID_433Mhz ^ (key_433Mhz << 8) ^ (key_433Mhz << 0);
        
        
        if((savedDatasCfg.crc_wireless != tmp_csrc_gfd) || channel_433Mhz == 0 || netID_433Mhz == 0 || key_433Mhz == 0){
                set_433_config_param(channel_433Mhz, netID_433Mhz, key_433Mhz, 0);
                cur_channel_433Mhz = channel_433Mhz;
                set_save_433wir_config(0,0,0,0);
                save_config(0);
        }else{
                set_433_config_param(channel_433Mhz, netID_433Mhz, key_433Mhz, 1);
                cur_channel_433Mhz = channel_433Mhz;
        }
        
}

void set_save_433wir_config(uint8_t channel_433Mhz, uint8_t netID_433Mhz, uint16_t key_433Mhz, uint8_t is_custom_X){
        
        if(is_custom_X == 0){//default
                saved_433_channel = cur_channel_433Mhz;
                cur_channel_433Mhz = DEFAULT_433_CHANNEL;
                tmp_433_mhz_channel = DEFAULT_433_CHANNEL;
                
                savedDatasCfg.Net_ID_433_Wir = DEFAULT_433_NET_ID;
                savedDatasCfg.Channel_433_Wir = DEFAULT_433_CHANNEL;
                savedDatasCfg.Key_433Mhz = 0xFFFF & ((DEFAULT_433_KEY_H << 8) | (DEFAULT_433_KEY_L << 0));
        }
        else if(is_custom_X == 100){//default
                cur_channel_433Mhz = channel_433Mhz;
                savedDatasCfg.Net_ID_433_Wir = DEFAULT_433_NET_ID;
                savedDatasCfg.Channel_433_Wir = channel_433Mhz;
                savedDatasCfg.Key_433Mhz = 0xFFFF & ((DEFAULT_433_KEY_H << 8) | (DEFAULT_433_KEY_L << 0));
        }
        else{
                cur_channel_433Mhz = channel_433Mhz;
                savedDatasCfg.Net_ID_433_Wir = netID_433Mhz;
                savedDatasCfg.Channel_433_Wir = channel_433Mhz;
                savedDatasCfg.Key_433Mhz = key_433Mhz;
        }
        
        uint8_t key_low_tmp = 0xFF & (savedDatasCfg.Key_433Mhz >> 8);
        uint8_t key_high_tmp = 0xFF & (savedDatasCfg.Key_433Mhz >> 0);
        savedDatasCfg.crc_wireless = savedDatasCfg.Channel_433_Wir ^ savedDatasCfg.Net_ID_433_Wir ^ key_low_tmp  ^ key_high_tmp;
        
        //savedDatasCfg.crc_wireless = savedDatasCfg.Net_ID_433_Wir ^ savedDatasCfg.Channel_433_Wir ^ (savedDatasCfg.Key_433Mhz << 8) ^ (savedDatasCfg.Key_433Mhz << 0);
        

}

uint8_t get_433_ch(void){
        
        uint8_t channel_433Mhz = savedDatasCfg.Channel_433_Wir;
        uint8_t netID_433Mhz = savedDatasCfg.Net_ID_433_Wir; 
        uint16_t key_433Mhz = savedDatasCfg.Key_433Mhz;
        uint8_t tmp_csrc_gfd = 0;
        
        
        uint8_t key_low_tmp = 0xFF & (savedDatasCfg.Key_433Mhz >> 8);
        uint8_t key_high_tmp = 0xFF & (savedDatasCfg.Key_433Mhz >> 0);
        tmp_csrc_gfd = savedDatasCfg.Channel_433_Wir ^ savedDatasCfg.Net_ID_433_Wir ^ key_low_tmp  ^ key_high_tmp;
        
        
        if((savedDatasCfg.crc_wireless != tmp_csrc_gfd) || channel_433Mhz == 0 || netID_433Mhz == 0 || key_433Mhz == 0){
                return DEFAULT_433_CHANNEL;
        }else{
                return savedDatasCfg.Channel_433_Wir;
        }
}




uint8_t get_433_ch_temp(void){
        
        if(tmp_433_mhz_channel < 1 || tmp_433_mhz_channel > MAXIMUM_433_CHANNEL){
                if(savedDatasCfg.Channel_433_Wir < 1 || savedDatasCfg.Channel_433_Wir > MAXIMUM_433_CHANNEL){
                        return DEFAULT_433_CHANNEL;
                }else{
                        return savedDatasCfg.Channel_433_Wir;
                }
        }else{
                return tmp_433_mhz_channel;
        }

}




void inc_433Mhz_chan(void){
        
        if(tmp_433_mhz_channel < MINIMUM_433_CHANNEL){
                tmp_433_mhz_channel = DEFAULT_433_CHANNEL;
        }
        if(tmp_433_mhz_channel < MAXIMUM_433_CHANNEL){
                tmp_433_mhz_channel++;
        }else{
                tmp_433_mhz_channel = MINIMUM_433_CHANNEL;
        }
}


void dec_433Mhz_chan(void){
        if(tmp_433_mhz_channel < MINIMUM_433_CHANNEL){
                tmp_433_mhz_channel = DEFAULT_433_CHANNEL;
        }
        
        if(tmp_433_mhz_channel == MINIMUM_433_CHANNEL){
                tmp_433_mhz_channel = MAXIMUM_433_CHANNEL;
        }else{
                tmp_433_mhz_channel--;
        }
}


uint8_t get_mod_sleep(void){
        
        uint8_t mods_on_sleep = 0;
        for(int i = 0; i < MAX_NUMBER_OF_DEVICES + MAX_NUMBER_OF_AB; i++){
                if(savedDatasCfg.sleep_mods[i] == 1){
                        mods_on_sleep++;
                }
        }
        
        return mods_on_sleep;

}



uint8_t get_is_slave_433_connected(uint8_t slave_Xds){
        return 0; // program only via 2.4Ghz
        //return slaves[slave_Xds].is433WirelessConnected;
}



uint8_t get_433_net_id(void){
        
        uint8_t channel_433Mhz = savedDatasCfg.Channel_433_Wir;
        uint8_t netID_433Mhz = savedDatasCfg.Net_ID_433_Wir; 
        uint16_t key_433Mhz = savedDatasCfg.Key_433Mhz;
        uint8_t tmp_crc_fds = 0;
        
        
        uint8_t key_low_tmp = savedDatasCfg.Key_433Mhz >> 8;
        uint8_t key_high_tmp = savedDatasCfg.Key_433Mhz >> 0;
        tmp_crc_fds = savedDatasCfg.Channel_433_Wir ^ savedDatasCfg.Net_ID_433_Wir ^ key_low_tmp  ^ key_high_tmp;
        
        
        if((savedDatasCfg.crc_wireless != tmp_crc_fds) || channel_433Mhz == 0 || netID_433Mhz == 0 || key_433Mhz == 0){
                return DEFAULT_433_NET_ID;
        }else{
                if(savedDatasCfg.Net_ID_433_Wir == 0xFF){
                        savedDatasCfg.Net_ID_433_Wir = 0x1F;
                }
                else if(savedDatasCfg.Net_ID_433_Wir == 0x00){
                        savedDatasCfg.Net_ID_433_Wir = 0xF1;
                }
                return savedDatasCfg.Net_ID_433_Wir;
        }
}



uint16_t get_433_net_key(void){
        uint8_t channel_433Mhz = savedDatasCfg.Channel_433_Wir;
        uint8_t netID_433Mhz = savedDatasCfg.Net_ID_433_Wir; 
        uint16_t key_433Mhz = savedDatasCfg.Key_433Mhz;
        uint8_t tmp_crc_fds = 0;
        
        
        uint8_t key_low_tmp = savedDatasCfg.Key_433Mhz >> 8;
        uint8_t key_high_tmp = savedDatasCfg.Key_433Mhz >> 0;
        tmp_crc_fds = savedDatasCfg.Channel_433_Wir ^ savedDatasCfg.Net_ID_433_Wir ^ key_low_tmp  ^ key_high_tmp;
        
        
        if((savedDatasCfg.crc_wireless != tmp_crc_fds) || channel_433Mhz == 0 || netID_433Mhz == 0 || key_433Mhz == 0){
                return DEFAULT_433_KEY_LH;
        }else{
                if(savedDatasCfg.Key_433Mhz == 0xFFFF){
                        savedDatasCfg.Key_433Mhz = 0x2A4F;
                }
                else if(savedDatasCfg.Key_433Mhz == 0x0000){
                        savedDatasCfg.Key_433Mhz = 0xB8D3;
                } 
                return savedDatasCfg.Key_433Mhz;
        }
}



uint16_t get_ping_timeX(uint8_t mods){
        return (GetCurrentSystemTime()/PING_433_GRANULARITY) - LastPingTime[current_Slave_For_D];
}


uint8_t get_rev_display(void){
        return 0;
}

VS_VOID set_refresh_lcdAC(){

}


uint8_t get_Lang(void){
        return LANGx;
}

VS_VOID rst_Last_Fired (VS_VOID){

}

void set_rev_display(void)//change display orientation
 {
         /*
       savedDatasCfg.is_reverse = !savedDatasCfg.is_reverse;
       disp_clear();
       display_refresh(1);
         
         erase_flash_script();
         save_script_flash();
         */
         /*
       erase_flash_CFG();
       save_flash_CFG();
         */

}

void set_AB_interface_ID(uint8_t ttmp_adr)
{
       device_datas.device_Address = ttmp_adr;
       savedDatasCfg.AB_address = ttmp_adr;
        
         erase_flash_script();
         save_script_flash();
         /*
       erase_flash_CFG();
       save_flash_CFG();
         */
}

uint8_t get_AB_Adre(void){
        if(savedDatasCfg.AB_address == 0){
                return 1;
        }else{
                return savedDatasCfg.AB_address;
        }
}

void set_AB_Master_slave(void){
       
        if(get_AB_Adre() == MAX_AB_ID){ //AB is Master
                Can_be_Slave = 2;
                device_datas.isMaster = 1;
                SEQ_AddEvent(ev_MS_ToMaster);
        }else{
                Can_be_Slave = 1;//Slave
        }
       
}


uint8_t get_Music_box_ID(void){
        return device_datas.device_Address;
}


uint8_t was_menu_selected(void){
        return 1;
}



int8_t read_encoder_XXX(int8_t min_val, int8_t max_val, int8_t cur_val){
        
uint8_t Rot_enc_A = 0;
uint8_t Rot_enc_B = 0;
//uint8_t Rot_enc_D = 0;


uint8_t _currValueAB = 0;
static uint8_t _prevValueAB = 0;
int _counter = cur_val;
        
Rot_enc_A = HAL_GPIO_ReadPin(ROT_ENCODE_A_PORT, ROT_ENCODE_A_PIN);
Rot_enc_B = HAL_GPIO_ReadPin(ROT_ENCODE_B_PORT, ROT_ENCODE_B_PIN);
//Rot_enc_D = HAL_GPIO_ReadPin(ROT_ENCODE_D_PORT, ROT_ENCODE_D_PIN);
        
  
  
  _currValueAB = (uint8_t)Rot_enc_A << 1;
  _currValueAB |= (uint8_t)Rot_enc_B;
  
  switch((_prevValueAB | _currValueAB))
  {
 
        case 1: //0b0001:
        _counter--;
        ID_autoset_timer = 0;
        break;
        case 14: //0b1110:
        _counter--;
        ID_autoset_timer = 0;
        break;

        case 4: //0b0100: 
        _counter++;
        ID_autoset_timer = 0;
        break;
        case 11://0b1011:                                
        _counter++;
        ID_autoset_timer = 0;
        break;
  }

  _prevValueAB = _currValueAB << 2;                             //update previouse state
  
  if(_counter > max_val){
        _counter = min_val;
  }
  else if(_counter < min_val){
        _counter = max_val;
  }
  
        return _counter;

}




void read_encoder_AB(void){
        
uint8_t Rot_enc_A = 0;
uint8_t Rot_enc_B = 0;
uint8_t Rot_enc_D = 0;


uint8_t _currValueAB = 0;
static uint8_t _prevValueAB = 0;
int _counter = device_datas.device_Address;
        
Rot_enc_A = HAL_GPIO_ReadPin(ROT_ENCODE_A_PORT, ROT_ENCODE_A_PIN);
Rot_enc_B = HAL_GPIO_ReadPin(ROT_ENCODE_B_PORT, ROT_ENCODE_B_PIN);
Rot_enc_D = HAL_GPIO_ReadPin(ROT_ENCODE_D_PORT, ROT_ENCODE_D_PIN);
        
        if(Rot_enc_D == 0){
                set_AD_ID = 0;
                return;
        }
        
  
  _currValueAB = (uint8_t)Rot_enc_A << 1;
  _currValueAB |= (uint8_t)Rot_enc_B;
  
  switch((_prevValueAB | _currValueAB))
  {
 
        case 1: //0b0001:
        _counter--;
        ID_autoset_timer = 0;
        break;
        case 14: //0b1110:
        _counter--;
        ID_autoset_timer = 0;
        break;

        case 4: //0b0100: 
        _counter++;
        ID_autoset_timer = 0;
        break;
        case 11://0b1011:                                
        _counter++;
        ID_autoset_timer = 0;
        break;
  }

  _prevValueAB = _currValueAB << 2;                             //update previouse state
  
  if(_counter > MAX_AB_ID){
        _counter = MIN_AB_ID;
  }
  else if(_counter < MIN_AB_ID){
        _counter = MAX_AB_ID;
  }
  
  device_datas.device_Address = _counter;

}

uint8_t calculate_433lost_Percent(void){
        if(device_datas.isWireless433Connected == 1){
                return 99;
        }else{
                return 0;
        }

}

void send_433_mod_ping_reply(void){
                        
                        
                        memset(msg_433_tmp, 0x00, sizeof(msg_433_tmp));        
                        
                        PC_Message tmp;
        
                        memset(&tmp, 0x00, sizeof(PC_Message));
                        
                        
                        msg_433_tmp[0] = START_CHAR_433MHZ_SM;
                        msg_433_tmp[1] = M433_PING_AB_REPLY;
                        msg_433_tmp[2] = 200 + device_datas.device_Address;
        
                        getStatusResponseMessage(&tmp);
                        //send status module and channel status
                        
                                for (int i = 0; i < 8; i++)
                                {
                                        msg_433_tmp[i + 0 + 3] = tmp.data[i];

                                        
                                }
                        
                        msg_433_tmp[12] = SB_AsMaster;                                
                                
        
        Wireless_433_Add_msg_to_sent(msg_433_tmp); 

}

void send_sync_433_msg(void){
        
                        if(get_AB_Adre() != MIN_AB_ID) return;
                        memset(msg_433_tmp, 0x00, sizeof(msg_433_tmp));        
                        
                        
                        msg_433_tmp[0] = START_CHAR_433MHZ_AB;
                        msg_433_tmp[1] = M433_PING_AB_SYNC;
                        msg_433_tmp[2] = M433_BROADCAST;

                        msg_433_tmp[3] = 10;// it is valid
        
                        //audio player timer
                        msg_433_tmp[4] = (device_time >> 0)  & 0xFF;
                        msg_433_tmp[5] = (device_time >> 8)  & 0xFF;
                        msg_433_tmp[6] = (device_time >> 16) & 0xFF;
                        msg_433_tmp[7] = (device_time >> 24) & 0xFF;
        
        
                        /*
                        msg_433_tmp[8] = (time_for_calibration >> 0)  & 0xFF;
                        msg_433_tmp[9] = (time_for_calibration >> 8)  & 0xFF;
                        msg_433_tmp[10] = (time_for_calibration >> 16) & 0xFF;
                        msg_433_tmp[11] = (time_for_calibration >> 24) & 0xFF;
                        */


                        uint32_t hw_111 = ID_MCU2;
                
                        msg_433_tmp[25] = 0xFF & (hw_111 >> 24);
                        msg_433_tmp[26] = 0xFF & (hw_111 >> 16);        
                        msg_433_tmp[27] = 0xFF & (hw_111 >> 8);
                        msg_433_tmp[28] = 0xFF & (hw_111 >> 0);
        
     
                         uint8_t Xor_CS = 0;
                         for(int i = 0; i < MAXIMUM_433_MESSAGE_SIZE - 2; i++)
                         {
                                Xor_CS = Xor_CS ^ msg_433_tmp[i];
                         }
                                                             
                        msg_433_tmp[30] = Xor_CS;
                        msg_433_tmp[31] = END_CHAR_433MHZ_SS;

                        last_msg_snt_timer = GetCurrentSystemTime();
                        HAL_UART_Transmit_IT(&huart6,(uint8_t*)msg_433_tmp, MAXIMUM_MS_433_MESSAGE_SIZE);

}



void updateDisable(MS_Message *msgX){
        // bytes 8 and 9 are for Safety zones
        // Bytes 10 and 11 are for positions
        for (int i = 0; i < 8; i++)
        {
                if(((msgX->data[8] >> i) & 0x01) ==  1)
                {
                        SafeZone_Disabled[i] = 1;
                }
                else
                {
                        SafeZone_Disabled[i] = 0;
                }
                if(((msgX->data[9] >> i) & 0x01) ==  1)
                {
                        SafeZone_Disabled[i+8] = 1;
                }
                else
                {
                        SafeZone_Disabled[i+8] = 0; 
                }
                
               // is_Pause_Script = msgX->data[10];
        }
        
}

void save_new_433_config(uint8_t save_config_X){
        if(save_config_X == 0){
                savedDatasCfg.Net_ID_433_Wir = DEFAULT_433_NET_ID;
                savedDatasCfg.Channel_433_Wir = DEFAULT_433_CHANNEL;
                savedDatasCfg.Key_433Mhz = 0xFFFF & ((DEFAULT_433_KEY_H << 8) | (DEFAULT_433_KEY_L << 0));
        }else{
                savedDatasCfg.Net_ID_433_Wir = rasb_433DataBuffer[5 + 3];
                savedDatasCfg.Channel_433_Wir = rasb_433DataBuffer[8 + 3];
                savedDatasCfg.Key_433Mhz = 0xFFFF & ((rasb_433DataBuffer[10 + 3] << 8) | (rasb_433DataBuffer[11 + 3] << 0));
        }
        
        uint8_t key_low_tmp = 0xFF & (savedDatasCfg.Key_433Mhz >> 8);
        uint8_t key_high_tmp = 0xFF & (savedDatasCfg.Key_433Mhz >> 0);
        savedDatasCfg.crc_wireless = savedDatasCfg.Channel_433_Wir ^ savedDatasCfg.Net_ID_433_Wir ^ key_low_tmp  ^ key_high_tmp;
        
        save_script_flash();
        
       // save_vol();
}


// New generate Time matrix events
void generateDMXTimeMatrixEvent(uint64_t currentTimeX)
{
        
        if(!(device_Status == Status_PowerEnable || device_Status == Status_Pause)) return;
        if(is_Pause_Script == 3) return;
        if(is_Pause_Script == 2) return;
        if(is_Pause == 1) return;
        if(device_datas.programmed_interface == 0) return;
//        #if DBG_TST_X 
                if(savedDatas.time_prog.count == 0) return;
  //      #endif 
        if(currentTimeX == 0) return;
        
        
        
        
        
        F_NextTime = 0;
        F_NextChannel = 0;
       
        
        
        clear_mark();
        
        dev_prog *tmp;
        
        //selected_Line = 0;
        uint64_t currentTime = 0;
        // selected_Column = 0;
        
        
        for (uint32_t i = 0; i < TimeMatrixDepth; i++) {
                tmp = (dev_prog*)(&(savedDatas.time_prog.programme[i]));
                
                if(mark_pos == MAX_CH_SAME_TIME - 1) break;
                
                if (tmp->seqID == 0) continue;
                if(seq_validT[tmp->seqID] == 0) continue;
                
                
                if(tmp->DMX_Ramp == 100 && tmp->Rmp_Cfg == 100) continue; //Arming event, Ignore line in Play
                
                if (tmp->valid == 0 || tmp->valid == 2) continue;
                if (tmp->seqID == 0) continue;
                
                
                if(tmp->Safe_Zone > 0){
                        if(SafeZone_Disabled[tmp->Safe_Zone - 1] == 1){
                                     buf_DMX[tmp->lineID] = 0;
                                continue;
                        }
                }
                
                if (tmp->seqID > 0)
                {
                        currentTime = currentTimeX + seq_timersX[tmp->seqID] - seq_timers[tmp->seqID];
                }
                
                
               currentTime = currentTime + 200;

                
                if (currentTime < 1000) {
                        if (tmp->time < currentTime) {

                                        if (tmp->valid == 1)
                                        {
                                                if(tmp->DMX_Ramp == 0)
                                                {
                                                        buf_DMX[tmp->lineID] = tmp->DMX_value;
                                                        DMX_Ramp_Vect[tmp->lineID][0] = 0; 
                                                        DMX_Ramp_Vect[tmp->lineID][1] = 0; 
                                                        DMX_Ramp_Vect[tmp->lineID][2] = 0; 
                                                        
                                                }
                                                else
                                                {
                                                        if(tmp->Rmp_Cfg != 0 && tmp->Rmp_Cfg != 100)
                                                        {
                                                                Calc_Ramp_Step(tmp->lineID, buf_DMX[tmp->lineID], tmp->DMX_value, tmp->Rmp_Cfg, tmp->DMX_Ramp);
                                                        }
                                                }
                                        }
                                        else if (tmp->valid == 3)
                                        {
                                                buf_DMX[tmp->lineID] = 0;
                                        }
//                                        #if DBG_TST_X 
                                                savedDatas.time_prog.mark[mark_pos] = i;
  //                                      #endif
                                        mark_pos++;
                        }
                }
                else
                {
                        if (tmp->time > (currentTime - 299)) {
                                
                                if (tmp->time < currentTime) {
                                        
                                        if (tmp->valid == 1)
                                        {
                                                if(tmp->DMX_Ramp == 0)
                                                {
                                                        buf_DMX[tmp->lineID] = tmp->DMX_value;
                                                        DMX_Ramp_Vect[tmp->lineID][0] = 0; 
                                                        DMX_Ramp_Vect[tmp->lineID][1] = 0; 
                                                        DMX_Ramp_Vect[tmp->lineID][2] = 0; 
                                                        
                                                }
                                                else
                                                {
                                                        if(tmp->Rmp_Cfg != 0 && tmp->Rmp_Cfg != 100)
                                                        {
                                                                Calc_Ramp_Step(tmp->lineID, buf_DMX[tmp->lineID], tmp->DMX_value, tmp->Rmp_Cfg, tmp->DMX_Ramp);
                                                        }
                                                }
                                        }
                                        else if (tmp->valid == 3)
                                        {
                                                buf_DMX[tmp->lineID] = 0;
                                        }
//                                        #if DBG_TST_X 
                                                savedDatas.time_prog.mark[mark_pos] = i;
    //                                    #endif
                                        mark_pos++;
                                        }

                                
                        }
                }
                
                
        }
        
        if(mark_pos != 0)
        {
                clearActivedTimeMartix();
        }
        
        
}


void clear_mark(void)
{
        
        for(int i = 0; i< MAX_CH_SAME_TIME; i++)
        {
//                #if DBG_TST_X 
                        savedDatas.time_prog.mark[i] = 0;
//                #endif
        }

        mark_pos = 0;
       
}


void Calc_Ramp_StepOLD(uint8_t CH_ID_tmp, uint8_t Cur_DMX_CH_value, uint8_t New_DMX_CH_value, uint8_t DMX_Dur, uint32_t DMX_rmp_tmp)
{
        if(New_DMX_CH_value != Cur_DMX_CH_value)
        {
                long int tmp_value_XXX = 0;
                long int tmp_value_yyy = 0;
                
                is_DMX_R_valid = 1;
                
                CH_ID_tmp = CH_ID_tmp - 100;
                
                if(DMX_Dur == 1)// instant change dmx value and back to 0
                {
                        buf_DMX[CH_ID_tmp] = New_DMX_CH_value; //change value
                        DMX_Ramp_Vect[CH_ID_tmp][0] = 1000;  
                        DMX_Ramp_Vect[CH_ID_tmp][1] = DMX_rmp_tmp;  
                        DMX_Ramp_Vect[CH_ID_tmp][2] = 0;
                }
                else if(DMX_Dur == 2)// instant change dmx value and back to previous value
                {
                        buf_DMX[CH_ID_tmp] = New_DMX_CH_value; //change value
                        DMX_Ramp_Vect[CH_ID_tmp][0] = Cur_DMX_CH_value + 1000;  
                        DMX_Ramp_Vect[CH_ID_tmp][1] = DMX_rmp_tmp;  
                        DMX_Ramp_Vect[CH_ID_tmp][2] = 0;
                }           
                else if(DMX_Dur == 3)// ramp change dmx value in a specific time
                {
                        DMX_Ramp_Vect[CH_ID_tmp][0] = New_DMX_CH_value + 2000; //Value at the end
                        
                        tmp_value_yyy = New_DMX_CH_value - Cur_DMX_CH_value;
                        
                        
                        tmp_value_XXX = (int)(DMX_rmp_tmp) / tmp_value_yyy;
                        if(tmp_value_XXX == 0)
                        {
                                tmp_value_XXX = 1;
                        }
                        DMX_Ramp_Vect[CH_ID_tmp][1] = tmp_value_XXX;  //How much time need for 1 increase
                        DMX_Ramp_Vect[CH_ID_tmp][2] = tmp_value_XXX;  //How much time need for 1 increase
                        
                }
                else if(DMX_Dur == 4)// ramp change dmx value with a specific speed
                {
                        DMX_Ramp_Vect[CH_ID_tmp][0] = New_DMX_CH_value + 2000; //Value at the end
                        
                        tmp_value_yyy = New_DMX_CH_value - Cur_DMX_CH_value;
                        
                        if(DMX_rmp_tmp > 1000)
                        {
                                DMX_rmp_tmp = 1000;
                        }
                        if(DMX_rmp_tmp > 0)
                        {
                                tmp_value_XXX = 1000/ DMX_rmp_tmp;
                        }
                        
                        if(tmp_value_yyy < 0)
                        {
                                tmp_value_XXX *= (-1);
                        }
                        
                        DMX_Ramp_Vect[CH_ID_tmp][1] = tmp_value_XXX;  //How much time need for 1 increase
                        DMX_Ramp_Vect[CH_ID_tmp][2] = tmp_value_XXX;  //How much time need for 1 increase
                        
                }           
                else
                {
                        buf_DMX[CH_ID_tmp] = 0;// error DMX channel resets
                }
        }
}


void clearActivedTimeMartixOLD(void)
{
 
//#if DBG_TST_X         
        uint8_t tmp_seq_ID = 0;
        
        for (int i = 0; i < mark_pos; i++) {
                
                if (savedDatas.time_prog.programme[savedDatas.time_prog.mark[i]].valid == 3)
                {
                        savedDatas.time_prog.programme[savedDatas.time_prog.mark[i]].time = savedDatas.time_prog.programme[savedDatas.time_prog.mark[i]].time - 100 * savedDatas.time_prog.programme[savedDatas.time_prog.mark[i]].Rmp_Cfg;
                        
                        savedDatas.time_prog.programme[savedDatas.time_prog.mark[i]].valid = 0;
                        savedDatas.time_prog.count--;
                        
                        tmp_seq_ID = savedDatas.time_prog.programme[savedDatas.time_prog.mark[i]].seqID;
                        savedDatas.time_prog.count_seq[tmp_seq_ID]--;
                        
                }
                else
                {
                        if (savedDatas.time_prog.programme[savedDatas.time_prog.mark[i]].valid == 1)
                        {
                                        if(savedDatas.time_prog.programme[savedDatas.time_prog.mark[i]].Rmp_Cfg != 0 && savedDatas.time_prog.programme[savedDatas.time_prog.mark[i]].DMX_Ramp == 0)// if Column 7 (Duration) is used in old format
                                        {
                                                savedDatas.time_prog.programme[savedDatas.time_prog.mark[i]].valid = 3;
                                                savedDatas.time_prog.programme[savedDatas.time_prog.mark[i]].time = savedDatas.time_prog.programme[savedDatas.time_prog.mark[i]].time 
                                                        + 100 * savedDatas.time_prog.programme[savedDatas.time_prog.mark[i]].Rmp_Cfg;
                                                
                                                savedDatas.time_prog.count--;
                                                
                                                tmp_seq_ID = savedDatas.time_prog.programme[savedDatas.time_prog.mark[i]].seqID;
                                                savedDatas.time_prog.count_seq[tmp_seq_ID]--;
                                        }
                                        else 
                                        {
                                                savedDatas.time_prog.programme[savedDatas.time_prog.mark[i]].valid = 0;
                                                savedDatas.time_prog.count--;
                                                
                                                tmp_seq_ID = savedDatas.time_prog.programme[savedDatas.time_prog.mark[i]].seqID;
                                                savedDatas.time_prog.count_seq[tmp_seq_ID]--;
                                        }
                        }
                }    
                
                
        }
   //     #endif
        //}
        clear_mark();
}


void set_TimeCode(uint8_t timeCC){
        if(timeCC == 0){
                timeCC = 1;
        }
        savedDatasCfg.Time_CodeX = timeCC;
        
}


void Init_TimeCode(void){
        
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  GPIO_InitStruct.Pin = TIME_CODE_FSK_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(TIME_CODE_FSK_PORT, &GPIO_InitStruct);
        

        if(savedDatasCfg.Time_CodeX == 1){ //"Read SMPTE 25/30fps"
                set_time_code_read_SMPTE();//OK
        }else if(savedDatasCfg.Time_CodeX == 2){ // "Read FSK PD"
                set_time_code_read_FSK_PD();//OK
        }else if(savedDatasCfg.Time_CodeX == 3){ //"Read FSK F1"
                set_time_code_read_FSK_F1();//OK
        }else if(savedDatasCfg.Time_CodeX == 4){ //"Generate SMPTE 25fps"
                set_time_code_Gen_SMPTE_25();//OK
        }else if(savedDatasCfg.Time_CodeX == 5){ //"Generate SMPTE 30fps"
                set_time_code_Gen_SMPTE_30();//Problems because frames are overlap
        }else if(savedDatasCfg.Time_CodeX == 6){ //"Generate FSK PD"
                set_time_code_Gen_FSK_PD();
        }else if(savedDatasCfg.Time_CodeX == 7){ //"Generate FSK F1"
                set_time_code_Gen_FSK_F1();
        }else{//"Read SMPTE 25/30fps"
                set_time_code_read_SMPTE();
        }

}

VS_VOID TimeCode_Gen_start(){
        Start_TimeCode();
}

VS_VOID TimeCode_Gen_stop(){
        Stop_TimeCode();
}
        

uint8_t get_TC_BackUp(void){
        return savedDatasCfg.TimeCode_BackUp;
}

void toggle_TC_BackUp(void){
        savedDatasCfg.TimeCode_BackUp = !savedDatasCfg.TimeCode_BackUp;
}

void Start_TimeCode(void){
        if(savedDatasCfg.Time_CodeX == 1){ //"Read SMPTE 25/30fps"
                //do nothing
        }else if(savedDatasCfg.Time_CodeX == 2){ // "Read FSK PD"
                //do nothing
        }else if(savedDatasCfg.Time_CodeX == 3){ //"Read FSK F1"
                //do nothing
        }else if(savedDatasCfg.Time_CodeX == 4){ //"Generate SMPTE 25fps"
                start_time_code_Gen_SMPTE();
                Time_code_Started = 1;
        }else if(savedDatasCfg.Time_CodeX == 5){ //"Generate SMPTE 30fps"
                start_time_code_Gen_SMPTE();
                Time_code_Started = 1;
        }else if(savedDatasCfg.Time_CodeX == 6){ //"Generate FSK PD"
                //do nothing
        }else if(savedDatasCfg.Time_CodeX == 7){ //"Generate FSK F1"
                //do nothing
        }else{//"Read SMPTE 25/30fps"
                //do nothing
        }
}

void Stop_TimeCode(void){
        if(savedDatasCfg.Time_CodeX == 1){ //"Read SMPTE 25/30fps"
                //do nothing
        }else if(savedDatasCfg.Time_CodeX == 2){ // "Read FSK PD"
                //do nothing
        }else if(savedDatasCfg.Time_CodeX == 3){ //"Read FSK F1"
                //do nothing
        }else if(savedDatasCfg.Time_CodeX == 4){ //"Generate SMPTE 25fps"
                stop_time_code_Gen_SMPTE();
                Time_code_Started = 0;
        }else if(savedDatasCfg.Time_CodeX == 5){ //"Generate SMPTE 30fps"
                stop_time_code_Gen_SMPTE();
                Time_code_Started = 0;
        }else if(savedDatasCfg.Time_CodeX == 6){ //"Generate FSK PD"
                //do nothing
        }else if(savedDatasCfg.Time_CodeX == 7){ //"Generate FSK F1"
                //do nothing
        }else{//"Read SMPTE 25/30fps"
                //do nothing
        }
}

void Disable_TC_SMPTE_Gen_Pin(void){
        
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  
  GPIO_InitStruct.Pin = CTR_TC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(CTR_TC_GPIO_Port, &GPIO_InitStruct);

}


void Enable_TC_SMPTE_Gen_Pin(void){
        
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  
  GPIO_InitStruct.Pin = CTR_TC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(CTR_TC_GPIO_Port, &GPIO_InitStruct);

}



uint8_t get_Time_CodeX(void){
        if(savedDatasCfg.Time_CodeX == 0){
                return 1;
        }else{
                return savedDatasCfg.Time_CodeX;
        }
}


uint8_t are_any_DMX_Evants(void){
        return savedDatas.is_DMX_events;

}


void save_Start_time_Config(void){
        savedDatasCfg.Start_Time = GPS_SetValue;
        savedDatasCfg.isGPSFTValid = isGPSFTValid;
}


void save_TimeZone(int8_t tmpTZxzx){
        savedDatasCfg.TimeZoneX = tmpTZxzx;
        TimeZone = tmpTZxzx;
}

void set_433Mhz_config(MS_Message *tmsg){

        tmp_crc_fds = tmsg->data[18] ^ tmsg->data[19] ^ tmsg->data[20] ^ tmsg->data[21];
        if(tmsg->data[22] == tmp_crc_fds && tmsg->data[18] != 0  && tmsg->data[19] != 0  && tmsg->data[20] != 0  && tmsg->data[21] != 0 && tmsg->data[22] != 0){
                uint16_t key_tmpCC = (0x00FF & tmsg->data[20] << 0) | (0xFF00 & tmsg->data[21] << 8);
                        if(tmsg->data[18] != savedDatasCfg.Channel_433_Wir || tmsg->data[19] != savedDatasCfg.Net_ID_433_Wir || key_tmpCC != savedDatasCfg.Key_433Mhz){
                                set_save_433wir_config(tmsg->data[18], tmsg->data[19], key_tmpCC,1);
                                save_config(0);
                                set_433_config_param(tmsg->data[18], tmsg->data[19], key_tmpCC, 1);
                                
                        }
        }
}


void set_default_settings(void){

        memset(&savedDatasCfg, 0x00, sizeof(savedDatasCfg));
        savedDatasCfg.AB_address = 1;
        savedDatasCfg.Audio_Volume = 60;
     
        
        //Time code
        //savedDatasCfg.SMPTE_TC_OUT
        
        //868Mhz
        //savedDatasCfg.Channel_433_Wir = 18;
        //savedDatasCfg.Key_433Mhz
        //savedDatasCfg.crc_wireless = ??
        //savedDatasCfg.Mast_ID_433Mhz
        //savedDatasCfg.Net_ID_433_Wir
        //savedDatasCfg.is_Custom_433_Wir
        
        //2.4Ghz
        //savedDatasCfg.CW_Address = 0; not need
        //savedDatasCfg.Network_NameS
        //savedDatasCfg.is_Custom_Wir
        
        //start time
        //savedDatasCfg.Start_Time = 0;
        //savedDatasCfg.isGPSFTValid
        
        //BUZZER
        //savedDatasCfg.BuzzerEnableX = 1;
        
        
        //savedDatasCfg.D_Mode
        //savedDatasCfg.GPS_HoursS
        //savedDatasCfg.is_433R_ARM
        //savedDatasCfg.is_433R_CC
        //savedDatasCfg.is_433R_EN
        //savedDatasCfg.is_Err_det
        //savedDatasCfg.is_ET
        //savedDatasCfg.is_last_sleep
        //savedDatasCfg.is_reverse
        //savedDatasCfg.is_sleep_stat
        //savedDatasCfg.Lang
        //savedDatasCfg.LCD_Backlight
        //savedDatasCfg.PB_EN
}


uint8_t get_is_custom(void){
        if(get_433_net_id() != DEFAULT_433_NET_ID || get_433_net_key() != DEFAULT_433_KEY_LH){
                return 1;
        }else{
                return 0;
        }
}

uint8_t get_wir_crc(void){
        return savedDatasCfg.crc_wireless;
}


uint8_t check_network_name(void){
        
        Network_Name[5] = 0;
        
        int numbertttt = (int)strtol(Network_Name, NULL, 16);

                if(savedDatasCfg.Key_433Mhz != numbertttt && savedDatasCfg.Key_433Mhz != 0xABCD){
                        return 0;
                }

        return 1;
}




void set_433Mhz_configNew(MS_Message *tmsg){

        for(int i = 0; i < 12; i++){
                defaultX_set3_433C[i] = tmsg->data[i+3];
        }

        
        savedDatasCfg.Net_ID_433_Wir = tmsg->data[5 + 3];
        savedDatasCfg.Channel_433_Wir = tmsg->data[8 + 3];
        savedDatasCfg.Key_433Mhz = 0xFFFF & ((tmsg->data[10 + 3] << 8) | (tmsg->data[11 + 3] << 0));
        
        
        uint8_t key_low_tmp = 0xFF & (savedDatasCfg.Key_433Mhz >> 8);
        uint8_t key_high_tmp = 0xFF & (savedDatasCfg.Key_433Mhz >> 0);
        savedDatasCfg.crc_wireless = savedDatasCfg.Channel_433_Wir ^ savedDatasCfg.Net_ID_433_Wir ^ key_low_tmp  ^ key_high_tmp;
        
        
        HAL_GPIO_WritePin(RES_B_PORT, RES_B_PIN, GPIO_PIN_SET);
        
        MX_433_TX_Init(9600);
        HAL_Delay(100);
        
        HAL_UART_Transmit_IT(&huart6,(uint8_t*) defaultX_set3_433C, 12);

        HAL_Delay(100);
        
        MX_433_TX_Init(115200);
        HAL_Delay(100);
        
        HAL_GPIO_WritePin(RES_B_PORT, RES_B_PIN, GPIO_PIN_RESET);
}



void save_433_config_param(uint8_t netch_tmp, uint8_t netid_tmp, uint16_t netkey_tmp, uint8_t tmp_is_cust){
        
        if(tmp_is_cust == 1){
                if(netch_tmp == savedDatasCfg.Channel_433_Wir && netid_tmp == savedDatasCfg.Net_ID_433_Wir && netkey_tmp == savedDatasCfg.Key_433Mhz){
                        return;
                }
        }else{
                if(savedDatasCfg.Channel_433_Wir == DEFAULT_433_CHANNEL && savedDatasCfg.Net_ID_433_Wir == DEFAULT_433_NET_ID && savedDatasCfg.Key_433Mhz == DEFAULT_433_KEY_LH){
                        return;
                }
        }
        
        if(tmp_is_cust == 0 || netch_tmp == 0 || netch_tmp > 80){
                savedDatasCfg.Channel_433_Wir = DEFAULT_433_CHANNEL;
                savedDatasCfg.Net_ID_433_Wir = DEFAULT_433_NET_ID;
                savedDatasCfg.Key_433Mhz = DEFAULT_433_KEY_LH;
        }else{
                savedDatasCfg.Channel_433_Wir = netch_tmp;
                savedDatasCfg.Net_ID_433_Wir = netid_tmp;
                savedDatasCfg.Key_433Mhz = netkey_tmp;
        }
        
        
        
        uint8_t key_low_tmp = 0xFF & (savedDatasCfg.Key_433Mhz >> 8);
        uint8_t key_high_tmp = 0xFF & (savedDatasCfg.Key_433Mhz >> 0);
        savedDatasCfg.crc_wireless = savedDatasCfg.Channel_433_Wir ^ savedDatasCfg.Net_ID_433_Wir ^ key_low_tmp  ^ key_high_tmp;
        
        set_433_config_param(netch_tmp, netid_tmp, netkey_tmp, 1);//set 433mhz wireless
        config_PAN_EPAN_new();//set 2.4Ghz wireless

        save_script_flash();
}

void set_itf_ID(uint8_t tmp_IDnumber)
{
        savedDatas.deviceAddress = tmp_IDnumber;
        device_datas.device_Address = tmp_IDnumber;
}



void Set_DMX_R(void)
{
        if(savedDatas.is_DMX_events == 1)
        {
                is_DMX_Pass = 1;
        }
}



VS_VOID RST_ARM_DMX(){
        memset(buf_DMX, 0x00, sizeof(buf_DMX));
        memset(buf_DMX_ARM, 0x00, sizeof(buf_DMX_ARM));
        memset(buf_DMX_DMA, 0x00, sizeof(buf_DMX_DMA));
}


void pwr_off_mod(void){//power off ab

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /*Configure GPIO pin : PWR_Pin */
  GPIO_InitStruct.Pin = PWR_BUT_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(PWR_BUT_PORT, &GPIO_InitStruct);
        
  HAL_GPIO_WritePin(PWR_BUT_PORT, PWR_BUT_PIN, GPIO_PIN_RESET);
  

}


VS_VOID STOP_TIMECODE(){
        if(Time_code_Started == 1){
                Stop_TimeCode();
                Time_code_Started = 0;
        }
}

