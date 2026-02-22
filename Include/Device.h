/***************************************************************************//**
 * @file     Device.h
 * @brief    Core device operation functions header file
 ******************************************************************************/

#ifndef _DEVICE_H_
#define _DEVICE_H_

#include "options.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx.h"

/**
@struct _devMsg
@brief Structure holding the device message data
*/
typedef struct _devMsg{
	uint8_t address; /*< Device address. It is 32-bit wide so we can use it with all CAN ID's*/
	uint8_t dataLength; /*< Size of the data in the "data" array*/
	uint8_t command;
	uint8_t data[MAX_MSG_SIZE]; /*< Array containig the messsage data payload */
}PC_Message;

typedef struct _devMsg1{
	uint16_t address; /*< Device address. It is 32-bit wide so we can use it with all CAN ID's*/
	uint8_t dataLength; /*< Size of the data in the "data" array*/
	uint8_t data[MAX_MSG_SIZE]; /*< Array containig the messsage data payload */
}MS_Message;

#define MAX_DEV_BUFFER		8

enum channels{
	Channel_None = 0,
	Channel_Wireless,
	Channel_CAN,
	Channel_Both,
};

typedef struct _dev_FIFO{
	uint8_t rdPtr;
	uint8_t wrPtr;
	MS_Message buffer[MAX_DEV_BUFFER];
	uint8_t revChannel[MAX_DEV_BUFFER];
	uint64_t revTime[MAX_DEV_BUFFER];
} dev_FIFO;

//*************************** manage slaves (for only master) ************
__packed typedef struct _slave{
	uint8_t isConnected;
	//uint8_t isCANConnected; // put it in isConnected and remove it
       // uint8_t isWirelessConnected; // put it in isConnected and remove it
        
	uint8_t slave_err;
        uint8_t slave_wng;


	uint8_t Mod_Kind; // rprg 10.07.2017
        uint8_t matrixARev;
        uint8_t matrixAStatusX;        
        uint8_t DMX_Pyro_Stat; //Enable/Disable DMX, Pyro
        
        
        uint8_t isArmedK;
        uint8_t missedCount;
        uint8_t is_PRG_MD5;
        uint8_t is_PRG_X;

        uint8_t isWirelessConnected;

        uint8_t is433WirelessConnected;
        uint8_t msg_lost;
        uint8_t signal_433SM; // signal of the module received by the controller
        
       
//        uint32_t LastResponseT;//Timer of the last response
        
	uint8_t interfaceStatus[8];
	uint8_t statusMatrixA[24];
        uint8_t statusMatrixB[8];
        uint8_t statusMatrixP[8];
        
} SlaveInfo;


__packed typedef struct _audioBox{
	uint8_t isConnected;
	//uint8_t isCANConnected; // put it in isConnected and remove it
       // uint8_t isWirelessConnected; // put it in isConnected and remove it
        
	uint8_t slave_err;
        uint8_t slave_wng;
        
        uint16_t AB_Addr;
        uint8_t isWir433_Connected;
        uint8_t signal_433SM;
        
        uint8_t msg_lost;


	uint8_t Mod_Kind; // rprg 10.07.2017
        uint8_t matrixARev;
        uint8_t matrixAStatusX;        
        uint8_t DMX_Pyro_Stat; //Enable/Disable DMX, Pyro
        
        
        
        
        uint8_t isArmedK;
        uint8_t missedCount;
        uint8_t is_PRG_MD5;
        uint8_t is_PRG_X;
        uint8_t isWirelessConnected;

        
//        uint32_t LastResponseT;//Timer of the last response
        
	uint8_t interfaceStatus[8];
	uint8_t statusMatrixA[24];
        uint8_t statusMatrixB[8];
        uint8_t statusMatrixP[8];
        
} audioBoxA;


//**********************manage self device ******************************
__packed typedef struct _devData{
	uint8_t min;
	uint8_t sec;
	uint8_t msec;
                uint8_t self_testing;
                
	//uint16_t master_Address;
	
        uint32_t error_Code;
        uint16_t device_Address;

	uint8_t isUSBConnected;
	uint8_t isCANConnected;
                
        uint8_t isWireless433Connected;
        
	uint8_t isWirelessConnected;
	uint8_t wireless_signal_value;
	
	//for interface status
	uint8_t join_status;
	uint8_t programmed_interface;
	uint8_t power_Enable;
	uint8_t isMaster;
                
	uint8_t isPlay;
	uint8_t isPause;
        
                /*
                uint8_t GHours;
                uint8_t GMin;
                uint8_t GSec;
                uint8_t GPS_ValidS;
*/
	uint8_t wireless_power_status;
                
	uint8_t internal_battery_status;
        uint16_t external_battery_status;
                
                /*
                uint8_t statusMatrixA[MAX_STAT_A];// Matrix channel status
                uint8_t statusMatrixB[MAX_STAT_B];// Matrix for Programmed
                uint8_t statusMatrixP[MAX_STAT_B];// Matrix for Enable/Disable
                
                uint8_t statusMatrixAx[MAX_STAT_A];// Matrix channel status
                uint8_t statusMatrixBx[MAX_STAT_B];// Matrix for Programmed
                uint8_t statusMatrixPx[MAX_STAT_B];// Matrix for Enable/Disable                
                */

} Device_Data;

//****************** For Time active matrix programming**********************
//#pragma pack(1)
//typedef struct _prog{
__packed typedef struct _prog{

                //uint32_t time;//4         
                //uint8_t M_ID;//4
                //uint32_t Address;//1
                uint8_t valid;
                //uint8_t Mod_ID;
        
                //Start MD5
                uint8_t lineID;
                uint8_t seqID;
        
             //   uint8_t Position;//1
                uint8_t Safe_Zone;//1
                
                uint8_t DMX_value;
                uint8_t Rmp_Cfg;        
                uint16_t DMX_Ramp;//Duration
                
                uint32_t time;
                //savedDatas                               0x20016d80   Data       20584  device.o(.bss)
                //savedDatas                               0x20016d80   Data       16584  device.o(.bss)
                //savedDatas                               0x20014980   Data       76584  device.o(.bss)
                
                
          
} dev_prog;



__packed typedef struct time_prog{
        
	        dev_prog programme[TimeMatrixDepth];
        
        #if DBG_TST_X 
                uint16_t mark[MAX_CH_SAME_TIME];
        
                uint16_t count;
               // uint8_t M_ID[TimeMatrixDepth];
        
                uint32_t Max_Time;
                uint32_t Min_Time;
        
              //  uint16_t count_Pyro;
        
                //uint8_t mark[100];
        
                char Seq_Name[MAX_SEQ][21];// Names of all sequences
                uint8_t Safe_Zone[MAX_SZ];   // Pyro SafeZone for all 16 channels 01 to 16
                uint8_t PRG_ID[MAX_NUMBER_OF_DEVICES];   // Which mods are programmed
                uint16_t count_seq[MAX_SEQ];
                uint8_t seq_valid[MAX_SEQ];   // Which sequences are programmed
                uint8_t seq_mods[MAX_SEQ];   // Which sequences are programmed
                uint16_t count_mod_ev[MAX_NUMBER_OF_DEVICES];

        #endif
               // char Chan_Name[MAX_SEQ][21];// Names of all sequences
} Device_Time_prog;



__packed typedef struct MD5_crc{
	unsigned char MD5_CRC_ModX[16];
} MD5_CRC;


__packed typedef struct saved_time_prog{

                Device_Time_prog time_prog;
        
      #if DBG_TST_X 
        	uint32_t preamble1;
                uint32_t deviceAddress;
                                
                char Prg_Name[MAX_SCRIPT_LENGTH_LINE_N];//program name
                
                char Seq_Name[MAX_SEQ][MAX_SEQ_NAME];// Names of all sequences
                
                MD5_CRC MD5_CRCxxx[MAX_NUMBER_OF_DEVICES];
                
                uint8_t validSavedData;


                uint8_t GPS_HoursS;
                uint8_t GPS_MinuteS;
                uint8_t GPS_secondsS;
                uint8_t GPS_ValidS;

                uint8_t is_R_Prg;
                uint8_t is_Mods;
                uint16_t is_Pyro_events;
                uint16_t is_DMX_events;
                uint16_t is_Pyro_eventsS0;
                uint16_t is_DMX_eventsS0;
                uint8_t is_SafetyZones;
                uint8_t is_Sequence;
        
                
                

                uint32_t preamble2;        
#endif              
} Saved_Time_prog;



__packed typedef struct saved_cfg{
                uint32_t ignore1;
        
                uint32_t preamble2;
                uint8_t validSavedDataCfg;
                
                //uint8_t device_AddressCfg;
                
        
                uint8_t LCD_Backlight; //LCDBacklight
        
                uint8_t BuzzerEnableX; //Enable or Disbale Buzzer
                int8_t TimeZoneX; //TimeZone
        
                uint8_t Audio_Volume;
                //uint8_t is_prg;
        
                uint8_t Lang;
        
                uint8_t is_Custom_433_Wir;

                uint8_t Channel_433_Wir;
                uint8_t Net_ID_433_Wir;
                uint16_t Key_433Mhz;
                uint32_t Mast_ID_433Mhz;
                uint32_t BMast_ID_433Mhz;
        
        
                uint8_t Time_CodeX;
                uint8_t TimeCode_BackUp;
        
        
                uint8_t is_Custom_Wir;
                uint64_t CW_Address;
                char Network_NameS[4];
        
                uint8_t sleep_mods[MAX_NUMBER_OF_DEVICES + MAX_NUMBER_OF_AB];
                uint8_t is_last_sleep; //0 - awake, 1 - Sleep
                uint8_t is_sleep_stat; //0 - See is_last_sleep, 1: Modules start go to sleep, 11: Modules start awake
                
                
                
        
                uint8_t PB_EN; // Power button in ARM mode - Not able to power off the module in ARM, PLAY - (PAUSE - in pause better to be able to power off)
                uint8_t SC_EN; // Safety connection - stop play the script if no messages are received form remote: Minim 10 seconds. Default 0 - Disable
               
               // uint8_t SMPTE_TC_OUT; // Time code generator in Play mode 0(default) - Disable, 1 - 25fps, 2- 33fpsND 3- FSK PD; 4- FSK F1

        // remote words
                uint32_t R433Mhz_B1b[2];
                uint32_t R433Mhz_B2b[2];
                uint32_t R433Mhz_B3b[2];
                uint32_t R433Mhz_B4b[2];
                
        // remote bit times
                uint16_t R433Mhz_B1bt;
                uint16_t R433Mhz_B2bt;
                uint16_t R433Mhz_B3bt;
                uint16_t R433Mhz_B4bt;                
                
                
                uint8_t GPS_HoursS;
	//uint8_t GPS_MinuteS;
	//uint8_t GPS_secondsS;
                
	
               
               
               uint8_t crc_wireless;
                
                
                uint8_t is_reverse; //LCD orientation
                uint8_t AB_address; //AB Address
                
                
                
                
                uint8_t is_Err_det;// 
                
                
                uint8_t TM_EN; //Talon mode

                
                uint8_t D_Mode;// default start mode - Master, Stand alone, Small remote,others. Default 0 - Start in SafeMode
                
                
               
                
                uint8_t is_ET; // External trigger - control module from external trigger Enable/Disable 
                
                uint8_t AS_EN; // Autoscroll in play - display next item to be fired. Default 0 - Disable, 1 Enabled (Pyro), 3. DMX + pyro, 4 only DMX
                

                
                uint8_t Step_Mode; // Step fire without programing. 0 (default) - increase channel, 1 - Increase rail

                
                uint8_t is_433R_ARM; // enable or disbale possibilities to arm from 4343mhz remote
                
                uint8_t is_433R_EN; // 433Mhz Default Enabled or disabled
                
                uint8_t is_433R_CC; // Small remote crew controll
                
                uint8_t AS_ID_Time; // Small remote crew controll
                
                uint8_t SA_Mode; //enable or disable stand alone mode
                uint8_t SRC_Mode; // increase rail or channel in step fire mode
                
               // uint32_t tmp[58]; // to fill 256 bytes
                
                uint8_t isGPSFTValid;
                uint32_t Start_Time;// Start Time 0 to 86400000 (0 to 24h)  0 means disable
                
                uint32_t preamble1;
                uint32_t ignore2;
	//uint32_t tmp[27];
} Saved_Cfg;


/*
typedef struct fixed_prog{
	uint32_t preamble1;
        int TimeZone;
	uint32_t Pass_W;
	uint32_t validSavedData;
	uint32_t EPID_W;
	uint32_t is_validTZ;
	uint32_t preamble2;
	uint32_t tmp[2];
} fixed_dev_prog;

*/
/**
@unsigned int device_Address
@brief value of device's address(device number). it will be 0 ~ 99.
*/
Device_Data* get_device_datas(void);


/**
@fn void InitDevice(uint32_t deviceAddress)
@brief Initialise the device
@param[in] deviceAddress Address of the device
@details Called after the power-on
*/
void InitDevice(uint32_t deviceAddress);
void InitDevice2(uint32_t deviceAddress);

/**
@fn Status ActivateMasterDevice(void)
@return Operation result (ERROR/SUCCESS)
@brief Puts the device into the "master" mode
*/
ErrorStatus ActivateMasterDevice(void);


/**
@fn void DuplicateDeviceDetectedAtAddress(uint32_t address)
@brief Reports the detection of a duplicate device address
@details Called by the master (wireless coordinator)
*/
void DuplicateDeviceDetectedAtAddress(uint32_t address);

/**
@fn void SetDeviceAddress(uint32_t address)
@param[in] address Address of the device
@brief Set the device address obtained from the address reader
@details Use at the software start
*/
void SetDeviceAddress(uint32_t address);

/**
@fn int32_t GetLastDeviceErrorNumber(void)
@return Last device error code
@brief Get the last error code detected locally by the device
@details Should be used from the display driver code
*/
int32_t GetLastDeviceErrorNumber(void);


void setMFinPlay(uint16_t MFloc, uint16_t MFlineID, uint64_t MFtime);



void setSlaveAddress(uint8_t address);

void getStatusResponseMessage(PC_Message *message);




dev_FIFO* getDevFIFO(void);

void syncPCTimer(PC_Message *msg);



void getMSEvent(void);


void generateTimeMatrixEvent(uint64_t currentTime);



void set_check_pwr(void);
uint8_t get_checkpwr(void);




uint8_t adress_interface(void);




void Delay(__IO uint32_t nCount);


enum _PC_Master_Messages_Command{
	PC_Master_Msg_Error=0,
	PC_Master_Msg_Init=1,
	PC_Master_Msg_Sync=2,
	PC_Master_Msg_Pwr_Enable=3,
	PC_Master_Msg_Start_Show=4,
	PC_Master_Msg_Stop_Show=5,
	PC_Master_Msg_Pause_Show=6,
	PC_Master_Msg_Prog=7,
	PC_Master_Msg_Finish_Prog=8,
	PC_Master_Msg_Manual_Act=9,
	PC_Master_Msg_Ping=10,	
	PC_Master_Msg_Can_Term=11,	
	PC_Master_Msg_Wireless_Power=12,
	PC_Master_Msg_Join_En_Dis=13,
	PC_Master_Msg_Status_Interface=14,
	PC_Master_Msg_Status_Matrix_A=15,
	PC_Master_Msg_Status_Matrix_B=16,
	PC_Master_Msg_Trans_Over=17,
       	PC_Master_Msg_SEQ_CTRL=18,
       	PC_Master_Msg_SEQ_MCTRL=19,        
      	//PC_Master_Msg_PRG_ANS=19,

};

void set_dev_address(uint8_t adr_ID);
uint8_t check_pwr(void);







void check_arm_st(void);
void sendMSPingBC(void);
void sendMSPingA(uint8_t mod_id_ping);
void send_sync_msg(int t_ossfet);
void set_Master(void);

void set_valid_script_pos(void);


uint8_t get_PDMode_val(uint8_t mod_ID);
void save_custom_Wir(uint64_t net_IDx);
void set_Last_ping_time(uint8_t adrr_xxx);
void setSlavePingedX(uint8_t pinged_slaveB);
void Set_Ping_TimeX(uint8_t pinged_slaveA);
void getSlaves_Errors(uint8_t pinged_slave_X);
uint8_t get_mod_kind(uint8_t Mod_ID_tmp, uint8_t DMX_Option);
uint8_t getSlaves_isProgrammed(uint8_t slv_disp);
uint8_t check_if_F1_M(uint8_t mod_tmp); // check if the slave it is an F1 firetek controller
uint8_t getSlaves_isProgrammedV(uint8_t slv_disp);//MD5 verification
uint8_t get_Slave_prgX(uint8_t isadrX);
uint32_t get_ping_timeInterval(void);
uint8_t getSlaves_Intern_Bat(uint8_t slave_bat);
uint16_t getSlaves_connected(void);
void read_prg_time(MS_Message *msg);
void extract_mtxB(uint8_t slave_req_id);
SlaveInfo* getCurrentSlaveInfo(uint8_t slavenum);
void setTOsalve(void);
void update_ping_times(void);
void update_rail_for_LCD(uint8_t mod_ID_up);
uint8_t get_rail_conStat(uint8_t mod_D_chk);
uint8_t check_R_S (uint8_t tmp_mod_chk, uint8_t tmp_rls);
uint8_t get_ch_stat (uint8_t mod_tmp_chk, int t_line, int t_col, int need_NX);


void sendMSManualActivationX(uint8_t Line_fired, uint8_t fire_typeX);

void send_ping_arm(uint8_t mod_ID_Ping);

void SetBackSlave(void);
void SetNextSlave(void);
void Set_Error_Slave(void);
uint8_t update_Mod_statsD(uint8_t mod_to_up);

uint16_t getSlaves_Extern_Bat(uint8_t mod_to_up);
uint8_t getSlaves_WirelessSignal(uint8_t mod_to_up);


uint8_t get_AP_Stat(void);
        
uint8_t update_Rem_stats2A(void);
uint8_t update_Rem_stats2B(void);
uint8_t update_Rem_stats2C(void);

void rail_stat_reset(uint8_t rst_rail);

uint8_t get_slaves_need(void);
uint8_t get_slaves_remain(void);
void rst_prg_script(void);        
uint8_t get_is_rem_programmed(void);
void set_Next_ev_time_disp(void);


uint8_t get_device_Status(void);
void set_dis_pyro_DMX(uint8_t mod, uint8_t state);
uint8_t update_Sys_infoD(void);
uint8_t update_modStats_prg(uint8_t check_x, uint8_t check_y);
uint8_t get_mod_prg_con_stats(uint8_t mod_tmp);
uint8_t getSlaves_isProgrammedRem(uint8_t slv_disp);
uint8_t Errors_found(void);
uint8_t set_first_connected_slave(void);
uint8_t get_mod_errors(void);
uint8_t getSlaves_isConnectedX(uint8_t adr_ID);


void sendMSJoinBroadcastD(void);
void rst_CPress(void);
uint8_t Send_RS_Prg(uint8_t addr_tmpP);
void send_prg_request(uint8_t prg_mode, uint8_t pro_mod, uint8_t prg_lineID, uint16_t prg_pos);
void rst_prg_ev(void);


uint8_t get_first_mod_to_prg(uint8_t last_mod);
uint8_t get_next_connected_slave(uint8_t mod_con);
void send_delete_prg(uint8_t mod_id);
uint16_t get_ev_mod (uint8_t mod_id);
uint8_t get_mods_prg(void);
uint16_t get_ev_Allmod (void);
void display_Mods_up_prg_LCD(void);
void set_up_mod_con(void);
uint8_t get_first_mod_con(uint8_t last_mod);
uint8_t get_mods_up_error(void);
uint8_t get_Next_mod_Err(uint8_t id_tmp);
void SetNextER_Up_Slave(void);
void SetBackER_Up_Slave(void);
void set_Seq_fire(uint8_t seq_fire);
uint8_t seq_sel_fired(uint8_t Line_fired);
void sendMSSequenceX(uint8_t Line_fired);
uint8_t get_number_of_seq(void);
void Update_SZ_fire(void);
void set_sz_fired(uint8_t fired_sz_pos);
uint8_t get_is_seq_dis(void);
uint8_t get_PD_Stat(void);
//uint8_t get_number_of_sz(void);
uint8_t get_SZ_stat(uint8_t sz_id);
uint8_t get_SZ_En_Dis(uint8_t sz_id);
uint8_t get_err_not_con(void);
void fire_next_channel(uint8_t mod_tofire, uint8_t chan_tofire);
void stp_fire(void);
void extractAold(uint8_t tmp_id);
uint8_t get_ab_con(void);

void snd_AB_IP_msg(uint8_t mp3_command, uint8_t AB_ID);
uint8_t read_mp3AB_status(uint8_t AB_ID);
uint8_t read_mp3_AB_pstatus(uint8_t AB_ID);

void SetNextMB(void);
void SetBackMB(void);

void sendMSPingX(uint8_t S_ID);
void check_arm_st_X(void);

uint8_t get_slave_con(void);

uint8_t get_slave_Fault(uint8_t slave_to_check);
uint8_t get_slave_Wng(uint8_t slave_to_check);
void error_check_report(void);
uint8_t get_err_stat(void);
void inc_err_stat(void);
uint8_t is_online(uint8_t mod_to_resp);
int get_time_zone(void);

void inc_Time_Zone(void);
void dec_Time_Zone(void);


void save_time_new(uint8_t tmp_hh, uint8_t tmp_mm, uint8_t tmp_ss);
void get_time_to_set(void);

void inc_Time_H(void);
void inc_Time_M(void);
void inc_Time_S(void);
void dec_Time_H(void);
void dec_Time_M(void);
void dec_Time_S(void);
void save_time_start_new(void);
void del_time_start_new(void);

uint8_t get_is_gps_fire_time(void);
uint32_t get_tus(void);

uint8_t get_buz_stat(void);
void set_rst_buz_stat(void);
void save_config(uint8_t display_NO);

uint8_t get_pwBut_stat(void);
void set_rst_pwBut_stat(void);

void dec_LCD_BackLight(void);
void inc_LCD_BackLight(void);
uint8_t get_BL_LCD_val(void);
void refresh_mod(void);
void wireless_config(uint8_t config_wir);
void sendMSJoinBroadcastWX(uint8_t set_rst);
void set_Time_Zone (uint16_t TZ_val);
void update_Config(void);
void set_lang_x(uint8_t lang_x);
void set_New_Net_name(uint8_t set_rest);

uint8_t get_Display_timer(void);
void  save_network_name(void);

void get_time_to_offset(void);
void set_Time_Code_offset(void);
void rst_Time_Code_offset(void);
uint32_t get_time_Code_ofset(void);
void sendMSSequenceBCAST(uint8_t Line_fired);
void sendMSSequenceUCAST(uint8_t Line_fired);

void rst_slaves_script(void);


void send_4332_msg_to_nextmod(void);
void send_uCast_433Mhz_msg(uint8_t mod_to_send, uint8_t stat_req);
void getMS_433_Event(void);
void set_433_wir_strength(uint8_t mod_to, uint8_t signal_str);
void rst_last_mod_ping(void);
void send_433_arm_check_msg_to_nextmod(void);        
void send_433_start_sync_msg(void);
void send_433_Sequnce_ctrl(uint8_t line_seq_frd);
void set_save_433wir_config(uint8_t channel_433Mhz, uint8_t netID_433Mhz, uint16_t key_433Mhz, uint8_t is_custom_X);
uint8_t get_433_ch(void);
void inc_433Mhz_chan(void);
void dec_433Mhz_chan(void);
uint8_t get_433_ch_temp(void);
void start_433_Mhz(void);
uint8_t update_ABStats_prg(uint8_t mod_to_up);
uint8_t getAB_Intern_Bat(uint8_t slave_bat);
uint8_t getAB_WirelessSignal(uint8_t mod_to_up);


uint8_t get_AB_Fault(uint8_t slave_to_check);
uint8_t get_AB_Wng(uint8_t slave_to_check);
uint8_t is_ABonline(uint8_t mod_to_resp);
uint8_t getAB_Wireless433Signal(uint8_t mod_to_up);
uint8_t getSlaves_Wireless433Signal(uint8_t mod_to_up);
uint8_t getSlaves_Wireless433Con(uint8_t mod_to_up);
uint8_t getAB_WirelessCon(uint8_t mod_to_up);

void send_433_BC_msg(void);
uint8_t get_sleep_state(void);
void set_sleep_state(uint8_t tmp_state);
void send_433_msg_to_nextmod_Sleep(void);
void rst_ping_433_stat(uint8_t stantss);
uint8_t get_mod_sleep(void);
uint8_t get_mod_stat_sleep(uint8_t mot_modss);
void set_sleep_stateX(uint8_t tmp_state);
uint8_t get_sleep_stateX(void);
uint8_t get_is_system_sleep_X(void);
uint8_t get_433_slave_con(void);
uint8_t get_time_left_SSS(void);
uint8_t get_is_slave_433_connected(uint8_t slave_Xds);
void send_prg_request_433Ext(uint8_t prg_mode, uint8_t pro_mod, uint8_t prg_lineID, uint16_t prg_pos, uint8_t events_load_it);
uint16_t get_433_net_key(void);
uint8_t get_433_net_id(void);
void send_delete_prg433Mhz(uint8_t mod_id);
uint16_t get_ping_timeX(uint8_t mods);


void send_TEST_msg_to_nextmod(void);
uint16_t get_ping_time(uint8_t tmp_id_AAAAA);
uint16_t get_ping_nmr(uint8_t tmp_id_AAAAA);
int get_signal_strength(uint8_t tmpsdff);

void set_main_screen(void);
void send_noise_msg(void);
uint8_t get_rev_display(void);
uint8_t get_Lang(void);
void set_AB_interface_ID(uint8_t ttmp_adr);
void set_AB_Master_slave(void);
void Start_music_Sync(void);
void send_sync_433_msg(void);
void send_433_mod_ping_reply(void);
void set_real_time(uint8_t tmp_fix, uint32_t tmp_val_time);
void updateDisable(MS_Message *msgX);
void save_new_433_config(uint8_t save_config_X);
void clear_mark(void);
void generateDMXTimeMatrixEvent(uint64_t currentTimeX);
void Calc_Ramp_Step(uint8_t CH_ID_tmp, uint8_t Cur_DMX_CH_value, uint8_t New_DMX_CH_value, uint8_t DMX_Dur, uint32_t DMX_rmp_tmp);
void clearActivedTimeMartix(void);
uint8_t get_Music_box_ID(void);
void read_encoder_AB(void);
uint8_t was_menu_selected(void);
int8_t read_encoder_XXX(int8_t min_val, int8_t max_val, int8_t cur_val);
void set_display_Save_Exit_choose(void);
void Disable_TC_SMPTE_Gen_Pin(void);
void Enable_TC_SMPTE_Gen_Pin(void);
void Init_TimeCode(void);
uint8_t get_AB_Adre(void);
uint8_t calculate_433lost_Percent(void);
void send_4332_msg_to_nextmod_ALL(void);
void set_time_out_mod(void);
void Start_TimeCode(void);
void Stop_TimeCode(void);
uint8_t are_any_DMX_Evants(void);

void save_Start_time_Config(void);
void save_TimeZone(int8_t tmpTZxzx);

uint8_t get_433_chane(void);
int get_mod_test_noise(void);
void set_new_chan(uint8_t chna_start);
void set_433Mhz_config(MS_Message *tmsg);
void set_default_settings(void);
uint8_t get_TC_BackUp(void);
void toggle_TC_BackUp(void);
uint8_t get_AB_con(void);

uint8_t get_is_custom(void);
uint8_t get_wir_crc(void);
uint8_t check_network_name(void);
void set_433Mhz_configNew(MS_Message *tmsg);
void save_433_config_param(uint8_t netch_tmp, uint8_t netid_tmp, uint16_t netkey_tmp, uint8_t tmp_is_cust);
void set_itf_ID(uint8_t tmp_IDnumber);

void reset_DMX_Seq(uint8_t sequenceRes);
void Set_DMX_R(void);
void update_buffer_DMX(void);
void set_433_seq_V(uint8_t *seq_msg);
void pwr_off_mod(void);


#endif
