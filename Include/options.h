/**************************************************************************//**
 * @file     options.h
 * @brief    Header file for various project configuration options
 ******************************************************************************/
#ifndef _OPTIONS_H_
#define _OPTIONS_H_


//#define TEST_DBG        (0)

/********************************************************
@def MAX_NUMBER_OF_DEVICES
@brief maximum number of devices (starting from address 0)
@details Maximum number of device adresses. Take care that adresses start from 0,
(all address switches are off) so the highest device address would be  equal
to MAX_NUMBER_OF_DEVICES-1
*******************************************************/
#define MAX_NUMBER_OF_DEVICES 	(100)

#define MAX_NUMBER_OF_DEVICES_TST        (100)

//#define MIN_NUMBER_OF_AUDIO_BOX (90)
#define MIN_NUMBER_OF_AUDIO_BOX (0)

#define MAX_NUMBER_OF_AB (9)


#define MAX_MOD_NC (5)
#define MAX_AB_NC (5)




/*******************************************************
@def MAX_ELEMENT_OF_MATRIX
@brief maximum number of matrix elements (starting from address 0)
@details
********************************************************/
#define MAX_ELEMENT_OF_MATRIX 	(64)

/*******************************************************
@brief if master will be not received answer from slave,
		then he will remove it from list.
*******************************************************/
#define MAX_Fail_Ping			(10)

/*******************************************************
@brief if slave does not receive ping message many times in idle,
		power enable or pause state, then he will go into init state.
*******************************************************/
#define MAX_MISSED_PING			(99)

/*******************************************************
@brief if device will be not received answer from master or PC,
		then he will go into idle state.
*******************************************************/
#define MAX_Missed_Sync			(3)

/**
@def DEVICE_DETECTION_DURATION
@brief Period in which device enumeration will be conducted
@details Time is in milliseconds (currently it is 5 minutes)
*/
#define DEVICE_DETECTION_DURATION 					(10)

#define DEVICE_SET_ID_DURATION 					(10000) // for tests with OD preset

/**
@def DEVICE_DETECTION_BROADCAST_REPEAT_PERIOD
@brief Period between transmission of two enumeration messages
*/
#define DEVICE_DETECTION_BROADCAST_REPEAT_PERIOD 	(3000)

/**
@def DEVICE_GET_MATRIXSTATUS_PERIOD
@brief Period between two getting matrix A action
*/
#define DEVICE_GET_MATRIXSTATUS_PERIOD 	(500)

/**
@def DEVICE_PING_PERIOD
@brief Period between ping actions
*/

#define TIME_ARM                        3000

#define MAX_AB_ID                                (10) //Message number position
#define MIN_AB_ID                                (1) //offset message vs 2.4ghz wirelesdss

#define MAX_AUTOSET_ID                                (9000) //Time to wait Audiobox ID set

#define ARM_KEY_TIMER                                   (3000)

#define PING_OFFSET                                     (250)//250



#define DEVICE_PING_PERIOD				(10000)
#define DEVICE_SYNC_PERIOD 				(3000) //must be multiple of 100, minimum 10, maximum 25000

#define DEVICE_PAUSE_PERIOD 				(4000)


#define PING_433_OFFSET                                     (250)//(500)//
#define PING_24_OFFSET                                      (300)//(500)//

#define RESEND_MS_PING_TIME				(3000)

#define FIRST_MSG_SENT			        	(2500)



#define MAX_TIME_WAIT_FINISH				(2000)
#define RETRY_TIMES_433MHZ				(6)

#define MAX_TIME_WAIT_PRG				(7000)//load from USB wait


#define SPAN_MESSAGES					(100) //800
#define MAX_PROGRAMMING_MESSAGE_SPAN	                (1000)
#define WAIT_MS_MESSAGE_PERIOD			        (1000)
#define WAIT_MS_PROGRAMING_MESSAGE                      (3000)
#define MAX_WAIT_TIME                                   (150) // Timeout wireless module - wait wireless module to respond
#define MAX_SIGNAL_TIME                                 (60000)

#define MAX_WIR_CHAN                                  (16)



#define MIN_MSG_SND                                     (3000)   

//In Init state, device will be resetted at every period.
#define RESET_TIME_PERIOD				(60000)// 60 seconds

#define DISPLAY_UPDATE_TIME				(20)
#define Battery_Update_Time				(DISPLAY_UPDATE_TIME*20)

#define MAX_WAIT_LCD_DEL_CHG				(2500)


#define ERROR_LIGHT_TIME				(1000)





#define GPS_UART      (LPC_UART0)


#define MAX_TC_WAIT     2500

#define MAX_MSG_SIZE    (24)

/**
@def MIN_USB_MESSAGE_LENGTH
@brief Minimal valid PC protocol message size
*/
#define MIN_USB_MESSAGE_LENGTH 			(2) //bytes

#define MAX_USB_BUFFER					(8)


#define SCROLL_WAIT_TIME                                        (500)
#define SCROLL_CNT_SKIP                                         (1)

/**
@def MAX_USB_MESSAGE_LENGTH
@brief Maximal valid PC protocol message size
*/
#define MAX_USB_MESSAGE_LENGTH 			(32) //bytes

#define TimeMatrixDepth			(6000)       //(8000) //number of events in memory
#define MAX_PRG_WAIT_TIME 	(10000)

#define PREAMBLE_1					(0x12345678)
#define PREAMBLE_2					(0x87654321)




#define MAX_SAFETY_ZONES                                  (16) // Maximum safety zones
#define MAX_POS_ZONES                                     (16) // Maximum positions zones

#define DMX_CH_MAX                                     (256) // Maximum positions zones
#define MAX_CH_SAME_TIME                               (255)


/**
@def ENUMERATION_BROADCAST_ADDRESS
@brief Address on which enumeration bradcast will be sent
*/
#define ENUMERATION_BROADCAST_ADDRESS 			(100)
#define ENUMERATION_SYNCRONIZATION_ADDRESS		(100)
#define ENUMERATION_POWERENABLE_ADDRESS			(101)
#define ENUMERATION_START_ADDRESS				(102)
#define ENUMERATION_PAUSE_ADDRESS				(103)
#define ENUMERATION_STOP_ADDRESS				(104)
#define ENUMERATION_CANTERM_ADDRESS				(105)
#define ENUMERATION_WIRELESSPOWER_ADDRESS		        (106)

#define PC_MASTER_Error							(200)
#define PC_MASTER_Init							(209)
#define PC_MASTER_Sync							(201)
#define PC_MASTER_PowerEnable					        (203)
#define PC_MASTER_Start							(204)
#define PC_MASTER_Stop							(205)
#define PC_MASTER_Pause							(206)
#define PC_MASTER_Ping							(202)
#define PC_MASTER_CANTerminal					        (207)
#define PC_MASTER_WirelessPower					        (208)
#define PC_MASTER_JoinEnable					        (210)
#define PC_MASTER_TransmissionFinished			                (211)
#define PC_MASTER_SEQ_CTRL			                        (212)
#define PC_MASTER_SEQ_MCTRL			                        (213)

#define MASTER_SLAVE_Ping						(0u)
#define MASTER_SLAVE_Error						(2000)
#define MASTER_SLAVE_Sync						(2001)
#define MASTER_SLAVE_Broadcast					        (2002)
#define MASTER_SLAVE_PowerEnable				        (2003)
#define MASTER_SLAVE_Start						(2004)
#define MASTER_SLAVE_Stop						(2005)
#define MASTER_SLAVE_Pause						(2006)
#define MASTER_SLAVE_Seq						(2007)
#define MASTER_SLAVE_AudioBOXadr				        (2008)
#define MASTER_SLAVE_AudioBOXcmd				        (2009)
#define MASTER_SLAVE_PingBC					        (2010)
#define MASTER_SLAVE_F1_UF_CTRL                                         (2011)
#define MASTER_SLAVE_PRG_CONF                                           (2012)
#define MASTER_SLAVE_F1_Pass                                            (2013)
#define MASTER_SLAVE_F1_DEL_S                                           (2014)
#define MASTER_SLAVE_MIR_MSG                                            (2020)// mirror message



#define MASTER_SLAVE_Programming				(100)
#define MASTER_SLAVE_FinishProgramming			                (200)
#define MASTER_SLAVE_ManualActivation			                (300)
#define MASTER_SLAVE_TR					(1300)


#define MASTER_SLAVE_WIRELESS_SET				(111)

#define PING_X (0u)//0 - ping a module for interface status and matrix status; Also can contain pyro and safet zones control
#define PING_I (1u)//1 - ping a module for interface status;
#define PING_A (2u)//2 - ping a module for Matrix statusA;
#define PING_BP (3u)//3 - ping a module for Matrix statusBP;
#define PING_MA (4u)//4 - Send a manual activation message
#define PING_RT (5u)//5 - request from module specific channel programing details; 
#define PING_DC (6u)//6 - Delete a pyro channel - the other channels are not affected
#define PING_AD (7u)//7 - respond to Master with programing details for a specific mode/line ID; 
#define PING_PS (8u)//8 - programing only a specific channel from a module (the other channels will not be affected) 
#define PING_PA (9u)//9 - programing a channel on a module - if the module was programmed before entire script will be deleted
#define PING_FP (10u)//10 - finish programing a module
#define PING_DS (11u)//11 - Delete programing from a module
#define PING_MD5 (12u)//12 - Send MD5 hastag on request
#define SEQ_FIRE (13u)//13 - Send Sequence fire


#define AB_CMD (50u)//50 - Audiobox commands: Play/Pause/Volume

#define  FILENAME_KEY  "1:/KEY.FTK"

//messages 433Mhz
#define PING_433_GRANULARITY                                     (10)//(10)

#define PING_433_OFFSET_TEST                                     (1500)//(500)//

#define PING_433_OFFSET_BC                                  (10000)//(20000)//(17000)

#define TIME_433_OFFSET                                  (90)//(75)//Time spent by 433mhz message on air



#define TIME_TO_SLEEP                                  (30)//1 to 255 Time the modules stay on sleep: 1 unit it is about 10seconds




#define MSG_433_TIME_OFFSET                                 (150)//(150)//

#define MAX_433_TIME_OUT_WAIT                                   2
#define MAXIMUM_433_MESSAGE_SIZE                                (32)
#define MAXIMUM_MS_433_MESSAGE_SIZE                             (32)
#define MAXIMUM_SM_433_MESSAGE_SIZE                             (64)
#define MAXIMUM_433_MESSAGE_SIZE_EXT                            (240) // maximum length used for programing

#define PRG_MSG_LENGTH                                          (12) // maximum length used for programing


#define WIRELESS_433_BUF_SIZE                                   6



#define START_CHAR_433MHZ_SET                               (0xC1) //Start char message for module settings

#define START_CHAR_433MHZ_SM                                (0xCC) //Start char message Slave to Master
#define START_CHAR_433MHZ_MS                                (0x99) //Start char message Master to Slave
#define START_CHAR_433MHZ_SS                                (0x66) //Start char message Slave to Slave
#define START_CHAR_433MHZ_AB                                (0xAA) //AB messages to ALL
#define START_CHAR_433MHZ_S1                                (0xC1) //Start char message for settings

#define START_CHAR_433MHZ_SP                                (0x55) //PROGRAMING MESSAGE Extended

#define END_CHAR_433MHZ_SS                                (0x11) //END char message Slave to Slave
#define END_CHAR_POS                                        (63)//(29) //End char position in message

#define M433_BROADCAST                                (100) //Broadcast message

#define MSG_LOST_POS                                (61) //Message lost position
#define MSG_WIR_PERCENT                             (60) //Wireless percent

#define AUDIO_BOX_OFFSET                           (200) //Audiobox offset
#define AUDIO_BOX_ADR_SET                          (99) //Audiobox offset


#define DEFAULT_433_PRG_EXT                          (0x00) //DEFAULT Power, and data package length In programing mode
#define DEFAULT_433_PWR_DAT                          (0x20)//(0xE0) //DEFAULT Power, and data package length in standard mode
#define DEFAULT_433_CHANNEL                          (0x12)//(0x12) //DEFAULT 433MHZ WIRELESS CHANNEL

//#define DEFAULT_433_SPEED                            (0x64)//(0x63)// DEFAULT 433MHZ SPEED, E2:2.4kbs, E3:4.8kbs, E4:9.6kbs
#define DEFAULT_433_SPEED                            (0xE5)//(0xE4)// DEFAULT 433MHZ SPEED, E2:2.4kbs, E3:4.8kbs, E4:9.6kbs


#define DEFAULT_433_NET_ID                          (0xAA) //DEFAULT 433MHZ WIRELESS Network address 0 - 255
#define DEFAULT_433_KEY_H                          (0xAB) //DEFAULT 433MHZ WIRELESS CHANNEL 0 - 255
#define DEFAULT_433_KEY_L                          (0xCD) //DEFAULT 433MHZ WIRELESS CHANNEL 0- 255

#define DEFAULT_433_KEY_LH                          (0xABCD) //DEFAULT key 433Mhz

#define MAXIMUM_433_CHANNEL                          (83) //MAXIMUM 433MHZ WIRELESS CHANNEL
#define MINIMUM_433_CHANNEL                          (0) //MAXIMUM 433MHZ WIRELESS CHANNEL



#define TEST_MOD_ID                                (1) //ID of the module tested

#define TIME_FOR_SYNC                                (10000) //Time used for calibration

//Commands 433Mhz
#define M433_PING_X (0u)//0 - ping a module for interface status and matrix status; Also can contain pyro and safet zones control
#define M433_PING_I (1u)//1 - ping a module for interface status;
#define M433_PING_A (2u)//2 - ping a module for Matrix statusA;
#define M433_PING_BP (3u)//3 - ping a module for Matrix statusBP;
#define M433_PING_MA (4u)//4 - Send a manual activation message
#define M433_PING_RT (5u)//5 - request from module specific channel programing details; 
#define M433_PING_DC (6u)//6 - Delete a pyro channel - the other channels are not affected
#define M433_PING_AD (7u)//7 - respond to Master with programing details for a specific mode/line ID; 
#define M433_PING_PS (8u)//8 - programing only a specific channel from a module (the other channels will not be affected) 
#define M433_PING_PA (9u)//9 - programing a channel on a module - if the module was programmed before entire script will be deleted
#define M433_PING_FP (10u)//10 - finish programing a module
#define M433_PING_DS (11u)//11 - Delete programing from a module
#define M433_PING_MD5 (12u)//12 - Send MD5 hastag on request
#define M433_SEQ_FIRE (13u)//13 - Send Sequence fire
#define M433_PING_MIR_PRG (14u)//14 - reply with mirro message when programing
#define M433_AB_MASTER (15u)//15 - Anounce an Audiobox as Master
#define M433_AB_TIMECODE (16u)//16 - START/STOP Time code in test on an audiobox


#define M433_PING_1_REPLY (21u)//21 - reply with ping top module stat 1
#define M433_PING_2_REPLY (22u)//22 - reply with ping top module stat 2
#define M433_PING_AB_REPLY (23u)//23 - reply with ping top AB stat
#define M433_PING_AB_SYNC  (24u)//24 - AB sent a message with sync time



#define M433_SETTINGS  (50u)//24 - 433Mhz wireless Settings message
#define M433_BCK_MASTER         (60u)//Backup_master anounce

#define M433_BACKUP_MASTER_REPLY  (30u)//24 - Master sent a message looking for backup master

#define M433_STOP_PING_SYNC_FIRE        (201u)//201 - Stop//Ping in Test
#define M433_ARM_SYNC_FIRE              (202u)//202 - Arm//sync in Arm
#define M433_PLAY_SYNC_FIRE             (203u)//203 - Start//sync in Play
#define M433_PAUSE_SYNC_FIRE            (204u)//204 - Pause//sync in Pause

#define M433_PLAY_ABM_FIRE              (205u)//203 - Start show when AB is Master

#define M433_AB_CMDS                    (210u)//210 - Audiobox commands



//#define MASTER_SLAVE_CANTerminal				(2007)
//#define MASTER_SLAVE_WirelessPower				(2008)
#define MASTER_SLAVE_AudioBOXadr				(2008)
#define MASTER_SLAVE_InterfaceStatus		                (400)
#define MASTER_SLAVE_MatrixA1Status				(500)
#define MASTER_SLAVE_MatrixA2Status				(600)
#define MASTER_SLAVE_MatrixA3Status				(700)
#define MASTER_SLAVE_MatrixB1Status				(800)
#define MASTER_SLAVE_MatrixB2Status				(900)
#define MASTER_SLAVE_PingA						(1000)
#define MASTER_SLAVE_PingB						(1100)

#define SLAVE_MASTER_TIME						(1200)// read channel time for edits


#define MASTER_SLAVE_NewStatus             (1700)//send new status
#define MASTER_SLAVE_PingBP                 (1800)// Send Matrix P (prg or no) and B (enable/disable)

#define MASTER_SLAVE_MD5                                                                                        (2021)// MD5 message



#define PYRO_CH_MAX                                      (16) // Maximum number of pyro channel per RAIL
#define PYRO_RAIL_MAX                                    (4) // Maximum number of Rails per module





#define TC_UART                          (LPC_UART2) 



//#define BT_UART                         (LPC_UART3)



#define MATRIX_LINE_PINS		(0x3FE3C700u)
#define MATRIX_COLUMN_PINS		(0x801C0000u)





#define GPS_PPS_PORT       (GPIOE)
#define GPS_PPS_PIN        (GPIO_PIN_7)







#define MTX_LNS_CHECK_CH                (1) // ADC channel to test lines

#define MTX_COL_CHECK_CH                (8) // ADC channel to test columns
#define COL_TST_PIN                     (5) // PIN to enable column check
#define COL_TST_PORT                    (2) // PORT to enable column check


#define MTX_CHECK_CH                    (2) // ADC channel to test MTX voltage


#define PC_CAP2                         (6) // ADC channel to test Cap 2 (test) voltage
#define PC_CAP5                         (5) // ADC channel to test cap 5 (arm/full) voltage

#define SSP_PAUSE_T                      (100) // ADC SPI Pause

#define BAT_VOLT                         (9) // ADC channel to test Cap 2 (test) voltage



#define ADC_PORT				(0)
#define ADC_PIN_MOSI			(9)
#define ADC_PIN_MISO			(8)
#define ADC_PIN_CLK				(7)
#define ADC_PIN_SDA				(6)






//for FTH-48Fx
#define A_Value					(15)
#define B_Value					(40)
#define C_Value                                  (240)



// Values for 0R
#define MIN_MTX_V                               (450)
#define MAX_MTX_V                               (650)


/******************************************************************************/
/* TODO to be set to value of 10dBm */
#define DEFAULT_WIRELESS_OUTPUT_POWER (0) /* value is in dBm */

/* defines allowed pause betwen two incoming message over different communication interfaces */
#define SLAVE_DUPLICATE_INTERFACE_MESSAGE_PERIOD (30) /* value in ms */

/* number of milliseconds between requests for wireless device value reading (temperature, signal percentage...) */
#define WIRELESS_VALUES_POLL_PERIOD 		(2000)
#define WIRELESS_RESET_PERIOD				(240000)
#define TIME_DELAY_BETWEEN_WIRELESS_FRAMES	 (10)
#define CAN_DisconnectTime					(15000)

#define LVL_VERY_QUIET                          85
#define LVL_QUIET                               95




//Buzzer Port
#define BUZ_PORT                         (3)
#define BUZ_PIN                         (25)
#define BUZ_LEN                         (100)



// ADC inputs
#define MTX_CH                          (7) // matrix test input channel

#define C5_ADC                          (5) // matrix total voltage
#define BAT_C5_R1                       (91)
#define BAT_C5_R2                       (10)



//#define ADC_rez                         (0.00322265625f) //for 10b
//#define ADC_rez                         (0.0008056640625f) //for 12b
#define ADC_rez                         (0.000806f) //for 12b




#define Min_Battery			        (3.45f)
#define BLVL0			                        (3.60f)
#define BLVL1			                        (3.70f)
#define BLVL2               			        (3.80f)
#define BLVL3			                        (3.85f)
#define BLVL4               			        (3.95f)
#define BLVL5			                        (4.50f)

#define ADC_UPDATE_TIME				(10)
#define VAL_BAT_LDO                             (2.5f)
#define BATTERY_ADC_CH  (3)
#define VBUS_ADC_CH     (4)

#define USB_VAL_VBUS     (700)




//Time code UART port
#define TC_TX_Pin GPIO_PIN_7
#define TC_TX_GPIO_Port GPIOF
#define TX_RX_Pin GPIO_PIN_6
#define TX_RX_GPIO_Port GPIOF


#define TIME_CODE_FSK_PIN GPIO_PIN_8
#define TIME_CODE_FSK_PORT GPIOB



#define AP_TEST_TIME                    (500)

#define MAX_SCRIPT_LENGTH_LINE                 (24)
#define MAX_SCRIPT_LENGTH_LINE_N               (24)
#define MAX_SEQ_NAME                           (15)
#define MIN_SEQ_START_NAME                      (3)


#define MAX_SEQ                                         (65)
#define MAX_SZ                                          (16)


#define MAX_SCRIPT_LINES                (5)




#define MAX_NOT_RESPOND_TIME                                    (60000)
#define MAX_DISC_TIME                                           (35000)

#define MAX_NOT_RESPOND_TIME_PER_MOD                            (1200)
#define MAX_DISC_TIME_PER_MOD                                   (1500)


/**********************SMPTE*********************************************/

#define SMPTE_Pin GPIO_PIN_6
#define SMPTE_GPIO_Port GPIOE
#define SMPTE_EXTI_IRQn EXTI9_5_IRQn

#define TIME_CODE_BACKUP        1 //(0 - default Pause if Time code is stoped, 1 - Continue with internal clock)

//for 33fps

#define one_time_min30          360 // 
#define one_time_max30          470 // 
#define zero_time_min30         760 // 
#define zero_time_max30         870 // 

//for 25fps

#define one_time_min25          450 // 
#define one_time_max25          550 // 
#define zero_time_min25         950 // 
#define zero_time_max25         1050 // 

/*
//for 33fps
#define one_time_min30          360 // 350
#define one_time_max30          450 // 450
#define zero_time_min30         750 // 760
#define zero_time_max30         880 // 870

//for 25fps
#define one_time_min25          450//450 // 
#define one_time_max25          550//550 // 
#define zero_time_min25         950//950 // 
#define zero_time_max25         1050//1050 // 
*/

//for 25fps Detect
#define one_time_min25D          480
#define one_time_max25D          510
#define zero_time_min25D         960
#define zero_time_max25D         1000

//for 33fps Detect
#define one_time_min30D          360 // 360
#define one_time_max30D          450 //450 
#define zero_time_min30D         750 // 770
#define zero_time_max30D         880 // 830


#define FSK_F1_Time_low         830
#define FSK_F1_Time_high        840
#define FSK_F1_Time_lowX        1000
#define FSK_F1_Time_highX       1020


#define FSK_PD_Time_low          450
#define FSK_PD_Time_high         480
#define FSK_PD_Time_lowX         740
#define FSK_PD_Time_highX        770


#define MAX_TC_TIME_OUT_WAIT        10



#define MIN_TC_VALID            4


#define WAIT_ON_FSK_F1         (1000)
#define WAIT_ON_LTC_TC          (50)
#define WAIT_ON_FSK_PD          (200)


#define end_data_position      63
#define end_sync_position      77
#define end_smpte_position     80



#define TG_Low_30fps             (208)
#define TG_Low_25fps             (250)

#define MIN_TIME_TC_WAIT                                  (50)
#define MIN_TIME_TC_WAIT_FSK                              (1000)





#define TIME_CODE_Pos                         (1) // define Time code output position in settings menu - OK
#define TIME_ZONE_Pos                         (2) // define Time zone position in settings menu - OK
#define START_TIME_Pos                        (3) // define Start GPS_Time position in settings menu - OK
#define PWR_BUT_Pos                           (4) // define Power button disable position in settings menu - OK
#define AUTOSCROLL_Pos                        (5) // define Autoscroll position in settings menu
#define SAFE_CON_Pos                          (6) // define Safe connection position in settings menu - OK
#define TALON_MOD_Pos                         (7) // Enable or disable Talon mode - OK
#define SOLO_VALID_Pos                        (8) // is solo valid - to fire wwith module without a remote - OK
#define R433_ARM_Pos                          (9) // Enable or disable possibility to arm form 433mhz remote - OK
#define R433_EN_Def                           (10) // Enable as default 433Mhz remote when is learned - OK
#define R433_Crew_Ctrl                        (11) // Crew Pause with small remote - OK
#define Ext_Trg_ctrl                          (12) // External trigger - OK
#define ID_SET_Pos                            (13) // ID SET - OK
#define BUZ_EN_Pos                            (14) //Buzzer Enable

#define MAX_PD_TIME                            (3276700)


#define MAX_MENU_SETTINGS                     (15) //Maximum settings

#define CFG_EDT_PRESS_TIME                    (300)// long press to edit current configuration




#define  MAX_USB_TIME                   (10000) //Maxim time to keep the USB powered on without be connected an USB drive



//Rottaery encoder
#define ROT_ENCODE_A_PIN GPIO_PIN_11
#define ROT_ENCODE_A_PORT GPIOD

#define ROT_ENCODE_B_PIN GPIO_PIN_10
#define ROT_ENCODE_B_PORT GPIOD

#define ROT_ENCODE_D_PIN GPIO_PIN_9
#define ROT_ENCODE_D_PORT GPIOD

//LCD RST
#define LCD_RST_Pin GPIO_PIN_4
#define LCD_RST_GPIO_Port GPIOC


//LCD CS
#define LCD_CS_PIN GPIO_PIN_4
#define LCD_CS_PIN_GPIO_Port GPIOA

//LCD A0
#define LCD_A0_Pin GPIO_PIN_5
#define LCD_A0_GPIO_Port GPIOC





//Buzzer
#define BUZ_Pin GPIO_PIN_6
#define BUZ_GPIO_Port GPIOA


//DI 
#define DI_Pin GPIO_PIN_2
#define DI_GPIO_Port GPIOA


//TimeCode generator
//built in

#define CTR_TC_Pin GPIO_PIN_8
#define CTR_TC_GPIO_Port GPIOF

//FSK Board
/*
#define CTR_TC_Pin GPIO_PIN_0
#define CTR_TC_GPIO_Port GPIOG
*/

//RCV Out
#define RCV_Out_Pin GPIO_PIN_3
#define RCV_Out_GPIO_Port GPIOA


//OP CMD
#define OP_CMD_Pin GPIO_PIN_2
#define OP_CMD_GPIO_Port GPIOE


//Red Start
#define RED_START_Pin GPIO_PIN_15
#define RED_START_Port GPIOE

//Green Start
#define GREEN_START_Pin GPIO_PIN_14
#define GREEN_START_Port GPIOE

//Blue Start
#define BLUE_START_Pin GPIO_PIN_9
#define BLUE_START_Port GPIOE



//Red Panel
#define RED_PANEL_Pin GPIO_PIN_6
#define RED_PANEL_Port GPIOD

//Green Panel
#define GREEN_PANEL_Pin GPIO_PIN_2
#define GREEN_PANEL_Port GPIOC

//Blue Panel
#define BLUE_PANEL_Pin GPIO_PIN_15
#define BLUE_PANEL_Port GPIOF




//M-
#define MM_BUT_PORT GPIOB
#define MM_BUT_PIN GPIO_PIN_0
#define MM_BUT_POS 0

//M+
#define MP_BUT_PORT GPIOG
#define MP_BUT_PIN GPIO_PIN_1
#define MP_BUT_POS 1


#define MAX_BUT_NR_LED 31

//DMS 1
#define DMS1_BUT_PORT GPIOD
#define DMS1_BUT_PIN GPIO_PIN_5
#define DMS1_POS 31

//DMS 2
#define DMS2_BUT_PORT GPIOD
#define DMS2_BUT_PIN GPIO_PIN_4
#define DMS2_POS 32

//DMS 3
#define DMS3_BUT_PORT GPIOD
#define DMS3_BUT_PIN GPIO_PIN_3
#define DMS3_POS 33


//ARM KEY
#define AK_BUT_PORT GPIOG
#define AK_BUT_PIN GPIO_PIN_7
#define ARMK_POS 34

/*
//ARM KEY EXT
#define AKE_BUT_PORT GPIOG
#define AKE_BUT_PIN GPIO_PIN_7
#define ARMKE_POS 35
*/

#define MAX_BUT_NR 35

#define MAX_PRESSED_TIME 1000
#define MIN_SHORT_PRESS_TIME 6


#define MIN_LONG_PRESS_TIME  500
#define MIN_LONG_PRESS_TIME_X   600


#define MIN_PRESS_TO_ARM_DISARM   1500//1500

#define MIN_UNPRESSED_TIME 3


#define BUTTON_LIGHT_TIME   7
#define BUTTON_READ_TIME 10



/*
#define BUTTON_LIGHT_TIME   17
#define BUTTON_READ_TIME 20
*/


//POWER BUTTON

/* //v1.4
#define PWR_BUT_PORT GPIOG
#define PWR_BUT_PIN GPIO_PIN_10
*/

//v1.5
#define PWR_BUT_PORT GPIOC
#define PWR_BUT_PIN GPIO_PIN_13


#define PWR_CTRL_Port                  PWR_BUT_PORT
#define PWR_CTRL_Pin                   PWR_BUT_PIN

#define PWR_STAT_Port                  GPIOE
#define PWR_STAT_Pin                   GPIO_PIN_4


#define DMX_TX_Port                  GPIOE
#define DMX_TX_Pin                   GPIO_PIN_1

#define DMX_RX_Port                  GPIOE
#define DMX_RX_Pin                   GPIO_PIN_0

#define DMX_CTRL_Port                  GPIOE
#define DMX_CTRL_Pin                   GPIO_PIN_2






#define MAXIM_RAILS     4
#define CHAN_STAT       25


#define MAXIM_STATS     21

#define IDs_POS          0 //0 to 99
#define BATTERYs_POS     1 //0 - Not connected, 1 to 6 charge Level
#define WIRELLESSs_POS   2 //0 - Not connected, 1 to 6 signal Level
#define PROGs_POS        3 //0 - Not programmed, 1 - Programmed with Pyro
#define TH_valid_POS     4 //0 - Not available, 1 - valid Temperature, 2 - Valid Humidity, 3 Valid Temp + Humidity
#define TEMPs_POS        5 //x - Temperature value
#define HUMs_POS         6 //x - Humidity value %
#define WIRE2s_POS       7 //0 - NOT Available, 1 - Not connected, 2  - Connected 10 - Disconnected
#define TCs_POS          8 //0 - NOT Available, 1 - Disabled, 2 - Wait for TC, 10 - Read TC25, 11 - Read TC33, 12 - Read TC FSK F1, 13 - Read TC FSK PD, 20 - Generate TC25, 21 - Generate TC33, 22 - Generate TC FSK F1, 23 - Generate TC FSK PD
#define APs_POS          9 //0 - NOT Available, 1 - Disabled, 2 - Enabled Internal memory, 3 - Enabled USB, 10 - Error File name, 11 - Error memory, 12 Error playing file
#define APx_POS          10 //0 - NOT Available, 1 - Play, 2 - Pause, 3 - Stop, 10 - Error
#define EXT_BAT_POS      11 //0 - Not available
#define MOD_KIND_POS     12 //0 - Not available
#define TIME_POS         13 //0 - Not available
#define MOD_FAULTS       16 //0 - Module Faults
#define MOD_WNG          17 //0 - Module Warnings
#define IS_ONLINE        18 //0 - Module Warnings
#define WIRELLESS_433s_POS   19 //0 - Not connected, 1 to 6 signal Level

#define IS_RCSTAT_POS     ERRS_POS - 1 //0 - Not available

#define ERRS_POS         15

#define TEM_HUM_REFRESH 6000


#define PRG_POS         0
#define FLT_POS         1
#define WNG_POS         2

#define MAXIM_STATS_ME  3



#define FAULTS_POS          0
#define WARNING_POS         1
#define MODS_CON_POS        2
#define MODS_NEED_POS       3
#define MODS_REM_POS        4
#define SCRIPT_NAME         5


#define MAX_SYS_STATS       10



#define LINE2A_SYS_STATS    7

#define SLAVES_CON_F        0 //0
#define SLAVES_CON_W        1 //0
#define BAT_F               2 //0
#define BAT_W               3 //0
#define PYRO_EV_REMAIN_POS  4 //0
#define DMX_EV_REMAIN_POS   5 //0
#define IS_LINE2A_POS       LINE2A_SYS_STATS - 1 //0



#define LINE2B_SYS_STATS    5
#define SYS_STATE_POS       0 //0
#define SLAVES_CON_POS      1 //0
#define DEV_TIME_POS        2 //0
#define SLAVES_ARM_POS      3 //0


#define IS_LINE2B_POS       LINE2B_SYS_STATS - 1 //0



#define LINE2C_SYS_STATS    7

#define SCRIPT_FAULT_POS    0 //0
#define SCRIPT_WNG_POS      1 //0
#define OTHER_FAULT_POS     2 //0
#define OTHER_WNG_POS       3 //0

#define TIME_LEFT_POS       4 //0
#define TIME_TO_POS         5 //0

#define IS_LINE2C_POS       LINE2C_SYS_STATS - 1 //0




/*
//LED GREEN
#define LEDG_PORT GPIOD
#define LEDG_PIN GPIO_PIN_6

//LED RED
#define LEDR_PORT GPIOC
#define LEDR_PIN GPIO_PIN_2

//LED BLUE
#define LEDB_PORT GPIOF
#define LEDB_PIN GPIO_PIN_15
*/


#define BLINK_TIME              15 //Time to Blink buttons last fire
#define BLINK_TIME_SLOW         100 //Time to Blink buttons last fire



//Enable USB Host 5V

#define HOST_POWERSW_PORT                  GPIOD
#define HOST_POWERSW_VBUS                  GPIO_PIN_0

#define SLEEP_PWR_PORT                 GPIOB
#define SLEEP_PWR_PIN                  GPIO_PIN_4



#define TOUCH_INT_PORT GPIOD
#define TOUCH_INT_PIN GPIO_PIN_9

#define TIME_ERROR_CHANGE       1500

//Slave Errors
#define DISC_NOT_CON_F          0 // disocnnected or not connected
#define SCRIPT_F                1 // Script, MD5 or module not programmed and rem programmed Faults
#define BATTERY_F               2 // Battery fault as is too low
#define OTHERS_F                3 //others


#define LS_NOTS_W               0 // Low signal or not in script
#define BATTERY_W               1 // battery warning as it is low
#define SCRIPT_W                2 //connected but not in script
#define OTHERS_W                3 // Other warnings

#define MAX_ERRORS              15 // Maxim errors to display


#define DEFAULT_FUNC         0
#define MANUAL_FIRE_FUNC     1 
#define SEQ_FIRE_FUNC        2 
#define SZ_FIRE_FUNC         3 
#define AP_FUNC              4
#define SET_FUNC             6
#define MANUAL_FIRE_X_FUNC   7


// area of last 4 columsn display
#define CTRL_DISPLAY            0
#define MOD_DISPLAY             1
#define SCRIPT_DISPLAY          2
#define SEQ_DISPLAY             3
#define SZ_DISPLAY              4
#define AP_DISPLAY              5
#define SET_DISPLAY             6
#define MAN_X_DISPLAY           7



//Settings Functions
#define SET_TIME                0
#define SET_TIMEZone            1
#define SET_TIMEStart           2

#define SET_TIME_CODE           10
#define SET_TIME_CODE_GEN       11

#define SET_TIME_CUST_NET       20


#define MAX_TIME_WAIT_FOR_POWER_OFF       5



#define DISABLE_START_TIME        1500



//I2S3 Audio out PIN COnfiguration
#if 1
#define I2S3                            SPI3

#define I2S3_CK_AF                      GPIO_AF6_SPI3
#define I2S3_CK_PIN                     GPIO_PIN_3
#define I2S3_CK_GPIO_PORT               GPIOB

#define I2S3_SD_AF                      GPIO_AF6_SPI3
#define I2S3_SD_PIN                     GPIO_PIN_5
#define I2S3_SD_GPIO_PORT               GPIOB

#define I2S3_WS_AF                      GPIO_AF6_SPI3
#define I2S3_WS_PIN                     GPIO_PIN_15
#define I2S3_WS_GPIO_PORT               GPIOA

#endif


#define MIN_LCD_BLIGHT               5
#define DEFAULT_LCD_BLIGHT           80



#define EXT_TRIG_PORT           GPIOE
#define EXT_TRIG_PIN            GPIO_PIN_5

#define GreenButton_GPIO_Port           GPIOG
#define GreenButton_GPIO_Pin            GPIO_PIN_0

#define BlueButton_GPIO_Port           GPIOG
#define BlueButton_GPIO_Pin            GPIO_PIN_1



        //Stop Charge - NOT USED
        #define STP_CHG_PORT                  GPIOB
        #define STP_CHG_PIN                   GPIO_PIN_2
        
       //Audio Power Control for Balanced Outputs // NOT USED
        #define AP_PWR_CTRL_PORT                  GPIOD
        #define AP_PWR_CTRL_PIN                   GPIO_PIN_10
        
        #define SD_DETECT_PIN                   GPIO_PIN_1                /* PA.08 */
        #define SD_DETECT_GPIO_PORT             GPIOD                       /* GPIOA */
        
        #define RES_A_PORT GPIOG
        #define RES_A_PIN  GPIO_PIN_5

        #define RES_B_PORT GPIOG
        #define RES_B_PIN  GPIO_PIN_4
        
        
        #define WIRELESS_RX   GPIO_PIN_7  
        #define WIRELESS_TX   GPIO_PIN_6
        #define WIRELESS_PORT GPIOB
        
        #define L02_BUT_PORT GPIOE
        #define L02_BUT_PIN GPIO_PIN_14
        #define L02_POS 8
        
        //VBUS Pin
        #define USB_VBUS_Pin GPIO_PIN_9
        #define USB_VBUS_Port GPIOA
        
        
        #define USB_DETECT_PIN                   GPIO_PIN_10                
        #define USB_DETECT_GPIO_PORT             GPIOA  


        #define PWR_STAT_PORT                  GPIOE
        #define PWR_STAT_PIN                   GPIO_PIN_4
        


#define TIME_FOR_TC_FULL                  2000 // minimum time to remain time code 
#define TIME_FOR_TC_START                 500 /// min time to be presed to wormk time code



#define TST_DEBUG       0
#define TEST_DBG        0

#define PRG_DBG         0


#define IS_TEST_WIRELESS        1

#define IS_433_MHZ_ENABLED      1

#define ENGLISH 0
#define ITALIAN 1
#define SPANISH 2
#define FRENCH  3
#define RUSSIAN 4

#define FW_VER          "0.79d LR"

#define TIME_PWR_OFF_BUT          5000
        
#define HW_VER          3

#if HW_VER == 2
#define LCD_BL_PORT GPIOD
#define LCD_BL_PIN GPIO_PIN_8

#elif HW_VER == 3
//LCD Backlight pin
#define LCD_BL_PORT GPIOD
#define LCD_BL_PIN GPIO_PIN_7

#endif







#endif

