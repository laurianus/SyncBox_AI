/***************************************************************************//**
 * @file     Wireless.h
 * @brief    Jennic JN5139-M04 wireless driver functions and data
 ******************************************************************************/

#ifndef _WIRELESS_H
#define _WIRELESS_H

#include "main.h"
#include "Device.h"

/**
@def TEMPERATURE_MIN
@brief Minimum valid temperature readout value
*/
//#define TEMPERATURE_MIN (-40.0f)

/**
@def TEMPERATURE_MAX
@brief Maximum valid temperature readout value
*/
//#define TEMPERATURE_MAX (85.0f)

/**
@def MAX_WIRELESS_MESSAGE_SIZE
@brief Maximum allowed length of the wireless message
*/
//#define MAX_WIRELESS_MESSAGE_SIZE 128
#define MAX_WIRELESS_MESSAGE_SIZE 256

/**
@def MAX_NUMBER_OF_WIRELESS_PARAMETERS
@brief Maximum number of received message elements in one message
*/
#define MAX_NUMBER_OF_WIRELESS_PARAMETERS 10

#define MAX_NUMBER_OF_MESSAGE_PARAMETERS   6
#define MIN_NUMBER_OF_MESSAGE_PARAMETERS   4

/**
@typedef WIRELESS_STATE
@brief States of the wireless device
*/
typedef enum WIRELESS_STATE{
	WIRELESS_INIT, /*< Device is in initialisation state (joining the wireless network)*/
	WIRELESS_IDLE, /*< Device is in idle state (fully operational)*/
	WIRELESS_BUSY,
	WIRELESS_ERROR, /*< Device is in error state (not functional and not connected to the network)*/
}WirelessState;


/**
@typedef WIRELESS_ADDRESS_TYPE
@brief Message addressing types
*/
typedef enum WIRELESS_ADDRESS_TYPE{
	WIRELESS_MESSAGE_BROADCAST,
	WIRELESS_MESSAGE_ADDRESSED
}WirelessAddressType;


/**
@typedef WIRELESS_MESSAGE_CODE
@brief Types of the wireless messages
*/
typedef enum WIRELESS_MESSAGE_CODE{
	WIRELESS_SET_OUTPUT_POWER_VALUE,
	WIRELESS_GET_SIGNAL_VALUE,
	WIRELESS_GET_SIGNAL_PERCENTAGE,
	WIRELESS_GET_OUTPUT_POWER_VALUE,
	WIRELESS_GET_TEMPERATURE_VALUE,
	WIRELESS_SEND_DATA	
}WirelessMessageCode;


/**
@struct WirelessValues
@brief Various values read from the wireless device (signal value, temperature...)
*/
typedef struct WIRELESS_VALUES{
	float temperature; /*!< Device chip temperature. Error value is less than TEMPERATURE_MIN */
	int signalPercentage; /*!< Receiver signal percentage. Error value is INT32_MIN */
	int signalValue; /*!< Receiver signal value (in dBm). Error value is INT32_MIN */
	int outputPower; /*!< Transmitter output power value (in dBm). Error value is INT32_MIN */
}WirelessValues; /*!< Various values read from the wireless device (signal value, temperature...) */


/**
@typedef WIRELESS_TX_MSG_STATUS
@brief Enum used for presenting wireless outgoing message status
*/
typedef enum WIRELESS_TX_MSG_STATUS{	
	WIRELESS_TX_OK, /*< Message transfered to outgoing FIFO */
	WIRELESS_TX_ERROR/*< Error while sending the message to the outgoing FIFO */
}Wireless_TxMsgStatus;


/**
@typedef WIRELESS_RX_MSG_STATUS
@brief Enum used for presenting wireless incoming message status
*/
typedef enum WIRELESS_RX_MSG_STATUS{
	WIRELESS_RX_OK, /*< Message received succesfully */
	WIRELESS_RX_NO_DATA, /*< No available data in input FIFO */
	WIRELESS_RX_ERROR /*< Error while reading incoming wireless message from the FIFO */
}Wireless_RxMsgStatus;

/**
@fn	void SetWirelessType(WirelessType type)
@param[in] type Wireless device type (router or coordinator)
@brief Set the type of the wireless device and reboot it
*/
void SetWirelessType(void);


/**
@fn	void InitWireless(void)
@brief Initialise wireless device
*/
void InitWireless(void);


/*
typedef struct _devMsg{
	uint8_t address; //< Device address. It is 32-bit wide so we can use it with all CAN ID's
	uint8_t dataLength; //< Size of the data in the "data" array
	uint8_t command;
	uint8_t data[32]; //< Array containig the messsage data payload 
}PC_Message;

*/

/*
typedef struct _devMsg1{
	uint16_t address; //< Device address. It is 32-bit wide so we can use it with all CAN ID's
	uint8_t dataLength; //< Size of the data in the "data" array
	uint8_t data[32]; //< Array containig the messsage data payload 
}MS_Message;
*/


/**
@fn	Wireless_TxMsgStatus Wireless_Send(const Device_Message *message)
@param[in] message pointer to the message that is to be sent
@return Wireless_TxMsgStatus enum value
@brief Send message to the wireless device
*/
Wireless_TxMsgStatus Wireless_Send(const MS_Message *message);


/**
@fn	Wireless_RxMsgStatus Wireless_Receive(Device_Message *message)
@param[in] message Pointer to the message that will hold the result
@return Wireless_RxMsgStatus enum value
@brief Read incoming wireless message (if any)
* /
Wireless_RxMsgStatus Wireless_Receive(MS_Message *message);*/


/**
@fn	Status IsWirelessDetected(void)
@return SUCCESS/ERROR
@brief Check is wireless device detected on the UART
*/
uint8_t IsWirelessDetected(void);

/**
@fn	void DoWirelessTasks(void)
@brief Wireless device housekeeping tasks
*/
void getWirelessMessage(void);

void saveAB_adr(MS_Message *msg);

void clearDeviceAddressForWireless(uint32_t address);

void send_tst_message(void);//for tests


//Netw 01.07.2017
void set_Wireless_NetXS(void);
void set_Wireless_Net(void);
void set_Wireless_NetX(void);
//END

void rstWireless(void);

void Config_WirX(void);
void set_Wireless_NetAAAA(void);
void startWireless(void);

void getWirMessages(void);


void startWirelessAB(void);
void startWirelessM(void);

void hard_reset_wireless(void);
void config_PAN_EPAN(uint8_t network_set);
void config_basic_wir(void);

void Config_Wir(uint8_t wir_cfg_mode);
void MX_WIRX_TX_Init(uint32_t uart_speed);
uint8_t get_ABox(void);

void reset_Duplicate_ID(int tmp_pops);

void reset_wireless_buffer(void);
uint8_t get_is_wireless_mod_OK(void);
void rst_wirelessDeviceTableAB(void);
void rst_wirelessDeviceTableMod(void);


void scan_energy(void);
uint8_t hex_to_int(char c);
int8_t hex_to_ascii(char c, char d);
void update_signal_stats(void);
void update_signal_values(void);
void sor_rec_chans(void);
void wireless_analyzer(void);
uint8_t get_val_noise(uint8_t chan_x);
uint8_t get_Wchan_stat(uint8_t chan_a);
void rst_wireless_analyzer(void);
void change_wir_chan(uint8_t new_chan);
void get_wir_info(void);

void config_PAN_EPAN_new(void);

#endif
