/***************************************************************************//**
* @file     Wireless.c
* @brief    Jennic JN5139-M04 wireless driver functions and data
******************************************************************************/

#include <string.h>
#include <stdio.h>
#include "stm32f4xx_hal.h"


//#include "lpc17xx_pinsel.h"
//#include "lpc17xx_gpio.h"
//#include "SystemTimer.h"
//#include "UART.h"

#include "options.h"
#include "Wireless.h"
#include "Buttons.h"
#include "main.h"
#include "uart.h"
#include "SystemTimer.h"

//Netw 01.07.2017
//#include "wdt.h"
//#include "Device.h"


#ifndef VS_READABLE_CODE
#include "System1.h"
#include "System1Action.h"
#include "System1Data.h"
#endif

#include "simpleEventHandler.h"         // Event Queue
#include "System1SEMLibB.h"                    // visualSTATE essentials

/**
@def WIRELESS_RING_BUFSIZE
@brief Size of the ring buffer for incoming messages from the wireless
*/
//#define WIRELESS_TX_BUF_SIZE  			100
#define WIRELESS_TX_BUF_SIZE  			16

uint8_t isCoordAdrSet = 0;

uint8_t wireless_disable = 0;


uint8_t is_wireless_respopnd = 0;

uint8_t is_Wir_Slot_Detect = 0;

uint8_t was_SET_Disp = 0;

uint16_t was_scan_complete = 0;
uint8_t chan_energy[MAX_WIR_CHAN] = {0};

//uint16_t Max_chan_energy[MAX_WIR_CHAN][3] = {{0}};

uint8_t Med_chan_energy[MAX_WIR_CHAN] = {0};

uint8_t last_chan_energy[MAX_WIR_CHAN] = {0};
uint8_t scan_wifi_energ = 0;

uint8_t Rec_chan_energy[MAX_WIR_CHAN] = {0};

uint8_t current_wir_chan = 0;
char wir_network_rem_name[4] = {0};


uint32_t t_spanaaaa = 0;

SlaveInfo slaves[MAX_NUMBER_OF_DEVICES];

audioBoxA AudioBoxes[MAX_NUMBER_OF_AB + 1];

uint32_t LastPingTime[MAX_NUMBER_OF_DEVICES] = {0}; // last ping time
int Ping_Slave_Time[MAX_NUMBER_OF_DEVICES];
uint8_t Pinged_Slave[MAX_NUMBER_OF_DEVICES];

long int DMX_Ramp_Vect[DMX_CH_MAX][3] ={{0}};


uint8_t tempBuf[128] = {0};


extern uint8_t was_started_config;

char Network_Name[6] = {0};

//uint8_t audio_boxes_con = 0;

char tmp_CMD_PAN[24] = {0};
char tmp_CMD_EPAN[24] = {0};

//uint16_t is_msg_snt_dbg = 0;

uint8_t is_wireless_ok = 0;

uint8_t Wireless_add_req = 0;
uint8_t get_wireless_name = 0;



//extern UART_HandleTypeDef huart4;
extern UART_HandleTypeDef huart1;

uint8_t is_duplicatID = 0;
uint8_t NO_network = 0;

uint64_t t_span_wir = 0;

uint8_t ID_duplicated[MAX_NUMBER_OF_DEVICES] = {0};
uint8_t ID_duplicatedX = 0;

uint8_t is_wireless_setup = 0;
uint8_t wireless_mode = 0;

uint64_t t_span = 0;

const char *cfgDetection = "ATI"; 
const char *cfgHReset = "AT&F";
const char *cfgSerial="ATS12=0C10";//no echo
//const char *cfgSerial="ATS12=0C00";//with echo

const char *cfgPower = "ATS01=-7";
const char *cfgChannelMask = "ATS00=6718"; //"ATS00=FFFF";

const char *cfgResponses = "ATS10=1002"; // "ATS10=400A";"ATS10=100A"
const char *cfgResp2="ATS0E=BFC0";


const char *cfgLinkKey = "ATS09=987654321:password";
const char *cfgTransmissionPower = "ATS0F=4007"; //"ATS0F=C002";

const char *cfgInterogate="AT+N?";
const char *cfgRouter="AT+JN";
const char *cfgDisconnect = "AT+DASSL";
const char *cfgReset="ATZ";

const char *cfgChChg="AT+CCHANGE"; //AT+CCHANGE[:XX]


const char *cfgWifiScan="AT+ESCAN";

const char *cfgCoordinator="AT+EN";

const char *getNetAddr="ATS04?";
const char *getNetName="ATS02?";

const char *getSerial="ATS12?";


uint16_t cf_wait_time = 1000;

extern uint8_t Mod_toPrg;


typedef enum
 {
       E_WIRELESS_DETECTION_IDLE       = 0u,
       E_WIRELESS_DETECTION_STARTED,
       E_WIRELESS_DETECTION_OK,
       E_WIRELESS_DETECTION_FAILED
 } E_WirelessDetectionState;

#define WIRELESS_DETECTION_IDLE                 0
#define WIRELESS_DETECTION_
/**
@var const char textEnd[]
@brief Bytes placed at the tail of the wireless message
*/
const char textEnd[]={(char)0x0D, '\0'};

/**
@var const char delimiters[]
@brief Wireless message delimiters used for message parsing
*/
const char delimiters[]={',', (char)0x0D, (char)0x0A, ':', '='};

/**
struct WIRELESS_RING_BUFFER_T
@brief Structure that defines incoming wireless message ring buffer
*/
typedef struct _WIRELESS_RING_BUFFER_T{
      __IO uint16_t rdIdx;                /*!< Wireless Rx ring buffer head index */
      __IO uint16_t wrIdx;                /*!< Wireless Rx ring buffer tail index */
      MS_Message buffer[WIRELESS_TX_BUF_SIZE];             /*!< Wireless Rx data ring buffer */
      uint16_t count;
}WIRELESS_RING_BUFFER_T;

//uint8_t is_Wir_Slot_Detect = 1; //Detect module from Wireless slot by checking communication

/**
@var WIRELESS_RING_BUFFER_T wirelessBuffer
@brief Variable containig incoming wireless message ring buffer
*/
WIRELESS_RING_BUFFER_T wirelessBuffer;

/**
@var uint64_t coordinatorAddress
@brief MAC address of the wireless coordinator
@details Used in slave or unknown device modes
*/
uint64_t coordinatorAddress = 0ul;
uint64_t AB_Address = 0ul;

uint64_t rull_NodesNetworkAddress = 0ul;



//Netw 01.07.2017
uint8_t WasChanged = 0;
uint64_t coordinatorAddressX = 0ul;

#define MAX_NUMBER_OF_DEVICES (100)

//static uint64_t wir_msg_wait = 0u;

/**
@var WirelessState wirelessState
@brief Current state of the wireless interface
*/
WirelessState wirelessState=WIRELESS_INIT;

/**
@var uint64_t wirelessDeviceTable[MAX_NUMBER_OF_DEVICES]
@brief Look-up table containing the relation of the device address
(the same address as used on the CAN-bus) and the wireless interface MAC address.
@details Used in master device mode operation
*/
//uint64_t wirelessDeviceTable[MAX_NUMBER_OF_DEVICES + 1]={0};
uint64_t wirelessDeviceTable[MAX_NUMBER_OF_DEVICES]={0};

uint64_t wirelessDeviceTableAB[MAX_NUMBER_OF_AB + 1]={0};





//static uint8_t S03_RQ = 0;

/**
@var Status wirelessDetected
@brief Is wireless device detected on the UART
*/
ErrorStatus wirelessDetected=(ERROR);

//For message span time
uint64_t t_span;

uint64_t t_signal_span;





//static uint8_t rbi_IsBroadcastMessageTxPending;
//static uint8_t rbi_IsUnicastMessageTxPending;
static char rasb_TxMessageBuffer[256u];
//static char rasb_TxMessageBuffer[128u];
static uint32_t rul_TxMessageSize;
static int8_t rsw_LastMessageRSSI;
static uint8_t ruw_LastMessageLQI;

uint8_t wirelessCustom = 0;
uint8_t signal_flag = 0;
//static uint8_t rbi_IsPreviousMessageATS06 = 0u;
//static uint8_t rbi_IsPreviousMessageATS07 = 0u;


//static void HardResetWireless(void);
static uint8_t SendWirelessMessageBroadcast(const uint8_t *message, uint32_t messageLength);
static uint8_t SendWirelessMessageToAddress(const uint8_t *message, uint32_t messageLength, uint64_t address);
//static void SendSignalPercentageRequest(void);
//static void SendSignalValueRequest(void);
//static void SendReadOutputPowerRequest(void);
//static void SendTemperatureRequest(void);
static void Sending(void);
static int32_t sendStringMessageToWireless(const char *message, uint32_t messageSize);
static int32_t calculateULongLongIntValue(const char *inputValue, uint32_t valueLength, uint64_t *outputValue);
//static int32_t calculateOutputPower(uint32_t inputValue);
static int32_t calculateUintValue(const char *inputValue, uint32_t valueLength, uint32_t *outputValue);
static int32_t calculateSintValue(const char *inputValue, uint32_t valueLength, int32_t *outputValue);
static int32_t sendMessageToWireless(WirelessMessageCode msgCode, uint64_t deviceAddress, const uint8_t *message,
                                     uint32_t messageSize, WirelessAddressType msgAddrType);
//static int32_t initTemperatureSensor(void);
//static void calculateTemperature(const char *inputAdcValue, uint32_t valueLength);
static uint32_t decodeDeviceAddress(const char *message, uint32_t messageLength, uint64_t *decodedAddress);
static void setCoordinatorEUI64Address(const char *message, uint32_t messageLength);
//static void setCoordinatorNetworkAddress(const char *message, uint32_t messageLength);

//static void calculateSignalValue(void);
static void calculateSignalPercentage(void);
//static void calculateOutputPowerReadout(const char *message, uint32_t messageLength);
//static int32_t startWireless(void);
static uint32_t calculateXorChecksum(uint8_t *message, uint32_t msgLength);
static void decodeDataMessage(char *message[], uint32_t parameterCount);
static uint32_t processMsg(const uint8_t *incomingMsg, uint32_t msgLength, char *result[]);
static void parseMessage(char *message[], uint32_t parameterCount);
//void getWirMessages(void);
//static void readWirelessDeviceValues(void);

//static uint8_t detectWirelessModule(void);

//static uint64_t Wir_Det_period =2000u;



void configWireless(void);

/********************************************************************
jcm-Complete
@fn	static int32_t calculateOutputPower(uint32_t inputValue)
@param[in] inputValue Otuput power value in range of 0 to 5
@return Calculated value in dBm
@brief Take the input value in range of 0 to 5 (6dBm steps) and calculate value in dBm from it.
@details Calculated value range will be from -12dBm (for input of 0) to 18dBm (for input of 5)
*********************************************************************/
/*
static int32_t calculateOutputPower(uint32_t inputValue){
int32_t buf=(int32_t)inputValue;
buf*=6;
buf-=12;
return buf;
}
*/
/********************************************************************
//jcm-Complete
@fn	static int32_t calculateUintValue(const char *inputValue, uint32_t valueLength, uint32_t *outputValue)
@param[in] inputValue Input string
@param[in] valueLength Length of the string
@param[out] outputValue Pointer to the 32-bit unsigned integer that will hold the result
@return Success(1)/Error(-1)
@brief Calculate unsigned integer value from the value in input string
*********************************************************************/
static int32_t calculateUintValue(const char *inputValue, uint32_t valueLength, uint32_t *outputValue){
      
      if(sscanf(inputValue, "%X", outputValue)<=0){
            return -1;
      }
      
      return 1;
}

/********************************************************************
//jcm-Complete
@fn	static int32_t calculateSintValue(const char *inputValue, uint32_t valueLength, int32_t *outputValue)
@param[in] inputValue Input string
@param[in] valueLength Length of the string
@param[out] outputValue Pointer to the 32-bit signed integer that will hold the result
@return Success(1)/Error(-1)
@brief Calculate unsigned integer value from the value in input string
*********************************************************************/
static int32_t calculateSintValue(const char *inputValue, uint32_t valueLength, int32_t *outputValue){
      
      if(sscanf(inputValue, "%d", outputValue)<=0){
            return -1;
      }
      
      return 1;
}

/********************************************************************
//jcm-Complete
static int32_t calculateULongLongIntValue(const char *inputValue, uint32_t valueLength, uint64_t *outputValue)
@param[in] inputValue Pointer to the input string
@param[in] valueLength Length of the string
@param[out] outputValue Pointer to the 64-bit unsigned integer that will hold the result
@return Success(1)/Error(-1)
@brief Calculate unsigned integer value from the value in input string
*********************************************************************/
static int32_t calculateULongLongIntValue(const char *inputValue, uint32_t valueLength, uint64_t *outputValue){
      if(sscanf(inputValue, "%016llX", outputValue)<=0) return -1;
      return 1;
}


/********************************************************************
//jcm-Complete
@fn	static int32_t sendStringMessageToWireless(const char *message, uint32_t messageSize)
@param[in] message Pointer to the output message in string format
@param[in] messageSize Length of the message
@return Success(1)/Fail(-1)/Data not sent(0)
@brief Used from the other "send" functions.
@details Messages to the wireless device are sent in the ASCII format
*********************************************************************/
static int32_t sendStringMessageToWireless(const char *message, uint32_t messageSize){
      static char out[MAX_WIRELESS_MESSAGE_SIZE+1];
      memset(out, 0x00, sizeof(out));
      
      if(messageSize<=(MAX_WIRELESS_MESSAGE_SIZE-sizeof(textEnd))){
              
           t_span_wir = GetCurrentSystemTime();

              
            /* copy to buffer */
            strcpy(out, message);
            /* append line end */
            strcat(out,textEnd);		
            // wirelessState=WIRELESS_IDLE;
            

          
          return HAL_UART_Transmit_IT(&huart1,(uint8_t*)out, (uint32_t)strlen(out));
              
          //TO_ADD - WIRELESS SEND
            
      }
      
      return -1;
}


/********************************************************************
@fn	static int32_t sendMessageToWireless(WirelessMessageCode msgCode, uint64_t deviceAddress, const uint8_t *message,
uint32_t messageSize, WirelessAddressType msgAddrType)
@param[in] msgCode Type of the message that is to be sent
@param[in] deviceAddress MAC address of the wireless device to which the message is to be sent
@param[in] message Pointer to the message buffer
@param[in] messageSize Size of the message
@param[in] msgAddrType Message sent as broadcast or to some specific MAC address
@return Succes(1)/Error(any other value)
@brief Send the message to the wireless device
*********************************************************************/
static int32_t sendMessageToWireless(WirelessMessageCode msgCode, uint64_t deviceAddress, const uint8_t *message,
                                     uint32_t messageSize, WirelessAddressType msgAddrType){

                                           if(wireless_disable == 1) return 0;

                                           char address[64]={0};
                                           /* tempBuf must hold hex-encoded message: (messageSize+1)*2 + null.
                                            * Max messageSize after bounds check below is ~100, so 256 is safe. */
                                           char tempBuf[MAX_WIRELESS_MESSAGE_SIZE]={0};
                                           static char outputM[MAX_WIRELESS_MESSAGE_SIZE]={0};

                                           uint8_t value=0;
                                           uint32_t countS=0;

                                           memset(address, 0x00, sizeof(address));
                                           memset(tempBuf, 0x00, sizeof(tempBuf));
                                           memset(rasb_TxMessageBuffer, 0x00, sizeof(rasb_TxMessageBuffer));

                                           /* Bounds check: hex encoding doubles size, plus header (~30 chars max).
                                            * Ensure (messageSize+1)*2 + 30 < MAX_WIRELESS_MESSAGE_SIZE */
                                           if(messageSize >= ((MAX_WIRELESS_MESSAGE_SIZE - 30) / 2)) {
                                                 return -2;
                                           }

                                           if(msgCode == WIRELESS_SEND_DATA){

                                                 if(msgAddrType == WIRELESS_MESSAGE_ADDRESSED)
                                                  {
                                                        memset(outputM, 0x00, sizeof(outputM));
                                                        /* Add unicast binary data message header. */
                                                        strncat(outputM, "AT+UCAST:", sizeof(outputM) - strlen(outputM) - 1);
                                                        memset(tempBuf, 0x00, sizeof(tempBuf));
                                                        /* Add the message receiver EUI64. */
                                                        snprintf(tempBuf, sizeof(tempBuf), "%016llX=", deviceAddress);
                                                        strncat(outputM, tempBuf, sizeof(outputM) - strlen(outputM) - 1);
                                                        memset(tempBuf, 0x00, sizeof(tempBuf));

                                                        /* Copy the input message to the transmit buffer. */
                                                        memcpy(&rasb_TxMessageBuffer[0u], message, messageSize);
                                                        /* Add a leading 0 - hex value 0x30 */
                                                        rasb_TxMessageBuffer[messageSize] = '0';
                                                        rul_TxMessageSize = messageSize + 1;
                                                        rul_TxMessageSize *= 2;

                                                        for (countS = 0u; countS < rul_TxMessageSize && countS < (sizeof(tempBuf) - 2); countS += 2)
                                                         {
                                                               /* Iterate through every byte in the buffer and convert it to hex string.
                                                               * Ex: 2002 decimal = 07D2 char */
                                                               value = rasb_TxMessageBuffer[countS / 2];
                                                               snprintf(&tempBuf[countS], sizeof(tempBuf) - countS, "%02X", value);
                                                         }

                                                        strncat(outputM, tempBuf, sizeof(outputM) - strlen(outputM) - 1);
                                                        //rbi_IsUnicastMessageTxPending = 1u;
                                                        //rbi_IsBroadcastMessageTxPending = 0u;
                                                  }
                                                 else
                                                  {
                                                        memset(outputM, 0x00, sizeof(outputM));
                                                        /* Add multicast data message header. */
                                                        strncat(outputM, "AT+BCAST:00,", sizeof(outputM) - strlen(outputM) - 1);
                                                        memset(tempBuf, 0x00, sizeof(tempBuf));
                                                        /* Copy the input message to the transmit buffer. */
                                                        memcpy(&rasb_TxMessageBuffer[0u], message, messageSize);
                                                        /* Add a leading 0 - hex value 0x30 */
                                                        rasb_TxMessageBuffer[messageSize] = '0';
                                                        rul_TxMessageSize = messageSize + 1;
                                                        rul_TxMessageSize *= 2;

                                                        for (countS = 0u; countS < rul_TxMessageSize && countS < (sizeof(tempBuf) - 2); countS += 2)
                                                         {
                                                               /* Iterate through every byte in the buffer and convert it to hex string.
                                                               * Ex: 2002 decimal = 07D2 char */
                                                               value = rasb_TxMessageBuffer[countS / 2];
                                                               snprintf(&tempBuf[countS], sizeof(tempBuf) - countS, "%02X", value);
                                                         }

                                                        strncat(outputM, tempBuf, sizeof(outputM) - strlen(outputM) - 1);

                                                  }
                                                 
                                                 
                                           }
                                           else
                                            {
                                                  /* unknown message type */
                                                  return -3;
                                            }
                                           
                                           sendStringMessageToWireless(outputM, (uint32_t)strlen(outputM));
                                           
                                           return 1;
                                     }






/********************************************************************
@fn	static uint32_t decodeDeviceAddress(const char *message, uint32_t messageLength, uint64_t *decodedAddress)
@param[in] message Pointer to the received message
@param[in] messageLength Size of the message
@param[out] decodedAddress Pointer to the 64-bit value that will hold the result
@return Success(1)/Error(0)
@brief Decode sender MAC address from the "DAT" message
*********************************************************************/
static uint32_t decodeDeviceAddress(const char *message, uint32_t messageLength, uint64_t *decodedAddress){
      uint64_t address=0ul;
      
      if(calculateULongLongIntValue(message, messageLength, &address)<0){
            //TODO log error
            return 0;
      }
      *decodedAddress=address;
      
      return 1;
}


/********************************************************************
@fn	static void setCoordinatorEUI64Address(const char *message, uint32_t messageLength)
@param[in] message Pointer to the incoming message
@param[in] messageLength Size of the message
@return Nothing
@brief Set the detected address of the wireless coordinator
@details Used in the slave or unknown device operation mode
*********************************************************************/
static void setCoordinatorEUI64Address(const char *message, uint32_t messageLength){
      uint64_t address=0ul;
      if(decodeDeviceAddress(message, messageLength, &address)<=0){
            //TODO log error?!
            return;
      }
      //Netw 01.07.2017
      coordinatorAddressX=address;
      //coordinatorAddress=address;
}



/********************************************************************
@fn	static void calculateSignalPercentage(void)
@return Nothing
@brief Calculate wireless receiver detected signal percentage and write
it to the currentWirelessReadings structure
*********************************************************************/
static void calculateSignalPercentage(void){
      int16_t rssi = rsw_LastMessageRSSI;
      
      /* Translate the value to the 0 - 255 range.
      * Apply the percentage factor (100) before the division so that the
      * final value will not be in the range [0-1], which can't be represented
      * as integer.
      * Apply division by the highest factor (256) to obtain the percentage. 
      */
      
      rssi += 100;
      rssi *= 100;
      rssi /= 45;
      if(rssi > 99) rssi = 99;
      
      get_device_datas()->wireless_power_status = rssi;

}


/********************************************************************
@fn	static void calculateOutputPowerReadout(const char *message, uint32_t messageLength)
@param[in] message Pointer to the incoming message
@param[in] messageLength Size of the message
@return Nothing
@brief Calculate wireless transmitter output power (in dBm)
*********************************************************************/
/*
static void calculateOutputPowerReadout(const char *message, uint32_t messageLength){
uint32_t value=0;
if(calculateUintValue(message, messageLength, &value)<0){
get_device_datas()->wireless_output_power = (uint8_t)INT32_MIN;
//TODO log error
return;
	}
get_device_datas()->wireless_output_power = (uint8_t)value;
}

*/
/********************************************************************
@fn	static int32_t startWireless()
@param[in] deviceType Device is wireless router or coordinator
@return Success(1)/Error(-1)
@brief Send initialisation messages to the wireless device
*********************************************************************/



void startWireless(void){
      
        
        //  const char *cfgDetection = "ATI"; 
//        const char *cfgDisconnect = "AT+DASSL";
	const char *cfgRouter="AT+JN";
        const char *cfgMaster="AT+EN";
//	const char *cfgReset="ATZ";
        const char *cfgInfo="AT+N";
        
      

        sendStringMessageToWireless(cfgInfo, (uint32_t)strlen(cfgInfo));
        t_span = GetCurrentSystemTime();
        
        
        while ((GetCurrentSystemTime() - t_span) < 1000u)
        {
                getWirMessages();             
        }
        
/*

        sendStringMessageToWireless(cfgDisconnect, (uint32_t)strlen(cfgDisconnect));
        t_span = GetCurrentSystemTime();
        
        
        while ((GetCurrentSystemTime() - t_span) < 1000u)
        {
                getWirMessages();             
        }
*/

        sendStringMessageToWireless(cfgRouter, (uint32_t)strlen(cfgRouter));
        t_span = GetCurrentSystemTime();
        
        
        while ((GetCurrentSystemTime() - t_span) < 1000u)
        {
                getWirMessages();             
        }

        if(NO_network == 1)
        {
                sendStringMessageToWireless(cfgMaster, (uint32_t)strlen(cfgMaster));
                t_span = GetCurrentSystemTime();
                
                
                while ((GetCurrentSystemTime() - t_span) < 1000u)
                {
                        getWirMessages();             
                }
        }

}




/********************************************************************
@fn	static uint32_t calculateXorChecksum(uint8_t *message, uint32_t msgLength)
@param[in] message Pointer to the message
@param[in] msgLength Message size
@return Result of the calculation
@brief Calculate simple XOR checksum for a given message
*********************************************************************/

static uint32_t calculateXorChecksum(uint8_t *message, uint32_t msgLength){
        uint32_t xor=0u;
        uint32_t n=0;
        for(n=0; n<msgLength;n++){
                xor^=message[n];
        }
        return xor;
}




static void decodeDataMessage(char *message[], uint32_t parameterCount){
        uint64_t senderWirelessAddress=0ul;
        //uint32_t offset=0;
        char buf[4]={0};
        static uint32_t data[40]={0};
        static uint8_t byteData[40]={0};
        uint32_t dataLength=0;
        uint8_t checksum=0;
        static uint8_t isCoordAdrSet=0;
        
        uint16_t deviceAddress=0;
        
        memset(data,0x00, sizeof(data));
        memset(byteData,0x00, sizeof(byteData));
        
        if (!((parameterCount <= MAX_NUMBER_OF_MESSAGE_PARAMETERS)
              && (parameterCount >= MIN_NUMBER_OF_MESSAGE_PARAMETERS)))
        {
                return;
        }
        
        if(decodeDeviceAddress(message[1], (uint32_t)strlen(message[1]), &senderWirelessAddress)<=0){
                return;
        }
        
        
        if(!get_device_datas()->isMaster & (isCoordAdrSet == 0)){
                coordinatorAddress = senderWirelessAddress;
                isCoordAdrSet = 1;
        }
        
        if (!(device_Status == Status_Init || device_Status == Status_Idle))
        {
                
                // modificat 05.12.2015
                if(!get_device_datas()->isMaster && isCoordAdrSet == 1){
                        /*
                        if(senderWirelessAddress!=coordinatorAddress && coordinatorAddress != 0){
                        if(senderWirelessAddress != AB_Address && AB_Address != 0)
                        {
                        return;
                }
                }
                        */
                }
                
                // end modificat 15.12.2015
                
        }
        
        if(calculateUintValue(message[2], (uint32_t)strlen(message[2]), &dataLength) <= 0){
                return;
        }
        
        /* Decrement dataLength by 2 units due to incorrect UART buffering */
        dataLength-=2;
        
        if(dataLength<=1){
                return;
        }
        
        for(uint32_t n=0; n < dataLength; n += 2){
                memset(buf, 0x00, sizeof(buf));
                strncpy(buf, &message[3][n], 2);
                if (calculateUintValue(buf, 2, &data[n / 2]) <= 0)
                {
                        return;
                }
        }
        
        dataLength = dataLength / 2;
        
        for(uint32_t n=dataLength; n>1;n--){
                if(data[n-1]>=0x100){
                        return;
                }
        }
        
        
        for(uint32_t n=0; n<=data[0];n++){
                byteData[n]=(uint8_t)data[n];
        }
        
        /* Check if the parameters are present. */
        if (parameterCount == MAX_NUMBER_OF_MESSAGE_PARAMETERS)
        {
                /* Store last received message RSSI (max_number_of_message_parameters - 2). */
                memset(buf, 0x00, sizeof(buf));
                strncpy(buf, message[MAX_NUMBER_OF_MESSAGE_PARAMETERS - 2], sizeof(buf));
                (void)calculateSintValue(buf, 3, (int32_t *)&rsw_LastMessageRSSI);            
                
                /* Store last received message LQI (max_number_of_message_parameters - 1). */
                memset(buf, 0x00, sizeof(buf));
                strncpy(buf, message[MAX_NUMBER_OF_MESSAGE_PARAMETERS - 1], sizeof(buf));
                (void)calculateUintValue(buf, 3, (uint32_t *)&ruw_LastMessageLQI);
                
                if(signal_flag == 1)
                {
                        calculateSignalPercentage();
                }
                else
                {
                        get_device_datas()->wireless_power_status = 100;                
                }
                
        }
        
        checksum=calculateXorChecksum(byteData, byteData[0]);
        
        if((uint8_t)checksum!=byteData[byteData[0]]){
                return;
        }
        
        get_device_datas()->isWirelessConnected = 1;
        deviceAddress = byteData[1];
        deviceAddress <<= 8;
        deviceAddress |= byteData[2];
        deviceAddress &= 0x07FF;
        
        
        
        
        // message filtering

        
        if(deviceAddress % MAX_NUMBER_OF_DEVICES == AUDIO_BOX_ADR_SET){
                        if(byteData[11] > 200 && byteData[11] < 210){
                                wirelessDeviceTableAB[byteData[11] - AUDIO_BOX_OFFSET] = senderWirelessAddress;
                        }
                }
         
        
        if (deviceAddress < MASTER_SLAVE_Error && ID_duplicated[deviceAddress % MAX_NUMBER_OF_DEVICES] == 0 && get_device_datas()->isMaster == 1 
            && wirelessDeviceTable[deviceAddress % MAX_NUMBER_OF_DEVICES] != senderWirelessAddress 
            && senderWirelessAddress > 0 && wirelessDeviceTable[deviceAddress % MAX_NUMBER_OF_DEVICES] != 0 && senderWirelessAddress < 99) {
                    
                if(deviceAddress % MAX_NUMBER_OF_DEVICES < 99){        
                        ID_duplicated[deviceAddress % MAX_NUMBER_OF_DEVICES] = 1;
                }
        }
        else if (deviceAddress < MASTER_SLAVE_Error && ID_duplicated[deviceAddress % MAX_NUMBER_OF_DEVICES] != 0 && get_device_datas()->isMaster == 1 && wirelessDeviceTable[deviceAddress % MAX_NUMBER_OF_DEVICES] != senderWirelessAddress 
            && senderWirelessAddress > 0 && wirelessDeviceTable[deviceAddress % MAX_NUMBER_OF_DEVICES] != 0) {
                
                ID_duplicatedX = deviceAddress % MAX_NUMBER_OF_DEVICES;
        }
        else
        {
                if (get_device_datas()->isMaster) {

                        if ((senderWirelessAddress == wirelessDeviceTable[99])&& (deviceAddress == MASTER_SLAVE_Start || deviceAddress == MASTER_SLAVE_Sync || deviceAddress == MASTER_SLAVE_Pause))
                        {
                                //do nothing
                        }
                        else
                        {
                                if (deviceAddress != MASTER_SLAVE_PRG_CONF && deviceAddress != MASTER_SLAVE_MIR_MSG && deviceAddress != MASTER_SLAVE_MD5)
                                {
                                        if (deviceAddress >= MASTER_SLAVE_Error)	return;
                                }
                                if (wirelessDeviceTable[deviceAddress % MAX_NUMBER_OF_DEVICES] != senderWirelessAddress && senderWirelessAddress > 0 && deviceAddress < MASTER_SLAVE_Error) {
                                        
                                        wirelessDeviceTable[deviceAddress % MAX_NUMBER_OF_DEVICES]= senderWirelessAddress;
                                }
                                
                                if(deviceAddress < MASTER_SLAVE_Error){
                                        getCurrentSlaveInfo(deviceAddress % MAX_NUMBER_OF_DEVICES)->isWirelessConnected = 1;
                                }
                                
                        }
                }
                else {
                        if (deviceAddress < MASTER_SLAVE_Error
                            && (deviceAddress % MAX_NUMBER_OF_DEVICES) != get_device_datas()->device_Address)
                                return;
                }
                
                dev_FIFO *fifo = getDevFIFO();
                MS_Message *tmpMSG = &(fifo->buffer[fifo->wrPtr]);
                uint64_t t = GetCurrentSystemTime();
                int flag = 0;
                if (MASTER_SLAVE_PingA <= deviceAddress && deviceAddress < MASTER_SLAVE_PingB) {//for pingA
                        tmpMSG->dataLength = 5;
                        tmpMSG->address = (deviceAddress % MAX_NUMBER_OF_DEVICES) + MASTER_SLAVE_InterfaceStatus;
                        for (int i = 0; i < tmpMSG->dataLength; i++)
                                tmpMSG->data[i] = byteData[i + 3];
                        
                        int32_t las = fifo->wrPtr + 1;
                        if (las >= MAX_DEV_BUFFER) las -= MAX_DEV_BUFFER;
                        
                        while (las != fifo->wrPtr) {
                                if (fifo->revChannel[las] == Channel_CAN
                                    && t - fifo->revTime[las] < SPAN_MESSAGES
                                            && strncmp((char*)tmpMSG, (char*)&(fifo->buffer[las]), tmpMSG->dataLength + 3) == 0) {
                                                    fifo->revChannel[las] = Channel_Both;
                                                    flag = 1;
                                                    break;
                                            }
                                las++;
                                if (las == MAX_DEV_BUFFER) las = 0;
                        }
                        if (!flag) {
                                fifo->revChannel[fifo->wrPtr] = Channel_Wireless;
                                fifo->revTime[fifo->wrPtr] = t;
                                fifo->wrPtr++;
                                if (fifo->wrPtr == MAX_DEV_BUFFER) {
                                        fifo->wrPtr = 0;
                                }
                                if (fifo->rdPtr == fifo->wrPtr) {
                                        fifo->rdPtr++;
                                        if (fifo->rdPtr == MAX_DEV_BUFFER) fifo->rdPtr = 0;
                                        //TODO-jcm *********** buffer over
                                        //return ;
                                }
                        }
                        
                        flag = 0;
                        tmpMSG = &(fifo->buffer[fifo->wrPtr]);
                        tmpMSG->dataLength = 8;
                        tmpMSG->address = (deviceAddress % MAX_NUMBER_OF_DEVICES) + MASTER_SLAVE_MatrixA1Status;
                        for (int i = 0; i < tmpMSG->dataLength; i++)
                                tmpMSG->data[i] = byteData[i + 3 + 8];
                        
                        las = fifo->wrPtr + 1;
                        if (las >= MAX_DEV_BUFFER) las -= MAX_DEV_BUFFER;
                        
                        while (las != fifo->wrPtr) {
                                if (fifo->revChannel[las] == Channel_CAN
                                    && t - fifo->revTime[las] < SPAN_MESSAGES
                                            && strncmp((char*)tmpMSG, (char*)&(fifo->buffer[las]), tmpMSG->dataLength + 3) == 0) {
                                                    fifo->revChannel[las] = Channel_Both;
                                                    flag = 1;
                                                    break;
                                            }
                                las++;
                                if (las == MAX_DEV_BUFFER) las = 0;
                        }
                        if (!flag) {
                                fifo->revChannel[fifo->wrPtr] = Channel_Wireless;
                                fifo->revTime[fifo->wrPtr] = t;
                                fifo->wrPtr++;
                                if (fifo->wrPtr == MAX_DEV_BUFFER) {
                                        fifo->wrPtr = 0;
                                }
                                if (fifo->rdPtr == fifo->wrPtr) {
                                        fifo->rdPtr++;
                                        if (fifo->rdPtr == MAX_DEV_BUFFER) fifo->rdPtr = 0;
                                        //TODO-jcm *********** buffer over
                                        //break;
                                }
                        }
                        
                        flag = 0;
                        tmpMSG = &(fifo->buffer[fifo->wrPtr]);
                        tmpMSG->dataLength = 8;
                        tmpMSG->address = (deviceAddress % MAX_NUMBER_OF_DEVICES) + MASTER_SLAVE_MatrixA2Status;
                        for (int i = 0; i < tmpMSG->dataLength; i++)
                                tmpMSG->data[i] = byteData[i + 3 + 16];
                        
                        las = fifo->wrPtr + 1;
                        if (las >= MAX_DEV_BUFFER) las -= MAX_DEV_BUFFER;
                        
                        while (las != fifo->wrPtr) {
                                if (fifo->revChannel[las] == Channel_CAN
                                    && t - fifo->revTime[las] < SPAN_MESSAGES
                                            && strncmp((char*)tmpMSG, (char*)&(fifo->buffer[las]), tmpMSG->dataLength + 3) == 0) {
                                                    fifo->revChannel[las] = Channel_Both;
                                                    flag = 1;
                                                    break;
                                            }
                                las++;
                                if (las == MAX_DEV_BUFFER) las = 0;
                        }
                        if (!flag) {
                                fifo->revChannel[fifo->wrPtr] = Channel_Wireless;
                                fifo->revTime[fifo->wrPtr] = t;
                                fifo->wrPtr++;
                                if (fifo->wrPtr == MAX_DEV_BUFFER) {
                                        fifo->wrPtr = 0;
                                }
                                if (fifo->rdPtr == fifo->wrPtr) {
                                        fifo->rdPtr++;
                                        if (fifo->rdPtr == MAX_DEV_BUFFER) fifo->rdPtr = 0;
                                        //TODO-jcm *********** buffer over
                                        //break ;
                                }
                        }
                        
                        flag = 0;
                        tmpMSG = &(fifo->buffer[fifo->wrPtr]);
                        tmpMSG->dataLength = 8;
                        tmpMSG->address = (deviceAddress % MAX_NUMBER_OF_DEVICES) + MASTER_SLAVE_MatrixA3Status;
                        for (int i = 0; i < tmpMSG->dataLength; i++)
                                tmpMSG->data[i] = byteData[i + 3 + 24];
                        
                        las = fifo->wrPtr + 1;
                        if (las >= MAX_DEV_BUFFER) las -= MAX_DEV_BUFFER;
                        
                        while (las != fifo->wrPtr) {
                                if (fifo->revChannel[las] == Channel_CAN
                                    && t - fifo->revTime[las] < SPAN_MESSAGES
                                            && strncmp((char*)tmpMSG, (char*)&(fifo->buffer[las]), tmpMSG->dataLength + 3) == 0) {
                                                    fifo->revChannel[las] = Channel_Both;
                                                    flag = 1;
                                                    break;
                                            }
                                las++;
                                if (las == MAX_DEV_BUFFER) las = 0;
                        }
                        if (!flag) {
                                fifo->revChannel[fifo->wrPtr] = Channel_Wireless;
                                fifo->revTime[fifo->wrPtr] = t;
                                fifo->wrPtr++;
                                if (fifo->wrPtr == MAX_DEV_BUFFER) {
                                        fifo->wrPtr = 0;
                                }
                                if (fifo->rdPtr == fifo->wrPtr) {
                                        fifo->rdPtr++;
                                        if (fifo->rdPtr == MAX_DEV_BUFFER) fifo->rdPtr = 0;
                                        //TODO-jcm *********** buffer over
                                        //break ;
                                }
                        }
                        
                }
                else if (MASTER_SLAVE_PingB <= deviceAddress && deviceAddress < MASTER_SLAVE_PingB + MAX_NUMBER_OF_DEVICES){//for pingB
                        tmpMSG->dataLength = 5;
                        tmpMSG->address = (deviceAddress % MAX_NUMBER_OF_DEVICES) + MASTER_SLAVE_InterfaceStatus;
                        for (int i = 0; i < tmpMSG->dataLength; i++)
                                tmpMSG->data[i] = byteData[i + 3];
                        
                        int32_t las = fifo->wrPtr + 1;
                        if (las >= MAX_DEV_BUFFER) las -= MAX_DEV_BUFFER;
                        
                        while (las != fifo->wrPtr) {
                                if (fifo->revChannel[las] == Channel_CAN
                                    && t - fifo->revTime[las] < SPAN_MESSAGES
                                            && strncmp((char*)tmpMSG, (char*)&(fifo->buffer[las]), tmpMSG->dataLength + 3) == 0) {
                                                    fifo->revChannel[las] = Channel_Both;
                                                    flag = 1;
                                                    break;
                                            }
                                las++;
                                if (las == MAX_DEV_BUFFER) las = 0;
                        }
                        if (!flag) {
                                fifo->revChannel[fifo->wrPtr] = Channel_Wireless;
                                fifo->revTime[fifo->wrPtr] = t;
                                fifo->wrPtr++;
                                if (fifo->wrPtr == MAX_DEV_BUFFER) {
                                        fifo->wrPtr = 0;
                                }
                                if (fifo->rdPtr == fifo->wrPtr) {
                                        fifo->rdPtr++;
                                        if (fifo->rdPtr == MAX_DEV_BUFFER) fifo->rdPtr = 0;
                                        //TODO-jcm *********** buffer over
                                        //break;
                                }
                        }
                        
                        flag = 0;
                        tmpMSG = &(fifo->buffer[fifo->wrPtr]);
                        tmpMSG->dataLength = 8;
                        tmpMSG->address = (deviceAddress % MAX_NUMBER_OF_DEVICES) + MASTER_SLAVE_MatrixB1Status;
                        for (int i = 0; i < tmpMSG->dataLength; i++)
                                tmpMSG->data[i] = byteData[i + 3 + 8];
                        
                        las = fifo->wrPtr + 1;
                        if (las >= MAX_DEV_BUFFER) las -= MAX_DEV_BUFFER;
                        
                        while (las != fifo->wrPtr) {
                                if (fifo->revChannel[las] == Channel_CAN
                                    && t - fifo->revTime[las] < SPAN_MESSAGES
                                            && strncmp((char*)tmpMSG, (char*)&(fifo->buffer[las]), tmpMSG->dataLength + 3) == 0) {
                                                    fifo->revChannel[las] = Channel_Both;
                                                    flag = 1;
                                                    break;
                                            }
                                las++;
                                if (las == MAX_DEV_BUFFER) las = 0;
                        }
                        if (!flag) {
                                fifo->revChannel[fifo->wrPtr] = Channel_Wireless;
                                fifo->revTime[fifo->wrPtr] = t;
                                fifo->wrPtr++;
                                if (fifo->wrPtr == MAX_DEV_BUFFER) {
                                        fifo->wrPtr = 0;
                                }
                                if (fifo->rdPtr == fifo->wrPtr) {
                                        fifo->rdPtr++;
                                        if (fifo->rdPtr == MAX_DEV_BUFFER) fifo->rdPtr = 0;
                                        //TODO-jcm *********** buffer over
                                        //break;
                                }
                        }
                        flag = 0;
                        tmpMSG = &(fifo->buffer[fifo->wrPtr]);
                        tmpMSG->dataLength = 4;
                        tmpMSG->address = (deviceAddress % MAX_NUMBER_OF_DEVICES) + MASTER_SLAVE_MatrixB2Status;
                        for (int i = 0; i < tmpMSG->dataLength; i++)
                                tmpMSG->data[i] = byteData[i + 3 + 16];
                        
                        las = fifo->wrPtr + 1;
                        if (las >= MAX_DEV_BUFFER) las -= MAX_DEV_BUFFER;
                        
                        while (las != fifo->wrPtr) {
                                if (fifo->revChannel[las] == Channel_CAN
                                    && t - fifo->revTime[las] < SPAN_MESSAGES
                                            && strncmp((char*)tmpMSG, (char*)&(fifo->buffer[las]), tmpMSG->dataLength + 3) == 0) {
                                                    fifo->revChannel[las] = Channel_Both;
                                                    flag = 1;
                                                    return;
                                            }
                                las++;
                                if (las == MAX_DEV_BUFFER) las = 0;
                        }
                        if (!flag) {
                                fifo->revChannel[fifo->wrPtr] = Channel_Wireless;
                                fifo->revTime[fifo->wrPtr] = t;
                                fifo->wrPtr++;
                                if (fifo->wrPtr == MAX_DEV_BUFFER) {
                                        fifo->wrPtr = 0;
                                }
                                if (fifo->rdPtr == fifo->wrPtr) {
                                        fifo->rdPtr++;
                                        if (fifo->rdPtr == MAX_DEV_BUFFER) fifo->rdPtr = 0;
                                        //TODO-jcm *********** buffer over
                                        //break;
                                }
                        }
                        
                }
                else {
                        tmpMSG->dataLength = byteData[0] - 3;
                        tmpMSG->address = deviceAddress;
                        for (int i = 0; i < tmpMSG->dataLength; i++)
                                tmpMSG->data[i] = byteData[i + 3];
                        
                        uint64_t t = GetCurrentSystemTime();
                        int32_t las = fifo->wrPtr + 1;
                        if (las >= MAX_DEV_BUFFER) las -= MAX_DEV_BUFFER;
                        
                        while (las != fifo->wrPtr) {
                                if (fifo->revChannel[las] == Channel_CAN
                                    && t - fifo->revTime[las] < SPAN_MESSAGES
                                            && strncmp((char*)tmpMSG, (char*)&(fifo->buffer[las]), tmpMSG->dataLength + 3) == 0) {
                                                    fifo->revChannel[las] = Channel_Both;
                                                    return;
                                            }
                                las++;
                                if (las == MAX_DEV_BUFFER) las = 0;
                        }
                        fifo->revChannel[fifo->wrPtr] = Channel_Wireless;
                        fifo->revTime[fifo->wrPtr] = t;
                        fifo->wrPtr++;
                        if (fifo->wrPtr == MAX_DEV_BUFFER) {
                                fifo->wrPtr = 0;
                        }
                        if (fifo->rdPtr == fifo->wrPtr) {
                                fifo->rdPtr++;
                                if (fifo->rdPtr == MAX_DEV_BUFFER) fifo->rdPtr = 0;
                                //TODO-jcm *********** buffer over
                                //break;
                        }
                }
        }
}

 

/********************************************************************
@fn	static uint32_t processMsg(const uint8_t *incomingMsg, uint32_t msgLength, char *result[])
@param[in] incomingMsg Pointer to the incoming message
@param[in] msgLength Length of the incoming message
@param[out] result Result (multi-string) of the operation
@return Number of rows (strings detected in the incoming message
@brief Split incoming message into rows
*********************************************************************/
static uint32_t processMsg(const uint8_t *incomingMsg, uint32_t msgLength, char *result[]){
      static char bufMsg[MAX_WIRELESS_MESSAGE_SIZE]={0};	
      uint32_t counter=0;
      uint32_t length=0;
      char *tempVal;
      
      memset(bufMsg, 0x00, sizeof(bufMsg));
      
      memcpy(bufMsg, incomingMsg, msgLength);
      if(bufMsg[msgLength-1]==(uint8_t)0x0A && bufMsg[msgLength-2]==(uint8_t)0x0D){
            bufMsg[msgLength-2]='\0';
      }else{
            return 0;
      }
      
      tempVal=strtok(&bufMsg[0u], delimiters);
      while(tempVal != NULL  && counter<MAX_NUMBER_OF_WIRELESS_PARAMETERS){							
            length=(uint32_t)strlen(tempVal);
            memcpy(*result, tempVal, length+1);				
            result++;
            counter++;
            tempVal=strtok(NULL, delimiters);
      };
      
      return counter;
}


/********************************************************************
@fn	static void parseMessage(char *message[], uint32_t parameterCount)
@param[in] message[] Pointer to the processed (multi-string) message
@param[in] parameterCount Number of rows (strings) in the message
@return Nothing
@brief Parse incoming message
*********************************************************************/
static void parseMessage(char *message[], uint32_t parameterCount){	

        
        if(is_Wir_Slot_Detect == 0)
        {
                is_Wir_Slot_Detect = 1;
        }
        
       if (strcmp(*message, "OK") == 0)
        {
                is_wireless_ok = 1;

                if(is_wireless_respopnd == 0){
                        is_wireless_respopnd = 1;
                }
                
                if (wirelessState == WIRELESS_BUSY)
                {
                        wirelessState = (WIRELESS_IDLE);
                }
                
                if (wirelessBuffer.count == 0)
                {
                        return;
                }
                
                wirelessBuffer.count--;
                wirelessBuffer.rdIdx++;
                if (wirelessBuffer.rdIdx == WIRELESS_TX_BUF_SIZE)
                {
                        wirelessBuffer.rdIdx = 0;
                }
                
                if (wirelessBuffer.count != 0)
                {
                        Sending();
                }
        }
        else if (strcmp(*message, ">") == 0)
        {
                sendStringMessageToWireless(&rasb_TxMessageBuffer[0u], (uint32_t)rul_TxMessageSize);
        }
        else if (strcmp(*message, "JPAN") == 0)
        {
                wirelessDetected = (SUCCESS); 
                wirelessState = WIRELESS_IDLE;   
                
                if((uint8_t)(strcmp(message[3], "12345678AAFFDD00") == 0))
                {
                        wirelessCustom = 1;
                        memcpy(Network_Name, message[4], 4);
                        get_wireless_name = 0;
                        
                }
                else
                {
                        wirelessCustom = 2;  
                        memcpy(Network_Name, message[2], 4);
                        get_wireless_name = 0;
                }
        }
        else if (strcmp(*message, "LeftPAN") == 0)
        {
                is_wireless_setup = 3;
        }
        else if (strcmp(*message, "+N") == 0)
        {
                if((uint8_t)(strcmp(message[1], "NoPAN") == 0))
                {
                        is_wireless_setup = 3;//module must join the network
                }
                else
                {
                        if((uint8_t)(strcmp(message[5], "12345678AAFFDD00") == 0))
                        {
                                wirelessCustom = 1;
                                memcpy(Network_Name, message[4], 4);
                                get_wireless_name = 0;
                        }
                        else
                        {
                                wirelessCustom = 2;  
                                memcpy(Network_Name, message[4], 4);
                                get_wireless_name = 0;
                        }
                        
                        //save wireless channel
                        char chan_wir[2] = {0};
                        memcpy(chan_wir, message[2], 2);
                        current_wir_chan = 10*(chan_wir[0] - '0') + (chan_wir[1] - '0');
                        if(current_wir_chan > 10){
                                current_wir_chan = current_wir_chan - 10;
                        }
                        
                        
                        if((uint8_t)(strcmp(message[1], "COO") == 0))
                        {
                                wireless_mode = 1;// coordinator
                        }
                        else if((uint8_t)(strcmp(message[1], "FFD") == 0))
                        {
                                wireless_mode = 2; //routrer
                        }
                        else
                        {
                                wireless_mode = 3; //others
                        }
                        
                        if(get_ABox() == 1)//if it is Audiobox
                        {
                                if(wireless_mode == 1)//if it is coordinator
                                {
                                        is_wireless_setup = 2;//Remote must left the network
                                }
                                else
                                {
                                        if((uint8_t)(strcmp(message[4], "AAAA") == 0) || wirelessCustom == 2)
                                        {
                                                is_wireless_setup = 200;//AB is joined OK
                                                wirelessDetected = (SUCCESS); 
                                                wirelessState = WIRELESS_IDLE;
                                        }
                                        else
                                        {
                                                is_wireless_setup = 2;//Remote must left the network
                                        }
                                }
                        }
                        else
                        {
                                if(wireless_mode != 1)//if it is coordinator
                                {
                                        is_wireless_setup = 2;//Remote must left the network
                                }
                                else
                                {
                                        if((uint8_t)(strcmp(message[4], "AAAA") == 0)  || wirelessCustom == 2)
                                        {
                                                is_wireless_setup = 100;//Remote started OK
                                                wirelessDetected = (SUCCESS); 
                                                wirelessState = WIRELESS_IDLE;
                                        }
                                        else
                                        {
                                                is_wireless_setup = 2;//remote must left the network
                                        }
                                }
                        }
                        
                        
                }
                
        }
        else if (strcmp(*message, "+PANSCAN") == 0)
        {
                
                if((uint8_t)(strcmp(message[2], "AAAA") == 0))
                {
                        //  wirelessDetected = (SUCCESS); 
                }
        }
        else if (strcmp(*message, "Telegesis ETRX357HR-LRS") == 0)
        {

        }
        else if ((strcmp(*message, "ERROR") == 0))
       {
                if (wirelessState == WIRELESS_BUSY)
                {
                        wirelessState = (WIRELESS_IDLE);
                }
               
                if (wirelessBuffer.count == 0)
                {
                        return;
                }
                
                wirelessBuffer.count--;
                wirelessBuffer.rdIdx++;
                if (wirelessBuffer.rdIdx == WIRELESS_TX_BUF_SIZE)
                {
                        wirelessBuffer.rdIdx = 0;
                }
                
                if (wirelessBuffer.count != 0)
                {
                        Sending();
                }
                
             static uint16_t err_wir = 0;
             err_wir++;
       } 
        else if ((strcmp(*message, "BCAST") == 0)
                 || (strcmp(*message, "UCAST") == 0))
        {
                decodeDataMessage(message, parameterCount);
                signal_flag = 0;
                t_signal_span = GetCurrentSystemTime();
        }
        else
        {
                
                if (Wireless_add_req == 1)
                {
                        setCoordinatorEUI64Address(*message, 0);
              
                        if(coordinatorAddressX > 0xFFFF){
                                Wireless_add_req = 0;
                        }
                }
                
                if (get_wireless_name == 1)
                {
                       uint32_t tmp_adr_net = 0;
                       memcpy(Network_Name, message[0], 4);
                       
                       tmp_adr_net = Network_Name[3] << 24 | Network_Name[2] << 16 | Network_Name[1] << 8 | Network_Name[0] << 0;
                        
                       if(tmp_adr_net != 0){
                                get_wireless_name = 0;
                       }
                }
                
        }
        
}


uint8_t get_val_noise(uint8_t chan_x){
        return Med_chan_energy[chan_x];
}

uint8_t get_Wchan_stat(uint8_t chan_a){
        return Rec_chan_energy[chan_a];
}


void update_signal_values(void){
        
        for(int i = 0; i < MAX_WIR_CHAN; i++){
                
                if(chan_energy[i] > (LVL_QUIET - 27)){
                        if(last_chan_energy[i] < 40){
                                last_chan_energy[i]++;
                        }
                }
                else{
                        if(last_chan_energy[i] > 0){
                                last_chan_energy[i]--;
                        }
                }
                if(Med_chan_energy[i] == 0){
                        Med_chan_energy[i] = chan_energy[i];
                }else{
                        Med_chan_energy[i] = (Med_chan_energy[i] + chan_energy[i]) / 2;
                }
         }
        sor_rec_chans();
        was_SET_Disp = 5;
}


void sor_rec_chans(void){

        
         for(int a = 0; a < MAX_WIR_CHAN; a++){

                        if(Med_chan_energy[a] < LVL_VERY_QUIET - 27){
                                        Rec_chan_energy[a] = 1;
                        }
                        else if(Med_chan_energy[a] < LVL_VERY_QUIET - 27 + 5){
                                        Rec_chan_energy[a] = 2;
                        }
                        else{
                                        Rec_chan_energy[a] = 3;
                        }
                       
                       if(last_chan_energy[a] > 10){
                                Rec_chan_energy[a] = 4;
                       }
                       else if(last_chan_energy[a] > 5){
                               if(Rec_chan_energy[a] < 3){
                                        Rec_chan_energy[a] = 3;
                               }
                       }
                }
        }

void scan_energy(void){
static uint8_t was_chan_scan = 0;

if(was_chan_scan == 16){
        if(tempBuf[0] == '2' && tempBuf[1] == '6' && tempBuf[2] == ':'){
                chan_energy[was_chan_scan - 1] = hex_to_ascii(tempBuf[3], tempBuf[4]);
                was_chan_scan = 0;
                was_scan_complete++;
                update_signal_values();
                scan_wifi_energ = 2;
        }else{
                was_chan_scan = 0;
                scan_wifi_energ = 0;
        }
}        
else if(was_chan_scan == 15){
        if(tempBuf[0] == '2' && tempBuf[1] == '5' && tempBuf[2] == ':'){
                chan_energy[was_chan_scan - 1] = hex_to_ascii(tempBuf[3], tempBuf[4]);
                was_chan_scan = 16;
        }else{
                was_chan_scan = 0;
                scan_wifi_energ = 0;
        }
} 
else if(was_chan_scan == 14){
        if(tempBuf[0] == '2' && tempBuf[1] == '4' && tempBuf[2] == ':'){
                chan_energy[was_chan_scan - 1] = hex_to_ascii(tempBuf[3], tempBuf[4]);
                was_chan_scan = 15;
        }else{
                was_chan_scan = 0;
                scan_wifi_energ = 0;
        }
}        
else if(was_chan_scan == 13){
        if(tempBuf[0] == '2' && tempBuf[1] == '3' && tempBuf[2] == ':'){
                chan_energy[was_chan_scan - 1] = hex_to_ascii(tempBuf[3], tempBuf[4]);
                was_chan_scan = 14;
        }else{
                was_chan_scan = 0;
                scan_wifi_energ = 0;
        }
}
else if(was_chan_scan == 12){
        if(tempBuf[0] == '2' && tempBuf[1] == '2' && tempBuf[2] == ':'){
                chan_energy[was_chan_scan - 1] = hex_to_ascii(tempBuf[3], tempBuf[4]);
                was_chan_scan = 13;
        }else{
                was_chan_scan = 0;
                scan_wifi_energ = 0;
        }
}        
else if(was_chan_scan == 11){
        if(tempBuf[0] == '2' && tempBuf[1] == '1' && tempBuf[2] == ':'){
                chan_energy[was_chan_scan - 1] = hex_to_ascii(tempBuf[3], tempBuf[4]);
                was_chan_scan = 12;
        }else{
                was_chan_scan = 0;
                scan_wifi_energ = 0;
        }
} 
else if(was_chan_scan == 10){
        if(tempBuf[0] == '2' && tempBuf[1] == '0' && tempBuf[2] == ':'){
                chan_energy[was_chan_scan - 1] = hex_to_ascii(tempBuf[3], tempBuf[4]);
                was_chan_scan = 11;
        }else{
                was_chan_scan = 0;
                scan_wifi_energ = 0;
        }
}        
else if(was_chan_scan == 9){
        if(tempBuf[0] == '1' && tempBuf[1] == '9' && tempBuf[2] == ':'){
                chan_energy[was_chan_scan - 1] = hex_to_ascii(tempBuf[3], tempBuf[4]);
                was_chan_scan = 10;
        }else{
                was_chan_scan = 0;
                scan_wifi_energ = 0;
        }
}  
else if(was_chan_scan == 8){
        if(tempBuf[0] == '1' && tempBuf[1] == '8' && tempBuf[2] == ':'){
                chan_energy[was_chan_scan - 1] = hex_to_ascii(tempBuf[3], tempBuf[4]);
                was_chan_scan = 9;
        }else{
                was_chan_scan = 0;
                scan_wifi_energ = 0;
        }
}        
else if(was_chan_scan == 7){
        if(tempBuf[0] == '1' && tempBuf[1] == '7' && tempBuf[2] == ':'){
                chan_energy[was_chan_scan - 1] = hex_to_ascii(tempBuf[3], tempBuf[4]);
                was_chan_scan = 8;
        }else{
                was_chan_scan = 0;
                scan_wifi_energ = 0;
        }
} 
else if(was_chan_scan == 6){
        if(tempBuf[0] == '1' && tempBuf[1] == '6' && tempBuf[2] == ':'){
                chan_energy[was_chan_scan - 1] = hex_to_ascii(tempBuf[3], tempBuf[4]);
                was_chan_scan = 7;
        }else{
                was_chan_scan = 0;
                scan_wifi_energ = 0;
        }
}        
else if(was_chan_scan == 5){
        if(tempBuf[0] == '1' && tempBuf[1] == '5' && tempBuf[2] == ':'){
                chan_energy[was_chan_scan - 1] = hex_to_ascii(tempBuf[3], tempBuf[4]);
                was_chan_scan = 6;
        }else{
                was_chan_scan = 0;
                scan_wifi_energ = 0;
        }
}  
else if(was_chan_scan == 4){
        if(tempBuf[0] == '1' && tempBuf[1] == '4' && tempBuf[2] == ':'){
                chan_energy[was_chan_scan - 1] = hex_to_ascii(tempBuf[3], tempBuf[4]);
                was_chan_scan = 5;
        }else{
                was_chan_scan = 0;
                scan_wifi_energ = 0;
        }
}        
else if(was_chan_scan == 3){
        if(tempBuf[0] == '1' && tempBuf[1] == '3' && tempBuf[2] == ':'){
                chan_energy[was_chan_scan - 1] = hex_to_ascii(tempBuf[3], tempBuf[4]);
                was_chan_scan = 4;
        }else{
                was_chan_scan = 0;
                scan_wifi_energ = 0;
        }
} 
else if(was_chan_scan == 2){
        if(tempBuf[0] == '1' && tempBuf[1] == '2' && tempBuf[2] == ':'){
                        chan_energy[was_chan_scan - 1] = hex_to_ascii(tempBuf[3], tempBuf[4]);
                        was_chan_scan = 3;

        }else{
                was_chan_scan = 0;
                scan_wifi_energ = 0;
        }
}        
else if(was_chan_scan == 1){
        if(tempBuf[0] == '1' && tempBuf[1] == '1' && tempBuf[2] == ':'){
               chan_energy[was_chan_scan - 1] = hex_to_ascii(tempBuf[3], tempBuf[4]);
               was_chan_scan = 2;
        }else{
                was_chan_scan = 0;
                scan_wifi_energ = 0;
        }
}
else if(was_chan_scan == 0){
         if(tempBuf[0] == '+' && tempBuf[1] == 'E' && tempBuf[2] == 'S' && tempBuf[3] == 'C' && tempBuf[4] == 'A' && tempBuf[5] == 'N'){
                        was_chan_scan = 1;
         }
 }
}


uint8_t hex_to_int(char c){
        uint8_t first = c / 16 - 3;
        uint8_t second = c % 16;
        uint8_t result = first*10 + second;
        if(result > 9) result--;
        return result;
}

int8_t hex_to_ascii(char c, char d){
        uint8_t high = hex_to_int(c) * 16;
        uint8_t low = hex_to_int(d);
        int8_t tmp_nmr = high + low + 100;//127;
        return tmp_nmr;
}


void update_signal_stats(void){
        scan_wifi_energ = 1;
        sendStringMessageToWireless(cfgWifiScan, (uint32_t)strlen(cfgWifiScan));
}


/********************************************************************
@fn	static void getWirMessages(void)
@param[in] Nothing
@return Nothing
@brief Read and process incoming wireless messages (if any)
*********************************************************************/
void getWirMessages(void)
 {
       Sending();  
         
       //static uint8_t tempBuf[256];
       static char resultBuf[MAX_NUMBER_OF_WIRELESS_PARAMETERS][MAX_WIRELESS_MESSAGE_SIZE];
       static char *pBuf[MAX_WIRELESS_MESSAGE_SIZE];
       uint32_t n=0;
       uint32_t msgSize = 0;
       int count=0;
       
       memset(resultBuf, 0x00, sizeof(resultBuf));
       memset(tempBuf, 0x00, sizeof(tempBuf));
       memset(pBuf, 0x00, MAX_WIRELESS_MESSAGE_SIZE);
       
       for(n=0; n<MAX_NUMBER_OF_WIRELESS_PARAMETERS; n++){
             pBuf[n]=resultBuf[n];
       }

       
       
       while((count = UartWirReceive(tempBuf)) > 0){

               
             if(scan_wifi_energ == 1){
                scan_energy();  
             }
               
             msgSize = processMsg(tempBuf, count, pBuf);

             for(n=0; n<MAX_NUMBER_OF_WIRELESS_PARAMETERS; n++){
                   pBuf[n]=resultBuf[n];
             }
             
             if(msgSize > 0){
                        parseMessage(pBuf, msgSize);
              }
             
             memset(tempBuf, 0x00, sizeof(tempBuf));
             memset(resultBuf, 0x00, sizeof(tempBuf));
       }
       
        if (wirelessState == WIRELESS_BUSY && CheckTimeSpan(t_span_wir) > MAX_WAIT_TIME && get_is_wireless_mod_OK() == 1) {
            
                wirelessState = WIRELESS_IDLE;  
                
                wirelessBuffer.count--;
                wirelessBuffer.rdIdx++;
                if (wirelessBuffer.rdIdx == WIRELESS_TX_BUF_SIZE)
                {
                        wirelessBuffer.rdIdx = 0;
                }
                
                if (wirelessBuffer.count != 0)
                {
                        Sending();
                }
        }
       /*
        if(wirelessBuffer.count > 3){
            wirelessState = WIRELESS_IDLE;
        }
        */
       
       //TO_ADD - WIRELESS ????
       
 }

/********************************************************************
//jcm-Complete
@fn	Status SendWirelessMessageBroadcast(const uint8_t *message, uint32_t messageLength)
@param[in] message Pointer to the message
@param[in] messageLength Size of the message
@return SUCCESS/ERROR
@brief Send broadcast to all of the wireless devices
*********************************************************************/
static uint8_t SendWirelessMessageBroadcast(const uint8_t *message, uint32_t messageLength){
      int32_t result = 0u;
      
//      tmp_wir_message_sent_Broadcast++;  

        
      if(wirelessState == WIRELESS_IDLE)
       {
             wirelessState = WIRELESS_BUSY;
             result = sendMessageToWireless(WIRELESS_SEND_DATA, 0, message, messageLength, WIRELESS_MESSAGE_BROADCAST);
             
       }
      
      if(result > 0)
       {
             return (SUCCESS);
       }
      
      return (ERROR);
}


/********************************************************************
//jcm-Complete
@fn	Status SendWirelessMessageToAddress(const uint8_t *message, uint32_t messageLength, uint64_t address)
@param[in] message Pointer to the message
@param[in] messageLength Size of the message
@param[in] address MAC address of the wireless device
@return SUCCESS/ERROR
@brief Send a message to device at MAC address
*********************************************************************/
static uint8_t SendWirelessMessageToAddress(const uint8_t *message, uint32_t messageLength, uint64_t address){
      int32_t result=0u;
      
  //      tmp_wir_message_sent_Unicast++;        
        
      if(wirelessState == WIRELESS_IDLE && address != 0){
            wirelessState = WIRELESS_BUSY;
            result = message[1];
            result <<= 8;
            result += message[2];
            
            
            
            result = sendMessageToWireless(WIRELESS_SEND_DATA, address, message, messageLength, WIRELESS_MESSAGE_ADDRESSED);
            
      }
      if(result > 0){
            return (SUCCESS);
      }
      
      return (ERROR);
}



/********************************************************************
//jcm-Complete
@fn	void HardResetWireless(void)
@param[in] Nothing
@return Nothing
@brief Hard-reset the wireless device
@details Sets the wireless device reset line to low state.
Use this when the rest of the code is not running (at the device start)
*********************************************************************/

// adaugat 06.12.2015

void saveAB_adr(MS_Message *msg)
 {
       uint64_t ab_adr1 = 0ul; 
       
       ab_adr1 = (((uint64_t)msg->data[7] << 56) & 0xFF00000000000000U)
             | (((uint64_t)msg->data[6] << 48) & 0x00FF000000000000U)
                   | (((uint64_t)msg->data[5] << 40) & 0x0000FF0000000000U)
                         | (((uint64_t)msg->data[4] << 32) & 0x000000FF00000000U)
                               | ((msg->data[3] << 24) & 0x00000000FF000000U)
                                     | ((msg->data[2] << 16) & 0x0000000000FF0000U)
                                           | ((msg->data[1] <<  8) & 0x000000000000FF00U)
                                                 | (msg->data[0]        & 0x00000000000000FFU);
       
       AB_Address = ab_adr1;
 }

// end 06.12.2015






/********************************************************************
@fn	Wireless_TxMsgStatus Wireless_Send(const Device_Message *message)
@param[in] message pointer to the message that is to be sent
@return Wireless_TxMsgStatus enum value
@brief Send message to the wireless device
*********************************************************************/
Wireless_TxMsgStatus Wireless_Send(const MS_Message *message)
 {
//       is_msg_snt_dbg++;
         
       if(was_started_config == 1) return (WIRELESS_TX_OK);  
         
       Sending();
         
       if (wirelessBuffer.count == WIRELESS_TX_BUF_SIZE - 1) return WIRELESS_TX_ERROR;
       
       memcpy(&(wirelessBuffer.buffer[wirelessBuffer.wrIdx]), message, sizeof(MS_Message));
       wirelessBuffer.wrIdx++;
       if (wirelessBuffer.wrIdx == WIRELESS_TX_BUF_SIZE) wirelessBuffer.wrIdx = 0;
       wirelessBuffer.count++;
       
       Sending();
       return (WIRELESS_TX_OK);
 }
/********************************************************************
@fn	Wireless_TxMsgStatus Wireless_Send(const Device_Message *message)
@param[in] message pointer to the message that is to be sent
@return Wireless_TxMsgStatus enum value
@brief Send message to the wireless device
*********************************************************************/
static void Sending(void)
 {
        uint8_t buf[40];
        uint8_t checksum=0;
        uint32_t localAddressBuffer=0;
        uint8_t result= (ERROR);
        MS_Message *message;
        //  static uint64_t t_spanF = 0;
        
        //if (CheckTimeSpan(t_spanF) < TIME_DELAY_BETWEEN_WIRELESS_FRAMES) return; //rprg 04.08.2017
       // dif_message_wir0 = GetCurrentSystemTime() - t_span;
        if (wirelessBuffer.count == 0) return;
      //   dif_message_wir1 = GetCurrentSystemTime() - t_span;
        if (wirelessState != WIRELESS_IDLE) return;
        
     //   dif_message_wir2 = GetCurrentSystemTime() - t_span;
         
        t_span = GetCurrentSystemTime();
         
        //t_spanF = GetCurrentSystemTime();
        message = &(wirelessBuffer.buffer[wirelessBuffer.rdIdx]);
        buf[0] = (message->dataLength)+3;
        localAddressBuffer = message->address;
        buf[1] = (uint8_t)(localAddressBuffer >> 8);
        buf[2] = (uint8_t)localAddressBuffer;
        memcpy(buf+3, message->data, message->dataLength);
        
        checksum=calculateXorChecksum(buf, buf[0]);
        buf[buf[0]]=(uint8_t)checksum;
        
        //Wir_MSG_NAKG_MS = GetCurrentSystemTime();

	if (message->address < MASTER_SLAVE_Error) {
                        if (!get_device_datas()->isMaster) {
                                result = SendWirelessMessageToAddress(buf, buf[0]+1, coordinatorAddress);
                                
                        }
                        else {
                              
                                if(message->data[0] == AB_CMD && localAddressBuffer > MASTER_SLAVE_Ping && localAddressBuffer < (MASTER_SLAVE_Ping + MAX_NUMBER_OF_AB) 
                                                && wirelessDeviceTableAB[localAddressBuffer] > 0){
                                                        result = SendWirelessMessageToAddress(buf, buf[0]+1, wirelessDeviceTableAB[localAddressBuffer]);// send a command top AB
                                         }                                
                                else if (wirelessDeviceTable[localAddressBuffer % MAX_NUMBER_OF_DEVICES] > 0)
                                {
                                            result = SendWirelessMessageToAddress(buf, buf[0]+1, wirelessDeviceTable[localAddressBuffer % MAX_NUMBER_OF_DEVICES]);// send a command to module
                                }
                                else
                                {
                                        
                                        
                                                wirelessBuffer.count--;
                                                wirelessBuffer.rdIdx++;
                                                if (wirelessBuffer.rdIdx == WIRELESS_TX_BUF_SIZE)
                                                {
                                                        wirelessBuffer.rdIdx = 0;
                                                }
                                }
                        }
	}
	else if (message->address >= MASTER_SLAVE_Error) {
                        result = SendWirelessMessageBroadcast(buf, buf[0] + 1);
                        
	}
	// problemmmmmmmm
	if (result != (ERROR)) {
                        wirelessState = WIRELESS_BUSY;
                        
                        //TODO-jcm - have to check if it is correct?
	}
      
 }

void clearDeviceAddressForWireless(uint32_t address)
 {
       memset(wirelessDeviceTable,0x00 , sizeof(wirelessDeviceTable));
       if (address < MAX_NUMBER_OF_DEVICES)
             wirelessDeviceTable[address] = 0;
 }

/********************************************************************
@fn	Status IsWirelessDetected(void)
@param[in] Nothing
@return SUCCESS/ERROR
@brief Check is wireless device detected on the UART
*********************************************************************/
uint8_t IsWirelessDetected(void){
      return wirelessDetected;
}

/********************************************************************
@fn	void getWirelessEvents(void)
@param[in] Nothing
@return Nothing
@brief Wireless device housekeeping tasks
*********************************************************************/

void getWirelessMessage(void){
      
      getWirMessages();
        

              if (wirelessState == WIRELESS_BUSY && CheckTimeSpan(t_span) > MAX_WAIT_TIME) {
            wirelessState = WIRELESS_IDLE;
                      /*
                 
                       wirelessBuffer.count--;
                wirelessBuffer.rdIdx++;
                if (wirelessBuffer.rdIdx == WIRELESS_TX_BUF_SIZE)
                {
                        wirelessBuffer.rdIdx = 0;
                }
                if (wirelessBuffer.count != 0)
                {
                        Sending();
                }
                */
      }
        

}







void set_Wireless_NetAAAA(void)
 {
       
       
       const char *cfgEPANIdX = "ATS03=12345678AAFFDD00";
       
       
       sendStringMessageToWireless(cfgEPANIdX, (uint32_t)strlen(cfgEPANIdX));
      for(int i = 0; i < 5; i++)
      {
            Delay(10);
            getWirMessages();             
      }
    
     //TO_ADD - WIRELESS ????
       
       
}


/********************************************************************
//jcm-Complete
@fn	void InitWireless(void)
@param[in] Nothing
@return Nothing
@brief Initialise wireless device
*********************************************************************/
void InitWireless(void) {


      memset(&wirelessBuffer, 0x00, sizeof(wirelessBuffer));
      memset(wirelessDeviceTable, 0x00, MAX_NUMBER_OF_DEVICES * sizeof(uint64_t));
      
      signal_flag = 0;
      wirelessState = WIRELESS_INIT;
      get_device_datas()->wireless_signal_value = (uint8_t)INT32_MIN;
        
      //Config_Wir(0);
        
     // startWireless();
        
      //set_Master();
      //startWirelessM();
       startWirelessAB();
        
        
     // wirelessState = WIRELESS_INIT;
        
    
      
      

}


// 0 -> Standard
// 1 -> Custom EPAN
// 2 -> Custom PAN and EPAN
// 3 -> Custom New PAN and EPAN
void config_PAN_EPAN(uint8_t network_set)
{
        char tmp_PAN[24];
        char tmp_EPAN[24];

        
        uint16_t coordinatorAddressXA = 0;
        
        
        memset(tmp_PAN, 0x00, sizeof(tmp_PAN));
        memset(tmp_EPAN, 0x00, sizeof(tmp_EPAN));
        memset(tmp_CMD_PAN, 0x00, sizeof(tmp_CMD_PAN));
        memset(tmp_CMD_EPAN, 0x00, sizeof(tmp_CMD_EPAN));
        
        sprintf(tmp_CMD_PAN, "ATS02=");
        sprintf(tmp_CMD_EPAN, "ATS03=");
        
        if (network_set == 1) // change only EPAN
        {
                
                sprintf(tmp_PAN, "%s", "AAAA");
                strcat(tmp_CMD_PAN, tmp_PAN);
                
                if(get_device_datas()->isMaster == 1 && get_device_datas()->join_status == 0)
                {
                        sprintf(tmp_EPAN, "%016llX", coordinatorAddressX);
                        strcat(tmp_CMD_EPAN, tmp_EPAN);
                        
                        //save_custom_Wir(coordinatorAddressX);
                }
                else if (get_device_datas()->isMaster == 0 && get_device_datas()->join_status == 1)
                {
                        sprintf(tmp_EPAN, "%016llX", coordinatorAddress);
                        strcat(tmp_CMD_EPAN, tmp_EPAN);
                        
                        //save_custom_Wir(coordinatorAddress);
                }
                else
                {
                        sprintf(tmp_EPAN, "%s", "12345678AAFFDD00");
                        strcat(tmp_CMD_EPAN, tmp_EPAN);
                        //save_custom_Wir(0);
                }
                
                
        } 
        else if (network_set == 2)// change EPAN and PAN
        {
                
                
                
                if(get_device_datas()->isMaster == 1 && get_device_datas()->join_status == 0)
                {
                        coordinatorAddressXA = (uint16_t)(coordinatorAddressX);
                        
                        sprintf(tmp_PAN, "%04X", coordinatorAddressXA);
                        strcat(tmp_CMD_PAN, tmp_PAN);
                        
                        sprintf(tmp_EPAN, "%016llX", coordinatorAddressX);
                        strcat(tmp_CMD_EPAN, tmp_EPAN);
                        
                     //   save_custom_Wir(coordinatorAddressXA);
                }
                else if (get_device_datas()->isMaster == 0 && get_device_datas()->join_status == 1)
                {
                        coordinatorAddressXA = (uint16_t)(coordinatorAddress);
                        
                        sprintf(tmp_PAN, "%04X", coordinatorAddressXA);
                        strcat(tmp_CMD_PAN, tmp_PAN);
                        
                        sprintf(tmp_EPAN, "%016llX", coordinatorAddress);
                        strcat(tmp_CMD_EPAN, tmp_EPAN);
                        
                        //save_custom_Wir(coordinatorAddressXA);
                }
                else
                {
                        
                        
                        sprintf(tmp_PAN, "%s", "AAAA");
                        strcat(tmp_CMD_PAN, tmp_PAN);
                        
                        sprintf(tmp_EPAN, "%s", "12345678AAFFDD00");
                        strcat(tmp_CMD_EPAN, tmp_EPAN);
                        
//                        save_custom_Wir(0);
                }
                
        }
        else if (network_set == 3)// change New EPAN and PAN
        {
               if (get_device_datas()->isMaster == 0 && get_device_datas()->join_status == 1)
                {
                        //coordinatorAddressXA = (uint16_t)(coordinatorAddress);

                        sprintf(tmp_PAN, "%s", wir_network_rem_name);
                        strcat(tmp_CMD_PAN, tmp_PAN);
                        
                        sprintf(tmp_EPAN, "%s", "12345678AAFF");
                        strcat(tmp_CMD_EPAN, tmp_EPAN);
                        
                        memset(tmp_EPAN, 0x00, sizeof(tmp_EPAN));
                        
                        sprintf(tmp_EPAN, "%s", wir_network_rem_name);
                        strcat(tmp_CMD_EPAN, tmp_EPAN);
                        
                }
                else
                {
                        
                        sprintf(tmp_PAN, "%s", "AAAA");
                        strcat(tmp_CMD_PAN, tmp_PAN);
                        
                        sprintf(tmp_EPAN, "%s", "12345678AAFFDD00");
                        strcat(tmp_CMD_EPAN, tmp_EPAN);
                        
                }
                
        }
        else // Default PAN and EPAN
        {
                sprintf(tmp_PAN, "%s", "AAAA");
                strcat(tmp_CMD_PAN, tmp_PAN);
                
                sprintf(tmp_EPAN, "%s", "12345678AAFFDD00");
                strcat(tmp_CMD_EPAN, tmp_EPAN);
                
           //     save_custom_Wir(0);
        }
        
        
        
        
        sendStringMessageToWireless(cfgDisconnect, (uint32_t)strlen(cfgDisconnect));
        
        uint32_t t_spanA = 0;
        t_spanA = GetCurrentSystemTime();
        
        while ((GetCurrentSystemTime() - t_spanA) < cf_wait_time)
        {
                getWirMessages();
        }
        
        
        sendStringMessageToWireless(tmp_CMD_PAN, (uint32_t)strlen(tmp_CMD_PAN));
        t_spanA = GetCurrentSystemTime();
        
        
        while ((GetCurrentSystemTime() - t_spanA) < cf_wait_time)
        {
                getWirMessages();
        }
        
        sendStringMessageToWireless(tmp_CMD_EPAN, (uint32_t)strlen(tmp_CMD_EPAN));
        t_spanA = GetCurrentSystemTime();
        while ((GetCurrentSystemTime() - t_spanA) < cf_wait_time)
        {
                getWirMessages();             
        }
}


// 0 -> Standard
// 1 -> Custom EPAN
// 2 -> Custom PAN and EPAN
void Config_Wir(uint8_t wir_cfg_mode)
{
        
        hard_reset_wireless(); //Reset to default settings
        
        sendStringMessageToWireless(cfgDetection, (uint32_t)strlen(cfgDetection));

        t_span = GetCurrentSystemTime();
        
        while ((GetCurrentSystemTime() - t_span) < cf_wait_time)
        {
                getWirMessages();             
        }
        
        config_PAN_EPAN(wir_cfg_mode); // set WirelessPAN
        
        config_basic_wir();// config basic settings wireless module
        
        while ((GetCurrentSystemTime() - t_span) < cf_wait_time)
        {
                getWirMessages();             
        }
        
}





//wireless module default settings reset

void hard_reset_wireless(void)
{
        MX_WIRX_TX_Init(115200);
        
        t_span = GetCurrentSystemTime();

        sendStringMessageToWireless(cfgHReset, (uint32_t)strlen(cfgHReset));
        
        
        while ((GetCurrentSystemTime() - t_span) < 5000)
        {
                getWirMessages();      
      
                
        }  
        t_span = GetCurrentSystemTime();             
        
        
        MX_WIRX_TX_Init(19200);
        
        while ((GetCurrentSystemTime() - t_span) < 1000)
        {
                getWirMessages();             
        }  
        t_span = GetCurrentSystemTime();             
        
        
        t_span = GetCurrentSystemTime();
        sendStringMessageToWireless(cfgHReset, (uint32_t)strlen(cfgHReset));
        
        
        while ((GetCurrentSystemTime() - t_span) < 2000)
        {
                getWirMessages();             
        }  
        t_span = GetCurrentSystemTime();
        
        
}


void config_basic_wir(void)
{
        sendStringMessageToWireless(cfgChannelMask, (uint32_t)strlen(cfgChannelMask));
        t_span = GetCurrentSystemTime();
        
        while ((GetCurrentSystemTime() - t_span) < cf_wait_time)
        {
                getWirMessages();
        }

        sendStringMessageToWireless(cfgPower, (uint32_t)strlen(cfgPower));
        t_span = GetCurrentSystemTime();
        
        
        while ((GetCurrentSystemTime() - t_span) < cf_wait_time)
        {
                getWirMessages();
        }
        
        sendStringMessageToWireless(cfgTransmissionPower, (uint32_t)strlen(cfgTransmissionPower));
        t_span = GetCurrentSystemTime();
        
        
        while ((GetCurrentSystemTime() - t_span) < cf_wait_time)
        {
                getWirMessages();
        }
        
       
        
        sendStringMessageToWireless(cfgResponses, (uint32_t)strlen(cfgResponses));
        t_span = GetCurrentSystemTime();

        
        
        while ((GetCurrentSystemTime() - t_span) < cf_wait_time)
        {
                getWirMessages();
        }
        
        
        sendStringMessageToWireless(cfgResp2, (uint32_t)strlen(cfgResp2));
        t_span = GetCurrentSystemTime();

        while ((GetCurrentSystemTime() - t_span) < cf_wait_time)
        {
                getWirMessages();
        }
        
        
        
        
        sendStringMessageToWireless(cfgLinkKey, (uint32_t)strlen(cfgLinkKey));
        t_span = GetCurrentSystemTime();
        
        
        while ((GetCurrentSystemTime() - t_span) < cf_wait_time)
        {
                getWirMessages();
        }   
        
        
        sendStringMessageToWireless(cfgSerial, (uint32_t)strlen(cfgSerial));
        t_span = GetCurrentSystemTime();
        
        while ((GetCurrentSystemTime() - t_span) < cf_wait_time)
        {
                getWirMessages();
        }      
        
         MX_WIRX_TX_Init(115200);
        
        while ((GetCurrentSystemTime() - t_span) < 500)
        {
                getWirMessages();
        }    
}






void startWirelessAB(void)
{
        
        sendStringMessageToWireless(cfgDetection, (uint32_t)strlen(cfgDetection));
        t_span = GetCurrentSystemTime();
        
        while ((GetCurrentSystemTime() - t_span) < 100u)
        {
                getWirMessages(); 
        }
        
        /*
        sendStringMessageToWireless(getNetName, (uint32_t)strlen(getNetName));
        t_span = GetCurrentSystemTime();
        
        while ((GetCurrentSystemTime() - t_span) < 100u)
        {
                getWirMessages(); 
        }
        */
        is_wireless_setup = 1;
        sendStringMessageToWireless(cfgInterogate, (uint32_t)strlen(cfgInterogate));
        t_span = GetCurrentSystemTime();
        
        while ((GetCurrentSystemTime() - t_span) < 100u)
        {
                getWirMessages();
                if(is_wireless_setup > 1)
                {
                        break;
                }
        }
        

        
        if(is_wireless_setup == 2)
        {
                sendStringMessageToWireless(cfgDisconnect, (uint32_t)strlen(cfgDisconnect));
                t_span = GetCurrentSystemTime();
                
                
                while ((GetCurrentSystemTime() - t_span) < 1000u)
                {
                        getWirMessages();
                        if(is_wireless_setup == 3)
                        {
                                break;
                        }
                }
        }
        
        if(is_wireless_setup == 3)
        {
                sendStringMessageToWireless(cfgRouter, (uint32_t)strlen(cfgRouter));
                t_span = GetCurrentSystemTime();
                
                while ((GetCurrentSystemTime() - t_span) < 1000u)
                {
                        getWirMessages();  
                        if(is_wireless_setup == 200)
                        {
                                break;
                        }
                        
                }
        }
        
        
        if(is_wireless_setup != 200)
        {
                sendStringMessageToWireless(cfgReset, (uint32_t)strlen(cfgReset));
                t_span = GetCurrentSystemTime();
                
                
                while ((GetCurrentSystemTime() - t_span) < 2000u)
                {
                        getWirMessages();
                        if(is_wireless_setup == 200)
                        {
                                break;
                        }
                }
                
                
                sendStringMessageToWireless(cfgInterogate, (uint32_t)strlen(cfgInterogate));
                t_span = GetCurrentSystemTime();
                
                while ((GetCurrentSystemTime() - t_span) < 3000u)
                {
                        getWirMessages();
                        if(is_wireless_setup > 1)
                        {
                                break;
                        }
                }
                
        }
     
        
        wirelessDetected = (SUCCESS); 
        wirelessState = WIRELESS_IDLE;
        
        //while(GetCurrentSystemTime() < 4000);
}

void reset_wireless_buffer(void){
      Uart_flush();
      memset(&wirelessBuffer, 0x00, sizeof(wirelessBuffer));
}


void startWirelessM(void)
{
        reset_wireless_buffer();
        
        get_wireless_name = 1;
      
        sendStringMessageToWireless(getNetName, (uint32_t)strlen(getNetName));
        t_span = GetCurrentSystemTime();
        
        while ((GetCurrentSystemTime() - t_span) < 500u)
        {
                
        }
        getWirMessages(); 
        getWirMessages(); 
        getWirMessages(); 
        
        sendStringMessageToWireless(getNetName, (uint32_t)strlen(getNetName));
        t_span = GetCurrentSystemTime();
        
        while ((GetCurrentSystemTime() - t_span) < 500u)
        {
                
        }
        getWirMessages(); 
        getWirMessages(); 
        getWirMessages(); 
        
        Wireless_add_req = 1;
        sendStringMessageToWireless(getNetAddr, (uint32_t)strlen(getNetAddr));
        
        t_span = GetCurrentSystemTime();
                
        while ((GetCurrentSystemTime() - t_span) < 500u)
                {
                        
                }
        
        getWirMessages(); 
        getWirMessages(); 
        getWirMessages();
                
        t_span = GetCurrentSystemTime();        
        sendStringMessageToWireless(cfgChannelMask, (uint32_t)strlen(cfgChannelMask));
        while ((GetCurrentSystemTime() - t_span) < 500u)
                {
                        
                }
        
        getWirMessages(); 
        getWirMessages(); 
        getWirMessages();
                
                
        sendStringMessageToWireless(getNetAddr, (uint32_t)strlen(getNetAddr));
        
        t_span = GetCurrentSystemTime();
                
        while ((GetCurrentSystemTime() - t_span) < 500u)
                {
                        
                }
        
        getWirMessages(); 
        getWirMessages(); 
        getWirMessages();
        
        sendStringMessageToWireless(cfgDetection, (uint32_t)strlen(cfgDetection));
        t_span = GetCurrentSystemTime();
        
        while ((GetCurrentSystemTime() - t_span) < 500u)
        {
                
        }
        
        getWirMessages();
        getWirMessages();
        getWirMessages();
        
        
        is_wireless_setup = 1;
        sendStringMessageToWireless(cfgInterogate, (uint32_t)strlen(cfgInterogate));
        t_span = GetCurrentSystemTime();
        
        
        while ((GetCurrentSystemTime() - t_span) < 1000u)
        {
                getWirMessages();
                if(is_wireless_setup == 2)
                {
                        break;
                }
        }
        
        if(is_wireless_setup == 2)
        {
                sendStringMessageToWireless(cfgDisconnect, (uint32_t)strlen(cfgDisconnect));
                t_span = GetCurrentSystemTime();
                
                
                while ((GetCurrentSystemTime() - t_span) < 1000u)
                {
                        getWirMessages();
                        if(is_wireless_setup == 3)
                        {
                                break;
                        }
                }
        }
        
        if(is_wireless_setup == 3)
        {
                
                sendStringMessageToWireless(cfgCoordinator, (uint32_t)strlen(cfgCoordinator));
                t_span = GetCurrentSystemTime();
                
                while ((GetCurrentSystemTime() - t_span) < 5000u)
                {
                        getWirMessages();  
                        if(is_wireless_setup == 100)
                        {
                                break;
                        }
                        
                }
        }
        
        if(is_wireless_setup != 100)
        {
                sendStringMessageToWireless(cfgReset, (uint32_t)strlen(cfgReset));
                t_span = GetCurrentSystemTime();
                
                
                while ((GetCurrentSystemTime() - t_span) < 3000u)
                {
                        getWirMessages();
                        if(is_wireless_setup == 100)
                        {
                                break;
                        }
                }
                
                sendStringMessageToWireless(cfgInterogate, (uint32_t)strlen(cfgInterogate));
                t_span = GetCurrentSystemTime();
                
                while ((GetCurrentSystemTime() - t_span) < 1000u)
                {
                        getWirMessages();
                        if(is_wireless_setup > 1)
                        {
                                break;
                        }
                }
                
        }
        
        

        sendStringMessageToWireless(cfgInterogate, (uint32_t)strlen(cfgInterogate));
                t_span = GetCurrentSystemTime();
                
                while ((GetCurrentSystemTime() - t_span) < 1000u)
                {
                        
                }
                
                getWirMessages();
                getWirMessages();
                getWirMessages();
        

                
        
        
        wirelessDetected = (SUCCESS); 
        wirelessState = WIRELESS_IDLE;
        
        while(GetCurrentSystemTime() < 4000);
        
}


void MX_WIRX_TX_Init(uint32_t uart_speed)
 {
       
       /* USER CODE BEGIN USART1_Init 0 */
       
       /* USER CODE END USART1_Init 0 */
       
       /* USER CODE BEGIN USART1_Init 1 */
       
       /* USER CODE END USART1_Init 1 */
       huart1.Instance = USART1;
       huart1.Init.BaudRate = uart_speed;
       //huart1.Init.BaudRate = 19200;
       huart1.Init.WordLength = UART_WORDLENGTH_8B;
       huart1.Init.StopBits = UART_STOPBITS_1;
       huart1.Init.Parity = UART_PARITY_NONE;
       huart1.Init.Mode = UART_MODE_TX_RX;
       huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
       huart1.Init.OverSampling = UART_OVERSAMPLING_16;
       if (HAL_UART_Init(&huart1) != HAL_OK)
        {
              Error_Handler();
        }
       /* USER CODE BEGIN USART1_Init 2 */
       
       /* USER CODE END USART1_Init 2 */
       
 }

 
 uint8_t get_ABox(void)
{
        return 0;
}

void reset_Duplicate_ID(int tmp_pops)
{
        if(tmp_pops < 0 || tmp_pops >= MAX_NUMBER_OF_DEVICES) return;  /* Bounds guard */
        ID_duplicated[tmp_pops] = 0;
}



uint8_t get_is_wireless_mod_OK(void){
        return is_wireless_respopnd;
}


void rst_wirelessDeviceTableAB(void){
        memset(wirelessDeviceTableAB,0x00 , sizeof(wirelessDeviceTableAB));
}

void rst_wirelessDeviceTableMod(void){
        memset(wirelessDeviceTable,0x00 , sizeof(wirelessDeviceTable));
}


void wireless_analyzer(void){
        scan_wifi_energ = 2;
        was_scan_complete = 0;
        memset(Med_chan_energy,0x00 , sizeof(Med_chan_energy));
        //memset(Max_chan_energy,0x00 , sizeof(Med_chan_energy));
        memset(last_chan_energy,0x00 , sizeof(Med_chan_energy));
}

void rst_wireless_analyzer(void){
        scan_wifi_energ = 0;
        was_scan_complete = 0;
        memset(Med_chan_energy,0x00 , sizeof(Med_chan_energy));
        //memset(Max_chan_energy,0x00 , sizeof(Med_chan_energy));
        memset(last_chan_energy,0x00 , sizeof(Med_chan_energy));
        
}

void change_wir_chan(uint8_t new_chan){
        
        rst_wireless_analyzer();
        //exit_chg_channel();
        

        
        
        t_span = GetCurrentSystemTime();
        while ((GetCurrentSystemTime() - t_span) < 4000u)
        {
                getWirMessages(); 
                getWirMessages(); 
                getWirMessages();        
        }
        
        char tmp_txts[24] = {0};
        
        //sprintf(tmp_txts, "AT+CCHANGE:%d", (new_chan + 10));
        if(new_chan == 1){
                sprintf(tmp_txts, "AT+CCHANGE:0B");
        }
        else if(new_chan == 2){
                sprintf(tmp_txts, "AT+CCHANGE:0C");
        }
        else if(new_chan == 3){
                sprintf(tmp_txts, "AT+CCHANGE:0D");
        }
        else if(new_chan == 4){
                sprintf(tmp_txts, "AT+CCHANGE:0E");
        }
        else if(new_chan == 5){
                sprintf(tmp_txts, "AT+CCHANGE:0F");
        }
        else if(new_chan == 6){
                sprintf(tmp_txts, "AT+CCHANGE:10");
        }
        else if(new_chan == 7){
                sprintf(tmp_txts, "AT+CCHANGE:11");
        }
        else if(new_chan == 8){
                sprintf(tmp_txts, "AT+CCHANGE:12");
        }
        else if(new_chan == 9){
                sprintf(tmp_txts, "AT+CCHANGE:13");
        }
        else if(new_chan == 10){
                sprintf(tmp_txts, "AT+CCHANGE:14");
        }
        else if(new_chan == 11){
                sprintf(tmp_txts, "AT+CCHANGE:15");
        }
        else if(new_chan == 12){
                sprintf(tmp_txts, "AT+CCHANGE:16");
        }
        else if(new_chan == 13){
                sprintf(tmp_txts, "AT+CCHANGE:17");
        }
        else if(new_chan == 14){
                sprintf(tmp_txts, "AT+CCHANGE:18");
        }
        else if(new_chan == 15){
                sprintf(tmp_txts, "AT+CCHANGE:19");
        }
        else if(new_chan == 16){
                sprintf(tmp_txts, "AT+CCHANGE:1A");
        }
        
        
        
        sendStringMessageToWireless(tmp_txts, (uint32_t)strlen(tmp_txts));
        t_span = GetCurrentSystemTime();
        
        while ((GetCurrentSystemTime() - t_span) < 10000u)
        {
                
        }
        getWirMessages(); 
        getWirMessages(); 
        getWirMessages();


        sendStringMessageToWireless(cfgInterogate, (uint32_t)strlen(cfgInterogate));
        t_span = GetCurrentSystemTime();
        
        while ((GetCurrentSystemTime() - t_span) < 100u)
        {
                
        }
        getWirMessages(); 
        getWirMessages(); 
        getWirMessages();
        
        
//        reset_SET_screen();

}

void get_wir_info(void){
sendStringMessageToWireless(cfgInterogate, (uint32_t)strlen(cfgInterogate));
        t_span = GetCurrentSystemTime();
        
        while ((GetCurrentSystemTime() - t_span) < 100u)
        {
                
        }
        getWirMessages(); 
        getWirMessages(); 
        getWirMessages();
}


void config_PAN_EPAN_new(void){
        char tmp_PAN[5];
        char tmp_EPAN[24];

        //char *AcfgLinkKey = "ATS09=987654321:password";
        
       
        memset(tmp_PAN, 0x00, sizeof(tmp_PAN));
        memset(tmp_EPAN, 0x00, sizeof(tmp_EPAN));
        memset(tmp_CMD_PAN, 0x00, sizeof(tmp_CMD_PAN));

        sprintf(tmp_CMD_PAN, "ATS02=");
        sprintf(tmp_CMD_EPAN, "ATS03=");
        

        if(get_is_custom() == 0){
                sprintf(tmp_PAN, "%s", "AAAA");
                sprintf(tmp_EPAN, "%s", "12345678AAFFDD00");
        
        }else{
                sprintf(tmp_PAN, "%04X", get_433_net_key());
                sprintf(tmp_EPAN, "12345678AA%04X%02X", get_433_net_key(), get_433_net_id());
                
        }
        
                strcat(tmp_CMD_PAN, tmp_PAN);
                strcat(tmp_CMD_EPAN, tmp_EPAN);

        sendStringMessageToWireless(cfgDisconnect, (uint32_t)strlen(cfgDisconnect));
        t_spanaaaa = GetCurrentSystemTime();
        
        while ((GetCurrentSystemTime() - t_spanaaaa) < 1000)
        {
                getWirMessages();
        }

        
        sendStringMessageToWireless(tmp_CMD_PAN, (uint32_t)strlen(tmp_CMD_PAN));
        t_spanaaaa = GetCurrentSystemTime();
        
        
        while ((GetCurrentSystemTime() - t_spanaaaa) < 1000)
        {
                getWirMessages();
        }
        
        sendStringMessageToWireless(tmp_CMD_EPAN, (uint32_t)strlen(tmp_CMD_EPAN));
        t_spanaaaa = GetCurrentSystemTime();
        while ((GetCurrentSystemTime() - t_spanaaaa) < 1000)
        {
                getWirMessages();             
        }
        
        
        sendStringMessageToWireless(cfgRouter, (uint32_t)strlen(cfgRouter));
        t_spanaaaa = GetCurrentSystemTime();
                
        while ((GetCurrentSystemTime() - t_spanaaaa) < 3000u)
        {
                 getWirMessages();  
                 
        }
        
        get_wireless_name = 1;
      
        sendStringMessageToWireless(getNetName, (uint32_t)strlen(getNetName));
        t_spanaaaa = GetCurrentSystemTime();
        
        while ((GetCurrentSystemTime() - t_spanaaaa) < 500u)
        {
             getWirMessages();   
        }

       //  WDTResetting();
}

