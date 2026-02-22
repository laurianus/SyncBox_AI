/*============================================================================*/
/*                        OBJECT SPECIFICATION                                */
/*============================================================================*
* $Source: GlobalPositioning.c $
* $Revision: 1.1 $
* Author: Alexandru Modoranu
* $Date: 2014/08/26 18:10:52CEST $
*/
/*============================================================================*/
/* FUNCTION COMMENT :                                                         */
/*!
* \file GlobalPositioning.c
* \brief Handles GPS related receive/transmit messages, buffering and parsing.
*/
/*============================================================================*/
/* COPYRIGHT (C) ALEXANDRU MODORANU 2014                                      */
/* ALL RIGHTS RESERVED                                                        */
/*============================================================================*/
/*                               OBJECT HISTORY                               */
/*============================================================================*/
/*  REVISION |   DATE      |                               |      AUTHOR      */
/*----------------------------------------------------------------------------*/
/* $Log: GlobalPositioning.c $ */

#include <string.h>
#include <stdlib.h>
#include "options.h"
#include "UART.h"
#include "GlobalPositioning.h"
#include "SystemTimer.h"


uint32_t GPS_timerValue = 0u;
uint32_t msg_gps_timer = 0u;

uint8_t isGPSTimerValid = 0u;
uint8_t GPS_Fixed = 0u;
uint8_t was_GPS_Time_Set = 0;


extern int TimeZone;




//uint8_t tmp_XXXAAA = 0;

extern RTC_HandleTypeDef hrtc;
extern RTC_TimeTypeDef sTime;
extern RTC_DateTypeDef sDate;



/*************************************************************************************************/
/*!
* \brief
* Defines the maximum number of characters that a GPS message can contain.
*/
#define GPS_MAXIMUM_MESSAGE_SIZE			(120u)

/*************************************************************************************************/
/*!
* \brief
* Defines the maximum number of parameters a GPS message can have.
*/
#define GPS_MAXIMUM_NUMBER_OF_PARAMETERS	(30u)

/*************************************************************************************************/
/*!
* \brief
* Defines the maximum length of a GPS parameter contained in a token.
*/
#define GPS_MAXIMUM_PARAMETER_SIZE			(30u)

/*************************************************************************************************/
/*!
* \brief
* Defines the parsed information as global date and time and number of tracked satellites.
*/
typedef struct 
{
	S_GpsGlobalDateTime gpsGlobalDateTime;
	uint8_t numberOfSatellites;	
} S_GpsData;

/*************************************************************************************************/
/*!
* \brief
* Defines the flags used by the module. 
*/
typedef struct 
{
	uint8_t numberOfReceivedBytes;
	uint8_t isDataValid;
	uint8_t numberOfTokens;
} S_GpsFlags;

/*************************************************************************************************/
/*!
* \brief
* Stores the GPS message read from the UART.
*/
//static char rasb_GpsDataBuffer[GPS_MAXIMUM_MESSAGE_SIZE];
static char rasb_GpsDataBuffer[GPS_MAXIMUM_MESSAGE_SIZE];

/*************************************************************************************************/
/*!
* \brief
* Stores the tokenized message received from the UART.
*/
static char rasb_GpsTokenizedData[GPS_MAXIMUM_NUMBER_OF_PARAMETERS][GPS_MAXIMUM_PARAMETER_SIZE];

/*************************************************************************************************/
/*!
* \brief
* Stores the parsed information extracted from the tokenized message.
*/
static S_GpsData rs_GpsData;

/*************************************************************************************************/
/*!
* \brief
* Stores the module flags.
*/
static S_GpsFlags rs_GpsFlags;

/* PRIVATE FUNCTION DECLARATIONS */
static void readDataFromUart(void);
static void parseData(void);
static uint8_t convertCharToHex(char value);
static uint8_t tokenize(char *input, char *output, unsigned char inc);

/* EXPORT FUNCTION IMPLEMENTATIONS */
/*************************************************************************************************/
/*!
* \brief
* Initializes the GPS module's internal data.
*/


void gps_init(void)
{
	(void)memset(&rs_GpsData, 0u, sizeof(rs_GpsData));
	(void)memset(&rasb_GpsDataBuffer[0u], 0u, sizeof(rasb_GpsDataBuffer));
        
//	UartInit((LPC_UART_TypeDef *)GPS_UART, 9600, UART_DATABIT_8, UART_STOPBIT_1, UART_PARITY_NONE);
}




/*************************************************************************************************/
/*!
* \brief
* GPS module task.
*/
void gps_task(void)
{
	readDataFromUart();
        
	if (rs_GpsFlags.isDataValid)
	{
		parseData();
	}
}



/*************************************************************************************************/
/*!
* \brief
* Used to get the global date and time from the GPS module.
*/
S_GpsGlobalDateTime gps_getGlobalDateTime(void)
{
	return rs_GpsData.gpsGlobalDateTime;
}

/*************************************************************************************************/
/*!
* \brief
* Used to get the global date from the GPS module.
*/
S_GpsGlobalDate gps_getGlobalDate(void)
{
	S_GpsGlobalDate tmp;
        
	tmp.year = rs_GpsData.gpsGlobalDateTime.year;
	tmp.month = rs_GpsData.gpsGlobalDateTime.month;
	tmp.day = rs_GpsData.gpsGlobalDateTime.day;
        
	return tmp;
}

/*************************************************************************************************/
/*!
* \brief
* Used to get the global time from the GPS module.
*/
S_GpsGlobalTime gps_getGlobalTime(void)
{
	S_GpsGlobalTime tmp;
        
	tmp.hour = rs_GpsData.gpsGlobalDateTime.hour;
	tmp.minute = rs_GpsData.gpsGlobalDateTime.minute;
	tmp.second = rs_GpsData.gpsGlobalDateTime.second;
	tmp.milliSecond = rs_GpsData.gpsGlobalDateTime.milliSecond;
        
	return tmp;
}

/*************************************************************************************************/
/*!
* \brief
* Used to get the number of tracked satellites from the GPS module.
*/
uint8_t gps_getNumberOfSatellites(void)
{
	return rs_GpsData.numberOfSatellites;
}

/* PRIVATE FUNCTION IMPLEMENTATIONS */
/*************************************************************************************************/
/*!
* \brief
* Reads data from the UART and does validity cheks on the received message.
* It checks the data start character, the data end character and the received message checksum.
*/
static void readDataFromUart(void)
{
       // static uint8_t tempBuf[256];
	uint8_t bytesCount = 0u;
	uint8_t endDataPos = 1u;	/* first character after $ */
	uint8_t msgChecksum;
	uint8_t computedChecksum;
        
	rs_GpsFlags.isDataValid = 1u;
        
	//bytesCount = UartReceive((LPC_UART_TypeDef *)GPS_UART, (uint8_t *)(&rasb_GpsDataBuffer[0u]));
        
       bytesCount = UartGPSReceive(rasb_GpsDataBuffer);
        
	if ((!(bytesCount > 0u))
            || (!(rasb_GpsDataBuffer[0u] == '$'))
                    || (!(strchr(&rasb_GpsDataBuffer[0u], '*') != NULL)))
	{
		rs_GpsFlags.isDataValid = 0u;
		return;
	}
        
	/* compute checksum */	
	computedChecksum = (uint8_t)rasb_GpsDataBuffer[endDataPos];	
	for (endDataPos = 2u; endDataPos < bytesCount; endDataPos++)
	{
		if (rasb_GpsDataBuffer[endDataPos] == '*')
		{
			break;
		}
                
		computedChecksum ^= rasb_GpsDataBuffer[endDataPos];
	}
        
	/* convert the checksum characters to numerals */
	rasb_GpsDataBuffer[endDataPos + 2u] = (char)convertCharToHex(rasb_GpsDataBuffer[endDataPos + 2u]);
	rasb_GpsDataBuffer[endDataPos + 1u] = (char)convertCharToHex(rasb_GpsDataBuffer[endDataPos + 1u]);
        
	/* extract received checksum */
	msgChecksum = ((uint8_t)rasb_GpsDataBuffer[endDataPos + 2u]) | (((uint8_t)rasb_GpsDataBuffer[endDataPos + 1u]) << 4u);
	/* compare the two checksums */
	if (computedChecksum != msgChecksum)
	{
		rs_GpsFlags.isDataValid = 0u;
		return;
	}
        
	/* delete the checksum field sign so that the string functions will stop at its position */
	rasb_GpsDataBuffer[endDataPos] = 0u;	
}

void update_GPS_time(void)
{
        if(get_device_Status() == 2 && was_GPS_Time_Set == 0)
        {
                                uint16_t HoursA = rs_GpsData.gpsGlobalDateTime.hour;
                                uint16_t MinA = rs_GpsData.gpsGlobalDateTime.minute;
                                uint16_t SecA = rs_GpsData.gpsGlobalDateTime.second;
                                
                                uint8_t year_A = rs_GpsData.gpsGlobalDateTime.year;
                                uint8_t day_A = rs_GpsData.gpsGlobalDateTime.day;
                                uint8_t month_A = rs_GpsData.gpsGlobalDateTime.month;
                                  
                                RTC_Set(year_A, month_A, day_A, HoursA, MinA, SecA); //year, month, day, hour, min, sec)
                                was_GPS_Time_Set = 1;
         }
}


void set_GPS_time(void){


        uint32_t msg_gps_timer_tmp = (uint32_t)GetCurrentSystemTime();
        
        if(msg_gps_timer_tmp - msg_gps_timer < 1000 && rs_GpsData.gpsGlobalDateTime.milliSecond == 0){
                //GPS_timerValue = rs_GpsData.gpsGlobalDateTime.hour * 3600000 + rs_GpsData.gpsGlobalDateTime.minute * 60000 + rs_GpsData.gpsGlobalDateTime.second * 1000 + 1000;
                GPS_timerValue = rs_GpsData.gpsGlobalDateTime.hour * 3600000 + rs_GpsData.gpsGlobalDateTime.minute * 60000 + rs_GpsData.gpsGlobalDateTime.second * 1000 + 1000;
                update_GPS_time();
                set_real_time(1, GPS_timerValue);  
                GPS_Fixed = 1;
                isGPSTimerValid = 3;
        }
}


/*************************************************************************************************/
/*!
* \brief
* Tokenizes the received message and then it parses it based on the relevant headers.
* The parser then stores the extracted data into the module buffers.
*/
static void parseData(void)
{
	/* clear the tokens buffer */
	(void)memset(&rasb_GpsTokenizedData[0u][0u], 0u, sizeof(rasb_GpsTokenizedData));
	/* break into tokens */
	rs_GpsFlags.numberOfTokens = 0u;
	rs_GpsFlags.numberOfTokens = tokenize(&rasb_GpsDataBuffer[0u], rasb_GpsTokenizedData[0u], GPS_MAXIMUM_PARAMETER_SIZE);
	
        //if (strcmp(rasb_GpsTokenizedData[0u], "$GPRMC") == 0)
        if (strcmp(rasb_GpsTokenizedData[0u], "$GNRMC") == 0 || strcmp(rasb_GpsTokenizedData[0u], "$GPRMC") == 0)
        {
		//if ((uint8_t)rasb_GpsTokenizedData[1u][0u] != 0x00u)
                
                /* Recommended Minimum Navigation Information */
                /* extract the UTC time */
                uint16_t tmp_h;
                tmp_h = ((uint8_t)rasb_GpsTokenizedData[1u][0u] - 0x30u) * 10u + ((uint8_t)rasb_GpsTokenizedData[1u][1u] - 0x30u);
                if (tmp_h + TimeZone < 0)
                {
                        tmp_h = tmp_h + 24 + TimeZone;
                }
                else if(tmp_h + TimeZone >23)
                {
                        tmp_h = tmp_h + TimeZone - 24;                            
                }
                else
                {
                        tmp_h = tmp_h + TimeZone;                            
                }
                
                rs_GpsData.gpsGlobalDateTime.hour = tmp_h;
                rs_GpsData.gpsGlobalDateTime.minute = ((uint8_t)rasb_GpsTokenizedData[1u][2u] - 0x30u) * 10u + ((uint8_t)rasb_GpsTokenizedData[1u][3u] - 0x30u);
                rs_GpsData.gpsGlobalDateTime.second = ((uint8_t)rasb_GpsTokenizedData[1u][4u] - 0x30u) * 10u + ((uint8_t)rasb_GpsTokenizedData[1u][5u] - 0x30u);
                rs_GpsData.gpsGlobalDateTime.milliSecond = ((uint8_t)rasb_GpsTokenizedData[1u][7u] - 0x30u) * 100u + ((uint8_t)rasb_GpsTokenizedData[1u][8u] - 0x30u) * 10u + ((uint8_t)rasb_GpsTokenizedData[1u][9u] - 0x30u);
                
                
                
                //for tests
                
                msg_gps_timer = (uint32_t)GetCurrentSystemTime();
                
                if ((uint8_t)rasb_GpsTokenizedData[9u][0u] != 0x00u)
                {
                        
                        rs_GpsData.gpsGlobalDateTime.day = ((uint8_t)rasb_GpsTokenizedData[9u][0u] - 0x30u) * 10u + ((uint8_t)rasb_GpsTokenizedData[9u][1u] - 0x30u);
                        rs_GpsData.gpsGlobalDateTime.month = ((uint8_t)rasb_GpsTokenizedData[9u][2u] - 0x30u) * 10u + ((uint8_t)rasb_GpsTokenizedData[9u][3u] - 0x30u);
                        rs_GpsData.gpsGlobalDateTime.year = ((uint8_t)rasb_GpsTokenizedData[9u][4u] - 0x30u) * 10u + ((uint8_t)rasb_GpsTokenizedData[9u][5u] - 0x30u);
                }
                
                
                
                if (strcmp(rasb_GpsTokenizedData[2u], "A") == 0u)	/* Data Valid */
                {
                        
                        isGPSTimerValid = 2;
                        
                        

                        //GPS_timerValue = rs_GpsData.gpsGlobalDateTime.hour * 3600000 + rs_GpsData.gpsGlobalDateTime.minute * 60000 + rs_GpsData.gpsGlobalDateTime.second * 1000;                        
                        
                        if ((strcmp(rasb_GpsTokenizedData[12u], "A") == 0u) || ((strcmp(rasb_GpsTokenizedData[12u], "D") == 0u))){
                                GPS_timerValue = rs_GpsData.gpsGlobalDateTime.hour * 3600000 + rs_GpsData.gpsGlobalDateTime.minute * 60000 + rs_GpsData.gpsGlobalDateTime.second * 1000;
                                isGPSTimerValid = 3;
                                GPS_Fixed = 1;
                                update_GPS_time();
                                set_real_time(1, GPS_timerValue + 150);  
                        }
                }
                else
                {
                        GPS_Fixed = 0;
                        if(rs_GpsData.gpsGlobalDateTime.year > 21 && rs_GpsData.gpsGlobalDateTime.year < 50 && rs_GpsFlags.isDataValid == 1)
                        {
                                isGPSTimerValid = 1;
                                /*
                                GPS_ModtimerValue = rs_GpsData.gpsGlobalDateTime.hour * 3600000 + rs_GpsData.gpsGlobalDateTime.minute * 60000 + rs_GpsData.gpsGlobalDateTime.second * 1000 + rs_GpsData.gpsGlobalDateTime.milliSecond;
                                */
                                
                        }else{
                                isGPSTimerValid = 5;
                        }
                }
                
                
	}
        else if (strcmp(rasb_GpsTokenizedData[0u], "$PMTK001") == 0){
                /*
                tmp_XXXAAA++;
                if(tmp_XXXAAA == 3){
                        tmp_XXXAAA = 10;
                }
        */
                
        }else{
                //do nothing
        }
        /*
        else if (strcmp(rasb_GpsTokenizedData[0u], "$GPGGA") == 0){
           uint16_t tmp_h;
                tmp_h = ((uint8_t)rasb_GpsTokenizedData[1u][0u] - 0x30u) * 10u + ((uint8_t)rasb_GpsTokenizedData[1u][1u] - 0x30u);
         }
        else if (strcmp(rasb_GpsTokenizedData[0u], "$GPZDA") == 0){
           uint16_t tmp_h;
                tmp_h = ((uint8_t)rasb_GpsTokenizedData[1u][0u] - 0x30u) * 10u + ((uint8_t)rasb_GpsTokenizedData[1u][1u] - 0x30u);
         }
        else if (strcmp(rasb_GpsTokenizedData[0u], "$GNGGA") == 0){
           uint16_t tmp_h;
                tmp_h = ((uint8_t)rasb_GpsTokenizedData[1u][0u] - 0x30u) * 10u + ((uint8_t)rasb_GpsTokenizedData[1u][1u] - 0x30u);
         }
        else if (strcmp(rasb_GpsTokenizedData[0u], "$BDGSV") == 0){
           uint16_t tmp_h;
                tmp_h = ((uint8_t)rasb_GpsTokenizedData[1u][0u] - 0x30u) * 10u + ((uint8_t)rasb_GpsTokenizedData[1u][1u] - 0x30u);
         }
        else if (strcmp(rasb_GpsTokenizedData[0u], "$GNGLL") == 0){
           uint16_t tmp_h;
                tmp_h = ((uint8_t)rasb_GpsTokenizedData[1u][0u] - 0x30u) * 10u + ((uint8_t)rasb_GpsTokenizedData[1u][1u] - 0x30u);
         }
        else if (strcmp(rasb_GpsTokenizedData[0u], "$GPTXT") == 0){
           uint16_t tmp_h;
                tmp_h = ((uint8_t)rasb_GpsTokenizedData[1u][0u] - 0x30u) * 10u + ((uint8_t)rasb_GpsTokenizedData[1u][1u] - 0x30u);
         }
        else if (strcmp(rasb_GpsTokenizedData[0u], "$GPGSA") == 0){
           uint16_t tmp_h;
                tmp_h = ((uint8_t)rasb_GpsTokenizedData[1u][0u] - 0x30u) * 10u + ((uint8_t)rasb_GpsTokenizedData[1u][1u] - 0x30u);
         }
        else if (strcmp(rasb_GpsTokenizedData[0u], "$GNVTG") == 0){
           uint16_t tmp_h;
                tmp_h = ((uint8_t)rasb_GpsTokenizedData[1u][0u] - 0x30u) * 10u + ((uint8_t)rasb_GpsTokenizedData[1u][1u] - 0x30u);
         }
        else if (strcmp(rasb_GpsTokenizedData[0u], "$BDGSA") == 0){
           uint16_t tmp_h;
                tmp_h = ((uint8_t)rasb_GpsTokenizedData[1u][0u] - 0x30u) * 10u + ((uint8_t)rasb_GpsTokenizedData[1u][1u] - 0x30u);
         }
        else if (strcmp(rasb_GpsTokenizedData[0u], "$GPGSV") == 0){
           uint16_t tmp_h;
                tmp_h = ((uint8_t)rasb_GpsTokenizedData[1u][0u] - 0x30u) * 10u + ((uint8_t)rasb_GpsTokenizedData[1u][1u] - 0x30u);
         }
        else if (strcmp(rasb_GpsTokenizedData[0u], "$GNZDA") == 0){
           uint16_t tmp_h;
                tmp_h = ((uint8_t)rasb_GpsTokenizedData[1u][0u] - 0x30u) * 10u + ((uint8_t)rasb_GpsTokenizedData[1u][1u] - 0x30u);
         }
	else
	{
                uint16_t tmp_h;
                tmp_h = ((uint8_t)rasb_GpsTokenizedData[1u][0u] - 0x30u) * 10u + ((uint8_t)rasb_GpsTokenizedData[1u][1u] - 0x30u);
	}
        */
        
        
}



uint32_t get_GPS_Time(void)
{
        return GPS_timerValue;
}

/*************************************************************************************************/
/*!
* \brief
* Converts hex char to numeral hex. 
*/
static uint8_t convertCharToHex(char value)
{
	uint8_t retValue = 0u;
        
	if (value >= 'A' && value <= 'F')	
	{
		retValue = value - 'A' + 10u;
	}
	else if (value >= 'a' && value <= 'f')
	{
		retValue = value - 'a' + 10u;	
	}
	else if (value >= '0' && value <= '9')
	{
		retValue = value - '0';
	}
	else
	{
		retValue = 0xFFu;
	}
        
	return retValue;
}

/*************************************************************************************************/
/*!
* \brief
* Converts intput string to output tokens without excluding null strings.
*/
static uint8_t tokenize(char *input, char *output, unsigned char inc)
{
	char *p;
	char *last;
	unsigned char len = 0u;
	unsigned char cnt = 0u;
        
	/* initialize last */
	last = input;
        
	/* start tokenizing */
	for (p = strchr(input, ','); p; p = strchr(p + 1, ','))
	{
		/* when the ',' delimiter is found convert it to NULL = 0 */
		*p = 0;
		
		/* compute the string length */
		len = strlen(last);
		
		/* if the length is 0 overwrite it with 1 so that the memcpy will copy at least a 0 */
		if (len == 0u)
		{
			len = 1u;
		}
                
		/* copy to the output buffer */
		(void)memcpy(output, last, len);
                
		/* increment the output buffer location for the next copy */
		output += inc;
		cnt++;
		/* change the value back */
		*p = ',';
		/* update last's start position */
		last = p + 1;
	}
        
	cnt++;
	/* prepare for copying the last parsed string */
	len = strlen(last);
        
	if (len == 0u)
	{
		len = 1u;
	}
        
	(void)memcpy(output, last, len);
        
	return cnt;
}


int RTC_Set(
        uint8_t year, uint8_t month, uint8_t day,
        uint8_t hour, uint8_t min, uint8_t sec) {
    HAL_StatusTypeDef res;
    RTC_TimeTypeDef time;
    RTC_DateTypeDef date;
    
    uint8_t dow = 0;
                        
    dow = ComputeDayOfWeek(month, day, year);
                
    memset(&time, 0, sizeof(time));
    memset(&date, 0, sizeof(date));

    date.WeekDay = dow;
    date.Year = year;
    date.Month = month;
    date.Date = day;

    res = HAL_RTC_SetDate(&hrtc, &date, RTC_FORMAT_BIN);
    if(res != HAL_OK) {
        //UART_Printf("HAL_RTC_SetDate failed: %d\r\n", res);
        return -1;
    }

    time.Hours = hour;
    time.Minutes = min;
    time.Seconds = sec;

    res = HAL_RTC_SetTime(&hrtc, &time, RTC_FORMAT_BIN);
    if(res != HAL_OK) {
        //UART_Printf("HAL_RTC_SetTime failed: %d\r\n", res);
        return -2;
    }

    return 0;
}
        
 // Computes the Day of the week and returns a number representing it
 // Sunday = 1, Monday = 2, Tuesday = 3...
 uint8_t ComputeDayOfWeek(uint8_t month, uint8_t day, uint8_t year)
 {
  uint16_t day_of_week;
  
  if (month < 3)
  {
      month += 12;
      --year;
    }
   day_of_week = day + (13 * month - 27) / 5 + year + year/4 - year/100 + year/400;
   return ((uint8_t)(day_of_week % 7));
 }

uint8_t get_Year(void){
        return rs_GpsData.gpsGlobalDateTime.year;
 }
 
uint8_t get_Month(void){
         return rs_GpsData.gpsGlobalDateTime.month;
 }
uint8_t get_Day(void){
        return rs_GpsData.gpsGlobalDateTime.day;
}

uint8_t get_noSat(void){
        return rs_GpsData.numberOfSatellites;
}


uint8_t get_GPS_valid(void){
        return isGPSTimerValid;
}


