/*============================================================================*/
/*                        OBJECT SPECIFICATION                                */
/*============================================================================*
* $Source: GlobalPositioning.h $
* $Revision: 1.1 $
* Author: Alexandru Modoranu
* $Date: 2014/08/26 18:10:52CEST $
*/
/*============================================================================*/
/* FUNCTION COMMENT :                                                         */
/*!
* \file GlobalPositioning.h
* \brief Interface declarations for the GPS module.
*/
/*============================================================================*/
/* COPYRIGHT (C) ALEXANDRU MODORANU 2014                                      */
/* ALL RIGHTS RESERVED                                                        */
/*============================================================================*/
/*                               OBJECT HISTORY                               */
/*============================================================================*/
/*  REVISION |   DATE      |                               |      AUTHOR      */
/*----------------------------------------------------------------------------*/
/* $Log: GlobalPositioning.h $ */

#ifndef _GLOBAL_POSITIONING_H_
#define _GLOBAL_POSITIONING_H_

#include "stm32f4xx_hal.h"

typedef struct 
{
	uint16_t year		 :12;	/* max value = 4095 */
	uint16_t month		 :4;	/* max value = 15 - max used = 12 */

	uint16_t day	 	 :5;	/* max value = 31 - max used = 31 */
	uint16_t hour	 	 :5;	/* max value = 31 - max used = 24 */
	uint16_t minute 	 :6;	/* max value = 64 - max used = 60 */

	uint16_t second 	 :6;	/* max value = 64 - max used = 60 */
	uint16_t milliSecond :10;	/* max value = 1024 - max used = 1000 */
} S_GpsGlobalDateTime;

typedef struct 
{
	uint16_t unused      :5;
	uint16_t hour	 	 :5;	/* max value = 31 - max used = 24 */
	uint16_t minute 	 :6;	/* max value = 64 - max used = 60 */

	uint16_t second 	 :6;	/* max value = 64 - max used = 60 */
	uint16_t milliSecond :10;	/* max value = 1024 - max used = 1000 */
} S_GpsGlobalTime;

typedef struct 
{
	uint16_t year		 :12;	/* max value = 4095 */
	uint16_t month		 :4;	/* max value = 15 - max used = 12 */
	uint16_t day	 	 :5;	/* max value = 31 - max used = 31 */
	uint16_t unused      :11;
} S_GpsGlobalDate;


extern void gps_task(void);

extern S_GpsGlobalDateTime gps_getGlobalDateTime(void);
extern S_GpsGlobalDate gps_getGlobalDate(void);
extern S_GpsGlobalTime gps_getGlobalTime(void);
extern uint8_t gps_getNumberOfSatellites(void);

void gps_init(void);

int RTC_Set(
        uint8_t year, uint8_t month, uint8_t day,
        uint8_t hour, uint8_t min, uint8_t sec);

uint8_t ComputeDayOfWeek(uint8_t month, uint8_t day, uint8_t year);

uint8_t get_Year(void);
uint8_t get_Month(void);
uint8_t get_Day(void);
uint32_t get_GPS_Time(void);
uint8_t get_noSat(void);
uint8_t get_GPS_valid(void);        

void run_pps_function(void);
void set_GPS_time(void);

#endif
