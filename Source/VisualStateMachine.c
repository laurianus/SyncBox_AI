/**************************************************************************//**
* @file     VisualStateMachine.c
* @brief    Core device operation functions
******************************************************************************/
#include <string.h>
#include <stdio.h>
#include "stm32f4xx_hal.h"


#include "options.h"
#include "Wireless.h"
#include "main.h"
#include "uart.h"


#ifndef VS_READABLE_CODE
#include "System1.h"
#include "System1Action.h"
#include "System1Data.h"
#endif

#include "simpleEventHandler.h"         		// Event Queue
#include "System1SEMLibB.h"                    	// visualSTATE essentials



