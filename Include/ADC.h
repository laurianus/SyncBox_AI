/***************************************************************************//**
 * @file     ADC.h
 * @brief    A/D converter (TLV1543) header file
 ******************************************************************************/
#ifndef _DEVICE_ADC_H_
#define _DEVICE_ADC_H_

#include "stm32f4xx_hal.h"
#include "stm32f4xx.h"





uint16_t readAdcA(uint8_t adcChannel);

void ADC_Task(void);

void ADC_refresh(void);
void do_ADC_Tasks(void);
void readInternalBattery(void);
uint16_t get_ADC_value(uint8_t chan);
#endif
