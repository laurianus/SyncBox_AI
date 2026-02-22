/***************************************************************************//**
* @file     ADC.c
* @brief    A/D converter (TLV1543) driver
******************************************************************************/
#include <string.h>
#include <stdio.h>
#include "ADC.h"
#include "main.h"


#include "options.h"
#include "Device.h"

#include "SystemTimer.h"

#ifndef VS_READABLE_CODE
#include "System1.h"
#include "System1Action.h"
#include "System1Data.h"
#endif

#include "simpleEventHandler.h"         		// Event Queue
#include "System1SEMLibB.h"                    	// visualSTATE essentials

#define SPI_BUFFER_SIZE 6
#define MAX_ADC_VALUES 7

extern SPI_HandleTypeDef hspi2;

volatile uint16_t ADC_values[MAX_ADC_VALUES + 1] = {0};
uint8_t ADC_val_Pos = 0;
float VBoutX = 0.0;



uint32_t tmp_adc_refrsh = 0;
uint32_t ADCUpdate_time = 50;

uint8_t is_SPI_ADC_Busy = 0;
uint8_t was_toggle = 0;


uint8_t Rx_Buf[SPI_BUFFER_SIZE];

/******************************************************************************
@fn uint32_t readAdc(uint32_t adcChannel)
@param[in] adcChannel Channel of the A/D converter to be read
@return A/D conversion value (readout)
@brief Read the value from the channel of the A/D converter
@details It is necessary to convert the number returned from this function to the
proper measurement value
******************************************************************************/
uint16_t readAdcA(uint8_t adcChannel){

      uint8_t Tx_Buf[SPI_BUFFER_SIZE];	

      
      is_SPI_ADC_Busy = 1;
      
     
        
      Tx_Buf[0] = (adcChannel & 0x0f) << 3;
      Tx_Buf[1] = (adcChannel & 0x0f) << 3;
      Tx_Buf[2] = (adcChannel & 0x0f) << 3;
      Tx_Buf[3] = (adcChannel & 0x0f) << 3;
      Tx_Buf[4] = (adcChannel & 0x0f) << 3;
      Tx_Buf[5] = (adcChannel & 0x0f) << 3;

        
        /*
      Tx_Buf[0] = (adcChannel & 0x0f) << 3;
      Tx_Buf[1] = (adcChannel & 0x0f) << 3;
  
*/
        if(was_toggle < 2)
        {
                ADCUpdate_time = ADC_UPDATE_TIME;
                HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_12);
                was_toggle++;
        }
       
        else if (was_toggle == 2)
        {
                  GPIO_InitTypeDef GPIO_InitStruct = {0};
                  GPIO_InitStruct.Pin = GPIO_PIN_12;
                  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
                  GPIO_InitStruct.Pull = GPIO_PULLUP;
                  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
                  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
                  ADCUpdate_time = 50 * ADC_UPDATE_TIME;
                  
                  was_toggle++;
                
                /*
                HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
         
                  GPIO_InitTypeDef GPIO_InitStruct = {0};
                  
                  GPIO_InitStruct.Pin = GPIO_PIN_12;
                  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
                  GPIO_InitStruct.Pull = GPIO_NOPULL;
                  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
                  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
                  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
                  was_toggle++;
                  ADCUpdate_time = 50 * ADC_UPDATE_TIME;
                */
        }
       
       
               
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);        
        /*
        HAL_SPI_TransmitReceive_IT(&hspi2, (uint8_t *)Tx_Buf, (uint8_t *)Rx_Buf, 6);
       */
        HAL_SPI_TransmitReceive(&hspi2, (uint8_t *)Tx_Buf, (uint8_t *)Rx_Buf, 2, 1000);
        HAL_SPI_TransmitReceive(&hspi2, (uint8_t *)Tx_Buf, (uint8_t *)Rx_Buf, 2, 1000);
        
        ADC_refresh(); 
        

        
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);

        
        
      return 0;
      



}


void ADC_Task(void)
{
        
       // ADC_values[ADC_val_Pos] = readAdcA(ADC_val_Pos);
        if(is_SPI_ADC_Busy == 0)
        {
                readAdcA(ADC_val_Pos);
                ADC_val_Pos++;
                if(ADC_val_Pos > MAX_ADC_VALUES)
                {
                        ADC_val_Pos = 0;
                }
        }
        else
        {
                is_SPI_ADC_Busy = 0;
        }

}

void ADC_refresh(void)
{
         uint16_t tmp = 0u;
              tmp = (Rx_Buf[3] << 8);
              tmp |= Rx_Buf[2];
              //tmp = tmp >> 2;
        
              ADC_values[ADC_val_Pos] = tmp;
              is_SPI_ADC_Busy = 0;
       
}

void do_ADC_Tasks(void)
{
        
        if(tmp_adc_refrsh == 0)
        {
                tmp_adc_refrsh = GetCurrentSystemTime();
        }
        else
        {
                
                if(GetCurrentSystemTime() - tmp_adc_refrsh > ADCUpdate_time)
                {
                        tmp_adc_refrsh = 0;
                        ADC_Task();
                        readInternalBattery();
                }
        }
}


void readInternalBattery(void)
 {
        uint16_t valueBBB = 0u;

         valueBBB = ADC_values[BATTERY_ADC_CH];

         VBoutX =  valueBBB * ADC_rez + VAL_BAT_LDO;
         VBoutX =  VBoutX * 0.965f;
         
         
         valueBBB = ADC_values[1];
         
         
     
       if (VBoutX <= Min_Battery) {
             get_device_datas()->internal_battery_status = 7; //No battery
       }
        else if (VBoutX <= BLVL0) {
             get_device_datas()->internal_battery_status = 0; //battery level 0
       }
       else if (VBoutX <= BLVL1) {
             get_device_datas()->internal_battery_status = 1;// battery level 1
       }
       
       else if (VBoutX <= BLVL2) {
             get_device_datas()->internal_battery_status = 2; // battery level 2
       }
       else if (VBoutX <= BLVL3) {
             get_device_datas()->internal_battery_status = 3; // battery level 3
       }
       
       else if (VBoutX <= BLVL4) {
             get_device_datas()->internal_battery_status = 4; // battery level 4
       }
       else if (VBoutX <= BLVL5) {
             get_device_datas()->internal_battery_status = 5; // battery level 5
       }
       else{
             
             get_device_datas()->internal_battery_status = 7; //No battery
       }
}
 
uint16_t get_ADC_value(uint8_t chan){
        
        return ADC_values[chan];

}
