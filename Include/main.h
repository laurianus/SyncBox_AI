/**
  ******************************************************************************
  * @file    Audio/Audio_playback_and_record/Inc/main.h
  * @author  MCD Application Team
  * @brief   Header for main.c module.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2017 STMicroelectronics International N.V. 
  * All rights reserved.</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */ 
  
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

/* Includes ------------------------------------------------------------------*/
#include <stdio.h>
#include "stm32f4xx_hal.h"
#include "stm32f4_discovery.h"
#include "stm32f4_discovery_audio.h"
#include "stm32f4xx_it.h"
#include "waveplayer.h"
#include "ff.h"
#include "ff_gen_drv.h"
#include "usbh_diskio_dma.h"
#include "sd_diskio.h"

/* Compilation options -------------------------------------------------------*/
//#define CONFIG_CLOCK_INTERNAL     // used for initial testing - NOT WORKING with SD/USB
#define CONFIG_CLOCK_EXTERNAL

//#define CONFIG_TEST_USB
//#define CONFIG_TEST_SD
//#define CONFIG_TEST_SD2

#define CONFIG_PLAYER_USB
//#define CONFIG_PLAYER_SD


typedef enum
{
    APPLICATION_IDLE = 0,  
    APPLICATION_START,    
    APPLICATION_RUNNING,
} MSC_ApplicationTypeDef;

/* State Machine for the USBH_USR_ApplicationState */
enum
{
    USBH_USR_IDLE = 0,
    USBH_USR_FS_INIT,
    USBH_USR_AUDIO,
};

/* Defines for LEDs lighting */
#define LED3_TOGGLE      0x03  /* Toggle LED3 */
#define LED4_TOGGLE      0x04  /* Toggle LED4 */
#define LED6_TOGGLE      0x06  /* Toggle LED6 */
#define LEDS_OFF         0x07  /* Turn OFF all LEDs */
#define STOP_TOGGLE      0x00  /* Stop LED Toggling */

/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
void Error_Handler(void);


void SPI1_IRQHandler(void);
void MX_GPIO_Init_LCD(void);

void start_Buz_PWM(void);
void stop_Buz_PWM(void);


void MX_SPI2_Init(void);

void LCD_test32(void);

void init_SPI_16b(uint8_t dat_bit_size);

FRESULT Test_SD_Card(void);
void Test_SD_File(void);

void Init_USB_Drive(void);
void DeInit_USB_Drive(void);

FRESULT Test_USB_Drive(void);
void Test_USB_File(void);
void StartRTC_Routine(void);
void set_BackLight_kbd(uint8_t dev_state);

void set_lcd_BL(uint8_t percent);
uint8_t get_Is_seq_music(uint8_t pos);
uint16_t get_Key_gen(void);
uint16_t get_net_Adr_gen(void);
void rst_Time_Code_init(void);
void set_DMX_timer(uint16_t presc, uint16_t arr);
void set_rst_DMX_Line(uint8_t dmx_line);
void Disable_TC_SMPTE_Read_Pin(void);//Disable SMPTE Read pin
void Enable_TC_SMPTE_Read_Pin(void);//Enable SMPTE Read pin

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
