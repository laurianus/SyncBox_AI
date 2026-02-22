/***************************************************************************//**
 * @file     Display.h
 * @brief    DOGS 102x68 display driver header file
 ******************************************************************************/

#ifndef _BOARD_DISPLAY_H_
#define _BOARD_DISPLAY_H_

#include "stm32f4xx_hal.h"

/**
@fn void InitDisplay(uint32_t deviceAddress)
@param[in] deviceAddress Device address
@brief Initialize display. Call at the device start
*/
void InitDisplay(uint32_t deviceAddress);
void LCD_Initial(void);
/**
@fn void DoDisplayTasks(void)
@brief Display housekeeping tasks. Call periodicaly from the main loop.
*/
void DoDisplayTasks(void);

//Netw 01.07.2017
void updateTextFields3(void);


typedef enum _spiSendType{
	SPI_Command, /*< SPI transfer is a device command */
	SPI_Data, /*< SPI transfer is the device data */
}SpiSendType;

void SPI_Send(const uint8_t *data, uint32_t dataLength, SpiSendType sendType);
//void SPI_Send(uint8_t data, uint32_t dataLength, SpiSendType sendType);
void hw_reset_LCD(void);
void display_refresh(uint8_t refresh_all);
void reverse_display(uint8_t cur_lineX);
uint8_t reverseBits(uint8_t num);
void res_buf_line (uint8_t l_buf_t);
void disp_clear(void);
void update_Wireless_Icon(void);
void update_2Wire_Icon(void);
void update_Bat1_Icon(void);
void update_AB_Icon(void);
uint8_t get_center(uint8_t len_ccc);
void write_2lines_mid(uint8_t line_1); //centered
void write_2lines(uint8_t line_1);//not centered

void LCD_FailCable(void);

void Update_States(void);
void Updates_Player_Stat1(void);
void Updates_Player_Stat2(void);
void set_Long_refresh(uint8_t is_rst_LR);
void Updates_Player_Stat3(void);
void Updates_Player_StatTime(void);
void Wireless_RST_Wait(uint8_t asdfg);
void Wireless_Choose(void);
void Wireless_Restart(void);
void set_display_AB_ID_choose(void);

void set_LCD_Line(uint8_t tmp_trs);

void settings_display(void);
void settings_Main(void);
void set_display_Time_choose(void);
void set_display_Wireless_choose(void);
void set_display_Audio_choose(void);
void set_display_TimeCode_choose(void);
void set_display_TimeReal_choose(void);
void set_display_TimeStart_choose(void);
void set_display_TimeZone_choose(void);
void set_display_Default_choose(void);

void Get_MCU_time(void);
void calculate_time(uint32_t time_to_calc);
void timer_saving(uint8_t is_saving);

uint8_t get_menu_item(void);
void timer_Start_saving(uint8_t is_saving);
void set_display_Wireless_LF_Set(void);
void set_display_Wireless_HF_Set(void);
void go_Back_main(void);
void Wireless_reset_disp(void);

void Wireless_config_Update(void);
void update_Wir_2Wire_Icon(void);
uint8_t small_remote_display(void);
void USB_Drive_menu (uint8_t USB_Stick_menu);

#endif
