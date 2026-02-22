/***************************************************************************//**
* @file     Display.c
* @brief    DOGS 102x68 display driver
******************************************************************************/

#include <string.h>
#include <stdio.h>

#include "Images.h"
#include "font_8px.h"
#include "Device.h"
#include "SystemTimer.h"
#include "waveplayer.h"
#include "sht3x.h"
#include "Display.h"
#include "uart.h"
#include "TimeCode.h"
#include "Wireless.h"

#include "GlobalPositioning.h"

#include "flash_if.h"

#include "stm32f4xx_hal_spi.h"

#ifndef VS_READABLE_CODE
#include "System1.h"
#include "System1Action.h"
#include "System1Data.h"
#endif

#include "simpleEventHandler.h"         // TEent Queue
#include "System1SEMLibB.h"                    // visualSTATE essentials

extern uint8_t is_init_done;
extern uint8_t Network_found;
extern uint8_t is_CopyMenu;

extern uint8_t isGPSTimerValid;
extern uint8_t isGPSFTValid;

extern uint32_t Line_read;
extern uint16_t DMX_ev;

extern uint8_t Time_code_Started;

extern uint8_t SB_AsMaster;


extern uint8_t Time_Code_disable;

extern uint8_t is_Wir_Slot_Detect;

extern uint8_t was_WirConfigRead;
extern uint8_t is_433Mhz_en;

extern uint32_t GPS_SetValue;

extern uint8_t USB_Stick_menu;

uint32_t ID_MCU2 = 0;

uint16_t Time_Blink_timer = 0;
uint8_t Cur_Time_pos = 0;

uint32_t timer_tmp = 0;
extern uint8_t failed_gps_up;

extern uint8_t isGPSTimerCTRL;
extern uint8_t GPS_Fixed;

uint16_t UhoursSMPTE = 0u;
uint16_t UminutesSMPTE = 0u;
uint16_t USecSMPTE = 0u;

#define WIRELESS_ICON_HORIZONTAL_OFFSETa (4)
#define WIRELESS_ICON_HORIZONTAL_OFFSETx (30)
char tempText3A[5] = {0};


const uint8_t ERR_RFRFSH = 80;
const uint8_t ERR_RFRFSH_MAX = 100;


extern uint8_t real_time_valid;

int8_t cure_val_saved = 1;      

extern uint8_t set_AD_ID;

extern uint16_t PPS_run_XXX;


extern SPI_HandleTypeDef hspi1;
extern uint8_t main_volume;

extern uint8_t is_wireless_ok;
extern uint8_t wirelessCustom;

extern uint32_t isLongBPress;
extern uint8_t PPS_Mp3_AB; 

int hhXXX = 0;
int mmXXX = 0;
int ssXXX = 0;
        
    

uint8_t butD_stat_aaa = 0;
uint8_t is_butXaDS_valid = 0;

uint8_t menu_item = 0;

uint32_t saved_Stat = 0;
uint32_t TimerStatAP = 0;
                             
extern uint16_t sync_times;

extern uint64_t sort_percent;
extern uint64_t TransferRate_USD;
extern uint64_t Remain_Copy_time;

extern uint32_t countLastTime1;

uint32_t tmp_disp_refrsh = 0;

uint32_t tmp_disp_up = 0;



int drift_AB_T = 0;

uint16_t test_xxx_AB = 0;



extern uint8_t is_power_on_enable;

uint8_t is_Wireless_res = 0;

extern char Network_Name[6];






#define BATTERY_ERR_REFRESH_MIN 25
#define BATTERY_ERR_REFRESH_MAX 35



#define TIME_CODE_BLINK_MIN 25
#define TIME_CODE_BLINK_MAX 35




//extern float VBoutX;

/**
@def DISPLAY_NUMBER_OF_COLUMNS
@brief Number of columns in one row (X-pixel row height)
*/
#define DISPLAY_NUMBER_OF_COLUMNS (128)

/**
@def DISPLAY_NUMBER_OF_PAGES
@brief Number of X-pixel display rows
*/
#define DISPLAY_NUMBER_OF_PAGES (8)

/**
@def SIZE_OF_DISPLAY_ROW
@brief Size of display row (in pixels)
*/
#define SIZE_OF_DISPLAY_ROW (8)

/**
@def MAX_TEXT_LENGTH
@brief Maximum number of alphanumerical characters in one row
*/
#define MAX_TEXT_LENGTH (21)

/**
@def TEXT_OFFSET
@brief Offset (in columns) of the text from the left display border
*/
#define TEXT_OFFSET (3)

/**
@def TEXT_LINE_DEVICE
@brief Row (memory page) position of the device address and operation mode data
text line
*/
#define TEXT_LINE_DEVICE (3)

/**
@def TEXT_LINE_STATUS
@brief Row (memory page) position of the device status text line
*/
#define TEXT_LINE_STATUS (5)

/**
@def TEXT_LINE_ERROR
@brief Row (memory page) position of the error text line
*/
#define TEXT_LINE_ERROR (7)

/**
@def WIRELESS_ICON_HORIZONTAL_OFFSET
@brief Offset (number of columns) of the wireless signal level icon from the
left display border
*/
#define WIRELESS_ICON_HORIZONTAL_OFFSET (0)

/**
@def CAN_ICON_HORIZONTAL_OFFSET
@brief Offset (number of columns) of the CAN-bus connected status  icon from the
left display border
*/
#define CAN_ICON_HORIZONTAL_OFFSET (31)

#define TC_OFFSET (44)
#define PLAYER_OFFSET (71)


/**
@def BATTERY_1_HORIZONTAL_OFFSET
@brief Offset (number of columns) of the battery 1 status icon from the
left display border
*/
#define BATTERY_1_HORIZONTAL_OFFSET (57)

/**
@def BATTERY_2_HORIZONTAL_OFFSET
@brief Offset (number of columns) of the battery 2 status icon from the
left display border
*/
#define BATTERY_2_HORIZONTAL_OFFSET (74)


/**
@def LCD_PAGE_ADDRESS
@brief Display page address set (lower 4 bits select one of 8 pages)
*/
#define LCD_PAGE_ADDRESS     0xB0


/**
@def LCD_COL_ADDRESS_MSB
@brief column address (lower 4 bits are upper / lower nibble of column address)
*/
#define LCD_COL_ADDRESS_MSB  0x10

/**
@def LCD_COL_ADDRESS_LSB
@brief second part of column address
*/
#define LCD_COL_ADDRESS_LSB  0x00


/**
@var uint32_t bufDeviceAddress
@brief Device address (set at the display initialisation)
*/

uint32_t EXT_Time = 0;
extern uint8_t TC_fps_setX;
uint32_t Music_time = 0;
uint32_t Remain_time = 0;
extern uint8_t file_read_error;


static void Display_Text(void);
static void Update_Icons(void);

uint8_t current_disp_line = 0;

uint8_t Err_refresh = 0;
uint16_t Alt_refresh = 0;
uint8_t Fst_refresh = 0;
uint16_t Long_refresh = 0;

extern uint16_t Temp_Hum_refresh;

char tempText[24] = {0};
char tempTextU[21] = {0};
char bufferDisp[21] = {0};

uint16_t is_blinkTimer_TC = 0;


//uint8_t LCD_Line_refresh = 0;



void Initial_Dispay_Line(unsigned char line);
void Set_Page_Address(unsigned char add);
void Set_Column_Address(unsigned char add);
void Power_Control(unsigned char vol);
void Regulor_Resistor_Select(unsigned char r);
void Set_Contrast_Control_Register(unsigned char mod);
void Lcd_Reset(void);
void Lcd_Clear_Adc(void);
void Lcd_Set_Shl(void);
void Lcd_Clear_Bias(void);
void Lcd_Display_On(void);
void Lcd__Entire_Display_On(void);
void Lcd__Entire_Display_Off(void);
void Lcd_Display_Off(void);
static void Update_Copyto_SD(void);




//static uint8_t MatrixSwitchxx;
//static uint8_t matrix_change = 1;

/**
@var uint8_t displayBuffer[DISPLAY_NUMBER_OF_PAGES][DISPLAY_NUMBER_OF_COLUMNS]
@brief Buffer (can be described also as a framebuffer) containing the data to be
sent to the display.
@details Contains the complete image to be shown on the display
*/
uint8_t displayBuffer[DISPLAY_NUMBER_OF_PAGES][DISPLAY_NUMBER_OF_COLUMNS];
uint8_t displayBufferX[DISPLAY_NUMBER_OF_PAGES][DISPLAY_NUMBER_OF_COLUMNS];// for tests

uint8_t displayBufferT[DISPLAY_NUMBER_OF_COLUMNS];
uint8_t displayBufferR[DISPLAY_NUMBER_OF_COLUMNS];

/*****************************************************************************
//jcm-Complete
@fn static void //sendToDisplay(uint32_t column, uint32_t page, const uint8_t *data,
uint32_t dataLength)
@param[in] column Address of the display write start column
@param[in] page Address of the display write start page
@param[in]  data Pointer to the payload that is to be sent
@param[in] dataLength Length of the payload
@brief Send the data (or the command) to the display over the SPI connection
*******************************************************************************/
static void sendToDisplay(uint32_t column, uint32_t page, const uint8_t *data,
                          uint32_t dataLength){
                                
                                
                                uint8_t cmdData[3];
                                
                                if((dataLength-column>DISPLAY_NUMBER_OF_COLUMNS) ||
                                   ((DISPLAY_NUMBER_OF_COLUMNS-column)<dataLength)){
                                         return;
                                   }
                                
                                cmdData[0]=(uint8_t)(LCD_PAGE_ADDRESS|(page&0x0F));
                                cmdData[1]=(uint8_t)(LCD_COL_ADDRESS_MSB|((column>>4)&0x0F));
                                cmdData[2]=(uint8_t)(LCD_COL_ADDRESS_LSB|(column&0x0F));	
                                SPI_Send(cmdData, 3, SPI_Command);
                                
                                SPI_Send(data, dataLength, SPI_Data);	
                              
                          }

/***************************************************************************//**
@fn void writeTextToFrameBuffer(uint32_t column, uint32_t page, char *text,
uint32_t textLength)
@param[in] column Address of the display write start column
@param[in] page Address of the display write start page
@param[in] text Pointer to the text that is to be sent
@param[in] textLength Length of the text
@brief Write a text to the framebuffer, but do not send it to the display
*******************************************************************************/
void writeTextToFrameBuffer(uint32_t column, uint32_t page, char *text,
                            uint32_t textLength){
                                  uint32_t n=0;
                                  uint8_t *p;
                                  
                                  if(textLength>MAX_TEXT_LENGTH){
                                        return;
                                  }
                                  
                                  if((column +(textLength*font_fixed_8px_width))>DISPLAY_NUMBER_OF_COLUMNS){
                                        return;
                                  }
                                  
                                  p=displayBuffer[page];
                                  
                                  for(n=0; n<textLength; n++){
                                        memcpy(p+column, font_fixed_8px_data+(text[n]*font_fixed_8px_width), font_fixed_8px_width);
                                        p+=6;
                                  }
                                  
                            }

/***************************************************************************//**
@fn void writeImageToFrameBuffer(uint32_t column, uint32_t page,
const uint8_t *image, uint32_t imageLength,
uint32_t width, uint32_t height)
@param[in] column Address of the display write start column
@param[in] page Address of the display write start page
@param[in] image Pointer to the bitmap that is to be sent
@param[in] imageLength Size (sizeof) of the bitmap
@param[in] width Bitmap width
@param[in] height Bitmap height
@brief Write a bitmap to the framebuffer but do not send it to the display
*******************************************************************************/
static void writeImageToFrameBuffer(uint32_t column, uint32_t page,
                             const uint8_t *image, uint32_t imageLength,
                             uint32_t width, uint32_t height){

                                     
                                                                        uint32_t numberOfRows=0u;
                                   uint32_t n=0;
                                   uint32_t positionCounter=0u;
                                   if(height>SIZE_OF_DISPLAY_ROW){
                                         numberOfRows=height/SIZE_OF_DISPLAY_ROW;
                                         if((height%SIZE_OF_DISPLAY_ROW)!=0){
                                               numberOfRows+=1;
                                         }
                                   }else{
                                         numberOfRows=1;
                                   }
                                   
                                   //project specific
                                   if(numberOfRows>2){
                                         return;
                                   }
                                   
                                   if((page+numberOfRows)>DISPLAY_NUMBER_OF_PAGES){
                                         return;
                                   }
                                   
                                   if(DISPLAY_NUMBER_OF_COLUMNS<(column+width)){
                                         return;
                                   }
                                   
                                   //project specific
                                   for(n=0;n<imageLength;n++){
                                         if((n%2)==0){
                                               displayBuffer[page][positionCounter+column]=image[n];			
                                         }else{
                                               displayBuffer[page+1][positionCounter+column]=image[n];
                                               positionCounter++;
                                         }		
                                   }

                     }
                             

// Power_Control   4 (internal converte ON) + 2 (internal regulor ON) + 1 (internal follower ON)
void Power_Control(unsigned char vol)
 {
       uint8_t cmdData[1];
       cmdData[0] = 0x28|vol;
       SPI_Send(cmdData, 1, SPI_Command);
       return;
 }

//  Regulor resistor select
//**            1+Rb/Ra  Vo=(1+Rb/Ra)Vev    Vev=(1-(63-a)/162)Vref   2.1v
//**            0  3.0       4  5.0(default)
//**            1  3.5       5  5.5
//**            2  4         6  6
//**            3  4.5       7  6.4

void Regulor_Resistor_Select(unsigned char r)
 {
       uint8_t cmdData[1];
       cmdData[0] = 0x20|r;
       SPI_Send(cmdData, 1, SPI_Command);
       return;
 }

//a(0-63) 32default   Vev=(1-(63-a)/162)Vref   2.1v
void Set_Contrast_Control_Register(unsigned char mod)
 {
       uint8_t cmdData[1];
       cmdData[0] = 0x81;
       SPI_Send(cmdData, 1, SPI_Command);
       cmdData[0] = mod;
       SPI_Send(cmdData, 1, SPI_Command);
       return;
 }

//Specify DDRAM line for COM0 0~63
void Initial_Dispay_Line(unsigned char line)
 {
       uint8_t cmdData[1];
       line|=0x40;
       cmdData[0] = line;
       SPI_Send(cmdData, 1, SPI_Command);
       return;
 }

void  Lcd_Reset(void)
 {
       uint8_t cmdData[1];
       cmdData[0] = 0xe2;
       SPI_Send(cmdData, 1, SPI_Command);
 }

void Lcd_Clear_Adc(void)
 {
       uint8_t cmdData[1];
       cmdData[0] = 0xa0;
       SPI_Send(cmdData, 1, SPI_Command);
 }

void Lcd_Set_Shl(void)
 {
       uint8_t cmdData[1];
       cmdData[0] = 0xc8;
       SPI_Send(cmdData, 1, SPI_Command);
 }

void Lcd_Clear_Bias(void)
 {
       uint8_t cmdData[1];
       cmdData[0] = 0xa2;
       SPI_Send(cmdData, 1, SPI_Command);
 }

void Lcd_Display_On(void)
 {
       uint8_t cmdData[1];
       cmdData[0] = 0xaf;
       SPI_Send(cmdData, 1, SPI_Command);
 }

void Lcd__Entire_Display_On(void)
 {
       uint8_t cmdData[1];
       cmdData[0] = 0xa5;
       SPI_Send(cmdData, 1, SPI_Command);
 }

void Lcd__Entire_Display_Off(void)
 {
       uint8_t cmdData[1];
       cmdData[0] = 0xa4;
       SPI_Send(cmdData, 1, SPI_Command);
 }
 
 void Lcd__Entire_Display_Invert(void)
 {
       uint8_t cmdData[1];
       cmdData[0] = 0xa6;
       SPI_Send(cmdData, 1, SPI_Command);
 }
 
 void Lcd__Entire_Display_Nor(void)
 {
       uint8_t cmdData[1];
       cmdData[0] = 0xa7;
       SPI_Send(cmdData, 1, SPI_Command);
 }

  void Lcd_Display_Onn(void)
 {
       uint8_t cmdData[1];
       cmdData[0] = 0xaf;
       SPI_Send(cmdData, 1, SPI_Command);
 }


VS_VOID rst_Display_M()
 {
       
 }
 
 static void ClearDisplay(void){
      
      uint32_t n=0;
      memset(displayBuffer, 0x00, sizeof(displayBuffer));
      for(n=0; n<DISPLAY_NUMBER_OF_PAGES;n++){
            sendToDisplay(0, n, displayBuffer[n], DISPLAY_NUMBER_OF_COLUMNS);
      }
      
      //Lcd__Entire_Display_Off();
}
 

/*
    RESET();
	RST=1;
	Delay(2000);
	RST=0;
	Delay(2000);
	RST=1;

	Delay(10000);
	CS1=0;
    CLEAR_ADC();
    SET_SHL();
    CLEAR_BIAS();
    Power_Control(0x07);
    Regulor_Resistor_Select(0x07);
    Set_Contrast_Control_Register(Contrast_level);
	Initial_Dispay_Line(0x00);
	DISPLAY_ON();
*/



void LCD_Initial(void)
 {
       Lcd_Reset();
       hw_reset_LCD();
       Delay(1);
       Lcd_Clear_Adc();
       Lcd_Set_Shl();
       Lcd_Clear_Bias();
       Power_Control(0x07);         
       Regulor_Resistor_Select(0x07); 
       Set_Contrast_Control_Register(32);   
       Lcd_Display_On();
         
       
       ClearDisplay();
         
       Lcd__Entire_Display_On();
         
       Lcd__Entire_Display_Off();
       
 
       sprintf(tempText, "%s", "SyncB");
       
       
       writeTextToFrameBuffer(91, 0, tempText,6);
       
       for(int x = 91; x< 128; x++)
        {
              
              displayBufferT[x] = displayBuffer[0][x];
              displayBuffer[0][x] = displayBufferT[x] <<4;
              displayBuffer[1][x] = displayBufferT[x] >>4;
        }
        
          memset(tempText, 0x00, sizeof(tempText));	
                    sprintf(tempText, "%s", "ID");
                    
                    writeTextToFrameBuffer(PLAYER_OFFSET, 0, tempText, 2);
    
               
                    memset(tempText, 0x00, sizeof(tempText));		
	
                    
                    sprintf(tempText, "%s", "xx");
                    
                    writeTextToFrameBuffer(PLAYER_OFFSET, 1, tempText,2);
                    


     
      
      
       writeImageToFrameBuffer(WIRELESS_ICON_HORIZONTAL_OFFSET, 0, WirelessSignal_NC_bmp, (uint32_t)sizeof(WirelessSignal_NC_bmp),
                               WirelessSignal_bmp_width, WirelessSignal_bmp_height);
       

          sprintf(tempText, " TC ");
          sprintf(tempTextU, " xx ");
        
           writeTextToFrameBuffer(TC_OFFSET, 0, tempText, 4);    
           writeTextToFrameBuffer(TC_OFFSET, 1, tempTextU, 4);    
         
       
       writeImageToFrameBuffer(BATTERY_1_HORIZONTAL_OFFSET, 0, NoBattery_bmp, (uint32_t)sizeof(NoBattery_bmp),
                               Battery_bmp_width, Battery_bmp_height);
      


       sendToDisplay(0, 0, displayBuffer[0], DISPLAY_NUMBER_OF_COLUMNS);
       sendToDisplay(0, 1, displayBuffer[1], DISPLAY_NUMBER_OF_COLUMNS);

       memset(tempTextU, 0x00, sizeof(tempTextU));       
       sprintf(tempTextU, "%s", "fireTEK SyncBox");
       write_2lines_mid(2);
       
       memset(tempTextU, 0x00, sizeof(tempTextU));       
       sprintf(tempTextU, "Firmware: %s", FW_VER);
       write_2lines_mid(4);
       
       uint32_t ID_MCU = HAL_GetUIDw1();
       ID_MCU2 = HAL_GetUIDw0();
       
       memset(tempTextU, 0x00, sizeof(tempTextU));       
       sprintf(tempTextU, "SN:%X-%X", ID_MCU, ID_MCU2);
       
       
       write_2lines_mid(6);
       
       sendToDisplay(0,2,displayBuffer[2], DISPLAY_NUMBER_OF_COLUMNS);
       sendToDisplay(0,3,displayBuffer[3], DISPLAY_NUMBER_OF_COLUMNS);
       sendToDisplay(0,4,displayBuffer[4], DISPLAY_NUMBER_OF_COLUMNS);
       sendToDisplay(0,5,displayBuffer[5], DISPLAY_NUMBER_OF_COLUMNS);
       sendToDisplay(0,6,displayBuffer[6], DISPLAY_NUMBER_OF_COLUMNS);
       sendToDisplay(0,7,displayBuffer[7], DISPLAY_NUMBER_OF_COLUMNS);
}     



void res_buf_line (uint8_t l_buf_t)
 {
       memset(displayBuffer[l_buf_t], 0x00, sizeof(displayBuffer[l_buf_t]));
 }






void DoDisplayTasks(void)
{

        if(tmp_disp_refrsh == 0)
        {
                tmp_disp_refrsh = GetCurrentSystemTime();
        }
        else
        {
                if(GetCurrentSystemTime() - tmp_disp_refrsh > 10)
                {
                        tmp_disp_refrsh = 0;
                        display_refresh(0);
                }
        }

        
        if(tmp_disp_up == 0)
        {
                tmp_disp_up = GetCurrentSystemTime();
        }
        else
        {
                if(GetCurrentSystemTime() - tmp_disp_up > DISPLAY_UPDATE_TIME)
                {
                        tmp_disp_up = 0;
                            Err_refresh++;
                            Fst_refresh++;
                            Alt_refresh++;
                            Long_refresh++;
                        
                            Update_Icons();
                            Display_Text();
                        
                        if(Err_refresh > BATTERY_ERR_REFRESH_MAX)
                        {
                                Err_refresh = 0;
                        }
                        if(Fst_refresh > 100)
                        {
                                Fst_refresh = 0;
                        }
                        if(Alt_refresh > 100)
                        {
                                Alt_refresh = 0;
                        }
                        if(Long_refresh > 300)
                        {
                                Long_refresh = 0;
                        }
                        
                       /*
                        if(Temp_Hum_refresh > 200)
                        {
                                Temp_Hum_refresh = 0;
                                read_T_H();
                        }
                        else
                        {
                                Temp_Hum_refresh++;
                        }
                       */
                }
        }
}


uint8_t reverseBits(uint8_t num) 
 { 
       uint8_t count = sizeof(num) * 8 - 1; 
       uint8_t reverse_num = num; 
       
       num >>= 1;  
       while(num) 
        { 
              reverse_num <<= 1;        
              reverse_num |= num & 1; 
              num >>= 1; 
              count--; 
        } 
       reverse_num <<= count; 
       return reverse_num; 
 } 

void reverse_display(uint8_t cur_lineX)
 {
       
       for (int i = 0; i < DISPLAY_NUMBER_OF_COLUMNS; i++) 
        {
              
              displayBufferR[i] = reverseBits(displayBuffer[cur_lineX][DISPLAY_NUMBER_OF_COLUMNS - i]);
        }
 }


void display_refresh(uint8_t refresh_all)
 {
       
//       uint8_t was_LCD_chg = 0;
       static uint8_t Line_RefreshX = 0;
         
       
       if(refresh_all == 0)
        {
                
                sendToDisplay(0, Line_RefreshX, displayBuffer[Line_RefreshX], DISPLAY_NUMBER_OF_COLUMNS);  
                Line_RefreshX++;
                     
                     if (Line_RefreshX > 7)
                      {
                            Line_RefreshX = 0;
                            
                      }
      
                
       /*
                
              do
               {
                     for(int m = 0; m < DISPLAY_NUMBER_OF_COLUMNS; m++)
                      {
                            if(displayBufferX[Line_RefreshX][m] != displayBuffer[Line_RefreshX][m])
                             {
                                   was_LCD_chg = 1;
                                   break;
                             }
                            
                      }
                     
                     if(was_LCD_chg == 1)
                      {
                            if(get_rev_display() == 0)
                             {
                                   sendToDisplay(0, Line_RefreshX, displayBuffer[Line_RefreshX], DISPLAY_NUMBER_OF_COLUMNS);  
                                   memcpy(displayBufferX[Line_RefreshX],displayBuffer[Line_RefreshX], DISPLAY_NUMBER_OF_COLUMNS);
                             }
                            else
                             {
                                   reverse_display(Line_RefreshX);
                                   sendToDisplay(0, DISPLAY_NUMBER_OF_PAGES - 1 - Line_RefreshX, displayBufferR, DISPLAY_NUMBER_OF_COLUMNS);    
                                   memcpy(displayBufferX[Line_RefreshX],displayBuffer[Line_RefreshX], DISPLAY_NUMBER_OF_COLUMNS);
                             }
                            was_LCD_chg = 2;
                      }
                     
                     Line_RefreshX++;
                     
                     if (Line_RefreshX > 7)
                      {
                            Line_RefreshX = 0;
                            was_LCD_chg = 2;
                      }
               } while(was_LCD_chg != 2);
               */
              
              
        }
       else
        {
              for(int a = 0; a < DISPLAY_NUMBER_OF_PAGES; a++)
               {
                     if(get_rev_display() == 1)
                      {
                            sendToDisplay(0, a, displayBuffer[a], DISPLAY_NUMBER_OF_COLUMNS);  
                            memcpy(displayBufferX[a],displayBuffer[a], DISPLAY_NUMBER_OF_COLUMNS);
                      }
                     else
                      {
                            reverse_display(a);
                            sendToDisplay(0, DISPLAY_NUMBER_OF_PAGES - 1 - a, displayBufferR, DISPLAY_NUMBER_OF_COLUMNS);    
                            memcpy(displayBufferX[a],displayBuffer[a], DISPLAY_NUMBER_OF_COLUMNS);
                      }
               }
              
        }
 }

 
 
 
void hw_reset_LCD(void)
{
        
        HAL_GPIO_WritePin(LCD_RST_GPIO_Port, LCD_RST_Pin, GPIO_PIN_RESET);
        Delay(1);
        HAL_GPIO_WritePin(LCD_RST_GPIO_Port, LCD_RST_Pin, GPIO_PIN_SET);
        Delay(1);
        HAL_GPIO_WritePin(LCD_RST_GPIO_Port, LCD_RST_Pin, GPIO_PIN_RESET);
        Delay(1);
        HAL_GPIO_WritePin(LCD_RST_GPIO_Port, LCD_RST_Pin, GPIO_PIN_SET);
}


void disp_clear(void)
 {
       
       for(int a = 0; a < DISPLAY_NUMBER_OF_PAGES; a++)
        {
              for(int m = 0; m < DISPLAY_NUMBER_OF_COLUMNS; m++)
               {
                     displayBufferX[a][m] = 0xFF;
                     displayBuffer[a][m] = 0x00;
               }
              
        }       
       
 }


 void SPI_Send(const uint8_t *data, uint32_t dataLength, SpiSendType sendType)
{
        
	//HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
        
        	if(sendType == SPI_Command){
                        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_RESET);
                }else{
                        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_SET);     
                }
                

         
         HAL_SPI_Transmit_IT(&hspi1, (uint8_t *)data, dataLength);
         
         //HAL_SPI_Transmit_DMA(&hspi1, (uint8_t *)data, dataLength);
}
 
 
 /*
void SPI_Send(const uint8_t *data, uint32_t dataLength, SpiSendType sendType)
{
        
	//HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
        
        	if(sendType == SPI_Command){
                        HAL_GPIO_WritePin(LCD_A0_GPIO_Port, LCD_A0_Pin, GPIO_PIN_RESET);
                }else{
                        HAL_GPIO_WritePin(LCD_A0_GPIO_Port, LCD_A0_Pin, GPIO_PIN_SET);     
                }
                

         
         HAL_SPI_Transmit_IT(&hspi1, (uint8_t *)data, dataLength);
         
         //HAL_SPI_Transmit_DMA(&hspi1, (uint8_t *)data, dataLength);
}
*/


static void Display_Text(void)
{
        memset(displayBuffer[2], 0x00, sizeof(displayBuffer[2]));
        memset(displayBuffer[3], 0x00, sizeof(displayBuffer[3]));
        memset(displayBuffer[4], 0x00, sizeof(displayBuffer[4]));
        memset(displayBuffer[5], 0x00, sizeof(displayBuffer[5]));
        memset(displayBuffer[6], 0x00, sizeof(displayBuffer[6]));
        memset(displayBuffer[7], 0x00, sizeof(displayBuffer[7]));

        if(is_Wireless_res == 1)//choose
        {
                Wireless_Choose();
        }
        else if(is_Wireless_res == 2 || is_Wireless_res == 3)// wait
        {
                Wireless_RST_Wait(0);
        }
        else if(is_Wireless_res == 4)//need restart
        {
               Wireless_RST_Wait(1);
        }
        else if(is_CopyMenu > 0 && is_CopyMenu < 10)        
        {
                Update_Copyto_SD();
        }
        else
        {
                Update_States();
                Updates_Player_StatTime();
                Updates_Player_Stat2();
        }

}

static void Update_Icons(void)
{
        memset(displayBuffer[0], 0x00, sizeof(displayBuffer[0]));
        memset(displayBuffer[1], 0x00, sizeof(displayBuffer[1]));
//        update_Wireless_Icon();
        // update_2Wire_Icon();
        update_Wir_2Wire_Icon();
       
        update_Bat1_Icon();
        update_AB_Icon();
}


static void Update_Copyto_SD(void)
{
        uint8_t buf_xxx = 0;
        
       if(is_CopyMenu == 1)//choose if to copy or no
       {
               memset(tempTextU, 0x00, sizeof(tempTextU));
               sprintf(tempTextU, "USB Drive Detected");  
               write_2lines_mid(2);
               
               memset(tempTextU, 0x00, sizeof(tempTextU));
                sprintf(tempTextU, "Please choose:");         
               buf_xxx = get_center((uint8_t)strlen(tempTextU));
               writeTextToFrameBuffer(buf_xxx, 4, tempTextU, (uint32_t)strlen(tempTextU));        
                
               memset(tempTextU, 0x00, sizeof(tempTextU));
               sprintf(tempTextU, "<Copy USB to AudioBox");         
               write_2lines_mid(5);
                
               memset(tempTextU, 0x00, sizeof(tempTextU));
               sprintf(tempTextU, "      Use USB Drive >");         
               buf_xxx = get_center((uint8_t)strlen(tempTextU));
               writeTextToFrameBuffer(buf_xxx, 7, tempTextU, (uint32_t)strlen(tempTextU));        
       }
       else if(is_CopyMenu == 2) //Copying
       {
               memset(tempTextU, 0x00, sizeof(tempTextU));
               sprintf(tempTextU, "Copying. Please wait");  
               write_2lines_mid(2);
               
               memset(tempTextU, 0x00, sizeof(tempTextU));
               sprintf(tempTextU, "%d%% Complete",(uint32_t)(sort_percent));         
               buf_xxx = get_center((uint8_t)strlen(tempTextU));
               writeTextToFrameBuffer(buf_xxx, 4, tempTextU, (uint32_t)strlen(tempTextU));        
                
               memset(tempTextU, 0x00, sizeof(tempTextU));
               sprintf(tempTextU, "Speed: %dKBs", (uint32_t)(TransferRate_USD));         
               write_2lines_mid(5);
                
               memset(tempTextU, 0x00, sizeof(tempTextU));
               int min_tmp =  ((uint32_t)(Remain_Copy_time)/60);
               int sec_tmp =  ((uint32_t)(Remain_Copy_time)%60);
               
               if(sec_tmp < 10)
               {
                        sprintf(tempTextU, "Remaining: %d:0%ds", min_tmp, sec_tmp);
               }
               else
               {
                        sprintf(tempTextU, "Remaining: %d:%ds", min_tmp, sec_tmp);
               }
               buf_xxx = get_center((uint8_t)strlen(tempTextU));
               writeTextToFrameBuffer(buf_xxx, 7, tempTextU, (uint32_t)strlen(tempTextU));        
       }
       else if(is_CopyMenu == 3) //Complete copying
       {
               memset(tempTextU, 0x00, sizeof(tempTextU));
               sprintf(tempTextU, "Transfer complete");  
               write_2lines_mid(2);
               
               memset(tempTextU, 0x00, sizeof(tempTextU));
               sprintf(tempTextU, "Remove USB Drive");         
               buf_xxx = get_center((uint8_t)strlen(tempTextU));
               writeTextToFrameBuffer(buf_xxx, 4, tempTextU, (uint32_t)strlen(tempTextU));        
                
               memset(tempTextU, 0x00, sizeof(tempTextU));
               sprintf(tempTextU, "Press any button");         
               write_2lines_mid(5);
                
               memset(tempTextU, 0x00, sizeof(tempTextU));

               sprintf(tempTextU, "to exit");

               buf_xxx = get_center((uint8_t)strlen(tempTextU));
               writeTextToFrameBuffer(buf_xxx, 7, tempTextU, (uint32_t)strlen(tempTextU));        
       }
        
}



uint8_t small_remote_display(void){
        /*
        if(was_b433_det[0] > 0)
        {
                memset(tempText3A, 0x00, sizeof(tempText3A));
                if(Fst_refresh < 30)
                {
                        sprintf(tempText3A, "%s", "B");
                        
                }
                else
                {
                        sprintf(tempText3A, "%s", " ");
                        
                }
                
                return 1;         
        }
        else if(was_b433_det[1] > 0)
        {
                memset(tempText3A, 0x00, sizeof(tempText3A));
                if(Fst_refresh < 30)
                {
                        sprintf(tempText3A, "%s", "G");
                        
                }
                else
                {
                        sprintf(tempText3A, "%s", " ");
                        
                }
                return 1; 
        }
        else if(was_b433_det[2] > 0)
        {
                memset(tempText3A, 0x00, sizeof(tempText3A));
                if(Fst_refresh < 30)
                {
                        sprintf(tempText3A, "%s", "1");
                        
                }
                else
                {
                        sprintf(tempText3A, "%s", " ");
                        
                }
               return 1;           
                
        }
        else if(was_b433_det[3] > 0)
        {
                memset(tempText3A, 0x00, sizeof(tempText3A));
                if(Fst_refresh < 30)
                {
                        sprintf(tempText3A, "%s", "2");
                        
                }
                else
                {
                        sprintf(tempText3A, "%s", " ");
                        
                }
               return 1;           
                
        }
        else if(was_b433_det[4] > 0)
        {
                memset(tempText3A, 0x00, sizeof(tempText3A));
                if(Fst_refresh < 30)
                {
                        sprintf(tempText3A, "%s", "X");
                }
                else
                {
                        sprintf(tempText3A, "%s", " ");
                }
                return 1;
                
        }
        */
        return 0;

}

void update_Wir_2Wire_Icon(void){
         
        
        if(device_Kind == Master_Device){      
                uint8_t con_slaves = 0;
                con_slaves = getSlaves_connected();
              //  disc_slaves = getSlaves_disc();
                
                if(small_remote_display() == 1){
                        writeTextToFrameBuffer(WIRELESS_ICON_HORIZONTAL_OFFSETa - 4, 0, tempText3A, 3);    
                }else{
                        if(get_433_net_key() == 0xABCD){
                                        sprintf(tempText3A, "STD");
                                        writeTextToFrameBuffer(WIRELESS_ICON_HORIZONTAL_OFFSETa - 4, 0, tempText3A, 3);    
                                }else{
                                        sprintf(tempText3A, "%04X", get_433_net_key());
                                        writeTextToFrameBuffer(WIRELESS_ICON_HORIZONTAL_OFFSETa - 4, 0, tempText3A, 4);    
                                }
                }
                
                   for(int x = 0; x < 24; x++)
                                        {
                                                displayBufferT[x] = displayBuffer[0][x];
                                                displayBuffer[0][x] = displayBufferT[x] <<4;
                                                displayBuffer[1][x] = displayBufferT[x] >>4;
                                        }
                        

                if(con_slaves != 0){
                        memset(tempText3A, 0x00, sizeof(tempText3A));
                        
                        sprintf(tempText3A, "S");
                        writeTextToFrameBuffer(32, 0, tempText3A, 1);  
                        
                        memset(tempText3A, 0x00, sizeof(tempText3A));
                        
                        sprintf(tempText3A, "%d", con_slaves);
                        writeTextToFrameBuffer(40, 0, tempText3A, 2);  
                }
                
                /*
                if(disc_slaves != 0){
                        memset(tempText3A, 0x00, sizeof(tempText3A));
                        if(Err_refresh < ERR_RFRFSH)
                        {
                                
                                sprintf(tempText3A, "E");
                                writeTextToFrameBuffer(32, 1, tempText3A, 1);  
                                
                                memset(tempText3A, 0x00, sizeof(tempText3A));
                                
                                sprintf(tempText3A, "%d", disc_slaves);
                                writeTextToFrameBuffer(40, 1, tempText3A, 2);  
                        }
                        else
                        {
                                sprintf(tempText3A, "   ");
                                writeTextToFrameBuffer(30, 1, tempText3A, 4);  
                                
                        }
                        
                }else{
                        if(con_slaves != 0){
                                sprintf(tempText3A, " OK ");
                                writeTextToFrameBuffer(30, 1, tempText3A, 4);  
                        }
                        
                }
                */
                
        }else{
                int W_Signal_temp = 0;
                
                if(is_Wir_Slot_Detect == 0){
                        writeImageToFrameBuffer(WIRELESS_ICON_HORIZONTAL_OFFSETa, 0, AAAAirelessXXX_bmp, (uint32_t)sizeof(AAAAirelessXXX_bmp),
                                                WirelessSignalAAAA_bmp_width, WirelessSignalAAAA_bmp_height);
                        
                if(small_remote_display() == 1){
                        writeTextToFrameBuffer(WIRELESS_ICON_HORIZONTAL_OFFSETa - 4, 0, tempText3A, 3);    
                }else{

                        if(get_433_net_key() == 0xABCD){
                                sprintf(tempText3A, "STD");
                                writeTextToFrameBuffer(WIRELESS_ICON_HORIZONTAL_OFFSETa - 4, 0, tempText3A, 3);    
                        }else{
                                sprintf(tempText3A, "%04X", get_433_net_key());
                                writeTextToFrameBuffer(WIRELESS_ICON_HORIZONTAL_OFFSETa - 4, 0, tempText3A, 4);    
                        }
                }
                        
                        for(int x = 0; x < 24; x++)
                                        {
                                                displayBufferT[x] = displayBuffer[0][x];
                                                displayBuffer[0][x] = displayBufferT[x] <<4;
                                                displayBuffer[1][x] = displayBufferT[x] >>4;
                                        }
                        
                        
                }
                else{
                        W_Signal_temp = (19 + get_device_datas()->wireless_power_status)/20;
                        
                        switch(W_Signal_temp){
                        case 0:
                                writeImageToFrameBuffer(WIRELESS_ICON_HORIZONTAL_OFFSETa, 0, AAAAireless000_bmp, (uint32_t)sizeof(AAAAireless000_bmp),
                                                        WirelessSignalAAAA_bmp_width, WirelessSignalAAAA_bmp_height);
                                break;
                        case 1:
                                writeImageToFrameBuffer(WIRELESS_ICON_HORIZONTAL_OFFSETa, 0, AAAAireless020_bmp, (uint32_t)sizeof(AAAAireless000_bmp),
                                                        WirelessSignalAAAA_bmp_width, WirelessSignalAAAA_bmp_height);
                                break;
                        case 2:
                                writeImageToFrameBuffer(WIRELESS_ICON_HORIZONTAL_OFFSETa, 0, AAAAireless040_bmp, (uint32_t)sizeof(AAAAireless000_bmp),
                                                        WirelessSignalAAAA_bmp_width, WirelessSignalAAAA_bmp_height);
                                break;
                        case 3:
                                writeImageToFrameBuffer(WIRELESS_ICON_HORIZONTAL_OFFSETa, 0, AAAAireless060_bmp, (uint32_t)sizeof(AAAAireless000_bmp),
                                                        WirelessSignalAAAA_bmp_width, WirelessSignalAAAA_bmp_height);
                                break;
                        case 4:
                                writeImageToFrameBuffer(WIRELESS_ICON_HORIZONTAL_OFFSETa, 0, AAAAireless080_bmp, (uint32_t)sizeof(AAAAireless000_bmp),
                                                        WirelessSignalAAAA_bmp_width, WirelessSignalAAAA_bmp_height);
                                break;
                        case 5:
                                writeImageToFrameBuffer(WIRELESS_ICON_HORIZONTAL_OFFSETa, 0, AAAAireless100_bmp, (uint32_t)sizeof(AAAAireless000_bmp),
                                                        WirelessSignalAAAA_bmp_width, WirelessSignalAAAA_bmp_height);
                                break;
                        }
                        
                    
                        if(small_remote_display() == 1){
                                writeTextToFrameBuffer(WIRELESS_ICON_HORIZONTAL_OFFSETa - 4, 0, tempText3A, 3);    
                        }else{

                                
                                if(get_433_net_key() == 0xABCD){
                                           if(Err_refresh < ERR_RFRFSH - 20){
                                                sprintf(tempText3A, "STD");
                                                writeTextToFrameBuffer(WIRELESS_ICON_HORIZONTAL_OFFSETa - 4, 0, tempText3A, 3);   
                                           }else{
                                                 
                                           }
                                }else{
                                           if(Err_refresh < ERR_RFRFSH - 20){
                                                sprintf(tempText3A, "%04X", get_433_net_key());
                                                writeTextToFrameBuffer(WIRELESS_ICON_HORIZONTAL_OFFSETa - 4, 0, tempText3A, 4);    
                                           }else{
                                                
                                           }
                                }
                        }
                
                }
                
                
                if(is_433Mhz_en!= 2){
                        
                        writeImageToFrameBuffer(WIRELESS_ICON_HORIZONTAL_OFFSETx, 0, AAAAirelessXXX_bmp, (uint32_t)sizeof(AAAAirelessXXX_bmp),
                                                WirelessSignalAAAA_bmp_width, WirelessSignalAAAA_bmp_height);
                        
                }else{
                        
                        
                        
                        W_Signal_temp = (19 + calculate_433lost_Percent())/20;

                        
                        switch(W_Signal_temp){
                        case 0:
                                writeImageToFrameBuffer(WIRELESS_ICON_HORIZONTAL_OFFSETx, 0, AAAAireless000_bmp, (uint32_t)sizeof(AAAAireless000_bmp),
                                                        WirelessSignalAAAA_bmp_width, WirelessSignalAAAA_bmp_height);
                                break;
                        case 1:
                                writeImageToFrameBuffer(WIRELESS_ICON_HORIZONTAL_OFFSETx, 0, AAAAireless020_bmp, (uint32_t)sizeof(AAAAireless000_bmp),
                                                        WirelessSignalAAAA_bmp_width, WirelessSignalAAAA_bmp_height);
                                break;
                        case 2:
                                writeImageToFrameBuffer(WIRELESS_ICON_HORIZONTAL_OFFSETx, 0, AAAAireless040_bmp, (uint32_t)sizeof(AAAAireless000_bmp),
                                                        WirelessSignalAAAA_bmp_width, WirelessSignalAAAA_bmp_height);
                                break;
                        case 3:
                                writeImageToFrameBuffer(WIRELESS_ICON_HORIZONTAL_OFFSETx, 0, AAAAireless060_bmp, (uint32_t)sizeof(AAAAireless000_bmp),
                                                        WirelessSignalAAAA_bmp_width, WirelessSignalAAAA_bmp_height);
                                break;
                        case 4:
                                writeImageToFrameBuffer(WIRELESS_ICON_HORIZONTAL_OFFSETx, 0, AAAAireless080_bmp, (uint32_t)sizeof(AAAAireless000_bmp),
                                                        WirelessSignalAAAA_bmp_width, WirelessSignalAAAA_bmp_height);
                                break;
                        case 5:
                                writeImageToFrameBuffer(WIRELESS_ICON_HORIZONTAL_OFFSETx, 0, AAAAireless100_bmp, (uint32_t)sizeof(AAAAireless000_bmp),
                                                        WirelessSignalAAAA_bmp_width, WirelessSignalAAAA_bmp_height);
                                break;
                        }
                        
                        if(Err_refresh < ERR_RFRFSH - 20){
                                //memset(tempText3A, 0x00, sizeof(tempText3A));
                        }
                        else{
                                if(get_mod_test_noise() != 0){
                                        sprintf(tempText3A, "%d ", get_mod_test_noise());
                                        writeTextToFrameBuffer(WIRELESS_ICON_HORIZONTAL_OFFSETx - 2, 0, tempText3A, 4);     
                                }
                        }
                        
                        
                }
                
                
        }
        
        
}


void update_Wireless_Icon(void)
 {
       
       int W_Signal_temp = 0;
         W_Signal_temp = calculate_433lost_Percent();
         
         if(W_Signal_temp == 0){
                W_Signal_temp = get_device_datas()->wireless_power_status;
         }
      //   char Network_NameSZ[4] = {'A','A','A','A'};
         
        // uint32_t tmp_adr_net = Network_Name[3] << 24 | Network_Name[2] << 16 | Network_Name[1] << 8 | Network_Name[0] << 0;
      //   uint32_t tmp_adr_netS = Network_NameSZ[3] << 24 | Network_NameSZ[2] << 16 | Network_NameSZ[1] << 8 | Network_NameSZ[0] << 0;
         
       
       int tempUintResult = W_Signal_temp/12;
       
       switch(tempUintResult){
         case 0:
                
             writeImageToFrameBuffer(WIRELESS_ICON_HORIZONTAL_OFFSET, 0, WirelessSignal_NC_bmp, (uint32_t)sizeof(WirelessSignal_NC_bmp),
                                     WirelessSignal_bmp_width, WirelessSignal_bmp_height);
         
         /*
         writeImageToFrameBuffer(WIRELESS_ICON_HORIZONTAL_OFFSET, 0, Wir2_Full, (uint32_t)sizeof(Wir2_Full),
                                     WirelessSignal_bmp_width_2, WirelessSignal_bmp_height_2);
         
         */
             break;
         case 1:
             writeImageToFrameBuffer(WIRELESS_ICON_HORIZONTAL_OFFSET, 0, WirelessSignal_1_bmp, (uint32_t)sizeof(WirelessSignal_1_bmp),
                                     WirelessSignal_bmp_width, WirelessSignal_bmp_height);
             break;
         case 2:
             writeImageToFrameBuffer(WIRELESS_ICON_HORIZONTAL_OFFSET, 0, WirelessSignal_2_bmp, (uint32_t)sizeof(WirelessSignal_2_bmp),
                                     WirelessSignal_bmp_width, WirelessSignal_bmp_height);
             break;
         case 3:
             writeImageToFrameBuffer(WIRELESS_ICON_HORIZONTAL_OFFSET, 0, WirelessSignal_3_bmp, (uint32_t)sizeof(WirelessSignal_3_bmp),
                                     WirelessSignal_bmp_width, WirelessSignal_bmp_height);
             break;
         case 4:
             writeImageToFrameBuffer(WIRELESS_ICON_HORIZONTAL_OFFSET, 0, WirelessSignal_4_bmp, (uint32_t)sizeof(WirelessSignal_4_bmp),
                                     WirelessSignal_bmp_width, WirelessSignal_bmp_height);
             break;
         case 5:
             writeImageToFrameBuffer(WIRELESS_ICON_HORIZONTAL_OFFSET, 0, WirelessSignal_5_bmp, (uint32_t)sizeof(WirelessSignal_5_bmp),
                                     WirelessSignal_bmp_width, WirelessSignal_bmp_height);
             break;
         case 6:
             writeImageToFrameBuffer(WIRELESS_ICON_HORIZONTAL_OFFSET, 0, WirelessSignal_6_bmp, (uint32_t)sizeof(WirelessSignal_6_bmp),
                                     WirelessSignal_bmp_width, WirelessSignal_bmp_height);
             break;
         case 7:
         case 8:
         case 9:
             writeImageToFrameBuffer(WIRELESS_ICON_HORIZONTAL_OFFSET, 0, WirelessSignal_7_bmp, (uint32_t)sizeof(WirelessSignal_7_bmp),
                                     WirelessSignal_bmp_width, WirelessSignal_bmp_height);				
             break;
       }
       
       
       memset(tempText, 0x00, sizeof(tempText));
       
       if(is_wireless_ok == 0 && get_device_datas()->isWirelessConnected == 0)
       {
                     sprintf(tempText, "E");
                     writeTextToFrameBuffer(0, 0, tempText, 2);  
       }
       else if(get_device_datas()->isWirelessConnected == 0  && strcmp(Network_Name, "AAAA") != 0)
        {
                /*
                memset(tempText, 0x00, sizeof(tempText));
                sprintf(tempText, "%s ", Network_Name);
                writeTextToFrameBuffer(0, 0, tempText, 4);                  
                */
                
        }
        else if(strcmp(Network_Name, "AAAA") == 0){ // if it is standard network
                sprintf(tempText, "%s", "S");
                writeTextToFrameBuffer(0, 0, tempText, 2);    
       
       }
        else if(get_device_datas()->isWirelessConnected == 1  && strcmp(Network_Name, "AAAA") != 0)
        {
               /* 
               sprintf(tempText, "C");
               writeTextToFrameBuffer(0, 0, tempText, 2);  
                */
        }
 }

void update_2Wire_Icon(void) //Time code stat update
 {
         uint8_t tmp_time_codeXXa = get_Time_CodeX();
         

       memset(tempText, 0x00, sizeof(tempText));         
       memset(tempTextU, 0x00, sizeof(tempTextU));
         
        if(get_is_TC_Active() > 0){
                is_blinkTimer_TC++;
        }else{
                is_blinkTimer_TC = 0;
        }
        if(get_is_TC_Active() == 0 || is_blinkTimer_TC < TIME_CODE_BLINK_MIN){
                if(tmp_time_codeXXa == 1 && get_AB_Adre() == MAX_AB_ID && Time_Code_disable != 1){ //"Read SMPTE 25/30fps"
                       sprintf(tempText, ">TC ");
                        if(TC_fps_setX == 1){
                                sprintf(tempTextU, " 25 "); 
                        }else if(TC_fps_setX == 2){
                                sprintf(tempTextU, " 30 "); 
                        }else if(TC_fps_setX == 10){
                                sprintf(tempTextU, " ?? "); 
                        }else{
                                sprintf(tempTextU, " xx "); 
                        }
                }else if(tmp_time_codeXXa == 2 && get_AB_Adre() == MAX_AB_ID && Time_Code_disable != 1){ // "Read FSK PD"
                       sprintf(tempText, ">FSK");
                       sprintf(tempTextU, " PD ");
                }else if(tmp_time_codeXXa == 3 && get_AB_Adre() == MAX_AB_ID && Time_Code_disable != 1){ //"Read FSK F1"
                       sprintf(tempText, ">FSK");
                       sprintf(tempTextU, " F1 ");
                }else if(tmp_time_codeXXa == 4){ //"Generate SMPTE 25fps"
                       sprintf(tempText, " TC>");
                       sprintf(tempTextU, " 25 ");
                }else if(tmp_time_codeXXa == 5){ //"Generate SMPTE 30fps"
                       sprintf(tempText, " TC>");
                       sprintf(tempTextU, " 30 ");
                }else if(tmp_time_codeXXa == 6){ //"Generate FSK PD"
                       sprintf(tempText, "FSK>");
                       sprintf(tempTextU, " PD ");
                }else if(tmp_time_codeXXa == 7){ //"Generate FSK F1"
                       sprintf(tempText, "FSK>");
                       sprintf(tempTextU, " F1 ");
                }else{//"Read SMPTE 25/30fps"
                        //do nothing
                }
        }else{
        
        }
        
        if(is_blinkTimer_TC > TIME_CODE_BLINK_MAX){
                is_blinkTimer_TC = 0;
        }
        
           writeTextToFrameBuffer(TC_OFFSET, 0, tempText, 4);    

           writeTextToFrameBuffer(TC_OFFSET, 1, tempTextU, 4);    
         
         
       /*
       
       if(get_device_datas()->isCANConnected == 1)
        {
              writeImageToFrameBuffer(CAN_ICON_HORIZONTAL_OFFSET, 0, CanConnected_bmp, (uint32_t)sizeof(CanConnected_bmp),
                                      CanConnected_bmp_width, CanConnected_bmp_height);			
             
        }
       else
        {
              writeImageToFrameBuffer(CAN_ICON_HORIZONTAL_OFFSET, 0, CanDisconnected_bmp, (uint32_t)sizeof(CanDisconnected_bmp),
                                      CanConnected_bmp_width, CanConnected_bmp_height);
        }
         */
 }


void update_Bat1_Icon(void)
 {
       
       
       uint8_t tempUintResult = get_device_datas()->internal_battery_status;
       
       switch(tempUintResult){
         case 0: // battery depleated
             if(Err_refresh < BATTERY_ERR_REFRESH_MIN)
              {
                    writeImageToFrameBuffer(BATTERY_1_HORIZONTAL_OFFSET, 0, Battery0Percent_bmp, (uint32_t)sizeof(Battery0Percent_bmp),
                                            Battery_bmp_width, Battery_bmp_height);
              }
             else
              {
                    writeImageToFrameBuffer(BATTERY_1_HORIZONTAL_OFFSET, 0, XXBattery_bmp, (uint32_t)sizeof(XXBattery_bmp),
                                            Battery_bmp_width, Battery_bmp_height);
                    
              }
             break;
             
         case 1:// battery up to 20%
             if(Err_refresh < BATTERY_ERR_REFRESH_MIN)
              {
                    writeImageToFrameBuffer(BATTERY_1_HORIZONTAL_OFFSET, 0, Battery20Percent_bmp, (uint32_t)sizeof(Battery20Percent_bmp),
                                            Battery_bmp_width, Battery_bmp_height);
              }
             else
              {
                    writeImageToFrameBuffer(BATTERY_1_HORIZONTAL_OFFSET, 0, XXBattery_bmp, (uint32_t)sizeof(XXBattery_bmp),
                                            Battery_bmp_width, Battery_bmp_height);
                    
              }
             break;
             
         case 2: //battery level 20 - 40%
             writeImageToFrameBuffer(BATTERY_1_HORIZONTAL_OFFSET, 0, Battery40Percent_bmp, (uint32_t)sizeof(Battery40Percent_bmp),
                                     Battery_bmp_width, Battery_bmp_height);
             break;
         case 3://battery level 40 - 60%
             writeImageToFrameBuffer(BATTERY_1_HORIZONTAL_OFFSET, 0, Battery60Percent_bmp, (uint32_t)sizeof(Battery60Percent_bmp),
                                     Battery_bmp_width, Battery_bmp_height);
             break;
             
         case 4://battery level 60 - 80%
             writeImageToFrameBuffer(BATTERY_1_HORIZONTAL_OFFSET, 0, Battery80Percent_bmp, (uint32_t)sizeof(Battery80Percent_bmp),
                                     Battery_bmp_width, Battery_bmp_height);
             break;
             
         case 5://battery level 80 - 100%
             writeImageToFrameBuffer(BATTERY_1_HORIZONTAL_OFFSET, 0, Battery100Percent_bmp, (uint32_t)sizeof(Battery100Percent_bmp),
                                     Battery_bmp_width, Battery_bmp_height);
             break;
             
         case 6: //battery charge
             writeImageToFrameBuffer(BATTERY_1_HORIZONTAL_OFFSET, 0, NoBattery_bmp, (uint32_t)sizeof(NoBattery_bmp),
                                     Battery_bmp_width, Battery_bmp_height);
             break;
         case 7: //battery error
             if(Err_refresh < BATTERY_ERR_REFRESH_MIN)
              {
                    writeImageToFrameBuffer(BATTERY_1_HORIZONTAL_OFFSET, 0, NoBattery_bmp, (uint32_t)sizeof(NoBattery_bmp),
                                            Battery_bmp_width, Battery_bmp_height);
              }
             else
              {
                    writeImageToFrameBuffer(BATTERY_1_HORIZONTAL_OFFSET, 0, XXBattery_bmp, (uint32_t)sizeof(XXBattery_bmp),
                                            Battery_bmp_width, Battery_bmp_height);
                    
              }
             
             break;
             
       }
       
       
       
 }


void update_AB_Icon(void)
 {
             uint8_t UDC_Mp3_AB = 0;
             
             UDC_Mp3_AB = read_mp3_status();

         
             if(UDC_Mp3_AB == 0)// USB Drive
              {
                    
                    memset(tempText, 0x00, sizeof(tempText));	
                    sprintf(tempText, "%s", "UD");
                    
                    writeTextToFrameBuffer(PLAYER_OFFSET, 0, tempText, 2);
                    
                    memset(tempText, 0x00, sizeof(tempText));		
                    sprintf(tempText, "%s", " ");
                    writeTextToFrameBuffer(85, 0, tempText, 1);
                    
                    memset(tempText, 0x00, sizeof(tempText));		

                    
                    
                   

                             if(PPS_Mp3_AB != saved_Stat)
                             {
                                     if(TimerStatAP == 10)
                                     {
                                            saved_Stat = PPS_Mp3_AB;
                                     }
                                     else if(TimerStatAP > 10)
                                     {
                                        //do nothing
                                     }
                                     else
                                     {
                                        TimerStatAP++;
                                     }
                                     
                             }
                             else
                             {
                                        TimerStatAP = 0;
                             }
                      
                    switch(saved_Stat)
                     {
                       case 1: sprintf(tempText, "%s", "\xF8\xF9"); break; //Play
                       case 2: sprintf(tempText, "%s", "\xFE\xFA"); break; //Stop
                       case 3: sprintf(tempText, "%s", "\xF7\xF6"); break; //Pause
                       case 10: sprintf(tempText, "%s", "xx"); break; //no File
                     }
                    
                    writeTextToFrameBuffer(PLAYER_OFFSET, 1, tempText, 2);
                    
                    memset(tempText, 0x00, sizeof(tempText));		
                    sprintf(tempText, "%s", " ");
                    writeTextToFrameBuffer(85, 1, tempText, 1);
                    
                    memset(tempText, 0x00, sizeof(tempText));
                    
                    //sprintf(tempText, "%s", "SyncB");
                    if (adress_interface() < 10 && SB_AsMaster == 0){
                        sprintf(tempText, "SLAVE");
                        writeTextToFrameBuffer(91, 0, tempText,6);
                    }else{
                        sprintf(tempText, "MASTER");
                        writeTextToFrameBuffer(87, 0, tempText,6);
                    }
                    

                    memset(tempText, 0x00, sizeof(tempText));
                     
                   if (adress_interface() < 10){
                        if(DMX_ev == 0){
                                sprintf(tempText, " A%d ", adress_interface());
                                writeTextToFrameBuffer(96, 1, tempText,4);
                        }else{
                                sprintf(tempText, " A%d D", adress_interface());
                                writeTextToFrameBuffer(91, 1, tempText,4);
                        }
                   }
                   else{
                        sprintf(tempText, "%02d:%d", get_slave_con(),get_AB_con());//M02/A2
                        writeTextToFrameBuffer(91, 1, tempText,6);
                   }
                   
                   

              }
             else if(UDC_Mp3_AB == 1) // Internal Drive
              {
                    memset(tempText, 0x00, sizeof(tempText));	
                    sprintf(tempText, "%s", "ID");
                    
                    writeTextToFrameBuffer(PLAYER_OFFSET, 0, tempText, 2);
                    
                    memset(tempText, 0x00, sizeof(tempText));		
                    sprintf(tempText, "%s", " ");
                    writeTextToFrameBuffer(85, 0, tempText, 1);
                    
                    
                    memset(tempText, 0x00, sizeof(tempText));		
	

                    switch(PPS_Mp3_AB)
                     {
                       case 1: sprintf(tempText, "%s", "\xF8\xF9"); break; //Play
                       case 2: sprintf(tempText, "%s", "\xFE\xFA"); break; //Stop
                       case 3: sprintf(tempText, "%s", "\xF7\xF6"); break; //Pause
                       case 10: sprintf(tempText, "%s", "xx"); break; //no File
                     }
                    
                    writeTextToFrameBuffer(PLAYER_OFFSET, 1, tempText,2);
                    
                    
                    memset(tempText, 0x00, sizeof(tempText));		
                    sprintf(tempText, "%s", " ");
                    writeTextToFrameBuffer(85, 1, tempText, 1);
                    
                    
                    memset(tempText, 0x00, sizeof(tempText));
                   

                     //sprintf(tempText, "%s", "SyncB");
                    if (adress_interface() < 10 && SB_AsMaster == 0){
                        sprintf(tempText, "SLAVE");
                        writeTextToFrameBuffer(91, 0, tempText,6);
                    }else{
                        sprintf(tempText, "MASTER");
                        writeTextToFrameBuffer(87, 0, tempText,6);
                    }
                    

                    memset(tempText, 0x00, sizeof(tempText));
                     
                    if (adress_interface() < 10){
                        if(DMX_ev == 0){
                                sprintf(tempText, " A%d ", adress_interface());
                                writeTextToFrameBuffer(96, 1, tempText,4);
                        }else{
                                sprintf(tempText, " A%d D", adress_interface());
                                writeTextToFrameBuffer(91, 1, tempText,4);
                        }
                   }
                   else{
                           sprintf(tempText, "%02d:%d", get_slave_con(),get_AB_con());//M02/A2
                        writeTextToFrameBuffer(91, 1, tempText,6);
                   }
                   

                 
              }
             else // NO USB Connnected, NO Internal Drive
              {
                    memset(tempText, 0x00, sizeof(tempText));	
                    

                    sprintf(tempText, "%s", "NA");
                      
                    writeTextToFrameBuffer(PLAYER_OFFSET, 0, tempText, 2);
                    memset(tempText, 0x00, sizeof(tempText));		
                    sprintf(tempText, "%s", " ");
                    writeTextToFrameBuffer(85, 0, tempText, 1);
                      
                    sprintf(tempText, "%s", "  ");
                      
                    writeTextToFrameBuffer(PLAYER_OFFSET, 1, tempText, 2);
                    memset(tempText, 0x00, sizeof(tempText));		
                    sprintf(tempText, "%s", " ");
                    writeTextToFrameBuffer(85, 1, tempText, 1);
        

                     for(int x = 73; x < 90; x++)
                            {
                                  
                                  displayBufferT[x] = displayBuffer[0][x];
                                  displayBuffer[0][x] = displayBufferT[x] <<4;
                                  displayBuffer[1][x] = displayBufferT[x] >>4;
                            }                      
                    
                    memset(tempText, 0x00, sizeof(tempText));
                    

                  //  sprintf(tempText, "%s", "SyncB");
                     if (adress_interface() < 10 && SB_AsMaster == 0){
                        sprintf(tempText, "SLAVE");
                        writeTextToFrameBuffer(91, 0, tempText,6);
                    }else{
                        sprintf(tempText, "MASTER");
                        writeTextToFrameBuffer(87, 0, tempText,6);
                    }
                    

                    memset(tempText, 0x00, sizeof(tempText));
                     
                   if (adress_interface() < 10){
                        if(DMX_ev == 0){
                                sprintf(tempText, " A%d ", adress_interface());
                                writeTextToFrameBuffer(96, 1, tempText,4);
                        }else{
                                sprintf(tempText, " A%d D", adress_interface());
                                writeTextToFrameBuffer(91, 1, tempText,4);
                        }
                   }
                   else{
                           sprintf(tempText, "%02d:%d", get_slave_con(),get_AB_con());//M02/A2
                        writeTextToFrameBuffer(91, 1, tempText,6);
                   }
                      
                   memset(tempText, 0x00, sizeof(tempText));
                   memset(bufferDisp, 0x00, sizeof(bufferDisp));
            }
}

void Updates_Player_Stat2(void)
{
        
        uint8_t buf_xxx;

        
     memset(tempTextU, 0x00, sizeof(tempTextU));
     memset(tempText, 0x00, sizeof(tempText));        
     if(read_mp3_status() == 3)
     {
             sprintf(tempTextU, "NO USB DRIVE");
             sprintf(tempText, "Max 32GB Format FAT32");
     }
     else if(file_read_error == 1)
     {
              sprintf(tempTextU, "NO MUSIC FILE");
              sprintf(tempText, "File name: Audio");
     }
     else 
     {

             if(device_Status < Status_PowerEnable)
             {                        

                                      if(is_init_done < 10)
                                      {
                                              memset(tempTextU, 0x00, sizeof(bufferDisp));
                                              sprintf(tempTextU, "PLEASE WAIT");
                                             
                                      }
                                      else if(is_init_done == 100)
                                      {
                                              memset(tempTextU, 0x00, sizeof(bufferDisp));
                                              sprintf(tempTextU, "PLEASE RESTART");
                                      }
                                      else
                                      {
                                        //if(get_AB_Adre() != MAX_AB_ID){
                                          if(1){
                                              sprintf(tempTextU, "<<");
                                              
                                              if(PPS_Mp3_AB == 1 || PPS_Mp3_AB == 2 || PPS_Mp3_AB == 3)
                                              {
                                                      memset(bufferDisp, 0x00, sizeof(bufferDisp));
                                                      sprintf(bufferDisp, "%s", "\xFE\xFA");
                                                      strcat(tempTextU, bufferDisp);
                                              }
                                              else
                                              {
                                                      memset(bufferDisp, 0x00, sizeof(bufferDisp));
                                                      sprintf(bufferDisp, "%s", "  ");
                                                      strcat(tempTextU, bufferDisp);
                                              }
                                             
                                      
                                      
                                              memset(bufferDisp, 0x00, sizeof(bufferDisp));
                                              sprintf(bufferDisp, "%s", " Short Press ");
                                              strcat(tempTextU, bufferDisp);
                                              

                                              
                                              if(PPS_Mp3_AB ==  2 || PPS_Mp3_AB == 3)
                                              {
                                                      memset(bufferDisp, 0x00, sizeof(bufferDisp));
                                                      sprintf(bufferDisp, "%s", "\xF8\xF9");
                                                      strcat(tempTextU, bufferDisp);
                                              }
                                              else if(PPS_Mp3_AB == 1)
                                              {
                                                      memset(bufferDisp, 0x00, sizeof(bufferDisp));
                                                      sprintf(bufferDisp, "%s", "\xF7\xF6");
                                                      strcat(tempTextU, bufferDisp);

                                              }
                                              else
                                              {
                                                      memset(bufferDisp, 0x00, sizeof(bufferDisp));
                                                      sprintf(bufferDisp, "%s", "  ");
                                                      strcat(tempTextU, bufferDisp);
                                              }
                                              
                                              memset(bufferDisp, 0x00, sizeof(bufferDisp));
                                              sprintf(bufferDisp, "%s", ">>");
                                              strcat(tempTextU, bufferDisp);
                                        }
                              }

                      //if(ret_Green_Pin() == 1 || ret_Green_Pin() == 3 || ret_Blue_Pin() == 1 || ret_Blue_Pin() == 3)
                              
                             if(is_init_done == 100)
                             {
                                   sprintf(tempText, "CHECK MUSIC FILE");     
                                     
                             }
                             else
                             {
                                     if(get_pwr_but_timer() > 200){
                                                sprintf(tempTextU, "HOLD FOR POWER OFF");
                                        }
                                     
                                               if(get_AB_Adre() != MAX_AB_ID){
                                                        if(ret_Green_Pin() == 3 || ret_Blue_Pin() == 3)
                                                             {
                                                                      if(main_volume == 100)
                                                                      {
                                                                              sprintf(tempText, "Volume: MAX");                              
                                                                      }
                                                                      else
                                                                      {
                                                                              sprintf(tempText, "Volume:");
                                                                              memset(bufferDisp, 0x00, sizeof(bufferDisp));
                                                                              sprintf(bufferDisp, "%d", main_volume);
                                                                              strcat(tempText, bufferDisp);
                                                                      }
                                                             }
                                                             else
                                                             {
                                                                    sprintf(tempText, "<<V-  LongPress  V+>>");
                                                             }
                                             }else{
                                                     if(device_Status == Status_Idle){
                                                             sprintf(tempText, "       HOLD FOR ARM>>");
                                                     }
                                                     else if(device_Status == Status_PowerEnable){
                                                             sprintf(tempText, "<<TEST        START>>");
                                                     }
                                                     else if(device_Status == Status_Play){
                                                             sprintf(tempText, "<<PAUSE    PAUSE>>");
                                                     }
                                                     else if(device_Status == Status_Pause){
                                                             sprintf(tempText, "<<TEST    RESUME>>");
                                                     }
                                             
                                             }
                                     
                                            
                                     
                        }
                }else if(device_Status == Status_PowerEnable && SB_AsMaster == 1){
                                sprintf(tempTextU, "MASTER MODE");
                                sprintf(tempText, "PRESS GREEN FOR START");
                }
                else
                     {
                             /*
                              memset(tempTextU, 0x00, sizeof(tempTextU));
                             if(isLongBPress > (5 * MIN_LONG_PRESS_TIME))
                             {
                                     sprintf(tempTextU, "ENABLE PWRBT PRESS>>");
                             }
                             else
                             {
                                     if(is_power_on_enable == 0)
                                             {
                                                sprintf(tempTextU, "PWR BUTTON ENABLED");
                                             }
                                             else
                                             {
                                                sprintf(tempTextU, "PWR BUTTON DISABLED");
                                             }
                             }

                             */
                             
                             if(get_pwr_but_timer() > 200){
                                    sprintf(tempTextU, "HOLD FOR POWER OFF");
                            }
                             
                             if(get_AB_Adre() != MAX_AB_ID){
                                      if(main_volume == 100)
                                      {
                                            sprintf(tempText, "<<V- Volume: MAX V+>>");
                                      }
                                      else
                                      {
                                              sprintf(tempText, "<<V-  Volume:");
                                              memset(bufferDisp, 0x00, sizeof(bufferDisp));
                                              sprintf(bufferDisp, "%d", main_volume);
                                              strcat(tempText, bufferDisp);
                                              
                                              memset(bufferDisp, 0x00, sizeof(bufferDisp));
                                              sprintf(bufferDisp, "%s", "  V+>>");
                                              strcat(tempText, bufferDisp);
                                      }
                             }else{
                                     if(device_Status == Status_Idle){
                                             sprintf(tempText, "       HOLD FOR ARM>>");
                                     }
                                     else if(device_Status == Status_PowerEnable){
                                             sprintf(tempText, "<<TEST        START>>");
                                     }
                                     else if(device_Status == Status_Play){
                                             sprintf(tempText, "<<PAUSE    PAUSE>>");
                                     }
                                     else if(device_Status == Status_Pause){
                                             sprintf(tempText, "<<TEST    RESUME>>");
                                     }
                             }
                             
                            
                     }
     }
     
     write_2lines_mid(5);
     
     buf_xxx = get_center((uint8_t)strlen(tempText));
     writeTextToFrameBuffer(buf_xxx, 7, tempText, (uint32_t)strlen(tempText));
}


void Updates_Player_Stat1(void)
{
       uint32_t minv;  
       uint32_t sec;
       uint32_t min;
       uint32_t hor;
       uint32_t milsec = 0;

       char bufferDisp[21] = {0};
       wave_player_current_miliseconds_get(&Music_time);
       
       milsec = Music_time;
       Music_time = Music_time/1000;
       
       milsec = milsec - (1000 * Music_time);
       milsec = milsec/100;
       
       sec = Music_time % 60;
       minv = Music_time / 60;
       min = minv % 60;                        
       hor = minv / 60;    
       
 
        memset(tempTextU, 0x00, sizeof(tempTextU));	
       
        if(read_mp3_Pstatus() == 1 || (read_mp3_Pstatus() == 3 && device_Status > Status_PowerEnable))
        {
        sprintf(tempTextU, "MusicTime");
                           
                           if(hor > 0)
                            {
                                  if(hor < 10)
                                   {
                                         sprintf(bufferDisp, "%s", " 0");
                                   }
                                  else
                                   {
                                         sprintf(bufferDisp, "%s", " ");
                                   }                                
                                  
                                  strcat(tempTextU, bufferDisp);
                                  
                                  memset(bufferDisp, 0x00, sizeof(bufferDisp));
                                  
                                  sprintf(bufferDisp, "%d", hor);
                                  strcat(tempTextU, bufferDisp);            
                                  
                                  memset(bufferDisp, 0x00, sizeof(bufferDisp));                                    
                                  sprintf(bufferDisp, "%s", ":");    
                                  strcat(tempTextU, bufferDisp);                                       
                                  memset(bufferDisp, 0x00, sizeof(bufferDisp));                                                                        
                            }
                           else
                            {
                                  sprintf(bufferDisp, "%s", "  ");    
                                  strcat(tempTextU, bufferDisp);                                       
                                  memset(bufferDisp, 0x00, sizeof(bufferDisp));                                                                   
                                  
                            }
                           if(min < 10)
                            {
                                  sprintf(bufferDisp, "%s", "0");
                                  strcat(tempTextU, bufferDisp);                                        
                            }
                           
                           
                           memset(bufferDisp, 0x00, sizeof(bufferDisp));
                           
                           sprintf(bufferDisp, "%d", min);
                           strcat(tempTextU, bufferDisp);
                           
                           if(sec < 10)
                            {
                                  memset(bufferDisp, 0x00, sizeof(bufferDisp));
                                  sprintf(bufferDisp, "%s", ":0");
                                  strcat(tempTextU, bufferDisp);
                            }
                           else
                            {
                                  memset(bufferDisp, 0x00, sizeof(bufferDisp));
                                  sprintf(bufferDisp, "%s", ":");
                                  strcat(tempTextU, bufferDisp);
                            }
                           
                           
                           memset(bufferDisp, 0x00, sizeof(bufferDisp));
                           sprintf(bufferDisp, "%d", sec);
                           strcat(tempTextU, bufferDisp);
                            
                           memset(bufferDisp, 0x00, sizeof(bufferDisp));
                           sprintf(bufferDisp, "%s", ".");
                           strcat(tempTextU, bufferDisp); 
                            
                           memset(bufferDisp, 0x00, sizeof(bufferDisp));
                           sprintf(bufferDisp, "%d", milsec);
                           strcat(tempTextU, bufferDisp);
                            
                    }
        
                    if(device_Status < Status_PowerEnable)
                    {
                        uint16_t buf_xxx = get_center((uint8_t)strlen(tempTextU));
                        writeTextToFrameBuffer(buf_xxx, 4, tempTextU, (uint32_t)strlen(tempTextU));
                    }
                    else
                    {
                        writeTextToFrameBuffer(0, 4, tempTextU, (uint32_t)strlen(tempTextU));
                    }
}


void Updates_Player_StatTime(void)
{
       uint32_t minv;  
       uint32_t sec;
       uint32_t min;
       uint32_t hor;
       uint32_t milsec = 0;

       char bufferDisp[21] = {0};
       wave_player_current_miliseconds_get(&Music_time);
       
       milsec = Music_time;
       Music_time = Music_time/1000;
       
       milsec = milsec - (1000 * Music_time);
       milsec = milsec/100;
       
       sec = Music_time % 60;
       minv = Music_time / 60;
       min = minv % 60;                        
       hor = minv / 60;    
       
 
        memset(tempTextU, 0x00, sizeof(tempTextU));	
       
        //if(read_mp3_Pstatus() == 1 || (read_mp3_Pstatus() == 3 && device_Status > Status_PowerEnable))
       if(read_mp3_Pstatus() == 1 || read_mp3_Pstatus() == 3)
        { 
        //sprintf(tempTextU, "MusicTime");
                           
                           if(hor > 0)
                            {
                                  if(hor < 10)
                                   {
                                         sprintf(bufferDisp, "%s", " 0");
                                   }
                                  else
                                   {
                                         sprintf(bufferDisp, "%s", " ");
                                   }                                
                                  
                                  strcat(tempTextU, bufferDisp);
                                  
                                  memset(bufferDisp, 0x00, sizeof(bufferDisp));
                                  
                                  sprintf(bufferDisp, "%d", hor);
                                  strcat(tempTextU, bufferDisp);            
                                  
                                  memset(bufferDisp, 0x00, sizeof(bufferDisp));                                    
                                  sprintf(bufferDisp, "%s", ":");    
                                  strcat(tempTextU, bufferDisp);                                       
                                  memset(bufferDisp, 0x00, sizeof(bufferDisp));                                                                        
                            }
                           else
                            {
                                    /*
                                  sprintf(bufferDisp, "%s", "  ");    
                                  strcat(tempTextU, bufferDisp);                                       
                                  memset(bufferDisp, 0x00, sizeof(bufferDisp));                                                                   
                                    */
                                  
                            }
                            
                           if(min < 10)
                            {
                                  sprintf(bufferDisp, "%s", "0");
                                  strcat(tempTextU, bufferDisp);                                        
                            }
                           
                           
                           memset(bufferDisp, 0x00, sizeof(bufferDisp));
                           
                           sprintf(bufferDisp, "%d", min);
                           strcat(tempTextU, bufferDisp);
                           
                           if(sec < 10)
                            {
                                  memset(bufferDisp, 0x00, sizeof(bufferDisp));
                                  sprintf(bufferDisp, "%s", ":0");
                                  strcat(tempTextU, bufferDisp);
                            }
                           else
                            {
                                  memset(bufferDisp, 0x00, sizeof(bufferDisp));
                                  sprintf(bufferDisp, "%s", ":");
                                  strcat(tempTextU, bufferDisp);
                            }
                           
                           
                           memset(bufferDisp, 0x00, sizeof(bufferDisp));
                           sprintf(bufferDisp, "%d", sec);
                           strcat(tempTextU, bufferDisp);
                            
                           memset(bufferDisp, 0x00, sizeof(bufferDisp));
                           sprintf(bufferDisp, "%s", ".");
                           strcat(tempTextU, bufferDisp); 
                            
                           memset(bufferDisp, 0x00, sizeof(bufferDisp));
                           sprintf(bufferDisp, "%d", milsec);
                           strcat(tempTextU, bufferDisp);
                            
                            if(hor == 0)
                            {
                                memset(bufferDisp, 0x00, sizeof(bufferDisp));
                                sprintf(bufferDisp, "%s", "  < >  ");
                                strcat(tempTextU, bufferDisp);
                            }
                            else
                            {
                                memset(bufferDisp, 0x00, sizeof(bufferDisp));
                                sprintf(bufferDisp, "%s", "<  >");
                                strcat(tempTextU, bufferDisp);
                            
                            }
                            
                               wave_player_remain_miliseconds_get(&Remain_time);
                               
                               milsec = Remain_time;
                               Remain_time = Remain_time/1000;
                               
                               milsec = milsec - (1000 * Remain_time);
                               milsec = milsec/100;
                               
                               sec = Remain_time % 60;
                               minv = Remain_time / 60;
                               min = minv % 60;                        
                               hor = minv / 60;  
                            

                           if(min < 10)
                            {
                                  sprintf(bufferDisp, "%s", "0");
                                  strcat(tempTextU, bufferDisp);                                        
                            }
                           
                           
                           memset(bufferDisp, 0x00, sizeof(bufferDisp));
                           
                           sprintf(bufferDisp, "%d", min);
                           strcat(tempTextU, bufferDisp);
                           
                           if(sec < 10)
                            {
                                  memset(bufferDisp, 0x00, sizeof(bufferDisp));
                                  sprintf(bufferDisp, "%s", ":0");
                                  strcat(tempTextU, bufferDisp);
                            }
                           else
                            {
                                  memset(bufferDisp, 0x00, sizeof(bufferDisp));
                                  sprintf(bufferDisp, "%s", ":");
                                  strcat(tempTextU, bufferDisp);
                            }
                           
                           
                           memset(bufferDisp, 0x00, sizeof(bufferDisp));
                           sprintf(bufferDisp, "%d", sec);
                           strcat(tempTextU, bufferDisp);
                            
                           memset(bufferDisp, 0x00, sizeof(bufferDisp));
                           sprintf(bufferDisp, "%s", ".");
                           strcat(tempTextU, bufferDisp); 
                            
                           memset(bufferDisp, 0x00, sizeof(bufferDisp));
                           sprintf(bufferDisp, "%d", milsec);
                           strcat(tempTextU, bufferDisp);
                            
                            
                     writeTextToFrameBuffer(0, 4, tempTextU, (uint32_t)strlen(tempTextU));       
                    }
        else
        {
                     if(read_mp3_status() == 3 || file_read_error == 1)
                     {
                              if(Err_refresh < BATTERY_ERR_REFRESH_MIN)
                              {
                                   sprintf(tempTextU, "AUDIO PLAYER ERROR");
                              }
                              else
                              {
                                   sprintf(tempTextU, "                     ");
                              }
                     }
                     else
                     {
                             if(is_init_done == 10)
                             {
                                     if(device_Status > Status_PowerEnable && PPS_Mp3_AB == 2)
                                     {
                                        sprintf(tempTextU, "MUSIC FINISHED");
                                     }
                                     else
                                     {
                                        sprintf(tempTextU, "AUDIO PLAYER READY");
                                     }
  
                             }
                             else if(is_init_done == 100)
                             {
                                     memset(tempTextU, 0x00, sizeof(bufferDisp));
                                      if(Err_refresh < BATTERY_ERR_REFRESH_MIN)
                                      {
                                           sprintf(tempTextU, "AUDIO PLAYER ERROR");
                                      }
                                      else
                                      {
                                           sprintf(tempTextU, "                     ");
                                      }
                             }
                             else
                             {
                                        if(Err_refresh < BATTERY_ERR_REFRESH_MIN)
                                        {
                                                sprintf(tempTextU, "STARTING AUDIO PLAYER");
                                        }
                                        else
                                        {
                                            sprintf(tempTextU, "                     ");
                                        }
                             }
                     }
                
                uint16_t buf_xxx = get_center((uint8_t)strlen(tempTextU));
                writeTextToFrameBuffer(buf_xxx, 4, tempTextU, (uint32_t)strlen(tempTextU));
                
        }
}


void Updates_Player_Stat3(void)
{
       uint32_t minv;  
       uint32_t sec;
       uint32_t min;
       uint32_t hor;
       uint32_t milsec = 0;

       char bufferDisp[21] = {0};
       wave_player_remain_miliseconds_get(&Remain_time);
       
       milsec = Remain_time;
       Remain_time = Remain_time/1000;
       
       milsec = milsec - (1000 * Remain_time);
       milsec = milsec/100;
       
       sec = Remain_time % 60;
       minv = Remain_time / 60;
       min = minv % 60;                        
       hor = minv / 60;    
       
 
        memset(tempTextU, 0x00, sizeof(tempTextU));	
       
        if(read_mp3_Pstatus() == 1 || (read_mp3_Pstatus() == 3 && device_Status > Status_PowerEnable))
        {
                sprintf(tempTextU, "Remaining:");
                           
                           if(hor > 0)
                            {
                                  if(hor < 10)
                                   {
                                         sprintf(bufferDisp, "%s", " 0");
                                   }
                                  else
                                   {
                                         sprintf(bufferDisp, "%s", " ");
                                   }                                
                                  
                                  strcat(tempTextU, bufferDisp);
                                  
                                  memset(bufferDisp, 0x00, sizeof(bufferDisp));
                                  
                                  sprintf(bufferDisp, "%d", hor);
                                  strcat(tempTextU, bufferDisp);            
                                  
                                  memset(bufferDisp, 0x00, sizeof(bufferDisp));                                    
                                  sprintf(bufferDisp, "%s", ":");    
                                  strcat(tempTextU, bufferDisp);                                       
                                  memset(bufferDisp, 0x00, sizeof(bufferDisp));                                                                        
                            }
                           else
                            {
                                  sprintf(bufferDisp, "%s", "  ");    
                                  strcat(tempTextU, bufferDisp);                                       
                                  memset(bufferDisp, 0x00, sizeof(bufferDisp));                                                                   
                                  
                            }
                           if(min < 10)
                            {
                                  sprintf(bufferDisp, "%s", "0");
                                  strcat(tempTextU, bufferDisp);                                        
                            }
                           
                           
                           memset(bufferDisp, 0x00, sizeof(bufferDisp));
                           
                           sprintf(bufferDisp, "%d", min);
                           strcat(tempTextU, bufferDisp);
                           
                           if(sec < 10)
                            {
                                  memset(bufferDisp, 0x00, sizeof(bufferDisp));
                                  sprintf(bufferDisp, "%s", ":0");
                                  strcat(tempTextU, bufferDisp);
                            }
                           else
                            {
                                  memset(bufferDisp, 0x00, sizeof(bufferDisp));
                                  sprintf(bufferDisp, "%s", ":");
                                  strcat(tempTextU, bufferDisp);
                            }
                           
                           
                           memset(bufferDisp, 0x00, sizeof(bufferDisp));
                           sprintf(bufferDisp, "%d", sec);
                           strcat(tempTextU, bufferDisp);
                            
                           memset(bufferDisp, 0x00, sizeof(bufferDisp));
                           sprintf(bufferDisp, "%s", ".");
                           strcat(tempTextU, bufferDisp); 
                            
                           memset(bufferDisp, 0x00, sizeof(bufferDisp));
                           sprintf(bufferDisp, "%d", milsec);
                           strcat(tempTextU, bufferDisp);
                            
                    }
        
                    if(device_Status < Status_PowerEnable)
                    {
                        write_2lines_mid(5);
                    }
                    else
                    {
                        write_2lines(5);
                    }
}

 
 void Update_States(void)
 {
       uint32_t milsec = 0;
       uint32_t sec;
       uint32_t min;
       uint32_t minv;                        
       uint32_t hor;      
       uint32_t systimer = 0;
       char bufferDisp[21] = {0};
       int minx = 0;
       
                    //uint16_t UhoursSMPTE = 0u;
                    //uint16_t UminutesSMPTE = 0u;
                    //uint16_t USecSMPTE = 0u;
                 //   uint16_t UFramesSMPTE = 0u;
                        
       

       systimer = getDevicetime()/1000;

       milsec = getDevicetime();
              
       milsec = milsec - (1000 * systimer);
       milsec = milsec/100;
       
       sec = systimer % 60;
       minv = systimer / 60;
       min = minv % 60;                        
       hor = minv / 60;                
       

       
       switch(device_Status){
         //case 0: case 2:// INIT State
           case 0:
                   
            memset(tempTextU, 0x00, sizeof(tempTextU));
         
           if(countLastTime1 > 0)
           {
                        sprintf(tempTextU, "FILE READ ERROR:%d", countLastTime1);   
                        strcat(tempTextU, bufferDisp);
           }
           else
           {
           
           
                     memset(tempTextU, 0x00, sizeof(tempTextU));	
            
                              if(countLastTime1 > 0)
                              {
                                      sprintf(tempTextU, "FILE READ ERROR:%d", countLastTime1);   
                              }
                              else
                              {
                                      if(Network_found == 0)
                                      {
                                                sprintf(tempTextU, "NO NETWORK");
                                      }
                                      else
                                      {
                                              sprintf(tempTextU, "NETWORK FOUND");
                                      }
                              }
           }
                     
                     write_2lines_mid(2);
                     break;
           
         case 1:// Create Network (Power on modules) state
             memset(tempTextU, 0x00, sizeof(tempTextU));	
             sprintf(tempTextU, "POWER ON MODULES");
             write_2lines_mid(2);
             break;
             
         case 2: //IDLE (TEST) State
             memset(tempTextU, 0x00, sizeof(tempTextU));
         
           if(countLastTime1 > 0)
           {
                        sprintf(tempTextU, "FILE READ ERROR:%d", countLastTime1);   
                        strcat(tempTextU, bufferDisp);
           }
           else
           {
                 /*
                    EXT_Time = get_real_time();
               
                    minx = EXT_Time/60000;
                    UhoursSMPTE = minx / 60;
                    UminutesSMPTE = minx % 60;
                    USecSMPTE = (EXT_Time - UhoursSMPTE * 3600000 - UminutesSMPTE * 60000)/1000;
                    UFramesSMPTE = (EXT_Time - UhoursSMPTE * 3600000 - UminutesSMPTE * 60000)%1000;
                   
                    sprintf(tempTextU, "%02d:%02d:%02d.%03d  %d", UhoursSMPTE, UminutesSMPTE, USecSMPTE, UFramesSMPTE, GPS_Fixed);
                   */
                 
              memset(tempTextU, 0x00, sizeof(tempTextU));
             sprintf(tempTextU, "TEST ");
                   
              if(Time_code_Started == 1){ 
                      EXT_Time = get_time_code_timer();
              }else if(get_is_TC_Active() != 0){//Time code
                      //Get Time code time
                       EXT_Time = get_SMPTE_Time();
              }
              else if(isGPSFTValid == 0){
                        EXT_Time = get_real_time();
              }else if(isGPSFTValid == 1){
                        EXT_Time = get_tus();
               }
              
               
                              
                    minx = EXT_Time/60000;
                    UhoursSMPTE = minx / 60;
                    UminutesSMPTE = minx % 60;
                    USecSMPTE = (EXT_Time - UhoursSMPTE * 3600000 - UminutesSMPTE * 60000)/1000;
                 
               
               memset(bufferDisp, 0x00, sizeof(bufferDisp));               

              if(Time_code_Started == 1){ 
                       sprintf(bufferDisp, "%02d:%02d:%02d TC>", UhoursSMPTE, UminutesSMPTE, USecSMPTE);
              }else if(get_is_TC_Active() != 0)
              {
               
                    switch (get_time_code_Stat())
                     {
                       case 1:
                           sprintf(bufferDisp, " L25 %02d:%02d:%02d", UhoursSMPTE, UminutesSMPTE, USecSMPTE);
                           break;
                       case 2:
                           sprintf(bufferDisp, " L33 %02d:%02d:%02d", UhoursSMPTE, UminutesSMPTE, USecSMPTE);
                           break;
                       case 3:
                           sprintf(bufferDisp, " FPD %02d:%02d:%02d", UhoursSMPTE, UminutesSMPTE, USecSMPTE);
                           break;
                       case 4:
                           sprintf(bufferDisp, " FF1 %02d:%02d:%02d", UhoursSMPTE, UminutesSMPTE, USecSMPTE);
                           break;
                       case 5:
                           sprintf(bufferDisp, " INT PL ");
                           break;
                           
                     }
              }
              else if (GPS_Fixed == 1){

                      if(isGPSFTValid == 1){
                                sprintf(bufferDisp, " TUS %02d:%02d:%02d G", UhoursSMPTE, UminutesSMPTE, USecSMPTE);
                      }
                      else{
                                
                                sprintf(bufferDisp, " GTC %02d:%02d:%02d", UhoursSMPTE, UminutesSMPTE, USecSMPTE);
                            //sprintf(bufferDisp, " GTC %02d:%02d:%02d %d", UhoursSMPTE, UminutesSMPTE, USecSMPTE, isGPSTimerValid);
                           //   sprintf(bufferDisp, "%02d:%02d:%02d %d", UhoursSMPTE, UminutesSMPTE, USecSMPTE, Dif_timerGPS);
                      }
              }
              else if (isGPSTimerCTRL == 1){
                              if(isGPSFTValid == 1){
                                        sprintf(bufferDisp, " TUS %02d:%02d:%02d R", UhoursSMPTE, UminutesSMPTE, USecSMPTE);
                              }else{
                                      sprintf(bufferDisp, " RTC %02d:%02d:%02d", UhoursSMPTE, UminutesSMPTE, USecSMPTE);
                              }
               }else{
                        
                                if(isGPSFTValid == 1){
                                                sprintf(bufferDisp, " TUS %02d:%02d:%02d I", UhoursSMPTE, UminutesSMPTE, USecSMPTE);
                                      }else{
                                               sprintf(bufferDisp, " ITC %02d:%02d:%02d", UhoursSMPTE, UminutesSMPTE, USecSMPTE);
                                      }
               }
                        
                        strcat(tempTextU, bufferDisp);
      }


             write_2lines_mid(2);
      
          //    sendToDisplay(0,2,displayBuffer[2], DISPLAY_NUMBER_OF_COLUMNS);
          //     sendToDisplay(0,3,displayBuffer[3], DISPLAY_NUMBER_OF_COLUMNS);
      
             break;
             
         case 3: // Power enable (Arm) state
             
             memset(tempTextU, 0x00, sizeof(tempTextU));
            sprintf(tempTextU, "ARMED");   
                memset(bufferDisp, 0x00, sizeof(bufferDisp));

                if (isGPSTimerValid != 0){
                      if(isGPSFTValid == 0){
                                EXT_Time = get_real_time();
                      }
                      else if(isGPSFTValid == 1){
                                EXT_Time = get_tus();
                      }
              }
              else if (isGPSTimerValid == 0){
                      if(isGPSFTValid == 1){
                                EXT_Time = GPS_SetValue;
                      }
              }
                              
                    minx = EXT_Time/60000;
                    UhoursSMPTE = minx / 60;
                    UminutesSMPTE = minx % 60;
                    USecSMPTE = (EXT_Time - UhoursSMPTE * 3600000 - UminutesSMPTE * 60000)/1000;
                              

              if (isGPSTimerValid != 0){
                      if(isGPSFTValid == 0){
                                sprintf(bufferDisp, " CTC %02d:%02d:%02d R", UhoursSMPTE, UminutesSMPTE, USecSMPTE);
                                strcat(tempTextU, bufferDisp);
                      }
                      else if(isGPSFTValid == 1){
                                sprintf(bufferDisp, " TUS %02d:%02d:%02d R", UhoursSMPTE, UminutesSMPTE, USecSMPTE);
                                strcat(tempTextU, bufferDisp);
                      }
              }
              else if (isGPSTimerValid == 0){
                      if(isGPSFTValid == 1){
                                sprintf(bufferDisp, " TTS %02d:%02d:%02d R", UhoursSMPTE, UminutesSMPTE, USecSMPTE);
                                strcat(tempTextU, bufferDisp);
                      }
              }         
         
            
             write_2lines_mid(2);
             break;
             
         case 4: //Play state
             

             
         memset(tempTextU, 0x00, sizeof(tempTextU));	
         
         //sprintf(tempTextU, "Show Time");
                           
                           if(hor > 0)
                            {
                                  if(hor < 10)
                                   {
                                         sprintf(bufferDisp, "%s", " 0");
                                   }
                                  else
                                   {
                                         sprintf(bufferDisp, "%s", " ");
                                   }                                
                                  
                                  strcat(tempTextU, bufferDisp);
                                  
                                  memset(bufferDisp, 0x00, sizeof(bufferDisp));
                                  
                                  sprintf(bufferDisp, "%d", hor);
                                  strcat(tempTextU, bufferDisp);            
                                  
                                  memset(bufferDisp, 0x00, sizeof(bufferDisp));                                    
                                  sprintf(bufferDisp, "%s", ":");    
                                  strcat(tempTextU, bufferDisp);                                       
                                  memset(bufferDisp, 0x00, sizeof(bufferDisp));                                                                        
                            }
                           else
                            {
                                    /*
                                  sprintf(bufferDisp, "%s", "  ");    
                                  strcat(tempTextU, bufferDisp);                                       
                                  memset(bufferDisp, 0x00, sizeof(bufferDisp));       
*/                                    
                                  
                            }
                           if(min < 10)
                            {
                                  sprintf(bufferDisp, "%s", "0");
                                  strcat(tempTextU, bufferDisp);                                        
                            }
                           
                           
                           memset(bufferDisp, 0x00, sizeof(bufferDisp));
                           
                           sprintf(bufferDisp, "%d", min);
                           strcat(tempTextU, bufferDisp);
                           
                           if(sec < 10)
                            {
                                  memset(bufferDisp, 0x00, sizeof(bufferDisp));
                                  sprintf(bufferDisp, "%s", ":0");
                                  strcat(tempTextU, bufferDisp);
                            }
                           else
                            {
                                  memset(bufferDisp, 0x00, sizeof(bufferDisp));
                                  sprintf(bufferDisp, "%s", ":");
                                  strcat(tempTextU, bufferDisp);
                            }
                           
                           
                           memset(bufferDisp, 0x00, sizeof(bufferDisp));
                           sprintf(bufferDisp, "%d", sec);
                           strcat(tempTextU, bufferDisp);
                            
                           memset(bufferDisp, 0x00, sizeof(bufferDisp));
                           sprintf(bufferDisp, "%s", ".");
                           strcat(tempTextU, bufferDisp); 
                            
                           memset(bufferDisp, 0x00, sizeof(bufferDisp));
                           sprintf(bufferDisp, "%d", milsec);
                           strcat(tempTextU, bufferDisp);
 
if(PPS_Mp3_AB == 1 || PPS_Mp3_AB == 3)
{                            
                        
                           memset(bufferDisp, 0x00, sizeof(bufferDisp));
                           sprintf(bufferDisp, "%s", " PLAY");
                           strcat(tempTextU, bufferDisp); 
                           
        /*
                           memset(bufferDisp, 0x00, sizeof(bufferDisp));
                           sprintf(bufferDisp, " %d", sync_times);
                           strcat(tempTextU, bufferDisp); 
*/
                        
                           test_xxx_AB++;
                           if(test_xxx_AB > 80)
                           {
                                test_xxx_AB = 0;
                                   wave_player_current_miliseconds_get(&Music_time);
                                   drift_AB_T = getDevicetime() - Music_time;
                           }
                                   
                           memset(bufferDisp, 0x00, sizeof(bufferDisp));
                           sprintf(bufferDisp, " %d", drift_AB_T);
                           strcat(tempTextU, bufferDisp); 
}
else
{         
                           memset(bufferDisp, 0x00, sizeof(bufferDisp));
                           sprintf(bufferDisp, "%s", "    PLAY");
                           strcat(tempTextU, bufferDisp); 
}

                            
             
             write_2lines(2);
             break;
             
         case 5: //Pause state
             memset(tempTextU, 0x00, sizeof(tempTextU));
             
                   // sprintf(tempTextU, "PAUSE");
                    
                    if(hor > 0)
                     {
                           if(hor < 10)
                            {
                                  sprintf(bufferDisp, "%s", " 0");
                            }
                           else
                            {
                                  sprintf(bufferDisp, "%s", " ");
                            }                                
                           
                           strcat(tempTextU, bufferDisp);
                           
                           memset(bufferDisp, 0x00, sizeof(bufferDisp));
                           
                           sprintf(bufferDisp, "%d", hor);
                           strcat(tempTextU, bufferDisp);            
                           
                           memset(bufferDisp, 0x00, sizeof(bufferDisp));                                    
                           sprintf(bufferDisp, "%s", ":");    
                           strcat(tempTextU, bufferDisp);                                       
                           memset(bufferDisp, 0x00, sizeof(bufferDisp));                                                                        
                     }
                    else
                     {
                             /*
                           sprintf(bufferDisp, "%s", "  ");    
                           strcat(tempTextU, bufferDisp);                                       
                           memset(bufferDisp, 0x00, sizeof(bufferDisp)); 
*/                             
                           
                     }
                    if(min < 10)
                     {
                           sprintf(bufferDisp, "%s", "0");
                           strcat(tempTextU, bufferDisp);                                        
                     }
                    
                    
                    memset(bufferDisp, 0x00, sizeof(bufferDisp));
                    
                    sprintf(bufferDisp, "%d", min);
                    strcat(tempTextU, bufferDisp);
                    
                    if(sec < 10)
                     {
                           memset(bufferDisp, 0x00, sizeof(bufferDisp));
                           sprintf(bufferDisp, "%s", ":0");
                           strcat(tempTextU, bufferDisp);
                     }
                    else
                     {
                           memset(bufferDisp, 0x00, sizeof(bufferDisp));
                           sprintf(bufferDisp, "%s", ":");
                           strcat(tempTextU, bufferDisp);
                     }
                    
                    memset(bufferDisp, 0x00, sizeof(bufferDisp));
                    sprintf(bufferDisp, "%d", sec);
                    strcat(tempTextU, bufferDisp);
                     
                    memset(bufferDisp, 0x00, sizeof(bufferDisp));
                    sprintf(bufferDisp, "%s", ".");
                    strcat(tempTextU, bufferDisp); 
                            
                    memset(bufferDisp, 0x00, sizeof(bufferDisp));
                    sprintf(bufferDisp, "%d", milsec);
                    strcat(tempTextU, bufferDisp);
                     
                    memset(bufferDisp, 0x00, sizeof(bufferDisp));
                    sprintf(bufferDisp, "%s", "    PAUSE");
                    strcat(tempTextU, bufferDisp); 
             
             write_2lines(2);
             break;
             
         case 6: // Manual (STEP) state
               //do nothing
             break;
             
             
         case 9: // Error state
             memset(tempTextU, 0x00, sizeof(tempTextU));	
             sprintf(tempTextU, "ERROR");
             write_2lines_mid(2);
             break;
             
             
       }
       
 }


 
 uint8_t get_center(uint8_t len_ccc)
 {
       
       uint8_t bufUint=0u;
       if(len_ccc < MAX_TEXT_LENGTH)
        {
              bufUint=(uint8_t)(MAX_TEXT_LENGTH - len_ccc);
              if((bufUint%2)!=0){
                    bufUint/=2;
                    bufUint*=font_fixed_8px_width;
                    bufUint+=6;
              }else{
                    bufUint/=2;
                    bufUint*=font_fixed_8px_width;
                    bufUint+=3;
              }
              
              return bufUint;
        }
       else
        {
              return 0;
        }
       
       
}
 

void write_2lines_mid(uint8_t line_1)
 {
       uint8_t buf_xxx;
       buf_xxx = get_center((uint8_t)strlen(tempTextU));

       writeTextToFrameBuffer(buf_xxx, line_1, tempTextU, (uint32_t)strlen(tempTextU));
       memset(tempTextU, 0x00, sizeof(tempTextU));		
       
         
       for(int x=0; x< 128; x++)
        {
              
              displayBufferT[x] = displayBuffer[line_1][x];
              displayBuffer[line_1][x] = displayBufferT[x] <<5;
              displayBuffer[line_1+1][x] = displayBufferT[x] >>3;
        }
}
 
void write_2lines(uint8_t line_1)
 {

       writeTextToFrameBuffer(0, line_1, tempTextU, (uint32_t)strlen(tempTextU));
       memset(tempTextU, 0x00, sizeof(tempTextU));		
         
           
         
         
       for(int x=0; x< 128; x++)
        {
              
              displayBufferT[x] = displayBuffer[line_1][x];
              displayBuffer[line_1][x] = displayBufferT[x] <<5;
              displayBuffer[line_1+1][x] = displayBufferT[x] >>3;
        }
}

void set_Long_refresh(uint8_t is_rst_LR)
{
        if(is_rst_LR == 0)
        {
                Long_refresh = 0;
        }
        else
        {
                Long_refresh = 150;
        }
}




 
void LCD_FailCable(void)
{

        memset(tempTextU, 0x00, sizeof(tempTextU));   
       
      memset(displayBuffer, 0x00, sizeof(displayBuffer));
      for(int n = 2; n < DISPLAY_NUMBER_OF_PAGES; n++){
            memset(displayBuffer[n], 0x00, sizeof(displayBuffer));
      }
      

       memset(tempTextU, 0x00, sizeof(tempTextU));   
       sprintf(tempTextU, "%s", "CHARGING");
       write_2lines_mid(2); 
       

               memset(tempTextU, 0x00, sizeof(tempTextU));   
               sprintf(tempTextU, "%s", "DISCONNECT USB CABLE");
               write_2lines_mid(4); 

               memset(tempTextU, 0x00, sizeof(tempTextU));   
               sprintf(tempTextU, "%s", "CONNECT USB DRIVE");
               write_2lines_mid(6);  


       sendToDisplay(0,2,displayBuffer[2], DISPLAY_NUMBER_OF_COLUMNS);
       sendToDisplay(0,3,displayBuffer[3], DISPLAY_NUMBER_OF_COLUMNS);
       sendToDisplay(0,4,displayBuffer[4], DISPLAY_NUMBER_OF_COLUMNS);
       sendToDisplay(0,5,displayBuffer[5], DISPLAY_NUMBER_OF_COLUMNS);
       sendToDisplay(0,6,displayBuffer[6], DISPLAY_NUMBER_OF_COLUMNS);
       sendToDisplay(0,7,displayBuffer[7], DISPLAY_NUMBER_OF_COLUMNS);
       
}


void Wireless_RST_Wait(uint8_t asdfg)
{
        
                uint8_t buf_xxx = 0;
        
                res_buf_line(2);
                res_buf_line(3);
                res_buf_line(4);
                res_buf_line(5);
                res_buf_line(6);
                res_buf_line(7);
                
        /*
                if(asdfg == 0)
                {
                        sprintf(tempTextU, "%s", "WIRELESS RESET");
                }
                else 
                {
                        sprintf(tempTextU, "%s", "WIRELESS LEARNING");
                }
                */
                
                sprintf(tempTextU, "%s", "WIRELESS SETUP");
                
                buf_xxx = get_center((uint8_t)strlen(tempTextU));
                writeTextToFrameBuffer(buf_xxx, 2, tempTextU, (uint32_t)strlen(tempTextU));	   
                
                write_2lines_mid(2);
                
                memset(tempTextU, 0x00, sizeof(tempTextU));	             
                
                
                if(asdfg == 0)
                {
                        sprintf(tempTextU, "%s", "Standard Network"); 
                }
                else{
                        sprintf(tempTextU, "%s", "Custom Network"); 
                }
                
                        
                buf_xxx = get_center((uint8_t)strlen(tempTextU));
                writeTextToFrameBuffer(buf_xxx, 4, tempTextU, (uint32_t)strlen(tempTextU));	   



                
                memset(tempTextU, 0x00, sizeof(tempTextU));	    
                
                sprintf(tempTextU, "%s", "PLEASE WAIT");
                
                buf_xxx = get_center((uint8_t)strlen(tempTextU));
                writeTextToFrameBuffer(buf_xxx, 5, tempTextU, (uint32_t)strlen(tempTextU));	   
                
                write_2lines_mid(5);
                
                memset(tempTextU, 0x00, sizeof(tempTextU));	    
                
                sprintf(tempTextU, "%s", "AUTOMATICALLY RESTART");
                        
                buf_xxx = get_center((uint8_t)strlen(tempTextU));
                writeTextToFrameBuffer(buf_xxx, 7, tempTextU, (uint32_t)strlen(tempTextU));	   
                
                /*
                memset(tempTextU, 0x00, sizeof(tempTextU));	             
                              
                
                sprintf(tempTextU, "%s", "MODULE WILL RESTART");
                buf_xxx = get_center((uint8_t)strlen(tempTextU));
                writeTextToFrameBuffer(buf_xxx, 5, tempTextU, (uint32_t)strlen(tempTextU));	   
                
                write_2lines_mid(5);             
               
                
                
                memset(tempTextU, 0x00, sizeof(tempTextU));	
                
                sprintf(tempTextU, "%s", "AUTOMATICALLY");
                buf_xxx = get_center((uint8_t)strlen(tempTextU));
                writeTextToFrameBuffer(buf_xxx, 7, tempTextU, (uint32_t)strlen(tempTextU));	
                */
               // display_refresh(1);
                
                       sendToDisplay(0,2,displayBuffer[2], DISPLAY_NUMBER_OF_COLUMNS);
       sendToDisplay(0,3,displayBuffer[3], DISPLAY_NUMBER_OF_COLUMNS);
       sendToDisplay(0,4,displayBuffer[4], DISPLAY_NUMBER_OF_COLUMNS);
       sendToDisplay(0,5,displayBuffer[5], DISPLAY_NUMBER_OF_COLUMNS);
       sendToDisplay(0,6,displayBuffer[6], DISPLAY_NUMBER_OF_COLUMNS);
       sendToDisplay(0,7,displayBuffer[7], DISPLAY_NUMBER_OF_COLUMNS);

}


void Wireless_Choose(void)
{
        
                uint8_t buf_xxx = 0;
        
                res_buf_line(2);
                res_buf_line(3);
                res_buf_line(4);
                res_buf_line(5);
                res_buf_line(6);
                res_buf_line(7);
                
                
                sprintf(tempTextU, "%s", "WIRELESS RESET");
                buf_xxx = get_center((uint8_t)strlen(tempTextU));
                writeTextToFrameBuffer(buf_xxx, 2, tempTextU, (uint32_t)strlen(tempTextU));	   
                
                write_2lines_mid(2);
                
                memset(tempTextU, 0x00, sizeof(tempTextU));	             
/*
                       memset(tempTextU, 0x00, sizeof(tempTextU));       
                       sprintf(tempTextU, "%s", "                     ");
                       write_2lines_mid(4);

      */  
                sprintf(tempTextU, "%s", "Long Press Buttons");
                buf_xxx = get_center((uint8_t)strlen(tempTextU));
                writeTextToFrameBuffer(buf_xxx, 5, tempTextU, (uint32_t)strlen(tempTextU));	   
                
                write_2lines_mid(5);             
               
                
                
                memset(tempTextU, 0x00, sizeof(tempTextU));	
                
                sprintf(tempTextU, "%s", "<<NO            YES>>");
                buf_xxx = get_center((uint8_t)strlen(tempTextU));
                writeTextToFrameBuffer(buf_xxx, 7, tempTextU, (uint32_t)strlen(tempTextU));	
                
                
       sendToDisplay(0,2,displayBuffer[2], DISPLAY_NUMBER_OF_COLUMNS);
       sendToDisplay(0,3,displayBuffer[3], DISPLAY_NUMBER_OF_COLUMNS);
       sendToDisplay(0,4,displayBuffer[4], DISPLAY_NUMBER_OF_COLUMNS);
       sendToDisplay(0,5,displayBuffer[5], DISPLAY_NUMBER_OF_COLUMNS);
       sendToDisplay(0,6,displayBuffer[6], DISPLAY_NUMBER_OF_COLUMNS);
       sendToDisplay(0,7,displayBuffer[7], DISPLAY_NUMBER_OF_COLUMNS);

}


void settings_display(void){
        
      
     uint8_t cure_val_savedX = cure_val_saved;      
        
     if(menu_item == 0){
        cure_val_saved = read_encoder_XXX(1, 6, cure_val_savedX);
     }
     else if(menu_item == 20){
        cure_val_saved = read_encoder_XXX(1, 5, cure_val_savedX);
     }
      else if(menu_item == 21){
        cure_val_saved = read_encoder_XXX(1, 7, cure_val_savedX);
     }
     else if(menu_item == 22 || menu_item == 23){
        if(Cur_Time_pos == 0){
                cure_val_saved = read_encoder_XXX(0, 23, hhXXX);
                hhXXX = cure_val_saved;
        }else if(Cur_Time_pos == 1){
                cure_val_saved = read_encoder_XXX(0, 59, mmXXX);
                mmXXX = cure_val_saved;
        }else if(Cur_Time_pos == 2){
                cure_val_saved = read_encoder_XXX(0, 59, ssXXX);
                ssXXX = cure_val_saved;
        }else{
                //do nothing as it is ok
        }
        
     }
     else if(menu_item == 24){
        cure_val_saved = read_encoder_XXX(-12, 14, cure_val_savedX);
     }
     else if(menu_item == 30){
        cure_val_saved = read_encoder_XXX(1, 4, cure_val_savedX);
     }
     else if(menu_item == 31){
        cure_val_saved = read_encoder_XXX(1, 79, cure_val_savedX);
     }
    
     uint8_t button_D_stat = HAL_GPIO_ReadPin(ROT_ENCODE_D_PORT, ROT_ENCODE_D_PIN);
      
      if(button_D_stat == 0){
              if(is_butXaDS_valid > 5){
                      butD_stat_aaa++;
                      if(butD_stat_aaa > 5){
                              if(menu_item == 0){
                                        if(cure_val_saved == 1){
                                                menu_item = 10;
                                                is_butXaDS_valid = 0;
                                                cure_val_saved = 1;
                                        }
                                        else if(cure_val_saved == 2){
                                                menu_item = 20;
                                                is_butXaDS_valid = 0;
                                                cure_val_saved = 1;
                                        }
                                        else if(cure_val_saved == 3){
                                                menu_item = 30;
                                                is_butXaDS_valid = 0;
                                                cure_val_saved = 1;
                                        }
                                        else if(cure_val_saved == 4){
                                               // menu_item = 40;
                                               // is_butXaDS_valid = 0;
                                               // cure_val_saved = 1;
                                        }
                                        else if(cure_val_saved == 5){
                                                menu_item = 50;
                                                is_butXaDS_valid = 0;
                                                cure_val_saved = 1;
                                        }
                                        else if(cure_val_saved == 6){
                                                menu_item = 100;
                                                is_butXaDS_valid = 0;
                                                cure_val_saved = 1;
                                        }
                              }
                              else if(menu_item == 10){
                                      menu_item = 0;
                                      is_butXaDS_valid = 0;
                                      cure_val_saved = 1;
                              }
                              else if(menu_item == 20){
                                      if(cure_val_saved == 1){
                                                menu_item = 21;
                                                is_butXaDS_valid = 0;
                                                cure_val_saved = get_Time_CodeX();
                                        }
                                        else if(cure_val_saved == 2){
                                                menu_item = 22;
                                                is_butXaDS_valid = 0;
                                                cure_val_saved = 1;
                                        }
                                        else if(cure_val_saved == 3){
                                                menu_item = 23;
                                                is_butXaDS_valid = 0;
                                                cure_val_saved = 1;
                                        }
                                        else if(cure_val_saved == 4){
                                                menu_item = 24;
                                                is_butXaDS_valid = 0;
                                                cure_val_saved = 1;
                                        }
                                        else if(cure_val_saved == 5){
                                                menu_item = 0;
                                                is_butXaDS_valid = 0;
                                                cure_val_saved = 1;
                                        }
                              }
                              else if(menu_item == 21){//Time code choose
                                      set_TimeCode(cure_val_saved);
                                      menu_item = 0;
                                      is_butXaDS_valid = 0;
                                      cure_val_saved = 1;
                              }
                              else if(menu_item == 22){//Current time set
                                      if(Cur_Time_pos == 2){
                                              is_butXaDS_valid = 0;
                                              Cur_Time_pos = 0;
                                      }else{
                                             is_butXaDS_valid = 0;
                                             Cur_Time_pos++;
                                      }
                              }
                              else if(menu_item == 23){//GPS start time
                                      if(Cur_Time_pos == 2){
                                              is_butXaDS_valid = 0;
                                              Cur_Time_pos = 0;
                                      }else{
                                             is_butXaDS_valid = 0;
                                             Cur_Time_pos++;
                                      }
                              }
                              else if(menu_item == 24){
                                      //Save Time zone
                                      save_TimeZone(cure_val_saved);
                                      menu_item = 0;
                                      is_butXaDS_valid = 0;
                                      cure_val_saved = 1;
                              }
                               else if(menu_item == 30){
                                       if(cure_val_saved == 1){//reset wireless
                                               Wireless_reset_disp();
                                                menu_item = 0;
                                                is_butXaDS_valid = 0;
                                                cure_val_saved = 1;
                                                set_custom_433_wireles(0,0,0,0);//reset wireless only for this part and continue
                                                Config_Wir(0);
                                                HAL_Delay(15);
                                       }else if(cure_val_saved == 2){
                                                start_433_Mhz();
                                                menu_item = 31;
                                                cure_val_saved = get_433_chane();
                                                is_butXaDS_valid = 0;
                                       }else if(cure_val_saved == 3){
                                                menu_item = 32;
                                                is_butXaDS_valid = 0;
                                                cure_val_saved = 1;
                                       }else if(cure_val_saved == 4){
                                                is_butXaDS_valid = 0;
                                                menu_item = 0;
                                                is_butXaDS_valid = 0;
                                                cure_val_saved = 1;
                                       }
                              }
                               else if(menu_item == 31){
                                      menu_item = 0;
                                      is_butXaDS_valid = 0;
                                      cure_val_saved = 1;
                              }
                        }
        }
      }else{
        butD_stat_aaa = 0;
        if(is_butXaDS_valid < 6){
                is_butXaDS_valid++;
        }
      }
        
     if(menu_item == 0){
                settings_Main();
     }
     else if(menu_item == 10){
                set_display_AB_ID_choose();
     }
     else if(menu_item == 20){
                set_display_Time_choose();
     }
     else if(menu_item == 21){
                set_display_TimeCode_choose();
     }
     else if(menu_item == 22){
                set_display_TimeReal_choose();
     }
     else if(menu_item == 23){
                set_display_TimeStart_choose();
     }
     else if(menu_item == 24){
                set_display_TimeZone_choose();
     }
     else if(menu_item == 30){
                set_display_Wireless_choose();
     }
     else if(menu_item == 31){
                set_display_Wireless_LF_Set();
     }
     else if(menu_item == 32){
                set_display_Wireless_HF_Set();
     }
     else if(menu_item == 40){
                set_display_Audio_choose();
     }
     else if(menu_item == 50){
                set_display_Default_choose();
     }
     else if(menu_item == 100){
                set_display_Save_Exit_choose();
     }
             
      
}

void set_display_Default_choose(void){
                 uint8_t buf_xxx = 0;
      
      res_buf_line(0);
      res_buf_line(1);
      res_buf_line(2);
      res_buf_line(3);
      res_buf_line(4);
      res_buf_line(5);
      res_buf_line(6);
      res_buf_line(7);
        
                
      sprintf(tempTextU, "SETTINGS");
      buf_xxx = get_center((uint8_t)strlen(tempTextU));
   //   writeTextToFrameBuffer(buf_xxx, 0, tempTextU, (uint32_t)strlen(tempTextU));	   
                
      write_2lines_mid(0);
        
      memset(tempTextU, 0x00, sizeof(tempTextU));
      
 
      sprintf(tempTextU, "Default Settings");
      buf_xxx = get_center((uint8_t)strlen(tempTextU)); 
      writeTextToFrameBuffer(buf_xxx, 3, tempTextU, (uint32_t)strlen(tempTextU));	
      memset(tempTextU, 0x00, sizeof(tempTextU));
        
      
      sprintf(tempTextU, "PLEASE CONFIRM");
        
      buf_xxx = get_center((uint8_t)strlen(tempTextU));  
      writeTextToFrameBuffer(buf_xxx, 5, tempTextU, (uint32_t)strlen(tempTextU));	
      memset(tempTextU, 0x00, sizeof(tempTextU));
        
        
      memset(tempTextU, 0x00, sizeof(tempTextU));	

      sprintf(tempTextU, "<<EXIT           OK>>");
                
      buf_xxx = get_center((uint8_t)strlen(tempTextU));
      writeTextToFrameBuffer(buf_xxx, 7, tempTextU, (uint32_t)strlen(tempTextU));	
                
               
       sendToDisplay(0,0,displayBuffer[0], DISPLAY_NUMBER_OF_COLUMNS);
       sendToDisplay(0,1,displayBuffer[1], DISPLAY_NUMBER_OF_COLUMNS);
       sendToDisplay(0,2,displayBuffer[2], DISPLAY_NUMBER_OF_COLUMNS);
       sendToDisplay(0,3,displayBuffer[3], DISPLAY_NUMBER_OF_COLUMNS);
       sendToDisplay(0,4,displayBuffer[4], DISPLAY_NUMBER_OF_COLUMNS);
       sendToDisplay(0,5,displayBuffer[5], DISPLAY_NUMBER_OF_COLUMNS);
       sendToDisplay(0,6,displayBuffer[6], DISPLAY_NUMBER_OF_COLUMNS);
       sendToDisplay(0,7,displayBuffer[7], DISPLAY_NUMBER_OF_COLUMNS);
       
}

void set_display_Save_Exit_choose(void){
         uint8_t buf_xxx = 0;
      
      res_buf_line(0);
      res_buf_line(1);
      res_buf_line(2);
      res_buf_line(3);
      res_buf_line(4);
      res_buf_line(5);
      res_buf_line(6);
      res_buf_line(7);
        
                
      sprintf(tempTextU, "SETTINGS");
      buf_xxx = get_center((uint8_t)strlen(tempTextU));
   //   writeTextToFrameBuffer(buf_xxx, 0, tempTextU, (uint32_t)strlen(tempTextU));	   
                
      write_2lines_mid(0);
        
      memset(tempTextU, 0x00, sizeof(tempTextU));
      
 
      sprintf(tempTextU, "SAVING");
      buf_xxx = get_center((uint8_t)strlen(tempTextU)); 
      writeTextToFrameBuffer(buf_xxx, 3, tempTextU, (uint32_t)strlen(tempTextU));	
      memset(tempTextU, 0x00, sizeof(tempTextU));
        
      
      sprintf(tempTextU, "PLEASE WAIT");
        
      buf_xxx = get_center((uint8_t)strlen(tempTextU));  
      writeTextToFrameBuffer(buf_xxx, 5, tempTextU, (uint32_t)strlen(tempTextU));	
      memset(tempTextU, 0x00, sizeof(tempTextU));
        
        

                
               
       sendToDisplay(0,0,displayBuffer[0], DISPLAY_NUMBER_OF_COLUMNS);
       sendToDisplay(0,1,displayBuffer[1], DISPLAY_NUMBER_OF_COLUMNS);
       sendToDisplay(0,2,displayBuffer[2], DISPLAY_NUMBER_OF_COLUMNS);
       sendToDisplay(0,3,displayBuffer[3], DISPLAY_NUMBER_OF_COLUMNS);
       sendToDisplay(0,4,displayBuffer[4], DISPLAY_NUMBER_OF_COLUMNS);
       sendToDisplay(0,5,displayBuffer[5], DISPLAY_NUMBER_OF_COLUMNS);
       sendToDisplay(0,6,displayBuffer[6], DISPLAY_NUMBER_OF_COLUMNS);
       sendToDisplay(0,7,displayBuffer[7], DISPLAY_NUMBER_OF_COLUMNS);
       
       
        save_script_flash();
        set_AD_ID = 0;

}



void set_display_Time_choose(void){

      res_buf_line(0);
      res_buf_line(1);
      res_buf_line(2);
      res_buf_line(3);
      res_buf_line(4);
      res_buf_line(5);
      res_buf_line(6);
      res_buf_line(7);
        
                
      sprintf(tempTextU, "TIME SETTINGS");
                
      write_2lines_mid(0);
      
        if(cure_val_saved == 1){
                sprintf(tempTextU, ">TimeCode");
        }else{
                sprintf(tempTextU, " TimeCode");
        }
        
        writeTextToFrameBuffer(0, 2, tempTextU, (uint32_t)strlen(tempTextU));	
        
        memset(tempTextU, 0x00, sizeof(tempTextU));
        
        if(cure_val_saved == 2){
                sprintf(tempTextU, ">SetCurrentTime");
        }else{
                sprintf(tempTextU, " SetCurrentTime");
        }
        
        writeTextToFrameBuffer(0, 3, tempTextU, (uint32_t)strlen(tempTextU));	
        
        memset(tempTextU, 0x00, sizeof(tempTextU));

        if(cure_val_saved == 3){
                sprintf(tempTextU, ">SetStartTime");
        }else{
                sprintf(tempTextU, " SetStartTime");
        }
        
        writeTextToFrameBuffer(0, 4, tempTextU, (uint32_t)strlen(tempTextU));	
      
        memset(tempTextU, 0x00, sizeof(tempTextU));        
        

        if(cure_val_saved == 4){
                sprintf(tempTextU, ">TimeZone");
        }else{
                sprintf(tempTextU, " TimeZone");
        }
        
       writeTextToFrameBuffer(0, 5, tempTextU, (uint32_t)strlen(tempTextU));	
        
                
       memset(tempTextU, 0x00, sizeof(tempTextU));	             
 

        if(cure_val_saved == 5){
                sprintf(tempTextU, ">Back");
        }else{
                sprintf(tempTextU, " Back");
        }
        
       writeTextToFrameBuffer(0, 6, tempTextU, (uint32_t)strlen(tempTextU));	
        
                
       memset(tempTextU, 0x00, sizeof(tempTextU));	             
                
               
       sendToDisplay(0,0,displayBuffer[0], DISPLAY_NUMBER_OF_COLUMNS);
       sendToDisplay(0,1,displayBuffer[1], DISPLAY_NUMBER_OF_COLUMNS);
       sendToDisplay(0,2,displayBuffer[2], DISPLAY_NUMBER_OF_COLUMNS);
       sendToDisplay(0,3,displayBuffer[3], DISPLAY_NUMBER_OF_COLUMNS);
       sendToDisplay(0,4,displayBuffer[4], DISPLAY_NUMBER_OF_COLUMNS);
       sendToDisplay(0,5,displayBuffer[5], DISPLAY_NUMBER_OF_COLUMNS);
       sendToDisplay(0,6,displayBuffer[6], DISPLAY_NUMBER_OF_COLUMNS);
       sendToDisplay(0,7,displayBuffer[7], DISPLAY_NUMBER_OF_COLUMNS);
}

void set_display_Audio_choose(void){

}


void set_display_Wireless_LF_Set(void){
        
                uint8_t buf_xxx = 0;
        
                res_buf_line(2);
                res_buf_line(3);
                res_buf_line(4);
                res_buf_line(5);
                res_buf_line(6);
                res_buf_line(7);
                
                
                sprintf(tempTextU, "CHOOSE LF CHANNEL");
                buf_xxx = get_center((uint8_t)strlen(tempTextU));
                writeTextToFrameBuffer(buf_xxx, 2, tempTextU, (uint32_t)strlen(tempTextU));	   
                
                write_2lines_mid(2);
                
                memset(tempTextU, 0x00, sizeof(tempTextU));	             
                
                
             
             sprintf(tempTextU, "CHANNEL:%d  %dMHz", cure_val_saved, (850 + cure_val_saved));
                
                buf_xxx = get_center((uint8_t)strlen(tempTextU));
                writeTextToFrameBuffer(buf_xxx, 4, tempTextU, (uint32_t)strlen(tempTextU));	   
                

                memset(tempTextU, 0x00, sizeof(tempTextU));	             
                             
                Time_Blink_timer++;             

                if( Time_Blink_timer > 150){
                        
                        Time_Blink_timer = 0;
                        if(cure_val_saved != get_433_chane()){
                                set_new_chan(cure_val_saved);
                        }else{
                                send_noise_msg();
                        }
                }
                w433_task();
                
                
                sprintf(tempTextU, "CHANNEL NOISE: %d", get_mod_test_noise());
                buf_xxx = get_center((uint8_t)strlen(tempTextU));
                writeTextToFrameBuffer(buf_xxx, 5, tempTextU, (uint32_t)strlen(tempTextU));	   
                
                write_2lines_mid(5);             
               
                
                
                memset(tempTextU, 0x00, sizeof(tempTextU));	
                /*
                sprintf(tempTextU, "%s", "<<EXIT(-)   (+)SAVE>>");
                buf_xxx = get_center((uint8_t)strlen(tempTextU));
                writeTextToFrameBuffer(buf_xxx, 7, tempTextU, (uint32_t)strlen(tempTextU));	
                */
                
                sendToDisplay(0, 2, displayBuffer[2], DISPLAY_NUMBER_OF_COLUMNS);	
                sendToDisplay(0, 3, displayBuffer[3], DISPLAY_NUMBER_OF_COLUMNS);
                sendToDisplay(0, 4, displayBuffer[4], DISPLAY_NUMBER_OF_COLUMNS);
                sendToDisplay(0, 5, displayBuffer[5], DISPLAY_NUMBER_OF_COLUMNS);
                sendToDisplay(0, 6, displayBuffer[6], DISPLAY_NUMBER_OF_COLUMNS);
                sendToDisplay(0, 7, displayBuffer[7], DISPLAY_NUMBER_OF_COLUMNS);


}

void set_display_Wireless_HF_Set(void){

}

void Wireless_reset_disp(void){
res_buf_line(2);
      res_buf_line(3);
      res_buf_line(4);
      res_buf_line(5);
      res_buf_line(6);
      res_buf_line(7);
        

      
        sprintf(tempTextU, "Wireless reset");


       write_2lines_mid(3);
        
        memset(tempTextU, 0x00, sizeof(tempTextU));
        
        

        
       sprintf(tempTextU, "Please wait");
        
        
       write_2lines_mid(6);
        
               
                
       memset(tempTextU, 0x00, sizeof(tempTextU));	             
               
       
       sendToDisplay(0,2,displayBuffer[2], DISPLAY_NUMBER_OF_COLUMNS);
       sendToDisplay(0,3,displayBuffer[3], DISPLAY_NUMBER_OF_COLUMNS);
       sendToDisplay(0,4,displayBuffer[4], DISPLAY_NUMBER_OF_COLUMNS);
       sendToDisplay(0,5,displayBuffer[5], DISPLAY_NUMBER_OF_COLUMNS);
       sendToDisplay(0,6,displayBuffer[6], DISPLAY_NUMBER_OF_COLUMNS);
       sendToDisplay(0,7,displayBuffer[7], DISPLAY_NUMBER_OF_COLUMNS);

}

void set_display_Wireless_choose(void){
        
      res_buf_line(2);
      res_buf_line(3);
      res_buf_line(4);
      res_buf_line(5);
      res_buf_line(6);
      res_buf_line(7);
        

      
        sprintf(tempTextU, "Wireless");


       write_2lines_mid(2);
        
        memset(tempTextU, 0x00, sizeof(tempTextU));
        
        if(cure_val_saved == 1){
                sprintf(tempTextU, ">Reset to default");
        }else{
                sprintf(tempTextU, " Reset to default");
        }
        
        writeTextToFrameBuffer(0, 4, tempTextU, (uint32_t)strlen(tempTextU));	
        
        memset(tempTextU, 0x00, sizeof(tempTextU));

        if(cure_val_saved == 2){
                sprintf(tempTextU, ">LF Settings");
        }else{
                sprintf(tempTextU, " LF Settings");
        }
        
        writeTextToFrameBuffer(0, 5, tempTextU, (uint32_t)strlen(tempTextU));	
      
        memset(tempTextU, 0x00, sizeof(tempTextU));        
        

        if(cure_val_saved == 3){
                sprintf(tempTextU, ">HF Settings");
        }else{
                sprintf(tempTextU, " HF Settings");
        }
        
       writeTextToFrameBuffer(0, 6, tempTextU, (uint32_t)strlen(tempTextU));	
        
                
       memset(tempTextU, 0x00, sizeof(tempTextU));	             
 

        if(cure_val_saved == 4){
                sprintf(tempTextU, ">Back");
        }else{
                sprintf(tempTextU, " Back");
        }
        
       writeTextToFrameBuffer(0, 7, tempTextU, (uint32_t)strlen(tempTextU));	
        
               
                
       memset(tempTextU, 0x00, sizeof(tempTextU));	             
               
       
       sendToDisplay(0,2,displayBuffer[2], DISPLAY_NUMBER_OF_COLUMNS);
       sendToDisplay(0,3,displayBuffer[3], DISPLAY_NUMBER_OF_COLUMNS);
       sendToDisplay(0,4,displayBuffer[4], DISPLAY_NUMBER_OF_COLUMNS);
       sendToDisplay(0,5,displayBuffer[5], DISPLAY_NUMBER_OF_COLUMNS);
       sendToDisplay(0,6,displayBuffer[6], DISPLAY_NUMBER_OF_COLUMNS);
       sendToDisplay(0,7,displayBuffer[7], DISPLAY_NUMBER_OF_COLUMNS);


}
        
      
void settings_Main(void){
      
      res_buf_line(0);
      res_buf_line(1);
      res_buf_line(2);
      res_buf_line(3);
      res_buf_line(4);
      res_buf_line(5);
      res_buf_line(6);
      res_buf_line(7);
        
                
      sprintf(tempTextU, "SETTINGS");
                
      write_2lines_mid(0);
      
        if(cure_val_saved == 1){
                sprintf(tempTextU, ">Set AB ID");
        }else{
                sprintf(tempTextU, " Set AB ID");
        }
        
        writeTextToFrameBuffer(0, 2, tempTextU, (uint32_t)strlen(tempTextU));	
        
        memset(tempTextU, 0x00, sizeof(tempTextU));
        
        if(cure_val_saved == 2){
                sprintf(tempTextU, ">Time");
        }else{
                sprintf(tempTextU, " Time");
        }
        
        writeTextToFrameBuffer(0, 3, tempTextU, (uint32_t)strlen(tempTextU));	
        
        memset(tempTextU, 0x00, sizeof(tempTextU));

        if(cure_val_saved == 3){
                sprintf(tempTextU, ">Wireless");
        }else{
                sprintf(tempTextU, " Wireless");
        }
        
        writeTextToFrameBuffer(0, 4, tempTextU, (uint32_t)strlen(tempTextU));	
      
        memset(tempTextU, 0x00, sizeof(tempTextU));        
        

        if(cure_val_saved == 4){
                sprintf(tempTextU, ">Audio");
        }else{
                sprintf(tempTextU, " Audio");
        }
        
       writeTextToFrameBuffer(0, 5, tempTextU, (uint32_t)strlen(tempTextU));	
        
                
       memset(tempTextU, 0x00, sizeof(tempTextU));	             
 

        if(cure_val_saved == 5){
                sprintf(tempTextU, ">DefaultSettings");
        }else{
                sprintf(tempTextU, " DefaultSettings");
        }
        
       writeTextToFrameBuffer(0, 6, tempTextU, (uint32_t)strlen(tempTextU));	
        
                
       memset(tempTextU, 0x00, sizeof(tempTextU));	             
                
        if(cure_val_saved == 6){
                sprintf(tempTextU, ">Save&Exit");
        }else{
                sprintf(tempTextU, " Save&Exit");
        }
        
       writeTextToFrameBuffer(0, 7, tempTextU, (uint32_t)strlen(tempTextU));	
        
                
       memset(tempTextU, 0x00, sizeof(tempTextU));	             
               
       sendToDisplay(0,0,displayBuffer[0], DISPLAY_NUMBER_OF_COLUMNS);
       sendToDisplay(0,1,displayBuffer[1], DISPLAY_NUMBER_OF_COLUMNS);
       sendToDisplay(0,2,displayBuffer[2], DISPLAY_NUMBER_OF_COLUMNS);
       sendToDisplay(0,3,displayBuffer[3], DISPLAY_NUMBER_OF_COLUMNS);
       sendToDisplay(0,4,displayBuffer[4], DISPLAY_NUMBER_OF_COLUMNS);
       sendToDisplay(0,5,displayBuffer[5], DISPLAY_NUMBER_OF_COLUMNS);
       sendToDisplay(0,6,displayBuffer[6], DISPLAY_NUMBER_OF_COLUMNS);
       sendToDisplay(0,7,displayBuffer[7], DISPLAY_NUMBER_OF_COLUMNS);
}



void set_display_AB_ID_choose(void){
        
                uint8_t buf_xxx = 0;
        
                res_buf_line(2);
                res_buf_line(3);
                res_buf_line(4);
                res_buf_line(5);
                res_buf_line(6);
                res_buf_line(7);
        
        
                static uint8_t cure_val_savedAAA = 1;      
                
                cure_val_savedAAA = read_encoder_XXX(MIN_AB_ID, MAX_AB_ID, adress_interface());
        
                set_dev_address(cure_val_savedAAA);
                
                
                sprintf(tempTextU, "%s", "Set MusicSyncBox ID");
                buf_xxx = get_center((uint8_t)strlen(tempTextU));
              //  writeTextToFrameBuffer(buf_xxx, 2, tempTextU, (uint32_t)strlen(tempTextU));	   
                
                write_2lines_mid(2);
                
                memset(tempTextU, 0x00, sizeof(tempTextU));	             
 

                
                if(get_Music_box_ID() == MAX_AB_ID){
                        sprintf(tempTextU, "       MASTER        ");
                }
                else{
                        sprintf(tempTextU, "         A%d          ", get_Music_box_ID());
                }
             
                
                buf_xxx = get_center((uint8_t)strlen(tempTextU));
                writeTextToFrameBuffer(buf_xxx, 5, tempTextU, (uint32_t)strlen(tempTextU));	
                
               
       sendToDisplay(0,2,displayBuffer[2], DISPLAY_NUMBER_OF_COLUMNS);
       sendToDisplay(0,3,displayBuffer[3], DISPLAY_NUMBER_OF_COLUMNS);
       sendToDisplay(0,4,displayBuffer[4], DISPLAY_NUMBER_OF_COLUMNS);
       sendToDisplay(0,5,displayBuffer[5], DISPLAY_NUMBER_OF_COLUMNS);
       sendToDisplay(0,6,displayBuffer[6], DISPLAY_NUMBER_OF_COLUMNS);
       sendToDisplay(0,7,displayBuffer[7], DISPLAY_NUMBER_OF_COLUMNS);
}


void set_display_TimeCode_choose(void){
       uint8_t buf_xxx = 0;
      
      res_buf_line(0);
      res_buf_line(1);
      res_buf_line(2);
      res_buf_line(3);
      res_buf_line(4);
      res_buf_line(5);
      res_buf_line(6);
      res_buf_line(7);
        
                
      sprintf(tempTextU, "TIME CODE SETTINGS");
      buf_xxx = get_center((uint8_t)strlen(tempTextU));
    //  writeTextToFrameBuffer(buf_xxx, 0, tempTextU, (uint32_t)strlen(tempTextU));	   
                
      write_2lines_mid(0);
        
        
        if(cure_val_saved == 1){
                sprintf(tempTextU, "Read SMPTE 25/30fps");
        }else if(cure_val_saved == 2){
                sprintf(tempTextU, "Read FSK PD");
        }else if(cure_val_saved == 3){
                sprintf(tempTextU, "Read FSK F1");
        }else if(cure_val_saved == 4){
                sprintf(tempTextU, "Generate SMPTE 25fps");
        }else if(cure_val_saved == 5){
                sprintf(tempTextU, "Generate SMPTE 30fps");
        }else if(cure_val_saved == 6){
                sprintf(tempTextU, "Generate FSK PD");
        }else if(cure_val_saved == 7){
                sprintf(tempTextU, "Generate FSK F1");
        }
        
        
        buf_xxx = get_center((uint8_t)strlen(tempTextU));
        writeTextToFrameBuffer(buf_xxx, 3, tempTextU, (uint32_t)strlen(tempTextU));	
      
        memset(tempTextU, 0x00, sizeof(tempTextU));        
      
        
        sprintf(tempTextU, "Rotate for choose");
         buf_xxx = get_center((uint8_t)strlen(tempTextU));
        
        writeTextToFrameBuffer(buf_xxx, 5, tempTextU, (uint32_t)strlen(tempTextU));	
        
        memset(tempTextU, 0x00, sizeof(tempTextU));
        
        sprintf(tempTextU, "Push for select");
         buf_xxx = get_center((uint8_t)strlen(tempTextU));
        
        writeTextToFrameBuffer(buf_xxx, 6, tempTextU, (uint32_t)strlen(tempTextU));
        write_2lines_mid(6);	
        
        memset(tempTextU, 0x00, sizeof(tempTextU));

          
                
               
       sendToDisplay(0,0,displayBuffer[0], DISPLAY_NUMBER_OF_COLUMNS);
       sendToDisplay(0,1,displayBuffer[1], DISPLAY_NUMBER_OF_COLUMNS);
       sendToDisplay(0,2,displayBuffer[2], DISPLAY_NUMBER_OF_COLUMNS);
       sendToDisplay(0,3,displayBuffer[3], DISPLAY_NUMBER_OF_COLUMNS);
       sendToDisplay(0,4,displayBuffer[4], DISPLAY_NUMBER_OF_COLUMNS);
       sendToDisplay(0,5,displayBuffer[5], DISPLAY_NUMBER_OF_COLUMNS);
       sendToDisplay(0,6,displayBuffer[6], DISPLAY_NUMBER_OF_COLUMNS);
       sendToDisplay(0,7,displayBuffer[7], DISPLAY_NUMBER_OF_COLUMNS);

}
void set_display_TimeReal_choose(void){
        
            
                uint8_t buf_xxx = 0;
        
                res_buf_line(2);
                res_buf_line(3);
                res_buf_line(4);
                res_buf_line(5);
                res_buf_line(6);
                res_buf_line(7);
        
        

                
                sprintf(tempTextU, "%s", "Set Current Time");
                buf_xxx = get_center((uint8_t)strlen(tempTextU));
          //      writeTextToFrameBuffer(buf_xxx, 2, tempTextU, (uint32_t)strlen(tempTextU));	   
                
                write_2lines_mid(2);
                
                memset(tempTextU, 0x00, sizeof(tempTextU));	             
 

                
               // sprintf(tempTextU, "Use knob wheel");
               if(timer_tmp == 0){
                        timer_tmp = get_real_time();
                        calculate_time(timer_tmp);
                }
                
                
               Time_Blink_timer++;
               if(Cur_Time_pos == 0){
                       if(Time_Blink_timer < 75){
                                sprintf(tempTextU, "%02d:%02d:%02d",hhXXX,mmXXX,ssXXX);
                       }else{
                                sprintf(tempTextU, "  :%02d:%02d",mmXXX,ssXXX);
                       }
               }else if(Cur_Time_pos == 1){
                       if(Time_Blink_timer < 75){
                                sprintf(tempTextU, "%02d:%02d:%02d",hhXXX,mmXXX,ssXXX);
                       }else{
                                sprintf(tempTextU, "%02d:  :%02d",hhXXX,ssXXX);
                       }
               }else if(Cur_Time_pos == 2){
                       if(Time_Blink_timer < 75){
                                sprintf(tempTextU, "%02d:%02d:%02d",hhXXX,mmXXX,ssXXX);
                       }else{
                                sprintf(tempTextU, "%02d:%02d:  ",hhXXX,mmXXX);
                       }
               }
               
               if(Time_Blink_timer > 100){
                        Time_Blink_timer = 0;
               }
             
               
               
                buf_xxx = get_center((uint8_t)strlen(tempTextU));
                writeTextToFrameBuffer(buf_xxx, 5, tempTextU, (uint32_t)strlen(tempTextU));	   
             
                
                write_2lines_mid(4);             
               
              
                
                memset(tempTextU, 0x00, sizeof(tempTextU));	

                sprintf(tempTextU, "<<EXIT         SAVE>>");
                
                buf_xxx = get_center((uint8_t)strlen(tempTextU));
                writeTextToFrameBuffer(buf_xxx, 7, tempTextU, (uint32_t)strlen(tempTextU));	
                
               
       sendToDisplay(0,2,displayBuffer[2], DISPLAY_NUMBER_OF_COLUMNS);
       sendToDisplay(0,3,displayBuffer[3], DISPLAY_NUMBER_OF_COLUMNS);
       sendToDisplay(0,4,displayBuffer[4], DISPLAY_NUMBER_OF_COLUMNS);
       sendToDisplay(0,5,displayBuffer[5], DISPLAY_NUMBER_OF_COLUMNS);
       sendToDisplay(0,6,displayBuffer[6], DISPLAY_NUMBER_OF_COLUMNS);
       sendToDisplay(0,7,displayBuffer[7], DISPLAY_NUMBER_OF_COLUMNS);

}
void set_display_TimeStart_choose(void){
                    
                uint8_t buf_xxx = 0;
        
                char bufferDispX[21] = {0};
        
                res_buf_line(2);
                res_buf_line(3);
                res_buf_line(4);
                res_buf_line(5);
                res_buf_line(6);
                res_buf_line(7);
        
        
               
                sprintf(tempTextU, "Set Start Time");
                buf_xxx = get_center((uint8_t)strlen(tempTextU));
         //       writeTextToFrameBuffer(buf_xxx, 2, tempTextU, (uint32_t)strlen(tempTextU));	   
                
                write_2lines_mid(2);
                
                memset(tempTextU, 0x00, sizeof(tempTextU));	             
 
 /*
                memset(bufferDispX, 0x00, sizeof(tempTextU));


                sprintf(tempTextU, "C>23:12:45 S>");
*/
               
                if(timer_tmp == 0){
                        timer_tmp = get_real_time();
                        calculate_time(timer_tmp);
                }
                
                
               Time_Blink_timer++;
               if(Cur_Time_pos == 0){
                       if(Time_Blink_timer < 75){
                                sprintf(bufferDispX, "%02d:%02d:%02d",hhXXX,mmXXX,ssXXX);
                       }else{
                                sprintf(bufferDispX, "  :%02d:%02d",mmXXX,ssXXX);
                       }
               }else if(Cur_Time_pos == 1){
                       if(Time_Blink_timer < 75){
                                sprintf(bufferDispX, "%02d:%02d:%02d",hhXXX,mmXXX,ssXXX);
                       }else{
                                sprintf(tempTextU, "%02d:  :%02d",hhXXX,ssXXX);
                       }
               }else if(Cur_Time_pos == 2){
                       if(Time_Blink_timer < 75){
                                sprintf(bufferDispX, "%02d:%02d:%02d",hhXXX,mmXXX,ssXXX);
                       }else{
                                sprintf(bufferDispX, "%02d:%02d:  ",hhXXX,mmXXX);
                       }
               }
               
               if(Time_Blink_timer > 100){
                        Time_Blink_timer = 0;
               }
             
                strcat(tempTextU, bufferDispX);
               
               
                buf_xxx = get_center((uint8_t)strlen(tempTextU));
                writeTextToFrameBuffer(buf_xxx, 5, tempTextU, (uint32_t)strlen(tempTextU));	   
             
                
                write_2lines_mid(4);             
               
              
                
                memset(tempTextU, 0x00, sizeof(tempTextU));	

                sprintf(tempTextU, "<<DISABLE    ENABLE>>");
                
                buf_xxx = get_center((uint8_t)strlen(tempTextU));
                writeTextToFrameBuffer(buf_xxx, 7, tempTextU, (uint32_t)strlen(tempTextU));	
                
               
       sendToDisplay(0,2,displayBuffer[2], DISPLAY_NUMBER_OF_COLUMNS);
       sendToDisplay(0,3,displayBuffer[3], DISPLAY_NUMBER_OF_COLUMNS);
       sendToDisplay(0,4,displayBuffer[4], DISPLAY_NUMBER_OF_COLUMNS);
       sendToDisplay(0,5,displayBuffer[5], DISPLAY_NUMBER_OF_COLUMNS);
       sendToDisplay(0,6,displayBuffer[6], DISPLAY_NUMBER_OF_COLUMNS);
       sendToDisplay(0,7,displayBuffer[7], DISPLAY_NUMBER_OF_COLUMNS);

}

void set_display_TimeZone_choose(void){

                        uint8_t buf_xxx = 0;

        
                res_buf_line(2);
                res_buf_line(3);
                res_buf_line(4);
                res_buf_line(5);
                res_buf_line(6);
                res_buf_line(7);
        
        
                
                
                sprintf(tempTextU, "Set Time Zone");
                buf_xxx = get_center((uint8_t)strlen(tempTextU));
         //       writeTextToFrameBuffer(buf_xxx, 2, tempTextU, (uint32_t)strlen(tempTextU));	   
                
                write_2lines_mid(2);
                
                memset(tempTextU, 0x00, sizeof(tempTextU));	             
 

                if(timer_tmp == 0){
                        timer_tmp = 1;
                        cure_val_saved = get_time_zone();
                }
                
                if(cure_val_saved == 0){
                        sprintf(tempTextU, "TimeZone:  0");
                }else if(cure_val_saved < 0){
                        sprintf(tempTextU, "TimeZone:-%02d", (-cure_val_saved));
                }else if(cure_val_saved > 0){
                        sprintf(tempTextU, "TimeZone:+%02d", cure_val_saved);
                }
                
               
               
                buf_xxx = get_center((uint8_t)strlen(tempTextU));
                writeTextToFrameBuffer(buf_xxx, 5, tempTextU, (uint32_t)strlen(tempTextU));	   
             
                
                write_2lines_mid(4);             
               
              
                
                memset(tempTextU, 0x00, sizeof(tempTextU));	
                writeTextToFrameBuffer(buf_xxx, 7, tempTextU, (uint32_t)strlen(tempTextU));	
                
               
       sendToDisplay(0,2,displayBuffer[2], DISPLAY_NUMBER_OF_COLUMNS);
       sendToDisplay(0,3,displayBuffer[3], DISPLAY_NUMBER_OF_COLUMNS);
       sendToDisplay(0,4,displayBuffer[4], DISPLAY_NUMBER_OF_COLUMNS);
       sendToDisplay(0,5,displayBuffer[5], DISPLAY_NUMBER_OF_COLUMNS);
       sendToDisplay(0,6,displayBuffer[6], DISPLAY_NUMBER_OF_COLUMNS);
       sendToDisplay(0,7,displayBuffer[7], DISPLAY_NUMBER_OF_COLUMNS);
        
}


void set_LCD_Line(uint8_t tmp_trs){

}



void calculate_time(uint32_t time_to_calc){
    hhXXX = 0;
    mmXXX = 0;
    ssXXX = 0;
        
    int sec = time_to_calc/1000;
    ssXXX = sec % 60;
    mmXXX = sec / 60;
    if (mmXXX >= 60) {
        hhXXX = mmXXX / 60;
        mmXXX %= 60;
    }
}



void timer_saving(uint8_t is_saving){
        if(is_saving == 1){
                save_time_new(hhXXX, mmXXX, ssXXX);
        }
        timer_tmp = 0;
        menu_item = 0;
        cure_val_saved = 2;
        Cur_Time_pos = 0;
}

void timer_Start_saving(uint8_t is_saving){
        if(is_saving == 1){//enable Start time
                isGPSFTValid = 1;
                GPS_SetValue = 1000 * 60 * 60 * hhXXX + 1000 * 60 * mmXXX + 1000 * ssXXX;
                set_dev_address(MAX_AB_ID);
                Time_Code_disable = 0;
        }else{//disable
                isGPSFTValid = 0;
                GPS_SetValue = 0;
        }
        save_Start_time_Config();
        timer_tmp = 0;
        menu_item = 0;
        cure_val_saved = 2;
        Cur_Time_pos = 0;
}

void go_Back_main(void){
        timer_tmp = 0;
        menu_item = 0;
        cure_val_saved = 1;
        Cur_Time_pos = 0;
}


uint8_t get_menu_item(void){
        return menu_item;
}

void Wireless_config_Update(void){
        
                uint8_t buf_xxx = 0;
        
                res_buf_line(2);
                res_buf_line(3);
                res_buf_line(4);
                res_buf_line(5);
                res_buf_line(6);
                res_buf_line(7);
                
                
                 if(was_WirConfigRead == 1 || was_WirConfigRead == 2){
                        res_buf_line(2);
                        res_buf_line(3);
                        res_buf_line(4);
                        res_buf_line(5);
                        res_buf_line(6);
                        res_buf_line(7);
                        
                        sprintf(tempTextU, "%s", "Wireless Key ERROR");
                        write_2lines_mid(2);
                        
                        memset(tempTextU, 0x00, sizeof(tempTextU));	
                        sprintf(tempTextU, "%s", "Remake KEY File");
                        buf_xxx = get_center((uint8_t)strlen(tempTextU));
                        writeTextToFrameBuffer(buf_xxx, 4, tempTextU, (uint32_t)strlen(tempTextU));	   
                        
                        
                        memset(tempTextU, 0x00, sizeof(tempTextU));	
                        sprintf(tempTextU, "%s", "Please Restart");
                        buf_xxx = get_center((uint8_t)strlen(tempTextU));
                        writeTextToFrameBuffer(buf_xxx, 7, tempTextU, (uint32_t)strlen(tempTextU));
                
                }
                else if(was_WirConfigRead == 10){
                        res_buf_line(2);
                        res_buf_line(3);
                        res_buf_line(4);
                        res_buf_line(5);
                        res_buf_line(6);
                        res_buf_line(7);
                        
                        sprintf(tempTextU, "%s", "Updating Wireless Key");
                        write_2lines_mid(2);
                        
                        memset(tempTextU, 0x00, sizeof(tempTextU));	
                        sprintf(tempTextU, "%s", "Please wait");
                        write_2lines_mid(5); 

                }
                else if(was_WirConfigRead == 100){
                        res_buf_line(2);
                        res_buf_line(3);
                        res_buf_line(4);
                        res_buf_line(5);
                        res_buf_line(6);
                        res_buf_line(7);
                        
                        sprintf(tempTextU, "%s", "Wireless Key Updated");
                        write_2lines_mid(2);
                        
                        memset(tempTextU, 0x00, sizeof(tempTextU));	
                        sprintf(tempTextU, "%s", "Press any key");
                        write_2lines_mid(5); 
                }
                
       sendToDisplay(0,2,displayBuffer[2], DISPLAY_NUMBER_OF_COLUMNS);
       sendToDisplay(0,3,displayBuffer[3], DISPLAY_NUMBER_OF_COLUMNS);
       sendToDisplay(0,4,displayBuffer[4], DISPLAY_NUMBER_OF_COLUMNS);
       sendToDisplay(0,5,displayBuffer[5], DISPLAY_NUMBER_OF_COLUMNS);
       sendToDisplay(0,6,displayBuffer[6], DISPLAY_NUMBER_OF_COLUMNS);
       sendToDisplay(0,7,displayBuffer[7], DISPLAY_NUMBER_OF_COLUMNS);

}



//USB drive script load
void USB_Drive_menu (uint8_t USB_Stick_menu)
{
        uint8_t buf_xxx = 0;
        
        switch (USB_Stick_menu) {
        case 1: 	//USB Drive Not supported
                res_buf_line(2);
                res_buf_line(3);
                res_buf_line(4);
                res_buf_line(5);
                res_buf_line(6);
                res_buf_line(7);
                
                
                sprintf(tempTextU, "SCRIPT UPLOAD");
                write_2lines_mid(2);
                
                memset(tempTextU, 0x00, sizeof(tempTextU));	
                sprintf(tempTextU, "PLEASE WAIT");
                write_2lines_mid(4);    
                
                break;

        case 2: // upload finish OK	
                res_buf_line(2);
                res_buf_line(3);
                res_buf_line(4);
                res_buf_line(5);
                res_buf_line(6);
                res_buf_line(7);
                
                sprintf(tempTextU, "UPLOAD COMPLETE - ALL");
                write_2lines_mid(2);
                
                memset(tempTextU, 0x00, sizeof(tempTextU));
/*	
                for(int i = 0; i< 20; i++)
                {
                        tempTextU[i] = tmp_prg_name[i];
                }
        */
                buf_xxx = get_center((uint8_t)strlen(tempTextU));
                writeTextToFrameBuffer(buf_xxx, 4, tempTextU, (uint32_t)strlen(tempTextU));	   
                
                
                memset(tempTextU, 0x00, sizeof(tempTextU));	
                sprintf(tempTextU, "DMX Events:%d", DMX_ev);
                write_2lines_mid(5);  
                
                
                
                memset(tempTextU, 0x00, sizeof(tempTextU));	
                sprintf(tempTextU, "%s", "REMOVE USB DRIVE");
                buf_xxx = get_center((uint8_t)strlen(tempTextU));
                writeTextToFrameBuffer(buf_xxx, 7, tempTextU, (uint32_t)strlen(tempTextU));	              
                
                
                
                
                //Upload complete
                // program name
                //number of sequences: xxx
                //Total events: xx,  Pyro ev: xxx DMX : xxx,
                // remove the USB drive
                break;
                
                
                
        case 10: // not enough memory. script too big
                res_buf_line(2);
                res_buf_line(3);
                res_buf_line(4);
                res_buf_line(5);
                res_buf_line(6);
                res_buf_line(7);
                
                sprintf(tempTextU, "%s", "UPLOAD ERROR");
                write_2lines_mid(2);
                
                memset(tempTextU, 0x00, sizeof(tempTextU));	
                sprintf(tempTextU, "%s", "TOO MANY EVENTS");
                write_2lines_mid(4); 
                
                
                memset(tempTextU, 0x00, sizeof(tempTextU));	
                sprintf(tempTextU, "MAXIMUM EVENTS:%d ", TimeMatrixDepth -1);
                writeTextToFrameBuffer(buf_xxx, 5, tempTextU, (uint32_t)strlen(tempTextU));	     
                
                memset(tempTextU, 0x00, sizeof(tempTextU));	
                sprintf(tempTextU, "%s", "REMOVE USB DRIVE");
                write_2lines_mid(6); 
                
                
                //Error loading
                // program name
                //number of sequences: xxx
                //Error on line: xx
                // remove the USB drive
                break;
                
        case 11: //Error when try to save
                res_buf_line(2);
                res_buf_line(3);
                res_buf_line(4);
                res_buf_line(5);
                res_buf_line(6);
                res_buf_line(7);
                
                sprintf(tempTextU, "%s", "UPLOAD ERROR");
                write_2lines_mid(2);
                
                memset(tempTextU, 0x00, sizeof(tempTextU));	
                sprintf(tempTextU, "ERRORS IN SCRIPT");
                write_2lines_mid(4); 
                
                
                memset(tempTextU, 0x00, sizeof(tempTextU));	
                sprintf(tempTextU, "%s", "REMOVE USB DRIVE");
                write_2lines_mid(6);   
                
                
                // program name
                //number of sequences: xxx
                //Error on line: xx
                // remove the USB drive
                break;  
                
                
        case 12: // ERROR ON A LINE
                res_buf_line(2);
                res_buf_line(3);
                res_buf_line(4);
                res_buf_line(5);
                res_buf_line(6);
                res_buf_line(7);
                
                sprintf(tempTextU, "%s", "UPLOAD ERROR");
                write_2lines_mid(2);
                
                memset(tempTextU, 0x00, sizeof(tempTextU));	
                sprintf(tempTextU, "ERROR ON LINE:%d", Line_read + 1);
                write_2lines_mid(4); 
                
                
                memset(tempTextU, 0x00, sizeof(tempTextU));	
                sprintf(tempTextU, "PLEASE CHECK SCRIPT");
                buf_xxx = get_center((uint8_t)strlen(tempTextU));
                writeTextToFrameBuffer(buf_xxx, 5, tempTextU, (uint32_t)strlen(tempTextU));	   
                
                memset(tempTextU, 0x00, sizeof(tempTextU));	
                sprintf(tempTextU, "%s", "REMOVE USB DRIVE");
                write_2lines_mid(6); 	 
                
                
                //Error loading
                // program name
                //number of sequences: xxx
                //Error on line: xx
                // remove the USB drive
                break;
                
        case 13: // ID NOT FOUND IN SCRIPT
                res_buf_line(2);
                res_buf_line(3);
                res_buf_line(4);
                res_buf_line(5);
                res_buf_line(6);
                res_buf_line(7);
                
                sprintf(tempTextU, "%s", "UPLOAD ERROR");
                write_2lines_mid(2);
                
                memset(tempTextU, 0x00, sizeof(tempTextU));	
                sprintf(tempTextU, "MODULE ID NOT FOUND");
                write_2lines_mid(4); 
                
                memset(tempTextU, 0x00, sizeof(tempTextU));	
                sprintf(tempTextU, "%s", "REMOVE USB DRIVE");
                write_2lines_mid(6); 
                
                
                //Error loading
                // program name
                //number of sequences: xxx
                //Error on line: xx
                // remove the USB drive
                break;             
                
        case 20: // USB host could not be Enabled
                res_buf_line(2);
                res_buf_line(3);
                res_buf_line(4);
                res_buf_line(5);
                res_buf_line(6);
                res_buf_line(7);
                
                sprintf(tempTextU, "%s", "USB ERROR: 0x01");
                write_2lines_mid(2);
                
                memset(tempTextU, 0x00, sizeof(tempTextU));	
                sprintf(tempTextU, "%s", "USB NOT ENABLED");
                write_2lines_mid(4); 
                
                
                memset(tempTextU, 0x00, sizeof(tempTextU));	
                sprintf(tempTextU, "%s", "PLEASE RESTART MODULE");
                write_2lines_mid(6); 
                
                break;
                
                
        case 21: // USB host could not be Enabled, USB charge connected
                res_buf_line(2);
                res_buf_line(3);
                res_buf_line(4);
                res_buf_line(5);
                res_buf_line(6);
                res_buf_line(7);
                
                sprintf(tempTextU, "%s", "USB ERROR: 0x02");
                write_2lines_mid(2);
                
                memset(tempTextU, 0x00, sizeof(tempTextU));	
                sprintf(tempTextU, "%s", "DISCONNECT USB");
                write_2lines_mid(4); 
                
                
                memset(tempTextU, 0x00, sizeof(tempTextU));	
                sprintf(tempTextU, "%s", "RESTART MODULE");
                write_2lines_mid(6); 
                
                break;    
                
        case 22: // USB host could not be Enabled, battery too low
                res_buf_line(2);
                res_buf_line(3);
                res_buf_line(4);
                res_buf_line(5);
                res_buf_line(6);
                res_buf_line(7);
                
                sprintf(tempTextU, "%s", "USB ERROR: 0x03");
                write_2lines_mid(2);
                
                memset(tempTextU, 0x00, sizeof(tempTextU));	
                sprintf(tempTextU, "%s", "TOO LOW BATTERY");
                write_2lines_mid(4); 
                
                
                memset(tempTextU, 0x00, sizeof(tempTextU));	
                sprintf(tempTextU, "%s", "RESTART MODULE");
                write_2lines_mid(6); 
                
                break;    
                
        case 50: 	// USB Not supported
                res_buf_line(2);
                res_buf_line(3);
                res_buf_line(4);
                res_buf_line(5);
                res_buf_line(6);
                res_buf_line(7);
                
                
                
                sprintf(tempTextU, "%s", "USB DEVICE");
                write_2lines_mid(2);
                
                memset(tempTextU, 0x00, sizeof(tempTextU));	
                sprintf(tempTextU, "%s", "NOT SUPPORTED");
                buf_xxx = get_center((uint8_t)strlen(tempTextU));
                writeTextToFrameBuffer(buf_xxx, 4, tempTextU, (uint32_t)strlen(tempTextU));	
                
                memset(tempTextU, 0x00, sizeof(tempTextU));	
                sprintf(tempTextU, "%s", "REMOVE USB DEVICE");
                write_2lines_mid(5);
                
                
                break;
                
        case 60: 	// USB Error
                res_buf_line(2);
                res_buf_line(3);
                res_buf_line(4);
                res_buf_line(5);
                res_buf_line(6);
                res_buf_line(7);
                
                
                
                sprintf(tempTextU, "%s", "USB DEVICE");
                write_2lines_mid(2);
                
                memset(tempTextU, 0x00, sizeof(tempTextU));	
                sprintf(tempTextU, "%s", "NOT SUPPORTED");
                buf_xxx = get_center((uint8_t)strlen(tempTextU));
                writeTextToFrameBuffer(buf_xxx, 4, tempTextU, (uint32_t)strlen(tempTextU));	
                
                memset(tempTextU, 0x00, sizeof(tempTextU));	
                sprintf(tempTextU, "%s", "REMOVE USB DEVICE");
                write_2lines_mid(5);
                
                break;
        }
        
       sendToDisplay(0,0,displayBuffer[0], DISPLAY_NUMBER_OF_COLUMNS);
       sendToDisplay(0,1,displayBuffer[1], DISPLAY_NUMBER_OF_COLUMNS);        
       sendToDisplay(0,2,displayBuffer[2], DISPLAY_NUMBER_OF_COLUMNS);
       sendToDisplay(0,3,displayBuffer[3], DISPLAY_NUMBER_OF_COLUMNS);
       sendToDisplay(0,4,displayBuffer[4], DISPLAY_NUMBER_OF_COLUMNS);
       sendToDisplay(0,5,displayBuffer[5], DISPLAY_NUMBER_OF_COLUMNS);
       sendToDisplay(0,6,displayBuffer[6], DISPLAY_NUMBER_OF_COLUMNS);
       sendToDisplay(0,7,displayBuffer[7], DISPLAY_NUMBER_OF_COLUMNS);
        
}

