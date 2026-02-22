//	MIT License
//
//	Copyright (c) 2017 Matej Artnak
//
//	Permission is hereby granted, free of charge, to any person obtaining a copy
//	of this software and associated documentation files (the "Software"), to deal
//	in the Software without restriction, including without limitation the rights
//	to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
//	copies of the Software, and to permit persons to whom the Software is
//	furnished to do so, subject to the following conditions:
//
//	The above copyright notice and this permission notice shall be included in all
//	copies or substantial portions of the Software.
//
//	THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
//	IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
//	FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
//	AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
//	LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
//	OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
//	SOFTWARE.
//
//
//
//-----------------------------------
//	ILI9341 GFX library for STM32
//-----------------------------------
//
//	Very simple GFX library built upon ILI9342_STM32_Driver library.
//	Adds basic shapes, image and font drawing capabilities to ILI9341
//
//	Library is written for STM32 HAL library and supports STM32CUBEMX. To use the library with Cube software
//	you need to tick the box that generates peripheral initialization code in their own respective .c and .h file
//
//
//-----------------------------------
//	How to use this library
//-----------------------------------
//
//	-If using MCUs other than STM32F7 you will have to change the #include "stm32f7xx_hal.h" in the ILI9341_GFX.h to your respective .h file
//
//	If using "ILI9341_STM32_Driver" then all other prequisites to use the library have allready been met
//	Simply include the library and it is ready to be used
//
//-----------------------------------

#include "tm_stm32f4_fonts.h"
#include "ILI9341_STM32_Driver.h"
#include "ILI9341_GFX.h"
#include "5x5_font.h"
#include "main.h"
#include "uart.h"
#include "USBHostMain.h"
#include "SystemTimer.h"
#include "Buttons.h"
#include "Display.h"
#include "Wireless.h"
#include "TimeCode.h"


#ifndef VS_READABLE_CODE
#include "System1.h"
#include "System1Action.h"
#include "System1Data.h"
#endif

#include "simpleEventHandler.h"         		// Event Queue
#include "System1SEMLibB.h"                    	// visualSTATE essentials
#include "options.h"





//#define LANG ENGLISH

uint8_t LANG = ENGLISH;

extern uint8_t is_433Mhz_en;

extern uint8_t was_LTC_Started;

extern uint8_t current_AB_For_D;
extern uint8_t current_wir_chan;

extern uint8_t Time_code_disable;
extern uint16_t was_here_time_plus;

extern uint8_t crit_errA;
extern int Adjusted_time;

extern uint8_t Hours_tmp;
extern uint8_t Min_tmp;
extern uint8_t Sec_tmp;

extern char Network_Name[6];

extern uint16_t error_load_script;


extern uint8_t is_manual_multiple_mods;

//unsigned char color_TST[3 * 480 * 40];

extern lcd_info_script script_info;
extern uint8_t file_read_error;

uint8_t LCD_too_long = 0;

extern uint8_t current_Slave_For_D;
extern uint8_t current_Slave_For_D_Prg;

extern char seq_txts[24];
extern uint8_t Current_Seq;

extern char tempTextTimetoEND[20];
extern char tempTextTimetoPass[20];

extern uint8_t Err_found_mods;
extern uint8_t script_is_end;

uint8_t Color_Scheme = 0;
uint32_t color_tmp_x = 0;

extern lcd_disp_script Script_to_LCD[MAX_SCRIPT_LINES];
extern uint64_t tmp_disp_refrsh;



uint8_t RED_x = 0; 
uint8_t GREEN_x = 0; 
uint8_t BLUE_x = 0;

/*
uint16_t Mod_Stat[MAXIM_STATS] = {0};
uint32_t Rem_Stat[MAXIM_STATS] = {0};


uint32_t Rem_Stat2A[LINE2A_SYS_STATS] = {0};
uint32_t Rem_Stat2B[LINE2B_SYS_STATS] = {0};
int Rem_Stat2C[LINE2C_SYS_STATS] = {0};
*/

extern char prg_name_LCD[MAX_SCRIPT_LENGTH_LINE_N];

uint8_t is_LCD_busy = 0;

uint16_t ILI9341_x;
uint16_t ILI9341_y;
TM_ILI931_Options_t ILI9341_Opts;

//LCD_Write_Line Line_cur[1];

extern uint8_t Rail_Stat[4][25];


//unsigned char color_TST[2 * 2 * 320* 24];

unsigned char color_TST[1 * 1 * 16];

uint32_t is_LCD_busy_count = 0;


//extern char tempTextTime[20];
//extern char tempTextTimeF[20];

//uint8_t Mod_Err_Stats[MAX_NUMBER_OF_DEVICES] = {0};



//uint8_t is_buffer_lcd = 0;

//uint16_t color_TST_16[320* 24];

extern SPI_HandleTypeDef hspi1;

/*Draw hollow circle at X,Y location with specified radius and colour. X and Y represent circles center */
void ILI9341_Draw_Hollow_Circle(uint16_t X, uint16_t Y, uint16_t Radius, uint16_t Colour)
{
	int x = Radius-1;
    int y = 0;
    int dx = 1;
    int dy = 1;
    int err = dx - (Radius << 1);

    while (x >= y)
    {
        ILI9341_Draw_Pixel(X + x, Y + y, Colour);
        ILI9341_Draw_Pixel(X + y, Y + x, Colour);
        ILI9341_Draw_Pixel(X - y, Y + x, Colour);
        ILI9341_Draw_Pixel(X - x, Y + y, Colour);
        ILI9341_Draw_Pixel(X - x, Y - y, Colour);
        ILI9341_Draw_Pixel(X - y, Y - x, Colour);
        ILI9341_Draw_Pixel(X + y, Y - x, Colour);
        ILI9341_Draw_Pixel(X + x, Y - y, Colour);

        if (err <= 0)
        {
            y++;
            err += dy;
            dy += 2;
        }
        if (err > 0)
        {
            x--;
            dx += 2;
            err += (-Radius << 1) + dx;
        }
    }
}

/*Draw filled circle at X,Y location with specified radius and colour. X and Y represent circles center */
void ILI9341_Draw_Filled_Circle(uint16_t X, uint16_t Y, uint16_t Radius, uint16_t Colour)
{
	
		int x = Radius;
    int y = 0;
    int xChange = 1 - (Radius << 1);
    int yChange = 0;
    int radiusError = 0;

    while (x >= y)
    {
        for (int i = X - x; i <= X + x; i++)
        {
            ILI9341_Draw_Pixel(i, Y + y,Colour);
            ILI9341_Draw_Pixel(i, Y - y,Colour);
        }
        for (int i = X - y; i <= X + y; i++)
        {
            ILI9341_Draw_Pixel(i, Y + x,Colour);
            ILI9341_Draw_Pixel(i, Y - x,Colour);
        }

        y++;
        radiusError += yChange;
        yChange += 2;
        if (((radiusError << 1) + xChange) > 0)
        {
            x--;
            radiusError += xChange;
            xChange += 2;
        }
    }
		//Really slow implementation, will require future overhaul
		//TODO:	https://stackoverflow.com/questions/1201200/fast-algorithm-for-drawing-filled-circles	
}

/*Draw a hollow rectangle between positions X0,Y0 and X1,Y1 with specified colour*/
void ILI9341_Draw_Hollow_Rectangle_Coord(uint16_t X0, uint16_t Y0, uint16_t X1, uint16_t Y1, uint16_t Colour)
{
	uint16_t 	X_length = 0;
	uint16_t 	Y_length = 0;
	uint8_t		Negative_X = 0;
	uint8_t 	Negative_Y = 0;
	float 		Calc_Negative = 0;
	
	Calc_Negative = X1 - X0;
	if(Calc_Negative < 0) Negative_X = 1;
	Calc_Negative = 0;
	
	Calc_Negative = Y1 - Y0;
	if(Calc_Negative < 0) Negative_Y = 1;
	
	
	//DRAW HORIZONTAL!
	if(!Negative_X)
	{
		X_length = X1 - X0;		
	}
	else
	{
		X_length = X0 - X1;		
	}
	ILI9341_Draw_Horizontal_Line(X0, Y0, X_length, Colour);
	ILI9341_Draw_Horizontal_Line(X0, Y1, X_length, Colour);
	
	
	
	//DRAW VERTICAL!
	if(!Negative_Y)
	{
		Y_length = Y1 - Y0;		
	}
	else
	{
		Y_length = Y0 - Y1;		
	}
	ILI9341_Draw_Vertical_Line(X0, Y0, Y_length, Colour);
	ILI9341_Draw_Vertical_Line(X1, Y0, Y_length, Colour);
	
	if((X_length > 0)||(Y_length > 0)) 
	{
		ILI9341_Draw_Pixel(X1, Y1, Colour);
	}
	
}

/*Draw a filled rectangle between positions X0,Y0 and X1,Y1 with specified colour*/
void ILI9341_Draw_Filled_Rectangle_Coord(uint16_t X0, uint16_t Y0, uint16_t X1, uint16_t Y1, uint16_t Colour)
{
	uint16_t 	X_length = 0;
	uint16_t 	Y_length = 0;
	uint8_t		Negative_X = 0;
	uint8_t 	Negative_Y = 0;
	int32_t 	Calc_Negative = 0;
	
	uint16_t X0_true = 0;
	uint16_t Y0_true = 0;
	
	Calc_Negative = X1 - X0;
	if(Calc_Negative < 0) Negative_X = 1;
	Calc_Negative = 0;
	
	Calc_Negative = Y1 - Y0;
	if(Calc_Negative < 0) Negative_Y = 1;
	
	
	//DRAW HORIZONTAL!
	if(!Negative_X)
	{
		X_length = X1 - X0;
		X0_true = X0;
	}
	else
	{
		X_length = X0 - X1;
		X0_true = X1;
	}
	
	//DRAW VERTICAL!
	if(!Negative_Y)
	{
		Y_length = Y1 - Y0;
		Y0_true = Y0;		
	}
	else
	{
		Y_length = Y0 - Y1;
		Y0_true = Y1;	
	}
	
	ILI9341_Draw_Rectangle(X0_true, Y0_true, X_length, Y_length, Colour);	
}

/*Draws a character (fonts imported from fonts.h) at X,Y location with specified font colour, size and Background colour*/
/*See fonts.h implementation of font on what is required for changing to a different font when switching fonts libraries*/
void ILI9341_Draw_Char(char Character, uint8_t X, uint8_t Y, uint16_t Colour, uint16_t Size, uint16_t Background_Colour) 
{
		uint8_t 	function_char;
    uint8_t 	i,j;
		
		function_char = Character;
		
    if (function_char < ' ') {
        Character = 0;
    } else {
        function_char -= 32;
		}
   	
		char temp[CHAR_WIDTH];
		for(uint8_t k = 0; k<CHAR_WIDTH; k++)
		{
		temp[k] = font[function_char][k];
		}
		
    // Draw pixels
		ILI9341_Draw_Rectangle(X, Y, CHAR_WIDTH*Size, CHAR_HEIGHT*Size, Background_Colour);
    for (j=0; j<CHAR_WIDTH; j++) {
        for (i=0; i<CHAR_HEIGHT; i++) {
            if (temp[j] & (1<<i)) {			
							if(Size == 1)
							{
              ILI9341_Draw_Pixel(X+j, Y+i, Colour);
							}
							else
							{
							ILI9341_Draw_Rectangle(X+(j*Size), Y+(i*Size), Size, Size, Colour);
							}
            }						
        }
    }
}





void Draw_Line_ftek (uint16_t x, uint16_t y, uint16_t xx, uint16_t yy, char *str, TM_FontDef_t *font, uint32_t foreground, uint32_t background)       
{
        uint16_t startX = 0;
	
	/* Set X and Y coordinates */
	ILI9341_x = 0;
	ILI9341_y = 0;
        

        memset(color_TST, background, sizeof(color_TST));

	
	while (*str) {
		/* New line */
		if (*str == '\n') {
			ILI9341_y += font->FontHeight + 1;
			/* if after \n is also \r, than go to the left of the screen */
			if (*(str + 1) == '\r') {
				ILI9341_x = 0;
				str++;
			} else {
				ILI9341_x = startX;
			}
			str++;
			continue;
		} else if (*str == '\r') {
			str++;
			continue;
		}
		
		/* Put character to LCD */
		Draw_Char_ftek(ILI9341_x, ILI9341_y, xx, yy, *str++, font, foreground, background);
	}
        
        
               //  int *intpointer = &intarray[3];
   
   

    send_to_LCDFont(3 * font->FontHeight * yy);

    
   // Draw_Colour_Burst_firetek(24*320);
}

void send_to_LCD(uint8_t line_x, uint16_t h)
{
           tmp_disp_refrsh = GetCurrentSystemTime();
        
           HAL_GPIO_WritePin(LCD_DC_PORT, LCD_DC_PIN, GPIO_PIN_SET);	
           HAL_GPIO_WritePin(LCD_CS_PORT, LCD_CS_PIN, GPIO_PIN_RESET);
           
           is_LCD_busy = 1;

           HAL_SPI_Transmit_DMA(HSPI_INSTANCE, (uint8_t *)color_TST, h * 3 * 480);
}


void send_to_LCDFont(uint16_t size)
{
           tmp_disp_refrsh = GetCurrentSystemTime();
        
           HAL_GPIO_WritePin(LCD_DC_PORT, LCD_DC_PIN, GPIO_PIN_SET);	
           HAL_GPIO_WritePin(LCD_CS_PORT, LCD_CS_PIN, GPIO_PIN_RESET);
           
           is_LCD_busy = 1;

           HAL_SPI_Transmit_DMA(HSPI_INSTANCE, (uint8_t *)color_TST, size); //height_font * 480);
}

void Draw_Char_ftek(uint16_t x, uint16_t y, uint16_t xx, uint16_t yy, char c, TM_FontDef_t *font, uint32_t foreground, uint32_t background)
{
       
        uint32_t add_pointsX = 0x00;
        
        uint32_t i, b, j, k;
	/* Set coordinates */
	ILI9341_x = x;
	ILI9341_y = y;
        
        if(font->FontWidth > 16)
        {
               add_pointsX = 0x80000000;
        }
        else
        {
                add_pointsX = 0x8000;
        }
	
	if ((ILI9341_x + font->FontWidth) > ILI9341_Opts.width) {
		/* If at the end of a line of display, go to new line and set x to 0 position */
		ILI9341_y += font->FontHeight;
		ILI9341_x = 0;
	}
	
	/* Draw font data */
	for (i = 0; i < font->FontHeight; i++) {
		b = font->data[(c - 32) * font->FontHeight + i];
                
		for (j = 0; j < font->FontWidth; j++) {
                        
                             k = ILI9341_x + j + yy * (ILI9341_y + i);
                        
                             if(3 * k + 3 > sizeof(color_TST))
                             {
                                LCD_too_long = 1;
                                break;
                             }
                        
			if ((b << j) & add_pointsX) {
                                               color_TST[3 * k] =  (uint8_t)((foreground >> 16) & 0x000000FF);
                                               color_TST[3 * k + 1] = (uint8_t)((foreground >> 8) & 0x000000FF); 
                                               color_TST[3 * k + 2] = (uint8_t)((foreground >> 0) & 0x000000FF); 
			}
                        else
                        {
                                
                                if(background != TRANSPARENT)
                                {
                                               color_TST[3 * k] =  (uint8_t)((background >> 16) & 0x000000FF);
                                               color_TST[3 * k + 1] = (uint8_t)((background >> 8) & 0x000000FF); 
                                               color_TST[3 * k + 2] = (uint8_t)((background >> 0) & 0x000000FF); 
                                }

                        }
		}
	}
        
	
	/* Set new pointer */
	ILI9341_x += font->FontWidth;
}


void set_mem_background(uint16_t background, uint16_t length)
{
       //uint32_t size = sizeof(color_TST) / 2;
        
        
        for (int i = 0; i < length; i = i + 3)
        {
                if(background == ILI9341_COLOR_GREEN2)
                {
                        //orange
                       
                        RED_x = 0xFF; //0xE8
                        GREEN_x = 0x87; //0xC0
                        BLUE_x =  0x21; 
                }
                else
                {
                        RED_x = 0xF0; //0xE8
                        GREEN_x = 0x00; //0xC0
                        BLUE_x =  0x00; 
                        /*
                        RED_x = (uint8_t)((background & 0x001F) << 3);
                        GREEN_x =  (uint8_t)((background & 0xF800) >> 8); 
                        BLUE_x = (uint8_t)((background & 0x07E0) >> 3); 
                        */
                }
                
                
                color_TST[i + 0] = RED_x; 
                color_TST[i + 1] = GREEN_x; 
                color_TST[i + 2] = BLUE_x;
        }
}

void set_mem_background32(uint32_t background, uint16_t start_adr, uint16_t length)
{

        for (int i = start_adr; i < length; i = i + 3)
        {
                 RED_x = (uint8_t)((background >> 16) & 0x000000FF);
                 GREEN_x =  (uint8_t)((background >> 8) & 0x000000FF); 
                 BLUE_x = (uint8_t)((background >> 0) & 0x000000FF); 
                
                color_TST[i + 0] = RED_x; 
                color_TST[i + 1] = GREEN_x; 
                color_TST[i + 2] = BLUE_x;
        }
}



void ili9488_FillRectTst(uint16_t X1, uint16_t Y1)
{
        
        ILI9341_Opts.width = ILI9341_WIDTH;
	ILI9341_Opts.height = ILI9341_HEIGHT;
        
}

uint16_t get_mid_string(uint8_t length, char *string, TM_FontDef_t *font, uint16_t size_len)
{
        
        uint16_t length_string = 0;
        
        for(int i = 0; i < length; i++)
        {
                if(string[i] == 0)
                {
                      length_string = i;
                        break;                        
                }
        }
        if(length_string == 0){
                length_string = length;
        }
        
        length_string = length_string * font->FontWidth;
        uint16_t start_string = (size_len - length_string)/2;
        return start_string;
}

