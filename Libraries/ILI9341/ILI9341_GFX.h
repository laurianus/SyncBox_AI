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

#ifndef ILI9341_GFX_H
#define ILI9341_GFX_H

#include "stm32f429xx.h"
#include "tm_stm32f4_fonts.h"

#define HORIZONTAL_IMAGE	0
#define VERTICAL_IMAGE		1




/* Private defines */
#define ILI9341_RESET				0x01
#define ILI9341_SLEEP_OUT			0x11
#define ILI9341_GAMMA				0x26
#define ILI9341_DISPLAY_OFF			0x28
#define ILI9341_DISPLAY_ON			0x29
#define ILI9341_COLUMN_ADDR			0x2A
#define ILI9341_PAGE_ADDR			0x2B
#define ILI9341_GRAM				0x2C
#define ILI9341_MAC					0x36
#define ILI9341_PIXEL_FORMAT		0x3A
#define ILI9341_WDB					0x51
#define ILI9341_WCD					0x53
#define ILI9341_RGB_INTERFACE		0xB0
#define ILI9341_FRC					0xB1
#define ILI9341_BPC					0xB5
#define ILI9341_DFC					0xB6
#define ILI9341_POWER1				0xC0
#define ILI9341_POWER2				0xC1
#define ILI9341_VCOM1				0xC5
#define ILI9341_VCOM2				0xC7
#define ILI9341_POWERA				0xCB
#define ILI9341_POWERB				0xCF
#define ILI9341_PGAMMA				0xE0
#define ILI9341_NGAMMA				0xE1
#define ILI9341_DTCA				0xE8
#define ILI9341_DTCB				0xEA
#define ILI9341_POWER_SEQ			0xED
#define ILI9341_3GAMMA_EN			0xF2
#define ILI9341_INTERFACE			0xF6
#define ILI9341_PRC					0xF7


/* Colors */
#define ILI9341_COLOR_WHITE			0xFFFF
#define ILI9341_COLOR_BLACK			0x0000
#define ILI9341_COLOR_RED       0xF800
#define ILI9341_COLOR_GREEN			0x0DA6 //0x07E0
//#define ILI9341_COLOR_GREEN			0x0383

#define ILI9341_COLOR_GREEN2		0xB723
#define ILI9341_COLOR_BLUE			0x001F
#define ILI9341_COLOR_BLUE2			0x051D
//#define ILI9341_COLOR_YELLOW		0xFFE0
#define ILI9341_COLOR_YELLOW		0xC600


#define COLOR32_BLACK			0x00000000
#define COLOR32_RED			0x00FF8721
#define COLOR32_GREEN			0x0000C000
#define COLOR32_BLUE			0x000000FF
#define COLOR32_WHITE			0x00FFFFFF
#define TRANSPARENT			0xFF000000
#define COLOR32_GRAY			0x00FAFAFA


//#define COLOR32_ORANGEk			0x00FF8721
//#define COLOR32_REDk			0x00E80000 //F0

#define COLOR32_ORANGEk			0x00FF6A00
#define COLOR32_REDk			0x00FA0000 //F0
#define COLOR32_GREENk			0x0000C000

                        
                        


#define ILI9341_COLOR_ORANGE		0xFBE4
#define ILI9341_COLOR_CYAN			0x07FF
#define ILI9341_COLOR_MAGENTA		0xA254

//#define ILI9341_COLOR_GRAY			0x7BEF
#define ILI9341_COLOR_GRAY			0x8410

#define ILI9341_COLOR_BROWN			0xBBCA

void ILI9341_Draw_Hollow_Circle(uint16_t X, uint16_t Y, uint16_t Radius, uint16_t Colour);
void ILI9341_Draw_Filled_Circle(uint16_t X, uint16_t Y, uint16_t Radius, uint16_t Colour);
void ILI9341_Draw_Hollow_Rectangle_Coord(uint16_t X0, uint16_t Y0, uint16_t X1, uint16_t Y1, uint16_t Colour);
void ILI9341_Draw_Filled_Rectangle_Coord(uint16_t X0, uint16_t Y0, uint16_t X1, uint16_t Y1, uint16_t Colour);
void ILI9341_Draw_Char(char Character, uint8_t X, uint8_t Y, uint16_t Colour, uint16_t Size, uint16_t Background_Colour);
void ILI9341_Draw_Text(const char* Text, uint8_t X, uint8_t Y, uint16_t Colour, uint16_t Size, uint16_t Background_Colour);
void ILI9341_Draw_Filled_Rectangle_Size_Text(uint16_t X0, uint16_t Y0, uint16_t Size_X, uint16_t Size_Y, uint16_t Colour);

//USING CONVERTER: http://www.digole.com/tools/PicturetoC_Hex_converter.php
//65K colour (2Bytes / Pixel)
void ILI9341_Draw_Image(const char* Image_Array, uint8_t Orientation);



/**
 * @brief  Possible orientations for LCD
 */
typedef enum {
	TM_ILI9341_Orientation_Portrait_1,  /*!< Portrait orientation mode 1 */
	TM_ILI9341_Orientation_Portrait_2,  /*!< Portrait orientation mode 2 */
	TM_ILI9341_Orientation_Landscape_1, /*!< Landscape orientation mode 1 */
	TM_ILI9341_Orientation_Landscape_2  /*!< Landscape orientation mode 2 */
} TM_ILI9341_Orientation_t;



/** Bit mask for flipping X for ili9488_set_orientation() */
#define ILI9488_FLIP_X 1
/** Bit mask for flipping Y for ili9488_set_orientation() */
#define ILI9488_FLIP_Y 2
/** Bit mask for swapping X and Y for ili9488_set_orientation() */
#define ILI9488_SWITCH_XY 4


/* ILI9488 screen size */
#define ILI9488_LCD_WIDTH  320
#define ILI9488_LCD_HEIGHT 480

/** Height of display using swapped X/Y orientation */
#define ILI9488_SWITCH_XY_HEIGHT 480

/** Width of display using swapped X/Y orientation */
#define ILI9488_SWITCH_XY_WIDTH  320

/* ILI9488 ID code */
#define ILI9488_DEVICE_CODE (0x9488u)

/* Level 1 Commands (from the display Datasheet) */
#define ILI9488_CMD_NOP                             0x00
#define ILI9488_CMD_SOFTWARE_RESET                  0x01
#define ILI9488_CMD_READ_DISP_ID                    0x04
#define ILI9488_CMD_READ_ERROR_DSI                  0x05
#define ILI9488_CMD_READ_DISP_STATUS                0x09
#define ILI9488_CMD_READ_DISP_POWER_MODE            0x0A
#define ILI9488_CMD_READ_DISP_MADCTRL               0x0B
#define ILI9488_CMD_READ_DISP_PIXEL_FORMAT          0x0C
#define ILI9488_CMD_READ_DISP_IMAGE_MODE            0x0D
#define ILI9488_CMD_READ_DISP_SIGNAL_MODE           0x0E
#define ILI9488_CMD_READ_DISP_SELF_DIAGNOSTIC       0x0F
#define ILI9488_CMD_ENTER_SLEEP_MODE                0x10
#define ILI9488_CMD_SLEEP_OUT                       0x11
#define ILI9488_CMD_PARTIAL_MODE_ON                 0x12
#define ILI9488_CMD_NORMAL_DISP_MODE_ON             0x13
#define ILI9488_CMD_DISP_INVERSION_OFF              0x20
#define ILI9488_CMD_DISP_INVERSION_ON               0x21
#define ILI9488_CMD_PIXEL_OFF                       0x22
#define ILI9488_CMD_PIXEL_ON                        0x23
#define ILI9488_CMD_DISPLAY_OFF                     0x28
#define ILI9488_CMD_DISPLAY_ON                      0x29
#define ILI9488_CMD_COLUMN_ADDRESS_SET              0x2A
#define ILI9488_CMD_PAGE_ADDRESS_SET                0x2B
#define ILI9488_CMD_MEMORY_WRITE                    0x2C
#define ILI9488_CMD_MEMORY_READ                     0x2E
#define ILI9488_CMD_PARTIAL_AREA                    0x30
#define ILI9488_CMD_VERT_SCROLL_DEFINITION          0x33
#define ILI9488_CMD_TEARING_EFFECT_LINE_OFF         0x34
#define ILI9488_CMD_TEARING_EFFECT_LINE_ON          0x35
#define ILI9488_CMD_MEMORY_ACCESS_CONTROL           0x36
#define ILI9488_CMD_VERT_SCROLL_START_ADDRESS       0x37
#define ILI9488_CMD_IDLE_MODE_OFF                   0x38
#define ILI9488_CMD_IDLE_MODE_ON                    0x39
#define ILI9488_CMD_COLMOD_PIXEL_FORMAT_SET         0x3A
#define ILI9488_CMD_WRITE_MEMORY_CONTINUE           0x3C
#define ILI9488_CMD_READ_MEMORY_CONTINUE            0x3E
#define ILI9488_CMD_SET_TEAR_SCANLINE               0x44
#define ILI9488_CMD_GET_SCANLINE                    0x45
#define ILI9488_CMD_WRITE_DISPLAY_BRIGHTNESS        0x51
#define ILI9488_CMD_READ_DISPLAY_BRIGHTNESS         0x52
#define ILI9488_CMD_WRITE_CTRL_DISPLAY              0x53
#define ILI9488_CMD_READ_CTRL_DISPLAY               0x54
#define ILI9488_CMD_WRITE_CONTENT_ADAPT_BRIGHTNESS  0x55
#define ILI9488_CMD_READ_CONTENT_ADAPT_BRIGHTNESS   0x56
#define ILI9488_CMD_WRITE_MIN_CAB_LEVEL             0x5E
#define ILI9488_CMD_READ_MIN_CAB_LEVEL              0x5F
#define ILI9488_CMD_READ_ABC_SELF_DIAG_RES          0x68
#define ILI9488_CMD_READ_ID1                        0xDA
#define ILI9488_CMD_READ_ID2                        0xDB
#define ILI9488_CMD_READ_ID3                        0xDC





/**
 * @brief  Orientation
 * @note   Used private
 */
typedef enum {
	TM_ILI9341_Landscape,
	TM_ILI9341_Portrait
} TM_ILI9341_Orientation;

/**
 * @brief  LCD options
 * @note   Used private
 */
typedef struct {
	uint16_t width;
	uint16_t height;
	TM_ILI9341_Orientation orientation; // 1 = portrait; 0 = landscape
} TM_ILI931_Options_t;



typedef struct {
	unsigned char color[2 * 320* 24];
        uint8_t Line[12];
} LCD_Write_Line;

/**
 * @brief  Fills entire LCD with color
 * @param  color: Color to be used in fill
 * @retval None
 */
void TM_ILI9341_Fill(uint32_t color);

/**
 * @brief  Rotates LCD to specific orientation
 * @param  orientation: LCD orientation. This parameter can be a value of @ref TM_ILI9341_Orientation_t enumeration
 * @retval None
 */
void TM_ILI9341_Rotate(TM_ILI9341_Orientation_t orientation);

/**
 * @brief  Puts single character to LCD
 * @param  x: X position of top left corner
 * @param  y: Y position of top left corner
 * @param  c: Character to be displayed
 * @param  *font: Pointer to @ref TM_FontDef_t used font
 * @param  foreground: Color for char
 * @param  background: Color for char background
 * @retval None
 */
void TM_ILI9341_Putc(uint16_t x, uint16_t y, char c, TM_FontDef_t* font, uint32_t foreground, uint32_t background);

/**
 * @brief  Puts string to LCD
 * @param  x: X position of top left corner of first character in string
 * @param  y: Y position of top left corner of first character in string
 * @param  *str: Pointer to first character
 * @param  *font: Pointer to @ref TM_FontDef_t used font
 * @param  foreground: Color for string
 * @param  background: Color for string background
 * @retval None
 */
void TM_ILI9341_Puts(uint16_t x, uint16_t y, char* str, TM_FontDef_t *font, uint32_t foreground, uint32_t background);

/**
 * @brief  Gets width and height of box with text
 * @param  *str: Pointer to first character
 * @param  *font: Pointer to @ref TM_FontDef_t used font
 * @param  *width: Pointer to variable to store width
 * @param  *height: Pointer to variable to store height
 * @retval None
 */
void TM_ILI9341_GetStringSize(char* str, TM_FontDef_t* font, uint16_t* width, uint16_t* height);

/**
 * @brief  Draws line to LCD
 * @param  x0: X coordinate of starting point
 * @param  y0: Y coordinate of starting point
 * @param  x1: X coordinate of ending point
 * @param  y1: Y coordinate of ending point
 * @param  color: Line color
 * @retval None
 */


void Draw_Char_ftek(uint16_t x, uint16_t y, uint16_t xx, uint16_t yy, char c, TM_FontDef_t *font, uint32_t foreground, uint32_t background);
void Draw_Line_ftek (uint16_t x, uint16_t y, uint16_t xx, uint16_t yy, char *str, TM_FontDef_t *font, uint32_t foreground, uint32_t background);
void Set_Address_ftek(uint16_t X1, uint16_t Y1, uint16_t X2, uint16_t Y2);
void Draw_Pixel_ftek(uint16_t color);
void send_to_LCD(uint8_t line_x, uint16_t h);

void Draw_Icon_ftek(uint16_t x, uint16_t y, uint16_t xx, uint16_t yy, uint8_t c_pos, TM_FontDef_t *font, uint32_t foreground, uint32_t background);
void Draw_Line_Rem_Icon_ftek (uint16_t x, uint16_t y, uint16_t xx, uint16_t yy, uint32_t background);
void Draw_String_Line_ftek (uint16_t x, uint16_t y,  uint16_t xx, uint16_t yy, char *str, TM_FontDef_t *font, uint32_t foreground, uint32_t background);


void Draw_String_Line32b_ftek (uint16_t x, uint16_t y, char *str, TM_FontDef_t *font, uint32_t foreground, uint32_t background);
void Draw_Char32b_ftek(uint16_t x, uint16_t y, char c, TM_FontDef_t *font, uint32_t foreground, uint32_t background);

void Draw_Line_Icon_ftekX(uint16_t x, uint16_t y, uint16_t xx,  uint16_t yy, uint32_t background, uint8_t is_AB_XXX);

void Draw_Line_Icon_Rail(uint16_t x, uint16_t y, uint32_t background, uint8_t rail_Xnr, uint8_t is_Audi_BoxA);
uint32_t get_color_ch_stat(uint8_t val_chan);

void send_to_LCDFont(uint16_t height_font);
void Draw_SLine_ftek(uint16_t x, uint16_t y, uint16_t length, uint32_t foreground);

void set_mem_background(uint16_t background, uint16_t length);

void ili9488_FillRectTst(uint16_t X1, uint16_t Y1);
void Draw_Line_Main_ftekX (uint16_t x, uint16_t y, uint16_t xx, uint16_t yy, uint32_t background);
void Draw_Line_Main_ftek (uint16_t x, uint16_t y, uint16_t xx, uint16_t yy, uint32_t background);
void Draw_Line_Main_ftekY (uint16_t x, uint16_t y, uint16_t xx, uint16_t yy, uint32_t background);
void set_mem_background32(uint32_t background, uint16_t start_adr, uint16_t length);

void Draw_Line_Script_Rail(uint16_t x, uint16_t y, uint16_t h, uint32_t background, uint8_t Script_Xnr);
uint16_t get_mid_string(uint8_t length, char *string, TM_FontDef_t *font, uint16_t size_len);

void Draw_Line_Script_Wait(uint16_t x, uint16_t y, uint16_t h, uint32_t background, uint16_t Script_Xnr);
void set_mem_background32Y(uint32_t background1, uint32_t background2, uint16_t start_adr, uint16_t Y_StartAdr, uint16_t length, uint16_t length_disp);
void Draw_Line_Script_info(uint16_t x, uint16_t y, uint16_t h, uint32_t background, uint8_t Script_Xnr);
void Draw_Mod_Stats_Icon_ftekX(uint16_t x, uint16_t y, uint32_t background, uint8_t mod_X, uint8_t mod_Y);
void set_mem_background32YY(uint32_t background1, uint32_t background2, uint16_t Y_StartAdr, uint16_t Y_EndAdr, uint16_t length, uint16_t length_disp);
void Draw_First_Screen(uint16_t x, uint16_t y, uint32_t background, uint8_t Line_X);


void Draw_Line_Audio(uint16_t x, uint16_t y, uint16_t h, uint32_t background, uint8_t Script_Xnr);

void Draw_Line_Script_Upload(uint16_t x, uint16_t y, uint16_t h, uint32_t background, uint16_t Script_Xnr, 
        uint16_t mod_id, uint16_t ev_tot, uint16_t ev_left, uint32_t time_left, uint32_t time_pass, uint16_t mod_up, uint16_t mod_tot);


uint8_t get_Last_Time_disp(uint8_t Multiplier);
uint16_t get_length_string(uint8_t length, char *string);

void Draw_Line_Seq(uint16_t x, uint16_t y, uint16_t h, uint32_t background, uint8_t Script_Xnr);
void Draw_Line_SZ(uint16_t x, uint16_t y, uint16_t h, uint32_t background, uint8_t Script_Xnr);

void Draw_Error_line(uint16_t x, uint16_t y, uint32_t background, uint8_t rail_Xnr);

void Draw_Line_Settings(uint16_t x, uint16_t y, uint16_t h, uint32_t background, uint16_t Opt_Xnr);

void set_lang(uint16_t lang_save);
uint8_t get_Lang(void);

#endif
