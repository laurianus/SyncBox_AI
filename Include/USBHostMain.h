
#include "stm32f4xx_hal.h"
#include "options.h"

#define  FILENAME_R  "SCRIPT.CSV"
#define  FILENAME_CFG  "CONFIG.CSV"

#define  FILENAME_AID "IDCHG.CSV"

#define  MAX_LS_BUFFER_SIZE             (256)
#define  MAX_READ_BUFFER_SIZE             (256)

//#define  WRITE_SIZE          (100)
#define MYEOF     26u


#define  MAX_SCRIPT_BUFFER_SIZE        (128)
#define  MAX_NEWEVENTS_EXT             (18)



typedef struct _progXXX{
        
                uint8_t valid;
                uint8_t Mod_ID;
                
                uint8_t lineID;//1
                uint8_t seqID;//1
                
                uint8_t DMX_value;//1
                uint8_t Rmp_Cfg;//1
                uint16_t DMX_Ramp;//2 
                
                uint32_t time;//4         
                
                uint8_t Position;//1
                uint8_t SZone;//1
        
               // char Channel_name[21];
                
        
} dev_progXXX;

typedef struct _lcd_disp_script{
        uint8_t valid;
        char ScriptLCD[MAX_SCRIPT_LENGTH_LINE]; 
        uint32_t time;//4     
        uint16_t is_DMX;
        uint16_t is_Pyro;
        
} lcd_disp_script;



typedef struct _lcd_info_script{
        uint8_t valid;
        uint8_t Mod_Used;
        uint8_t SEQ_Used;
        uint8_t SZ_Used;
        uint16_t Pyro_ev;
        uint16_t DMX_ev;
} lcd_info_script;


/*
**************************************************************************************************************
*                                     FUNCTION PROTOTYPES
**************************************************************************************************************
*/

void host_main(void);
int32_t host_loop(void);

void  Main_Read (void);
void  Main_Write(void);
void  Main_Copy (void);

uint8_t ReadByLine (volatile  uint8_t  *buffer, uint32_t num_bytes);
uint32_t convert_BufToint(uint8_t tmp_ind_posX);
uint8_t decode_line(void);

uint32_t get_next_address(uint8_t Id_tmp_x);

uint8_t ReadPrjName (void);
uint8_t ReadLineZ (volatile  uint8_t  *buffer, uint32_t   num_bytes);
uint8_t check_prj_name(void);



void set_bytes_to_WNN(void);
void rst_ID_Prg(void);

void Save_File_APrg(uint8_t id_prg_ttt);
void rst_USB_Buf_ID(void);
void rst_Uprg_var(void);
uint8_t ReadCfg(void);
uint8_t ReadByLineCFG (volatile  uint8_t  *Zbuffer, uint32_t   num_bytes);
void init_var_USB_lines(void);
uint8_t decode_lineCFG(void);
uint8_t prg_cfg_file(uint32_t tmp_number);
void clear_prog_MOD(void);
uint8_t finish_prg(void);
void set_prg_name(void);
//uint8_t program_line(uint32_t Line_readA, uint8_t Loc_pos_T,uint32_t tmp_number, uint32_t tmp_adr_byte);
uint8_t program_line(uint32_t Line_readA, uint8_t Loc_pos_T,uint32_t tmp_number);
void prg_name_Seq(uint8_t SEQ_length);
void rst_all_usbScripts(void);
void swap_elements_mem_cpy(uint16_t Element_1, uint16_t Element_2);
void bubble_sort(void) ;

void quick_sortX(uint32_t low, uint32_t high);

void  Main_Transfer (uint16_t ID_tmp);

uint8_t get_current_load(void);

void prg_name_ch(uint8_t CH_length, uint8_t ch_id_tmp);
uint8_t update_Script_stats(uint8_t rail_xfds);
void update_script_lcd_X(void);
uint8_t get_free_load(void);
void do_upload_script(void);
void do_LCD_transfer(void);
uint8_t get_free_LCD(void);
void update_script_LCD_load(void);
void init_lcd_script(void);
void update_Multiple(void);
void load_prg_name_LCD(void);
uint8_t update_script_info(void);
uint8_t get_numb_sz(void);
uint8_t get_numb_seq(void);
uint8_t get_numb_mods(void);
void set_script_info(uint8_t pos_to_rst);


uint8_t get_next_event(uint8_t mod_id);
void reset_mod_prg_var(void);
void reset_mod_prg_var_Next(void);
uint8_t fill_nextSeqName(uint8_t pos_disp);
uint8_t get_next_sequence_disp(uint8_t pos_disp);
uint8_t get_max_seq(void);
void get_prev_sequence_active(void);
void get_next_sequence_active(void);
uint8_t get_curLoc_NT(void);
void fire_next_Time(uint32_t dev_time_tmp);
void reset_script_run(void);
void copyScript_toSD (void);
uint8_t get_is_internal_drive(void);
void load_Script_Data(void);
uint8_t get_music_copy_stat(void);
void  Music_Copy (void);
void delete_script_int(void);
void delete_audio_int(void);
uint8_t get_is_file_exists(void);

void rst_first_seq_time(void);
void add_new_event(uint8_t new_ev_gfds, uint8_t pos_in_script);
uint8_t get_next_eventsEXT(uint8_t mod_id);
uint8_t delete_music_int(void);

/*
void swap_elements(uint16_t Element_1, uint16_t Element_2);
void quicksort(int first,int last);
void quick_sortX(uint32_t low, uint32_t high);
uint32_t partition(uint32_t low, uint32_t high);
*/

