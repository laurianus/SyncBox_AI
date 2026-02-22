
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "USBHostMain.h"
#include "Device.h"
#include "math.h"
#include "math.h"
#include "SystemTimer.h"
#include "flash_if.h"
#include "Display.h"


#ifndef VS_READABLE_CODE
#include "System1.h"
#include "System1Action.h"
#include "System1Data.h"
#endif

#include "simpleEventHandler.h"         // Event Queue
#include "System1SEMLibB.h"	   // visualSTATE essentials


uint8_t USB_Stick_menu = 0;
uint16_t LineXa = 0;
uint8_t prj_format = 0;

uint16_t tmp_pyro_ev = 0;
uint16_t tmp_DMX_ev = 0;

extern uint8_t music_not_copied;

uint32_t last_time_saved = 0;

uint16_t DMX_ev = 0;

extern Device_Data device_datas;

extern FATFS SDFatFs;;	/* File system object for SDdisk logical drive */
extern char SDPath[4];         /* SD CARD logical drive path */

extern FATFS USBDISKFatFs;	/* File system object for USB disk logical drive */
extern char USBDISKPath[4];         /* USB Host logical drive path */

uint8_t is_internal_drive = 0;

extern uint8_t current_Slave_For_D_Prg;
extern uint8_t is_UpFailed[MAX_NUMBER_OF_DEVICES];

uint16_t cur_up_pos = 0;
extern uint16_t ev_mod_rem; // events to program for current module
extern uint8_t mod_prg_left;
extern uint8_t Del_or_prg;

uint16_t error_load_script = 0;
uint8_t was_new_start_seq = 0;
uint8_t was_last_seq = 0;

uint16_t save_new_line = 0;
uint8_t was_state_up = 0;

uint32_t Seq_Firs_Time[MAX_SEQ] = {0};


char seq_txts[32] = {0};  /* Was 24 — too small for "XX SEQUENCE XX     R1+16" (27 chars + null) */
extern uint8_t Current_Seq;
uint8_t Current_Seq_D = 1;
//uint8_t Current_Seq_D_tmpX = 0;
uint8_t Current_Seq_D_tmp = 0;


//char pathScript[] = "1:/";
//static char path[] = "0:/";
//for tests
uint32_t Buble_sort_time = 0;
uint32_t enter_time_buble = 0;              
uint32_t Save_flash_time = 0;

uint16_t curent_pos_load = 0;
uint32_t Tot_Bytes_L = 0;

uint8_t tmpMod_ID = 0;
uint16_t tmptime_progcount = 0;

int is_remain_Pyro =  0;
int is_remain_DMX = 0;
uint8_t script_is_end = 0;

uint16_t was_here_up = 0;

//FIL FileRead_Copy_Source;
FIL FileRead_Copy_Dest;

extern uint8_t error_XXCpy; //Error for loading
extern uint32_t Copy_Bytes;
extern uint32_t file_sizeX;
uint32_t percent_i;
uint8_t is_Script_copy = 0;


FIL FileRead_Sx;
//extern char path[];
char pathScript[] = "1:/Script.csv";  

uint8_t was_filed_open = 0;

uint8_t is_open_file_up = 0;

uint8_t cur_loc_script;

lcd_disp_script Script_to_LCD[MAX_SCRIPT_LINES];
char prg_name_LCD[MAX_SCRIPT_LENGTH_LINE_N] = {0};
uint8_t was_lcd_name_load = 0;


uint8_t was_LCD_Script_Changed = 0;


lcd_info_script script_info;


uint32_t buble_i = 0;
uint32_t sort_percent = 0;



FSIZE_t file_sizeS = 0;

uint8_t F_buffer[MAX_READ_BUFFER_SIZE] = {0};
uint8_t line_buffer[MAX_LS_BUFFER_SIZE] = {0};

dev_progXXX ScriptLoad[MAX_SCRIPT_BUFFER_SIZE];

dev_progXXX ScriptUpLoad;

dev_progXXX ScriptUpLoadEXT[MAX_NEWEVENTS_EXT] = {0};
uint16_t cur_up_pos_ext = 0;


uint16_t cur_line_load = 0;



uint32_t line_len = 0;
uint32_t Line_read = 0;
uint32_t Loc_pos_TZ = 0;
char loc_buffer[64] = {0};

uint8_t File_read_error = 0;

#if DEBUG_Y
extern uint8_t valid_script_pos[TimeMatrixDepth];
#endif 

char tmp_prgnameX[21] = {0};
char UserBufferX[36] = {0};
uint8_t prj_lng = 0;

extern Saved_Time_prog savedDatas;

uint16_t line_fill = 0;

uint16_t tmp_loc_X = 0;

extern DIR Directory;

uint32_t Tot_Bytes_S = 0;
uint32_t Tot_Bytes_FS = 0;

uint16_t cur_line = 0;

uint16_t found_tst = 0;
uint16_t nfound_tst = 0;

uint32_t enter_time_tstX = 0;
uint32_t load_time_tstX = 0;

void rst_all_usbScripts(void)
{
        Tot_Bytes_S = 0;
        line_fill = 0;
        tmp_loc_X = 0;
        LineXa = 0;
        prj_format = 0;
        line_len = 0;
        Line_read = 0;
        Loc_pos_TZ = 0;
        USB_Stick_menu = 0;
        cur_line = 0;

         memset(line_buffer, 0x00, sizeof(line_buffer));
         memset(F_buffer, 0x00, sizeof(F_buffer));
         memset(loc_buffer, 0x00, sizeof(loc_buffer));
         memset((uint8_t*)&ScriptLoad, 0x00, sizeof(ScriptLoad)); 
        
      //   f_close(&FileRead_Sx);
        
}


void checkTimings(void)
{
      
        /*
enter_time_tstX = GetCurrentSystemTime();

                                
for(int i = 0; i < TimeMatrixDepth; i++)
        {
                
                if(savedDatas.time_prog.programme[i].time == 20)
                {
                        found_tst++;
                }
                else
                {
                        nfound_tst++;
                }
        
        }
        
for(int i = 0; i < TimeMatrixDepth - 2; i++)
        {
                if(savedDatas.time_prog.programme[i].time == 10)
                {
                        found_tst++;
                }
                else
                {
                        nfound_tst++;
                }
        
        }

for(int i = 0; i < TimeMatrixDepth - 1; i++)
        {
                if(savedDatas.time_prog.programme[i].time == 2)
                {
                        found_tst++;
                }
                else
                {
                        nfound_tst++;
                }
        
        }
load_time_tstX = GetCurrentSystemTime() - enter_time_tstX;
*/
}


uint32_t get_next_address(uint8_t Id_tmp_x){

        /*
       if(Id_tmp_x == 100){
                if(savedDatas.time_prog.programme[curent_pos_load].Address != 0)
                 {
                         curent_pos_load++;
                         return savedDatas.time_prog.programme[curent_pos_load - 1].Address;
                 }
       }
       else{
                for(int i = curent_pos_load; i < TimeMatrixDepth - 2; i++)
                {
                        if(savedDatas.time_prog.programme[i].Address != 0 && (savedDatas.time_prog.M_ID[i] % 100) == Id_tmp_x)
                         {
                                 curent_pos_load = i + 1;
                                 return savedDatas.time_prog.programme[curent_pos_load - 1].Address;
                         }
                }
       }
 */
         return 0;
       
}

uint8_t get_current_load(void)
{
        
        if(cur_line_load + 1 == MAX_SCRIPT_BUFFER_SIZE)
        {
                cur_line_load = 0;
        }
        
        for(int i = cur_line_load + 1; i < MAX_SCRIPT_BUFFER_SIZE; i++)
        {
              if(ScriptLoad[i].valid == 0)
              {
                      cur_line_load = i;
                      return 1; 
              }
        }
        
        for(int i = 0; i < MAX_SCRIPT_BUFFER_SIZE; i++)
        {
              if(ScriptLoad[i].valid == 0)
              {
                      cur_line_load = i;
                      return 1;
              }
        }
        
        return 0;
}


void prg_name_ch(uint8_t CH_length, uint8_t ch_id_tmp)
{
        /*
                for (int k = 0; k < 21; k++)
                {
                        if(k < CH_length)
                        {
                                 ScriptLoad[ch_id_tmp].Channel_name[k] = loc_buffer[k];
                        }
                        else
                        {
                                 ScriptLoad[ch_id_tmp].Channel_name[k] = 0x00;
                        }
                }
        */
}

void do_upload_script(void)
{

}

void do_LCD_transfer(void)
{

        
}

void  Main_Transfer (uint16_t ID_tmp)
 {

         
       FRESULT rc;		/* Result code */
       UINT br;

         if(was_filed_open == 0)
         {
                memset((uint8_t*)&ScriptLoad, 0x00, sizeof(ScriptLoad)); 
                tmp_pyro_ev = 0;
                tmp_DMX_ev = 0;
                memset(&savedDatas,  0x00 , sizeof(savedDatas));  
         }
       if(get_current_load() == 0)
       {
                return;
       }
        
       /*
                if(is_internal_drive == 1){
                        sprintf(pathScript, "0:/Script.csv");
                }else{
                        sprintf(pathScript, "1:/Script.csv");
                }
       */
         
        Tot_Bytes_L = get_next_address(ID_tmp);
        

        if(Tot_Bytes_L != 0)
            {
                       if(was_filed_open == 0)
                       {
                                rst_all_usbScripts();
                                Line_read = 2;
                               
                                rc = f_open(&FileRead_Sx, pathScript , FA_READ);
                       }

                       if(rc == FR_OK || was_filed_open == 1) 
                              {
                                was_filed_open = 2;
                                file_sizeS = f_size(&FileRead_Sx);
                                       
                                       if(Tot_Bytes_L < file_sizeS)
                                       {
                                               rc = f_lseek(&FileRead_Sx, Tot_Bytes_L); 
                                               if(rc != FR_OK){
                                                        f_close(&FileRead_Sx);  
                                                        //memset(&FileRead_Sx, 0x00, sizeof(FileRead_Sx));
                                                        rc = f_open(&FileRead_Sx, pathScript , FA_READ);    
                                                        if(rc == FR_OK){
                                                                rc = f_lseek(&FileRead_Sx, Tot_Bytes_L);   
                                                        }else{
                                                                //f_close(&FileRead_Sx); 
                                                        }
                                               }
                                               
                                                rc = f_read(&FileRead_Sx, F_buffer, sizeof F_buffer, &br);
                                                ReadByLine(F_buffer, br);
                                       }
                                       else
                                        {
                                              //error
                                                Tot_Bytes_L = 0;
                                                file_sizeS = 0;
                                                was_filed_open = 10;
                                                
                                        
                                        }
                                       
                             }
                             was_filed_open = 1;
                }

 }

void  Main_Copy (void)
 {
//       uint8_t tst_xxx = 0;
//       uint8_t percent_i_MT = 100;

       FRESULT rc, rw;		/* Result code */
         

        uint8_t was_screen_filled = 0; 
        enter_time_tstX = GetCurrentSystemTime();  
        
  //      uint8_t percent_i_MT = 100;
         
     //  copyScript_toSD();  
         
       //static char path[] = "0:/";  
         
       //f_mount(&USBDISKFatFs, "1:/", 1); 
       clear_prog_MOD();// delete curent programing
       
         
       UINT br, bw;

       rst_all_usbScripts();
       rst_first_seq_time();
       //enter_time_tstX = GetCurrentSystemTime();  
         
       //rc = f_opendir(&Directory, pathScript);
       
       USB_Drive_menu(1);
         
       sprintf(pathScript, "1:/Script.csv");
//       uint8_t was_screen_filled = 0; 
       error_load_script = 0;
         
                       rc = f_open(&FileRead_Sx, pathScript , FA_READ);
                    
                       if(rc == FR_OK) 
                               {
                                file_sizeS = f_size(&FileRead_Sx);

                                rw = f_open(&FileRead_Copy_Dest, "0:/Script.csv", FA_WRITE | FA_CREATE_ALWAYS);
                                  if(rw == FR_OK && file_sizeS > 0){   
                                        for (;;) {
                                                rc = f_read(&FileRead_Sx, F_buffer, sizeof F_buffer, &br); /* Read a chunk of data from the source file */
                                                         if (rc || !br) {//error OR END OF FILE
                                                                //error_XXCpy = 5;
                                                                break; /* error or eof */
                                                        }
                                                if(rw == FR_OK && rc == FR_OK){
                                                                rc = f_write(&FileRead_Copy_Dest, F_buffer, br, &bw);           /* Write it to the destination file */
                                                                if (bw < br) {//error
                                                                        error_XXCpy = 2;
                                                                        break; /* error or eof */
                                                                }
                                                                
                                                                
                                                                
                                                                if(was_screen_filled == 0){
                                                                                was_screen_filled = 1;
                                                                                //lcd update
                                                                        }
                                                                        
                                                                        
                                                                        
                                                                
                                                                /*
                                                                Copy_Bytes =  Copy_Bytes + sizeof (F_buffer);
                                                                
                                                                
                                                                if(percent_i_MT == 100)   
                                                                {
                                                                        percent_i_MT = 0;
                                                                        sort_percent = (100 * Copy_Bytes)/file_sizeS;
                                                                        
                                                                        uint8_t Percent_tmp = 0;
                                                                        

                                                                        if(was_screen_filled == 0){
                                                                                was_screen_filled = 1;
                                                                                Draw_Line_Script_Wait(0,240, 40, BLACK, 10);
                                                                                HAL_Delay(DISPLAY_UPDATE_TIME);
                                                                                Draw_Line_Script_Wait(0,280, 40, BLACK, 10);
                                                                                HAL_Delay(DISPLAY_UPDATE_TIME);
                                                                        }
                                                                        
                                                                        Percent_tmp = 200 + sort_percent;
                                                                        Draw_Line_Script_Wait(0,200, 40, WHITE, Percent_tmp);
                                                                }
                                                                else
                                                                {
                                                                        percent_i_MT++;
                                                                }
                                                                */
                                                        }
                                                }
                                                f_lseek(&FileRead_Sx, 0); 
                                                
                                                f_close(&FileRead_Sx);
                                                f_close(&FileRead_Copy_Dest);
                                                
                                                rc = f_open(&FileRead_Sx, "0:/Script.csv" , FA_READ);
                                                if(rc == FR_OK){
                                                        file_sizeS = f_size(&FileRead_Sx);
                                                        is_internal_drive = 1;
                                                }
                                                else {
                                                        f_close(&FileRead_Sx);
                                                        rc = f_open(&FileRead_Sx, "1:/Script.csv" , FA_READ);
                                                        if(rc == FR_OK){
                                                                file_sizeS = f_size(&FileRead_Sx);
                                                                
                                                        }else{
                                                                file_sizeS = 0;
                                                        }
                                                        is_internal_drive = 0; 
                                                }
                                                
                                                
                                                
                                        }
                                  else{
                                        //could not copy
                                  }
                                        enter_time_tstX = GetCurrentSystemTime();  
                                        load_Script_Data();
                        }  
                        else
                        {
                                       sprintf(pathScript, "0:/Script.csv");
         
                                       rc = f_open(&FileRead_Sx, pathScript , FA_READ);
                    
                                       if(rc == FR_OK) {
                                               file_sizeS = f_size(&FileRead_Sx);
                                                enter_time_tstX = GetCurrentSystemTime();  
                                                load_Script_Data();
                                       }
                                       else{
                                                USB_Stick_menu = 3;// file could not be open
                                                //lcd update
                                       }
                        }   
}


uint8_t ReadByLine (volatile  uint8_t  *Zbuffer, uint32_t   num_bytes)
 {
       uint32_t i = 0;
       uint32_t tmp_locX = 0;
       
        memset(line_buffer, 0x00, sizeof(line_buffer));
       
       while(i<num_bytes)
        {
              Tot_Bytes_S++;
              line_buffer[line_len++] = Zbuffer[i++];
              
              
             // if(Zbuffer[i-1] == '\n' || tmp_locX > 8)			//	Detected a new line
               if(Zbuffer[i-1] == '\n')			//	Detected a new line
               {
                     if(prj_format != 10)
                      {
                            if(Line_read == 0)
                             {
                                   
                                   set_prg_name();
                             }
                             else if(Line_read == 1)
                             {
                                      // DO nothing - script header
                             }
                            else
                             {
                                   if(decode_line() == 10)
                                    {
                                          return 10;
                                    }
                                   
                             }
                            line_len = 0;
                            Line_read++;
                            Loc_pos_TZ = 0;
                            tmp_locX = 0;
                      }
                     else
                      {
                            if(Line_read > 1)
                             {
                                   if(decode_line() == 10)
                                    {
                                          return 10;
                                    }
                                   
                             }
                            
                            line_len = 0;
                            Line_read++;
                            Loc_pos_TZ = 0;
                            tmp_locX = 0;
                            
                      }
                     
                 memset(line_buffer, 0x00, sizeof(line_buffer));
                break;
               }
              
              if(line_buffer[i] == ',' || line_buffer[i] == ';')
               {
                     tmp_locX++;
               }
        }
       
       return 0;
       
 }


uint8_t ReadByLineOLD (volatile  uint8_t  *Zbuffer, uint32_t num_bytes)
 {
       uint32_t i = 0;
       uint32_t tmp_locX = 0;
       line_len = 0;
         
      memset(line_buffer, 0x00, sizeof(line_buffer));
       
       Tot_Bytes_FS = Tot_Bytes_S;
       while(i < num_bytes)
        {
              Tot_Bytes_S++;
                
              line_buffer[line_len++] = Zbuffer[i++];
              
              
              if(Zbuffer[i-1] == '\n')			//	Detected a new line
               {
                      
                            if(Line_read == 0)
                             {
                                   set_prg_name();
                             }
                             else if(Line_read == 1)
                             {
                                  //do nothing because it is header
                             }
                            else
                             {
                                   if(decode_line() == 10)
                                    {
                                          return 10;
                                    }
                             }
                            line_len = 0;
                            Line_read++;
                            Loc_pos_TZ = 0;
                            tmp_locX = 0;

                     
                        memset(line_buffer, 0x00, sizeof(line_buffer));
                        break;
               }
              
              if(line_buffer[i] == ',')
               {
                     tmp_locX++;
               }
        }
       
       return 0;
       
 }



 
uint8_t decode_line(void)
 {
       
       uint8_t tmp_ind_pos = 0;
       uint8_t code_ret = 0;
       
       for(int a = 0; a < line_len - 1; a++)
        {
              
              if(line_buffer[a] != ',' && line_buffer[a] != ';' && line_buffer[a] != '\r')
               {
                     if(Loc_pos_TZ < 12 && tmp_ind_pos < 21)
                      {
                            loc_buffer[tmp_ind_pos] = line_buffer[a];
                            tmp_ind_pos++;
                      }
               }
              else
               {
                     if(Loc_pos_TZ < 10)
                      {
                            if(tmp_ind_pos < 15 && tmp_ind_pos > 0)
                             {
                                   uint32_t tmp_number = convert_BufToint(tmp_ind_pos);
                                   
                                  // if(Loc_pos_TZ == 0 && id_was_set != 2)
                                    if(Loc_pos_TZ == 0)
                                    {
                                          if((tmp_number > 10 && tmp_number < 200) || tmp_number > 210)// error ID
                                           {
                                                 USB_Stick_menu = 12;
                                                 return 12;
                                           }
                                          
                                           /*
                                          if(id_was_set == 0)
                                           {
                                                 
                                                 if (is_ID_prg(tmp_number) == 0)
                                                  {
                                                        set_itf_ID(tmp_number);
                                                        ID_mod_t = tmp_number;
                                                        id_was_set = 1;
                                                        more_ID_PRG = 1;
                                                  }
                                           }
                                          else if(more_ID_PRG == 1 && id_was_set == 1 && tmp_number != ID_mod_t)
                                           {
                                                 if (is_ID_prg(tmp_number) == 0)
                                                  {
                                                        more_ID_PRG = 2;
                                                  }
                                                 
                                           }
                                           */
                                    }
                                   
                                   
                                   code_ret = program_line(Line_read, Loc_pos_TZ,tmp_number);
                                   
                                   if (code_ret >= 10)
                                    {
                                          return 10;
                                    }
                                   else if (code_ret == 0)
                                    {
                                          tmp_ind_pos = 0;
                                          break;
                                    }
                                   else
                                    {
                                          //do nothing
                                    }
                             }
                            else
                             {
                                   //error
                                   return 0;
                                   
                             }
                      }
                     else if (Loc_pos_TZ == 10)// Channel Name on position 10
                      {
                            //prg_name_ch(tmp_ind_pos);
                            
                      }
                     else if (Loc_pos_TZ == 11)// Seqeunce Name on position 11
                      {
                            prg_name_Seq(tmp_ind_pos);
                            
                      }                     
                     else
                      {
                            
                            //do nothing
                      }
                     tmp_ind_pos = 0;
                     Loc_pos_TZ++;
               }
              if(Loc_pos_TZ > 11)
               {
                     break;
               }
              
        }
       return 1;
 }
 
 
 

uint8_t decode_line_OLD(void)
 {
       
       uint8_t tmp_ind_pos = 0;
       uint8_t code_ret = 0;
       Loc_pos_TZ = 0;
         
       memset(loc_buffer, 0x00, sizeof(loc_buffer));     
         
       for(int a = 0; a < line_len - 1; a++)
        {
              
              if(line_buffer[a] != ',' && line_buffer[a] != '\r')
               {
                     if(Loc_pos_TZ < 12 && tmp_ind_pos < 21)
                      {
                            loc_buffer[tmp_ind_pos] = line_buffer[a];
                            tmp_ind_pos++;
                      }
               }
              else
               {
                     if(Loc_pos_TZ < 10)
                      {
                            if(tmp_ind_pos < 15 && tmp_ind_pos > 0)
                             {
                                   uint32_t tmp_number = convert_BufToint(tmp_ind_pos);
                                   
                                   if(Loc_pos_TZ == 0)
                                    {
                                          if(tmp_number == 0 || tmp_number > (MAX_NUMBER_OF_DEVICES - 1))// error ID
                                           {
                                                 USB_Stick_menu = 12;
                                                 return 12;
                                           }
                                    }
                                   
                                   
                               //    code_ret = program_line(Line_read, Loc_pos_TZ,tmp_number, Tot_Bytes_FS);
                                   
                                   if (code_ret >= 10)
                                    {
                                          return 10;
                                    }
                                   else if (code_ret == 0)
                                    {
                                          tmp_ind_pos = 0;
                                          break;
                                    }
                                   else
                                    {
                                          //do nothing
                                    }
                             }
                            else
                             {
                                   //error
                                   return 0;
                                   
                             }
                      }
                     else if (Loc_pos_TZ == 10)// Channel Name on position 10
                      {
                               if(was_filed_open == 2)
                               {
                                    prg_name_ch(tmp_ind_pos, cur_line_load);
                               }
                      }
                     else if (Loc_pos_TZ == 11)// Seqeunce Name on position 11
                      {
                            prg_name_Seq(tmp_ind_pos);
                      }                     
                     else
                      {
                            
                            //do nothing
                      }
                     tmp_ind_pos = 0;
                     Loc_pos_TZ++;
               }
              if(Loc_pos_TZ > 11)
               {
                     break;
               }
              
        }
       return 1;
 }



uint32_t convert_BufToint(uint8_t tmp_ind_posX)
 {
       uint32_t chValT = 0;
       static uint8_t chValR = 0;
       static uint32_t chValP = 0;
       for(int m = 0; m < tmp_ind_posX; m++)
        {
              if ((loc_buffer[m] >= '0') && (loc_buffer[m] <= '9')) {
                    chValR = loc_buffer[m] - '0';
                    chValP = tmp_ind_posX - 1 - m;
                    chValT = (int) (chValT + chValR * pow(10 , chValP));
                    
              }
              else
               {
                     chValT = 86400001;
                     return chValT;
                     // to do error on line X
               }
              
        }
       
       return chValT;
 }







void set_bytes_to_WNN(void) // without name on script file
 {
       
       UserBufferX[0] = '$'; 
       UserBufferX[1] = '$';
       UserBufferX[2] = '$'; 
       UserBufferX[3] = 'N';
       UserBufferX[4] = 'O'; 
       UserBufferX[5] = ' ';
       UserBufferX[6] = 'N'; 
       UserBufferX[7] = 'A';
       UserBufferX[8] = 'M'; 
       UserBufferX[9] = 'E';
       UserBufferX[10] = '$'; 
       UserBufferX[11] = '$'; 
       UserBufferX[12] = '$'; 
       UserBufferX[13] = '\r';

 }

void rst_ID_Prg(void)
 {

       for(int i = 0; i < 100; i++)
        {
              savedDatas.time_prog.PRG_ID[i] = 0;
        }
       
       for(int j = 0; j < 21; j++)
        {
               savedDatas.Prg_Name[j] = 0;
        }

       
 }


void rst_USB_Buf_ID(void)
 {
       
       for(int j = 0; j < 36; j++)
        {
              UserBufferX[j] = 0;
        }
       
       
 }

void rst_Uprg_var(void)
 {
       line_len = 0;
       Line_read = 0;
       Loc_pos_TZ = 0;
       prj_lng = 0;
       
 }

void clear_prog_MOD(void)
 {
       memset((uint8_t*)&savedDatas, 0x00, sizeof(savedDatas));
         #if DEBUG_Y
                memset(valid_script_pos, 0x00, sizeof(valid_script_pos));
         #endif

 }
 
 uint8_t finish_prg_OLD(void)
 {
#if DBG_TST_X                    
       savedDatas.is_DMX_events =  tmp_DMX_ev;
       //savedDatas.is_Pyro_events =  tmp_pyro_ev;
                    

       
        savedDatas.time_prog.count = tmptime_progcount;

        //is_remain_Pyro =  savedDatas.is_Pyro_eventsS0;
        is_remain_DMX = savedDatas.is_DMX_eventsS0;
        script_is_end = 0;
        
         
        if (savedDatas.time_prog.count != tmp_DMX_ev)
        {
              USB_Stick_menu = 13;
              savedDatas.is_R_Prg = 0;
              return 0; // error programing
        
        }
        if(savedDatas.time_prog.count != 0)
        {
                savedDatas.is_R_Prg = 1;
        }
        else
        {
              USB_Stick_menu = 13;
              savedDatas.is_R_Prg = 0;
              return 0; // error programing
        }
        
        //savedDatas.is_Mods = get_numb_mods();
        savedDatas.is_SafetyZones = get_numb_sz();
        savedDatas.is_Sequence = get_numb_seq();
        

        memset(&script_info,  0x00 , sizeof(script_info));
       // init_lcd_script();
#endif
 

       return 1;
 }
 
 uint8_t get_numb_mods(void)
 {
         uint8_t numb_mod_tmp = 0;
         
         for( int i = 1; i < MAX_NUMBER_OF_DEVICES; i++)
         {
                 //#if DBG_TST_X 
                if(savedDatas.time_prog.PRG_ID[i] != 0)
                {
                        numb_mod_tmp++;
                }
                 //#endif
         }
         return numb_mod_tmp;
 }
 
 
  uint8_t get_numb_seq(void)
 {
         uint8_t numb_mod_tmp = 0;
         
         for( int i = 1; i < MAX_SEQ; i++)
         {
//                 #if DBG_TST_X 
                if(savedDatas.time_prog.count_seq[i] != 0)
                {
                        numb_mod_tmp++;
                }
  //               #endif
         }
         return numb_mod_tmp;
 }
 
 
 uint8_t get_numb_sz(void)
 {
         uint8_t numb_mod_tmp = 0;
         
         for( int i = 0; i < MAX_SZ; i++)
         {
//                 #if DBG_TST_X 
                if(savedDatas.time_prog.Safe_Zone[i] != 0)
                {
                        numb_mod_tmp++;
                }
  //              #endif
         }
         return numb_mod_tmp;
 }

 
 void set_prg_name(void)
 {
//         #if DBG_TST_X 
        for (int i = 0; i < (sizeof(line_buffer) - 2); i++)
        {
              savedDatas.Prg_Name[i] = line_buffer[i + 2];
                if(i > MAX_SCRIPT_LENGTH_LINE_N -  1)
                {
                        break;
                }
        }
 //       #endif
 }
 
 
void load_prg_name_LCD(void)
{
//        #if DBG_TST_X 
        if (was_lcd_name_load == 1) return; 
        
        for (int i = 0; i < MAX_SCRIPT_LENGTH_LINE_N; i++)
        {
              if(savedDatas.Prg_Name[i] == ',') break;
              prg_name_LCD [i] = savedDatas.Prg_Name[i];
        }
        
        was_lcd_name_load = 1; 
        was_LCD_Script_Changed = was_LCD_Script_Changed | 0x20;
//#endif
}
 
VS_VOID clearTimeMatrix()
{
//        #if DBG_TST_X 
        //first_el = 0;
 //       device_datas.programmed_interface = 0;
        savedDatas.is_DMX_events = 0;
        savedDatas.is_Pyro_events = 0;
        memset(&savedDatas.time_prog, 0x00, sizeof(savedDatas.time_prog));
   //     clearStatusMatrixB();
        
  //      #endif
}
 
uint8_t program_line(uint32_t Line_readA, uint8_t Loc_pos_T,uint32_t tmp_number)
{
        
        //for tests
        /*
        static uint32_t Line_readAZ = 0;
        static uint32_t Loc_pos_TZ = 0;
        static uint32_t tmp_numberZ = 0;
        static uint32_t is_error_asdf = 0;
        
        Line_readAZ = Line_readA;
        Loc_pos_TZ =  Loc_pos_T;
        tmp_numberZ = tmp_number;
        
        
        if(Line_readAZ == 1827)
        {
        is_error_asdf = 1;
}
        //end tests
        */
        
        static uint16_t line_fill = 0;
        static uint16_t cur_line = 0;
        
        if((Loc_pos_T == 0 && (tmp_number == (200 + get_AB_Adre()) || tmp_number == get_AB_Adre())) || Loc_pos_T > 0)
        {
                if(tmp_loc_X % 10 == 0)
                {
                        USB_Stick_menu = 4;
                       // load_refresh();
                }
                if(cur_line == 0)
                {
                        clearTimeMatrix();
                        cur_line = Line_readA;
                        tmp_loc_X = 0;
                        line_fill = 0;
                        USB_Stick_menu = 4;
                        //load_refresh();
                        
                }
                else if(Line_readA != cur_line)
                {
                        if(line_fill == 3 && tmp_loc_X < TimeMatrixDepth)
                        {
                                cur_line = Line_readA;
                                line_fill = 0;
                                tmp_loc_X++;
                        }
                        else
                        {
                                USB_Stick_menu = 10;
                                if(tmp_loc_X >= TimeMatrixDepth)
                                {
                                        return 10; // error programing
                                }
                                
                        }      
                        
                }
                
                if(Loc_pos_T > 3 && line_fill < 3)
                {
                        USB_Stick_menu = 12;
                        return 12; // error programing
                        
                }
                
                dev_prog *tmp = &(savedDatas.time_prog.programme[tmp_loc_X]);
                
                switch (Loc_pos_T) {
                case 0:
                        if(line_fill == 0 && ((tmp_number > 200 && tmp_number < 211) || tmp_number < 11))
                        {
                                //tmp->Mod_ID = 0;
                                tmp->lineID = 0;
                                tmp->valid = 0;
                                tmp->time = 0;
                                tmp->seqID = 0;
                                tmp->DMX_value = 0;
                                tmp->Rmp_Cfg = 0;
                                line_fill = 1;
                        }
                        else
                        {
                                USB_Stick_menu = 12;
                                return 12; // error programing
                                
                        }
                        
                        break;
                        
                case 1:
                        if(line_fill == 1 && tmp_number > 0 && tmp_number < PYRO_RAIL_MAX + 1)
                        {
                                //tmp->Mod_ID = device_datas.device_Address;
                                tmp->lineID = 16 * (tmp_number - 1);
                                line_fill = 2;
                        }
                        else
                        {
                                USB_Stick_menu = 12;
                                return 12; // error programing
                                
                        }      
                        
                        break;
                        
                case 2:
                        if(line_fill == 2)
                        {
                                if(tmp_number > 0 && tmp_number < PYRO_CH_MAX + 1)
                                {
                                        savedDatas.is_Pyro_events = 1;
                                        tmp->lineID = tmp->lineID + tmp_number;
                                        line_fill = 3;
                                }
                                else if(tmp_number > 100 && tmp_number < 100 + DMX_CH_MAX)
                                {
                                        savedDatas.is_DMX_events = 1;
                                        tmp->lineID = tmp_number;
                                        line_fill = 3;
                                }  
                                else
                                {
                                        USB_Stick_menu = 12;
                                        return 12; // error programing
                                        
                                } 
                        }
                        else
                        {
                                USB_Stick_menu = 12;
                                return 12; // error programing
                                
                        }      
                        
                        
                        break;
                        
                case 3://Time
                        
                        if(line_fill == 3 && tmp_number < 86400001) 
                        {
                                if(tmp_number == 0){
                                        tmp_number = 10;
                                }
                                tmp->valid = 1;
                                tmp->time = tmp_number;
                                line_fill = 3;
                        }
                        else
                        {
                                USB_Stick_menu = 12;
                                return 12; // error programing
                                
                        }      
                        
                        break; 
                        
                case 4://Sequence
                        if(tmp_number < MAX_SEQ) 
                        {
                                tmp->seqID = tmp_number;
                        }
                        else
                        {
                                USB_Stick_menu = 12;
                                return 12; // error programing
                                
                        }      
                        break;
                        
                case 5://DMX channel value
                        if(tmp_number < 256){ 
                                tmp->DMX_value = tmp_number;
                        }
                        else
                        {
                                USB_Stick_menu = 12;
                                return 12; // error programing
                        }  
                        
                        break;
                        
                case 6://Rmp_Cfg
                        if(tmp_number < 250){
                                tmp->Rmp_Cfg = tmp_number;
                        }
                        else
                        {
                                USB_Stick_menu = 12;
                                return 12; // error programing
                        }  
                        break;  
                case 7://DMX Ramp
                        if(tmp_number < 65500){
                                tmp->DMX_Ramp = tmp_number;
                        }
                        else
                        {
                                USB_Stick_menu = 12;
                                return 12; // error programing
                        }
                        
                        break;  
                case 8://Position
                     //   tmp->Position = tmp_number;
                        
                        break;  
                case 9://SafetyZone
                        if(tmp_number < MAX_SAFETY_ZONES + 1)
                        {
                                tmp->Safe_Zone = tmp_number;
                        }
                        else
                        {
                                USB_Stick_menu = 12;
                                return 12; // error programing
                        }
                        break;                      
                }
        }
        else
        {
                return 0;
        }
        return 1;
}

uint8_t finish_prg(void)
{
        DMX_ev = 0;
      //  Pyro_ev = 0;
        
        for (int i = 0; i < TimeMatrixDepth; i++) {
                if (savedDatas.time_prog.programme[i].valid == 1)
                {
                     //   dev_prog *tmp = &(savedDatas.time_prog.programme[i]);
                       // savedDatas.time_prog.count_seq[tmp->seqID]++;
                        savedDatas.time_prog.count++;
                        
                        if (savedDatas.time_prog.programme[i].lineID > 100)
                        {
                                DMX_ev++;
                        }
                        else
                        {
        //                        Pyro_ev++;
                        }
                }
        }
        
        if(savedDatas.time_prog.count == 0)
        {
                USB_Stick_menu = 13;
             //   Switch_USB();
                return 0; // error programing
        }
        
        if(tmp_loc_X + 1 == savedDatas.time_prog.count)
        {
                device_datas.programmed_interface = 1;
                
            //    CalculateMD5();                
                
                __disable_irq();
             //   saveFlashData();//OK
             //   saveFlashDataCFG();
                
                __enable_irq();
                
               
                USB_Drive_menu(2);
                
                USB_Stick_menu = 5;
                
        }
        else
        {
                USB_Stick_menu = 11;
             //   Switch_USB();
                return 0; // error programing
                
        }   
     //   Switch_USB();
        return 1;
}



uint8_t program_line_OLD(uint32_t Line_readA, uint8_t Loc_pos_T,uint32_t tmp_number, uint32_t tmp_adr_byte)
 {
         
  static uint16_t line_fill = 0;
        static uint16_t cur_line = 0;
        
        if(Loc_pos_T == 0 || Loc_pos_T > 0)
        {
                if(tmp_loc_X % 10 == 0)
                {
                        USB_Stick_menu = 4;
                       // load_refresh();
                }
                if(cur_line == 0)
                {
                        clearTimeMatrix();
                        cur_line = Line_readA;
                        tmp_loc_X = 0;
                        line_fill = 0;
                        USB_Stick_menu = 4;
                       // load_refresh();
                        
                }
                else if(Line_readA != cur_line)
                {
                        if(line_fill == 3 && tmp_loc_X < TimeMatrixDepth)
                        {
                                cur_line = Line_readA;
                                line_fill = 0;
                                tmp_loc_X++;
                        }
                        else
                        {
                                USB_Stick_menu = 10;
                                if(tmp_loc_X >= TimeMatrixDepth)
                                {
                                        return 10; // error programing
                                }
                                
                        }      
                        
                }
                
                if(Loc_pos_T > 3 && line_fill < 3)
                {
                        USB_Stick_menu = 12;
                        return 12; // error programing
                        
                }
                
                dev_prog *tmp = &(savedDatas.time_prog.programme[tmp_loc_X]);
                
                switch (Loc_pos_T) {
                case 0:
                        if(line_fill == 0 && tmp_number > 0 && tmp_number < (MAX_NUMBER_OF_DEVICES - 1))
                        {
                                //tmp->Mod_ID = 0;
                                tmp->lineID = 0;
                                tmp->time = 0;
                                tmp->seqID = 0;
                                tmp->DMX_value = 0;
                                tmp->Rmp_Cfg = 0;
                                //tmp->Position = 0;
                                tmp->Safe_Zone = 0;
                                line_fill = 1;
                        }
                        else
                        {
                                USB_Stick_menu = 12;
                                return 12; // error programing
                                
                        }
                        
                        break;
                        
                case 1:
                        if(line_fill == 1 && tmp_number > 0 && tmp_number < 100)
                        {
                                //tmp->Mod_ID = device_datas.device_Address;
                                //tmp->lineID = 16 * (tmp_number - 1);
                                line_fill = 2;
                        }
                        else
                        {
                                USB_Stick_menu = 12;
                                return 12; // error programing
                                
                        }      
                        
                        break;
                        
                case 2:
                        if(line_fill == 2)
                        {
                                if(tmp_number > 100 && tmp_number < 256 + 100)
                                {
//                                        #if DBG_TST_X 
                                        savedDatas.is_DMX_events = 1;
                                       // #endif
                                        tmp->lineID = tmp_number - 100;
                                        line_fill = 3;
                                        
                                }  
                                else
                                {
                                        USB_Stick_menu = 12;
                                        return 12; // error programing
                                        
                                } 
                        }
                        else
                        {
                                USB_Stick_menu = 12;
                                return 12; // error programing
                                
                        }      
                        
                        
                        break;
                        
                case 3://Time
                        
                        if(line_fill == 3 && tmp_number < 86400001) 
                        {
                                if(tmp_number == 0){
                                        tmp_number = 10;
                                }
                               // tmp->valid = 1;
                                tmp->time = tmp_number;
                                line_fill = 3;
                        }
                        else
                        {
                                USB_Stick_menu = 12;
                                return 12; // error programing
                                
                        }      
                        
                        break; 
                        
                case 4://Sequence
                        if(tmp_number < MAX_SEQ) 
                        {
                                tmp->seqID = tmp_number;
                                //line_fill = 5;
                        }
                        else
                        {
                                USB_Stick_menu = 12;
                                return 12; // error programing
                                
                        }      
                        break;
                        
                case 5://DMX channel value
                        if(tmp_number < 256){ 
                                tmp->DMX_value = tmp_number;
                                //line_fill = 6;
                        }
                        else
                        {
                                USB_Stick_menu = 12;
                                return 12; // error programing
                        }  
                        
                        break;
                        
                case 6://Rmp_Cfg
                        if(tmp_number < 250){
                                tmp->Rmp_Cfg = tmp_number;
                                //line_fill = 7;
                        }
                        else
                        {
                                USB_Stick_menu = 12;
                                return 12; // error programing
                        }  
                        break;  
                case 7://DMX Ramp
                        if(tmp_number < 65500){
                                tmp->DMX_Ramp = tmp_number;
                                //line_fill = 8;
                        }
                        else
                        {
                                USB_Stick_menu = 12;
                                return 12; // error programing
                        }
                        
                        break;  
                case 8://Position
                         if(tmp_number < 256){ 
                                //tmp->Position = tmp_number;
                                // line_fill = 9;
                         }
                        
                        break;  
                case 9://SafetyZone
                        if(tmp_number < MAX_SAFETY_ZONES + 1)
                        {
                                tmp->Safe_Zone = tmp_number;
                                //line_fill = 0;
                        }
                        else
                        {
                                USB_Stick_menu = 12;
                                return 12; // error programing
                        }
                        break;                      
                }
        }
        else
        {
                return 0;
        }
        return 1;
 }

 
 
void prg_name_Seq(uint8_t SEQ_length)
{

//#if DBG_TST_X 
       if(was_new_start_seq < MAX_SEQ && was_new_start_seq > 0)
        {
              for (int k = 0; k < MAX_SEQ_NAME; k++)
               {
                     if(k < SEQ_length)
                      {
                            savedDatas.Seq_Name[was_new_start_seq][k] = loc_buffer[k];
                      }
                     else
                      {
                            savedDatas.Seq_Name[was_new_start_seq][k] = 0x00;
                      }
               }
        }
        
     //   #endif

}


uint8_t fill_nextSeqName(uint8_t pos_disp){
        
//        #if DBG_TST_X 
        
        char buffer[24] = {0};
        
       

         if(pos_disp == 0){
                Current_Seq_D_tmp = get_next_sequence_disp(Current_Seq_D);
         }
         else{
                Current_Seq_D_tmp = get_next_sequence_disp(Current_Seq_D_tmp + 1);
                //Current_Seq_D_tmp = Current_Seq_D;
         }
        
        
        if(Current_Seq_D_tmp == 0 || (Current_Seq_D_tmp == 1 && pos_disp == 3)){
                was_last_seq++;
                sprintf(seq_txts, "                        ");
        }
        else{
                
                memset(seq_txts, 0x00, sizeof(seq_txts));
                snprintf(seq_txts, sizeof(seq_txts), "%d%d ", Current_Seq_D_tmp / 10, Current_Seq_D_tmp % 10);


                if(savedDatas.Seq_Name[Current_Seq_D_tmp][0] == 0){
                        memset(buffer, 0x00, sizeof(buffer));

                        snprintf(buffer, sizeof(buffer), "SEQUENCE %d%d    ", Current_Seq_D_tmp / 10, Current_Seq_D_tmp % 10);
                        strncat(seq_txts, buffer, sizeof(seq_txts) - strlen(seq_txts) - 1);
                }
                else{
                        for(int i = MIN_SEQ_START_NAME; i < MIN_SEQ_START_NAME + MAX_SEQ_NAME && i < (int)(sizeof(seq_txts) - 1); i++){

                                seq_txts[i] = (savedDatas.Seq_Name[Current_Seq_D_tmp][i - MIN_SEQ_START_NAME]);
                                if(seq_txts[i] == 0){
                                        seq_txts[i] = ' ';
                                }
                        }
                }

                memset(buffer, 0x00, sizeof(buffer));
                uint8_t tmp_press_b = Current_Seq_D_tmp % 16;
                if(tmp_press_b == 0){
                        tmp_press_b = 16;
                }
                uint8_t tmp_press_b2 = 0;

                if(Current_Seq_D_tmp > 0){
                        tmp_press_b2 = (Current_Seq_D_tmp -1) / 16;
                }
                else{
                        tmp_press_b2 = (Current_Seq_D_tmp) / 16;
                }


                snprintf(buffer, sizeof(buffer), " R%d+%d%d", tmp_press_b2 + 1, tmp_press_b/10, tmp_press_b%10);
                strncat(seq_txts, buffer, sizeof(seq_txts) - strlen(seq_txts) - 1);
        }
        
        return savedDatas.time_prog.seq_valid[Current_Seq_D_tmp];
 //       #else
  //      return 0;
  //      #endif
        
}


uint8_t get_next_sequence_disp(uint8_t pos_disp){
        
        for(int i = pos_disp; i < MAX_SEQ; i++){
//                #if DBG_TST_X 
                if(savedDatas.time_prog.count_seq[i] > 0 ){
                        return i;
                }
          //      #endif
        }
        return 0;
}


void get_next_sequence_active(void){
        
        for(int i = Current_Seq_D + 1; i < MAX_SEQ; i++){
//                #if DBG_TST_X 
                if(savedDatas.time_prog.count_seq[i] > 0 ){
                        Current_Seq_D = i;
                        break;
                }
       //         #endif
        }
}

void get_prev_sequence_active(void){
        
        for(int i = Current_Seq_D - 1; i > 0; i--){
//                #if DBG_TST_X 
                if(savedDatas.time_prog.count_seq[i] > 0 ){
                        Current_Seq_D = i;
                        break;
                }
            //    #endif
        }
}


uint8_t get_max_seq(void){
        for(int i = MAX_SEQ - 1; i > 0; i--){
//                #if DBG_TST_X 
                        if(savedDatas.time_prog.count_seq[i] > 0 ){
                                return i;
                        }
   //             #endif
                }
        return 0;
}

void bubble_sort(void) 
{
 
     /*  
   uint8_t swapped = 0;

   buble_i = 0;
      // buble_j = 0;
    
    for (buble_i = 0; buble_i < tmp_loc_X; buble_i++) {   // loop n times - 1 per element
        swapped = 0;   
        //for (int buble_j = 0; buble_j < tmp_loc_X - buble_i - 1; buble_j++) { // last i elements are sorted already
            for (int buble_j = 0; buble_j < tmp_loc_X - buble_i; buble_j++) { // last i elements are sorted already
            if (savedDatas.time_prog.programme[buble_j].time > savedDatas.time_prog.programme[buble_j + 1].time) {  // swap if order is broken
               //swap_elements(buble_j, buble_j+1);     
               swap_elements_mem_cpy(buble_j, buble_j+1);
               swapped = 1;
            }
        }
        if(buble_i % 100 == 0)   
        {
                sort_percent = (100 * buble_i)/tmp_loc_X;
                DoDisplayTasks();
        }
        if (swapped == 0)        break;
    }
    */
}


void swap_elements_mem_cpy(uint16_t Element_1, uint16_t Element_2)
{
        
        
        uint8_t time_tmp[14] = {0};

        memcpy(time_tmp, &savedDatas.time_prog.programme[Element_1], 12);
        memcpy(&savedDatas.time_prog.programme[Element_1], &savedDatas.time_prog.programme[Element_2], 12);
        memcpy(&savedDatas.time_prog.programme[Element_2], time_tmp, 12);
        

}






uint8_t get_free_load(void)
{

        
        return 1;
}

uint8_t get_free_LCD(void)
{

        return 10;

}

void update_script_LCD_load(void)
{
        
}

void update_script_lcd_X(void)
{
        

}

void init_lcd_script(void)
{


}


void update_Multiple(void)
{

         
}

uint8_t get_curLoc_NT(void){

        uint32_t min_time_tmp = 0xFFFFFFFF;
        uint8_t min_pos_tmp = 0xFF;
        
        for(int i = 0; i < MAX_SCRIPT_BUFFER_SIZE; i++){
                 if(ScriptLoad[i].seqID > 0){
                                ScriptLoad[i].valid = 0;
                        }
                  if(ScriptLoad[i].valid == 1 || ScriptLoad[i].valid == 2){
                        if(ScriptLoad[i].time < min_time_tmp){
                                min_time_tmp = ScriptLoad[i].time;
                                min_pos_tmp = i;
                        }
                  }
        }

        return min_pos_tmp;
}

void fire_next_Time(uint32_t dev_time_tmp){
       for(int i = 1; i < MAX_SCRIPT_BUFFER_SIZE; i++){
                 if(ScriptLoad[i].seqID == 0){
                          if(ScriptLoad[i].valid == 1){
                                if(ScriptLoad[i].time < dev_time_tmp && ScriptLoad[i].time > 0){
                                        start_buz(20);
                                        ScriptLoad[i].valid = 2;
                                }
                          }
                          else if(ScriptLoad[i].valid == 10){
                                if(ScriptLoad[i].time < dev_time_tmp && ScriptLoad[i].time > 0){
                                        start_buz(20);
                                        ScriptLoad[i].valid = 0;
                                }
                          }
                }
        }

}

uint8_t update_Script_stats(uint8_t rail_xfds)
{
        /*
        char tmp_txtF[31] = {0};
        uint8_t cur_loc_script_tmp = 0;
//        uint8_t was_changed = 0;
        uint32_t tmp_timer = 0;
        
       // cur_loc_script_tmp = cur_loc_script;
        
        uint8_t cur_loc_lcd = 0;

        load_prg_name_LCD();

        
        for (;;)
        {
                
                        if(get_free_LCD() == 10)
                        {
                                break;
                        }
                        cur_loc_script_tmp = get_curLoc_NT();
        
                        if(cur_loc_script_tmp == 0xFF) break;
                
                
                        if(ScriptLoad[cur_loc_script_tmp].valid == 1 || ScriptLoad[cur_loc_script_tmp].valid == 2)
                        {
                                if(tmp_timer == 0)
                                {
                                        tmp_timer = ScriptLoad[cur_loc_script_tmp].time /100;
                                        cur_loc_lcd = get_free_LCD();
                                        
                                         //start_buz(10);
                                        
                                       // was_LCD_Script_Changed = was_LCD_Script_Changed | (0x01 << cur_loc_lcd);
                                       // was_LCD_Script_Changed = was_LCD_Script_Changed | 0x1F;
                                        
                                        sprintf(tmp_txtF, "%d %d %s", ScriptLoad[cur_loc_script_tmp].Mod_ID, ScriptLoad[cur_loc_script_tmp].lineID, ScriptLoad[cur_loc_script_tmp].Channel_name);
                                        
                                        tmp_txtF[23] = '1';
                                        Script_to_LCD[cur_loc_lcd].valid = 1;
                                        
                                        
                                        Script_to_LCD[cur_loc_lcd].time = tmp_timer;
                                        
                                                for (int j = 0; j < MAX_SCRIPT_LENGTH_LINE; j++)
                                                {
                                                        if(tmp_txtF[j] == 0x00 && j < 23)
                                                                        {
                                                                                tmp_txtF[j] = ' ';
                                                                        }
                                                        Script_to_LCD[cur_loc_lcd].ScriptLCD[j] = tmp_txtF[j];
                                                }
                                                
                                                if( ScriptLoad[cur_loc_script_tmp].lineID < 100)//is pyro
                                                {
                                                        Script_to_LCD[cur_loc_lcd].is_Pyro++;
                                                }
                                                else
                                                {
                                                        Script_to_LCD[cur_loc_lcd].is_DMX++;
                                                }
                                                
                                                ScriptLoad[cur_loc_script_tmp].valid = 10;//0
                                                
                                }
                                else
                                {
                                        if(tmp_timer == ScriptLoad[cur_loc_script_tmp].time /100)
                                        {
                                                
                                                 Script_to_LCD[cur_loc_lcd].ScriptLCD[23]++;
                                                
                                                if( ScriptLoad[cur_loc_script_tmp].lineID < 100)//is pyro
                                                {
                                                        Script_to_LCD[cur_loc_lcd].is_Pyro++;
                                                }
                                                else
                                                {
                                                        Script_to_LCD[cur_loc_lcd].is_DMX++;
                                                }
                                                
                                                 ScriptLoad[cur_loc_script_tmp].valid = 10; //0;
                                        }
                                        else
                                        {
                                                 cur_loc_lcd = get_free_LCD();
                                                 if(cur_loc_lcd == 10)
                                                 {
                                                        
                                                         
                                                        sprintf(tmp_txtF, "%d %d %s", ScriptLoad[cur_loc_script_tmp].Mod_ID, ScriptLoad[cur_loc_script_tmp].lineID, ScriptLoad[cur_loc_script_tmp].Channel_name);
                                                        tmp_txtF[23] = '1';
                                                        
                                                         
                                                       // was_LCD_Script_Changed = was_LCD_Script_Changed | 0x1F;                                                         
                                                         //was_LCD_Script_Changed = was_LCD_Script_Changed | (0x01 << cur_loc_lcd);
                                                         
                                                         
                                                        Script_to_LCD[cur_loc_lcd].valid = 1;
                                                         

                                                        
                                                        Script_to_LCD[cur_loc_lcd].time = tmp_timer;
                                                         
                                                        for (int j = 0; j < MAX_SCRIPT_LENGTH_LINE; j++)
                                                        {
                                                                if(tmp_txtF[j] == 0x00 && j < 23)
                                                                                {
                                                                                        tmp_txtF[j] = ' ';
                                                                                }
                                                                Script_to_LCD[cur_loc_lcd].ScriptLCD[j] = tmp_txtF[j];
                                                        }
                                                        
                                                        if( ScriptLoad[cur_loc_script_tmp].lineID < 100)//is pyro
                                                        {
                                                                Script_to_LCD[cur_loc_lcd].is_Pyro++;
                                                        }
                                                        else
                                                        {
                                                                Script_to_LCD[cur_loc_lcd].is_DMX++;
                                                        }
                                                        
                                                        ScriptLoad[cur_loc_script_tmp].valid = 10; //0;
                                                 }
                                                 else
                                                 {
                                                        break;
                                                 }

                                        }
                                }
                               // cur_loc_script++;
                        }
                        else
                        {
                             //   break;
                        }
                


                      

        }
//        Area_state = SCRIPT_DISPLAY;
        */
        return 1; 
}

void set_script_info(uint8_t pos_to_rst)
{
        if(pos_to_rst == 0)
        {
                script_info.valid = script_info.valid & 0xFE;
        }
        else if(pos_to_rst == 1)
        {
                script_info.valid = script_info.valid & 0xFD;
        }
        else if(pos_to_rst == 2)
        {
                script_info.valid = script_info.valid & 0xFB;
        }
        else if(pos_to_rst == 3)
        {
                script_info.valid = script_info.valid & 0xF7;
        }
        else if(pos_to_rst == 4)
        {
                script_info.valid = script_info.valid & 0xEF;
        }
        else if(pos_to_rst == 10)
        {
                script_info.valid = 0xFF;
        }
        
}

uint8_t update_script_info(void)
{
        
//        #if DBG_TST_X 
        if(savedDatas.is_R_Prg == 0) return 0;
        
       // script_info.valid = 0;

        if(script_info.Mod_Used != savedDatas.is_Mods)
        {
                script_info.Mod_Used = savedDatas.is_Mods; 
                script_info.valid = script_info.valid | (0x01 << 0);
        }
        
        if(script_info.DMX_ev != savedDatas.is_DMX_events)
        {
                script_info.DMX_ev = savedDatas.is_DMX_events; 
                script_info.valid = script_info.valid | (0x01 << 1);
        }
        
        if(script_info.Pyro_ev != savedDatas.is_Pyro_events)
        {
                script_info.Pyro_ev = savedDatas.is_Pyro_events;
                script_info.valid = script_info.valid | (0x01 << 2);
        }
        
        if(script_info.SEQ_Used != savedDatas.is_Sequence)
        {
                script_info.SEQ_Used = savedDatas.is_Sequence;
                script_info.valid = script_info.valid | (0x01 << 3);
        }
        
        if(script_info.SZ_Used != savedDatas.is_SafetyZones)
        {
                script_info.SZ_Used = savedDatas.is_SafetyZones;
                script_info.valid = script_info.valid | (0x01 << 4);
        }
        
        return script_info.valid;
  //      #else
   //             return 0;
   //     #endif
}


uint8_t get_next_event(uint8_t mod_id){

        /*
        int was_ev_found = -1;
        
        if(cur_up_pos + 1 == MAX_SCRIPT_BUFFER_SIZE)
        {
                cur_up_pos = 0;
        }
        for (int i = cur_up_pos + 1; i < MAX_SCRIPT_BUFFER_SIZE; i++)
        {
                if(ScriptLoad[i].valid > 0)
                {
                        if(ScriptLoad[i].Mod_ID != mod_id)
                        {
                                ScriptLoad[i].valid = 0;
                                ScriptLoad[i].Mod_ID = 0;
                                ScriptLoad[i].lineID = 0;
                                ScriptLoad[i].time = 0;
                                ScriptLoad[i].seqID = 0;
                                ScriptLoad[i].DMX_value = 0;
                                ScriptLoad[i].Rmp_Cfg = 0;
                                ScriptLoad[i].DMX_Ramp = 0;
                                ScriptLoad[i].SZone = 0;
                                ScriptLoad[i].Position = 0;
                                memset(ScriptLoad[i].Channel_name, 0x00, sizeof(ScriptLoad[i].Channel_name));
                        }
                        else
                        {
                                was_ev_found = i;
                                cur_up_pos = i;
                                break;
                        }
                        
                }
        }
        
        if(was_ev_found == -1)
        {
                cur_up_pos = 0;
                for (int i = cur_up_pos; i < MAX_SCRIPT_BUFFER_SIZE; i++)
                {
                        if(ScriptLoad[i].valid > 0)
                        {
                                if(ScriptLoad[i].Mod_ID != mod_id)
                                {
                                        ScriptLoad[i].valid = 0;
                                        ScriptLoad[i].Mod_ID = 0;
                                        ScriptLoad[i].lineID = 0;
                                        ScriptLoad[i].time = 0;
                                        ScriptLoad[i].seqID = 0;
                                        ScriptLoad[i].DMX_value = 0;
                                        ScriptLoad[i].Rmp_Cfg = 0;
                                        ScriptLoad[i].DMX_Ramp = 0;
                                        ScriptLoad[i].SZone = 0;
                                        ScriptLoad[i].Position = 0;
                                        memset(ScriptLoad[i].Channel_name, 0x00, sizeof(ScriptLoad[i].Channel_name));
                                }
                                else
                                {
                                        was_ev_found = i;
                                        cur_up_pos = i;
                                        break;
                                }
                                
                        }
                }
        
        }
        else{
                //do nothing
        }
        
        if(was_ev_found > -1)
        {
                        ScriptUpLoad.valid = ScriptLoad[was_ev_found].valid;
                        ScriptUpLoad.Mod_ID = ScriptLoad[was_ev_found].Mod_ID;
                        ScriptUpLoad.lineID = ScriptLoad[was_ev_found].lineID;
                        ScriptUpLoad.time = ScriptLoad[was_ev_found].time;
                        ScriptUpLoad.seqID = ScriptLoad[was_ev_found].seqID;
                        ScriptUpLoad.DMX_value = ScriptLoad[was_ev_found].DMX_value;
                        ScriptUpLoad.Rmp_Cfg = ScriptLoad[was_ev_found].Rmp_Cfg;
                        ScriptUpLoad.DMX_Ramp = ScriptLoad[was_ev_found].DMX_Ramp;
                        ScriptUpLoad.SZone = ScriptLoad[was_ev_found].SZone;
                        ScriptUpLoad.Position = ScriptLoad[was_ev_found].Position;
                        
                        ScriptLoad[was_ev_found].valid = 0;
                        ScriptLoad[was_ev_found].Mod_ID = 0;
                        ScriptLoad[was_ev_found].lineID = 0;
                        ScriptLoad[was_ev_found].time = 0;
                        ScriptLoad[was_ev_found].seqID = 0;
                        ScriptLoad[was_ev_found].DMX_value = 0;
                        ScriptLoad[was_ev_found].Rmp_Cfg = 0;
                        ScriptLoad[was_ev_found].DMX_Ramp = 0;
                        ScriptLoad[was_ev_found].SZone = 0;
                        ScriptLoad[was_ev_found].Position = 0;
                        memset(ScriptLoad[was_ev_found].Channel_name, 0x00, sizeof(ScriptLoad[was_ev_found].Channel_name));
                        return 1;
         }
        else{
                return 0;
                // no more events
        }
        */
        return 0;
}




uint8_t get_next_eventsEXT(uint8_t mod_id){

        /*
        uint8_t new_events_found = 0;
        
        memset(ScriptUpLoadEXT, 0x00, sizeof(ScriptUpLoadEXT));
        
        
        if(cur_up_pos_ext + 1 == MAX_SCRIPT_BUFFER_SIZE)
        {
                cur_up_pos_ext = 0;
        }
        for (int i = cur_up_pos_ext + 1; i < MAX_SCRIPT_BUFFER_SIZE; i++)
        {
                if(ScriptLoad[i].valid > 0)
                {
                        if(ScriptLoad[i].Mod_ID != mod_id)
                        {
                                ScriptLoad[i].valid = 0;
                                ScriptLoad[i].Mod_ID = 0;
                                ScriptLoad[i].lineID = 0;
                                ScriptLoad[i].time = 0;
                                ScriptLoad[i].seqID = 0;
                                ScriptLoad[i].DMX_value = 0;
                                ScriptLoad[i].Rmp_Cfg = 0;
                                ScriptLoad[i].DMX_Ramp = 0;
                                ScriptLoad[i].SZone = 0;
                                ScriptLoad[i].Position = 0;
                                memset(ScriptLoad[i].Channel_name, 0x00, sizeof(ScriptLoad[i].Channel_name));
                        }
                        else
                        {
                                cur_up_pos_ext = i;
                                
                                if(new_events_found < MAX_NEWEVENTS_EXT){
                                        add_new_event(new_events_found, cur_up_pos_ext);
                                        new_events_found++;
                                }
                        }
                        
                }
        }
        
        if(new_events_found < MAX_NEWEVENTS_EXT)
        {
                cur_up_pos_ext = 0;
                for (int i = cur_up_pos_ext; i < MAX_SCRIPT_BUFFER_SIZE; i++)
                {
                        if(ScriptLoad[i].valid > 0)
                        {
                                if(ScriptLoad[i].Mod_ID != mod_id)
                                {
                                        ScriptLoad[i].valid = 0;
                                        ScriptLoad[i].Mod_ID = 0;
                                        ScriptLoad[i].lineID = 0;
                                        ScriptLoad[i].time = 0;
                                        ScriptLoad[i].seqID = 0;
                                        ScriptLoad[i].DMX_value = 0;
                                        ScriptLoad[i].Rmp_Cfg = 0;
                                        ScriptLoad[i].DMX_Ramp = 0;
                                        ScriptLoad[i].SZone = 0;
                                        ScriptLoad[i].Position = 0;
                                        memset(ScriptLoad[i].Channel_name, 0x00, sizeof(ScriptLoad[i].Channel_name));
                                }
                                else
                                {
                                        cur_up_pos_ext = i;
                               
                                        if(new_events_found < MAX_NEWEVENTS_EXT){
                                                add_new_event(new_events_found, cur_up_pos_ext);
                                                new_events_found++;
                                        }
                                }
                                
                        }
                }
        
        }
        else{
                //do nothing
        }
        return new_events_found;
        */
        
        return 0;
}


void add_new_event(uint8_t new_ev_gfds, uint8_t pos_in_script){
        /*
        
                        ScriptUpLoadEXT[new_ev_gfds].valid = ScriptLoad[pos_in_script].valid;
                        ScriptUpLoadEXT[new_ev_gfds].Mod_ID = ScriptLoad[pos_in_script].Mod_ID;
                        ScriptUpLoadEXT[new_ev_gfds].lineID = ScriptLoad[pos_in_script].lineID;
                        ScriptUpLoadEXT[new_ev_gfds].time = ScriptLoad[pos_in_script].time;
                        ScriptUpLoadEXT[new_ev_gfds].seqID = ScriptLoad[pos_in_script].seqID;
                        ScriptUpLoadEXT[new_ev_gfds].DMX_value = ScriptLoad[pos_in_script].DMX_value;
                        ScriptUpLoadEXT[new_ev_gfds].Rmp_Cfg = ScriptLoad[pos_in_script].Rmp_Cfg;
                        ScriptUpLoadEXT[new_ev_gfds].DMX_Ramp = ScriptLoad[pos_in_script].DMX_Ramp;
                        ScriptUpLoadEXT[new_ev_gfds].SZone = ScriptLoad[pos_in_script].SZone;
                        ScriptUpLoadEXT[new_ev_gfds].Position = ScriptLoad[pos_in_script].Position;
                        
                        ScriptLoad[pos_in_script].valid = 0;
                        ScriptLoad[pos_in_script].Mod_ID = 0;
                        ScriptLoad[pos_in_script].lineID = 0;
                        ScriptLoad[pos_in_script].time = 0;
                        ScriptLoad[pos_in_script].seqID = 0;
                        ScriptLoad[pos_in_script].DMX_value = 0;
                        ScriptLoad[pos_in_script].Rmp_Cfg = 0;
                        ScriptLoad[pos_in_script].DMX_Ramp = 0;
                        ScriptLoad[pos_in_script].SZone = 0;
                        ScriptLoad[pos_in_script].Position = 0;
                        memset(ScriptLoad[pos_in_script].Channel_name, 0x00, sizeof(ScriptLoad[pos_in_script].Channel_name));
                        */

}


void reset_mod_prg_var_Next(void){
        Line_read = 2;
        cur_up_pos = 0;     
        cur_up_pos_ext = 0;
        curent_pos_load = 0;
        initTimer4X();
        line_len = 0;
        cur_line = 0;
       // was_filed_open = 0;
        
                memset(ScriptUpLoadEXT, 0x00, sizeof(ScriptUpLoadEXT));

                memset(ScriptLoad, 0x00, sizeof(ScriptLoad));
                        ScriptUpLoad.valid = 0;
                        ScriptUpLoad.Mod_ID = 0;
                        ScriptUpLoad.lineID = 0;
                        ScriptUpLoad.time = 0;
                        ScriptUpLoad.seqID = 0;
                        ScriptUpLoad.DMX_value = 0;
                        ScriptUpLoad.Rmp_Cfg = 0;
                        ScriptUpLoad.DMX_Ramp = 0;
                        ScriptUpLoad.SZone = 0;
                        ScriptUpLoad.Position = 0;
}

void reset_mod_prg_var(void){
        
        cur_up_pos_ext = 0;
        cur_up_pos = 0;
        ev_mod_rem = 0;
//        Del_or_prg = 0;
        Line_read = 2;
        curent_pos_load = 0;
        mod_prg_left = 0;
        line_len = 0;
        cur_line = 0;
        
        initTimer4X();
        
        
        memset(is_UpFailed, 0x00, sizeof(is_UpFailed));
        
        memset(ScriptLoad, 0x00, sizeof(ScriptLoad));
                        ScriptUpLoad.valid = 0;
                        ScriptUpLoad.Mod_ID = 0;
                        ScriptUpLoad.lineID = 0;
                        ScriptUpLoad.time = 0;
                        ScriptUpLoad.seqID = 0;
                        ScriptUpLoad.DMX_value = 0;
                        ScriptUpLoad.Rmp_Cfg = 0;
                        ScriptUpLoad.DMX_Ramp = 0;
                        ScriptUpLoad.SZone = 0;
                        ScriptUpLoad.Position = 0;
        
}


void reset_script_run(void){
        
//        #if DBG_TST_X 
        memset(ScriptLoad, 0x00, sizeof(ScriptLoad));
        memset(Script_to_LCD, 0x00, sizeof(Script_to_LCD));
        Script_to_LCD[0].valid = 1;
        //Line_read = 2;
        script_is_end = 0;
        curent_pos_load = 0;
        cur_line_load = 1;
        is_remain_Pyro =  savedDatas.is_Pyro_eventsS0;
        is_remain_DMX = savedDatas.is_DMX_eventsS0;
        is_Script_copy = 0;
    //    #endif
        
}


uint8_t get_is_internal_drive(void){
        return is_internal_drive;
}


void load_Script_Data(void){
        
        FRESULT rc;		/* Result code */
        UINT br;
        
        uint8_t was_screen_filled = 0; 
       // enter_time_tstX = GetCurrentSystemTime();  
        
        uint8_t tst_xxx = 0;
        uint8_t percent_i_MT = 100;
        
        
        if(file_sizeS > 0){
                                        
                                        for (;;) {
                                           /* Read a chunk of file */
                                           rc = f_read(&FileRead_Sx, F_buffer, sizeof F_buffer, &br);
                                           if (rc || !br) {
                                                 break;					/* Error or end of file */
                                           }
                                           

                                            tst_xxx = ReadByLine(F_buffer, br);
                                          
                                                if(percent_i_MT == 100)   
                                                {
                                                        percent_i_MT = 0;
                                                        sort_percent = (100 * Tot_Bytes_S)/file_sizeS;
                                                        
                                                        //uint8_t Percent_tmp = 0;
                                                        

                                                        if(was_screen_filled == 0){
                                                                was_screen_filled = 1;
                                                                //lcd update
                                                        }
                                                        
                                                       // Percent_tmp = 100 + sort_percent;
                                                        //lcd update
                                                }
                                                else
                                                {
                                                        percent_i_MT++;
                                                }
                                           
                                            if(tst_xxx == 10)
                                            {
                                                  error_load_script = Line_read;
                                                  break;
                                            }
                                            else
                                            {
                                               f_lseek(&FileRead_Sx, Tot_Bytes_S); 
                                            }
                                    }
                                
                                        
                                     if(tst_xxx != 10)
                                      {
                                            load_time_tstX = GetCurrentSystemTime() - enter_time_tstX;
                                            finish_prg();// finish programing
                                            Tot_Bytes_S = 0;
                                      }
                                      
                                      was_filed_open = 1;
                                      rst_all_usbScripts();
                                      Line_read = 2;
                                }
}





void delete_script_int(void){
        was_filed_open = 0;
        rst_all_usbScripts();
        Line_read = 2;
        rst_ID_Prg();
        
        
        f_close(&FileRead_Sx);
        f_unlink("0:/Script.csv");
        
        memset(prg_name_LCD, 0x00, sizeof(prg_name_LCD)); 
        
                was_filed_open = 0;
//                #if DBG_TST_X 
                        savedDatas.is_R_Prg = 0;
  //              #endif 
                memset(&savedDatas,  0x00 , sizeof(savedDatas));
                is_remain_Pyro =  0;
                is_remain_DMX = 0;
                tmp_DMX_ev = 0;
                tmp_pyro_ev = 0;
        
        rst_slaves_script();
        getSlaves_connected();
        was_LCD_Script_Changed = 0x3F;
        
}





uint8_t get_is_file_exists(void){
  FRESULT rc;		/* Result code */

  sprintf(pathScript, "0:/Script.csv");
    
         
        rc = f_open(&FileRead_Sx, pathScript , FA_READ);
        f_close(&FileRead_Sx);

        if(rc == FR_OK) {
                return 1;
        }else{
                return 0;
        }
}


void rst_first_seq_time(void){
        for(int i = 0; i < MAX_SEQ; i++){
                Seq_Firs_Time[i]= 0xFFFFFFFF;
        }
        
}
