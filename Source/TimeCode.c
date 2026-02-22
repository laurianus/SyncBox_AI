/**************************************************************************//**
* @file     TC.c
* @brief    Core device operation functions
******************************************************************************/
#include <string.h>
#include <stdio.h>

#include "stm32f4xx_hal.h"
#include "waveplayer.h"


#include "SystemTimer.h"
#include "main.h"
#include "Device.h"
#include "Display.h"
#include "options.h"
#include "Wireless.h"
#include "TimeCode.h"
#include "uart.h"


#ifndef VS_READABLE_CODE
#include "System1.h"
#include "System1Action.h"
#include "System1Data.h"
#endif

#include "simpleEventHandler.h"         		// Event Queue
#include "System1SEMLibB.h"                    	// visualSTATE essentials




//functions declarations
static void update_smpte_timer(void);
static void autodetect_TC_fps(void);
static void decode_TC(void);

/*
uint32_t aaa_time_bits[100] = {0};
uint16_t stock__aaa = 0;
*/

int FSK_time_Cread = 0;
int FSK_time_Lread = 0;

extern uint8_t was_ButStoped;

extern uint8_t SMPTE_RF;
//uint8_t FSK_time_sec = 0;
//uint8_t FSK_time_Min = 0;
uint8_t was_Frame_det = 0;
uint8_t Frame_pos = 0;

uint8_t FSK_PD_Frame[8] = {0};
uint8_t FSK_PD_Frame_Hex[4] = {0};
uint32_t fsk_err = 0;
uint32_t fsk_err_free = 0;
uint32_t chk_sum_X = 0;

uint8_t Time_Code_disable = 0;

/*
uint32_t fsk_err0 = 0;
uint32_t fsk_err2 = 0;
*/


uint8_t FSK_F1_Frame[21] = {0};  
  


extern uint8_t is_FSK_setup;

//NOT USED
uint16_t tst_vect_Time[4] = {0};

char buf_FSK[24] = {0};

long int time_dif_xxx = 0;

uint32_t smpte_time_xxx = 0u;
int smpte_time_yyy = 0u; // for tests
uint32_t t_bits_error = 0u;
//uint32_t t_bits_error_0_XXX = 0u;

//static uint32_t smpte_time_zzz = 0u;
//static uint8_t timeCodeX[11];
/*****************************/

volatile uint8_t valid_tc_word = 0u;
volatile uint8_t ones_bit_count= 0u;
volatile uint8_t tc_sync = 0u;
volatile uint8_t drop_frame_flag = 0u;;

//volatile uint8_t total_bits = 0;

static uint8_t total_bits = 0;

volatile uint8_t current_bit = 0;
volatile uint8_t sync_count = 0;


extern UART_HandleTypeDef huart7;



uint16_t one_time_max;
uint16_t one_time_min;
uint16_t zero_time_max;
uint16_t zero_time_min;
uint16_t FPS_ADD;
uint8_t A25_fps_detect = 0;
uint8_t A30_fps_detect = 0;
uint8_t TC_fps_detect = 0;
uint8_t fps_err = 0;


uint16_t Smpte_err = 0;

uint8_t tcx[8] = {0};
//volatile char timeCode[11];

//static uint8_t tc[8];
//static char timeCode[11];




uint16_t one_time_max = 0;
uint16_t one_time_min = 0;
uint16_t zero_time_max = 0;
uint16_t zero_time_min = 0;
uint8_t TC_fps_set = 0;
uint8_t TC_fps_setX = 0;
uint32_t TC_Errors  = 0;
//uint16_t valid_decoded_TC = 0; //check if teh time code is correct


uint32_t bit_time = 0;

uint16_t bit_33fps = 0;
uint16_t bit_25fps = 0;
uint16_t bit_errfps = 0;

uint16_t bit_FSK_F1 = 0;
uint16_t bit_FSK_PD = 0;



uint16_t bit_33fpsT = 0;
uint16_t bit_25fpsT = 0;
uint16_t bit_errfpsT = 0;

uint16_t bit_FSK_F1T = 0;
uint16_t bit_FSK_PDT = 0;



uint16_t Wait_TC_Time = 0;


uint32_t smpte_timeX = 0;
int smpte_time = 0;



uint16_t min0_time_bit = 0;
uint16_t max0_time_bit = 0;

uint16_t min1_time_bit = 0;
uint16_t max1_time_bit = 0;

uint32_t tmp_smpte_time = 0;


uint8_t SMPTE_Start = 0;

uint8_t SMPTE_Valid_PT;
uint8_t SMPTE_Valid_XT;

uint8_t isLTCg_valid = 0;

uint8_t was_Pause_byM = 0;
uint8_t SafeTCstop = 0;
uint8_t was_LTC_Started = 0;

long int smpte_time_vvv = 0;

//Not used
uint8_t SMPTE_R = 0;
uint8_t SMPTE_Start_V = 0;


uint8_t is_smpte_active = 0;

//uint32_t Test_TC0 = 0;
//uint32_t Test_TC1 = 0;

//uint8_t DBG_TEST = 0;

uint32_t was_TC_error = 0;

void SMPTE_IRQHandler (uint32_t bit_timer)
{
        if(get_AB_Adre() != MAX_AB_ID || Time_Code_disable == 1){
                 return;
         }
        
        SMPTE_R = 1;
        
        is_smpte_active = 1;
        bit_time = bit_timer;
        
     //  test_bit_time();
        
              if(isLTCg_valid == 0)
               {
                   if(TC_fps_setX == 0)
                             {

                                   TC_fps_set =  0;
                                   one_time_min = 0;
                                   one_time_max = 0;
                                   zero_time_min = 0;
                                   zero_time_max = 0;
                                   
                                   autodetect_TC_fps();
                             }
                            else if (TC_fps_setX == 1 || TC_fps_setX == 2)
                             {
                                  // test_bit_time();
                                   decode_TC();
                                     if(((t_bits_error > 10000) || (Smpte_err > 50)) && device_Status < Status_PowerEnable)
                                     {
                                        init_time_code();                                           
                                     }
                             }
                             else if(TC_fps_setX == 10)
                             {
                                     was_TC_error++;
                                     
                                     if(was_TC_error > 5000)
                                     {
                                        init_time_code();
                                     }
                             }
                             else
                             {
                                   //do nothing
                             }
               }

              return;
}


void set_time_code_read_SMPTE(void){
        //Disable FSK
        //Disbale_FSK();
       
        MX_TIM9_Init();//SMPTE Timer
        Disable_TC_SMPTE_Gen_Pin();//Disable SMPTE Gen pin
        init_time_code();
        Enable_TC_SMPTE_Read_Pin();
}
void init_time_code(void)
{
        TC_fps_setX = 0;
        Smpte_err = 0;
        
        t_bits_error = 0;
        bit_errfps = 0;
        bit_25fps = 0;
        bit_33fps = 0;
        bit_FSK_F1 = 0;
        bit_FSK_PD = 0;
        was_TC_error = 0;
}


void autodetect_TC_fps(void)
 {
       if((bit_time > one_time_min25 && bit_time < one_time_max25) || (bit_time > zero_time_min25 && bit_time < zero_time_max25))
        {
              bit_25fps++;
        }
       else if((bit_time > one_time_min30 && bit_time < one_time_max30) || (bit_time > zero_time_min30 && bit_time < zero_time_max30))
        {
              bit_33fps++;
        }
       else
        {
              if(bit_25fps > 10 || bit_33fps > 10)
               {
                     bit_errfps++;
               }
              
        }
       
       
       uint32_t tot_TC_bits = bit_errfps + bit_25fps + bit_33fps;
       
       
       if(tot_TC_bits > 500)
        {
              if(bit_25fps > (tot_TC_bits * 0.8))
               {
                     Wait_TC_Time = WAIT_ON_LTC_TC;
                     TC_fps_setX = 1;
                     TC_fps_set = 40;
                     one_time_min = one_time_min25;
                     one_time_max = one_time_max25;
                     zero_time_min = zero_time_min25;
                     zero_time_max = zero_time_max25;
                     Smpte_err = 0;
               }
              else if(bit_33fps > (tot_TC_bits *0.8))
               {
                     Wait_TC_Time = WAIT_ON_LTC_TC;
                     TC_fps_setX = 2;
                     TC_fps_set =  33;
                     one_time_min = one_time_min30;
                     one_time_max = one_time_max30;
                     zero_time_min = zero_time_min30;
                     zero_time_max = zero_time_max30;
                     Smpte_err = 0;
               }
               /*
               else if (tst_vect_Time[1] > (tot_TC_bits *0.7)) //PD TC
               {
                     Wait_TC_Time = WAIT_ON_FSK_PD;
                     TC_fps_setX = 3;
               }
              else if (tst_vect_Time[2] > (tot_TC_bits *0.7))//F1 TC
               {
                     Wait_TC_Time = WAIT_ON_FSK_F1;
                     TC_fps_setX = 4;
               }
               */
              else
               {
                       TC_fps_setX = 10; //Error recgnizing the time code
                     //error - do nothing or report
               }
        }
 }


void test_bit_time(void)
 {
        //for tests
        static uint16_t bit_vect_Time[25] = {0};

        if(bit_time < 300)
        {
              bit_vect_Time[0]++;
        }
        else if(bit_time >= 300 && bit_time < 340)
        {
              bit_vect_Time[1]++;//33
        }
        else if(bit_time >= 340 && bit_time < 465)
        {
              bit_vect_Time[2]++;
        }
        else if(bit_time >= 465 && bit_time < 470)
        {
              bit_vect_Time[3]++; //PD 20
        }
        else if(bit_time >= 470 && bit_time < 480)
        {
              bit_vect_Time[4]++; //PD 20
        }
        else if(bit_time >= 480 && bit_time < 490)
        {
              bit_vect_Time[5]++; //25
        }
        else if(bit_time >= 490 && bit_time < 500)
        {
              bit_vect_Time[6]++; //PD 10 // 25
        }
        else if(bit_time >= 500 && bit_time < 510)
        {
              bit_vect_Time[7]++; 
        }
        else if(bit_time >= 510 && bit_time < 520)
        {
              bit_vect_Time[8]++;
        }
        else if(bit_time >= 520 && bit_time < 530)
        {
              bit_vect_Time[9]++;
        }
        else if(bit_time >= 530 && bit_time < 540)
        {
              bit_vect_Time[10]++;
        }
        else if(bit_time >= 540 && bit_time < 550)
        {
              bit_vect_Time[11]++; //PD 25
        }
        else if(bit_time >= 550 && bit_time < 950)
        {
              bit_vect_Time[12]++; //PD 25
        }
        else if(bit_time >= 950 && bit_time < 960)
        {
              bit_vect_Time[13]++;
        }
        else if(bit_time >= 960 && bit_time < 970)
        {
              bit_vect_Time[14]++;
        }
        else if(bit_time >= 970 && bit_time < 980)
        {
              bit_vect_Time[15]++;
        }
        else if(bit_time >= 980 && bit_time < 990)
        {
              bit_vect_Time[16]++;
        }
        else if(bit_time >= 990 && bit_time < 1000)
        {
              bit_vect_Time[17]++;
        }
        else if(bit_time >= 1000 && bit_time < 1010)
        {
              bit_vect_Time[18]++;
        }
        else if(bit_time >= 1010 && bit_time < 1020)
        {
              bit_vect_Time[19]++;
        }
        else if(bit_time >= 1020 && bit_time < 1030)
        {
              bit_vect_Time[20]++;
        }
        else if(bit_time >= 1030 && bit_time < 1040)
        {
              bit_vect_Time[21]++;
        }
        else if(bit_time >= 1040 && bit_time < 1050)
        {
              bit_vect_Time[22]++;
        }
       else
        {
              bit_vect_Time[23]++;  //25 fps 20%
        }
        
 }



void decode_TC(void)
 {
       
         
       SMPTE_Start_V = 0;
       if ((bit_time < one_time_min)  || ((bit_time > one_time_max) && (bit_time < zero_time_min)) || (bit_time > zero_time_max)) // get rid of anything way outside the norm
      //  if ((bit_time < one_time_min) || (bit_time > zero_time_max)) // get rid of anything way outside the norm
        {
              total_bits = 0;
              
              if(bit_time > 0)
               {
                     t_bits_error++;
               }
               else{
                 //    t_bits_error_0_XXX++;
               }
            

        }
       else
        {
              if (ones_bit_count == 1) // only count the second ones pluse
                    ones_bit_count = 0;
              else
               {    
                     if (bit_time > zero_time_min)
                      {
                            current_bit = 0;
                            sync_count = 0;
                      }
                     else //if (bit_time < one_time_max)
                      {
                            ones_bit_count = 1;
                            current_bit = 1;
                            sync_count++;
                            if (sync_count == 12) // part of the last two bytes of a timecode word
                             {
                                   sync_count = 0;
                                   tc_sync = 1;
                                   total_bits = end_sync_position;
                             }
                      }
                     
                     if (total_bits <= end_data_position) // timecode runs least to most so we need
                      {                                    // to shift things around
                            tcx[0] = tcx[0] >> 1;
                            
                            for(int n=1;n<8;n++)
                             {
                                   if(tcx[n] & 1)
                                         tcx[n-1] |= 0x80;
                                   
                                   tcx[n] = tcx[n] >> 1;
                             }
                            
                            if(current_bit == 1)
                                  tcx[7] |= 0x80;
                      }
                     total_bits++;
               }
              
              if (total_bits == end_smpte_position) // we have the 80th bit
               {
                     total_bits = 0;
                     tmp_smpte_time = 0;
                     if (tc_sync)
                      {
                            tc_sync = 0;
                            valid_tc_word = 1;
                      }
               }
              
              
              if(valid_tc_word == 1)
               {
                     update_smpte_timer();
               }else{
                     //Smpte_err++;
               }
        }
 }




void update_smpte_timer(void)
 {
       valid_tc_word = 0;
       
       smpte_timeX = (tcx[7] &0x03)  * 10 * 3600000 + (tcx[6] &0x0F)  * 3600000 + ((tcx[2]&0x0F) + (tcx[3]&0x07)* 10) * 1000 
             + ((tcx[4]&0x0F) + (tcx[5]&0x07)* 10) * 60000 + ((tcx[0]&0x0F) + (tcx[1]&0x03)* 10 + 1) * TC_fps_set + TC_fps_set;
       

       smpte_time_vvv = smpte_timeX - smpte_time_xxx;
       
       
      // if((smpte_time_vvv == TC_fps_set) || (TC_fps_set == 33 && (smpte_time_vvv == 43 || smpte_time_vvv == 10)))
        if((smpte_time_vvv == TC_fps_set) || (TC_fps_set == 33 && smpte_time_vvv == 43))
        {
              smpte_time_yyy ++;
        }
       else
        {
              smpte_time_yyy = 0;
              Smpte_err++;
           
        }
       
       
       if(TC_fps_setX == 0)
        {
              if (smpte_time_yyy > 2 * MIN_TC_VALID)
               {
                     smpte_time_yyy = MIN_TC_VALID + 2;
               }
        }
       else
        {
              if(device_Status == Status_Pause)
               {
                     SMPTE_R = 0u;
                     
                     SMPTE_Valid_PT = 1;
                     
                      if(was_Pause_byM == 0 && was_LTC_Started != 2)
                                    {
                                         //SEQ_AddEvent(ev_SMPTEstartP);
                                          SEQ_AddEvent(ev_SMPTEstart);
                                          was_LTC_Started = 1;
                                    }
               }
              else if(device_Status == Status_Idle )
               {
                     SMPTE_Valid_XT = 1;
               }
              
              
              if(smpte_time_yyy > MIN_TC_VALID)
               {
                     t_bits_error = 0;
                    
                     smpte_time_yyy = MIN_TC_VALID + 1;
                     
                     smpte_time = smpte_timeX;
                     
                       
                     time_dif_xxx = smpte_time - get_time_Code_ofset();
                     //  time_dif_xxx = smpte_time;
                       
                     if(time_dif_xxx > 0){
                        
                             if(device_Status == Status_PowerEnable && time_dif_xxx > 0)
                              {
                                    
                                    SMPTE_Valid_PT = 1;
                                    was_LTC_Started = 1;
                                    setDevicetime(time_dif_xxx);
                                    SEQ_AddEvent(ev_SMPTEstart);
                                    
                              }
                             
                             if((device_Status == Status_Play || device_Status == Status_Pause) && time_dif_xxx > 0 && was_LTC_Started > 0)
                              {
                                    setDevicetime(time_dif_xxx);
                                    was_LTC_Started = 1;
                                    
                              }
                }
                     
               }
        }
       smpte_time_xxx = smpte_timeX;
       
        /*
                  if (is_error_correction == 1 && device_Status == Status_Idle)
                      {
                            uint32_t T_Drift_error;
                            T_Drift_error = GetCurrentSystemTime() - Start_music_time;
                            Error_time_drift = T_Drift_error - smpte_time;
                      }
        */
 }



long int get_time_codeOffset(void){
        return time_dif_xxx;
}

 
uint32_t get_SMPTE_Time(void)
{
        return smpte_time;
}


/*
void set_Err_Correction(uint64_t sys_time_mct)
 {
       Start_music_time = sys_time_mct;
       is_error_correction = 1;
 }

void cal_Error_correction(void)
 {
       
       uint64_t tmp_e1;
       //static int cor_fact_fxxx;
       
       
       if(Error_time_drift < 0)
        {
              
              Error_time_drift = - (Error_time_drift);
              
              
              
              tmp_e1 = (Error_time_drift + 55 )* 10000;
              cor_fact2 = tmp_e1/(float)(smpte_time);
              cor_fact = (int)cor_fact2;
              
              if (cor_fact < cor_fact2)
               {
                     cor_fact2 = cor_fact2 - (float)cor_fact;
               }
              else
               {
                     cor_fact2 = (float)cor_fact - cor_fact2;
               }
              
        }
       else
        {
              
              tmp_e1 = (Error_time_drift + 55 )* 10000;
              cor_fact2 = tmp_e1/(float)(smpte_time);
              cor_fact = (int)cor_fact2;
              
              if (cor_fact < cor_fact2)
               {
                     cor_fact2 = cor_fact2 - (float)cor_fact;   
               }
              else
               {
                     cor_fact2 = (float)cor_fact - cor_fact2;
               }
              
              
              cor_fact = - cor_fact;
              
        }
       
       if(cor_fact < 75 && cor_fact > -75)
        {
              
              set_Slave_prgY();
              get_prg_mod_X();                
              
              saveFlashData();
              is_error_correction = 2;
        }
       else
        {
              is_error_correction = 3;        
        }
      
 }
 */


void UartFSK_CMD_Mode(void){
        
       buf_FSK[0] = '+';
       buf_FSK[1] = '+';
       buf_FSK[2] = '+';
       buf_FSK[3] = 'A';
       buf_FSK[4] = 'T'; //4 for receive
       buf_FSK[5] = '\r';
       
      HAL_UART_Transmit_IT(&huart7, (uint8_t *)buf_FSK, 6);
 
}

void UartFSK_CMD_ModeX(void){
        
       buf_FSK[0] = '+';
       buf_FSK[1] = '+';
       buf_FSK[2] = '+';
       buf_FSK[3] = '\r';
       
      HAL_UART_Transmit_IT(&huart7, (uint8_t *)buf_FSK, 4);
 
}

void UartFSK_PD_set(uint8_t tmp_rcv_trans)
 {
       
       buf_FSK[0] = 'A';
       buf_FSK[1] = 'T';
       buf_FSK[2] = 'F';
       buf_FSK[3] = 'B';
       
    //   buf_FSK[4] = '4'; //4 for receive

       if(tmp_rcv_trans  == 0)
        {
              buf_FSK[4] = '4'; //for receive
        }
       else
        {
              buf_FSK[4] = '5'; //for transmit
        }

       buf_FSK[5] = 'W';
       buf_FSK[6] = '1';
       buf_FSK[7] = 'S';
       buf_FSK[8] = '3';
       buf_FSK[9] = '0';     
       buf_FSK[10] = '=';
       buf_FSK[11] = '2';
       buf_FSK[12] = '@';
       buf_FSK[13] = 'L';
       buf_FSK[14] = '1';
       buf_FSK[15] = '\r';
       
      HAL_UART_Transmit_IT(&huart7, (uint8_t *)buf_FSK, 16);
 }

 
 void UartFSK_PD_setNew(uint8_t tmp_rcv_trans)
 {

       buf_FSK[0] = '+';
       buf_FSK[1] = '+';
       buf_FSK[2] = '+';         
       buf_FSK[3] = 'A';
       buf_FSK[4] = 'T';
       buf_FSK[5] = 'F';
       buf_FSK[6] = 'B';
       

       if(tmp_rcv_trans  == 0)
        {
              buf_FSK[7] = '4'; //for receive
        }
       else
        {
              buf_FSK[7] = '5'; //for transmit
        }

       buf_FSK[8] = 'W';
       buf_FSK[9] = '1';
       buf_FSK[10] = 'S';
       buf_FSK[11] = '3';
       buf_FSK[12] = '0';     
       buf_FSK[13] = '=';
       buf_FSK[14] = '2';
       buf_FSK[15] = '@';
       buf_FSK[16] = 'L';
       buf_FSK[17] = '1';
       buf_FSK[18] = 'O';
       buf_FSK[19] = '\r';
       
      HAL_UART_Transmit_IT(&huart7, (uint8_t *)buf_FSK, 20);
 }
 
void UartFSK_F1_set(void)
 {
       buf_FSK[0] = 'A';
       buf_FSK[1] = 'T';
       buf_FSK[2] = 'F';
       buf_FSK[3] = 'B';
       buf_FSK[4] = '0';
       buf_FSK[5] = 'S';
       buf_FSK[6] = '3';
       buf_FSK[7] = '0';
       buf_FSK[8] = '=';
       buf_FSK[9] = '6';     
       buf_FSK[10] = '4';
       buf_FSK[11] = '@';
       buf_FSK[12] = 'L';
       buf_FSK[13] = '1';
       buf_FSK[14] = '\r';
       buf_FSK[15] = 0x00;
         
       HAL_UART_Transmit_IT(&huart7, (uint8_t *)buf_FSK, 15);
 }


void UartFSK_F1_setNew(void)
 {
       buf_FSK[0] = '+';
       buf_FSK[1] = '+';
       buf_FSK[2] = '+'; 
       buf_FSK[3] = 'A';
       buf_FSK[4] = 'T';
       buf_FSK[5] = 'F';
       buf_FSK[6] = 'B';
       buf_FSK[7] = '0';
       buf_FSK[8] = 'S';
       buf_FSK[9] = '3';
       buf_FSK[10] = '0';
       buf_FSK[11] = '=';
       buf_FSK[12] = '6';     
       buf_FSK[13] = '4';
       buf_FSK[14] = '@';
       buf_FSK[15] = 'L';
       buf_FSK[16] = '1';
       buf_FSK[17] = 'O';
       buf_FSK[18] = '\r';
         
       HAL_UART_Transmit_IT(&huart7, (uint8_t *)buf_FSK, 19);
 }
 
 
void UartFSK_ATX_set(uint8_t tmp_time_code)
 {
       buf_FSK[0] = 'A';
       buf_FSK[1] = 'T';
       
       if(tmp_time_code == 0)//PD
        {
              buf_FSK[2] = 'D';
        }
       else 
        {
                buf_FSK[2] = 'A';
        }
       
       buf_FSK[3] = '\r';
       buf_FSK[4] = 0x00;
       buf_FSK[5] = 0x00;
       buf_FSK[6] = 0x00;
       buf_FSK[7] = 0x00;
       buf_FSK[8] = 0x00;
       buf_FSK[9] = 0x00;     
       buf_FSK[10] = 0x00;
       buf_FSK[11] = 0x00;
       buf_FSK[12] = 0x00;
       buf_FSK[13] = 0x00;
       buf_FSK[14] = 0x00;
       buf_FSK[15] = 0x00;
       
       HAL_UART_Transmit_IT(&huart7, (uint8_t *)buf_FSK, 4);
 } 


uint8_t get_time_code_Stat(void){
        return TC_fps_setX;
}

 




uint8_t get_TC_fps_setX(void){
        return TC_fps_setX;
}




uint16_t get_time_codeBrut(void){
        return FSK_time_Cread;
}
uint16_t get_FSK_err(void){
        return fsk_err;
}


void read_gen_FSK_Time(void)
 {
       
       uint8_t rasb_FSK[24] ={0};
       int bytesCount = 0;
       
       Frame_pos = 0;
       was_Frame_det = 0;
       
         if(get_AB_Adre() != MAX_AB_ID || Time_Code_disable == 1){
                 reset_TimeCodeBuffer();//empty buffer
                 return;
         }
              
       
       //if(TC_fps_setX == 3 || TC_fps_setX == 30 || TC_fps_setX == 4)
       if(TC_fps_setX == 3 || TC_fps_setX == 4)
       {

       bytesCount = UartFSK_TC_Receive(rasb_FSK);

       if(bytesCount < 1) return;
               
       SMPTE_RF = 1;
       
       if(TC_fps_setX == 3) // read PD FSK Time code
        {
              for(int i = 0; i < bytesCount; i++)
               {
                     if(rasb_FSK[i] == 'S')
                      {
                            was_Frame_det = 1;
                            Frame_pos = 0;
                      }
                     else if(was_Frame_det == 1)
                      {
                            if(CharToHex(rasb_FSK[i]) != 16)
                             {
                                   FSK_PD_Frame[Frame_pos] = rasb_FSK[i];
                                   Frame_pos++;
                             }
                            else
                             {
                                   was_Frame_det = 0;
                                   Frame_pos = 0;
                                   
                             }
                      }
               }
              
              if(Frame_pos > 5)//entire frame received
               {
                     is_smpte_active = 1;
                     Frame_pos = 0;
                     was_Frame_det = 0;
                     
                     FSK_PD_Frame_Hex [0] = ((CharToHex(FSK_PD_Frame[0]) << 4) | CharToHex(FSK_PD_Frame[1]));
                     FSK_PD_Frame_Hex [1] = ((CharToHex(FSK_PD_Frame[2]) << 4) | CharToHex(FSK_PD_Frame[3]));
                     FSK_PD_Frame_Hex [2] = ((CharToHex(FSK_PD_Frame[4]) << 4) | CharToHex(FSK_PD_Frame[5]));
                     FSK_PD_Frame_Hex[3] = FSK_PD_Frame_Hex [0] + FSK_PD_Frame_Hex [1];
                     
                     if(FSK_PD_Frame_Hex [2] == FSK_PD_Frame_Hex [3]) //message valid
                      {
                            FSK_time_Cread = ((FSK_PD_Frame_Hex [0] & 0xFF) << 8) | (FSK_PD_Frame_Hex [1] & 0xFF);
                            
                             if((FSK_time_Cread - FSK_time_Lread == 0) || (FSK_time_Cread - FSK_time_Lread == 1) || (FSK_time_Cread - FSK_time_Lread == 2))
                             {
                                   if(fsk_err_free < 3){
                                        fsk_err_free++;
                                   }
                             }else{
                                   fsk_err_free = 0;
                             }
                            
                         //   
                             
                             if(fsk_err_free > 2){
                                update_TC_timer(100 * FSK_time_Cread);
                                      
                                      if(device_Status == Status_PowerEnable || device_Status == Status_Pause)
                                      {
                                              
                                           if(device_Status == Status_Pause && was_ButStoped == 1) {
                                                    setDevicetime(100 * FSK_time_Cread);
                                           }else{
                                                    SMPTE_Valid_PT = 1;
                                                    was_LTC_Started = 1;
                                                    setDevicetime(100 * FSK_time_Cread);
                                                    SEQ_AddEvent(ev_SMPTEstart);
                                           }
                                            
                                      }    
                             }
                             else{
                                fsk_err++;
                             }
                             
                             /*
                            if(FSK_time_Cread - FSK_time_Lread == 0){
                                fsk_err0++;
                            }
                            if(FSK_time_Cread - FSK_time_Lread == 2){
                                fsk_err2++;
                            }
                             */
                            FSK_time_Lread = FSK_time_Cread;
                      }
                     else
                      {
                            chk_sum_X++;
                      }
               }
        }
        
       else if (TC_fps_setX == 4)// read F1 FSK Time code
        {
              for(int i = 0; i < bytesCount; i++)
               {
                     if(rasb_FSK[i] == 'S')
                      {
                            was_Frame_det = 1;
                            Frame_pos = 0;
                      }
                     else if(rasb_FSK[i] == 'Y')
                      {
                            was_Frame_det = 2;
                            Frame_pos = 0;
                      }
                     else if(was_Frame_det == 2)
                      {
                            FSK_F1_Frame[Frame_pos] = rasb_FSK[i];
                            Frame_pos++;
                      }
                     else
                      {
                            was_Frame_det = 0;
                            Frame_pos = 0;
                      }
               }
              
              if(Frame_pos > 9)
               {
                     if(XOR_chks_FSK() == 1)
                      {
                             is_smpte_active = 1;
                            FSK_time_Cread = (((FSK_F1_Frame [3] & 0x7F) << 16) | ((FSK_F1_Frame [4] & 0xFF) << 8) | (FSK_F1_Frame [5] & 0xFF));
                            if((FSK_F1_Frame [3] & 0x80) != 0x00)
                             {
                                   FSK_time_Cread = 0 - FSK_time_Cread;
                             }
                             
                             if((FSK_time_Cread - FSK_time_Lread == 0) || (FSK_time_Cread - FSK_time_Lread == 1))
                             {
                                   if(fsk_err_free < 3){
                                        fsk_err_free++;
                                   }
                             }else{
                                   fsk_err_free = 0;
                             }
                            
                             
                             if(fsk_err_free > 1){
                                update_TC_timer(1000 * FSK_time_Cread);
                                     
                                     if(device_Status == Status_PowerEnable || device_Status == Status_Pause)
                                      {
                                          if(device_Status == Status_Pause && was_ButStoped == 1) {
                                                    setDevicetime(1000 * FSK_time_Cread);
                                           }else{
                                                    SMPTE_Valid_PT = 1;
                                                    was_LTC_Started = 1;
                                                    setDevicetime(1000 * FSK_time_Cread);
                                                    SEQ_AddEvent(ev_SMPTEstart);
                                           }
                                            
                                      }   
                             }
                             else{
                                fsk_err++;
                             }
                             
                            
                            FSK_time_Lread = FSK_time_Cread;
                            
                            was_Frame_det = 0;
                            Frame_pos = 0;
                            
                      }
                     else
                      {
                            chk_sum_X++;
                      }
                }
                }
        }else{
                reset_TimeCodeBuffer();//empty buffer
        }
       
 }

 void UartFSK_PD_sent(uint16_t snt_time_code)
 {
       uint8_t tmp_xxx = 0;
         
       tmp_xxx = (0xFF & (snt_time_code >> 0)) + (0xFF & (snt_time_code >> 8));
         
       sprintf(buf_FSK, "S%04x%02x", snt_time_code, tmp_xxx);        
       
       HAL_UART_Transmit_IT(&huart7, (uint8_t *)buf_FSK, 7);
 } 
 
 void UartFSK_F1_sent(int snt_time_code)
 {
         
      // sprintf(buf_FSK, "SY%03x%03x%03x", snt_time_code, snt_time_code, snt_time_code);
      //   sprintf(buf_FSK, "SY%03x%03x%03x", snt_time_code, snt_time_code, snt_time_code);
        
        buf_FSK[0] = 'S';
        buf_FSK[1] = 'Y';
        buf_FSK[2] = 0xFF & (snt_time_code >> 16);
        buf_FSK[3] = 0xFF & (snt_time_code >> 8);
        buf_FSK[4] = 0xFF & (snt_time_code >> 0);
        buf_FSK[5] = 0xFF & (snt_time_code >> 16);
        buf_FSK[6] = 0xFF & (snt_time_code >> 8);
        buf_FSK[7] = 0xFF & (snt_time_code >> 0);
        buf_FSK[8] = 0xFF & (snt_time_code >> 16);
        buf_FSK[9] = 0xFF & (snt_time_code >> 8);
        buf_FSK[10] = 0xFF & (snt_time_code >> 0);
         
  
        uint8_t buf_chk = 0x0A;
       
       
       for(int i = 2; i < 11; i++){
             buf_chk ^= buf_FSK[i];
       }         
    
       buf_FSK[11] = buf_chk;
       
       HAL_UART_Transmit_IT(&huart7, (uint8_t *)buf_FSK, 12);
 }  
 

uint8_t CharToHex(char c)
 {
       switch( c ) {
         case '0': return 0;
         case '1': return 1;
         case '2': return 2;
         case '3': return 3;
         case '4': return 4;
         case '5': return 5;
         case '6': return 6;
         case '7': return 7;
         case '8': return 8;
         case '9': return 9;
         case 'a': case 'A': return 10;
         case 'b': case 'B': return 11;
         case 'c': case 'C': return 12;
         case 'd': case 'D': return 13;
         case 'e': case 'E': return 14;
         case 'f': case 'F': return 15;
       }
       return 16;
 }

uint8_t XOR_chks_FSK(void)
 {
       uint8_t buf_chk = 0x0A;
       
       
       for(int i = 0; i < 9; i++){
             buf_chk ^= FSK_F1_Frame[i];
       }
       
       if(buf_chk == FSK_F1_Frame[9])   
        {
              return 1;
        }
       else
        {
              return 0;  
        }
}


uint8_t get_is_TC_Active(void){
        if(device_Status == Status_Play && get_Time_CodeX() > 3)
                {
                        return 1;
                }else{
                        return is_smpte_active;
                }
}

