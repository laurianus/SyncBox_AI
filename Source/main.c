
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "uart.h"
#include "Wireless.h"
#include "Display.h"
#include "ADC.h"
#include "Buttons.h"
#include "USBHostMain.h"
#include "flash_if.h"
#include "TimeCode.h"
#include "SystemTimer.h"
#include "GlobalPositioning.h"

#include "usbd_cdc_if.h"
#include "usb_device.h"

#include "ILI9341_STM32_Driver.h"
#include "snow_tiger.h"


#ifndef VS_READABLE_CODE
#include "System1.h"
#include "System1Action.h"
#include "System1Data.h"
#endif

#include "simpleEventHandler.h"         // Event Queue
#include "System1SEMLibB.h"	   // visualSTATE essentials


#include "stm32f4xx_hal.h"


#define APP_VERSION 4

#if defined(CONFIG_PLAYER_SD)
#define APP_MODE    "SD"
#elif defined(CONFIG_PLAYER_USB)
#define APP_MODE    "USB"
#endif

extern uint8_t was_LTC_Started;
uint8_t Time_code_disable = 0;

uint8_t is_seq_A[5] = {0};// If audiofile exists for sequences

uint8_t set_AD_ID = 0;

uint32_t SN_Part0 = 0;
uint32_t SN_Part1 = 0;
uint32_t SN_Part2 = 0;


//uint8_t fps_gen = 25;
uint8_t fps_gen = 29;


uint16_t data_pass = 0;

uint16_t stuck_here_TC = 0;

volatile uint8_t DMX_stat = 0;

extern FIL FileRead_S, FileRead_D;

char SD_Path[] = "0:/";
char USB_Path[] = "1:/";

extern uint8_t SD_Card_stat;
extern uint8_t USB_Drive_stat;

FRESULT error_SD_card;


FATFS USBDISKFatFs;	/* File system object for USB disk logical drive */

FATFS SDFatFs;  /* File system object for SD card logical drive */



USBH_HandleTypeDef hUSB_Host; /* USB Host handle */
FRESULT error_USB_Drive;


extern DIR Directory;

extern uint8_t is_CopyMenu;
uint8_t is_Play_Drive = 0;

uint8_t was_mount_USB_ok = 0;
uint8_t was_mount_SD_ok = 0;

//FATFS fsT0, fsT1;      /* Work area (filesystem object) for logical drives */
FIL fTst;      /* File objects */
FRESULT SDtst, USB_File_Stat, SD_File_Stat;

DMA_HandleTypeDef hdma_spi1_tx;

void MX_I2C1_Init(void);
I2C_HandleTypeDef hi2c1;


uint8_t is_here_LCD = 0; //FOR TEST
uint8_t DELAY_Frames = 0; //FOR TEST

extern uint8_t is_LCD_busy;


/*
uint32_t IC_Val1 = 0;
uint32_t IC_Val2 = 0;
int Is_First_Captured = 0;
*/

uint32_t Difference = 0;

uint32_t Flag_TIM0f = 0;
uint32_t Flag_TIM1f = 0;
uint32_t Flag_TIM2f = 0;
uint32_t Flag_TIM3f = 0;
uint32_t Flag_TIM4f = 0;
        
uint32_t Flag_TIM2fx = 0;
uint32_t Flag_TIM1 = 0;
uint32_t Flag_TIM2 = 0;
uint32_t Flag_TIM3 = 0;
uint32_t Flag_TIM4 = 0;


uint32_t is_error = 0;
uint16_t prescalervalue = 0;

uint32_t freq_buz_pwm = 250000;

extern uint8_t file_read_error;

extern I2S_HandleTypeDef       hAudioOutI2s;
extern __IO HOST_StateTypeDef USB_stats;
extern uint8_t state_wave_player;
extern uint8_t USB_recconnect;


/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;



UART_HandleTypeDef huart4;
UART_HandleTypeDef huart5;
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
UART_HandleTypeDef huart6;
UART_HandleTypeDef huart7;
UART_HandleTypeDef huart8;


TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim7;
TIM_HandleTypeDef htim9;
TIM_HandleTypeDef htim13;



RTC_HandleTypeDef hrtc;
RTC_TimeTypeDef sTime;
RTC_DateTypeDef sDate;
HAL_StatusTypeDef resX;



/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef hTimLed;
TIM_OC_InitTypeDef sConfigLed;

/* Capture Compare Register Value.
Defined as external in stm32f4xx_it.c file */
__IO uint16_t CCR1Val = 16826;	    
__IO uint32_t LEDsState;

/* NOTE: USBDISKFatFs, SDFatFs, hUSB_Host already defined above (lines 76-82).
 * Duplicate definitions removed to prevent undefined behavior. */
char USBDISKPath[4];         /* USB Host logical drive path */
char SDPath[4]; /* SD card logical drive path */

MSC_ApplicationTypeDef AppliState = APPLICATION_IDLE;
//static uint8_t  USBH_USR_ApplicationState = USBH_USR_FS_INIT;
uint8_t  USBH_USR_ApplicationState = USBH_USR_FS_INIT;

/* Private function prototypes -----------------------------------------------*/

static void SystemClock_Config(void);
static void USBH_UserProcess(USBH_HandleTypeDef *pHost, uint8_t vId);
//static void Delay(__IO uint32_t nCount);

static void player_app(void);
static void MX_TIM13_Init(void);






#if defined(CONFIG_TEST_USB) || defined(CONFIG_TEST_SD)
static void fault_err (FRESULT rc);
#endif  // defined(CONFIG_TEST_USB) || defined(CONFIG_TEST_SD)






enum {
      TASK_Rest = 0,
      TASK_PCMessage,
      TASK_MSMessage,
      TASK_TimeEvent,
      TASK_Ping,
      TASK_Getting,
      TASK_Reporting,
};


void HandleError(unsigned char cc)
 {
       // Called when visualSTATE encounters a problem
       // Nothing here for the moment..
 }


uint32_t task_Scheduler;

//uint32_t TC_XXX_Frames[8] = {0};
uint32_t last_valX[8] = {0};

//uint32_t TC_XXX_FramesA[1000] = {0}; test fps







/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/

static void MX_SPI1_Init(void);


void MX_UART4_Init(void);
static void MX_USART1_UART_Init(void);
void MX_UART7_UART_Init(void);// Time Code
static void MX_USART3_UART_Init(uint8_t uart_speed); // GPS
static void MX_USART6_UART_Init(void); //433MHZ tRANCEIVER



//UartInit((LPC_UART_TypeDef *)TC_UART, 1200, UART_DATABIT_8, UART_STOPBIT_1, UART_PARITY_NONE); // PD FSK
 //UartInit((LPC_UART_TypeDef *)TC_UART, 300, UART_DATABIT_8, UART_STOPBIT_1, UART_PARITY_NONE);// F1 FSK
 

void MX_TIM1_Init(void);
void MX_TIM2_Init(void);
void MX_TIM3_Init(void);
void MX_TIM4_Init(void);
void MX_TIM5_Init(void);
void MX_TIM6_Init(void);

void MX_GPIO_Init_Button(void);

void MX_RNG_Init(void);
RNG_HandleTypeDef hrng;
uint32_t no_rng_gen = 0;


/* Private functions ---------------------------------------------------------*/

uint8_t main_volume = 60;

/*
char filenames[][_MAX_LFN] = 
{
"art_of_gardens.wav",
"file_example_WAV_1MG.wav",
"file_example_WAV_2MG.wav",
"file_example_WAV_5MG.wav",
"file_example_WAV_10MG.wav",
"Calibration.wav",
"LTC_00_00_00_00__3mins_25.wav",
};
*/

//uint8_t *data = "b;a bla bl;a nbla\n";

uint8_t is_init_done = 0;

/*
char filenames[][_MAX_LFN] = 
 {
       //"art_of_gardens_48k_PCM_s16b.wav",
       "Music.wav",
       "art_of_gardens_48k_PCM_s16b_mono.wav",
       "art_of_gardens_48k_PCM_s24b.wav",
       "art_of_gardens_48k_PCM_s24b_mono.wav",
       "art_of_gardens_48k_PCM_s32b.wav",
       "art_of_gardens_48k_PCM_s32b_mono.wav",
       "art_of_gardens_48k_PCM_u8b.wav",
       "art_of_gardens_48k_PCM_u8b_mono.wav",
       "art_of_gardens_96k_PCM_s16b.wav",
 };
 */

//uint8_t filename_selected;

//#define NUMBER_OF_FILES (sizeof(filenames) / sizeof(filenames[0]))




/**
* @brief  Main program
* @param  None
* @retval None
*/
int main(void)
       {

       /* STM32F4xx HAL library initialization:
       - Configure the Flash prefetch, instruction and Data caches
       - Configure the Systick to generate an interrupt each 1 msec
       - Set NVIC Group Priority to 4
       - Global MSP (MCU Support Package) initialization
       MUST be called BEFORE any peripheral init (SPI, GPIO, LCD, etc.)
       */
       HAL_Init();
       SystemClock_Config();

       /* Initialize Independent Watchdog — auto-reset on HardFault/lockup (~26s timeout) */
       IWDG_HandleTypeDef hiwdg;
       hiwdg.Instance = IWDG;
       hiwdg.Init.Prescaler = IWDG_PRESCALER_256;
       hiwdg.Init.Reload = 4095;  /* ~32 kHz LSI / 256 / 4095 ≈ 26 seconds */
       HAL_IWDG_Init(&hiwdg);

       /* Now safe to init peripherals after HAL and clocks are configured */
       MX_SPI1_Init();
       MX_GPIO_Init_LCD();
       LCD_Initial();

       unsigned char cc;
       SEM_EVENT_TYPE eventNo;

       // Initialize the VS System. Actual function name are different
       // for the two different code generation types. Also, for Table
       // Based code generation, we need an actionExpressionNo variable

#ifdef VS_READABLE_CODE
       VSInitAll();
#else
       SEM_ACTION_EXPRESSION_TYPE actionExpressNo;
       System1SEM_InitAll();
#endif

       // Initialize the VS Event queue and add a first SE_RESET event to start with
       SEQ_Initialize();
       SEQ_AddEvent(SE_RESET);

       // Initializing RealLink API if included.
#if (VS_REALLINKMODE == 1)
       VS_RealLinkInit();
#endif
              
      // MX_UART4_Init();
      // Ringbuf_init1();
      
   //   MX_USB_DEVICE_Init();
      
      
      
     load_flash_script();
     update_Config();

#if 1



    //init some pins and buttons
       MX_GPIO_Init_Button();


//Random number init
     //  MX_RNG_Init();
     
         // RealTime clock config
    __HAL_RCC_RTC_ENABLE();
    StartRTC_Routine();
     
    Get_MCU_time();
    
    //wireless uart
     MX_USART1_UART_Init();
     Ringbuf_init();
     
     //868 UART
        MX_USART6_UART_Init();
        Ringbuf433_init();
        
        
            set_AD_ID = !(HAL_GPIO_ReadPin(ROT_ENCODE_D_PORT, ROT_ENCODE_D_PIN));

            uint8_t saved_adr = adress_interface();
         

            while(set_AD_ID == 1){
                static uint16_t was_lcd_refsehed = 0;
                    
                if(was_lcd_refsehed > 100){
                        settings_display();
                        was_lcd_refsehed = 0;
                }else{
                        was_lcd_refsehed++;
                        getWirMessages();
                }
            
            }
            
            if(saved_adr != adress_interface()){
                //save flash
                save_script_flash();
            }
        
        
       
   wave_player_volume_set(main_volume);
       
    
    error_SD_card = Test_SD_Card();
    
   if(error_SD_card != FR_OK)
    {
        Delay(50);
        FATFS_UnLinkDriverEx("0:/", 0);
        Delay(50);
        error_SD_card = Test_SD_Card();
    }
    
    if(error_SD_card != FR_OK)
    {
        was_mount_USB_ok = 0;
        is_Play_Drive = 1;
    }
    else
    {
       AppliState = APPLICATION_START;
       USBH_USR_ApplicationState = USBH_USR_FS_INIT;
    }
    
    
        //ADC Init
    MX_SPI2_Init();
    /*
        HAL_Delay(5);
    readAdcA(VBUS_ADC_CH - 0);
        HAL_Delay(10);
    readAdcA(VBUS_ADC_CH - 0);
        HAL_Delay(10);
    readAdcA(VBUS_ADC_CH - 0);
            HAL_Delay(10);
    readAdcA(VBUS_ADC_CH - 0);
            HAL_Delay(10);
    readAdcA(VBUS_ADC_CH - 0);
        HAL_Delay(10);
*/
    
  
        if(HAL_GPIO_ReadPin(USB_VBUS_Port, USB_VBUS_Pin) == GPIO_PIN_RESET){//If is connected
                Init_USB_Drive();                           
        }


    player_app();
       

    
       
       //DMX UART DMA and Timer
    set_rst_DMX_Line(10);
    MX_TIM6_Init_LL();
    MX_UART8_InitLL();
    
    
    
    //Buzzer Init
    MX_TIM13_Init();





    //TimeCode UART FSK Init - 
    MX_UART7_UART_Init();
    RingbufTC_init();
  
    

    //GPS UART Init
    MX_USART3_UART_Init(0); //Default UART
    RingbufGPS_init();
    
    GPS_Update_Settings(0);
    GPS_Update_Settings(1);
    GPS_Update_Settings(2);
    GPS_Update_Settings(3);
    MX_USART3_UART_Init(1); //UART 115k
    
    


        
    //Wireless init
//    MX_USART1_UART_Init();
    //Ringbuf_init();
    InitWireless();
   // Config_Wir(0);
   
    //433mHZ WIRELES TRANCEIVER
    if(IS_433_MHZ_ENABLED == 1){
           // MX_USART6_UART_Init();
           // Ringbuf433_init();
            
            HAL_Delay(5);  
            
            start_433_Mhz();
    }
   

 //   GPS_Update_Settings(1);

     set_AB_Master_slave();    
    
    Init_TimeCode();
    
//DMX tests
   /*
    DMX_stat = 1;
    usart_start_tx_dma();
    start_dmx_timer();
*/
    
#endif    
        
#if 1
       task_Scheduler = TASK_PCMessage;

    
       for(;;)
        {

               

              HAL_IWDG_Refresh(&hiwdg);  /* Feed the watchdog every main loop iteration */

              player_app();// FOR TESTS


               /*
               data_pass++;
               if(data_pass == 10000){
                        CDC_Transmit_FS(data, strlen(data));
                             data_pass = 0;
                     }
*/
              DoDisplayTasks();
                
                
              do_FSK_TimeCode();

              task_Scheduler++;
              switch (task_Scheduler)
               {
                 case TASK_PCMessage:  //      USB status
                     /*
                     if (usb_host_mode == 0)
                     {
                     getUSBEvents();		
               }*/
                     break;
                 case TASK_MSMessage:
                     
                     getWirMessages();
                     getWirMessages();
                     getMSEvent();
                     getMSEvent();
                     getMSEvent();
              
                     break;
                 case TASK_TimeEvent:
                     updateTimers();
                     break;
                 case TASK_Ping:
                        if(IS_433_MHZ_ENABLED == 1){
                                w433_task();
                        }
                 
                     break;
                 case TASK_Getting:
                     do_ADC_Tasks();
                     gps_task();
                     break;
                 case TASK_Reporting:
                         
                      
                     //do_upload_script();
                    // do_LCD_transfer();
                     task_Scheduler = TASK_Rest;
                     break;
                 default:
                     task_Scheduler = TASK_Rest;
                     break;
               }
              
              while (SEQ_RetrieveEvent(&eventNo) != UCC_QUEUE_EMPTY)
               {
#ifdef VS_READABLE_CODE
                     // Deduct and handle in a single function for Readable Code
                     cc = VSDeduct(eventNo);
                     if (cc != SES_OKAY)
                           HandleError(eventNo);
#else
                     // Deduct the event for classical Table based code
                     if ((cc = System1SEM_Deduct(eventNo)) != SES_OKAY)
                           HandleError(eventNo);
                     // Get resulting action expressions and execute them
                     while ((cc = System1SEM_GetOutput(&actionExpressNo)) == SES_FOUND)
                           System1SEM_Action(actionExpressNo);
                     // Check for error
                     if (cc != SES_OKAY)
                           HandleError(cc);
                     // Change the next state vector
                     if ((cc = System1SEM_NextState()) != SES_OKAY)
                           HandleError(cc);
#endif
               }
        }
#endif

       
       

 }


/**
* @brief  User Process
* @param  phost: Host Handle
* @param  id: Host Library user message ID
* @retval None
*/
static void USBH_UserProcess (USBH_HandleTypeDef *pHost, uint8_t vId)
 {  
         
         
       switch (vId)
        { 
          case HOST_USER_CLASS_ACTIVE:
              // printf("USBH_UserProcess: HOST_USER_CLASS_ACTIVE\r\n");
          /*
              if(PPS_Mp3_AB != 1 && device_Status < Status_PowerEnable)
              {
                      USBH_USR_ApplicationState = USBH_USR_FS_INIT;
                      AppliState = APPLICATION_START;
                      AP_Switch_UD();
              }
          */
          /*
              if(PPS_Mp3_AB != 1 && device_Status < Status_PowerEnable && (is_init_done == 0 || is_init_done == 100))
              {
                      USBH_USR_ApplicationState = USBH_USR_FS_INIT;
                      AppliState = APPLICATION_START;
                      AP_Switch_UD();
              }
          */
              break;
          case HOST_USER_DISCONNECTION:
              //  printf("USBH_UserProcess: HOST_USER_DISCONNECTION\r\n");
          /*
              if(is_Play_Drive == 1)
              {
                      WavePlayer_CallBack();
                      AppliState = APPLICATION_IDLE;
              }
          */
              f_mount(NULL, "1:/", 0);	
              break;
              
          case HOST_USER_SELECT_CONFIGURATION:
              //    printf("USBH_UserProcess: HOST_USER_SELECT_CONFIGURATION\r\n");
              break;
          case HOST_USER_CLASS_SELECTED:
              //  printf("USBH_UserProcess: HOST_USER_CLASS_SELECTED\r\n");
              break;
          case HOST_USER_UNRECOVERED_ERROR:
              // printf("USBH_UserProcess: HOST_USER_UNRECOVERED_ERROR\r\n");
              break;
          case HOST_USER_CONNECTION:
              //  printf("USBH_UserProcess: HOST_USER_CONNECTION\r\n");
              break;
          default:
              //	  printf("USBH_UserProcess: default, %02X\r\n", vId);
              break; 
        }
 }


/**
* @brief  System Clock Configuration
*         The system Clock is configured as follow : 
*	  System Clock source	  = PLL (HSE)
*	  SYSCLK(Hz)		 = 168000000
*	  HCLK(Hz)		   = 168000000
*	  AHB Prescaler	        = 1
*	  APB1 Prescaler	       = 4
*	  APB2 Prescaler	       = 2
*	  HSE Frequency(Hz)	    = 8000000
*	  PLL_M		      = 8
*	  PLL_N		      = 336
*	  PLL_P		      = 2
*	  PLL_Q		      = 7
*	  VDD(V)		     = 3.3
*	  Main regulator output voltage  = Scale1 mode
*	  Flash Latency(WS)	    = 5
* @param  None
* @retval None
*/


#if 0 //audiobox clock config
#if defined(CONFIG_CLOCK_EXTERNAL)
static void SystemClock_Config(void)
 {
       RCC_ClkInitTypeDef RCC_ClkInitStruct;
       RCC_OscInitTypeDef RCC_OscInitStruct;
       
       /* Enable Power Control clock */
       __HAL_RCC_PWR_CLK_ENABLE();
       
       /* The voltage scaling allows optimizing the power consumption when the device is 
       clocked below the maximum system frequency, to update the voltage scaling value 
       regarding system frequency refer to product datasheet.  */
       __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
       
       /* Enable HSE Oscillator and activate PLL with HSE as source */
       RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
       RCC_OscInitStruct.HSEState = RCC_HSE_ON;
       RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
       RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
       RCC_OscInitStruct.PLL.PLLM = 8;
       RCC_OscInitStruct.PLL.PLLN = 336;
       RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
       RCC_OscInitStruct.PLL.PLLQ = 7;
       if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
        {
              Error_Handler();
        }
       
       /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2 
       clocks dividers */
       RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
       RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
       RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
       RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;  
       RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;  
       if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
        {
              Error_Handler();
        }
       
       /* STM32F405x/407x/415x/417x Revision Z devices: prefetch is supported  */
       if (HAL_GetREVID() == 0x1001)
        {
              /* Enable the Flash prefetch */
              __HAL_FLASH_PREFETCH_BUFFER_ENABLE();
        }  
 }
#endif  // defined(CONFIG_CLOCK_EXTERNAL)
#endif

#if 1
#if defined(CONFIG_CLOCK_EXTERNAL)
static void SystemClock_Config(void)
 {
       RCC_ClkInitTypeDef RCC_ClkInitStruct;
       RCC_OscInitTypeDef RCC_OscInitStruct;
       RCC_PeriphCLKInitTypeDef PeriphClkInitStruct;
       RCC_OscInitTypeDef RCC_OscInitLSE;
         
       /* Enable Power Control clock */
       __HAL_RCC_PWR_CLK_ENABLE();
       
       /* The voltage scaling allows optimizing the power consumption when the device is 
       clocked below the maximum system frequency, to update the voltage scaling value 
       regarding system frequency refer to product datasheet.  */
       __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
       
       /* Enable HSE Oscillator and activate PLL with HSE as source */
       RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
       RCC_OscInitStruct.HSEState = RCC_HSE_ON;
       RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
       RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
       RCC_OscInitStruct.PLL.PLLM = 8;
       RCC_OscInitStruct.PLL.PLLN = 336;
       RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
       RCC_OscInitStruct.PLL.PLLQ = 7;
       if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
        {
              Error_Handler();
        }
       
       /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2 
       clocks dividers */
       RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
       RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
       RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
       RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;  
       RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;  
       if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
        {
              Error_Handler();
        }
       
       /* STM32F405x/407x/415x/417x Revision Z devices: prefetch is supported  */
       if (HAL_GetREVID() == 0x1001)
        {
              /* Enable the Flash prefetch */
              __HAL_FLASH_PREFETCH_BUFFER_ENABLE();
        }  
        
        
        PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_RTC;
        PeriphClkInitStruct.RTCClockSelection = RCC_RTCCLKSOURCE_LSE; //RCC_RTCCLKSOURCE_LSI;
        
        
        if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
          {
                Error_Handler();
          }
          
        RCC_OscInitLSE.OscillatorType = RCC_OSCILLATORTYPE_LSE;
        RCC_OscInitLSE.LSEState = RCC_LSE_ON;

        if(HAL_RCC_OscConfig(&RCC_OscInitLSE) != HAL_OK){
              Error_Handler();
        }
 }
#endif  // defined(CONFIG_CLOCK_EXTERNAL)
#endif
 
 
#if defined(CONFIG_CLOCK_INTERNAL)
// internal clock
static void SystemClock_Config(void)
 {
       RCC_ClkInitTypeDef RCC_ClkInitStruct;
       RCC_OscInitTypeDef RCC_OscInitStruct;
       
       /* Enable Power Control clock */
       __HAL_RCC_PWR_CLK_ENABLE();
       
       /* The voltage scaling allows optimizing the power consumption when the device is 
       clocked below the maximum system frequency, to update the voltage scaling value 
       regarding system frequency refer to product datasheet.  */
       __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
       
       
       RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
       RCC_OscInitStruct.HSIState = RCC_HSI_ON;
       //    RCC_OscInitStruct.HSICalibrationValue = 16;
       RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
       //    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
       RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
       RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
       //    RCC_OscInitStruct.PLL.PLLM = 16;
       RCC_OscInitStruct.PLL.PLLM = 8;
       RCC_OscInitStruct.PLL.PLLN = 336;
       RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
       RCC_OscInitStruct.PLL.PLLQ = 7;
       if(HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
        {
              Error_Handler();
        }
       
       RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
             | RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
       RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
       //    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
       //    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
       //    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
       RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
       RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;  
       RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;  
       //    HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0);
       if(HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
        {
              Error_Handler();
        }
#if 0
       HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);
       HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);
       
       /* SysTick_IRQn interrupt configuration */
       //    HAL_NVIC_SetPriority(SysTick_IRQn, 15, 0);
       HAL_NVIC_SetPriority(SysTick_IRQn, SYSTICK_IRQ_PRIORITY, SYSTICK_IRQ_PRIORITY_SUB);
#endif
       /* STM32F405x/407x/415x/417x Revision Z devices: prefetch is supported  */
       if (HAL_GetREVID() == 0x1001)
        {
              /* Enable the Flash prefetch */
              __HAL_FLASH_PREFETCH_BUFFER_ENABLE();
        }
 }
#endif  // defined(CONFIG_CLOCK_INTERNAL)



/**
* @brief  This function is executed in case of error occurrence.
* @param  None
* @retval None
*/
void Error_Handler(void)
 {
    is_error++;
 }

/**
* @brief  Output Compare callback in non blocking mode 
* @param  htim : TIM OC handle
* @retval None
*/
void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim)
 {
       
       uint32_t capture = 0; 
       
              
       /* Get the TIM4 Input Capture 1 value */
       capture = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
       
       /* Set the TIM4 Capture Compare1 Register value */
       __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_1, (CCR1Val + capture));
 }



#if defined(CONFIG_TEST_USB) || defined(CONFIG_TEST_SD)
static void fault_err (FRESULT rc)
 {
       const char *str =
             "OK\0" "DISK_ERR\0" "INT_ERR\0" "NOT_READY\0" "NO_FILE\0" "NO_PATH\0"
                   "INVALID_NAME\0" "DENIED\0" "EXIST\0" "INVALID_OBJECT\0" "WRITE_PROTECTED\0"
                         "INVALID_DRIVE\0" "NOT_ENABLED\0" "NO_FILE_SYSTEM\0" "MKFS_ABORTED\0" "TIMEOUT\0"
                               "LOCKED\0" "NOT_ENOUGH_CORE\0" "TOO_MANY_OPEN_FILES\0";
       FRESULT i;
       
       for (i = (FRESULT)0; i != rc && *str; i++) {
             while (*str++) ;
       }
       printf("rc=%u FR_%s\n\r", (UINT)rc, str);
       while(1);
 }
#endif  // defined(CONFIG_TEST_USB) || defined(CONFIG_TEST_SD)

#ifdef USE_FULL_ASSERT

/**
* @brief  Reports the name of the source file and the source line number
*         where the assert_param error has occurred.
* @param  file: pointer to the source file name
* @param  line: assert_param error line source number
* @retval None
*/
void assert_failed(uint8_t* file, uint32_t line)
 { 
       /* User can add his own implementation to report the file name and line number,
       ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
       
       /* Infinite loop */
       while (1)
        {
        }
 }
#endif




/**
* @brief USART1 Initialization Function
* @param None
* @retval None
*/
static void MX_USART1_UART_Init(void)
 {
       
       /* USER CODE BEGIN USART1_Init 0 */
       
       /* USER CODE END USART1_Init 0 */
       
       /* USER CODE BEGIN USART1_Init 1 */
       
       /* USER CODE END USART1_Init 1 */
       huart1.Instance = USART1;
       huart1.Init.BaudRate = 115200;
      // huart1.Init.BaudRate = 19200;
       huart1.Init.WordLength = UART_WORDLENGTH_8B;
       huart1.Init.StopBits = UART_STOPBITS_1;
       huart1.Init.Parity = UART_PARITY_NONE;
       huart1.Init.Mode = UART_MODE_TX_RX;
       huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
       huart1.Init.OverSampling = UART_OVERSAMPLING_16;
       if (HAL_UART_Init(&huart1) != HAL_OK)
        {
              Error_Handler();
        }
       /* USER CODE BEGIN USART1_Init 2 */
       
       /* USER CODE END USART1_Init 2 */
       
 }
 

void MX_UART4_Init(void)
 {
       
       /* USER CODE BEGIN UART4_Init 0 */
       
       /* USER CODE END UART4_Init 0 */
       
       /* USER CODE BEGIN UART4_Init 1 */
       
       /* USER CODE END UART4_Init 1 */
       huart4.Instance = UART4;
       huart4.Init.BaudRate = 115200;
       huart4.Init.WordLength = UART_WORDLENGTH_8B;
       huart4.Init.StopBits = UART_STOPBITS_1;
       huart4.Init.Parity = UART_PARITY_NONE;
       huart4.Init.Mode = UART_MODE_TX_RX;
       huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
       huart4.Init.OverSampling = UART_OVERSAMPLING_16;
       if (HAL_UART_Init(&huart4) != HAL_OK)
        {
              Error_Handler();
        }
       /* USER CODE BEGIN UART4_Init 2 */
       
       /* USER CODE END UART4_Init 2 */
       
 }






#if 1
void player_app(void)
{

        
 switch(AppliState)
            {
                case APPLICATION_START:
                    switch (USBH_USR_ApplicationState)
                    {
                        case USBH_USR_AUDIO:
                            if (wave_player_process() == 0)
                            {
                                //printf("ERROR: wave_player_process ->IDLE\r\n");
                                AppliState = APPLICATION_IDLE;
                                USBH_USR_ApplicationState = USBH_USR_IDLE;
                            }
                            else
                            {
                                    // DO Nothing
                            }
                //            USBH_USR_ApplicationState = USBH_USR_FS_INIT;
                            break;
                        case USBH_USR_FS_INIT:
                            //printf("MSC_Application: USBH_USR_FS_INIT\r\n");
                            /* Initializes the File System */
                        
                       
                         if(is_CopyMenu == 1 || is_CopyMenu == 3)
                         {
                                break;
                         }
                         else if(is_CopyMenu == 2)
                         {
                                 
                                f_close(&FileRead_S);
                                copy_toSD();
                                is_CopyMenu = 3;
                                is_Play_Drive = 1;
                                 
                                AP_Switch_ID();
                                
                                break;
                                 
                         }
                      
                         if(is_Play_Drive == 0)
                         {
                                 if (f_mount(&SDFatFs, "0:/", 1 ) != FR_OK ) 
                                    {
                                        /* FatFs initialisation fails on SD Card*/
                                        break;
                                    }
                         
                         }
                         else if(is_Play_Drive == 1)
                         {
                                 if (f_mount(&USBDISKFatFs, "1:/", 1 ) != FR_OK ) 
                                    {
                                        /* FatFs initialisation fails on USB*/
                                        break;
                                    }
                         }
                         
                            USBH_USR_ApplicationState = USBH_USR_AUDIO;
                            HAL_I2S_DeInit(&hAudioOutI2s);
                            is_init_done = 1;
                            USB_recconnect = 0;
                            file_read_error = 0;  
                         
                            break;
                        default:
                            break;
                    }
                    break;      
                case APPLICATION_IDLE:
                default:
                    break;      
            }

           // USBH_Process(&hUSB_Host);
            /* USBH_Background Process */
            
           if(is_Play_Drive == 1)
            {
                    if(read_mp3_Pstatus() != 1){
                        USBH_Process(&hUSB_Host);
                    }
                   //if(USB_stats == HOST_IDLE && (USBH_USR_ApplicationState != USBH_USR_IDLE || state_wave_player != STATE_WAVE_PLAYER_INIT)) 
                   if(USB_stats == HOST_IDLE) 
                   {
                        AP_Switch_ID();
                        rst_prg_script();
                   }
            }
            else if(need_toSwitch() == 1)
              {
                      USBH_Process(&hUSB_Host);
                     
              }
              
    }
#endif 
  
    

void SPI1_IRQHandler(void)
{
  /* USER CODE BEGIN SPI1_IRQn 0 */

  /* USER CODE END SPI1_IRQn 0 */
  HAL_SPI_IRQHandler(&hspi1);
  /* USER CODE BEGIN SPI1_IRQn 1 */

  /* USER CODE END SPI1_IRQn 1 */
}



void SPI2_IRQHandler(void)
{
  /* USER CODE BEGIN SPI2_IRQn 0 */

  /* USER CODE END SPI2_IRQn 0 */
  HAL_SPI_IRQHandler(&hspi2);
 // HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
     
//  ADC_refresh();
  /* USER CODE BEGIN SPI2_IRQn 1 */

  /* USER CODE END SPI2_IRQn 1 */
}

void TIM1_BRK_TIM9_IRQHandler(void)
{

      HAL_NVIC_DisableIRQ(TIM1_BRK_TIM9_IRQn);
       Difference = HAL_TIM_ReadCapturedValue(&htim9, TIM_CHANNEL_2);  // read second value

        __HAL_TIM_SET_COUNTER(&htim9, 0);  // reset the counter
        HAL_TIM_IRQHandler(&htim9);

       if(Difference > 100){
              SMPTE_IRQHandler(Difference);//smpte decode
       }
              HAL_NVIC_EnableIRQ(TIM1_BRK_TIM9_IRQn);
                
        if(device_Status == Status_Play && was_LTC_Started == 0){
                                HAL_NVIC_DisableIRQ(TIM1_BRK_TIM9_IRQn);
                                Time_code_disable = 2;
                       }
 
}


void rst_Time_Code_init(void){

        __HAL_TIM_SET_COUNTER(&htim9, 0);  // reset the counter
        HAL_NVIC_EnableIRQ(TIM1_BRK_TIM9_IRQn);

        Time_code_disable = 0;
}


void TIM1_CC_IRQHandler(void)
{
        HAL_TIM_IRQHandler(&htim1);
       
                      
}



/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
void MX_GPIO_Init_LCD(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */

  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  


  /*Configure GPIO pins : LCD_CS_Pin*/
/*
  GPIO_InitStruct.Pin = LCD_CS_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LCD_CS_PIN_GPIO_Port, &GPIO_InitStruct);
  
  HAL_GPIO_WritePin(LCD_CS_PIN_GPIO_Port, LCD_CS_PIN, GPIO_PIN_RESET);
*/

  /*Configure GPIO pins : LCD_RST_Pin*/
   GPIO_InitStruct.Pin = LCD_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LCD_RST_GPIO_Port, &GPIO_InitStruct);

  HAL_GPIO_WritePin(LCD_RST_GPIO_Port, LCD_RST_Pin, GPIO_PIN_RESET);
  

  /*Configure GPIO pins : LCD_A0*/
  GPIO_InitStruct.Pin = LCD_A0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LCD_A0_GPIO_Port, &GPIO_InitStruct);
  
  HAL_GPIO_WritePin(LCD_A0_GPIO_Port, LCD_A0_Pin, GPIO_PIN_RESET);


/*Configure GPIO pins : LCD_BL*/
  GPIO_InitStruct.Pin = LCD_BL_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LCD_BL_PORT, &GPIO_InitStruct);
  
  HAL_GPIO_WritePin(LCD_BL_PORT, LCD_BL_PIN, GPIO_PIN_SET);
  
}




void MX_GPIO_Init_Button(void)
{
       
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
        
  
  /* SPI 2 start and reset PINS*/

  
//EXTERNAL TRIGGER
  GPIO_InitStruct.Pin = EXT_TRIG_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(EXT_TRIG_PORT, &GPIO_InitStruct);

  
/*Configure GPIO pin : Rottary encoder */
  GPIO_InitStruct.Pin = ROT_ENCODE_A_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ROT_ENCODE_A_PORT, &GPIO_InitStruct);
  
  GPIO_InitStruct.Pin = ROT_ENCODE_B_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ROT_ENCODE_B_PORT, &GPIO_InitStruct);
  
  
  GPIO_InitStruct.Pin = ROT_ENCODE_D_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ROT_ENCODE_D_PORT, &GPIO_InitStruct);
  
  
/*Configure GPIO pin : PWR_Pin */
  GPIO_InitStruct.Pin = PWR_BUT_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(PWR_BUT_PORT, &GPIO_InitStruct);
  
    //433Mhz Tranceiver M1
  
 GPIO_InitStruct.Pin = RES_B_PIN;
 GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
 GPIO_InitStruct.Pull = GPIO_NOPULL;
 GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
 HAL_GPIO_Init(RES_B_PORT, &GPIO_InitStruct);

 HAL_GPIO_WritePin(RES_B_PORT, RES_B_PIN, GPIO_PIN_RESET);
          

  GPIO_InitStruct.Pin = GPS_PPS_PIN;
//  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPS_PPS_PORT, &GPIO_InitStruct);          


/*Configure GPIO pin : GPS_PPS Pin*/
/*
  GPIO_InitStruct.Pin = GPS_PPS_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  //GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPS_PPS_PORT, &GPIO_InitStruct);          
  
  // EXTI interrupt init
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);  
  */
  
  //Green button
    GPIO_InitStruct.Pin = GreenButton_GPIO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GreenButton_GPIO_Port, &GPIO_InitStruct);

    /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);
  
  //Blue button
    GPIO_InitStruct.Pin = BlueButton_GPIO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(BlueButton_GPIO_Port, &GPIO_InitStruct);
  
    /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);


  GPIO_InitStruct.Pin = BUZ_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(DI_GPIO_Port, &GPIO_InitStruct);
  
  HAL_GPIO_WritePin(BUZ_GPIO_Port, BUZ_Pin, GPIO_PIN_RESET);


  GPIO_InitStruct.Pin = SLEEP_PWR_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SLEEP_PWR_PORT, &GPIO_InitStruct);
  
  HAL_GPIO_WritePin(SLEEP_PWR_PORT, SLEEP_PWR_PIN, GPIO_PIN_SET);
  
  
  //Time code test pin

  GPIO_InitStruct.Pin = CTR_TC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  
 // GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(CTR_TC_GPIO_Port, &GPIO_InitStruct);

/*
   HAL_GPIO_WritePin(GPIOF, GPIO_PIN_8, GPIO_PIN_SET);
   
   HAL_GPIO_WritePin(GPIOF, GPIO_PIN_8, GPIO_PIN_RESET);
        
   HAL_GPIO_WritePin(GPIOF, GPIO_PIN_8, GPIO_PIN_SET);
   
   HAL_GPIO_WritePin(GPIOF, GPIO_PIN_8, GPIO_PIN_RESET);
  */
}


/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{
          hspi1.Instance = SPI1;
          hspi1.Init.Mode = SPI_MODE_MASTER;
          hspi1.Init.Direction = SPI_DIRECTION_2LINES;
          hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
          hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
          hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
          hspi1.Init.NSS = SPI_NSS_HARD_OUTPUT;
          hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
          hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
          hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
          hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
          hspi1.Init.CRCPolynomial = 10;
          if (HAL_SPI_Init(&hspi1) != HAL_OK)
          {
            Error_Handler();
          }
}



void init_SPI_16b(uint8_t dat_bit_size)
{
      
        
  if(dat_bit_size == 0)
  {
          hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  }
  else
  {
        hspi1.Init.DataSize = SPI_DATASIZE_16BIT;
  }
        
  WRITE_REG(hspi1.Instance->CR1, (hspi1.Init.Mode | hspi1.Init.Direction | hspi1.Init.DataSize |
                                  hspi1.Init.CLKPolarity | hspi1.Init.CLKPhase | (hspi1.Init.NSS & SPI_CR1_SSM) |
                                  hspi1.Init.BaudRatePrescaler | hspi1.Init.FirstBit  | hspi1.Init.CRCCalculation));
    
}


void HAL_MspInit(void)
{
  /* USER CODE BEGIN MspInit 0 */

  /* USER CODE END MspInit 0 */

  __HAL_RCC_SYSCFG_CLK_ENABLE();
  __HAL_RCC_PWR_CLK_ENABLE();

  /* System interrupt init*/

  /* USER CODE BEGIN MspInit 1 */

  /* USER CODE END MspInit 1 */
}

/**
* @brief SPI MSP Initialization
* This function configures the hardware resources used in this example
* @param hspi: SPI handle pointer
* @retval None
*/
void HAL_SPI_MspInit(SPI_HandleTypeDef* hspi)
{
 
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(hspi->Instance==SPI1)
  {
  /* USER CODE BEGIN SPI1_MspInit 0 */

  /* USER CODE END SPI1_MspInit 0 */
    /* Peripheral clock enable */
    __HAL_RCC_SPI1_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**SPI1 GPIO Configuration
    PA4     ------> SPI1_NSS
    PA5     ------> SPI1_SCK
    PA7     ------> SPI1_MOSI
    */
    GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* SPI1 interrupt Init */
    HAL_NVIC_SetPriority(SPI1_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(SPI1_IRQn);
  /* USER CODE BEGIN SPI1_MspInit 1 */

  /* USER CODE END SPI1_MspInit 1 */
  }
    else if(hspi->Instance==SPI2)
  {
  /* USER CODE BEGIN SPI2_MspInit 0 */

  /* USER CODE END SPI2_MspInit 0 */
    /* Peripheral clock enable */
    __HAL_RCC_SPI2_CLK_ENABLE();

    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**SPI2 GPIO Configuration
    PB12     ------> SPI2_NSS
    PB13     ------> SPI2_SCK
    PB14     ------> SPI2_MISO
    PB15     ------> SPI2_MOSI
    */
   // GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
    GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* SPI2 interrupt Init */
    HAL_NVIC_SetPriority(SPI2_IRQn, 4, 0);
    HAL_NVIC_EnableIRQ(SPI2_IRQn);
  /* USER CODE BEGIN SPI2_MspInit 1 */

  /* USER CODE END SPI2_MspInit 1 */
  }
}

/**
* @brief SPI MSP De-Initialization
* This function freeze the hardware resources used in this example
* @param hspi: SPI handle pointer
* @retval None
*/
void HAL_SPI_MspDeInit(SPI_HandleTypeDef* hspi)
{
  if(hspi->Instance==SPI1)
  {
  /* USER CODE BEGIN SPI1_MspDeInit 0 */

  /* USER CODE END SPI1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_SPI1_CLK_DISABLE();

    /**SPI1 GPIO Configuration
    PA4     ------> SPI1_NSS
    PA5     ------> SPI1_SCK
    PA7     ------> SPI1_MOSI
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_7);

    /* SPI1 interrupt DeInit */
    HAL_NVIC_DisableIRQ(SPI1_IRQn);
  /* USER CODE BEGIN SPI1_MspDeInit 1 */

  /* USER CODE END SPI1_MspDeInit 1 */
  }
  else if(hspi->Instance==SPI2)
  {
  /* USER CODE BEGIN SPI2_MspDeInit 0 */

  /* USER CODE END SPI2_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_SPI2_CLK_DISABLE();

    /**SPI2 GPIO Configuration
    PB12     ------> SPI2_NSS
    PB13     ------> SPI2_SCK
    PB14     ------> SPI2_MISO
    PB15     ------> SPI2_MOSI
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15);

    /* SPI2 interrupt DeInit */
    HAL_NVIC_DisableIRQ(SPI2_IRQn);
  /* USER CODE BEGIN SPI2_MspDeInit 1 */

  /* USER CODE END SPI2_MspDeInit 1 */
  }

}


/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
//static void MX_SPI2_Init(void)

void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_16BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_HIGH; 
  hspi2.Init.CLKPhase = SPI_PHASE_2EDGE;
 // hspi2.Init.NSS = SPI_NSS_HARD_OUTPUT;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  //hspi2.Init.FirstBit = SPI_FIRSTBIT_LSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}



/**
* @brief TIM_Base MSP Initialization
* This function configures the hardware resources used in this example
* @param htim_base: TIM_Base handle pointer
* @retval None
*/
void HAL_TIM_Base_MspInit(TIM_HandleTypeDef* htim_base)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(htim_base->Instance==TIM1)
  {
  /* USER CODE BEGIN TIM1_MspInit 0 */

  /* USER CODE END TIM1_MspInit 0 */
    /* Peripheral clock enable */
    __HAL_RCC_TIM1_CLK_ENABLE();


  /* USER CODE END TIM1_MspInit 1 */
  }
  else if(htim_base->Instance==TIM2)
  {
  /* USER CODE BEGIN TIM2_MspInit 0 */

  /* USER CODE END TIM2_MspInit 0 */
    /* Peripheral clock enable */
    __HAL_RCC_TIM2_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**TIM2 GPIO Configuration
    PA3     ------> TIM2_CH4
    */
    GPIO_InitStruct.Pin = RCV_Out_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
    HAL_GPIO_Init(RCV_Out_GPIO_Port, &GPIO_InitStruct);

    /* TIM2 interrupt Init */
    HAL_NVIC_SetPriority(TIM2_IRQn, 2, 1);
    HAL_NVIC_EnableIRQ(TIM2_IRQn);
  /* USER CODE BEGIN TIM2_MspInit 1 */

  /* USER CODE END TIM2_MspInit 1 */
  }
  else if(htim_base->Instance==TIM3)
  {
  /* USER CODE BEGIN TIM3_MspInit 0 */

  /* USER CODE END TIM3_MspInit 0 */
    /* Peripheral clock enable */
    __HAL_RCC_TIM3_CLK_ENABLE();
    /* TIM3 interrupt Init */
    HAL_NVIC_SetPriority(TIM3_IRQn, 2, 2);
    HAL_NVIC_EnableIRQ(TIM3_IRQn);
  /* USER CODE BEGIN TIM3_MspInit 1 */

  /* USER CODE END TIM3_MspInit 1 */
  }
  else if(htim_base->Instance==TIM4)
  {
  /* USER CODE BEGIN TIM4_MspInit 0 */

  /* USER CODE END TIM4_MspInit 0 */
    /* Peripheral clock enable */
    __HAL_RCC_TIM4_CLK_ENABLE();

    __HAL_RCC_GPIOD_CLK_ENABLE();
    /**TIM4 GPIO Configuration
    PD15     ------> TIM4_CH4
    */
    GPIO_InitStruct.Pin = GPIO_PIN_14;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF2_TIM4;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    /* TIM4 interrupt Init */
    HAL_NVIC_SetPriority(TIM4_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(TIM4_IRQn);
  /* USER CODE BEGIN TIM4_MspInit 1 */

  /* USER CODE END TIM4_MspInit 1 */
  }
  else if(htim_base->Instance==TIM5)
  {
  /* USER CODE BEGIN TIM5_MspInit 0 */

  /* USER CODE END TIM5_MspInit 0 */
    /* Peripheral clock enable */
    __HAL_RCC_TIM5_CLK_ENABLE();
    /* TIM5 interrupt Init */
    HAL_NVIC_SetPriority(TIM5_IRQn, 2, 5);
    HAL_NVIC_EnableIRQ(TIM5_IRQn);
  /* USER CODE BEGIN TIM5_MspInit 1 */

  /* USER CODE END TIM5_MspInit 1 */
  }
  else if(htim_base->Instance==TIM6)
  {
  /* USER CODE BEGIN TIM6_MspInit 0 */

  /* USER CODE END TIM6_MspInit 0 */
    /* Peripheral clock enable */
    __HAL_RCC_TIM6_CLK_ENABLE();
    /* TIM6 interrupt Init */
    HAL_NVIC_SetPriority(TIM6_DAC_IRQn, 1, 10);
    HAL_NVIC_EnableIRQ(TIM6_DAC_IRQn);
  /* USER CODE BEGIN TIM6_MspInit 1 */

  /* USER CODE END TIM6_MspInit 1 */
  }
  else if(htim_base->Instance==TIM7)
  {
  /* USER CODE BEGIN TIM7_MspInit 0 */

  /* USER CODE END TIM7_MspInit 0 */
    /* Peripheral clock enable */
    __HAL_RCC_TIM7_CLK_ENABLE();
    /* TIM7 interrupt Init */
    HAL_NVIC_SetPriority(TIM7_IRQn, 0, 2);
    HAL_NVIC_EnableIRQ(TIM7_IRQn);
  /* USER CODE BEGIN TIM7_MspInit 1 */

  /* USER CODE END TIM7_MspInit 1 */
  }
  else if(htim_base->Instance==TIM9)
  {
 /* USER CODE BEGIN TIM9_MspInit 0 */

  /* USER CODE END TIM9_MspInit 0 */
    /* Peripheral clock enable */
    __HAL_RCC_TIM9_CLK_ENABLE();

    __HAL_RCC_GPIOE_CLK_ENABLE();
    /**TIM9 GPIO Configuration
    PE6     ------> TIM9_CH2
    */
         
    GPIO_InitStruct.Pin = SMPTE_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL; //GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF3_TIM9;
    HAL_GPIO_Init(SMPTE_GPIO_Port, &GPIO_InitStruct);

    /* TIM9 interrupt Init */
        
    HAL_NVIC_SetPriority(TIM1_BRK_TIM9_IRQn, 0, 1);
    HAL_NVIC_EnableIRQ(TIM1_BRK_TIM9_IRQn);

  /* USER CODE BEGIN TIM9_MspInit 1 */

  /* USER CODE END TIM9_MspInit 1 */
  }
  else if(htim_base->Instance==TIM13)
  {
  /* USER CODE BEGIN TIM4_MspInit 0 */

  /* USER CODE END TIM4_MspInit 0 */
    /* Peripheral clock enable */
    __HAL_RCC_TIM13_CLK_ENABLE();
    /* TIM4 interrupt Init */
   // HAL_NVIC_SetPriority(TIM13_IRQn, 18, 4);
   // HAL_NVIC_EnableIRQ(TIM13_IRQn);
  /* USER CODE BEGIN TIM4_MspInit 1 */

  /* USER CODE END TIM4_MspInit 1 */
  }

}

void HAL_TIM_MspPostInit(TIM_HandleTypeDef* htim)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(htim->Instance==TIM1)
  {
  /* USER CODE BEGIN TIM1_MspPostInit 0 */

  /* USER CODE END TIM1_MspPostInit 0 */
          



  /* USER CODE BEGIN TIM1_MspPostInit 1 */

  /* USER CODE END TIM1_MspPostInit 1 */
  }
  else if(htim->Instance==TIM3)
  {
  /* USER CODE BEGIN TIM3_MspPostInit 0 */

  /* USER CODE END TIM3_MspPostInit 0 */
    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**TIM3 GPIO Configuration
    PB0     ------> TIM3_CH3
    */
          /*
    GPIO_InitStruct.Pin = CTR_TC_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLDOWN;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
    HAL_GPIO_Init(CTR_TC_GPIO_Port, &GPIO_InitStruct);
          */

  /* USER CODE BEGIN TIM3_MspPostInit 1 */

  /* USER CODE END TIM3_MspPostInit 1 */
  }
  else if(htim->Instance==TIM4)
  {
  /* USER CODE BEGIN TIM4_MspPostInit 0 */

  /* USER CODE END TIM4_MspPostInit 0 */

    __HAL_RCC_GPIOD_CLK_ENABLE();
    /**TIM1 GPIO Configuration
    PE14     ------> TIM1_CH4
    */
    GPIO_InitStruct.Pin = LCD_BL_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF2_TIM4;
    HAL_GPIO_Init(LCD_BL_PORT, &GPIO_InitStruct);

  /* USER CODE BEGIN TIM4_MspPostInit 1 */

  /* USER CODE END TIM4_MspPostInit 1 */
  }
  else if(htim->Instance==TIM5)
  {
  /* USER CODE BEGIN TIM5_MspPostInit 0 */

  /* USER CODE END TIM5_MspPostInit 0 */

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**TIM5 GPIO Configuration
    PA2     ------> TIM5_CH3
    */
    GPIO_InitStruct.Pin = DI_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF2_TIM5;
    HAL_GPIO_Init(DI_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN TIM5_MspPostInit 1 */

  /* USER CODE END TIM5_MspPostInit 1 */
  }
  else if(htim->Instance==TIM13)
  {
  /* USER CODE BEGIN TIM13_MspPostInit 0 */

  /* USER CODE END TIM13_MspPostInit 0 */

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**TIM13 GPIO Configuration
    PA6     ------> TIM13_CH1
    */
    GPIO_InitStruct.Pin = BUZ_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF9_TIM13;
    HAL_GPIO_Init(BUZ_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN TIM13_MspPostInit 1 */

  /* USER CODE END TIM13_MspPostInit 1 */
  }

}
/**
* @brief TIM_Base MSP De-Initialization
* This function freeze the hardware resources used in this example
* @param htim_base: TIM_Base handle pointer
* @retval None
*/
void HAL_TIM_Base_MspDeInit(TIM_HandleTypeDef* htim_base)
{
  if(htim_base->Instance==TIM1)
  {
  /* USER CODE BEGIN TIM1_MspDeInit 0 */

  /* USER CODE END TIM1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_TIM1_CLK_DISABLE();

    /**TIM1 GPIO Configuration
    PE11     ------> TIM1_CH2
    */
    HAL_GPIO_DeInit(SMPTE_GPIO_Port, SMPTE_Pin);

    /* TIM1 interrupt DeInit */
    HAL_NVIC_DisableIRQ(TIM1_CC_IRQn);
  /* USER CODE BEGIN TIM1_MspDeInit 1 */

  /* USER CODE END TIM1_MspDeInit 1 */
  }
  else if(htim_base->Instance==TIM2)
  {
  /* USER CODE BEGIN TIM2_MspDeInit 0 */

  /* USER CODE END TIM2_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_TIM2_CLK_DISABLE();

    /**TIM2 GPIO Configuration
    PA3     ------> TIM2_CH4
    */
    HAL_GPIO_DeInit(RCV_Out_GPIO_Port, RCV_Out_Pin);

    /* TIM2 interrupt DeInit */
    HAL_NVIC_DisableIRQ(TIM2_IRQn);
  /* USER CODE BEGIN TIM2_MspDeInit 1 */

  /* USER CODE END TIM2_MspDeInit 1 */
  }
  else if(htim_base->Instance==TIM3)
  {
  /* USER CODE BEGIN TIM3_MspDeInit 0 */

  /* USER CODE END TIM3_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_TIM3_CLK_DISABLE();

    /* TIM3 interrupt DeInit */
    HAL_NVIC_DisableIRQ(TIM3_IRQn);
  /* USER CODE BEGIN TIM3_MspDeInit 1 */

  /* USER CODE END TIM3_MspDeInit 1 */
  }
  else if(htim_base->Instance==TIM4)
  {
  /* USER CODE BEGIN TIM4_MspDeInit 0 */

  /* USER CODE END TIM4_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_TIM4_CLK_DISABLE();

    /* TIM4 interrupt DeInit */
    HAL_NVIC_DisableIRQ(TIM4_IRQn);
  /* USER CODE BEGIN TIM4_MspDeInit 1 */

  /* USER CODE END TIM4_MspDeInit 1 */
  }
  else if(htim_base->Instance==TIM5)
  {
  /* USER CODE BEGIN TIM5_MspDeInit 0 */

  /* USER CODE END TIM5_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_TIM5_CLK_DISABLE();

    /* TIM5 interrupt DeInit */
    HAL_NVIC_DisableIRQ(TIM5_IRQn);
  /* USER CODE BEGIN TIM5_MspDeInit 1 */

  /* USER CODE END TIM5_MspDeInit 1 */
  }
  else if(htim_base->Instance==TIM6)
  {
  /* USER CODE BEGIN TIM6_MspDeInit 0 */

  /* USER CODE END TIM6_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_TIM6_CLK_DISABLE();

    /* TIM6 interrupt DeInit */
    HAL_NVIC_DisableIRQ(TIM6_DAC_IRQn);
  /* USER CODE BEGIN TIM6_MspDeInit 1 */

  /* USER CODE END TIM6_MspDeInit 1 */
  }
  else if(htim_base->Instance==TIM9)
  {
  /* USER CODE BEGIN TIM9_MspDeInit 0 */

  /* USER CODE END TIM9_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_TIM9_CLK_DISABLE();
  /* USER CODE BEGIN TIM9_MspDeInit 1 */

  /* USER CODE END TIM9_MspDeInit 1 */
  }

}




/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

        /*
    uint32_t tmpvalue = 0;
    uint32_t Periodvalue = 0;
    uint32_t prescalervalue = 0;
    uint32_t freq_buz_pwm = 270000;
        */
  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  /* USER CODE BEGIN TIM1_Init 1 */
  
  /*
  tmpvalue = HAL_RCC_GetPCLK1Freq();
  prescalervalue = (uint16_t) ((tmpvalue * 2) / freq_buz_pwm) - 1;
  Periodvalue = SystemCoreClock / (freq_buz_pwm * htim4.Init.Prescaler) - 1;
  */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 999;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 61;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  
 
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = htim1.Init.Period >> 1;//Period 50%
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  
  /* USER CODE BEGIN TIM1_Init 2 */
        HAL_TIM_MspPostInit(&htim1);
  /* USER CODE END TIM1_Init 2 */

}



/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_BOTHEDGE;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}


#if 0
/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

   /* USER CODE BEGIN TIM4_Init 0 */
        /*
    uint32_t tmpvalue = 0;
    uint32_t Periodvalue = 0;
    uint32_t prescalervalue = 0;
    uint32_t freq_buz_pwm = 270000;
        */
  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */
/*
  tmpvalue = HAL_RCC_GetPCLK1Freq();
  prescalervalue = (uint16_t) ((tmpvalue * 2) / freq_buz_pwm) - 1;
  Periodvalue = SystemCoreClock / (freq_buz_pwm * htim4.Init.Prescaler) - 1;
  */
  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  //htim4.Init.Prescaler = prescalervalue;
  htim4.Init.Prescaler = 2; //999;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
 // htim4.Init.Period = Periodvalue;
   htim4.Init.Period = 55999;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = htim4.Init.Period; //>> 1;//Period 50%
  //sConfigOC.Pulse = 130;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}
#endif

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 2;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 55999;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = htim4.Init.Period;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_BOTHEDGE;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim4, &sConfigIC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 0;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 4294967295;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */
  HAL_TIM_MspPostInit(&htim5);

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
void MX_TIM6_Init(void) // Time code output
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 0;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 1175;
  //htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}



/**
  * @brief TIM9 Initialization Function
  * @param None
  * @retval None
  */
void MX_TIM9_Init(void)
{

  /* USER CODE BEGIN TIM9_Init 0 */

  /* USER CODE END TIM9_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM9_Init 1 */

  /* USER CODE END TIM9_Init 1 */
  htim9.Instance = TIM9;
  htim9.Init.Prescaler = 84;
  htim9.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim9.Init.Period = 65535;
  htim9.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim9.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim9) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim9, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim9) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_BOTHEDGE;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 15;
  if (HAL_TIM_IC_ConfigChannel(&htim9, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM9_Init 2 */

  /* USER CODE END TIM9_Init 2 */

}



void start_Buz_PWM(void)
{
  //HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);   
   HAL_TIM_PWM_Start(&htim13, TIM_CHANNEL_1);   
}

void stop_Buz_PWM(void)
{
  //HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_4);   
  HAL_TIM_PWM_Stop(&htim13, TIM_CHANNEL_1);   
}



/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
void MX_UART7_UART_Init(void) // Time code
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart7.Instance = UART7;
  huart7.Init.BaudRate = 9600;
  huart7.Init.WordLength = UART_WORDLENGTH_8B;
  huart7.Init.StopBits = UART_STOPBITS_1;
  huart7.Init.Parity = UART_PARITY_NONE;
  huart7.Init.Mode = UART_MODE_TX_RX;
  huart7.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart7.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart7) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}


/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(uint8_t uart_speed)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  if(uart_speed == 0){
        huart3.Init.BaudRate = 9600;
  }else{
        huart3.Init.BaudRate = 115200;
  }
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}




/**
  * @brief This function handles DMA2 stream5 global interrupt.
  */
void DMA2_Stream5_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2_Stream5_IRQn 0 */

  /* USER CODE END DMA2_Stream5_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_spi1_tx);
 
  //HAL_GPIO_WritePin(LCD_CS_PORT, LCD_CS_PIN, GPIO_PIN_SET);
  is_LCD_busy = 0;
 
  /* USER CODE BEGIN DMA2_Stream5_IRQn 1 */

  /* USER CODE END DMA2_Stream5_IRQn 1 */
}

/*
void set_Tra_finish(void)
{
  HAL_GPIO_WritePin(LCD_CS_PORT, LCD_CS_PIN, GPIO_PIN_SET);
        is_LCD_busy = 0;

}
*/


/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
* @brief I2C MSP Initialization
* This function configures the hardware resources used in this example
* @param hi2c: I2C handle pointer
* @retval None
*/
void HAL_I2C_MspInit(I2C_HandleTypeDef* hi2c)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(hi2c->Instance==I2C1)
  {
  /* USER CODE BEGIN I2C1_MspInit 0 */

  /* USER CODE END I2C1_MspInit 0 */

    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**I2C1 GPIO Configuration
    PB8     ------> I2C1_SCL
    PB9     ------> I2C1_SDA
    */
    GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull = GPIO_PULLUP; // 
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW; //GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* Peripheral clock enable */
    __HAL_RCC_I2C1_CLK_ENABLE();
    /* I2C1 interrupt Init */
    HAL_NVIC_SetPriority(I2C1_EV_IRQn, 3, 0);
    HAL_NVIC_EnableIRQ(I2C1_EV_IRQn);
  /* USER CODE BEGIN I2C1_MspInit 1 */

  /* USER CODE END I2C1_MspInit 1 */
  }

}



FRESULT Test_SD_Card(void)
{    
   SDtst = FR_DISK_ERR;   
   if(FATFS_LinkDriver(&SD_Driver, "0:/") == 0)
   {
           SDtst = f_mount(&SDFatFs, "0:/", 0);

           if(SDtst == FR_OK)
            {        
                Test_SD_File();
                SDtst =  SD_File_Stat;
                f_mount(0, "0:/", 0);
            }
    }
   
    return (SDtst);
}


void Init_USB_Drive(void)
{    
    if(FATFS_LinkDriver(&USBH_Driver, "1:/") == 0)
    { 
        /*##-2- Init Host Library ################################################*/
        USBH_Init(&hUSB_Host, USBH_UserProcess, 0);
        /*##-3- Add Supported Class ##############################################*/
        USBH_RegisterClass(&hUSB_Host, USBH_MSC_CLASS);
        /*##-4- Start Host Process ###############################################*/
        USBH_Start(&hUSB_Host);
    } 
}

void DeInit_USB_Drive(void)
{    
        FATFS_UnLinkDriverEx("1:/", 0);
        USBH_Stop(&hUSB_Host);
        USBH_DeInit(&hUSB_Host);
}

FRESULT Test_USB_Drive(void)
{    

           SDtst = f_mount(&USBDISKFatFs, "1:/", 0);
           if(SDtst == FR_OK)
            {        
                   Test_USB_File();
                   SDtst = f_mount(0, "1:/", 0);
            }

     return (SDtst);
}

void Test_USB_File(void)
{    
        
//        char pathX[16] = {0};
        
                //SDtst = f_opendir(&Directory, "1:/");
                SDtst = f_mount(&USBDISKFatFs, "1:/", 0);
                    if(SDtst == FR_OK)
                    {
                              /*
                            for(int x = 1; x < 6; x++){
                              sprintf(pathX, "1:/Audio%d.wav", x);
                              SDtst = f_open(&fTst, pathX, FA_READ);
                                if(SDtst == FR_OK)
                                {
                                        is_seq_A[x - 1] = is_seq_A[x - 1] + 1;
                                        SDtst = f_close(&fTst);
                                }
                            }
                            */
                                
                               SDtst = f_open(&fTst, "1:/Audio.wav", FA_READ);
                               if(SDtst == FR_OK)
                                {
                                        SDtst = f_close(&fTst);
                                }
                                
                    }
                    
            
        USB_File_Stat = SDtst;
}


void Test_SD_File(void)
{
        char pathX[16] = {0};
        
                SDtst = f_opendir(&Directory, "0:/");
                    if(SDtst == FR_OK)
                    {
                            
                            for(int x = 1; x < 6; x++){
                                      sprintf(pathX, "0:/Audio%d.wav", x);
                                      SDtst = f_open(&fTst, pathX, FA_READ);
                                        if(SDtst == FR_OK)
                                        {
                                                
                                                is_seq_A[x - 1] = is_seq_A[x - 1] + 10;
                                                SDtst = f_close(&fTst);
                                     
                                        }
                                }
                              SDtst = f_open(&fTst, "0:/Audio.wav", FA_READ);
                               if(SDtst == FR_OK)
                                {
                                        SDtst = f_close(&fTst);

                                }
                    }
        SD_File_Stat = SDtst;
}

#if 0
static void MX_RTC_Init(void)
{

    /**Initialize RTC Only 
    */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  HAL_PWR_EnableBkUpAccess();
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    //_Error_Handler(__FILE__, __LINE__);
  }


    /**Initialize RTC and set the Time and Date 
    */
  /*
  if(HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR0) != 0x32F2){
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN) != HAL_OK)
  {
    //_Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN) != HAL_OK)
  {
    //_Error_Handler(__FILE__, __LINE__);
  }

    HAL_RTCEx_BKUPWrite(&hrtc,RTC_BKP_DR0,0x32F2);
  }
*/
}


void StartRTC_Routine(void)
{
       MX_RTC_Init();
        
                #if 0
    HAL_PWR_EnableBkUpAccess();
    __HAL_RCC_RTC_ENABLE();
  hrtc1.Instance = RTC;
  hrtc1.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc1.Init.AsynchPrediv = 127;
  hrtc1.Init.SynchPrediv = 255;
  HAL_RTC_Init(&hrtc1); 
        

    if(HAL_RTCEx_BKUPRead(&hrtc1,RTC_BKP_DR0) != RTC_BKP_DATE_TIME_UPDTATED)             
    {
        /*RTC Init*/
        MX_RTC_Init();

        RTC_Default();
        HAL_RTCEx_BKUPWrite(&hrtc1,RTC_BKP_DR0,RTC_BKP_DATE_TIME_UPDTATED);                  
    }
    else
    {
        /* Check if the Power On Reset flag is set */
        if(__HAL_RCC_GET_FLAG(RCC_FLAG_PINRST) != RESET)
        {
            MyFlag = 0xBA;
        }
        /* Check if the Soft Reset flag is set */
        if(__HAL_RCC_GET_FLAG(RCC_FLAG_SFTRST) != RESET)
        {
            MyFlag = 0xFB;
        }       
    }
    #endif
}
#endif

void StartRTC_Routine(void)
{
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  HAL_PWR_EnableBkUpAccess();
        
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
        Error_Handler();
  }
}




void set_BackLight_kbd(uint8_t dev_state)
{
        /*
 if(dev_state == Status_Idle)
 {
        HAL_GPIO_WritePin(LEDR_PORT, LEDR_PIN, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(LEDG_PORT, LEDG_PIN, GPIO_PIN_SET);
        HAL_GPIO_WritePin(LEDB_PORT, LEDB_PIN, GPIO_PIN_RESET);
        
 }
 else if(dev_state == Status_PowerEnable)
 {
        HAL_GPIO_WritePin(LEDR_PORT, LEDR_PIN, GPIO_PIN_SET);
        HAL_GPIO_WritePin(LEDG_PORT, LEDG_PIN, GPIO_PIN_SET);
        HAL_GPIO_WritePin(LEDB_PORT, LEDB_PIN, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(LEDR_PORT, LEDR_PIN, GPIO_PIN_SET);
        HAL_GPIO_WritePin(LEDG_PORT, LEDG_PIN, GPIO_PIN_SET);
        HAL_GPIO_WritePin(LEDB_PORT, LEDB_PIN, GPIO_PIN_SET);
 }
 else if(dev_state == Status_Play)
 {
        HAL_GPIO_WritePin(LEDR_PORT, LEDR_PIN, GPIO_PIN_SET);
        HAL_GPIO_WritePin(LEDG_PORT, LEDG_PIN, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(LEDB_PORT, LEDB_PIN, GPIO_PIN_RESET);
 }
 else if(dev_state == Status_Pause)
 {
         HAL_GPIO_WritePin(LEDR_PORT, LEDR_PIN, GPIO_PIN_SET);
        HAL_GPIO_WritePin(LEDG_PORT, LEDG_PIN, GPIO_PIN_SET);
        HAL_GPIO_WritePin(LEDB_PORT, LEDB_PIN, GPIO_PIN_RESET);
 }
 else
 {
        HAL_GPIO_WritePin(LEDR_PORT, LEDR_PIN, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(LEDG_PORT, LEDG_PIN, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(LEDB_PORT, LEDB_PIN, GPIO_PIN_RESET);
 
 }
       */  

}


static void MX_TIM13_Init(void)
{

   /* USER CODE BEGIN TIM4_Init 0 */
        /*
    uint32_t tmpvalue = 0;
    uint32_t Periodvalue = 0;
    uint32_t prescalervalue = 0;
    uint32_t freq_buz_pwm = 270000;
        */
  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */
/*
  tmpvalue = HAL_RCC_GetPCLK1Freq();
  prescalervalue = (uint16_t) ((tmpvalue * 2) / freq_buz_pwm) - 1;
  Periodvalue = SystemCoreClock / (freq_buz_pwm * htim4.Init.Prescaler) - 1;
  */
  /* USER CODE END TIM4_Init 1 */
  htim13.Instance = TIM13;
  //htim4.Init.Prescaler = prescalervalue;
  htim13.Init.Prescaler = 999;
  htim13.Init.CounterMode = TIM_COUNTERMODE_UP;
 // htim4.Init.Period = Periodvalue;
   htim13.Init.Period = 30;
  htim13.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim13.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim13) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim13, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim13) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim13, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = htim13.Init.Period >> 1;//Period 50%
  //sConfigOC.Pulse = 130;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim13, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim13);

}

void set_lcd_BL(uint8_t percent){
        
  TIM_OC_InitTypeDef sConfigOC = {0};
        
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
        
  sConfigOC.Pulse = (percent * htim4.Init.Period )/100;

  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
   HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1); 
}

        


uint8_t get_Is_seq_music(uint8_t pos){
        return is_seq_A[pos - 1];
}




/**
  * @brief USART6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 9600;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */

}

/**
  * @brief RNG Initialization Function
  * @param None
  * @retval None
  */
void MX_RNG_Init(void)
{

  /* USER CODE BEGIN RNG_Init 0 */

  /* USER CODE END RNG_Init 0 */

  /* USER CODE BEGIN RNG_Init 1 */

  /* USER CODE END RNG_Init 1 */
  hrng.Instance = RNG;
  if (HAL_RNG_Init(&hrng) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RNG_Init 2 */

   HAL_RNG_GenerateRandomNumber_IT(&hrng);
  
  /* USER CODE END RNG_Init 2 */

}

/**
  * @brief This function handles HASH and RNG global interrupts.
  */
void HASH_RNG_IRQHandler(void)
{
  /* USER CODE BEGIN HASH_RNG_IRQn 0 */

  /* USER CODE END HASH_RNG_IRQn 0 */
  HAL_RNG_IRQHandler(&hrng);
        
  no_rng_gen = hrng.RandomNumber;
        
  /* USER CODE BEGIN HASH_RNG_IRQn 1 */

  /* USER CODE END HASH_RNG_IRQn 1 */
}

/**
* @brief RNG MSP Initialization
* This function configures the hardware resources used in this example
* @param hrng: RNG handle pointer
* @retval None
*/
void HAL_RNG_MspInit(RNG_HandleTypeDef* hrng)
{
  if(hrng->Instance==RNG)
  {
  /* USER CODE BEGIN RNG_MspInit 0 */

  /* USER CODE END RNG_MspInit 0 */
    /* Peripheral clock enable */
    __HAL_RCC_RNG_CLK_ENABLE();
    /* RNG interrupt Init */
    HAL_NVIC_SetPriority(HASH_RNG_IRQn, 2, 2);
    HAL_NVIC_EnableIRQ(HASH_RNG_IRQn);
  /* USER CODE BEGIN RNG_MspInit 1 */

  /* USER CODE END RNG_MspInit 1 */
  }

}

/**
* @brief RNG MSP De-Initialization
* This function freeze the hardware resources used in this example
* @param hrng: RNG handle pointer
* @retval None
*/
void HAL_RNG_MspDeInit(RNG_HandleTypeDef* hrng)
{
  if(hrng->Instance==RNG)
  {
  /* USER CODE BEGIN RNG_MspDeInit 0 */

  /* USER CODE END RNG_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_RNG_CLK_DISABLE();

    /* RNG interrupt DeInit */
    HAL_NVIC_DisableIRQ(HASH_RNG_IRQn);
  /* USER CODE BEGIN RNG_MspDeInit 1 */

  /* USER CODE END RNG_MspDeInit 1 */
  }

}


uint16_t get_net_Adr_gen(void){
        uint8_t tmp_net_id = 0xFF & (no_rng_gen >> 16);
        return tmp_net_id;
}



uint16_t get_Key_gen(void){
        return (uint16_t)(no_rng_gen);

}





void set_DMX_timer(uint16_t presc, uint16_t perdio){
  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = presc;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = perdio;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
 // htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

 
void set_rst_DMX_Line(uint8_t dmx_line){
GPIO_InitTypeDef GPIO_InitStruct = {0};



  /*Configure GPIO pins: DMX UART TX Pin*/
  GPIO_InitStruct.Pin = DMX_TX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(DMX_TX_Port, &GPIO_InitStruct);

        if(dmx_line == 0){
                HAL_GPIO_WritePin(DMX_TX_Port, DMX_TX_Pin, GPIO_PIN_RESET);
        }else{
                HAL_GPIO_WritePin(DMX_TX_Port, DMX_TX_Pin, GPIO_PIN_SET);
        }

        if(dmx_line == 10){

         __HAL_RCC_GPIOA_CLK_ENABLE();
                
          /*Configure GPIO pins: DMX CTR TX/RX Pin*/
          GPIO_InitStruct.Pin = DMX_CTRL_Pin;
          GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
          GPIO_InitStruct.Pull = GPIO_PULLDOWN;
          GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
          HAL_GPIO_Init(DMX_CTRL_Port, &GPIO_InitStruct);

          HAL_GPIO_WritePin(DMX_CTRL_Port, DMX_CTRL_Pin, GPIO_PIN_SET);
        }
  
}


void Disbale_FSK(void){//disbale FSK... Start mode read FSK PD
        /*
    UartFSK_CMD_Mode();
    HAL_Delay(100);
    UartFSK_PD_set(0);
    HAL_Delay(500);
    UartFSK_CMD_ModeX();
    HAL_Delay(100);    
    UartFSK_ATX_set(1);
    HAL_Delay(500);
        */
}

void Disable_TC_SMPTE_Read_Pin(void){//Disable SMPTE Read pin
    HAL_TIM_IC_Stop_IT(&htim9, TIM_CHANNEL_2); // STOP SMPTE
}


void Enable_TC_SMPTE_Read_Pin(void){//Enable SMPTE Read pin
    HAL_TIM_IC_Start_IT(&htim9, TIM_CHANNEL_2); // START SMPTE
}

