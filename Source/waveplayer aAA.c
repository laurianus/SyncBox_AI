
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "waveplayer.h"
#include "SystemTimer.h"
#include "Display.h"
#include "USBHostMain.h"

#include "ILI9341_STM32_Driver.h"
#include "ILI9341_GFX.h"

#ifndef VS_READABLE_CODE
#include "System1.h"
#include "System1Action.h"
#include "System1Data.h"
#endif

#include "simpleEventHandler.h"         		// Event Queue
#include "System1SEMLibB.h"                    	// visualSTATE essentials


extern uint8_t  USBH_USR_ApplicationState;
extern __IO HOST_StateTypeDef USB_stats;

//#define AUDIO_BUFFER_SIZE             65536      // not working - RAM space overflow
//#define AUDIO_BUFFER_SIZE             32768      // not working - RAM space overflow
//#define AUDIO_BUFFER_SIZE             16384      // not working - RAM space overflow
//#define AUDIO_BUFFER_SIZE             8192      // working
//#define AUDIO_BUFFER_SIZE             4096      // working
//#define AUDIO_BUFFER_SIZE             2048      // working
//#define AUDIO_BUFFER_SIZE             1024      // not working
//#define AUDIO_BUFFER_SIZE             512       // not working
//#define AUDIO_BUFFER_SIZE             256       // not working

#define AUDIO_BUFFER_SIZE             8192      // working

//#define AUDIO_BUFFER_SIZE             2048
//#define AUDIO_BUFFER_SIZE             4096      // working

#define CHANGE_TIME_STEP        (10)

uint32_t spped_transfer = 0;

uint8_t was_SeqMusic = 0;

extern uint8_t was_AP_Disp;

UINT bytesread = 0;

extern I2S_HandleTypeDef       hAudioOutI2s;

FRESULT SDXtst;

extern uint8_t is_internal_drive;
extern FSIZE_t file_sizeS;

extern uint32_t device_time;

FIL FileRead_Copy_DestM;

uint32_t display_time_refdf = 0;
uint32_t write_time = 0;
uint32_t read_time = 0;

char path[] = "0:/Audio.wav";

extern uint8_t was_filed_open;

uint8_t music_not_copied = 0;

uint8_t is_MenusDisabled = 0;


extern uint8_t error_SD_card;
uint8_t is_CopyMenu = 0;

extern uint8_t PPS_Mp3_AB; 

extern uint8_t is_Play_Drive;
extern uint8_t was_mount_SD_ok;
extern uint8_t was_mount_USB_ok;


extern uint8_t error_USB_Drive;

extern uint32_t StartUpTime;
extern uint32_t tmp_time_up;
extern uint32_t TimeTillEnd;



uint32_t nowTime_time_here1 = 0;
uint32_t last_time_here1 = 0;
uint32_t countLastTime1 = 0;


#if TEST_DBG
//uint32_t time_passedR[100] = {0}; //for tests only
#endif

extern FATFS USBDISKFatFs;	/* File system object for USB disk logical drive */

extern FATFS SDFatFs;  /* File system object for SD card logical drive */


uint8_t SD_Card_stat = 0;
uint8_t USB_Drive_stat = 0;

uint8_t error_XXCpy = 0;
uint16_t err_read_FDSA = 0;

uint32_t file_sizeX = 0;
uint8_t percent_i_AP = 0;
extern uint32_t sort_percent;
extern uint32_t enter_time_tstX;
extern uint32_t load_time_tstX;    
uint64_t Copy_Bytes = 0;
uint64_t TransferRate_USD = 0;
uint64_t Remain_Copy_time = 0;

uint8_t SD_CARD_OK = 0;

uint8_t file_read_error = 0;
uint16_t file_read_errorX = 0;


uint64_t ap_stat_refresh = 0;
uint8_t last_stat = 10;
uint8_t USB_recconnect = 0;

FSIZE_t current_positionX = 0;
uint8_t is_Audio_PlayerStoped = 0;
// 0 - Stop
// 1 - Pause
// 2 - Play

/* LED State (Toggle or OFF)*/
extern __IO uint32_t LEDsState;

__IO uint32_t PauseResumeStatus;

/* Audio wave data length to be played */
//static uint32_t WaveDataLength = 0;
uint32_t WaveDataLength = 0;

/* Audio wave remaining data length to be played */
//static __IO uint32_t AudioRemSize = 0;
__IO uint32_t AudioRemSize = 0;

/* Ping-Pong buffer used for audio play */
//uint8_t Audio_Buffer_File[AUDIO_BUFFER_SIZE*8];
//uint8_t Audio_Buffer_Samples[AUDIO_BUFFER_SIZE*8];
uint8_t Audio_Buffer_File[AUDIO_BUFFER_SIZE*2];


uint8_t Audio_Buffer_Samples[AUDIO_BUFFER_SIZE*8];

/* Position in the audio play buffer */
__IO BUFFER_StateTypeDef buffer_offset = BUFFER_OFFSET_NONE;

/* Initial Volume level (from 0 (Mute) to 100 (Max)) */
uint8_t Volume = 100;
#define VOLUME_MAX  100

//static FSIZE_t current_position;
FSIZE_t current_position;
WAVE_FormatTypeDef waveformat;

/* Variable used by FatFs*/
FIL FileRead_S, FileRead_D;
DIR Directory;


uint8_t state_wave_player = STATE_WAVE_PLAYER_INIT;

static char wavefilename[_MAX_LFN];

static UINT file_buffer_size;
static UINT samples_buffer_size;

uint8_t debug_enabled = 0;

/* Variable to indicate USB state (start/idle) */
/* Defined in main.c */
extern MSC_ApplicationTypeDef AppliState;

extern uint32_t devicetime_MSx;
//extern uint8_t is_audio_skip;

extern uint8_t is_init_done;

void wave_player_volume_set(uint8_t volume)
{
    if (volume <= VOLUME_MAX)
    {
        Volume = volume;
    }
}

uint16_t get_volume(void)
{
        return Volume;
}


void wave_player_current_position_set(FSIZE_t position)
{
    current_position = position + sizeof(waveformat);
}

void wave_player_current_seconds_set(uint32_t seconds)
{
    uint32_t size_of_each_sample = (waveformat.NbrChannels * waveformat.BitPerSample) / 8;
//    current_position = (waveformat.SampleRate * seconds) * size_of_each_sample;
    current_position = (waveformat.SampleRate * seconds) * size_of_each_sample + sizeof(waveformat);
}

void wave_player_current_miliseconds_set(uint32_t mseconds)
{
    uint32_t size_of_each_sample = (waveformat.NbrChannels * waveformat.BitPerSample) / 8;
//    current_position = (uint32_t)(((uint64_t) waveformat.SampleRate * mseconds * size_of_each_sample) / 1000);
    current_position = (uint32_t)(((uint64_t) waveformat.SampleRate * mseconds * size_of_each_sample) / 1000) + sizeof(waveformat);
}

uint64_t ret_wave_player_size(uint32_t mseconds)
{
    uint64_t current_positionTEST = 0;
    uint32_t size_of_each_sample = (waveformat.NbrChannels * waveformat.BitPerSample) / 8;
//    current_position = (uint32_t)(((uint64_t) waveformat.SampleRate * mseconds * size_of_each_sample) / 1000);
    current_positionTEST = (uint32_t)(((uint64_t) waveformat.SampleRate * mseconds * size_of_each_sample) / 1000) + sizeof(waveformat);
    return current_positionTEST;
}

uint64_t getWaveDataLength(void)
{
        return WaveDataLength;
}

void wave_player_current_miliseconds_get(uint32_t *mseconds)
{
    uint32_t millis;
    float current_time_seconds;
    
    current_time_seconds = (float) current_position / waveformat.ByteRate;

    millis = (uint32_t) (current_time_seconds * 1000);
    *mseconds = millis;
}


void wave_player_remain_miliseconds_get(uint32_t *mseconds)
{
    uint32_t millis;
    float current_time_seconds;
    
    current_time_seconds = (float) AudioRemSize / waveformat.ByteRate;

    millis = (uint32_t) (current_time_seconds * 1000);
    *mseconds = millis;
}


void  wave_player_pause_resume_set(uint32_t state)
{
    PauseResumeStatus = state;
}

void  wave_player_state_set(uint8_t state)
{
    state_wave_player = state;
        
        if(state ==  STATE_WAVE_PLAYER_START)
        {
                is_Audio_PlayerStoped = 2;
                set_AP_Start_time();
        }

}

void  wave_player_debug_set(uint8_t debug)
{
    debug_enabled = debug;
}

/**
  * @brief  Pauses or Resumes a played Wave.
  * @param  state: Player state: Pause, Resume or Idle
  * @retval None
  */
void WavePlayerPauseResume(uint32_t wState)
{
        
    if(wState == PAUSE_STATUS)
    {
        BSP_AUDIO_OUT_Pause();   
        is_Audio_PlayerStoped = 1;
    }
    else
    {
        BSP_AUDIO_OUT_Resume();   
        is_Audio_PlayerStoped = 2;
    }
}

/**
  * @brief  Stops playing Wave.
  * @param  None
  * @retval None
  */
void WavePlayerStop(void)
{ 
    BSP_AUDIO_OUT_Stop();
    is_Audio_PlayerStoped = 0;
}
 
/**
* @brief  Initializes the Wave player.
* @param  AudioFreq: Audio sampling frequency
* @retval None
*/
int WavePlayerInit(uint32_t AudioFreq, uint8_t bits)
{ 
    /* Initialize the Audio codec and all related peripherals (I2S, I2C, IOExpander, IOs...) */  
    return(BSP_AUDIO_OUT_Init(Volume, AudioFreq, bits));  
}

/*--------------------------------
Callbacks implementation:
The callbacks prototypes are defined in the stm32f4_discovery_audio_codec.h file
and their implementation should be done in the user code if they are needed.
Below some examples of callback implementations.
--------------------------------------------------------*/

/**
  * @brief  Manages the DMA Half Transfer complete interrupt.
  * @param  None
  * @retval None
  */
void BSP_AUDIO_OUT_HalfTransfer_CallBack(void)
{ 
    buffer_offset = BUFFER_OFFSET_HALF;
}

/**
* @brief  Calculates the remaining file size and new position of the pointer.
* @param  None
* @retval None
*/
void BSP_AUDIO_OUT_TransferComplete_CallBack(void)
{
    buffer_offset = BUFFER_OFFSET_FULL;
    BSP_AUDIO_OUT_ChangeBuffer((uint16_t*)&Audio_Buffer_Samples[0], samples_buffer_size / 2);
}

/**
* @brief  Manages the DMA FIFO error interrupt.
* @param  None
* @retval None
*/
void BSP_AUDIO_OUT_Error_CallBack(void)
{
    /* Stop the program with an infinite loop */
    Error_Handler();
    /* Could also generate a system reset to recover from the error */
    /* .... */
}

void wave_player_stop(void)
{
//    printf("wave_player_process: STATE_WAVE_PLAYER_PLAY -> exit\r\n");
    LEDsState = LEDS_OFF;
    /* Stop playing Wave */
    WavePlayerPauseResume(PAUSE_STATUS);
    WavePlayerStop();
    /* Close file */
    f_close(&FileRead_S);
    f_closedir(&Directory);
    state_wave_player = STATE_WAVE_PLAYER_IDLE;
}

uint8_t wave_player_dump_wave_header(char *filename)
{
    uint8_t result;



    result = 1;

    switch (state_wave_player)
    {
        case STATE_WAVE_PLAYER_INIT:
        case STATE_WAVE_PLAYER_IDLE:
            /* Get the read out protection status */
            if(f_opendir(&Directory, path) == FR_OK)
            {
                /* Open the Wave file to be played */
                if(f_open(&FileRead_S, filename , FA_READ) != FR_OK)
                {
                    //printf("ERROR: Opening '%s' for read\r\n", filename);
                    result = 0;
                    break;
                }
                else
                {    
                    /* Read sizeof(WaveFormat) from the selected file */
                    current_position = 0;
                    f_lseek(&FileRead_S, current_position);
                    
                    f_read(&FileRead_S, &waveformat, sizeof(waveformat), &bytesread);

                    // calculate no.of samples
                 //   long num_samples = (8 * waveformat.SubChunk2Size) / (waveformat.NbrChannels * waveformat.BitPerSample);
                //    long size_of_each_sample = (waveformat.NbrChannels * waveformat.BitPerSample) / 8;
                    // calculate duration of file
                   // float duration_in_seconds = (float) waveformat.FileSize / waveformat.ByteRate;

                    /*
                    printf("\r\n");
                    printf("------------------------------------------------\r\n");
                    printf("Filename :      '%s'\r\n", filename);

                    printf("ChunkID:        '%c%c%c%c'\r\n", ((char *) &waveformat)[0], ((char *) &waveformat)[1], ((char *) &waveformat)[2], ((char *) &waveformat)[3]);
                    printf("FileSize:       %d\r\n", waveformat.FileSize);
                    printf("FileFormat:     '%c%c%c%c'\r\n", ((char *) &waveformat)[8], ((char *) &waveformat)[9], ((char *) &waveformat)[10], ((char *) &waveformat)[11]);
                    printf("SubChunk1ID:    '%c%c%c%c'\r\n", ((char *) &waveformat)[12], ((char *) &waveformat)[13], ((char *) &waveformat)[14], ((char *) &waveformat)[15]);
                    printf("SubChunk1Size:  %d\r\n", waveformat.SubChunk1Size);
                    printf("AudioFormat:    %X\r\n", waveformat.AudioFormat);
                    printf("NbrChannels:    %X\r\n", waveformat.NbrChannels);
                    printf("SampleRate:     %d\r\n", waveformat.SampleRate);
                    printf("ByteRate:       %d\r\n", waveformat.ByteRate);
                    printf("BlockAlign:     %d\r\n", waveformat.BlockAlign);
                    printf("BitPerSample:   %d\r\n", waveformat.BitPerSample);
                    printf("SubChunk2ID:    '%c%c%c%c'\r\n", ((char *) &waveformat)[36], ((char *) &waveformat)[37], ((char *) &waveformat)[38], ((char *) &waveformat)[39]);
                    printf("SubChunk2Size:  %d\r\n", waveformat.SubChunk2Size);

                    printf("N. of samples:  %lu\r\n", num_samples);
                    printf("Sample size:    %ld bytes\r\n", size_of_each_sample);
                    printf("Duration (sec): %f\r\n", duration_in_seconds);
                    printf("hh:mm:ss.ms :   %s\r\n", seconds_to_time(duration_in_seconds, buffer_time));
                    printf("------------------------------------------------\r\n");
*/

                    f_close(&FileRead_S);
                }    
            }
            break;
        default:
           // printf("ERROR: Can only check wave file if not playing!\r\n");
            break;
    }
    
    return result;
}

void wave_player_change_file(char *filename)
{
   // printf("wave_player_change_file: '%s'\r\n", filename);
    strcpy(wavefilename, filename);
}

void wave_player_init(char* filename)
{
   // printf("wave_player_init: ->STATE_WAVE_PLAYER_IDLE\r\n");
//    state_wave_player = STATE_WAVE_PLAYER_INIT;
    state_wave_player = STATE_WAVE_PLAYER_IDLE;
    strcpy(wavefilename, filename);
}

uint32_t get_bytes_per_pcm_frame(uint16_t bitsPerSample, uint16_t channels, uint16_t blockAlign)
{
    /*
    The bytes per frame is a bit ambiguous. It can be either be based on the bits per sample, or the block align. The way I'm doing it here
    is that if the bits per sample is a multiple of 8, use floor(bitsPerSample*channels/8), otherwise fall back to the block align.
    */
    if ((bitsPerSample & 0x7) == 0) {
        /* Bits per sample is a multiple of 8. */
        return (bitsPerSample * channels) >> 3;
    } else {
        return blockAlign;
    }
}

void dump_buffer(const uint8_t *buffer, size_t size)
{
    for (uint32_t i = 0; i < size; i++)
    {
        if ((i != 0) && ((i % 16) == 0))
        {
            //printf("\r\n");
        }
       // printf("%02X ", buffer[i]);
    }
   // printf("\r\n");
}

uint8_t wave_player_dump_wave_buffer(char *filename)
{
    uint8_t result;

    static UINT bytesread = 0;
    static UINT file_buffer_size;

    result = 1;

    switch (state_wave_player)
    {
        case STATE_WAVE_PLAYER_INIT:
        case STATE_WAVE_PLAYER_IDLE:
            /* Get the read out protection status */
            if(f_opendir(&Directory, path) == FR_OK)
            {
                /* Open the Wave file to be played */
                if(f_open(&FileRead_S, filename , FA_READ) != FR_OK)
                {
                    //printf("ERROR: Opening '%s' for read\r\n", filename);
                    result = 0;
                    break;
                }
                else
                {    
                    /* Read sizeof(WaveFormat) from the selected file */
                    current_position = 0;
                    f_lseek(&FileRead_S, current_position);
                    f_read(&FileRead_S, &waveformat, sizeof(waveformat), &bytesread);

                    // calculate no.of samples
                   // long num_samples = (8 * waveformat.SubChunk2Size) / (waveformat.NbrChannels * waveformat.BitPerSample);
                   // long size_of_each_sample = (waveformat.NbrChannels * waveformat.BitPerSample) / 8;
                    // calculate duration of file
                  //  float duration_in_seconds = (float) waveformat.FileSize / waveformat.ByteRate;

                    
                    /*
                    printf("\r\n");
                    printf("------------------------------------------------\r\n");
                    printf("Filename :      '%s'\r\n", filename);

                    printf("ChunkID:        '%c%c%c%c'\r\n", ((char *) &waveformat)[0], ((char *) &waveformat)[1], ((char *) &waveformat)[2], ((char *) &waveformat)[3]);
                    printf("FileSize:       %d\r\n", waveformat.FileSize);
                    printf("FileFormat:     '%c%c%c%c'\r\n", ((char *) &waveformat)[8], ((char *) &waveformat)[9], ((char *) &waveformat)[10], ((char *) &waveformat)[11]);
                    printf("SubChunk1ID:    '%c%c%c%c'\r\n", ((char *) &waveformat)[12], ((char *) &waveformat)[13], ((char *) &waveformat)[14], ((char *) &waveformat)[15]);
                    printf("SubChunk1Size:  %d\r\n", waveformat.SubChunk1Size);
                    printf("AudioFormat:    %X\r\n", waveformat.AudioFormat);
                    printf("NbrChannels:    %X\r\n", waveformat.NbrChannels);
                    printf("SampleRate:     %d\r\n", waveformat.SampleRate);
                    printf("ByteRate:       %d\r\n", waveformat.ByteRate);
                    printf("BlockAlign:     %d\r\n", waveformat.BlockAlign);
                    printf("BitPerSample:   %d\r\n", waveformat.BitPerSample);
                    printf("SubChunk2ID:    '%c%c%c%c'\r\n", ((char *) &waveformat)[36], ((char *) &waveformat)[37], ((char *) &waveformat)[38], ((char *) &waveformat)[39]);
                    printf("SubChunk2Size:  %d\r\n", waveformat.SubChunk2Size);

                    printf("N. of samples:  %lu\r\n", num_samples);
                    printf("Sample size:    %ld bytes\r\n", size_of_each_sample);
                    printf("Duration (sec): %f\r\n", duration_in_seconds);
                    printf("hh:mm:ss.ms :   %s\r\n", seconds_to_time(duration_in_seconds, buffer_time));
                    printf("------------------------------------------------\r\n");
*/


                    switch (waveformat.BitPerSample)
                    {
                        case 8:
                            file_buffer_size = AUDIO_BUFFER_SIZE/2;
                            samples_buffer_size = AUDIO_BUFFER_SIZE;
                            if (waveformat.NbrChannels == 1)
                            {
                                file_buffer_size = file_buffer_size/2;
                            }
                            break;
                        default:
                        case 16:
                            file_buffer_size = AUDIO_BUFFER_SIZE;
                            samples_buffer_size = AUDIO_BUFFER_SIZE;
                            if (waveformat.NbrChannels == 1)
                            {
                                file_buffer_size = file_buffer_size/2;
                            }
                            break;
                        case 24:
                            file_buffer_size = (AUDIO_BUFFER_SIZE*3)/2;
                            samples_buffer_size = AUDIO_BUFFER_SIZE;
                            if (waveformat.NbrChannels == 1)
                            {
                                file_buffer_size = file_buffer_size/2;
                            }
                            break;
                        case 32:
                            file_buffer_size = AUDIO_BUFFER_SIZE*2;
                            samples_buffer_size = AUDIO_BUFFER_SIZE;
                            if (waveformat.NbrChannels == 1)
                            {
                                file_buffer_size = file_buffer_size/2;
                            }
                            break;
                    }

                   // printf("file_buffer_size:    %d bytes\r\n", file_buffer_size);
                   // printf("samples_buffer_size: %d bytes\r\n", samples_buffer_size);
                    f_read (&FileRead_S, &Audio_Buffer_File[0], file_buffer_size, &bytesread);
                    //printf("f_read: (start) %d/%d bytes\r\n", bytesread, file_buffer_size);
                   
                    switch (waveformat.BitPerSample)
                    {
                        case 8:
                            {
                                int r;
                                size_t i;
                                size_t j;
                                int16_t* pOut;
                                const uint8_t* pIn;
                                size_t sampleCount;

                                pOut = (int16_t *) &Audio_Buffer_Samples;
                                pIn = Audio_Buffer_File;
                                sampleCount = bytesread;
                                if (waveformat.NbrChannels == 1)
                                {
                                    sampleCount = sampleCount * 2;
                                }

                                j = 0;
                                for (i = 0; i < sampleCount;)
                                {
                                    int x = pIn[i];
                                    r = x << 8;
                                    r = r - 32768;
                                    pOut[j] = (short)r;
                                    i++;
                                    j++;
                                    if (waveformat.NbrChannels == 1)
                                    {
                                        pOut[j] = (short)r;
//                                        pOut[j] = 0;
                                        j++;
                                    }
                                }
                            }
                            break;
                        default:
                        case 16:
                            {
//                                int r;
                                size_t i;
                                size_t j;
                                int16_t* pOut;
                                const int16_t* pIn;
                                size_t sampleCount;

                                pOut = (int16_t *) &Audio_Buffer_Samples;
                                pIn = (int16_t *) &Audio_Buffer_File;
                                sampleCount = bytesread;
                                if (waveformat.NbrChannels == 1)
                                {
                                    sampleCount = sampleCount * 2;
                                }

                                j = 0;
                                for (i = 0; i < sampleCount;)
                                {
                                    int x = pIn[i];
                                    pOut[j] = (short)x;
                                    i++;
                                    j++;
                                    if (waveformat.NbrChannels == 1)
                                    {
                                        pOut[j] = (short)x;
//                                        pOut[j] = 0;
                                        j++;
                                    }
                                }
                            }
                            break;
                        case 24:
                            {
                                int r;
                                size_t i;
                                size_t j;
                                int16_t* pOut;
                                const uint8_t* pIn;
                                size_t sampleCount;
                                
                                pOut = (int16_t *) &Audio_Buffer_Samples;
                                pIn = Audio_Buffer_File;
                                sampleCount = bytesread;
                                if (waveformat.NbrChannels == 1)
                                {
                                    sampleCount = sampleCount * 2;
                                }
                                
                                j = 0;
                                for (i = 0; i < sampleCount;)
                                {
                                    int x = ((int)(((unsigned int)(((const uint8_t*)pIn)[i*3+0]) << 8) | ((unsigned int)(((const uint8_t*)pIn)[i*3+1]) << 16) | ((unsigned int)(((const uint8_t*)pIn)[i*3+2])) << 24)) >> 8;
                                    r = x >> 8;
                                    pOut[j] = (short)r;
                                    i++;
                                    j++;
                                    if (waveformat.NbrChannels == 1)
                                    {
                                        pOut[j] = (short)r;
//                                        pOut[j] = 0;
                                        j++;
                                    }
                                }
                            }
                            break;
                        case 32:
                            {
                                int r;
                                size_t i;
                                size_t j;
                                int16_t* pOut;
                                const int32_t* pIn;
                                size_t sampleCount;
                                
                                pOut = (int16_t *) &Audio_Buffer_Samples;
                                pIn = (int32_t *) &Audio_Buffer_File;
                                sampleCount = bytesread;
                                if (waveformat.NbrChannels == 1)
                                {
                                    sampleCount = sampleCount * 2;
                                }

                                j = 0;
                                for (i = 0; i < sampleCount;)
                                {
                                    int x = pIn[i];
                                    r = x >> 16;
                                    pOut[j] = (short)r;
                                    i++;
                                    j++;
                                    if (waveformat.NbrChannels == 1)
                                    {
                                        pOut[j] = (short)r;
                                        j++;
                                    }
                                }
                            }
                            break;
                    }

//                    printf("WavePlayBack, file header: '%c%c%c%c'\r\n", ((char *) &Audio_Buffer)[0], ((char *) &Audio_Buffer)[1], ((char *) &Audio_Buffer)[2], ((char *) &Audio_Buffer)[3]);
                   // printf("File buffer (%d bytes)\r\n", bytesread);
                    dump_buffer((uint8_t*)&Audio_Buffer_File[0], 64);
                   // printf("Audio buffer (%d bytes)\r\n", AUDIO_BUFFER_SIZE);
                    dump_buffer((uint8_t*)&Audio_Buffer_Samples[0], 64);

                    f_close(&FileRead_S);
                }    
            }
            break;
        default:
          //  printf("ERROR: Can only check wave file if not playing!\r\n");
            break;
    }
    
    return result;
}




#if 1 // new wavplayer

uint8_t wave_player_process(void)
{
    uint8_t result;

    static UINT bytesread = 0;
//    uint32_t bytes_per_pcm_frame;

    float   sample_volume;
//    static char    buffer_time[100];
//    static float current_time_seconds;

        
       
        result = 1;
        
        
        
                if(USB_recconnect == 0)
                {
                        if(was_SeqMusic == 0){
                                if (is_Play_Drive == 0 && was_mount_SD_ok == 0)
                                {
                                        f_mount(&SDFatFs, "0:/", 0);
                                        memset(path, 0x00, sizeof(path));
                                        sprintf(path, "0:/Audio.wav");
                                        was_mount_SD_ok = 1;
                                }
                                else if (is_Play_Drive == 1 && was_mount_USB_ok == 0)
                                {
                                        
                                        f_mount(&USBDISKFatFs, "1:/", 0);
                                        memset(path, 0x00, sizeof(path));
                                        sprintf(path, "1:/Audio.wav");
                                        was_mount_USB_ok = 1;
                                }
                        }
                        else{
                                if (is_Play_Drive == 0 && was_mount_SD_ok == 0)
                                {
                                        f_mount(&SDFatFs, "0:/", 0);
                                        memset(path, 0x00, sizeof(path));
                                        sprintf(path, "0:/Audio%d.wav",was_SeqMusic);
                                        was_mount_SD_ok = 1;
                                }
                                else if (is_Play_Drive == 1 && was_mount_USB_ok == 0)
                                {
                                        
                                        f_mount(&USBDISKFatFs, "1:/", 0);
                                        memset(path, 0x00, sizeof(path));
                                        sprintf(path, "1:/Audio%d.wav", was_SeqMusic);
                                        was_mount_USB_ok = 1;
                                }
                        }
                }

    switch (state_wave_player)
    {
        case STATE_WAVE_PLAYER_INIT:
         //   printf("wave_player_process: STATE_WAVE_PLAYER_INIT\r\n");
            state_wave_player = STATE_WAVE_PLAYER_IDLE;
            break;
        case STATE_WAVE_PLAYER_IDLE:
             
        if(USB_recconnect == 0 && file_read_error == 0)
        {
                SDXtst = f_open(&FileRead_S, path , FA_READ);
                
                
                if(SDXtst != FR_OK)
                {
                    //printf("ERROR: Opening '%s' for read\r\n", wavefilename);
                    file_read_error = 1;    
                }
                else
                {
                    USB_recconnect = 1;
                    f_close(&FileRead_S);
                }
        }
       
        
                /*
                            if (AudioRemSize == 0)
                            { 
                               // printf("wave_player_process: STATE_WAVE_PLAYER_PLAY -> exit\r\n");

                                AudioRemSize = 0;
                                LEDsState = LEDS_OFF;
                                // Stop playing Wave 
                                WavePlayerPauseResume(PAUSE_STATUS);
                                WavePlayerStop();
                                // Close file
                                f_close(&FileRead);
                //                state_wave_player = STATE_WAVE_PLAYER_START;
                                state_wave_player = STATE_WAVE_PLAYER_IDLE;
                                break;
                            }
    */
//            printf("wave_player_process: STATE_WAVE_PLAYER_IDLE\r\n");
//            state_wave_player = STATE_WAVE_PLAYER_START;
            break;
        case STATE_WAVE_PLAYER_START:
           // printf("wave_player_process: STATE_WAVE_PLAYER_START\r\n");
            /* Get the read out protection status */
          //  if(f_opendir(&Directory, path) == FR_OK)
          if(1)
            {
                    /*
                if (is_Play_Drive == 1 && is_CopyMenu == 1)
                {
                        copy_toSD();//OK
                }
                    */
                /* Open the Wave file to be played */
                if(f_open(&FileRead_S, path , FA_READ) != FR_OK)
                {
                    //printf("ERROR: Opening '%s' for read\r\n", wavefilename);
                    file_read_error = 1;    
                    BSP_LED_On(LED5);
                    result = 0;
                    state_wave_player = STATE_WAVE_PLAYER_IDLE;
                    break;
                }
                else
                {    
                    file_read_error = 0;  
                    /* Read sizeof(WaveFormat) from the selected file */
                    current_position = 0;
                    f_lseek(&FileRead_S, current_position);
                    
                    f_read(&FileRead_S, &waveformat, sizeof(waveformat), &bytesread);
                    current_position += bytesread;

                    // calculate no.of samples
//                    long num_samples = (8 * waveformat.SubChunk2Size) / (waveformat.NbrChannels * waveformat.BitPerSample);
                 //   long size_of_each_sample = (waveformat.NbrChannels * waveformat.BitPerSample) / 8;
                    // calculate duration of file
                   // float duration_in_seconds = (float) waveformat.FileSize / waveformat.ByteRate;

//                    bytes_per_pcm_frame = get_bytes_per_pcm_frame(waveformat.BitPerSample, waveformat.NbrChannels, waveformat.BlockAlign);

                    

                    /* Set WaveDataLenght to the Speech Wave length */
//                    WaveDataLength = waveformat.SubChunk2Size;
                    WaveDataLength = waveformat.FileSize;

                    /* Start playing */
                    /* Initialize Wave player (Codec, DMA, I2C) */
                    if(WavePlayerInit(waveformat.SampleRate, 16) != 0)
                    {
                        Error_Handler();
                    }

                    BSP_LED_Off(LED5);

                    switch (waveformat.BitPerSample)
                    {
                        case 8:
                            file_buffer_size = AUDIO_BUFFER_SIZE/2;
                            samples_buffer_size = AUDIO_BUFFER_SIZE;
                            if (waveformat.NbrChannels == 1)
                            {
                                file_buffer_size = file_buffer_size/2;
                            }
                            break;
                        case 16:
                            file_buffer_size = AUDIO_BUFFER_SIZE;
                            samples_buffer_size = AUDIO_BUFFER_SIZE;
                            if (waveformat.NbrChannels == 1)
                            {
                                file_buffer_size = file_buffer_size/2;
                            }
                            break;
                        case 24:
                            file_buffer_size = (AUDIO_BUFFER_SIZE*3)/2;
                            samples_buffer_size = AUDIO_BUFFER_SIZE;
                            if (waveformat.NbrChannels == 1)
                            {
                                file_buffer_size = file_buffer_size/2;
                            }
                            break;
                        case 32:
                            file_buffer_size = AUDIO_BUFFER_SIZE*2;
                            samples_buffer_size = AUDIO_BUFFER_SIZE;
                            if (waveformat.NbrChannels == 1)
                            {
                                file_buffer_size = file_buffer_size/2;
                            }
                            break;
                        default:
                           // printf("ERROR: Unsupported bits per sample (%d)\r\n", waveformat.BitPerSample);
                            BSP_LED_On(LED5);
                            WavePlayerPauseResume(PAUSE_STATUS);
                            WavePlayerStop();
                            /* Close file */
                            f_close(&FileRead_S);
                            f_closedir(&Directory);
                            state_wave_player = STATE_WAVE_PLAYER_IDLE;
                            result = 0;
                            return result;
                    }

                  //  printf("File buffer size:   %d bytes\r\n", file_buffer_size);
                  //  printf("Sample buffer size: %d bytes\r\n", samples_buffer_size);

                    /* Get Data from USB Flash Disk */
                    f_read (&FileRead_S, &Audio_Buffer_File[0], file_buffer_size, &bytesread);
                  //  printf("f_read: (start) %d/%d bytes\r\n", bytesread, file_buffer_size);

                    AudioRemSize = WaveDataLength - current_position;
//                    printf("AudioRemSize: %d bytes\r\n", AudioRemSize);
                   
                               AudioRemSize = WaveDataLength - current_position;
                    
                    
                            /* Check if the device is connected.*/
                            if ((AudioRemSize < (2 * AUDIO_BUFFER_SIZE)) || (AppliState == APPLICATION_IDLE))
                            { 
                               // printf("wave_player_process: STATE_WAVE_PLAYER_PLAY -> exit\r\n");

                                AudioRemSize = 0;
                                LEDsState = LEDS_OFF;
                                /* Stop playing Wave */
                                WavePlayerPauseResume(PAUSE_STATUS);
                                WavePlayerStop();
                                /* Close file */
                                f_close(&FileRead_S);                                   
                //                state_wave_player = STATE_WAVE_PLAYER_START;
                                state_wave_player = STATE_WAVE_PLAYER_IDLE;
                                break;
                            }
                    
                    current_position += bytesread;

                    sample_volume = ((float) Volume)/VOLUME_MAX;

                    switch (waveformat.BitPerSample)
                    {
                        case 8:
                            {
                                int r;
                                size_t i;
                                size_t j;
                                int16_t* pOut;
                                const uint8_t* pIn;
                                size_t sampleCount;

                                pOut = (int16_t *) &Audio_Buffer_Samples;
                                pIn = Audio_Buffer_File;
                                sampleCount = bytesread;
                                if (waveformat.NbrChannels == 1)
                                {
                                    sampleCount = sampleCount * 2;
                                }

                                j = 0;
                                for (i = 0; i < sampleCount;)
                                {
                                    int x = pIn[i];
                                    r = x << 8;
                                    r = r - 32768;
                                    pOut[j] = (short)r;
                                    pOut[j] = pOut[j] * sample_volume;
                                    i++;
                                    j++;
                                    if (waveformat.NbrChannels == 1)
                                    {
                                        pOut[j] = (short)r;
                                        pOut[j] = pOut[j] * sample_volume;
//                                        pOut[j] = 0;
                                        j++;
                                    }
                                }
                            }
                            break;
                        default:
                        case 16:
                            {
//                                int r;
                                size_t i;
                                size_t j;
                                int16_t* pOut;
                                const int16_t* pIn;
                                size_t sampleCount;

                                pOut = (int16_t *) &Audio_Buffer_Samples;
                                pIn = (int16_t *) &Audio_Buffer_File;
                                sampleCount = bytesread;
                                if (waveformat.NbrChannels == 1)
                                {
                                    sampleCount = sampleCount * 2;
                                }

                                j = 0;
                                for (i = 0; i < sampleCount;)
                                {
                                    int x = pIn[i];
                                    pOut[j] = (short)x;
                                    pOut[j] = pOut[j] * sample_volume;
                                    i++;
                                    j++;
                                    if (waveformat.NbrChannels == 1)
                                    {
                                        pOut[j] = (short)x;
                                        pOut[j] = pOut[j] * sample_volume;
//                                        pOut[j] = 0;
                                        j++;
                                    }
                                }
                            }
                            break;
                        case 24:
                            {
                                int r;
                                size_t i;
                                size_t j;
                                int16_t* pOut;
                                const uint8_t* pIn;
                                size_t sampleCount;
                                
                                pOut = (int16_t *) &Audio_Buffer_Samples;
                                pIn = Audio_Buffer_File;
                                sampleCount = bytesread;
                                if (waveformat.NbrChannels == 1)
                                {
                                    sampleCount = sampleCount * 2;
                                }
                                
                                j = 0;
                                for (i = 0; i < sampleCount;)
                                {
                                    int x = ((int)(((unsigned int)(((const uint8_t*)pIn)[i*3+0]) << 8) | ((unsigned int)(((const uint8_t*)pIn)[i*3+1]) << 16) | ((unsigned int)(((const uint8_t*)pIn)[i*3+2])) << 24)) >> 8;
                                    r = x >> 8;
                                    pOut[j] = (short)r;
                                    pOut[j] = pOut[j] * sample_volume;
                                    i++;
                                    j++;
                                    if (waveformat.NbrChannels == 1)
                                    {
                                        pOut[j] = (short)r;
                                        pOut[j] = pOut[j] * sample_volume;
//                                        pOut[j] = 0;
                                        j++;
                                    }
                                }
                            }
                            break;
                        case 32:
                            {
                                int r;
                                size_t i;
                                size_t j;
                                int16_t* pOut;
                                const int32_t* pIn;
                                size_t sampleCount;
                                
                                pOut = (int16_t *) &Audio_Buffer_Samples;
                                pIn = (int32_t *) &Audio_Buffer_File;
                                sampleCount = bytesread;
                                if (waveformat.NbrChannels == 1)
                                {
                                    sampleCount = sampleCount * 2;
                                }

                                j = 0;
                                for (i = 0; i < sampleCount;)
                                {
                                    int x = pIn[i];
                                    r = x >> 16;
                                    pOut[j] = (short)r;
                                    pOut[j] = pOut[j] * sample_volume;
                                    i++;
                                    j++;
                                    if (waveformat.NbrChannels == 1)
                                    {
                                        pOut[j] = (short)r;
                                        pOut[j] = pOut[j] * sample_volume;
                                        j++;
                                    }
                                }
                            }
                            break;
                    }
#if 0
//                    printf("WavePlayBack, file header: '%c%c%c%c'\r\n", ((char *) &Audio_Buffer)[0], ((char *) &Audio_Buffer)[1], ((char *) &Audio_Buffer)[2], ((char *) &Audio_Buffer)[3]);
                    printf("File buffer (%d bytes)\r\n", bytesread);
                    dump_buffer((uint8_t*)&Audio_Buffer_File[0], 64);
                    printf("Audio buffer (%d bytes)\r\n", samples_buffer_size);
                    dump_buffer((uint8_t*)&Audio_Buffer_Samples[0], 64);
#endif
                    /* Start playing Wave */
                    BSP_AUDIO_OUT_Play((uint16_t*)&Audio_Buffer_Samples[0], samples_buffer_size);
//                    printf("BSP_AUDIO_OUT_Play, size: %d\r\n", samples_buffer_size);
                    LEDsState = LED6_TOGGLE;
                    PauseResumeStatus = RESUME_STATUS;
                    state_wave_player = STATE_WAVE_PLAYER_PLAY;
                    buffer_offset = BUFFER_OFFSET_NONE;
                }    
            }
            else
            {
                printf("ERROR: Opening root directory\r\n");
                BSP_LED_On(LED5);
                result = 0;
                state_wave_player = STATE_WAVE_PLAYER_IDLE;
                break;
            }
            break;
        case STATE_WAVE_PLAYER_PLAY:

           AudioRemSize = WaveDataLength - current_position;
            /* Check if the device is connected.*/
            if ((AudioRemSize < (2 * AUDIO_BUFFER_SIZE)) || (AppliState == APPLICATION_IDLE))
            { 
               // printf("wave_player_process: STATE_WAVE_PLAYER_PLAY -> exit\r\n");

                AudioRemSize = 0;
                LEDsState = LEDS_OFF;
                /* Stop playing Wave */
                WavePlayerPauseResume(PAUSE_STATUS);
                WavePlayerStop();
                /* Close file */
                f_close(&FileRead_S);
//                state_wave_player = STATE_WAVE_PLAYER_START;
                state_wave_player = STATE_WAVE_PLAYER_IDLE;
                break;
            }

            if(PauseResumeStatus == PAUSE_STATUS)
            {
                /* Stop Toggling LED2 to signal Pause */
              //  LEDsState = STOP_TOGGLE;
                /* Pause playing Wave */
                WavePlayerPauseResume(PauseResumeStatus);
                PauseResumeStatus = IDLE_STATUS;
            }
            else if(PauseResumeStatus == RESUME_STATUS)
            {
                /* Toggling LED6 to signal Play */
            //    LEDsState = LED6_TOGGLE;
                /* Resume playing Wave */
                WavePlayerPauseResume(PauseResumeStatus);
                PauseResumeStatus = IDLE_STATUS;

            }  

            bytesread = 0;

            if(buffer_offset == BUFFER_OFFSET_HALF)
            {
                    
                buffer_offset = BUFFER_OFFSET_NONE;

                sample_volume = ((float) Volume)/VOLUME_MAX;

            //    current_time_seconds = (float) current_position / waveformat.ByteRate;

                last_time_here1 = GetCurrentSystemTime();
                    
                f_lseek(&FileRead_S, current_position);
                f_read (&FileRead_S, &Audio_Buffer_File[0], file_buffer_size/2, &bytesread);

               

                nowTime_time_here1 =  GetCurrentSystemTime() - last_time_here1;
                    
                  
                if(nowTime_time_here1 > 10)
                    {
                            if(nowTime_time_here1 > 1000){
                                countLastTime1 = countLastTime1 + 1000;
                            }
                            else if(nowTime_time_here1 > 40)
                            {
                                countLastTime1 = countLastTime1+ 100;
                            }
                            else
                            {
                                countLastTime1++;
                            }
                            
                            /*
                            if(countLastTime1 < 100)
                            {
                                time_passedR[countLastTime1] = nowTime_time_here1;
                            }
                            */
                    }
   
                    

                current_position += bytesread;

                if (debug_enabled)
                {
                    // for 96 kHz sample rate, debug info has heavy impact on processing -> no debug messages
                        /*
                    if (waveformat.SampleRate < 60000)
                    {
                        printf("f_read: (half) %d/%d bytes, %s\r\n", bytesread, file_buffer_size/2, seconds_to_time(current_time_seconds, buffer_time));
                    }
                        */
                }
                if (bytesread < (file_buffer_size/2))
                {
                    AudioRemSize = 0;
                    printf("f_read: (half) EOF\r\n");
                }

                switch (waveformat.BitPerSample)
                {
                    case 8:
                        {
                            int r;
                            size_t i;
                            size_t j;
                            int16_t* pOut;
                            const uint8_t* pIn;
                            size_t sampleCount;

                            pOut = (int16_t *) &Audio_Buffer_Samples;
                            pIn = Audio_Buffer_File;
                            sampleCount = bytesread;
                            if (waveformat.NbrChannels == 2)
                            {
                                sampleCount = sampleCount * 2;
                            }

                            j = 0;
                            for (i = 0; i < sampleCount;)
                            {
                                int x = pIn[i];
                                r = x << 8;
                                r = r - 32768;
                                pOut[j] = (short)r;
                                pOut[j] = pOut[j] * sample_volume;
                                i++;
                                j++;
                                if (waveformat.NbrChannels == 1)
                                {
                                    pOut[j] = (short)r;
                                    pOut[j] = pOut[j] * sample_volume;
//                                    pOut[j] = 0;
                                    j++;
                                }
                            }
                        }
                        break;
                    default:
                    case 16:
                        {
//                            int r;
                            size_t i;
                            size_t j;
                            int16_t* pOut;
                            const int16_t* pIn;
                            size_t sampleCount;

                            pOut = (int16_t *) &Audio_Buffer_Samples;
                            pIn = (int16_t *) &Audio_Buffer_File;
                            sampleCount = bytesread;
                            if (waveformat.NbrChannels == 1)
                            {
                                sampleCount = sampleCount * 2;
                            }

                            j = 0;
                            for (i = 0; i < sampleCount;)
                            {
                                int x = pIn[i];
                                pOut[j] = (short)x;
                                pOut[j] = pOut[j] * sample_volume;
                                i++;
                                j++;
                                if (waveformat.NbrChannels == 1)
                                {
                                    pOut[j] = (short)x;
                                    pOut[j] = pOut[j] * sample_volume;
//                                    pOut[j] = 0;
                                    j++;
                                }
                            }
                        }
                        break;
                    case 24:
                        {
                            int r;
                            size_t i;
                            size_t j;
                            int16_t* pOut;
                            const uint8_t* pIn;
                            size_t sampleCount;
                            
                            pOut = (int16_t *) &Audio_Buffer_Samples;
                            pIn = Audio_Buffer_File;
                            sampleCount = bytesread;
                            if (waveformat.NbrChannels == 1)
                            {
                                sampleCount = sampleCount * 2;
                            }
                            
                            j = 0;
                            for (i = 0; i < sampleCount;)
                            {
                                int x = ((int)(((unsigned int)(((const uint8_t*)pIn)[i*3+0]) << 8) | ((unsigned int)(((const uint8_t*)pIn)[i*3+1]) << 16) | ((unsigned int)(((const uint8_t*)pIn)[i*3+2])) << 24)) >> 8;
                                r = x >> 8;
                                pOut[j] = (short)r;
                                pOut[j] = pOut[j] * sample_volume;
                                i++;
                                j++;
                                if (waveformat.NbrChannels == 1)
                                {
                                    pOut[j] = (short)r;
                                    pOut[j] = pOut[j] * sample_volume;
//                                    pOut[j] = 0;
                                    j++;
                                }
                            }
                        }
                        break;
                    case 32:
                        {
                            int r;
                            size_t i;
                            size_t j;
                            int16_t* pOut;
                            const int32_t* pIn;
                            size_t sampleCount;
                            
                            pOut = (int16_t *) &Audio_Buffer_Samples;
                            pIn = (int32_t *) &Audio_Buffer_File;
                            sampleCount = bytesread;
                            if (waveformat.NbrChannels == 1)
                            {
                                sampleCount = sampleCount * 2;
                            }

                            j = 0;
                            for (i = 0; i < sampleCount;)
                            {
                                int x = pIn[i];
                                r = x >> 16;
                                pOut[j] = (short)r;
                                pOut[j] = pOut[j] * sample_volume;
                                i++;
                                j++;
                                if (waveformat.NbrChannels == 1)
                                {
                                    pOut[j] = (short)r;
                                    pOut[j] = pOut[j] * sample_volume;
                                    j++;
                                }
                            }
                        }
                        break;
                }
            }

            if(buffer_offset == BUFFER_OFFSET_FULL)
            {
                buffer_offset = BUFFER_OFFSET_NONE;
                    
                    
                    /*
                                if(is_audio_skip == 1)
                                {
                                        if(was_SeqMusic == 0){
                                                wave_player_current_miliseconds_set(device_time + 5);
                                        }
                                        
                                        if(WaveDataLength < current_position)
                                        {
                                                current_position = 0;
                                               // WavePlayerStop();
                                        }
                                                
                                        is_audio_skip = 0;
                                }
                    */

                sample_volume = ((float) Volume)/VOLUME_MAX;

//                current_time_seconds = (float) current_position / waveformat.ByteRate;

                f_lseek(&FileRead_S, current_position);
                f_read (&FileRead_S, &Audio_Buffer_File[file_buffer_size/2], file_buffer_size/2, &bytesread);

                current_position += bytesread;

                if (debug_enabled)
                {
                    // for 96 kHz sample rate, debug info has heavy impact on processing -> no debug messages
                        /*
                    if (waveformat.SampleRate < 60000)
                    {
                        printf("f_read: (full) %d/%d bytes, %s\r\n", bytesread, file_buffer_size/2, seconds_to_time(current_time_seconds, buffer_time));
                    }
                        */
                }

                if (bytesread < (file_buffer_size/2))
                {
                    AudioRemSize = 0;
                    printf("f_read: (full) EOF\r\n");
                }
                
                switch (waveformat.BitPerSample)
                {
                    case 8:
                        {
                            int r;
                            size_t i;
                            size_t j;
                            int16_t* pOut;
                            const uint8_t* pIn;
                            size_t sampleCount;

                            pOut = (int16_t *) &Audio_Buffer_Samples[samples_buffer_size/2];
                            pIn = &Audio_Buffer_File[file_buffer_size/2];
                            sampleCount = bytesread;
                            if (waveformat.NbrChannels == 2)
                            {
                                sampleCount = sampleCount * 2;
                            }

                            j = 0;
                            for (i = 0; i < sampleCount;)
                            {
                                int x = pIn[i];
                                r = x << 8;
                                r = r - 32768;
                                pOut[j] = (short)r;
                                pOut[j] = pOut[j] * sample_volume;
                                i++;
                                j++;
                                if (waveformat.NbrChannels == 1)
                                {
                                    pOut[j] = (short)r;
                                    pOut[j] = pOut[j] * sample_volume;
//                                    pOut[j] = 0;
                                    j++;
                                }
                            }
                        }
                        break;
                    default:
                    case 16:
                        {
//                                int r;
                            size_t i;
                            size_t j;
                            int16_t* pOut;
                            const int16_t* pIn;
                            size_t sampleCount;

                            pOut = (int16_t *) &Audio_Buffer_Samples[samples_buffer_size/2];
                            pIn = (int16_t *) &Audio_Buffer_File[file_buffer_size/2];
                            sampleCount = bytesread;
                            if (waveformat.NbrChannels == 1)
                            {
                                sampleCount = sampleCount * 2;
                            }

                            j = 0;
                            for (i = 0; i < sampleCount;)
                            {
                                int x = pIn[i];
                                pOut[j] = (short)x;
                                pOut[j] = pOut[j] * sample_volume;
                                i++;
                                j++;
                                if (waveformat.NbrChannels == 1)
                                {
                                    pOut[j] = (short)x;
                                    pOut[j] = pOut[j] * sample_volume;
//                                        pOut[j] = 0;
                                    j++;
                                }
                            }
                        }
                        break;
                    case 24:
                        {
                            int r;
                            size_t i;
                            size_t j;
                            int16_t* pOut;
                            const uint8_t* pIn;
                            size_t sampleCount;
                            
                            pOut = (int16_t *) &Audio_Buffer_Samples[samples_buffer_size/2];
                            pIn = (uint8_t *) &Audio_Buffer_File[file_buffer_size/2];
                            sampleCount = bytesread;
                            if (waveformat.NbrChannels == 1)
                            {
                                sampleCount = sampleCount * 2;
                            }
                            
                            j = 0;
                            for (i = 0; i < sampleCount;)
                            {
                                int x = ((int)(((unsigned int)(((const uint8_t*)pIn)[i*3+0]) << 8) | ((unsigned int)(((const uint8_t*)pIn)[i*3+1]) << 16) | ((unsigned int)(((const uint8_t*)pIn)[i*3+2])) << 24)) >> 8;
                                r = x >> 8;
                                pOut[j] = (short)r;
                                pOut[j] = pOut[j] * sample_volume;
                                i++;
                                j++;
                                if (waveformat.NbrChannels == 1)
                                {
                                    pOut[j] = (short)r;
                                    pOut[j] = pOut[j] * sample_volume;
//                                        pOut[j] = 0;
                                    j++;
                                }
                            }
                        }
                        break;
                    case 32:
                        {
                            int r;
                            size_t i;
                            size_t j;
                            int16_t* pOut;
                            const int32_t* pIn;
                            size_t sampleCount;
                            
                            pOut = (int16_t *) &Audio_Buffer_Samples[samples_buffer_size/2];
                            pIn = (int32_t *) &Audio_Buffer_File[file_buffer_size/2];
                            sampleCount = bytesread;
                            if (waveformat.NbrChannels == 1)
                            {
                                sampleCount = sampleCount * 2;
                            }

                            j = 0;
                            for (i = 0; i < sampleCount;)
                            {
                                int x = pIn[i];
                                r = x >> 16;
                                pOut[j] = (short)r;
                                pOut[j] = pOut[j] * sample_volume;
                                i++;
                                j++;
                                if (waveformat.NbrChannels == 1)
                                {
                                    pOut[j] = (short)r;
                                    pOut[j] = pOut[j] * sample_volume;
                                    j++;
                                }
                            }
                        }
                        break;
                }
            } 
            break;
    }
    
    return result;
}

#endif



/**
 * Convert seconds into hh:mm:ss format
 * Params:
 *	seconds - seconds value
 * Returns: hms - formatted string
 **/
char* seconds_to_time(float raw_seconds, char *hms)
{
    int hours, hours_residue, minutes, seconds, milliseconds;

    sprintf(hms, "%f", raw_seconds);

    hours = (int) raw_seconds/3600;
    hours_residue = (int) raw_seconds % 3600;
    minutes = hours_residue/60;
    seconds = hours_residue % 60;
    milliseconds = 0;

    // get the decimal part of raw_seconds to get milliseconds
    char *pos;
    pos = strchr(hms, '.');
    int ipos = (int) (pos - hms);
    char decimalpart[15];
    memset(decimalpart, ' ', sizeof(decimalpart));
    strncpy(decimalpart, &hms[ipos+1], 3);
    milliseconds = atoi(decimalpart);	

    sprintf(hms, "%02d:%02d:%02d.%03d", hours, minutes, seconds, milliseconds);
    return hms;
}
 
void WavePlayer_CallBack(void)
{
    if(AppliState != APPLICATION_IDLE)
    {
        /* Reset the Wave player variables */
        LEDsState = LEDS_OFF;
        PauseResumeStatus = RESUME_STATUS;
        WaveDataLength = 0;

        /* Stop the Codec */
        if(BSP_AUDIO_OUT_Stop() != AUDIO_OK)
        {
            Error_Handler();
        }

        /* Turn OFF LED3, LED4 and LED6 */
        BSP_LED_Off(LED3);
        BSP_LED_Off(LED4);
        BSP_LED_Off(LED6);
    }
} 


void set_curent_pos(void)
{
        current_position = 0;
}


void AP_Switch_ID(void)
{
             if(is_Play_Drive == 1)
             {
                        set_curent_pos();
//                        f_mount(0, "1:/", 0); 
                        AppliState = APPLICATION_START;
                        USBH_USR_ApplicationState = USBH_USR_FS_INIT;
                        state_wave_player = STATE_WAVE_PLAYER_INIT;
                        is_Play_Drive = 0; 
                        was_mount_USB_ok = 0;
                        was_mount_SD_ok = 0;
                     
                        if(get_is_internal_drive() == 0){
                                was_filed_open = 0;
                        }
     }
}

void AP_Switch_UD(void)
{
             if(is_Play_Drive == 0)
             {
                        set_curent_pos();
//                        f_mount(0, "0:/", 0); 
                        AppliState = APPLICATION_START;
                        USBH_USR_ApplicationState = USBH_USR_FS_INIT;
                        state_wave_player = STATE_WAVE_PLAYER_INIT;
                        is_Play_Drive = 1; 
                        was_mount_USB_ok = 0;
                        is_MenusDisabled = 0;
                     
                        if(get_is_internal_drive() == 0){
                                was_filed_open = 0;
                        }
             }
}


uint8_t get_play_drive(void)
{
        return is_Play_Drive;
}

uint8_t need_toSwitch(void)
{
        if(PPS_Mp3_AB != 1 && device_Status < Status_PowerEnable && (is_init_done == 0 || is_init_done == 10  || is_init_done == 100) && is_CopyMenu < 3 && is_internal_drive == 0)
        {
                return 1;
        }
        else
        {
                return 0;
        }

}

uint8_t read_mp3_status(void)
{
      
        if(USB_stats == HOST_CLASS && is_Play_Drive == 1)
        {
             return 0;
        }
        else if(is_Play_Drive == 0)
        {
            return 1;
        }
        else
        {
                        is_init_done = 0;
                        current_position = 0;
                        return 3;
        }        
        
#if 0        
        if(USB_stats == HOST_CLASS && is_Play_Drive == 1)
        {
             return 0;
        }
        else if(is_Play_Drive == 0)
        {
            return 1;
        }
        else
        {
                if(is_Play_Drive == 1 && SD_Card_stat > 0)
                {
                      USBH_USR_ApplicationState = USBH_USR_IDLE;
                      state_wave_player = STATE_WAVE_PLAYER_INIT;  

                      //USBH_USR_ApplicationState = USBH_USR_FS_INIT;
                     // AppliState = APPLICATION_START;
                        
                       is_Play_Drive = 0;
                       was_mount_USB_ok = 0;
                       USB_recconnect = 0;
                       return 1;
                }

                        is_init_done = 0;
                        current_position = 0;
                        return 3;
        }
#endif
}

uint8_t read_mp3_Pstatus(void)
{
        //1 - Play
        //2 - Stop
        //3 - Pause
    
        
    if(GetCurrentSystemTime() - ap_stat_refresh > 100)    
    {
            ap_stat_refresh = GetCurrentSystemTime();    
            
            if(file_read_error == 1)
            {
                    last_stat = 10;
                    //return 10; //Audio player init - file not load it
            }
            //else if(current_position < 1000)
            else if(is_Audio_PlayerStoped == 0 || current_position < 1000)
            {
                    //current_positionX = current_position;
                    last_stat = 2;
                   // return 2;//Stop
            }
            else if(current_position == current_positionX)
            {
                    //current_positionX = current_position;
                    last_stat = 3;
                   // return 3;//Pause
            }
            else if(current_position != current_positionX)
            {
                    //current_positionX = current_position;
                    last_stat = 1;
                    //return 1;//Play
            }
            else
            {
                    last_stat = 10; 
                   // return 10;
            }
            
            current_positionX = current_position;
    }

    
return last_stat;        
     
  
#if 0
            if(file_read_error == 1)
            {
                    if(last_stat != 10 && ap_stat_refresh < CHANGE_TIME_STEP)
                    {
                            ap_stat_refresh++;
                            return last_stat;
                    }
                    else
                    {
                            ap_stat_refresh = 0;
                            last_stat = 10;
                            return 10; //Audio player init - file not load it
                    }
                    
                    
            }
            //else if(current_position < 1000)
            else if(is_Audio_PlayerStoped == 0 || current_position < 1000)
            {
                    if(last_stat != 2 && ap_stat_refresh < CHANGE_TIME_STEP)
                    {
                            ap_stat_refresh++;
                            return last_stat;
                    }
                    else
                    {
                            ap_stat_refresh = 0;
                            current_positionX = current_position;
                            last_stat = 2;
                            return 2;//Stop
                    }
                    
            }
            else if(current_position - current_positionX == 0)
            {
                    if(last_stat != 3 && ap_stat_refresh < CHANGE_TIME_STEP)
                    {
                            ap_stat_refresh++;
                            return last_stat;
                    }
                    else
                    {
                            ap_stat_refresh = 0;
                            current_positionX = current_position;
                            last_stat = 3;
                            return 3;//Pause
                    }
            }
            else if(current_position - current_positionX > 0)
            {
                    if(last_stat != 1 && ap_stat_refresh < CHANGE_TIME_STEP)
                    {
                            ap_stat_refresh++;
                            return last_stat;
                    }
                    else
                    {
                            ap_stat_refresh = 0;
                            current_positionX = current_position;
                            last_stat = 1;
                            return 1;//Play
                    }
            }
            else
            {
                    last_stat = 10; 
                    return 10;
            }
    #endif

}



/* Copy a file "file.bin" on the drive 1 to drive 0 */

void copy_toSD (void)
{

    FRESULT fr;          /* FatFs function common result code */
    UINT br, bw;         /* File read/write count */

       
        
     enter_time_tstX = GetCurrentSystemTime();     
        
     //   is_CopyMenu = 2;            

      
    /* Give work areas to each logical drive */
    if(f_mount(&SDFatFs, "0:/", 0) != FR_OK )
    {
        error_XXCpy = 3;
    }
    
    
    #if 0
    if(f_mount(&USBDISKFatFs, "1:/", 0) != FR_OK )
    {
        error_XXCpy = 4;
    }

#endif
    /* Open source file on the drive 1 */
    fr = f_open(&FileRead_S, "1:/Audio.wav", FA_READ);
    if (fr) 
    {
        error_XXCpy = 1;
    }
     else
     {
        file_sizeX = f_size(&FileRead_S);
     }
   
    
 //   file_sizeX = f_size(&FileRead_S);
    
    /* Create destination file on the drive 0 */
  
    fr = f_open(&FileRead_D, "0:/Audio.wav", FA_WRITE | FA_CREATE_ALWAYS);
    if (fr) 
    {
            error_XXCpy = 2;
    }

     error_XXCpy = 0;
    /* Copy source to destination */
    for (;;) {
            error_XXCpy = 0;
        fr = f_read(&FileRead_S, Audio_Buffer_Samples, sizeof Audio_Buffer_Samples, &br); /* Read a chunk of data from the source file */
        if (br == 0 || fr != 0) break; /* error or eof */
        


        fr = f_write(&FileRead_D, Audio_Buffer_Samples, br, &bw);           /* Write it to the destination file */
        if (bw < br) break; /* error or disk full */
            
            Copy_Bytes =  Copy_Bytes + sizeof (Audio_Buffer_Samples);
            
                                        if(percent_i_AP == 10)   
                                        {
                                                percent_i_AP = 0;
                                                sort_percent = (100 * Copy_Bytes)/file_sizeX;
                                                load_time_tstX = GetCurrentSystemTime() - enter_time_tstX;
                                                TransferRate_USD = (Copy_Bytes/1000)/(load_time_tstX /1000);
                                                Remain_Copy_time = (file_sizeX - Copy_Bytes)/(1000 * TransferRate_USD);
                                                DoDisplayTasks();
                                        }
                                        else
                                        {
                                                DoDisplayTasks();
                                                percent_i_AP++;
                                        }
    }

    /* Close open files */

    f_close(&FileRead_S);
    f_close(&FileRead_D);

    /* Unregister work area prior to discard it */
    f_mount(0, "0:/", 0);
    f_mount(0, "1:/", 0);
load_time_tstX = GetCurrentSystemTime() - enter_time_tstX;
}


void rst_player(void)
{        
                            if(is_Play_Drive == 0 && SD_Card_stat == 0)
                            {
                                
                               // USBH_USR_ApplicationState = USBH_USR_IDLE;
                                //state_wave_player = STATE_WAVE_PLAYER_INIT;                                    
                                    
                               /*
                                 if(is_CopyMenu < 3)
                                 {
                                        is_Play_Drive = 1;
                                 }
                                           */     
                                    
                                if(is_init_done == 10)
                                {
                                        SD_Card_stat = 3;
                                }
                                else
                                {
                                        SD_Card_stat = 2;
                                }
                            }
                            else if(is_Play_Drive == 1)
                            {
                                if(is_init_done == 10)
                                {
                                        USB_Drive_stat = 3;

                                        if((error_SD_card == 0 || error_SD_card == 4) && is_CopyMenu != 10 && is_CopyMenu != 2)
                                        {
                                                //is_CopyMenu = 1; //ASK for copy
                                        }
                                }
                                else
                                {
                                        USB_Drive_stat = 2;
                                }
                            }
}




uint8_t get_music_copy_stat(void){
        return music_not_copied;
}




void  Music_Copy (void){

       FRESULT rc, rw;		/* Result code */
       UINT br, bw;
        
       
        
     //  uint8_t F_bufferM[MAX_READ_BUFFER_SIZE] = {0};
        
       uint8_t was_screen_filled = 0; 
       
       uint32_t time_up_tmp = 0;

        
        
       enter_time_tstX = GetCurrentSystemTime();  
        
       f_close(&FileRead_S);
       f_close(&FileRead_Copy_DestM);
       wave_player_stop();
        
       uint8_t percent_i_MT = 5;
         

       StartUpTime = GetCurrentSystemTime();  
        
                       rc = f_open(&FileRead_S, "1:/Audio.wav" , FA_READ);
                        is_MenusDisabled = 1;
                    
                       if(rc == FR_OK) 
                               {
                                 file_sizeS = f_size(&FileRead_S);

                                 rc = f_mount(&SDFatFs, "0:/", 0);

                                  //rw = f_unlink("0:/Audio.wav");
                                       
                                  rw = f_open(&FileRead_Copy_DestM, "0:/Audio.wav", FA_WRITE | FA_CREATE_ALWAYS);
                                       
                                    
                                // load_time_tstX = GetCurrentSystemTime() - enter_time_tstX;                                      
                                       
                                  if(rw == FR_OK && file_sizeS > 0){   
                                                for (;;) {
                                                        
                                                        if(was_screen_filled == 0){
                                                                                HAL_Delay(DISPLAY_UPDATE_TIME);
                                                                                Draw_Line_Script_Upload(0, 160, 40, WHITE, 201, 0, 0, 0, 0, 0, 0,  0);
                                                                                HAL_Delay(DISPLAY_UPDATE_TIME);
                                                                                
                                                                                was_screen_filled = 1;
                                                                                Draw_Line_Script_Wait(0,240, 40, BLACK, 5);
                                                                                HAL_Delay(DISPLAY_UPDATE_TIME);
                                                                                Draw_Line_Script_Wait(0,160, 40, BLACK, 6);
                                                                                HAL_Delay(DISPLAY_UPDATE_TIME);
                                                                                Draw_Line_Script_Wait(0,280, 40, BLACK, 10);
                                                                                HAL_Delay(DISPLAY_UPDATE_TIME);
                                                                        }
                                                        
                                                        
                                                        rc = f_read(&FileRead_S, Audio_Buffer_Samples, sizeof Audio_Buffer_Samples, &br); /* Read a chunk of data from the source file */
                                                                 if (rc || !br) {//error OR END OF FILE
                                                                        //error_XXCpy = 5;
                                                                        break; /* error or eof */
                                                                }
                                                                 
                                                                if(percent_i_MT == 5)   
                                                                {
                                                                        percent_i_MT = 0;
                                                                        sort_percent = (100 * (Copy_Bytes + sizeof (Audio_Buffer_Samples)))/file_sizeS;
                                                                        
                                                                        uint16_t Percent_tmp = 0;


                                                                        
                                                                        Percent_tmp = 201 + sort_percent;
                                                                        
                                                                        tmp_time_up = GetCurrentSystemTime() - StartUpTime;
                                                                        
                                                                        spped_transfer = ((Copy_Bytes + sizeof (Audio_Buffer_Samples)) / 1000)/(tmp_time_up/1000);
                                        
                                                                        time_up_tmp = tmp_time_up;
                                                                                
                                                                        tmp_time_up = time_up_tmp/sort_percent;
                                                                                                                                                                
                                                                        TimeTillEnd = 1000 + tmp_time_up * (100 - sort_percent);
                                                                        
                                                                        Draw_Line_Script_Upload(0, 160, 40, WHITE, Percent_tmp, 0, 0, 
                                                                                0, TimeTillEnd, time_up_tmp, 0,  0);
                                                                        
                                                                }
                                                                else
                                                                {
                                                                        percent_i_MT++;
                                                                }
                                                                
                                                        if(rw == FR_OK && rc == FR_OK){
                                                                        rc = f_write(&FileRead_Copy_DestM, Audio_Buffer_Samples, br, &bw);           /* Write it to the destination file */
                                                                        if (bw < br) {//error
                                                                                error_XXCpy = 2;
                                                                                break; /* error or eof */
                                                                        }
                                                                   

                                                                Copy_Bytes =  Copy_Bytes + sizeof (Audio_Buffer_Samples);
                                                        }
                                                }
                                                f_lseek(&FileRead_S, 0); 
                                                
                                                f_close(&FileRead_S);
                                                f_close(&FileRead_Copy_DestM);
                                                
                                                rc = f_open(&FileRead_S, "0:/Audio.wav" , FA_READ);
                                                if(rc == FR_OK){
                                                        file_sizeS = f_size(&FileRead_S);
                                                        is_internal_drive = 1;
                                                        music_not_copied = 1;//music was copied
                                                        AP_Switch_ID();
                                                }
                                                else {
                                                        f_close(&FileRead_S);
                                                        rc = f_open(&FileRead_S, "1:/Audio.wav" , FA_READ);
                                                        if(rc == FR_OK){
                                                                file_sizeS = f_size(&FileRead_S);
                                                                
                                                        }else{
                                                                file_sizeS = 0;
                                                        }
                                                        is_internal_drive = 0; 
                                                }
                                                
                                                
                                                
                                        }
                                  else{
                                        //could not copy
                                  }
                                  
                                 
                                  

                        }
                        was_AP_Disp = 0;     
                        is_MenusDisabled = 0;
                               
                        
}


uint8_t get_is_menu_disabled(void){
        return is_MenusDisabled;
}


uint8_t start_music(uint8_t audio_file){

        FRESULT fr;          /* FatFs function common result code */
        
        f_close(&FileRead_S);
        
        if (is_Play_Drive == 0){
                f_mount(&SDFatFs, "0:/", 0);
                memset(path, 0x00, sizeof(path));
                sprintf(path, "0:/Audio%d.wav",audio_file);
                was_mount_SD_ok = 1;
        }
        else if (is_Play_Drive == 1){
                f_mount(&USBDISKFatFs, "1:/", 0);
                memset(path, 0x00, sizeof(path));
                sprintf(path, "1:/Audio%d.wav", audio_file);
                was_mount_USB_ok = 1;
        }
        
        fr = f_open(&FileRead_S, path , FA_READ);
        if(fr == FR_OK){
                was_SeqMusic = audio_file;
                state_wave_player = STATE_WAVE_PLAYER_IDLE;
                current_position = 0;
                return 1;
        }
        else{
                f_close(&FileRead_S);
                was_SeqMusic = 0;
        
                if (is_Play_Drive == 0){
                        f_mount(&SDFatFs, "0:/", 0);
                        memset(path, 0x00, sizeof(path));
                        sprintf(path, "0:/Audio.wav");
                        was_mount_SD_ok = 1;
                }
                else if (is_Play_Drive == 1){
                        f_mount(&USBDISKFatFs, "1:/", 0);
                        memset(path, 0x00, sizeof(path));
                        sprintf(path, "1:/Audio.wav");
                        was_mount_USB_ok = 1;
                }
                fr = f_open(&FileRead_S, path , FA_READ);
                return 0;
        }
}



void rst_music_player(void){
{
        
//        FRESULT fr;          /* FatFs function common result code */
        
        was_SeqMusic = 0;
                        f_close(&FileRead_S);
        
                        if (is_Play_Drive == 0){
                                f_mount(&SDFatFs, "0:/", 0);
                                memset(path, 0x00, sizeof(path));
                                sprintf(path, "0:/Audio.wav");
                                was_mount_SD_ok = 1;
                        }
                        else if (is_Play_Drive == 1){
                                f_mount(&USBDISKFatFs, "1:/", 0);
                                memset(path, 0x00, sizeof(path));
                                sprintf(path, "1:/Audio.wav");
                                was_mount_USB_ok = 1;
                        }
                        f_open(&FileRead_S, path , FA_READ);
                }
        }

uint8_t get_was_SeqMusic(void){
        return was_SeqMusic;
}


uint32_t get_error_SD(void){
        return countLastTime1;
}

void delete_audio_int(void){
  FRESULT rc;		/* Result code */
        
        char pathM[] = "0:/Audio.wav";
        
         memset(pathM, 0x00, sizeof(pathM));

        sprintf(pathM, "0:/Audio.wav");
    
         
        rc = f_open(&FileRead_S, pathM , FA_READ);
        f_close(&FileRead_S);

        if(rc == FR_OK) {

        f_unlink("0:/Audio.wav");
        }
        
}


void test_music_file(void){
                SDXtst = f_open(&FileRead_S, path , FA_READ);

                if(SDXtst != FR_OK)
                {
                    //printf("ERROR: Opening '%s' for read\r\n", wavefilename);
                    file_read_errorX++;    
                        
                }
                else
                {    
                    file_read_error = 0;  
                    /* Read sizeof(WaveFormat) from the selected file */
                    current_position = 0;
                   // f_lseek(&FileRead_S, current_position);
                    
                    f_read(&FileRead_S, &waveformat, sizeof(waveformat), &bytesread);
                    current_position += bytesread;

                    // calculate no.of samples
//                    long num_samples = (8 * waveformat.SubChunk2Size) / (waveformat.NbrChannels * waveformat.BitPerSample);
                 //   long size_of_each_sample = (waveformat.NbrChannels * waveformat.BitPerSample) / 8;
                    // calculate duration of file
                   // float duration_in_seconds = (float) waveformat.FileSize / waveformat.ByteRate;

//                    bytes_per_pcm_frame = get_bytes_per_pcm_frame(waveformat.BitPerSample, waveformat.NbrChannels, waveformat.BlockAlign);

                    

                    /* Set WaveDataLenght to the Speech Wave length */
//                    WaveDataLength = waveformat.SubChunk2Size;
                    WaveDataLength = waveformat.FileSize;

                    /* Start playing */
                    /* Initialize Wave player (Codec, DMA, I2C) */
                    if(WavePlayerInit(waveformat.SampleRate, 16) != 0)
                    {
                        Error_Handler();
                    }
            }
                
             SDXtst = f_close(&FileRead_S);
             if(SDXtst != FR_OK)
                {
                    //printf("ERROR: Opening '%s' for read\r\n", wavefilename);
                    file_read_errorX++;    
                        
                }
}
