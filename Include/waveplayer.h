  
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __WAVEPLAYER_H
#define __WAVEPLAYER_H

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "ff.h"

/* Exported types ------------------------------------------------------------*/
typedef enum
{
  BUFFER_OFFSET_NONE = 0,  
  BUFFER_OFFSET_HALF,  
  BUFFER_OFFSET_FULL,     
} BUFFER_StateTypeDef;

typedef struct
{
  uint32_t   ChunkID;       /* 0 */ 
  uint32_t   FileSize;      /* 4 */
  uint32_t   FileFormat;    /* 8 */
  uint32_t   SubChunk1ID;   /* 12 */
  uint32_t   SubChunk1Size; /* 16*/  
  uint16_t   AudioFormat;   /* 20 */ 
  uint16_t   NbrChannels;   /* 22 */   
  uint32_t   SampleRate;    /* 24 */
  
  uint32_t   ByteRate;      /* 28 */
  uint16_t   BlockAlign;    /* 32 */  
  uint16_t   BitPerSample;  /* 34 */  
  uint32_t   SubChunk2ID;   /* 36 */   
  uint32_t   SubChunk2Size; /* 40 */    

}WAVE_FormatTypeDef;

enum
{
    STATE_WAVE_PLAYER_INIT = 0,
    STATE_WAVE_PLAYER_IDLE,
    STATE_WAVE_PLAYER_START,
    STATE_WAVE_PLAYER_PLAY,
    
};

/* Defines for the Audio playing process */
enum
{
    PAUSE_STATUS = 0,       /* Audio Player in Pause Status */
    RESUME_STATUS,          /* Audio Player in Resume Status */
    IDLE_STATUS,            /* Audio Player in Idle Status */
    //STOP_STATUS,            /* Audio Player in Stop Status */
};

/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
int   WavePlayerInit(uint32_t AudioFreq, uint8_t bits);
void  WavePlayerStop(void);
void  WavePlayerPauseResume(uint32_t state);
void  WavePlayer_CallBack(void);

uint8_t wave_player_dump_wave_header(char *filename);
uint8_t wave_player_dump_wave_buffer(char *filename);
void wave_player_change_file(char *filename);

void wave_player_stop(void);

void wave_player_init(char* filename);
uint8_t wave_player_process(void);

void wave_player_volume_set(uint8_t volume);
void wave_player_current_position_set(FSIZE_t position);
void wave_player_current_seconds_set(uint32_t seconds);
void wave_player_current_miliseconds_set(uint32_t mseconds);
void wave_player_current_miliseconds_get(uint32_t *mseconds);
void wave_player_remain_miliseconds_get(uint32_t *mseconds);

char* seconds_to_time(float raw_seconds, char *buffer);

void  wave_player_pause_resume_set(uint32_t state);
void  wave_player_state_set(uint8_t state);
void  wave_player_debug_set(uint8_t debug);
void set_curent_pos(void);
uint8_t read_mp3_status(void);
uint8_t read_mp3_Pstatus(void);

void copy_toSD(void);
void rst_player(void);

void AP_Switch_ID(void);
void AP_Switch_UD(void);

uint8_t need_toSwitch(void);
uint16_t get_volume(void);
uint64_t ret_wave_player_size(uint32_t mseconds);
uint64_t getWaveDataLength(void);
uint8_t get_play_drive(void);
uint8_t get_is_menu_disabled(void);
uint8_t start_music(uint8_t audio_file);
void rst_music_player(void);
uint8_t get_was_SeqMusic(void);
uint32_t get_error_SD(void);
void test_music_file(void);

FRESULT check_player_stat(uint8_t tmp_stats);

uint64_t getCur_pos(void);
uint8_t ReadWirKey(void);
void reset_Audiobox(void);


#endif /* __WAVEPLAYER_H */


