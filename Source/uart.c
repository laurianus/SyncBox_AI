
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"
#include "stdio.h"
#include "uart.h"
#include "options.h"
#include "SystemTimer.h"
#include "TimeCode.h"
#include "GlobalPositioning.h"

#include "stm32f4xx_ll_dma.h"
#include "stm32f4xx_ll_bus.h"
#include "stm32f4xx_ll_usart.h"
#include "stm32f4xx_ll_tim.h"

#include "stm32f4xx_ll_gpio.h"



uint8_t msg_number = 0;

char tst_msg_fsgdASD[64] = {0};

uint16_t msg_433_err = 0;

extern uint32_t ID_MCU2;


//uint8_t was_433_ok_here = 0;

int noise_level_433 = 0;

uint16_t msg_433_mhz_sent = 0;


volatile uint32_t last_msg_433Mhz_timer = 0;

extern volatile uint8_t DMX_stat;

int noise_level_433Tr = 0;
int last_msg_level_433 = 0;

uint8_t DMX_stateX = 0;

extern uint8_t buf_DMX_DMA[DMX_CH_MAX]; //what would be sent to DMA

extern uint8_t buf_DMX[DMX_CH_MAX];
extern uint8_t buf_DMX_ARM[DMX_CH_MAX];

extern uint8_t msg_433_tmpA[MAXIMUM_433_MESSAGE_SIZE];

char tst_msg_fsgAAAA [12] = {0};
extern uint64_t last_msg_snt_timer;

uint32_t Time_between_valid_msg = 0;
uint32_t last_msg_rcvd_here = 0;
uint16_t msg_valid_err = 0;

//uint8_t was_433_ok_here_mhz_TEST = 0;


/*
uint8_t tst_msg_fsgdAAA[255] = {0};
uint8_t tmp_pos_ms_nbr = 0;
*/
/*
uint64_t timer_of_last_433_msg = 0;
uint32_t timer_rcvd_msg_433_X = 0;
*/

/** @addtogroup STM32F4xx_HAL_Examples
  * @{
  */

/** @addtogroup UART_Printf
  * @{
  */ 

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* UART handler declaration */
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart4;
extern UART_HandleTypeDef huart5;
extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart6;
extern UART_HandleTypeDef huart7;
extern UART_HandleTypeDef huart8;

/* Place DMA handle in Main SRAM (not CCM) — DMA controller cannot access CCM */
DMA_HandleTypeDef hdma_uart8_tx __attribute__((section(".dma_data")));

extern TIM_HandleTypeDef htim6;

//extern uint32_t no_rng_gen;

/* Private function prototypes -----------------------------------------------*/
#ifdef __GNUC__
  /* With GCC, small printf (option LD Linker->Libraries->Small printf
     set to 'Yes') calls __io_putchar() */
  #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
  #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
//  #define GETCHAR_PROTOTYPE int fgetc(FILE *f)
#endif /* __GNUC__ */

  
uint16_t fsk_errrr = 0;
//uint16_t fsk_ERRER = 0;
extern uint8_t TC_fps_setX;
  
  /**
@def __BUF_IS_EMPTY
@brief Check is buffer empty
*/
  
  
#if TST_DEBUG
uint64_t Start_433Mhz_timer = 0;
uint16_t MSG_433Mhz_timer = 0;
int Meesage_received = 0;
#endif
  
uint8_t rasb_433DataBuffer[UART_BUFFER_SIZE] = {0};
uint8_t Bytes_433_rcvd = 0;

/*
uint8_t default_set_433[] = {0xC0,0x00,0x09,0x00,0x00,0x00,0xE2,DEFAULT_433_PWR_DAT,0x28,0x80,0xAB,0xCD}; //2.4kbs
uint8_t defaultX_set1_433[] = {0xC0,0x00,0x09,0x00,0x00,0x00,0xE2,DEFAULT_433_PWR_DAT,DEFAULT_433_CHANNEL,0x80,0xAB,0xCD}; //2.4kbs
uint8_t defaultX_set2_433[] = {0xC0,0x00,0x09,0x00,0x00,0x00,0xE3,DEFAULT_433_PWR_DAT,DEFAULT_433_CHANNEL,0x80,0xAB,0xCD}; //4.8kbs
uint8_t defaultX_set3_433[] = {0xC0,0x00,0x09,0x00,0x00,0x00,0xE4,DEFAULT_433_PWR_DAT,DEFAULT_433_CHANNEL,0x80,0xAB,0xCD}; //9.6kbs
uint8_t defaultX_set4_433[] = {0xC0,0x00,0x09,0x00,0x00,0x00,0xE5,DEFAULT_433_PWR_DAT,DEFAULT_433_CHANNEL,0x80,0xAB,0xCD}; //19.2kbs
uint8_t defaultX_set5_433[] = {0xC0,0x00,0x09,0x00,0x00,0x00,0xE7,DEFAULT_433_PWR_DAT,DEFAULT_433_CHANNEL,0x80,0xAB,0xCD}; //62.5kbs
*/


uint8_t defaultX_set3_433C[] = {0xC0,0x00,0x09,0x00,0x00,0x00,DEFAULT_433_SPEED,DEFAULT_433_PWR_DAT,DEFAULT_433_CHANNEL,0x83,0xAB,0xCD}; //9.6kbs
//uint8_t defaultX_set3_433C[] = {0xC0,0x00,0x09,0x00,0x00,0x00,DEFAULT_433_SPEED,DEFAULT_433_PWR_DAT,DEFAULT_433_CHANNEL,0x00,0xAB,0xCD}; //9.6kbs
//uint8_t defaultX_set3_433C[] = {0xC0,0x00,0x09,0x00,0x00,0xE3,0xE0,0x23,0x50,0x8B,0x00,0x00}; //9.6kbs
//uint8_t defaultX_set3_433C[] = {0xC0,0x00,0x09,0x00,0x00,0x00,0xE3,0xE0,0x23,0x50,0x00,0x00}; //9.6kbs

 

uint8_t RSSI_Enable[] = {0xC0,0x04,0x01,0x20}; //RSSI Envoirment enable


  
uint8_t buffer_433_snd[WIRELESS_433_BUF_SIZE][MAXIMUM_433_MESSAGE_SIZE] = {{0}}; 
uint8_t last_433_msg_snt = 0;
uint8_t cur_433_msg_snt = 0;
uint8_t msg_to_be_snt_433Mhz = 0;

uint8_t is_433Mhz_en = 0;


  
static int32_t getIncomingMessageSizeGPS(uint32_t headw, uint32_t tailw);
  
/* Private functions ---------------------------------------------------------*/

void uart_init(void)
{
  /*##-1- Configure the UART peripheral ######################################*/
  /* Put the USART peripheral in the Asynchronous mode (UART Mode) */
  /* UART1 configured as follows:
      - Word Length = 8 Bits
      - Stop Bit = One Stop bit
      - Parity = No parity
      - BaudRate = 115200 baud
      - Hardware flow control disabled (RTS and CTS signals) */
  //UartHandle.Instance          = USARTx;
  huart4.Instance          = UART4;
  
  huart4.Init.BaudRate     = 115200;
  huart4.Init.WordLength   = UART_WORDLENGTH_8B;
  huart4.Init.StopBits     = UART_STOPBITS_1;
  huart4.Init.Parity       = UART_PARITY_NONE;
  huart4.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
  huart4.Init.Mode         = UART_MODE_TX_RX;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
    
  if(HAL_UART_Init(&huart4) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler(); 
  }
}

/**
  * @brief  Retargets the C library printf function to the USART.
  * @param  None
  * @retval None
  */

#if 0
PUTCHAR_PROTOTYPE
{
  /* Place your implementation of fputc here */
  /* e.g. write a character to the EVAL_COM1 and Loop until the end of transmission */
//  HAL_UART_Transmit(&UartHandle, (uint8_t *)&ch, 1, 0xFFFF); 
    Uart_write(ch);
    
  return ch;
}
#endif

#if 0
GETCHAR_PROTOTYPE
{
    HAL_StatusTypeDef status;
    uint8_t data;
    
//    if (__HAL_UART_GET_FLAG(&UartHandle, UART_FLAG_RXNE) != RESET)
    {
        status = HAL_UART_Receive(&UartHandle, (uint8_t *) &data, 1, HAL_MAX_DELAY);
//        status = HAL_UART_Receive(&UartHandle, (uint8_t *) &data, 1, 0);
        if (status == HAL_OK)
        {
            return ((int) data);
        }
        else
        {
            return EOF;
        }
    }
//	return EOF;
}

uint8_t uart_get_char(char *data)
{
    HAL_StatusTypeDef status;
    
//    if (__HAL_UART_GET_FLAG(&UartHandle, UART_FLAG_RXNE) != RESET)
    {
//        status = HAL_UART_Receive(&UartHandle, (uint8_t *) &data, 1, HAL_MAX_DELAY);
        status = HAL_UART_Receive(&UartHandle, (uint8_t *) &data, 1, 10);
        if (status == HAL_OK)
        {
            return 1;
        }
    }
    return 0;
}
#endif






/**
* @brief UART MSP Initialization
* This function configures the hardware resources used in this example
* @param huart: UART handle pointer
* @retval None
*/
void HAL_UART_MspInit(UART_HandleTypeDef* huart)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(huart->Instance==UART4)
  {
  /* USER CODE BEGIN UART4_MspInit 0 */

  /* USER CODE END UART4_MspInit 0 */
    /* Peripheral clock enable */
    __HAL_RCC_UART4_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**UART4 GPIO Configuration
    PA0-WKUP     ------> UART4_TX
    PA1     ------> UART4_RX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF8_UART4;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* UART4 interrupt Init */
    HAL_NVIC_SetPriority(UART4_IRQn, 3, 1);
    HAL_NVIC_EnableIRQ(UART4_IRQn);

  /* USER CODE BEGIN UART4_MspInit 1 */

  /* USER CODE END UART4_MspInit 1 */
  }
  else if(huart->Instance==UART5)
  {
  /* USER CODE BEGIN UART5_MspInit 0 */

  /* USER CODE END UART5_MspInit 0 */
    /* Peripheral clock enable */
    __HAL_RCC_UART5_CLK_ENABLE();

    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    /**UART5 GPIO Configuration
    PC12     ------> UART5_TX
    PD2     ------> UART5_RX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF8_UART5;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_2;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF8_UART5;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    /* UART5 interrupt Init */
    HAL_NVIC_SetPriority(UART5_IRQn, 1, 6);
    HAL_NVIC_EnableIRQ(UART5_IRQn);
  /* USER CODE BEGIN UART5_MspInit 1 */

  /* USER CODE END UART5_MspInit 1 */
  }
  else if(huart->Instance==USART1)
  {
  /* USER CODE BEGIN USART1_MspInit 0 */

  /* USER CODE END USART1_MspInit 0 */
    /* Peripheral clock enable */
    __HAL_RCC_USART1_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**USART1 GPIO Configuration
    PA9     ------> USART1_TX
    PA10     ------> USART1_RX
    */
        GPIO_InitStruct.Pin = WIRELESS_RX|WIRELESS_TX;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
        HAL_GPIO_Init(WIRELESS_PORT, &GPIO_InitStruct);

    

  /* USER CODE BEGIN USART1_MspInit 1 */

  /* USER CODE END USART1_MspInit 1 */

    /* USART1 interrupt Init - priority 1 to avoid blocking Timer IRQs at priority 0 */
    HAL_NVIC_SetPriority(USART1_IRQn, 1, 3);
    HAL_NVIC_EnableIRQ(USART1_IRQn);
  /* USER CODE BEGIN USART1_MspInit 1 */

  /* USER CODE END USART1_MspInit 1 */
  }
  else if(huart->Instance==USART2)
  {
  /* USER CODE BEGIN USART2_MspInit 0 */

  /* USER CODE END USART2_MspInit 0 */
    /* Peripheral clock enable */
    __HAL_RCC_USART2_CLK_ENABLE();

    __HAL_RCC_GPIOD_CLK_ENABLE();
    /**USART2 GPIO Configuration
    PD5     ------> USART2_TX
    PD6     ------> USART2_RX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_6;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    /* USART2 interrupt Init */
    HAL_NVIC_SetPriority(USART2_IRQn, 3, 7);
    HAL_NVIC_EnableIRQ(USART2_IRQn);
  /* USER CODE BEGIN USART2_MspInit 1 */

  /* USER CODE END USART2_MspInit 1 */
  }
  else if(huart->Instance==USART3)
  {
  /* USER CODE BEGIN USART3_MspInit 0 */

  /* USER CODE END USART3_MspInit 0 */
    /* Peripheral clock enable */
    __HAL_RCC_USART3_CLK_ENABLE();

    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**USART3 GPIO Configuration
    PB10     ------> USART3_TX
    PB11     ------> USART3_RX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_11;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART3;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    HAL_NVIC_SetPriority(USART3_IRQn, 1, 9);
    HAL_NVIC_EnableIRQ(USART3_IRQn);          
          
  /* USER CODE BEGIN USART3_MspInit 1 */

  /* USER CODE END USART3_MspInit 1 */
  }
  else if(huart->Instance==USART6)
  {
  /* USER CODE BEGIN USART6_MspInit 0 */

  /* USER CODE END USART6_MspInit 0 */
    /* Peripheral clock enable */
    __HAL_RCC_USART6_CLK_ENABLE();

    __HAL_RCC_GPIOC_CLK_ENABLE();
    /**USART6 GPIO Configuration
    PC6     ------> USART6_TX
    PC7     ------> USART6_RX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF8_USART6;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    /* USART6 interrupt Init - priority 1 to avoid blocking Timer IRQs at priority 0 */
    HAL_NVIC_SetPriority(USART6_IRQn, 1, 3);
    HAL_NVIC_EnableIRQ(USART6_IRQn);
  /* USER CODE BEGIN USART6_MspInit 1 */

  /* USER CODE END USART6_MspInit 1 */
  }
  else if(huart->Instance==UART7)
  {
  /* USER CODE BEGIN UART7_MspInit 0 */

  /* USER CODE END UART7_MspInit 0 */
    /* Peripheral clock enable */
    __HAL_RCC_UART7_CLK_ENABLE();

    __HAL_RCC_GPIOF_CLK_ENABLE();
    /**UART7 GPIO Configuration
    PF6     ------> UART7_RX
    PF7     ------> UART7_TX
    */
    GPIO_InitStruct.Pin = TC_TX_Pin|TX_RX_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF8_UART7;
    HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /* USER CODE BEGIN UART7_MspInit 1 */
          
    HAL_NVIC_SetPriority(UART7_IRQn, 1, 8);
    HAL_NVIC_EnableIRQ(UART7_IRQn);

  /* USER CODE END UART7_MspInit 1 */
  }
  
 #if 0 
    else if(huart->Instance==UART8)
  {
  /* USER CODE BEGIN UART8_MspInit 0 */

  /* USER CODE END UART8_MspInit 0 */
    /* Peripheral clock enable */
    __HAL_RCC_UART8_CLK_ENABLE();

    __HAL_RCC_GPIOE_CLK_ENABLE();
    /**UART8 GPIO Configuration
    PE0     ------> UART8_RX
    PE1     ------> UART8_TX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF8_UART8;
    HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /* USER CODE BEGIN UART8_MspInit 1 */

    HAL_NVIC_SetPriority(UART8_IRQn, 0, 10);
    HAL_NVIC_EnableIRQ(UART8_IRQn);

  /* USER CODE END UART7_MspInit 1 */
  }
  #endif
  #if 1
    else if(huart->Instance==UART8)
  {
  /* USER CODE BEGIN UART8_MspInit 0 */

  /* USER CODE END UART8_MspInit 0 */
    /* Peripheral clock enable */
    __HAL_RCC_UART8_CLK_ENABLE();

    __HAL_RCC_GPIOE_CLK_ENABLE();
    /**UART8 GPIO Configuration
    PE0     ------> UART8_RX
    PE1     ------> UART8_TX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF8_UART8;
    HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

    /* UART8 DMA Init */
    /* UART8_TX Init */
    hdma_uart8_tx.Instance = DMA1_Stream0;
    hdma_uart8_tx.Init.Channel = DMA_CHANNEL_5;
    hdma_uart8_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_uart8_tx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_uart8_tx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_uart8_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_uart8_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_uart8_tx.Init.Mode = DMA_NORMAL;
    hdma_uart8_tx.Init.Priority = DMA_PRIORITY_VERY_HIGH;
    hdma_uart8_tx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_uart8_tx) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(huart,hdmatx,hdma_uart8_tx);

  /* USER CODE BEGIN UART8_MspInit 1 */

  /* USER CODE END UART8_MspInit 1 */
  }
  #endif

}

/**
* @brief UART MSP De-Initialization
* This function freeze the hardware resources used in this example
* @param huart: UART handle pointer
* @retval None
*/
void HAL_UART_MspDeInit(UART_HandleTypeDef* huart)
{
  if(huart->Instance==UART4)
  {
  /* USER CODE BEGIN UART4_MspDeInit 0 */

  /* USER CODE END UART4_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_UART4_CLK_DISABLE();

    /**UART4 GPIO Configuration
    PA0-WKUP     ------> UART4_TX
    PA1     ------> UART4_RX
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_0|GPIO_PIN_1);

    /* UART4 interrupt DeInit */
    HAL_NVIC_DisableIRQ(UART4_IRQn);
  /* USER CODE BEGIN UART4_MspDeInit 1 */

  /* USER CODE END UART4_MspDeInit 1 */
  }
  else if(huart->Instance==UART5)
  {
  /* USER CODE BEGIN UART5_MspDeInit 0 */

  /* USER CODE END UART5_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_UART5_CLK_DISABLE();

    /**UART5 GPIO Configuration
    PC12     ------> UART5_TX
    PD2     ------> UART5_RX
    */
    HAL_GPIO_DeInit(GPIOC, GPIO_PIN_12);

    HAL_GPIO_DeInit(GPIOD, GPIO_PIN_2);

    /* UART5 interrupt DeInit */
    HAL_NVIC_DisableIRQ(UART5_IRQn);
  /* USER CODE BEGIN UART5_MspDeInit 1 */

  /* USER CODE END UART5_MspDeInit 1 */
  }
  else if(huart->Instance==USART1)
  {
  /* USER CODE BEGIN USART1_MspDeInit 0 */

  /* USER CODE END USART1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART1_CLK_DISABLE();

    /**USART1 GPIO Configuration
    PA9     ------> USART1_TX
    PA10     ------> USART1_RX
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_9|GPIO_PIN_10);

    /* USART1 interrupt DeInit */
    HAL_NVIC_DisableIRQ(USART1_IRQn);
  /* USER CODE BEGIN USART1_MspDeInit 1 */

  /* USER CODE END USART1_MspDeInit 1 */
  }
  else if(huart->Instance==USART3)
  {
  /* USER CODE BEGIN USART3_MspDeInit 0 */

  /* USER CODE END USART3_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART3_CLK_DISABLE();

    /**USART3 GPIO Configuration
    PB10     ------> USART3_TX
    PB11     ------> USART3_RX
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_10|GPIO_PIN_11);

    /* USART3 interrupt DeInit */
    HAL_NVIC_DisableIRQ(USART3_IRQn);
  /* USER CODE BEGIN USART3_MspDeInit 1 */

  /* USER CODE END USART3_MspDeInit 1 */
  }
  else if(huart->Instance==USART6)
  {
  /* USER CODE BEGIN USART6_MspDeInit 0 */

  /* USER CODE END USART6_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART6_CLK_DISABLE();

    /**USART6 GPIO Configuration
    PC6     ------> USART6_TX
    PC7     ------> USART6_RX
    */
    HAL_GPIO_DeInit(GPIOC, GPIO_PIN_6|GPIO_PIN_7);

    /* USART6 interrupt DeInit */
    HAL_NVIC_DisableIRQ(USART6_IRQn);
  /* USER CODE BEGIN USART6_MspDeInit 1 */

  /* USER CODE END USART6_MspDeInit 1 */
  }
  else if(huart->Instance==UART7)
  {
  /* USER CODE BEGIN UART7_MspDeInit 0 */

  /* USER CODE END UART7_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_UART7_CLK_DISABLE();

    /**UART7 GPIO Configuration
    PF6     ------> UART7_RX
    PF7     ------> UART7_TX
    */
    HAL_GPIO_DeInit(GPIOF, TX_RX_Pin|TC_TX_Pin);

    /* UART7 interrupt DeInit */
    HAL_NVIC_DisableIRQ(UART7_IRQn);
  /* USER CODE BEGIN UART7_MspDeInit 1 */

  /* USER CODE END UART7_MspDeInit 1 */
  }

}






/**
  * @}
  */ 

/**
  * @}
  */ 


/*
 * UartRingbuffer.c
 *
 *  Created on: 10-Jul-2019
 *      Author: Controllerstech
 *
 *  Modified on: 11-April-2020
 */





ring_buffer rx_buffer = { { 0 }, 0, 0};
//ring_buffer tx_buffer = { { 0 }, 0, 0};

ring_buffer *_rx_buffer;
//ring_buffer *_tx_buffer;


ring_buffer rx_buffer1 = { { 0 }, 0, 0};
//ring_buffer tx_buffer1 = { { 0 }, 0, 0};

ring_buffer *_rx_buffer1;
//ring_buffer *_tx_buffer1;


ring_buffer rxTC_buffer = { { 0 }, 0, 0};
//ring_buffer txTC_buffer = { { 0 }, 0, 0};

ring_buffer *_rxTC_buffer;
//ring_buffer *_txTC_buffer;

ring_buffer rxGPS_buffer = { { 0 }, 0, 0};
//ring_buffer txGPS_buffer = { { 0 }, 0, 0};

ring_buffer *_rxGPS_buffer;
//ring_buffer *_txGPS_buffer;

ring_buffer rx433_buffer = { { 0 }, 0, 0};
//ring_buffer tx433_buffer = { { 0 }, 0, 0};

ring_buffer *_rx433_buffer;
//ring_buffer *_tx433_buffer;


void store_char(unsigned char c, ring_buffer *buffer);


void Ringbuf_init(void)
{
  _rx_buffer = &rx_buffer;
  //_tx_buffer = &tx_buffer;
    
 //   huart

  /* Enable the UART Error Interrupt: (Frame error, noise error, overrun error) */
//  __HAL_UART_ENABLE_IT(uart, UART_IT_ERR);

  /* Enable the UART Data Register not empty Interrupt */
  __HAL_UART_ENABLE_IT(&huart1, UART_IT_RXNE);
}


void RingbufTC_init(void)
{
  _rxTC_buffer = &rxTC_buffer;
 // _txTC_buffer = &txTC_buffer;

  /* Enable the UART Data Register not empty Interrupt */
  __HAL_UART_ENABLE_IT(&huart7, UART_IT_RXNE);

}


void RingbufGPS_init(void)
{
  _rxGPS_buffer = &rxGPS_buffer;
 // _txGPS_buffer = &txGPS_buffer;

  /* Enable the UART Data Register not empty Interrupt */
  __HAL_UART_ENABLE_IT(&huart3, UART_IT_RXNE);

}


void Ringbuf433_init(void)
{
  _rx433_buffer = &rx433_buffer;
  //_tx433_buffer = &tx433_buffer;

  /* Enable the UART Data Register not empty Interrupt */
  __HAL_UART_ENABLE_IT(&huart6, UART_IT_RXNE);

}

void Ringbuf_init1(void)
{
  _rx_buffer1 = &rx_buffer1;
  //_tx_buffer1 = &tx_buffer1;

  /* Enable the UART Error Interrupt: (Frame error, noise error, overrun error) */
//  __HAL_UART_ENABLE_IT(uart, UART_IT_ERR);

  /* Enable the UART Data Register not empty Interrupt */
  __HAL_UART_ENABLE_IT(&huart4, UART_IT_RXNE);
}


void store_char(unsigned char c, ring_buffer *buffer)
{
  int i = (unsigned int)(buffer->head + 1) % UART_BUFFER_SIZE;

  // if we should be storing the received character into the location
  // just before the tail (meaning that the head would advance to the
  // current location of the tail), we're about to overflow the buffer
  // and so we don't write the character or advance the head.
  if(i != buffer->tail) {
    buffer->buffer[buffer->head] = c;
    buffer->head = i;
  }
}


static int32_t getIncomingMessageSize(uint32_t headw, uint32_t tailw){
      uint32_t size = 0;
      uint8_t bufaaa = 0;
      

             while(!__BUF_IS_EMPTY(headw, tailw)){
                   if(_rx_buffer->buffer[tailw] == 0x0A && bufaaa==0x0D)
                    {
                          size++;
                          return size;
                    }		
                   bufaaa = _rx_buffer->buffer[tailw];
                   __BUF_INCR(tailw);
                   size++;
             }
             
      return -1;

}





int Uart_read(void)
{
  // if the head isn't ahead of the tail, we don't have any characters
   
  if(_rx_buffer->head == _rx_buffer->tail)
  {
    return -1;
  }
  else
  {
      
    unsigned char c = _rx_buffer->buffer[_rx_buffer->tail];
    _rx_buffer->tail = (unsigned int)(_rx_buffer->tail + 1) % UART_BUFFER_SIZE;
    return c;
  }
    
}



uint32_t UartWirReceive(uint8_t *buffer)
 {
       uint8_t *data = (uint8_t *)buffer;
       
       
       uint32_t count = 0;
       uint32_t bufbbb = 0;
       int32_t position = 0;
       uint32_t returnValue = 0;
         
       uint16_t tmp_head = _rx_buffer->head;
       uint16_t tmp_tail = _rx_buffer->tail;
     
      //  __HAL_UART_DISABLE_IT(&huart1, UART_IT_RXNE);
               position = getIncomingMessageSize(tmp_head, tmp_tail);
               
               if(position > 0){
                     
                     count = position;
                     returnValue = count;
                     bufbbb = count;
                     
                     //UART_IntConfig(UARTPort, UART_INTCFG_RBR, DISABLE); Disable intrerupt
                     
                     // Loop until receive buffer ring is empty or until max_bytes expires 

                     
                    while ((count > 0) && (!(__BUF_IS_EMPTY(tmp_head, _rx_buffer->tail)))){
                  // while (count > 0){
		
                           if(position > 2){
                                   if(bufbbb  < MAX_WIRELESS_MESSAGE_SIZE){
                                         // Read data from ring buffer into user buffer 
                                         *data = _rx_buffer->buffer[_rx_buffer->tail];
                                         data++;				
                                   }else{
                                         returnValue = 0;
                                   }
                           }
                           
                           // Update tail pointer 
                           __BUF_INCR(_rx_buffer->tail);
                           
                           // decrement buffer size count
                           count--;	
                     }
                     
                     // Re-enable UART interrupts
                  //   UART_IntConfig(UARTPort, UART_INTCFG_RBR, ENABLE);
               }
               
         //      __HAL_UART_ENABLE_IT(&huart1, UART_IT_RXNE);
               
               return returnValue;

 }

/*
int Uart_read(void)
{
 */
    /*
       uint8_t data_wir[256] = {0};
       uint8_t data = 0;
       uint32_t head=0;
       uint32_t tail=0;
       
       uint32_t count=0;
       uint32_t bufbbb =0;
       int32_t position=0;
       uint32_t returnValue=0;
    
  // if the head isn't ahead of the tail, we don't have any characters
  if(_rx_buffer->head == _rx_buffer->tail)
  {
    return -1;
  }
  else
  {
      
               position = getIncomingMessageSize(head, tail);
               
               if(position>0){
                     
                     count=position;
                     returnValue=count;
                     bufbbb=count;
                     
                     //UART_IntConfig(UARTPort, UART_INTCFG_RBR, DISABLE); Disable intrerupt
                     
                     // Loop until receive buffer ring is empty or                     until max_bytes expires 

                     
                     while ((count > 0) && (!(__BUF_IS_EMPTY(_rx_buffer->head, _rx_buffer->tail)))){
		
                           if(bufbbb  < MAX_WIRELESS_MESSAGE_SIZE){
                                 // Read data from ring buffer into user buffer 
                                 data_wir[data] = _rx_buffer->buffer[_rx_buffer->tail];
                                 //data = rb[ringBuffer].rx[rb[ringBuffer].rx_tail];
                                 data++;				
                           }else{
                                 returnValue=0;
                           }
                           
                           // Update tail pointer 
                           __BUF_INCR(_rx_buffer->tail);
                           
                           // decrement buffer size count
                           count--;	
                     }
                     
                     // Re-enable UART interrupts 
                  //   UART_IntConfig(UARTPort, UART_INTCFG_RBR, ENABLE);
               }
  }
  return -1;
  */
  /*
  return -1;
}


*/

void Uart_write(int c)
{
        /*
	if (c >= 0)
	{
		int i = (_tx_buffer->head + 1) % UART_BUFFER_SIZE;

		// If the output buffer is full, there's nothing for it other than to
		// wait for the interrupt handler to empty it a bit
		// ???: return 0 here instead?
		while (i == _tx_buffer->tail);

		_tx_buffer->buffer[_tx_buffer->head] = (uint8_t)c;
		_tx_buffer->head = i;

		__HAL_UART_ENABLE_IT(&huart4, UART_IT_TXE); // Enable UART transmission interrupt
	}
        */
}

int IsDataAvailable(void)
{
//    return (uint16_t)(UART_BUFFER_SIZE + _rx_buffer->head - _rx_buffer->tail) % UART_BUFFER_SIZE;

    uint16_t result;
    
    result = (uint16_t)(UART_BUFFER_SIZE + _rx_buffer->head - _rx_buffer->tail) % UART_BUFFER_SIZE;
    if (result)
    {
        return result;
    }
    else
    {
        return 0;
    }
}

void Uart_flush (void)
{
	memset(_rx_buffer->buffer,'\0', UART_BUFFER_SIZE);
	_rx_buffer->head = 0;
}


void Uart_wir_flush (void)
{
	memset(_rx_buffer1->buffer,'\0', UART_BUFFER_SIZE);
	_rx_buffer1->head = 0;
}


void Uart_wir_isr (UART_HandleTypeDef *huart)
{
	  uint32_t isrflags   = READ_REG(huart->Instance->SR);
	  uint32_t cr1its     = READ_REG(huart->Instance->CR1);


    /* if DR is not empty and the Rx Int is enabled */
    if (((isrflags & USART_SR_RXNE) != RESET) && ((cr1its & USART_CR1_RXNEIE) != RESET))
    {
    	 /******************
    	    	      *  @note   PE (Parity error), FE (Framing error), NE (Noise error), ORE (Overrun
    	    	      *          error) and IDLE (Idle line detected) flags are cleared by software
    	    	      *          sequence: a read operation to USART_SR register followed by a read
    	    	      *          operation to USART_DR register.
    	    	      * @note   RXNE flag can be also cleared by a read to the USART_DR register.
    	    	      * @note   TC flag can be also cleared by software sequence: a read operation to
    	    	      *          USART_SR register followed by a write operation to USART_DR register.
    	    	      * @note   TXE flag is cleared only by a write to the USART_DR register.

    	 *********************/
		huart->Instance->SR;                       /* Read status register */
        unsigned char c = huart->Instance->DR;     /* Read data register */

                    store_char (c, _rx_buffer);  // store data in buffer
       return;
    }


}


void Uart_TC_isr (UART_HandleTypeDef *huart)
{
	  uint32_t isrflags   = READ_REG(huart->Instance->SR);
	  uint32_t cr1its     = READ_REG(huart->Instance->CR1);


    /* if DR is not empty and the Rx Int is enabled */
    if (((isrflags & USART_SR_RXNE) != RESET) && ((cr1its & USART_CR1_RXNEIE) != RESET))
    {
    	 /******************
    	    	      *  @note   PE (Parity error), FE (Framing error), NE (Noise error), ORE (Overrun
    	    	      *          error) and IDLE (Idle line detected) flags are cleared by software
    	    	      *          sequence: a read operation to USART_SR register followed by a read
    	    	      *          operation to USART_DR register.
    	    	      * @note   RXNE flag can be also cleared by a read to the USART_DR register.
    	    	      * @note   TC flag can be also cleared by software sequence: a read operation to
    	    	      *          USART_SR register followed by a write operation to USART_DR register.
    	    	      * @note   TXE flag is cleared only by a write to the USART_DR register.

    	 *********************/
		huart->Instance->SR;                       /* Read status register */
        unsigned char c = huart->Instance->DR;     /* Read data register */

                    store_char (c, _rxTC_buffer);  // store data in buffer
       return;
    }


}


void reset_TimeCodeBuffer(void){
        memset(_rxTC_buffer, 0x00, sizeof(ring_buffer));     
}


int UartFSK_TC_Receive(uint8_t *buffer)
 {
       uint8_t *data = (uint8_t *)buffer;
       
       
      // int count = 0;
       uint32_t countX = 0;
         
       uint8_t countXY = 0;
       uint32_t tailX = 0;
       uint8_t start_FSK_OK = 0;
         
      
       
       uint8_t dataXtmp = 0;
         
       int32_t position = 0;
       uint32_t returnValue = 0;
         
      //   __HAL_UART_DISABLE_IT(&huart7, UART_IT_RXNE);
     
         position = _rxTC_buffer->head - _rxTC_buffer->tail;
         
        
         if(position < 0){
                position = position + UART_BUFFER_SIZE;
         }

                       if(position > 0){
                             
                             countX = position;
                             
                             
                             //UART_IntConfig(UARTPort, UART_INTCFG_RBR, DISABLE); Disable intrerupt
                             // Loop until receive buffer ring is empty or until max_bytes expires 
                               
                              tailX = _rxTC_buffer->tail;
                               
                              while (countX > 0){
                                        dataXtmp = _rxTC_buffer->buffer[tailX];
                                      
                                        if(dataXtmp == 'S' && start_FSK_OK == 0){
                                                start_FSK_OK = 1;
                                        }
                                        else if (start_FSK_OK == 1 && TC_fps_setX == 3){
                                                if (countX > 6){
                                                        countXY = 7;
                                                        break;
                                                }
                                                else {
                                                        return -1;
                                                }
                                        }
                                        else if (start_FSK_OK == 1 && TC_fps_setX == 4){
                                                if(dataXtmp == 'Y'){
                                                        if (countX > 11) {
                                                                countXY = 12;
                                                                break;
                                                        }
                                                        else {
                                                                return -1;
                                                        }
                                                }
                                                else{
                                                      fsk_errrr++;
                                                }
                                        }else{
                                                fsk_errrr++;
                                        }

                                        if(start_FSK_OK == 0){
                                                __BUF_INCR(_rxTC_buffer->tail);
                                                fsk_errrr = 0;
                                                start_FSK_OK = 0;
                                        }else if(fsk_errrr > 0){
                                                __BUF_INCR(_rxTC_buffer->tail);
                                                fsk_errrr = 0;
                                                start_FSK_OK = 0;
                                        }

                                      __BUF_INCR(tailX);
                                   
                                    // decrement buffer size count
                                    countX--;
                             }

                             //count = position - countX;
                             returnValue = countXY;
                             
                            while (countXY > 0){
                                         // Read data from ring buffer into user buffer 
                                         *data = _rxTC_buffer->buffer[_rxTC_buffer->tail];
                                         data++;				

                                   // Update tail pointer 
                                   __BUF_INCR(_rxTC_buffer->tail);
                                   
                                   // decrement buffer size count
                                   countXY--;	
                             }
                             
                             // Re-enable UART interrupts
                          //   UART_IntConfig(UARTPort, UART_INTCFG_RBR, ENABLE);
                             
                       }
                       
        //   __HAL_UART_ENABLE_IT(&huart7, UART_IT_RXNE);
               
      return returnValue;
}



void Uart_GPS_isr (UART_HandleTypeDef *huart)
{
	  uint32_t isrflags   = READ_REG(huart->Instance->SR);
	  uint32_t cr1its     = READ_REG(huart->Instance->CR1);


    /* if DR is not empty and the Rx Int is enabled */
    if (((isrflags & USART_SR_RXNE) != RESET) && ((cr1its & USART_CR1_RXNEIE) != RESET))
    {
    	 /******************
    	    	      *  @note   PE (Parity error), FE (Framing error), NE (Noise error), ORE (Overrun
    	    	      *          error) and IDLE (Idle line detected) flags are cleared by software
    	    	      *          sequence: a read operation to USART_SR register followed by a read
    	    	      *          operation to USART_DR register.
    	    	      * @note   RXNE flag can be also cleared by a read to the USART_DR register.
    	    	      * @note   TC flag can be also cleared by software sequence: a read operation to
    	    	      *          USART_SR register followed by a write operation to USART_DR register.
    	    	      * @note   TXE flag is cleared only by a write to the USART_DR register.

    	 *********************/
		huart->Instance->SR;                       /* Read status register */
        unsigned char c = huart->Instance->DR;     /* Read data register */

                    store_char (c, _rxGPS_buffer);  // store data in buffer
       return;
    }


}

uint32_t UartGPSReceive(char *buffer)
 {
       char *data = (char *)buffer;
       
       
       uint32_t count = 0;
       uint32_t bufbbb = 0;
       int32_t position = 0;
       uint32_t returnValue = 0;
         
       uint16_t tmp_head = _rxGPS_buffer->head;
       uint16_t tmp_tail = _rxGPS_buffer->tail;
     
     //    __HAL_UART_DISABLE_IT(&huart3, UART_IT_RXNE);
     
               position = getIncomingMessageSizeGPS(tmp_head, tmp_tail);
               
               if(position > 0){
                     
                     count = position;
                     returnValue = count;
                     bufbbb = count;
                     
                     //UART_IntConfig(UARTPort, UART_INTCFG_RBR, DISABLE); Disable intrerupt
                     
                     // Loop until receive buffer ring is empty or until max_bytes expires 

                     
                    while ((count > 0) && (!(__BUF_IS_EMPTY(tmp_head, _rxGPS_buffer->tail)))){
                  // while (count > 0){
		
                           if(bufbbb  < MAX_WIRELESS_MESSAGE_SIZE){
                                 // Read data from ring buffer into user buffer 
                                 *data = _rxGPS_buffer->buffer[_rxGPS_buffer->tail];
                                 data++;				
                           }else{
                                 returnValue = 0;
                           }
                           
                           // Update tail pointer 
                           __BUF_INCR(_rxGPS_buffer->tail);
                           
                           // decrement buffer size count
                           count--;	
                     }
                     
                     // Re-enable UART interrupts
                  //   UART_IntConfig(UARTPort, UART_INTCFG_RBR, ENABLE);
               }
               
           //    __HAL_UART_ENABLE_IT(&huart3, UART_IT_RXNE);
               
               return returnValue;

 }

 
 static int32_t getIncomingMessageSizeGPS(uint32_t headw, uint32_t tailw){
      uint32_t size = 0;
      uint8_t bufaaa = 0;
      

             while(!__BUF_IS_EMPTY(headw, tailw)){
                   if(_rxGPS_buffer->buffer[tailw] == 0x0A && bufaaa==0x0D)
                    {
                          size++;
                          return size;
                    }		
                   bufaaa = _rxGPS_buffer->buffer[tailw];
                   __BUF_INCR(tailw);
                   size++;
             }
             
      return -1;

}


void MX_433_TX_Init(uint32_t uart_speed)
 {
       
  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = uart_speed;
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
 

void w433_task(void)
{
        uint8_t res_valid_433Mhz = 0;
//        was_433_ok_here = 1;
        memset(rasb_433DataBuffer, 0x00, sizeof(rasb_433DataBuffer));
        
        Bytes_433_rcvd = Uart433Receive(rasb_433DataBuffer);

//        was_433_ok_here = 7;
        if(Bytes_433_rcvd > 0){
        res_valid_433Mhz = validate_433_msg();
        
//                was_433_ok_here = 8;
                                if(res_valid_433Mhz > 0){
//                                                was_433_ok_here = 9;
                                        Time_between_valid_msg = GetCurrentSystemTime() - last_msg_rcvd_here;
                                        last_msg_rcvd_here = GetCurrentSystemTime();
                                        getMS_433_Event();
//                                                was_433_ok_here = 10;
                                }else{
                                        if(Bytes_433_rcvd > 8){
                                                msg_valid_err++;
                                        }
                                }
                        }

//        was_433_ok_here = 0;
}

 

void Uart_433_isr (UART_HandleTypeDef *huart)
{
	  uint32_t isrflags   = READ_REG(huart->Instance->SR);
	  uint32_t cr1its     = READ_REG(huart->Instance->CR1);


    /* if DR is not empty and the Rx Int is enabled */
    if (((isrflags & USART_SR_RXNE) != RESET) && ((cr1its & USART_CR1_RXNEIE) != RESET))
    {
    	 /******************
    	    	      *  @note   PE (Parity error), FE (Framing error), NE (Noise error), ORE (Overrun
    	    	      *          error) and IDLE (Idle line detected) flags are cleared by software
    	    	      *          sequence: a read operation to USART_SR register followed by a read
    	    	      *          operation to USART_DR register.
    	    	      * @note   RXNE flag can be also cleared by a read to the USART_DR register.
    	    	      * @note   TC flag can be also cleared by software sequence: a read operation to
    	    	      *          USART_SR register followed by a write operation to USART_DR register.
    	    	      * @note   TXE flag is cleared only by a write to the USART_DR register.

    	 *********************/
		huart->Instance->SR;                       /* Read status register */
        unsigned char c = huart->Instance->DR;     /* Read data register */
                    store_char (c, _rx433_buffer);  // store data in buffer
                    last_msg_433Mhz_timer = GetCurrentSystemTime();
   
       return;
    }
}

/*
static int32_t getIncoming433MessageSize(uint32_t headw, uint32_t tailw){
      uint32_t size = 0;
      uint32_t tmp_tailW = 0;
        
        uint8_t bufaaa = 0;
        uint8_t bufxxx = 0;
        
        uint16_t while_exit = 0;
        
      
        tmp_tailW = tailw;
        
             while(!__BUF_IS_EMPTY(headw, tailw)){
                    if(_rx433_buffer->buffer[tmp_tailW] == START_CHAR_433MHZ_SM)
                    {
                          bufxxx = 1;
                          if(size > 0 && bufaaa == 1){
                                return size; // message was truncated
                          }
                          else{
                                       if(size == END_CHAR_POS){
                                                if(_rx433_buffer->buffer[tailw] == END_CHAR_433MHZ_SS){
                                                        bufaaa = 2;
                                                }else{
                                                        return 1; //messsage error
                                                }
                                        }else if(size == END_CHAR_POS + 1){
                                                if(bufaaa == 2){
                                                        size++;
                                                        return size;
                                                }else{
                                                        return 1; //messsage error
                                                }
                                        }
                             }
                             __BUF_INCR(tailw);
                             size++;
                    }else if(_rx433_buffer->buffer[tmp_tailW] == START_CHAR_433MHZ_AB){
                        bufxxx = 1;
                        if(size > 0 && bufaaa == 1){
                                return size; // message was truncated
                        }
                        else{
                                if(size == 31){
                                        if(_rx433_buffer->buffer[tailw] == END_CHAR_433MHZ_SS){
                                                bufaaa = 2;
                                        }else{
                                                return 1; //messsage error
                                        }
                                }else if(size == 32){
                                        if(bufaaa == 2){
                                                size++;
                                                return size;
                                        }else{
                                                return 1; //messsage error
                                        }
                                }
                        }
                        __BUF_INCR(tailw);
                        size++;
                }else{
                        __BUF_INCR(tailw);
                        tmp_tailW = tailw;
                        size++;
                        bufaaa = 1;
                    }
                
                    
                while_exit++;
                
                if(while_exit > 500){
                        break;
                }
             }
             
             if(bufxxx == 0){
                return size;
             }else{
                return -1;
             }

}
*/

uint32_t Uart433Receive(uint8_t *buffer)
 {
       uint8_t *data = (uint8_t *)buffer;
       
       uint16_t tmp_head = _rx433_buffer->head;
       uint16_t tmp_tail = _rx433_buffer->tail;
         
       int32_t count = 0;
         
     
   //  __HAL_UART_DISABLE_IT(&huart6, UART_IT_RXNE);
         
        // position = _rx433_buffer->head - _rx433_buffer->tail;
         
//         was_433_ok_here = 2;
         
         if(!(__BUF_IS_EMPTY(tmp_head, tmp_tail))){
                is_433Mhz_en = 2;
         }
         
//         was_433_ok_here = 3;
         
        uint32_t last_msg_433Mhz_timer_NOW = GetCurrentSystemTime();
        
        if((last_msg_433Mhz_timer_NOW - last_msg_433Mhz_timer > 2) && (!__BUF_IS_EMPTY(tmp_head, tmp_tail))){
                //count = getIncoming433MessageSize(tmp_head, tmp_tail);
         
//                        was_433_ok_here = 4;
                        
                        //while ((count > 0) && (!(__BUF_IS_EMPTY(tmp_head, _rx433_buffer->tail)))){
                        while (!(__BUF_IS_EMPTY(tmp_head, _rx433_buffer->tail))){

                                                 // Read data from ring buffer into user buffer 
                                              *data = _rx433_buffer->buffer[_rx433_buffer->tail];
                                               data++;				
                                           
                                           // Update tail pointer 
                                           __BUF_INCR(_rx433_buffer->tail);
                                           
                                           // decrement buffer size count
                                           //count--;	
                                        count++;	
//                                        was_433_ok_here = 5;
                                     }

                               
                     //  __HAL_UART_ENABLE_IT(&huart6, UART_IT_RXNE);
//                        was_433_ok_here = 6;
        }
        return count;
}
 
void Transmit_433_Mhz(void){
        /*
        static uint8_t outX[MAXIMUM_433_MESSAGE_SIZE + 1];
        memset(outX, 0x01, sizeof(outX));
        strcpy(outX, rasb_433DataBuffer);
        
        outX[10] = 3;
        
        HAL_UART_Transmit_IT(&huart6,(uint8_t*)outX, 32);
        */
}


void Set_433_Mhz_mod(uint8_t cmd_X){
        //set Command pin
        //433Mhz Enable
        
        HAL_GPIO_WritePin(RES_B_PORT, RES_B_PIN, GPIO_PIN_SET);
        
        MX_433_TX_Init(9600);
        
        HAL_Delay(100);
        
        HAL_UART_Transmit_IT(&huart6,(uint8_t*)defaultX_set3_433C, 12);
        
        /*
        if(cmd_X == 0){// default settings
          HAL_UART_Transmit_IT(&huart6,(uint8_t*)default_set_433, 12);
        }
        else if(cmd_X == 1){// XXX settings
          HAL_UART_Transmit_IT(&huart6,(uint8_t*)defaultX_set1_433, 12);
        }
        else if(cmd_X == 2){// XXX settings
          HAL_UART_Transmit_IT(&huart6,(uint8_t*)defaultX_set2_433, 12);
        }
        else if(cmd_X == 3){// XXX settings
          HAL_UART_Transmit_IT(&huart6,(uint8_t*)defaultX_set3_433, 12);
        }
        else if(cmd_X == 4){// XXX settings
          HAL_UART_Transmit_IT(&huart6,(uint8_t*)defaultX_set4_433, 12);
        }
        else if(cmd_X == 5){// XXX settings
          HAL_UART_Transmit_IT(&huart6,(uint8_t*)defaultX_set5_433, 12);
        }
        */
        
          HAL_Delay(100);
        
          MX_433_TX_Init(115200);
          HAL_Delay(100);
        
          HAL_GPIO_WritePin(RES_B_PORT, RES_B_PIN, GPIO_PIN_RESET);
        
        
}


//Message 433Mhz
//Byte 0 - START Communication char
        // - 0xCC - Master to ALL communication
        // - 0x99 - Module to Master communication
        // - 0x66 - Module to ALL communication
//Byte 1 - ID Commands
        //X = 0 -  NOT USED
        //X = 1 to 99 Unicast messages from controller to a specific module with ID X
        //X = 101 to 199 Unicast messages from controller to a specific module with ID X
        //X = 100 - NOT USED
        //X = 200 - NOT USED
        //X = 201 to 240 - Broadcast messages
        //X = 241 to 250 Audiobox ID
        //X = 251 to 255 - NOT USED


//Byte 2 to 29 - DATA 28 Bytes
//Byte 30 - CHECK SUM
//Byte 31 - END Char 
        // - 0x11 - Master to ALL communication

uint8_t get_msg_number(void){
        return msg_number;
}


void Wireless_433_Add_msg_to_sent(uint8_t *message)
{
     uint32_t hw_111 = ID_MCU2;
        
     message[25] = 0xFF & (hw_111 >> 24);
     message[26] = 0xFF & (hw_111 >> 16);        
     message[27] = 0xFF & (hw_111 >> 8);
     message[28] = 0xFF & (hw_111 >> 0);
     message[29] = msg_number;
     message[30] = checksum_calc(message);
     message[31] = END_CHAR_433MHZ_SS;
        
        /*
     tst_msg_fsgdAAA[tmp_pos_ms_nbr] = msg_number;   
        
        tmp_pos_ms_nbr++;
        if(tmp_pos_ms_nbr == 255){
                tmp_pos_ms_nbr = 1;
        }
        */
     msg_number++;
        if(msg_number == 255){
                msg_number = 1;
        }
        
        
        
        

/*        
     for (int i = 0; i <32; i++){
        tst_msg_fsgd[i] = message[i];
     }
        */
        
        if(buffer_433_snd[cur_433_msg_snt][0] == 0){
                for (int i = 0; i < MAXIMUM_433_MESSAGE_SIZE; i++){
                        buffer_433_snd[cur_433_msg_snt][i] = message[i];
                     }
                     msg_to_be_snt_433Mhz++;
                     
                     cur_433_msg_snt++;
                     if(cur_433_msg_snt == WIRELESS_433_BUF_SIZE){
                                cur_433_msg_snt = 0;
                     }
        }
        
     
     /*
     uint8_t buffer_433_snd[WIRELESS_433_BUF_SIZE][MAXIMUM_433_MESSAGE_SIZE] = {{0}}; 
     uint8_t last_433_msg_snt = 0;
     */
     
     //HAL_UART_Transmit_IT(&huart6,(uint8_t*)tst_msg_fsgd, 32);    
}

void wireless_433Mhz_send(void){
        if(buffer_433_snd[last_433_msg_snt][0] != 0){
                for (int i = 0; i < MAXIMUM_433_MESSAGE_SIZE; i++){
                        tst_msg_fsgdASD[i] = buffer_433_snd[last_433_msg_snt][i];
                        buffer_433_snd[last_433_msg_snt][i] = 0;
                }
                
                     if(msg_to_be_snt_433Mhz > 0){
                                msg_to_be_snt_433Mhz--;
                     }
                
                     last_433_msg_snt++;
                     if(last_433_msg_snt == WIRELESS_433_BUF_SIZE){
                                last_433_msg_snt = 0;
                     }
                     
                     msg_433_mhz_sent++;
                     HAL_UART_Transmit_IT(&huart6,(uint8_t*)tst_msg_fsgdASD, MAXIMUM_433_MESSAGE_SIZE);
                     last_msg_snt_timer = GetCurrentSystemTime();
                }else{
                        for (int i = 0; i < WIRELESS_433_BUF_SIZE; i++){
                                if(buffer_433_snd[i][0] != 0){
                                        last_433_msg_snt = i;
                                        
                                        for (int i = 0; i < MAXIMUM_433_MESSAGE_SIZE; i++){
                                                        tst_msg_fsgdASD[i] = buffer_433_snd[last_433_msg_snt][i];
                                                        buffer_433_snd[last_433_msg_snt][i] = 0;
                                                }
                                                
//                                                     msg_to_be_snt_433Mhz--;
                                                
                                                     last_433_msg_snt++;
                                                     if(last_433_msg_snt == WIRELESS_433_BUF_SIZE){
                                                                last_433_msg_snt = 0;
                                                     }
                                                     msg_433_mhz_sent++;
                                                     HAL_UART_Transmit_IT(&huart6,(uint8_t*)tst_msg_fsgdASD, MAXIMUM_433_MESSAGE_SIZE);
                                                     last_msg_snt_timer = GetCurrentSystemTime();
                     
                                        break;
                                }
                        }
                }
}

uint8_t checksum_calc(uint8_t *message){
        uint8_t Xor_CS = 0;
        for(int i = 0; i < MAXIMUM_433_MESSAGE_SIZE - 2; i++)
              {
                        Xor_CS = Xor_CS ^ message[i];
              }
              return Xor_CS;
}





uint8_t validate_433_msg(void){
        
        //if(rasb_433DataBuffer[0] == START_CHAR_433MHZ_MS || rasb_433DataBuffer[0] == START_CHAR_433MHZ_SM || rasb_433DataBuffer[0] == START_CHAR_433MHZ_SS){
          if(rasb_433DataBuffer[0] == START_CHAR_433MHZ_MS){//message received from other master
                  return 1;
          }
          else if(rasb_433DataBuffer[0] == START_CHAR_433MHZ_S1){//message settings received
                if(rasb_433DataBuffer[1] == 0x00 && rasb_433DataBuffer[2] == 0x02){ // noise read
                        uint8_t noise_level_tmp = rasb_433DataBuffer[3];
                        noise_level_433Tr = rasb_433DataBuffer[3];
                        noise_level_433 = -(256 - noise_level_tmp);
                        noise_level_tmp = rasb_433DataBuffer[4];
                        last_msg_level_433 = -(256 - noise_level_tmp);
                }
                return 0;
          }
          else if(rasb_433DataBuffer[0] == START_CHAR_433MHZ_SS){//message received from a slave should be sent to a slave
                  return 3;
          }
          else if(rasb_433DataBuffer[0] == START_CHAR_433MHZ_AB){//AB to Slaves
                  uint8_t Xor_CS_AAA = 0;
              //  uint8_t msg_size_tmpS = 0;
                
                
                for(int i = 0; i < MAXIMUM_433_MESSAGE_SIZE - 2; i++)
                {
                        Xor_CS_AAA = Xor_CS_AAA ^ rasb_433DataBuffer[i];
                }
                
                if(Xor_CS_AAA == rasb_433DataBuffer[MAXIMUM_433_MESSAGE_SIZE - 2]){
                        return 4;
                }
                else {
                        msg_433_err++;
                        return 0; //Message error
                }
          }
          else if(rasb_433DataBuffer[0] == START_CHAR_433MHZ_SM){
                
                uint8_t Xor_CS_AAA = 0;
                uint8_t msg_size_tmpS = 0;
               
                if(rasb_433DataBuffer[1] == M433_PING_AB_REPLY && rasb_433DataBuffer[2] > 200){
                        msg_size_tmpS = MAXIMUM_433_MESSAGE_SIZE;
                }else{
                        msg_size_tmpS = MAXIMUM_SM_433_MESSAGE_SIZE;
                }
                
                for(int i = 0; i < msg_size_tmpS - 2; i++)
                {
                        Xor_CS_AAA = Xor_CS_AAA ^ rasb_433DataBuffer[i];
                }
                
                if(Xor_CS_AAA == rasb_433DataBuffer[msg_size_tmpS - 2]){
                        
                        
                       set_433_wir_strength(rasb_433DataBuffer[2], rasb_433DataBuffer[msg_size_tmpS]);

                        return 2;
                }
                else {
                        msg_433_err++;
                        return 0; //Message error
                }
        }
        else {
                //  msg_433_err++;
                return 0; //Message error
        }
}


int get_mod_test_noise(void){
        return noise_level_433;
}

uint16_t get_errors_msg(void){
        return msg_433_err;
}

uint16_t get_433_msg_snt(void){
        return msg_433_mhz_sent;
}

void init_433Mhz_prg(void){

        
        uint8_t channel_433Mhz = get_433_ch();
        uint8_t netID_433Mhz = get_433_net_id(); 
        uint16_t key_433Mhz = get_433_net_key();
        
        defaultX_set3_433C[5] = netID_433Mhz;
        defaultX_set3_433C[8] = channel_433Mhz;
        
        if(channel_433Mhz < MAXIMUM_433_CHANNEL + 1){
                        defaultX_set3_433C[5] = netID_433Mhz;
                        defaultX_set3_433C[8] = channel_433Mhz;
                }
                else{
                        defaultX_set3_433C[5] = DEFAULT_433_NET_ID;
                        defaultX_set3_433C[8] = DEFAULT_433_CHANNEL;
                }
                
                if(key_433Mhz != 0 && key_433Mhz != 0xFFFF){
                        defaultX_set3_433C[10] = (key_433Mhz >> 8) & 0xFF;
                        defaultX_set3_433C[11] = (key_433Mhz >> 0) & 0xFF;
                }else{
                        defaultX_set3_433C[5] = DEFAULT_433_NET_ID;
                        defaultX_set3_433C[8] = DEFAULT_433_CHANNEL;
                        
                        defaultX_set3_433C[10] = DEFAULT_433_KEY_H;
                        defaultX_set3_433C[11] = DEFAULT_433_KEY_L;
                }
                
                        defaultX_set3_433C[7] = DEFAULT_433_PRG_EXT;
                
          HAL_GPIO_WritePin(RES_B_PORT, RES_B_PIN, GPIO_PIN_SET);
        

          
          HAL_Delay(100);
          MX_433_TX_Init(9600);
          HAL_Delay(100);
        
          HAL_UART_Transmit_IT(&huart6,(uint8_t*) defaultX_set3_433C, 12);

        
          HAL_Delay(100);
          MX_433_TX_Init(115200);
          HAL_Delay(100);
        

        
          HAL_GPIO_WritePin(RES_B_PORT, RES_B_PIN, GPIO_PIN_RESET);
        
}


void set_433_config_param(uint8_t channel_433Mhz, uint8_t netID_433Mhz, uint16_t key_433Mhz, uint8_t is_custom){
        
       if(is_custom == 0){
                //do nothing as are set default settings
                defaultX_set3_433C[5] = DEFAULT_433_NET_ID;
                defaultX_set3_433C[8] = DEFAULT_433_CHANNEL;
                defaultX_set3_433C[10] = DEFAULT_433_KEY_H;
                defaultX_set3_433C[11] = DEFAULT_433_KEY_L;
        }
        else{
                if(channel_433Mhz < MAXIMUM_433_CHANNEL + 1){
                        defaultX_set3_433C[5] = netID_433Mhz;
                        defaultX_set3_433C[8] = channel_433Mhz;
                }
                else{
                        defaultX_set3_433C[5] = DEFAULT_433_NET_ID;
                        defaultX_set3_433C[8] = DEFAULT_433_CHANNEL;
                }
                
                if(key_433Mhz != 0 && key_433Mhz != 0xFFFF){
                        defaultX_set3_433C[10] = (key_433Mhz >> 8) & 0xFF;
                        defaultX_set3_433C[11] = (key_433Mhz >> 0) & 0xFF;
                }else{
                        defaultX_set3_433C[5] = DEFAULT_433_NET_ID;
                        defaultX_set3_433C[8] = DEFAULT_433_CHANNEL;
                        
                        defaultX_set3_433C[10] = DEFAULT_433_KEY_H;
                        defaultX_set3_433C[11] = DEFAULT_433_KEY_L;
                }
        }
       
        
    
        //enter in Configuration mode
          HAL_GPIO_WritePin(RES_B_PORT, RES_B_PIN, GPIO_PIN_SET);
  
          HAL_Delay(100);
          // set UART 9600
          MX_433_TX_Init(9600);
          HAL_Delay(100);
       
        //Transmit Configuration message
          HAL_UART_Transmit_IT(&huart6,(uint8_t*) defaultX_set3_433C, 12);
      
          HAL_Delay(100);
        
        // set UART 115200
         MX_433_TX_Init(115200);
       //  HAL_Delay(100);
        
        //exit configuration mode and enter Normal mode
         HAL_GPIO_WritePin(RES_B_PORT, RES_B_PIN, GPIO_PIN_RESET);
}


void send_noise_msg(void){
        tst_msg_fsgAAAA[0] = 0xC0;
        tst_msg_fsgAAAA[1] = 0xC1;
        tst_msg_fsgAAAA[2] = 0xC2;
        tst_msg_fsgAAAA[3] = 0xC3;
        tst_msg_fsgAAAA[4] = 0x00;
        tst_msg_fsgAAAA[5] = 0x02;
        
        // send message for mabient RSSI in Normal mode
        HAL_UART_Transmit_IT(&huart6,(uint8_t*)tst_msg_fsgAAAA, 6);

}

void set_custom_433_wireles (uint8_t channel_433Mhz, uint16_t netID_433Mhz, uint16_t key_433Mhz, uint8_t is_custom_wir){

       // uint8_t tmp_msg_set_settings[32] = {0};
        

        
        if(is_custom_wir == 0){
                //do nothing as are set default settings
                defaultX_set3_433C[5] = DEFAULT_433_NET_ID;
                defaultX_set3_433C[8] = DEFAULT_433_CHANNEL;
                defaultX_set3_433C[10] = DEFAULT_433_KEY_H;
                defaultX_set3_433C[11] = DEFAULT_433_KEY_L;
        }
        else if(is_custom_wir == 100){//change only channel
                //do nothing as are set default settings
                defaultX_set3_433C[5] = DEFAULT_433_NET_ID;
                
                defaultX_set3_433C[8] = channel_433Mhz;
                
                defaultX_set3_433C[10] = DEFAULT_433_KEY_H;
                defaultX_set3_433C[11] = DEFAULT_433_KEY_L;
                
                netID_433Mhz = DEFAULT_433_NET_ID;
                key_433Mhz = DEFAULT_433_KEY_LH;
        }
        else{
                if(channel_433Mhz < MAXIMUM_433_CHANNEL + 1){
                        defaultX_set3_433C[5] = netID_433Mhz;
                        defaultX_set3_433C[8] = channel_433Mhz;
                }
                else{
                        defaultX_set3_433C[5] = DEFAULT_433_NET_ID;
                        defaultX_set3_433C[8] = DEFAULT_433_CHANNEL;
                }
                
                if(key_433Mhz != 0 && key_433Mhz != 0xFFFF){
                        defaultX_set3_433C[10] = (key_433Mhz >> 8) & 0xFF;
                        defaultX_set3_433C[11] = (key_433Mhz >> 0) & 0xFF;
                }else{
                        defaultX_set3_433C[10] = DEFAULT_433_KEY_H;
                        defaultX_set3_433C[11] = DEFAULT_433_KEY_L;
                }
        }
        
      
               // HAL_Delay(100);
                
       if(is_custom_wir != 0){
               
                msg_433_tmpA[0] = START_CHAR_433MHZ_MS;
                msg_433_tmpA[1] = M433_SETTINGS;
                msg_433_tmpA[2] = M433_BROADCAST;
        
                for(int i = 0; i < 12; i++){
                        msg_433_tmpA[i+3] = defaultX_set3_433C[i];
                }
          
                Wireless_433_Add_msg_to_sent(msg_433_tmpA);
                Wireless_433_Add_msg_to_sent(msg_433_tmpA);
                Wireless_433_Add_msg_to_sent(msg_433_tmpA);
        }
                set_save_433wir_config(channel_433Mhz, netID_433Mhz, key_433Mhz, is_custom_wir);
}


void update_433Mhz_settings(void){
        
          HAL_GPIO_WritePin(RES_B_PORT, RES_B_PIN, GPIO_PIN_SET);
          
          HAL_Delay(100);
          MX_433_TX_Init(9600);
          HAL_Delay(100);
        
          HAL_UART_Transmit_IT(&huart6,(uint8_t*) defaultX_set3_433C, 12);

        
          HAL_Delay(100);
          MX_433_TX_Init(115200);
          HAL_Delay(50);
        
          HAL_GPIO_WritePin(RES_B_PORT, RES_B_PIN, GPIO_PIN_RESET);
          
}


void set_custom_433_wireless(void){
        
        for(int i = 0; i < 12; i++){
                defaultX_set3_433C[i] = rasb_433DataBuffer[i+3];
                        
        }
        
        /*
                for(int i = 0; i < 12; i++){
                        if(i == 7){
                                defaultX_set3_433C[i] = 0xA3;
                        }else{
                                defaultX_set3_433C[i] = rasb_433DataBuffer[i+3];
                        }
                }
                
                
        
          HAL_GPIO_WritePin(RES_B_PORT, RES_B_PIN, GPIO_PIN_SET);
        
          MX_433_TX_Init(9600);
       
          HAL_Delay(100);
        
          HAL_UART_Transmit_IT(&huart6,(uint8_t*) defaultX_set3_433C, 12);

        
          HAL_Delay(100);
        
          //MX_433_TX_Init(115200);
          MX_433_TX_Init(9600);
          HAL_Delay(100);
        
          HAL_GPIO_WritePin(RES_B_PORT, RES_B_PIN, GPIO_PIN_RESET);
               */         
}


void Change_wireless_settings(void){
 
          HAL_GPIO_WritePin(RES_B_PORT, RES_B_PIN, GPIO_PIN_SET);
        
          MX_433_TX_Init(9600);
       
          HAL_Delay(100);
        
          HAL_UART_Transmit_IT(&huart6,(uint8_t*) defaultX_set3_433C, 12);

        
          HAL_Delay(100);
        
          MX_433_TX_Init(115200);
          HAL_Delay(100);
        
          HAL_GPIO_WritePin(RES_B_PORT, RES_B_PIN, GPIO_PIN_RESET);

}


void uart_DMX_send(void){
        if (!LL_DMA_IsEnabledStream(DMA1, LL_DMA_STREAM_0))
        {

                 //   update_buffer_DMX();
              
  
                    LL_DMA_ClearFlag_TC0(DMA1);
                    LL_DMA_ClearFlag_HT0(DMA1);
                    LL_DMA_ClearFlag_DME0(DMA1);
                    LL_DMA_ClearFlag_FE0(DMA1);
                    LL_DMA_ClearFlag_TE0(DMA1);
                
                    LL_DMA_EnableStream(DMA1, LL_DMA_STREAM_0);
        }
}


void run_DMX(void){
    if(get_device_Status() > 2){ // if is in armed/play/pause
         if(are_any_DMX_Evants() == 1){
            if(DMX_stat == 0){
                    clear_dmx_timer_irq();
                    DMX_stat = 1;
                   // DMX_stateX = 0;
                    usart_start_tx_dma();
                    start_dmx_timer();
            }
        }
    }else if(DMX_stat != 0){
        DMX_stat = 0;
    }
}

void run_DMX_OLD(void){
    if(are_any_DMX_Evants() == 1){
            if(DMX_stat == 0){
                    buf_DMX[1] =0xFF;
                    buf_DMX[2] =0xFF;
                    buf_DMX[3] =0xFF;
                    buf_DMX[4] =0xFF;
                    
                    clear_dmx_timer_irq();
                    DMX_stat = 1;
                    DMX_stateX = 0;
                    usart_start_tx_dma();
                    start_dmx_timer();
  
            }else{
                    DMX_stateX = ~DMX_stateX;
            }
    }
}

void Stop_DMX(void){
    DMX_stat = 0;
}




/**
  * @brief This function handles DMA1 stream0 global interrupt.
  */
void DMA1_Stream0_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream0_IRQn 0 */

  /* USER CODE END DMA1_Stream0_IRQn 0 */
 // HAL_DMA_IRQHandler(&hdma_uart8_tx);
      
 // LL_DMA_DisableStream(DMA1, LL_DMA_STREAM_0);
  LL_DMA_ClearFlag_TC0(DMA1);

  /* USER CODE BEGIN DMA1_Stream0_IRQn 1 */
  // HAL_TIM_Base_Start_IT(&htim6);
  /* USER CODE END DMA1_Stream0_IRQn 1 */
}


// Provide the DMA peripheral with the ringbuffer output data
void usart_start_tx_dma( void )
{
   // uint32_t primask;
    uint16_t usart_dma_tx_len;

   // primask = __get_PRIMASK();
   // __disable_irq();

    /* If transfer is not on-going */
    if (!LL_DMA_IsEnabledStream(DMA1, LL_DMA_STREAM_0))
    {
            usart_dma_tx_len = DMX_CH_MAX;

           // void* ptr = buf_DMX;
             void* ptr = buf_DMX_DMA;
            

            /* Configure DMA */
            LL_DMA_SetDataLength(DMA1, LL_DMA_STREAM_0, usart_dma_tx_len);
            LL_DMA_SetMemoryAddress(DMA1, LL_DMA_STREAM_0, (uint32_t)ptr);

            /* Clear all flags */
            LL_DMA_ClearFlag_TC0(DMA1);
            LL_DMA_ClearFlag_HT0(DMA1);
            LL_DMA_ClearFlag_DME0(DMA1);
            LL_DMA_ClearFlag_FE0(DMA1);
            LL_DMA_ClearFlag_TE0(DMA1);

            /* Start transfer */
            LL_DMA_EnableStream(DMA1, LL_DMA_STREAM_0);
    }

  //  __set_PRIMASK(primask);
}


void MX_UART8_Init_LL_PINs(void)
{

    LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

    /**UART8 GPIO Configuration
    PE0     ------> UART8_RX
    PE1     ------> UART8_TX
    */
    GPIO_InitStruct.Pin = LL_GPIO_PIN_0|LL_GPIO_PIN_1;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
    GPIO_InitStruct.Alternate = LL_GPIO_AF_8;
    LL_GPIO_Init(GPIOE, &GPIO_InitStruct);

}


void MX_UART8_InitLL(void)
{
 // LL setup
    LL_USART_InitTypeDef USART_InitStruct = {0};
    LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* Peripheral clock enable */
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_UART8);
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOD);
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA1);

     /**UART8 GPIO Configuration
    PE0     ------> UART8_RX
    PE1     ------> UART8_TX
    */
    GPIO_InitStruct.Pin = LL_GPIO_PIN_0|LL_GPIO_PIN_1;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
    GPIO_InitStruct.Alternate = LL_GPIO_AF_8;
    LL_GPIO_Init(GPIOE, &GPIO_InitStruct);


    /* UART8 DMA Init */
    /* UART8_TX Init */
    
        /* USART3 DMA Init TX */
    LL_DMA_SetChannelSelection(DMA1, LL_DMA_STREAM_0, LL_DMA_CHANNEL_5);
    LL_DMA_SetDataTransferDirection(DMA1, LL_DMA_STREAM_0, LL_DMA_DIRECTION_MEMORY_TO_PERIPH);
    LL_DMA_SetStreamPriorityLevel(DMA1, LL_DMA_STREAM_0, LL_DMA_PRIORITY_MEDIUM);
    LL_DMA_SetMode(DMA1, LL_DMA_STREAM_0, LL_DMA_MODE_NORMAL);
    LL_DMA_SetPeriphIncMode(DMA1, LL_DMA_STREAM_0, LL_DMA_PERIPH_NOINCREMENT);
    LL_DMA_SetMemoryIncMode(DMA1, LL_DMA_STREAM_0, LL_DMA_MEMORY_INCREMENT);
    LL_DMA_SetPeriphSize(DMA1, LL_DMA_STREAM_0, LL_DMA_PDATAALIGN_BYTE);
    LL_DMA_SetMemorySize(DMA1, LL_DMA_STREAM_0, LL_DMA_MDATAALIGN_BYTE);
    LL_DMA_DisableFifoMode(DMA1, LL_DMA_STREAM_0);

    LL_DMA_SetPeriphAddress(DMA1, LL_DMA_STREAM_0, (uint32_t)&UART8->DR);
    LL_DMA_EnableIT_TC(DMA1, LL_DMA_STREAM_0); /* Enable TX TC interrupt */

    /* DMA interrupt init */
     NVIC_SetPriority(DMA1_Stream0_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 5, 1));
  //  NVIC_SetPriority(DMA1_Stream0_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 3, 1));
    NVIC_EnableIRQ(DMA1_Stream0_IRQn);




    /* USART configuration */
    USART_InitStruct.BaudRate = 250000;
    USART_InitStruct.DataWidth = LL_USART_DATAWIDTH_8B;
    USART_InitStruct.StopBits = LL_USART_STOPBITS_2;
    USART_InitStruct.Parity = LL_USART_PARITY_NONE;
    USART_InitStruct.TransferDirection = LL_USART_DIRECTION_TX;
    USART_InitStruct.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
    USART_InitStruct.OverSampling = LL_USART_OVERSAMPLING_16;
    LL_USART_Init(UART8, &USART_InitStruct);

    LL_USART_ConfigAsyncMode(UART8);
    LL_USART_EnableDMAReq_TX(UART8);
 
    /* USART interrupt */
    NVIC_SetPriority(UART8_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 5, 0));
 //   NVIC_SetPriority(UART8_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 3, 1));
    NVIC_EnableIRQ(UART8_IRQn);

  // memset(&buf_DMX, 0x00, sizeof(buf_DMX));
 //  void* ptr = buf_DMX;
  
   memset(&buf_DMX_DMA, 0x00, sizeof(buf_DMX_DMA));
   void* ptr = buf_DMX_DMA;
  
  
    

   /* Configure DMA */
   LL_DMA_SetDataLength(DMA1, LL_DMA_STREAM_0, DMX_CH_MAX);
   LL_DMA_SetMemoryAddress(DMA1, LL_DMA_STREAM_0, (uint32_t)ptr);


    /* Enable USART */
    LL_DMA_EnableStream(DMA1, LL_DMA_STREAM_1);     // rx stream
    LL_USART_Enable(UART8);
}


void MX_TIM6_Init_LL(void){
  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  LL_TIM_InitTypeDef TIM_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM6);

  /* TIM6 interrupt Init */
  NVIC_SetPriority(TIM6_DAC_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),2, 0));
  //NVIC_SetPriority(TIM6_DAC_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 2));
  NVIC_EnableIRQ(TIM6_DAC_IRQn);

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  TIM_InitStruct.Prescaler = 0;
  //TIM_InitStruct.Prescaler = 139;
  TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
  TIM_InitStruct.Autoreload = 1175;
  //TIM_InitStruct.Autoreload = 59999;
  
  LL_TIM_Init(TIM6, &TIM_InitStruct);
  LL_TIM_SetOnePulseMode(TIM6, LL_TIM_ONEPULSEMODE_SINGLE);
  LL_TIM_SetUpdateSource(TIM6, LL_TIM_UPDATESOURCE_COUNTER);
  //LL_TIM_EnableARRPreload(TIM6);
  LL_TIM_DisableARRPreload(TIM6);
  LL_TIM_SetTriggerOutput(TIM6, LL_TIM_TRGO_RESET);
  LL_TIM_DisableMasterSlaveMode(TIM6);
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}



void MX_TIM6_Init_LL_XX(uint16_t presc, uint16_t Arr_x){
  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  LL_TIM_InitTypeDef TIM_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM6);

  /* TIM6 interrupt Init */
  NVIC_SetPriority(TIM6_DAC_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),2, 0));
  //NVIC_SetPriority(TIM6_DAC_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 2));
  NVIC_EnableIRQ(TIM6_DAC_IRQn);

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  TIM_InitStruct.Prescaler = presc;
  TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
  TIM_InitStruct.Autoreload = Arr_x;
  LL_TIM_Init(TIM6, &TIM_InitStruct);
  //LL_TIM_EnableARRPreload(TIM6);
  LL_TIM_SetOnePulseMode(TIM6, LL_TIM_ONEPULSEMODE_SINGLE);
  LL_TIM_DisableARRPreload(TIM6);
  LL_TIM_SetTriggerOutput(TIM6, LL_TIM_TRGO_RESET);
  LL_TIM_SetUpdateSource(TIM6, LL_TIM_UPDATESOURCE_COUNTER);
  LL_TIM_DisableMasterSlaveMode(TIM6);
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */
}

void start_dmx_timer(void){
      //  LL_TIM_EnableCounter(TIM6);
//        LL_TIM_EnableUpdateEvent(TIM6);
        LL_TIM_EnableIT_UPDATE(TIM6);
}


void clear_dmx_timer_irq(void){
        LL_TIM_ClearFlag_UPDATE(TIM6);
        LL_TIM_EnableCounter(TIM6);
        LL_TIM_EnableIT_UPDATE(TIM6);
        LL_TIM_SetCounter(TIM6, 0);
}



/**
  * @brief This function handles TIM7 global interrupt.
  */
void TIM6_DAC_IRQHandler(void)
{
 clear_dmx_timer_irq();   
        
  if(DMX_stat == 0){// DMX Disable
        //stop timer
        LL_TIM_DisableCounter(TIM6);
  }else if(DMX_stat == 1){//Start pause break(low TX line) of at least 88us
          DMX_stat = 2;
          set_rst_DMX_Line(0);
          //MX_TIM6_Init_LL_XX(0,7391); //about 88 us... BLINK if set
         // MX_TIM6_Init_LL_XX(0,10000); // need mroe time otherwise dmx does nwot work well... it blinks 
          MX_TIM6_Init_LL_XX(0,15000); // need mroe time otherwise dmx does nwot work well... it blinks 
          LL_TIM_EnableCounter(TIM6);
          return;
  }else if(DMX_stat == 2){//Start Mark of at least 12 us
          DMX_stat = 3;
          set_rst_DMX_Line(1);
          //MX_TIM6_Init_LL_XX(0,1007);  //about 12 us ... BLINK if set
          //MX_TIM6_Init_LL_XX(0,1500);  // need mroe time otherwise dmx does nwot work well.. it blinks
          MX_TIM6_Init_LL_XX(0,2000);  // need mroe time otherwise dmx does nwot work well.. it blinks
          LL_TIM_EnableCounter(TIM6);
          return;
  }else if(DMX_stat == 3){//Start sending data
         // DMX_stat = 1;
           DMX_stat = 4;
          MX_UART8_Init_LL_PINs();
          uart_DMX_send();
          MX_TIM6_Init_LL_XX(50,29999);  
        //  MX_TIM6_Init_LL_XX(50,29999);  
          LL_TIM_EnableCounter(TIM6);
          return;
  }else if(DMX_stat == 4){//Start sending data
          DMX_stat = 1;
          MX_TIM6_Init_LL_XX(50,29999);  
          LL_TIM_EnableCounter(TIM6);
          return;
  }     
          
    

}


void set_time_code_read_FSK_PD(void){
//Disable SMPTE
  Disable_TC_SMPTE_Gen_Pin();//Disable SMPTE Gen pin
  Disable_TC_SMPTE_Read_Pin();//Enable SMPTE Read pin
  init_time_code();

    TC_fps_setX = 3;        
        
        //Start read FSK PD

    UartFSK_CMD_Mode();
    HAL_Delay(100);
    UartFSK_PD_set(0);
    HAL_Delay(1000);
    UartFSK_ATX_set(1);
    HAL_Delay(1000);    
        
        /*
   UartFSK_PD_setNew(0);
   HAL_Delay(500);
   UartFSK_ATX_set(1);
   HAL_Delay(500);
        */

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

void set_time_code_read_FSK_F1(void){
//Disable SMPTE
 Disable_TC_SMPTE_Gen_Pin();//Disable SMPTE Gen pin
 Disable_TC_SMPTE_Read_Pin();//Enable SMPTE Read pin
 init_time_code();
        
 TC_fps_setX = 4;              
        
   
 UartFSK_CMD_Mode();
 HAL_Delay(100);
 UartFSK_F1_set();
 HAL_Delay(1000);
 UartFSK_ATX_set(1);
 HAL_Delay(1000);    
        
        
 /*
  UartFSK_F1_setNew();
  HAL_Delay(500);
  UartFSK_ATX_set(1);
  HAL_Delay(500);
  UartFSK_ATX_set(0);
  HAL_Delay(500);
        */
        
 /*       
 UartFSK_CMD_Mode();
 HAL_Delay(100);
 UartFSK_F1_set();
 HAL_Delay(500);
 UartFSK_CMD_ModeX();
 HAL_Delay(100);
 UartFSK_ATX_set(1);
 HAL_Delay(500);
 */       
}

void set_time_code_Gen_FSK_PD(void){
//Disable SMPTE
 Disable_TC_SMPTE_Gen_Pin();//Disable SMPTE Gen pin
 Disable_TC_SMPTE_Read_Pin();//Enable SMPTE Read pin
 init_time_code();
        
 TC_fps_setX = 30;
 UartFSK_CMD_Mode();
 HAL_Delay(100);
 UartFSK_PD_set(1);
 HAL_Delay(500);
 UartFSK_CMD_ModeX();
 HAL_Delay(100);       
 UartFSK_ATX_set(1);
 HAL_Delay(500);

}

void set_time_code_Gen_FSK_F1(void){
//Disable SMPTE
//Disable SMPTE
         Disable_TC_SMPTE_Gen_Pin();//Disable SMPTE Gen pin
         Disable_TC_SMPTE_Read_Pin();//Enable SMPTE Read pin
         init_time_code();
                
         TC_fps_setX = 40;
         UartFSK_CMD_Mode();
         HAL_Delay(100);
         UartFSK_F1_set();
         HAL_Delay(500);
         UartFSK_CMD_ModeX();
         HAL_Delay(100);
         UartFSK_ATX_set(0);
         HAL_Delay(500);        
}

void do_FSK_TimeCode(void){
        
        if(get_Time_CodeX() == 6 ||  get_Time_CodeX() == 7){
                fsk_gen_Timer();
         }
        else if(get_Time_CodeX() == 2 ||  get_Time_CodeX() == 3){
                read_gen_FSK_Time();
        }
}


static void sendStringMessageToGPS(const char *message, uint32_t messageSize){
	static char outgps[128];
        const char textEnd[]={(char)0x0D, (char)0x0A};
	memset(outgps, 0x00, sizeof(outgps));

		/* copy to buffer */
		strcpy(outgps, message);
		/* append line end */
		strcat(outgps,textEnd);		

                HAL_UART_Transmit_IT(&huart3, (uint8_t *)outgps, sizeof(outgps));

}

void GPS_Update_Settings(uint8_t settings_X){
        
   const char *cfgGPSRMK = "$PMTK314,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29"; //update what messages to be sent
   const char *cfgGPS_PPS = "$PMTK255,1*2D"; //Enable module PPS
  // const char *cfgGPS_PPScfg = "$PMTK285,1,100*3D"; //Sonfigure PPS Signal
   const char *cfgGPS_PPScfg = "$PMTK285,2,100*3E"; //Sonfigure PPS Signal

   const char *cfgGPS115 = "$PMTK251,115200*1F"; //set module speed to 115200         
        
        uint32_t t_span = 0;
        if(settings_X == 0){
                sendStringMessageToGPS(cfgGPSRMK, (uint32_t)strlen(cfgGPSRMK));
                 t_span = GetCurrentSystemTime();
        
                while ((GetCurrentSystemTime() - t_span) < 200u)
                {
                        gps_task();
                }
                
        }else if(settings_X == 1){
                 t_span = GetCurrentSystemTime();
                sendStringMessageToGPS(cfgGPS_PPS, (uint32_t)strlen(cfgGPS_PPS));
                
                while ((GetCurrentSystemTime() - t_span) < 500u)
                {
                      
                        gps_task();
                }

                
        }else if(settings_X == 2){
                 t_span = GetCurrentSystemTime();
                sendStringMessageToGPS(cfgGPS_PPScfg, (uint32_t)strlen(cfgGPS_PPScfg));
              
                while ((GetCurrentSystemTime() - t_span) < 500u)
                {
                        gps_task();
                }
                

        }else if(settings_X == 3){
                 t_span = GetCurrentSystemTime();
                sendStringMessageToGPS(cfgGPS115, (uint32_t)strlen(cfgGPS115));
               
                while ((GetCurrentSystemTime() - t_span) < 500u)
                {
                        gps_task();
                }
              

        }
}



void rst_433Rx_buffer(void){
        memset(rasb_433DataBuffer, 0x00, sizeof(rasb_433DataBuffer));
        
        Bytes_433_rcvd = Uart433Receive(rasb_433DataBuffer);
        Bytes_433_rcvd = Uart433Receive(rasb_433DataBuffer);
        Bytes_433_rcvd = Uart433Receive(rasb_433DataBuffer);
        
        memset(rasb_433DataBuffer, 0x00, sizeof(rasb_433DataBuffer));
        
        
}


