/**
  */ 
  
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __UART_H__
#define __UART_H__

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "stdio.h"

/* User can use this section to tailor USARTx/UARTx instance used and associated 
   resources */
/* Definition for USARTx clock resources */
#if 0
#define USARTx                           USART6
#define USARTx_CLK_ENABLE()              __HAL_RCC_USART6_CLK_ENABLE();
#define USARTx_RX_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOC_CLK_ENABLE()
#define USARTx_TX_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOC_CLK_ENABLE() 

#define USARTx_FORCE_RESET()             __HAL_RCC_USART6_FORCE_RESET()
#define USARTx_RELEASE_RESET()           __HAL_RCC_USART6_RELEASE_RESET()

/* Definition for USARTx Pins */
#define USARTx_TX_PIN                    GPIO_PIN_6
#define USARTx_TX_GPIO_PORT              GPIOC  
#define USARTx_TX_AF                     GPIO_AF8_USART6
#define USARTx_RX_PIN                    GPIO_PIN_7
#define USARTx_RX_GPIO_PORT              GPIOC 
#define USARTx_RX_AF                     GPIO_AF8_USART6

/* Definition for USARTx's NVIC */
#define USARTx_IRQn                      USART6_IRQn
#define USARTx_IRQHandler                USART6_IRQHandler
#endif

#if 0
#define USARTx                           UART4
#define USARTx_CLK_ENABLE()              __HAL_RCC_UART4_CLK_ENABLE();
#define USARTx_RX_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOA_CLK_ENABLE()
#define USARTx_TX_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOA_CLK_ENABLE() 

#define USARTx_FORCE_RESET()             __HAL_RCC_UART4_FORCE_RESET()
#define USARTx_RELEASE_RESET()           __HAL_RCC_UART4_RELEASE_RESET()

/* Definition for USARTx Pins */
#define USARTx_TX_PIN                    GPIO_PIN_0
#define USARTx_TX_GPIO_PORT              GPIOA
#define USARTx_TX_AF                     GPIO_AF8_UART4
#define USARTx_RX_PIN                    GPIO_PIN_1
#define USARTx_RX_GPIO_PORT              GPIOA
#define USARTx_RX_AF                     GPIO_AF8_UART4

/* Definition for USARTx's NVIC */
#define USARTx_IRQn                      UART4_IRQn
#define USARTx_IRQHandler                UART4_IRQHandler
#endif



/**
@def __BUF_MASK
@brief Buffer mask
*/
#define __BUF_MASK (UART_BUFFER_SIZE-1)

/**
@def __BUF_IS_FULL
@brief Check is buffer full
*/
#define __BUF_IS_FULL(head, tail) ((tail&__BUF_MASK)==((head+1)&__BUF_MASK))

/**
@def __BUF_WILL_FULL
@brief Check if buffer will be full on next receive
*/
#define __BUF_WILL_FULL(head, tail) ((tail&__BUF_MASK)==((head+2)&__BUF_MASK))

/**
@def __BUF_AVAILABLE
@brief Check is there any available space in the buffer
*/
#define __BUF_AVAILABLE(head, tail, dataSize) ((tail&__BUF_MASK)!=((head+(dataSize+1))&__BUF_MASK))

/**
@def __BUF_IS_EMPTY
@brief Check is buffer empty
*/
#define __BUF_IS_EMPTY(head, tail) ((head&__BUF_MASK)==(tail&__BUF_MASK))

/* Reset buf */
/**
@def __BUF_RESET
@brief Reset buffer index
*/
#define __BUF_RESET(bufidx)	(bufidx=0)

/**
@def __BUF_INCR
@brief Increment buffer index
*/
#define __BUF_INCR(bufidx)	(bufidx=(bufidx+1)&__BUF_MASK)


#define MAX_WIRELESS_MESSAGE_SIZE 256  
/* change the size of the buffer */
#define UART_BUFFER_SIZE 512 // to put 1024


/* Exported functions ------------------------------------------------------- */
void uart_init(void);
int fputc(int ch, FILE *f);
int fgetc(FILE *f);

uint8_t uart_get_char(char *data);



typedef struct
{
  volatile unsigned char buffer[UART_BUFFER_SIZE];
  volatile unsigned int head;
  volatile unsigned int tail;
} ring_buffer;


/* Initialize the ring buffer */
void Ringbuf_init(void);
void Ringbuf_init1(void);
void RingbufTC_init(void);

void RingbufGPS_init(void);
void Ringbuf433_init(void);

/* reads the data in the rx_buffer and increment the tail count in rx_buffer */
int Uart_read(void);

uint32_t UartWirReceive(uint8_t *buffer);
uint32_t UartWirSend(const uint8_t *txbuf, uint8_t buflen);

/* writes the data to the tx_buffer and increment the head count in tx_buffer */
void Uart_write(int c);

/* Checks if the data is available to read in the rx_buffer */
int IsDataAvailable(void);

/* Resets the entire ring buffer, the new data will start from position 0 */
void Uart_flush (void);
void Uart_wir_flush (void);

/* the ISR for the uart. put it in the IRQ handler */
void Uart_wir_isr (UART_HandleTypeDef *huart);
void Uart_TC_isr (UART_HandleTypeDef *huart);

void Uart_GPS_isr (UART_HandleTypeDef *huart);
uint32_t UartGPSReceive(char *buffer);

void Uart_433_isr (UART_HandleTypeDef *huart);
void Transmit_433_Mhz(void);


uint32_t Uart433Receive(uint8_t *buffer);
void Set_433_Mhz_mod(uint8_t cmd_X);
void MX_433_TX_Init(uint32_t uart_speed);
void w433_task(void);

void Wireless_433_Add_msg_to_sent(uint8_t *message);
uint8_t checksum_calc(uint8_t *message);
int UartFSK_TC_Receive(uint8_t *buffer);
void wireless_433Mhz_send(void);
uint8_t validate_433_msg(void);
uint8_t get_msg_number(void);
void set_custom_433_wireles (uint8_t channel_433Mhz, uint16_t netID_433Mhz, uint16_t key_433Mhz, uint8_t is_custom_wir);
void set_433_config_param(uint8_t channel_433Mhz, uint8_t netID_433Mhz, uint16_t key_433Mhz, uint8_t is_custom);
void init_433Mhz_prg(void);

uint16_t get_errors_msg(void);
uint16_t get_433_msg_snt(void);
void set_custom_433_wireless(void);
void uart_DMX_send(void);
void usart_start_tx_dma( void );
void MX_UART8_InitLL(void);
void MX_UART8_Init_LL_PINs(void);
void MX_TIM6_Init_LL(void);
void start_dmx_timer(void);
void clear_dmx_timer_irq(void);
void MX_TIM6_Init_LL_XX(uint16_t presc, uint16_t Arr_x);

void set_time_code_read_FSK_PD(void);
void set_time_code_read_FSK_F1(void);
void set_time_code_Gen_FSK_F1(void);
void set_time_code_Gen_FSK_PD(void);
void Disbale_FSK(void);
void do_FSK_TimeCode(void);
void run_DMX(void);
void Stop_DMX(void);

void Change_wireless_settings(void);

void GPS_Update_Settings(uint8_t settings_X);
void rst_433Rx_buffer(void);

void reset_TimeCodeBuffer(void);
#endif /* __UART_H__ */
