/**
  ******************************************************************************
  * @file    Audio/Audio_playback_and_record/Src/stm32f4xx_it.c 
  * @author  MCD Application Team
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and 
  *          peripherals interrupt service routine.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2017 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */  

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "uart.h"
#include "SystemTimer.h"

/** @addtogroup STM32F4-Discovery_Audio_Player_Recorder
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
extern SD_HandleTypeDef uSdHandle;

extern PCD_HandleTypeDef hpcd_USB_OTG_FS;

//uint32_t Difference_433 = 0;

/* Timer used for LED blinking (defined in main.c */
extern TIM_HandleTypeDef hTimLed;

extern I2S_HandleTypeDef       hAudioOutI2s;

extern HCD_HandleTypeDef hhcd;

/* UART handler declared in "main.c" file */
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart4;
extern UART_HandleTypeDef huart6;
extern UART_HandleTypeDef huart7;


extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim5;
extern TIM_HandleTypeDef htim6;


/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M4 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief  This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
//  __ASM volatile("BKPT #01");
  while (1)
  {
  }
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void SVC_Handler(void)
{
}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{
}

/**
  * @brief  This function handles PendSVC exception.
  * @param  None
  * @retval None
  */
void PendSV_Handler(void)
{
}

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */

void SysTick_Handler(void)
{
  HAL_IncTick();
  SysTimer();
}


/******************************************************************************/
/*                 STM32F4xx Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f40xx.s).                                              */
/******************************************************************************/

/**
  * @brief  This function handles External line 0 interrupt request.
  * @param  None
  * @retval None
  */
void EXTI0_IRQHandler(void)
{
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_0);
}


/**
  * @brief This function handles EXTI line1 interrupt.
  */
void EXTI1_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI1_IRQn 0 */

  /* USER CODE END EXTI1_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_1);
  /* USER CODE BEGIN EXTI1_IRQn 1 */

  /* USER CODE END EXTI1_IRQn 1 */
}

/**
  * @brief This function handles EXTI line[15:10] interrupts.
  */
void EXTI15_10_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI15_10_IRQn 0 */

  /* USER CODE END EXTI15_10_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_15);
  /* USER CODE BEGIN EXTI15_10_IRQn 1 */

  /* USER CODE END EXTI15_10_IRQn 1 */
}


/**
  * @brief  This function handles main I2S interrupt. 
  * @param  None
  * @retval 0 if correct communication, else wrong communication
  */
void I2S3_IRQHandler(void)
{ 
  HAL_DMA_IRQHandler(hAudioOutI2s.hdmatx);
}

/**
  * @brief This function handles TIM4 global interrupt.
  */
void TIM4_IRQHandler(void)
{
  /* USER CODE BEGIN TIM4_IRQn 0 */

  /* USER CODE END TIM4_IRQn 0 */
// Difference_433 = HAL_TIM_ReadCapturedValue(&htim4, TIM_CHANNEL_3);  // read second value
  HAL_TIM_IRQHandler(&htim4);
         
        
  //HAL_GPIO_TogglePin(DI_GPIO_Port, DI_Pin);
  //start_Buz_PWM();
  /* USER CODE BEGIN TIM4_IRQn 1 */

  /* USER CODE END TIM4_IRQn 1 */
}



/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles USB On The Go FS global interrupt.
  */
void OTG_FS_IRQHandler(void)
{
  /* USER CODE BEGIN OTG_FS_IRQn 0 */

  /* USER CODE END OTG_FS_IRQn 0 */
 
        // HAL_PCD_IRQHandler(&hpcd_USB_OTG_FS); // For USB Device
   
        HAL_HCD_IRQHandler(&hhcd); // for USB Host
        
  /* USER CODE BEGIN OTG_FS_IRQn 1 */

  /* USER CODE END OTG_FS_IRQn 1 */
}



/**
  * @brief  This function handles UART interrupt request.  
  * @param  None
  * @retval None
  * @Note   This function is redefined in "main.h" and related to DMA stream 
  *         used for USART data transmission     
  */
void UART4_IRQHandler(void)
{

        HAL_UART_IRQHandler(&huart4);
    

}


void USART1_IRQHandler(void)
{
  Uart_wir_isr(&huart1);    
  HAL_UART_IRQHandler(&huart1);
}


void UART7_IRQHandler(void)
{
  Uart_TC_isr(&huart7);    
  HAL_UART_IRQHandler(&huart7);
}

void USART3_IRQHandler(void)
{
  Uart_GPS_isr(&huart3);    
  HAL_UART_IRQHandler(&huart3);
}

void USART6_IRQHandler(void)
{
        /*
  HAL_NVIC_DisableIRQ(USART6_IRQn);
        Uart_433_isr(&huart6);    
  HAL_NVIC_EnaableIRQ(USART6_IRQn);
        */
  Uart_433_isr(&huart6);  
            
  HAL_UART_IRQHandler(&huart6);
}

/**
  * @brief  This function handles DMA2 Stream 3 interrupt request.
  * @param  None
  * @retval None
  */
void DMA2_Stream3_IRQHandler(void)
{
  HAL_DMA_IRQHandler(uSdHandle.hdmarx);
}

/**
  * @brief  This function handles DMA2 Stream 6 interrupt request.
  * @param  None
  * @retval None
  */
void DMA2_Stream6_IRQHandler(void)
{
  HAL_DMA_IRQHandler(uSdHandle.hdmatx);
}

/**
  * @brief  This function handles SDIO interrupt request.
  * @param  None
  * @retval None
  */
void SDIO_IRQHandler(void)
{
  HAL_SD_IRQHandler(&uSdHandle);
}



//void EXTI15_10_IRQHandler(void)
void EXTI9_5_IRQHandler(void)
{

  if(__HAL_GPIO_EXTI_GET_IT(GPS_PPS_PIN) != RESET)
  {
          __HAL_GPIO_EXTI_CLEAR_IT(GPS_PPS_PIN);
          run_pps_function();
   
  }
//   __HAL_GPIO_EXTI_CLEAR_IT(GPS_PPS_PIN);
  // run_pps_function();
  

}

/**
  * @}
  */ 
  
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
