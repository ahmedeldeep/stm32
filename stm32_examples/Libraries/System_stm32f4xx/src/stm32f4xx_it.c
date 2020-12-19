/**
  ******************************************************************************
  * @file    Project/STM32F4xx_StdPeriph_Template/stm32f4xx_it.c 
  * @author  MCD Application Team
  * @version V1.1.0
  * @date    18-January-2013
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and 
  *          peripherals interrupt service routine.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2013 STMicroelectronics</center></h2>
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
#include "nvic.h"
#include "stm32f4xx_it.h"
#include "SysTick.h"
#include "gpio.h"
#include "usart1.h"
#include "DS18B20.h"
#include "LIN_Slave.h"
#include "rcc.h"
#include "timer.h"
#include "exti.h"
#include "lpwr.h"
#include "L3GD20.h"
#include "adc.h"
#include "dac.h"
#include "audio.h"
#include "IKS01A2.h"
#include "flash.h"

/** @addtogroup Template_Project
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M4 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief   This function handles NMI exception.
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
  SysTick_IncrementTicks_cb();
}

/******************************************************************************/
/*                 STM32F4xx Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f40xx.s/startup_stm32f427x.s).                         */
/******************************************************************************/

/**
  * @brief  This function handles EXTI0 interrupt request.
  * @param  None
  * @retval None
  */
void EXTI0_IRQHandler(void)
{
  AUDIO_PB_Callback();
}

/**
  * @brief  This function handles EXTI1 interrupt request.
  * @param  None
  * @retval None
  */
void EXTI1_IRQHandler(void)
{
  EXTI1_MEMS_INT1_Callback();
}

/**
  * @brief  This function handles EXTI2 interrupt request.
  * @param  None
  * @retval None
  */
void EXTI2_IRQHandler(void)
{
  EXTI2_MEMS_INT2_Callback();
}

/**
  * @brief  This function handles EXTI3 interrupt request.
  * @param  None
  * @retval None
  */
void EXTI3_IRQHandler(void)
{

}

/**
  * @brief  This function handles DMA1 Stream0 interrupt request.
  * @param  None
  * @retval None
  */
void DMA1_Stream0_IRQHandler(void)
{
  LIN_Slave_RX_DMA_IRQ_Callback();
}

/**
  * @brief  This function handles DMA1 Stream2 interrupt request.
  * @param  None
  * @retval None
  */
void DMA1_Stream2_IRQHandler(void)
{
  I2C3_RX_DMA_IRQ_Callback();
}

/**
  * @brief  This function handles DMA1 Stream4 interrupt request.
  * @param  None
  * @retval None
  */
void DMA1_Stream4_IRQHandler(void)
{
  I2C3_TX_DMA_IRQ_Callback();
}
/**
  * @brief  This function handles DMA1 Stream6 interrupt request.
  * @param  None
  * @retval None
  */
void DMA1_Stream6_IRQHandler(void)
{
  AUDIO_DMA1_Stream6_Callback();
}

/**
  * @brief  This function handles DMA1 Stream7 interrupt request.
  * @param  None
  * @retval None
  */
void DMA1_Stream7_IRQHandler(void)
{
  LIN_Slave_TX_DMA_IRQ_Callback();
}

/**
  * @brief  This function handles DMA2 Stream0 interrupt request.
  * @param  None
  * @retval None
  */
void DMA2_Stream0_IRQHandler(void)
{
  AUDIO_DMA2_Stream0_Callback();
}

/**
  * @brief  This function handles DMA2 Stream3 interrupt request.
  * @param  None
  * @retval None
  */
void DMA2_Stream3_IRQHandler(void)
{
  SPI5_RX_DMA_IRQ_Callback();
}

/**
  * @brief  This function handles DMA2 Stream4 interrupt request.
  * @param  None
  * @retval None
  */
void DMA2_Stream4_IRQHandler(void)
{
  SPI5_TX_DMA_IRQ_Callback();
}
/**
  * @brief  This function handles DMA2 Stream5 interrupt request.
  * @param  None
  * @retval None
  */
void DMA2_Stream5_IRQHandler(void)
{
  FLASH_USART1_RX_DMA_IRQ_Callback();
}

/**
  * @brief  This function handles DMA2 Stream7 interrupt request.
  * @param  None
  * @retval None
  */
void DMA2_Stream7_IRQHandler(void)
{
  FLASH_USART1_TX_DMA_IRQ_Callback();
}

/**
  * @brief  This function handles USART1 interrupt request.
  * @param  None
  * @retval None
  */
void USART1_IRQHandler(void)
{
  FLASH_USART1_IRQ_Callback();
}

/**
  * @brief  This function handles UART5 interrupt request.
  * @param  None
  * @retval None
  */
void UART5_IRQHandler(void)
{
  LIN_Slave_UART5_IRQ_Callback();
}

/**
  * @brief  This function handles TIM4 interrupt request.
  * @param  None
  * @retval None
  */
void TIM4_IRQHandler(void)
{
  TIM4_IRQ_Callback();
}

/**
  * @brief  This function handles TIM8 UP and TIM13 interrupt requests.
  * @param  None
  * @retval None
  */
void TIM8_UP_TIM13_IRQHandler(void)
{
  TIM8_IRQ_Callback();
}

/**
  * @brief  This function handles TIM8 TRG, COM and TIM14 interrupt requests.
  * @param  None
  * @retval None
  */
void TIM8_TRG_COM_TIM14_IRQHandler(void)
{
  TIM8_IRQ_Callback();
}

/**
  * @brief  This function handles ADC interrupt requests.
  * @param  None
  * @retval None
  */
void ADC_IRQHandler(void)
{
  ADC_IRQ_Callback();
}

/**
  * @}
  */ 


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
