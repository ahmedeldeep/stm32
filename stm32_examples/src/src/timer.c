/*******************************************************************************
 * @file    timer.c
 * @author  Ahmed Eldeep
 * @email   ahmed@almohandes.org
 * @website http://almohandes.org/stm32
 * @date    11.09.2018
 *          
 * @brief   Timer examples
 * @note    
 *
@verbatim
Copyright (C) Almohandes.org, 2018

This program is free software: you can redistribute it and/or modify it under
the terms of the GNU General Public License as published by the Free Software
Foundation, either version 3 of the License, or any later version.

This program is distributed in the hope that it will be useful, but WITHOUT
ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with
this program.  If not, see <http://www.gnu.org/licenses/>.
@endverbatim
*******************************************************************************/

/* Includes */
#include "timer.h"
#include "gpio.h"

/**
 * @addtogroup stm32_examples
 * @{
 */

/**
 * @defgroup timer
 * @brief
 * @{
 */

/**
 * @defgroup timer_private_typedefs
 * @{
 */

/**
 * @}
 */

/**
 * @defgroup timer_private_defines
 * @{
 */

/**
 * @brief   AF3 PA0 pin masks
 */
#define GPIO_AFRL_AFRL0                      ((uint32_t) 0x0000000F)
#define GPIO_AFRL_AFRL0_AF3                  ((uint32_t) 0x00000003)

/**
 * @}
 */

/**
 * @defgroup timer_private_macros
 * @{
 */

/**
 * @}
 */

/**
 * @defgroup timer_private_constants
 * @{
 */

/**
 * @}
 */

/**
 * @defgroup timer_private_variables
 * @{
 */

/**
 * @}
 */

/**
 * @defgroup timer_private_function_prototypes
 * @{
 */

/**
 * @}
 */

/**
 * @defgroup timer_private_functions
 * @{
 */

/**
 * @}
 */

/**
 * @defgroup timer_exported_functions
 * @{
 */

/**
 * @brief   Timer6 as up counter Configuration function
 * @note
 * @param   
 * @retval
 */
void TIM6_UpCount_Config(void)
{
  /* Enable TIM6 clock */
  RCC->APB1ENR |= RCC_APB1ENR_TIM6EN;

  /* Set counter direction as up-counter */
  TIM6->CR1 &= ~TIM_CR1_DIR;

  /* Set timer Prescaler, bus clock = 45 MHz, fCK_PSC / (PSC[15:0] + 1)
   * CK_CNT = 45000000 / (44999 + 1) -> 1000 Hz -> time base = 1 ms */
  TIM6->PSC = 44999;

  /* Set timer reload value */
  TIM6->ARR = 5000;

  /* Enable TIM6 counter */
  TIM6->CR1 |= TIM_CR1_CEN;
}

/**
 * @brief   Timer3 as down counter Configuration function
 * @note
 * @param
 * @retval
 */
void TIM3_DownCount_Config(void)
{
  /* Enable TIM5 clock */
  RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;

  /* Set counter direction as down-counter */
  TIM3->CR1 |= TIM_CR1_DIR;

  /* Set timer Prescaler, bus clock = 45 MHz, fCK_PSC / (PSC[15:0] + 1)
   * CK_CNT = 45000000 / (44999 + 1) -> 1000 Hz -> time base = 1 ms */
  TIM3->PSC = 56249;

  /* Set timer reload value */
  TIM3->ARR = 5000;

  /* Enable TIM5 counter */
  TIM3->CR1 |= TIM_CR1_CEN;
}

/**
 * @brief   Timer4 as up down counter Configuration function
 * @note
 * @param
 * @retval
 */
void TIM4_UpDownCount_Config(void)
{
  /* Enable TIM4 clock */
  RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;

  /* Set counter direction as up-down-counter */
  TIM4->CR1 |= TIM_CR1_CMS;

  /* Set timer Prescaler, bus clock = 45 MHz, fCK_PSC / (PSC[15:0] + 1)
   * CK_CNT = 45000000 / (44999 + 1) -> 1000 Hz -> time base = 1 ms */
  TIM4->PSC = 44999;

  /* Set timer reload value */
  TIM4->ARR = 5000;

  /* Enable update event interrupt */
  TIM4->DIER |= TIM_DIER_UIE;

  /* Enable TIM4 counter */
  TIM4->CR1 |= TIM_CR1_CEN;
}

/**
 * @brief   Timer4 IRQ callback function
 * @note
 * @param
 * @retval
 */
void TIM4_IRQ_Callback(void)
{
  /* Clear update interrupt flag */
  TIM4->SR &= ~TIM_SR_UIF;

  /* Toggle green LED */
  GPIO_Toggle_LED(EVAL_GREEN_LED);
}

/**
 * @brief   TIM8 configuration function
 * @note    Configure TIM8 external clock mode 1 and 2 using external trigger
 *          input (TIM8_ETR) mapped to PA0 using AF3
 * @param
 * @retval
 */
void TIM8_ETR_Config(void)
{
  /* Configure GPIO for PA0 */
  /* Enable GPIOA clock in RCC */
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;

  /* Select alternate function mode for PA0 */
  GPIOA->MODER &= ~GPIO_MODER_MODER0_0;
  GPIOA->MODER |= GPIO_MODER_MODER0_1;

  /* Select no pull up because it has external pull down */
  GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPDR0);

  /* Select alternate function AF3 */
  GPIOA->AFR[0] &= ~GPIO_AFRL_AFRL0;
  GPIOA->AFR[0] |= GPIO_AFRL_AFRL0_AF3;

  /* Timer 8 time base configuration */
  /* Enable TIM8 clock */
  RCC->APB2ENR |= RCC_APB2ENR_TIM8EN;

  /* Set counter direction as up-counter */
  TIM8->CR1 &= ~TIM_CR1_DIR;

  /* No need for prescaler configuration */

  /* Set timer reload value */
  TIM8->ARR = 10;

  /* Enable update event interrupt */
  TIM8->DIER |= TIM_DIER_UIE;

  /* External trigger configuration */
  /* Select external trigger polarity active high edge */
  TIM8->SMCR &= ~TIM_SMCR_ETP;

  /* Select no external trigger prescaler */
  TIM8->SMCR &= ~TIM_SMCR_ETPS;

  /* Select external trigger filter */
  TIM8->SMCR |= TIM_SMCR_ETF;

//  /* External clock mode 1 configuration */
//  /* Reset SMS bits */
//  TIM8->SMCR &= ~TIM_SMCR_SMS;
//
//  /* Select external trigger ETRF */
//  TIM8->SMCR |= TIM_SMCR_TS;
//
//  /* Select slave mode as external clock mode 1 */
//  TIM8->SMCR |= TIM_SMCR_SMS;

  /* External clock mode 2 configuration */
  TIM8->SMCR |= TIM_SMCR_ECE;

  /* Enable trigger event interrupt */
  TIM8->DIER |= TIM_DIER_TIE;

  /* Enable TIM8 */
  TIM8->CR1 |= TIM_CR1_CEN;
}

/**
 * @brief   Timer8 IRQ callback function
 * @note
 * @param
 * @retval
 */
void TIM8_IRQ_Callback(void)
{
  /* Check if update event was happened */
  if(TIM_SR_UIF == (TIM_SR_UIF & TIM8->SR))
  {
    /* Clear update interrupt flag */
    TIM8->SR &= ~TIM_SR_UIF;

    /* Toggle red LED */
    GPIO_Toggle_LED(EVAL_RED_LED);
  }

  /* Check if trigger event was happened */
  if(TIM_SR_TIF == (TIM_SR_TIF & TIM8->SR))
  {
    /* Clear trigger interrupt flag */
    TIM8->SR &= ~TIM_SR_TIF;

    /* Toggle green LED */
    GPIO_Toggle_LED(EVAL_GREEN_LED);
  }
}

/**
 * @}
 */
/**
 * @}
 */
/**
 * @}
 */
