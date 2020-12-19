/*******************************************************************************
 * @file    adc.c
 * @author  Ahmed Eldeep
 * @email   ahmed@almohandes.org
 * @website http://almohandes.org/stm32
 * @date    19.11.2018
 *          
 * @brief   ADC Example
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
#include "adc.h"
#include "SysTick.h"
#include "gpio.h"

/**
 * @addtogroup stm32_examples
 * @{
 */

/**
 * @defgroup adc
 * @brief
 * @{
 */

/**
 * @defgroup adc_private_typedefs
 * @{
 */

/**
 * @}
 */

/**
 * @defgroup adc_private_defines
 * @{
 */

/**
 * @}
 */

/**
 * @defgroup adc_private_macros
 * @{
 */

/**
 * @}
 */

/**
 * @defgroup adc_private_constants
 * @{
 */

/**
 * @}
 */

/**
 * @defgroup adc_private_variables
 * @{
 */

/**
 * @brief   Green led timeout
 */
static uint16_t gLEDTimeOut = 0;

/**
 * @brief   Red led timeout
 */
static uint16_t rLEDTimeOut = 0;

/**
 * @brief   Analog data
 */
static uint16_t analogData_1 = 0;
static uint16_t analogData_2 = 0;

/**
 * @brief   Store start tick
 */
static uint32_t gLEDStartTick = 0;
static uint32_t rLEDStartTick = 0;

/**
 * @}
 */

/**
 * @defgroup adc_private_function_prototypes
 * @{
 */

/**
 * @}
 */

/**
 * @defgroup adc_private_functions
 * @{
 */

/**
 * @}
 */

/**
 * @defgroup adc_exported_functions
 * @{
 */

/**
 * @brief   ADC1 configuration function
 * @note    PA5 -> ADC1_IN5 (potentiometer input used to control the green led)
 *            -> Regular channel
 *            -> Software triggered
 *
 *          PC3 -> ADC1_IN13 (potentiometer input used to control the red led)
*            -> Injected channel conversion
*            -> Timer triggered
 * @param   
 * @retval
 */
void ADC1_Config(void)
{
  /* GPIO Configuration for PA5 and PC3 */
  /* ********************************** */

  /* Enable port A clock */
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;

  /* Enable port C clock */
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;

  /* Select Analog mode */
  GPIOA->MODER |= GPIO_MODER_MODER5;
  GPIOC->MODER |= GPIO_MODER_MODER3;

  /* Select no pull-up, pull-down (reset state) */
  GPIOA->PUPDR &= ~GPIO_PUPDR_PUPDR5;
  GPIOC->PUPDR &= ~GPIO_PUPDR_PUPDR3;

  /* ADC Configuration for (PA5 -> ADC1_IN5) and (PC3 -> ADC1_IN13) */
  /* ************************************************************** */

  /* Enable ADC1 clock */
  RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;

  /* Enable ADC */
  ADC1->CR2 |= ADC_CR2_ADON;

  /* ADC prescaler PCLK2 divided by 2 */
  ADC->CCR &= ~ADC_CCR_ADCPRE;

  /* Select ADC1 12-bits Resolution */
  ADC1->CR1 &= ~ADC_CR1_RES;

  /* Select Right alignment */
  ADC1->CR2 &= ~ADC_CR2_ALIGN;

  /* Enable interrupt for injected channels */
  ADC1->CR1 |= ADC_CR1_JEOCIE;

  /* Enable interrupt for regular channels */
  ADC1->CR1 |= ADC_CR1_EOCIE;

  /* External trigger enable for injected channels */
  ADC1->CR2 |= ADC_CR2_JEXTEN_0;

  /* Select Timer 2 TRGO event as external event for injected group */
  ADC1->CR2 &= ~ADC_CR2_JEXTSEL;
  ADC1->CR2 |= (ADC_CR2_JEXTSEL_0 | ADC_CR2_JEXTSEL_1);

  /* Select channel 5 regular */
  ADC1->SQR1 &= ~ADC_SQR1_L;
  ADC1->SQR3 &= ~ADC_SQR3_SQ1;
  ADC1->SQR3 |= (ADC_SQR3_SQ1_0 | ADC_SQR3_SQ1_2);

  /* Select channel 13 injected */
  ADC1->JSQR &= ~ADC_JSQR_JL;
  ADC1->JSQR &= ~ADC_JSQR_JSQ4;
  ADC1->JSQR |= (ADC_JSQR_JSQ4_0 | ADC_JSQR_JSQ4_2 | ADC_JSQR_JSQ4_3);

  /* Timer 2 TRGO event Configuration for ADC1_IN13 */
  /* ************************************************************** */
  /* Enable TIM2 clock */
  RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;

  /* Set counter direction as up-counter */
  TIM2->CR1 &= ~TIM_CR1_DIR;

  /* Set timer Prescaler, bus clock = 45 MHz, Tim_Clock = 90 MHz
   * fCK_PSC / (PSC[15:0] + 1)
   * CK_CNT = 90000000 / (44999 + 1) -> 2000 Hz -> time base = 0.5 ms */
  TIM2->PSC = 44999;

  /* Set timer reload value, update event each 5 ms */
  TIM2->ARR = 10;

  /* Select update event as trigger output (TRGO) */
  TIM2->CR2 &= ~TIM_CR2_MMS;
  TIM2->CR2 |= TIM_CR2_MMS_1;

  /* Enable TIM2 counter */
  TIM2->CR1 |= TIM_CR1_CEN;

  /* Start conversion of regular channels */
  ADC1->CR2 |= ADC_CR2_SWSTART;
}

/**
 * @brief   Main function
 * @note    toggle LEDs based on the analog data
 * @param
 * @retval
 */
void ADC1_Main(void)
{
  /* Get ADC data */
  gLEDTimeOut = analogData_1 / 4;
  rLEDTimeOut = analogData_2 / 4;

  /* Get current system tick */
  uint32_t currentTick = SysTick_GetCurrentTick();

  /* Calculate time passed */
  uint32_t gLEDTimePassed = currentTick - gLEDStartTick;
  uint32_t rLEDTimePassed = currentTick - rLEDStartTick;

  /* Check green led time out */
  if(gLEDTimePassed > gLEDTimeOut)
  {
    /* Toggle green led */
    GPIO_Toggle_LED(EVAL_GREEN_LED);

    /* Update start tick */
    gLEDStartTick = SysTick_GetCurrentTick();
  }
  else
  {
    /* Do nothing, No timeout */
  }

  /* Check red led time out */
  if(rLEDTimePassed > rLEDTimeOut)
  {
    /* Toggle red led */
    GPIO_Toggle_LED(EVAL_RED_LED);

    /* Update start tick */
    rLEDStartTick = SysTick_GetCurrentTick();
  }
  else
  {
    /* Do nothing, No timeout */
  }

}


/**
 * @brief   ADC interrupt callback
 * @note    Common for all ADC units
 * @param
 * @retval
 */
void ADC_IRQ_Callback(void)
{
  /* Check ADC1 EOC */
  if((ADC1->SR & ADC_SR_EOC) == ADC_SR_EOC)
  {
    /* Read analog data and clear EOC flag */
    analogData_1 = ADC1->DR;

    /* Start conversion of regular channels */
    ADC1->CR2 |= ADC_CR2_SWSTART;
  }
  else
  {
    /* Regular conversion is not done */
  }

  /* Check ADC1 JEOC */
  if((ADC1->SR & ADC_SR_JEOC) == ADC_SR_JEOC)
  {
    /* Read analog data */
    analogData_2 = ADC1->JDR1;

    /* Clear JEOC flag */
    ADC1->SR &= ~ADC_SR_JEOC;

    /* Next conversion will be trigger by timer 2 TRGO event */
  }
  else
  {
    /* Regular conversion is not done */
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
