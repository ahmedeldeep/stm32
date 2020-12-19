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
 * @brief   for storing the capture compare registers
 */
typedef struct CC_Reg {
  uint32_t  CCR1;
  uint32_t  CCR2;
} CC_RegType;

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
 * @brief   AF3 PC6 pin masks
 */
#define GPIO_AFRL_AFRL6                      ((uint32_t) 0x0F000000)
#define GPIO_AFRL_AFRL6_AF3                  ((uint32_t) 0x03000000)

/**
 * @brief   AF1 PE9 pin masks
 */
#define GPIO_AFRH_AFRH9                      ((uint32_t) 0x000000F0)
#define GPIO_AFRH_AFRH9_AF1                  ((uint32_t) 0x00000010)

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
 * @brief   for storing the capture compare registers
 */
static CC_RegType TIM1_CC_Reg;

/**
 * @brief   Variables used for PWM calculation
 */
static float frequency = 0;
static float duty_cycle = 0;

/**
 * @brief   Variables used for output PWM generation
 */
static uint16_t PWM_Output_frequency = 0;
static uint16_t PWM_Output_duty_cycle = 0;


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
 * @brief   TIM1 configuration function
 * @note    Configure TIM1 to measure the PWM duty cycle and frequency of an
 *          input signal
 *          input (TIM1_CH1) mapped to PE9 using AF1
 * @param
 * @retval
 */
void TIM1_Measure_PWM_Config(void)
{
  /* GPIO Configuration */
  /* Configure GPIO for PE9 */
  /* Enable GPIOE clock in RCC */
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOEEN;

  /* Select alternate function mode for PE9 */
  GPIOE->MODER &= ~GPIO_MODER_MODER9_0;
  GPIOE->MODER |= GPIO_MODER_MODER9_1;

  /* Select no pull-up, pull-down */
  GPIOE->PUPDR &= ~(GPIO_PUPDR_PUPDR9);

  /* Select alternate function AF1 for PE9 */
  GPIOE->AFR[1] &= ~GPIO_AFRH_AFRH9;
  GPIOE->AFR[1] |= GPIO_AFRH_AFRH9_AF1;



  /* Timer Configuration */
  /* Timer 1 time base configuration */
  /* Enable TIM1 clock */
  RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;

  /* Set counter direction as up-counter */
  TIM1->CR1 &= ~TIM_CR1_DIR;

  /* Set timer Prescaler, bus clock = 90 MHz, fCK_PSC / (PSC[15:0] + 1)
   * CK_CNT = 180000000 / (1799 + 1) -> 100000 Hz -> time base = 10 us */
  TIM1->PSC = 1799;

  /* Set timer auto reload value to maximum */
  TIM1->ARR = 0xFFFF;

  /* Timer 1 input channel 1 IC1 configuration */
  /* CC1 channel is configured as input, IC1 is mapped on TI1 */
  TIM1->CCMR1 &= ~TIM_CCMR1_CC1S;
  TIM1->CCMR1 |= TIM_CCMR1_CC1S_0;

  /* No input capture 1 prescaler */
  /* No Input capture 1 filter */

  /* Select input polarity for TI1FP1 active on rising edge */
  TIM1->CCER &= ~TIM_CCER_CC1P;
  TIM1->CCER &= ~TIM_CCER_CC1NP;

  /* Timer 1 input channel 1 IC2 configuration */
  /* CC2 channel is configured as input, IC2 is mapped on TI1 */
  TIM1->CCMR1 &= ~TIM_CCMR1_CC2S;
  TIM1->CCMR1 |= TIM_CCMR1_CC2S_1;

  /* Select input polarity for TI1FP2 active on falling edge */
  TIM1->CCER |= TIM_CCER_CC2P;
  TIM1->CCER &= ~TIM_CCER_CC1NP;

  /* Timer 1 trigger configuration */
  /* Select Filtered Timer Input 1 (TI1FP1) trigger */
  TIM1->SMCR &= ~TIM_SMCR_TS;
  TIM1->SMCR |= (TIM_SMCR_TS_0 | TIM_SMCR_TS_2);

  /* Select reset mode */
  TIM1->SMCR &= ~TIM_SMCR_SMS;
  TIM1->SMCR |= TIM_SMCR_SMS_2;



  /* DMA Configuration */
  /* DMA2 - Channel 0 - Stream 6 */
  /* Enable DMA clock in RCC */
  RCC->AHB1ENR |= RCC_AHB1ENR_DMA2EN;

  /* Select the DMA2 channel 0 */
  DMA2_Stream6->CR &= ~DMA_SxCR_CHSEL;

  /* Select stream priority very high */
  DMA2_Stream6->CR |= DMA_SxCR_PL;

  /* Select memory and peripherals sizes word (32-bit) */
  DMA2_Stream6->CR |= DMA_SxCR_MSIZE_1;
  DMA2_Stream6->CR |= DMA_SxCR_PSIZE_1;

  /* Select memory incremented mode */
  DMA2_Stream6->CR |= DMA_SxCR_MINC;

  /* Select circular mode */
  DMA2_Stream6->CR |= DMA_SxCR_CIRC;

  /* Select the data transfer direction peripheral to memory */
  DMA2_Stream6->CR &= ~DMA_SxCR_DIR;

  /* Set number of data items */
  DMA2_Stream6->NDTR = 2;

  /* Set the source address to the peripheral port */
  DMA2_Stream6->PAR = (uint32_t)&TIM1->DMAR;

  /* Set the destination address to the memory port */
  DMA2_Stream6->M0AR = (uint32_t)&TIM1_CC_Reg.CCR1;



  /* Configure Timer DMA related registers */
  /* Set DMA base address to CC register */
  TIM1->DCR &= ~TIM_DCR_DBA;
  TIM1->DCR |= (TIM_DCR_DBA_0 | TIM_DCR_DBA_2 | TIM_DCR_DBA_3);

 /* Select DMA burst length = 2 */
  TIM1->DCR &= ~TIM_DCR_DBL;
  TIM1->DCR |= TIM_DCR_DBL_0;

  /* Enable Capture/Compare 1 DMA request enable */
  TIM1->DIER |= TIM_DIER_CC1DE;

  /* Enable DMA 2 stream 6 */
  DMA2_Stream6->CR |= DMA_SxCR_EN;

  /* Enable CC1 and CC2 */
  TIM1->CCER |= (TIM_CCER_CC1E | TIM_CCER_CC2E);

  /* Enable TIM1 */
  TIM1->CR1 |= TIM_CR1_CEN;
}

/**
 * @brief   TIM1 measure PWM main function
 * @note    calculates the duty cycle and frequency.
 * @param
 * @retval
 */
void TIM1_Measure_PWM_Main(void)
{
  /* Avoid division by zero */
  if (0 != TIM1_CC_Reg.CCR1)
  {
    /* Calculate PWM frequency = Prescaler / CC1  */
    frequency = 100000.0 / (float)TIM1_CC_Reg.CCR1;

    /* Calculate duty cycle = (Ton / period) * 100 */
    duty_cycle = ((float)TIM1_CC_Reg.CCR2 / (float)TIM1_CC_Reg.CCR1) * 100.0;
  }
  else
  {
    /* Reset */
    frequency = 0;
    duty_cycle = 0;
  }
}

/**
 * @brief   TIM1 configuration function
 * @note    Configure TIM1 to generate PWM with configurable duty cycle
 *          and frequency. (TIM1_CH1) mapped to PE9 using AF1
 * @param
 * @retval
 */
void TIM1_Generate_PWM_Config(void)
{
  /* GPIO Configuration */
  /* Configure GPIO for PE9 */
  /* Enable GPIOE clock in RCC */
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOEEN;

  /* Select alternate function mode for PE9 */
  GPIOE->MODER &= ~GPIO_MODER_MODER9_0;
  GPIOE->MODER |= GPIO_MODER_MODER9_1;

  /* Select no pull-up, pull-down */
  GPIOE->PUPDR &= ~(GPIO_PUPDR_PUPDR9);

  /* Select alternate function AF1 for PE9 */
  GPIOE->AFR[1] &= ~GPIO_AFRH_AFRH9;
  GPIOE->AFR[1] |= GPIO_AFRH_AFRH9_AF1;


  /* Timer Configuration */
  /* Timer 1 time base configuration */
  /* Enable TIM1 clock */
  RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;

  /* Enable Auto-reload preload */
  TIM1->CR1 |= TIM_CR1_ARPE;

  /* Set counter direction as up-counter */
  TIM1->CR1 &= ~(TIM_CR1_DIR | TIM_CR1_CMS);

  /* Select master mode Compare */
  TIM1->CR2 |= TIM_CR2_MMS_2;

  /* Master/slave mode effect is delayed to allow a perfect synchronization */
  TIM1->SMCR |= TIM_SMCR_MSM;

  /* Set timer Prescaler, bus clock = 90 MHz, fCK_PSC / (PSC[15:0] + 1)
   * CK_CNT = 180000000 / (1799 + 1) -> 100000 Hz -> time base = 10 us */
  TIM1->PSC = 1799;

  /* Set timer auto reload value to maximum */
  TIM1->ARR = 0xFFFF;

  /* Set 50% duty cycle */
  TIM1->CCR1 = 0xFFFF / 2;

  /* Set Capture/Compare 1 as output */
  TIM1->CCMR1 &= ~TIM_CCMR1_CC1S;

  /* Output Compare 1 preload enable */
  TIM1->CCMR1 |= TIM_CCMR1_OC1PE;

  /* Select Output Compare 1 PWM mode 1
   * TIMx_CNT < TIMx_CCR1 -> Output Active
   * TIMx_CNT >= TIMx_CCR1 -> Output Inactive */
  TIM1->CCMR1 |= (TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2);

  /* Select Capture/Compare 1 output polarity active high */
  TIM1->CCER &= ~TIM_CCER_CC1P;

  /* Initialize all the registers */
  TIM1->EGR |= TIM_EGR_UG;

  /* Enable Capture/Compare 1 output */
  TIM1->CCER |= TIM_CCER_CC1E;

  /* Enable timer main output */
  TIM1->BDTR |= TIM_BDTR_MOE;

  /* Enable TIM1 */
  TIM1->CR1 |= TIM_CR1_CEN;
}

/**
 * @brief   PWM update function
 * @note
 * @param
 * @retval
 */
void TIM1_Update_PWM(void)
{
  /* Calculate auto reload value,
   * update_time = (auto_reload * time_base)
   * pwm_frequency = 1 / (auto_reload * time_base)
   * auto_reload = 1 / (pwm_frequency * time_base) = 100000 / pwm_frequency */
  if(0 != PWM_Output_frequency)
  {
    /* Calculate auto reload value */
    uint16_t auto_reload = (uint16_t)(100000 / PWM_Output_frequency);

    /* Write auto reload register */
    TIM1->ARR = auto_reload;

    /* Update CCR1 with duty cycle
     * PWM_Output_duty_cycle = (TIM1->CCR1 / TIM1->ARR) * 100
     * TIM1->CCR1 = (PWM_Output_duty_cycle * TIM1->ARR) / 100 */
    /* Calculate capture compare value */
    uint16_t capture_compare =
        (uint16_t)((PWM_Output_duty_cycle * auto_reload) / 100);

    /* Write capture compare register */
    TIM1->CCR1 = capture_compare;

    /* update timer 8 */
    TIM8->ARR = auto_reload / 2;
    TIM8->CCR1 = capture_compare / 2;
  }
  else
  {
    /* Do nothing */
  }
}

/**
 * @brief   TIM8 configuration function
 * @note    Configure TIM8 to generate one pulse using timer synchronization.
 * @param
 * @retval
 */
void TIM8_Generate_OnePulse_Config(void)
{
  /* GPIO Configuration */
  /* Configure GPIO for PC6 */
  /* Enable GPIOE clock in RCC */
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;

  /* Select alternate function mode for PC6 */
  GPIOC->MODER &= ~GPIO_MODER_MODER6_0;
  GPIOC->MODER |= GPIO_MODER_MODER6_1;

  /* Select no pull-up, pull-down */
  GPIOC->PUPDR &= ~(GPIO_PUPDR_PUPDR6);

  /* Select alternate function AF3 for PC6 */
  GPIOC->AFR[0] &= ~GPIO_AFRL_AFRL6;
  GPIOC->AFR[0] |= GPIO_AFRL_AFRL6_AF3;


  /* Timer Configuration */
  /* Timer 8 time base configuration */
  /* Enable TIM8 clock */
  RCC->APB2ENR |= RCC_APB2ENR_TIM8EN;

  /* Enable Auto-reload preload */
  TIM8->CR1 |= TIM_CR1_ARPE;

  /* Set counter direction as up-counter */
  TIM8->CR1 &= ~(TIM_CR1_DIR | TIM_CR1_CMS);

  /* Enable One pulse mode */
  TIM8->CR1 |= TIM_CR1_OPM;

  /* Set timer Prescaler, bus clock = 90 MHz, fCK_PSC / (PSC[15:0] + 1)
   * CK_CNT = 180000000 / (1799 + 1) -> 100000 Hz -> time base = 10 us */
  TIM8->PSC = 1799;

  /* Set timer auto reload value to maximum */
  TIM8->ARR = 0xFFFF / 2;

  /* Set pulse delay after trigger */
  TIM8->CCR1 = 0xFFFF / 4;

  /* Select trigger input from TIM1 ITR0 */
  TIM8->SMCR &= ~TIM_SMCR_TS;

  /* set slave mode selection (Trigger Mode) */
  TIM8->SMCR |= (TIM_SMCR_SMS_1 | TIM_SMCR_SMS_2);

  /* Set Capture/Compare 1 as output */
  TIM8->CCMR1 &= ~TIM_CCMR1_CC1S;

  /* Output Compare 1 preload enable */
  TIM8->CCMR1 |= TIM_CCMR1_OC1PE;

  /* Select Output Compare 1 PWM mode 2
   * TIMx_CNT < TIMx_CCR1 -> Output Active
   * TIMx_CNT >= TIMx_CCR1 -> Output Inactive */
  TIM8->CCMR1 |= TIM_CCMR1_OC1M;

  /* Select Capture/Compare 1 output polarity active high */
  TIM8->CCER &= ~TIM_CCER_CC1P;

  /* Initialize all the registers */
  TIM8->EGR |= TIM_EGR_UG;

  /* Enable Capture/Compare 1 output */
  TIM8->CCER |= TIM_CCER_CC1E;

  /* Enable timer main output */
  TIM8->BDTR |= TIM_BDTR_MOE;

  /* Enable TIM8 */
  TIM8->CR1 |= TIM_CR1_CEN;

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
