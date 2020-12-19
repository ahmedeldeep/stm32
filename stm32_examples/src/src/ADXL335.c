/*******************************************************************************
 * @file    ADXL335.c
 * @author  Ahmed Eldeep
 * @email   ahmed@almohandes.org
 * @website http://almohandes.org/stm32
 * @date    05.12.2018
 *          
 * @brief   Interfacing Accelerometer ADXL335
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
#include "ADXL335.h"

/**
 * @addtogroup stm32_examples
 * @{
 */

/**
 * @defgroup ADXL335
 * @brief
 * @{
 */

/**
 * @defgroup ADXL335_private_typedefs
 * @{
 */

/**
 * @}
 */

/**
 * @defgroup ADXL335_private_defines
 * @{
 */

/**
 * @brief   Sensor sensitivity = 300 mV/g
 *          300mV = 1g = 90 deg then 1 mv = (1/300)g = 90/300 = 0.3 deg
 *
 */
#define SENSOR_SENSITIVITY                   ((float) 0.3)

/**
 * @brief   Zero g bias level = VCC / 2
 */
#define ZERO_G_BIAS_LEVEL                    ((int16_t) 2048)

/**
 * @brief   Convert ADC reading to mV
 *          ADC = 4095 when 3000 mv
 *          ADC resolution = 3000 / 4096 = 0.732 mV/LSB
 */
#define ADC_RESOLUTION                       ((float) 0.732)

/**
 * @}
 */

/**
 * @defgroup ADXL335_private_macros
 * @{
 */

/**
 * @}
 */

/**
 * @defgroup ADXL335_private_constants
 * @{
 */

/**
 * @}
 */

/**
 * @defgroup ADXL335_private_variables
 * @{
 */

/**
 * @brief   ADXL335 Analog reading buffer
 */
static uint16_t Analog_buffer[3];

/**
 * @brief   Tilting angles
 */
static float TiltAngle_X = 0;
static float TiltAngle_Y = 0;
static float TiltAngle_Z = 0;

/**
 * @}
 */

/**
 * @defgroup ADXL335_private_function_prototypes
 * @{
 */

/**
 * @}
 */

/**
 * @defgroup ADXL335_private_functions
 * @{
 */

/**
 * @}
 */

/**
 * @defgroup ADXL335_exported_functions
 * @{
 */

/**
 * @brief   ADC1 configuration function
 * @note    PA_5 -> ADC1_IN5
 *          PC_3 -> ADC2_IN13
 *          PF_6 -> ADC3_IN4
 *
 *          ADC is configured in triple mode with external
 *          timer trigger and DMA
 * @param
 * @retval
 */
void ADXL335_Config(void)
{
  /* GPIO Configuration for PA_5, PC_3 and PF_6 */
  /* ****************************************** */

  /* Enable port A, C, F clocks */
  RCC->AHB1ENR |= (RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOCEN
      | RCC_AHB1ENR_GPIOFEN);

  /* Select Analog mode */
  GPIOA->MODER |= GPIO_MODER_MODER5;
  GPIOC->MODER |= GPIO_MODER_MODER3;
  GPIOF->MODER |= GPIO_MODER_MODER6;

  /* Select no pull-up, pull-down (reset state) */
  GPIOA->PUPDR &= ~GPIO_PUPDR_PUPDR5;
  GPIOC->PUPDR &= ~GPIO_PUPDR_PUPDR3;
  GPIOF->PUPDR &= ~GPIO_PUPDR_PUPDR6;

  /* ADC Configuration ADC1_IN5, ADC2_IN13, ADC3_IN4 */
  /* *********************************************** */
  /* ADC prescaler PCLK2 divided by 2 */
  /* 12-bits Resolution */
  /* Right alignment */

  /* Enable ADC clocks */
  RCC->APB2ENR |= (RCC_APB2ENR_ADC1EN | RCC_APB2ENR_ADC2EN
      | RCC_APB2ENR_ADC3EN);

  /* Enable ADC */
  ADC1->CR2 |= ADC_CR2_ADON;
  ADC2->CR2 |= ADC_CR2_ADON;
  ADC3->CR2 |= ADC_CR2_ADON;

  /* External trigger enable for regular channels, only ADC1 */
  ADC1->CR2 |= ADC_CR2_EXTEN_0;

  /* Select Timer 2 TRGO event as external event for regular group */
  ADC1->CR2 &= ~ADC_CR2_EXTSEL;
  ADC1->CR2 |= (ADC_CR2_EXTSEL_1 | ADC_CR2_EXTSEL_2);

  /* Select ADC1_IN5 regular */
  ADC1->SQR1 &= ~ADC_SQR1_L;
  ADC1->SQR3 &= ~ADC_SQR3_SQ1;
  ADC1->SQR3 |= (ADC_SQR3_SQ1_0 | ADC_SQR3_SQ1_2);

  /* Select ADC2_IN13 regular */
  ADC2->SQR1 &= ~ADC_SQR1_L;
  ADC2->SQR3 &= ~ADC_SQR3_SQ1;
  ADC2->SQR3 |= (ADC_SQR3_SQ1_0 | ADC_SQR3_SQ1_2 | ADC_SQR3_SQ1_3);

  /* Select ADC3_IN4 regular */
  ADC3->SQR1 &= ~ADC_SQR1_L;
  ADC3->SQR3 &= ~ADC_SQR3_SQ1;
  ADC3->SQR3 |= (ADC_SQR3_SQ1_2);

  /* Select DMA mode 1 */
  ADC->CCR |= ADC_CCR_DMA_0;

  /* Continuous DMA requests */
  ADC->CCR |= ADC_CCR_DDS;

  /* Triple Regular simultaneous mode */
  ADC->CCR |= ADC_CCR_MULTI_1 | ADC_CCR_MULTI_2 | ADC_CCR_MULTI_4;


  /* DMA Circular mode configuration ADC -> DMA2_Stream0 (Channel 0) */
  /* *************************************************************** */

  /* Enable DMA clock in RCC */
  RCC->AHB1ENR |= RCC_AHB1ENR_DMA2EN;

  /* Make sure that the DMA2_Stream0 is disabled */
  if(DMA_SxCR_EN == (DMA_SxCR_EN & DMA2_Stream0->CR))
  {
    /* DMA2_Stream0 is enabled, shall be disabled first */
    DMA2_Stream0->CR &= ~DMA_SxCR_EN;

    /* Wait until EN bit is cleared */
    while(DMA_SxCR_EN == (DMA_SxCR_EN & DMA2_Stream0->CR))
    {
      /* Do nothing until EN bit is cleared */
    }
  }
  else
  {
    /* Do nothing, DMA2_Stream0 is not enabled */
  }


  /* Select the DMA channel 0 in CHSEL[2:0] in the DMA_SxCR */
  DMA2_Stream0->CR &= ~DMA_SxCR_CHSEL;

  /* Select stream priority very high */
  DMA2_Stream0->CR |= DMA_SxCR_PL;

  /* Select the data transfer direction peripheral(ADC)-to-memory */
  DMA2_Stream0->CR &= ~DMA_SxCR_DIR;

  /* Select memory and peripherals sizes 2 bytes*/
  DMA2_Stream0->CR |= DMA_SxCR_MSIZE_0;
  DMA2_Stream0->CR |= DMA_SxCR_PSIZE_0;

  /* Select FIFO mode */
  DMA2_Stream0->FCR |= DMA_SxFCR_DMDIS;

  /* Select FIFO threshold level half FIFO */
  DMA2_Stream0->FCR |= (DMA_SxFCR_FTH_0 | DMA_SxFCR_FTH_1);

  /* Select memory incremented mode, peripheral shall has fixed address */
  DMA2_Stream0->CR |= DMA_SxCR_MINC;

  /* Select circular mode */
  DMA2_Stream0->CR |= DMA_SxCR_CIRC;

  /* Set peripheral address */
  DMA2_Stream0->PAR = (uint32_t)&ADC->CDR;

  /* Set memory address */
  DMA2_Stream0->M0AR = (uint32_t)Analog_buffer;

  /* Set number of data items, each conversion 3 requests */
  DMA2_Stream0->NDTR = 3;

  /* Enable DMA2_Stream0 */
  DMA2_Stream0->CR |= DMA_SxCR_EN;

  /* Timer 2 TRGO event Configuration */
  /* ******************************** */
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
}

/**
 * @brief   Main function
 * @note
 * @param
 * @retval
 */
void ADXL335_Main(void)
{
  /* Accelerometer data offset */
  int16_t offset_X = 0;
  int16_t offset_Y = 0;
  int16_t offset_Z = 0;

  /* Calculate analog data offset */
  offset_X = (int16_t)Analog_buffer[0] - ZERO_G_BIAS_LEVEL;
  offset_Y = (int16_t)Analog_buffer[1] - ZERO_G_BIAS_LEVEL;
  offset_Z = (int16_t)Analog_buffer[2] - ZERO_G_BIAS_LEVEL;

  /* Calculate tilting angles */
  TiltAngle_X = ((float) offset_X) * ADC_RESOLUTION * SENSOR_SENSITIVITY;
  TiltAngle_Y = ((float) offset_Y) * ADC_RESOLUTION * SENSOR_SENSITIVITY;
  TiltAngle_Z = ((float) offset_Z) * ADC_RESOLUTION * SENSOR_SENSITIVITY;
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
