/*******************************************************************************
 * @file    dac.c
 * @author  Ahmed Eldeep
 * @email   ahmed@almohandes.org
 * @website http://almohandes.org/stm32
 * @date    16.12.2018
 *          
 * @brief   DAC examples
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
#include "dac.h"

/**
 * @addtogroup stm32_examples
 * @{
 */

/**
 * @defgroup dac
 * @brief
 * @{
 */

/**
 * @defgroup dac_private_typedefs
 * @{
 */

/**
 * @brief   DAC generated output
 */
typedef enum DAC_Output
{
  DAC_OUTPUT_NO_WAVE,
  DAC_OUTPUT_NOISE,
  DAC_OUTPUT_TRIANGLE_WAVE,
  DAC_OUTPUT_SINE_WAVE,
} DAC_OutputType;

/**
 * @}
 */

/**
 * @defgroup dac_private_defines
 * @{
 */

/**
 * @}
 */

/**
 * @defgroup dac_private_macros
 * @{
 */

/**
 * @}
 */

/**
 * @defgroup dac_private_constants
 * @{
 */
static const uint16_t sineWave[256] =
{
  0x800,0x832,0x864,0x897,0x8c9,0x8fb,0x92d,0x95f,
  0x990,0x9c2,0x9f3,0xa24,0xa54,0xa84,0xab4,0xae3,
  0xb12,0xb40,0xb6e,0xb9b,0xbc8,0xbf4,0xc20,0xc4b,
  0xc75,0xc9e,0xcc7,0xcef,0xd17,0xd3d,0xd63,0xd88,
  0xdac,0xdcf,0xdf1,0xe12,0xe33,0xe52,0xe71,0xe8e,
  0xeaa,0xec6,0xee0,0xef9,0xf11,0xf28,0xf3e,0xf53,
  0xf67,0xf79,0xf8b,0xf9b,0xfaa,0xfb8,0xfc4,0xfd0,
  0xfda,0xfe3,0xfea,0xff1,0xff6,0xffa,0xffd,0xfff,
  0xfff,0xffe,0xffc,0xff8,0xff4,0xfee,0xfe7,0xfde,
  0xfd5,0xfca,0xfbe,0xfb1,0xfa2,0xf93,0xf82,0xf70,
  0xf5d,0xf49,0xf33,0xf1d,0xf05,0xeed,0xed3,0xeb8,
  0xe9c,0xe7f,0xe61,0xe43,0xe23,0xe02,0xde0,0xdbd,
  0xd9a,0xd75,0xd50,0xd2a,0xd03,0xcdb,0xcb3,0xc8a,
  0xc60,0xc35,0xc0a,0xbde,0xbb2,0xb85,0xb57,0xb29,
  0xafb,0xacc,0xa9c,0xa6c,0xa3c,0xa0b,0x9da,0x9a9,
  0x978,0x946,0x914,0x8e2,0x8b0,0x87e,0x84b,0x819,
  0x7e6,0x7b4,0x781,0x74f,0x71d,0x6eb,0x6b9,0x687,
  0x656,0x625,0x5f4,0x5c3,0x593,0x563,0x533,0x504,
  0x4d6,0x4a8,0x47a,0x44d,0x421,0x3f5,0x3ca,0x39f,
  0x375,0x34c,0x324,0x2fc,0x2d5,0x2af,0x28a,0x265,
  0x242,0x21f,0x1fd,0x1dc,0x1bc,0x19e,0x180,0x163,
  0x147,0x12c,0x112,0xfa,0xe2,0xcc,0xb6,0xa2,
  0x8f,0x7d,0x6c,0x5d,0x4e,0x41,0x35,0x2a,
  0x21,0x18,0x11,0xb,0x7,0x3,0x1,0x0,
  0x0,0x2,0x5,0x9,0xe,0x15,0x1c,0x25,
  0x2f,0x3b,0x47,0x55,0x64,0x74,0x86,0x98,
  0xac,0xc1,0xd7,0xee,0x106,0x11f,0x139,0x155,
  0x171,0x18e,0x1ad,0x1cc,0x1ed,0x20e,0x230,0x253,
  0x277,0x29c,0x2c2,0x2e8,0x310,0x338,0x361,0x38a,
  0x3b4,0x3df,0x40b,0x437,0x464,0x491,0x4bf,0x4ed,
  0x51c,0x54b,0x57b,0x5ab,0x5db,0x60c,0x63d,0x66f,
  0x6a0,0x6d2,0x704,0x736,0x768,0x79b,0x7cd,0x800
};

/**
 * @}
 */

/**
 * @defgroup dac_private_variables
 * @{
 */

/**
 * @brief   DAC current output generation
 */
static DAC_OutputType currentOutput = DAC_OUTPUT_NO_WAVE;

/**
 * @}
 */

/**
 * @defgroup dac_private_function_prototypes
 * @{
 */

/**
 * @}
 */

/**
 * @defgroup dac_private_functions
 * @{
 */

/**
 * @}
 */

/**
 * @defgroup dac_exported_functions
 * @{
 */

/**
 * @brief   DAC Configuration function
 * @note    DAC_OUT2 -> PA5
 * @param   
 * @retval
 */
void DAC_Config(void)
{
  /* GPIO Configuration for PA5 */
  /* ************************** */
  /* Enable port A clock */
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;

  /* Select Analog mode */
  GPIOA->MODER |= GPIO_MODER_MODER5;

  /* Select no pull-up, pull-down (reset state) */
  GPIOA->PUPDR &= ~GPIO_PUPDR_PUPDR5;

  /* DAC Configuration DAC_OUT2 */
  /* ************************** */
  /* Enable DAC clock */
  RCC->APB1ENR |= RCC_APB1ENR_DACEN;

  /* DAC channel2 unmask all amplitude bits */
  DAC->CR |= DAC_CR_MAMP2;

  /* Select no wave generation */
  DAC->CR &= ~DAC_CR_WAVE2;

  /* Select external trigger Timer 2 TRGO event */
  DAC->CR &= ~DAC_CR_TSEL2;
  DAC->CR |= DAC_CR_TSEL2_2;

  /* DAC channel2 enable */
  DAC->CR |= DAC_CR_EN2;

  /* DMA1 Stream 6 Channel 7 Circular mode Configuration */
  /* *************************************************** */
  /* Enable DMA clock in RCC */
  RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN;

  /* Make sure that the DMA1_Stream6 is disabled */
  if(DMA_SxCR_EN == (DMA_SxCR_EN & DMA1_Stream6->CR))
  {
    /* DMA1_Stream6 is enabled, shall be disabled first */
    DMA1_Stream6->CR &= ~DMA_SxCR_EN;

    /* Wait until EN bit is cleared */
    while(DMA_SxCR_EN == (DMA_SxCR_EN & DMA1_Stream6->CR))
    {
      /* Do nothing until EN bit is cleared */
    }
  }
  else
  {
    /* Do nothing, DMA1_Stream6 is not enabled */
  }

  /* Select the DMA channel 7 in CHSEL[2:0] in the DMA_SxCR */
  DMA1_Stream6->CR |= DMA_SxCR_CHSEL;

  /* Select stream priority very high */
  DMA1_Stream6->CR |= DMA_SxCR_PL;

  /* Select the data transfer direction memory-to-peripheral */
  DMA1_Stream6->CR &= ~DMA_SxCR_DIR;
  DMA1_Stream6->CR |= DMA_SxCR_DIR_0;

  /* Select memory and peripherals sizes 2 bytes*/
  DMA1_Stream6->CR |= DMA_SxCR_MSIZE_0;
  DMA1_Stream6->CR |= DMA_SxCR_PSIZE_0;

  /* Select FIFO mode */
  DMA1_Stream6->FCR |= DMA_SxFCR_DMDIS;

  /* Select FIFO threshold level half FIFO */
  DMA1_Stream6->FCR |= (DMA_SxFCR_FTH_0 | DMA_SxFCR_FTH_1);

  /* Select memory incremented mode, peripheral shall has fixed address */
  DMA1_Stream6->CR |= DMA_SxCR_MINC;

  /* Select circular mode */
  DMA1_Stream6->CR |= DMA_SxCR_CIRC;

  /* Set peripheral address - 12-bits right alignment */
  DMA1_Stream6->PAR = (uint32_t)&DAC->DHR12R2;

  /* Set memory address */
  DMA1_Stream6->M0AR = (uint32_t)sineWave;

  /* Set number of data items, each conversion 3 requests */
  DMA1_Stream6->NDTR = 256;

  /* Enable DMA1_Stream6 */
  DMA1_Stream6->CR |= DMA_SxCR_EN;

  /* Timer 2 TRGO event Configuration */
  /* ******************************** */
  /* Enable TIM2 clock */
  RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;

  /* Set counter direction as up-counter */
  TIM2->CR1 &= ~TIM_CR1_DIR;

  /* Set timer Prescaler, bus clock = 45 MHz, Tim_Clock = 90 MHz
   * fCK_PSC / (PSC[15:0] + 1)
   * CK_CNT = 90000000 / (449 + 1) -> 200000 Hz -> time base = 5 us */
  TIM2->PSC = 4499;

  /* Set timer reload value, update event each 5 us */
  TIM2->ARR = 1;

  /* Select update event as trigger output (TRGO) */
  TIM2->CR2 &= ~TIM_CR2_MMS;
  TIM2->CR2 |= TIM_CR2_MMS_1;

  /* Enable TIM2 counter */
  TIM2->CR1 |= TIM_CR1_CEN;
}

/**
 * @brief   DAC PB Callback function
 * @note    used to switch DAC output generation
 * @param
 * @retval
 */
void DAC_PB_Callback(void)
{
  /* Clear pending bit */
  EXTI->PR |= EXTI_PR_PR0;

  /* Check current output wave */
  switch (currentOutput)
  {
    case DAC_OUTPUT_NO_WAVE:
      /* Generate noise wave */
      DAC->CR &= ~DAC_CR_WAVE2;
      DAC->CR |= DAC_CR_WAVE2_0;

      /* Change current output */
      currentOutput = DAC_OUTPUT_NOISE;
      break;

    case DAC_OUTPUT_NOISE:
      /* Generate triangle wave */
      DAC->CR &= ~DAC_CR_WAVE2;
      DAC->CR |= DAC_CR_WAVE2_1;

      /* Change current output */
      currentOutput = DAC_OUTPUT_TRIANGLE_WAVE;
      break;

    case DAC_OUTPUT_TRIANGLE_WAVE:
      /* Disable wave generation */
      DAC->CR &= ~DAC_CR_WAVE2;

      /* DAC channel2 DMA enable */
      DAC->CR |= DAC_CR_DMAEN2;

      /* Change current output */
      currentOutput = DAC_OUTPUT_SINE_WAVE;
      break;

    case DAC_OUTPUT_SINE_WAVE:
      /* DAC channel2 DMA disable */
      DAC->CR &= ~DAC_CR_DMAEN2;

      /* Reset data holding register */
      DAC->DHR12R2 = 0;

      /* Change current output */
      currentOutput = DAC_OUTPUT_NO_WAVE;
      break;

    default:
      break;
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
