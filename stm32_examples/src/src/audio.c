/*******************************************************************************
 * @file    audio.c
 * @author  Ahmed Eldeep
 * @email   ahmed@almohandes.org
 * @website http://almohandes.org/stm32
 * @date    22.12.2018
 *          
 * @brief   Audio recording and playing example
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
#include "audio.h"
#include "SysTick.h"
#include "gpio.h"

/**
 * @addtogroup stm32_examples
 * @{
 */

/**
 * @defgroup audio
 * @brief
 * @{
 */

/**
 * @defgroup audio_private_typedefs
 * @{
 */

/**
 * @brief   Current Audio mode
 */
typedef enum AUDIO_Mode
{
  AUDIO_MODE_IDLE,
  AUDIO_MODE_START_RECORDING,
  AUDIO_MODE_RECORDING,
  AUDIO_MODE_START_PLAYING,
  AUDIO_MODE_PLAYING,
} AUDIO_ModeType;

/**
 * @}
 */

/**
 * @defgroup audio_private_defines
 * @{
 */

/**
 * @brief   Timeout value in ms for the PB to remain pressed
 *          for switch to recording mode
 */
#define PB_PRESSED_TIMEOUT                   ((uint32_t) 3000)

/**
 * @brief   Number of audio samples, recording length = 8.19s
 */
#define NUM_OF_AUDIO_SAMPLES                 ((uint32_t) 0xFFFF)


/**
 * @}
 */

/**
 * @defgroup audio_private_macros
 * @{
 */

/**
 * @}
 */

/**
 * @defgroup audio_private_constants
 * @{
 */

/**
 * @}
 */

/**
 * @defgroup audio_private_variables
 * @{
 */

/**
 * @brief   Current audio mode
 */
static AUDIO_ModeType currentMode = AUDIO_MODE_IDLE;

/**
 * @brief   Recorded sound data
 */
static uint16_t recordedSound[NUM_OF_AUDIO_SAMPLES] = {0};

/**
 * @brief   Recording finished flag
 */
static uint8_t recordingFinished = 0;

/**
 * @brief   Playing finished flag
 */
static uint8_t playingFinished = 0;

/**
 * @}
 */

/**
 * @defgroup audio_private_function_prototypes
 * @{
 */

/**
 * @}
 */

/**
 * @defgroup audio_private_functions
 * @{
 */

/**
 * @}
 */

/**
 * @defgroup audio_exported_functions
 * @{
 */

/**
 * @brief   Audio Configuration function
 * @note    Microphone input -> PC3 -> ADC1_IN13
 *          Speaker output   -> DAC_OUT2 -> PA5
 * @param
 * @retval
 */
void AUDIO_ADC_Config(void)
{
  /* GPIO Configuration for PC3 */
  /* ********************************** */
  /* Enable port C clock */
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;

  /* Select Analog mode */
  GPIOC->MODER |= GPIO_MODER_MODER3;

  /* Select no pull-up, pull-down (reset state) */
  GPIOC->PUPDR &= ~GPIO_PUPDR_PUPDR3;

  /* ADC Configuration for PC3 -> ADC1_IN13 */
  /* ************************************** */
  /* Enable ADC1 clock */
  RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;

  /* Enable ADC */
  ADC1->CR2 |= ADC_CR2_ADON;

  /* Select Timer 2 TRGO event as external event for regular group */
  ADC1->CR2 &= ~ADC_CR2_EXTSEL;
  ADC1->CR2 |= (ADC_CR2_EXTSEL_1 | ADC_CR2_EXTSEL_2);

  /* Select ADC1_IN13 regular */
  ADC1->SQR1 &= ~ADC_SQR1_L;
  ADC1->SQR3 &= ~ADC_SQR3_SQ1;
  ADC1->SQR3 |= (ADC_SQR3_SQ1_0 | ADC_SQR3_SQ1_2 | ADC_SQR3_SQ1_3);

  /* Continuous DMA requests */
  ADC1->CR2 |= ADC_CR2_DDS;

  /* Enable DMA mode in ADC */
  ADC1->CR2 |= ADC_CR2_DMA;

  /* DMA configuration ADC1 -> DMA2_Stream0 (Channel 0) */
  /* ************************************************** */
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

  /* Enable DMA transfer complete interrupt */
  DMA2_Stream0->CR |= DMA_SxCR_TCIE;

  /* Set peripheral address */
  DMA2_Stream0->PAR = (uint32_t)&ADC1->DR;

  /* Set memory address */
  DMA2_Stream0->M0AR = (uint32_t)recordedSound;

  /* Set number of data items */
  DMA2_Stream0->NDTR = NUM_OF_AUDIO_SAMPLES;
}

/**
 * @brief   Audio Configuration function
 * @note    Microphone input -> PC3 -> ADC1_IN13
 *          Speaker output   -> DAC_OUT2 -> PA5
 * @param
 * @retval
 */
void AUDIO_DAC_Config(void)
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

  /* DAC channel2 enable */
  DAC->CR |= DAC_CR_EN2;

  /* Select external trigger Timer 2 TRGO event */
  DAC->CR &= ~DAC_CR_TSEL2;
  DAC->CR |= DAC_CR_TSEL2_2;

  /* DAC channel2 DMA enable */
  DAC->CR |= DAC_CR_DMAEN2;

  /* DMA1 Stream 6 Channel 7 Configuration */
  /* ************************************* */
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

  /* Enable DMA transfer complete interrupt */
  DMA1_Stream6->CR |= DMA_SxCR_TCIE;

  /* Set peripheral address - 12-bits right alignment */
  DMA1_Stream6->PAR = (uint32_t)&DAC->DHR12R2;

  /* Set memory address */
  DMA1_Stream6->M0AR = (uint32_t)recordedSound;

  /* Set number of data items, each conversion 3 requests */
  DMA1_Stream6->NDTR = NUM_OF_AUDIO_SAMPLES;
}

/**
 * @brief   Audio Configuration function
 * @note
 * @param
 * @retval
 */
void AUDIO_Timer_Config(void)
{
  /* Timer 2 TRGO event Configuration */
  /* ******************************** */
  /* Enable TIM2 clock */
  RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;

  /* Set counter direction as up-counter */
  TIM2->CR1 &= ~TIM_CR1_DIR;

  /* Set timer Prescaler, bus clock = 45 MHz, Tim_Clock = 90 MHz
   * fCK_PSC / (PSC[15:0] + 1)
   * CK_CNT = 90000000 / (89 + 1) -> 1000000 Hz -> time base = 1 us */
  TIM2->PSC = 89;

  /* Set timer reload value, update event each 125 us */
  TIM2->ARR = 125;

  /* Select update event as trigger output (TRGO) */
  TIM2->CR2 &= ~TIM_CR2_MMS;
  TIM2->CR2 |= TIM_CR2_MMS_1;

  /* Enable TIM2 counter */
  TIM2->CR1 |= TIM_CR1_CEN;
}

/**
 * @brief   Audio main function
 * @note
 * @param
 * @retval
 */
void AUDIO_Main(void)
{
  /* Check current mode */
  switch (currentMode)
  {
    case AUDIO_MODE_IDLE:
      break;

    case AUDIO_MODE_START_PLAYING:
      /* Turn on green led */
      GPIO_TurnON_LED(EVAL_GREEN_LED);

      /* Enable DMA1_Stream6 */
      DMA1_Stream6->CR |= DMA_SxCR_EN;

      /* Enable external trigger */
      DAC->CR |= DAC_CR_TEN2;

      /* Change current mode */
      currentMode = AUDIO_MODE_PLAYING;
      break;

    case AUDIO_MODE_PLAYING:
      /* Check playing finished flag */
      if(1 == playingFinished)
      {
        /* Disable external trigger */
        DAC->CR &= ~DAC_CR_TEN2;

        GPIO_TurnOFF_LED(EVAL_GREEN_LED);

        /* Reset playing finished flag */
        playingFinished = 0;

        /* Change current mode */
        currentMode = AUDIO_MODE_IDLE;
      }
      else
      {
        /* Do nothing, still playing */
      }
      break;

    case AUDIO_MODE_START_RECORDING:
      /* Turn on red led */
      GPIO_TurnON_LED(EVAL_RED_LED);

      /* Enable DMA2_Stream0 */
      DMA2_Stream0->CR |= DMA_SxCR_EN;

      /* Enable external trigger */
      ADC1->CR2 |= ADC_CR2_EXTEN_0;

      /* Change current mode */
      currentMode = AUDIO_MODE_RECORDING;
      break;

    case AUDIO_MODE_RECORDING:
      /* Check recording finished flag */
      if(1 == recordingFinished)
      {
        /* Disable external trigger */
        ADC1->CR2 &= ~ADC_CR2_EXTEN;

        GPIO_TurnOFF_LED(EVAL_RED_LED);

        /* Reset recording finished flag */
        recordingFinished = 0;

        /* Change current mode */
        currentMode = AUDIO_MODE_IDLE;
      }
      else
      {
        /* Do nothing, still recording */
      }
      break;

    default:
      break;
  }
}

/**
 * @brief   PB Callback function
 * @note
 * @param
 * @retval
 */
void AUDIO_PB_Callback(void)
{
  /* Current system tick */
  uint32_t currentTick = 0;

  /* Clear pending bit */
  EXTI->PR |= EXTI_PR_PR0;

  /* Get start tick */
  uint32_t startTick = SysTick_GetCurrentTick();

  /* Set current mode to playing */
  currentMode = AUDIO_MODE_START_PLAYING;

  /* Check if the button is still pressed for 5 seconds */
  while(GPIO_IDR_IDR_0 == (GPIOA->IDR & GPIO_IDR_IDR_0))
  {
    /* Get current tick */
    currentTick = SysTick_GetCurrentTick();

    /* Check Timeout */
    if((currentTick - startTick) >= PB_PRESSED_TIMEOUT)
    {
      /* Switch current mode to recording */
      currentMode = AUDIO_MODE_START_RECORDING;
    }
    else
    {
      /* Do nothing */
    }
  }
}

/**
 * @brief   IRQ callback function
 * @note
 * @param   None
 * @retval  None
 */
void AUDIO_DMA2_Stream0_Callback(void)
{
  /* Check transfer complete flag */
  if(DMA_LISR_TCIF0 == (DMA_LISR_TCIF0 & DMA2->LISR))
  {
    /* Clear all interrupt flags */
    DMA2->LIFCR = (DMA_LIFCR_CFEIF0 | DMA_LIFCR_CDMEIF0 | DMA_LIFCR_CTEIF0
        | DMA_LIFCR_CHTIF0 | DMA_LIFCR_CTCIF0);

    /* Set Recording finished flag */
    recordingFinished = 1;
  }
  else
  {
    /* Do nothing, this interrupt is not handled */
  }
}

/**
 * @brief   IRQ callback function
 * @note
 * @param   None
 * @retval  None
 */
void AUDIO_DMA1_Stream6_Callback(void)
{
  /* Check transfer complete flag */
  if(DMA_HISR_TCIF6 == (DMA_HISR_TCIF6 & DMA1->HISR))
  {
    /* Clear all interrupt flags */
    DMA1->HIFCR = (DMA_HIFCR_CFEIF6 | DMA_HIFCR_CDMEIF6 | DMA_HIFCR_CTEIF6
        | DMA_HIFCR_CHTIF6 | DMA_HIFCR_CTCIF6);

    /* Set playing finished flag */
    playingFinished = 1;
  }
  else
  {
    /* Do nothing, this interrupt is not handled */
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
