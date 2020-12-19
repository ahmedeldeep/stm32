/*******************************************************************************
 * @file    LIN_Slave.c
 * @author  Ahmed Eldeep
 * @email   ahmed@almohandes.org
 * @website http://almohandes.org/stm32
 * @date    17.06.2018
 *          
 * @brief   LIN slave node driver
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
#include <stddef.h>
#include "gpio.h"
#include "LIN_Slave.h"

/**
 * @addtogroup stm32_examples
 * @{
 */

/**
 * @defgroup LIN_Slave
 * @brief
 * @{
 */

/**
 * @defgroup LIN_Slave_private_typedefs
 * @{
 */

/**
 * @brief   LIN slave states definition
 */
typedef enum
{
  LINS_IDLE,
  LINS_BREAK_RECEIVED,
  LINS_WAIT_FOR_HEADER,
  LINS_HEADER_RECEIVED,
  LINS_RX_DATA,
  LINS_WAIT_FOR_RX_DATA,
  LINS_RX_DATA_RECEIVED
} LIN_Slave_StateType;

/**
 * @brief   LIN slave error status definition
 */

typedef enum
{
  LINS_NO_ERROR,
  LINS_SYNC_ERROR,
  LINS_CHECKSUM_INVALID_ERROR
} LIN_Slave_ErrorStatusType;

/**
 * @}
 */

/**
 * @defgroup LIN_Slave_private_defines
 * @{
 */

/**
 * @brief   AF8 PC12 pin masks
 */
#define GPIO_AFRH_AFRH12                     ((uint32_t) 0x000F0000)
#define GPIO_AFRH_AFRH12_AF8                 ((uint32_t) 0x00080000)

/**
 * @brief   AF8 PD2 pin masks
 */
#define GPIO_AFRL_AFRL2                      ((uint32_t) 0x00000F00)
#define GPIO_AFRL_AFRL2_AF8                  ((uint32_t) 0x00000800)

/**
 * @brief   LIN Header Length
 */
#define LIN_HEADER_LENGTH                    ((uint8_t) 2u)

/**
 * @brief   LIN data length including the checksum
 */
#define LIN_DATA_LENGTH                      ((uint8_t) 3u)

/**
 * @}
 */

/**
 * @defgroup LIN_Slave_private_macros
 * @{
 */

/**
 * @}
 */

/**
 * @defgroup LIN_Slave_private_constants
 * @{
 */

/**
 * @}
 */

/**
 * @defgroup LIN_Slave_private_variables
 * @{
 */

/**
 * @brief   LIN slave current state
 */
static LIN_Slave_StateType currentState = LINS_IDLE;

/**
 * @brief   LIN slave current error status
 */
static LIN_Slave_ErrorStatusType currentErrorStatus = LINS_NO_ERROR;

/**
 * @brief   LIN Header, 2 bytes
 */
static uint8_t linHeader[LIN_HEADER_LENGTH];

/**
 * @brief   LIN Data, 3 bytes
 */
static uint8_t linData[LIN_DATA_LENGTH];

/**
 * @brief   DMA received flag
 */
static uint8_t RxDMAReceived = 0;

/**
 * @}
 */

/**
 * @defgroup LIN_Slave_private_function_prototypes
 * @{
 */

/**
 * @brief   DMA receive
 * @note
 * @param   data, size
 * @retval  None
 */
static void DMAReceive(const uint8_t * data, uint8_t size);

/**
 * @}
 */

/**
 * @defgroup LIN_Slave_private_functions
 * @{
 */

/**
 * @brief   DMA receive
 * @note
 * @param   data, size
 * @retval  None
 */
static void DMAReceive(const uint8_t * data, uint8_t size)
{
  /* Check null pointers */
  if(NULL != data)
  {
    /* Wait until DMA1 stream 0 is disabled */
    while(DMA_SxCR_EN == (DMA_SxCR_EN & DMA1_Stream0->CR))
    {
      /* Do nothing, the enable flag shall reset
       * when DMA transfer complete */
    }

    /* Set memory address */
    DMA1_Stream0->M0AR = (uint32_t)data;

    /* Set number of data items */
    DMA1_Stream0->NDTR = size;

    /* Clear all interrupt flags */
    DMA1->LIFCR = (DMA_LIFCR_CFEIF0 | DMA_LIFCR_CDMEIF0 | DMA_LIFCR_CTEIF0
        | DMA_LIFCR_CHTIF0 | DMA_LIFCR_CTCIF0);

    /* Clear any UART pending DMA requests */
    UART5->CR3 &= ~USART_CR3_DMAR;

    /* Enable DMA mode for reception */
    UART5->CR3 |= USART_CR3_DMAR;

    /* Enable DMA 1 stream 0 */
    DMA1_Stream0->CR |= DMA_SxCR_EN;
  }
  else
  {
    /* Null pointers, do nothing */
  }
}

/**
 * @}
 */

/**
 * @defgroup LIN_Slave_exported_functions
 * @{
 */

/**
 * @brief   Configure GPIO
 * @note    UART5_TX -> PC12, UART5_RX -> PD2
 *          UART5 mapped to alternate function AF8
 *          UART5 connected to APB1 with 45MHz max clock
 * @param   None
 * @retval  None
 */
void LIN_Slave_GPIO_Init(void)
{
  /* Enable port C and D clocks */
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;

  /* Select alternate function mode */
  GPIOC->MODER &= ~(GPIO_MODER_MODER12);
  GPIOC->MODER |= GPIO_MODER_MODER12_1;
  GPIOD->MODER &= ~(GPIO_MODER_MODER2);
  GPIOD->MODER |= GPIO_MODER_MODER2_1;

  /* Select output type push-pull */
  GPIOC->OTYPER &= ~(GPIO_OTYPER_OT_12);

  /* Select output speed medium */
  GPIOC->OSPEEDR &= ~(GPIO_OSPEEDER_OSPEEDR12);
  GPIOC->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR12_0;

  /* Select no pull-up, pull-down */
  GPIOC->PUPDR &= ~(GPIO_PUPDR_PUPDR12);
  GPIOD->PUPDR &= ~(GPIO_PUPDR_PUPDR2);

  /* Select AF8 */
  GPIOC->AFR[1] &= ~(GPIO_AFRH_AFRH12);
  GPIOC->AFR[1] |= GPIO_AFRH_AFRH12_AF8;
  GPIOD->AFR[0] &= ~(GPIO_AFRL_AFRL2);
  GPIOD->AFR[0] |= GPIO_AFRL_AFRL2_AF8;
}

/**
 * @brief   Configure UART5
 * @note
 * @param   None
 * @retval  None
 */
void LIN_Slave_UART5_Init(void)
{
  /* Enable UART5 clock */
  RCC->APB1ENR = RCC_APB1ENR_UART5EN;

  /* Select oversampling by 16 mode */
  UART5->CR1 &= ~USART_CR1_OVER8;

  /* Select 1 Start bit, 8 Data bits, n Stop bit */
  UART5->CR1 &= ~USART_CR1_M;

  /* Select 1 stop bit */
  UART5->CR2 &= ~USART_CR2_STOP;

  /* Select three sample bit method */
  UART5->CR3 &= ~USART_CR3_ONEBIT;

  /* Select LIN mode */
  UART5->CR2 |= USART_CR2_LINEN;

  /* Select LIN break detection length 11 bits */
  UART5->CR2 |= USART_CR2_LBDL;

  /* Enable LIN break detection interrupt */
  UART5->CR2 |= USART_CR2_LBDIE;

  /* Set baud rate = 9600 Bps
   * USARTDIV = Fck / (16 * baud_rate)
   *          = 45000000 / (16 * 9600) = 292.96
   *
   * DIV_Fraction = 16 * 0.96 = 15.36 = 15 = 0xF
   * DIV_Mantissa = 292 = 0x124
   *
   * BRR          = 0x124F */
  UART5->BRR = 0x124F;
}

/**
 * @brief   Configure DMA for UART TX
 * @note    UART5_TX -> DMA1_Stream7 (Channel 4)
 * @param   None
 * @retval  None
 */
void LIN_Slave_TX_DMA_Init(void)
{
  /* Enable DMA clock in RCC */
  RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN;

  /* Make sure that the DMA1 stream 7 is disabled */
  if(DMA_SxCR_EN == (DMA_SxCR_EN & DMA1_Stream7->CR))
  {
    /* DMA 1 stream 7 is enabled, shall be disabled first */
    DMA1_Stream7->CR &= ~DMA_SxCR_EN;

    /* Wait until EN bit is cleared */
    while(DMA_SxCR_EN == (DMA_SxCR_EN & DMA1_Stream7->CR))
    {
      /* Do nothing until EN bit is cleared */
    }
  }
  else
  {
    /* Do nothing, stream 7 is not enabled */
  }

  /* Select the DMA channel 4 in CHSEL[2:0] in the DMA_SxCR */
  DMA1_Stream7->CR &= ~DMA_SxCR_CHSEL;
  DMA1_Stream7->CR |= DMA_SxCR_CHSEL_2;

  /* Select stream priority very high */
  DMA1_Stream7->CR |= DMA_SxCR_PL;

  /* Select the data transfer direction memory-to-peripheral */
  DMA1_Stream7->CR &= ~DMA_SxCR_DIR;
  DMA1_Stream7->CR |= DMA_SxCR_DIR_0;

  /* Select memory and peripherals sizes byte (8-bit) */
  DMA1_Stream7->CR &= ~DMA_SxCR_MSIZE;
  DMA1_Stream7->CR &= ~DMA_SxCR_PSIZE;

  /* Select FIFO mode */
  DMA1_Stream7->FCR |= DMA_SxFCR_DMDIS;

  /* Select FIFO threshold level half FIFO */
  DMA1_Stream7->FCR |= (DMA_SxFCR_FTH_0 | DMA_SxFCR_FTH_1);

  /* Select memory incremented mode, peripheral shall has fixed address */
  DMA1_Stream7->CR |= DMA_SxCR_MINC;

  /* Enable DMA transfer complete interrupt */
  DMA1_Stream7->CR |= DMA_SxCR_TCIE;

  /* Set peripheral address */
  DMA1_Stream7->PAR = (uint32_t)&UART5->DR;
}

/**
 * @brief   Configure DMA for UART RX
 * @note    UART5_RX -> DMA1_Stream0 (Channel 4)
 * @param   None
 * @retval  None
 */
void LIN_Slave_RX_DMA_Init(void)
{
  /* Enable DMA clock in RCC */
  RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN;

  /* Make sure that the DMA1 stream 0 is disabled */
  if(DMA_SxCR_EN == (DMA_SxCR_EN & DMA1_Stream0->CR))
  {
    /* DMA 1 stream 0 is enabled, shall be disabled first */
    DMA1_Stream0->CR &= ~DMA_SxCR_EN;

    /* Wait until EN bit is cleared */
    while(DMA_SxCR_EN == (DMA_SxCR_EN & DMA1_Stream0->CR))
    {
      /* Do nothing until EN bit is cleared */
    }
  }
  else
  {
    /* Do nothing, stream 5 is not enabled */
  }

  /* Select the DMA channel 4 in CHSEL[2:0] in the DMA_SxCR */
  DMA1_Stream0->CR &= ~DMA_SxCR_CHSEL;
  DMA1_Stream0->CR |= DMA_SxCR_CHSEL_2;

  /* Select stream priority very high */
  DMA1_Stream0->CR |= DMA_SxCR_PL;

  /* Select the data transfer direction peripheral-to-memory */
  DMA1_Stream0->CR &= ~DMA_SxCR_DIR;

  /* Select memory and peripherals sizes byte (8-bit) */
  DMA1_Stream0->CR &= ~DMA_SxCR_MSIZE;
  DMA1_Stream0->CR &= ~DMA_SxCR_PSIZE;

  /* Select FIFO mode */
  DMA1_Stream0->FCR |= DMA_SxFCR_DMDIS;

  /* Select FIFO threshold level half FIFO */
  DMA1_Stream0->FCR |= (DMA_SxFCR_FTH_0 | DMA_SxFCR_FTH_1);

  /* Select memory incremented mode, peripheral shall has fixed address */
  DMA1_Stream0->CR |= DMA_SxCR_MINC;

  /* Enable DMA transfer complete interrupt */
  DMA1_Stream0->CR |= DMA_SxCR_TCIE;

  /* Set peripheral address */
  DMA1_Stream0->PAR = (uint32_t)&UART5->DR;
}

/**
 * @brief   Enable LIN slave node communications
 * @note
 * @param   None
 * @retval  None
 */
void LIN_Slave_Enable(void)
{
  /* Enable UART5 */
  UART5->CR1 |= USART_CR1_UE;

  /* Enable transmitter */
  UART5->CR1 |= USART_CR1_TE;

  /* Enable receiver */
  UART5->CR1 |= USART_CR1_RE;
}

/**
 * @brief   IRQ callback function
 * @note
 * @param   None
 * @retval  None
 */
void LIN_Slave_RX_DMA_IRQ_Callback(void)
{
  /* Check transfer complete flag */
  if(DMA_LISR_TCIF0 == (DMA_LISR_TCIF0 & DMA1->LISR))
  {
    /* Clear all interrupt flags */
    DMA1->LIFCR = (DMA_LIFCR_CFEIF0 | DMA_LIFCR_CDMEIF0 | DMA_LIFCR_CTEIF0
        | DMA_LIFCR_CHTIF0 | DMA_LIFCR_CTCIF0);

    /* Set Rx flag */
    RxDMAReceived = 1;
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
void LIN_Slave_TX_DMA_IRQ_Callback(void)
{
  /* Check transfer complete flag */
  if(DMA_HISR_TCIF7 == (DMA_HISR_TCIF7 & DMA1->HISR))
  {
    /* Clear all interrupt flags */
    DMA1->HIFCR = (DMA_HIFCR_CFEIF7 | DMA_HIFCR_CDMEIF7 | DMA_HIFCR_CTEIF7
        | DMA_HIFCR_CHTIF7 | DMA_HIFCR_CTCIF7);
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
void LIN_Slave_UART5_IRQ_Callback(void)
{
  /* Check if LIN break detected */
  if((UART5->SR & USART_SR_LBD) == USART_SR_LBD)
  {
    /* Clear LIN break detection flag */
    UART5->SR &= ~(USART_SR_LBD);

    /* Clear read data register not empty */
    UART5->SR &= ~(USART_SR_RXNE);

    /* Set LIN break detected */
    currentState = LINS_BREAK_RECEIVED;
  }
  else
  {
    /* No new LIN Frame received */
  }
}

/**
 * @brief   LIN slave node task
 * @note    This function shall be called periodically.
 * @param   None
 * @retval  None
 */
void LIN_Slave_Process(void)
{
  /* Check current LIN slave state */
  switch (currentState)
  {
    case LINS_IDLE:
      /* Wait for LIN break */
      break;

    case LINS_BREAK_RECEIVED:
      /* Start DMA for LIN header reception */
      DMAReceive(linHeader, LIN_HEADER_LENGTH);

      /* Wait for LIN header */
      currentState = LINS_WAIT_FOR_HEADER;
      break;

    case LINS_WAIT_FOR_HEADER:
      /* Wait for LIN header */
      if(1 == RxDMAReceived)
      {
        /* New DMA data was received, reset DMA flag */
        RxDMAReceived = 0;

        /* Go to next state */
        currentState = LINS_HEADER_RECEIVED;
      }
      else
      {
        /* Do nothing */
      }
      break;

    case LINS_HEADER_RECEIVED:
      /* Check sync field */
      if(0x55 == linHeader[0])
      {
        /* Sync is OK, Check PID */
        /* We expecting only one frame with PID = 0xCF */
        if(0xCF == linHeader[1])
        {
          /* PID is OK, go to RX data */
          currentState = LINS_RX_DATA;
        }
        else
        {
          /* PID is unknown, go to idle */
          currentState = LINS_IDLE;
        }
      }
      else
      {
        /* Sync field not OK, go to idle */
        currentErrorStatus = LINS_SYNC_ERROR;
        currentState = LINS_IDLE;
      }
      break;

    case LINS_RX_DATA:
      /* Start DMA for LIN header reception */
      DMAReceive(linData, LIN_DATA_LENGTH);

      /* Wait for LIN Data reception */
      currentState = LINS_WAIT_FOR_RX_DATA;
      break;

    case LINS_WAIT_FOR_RX_DATA:
      /* Wait for LIN data */
      if(1 == RxDMAReceived)
      {
        /* New DMA data was received, reset DMA flag */
        RxDMAReceived = 0;

        /* Go to next state */
        currentState = LINS_RX_DATA_RECEIVED;
      }
      else
      {
        /* Do nothing */
      }
      break;

    case LINS_RX_DATA_RECEIVED:
      /* Check the received checksum, shall be calculated based
       * on the received data */
      if((0xCA == linData[2]) || (0xEA == linData[2]))
      {
        /* Checksum is OK, check required command for the green LED */
        if(0x0A == linData[0])
        {
          /* Turn off green LED */
          GPIO_TurnOFF_LED(EVAL_GREEN_LED);
        }
        else if(0x1A == linData[0])
        {
          /* Turn on green LED */
          GPIO_TurnON_LED(EVAL_GREEN_LED);
        }
        else
        {
          /* Do nothing */
        }

        /* Command for the red LED */
        if(0x0B == linData[1])
        {
          /* Turn off red LED */
          GPIO_TurnOFF_LED(EVAL_RED_LED);
        }
        else if(0x1B == linData[1])
        {
          /* Turn on red LED */
          GPIO_TurnON_LED(EVAL_RED_LED);
        }
        else
        {
          /* Do nothing */
        }

        /* Reset error state and go to idle */
        currentState = LINS_IDLE;
        currentErrorStatus = LINS_NO_ERROR;
      }
      else
      {
        /* Checksum error, go to idle */
        currentState = LINS_IDLE;
        currentErrorStatus = LINS_CHECKSUM_INVALID_ERROR;
      }
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
