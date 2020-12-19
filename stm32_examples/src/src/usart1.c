/*******************************************************************************
 * @file    usart1.c
 * @author  Ahmed Eldeep
 * @email   ahmed@almohandes.org
 * @website http://almohandes.org/stm32
 * @date    09.05.2018
 *          
 * @brief   USART1 example using ST virtual COM port (VCP)
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
#include "usart1.h"

/**
 * @addtogroup stm32_examples
 * @{
 */

/**
 * @defgroup usart1
 * @brief
 * @{
 */

/**
 * @defgroup usart1_private_typedefs
 * @{
 */

/**
 * @brief   USART1 states definition
 */
typedef enum
{
  USART1_IDLE,
  USART1_WAIT_FOR_RESPONCE,
  USART1_ASK_FOR_NAME,
  USART1_WAIT_FOR_NAME,
  USART1_WAIT_FOR_COMMAND
} USART1_StateType;

/**
 * @brief   USART1 error status definition
 */

typedef enum
{
  USART1_NO_ERROR,
  USART1_PARITY_ERROR
} USART1_ErrorStatusType;

/**
 * @brief   String compare return type
 */
typedef enum
{
  STR_NOT_EQUAL,
  STR_EQUAL
} strCmpReturnType;

/**
 * @}
 */

/**
 * @defgroup usart1_private_defines
 * @{
 */

/**
 * @brief   AF PA8 - PA12 pin masks
 */
#define GPIO_AFRH_AFRH8                      ((uint32_t) 0x0000000F)
#define GPIO_AFRH_AFRH8_AF7                  ((uint32_t) 0x00000007)
#define GPIO_AFRH_AFRH9                      ((uint32_t) 0x000000F0)
#define GPIO_AFRH_AFRH9_AF7                  ((uint32_t) 0x00000070)
#define GPIO_AFRH_AFRH10                     ((uint32_t) 0x00000F00)
#define GPIO_AFRH_AFRH10_AF7                 ((uint32_t) 0x00000700)
#define GPIO_AFRH_AFRH11                     ((uint32_t) 0x0000F000)
#define GPIO_AFRH_AFRH11_AF7                 ((uint32_t) 0x00007000)
#define GPIO_AFRH_AFRH12                     ((uint32_t) 0x000F0000)
#define GPIO_AFRH_AFRH12_AF7                 ((uint32_t) 0x00070000)

/**
 * @brief   Maximum USART reception buffer length
 */
#define MAX_BUFFER_LENGTH                     ((uint32_t) 128u)

/**
 * @}
 */

/**
 * @defgroup usart1_private_macros
 * @{
 */

/**
 * @}
 */

/**
 * @defgroup usart1_private_constants
 * @{
 */

/**
 * @brief   USART1 messages to be transmitted
 */
static char hello_world[]        = "Hello World!";
static char ask_for_name[]       = "What is your name?";
static char hi[]                 = "Hi,";
static char ask_for_command[]    = "Please, send command";
static char ask_for_command_ex[] = "Action[turn_on / turn_off] Led[green_led / red_led]";
static char turn_on_green_led[]  = "turn_on green_led";
static char turn_on_red_led[]    = "turn_on red_led";
static char turn_off_green_led[] = "turn_off green_led";
static char turn_off_red_led[]   = "turn_off red_led";
static char done[]               = "Done";
static char wrong_command[]      = "Wrong Command";
static char parity_error[]       = "Parity Error";

/**
 * @}
 */

/**
 * @defgroup usart1_private_variables
 * @{
 */

/**
 * @brief   USART1 current state
 */
static USART1_StateType currentState = USART1_IDLE;

/**
 * @brief   USART1 current error status
 */
static USART1_ErrorStatusType currentErrorStatus = USART1_NO_ERROR;

/**
 * @brief   USART1 RX message buffer
 */
static char RxBuffer[MAX_BUFFER_LENGTH];
static char RxDMABuffer[MAX_BUFFER_LENGTH];

/**
 * @brief   USART1 RX message length
 */
static uint8_t RxMessageLength = 0;

/**
 * @}
 */

/**
 * @defgroup usart1_private_function_prototypes
 * @{
 */

/**
 * @}
 */

/**
 * @defgroup usart1_private_functions
 * @{
 */

/**
 * @brief   Compare two strings
 * @note    take the size of the predefined string
 * @param   str1, str2, size
 * @retval  strCmpReturnType
 */
static strCmpReturnType strCmp(const char * str1, const char * str2,
    const uint8_t size)
{
  /* Compare status */
  strCmpReturnType cmpStatus = STR_EQUAL;

  /* Check null pointers */
  if((NULL != str1) && (NULL != str2))
  {
    /* Start comparing */
    for (int idx = 0; idx < size; idx++)
    {
      /* When not equal set the return status */
      if(str1[idx] != str2[idx])
      {
        cmpStatus = STR_NOT_EQUAL;
      }
      else
      {
        /* Do nothing */
      }
    }
  }
  else
  {
    /* Null pointers, do nothing */
  }
  return cmpStatus;
}

/**
 * @brief   DMA string transmit
 * @note
 * @param   str, size
 * @retval  None
 */
static void strTransmit(const char * str, uint8_t size)
{
  /* Check null pointers */
  if(NULL != str)
  {
    /* Wait until DMA2 stream 7 is disabled */
    while(DMA_SxCR_EN == (DMA_SxCR_EN & DMA2_Stream7->CR))
    {
      /* Do nothing, the enable flag shall reset
       * when DMA transfer complete */
    }

    /* Set memory address */
    DMA2_Stream7->M0AR = (uint32_t)str;

    /* Set number of data items */
    DMA2_Stream7->NDTR = size;

    /* Clear all interrupt flags */
    DMA2->HIFCR = (DMA_HIFCR_CFEIF7 | DMA_HIFCR_CDMEIF7 | DMA_HIFCR_CTEIF7
        | DMA_HIFCR_CHTIF7 | DMA_HIFCR_CTCIF7);

    /* Enable DMA2 stream 7 */
    DMA2_Stream7->CR |= DMA_SxCR_EN;
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
 * @defgroup usart1_exported_functions
 * @{
 */

/**
 * @brief   Configure GPIO with AF7, USART1 connected to APB2 with 90MHz clock
 * @note    USART1_CK  -> PA8  (OUT)
 *          USART1_TX  -> PA9  (OUT)
 *          USART1_RX  -> PA10 (IN)
 *          USART1_CTS -> PA11 (IN)
 *          USART1_RTS -> PA12 (OUT)
 *
 * @param   None
 * @retval  None
 */
void USART1_GPIO_Config(void)
{
  /* Enable port A clock */
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;

  /* Select alternate function mode */
  GPIOA->MODER &= ~(GPIO_MODER_MODER8 | GPIO_MODER_MODER9
      | GPIO_MODER_MODER10 | GPIO_MODER_MODER11 | GPIO_MODER_MODER12);
  GPIOA->MODER |= (GPIO_MODER_MODER8_1 | GPIO_MODER_MODER9_1
      | GPIO_MODER_MODER10_1 | GPIO_MODER_MODER11_1 | GPIO_MODER_MODER12_1);

  /* Select output type push-pull */
  GPIOA->OTYPER &= ~(GPIO_OTYPER_OT_8 | GPIO_OTYPER_OT_9
      | GPIO_OTYPER_OT_12);

  /* Select output speed medium */
  GPIOA->OSPEEDR &= ~(GPIO_OSPEEDER_OSPEEDR8 | GPIO_OSPEEDER_OSPEEDR9
      | GPIO_OSPEEDER_OSPEEDR12);
  GPIOA->OSPEEDR |= (GPIO_OSPEEDER_OSPEEDR8_0 | GPIO_OSPEEDER_OSPEEDR9_0
      | GPIO_OSPEEDER_OSPEEDR12_0);

  /* Select pull up */
  GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPDR8 | GPIO_PUPDR_PUPDR9
      | GPIO_PUPDR_PUPDR10 | GPIO_PUPDR_PUPDR11 | GPIO_PUPDR_PUPDR12);
  GPIOA->PUPDR |= (GPIO_PUPDR_PUPDR8_0 | GPIO_PUPDR_PUPDR9_0
      | GPIO_PUPDR_PUPDR10_0 | GPIO_PUPDR_PUPDR11_0 | GPIO_PUPDR_PUPDR12_0);

  /* Select AF7 */
  GPIOA->AFR[1] &= ~(GPIO_AFRH_AFRH8 | GPIO_AFRH_AFRH9
      | GPIO_AFRH_AFRH10 | GPIO_AFRH_AFRH11 | GPIO_AFRH_AFRH12);
  GPIOA->AFR[1] |= (GPIO_AFRH_AFRH8_AF7 | GPIO_AFRH_AFRH9_AF7
      | GPIO_AFRH_AFRH10_AF7 | GPIO_AFRH_AFRH11_AF7 | GPIO_AFRH_AFRH12_AF7);
}

/**
 * @brief   Configure DMA for USART TX
 * @note    USART1_TX -> DMA2_Stream7 (Channel 4)
 * @param   None
 * @retval  None
 */
void USART1_TX_DMA_Config(void)
{
  /* Enable DMA clock in RCC */
  RCC->AHB1ENR |= RCC_AHB1ENR_DMA2EN;

  /* Make sure that the DMA2 stream 7 is disabled */
  if(DMA_SxCR_EN == (DMA_SxCR_EN & DMA2_Stream7->CR))
  {
    /* DMA 2 stream 7 is enabled, shall be disabled first */
    DMA2_Stream7->CR &= ~DMA_SxCR_EN;

    /* Wait until EN bit is cleared */
    while(DMA_SxCR_EN == (DMA_SxCR_EN & DMA2_Stream7->CR))
    {
      /* Do nothing until EN bit is cleared */
    }
  }
  else
  {
    /* Do nothing, stream 7 is not enabled */
  }

  /* Select the DMA channel 4 in CHSEL[2:0] in the DMA_SxCR */
  DMA2_Stream7->CR &= ~DMA_SxCR_CHSEL;
  DMA2_Stream7->CR |= DMA_SxCR_CHSEL_2;

  /* Select stream priority very high */
  DMA2_Stream7->CR |= DMA_SxCR_PL;

  /* Select the data transfer direction memory-to-peripheral */
  DMA2_Stream7->CR &= ~DMA_SxCR_DIR;
  DMA2_Stream7->CR |= DMA_SxCR_DIR_0;

  /* Select memory and peripherals sizes byte (8-bit) */
  DMA2_Stream7->CR &= ~DMA_SxCR_MSIZE;
  DMA2_Stream7->CR &= ~DMA_SxCR_PSIZE;

  /* Select FIFO mode */
  DMA2_Stream7->FCR |= DMA_SxFCR_DMDIS;

  /* Select FIFO threshold level half FIFO */
  DMA2_Stream7->FCR |= (DMA_SxFCR_FTH_0 | DMA_SxFCR_FTH_1);

  /* Select memory incremented mode, peripheral shall has fixed address */
  DMA2_Stream7->CR |= DMA_SxCR_MINC;

  /* Enable DMA transfer complete interrupt */
  DMA2_Stream7->CR |= DMA_SxCR_TCIE;

  /* Set peripheral address */
  DMA2_Stream7->PAR = (uint32_t)&USART1->DR;
}

/**
 * @brief   Configure DMA for USART RX
 * @note    USART1_RX -> DMA2_Stream5 (Channel 4)
 * @param   None
 * @retval  None
 */
void USART1_RX_DMA_Config(void)
{
  /* Enable DMA clock in RCC */
  RCC->AHB1ENR |= RCC_AHB1ENR_DMA2EN;

  /* Make sure that the DMA2 stream 5 is disabled */
  if(DMA_SxCR_EN == (DMA_SxCR_EN & DMA2_Stream5->CR))
  {
    /* DMA 2 stream 5 is enabled, shall be disabled first */
    DMA2_Stream5->CR &= ~DMA_SxCR_EN;

    /* Wait until EN bit is cleared */
    while(DMA_SxCR_EN == (DMA_SxCR_EN & DMA2_Stream5->CR))
    {
      /* Do nothing until EN bit is cleared */
    }
  }
  else
  {
    /* Do nothing, stream 5 is not enabled */
  }

  /* Select the DMA channel 4 in CHSEL[2:0] in the DMA_SxCR */
  DMA2_Stream5->CR &= ~DMA_SxCR_CHSEL;
  DMA2_Stream5->CR |= DMA_SxCR_CHSEL_2;

  /* Select stream priority very high */
  DMA2_Stream5->CR |= DMA_SxCR_PL;

  /* Select the data transfer direction peripheral-to-memory */
  DMA2_Stream5->CR &= ~DMA_SxCR_DIR;

  /* Select memory and peripherals sizes byte (8-bit) */
  DMA2_Stream5->CR &= ~DMA_SxCR_MSIZE;
  DMA2_Stream5->CR &= ~DMA_SxCR_PSIZE;

  /* Select FIFO mode */
  DMA2_Stream5->FCR |= DMA_SxFCR_DMDIS;

  /* Select FIFO threshold level half FIFO */
  DMA2_Stream5->FCR |= (DMA_SxFCR_FTH_0 | DMA_SxFCR_FTH_1);

  /* Select memory incremented mode, peripheral shall has fixed address */
  DMA2_Stream5->CR |= DMA_SxCR_MINC;

  /* Enable DMA transfer complete interrupt */
  DMA2_Stream5->CR |= DMA_SxCR_TCIE;

  /* Set peripheral address */
  DMA2_Stream5->PAR = (uint32_t)&USART1->DR;

  /* Set memory address */
  DMA2_Stream5->M0AR = (uint32_t)RxDMABuffer;

  /* Set number of data items */
  DMA2_Stream5->NDTR = MAX_BUFFER_LENGTH;
}

/**
 * @brief   Configure USART1 for ST virtual COM port (VCP)
 * @note
 * @param   None
 * @retval  None
 */
void USART1_Init(void)
{
  /* Configure USART1 */
  /* Enable USART1 clock */
  RCC->APB2ENR = RCC_APB2ENR_USART1EN;

  /* Select oversampling by 16 mode */
  USART1->CR1 &= ~USART_CR1_OVER8;

  /* Select one sample bit method */
  USART1->CR3 |= USART_CR3_ONEBIT;

  /* Select 1 Start bit, 9 Data bits, n Stop bit */
  USART1->CR1 |= USART_CR1_M;

  /* Select 1 stop bit */
  USART1->CR2 &= ~USART_CR2_STOP;

  /* Enable parity control */
  USART1->CR1 |= USART_CR1_PCE;

  /* Select odd parity */
  USART1->CR1 |= USART_CR1_PS;

  /* Set baud rate = 115200 Bps
   * USARTDIV = Fck / (16 * baud_rate)
   *          = 90000000 / (16 * 115200) = 48.82
   *
   * DIV_Fraction = 16 * 0.82 = 13.12 = 13 = 0xD
   * DIV_Mantissa = 48 = 0x30
   *
   * BRR          = 0x30D */
  USART1->BRR = 0x30D;

//  /* Select USART synchronous mode */
//  USART1->CR2 |= USART_CR2_CLKEN;

  /* Select clock polarity low outside transmission window */
  USART1->CR2 &= ~USART_CR2_CPOL;

  /* Select clock phase first clock transition is the capture edge */
  USART1->CR2 &= ~USART_CR2_CPHA;

  /* Disable clock output for the last bit */
  USART1->CR2 &= ~USART_CR2_LBCL;

  /* Enable RTS flow control */
  USART1->CR3 |= USART_CR3_RTSE;

  /* Enable CTS flow control */
  USART1->CR3 |= USART_CR3_CTSE;
}

/**
 * @brief   Enable USART1 transmitter and receiver
 * @note
 * @param   None
 * @retval  None
 */
void USART1_Enable(void)
{
  /* Enable USART1 */
  USART1->CR1 |= USART_CR1_UE;

  /* Enable transmitter */
  USART1->CR1 |= USART_CR1_TE;

  /* Enable receiver */
  USART1->CR1 |= USART_CR1_RE;

  /* Enable parity error interrupt */
  USART1->CR1 |= USART_CR1_PEIE;

  /* Enable idle line detection interrupt */
  USART1->CR1 |= USART_CR1_IDLEIE;

  /* Enable DMA mode for transmitter and receiver */
  USART1->CR3 |= USART_CR3_DMAT;
  USART1->CR3 |= USART_CR3_DMAR;

  /* Clear all DMA interrupt flags for RX */
  DMA2->HIFCR = (DMA_HIFCR_CFEIF5 | DMA_HIFCR_CDMEIF5 | DMA_HIFCR_CTEIF5
      | DMA_HIFCR_CHTIF5 | DMA_HIFCR_CTCIF5);

  /* Enable DMA2 stream 5 for RX */
  DMA2_Stream5->CR |= DMA_SxCR_EN;
}

/**
 * @brief   IRQ callback function
 * @note
 * @param   None
 * @retval  None
 */
void USART1_IRQ_Callback(void)
{
  /* Check if parity error detected */
  if((USART1->SR & USART_SR_PE) == USART_SR_PE)
  {
    while((USART1->SR & USART_SR_RXNE) != USART_SR_RXNE)
    {
      /* Wait for RXNE flag to be set */
    }

    /* Read data register to clear parity error */
    (void)USART1->DR;

    /* Set parity error */
    currentErrorStatus = USART1_PARITY_ERROR;

    /* Disable DMA stream for RX */
    DMA2_Stream5->CR &= ~DMA_SxCR_EN;
  }
  else
  {
    /* No parity error */
  }

  /* Check if idle line detected */
  if((USART1->SR & USART_SR_IDLE) == USART_SR_IDLE)
  {
    /* Read data register to clear idle line flag */
    (void)USART1->DR;

    /* Disable DMA stream for RX */
    DMA2_Stream5->CR &= ~DMA_SxCR_EN;
  }
  else
  {
    /* No new data received */
  }

}

/**
 * @brief   IRQ callback function
 * @note
 * @param   None
 * @retval  None
 */
void USART1_TX_DMA_IRQ_Callback(void)
{
  /* Check transfer complete flag */
  if(DMA_HISR_TCIF7 == (DMA_HISR_TCIF7 & DMA2->HISR))
  {
    /* Clear all interrupt flags */
    DMA2->HIFCR = (DMA_HIFCR_CFEIF7 | DMA_HIFCR_CDMEIF7 | DMA_HIFCR_CTEIF7
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
void USART1_RX_DMA_IRQ_Callback(void)
{
  /* Check transfer complete flag */
  if(DMA_HISR_TCIF5 == (DMA_HISR_TCIF5 & DMA2->HISR))
  {
    /* Calculate amount of data received */
    RxMessageLength = MAX_BUFFER_LENGTH - DMA2_Stream5->NDTR;

    /* Copy data into RX buffer */
    for(int idx = 0; idx < RxMessageLength; idx++)
    {
      RxBuffer[idx] = RxDMABuffer[idx];
    }

    /* Check error status */
    if(USART1_NO_ERROR != currentErrorStatus)
    {
      /* Error detected, discard the received data */
      RxMessageLength = 0;
    }
    else
    {
      /* No error detected */
    }

    /* Clear all interrupt flags */
    DMA2->HIFCR = (DMA_HIFCR_CFEIF5 | DMA_HIFCR_CDMEIF5 | DMA_HIFCR_CTEIF5
        | DMA_HIFCR_CHTIF5 | DMA_HIFCR_CTCIF5);

    /* Enable DMA 2 stream 5 */
    DMA2_Stream5->CR |= DMA_SxCR_EN;
  }
  else
  {
    /* Do nothing, this interrupt is not handled */
  }
}

/**
 * @brief   USART1 transmit and receive data
 * @note
 * @param   None
 * @retval  None
 */
void USART1_Process(void)
{
  /* Check error status */
  if(USART1_NO_ERROR == currentErrorStatus)
  {
    /* Check current USART state */
    switch (currentState)
    {
      case USART1_IDLE:
        /* Transmit data */
        strTransmit(hello_world, sizeof(hello_world));

        /* Go to next state */
        currentState = USART1_WAIT_FOR_RESPONCE;
        break;

      case USART1_WAIT_FOR_RESPONCE:
        /* Check if new message received */
        if(0 != RxMessageLength)
        {
          /* Reset message length */
          RxMessageLength = 0;

          /* Go to next state */
          currentState = USART1_ASK_FOR_NAME;
        }
        else
        {
          /* Nothing received yet */
        }
        break;

      case USART1_ASK_FOR_NAME:
        /* Transmit data */
        strTransmit(ask_for_name, sizeof(ask_for_name));

        /* Go to next state */
        currentState = USART1_WAIT_FOR_NAME;
        break;

      case USART1_WAIT_FOR_NAME:
        /* Check if new message received */
        if(0 != RxMessageLength)
        {
          /* Transmit data */
          strTransmit(hi, sizeof(hi));
          strTransmit(RxBuffer, RxMessageLength);
          strTransmit(ask_for_command, sizeof(ask_for_command));
          strTransmit(ask_for_command_ex, sizeof(ask_for_command_ex));

          /* Reset message length */
          RxMessageLength = 0;

          /* Go to next state */
          currentState = USART1_WAIT_FOR_COMMAND;
        }
        else
        {
          /* Nothing received yet */
        }
        break;

      case USART1_WAIT_FOR_COMMAND:
        /* Check if new message received */
        if(0 != RxMessageLength)
        {
          /* Reset message length */
          RxMessageLength = 0;

          /* String compare results */
          strCmpReturnType isMatch_01 = STR_NOT_EQUAL;
          strCmpReturnType isMatch_02 = STR_NOT_EQUAL;
          strCmpReturnType isMatch_03 = STR_NOT_EQUAL;
          strCmpReturnType isMatch_04 = STR_NOT_EQUAL;

          /* Compare with turn on green led command */
          isMatch_01 = strCmp(turn_on_green_led, RxBuffer,
              sizeof(turn_on_green_led));

          /* Check return status */
          if(STR_EQUAL == isMatch_01)
          {
            /* Turn on green led */
            GPIO_TurnON_LED(EVAL_GREEN_LED);

            /* Transmit data */
            strTransmit(done, sizeof(done));
          }
          else
          {
            /* Compare with turn on red led command */
            isMatch_02 = strCmp(turn_on_red_led, RxBuffer,
                sizeof(turn_on_red_led));
          }

          /* Check return status */
          if(STR_EQUAL == isMatch_02)
          {
            /* Turn on red led */
            GPIO_TurnON_LED(EVAL_RED_LED);

            /* Transmit data */
            strTransmit(done, sizeof(done));
          }
          else if(STR_NOT_EQUAL == isMatch_01)
          {
            /* Compare with turn off green led command */
            isMatch_03 = strCmp(turn_off_green_led, RxBuffer,
                sizeof(turn_off_green_led));
          }
          else
          {
            /* Do nothing */
          }

          /* Check return status */
          if(STR_EQUAL == isMatch_03)
          {
            /* Turn off green led */
            GPIO_TurnOFF_LED(EVAL_GREEN_LED);

            /* Transmit data */
            strTransmit(done, sizeof(done));
          }
          else if((STR_NOT_EQUAL == isMatch_02)
              && (STR_NOT_EQUAL == isMatch_01))
          {
            /* Compare with turn off red led command */
            isMatch_04 = strCmp(turn_off_red_led, RxBuffer,
                sizeof(turn_off_red_led));
          }
          else
          {
            /* Do nothing */
          }

          /* Check return status */
          if(STR_EQUAL == isMatch_04)
          {
            /* Turn off red led */
            GPIO_TurnOFF_LED(EVAL_RED_LED);

            /* Transmit data */
            strTransmit(done, sizeof(done));
          }
          else if((STR_NOT_EQUAL == isMatch_03)
              && (STR_NOT_EQUAL == isMatch_02)
              && (STR_NOT_EQUAL == isMatch_01))
          {
            /* Transmit data */
            strTransmit(wrong_command, sizeof(wrong_command));
          }
          else
          {
            /* Do nothing */
          }
        }
        else
        {
          /* Nothing received yet */
        }
        break;

      default:
        break;
    }
  }
  else if(USART1_PARITY_ERROR == currentErrorStatus)
  {
    /* Transmit parity error */
    strTransmit(parity_error, sizeof(parity_error));

    /* Clear error status */
    currentErrorStatus = USART1_NO_ERROR;
  }
  else
  {
    /* No error detected */
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
