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
  USART1_WAIT_FOR_COMMAND,
} USART1_StateType;

/**
 * @brief   USART1 IRQ status definition
 */

typedef enum
{
  USART1_NO_IRQ,
  USART1_CHAR_RECEIVED,
  USART1_PARITY_ERROR,
} USART1_IRQStatusType;

/**
 * @brief   Return type
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
 * @brief   AF PA9 / PA10 pin masks
 */
#define GPIO_AFRH_AFRH9                      ((uint32_t) 0x000000F0)
#define GPIO_AFRH_AFRH9_AF7                  ((uint32_t) 0x00000070)
#define GPIO_AFRH_AFRH10                     ((uint32_t) 0x00000F00)
#define GPIO_AFRH_AFRH10_AF7                 ((uint32_t) 0x00000700)

/**
 * @brief   Maximum USART reception buffer length
 */
#define MAX_BUFFER_LENGTH                     ((uint32_t) 200u)

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
static const char hello_world[]        = "Hello World!";
static const char ask_for_name[]       = "What is your name?";
static const char hi[]                 = "Hi,";
static const char ask_for_command[]    = "Please, send command";
static const char ask_for_command_ex[] = "Action[turn_on / turn_off] Led[green_led / red_led]";
static const char turn_on_green_led[]  = "turn_on green_led";
static const char turn_on_red_led[]    = "turn_on red_led";
static const char turn_off_green_led[] = "turn_off green_led";
static const char turn_off_red_led[]   = "turn_off red_led";
static const char done[]               = "Done";
static const char wrong_command[]      = "Wrong Command";
static const char parity_error[]       = "Parity Error";

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
 * @brief   USART1 current IRQ status
 */
static USART1_IRQStatusType currentIRQStatus = USART1_NO_IRQ;

/**
 * @brief   USART1 last char recieved
 */
static char RxChar = 0;

/**
 * @brief   USART1 message buffer
 */
static char RxBuffer[MAX_BUFFER_LENGTH + 1];

/**
 * @brief   USART1 message length
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
 * @brief   String transmit
 * @note
 * @param   str, size
 * @retval  None
 */
static void strTransmit(const char * str, uint8_t size)
{
  /* Check null pointers */
  if(NULL != str)
  {
    /* Send all string characters */
    for(int idx = 0; idx < size; idx++)
    {
      /* Check USART status register */
      while(!(USART1->SR & USART_SR_TXE))
      {
        /* Wait for transmission buffer empty flag */
      }

      /* Write data into transmit data register */
      USART1->DR = str[idx];
    }
  }
  else
  {
    /* Null pointers, do nothing */
  }
}

/**
 * @brief   String receive
 * @note
 * @param   None
 * @retval  None
 */
static void strReceive(void)
{
  /* Local string buffer */
  static char RxLocalBuffer[MAX_BUFFER_LENGTH];

  /* Current reception index */
  static int RxIndex = 0;

  /* Check for end-of-line condition */
  if (RxChar == '\0')
  {
    /* Check if string data was received before */
    if (RxIndex != 0)
    {
      /* Copy string data from local buffer */
      for(int idx = 0; idx < RxIndex; idx++)
      {
        RxBuffer[idx] = RxLocalBuffer[idx];
      }

      /* Add terminating NULL at the end */
      RxBuffer[RxIndex] = 0;

      /* Set message length */
      RxMessageLength = RxIndex + 1;

      /* Reset current index */
      RxIndex = 0;
    }
    else
    {
      /* String buffer already empty */
    }
  }
  else
  {
    /* Char data was received, Check for buffer overflow */
    if (MAX_BUFFER_LENGTH == RxIndex)
    {
      /* Reset the reception buffer */
      RxIndex = 0;
    }
    else
    {
      /* Do nothing, No overflow */
    }

    /* Copy char data into string buffer */
    RxLocalBuffer[RxIndex] = RxChar;

    /* Increment current index for the next char reception */
    RxIndex++;
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
 * @note    PA9 -> USART1_TX, PA10 -> USART1_RX
 * @param   None
 * @retval  None
 */
void USART1_GPIO_Config(void)
{
  /* Enable port A clock */
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;

  /* Select alternate function mode */
  GPIOA->MODER &= ~(GPIO_MODER_MODER9 | GPIO_MODER_MODER10);
  GPIOA->MODER |= (GPIO_MODER_MODER9_1 | GPIO_MODER_MODER10_1);

  /* Select output type push-pull for Tx(PA9) */
  GPIOA->OTYPER &= ~(GPIO_OTYPER_OT_9);

  /* Select output speed medium for Tx(PA9) */
  GPIOA->OSPEEDR &= ~(GPIO_OSPEEDER_OSPEEDR9);
  GPIOA->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR9_0;

  /* Select pull up */
  GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPDR9 | GPIO_PUPDR_PUPDR10);
  GPIOA->PUPDR |= (GPIO_PUPDR_PUPDR9_0 | GPIO_PUPDR_PUPDR10_0);

  /* Select AF7 */
  GPIOA->AFR[1] &= ~(GPIO_AFRH_AFRH9 | GPIO_AFRH_AFRH10);
  GPIOA->AFR[1] |= (GPIO_AFRH_AFRH9_AF7 | GPIO_AFRH_AFRH10_AF7);
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

  /* Enable reception buffer not empty flag interrupt */
  USART1->CR1 |= USART_CR1_RXNEIE;

  /* Enable parity error interrupt */
  USART1->CR1 |= USART_CR1_PEIE;
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
    USART1->DR;

    /* Set parity error */
    currentIRQStatus = USART1_PARITY_ERROR;
  }
  else
  {
    /* No parity error */
  }

  /* Check USART receiver */
  if((USART1->SR & USART_SR_RXNE) == USART_SR_RXNE)
  {
    /* Read character */
    RxChar = USART1->DR;

    /* Set IRQ status */
    currentIRQStatus = USART1_CHAR_RECEIVED;
  }
  else
  {
    /* No new data received */
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
  switch (currentIRQStatus)
  {
    case USART1_PARITY_ERROR:
      /* Transmit parity error */
      strTransmit(parity_error, sizeof(parity_error));

      /* Reset USART1 state */
      currentState = USART1_IDLE;

      /* Reset IRQ status */
      currentIRQStatus = USART1_NO_IRQ;
      break;

    case USART1_CHAR_RECEIVED:
      /* Receive string data */
      strReceive();

      /* Reset IRQ status */
      currentIRQStatus = USART1_NO_IRQ;
      break;

    case USART1_NO_IRQ:
      break;

    default:
      break;
  }

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
        isMatch_01 =  strCmp(turn_on_green_led, RxBuffer,
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
          isMatch_02 =  strCmp(turn_on_red_led, RxBuffer,
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
          isMatch_03 =  strCmp(turn_off_green_led, RxBuffer,
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
          isMatch_04 =  strCmp(turn_off_red_led, RxBuffer,
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

/**
 * @}
 */
/**
 * @}
 */
/**
 * @}
 */
