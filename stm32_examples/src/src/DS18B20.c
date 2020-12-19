/*******************************************************************************
 * @file    DS18B20.c
 * @author  Ahmed Eldeep
 * @email   ahmed@almohandes.org
 * @website http://almohandes.org/stm32
 * @date    22.05.2018
 *          
 * @brief   Interfacing temperature sensor DS18B20 using UART over one-wire
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
#include "SysTick.h"
#include "gpio.h"
#include "DS18B20.h"

/**
 * @addtogroup stm32_examples
 * @{
 */

/**
 * @defgroup DS18B20
 * @brief
 * @{
 */

/**
 * @defgroup DS18B20_private_typedefs
 * @{
 */

/**
 * @}
 */

/**
 * @defgroup DS18B20_private_defines
 * @{
 */

/**
 * @brief   AF8 PC12 pin masks
 */
#define GPIO_AFRH_AFRH12                     ((uint32_t) 0x000F0000)
#define GPIO_AFRH_AFRH12_AF8                 ((uint32_t) 0x00080000)

/**
 * @brief   Reset command
 */
#define DS18B20_RESET_CMD                    ((uint8_t) 0xF0)

/**
 * @brief   Logical bit values
 */
#define BIT_0                                ((uint8_t) 0x00)
#define BIT_1                                ((uint8_t) 0xFF)

/**
 * @brief   Conversion time in ms, from DS18B20 datasheet
 */
#define MAX_CONVERSION_TIME                  ((uint32_t) 750)

/**
 * @}
 */

/**
 * @defgroup DS18B20_private_macros
 * @{
 */

/**
 * @}
 */

/**
 * @defgroup DS18B20_private_constants
 * @{
 */

/**
 * @brief   Temperature convert, {Skip ROM = 0xCC, Convert = 0x44}
 */
static const uint8_t temp_convert[] =
{
  BIT_0, BIT_0, BIT_1, BIT_1, BIT_0, BIT_0, BIT_1, BIT_1,
  BIT_0, BIT_0, BIT_1, BIT_0, BIT_0, BIT_0, BIT_1, BIT_0
};

/**
 * @brief   Temperature data read, {Skip ROM = 0xCC, Scratch read = 0xBE}
 */
static const uint8_t temp_read[] =
{
  BIT_0, BIT_0, BIT_1, BIT_1, BIT_0, BIT_0, BIT_1, BIT_1,
  BIT_0, BIT_1, BIT_1, BIT_1, BIT_1, BIT_1, BIT_0, BIT_1,
  BIT_1, BIT_1, BIT_1, BIT_1, BIT_1, BIT_1, BIT_1, BIT_1,
  BIT_1, BIT_1, BIT_1, BIT_1, BIT_1, BIT_1, BIT_1, BIT_1
};

/**
 * @}
 */

/**
 * @defgroup DS18B20_private_variables
 * @{
 */

/**
 * @brief   Received temperature data using DMA
 */
static uint8_t temperatureData[sizeof(temp_read)];

/**
 * @brief   Temperature data received flag
 */
static uint8_t temperatureDataReceived = 0;

/**
 * @brief   Current temperature value in degree celsius
 */
static float currentTemperature = 0;

/**
 * @}
 */

/**
 * @defgroup DS18B20_private_function_prototypes
 * @{
 */

/**
 * @brief   DMA command transmit
 * @note
 * @param   cmd, size
 * @retval  None
 */
static void cmdTransmit(const uint8_t * cmd, uint8_t size);

/**
 * @brief   DMA command receive
 * @note
 * @param   cmd, size
 * @retval  None
 */
static void cmdReceive(const uint8_t * cmd, uint8_t size);

/**
 * @brief   Send reset pulse to DS18B20
 * @note
 * @param   None
 * @retval  None
 */
static uint8_t cmdReset(void);

/**
 * @}
 */

/**
 * @defgroup DS18B20_private_functions
 * @{
 */

/**
 * @brief   DMA string transmit
 * @note    IMPORTANT: Since we send and receive the reset pulse without DMA,
 *          its necessary to clear any pending DMA requests before
 *          enable transmission DMA stream.
 * @param   cmd, size
 * @retval  None
 */
static void cmdTransmit(const uint8_t * cmd, uint8_t size)
{
  /* Check null pointers */
  if(NULL != cmd)
  {
    /* Wait until DMA1 stream 7 is disabled */
    while(DMA_SxCR_EN == (DMA_SxCR_EN & DMA1_Stream7->CR))
    {
      /* Do nothing, the enable flag shall reset
       * when DMA transfer complete */
    }

    /* Set memory address */
    DMA1_Stream7->M0AR = (uint32_t)cmd;

    /* Set number of data items */
    DMA1_Stream7->NDTR = size;

    /* Clear all interrupt flags */
    DMA1->HIFCR = (DMA_HIFCR_CFEIF7 | DMA_HIFCR_CDMEIF7 | DMA_HIFCR_CTEIF7
        | DMA_HIFCR_CHTIF7 | DMA_HIFCR_CTCIF7);

    /* Clear any UART pending DMA requests */
    UART5->CR3 &= ~USART_CR3_DMAT;

    /* Enable DMA mode for transmitter */
    UART5->CR3 |= USART_CR3_DMAT;

    /* Enable DMA 1 stream 7 */
    DMA1_Stream7->CR |= DMA_SxCR_EN;
  }
  else
  {
    /* Null pointers, do nothing */
  }
}

/**
 * @brief   DMA string receive
 * @note    IMPORTANT: Since we send and receive the reset pulse without DMA,
 *          its necessary to clear any pending DMA requests before
 *          enable reception DMA stream.
 * @param   cmd, size
 * @retval  None
 */
static void cmdReceive(const uint8_t * cmd, uint8_t size)
{
  /* Check null pointers */
  if(NULL != cmd)
  {
    /* Wait until DMA1 stream 0 is disabled */
    while(DMA_SxCR_EN == (DMA_SxCR_EN & DMA1_Stream0->CR))
    {
      /* Do nothing, the enable flag shall reset
       * when DMA transfer complete */
    }

    /* Set memory address */
    DMA1_Stream0->M0AR = (uint32_t)cmd;

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
 * @brief   Send reset pulse to DS18B20
 * @note
 * @param   None
 * @retval  None
 */
static uint8_t cmdReset(void)
{
  uint8_t isSensorDetected = 0;

  /* Disable UART5 prescaler and outputs */
  UART5->CR1 &= ~USART_CR1_UE;

  /* Set baud rate = 9600 Bps
   * USARTDIV = Fck / (16 * baud_rate)
   *          = 45000000 / (16 * 9600) = 292.96
   *
   * DIV_Fraction = 16 * 0.96 = 15.36 = 15 = 0xF
   * DIV_Mantissa = 292 = 0x124
   *
   * BRR          = 0x124F */
  UART5->BRR = 0x124F;

  /* Enable UART5 prescaler and outputs */
  UART5->CR1 |= USART_CR1_UE;

  /* Check USART status register */
  while(!(UART5->SR & USART_SR_TXE))
  {
    /* Wait for transmission buffer empty flag */
  }

  /* Write reset command */
  UART5->DR = DS18B20_RESET_CMD;

  /* Check USART status register */
  while(!(UART5->SR & USART_SR_TC))
  {
    /* Wait for transmission complete flag */
  }

  /* Read Rx Data */
  uint16_t Rx = UART5->DR;

  /* Check sensor presence */
  if((DS18B20_RESET_CMD != Rx) && ( BIT_0 != Rx))
  {
    /* Temp sensor was detected */
    isSensorDetected = 1;
  }
  else
  {
    /* Do nothing, No sensor was detected */
  }

  /* Disable UART5 prescaler and outputs */
  UART5->CR1 &= ~USART_CR1_UE;

  /* Set baud rate = 115200 Bps
   * USARTDIV = Fck / (16 * baud_rate)
   *          = 45000000 / (16 * 115200) = 24.41
   *
   * DIV_Fraction = 16 * 0.41 = 6.56 = 7 = 0x7
   * DIV_Mantissa = 24 = 0x18
   *
   * BRR          = 0x187 */
  UART5->BRR = 0x187;

  /* Enable UART5 prescaler and outputs */
  UART5->CR1 |= USART_CR1_UE;

  return isSensorDetected;
}

/**
 * @}
 */

/**
 * @defgroup DS18B20_exported_functions
 * @{
 */

/**
 * @brief   Configure GPIO
 * @note    UART5_TX -> PC12, UART5_RX -> PD2 (Not Used)
 *          UART5 mapped to alternate function AF8
 *          UART5 connected to APB1 with 45MHz max clock
 * @param   None
 * @retval  None
 */
void DS18B20_GPIO_Init(void)
{
  /* Enable port C clock */
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;

  /* Select alternate function mode */
  GPIOC->MODER &= ~(GPIO_MODER_MODER12);
  GPIOC->MODER |= GPIO_MODER_MODER12_1;

  /* Select output type open-drain */
  GPIOC->OTYPER |= GPIO_OTYPER_OT_12;

  /* Select output speed medium */
  GPIOC->OSPEEDR &= ~(GPIO_OSPEEDER_OSPEEDR12);
  GPIOC->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR12_0;

  /* Select no pull-up, pull-down */
  GPIOC->PUPDR &= ~(GPIO_PUPDR_PUPDR12);

  /* Select AF8 */
  GPIOC->AFR[1] &= ~(GPIO_AFRH_AFRH12);
  GPIOC->AFR[1] |= GPIO_AFRH_AFRH12_AF8;
}

/**
 * @brief   Configure DMA for UART TX
 * @note    UART5_TX -> DMA1_Stream7 (Channel 4)
 * @param   None
 * @retval  None
 */
void DS18B20_TX_DMA_Init(void)
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
void DS18B20_RX_DMA_Init(void)
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
 * @brief   Configure UART5 for DS18B20
 * @note
 * @param   None
 * @retval  None
 */
void DS18B20_UART5_Init(void)
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

  /* Select Single-wire Half-duplex mode */
  UART5->CR3 |= USART_CR3_HDSEL;
}

/**
 * @brief   Enable communications with DS18B20
 * @note
 * @param   None
 * @retval  None
 */
void DS18B20_UART5_Enable(void)
{
  /* Enable UART5 */
  UART5->CR1 |= USART_CR1_UE;

  /* Enable transmitter */
  UART5->CR1 |= USART_CR1_TE;

  /* Enable receiver */
  UART5->CR1 |= USART_CR1_RE;
}

/**
 * @brief   DS18B20 process function
 * @note
 * @param   None
 * @retval  None
 */
void DS18B20_Process(void)
{
  /* Sensor detected flag */
  uint8_t isSensorDetected = 0;

  /* Send reset pulse */
  isSensorDetected = cmdReset();

  /* Check if the sensor was detected */
  if(1 == isSensorDetected)
  {
    /* Turn on green LED */
    GPIO_TurnON_LED(EVAL_GREEN_LED);

    /* Send temperature conversion command */
    cmdTransmit(temp_convert, sizeof(temp_convert));

    /* Wait conversion time */
    SysTick_Delay(MAX_CONVERSION_TIME);

    /* Send reset pulse */
    cmdReset();

    /* Enable temperature data reception with DMA */
    cmdReceive(temperatureData, sizeof(temperatureData));

    /* Send temperature read command */
    cmdTransmit(temp_read, sizeof(temp_read));

    /* Check temperature data received flag */
    while (temperatureDataReceived == 0)
    {
      /* Wait until DMA receive temperature data */
    }

    /* Reset temperature data received flag */
    temperatureDataReceived = 0;

    /* Temporarily variable for extracting temperature data */
    uint16_t temperature = 0;

    /* Extract new temperature data */
    for (int idx = 16; idx < 32; idx++)
    {
      if (BIT_1 == temperatureData[idx])
      {
        /* Bit value is 1 */
        temperature = (temperature >> 1) | 0x8000;
      }
      else
      {
        /* Bit value is 0 */
        temperature = temperature >> 1;
      }
    }

    /* Copying new temperature data and divide by 16 for fraction part */
    currentTemperature = (float) temperature / (float) 16;

  }
  else
  {
    /* Turn on red LED, indicates sensor detection failed */
    GPIO_TurnON_LED(EVAL_RED_LED);

    /* Temperature data not valid */
    currentTemperature = 0;
  }
}

/**
 * @brief   IRQ callback function
 * @note
 * @param   None
 * @retval  None
 */
void UART5_TX_DMA_IRQ_Callback(void)
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
void UART5_RX_DMA_IRQ_Callback(void)
{
  /* Check transfer complete flag */
  if(DMA_LISR_TCIF0 == (DMA_LISR_TCIF0 & DMA1->LISR))
  {
    /* Clear all interrupt flags */
    DMA1->LIFCR = (DMA_LIFCR_CFEIF0 | DMA_LIFCR_CDMEIF0 | DMA_LIFCR_CTEIF0
        | DMA_LIFCR_CHTIF0 | DMA_LIFCR_CTCIF0);

    /* Set transfer complete flag */
    temperatureDataReceived = 1;
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
