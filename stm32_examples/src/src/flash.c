/*******************************************************************************
 * @file    flash.c
 * @author  Ahmed Eldeep
 * @email   ahmed@almohandes.org
 * @website http://almohandes.org/stm32
 * @date    25.03.2019
 *          
 * @brief   Flash Examples
 * @note    
 *
@verbatim
Copyright (C) Almohandes.org, 2019

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
#include "flash.h"

/**
 * @addtogroup stm32_examples
 * @{
 */

/**
 * @defgroup flash
 * @brief
 * @{
 */

/**
 * @defgroup flash_private_typedefs
 * @{
 */

/**
 * @brief   FLASH states definition
 */
typedef enum
{
  FLASH_IDLE,
  FLASH_WAIT_FOR_CMD,
} FLASH_StateType;

/**
 * @}
 */

/**
 * @defgroup flash_private_defines
 * @{
 */

/**
 * @brief   AF7 PA9 - PA10 pin masks
 */
#define GPIO_AFRH_AFRH9                      ((uint32_t) 0x000000F0)
#define GPIO_AFRH_AFRH9_AF7                  ((uint32_t) 0x00000070)
#define GPIO_AFRH_AFRH10                     ((uint32_t) 0x00000F00)
#define GPIO_AFRH_AFRH10_AF7                 ((uint32_t) 0x00000700)

/**
 * @brief   Maximum USART reception buffer length
 */
#define MAX_BUFFER_LENGTH                     ((uint32_t) 128u)

/**
 * @}
 */

/**
 * @defgroup flash_private_macros
 * @{
 */

/**
 * @}
 */

/**
 * @defgroup flash_private_constants
 * @{
 */

/**
 * @brief   USART1 messages to be transmitted
 */
static char hello[] = "Welcome to Flash Controller !";
static char select_cmd[] = "Select command from the following: ";
static char lock[] = "1: Flash Lock";
static char unlock[] = "2: Flash Unlock";
static char write_data[] = "3: Write Data";
static char read_data[] = "4: Read Data";
static char erase[] = "5: Sector Erase";
static char write_protect[] = "6: Write Protection";
static char remove_write_protect[] = "7: Remove Write Protection";
static char ob_unlock[] = "8: Option Bytes Unlock";
static char ob_lock[] = "9: Option Bytes Lock";
static char jump[] = "10: Direct Jump";
static char line[] = "**********************************************";

static char error_PGSERR[] = "Programming sequence error";
static char error_PGPERR[] = "Programming parallelism error";
static char error_PGAERR[] = "Programming alignment error";
static char error_WRPERR[] = "Write protection error";

static char ACK[] = "ACK";
static char NACK[] = "NACK";

/**
 * @}
 */

/**
 * @defgroup flash_private_variables
 * @{
 */

/**
 * @brief   Flash current state
 */
static FLASH_StateType currentState = FLASH_IDLE;

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
 * @defgroup flash_private_function_prototypes
 * @{
 */
static void strTransmit(const char * str, uint8_t size);
static void process_command();
static void check_errors();

/**
 * @}
 */

/**
 * @defgroup flash_private_functions
 * @{
 */

/**
 * @brief   Check flash errors
 * @param   none
 * @retval  none
 */
static void check_errors()
{
  if(FLASH_SR_PGSERR == (FLASH->SR & FLASH_SR_PGSERR))
  {
    /* Programming sequence error */
    strTransmit(NACK, sizeof(NACK));
    strTransmit(error_PGSERR, sizeof(error_PGSERR));
    strTransmit(line, sizeof(line));

    /* Clear */
    FLASH->SR |= FLASH_SR_PGSERR;
  }
  else if(FLASH_SR_PGPERR == (FLASH->SR & FLASH_SR_PGPERR))
  {
    /* Programming parallelism error */
    strTransmit(NACK, sizeof(NACK));
    strTransmit(error_PGPERR, sizeof(error_PGPERR));
    strTransmit(line, sizeof(line));

    /* Clear */
    FLASH->SR |= FLASH_SR_PGPERR;
  }
  else if(FLASH_SR_PGAERR == (FLASH->SR & FLASH_SR_PGAERR))
  {
    /* Programming alignment error */
    strTransmit(NACK, sizeof(NACK));
    strTransmit(error_PGAERR, sizeof(error_PGAERR));
    strTransmit(line, sizeof(line));

    /* Clear */
    FLASH->SR |= FLASH_SR_PGAERR;
  }
  else if(FLASH_SR_WRPERR == (FLASH->SR & FLASH_SR_WRPERR))
  {
    /* Write protection error */
    strTransmit(NACK, sizeof(NACK));
    strTransmit(error_WRPERR, sizeof(error_WRPERR));
    strTransmit(line, sizeof(line));

    /* Clear */
    FLASH->SR |= FLASH_SR_WRPERR;
  }
  else
  {
    /* No Errors */
    strTransmit(ACK, sizeof(ACK));
  }
}

/**
 * @brief   Process flash command
 * @param   none
 * @retval  none
 */
static void process_command()
{
  uint32_t address = 0;
  uint32_t data = 0;
  uint32_t val = 0;

  /* Process received command */
  switch (RxBuffer[0])
  {
    case 1: /* Flash Lock */
      /* Write lock bit */
      FLASH->CR |= FLASH_CR_LOCK;

      break;

    case 2: /* Flash Unlock */
      /* Write KEY1 */
      FLASH->KEYR = 0x45670123;

      /* Write KEY2 */
      FLASH->KEYR = 0xCDEF89AB;

      break;

    case 3: /* Write Data */
      /* Wait bsy flag */
      while(0 != (FLASH->SR & FLASH_SR_BSY))
      {
        /* Waiting */
      }

      /* Enable flash programming */
      FLASH->CR |= FLASH_CR_PG;

      /* Write data into flash */
      address = * (uint32_t *) &RxBuffer[1];

      for(uint32_t idx = 0; idx < 4; ++idx)
      {
        data = * (uint32_t *) &RxBuffer[5 + (idx * 4)];
        *(volatile uint32_t*)(address) = data;

        address += 4;

        /* Wait bsy flag */
        while(0 != (FLASH->SR & FLASH_SR_BSY))
        {
          /* Waiting */
        }
      }

      /* Disable flash programming */
      FLASH->CR &= ~FLASH_CR_PG;

      break;

    case 4: /* Read Data */
      /* 4 00 00 1E 08 */
      address = * (uint32_t *) &RxBuffer[1];
      val = * (uint32_t *) address;

      strTransmit((char *)&val, sizeof(val));
      break;

    case 5: /* Sector Erase */
      /* Wait bsy flag */
      while(0 != (FLASH->SR & FLASH_SR_BSY))
      {
        /* Waiting */
      }

      /* Select sector erase */
      FLASH->CR |= FLASH_CR_SER;

      /* Check sector number for bank 2 */
      uint32_t sector_num = RxBuffer[1];
      if(12 <= sector_num)
      {
        /* Bank 2 */
        sector_num = sector_num + 4;
      }
      else
      {

      }

      /* Write sector number */
      FLASH->CR |= (FLASH_CR_SNB & (sector_num << 3));

      /* Start sector erase */
      FLASH->CR |= FLASH_CR_STRT;

      /* Wait bsy flag */
      while(0 != (FLASH->SR & FLASH_SR_BSY))
      {
        /* Waiting */
      }

      /* Disable sector erase */
      FLASH->CR &= ~FLASH_CR_SER;

      break;

    case 6: /* Write Protection */
      /* Wait bsy flag */
      while(0 != (FLASH->SR & FLASH_SR_BSY))
      {
        /* Waiting */
      }

      /* Check sector number */
      sector_num = RxBuffer[1];
      if(12 <= sector_num)
      {
        /* Bank 2 */
        sector_num = sector_num - 12;
        FLASH->OPTCR1 &= ~((1 << sector_num) << 16);
      }
      else
      {
        /* Bank 2 */
        FLASH->OPTCR &= ~((1 << sector_num) << 16);
      }

      /* Start sector write protection */
      FLASH->OPTCR |= FLASH_OPTCR_OPTSTRT;

      /* Wait bsy flag */
      while(0 != (FLASH->SR & FLASH_SR_BSY))
      {
        /* Waiting */
      }

      break;

    case 7: /* Remove Write Protection */
      /* Wait bsy flag */
      while(0 != (FLASH->SR & FLASH_SR_BSY))
      {
        /* Waiting */
      }

      /* Check sector number */
      sector_num = RxBuffer[1];
      if(12 <= sector_num)
      {
        /* Bank 2 */
        sector_num = sector_num - 12;
        FLASH->OPTCR1 |= ((1 << sector_num) << 16);
      }
      else
      {
        /* Bank 2 */
        FLASH->OPTCR |= ((1 << sector_num) << 16);
      }

      /* Start sector write protection */
      FLASH->OPTCR |= FLASH_OPTCR_OPTSTRT;

      /* Wait bsy flag */
      while(0 != (FLASH->SR & FLASH_SR_BSY))
      {
        /* Waiting */
      }

      break;

    case 8: /* Option Bytes Unlock */
      /* Write OPTKEY1 */
      FLASH->OPTKEYR = 0x08192A3B;

      /* Write OPTKEY2 */
      FLASH->OPTKEYR = 0x4C5D6E7F;

      break;

    case 9: /* Option Bytes Lock */
      /* Write lock bit */
      FLASH->OPTCR |= FLASH_OPTCR_OPTLOCK;

      break;

    case 10: /* Jump */
      /* Get jump address */
      address = * (uint32_t *) &RxBuffer[1];
      val = * (uint32_t *) address;

      /* Check if it has valid stack pointer in the RAM */
      if(0x20000000 == (val & 0x20000000))
      {
        /* Disable all interrupts */
        __disable_irq();

        /* Reset GPIOA and DMA2 */
        RCC->AHB1RSTR = (RCC_AHB1RSTR_GPIOARST | RCC_AHB1RSTR_DMA2RST);

        /* Release reset */
        RCC->AHB1RSTR = 0;

        /* Reset USART1 */
        RCC->APB2RSTR = RCC_APB2RSTR_USART1RST;

        /* Release reset */
        RCC->APB2RSTR = 0;

        /* Reset RCC */
        /* Set HSION bit to the reset value */
        RCC->CR |= RCC_CR_HSION;

        /* Wait till HSI is ready */
        while(RCC_CR_HSIRDY != (RCC_CR_HSIRDY & RCC->CR))
        {
          /* Waiting */
        }

        /* Set HSITRIM[4:0] bits to the reset value */
        RCC->CR |= RCC_CR_HSITRIM_4;

        /* Reset CFGR register */
        RCC->CFGR = 0;

        /* Wait till clock switch is ready and
         * HSI oscillator selected as system clock */
        while(0 != (RCC_CFGR_SWS & RCC->CFGR))
        {
          /* Waiting */
        }

        /* Clear HSEON, HSEBYP and CSSON bits */
        RCC->CR &= ~(RCC_CR_HSEON | RCC_CR_HSEBYP | RCC_CR_CSSON);

        /* Wait till HSE is disabled */
        while(0 != (RCC_CR_HSERDY & RCC->CR))
        {
          /* Waiting */
        }

        /* Clear PLLON bit */
        RCC->CR &= ~RCC_CR_PLLON;

        /* Wait till PLL is disabled */
        while(0 != (RCC_CR_PLLRDY & RCC->CR))
        {
          /* Waiting */
        }

        /* Reset PLLCFGR register to default value */
        RCC->PLLCFGR = RCC_PLLCFGR_PLLM_4 | RCC_PLLCFGR_PLLN_6
            | RCC_PLLCFGR_PLLN_7 | RCC_PLLCFGR_PLLQ_2;

        /* Reset SysTick */
        SysTick->CTRL = 0;
        SysTick->LOAD = 0;
        SysTick->VAL = 0;

        /* Check jump address */
        if(0x1FFF0000 == address)
        {
          /* Enable SYSCFG clock */
          RCC->APB2ENR |= RCC_APB2LPENR_SYSCFGLPEN;

          /* Map address 0x0 to system memory */
          SYSCFG->MEMRMP = SYSCFG_MEMRMP_MEM_MODE_0;
        }
        else
        {
          /* Vector Table Relocation in Internal FLASH */
          __DMB();
          SCB->VTOR = address;
          __DSB();
        }

        /* Set jump to the reset handler */
        void (*jump_address)(void) = (void *)(*((uint32_t *)(address + 4)));

        /* Set stack pointer */
        __set_MSP(val);

        /* Jump */
        jump_address();


      }
      else
      {
        /* No valid stack pointer */
        strTransmit(NACK, sizeof(NACK));
      }

      break;

    default:
      break;
  }
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
 * @defgroup flash_exported_functions
 * @{
 */

/**
 * @brief   Configure GPIO with AF7, USART1 connected to APB2 with 90MHz clock
 * @note    USART1_TX  -> PA9  (OUT)
 *          USART1_RX  -> PA10 (IN)
 * @param   None
 * @retval  None
 */
void FLASH_USART1_GPIO_Config(void)
{
  /* Enable port A clock */
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;

  /* Select alternate function mode */
  GPIOA->MODER &= ~(GPIO_MODER_MODER9 | GPIO_MODER_MODER10);
  GPIOA->MODER |= (GPIO_MODER_MODER9_1 | GPIO_MODER_MODER10_1);

  /* Select output type push-pull */
  GPIOA->OTYPER &= ~GPIO_OTYPER_OT_9;

  /* Select output speed medium */
  GPIOA->OSPEEDR &= ~GPIO_OSPEEDER_OSPEEDR9;
  GPIOA->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR9_0;

  /* Select pull up */
  GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPDR9 | GPIO_PUPDR_PUPDR10);
  GPIOA->PUPDR |= (GPIO_PUPDR_PUPDR9_0 | GPIO_PUPDR_PUPDR10_0);

  /* Select AF7 */
  GPIOA->AFR[1] &= ~(GPIO_AFRH_AFRH9 | GPIO_AFRH_AFRH10);
  GPIOA->AFR[1] |= (GPIO_AFRH_AFRH9_AF7 | GPIO_AFRH_AFRH10_AF7);
}

/**
 * @brief   Configure DMA for USART TX
 * @note    USART1_TX -> DMA2_Stream7 (Channel 4)
 * @param   None
 * @retval  None
 */
void FLASH_USART1_TX_DMA_Config(void)
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
void FLASH_USART1_RX_DMA_Config(void)
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
void FLASH_USART1_Init(void)
{
  /* Configure USART1 */
  /* Enable USART1 clock */
  RCC->APB2ENR = RCC_APB2ENR_USART1EN;

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
void FLASH_USART1_Enable(void)
{
  /* Enable USART1 */
  USART1->CR1 |= USART_CR1_UE;

  /* Enable transmitter */
  USART1->CR1 |= USART_CR1_TE;

  /* Enable receiver */
  USART1->CR1 |= USART_CR1_RE;

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
void FLASH_USART1_IRQ_Callback(void)
{
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
void FLASH_USART1_TX_DMA_IRQ_Callback(void)
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
void FLASH_USART1_RX_DMA_IRQ_Callback(void)
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
 * @brief   Flash Initialization function
 * @note
 * @param   None
 * @retval  None
 */
void FLASH_Init(void)
{
  /* Write KEY1 */
  FLASH->KEYR = 0x45670123;

  /* Write KEY2 */
  FLASH->KEYR = 0xCDEF89AB;

  /* Select flash parallelism x32 */
  FLASH->CR &= ~FLASH_CR_PSIZE;
  FLASH->CR |= FLASH_CR_PSIZE_1;

  /* Write lock bit */
  FLASH->CR |= FLASH_CR_LOCK;
}

/**
 * @brief   USART1 transmit and receive data
 * @note
 * @param   None
 * @retval  None
 */
void FLASH_Main(void)
{
  /* Check current USART state */
  switch (currentState)
  {
    case FLASH_IDLE:
      /* Transmit data */
      strTransmit(line, sizeof(line));
      strTransmit(hello, sizeof(hello));
      strTransmit(select_cmd, sizeof(select_cmd));
      strTransmit(line, sizeof(line));
      strTransmit(lock, sizeof(lock));
      strTransmit(unlock, sizeof(unlock));
      strTransmit(write_data, sizeof(write_data));
      strTransmit(read_data, sizeof(read_data));
      strTransmit(erase, sizeof(erase));
      strTransmit(write_protect, sizeof(write_protect));
      strTransmit(remove_write_protect, sizeof(remove_write_protect));
      strTransmit(ob_unlock, sizeof(ob_unlock));
      strTransmit(ob_lock, sizeof(ob_lock));
      strTransmit(jump, sizeof(jump));
      strTransmit(line, sizeof(line));

      /* Go to next state */
      currentState = FLASH_WAIT_FOR_CMD;
      break;

    case FLASH_WAIT_FOR_CMD:
      /* Check if new message received */
      if(0 != RxMessageLength)
      {
        /* Process command */
        process_command();

        /* Check errors */
        check_errors();

        /* Reset message length */
        RxMessageLength = 0;
      }
      else
      {
        /* Nothing received yet */
      }

      /* Go to next state */
      currentState = FLASH_WAIT_FOR_CMD;
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
