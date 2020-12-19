/*******************************************************************************
 * @file    L3GD20.c
 * @author  Ahmed Eldeep
 * @email   ahmed@almohandes.org
 * @website http://almohandes.org/stm32
 * @date    30.10.2018
 *          
 * @brief   Interfacing gyroscope sensor L3GD20 using SPI
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
#include "exti.h"
#include "gpio.h"
#include "SysTick.h"
#include "L3GD20.h"

/**
 * @addtogroup stm32_examples
 * @{
 */

/**
 * @defgroup L3GD20
 * @brief
 * @{
 */

/**
 * @defgroup L3GD20_Private_Typedefs
 * @{
 */

/**
 * @brief   L3GD20 states definition
 */
typedef enum
{
  L3GD20_READ,
  L3GD20_WAIT_FOR_DATA,
  L3GD20_DATA_PROCESS,
} L3GD20_StateType;

/**
 * @brief   L3GD20 calibration status definition
 */
typedef enum
{
  L3GD20_COLLECT_CALIBRATION_SAMPLES,
  L3GD20_PROCESS_CALIBRATION_SAMPLES,
  L3GD20_CALIBRATED,
} L3GD20_CaliStateType;

/**
 * @brief   L3GD20 data ready flag
 */
typedef enum
{
  L3GD20_DATA_NOT_READY,
  L3GD20_DATA_READY,
} L3GD20_DataReadyFlagType;

/**
 * @}
 */

/**
 * @defgroup L3GD20_Private_Defines
 * @{
 */

/**
 * @brief   AF5 pin 7, 8, 8 masks
 */
#define GPIO_AFRL_AFRL7                      ((uint32_t) 0xF0000000)
#define GPIO_AFRL_AFRL7_AF5                  ((uint32_t) 0x50080000)

#define GPIO_AFRH_AFRH8                      ((uint32_t) 0x0000000F)
#define GPIO_AFRH_AFRH8_AF5                  ((uint32_t) 0x00080005)

#define GPIO_AFRH_AFRH9                      ((uint32_t) 0x000000F0)
#define GPIO_AFRH_AFRH9_AF5                  ((uint32_t) 0x00080050)

/**
 * @brief   Maximum DMA TX/RX buffer length
 */
#define MAX_BUFFER_LENGTH                    ((uint8_t) 7u)

/**
 * @brief   Number of samples collected for offset calculations
 */
#define CALIBRATION_BUFFER_LENGTH            ((uint32_t) 2000u)

/**
 * @brief   Number of samples collected for offset calculations
 */
#define AVERAGE_WINDOW_SIZE                  ((uint32_t) 10u)

/**
 * @brief   L3GD20 mask commands
 */
#define RW_READ_WRITE_CMD                   ((uint8_t) 0x80)
#define MULTIPLEBYTE_CMD                    ((uint8_t) 0x40)

/**
 * @brief   L3GD20 registers
 */

/* Control registers 1:5 */
#define L3GD20_CTRL_REG1_ADDR         ((uint8_t) 0x20)
#define L3GD20_CTRL_REG2_ADDR         ((uint8_t) 0x21)
#define L3GD20_CTRL_REG3_ADDR         ((uint8_t) 0x22)
#define L3GD20_CTRL_REG4_ADDR         ((uint8_t) 0x23)
#define L3GD20_CTRL_REG5_ADDR         ((uint8_t) 0x24)

/* Output Registers X,Y,z */
#define L3GD20_OUT_X_L_ADDR           ((uint8_t) 0x28)
#define L3GD20_OUT_X_H_ADDR           ((uint8_t) 0x29)
#define L3GD20_OUT_Y_L_ADDR           ((uint8_t) 0x2A)
#define L3GD20_OUT_Y_H_ADDR           ((uint8_t) 0x2B)
#define L3GD20_OUT_Z_L_ADDR           ((uint8_t) 0x2C)
#define L3GD20_OUT_Z_H_ADDR           ((uint8_t) 0x2D)

/**
 * @brief   L3GD20 registers configuration
 */

/* Control registers 1
 * Output data rate = 760 Hz
 * Cut-Off = 100
 * Power mode = Normal
 * Enable X, Y, Z
 *  */
#define L3GD20_CTRL_REG1_VAL          ((uint8_t) 0xFF)

/* Control registers 2,
 * High-pass filter mode = Normal
 * High-pass filter cut off frequency = 51.4 Hz */
#define L3GD20_CTRL_REG2_VAL          ((uint8_t) 0x00)

/* Control registers 3 (Reset) */
#define L3GD20_CTRL_REG3_VAL          ((uint8_t) 0x00)

/* Control registers 4,
 * Full scale = 2000 dps */
#define L3GD20_CTRL_REG4_VAL          ((uint8_t) 0x20)

/* Control registers 5,
 * High-pass filter enabled */
#define L3GD20_CTRL_REG5_VAL          ((uint8_t) 0x10)

/* Set sensitivity scale correction */
/* Sensitivity at 2000 range = 70 mdps/digit = 0.07 */
#define L3GD20_SENSITIVITY            ((float) 0.07)

/**
 * @}
 */

/**
 * @defgroup L3GD20_Private_Macros
 * @{
 */

/**
 * @}
 */

/**
 * @defgroup L3GD20_Private_Constants
 * @{
 */

/**
 * @}
 */

/**
 * @defgroup L3GD20_Private_variables
 * @{
 */

/**
 * @brief   DMA TX/RX buffers
 */
static uint8_t RxDMABuffer[MAX_BUFFER_LENGTH];
static uint8_t TxDMABuffer[MAX_BUFFER_LENGTH];

/**
 * @brief   Calibration buffers
 */
static int16_t calibrationBuffer_X[CALIBRATION_BUFFER_LENGTH];
static int16_t calibrationBuffer_Y[CALIBRATION_BUFFER_LENGTH];
static int16_t calibrationBuffer_Z[CALIBRATION_BUFFER_LENGTH];

/**
 * @brief   L3GD20 current state
 */
static L3GD20_StateType currentState = L3GD20_READ;

/**
 * @brief   L3GD20 calibration state definition
 */
static L3GD20_CaliStateType currentCaliState = L3GD20_COLLECT_CALIBRATION_SAMPLES;

/**
 * @brief   L3GD20 data ready flag
 */
static L3GD20_DataReadyFlagType dataReadyFlag = L3GD20_DATA_NOT_READY;

/**
 * @brief   Calculated angle values in degrees
 */
static float Angle_X = 0;
static float Angle_Y = 0;
static float Angle_Z = 0;

/**
 * @brief   Calculated angle rate values in degrees/s
 */
static float AngleRate_X = 0;
static float AngleRate_Y = 0;
static float AngleRate_Z = 0;

/**
 * @brief   Last calculated angle rate values in degrees/s
 */
static float LastAngleRate_X = 0;
static float LastAngleRate_Y = 0;
static float LastAngleRate_Z = 0;

/**
 * @brief   Calibration Counter
 */
static uint32_t caliCounter = 0;

/**
 * @brief   Noise values
 */
static float Noise_X = 0;
static float Noise_Y = 0;
static float Noise_Z = 0;

/**
 * @brief   Temp variables for collecting noise values
 */
static int32_t TempNoise_X = 0;
static int32_t TempNoise_Y = 0;
static int32_t TempNoise_Z = 0;

/**
 * @brief   Offset values
 */
static int32_t Offset_X = 0;
static int32_t Offset_Y = 0;
static int32_t Offset_Z = 0;

/**
 * @}
 */

/**
 * @defgroup L3GD20_Private_Function_Prototypes
 * @{
 */
static void L3GD20_GPIO_Init(void);
static void L3GD20_EXTI_Init(void);
static void L3GD20_SPI5_Init(void);
static void L3GD20_TX_DMA_Init(void);
static void L3GD20_RX_DMA_Init(void);
static void L3GD20_Write(uint8_t * pBuffer, uint8_t WriteAddr,
    uint16_t NumByteToWrite);
static void L3GD20_Read(uint8_t * pBuffer, uint8_t ReadAddr,
    uint16_t NumByteToRead);
static void Transmit(const uint8_t * pBuffer, uint8_t size);
static void Receive(const uint8_t * pBuffer, uint8_t size);

/**
 * @}
 */

/**
 * @defgroup L3GD20_Private_Functions
 * @{
 */

/**
 * @brief   Configure GPIO
 * @note    PF7 -> SPI5_SCK (AF5)
 *          PF8 -> SPI5_MISO (AF5)
 *          PF9 -> SPI5_MOSI (AF5)
 *          PC1 -> NCS_MEMS_SPI
 *          PA2 -> MEMS_INT2
 *          PA1 -> MEMS_INT1
 *          SPI5 connected to APB2 with 90MHz max clock
 * @param   None
 * @retval  None
 */
static void L3GD20_GPIO_Init(void)
{
  /* Configure MISO, MOSI and SCK */
  /* **************************** */

  /* Enable port F clock */
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOFEN;

  /* Select Alternate function mode */
  GPIOF->MODER &= ~(GPIO_MODER_MODER7 | GPIO_MODER_MODER8
      | GPIO_MODER_MODER9);
  GPIOF->MODER |= (GPIO_MODER_MODER7_1 | GPIO_MODER_MODER8_1
      | GPIO_MODER_MODER9_1);

  /* Output push-pull (reset state) */
  GPIOF->OTYPER &= ~(GPIO_OTYPER_OT_7 | GPIO_OTYPER_OT_9);

  /* Select output speed medium */
  GPIOF->OSPEEDR &= ~(GPIO_OSPEEDER_OSPEEDR7 | GPIO_OSPEEDER_OSPEEDR9);
  GPIOF->OSPEEDR |= (GPIO_OSPEEDER_OSPEEDR7_0 | GPIO_OSPEEDER_OSPEEDR9_0);

  /* Select no pull-up, pull-down (reset state) */
  GPIOF->PUPDR &= ~(GPIO_PUPDR_PUPDR7 | GPIO_PUPDR_PUPDR8
      | GPIO_PUPDR_PUPDR9);

  /* Select AF5 */
  GPIOF->AFR[0] &= ~(GPIO_AFRL_AFRL7);
  GPIOF->AFR[1] &= ~(GPIO_AFRH_AFRH8 | GPIO_AFRH_AFRH9);
  GPIOF->AFR[0] |= GPIO_AFRL_AFRL7_AF5;
  GPIOF->AFR[1] |= (GPIO_AFRH_AFRH8_AF5 | GPIO_AFRH_AFRH9_AF5);

  /* Configure PC1 -> NCS_MEMS_SPI as GPIO */
  /* ************************************* */

  /* Enable port C clock */
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;

  /* Select General purpose output mode */
  GPIOC->MODER &= ~(GPIO_MODER_MODER1);
  GPIOC->MODER |= GPIO_MODER_MODER1_0;

  /* Output push-pull (reset state) */
  GPIOC->OTYPER &= ~GPIO_OTYPER_OT_1;

  /* Select output speed medium */
  GPIOC->OSPEEDR &= ~(GPIO_OSPEEDER_OSPEEDR1);
  GPIOC->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR1_0;

  /* Select no pull-up, pull-down (reset state) */
  GPIOC->PUPDR &= ~GPIO_PUPDR_PUPDR1;

  /* Set Chip Select high */
  GPIOC->BSRRL = GPIO_BSRR_BS_1;

  /* Configure PA2 -> MEMS_INT2 & PA1 -> MEMS_INT1 */
  /* ********************************************* */

  /* Enable port A clock */
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;

  /* Select input mode */
  GPIOA->MODER &= ~(GPIO_MODER_MODER1 | GPIO_MODER_MODER2);

  /* Select no pull-up, pull-down (reset state) */
  GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPDR1 | GPIO_PUPDR_PUPDR2);
}

/**
 * @brief   Configure EXTI
 * @note    PA2 -> MEMS_INT2
 *          PA1 -> MEMS_INT1
 * @param   None
 * @retval  None
 */
static void L3GD20_EXTI_Init(void)
{
  /* Enable SYSCFG clock */
  RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;

  /* Map PA1 to EXT1 & PA2 to EXT2 */
  SYSCFG->EXTICR[0] |= (SYSCFG_EXTICR1_EXTI1_PA | SYSCFG_EXTICR1_EXTI2_PA);

  /* Enable interrupt line */
  EXTI->IMR |= (EXTI_IMR_MR1 | EXTI_IMR_MR2);

  /* Enable rising edge trigger */
  EXTI->RTSR |= (EXTI_RTSR_TR1 | EXTI_RTSR_TR2);

  /* Disable falling edge trigger */
  EXTI->FTSR &= ~(EXTI_FTSR_TR1 | EXTI_FTSR_TR2);
}

/**
 * @brief   Configure SPI5
 * @note    SPI5 connected to APB2 with 90MHz max clock
 *
 * @param   None
 * @retval  None
 */
static void L3GD20_SPI5_Init(void)
{
  /* Enable SPI5 clock */
  RCC->APB2ENR |= RCC_APB2ENR_SPI5EN;

  /* Reset SPI control register 1,
   * 2-line unidirectional data mode selected
   * CRC calculation disabled
   * 8-bit data frame format is selected for transmission/reception
   * Full duplex (Transmit and receive)
   * MSB transmitted first */
  SPI5->CR1 = 0;

  /* Enable Software slave management */
  SPI5->CR1 |= SPI_CR1_SSM;

  /* Set Master mode and Set master NSS to high  */
  SPI5->CR1 |= (SPI_CR1_SSI | SPI_CR1_MSTR);

  /* Set baud rate fPCLK/32 -> 90/32 MHz*/
  SPI5->CR1 |= SPI_CR1_BR_2;

  /* Select Clock polarity high when idle */
  SPI5->CR1 |= SPI_CR1_CPOL;

  /* Select capture strobe at second edge */
  SPI5->CR1 |= SPI_CR1_CPHA;

  /* Reset SPI control register 2,
   * TXE interrupt masked
   * RXNE interrupt masked
   * Error interrupt is masked
   * SPI Motorola mode
   * SS output is disabled in master mode */
  SPI5->CR2 = 0;

  /* Enable Tx and Rx DMA */
  SPI5->CR2 |= (SPI_CR2_TXDMAEN | SPI_CR2_RXDMAEN);

  /* Enable SPI5 */
  SPI5->CR1 |= SPI_CR1_SPE;
}

/**
 * @brief   Configure DMA for SPI5 TX
 * @note    SPI5_TX -> DMA2_Stream4 (Channel 2)
 * @param   None
 * @retval  None
 */
static void L3GD20_TX_DMA_Init(void)
{
  /* Enable DMA clock in RCC */
  RCC->AHB1ENR |= RCC_AHB1ENR_DMA2EN;

  /* Make sure that the DMA2_Stream4 is disabled */
  if(DMA_SxCR_EN == (DMA_SxCR_EN & DMA2_Stream4->CR))
  {
    /* DMA2_Stream4 is enabled, shall be disabled first */
    DMA2_Stream4->CR &= ~DMA_SxCR_EN;

    /* Wait until EN bit is cleared */
    while(DMA_SxCR_EN == (DMA_SxCR_EN & DMA2_Stream4->CR))
    {
      /* Do nothing until EN bit is cleared */
    }
  }
  else
  {
    /* Do nothing, DMA2_Stream4 is not enabled */
  }

  /* Select the DMA channel 2 in CHSEL[2:0] in the DMA_SxCR */
  DMA2_Stream4->CR &= ~DMA_SxCR_CHSEL;
  DMA2_Stream4->CR |= DMA_SxCR_CHSEL_1;

  /* Select stream priority very high */
  DMA2_Stream4->CR |= DMA_SxCR_PL;

  /* Select the data transfer direction memory-to-peripheral */
  DMA2_Stream4->CR &= ~DMA_SxCR_DIR;
  DMA2_Stream4->CR |= DMA_SxCR_DIR_0;

  /* Select memory and peripherals sizes byte (8-bit) */
  DMA2_Stream4->CR &= ~DMA_SxCR_MSIZE;
  DMA2_Stream4->CR &= ~DMA_SxCR_PSIZE;

  /* Select FIFO mode */
  DMA2_Stream4->FCR |= DMA_SxFCR_DMDIS;

  /* Select FIFO threshold level half FIFO */
  DMA2_Stream4->FCR |= (DMA_SxFCR_FTH_0 | DMA_SxFCR_FTH_1);

  /* Select memory incremented mode, peripheral shall has fixed address */
  DMA2_Stream4->CR |= DMA_SxCR_MINC;

  /* Enable DMA transfer complete interrupt */
  DMA2_Stream4->CR |= DMA_SxCR_TCIE;

  /* Set peripheral address */
  DMA2_Stream4->PAR = (uint32_t)&SPI5->DR;
}

/**
 * @brief   Configure DMA for SPI5 RX
 * @note    SPI5_RX -> DMA2_Stream3 (Channel 2)
 * @param   None
 * @retval  None
 */
static void L3GD20_RX_DMA_Init(void)
{
  /* Enable DMA clock in RCC */
  RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN;

  /* Make sure that the DMA2_Stream3 is disabled */
  if(DMA_SxCR_EN == (DMA_SxCR_EN & DMA2_Stream3->CR))
  {
    /* DMA2_Stream3 is enabled, shall be disabled first */
    DMA2_Stream3->CR &= ~DMA_SxCR_EN;

    /* Wait until EN bit is cleared */
    while(DMA_SxCR_EN == (DMA_SxCR_EN & DMA2_Stream3->CR))
    {
      /* Do nothing until EN bit is cleared */
    }
  }
  else
  {
    /* Do nothing, DMA2_Stream3 is not enabled */
  }

  /* Select the DMA channel 2 in CHSEL[2:0] in the DMA_SxCR */
  DMA2_Stream3->CR &= ~DMA_SxCR_CHSEL;
  DMA2_Stream3->CR |= DMA_SxCR_CHSEL_1;

  /* Select stream priority very high */
  DMA2_Stream3->CR |= DMA_SxCR_PL;

  /* Select the data transfer direction peripheral-to-memory */
  DMA2_Stream3->CR &= ~DMA_SxCR_DIR;

  /* Select memory and peripherals sizes byte (8-bit) */
  DMA2_Stream3->CR &= ~DMA_SxCR_MSIZE;
  DMA2_Stream3->CR &= ~DMA_SxCR_PSIZE;

  /* Select FIFO mode */
  DMA2_Stream3->FCR |= DMA_SxFCR_DMDIS;

  /* Select FIFO threshold level half FIFO */
  DMA2_Stream3->FCR |= (DMA_SxFCR_FTH_0 | DMA_SxFCR_FTH_1);

  /* Select memory incremented mode, peripheral shall has fixed address */
  DMA2_Stream3->CR |= DMA_SxCR_MINC;

  /* Enable DMA transfer complete interrupt */
  DMA2_Stream3->CR |= DMA_SxCR_TCIE;

  /* Set peripheral address */
  DMA2_Stream3->PAR = (uint32_t)&SPI5->DR;
}

/**
 * @brief   Write register data
 * @note
 * @param   pBuffer, WriteAddr, NumByteToWrite
 * @retval  None
 */
static void L3GD20_Write(uint8_t * pBuffer, uint8_t WriteAddr,
    uint16_t NumByteToWrite)
{
  /* Select write mode */
  WriteAddr &= ~RW_READ_WRITE_CMD;

  /* Configure the MS bit to auto increment when multiplebyte transfer */
  if(NumByteToWrite > 0x01)
  {
    WriteAddr |= MULTIPLEBYTE_CMD;
  }
  else
  {
    /* Single byte */
  }

  /* Set Chip Select low */
  GPIOC->BSRRH = GPIO_BSRR_BS_1;

  /* Start RX DMA to read the data, first byte is dummy */
  Receive(RxDMABuffer, (NumByteToWrite + 1));

  /* Insert register address into TX buffer */
  pBuffer[0] = WriteAddr;

  /* Start TX DMA to request data read, one more byte is sent as dummy */
  Transmit(pBuffer, (NumByteToWrite + 1));
}

/**
 * @brief   Read register data
 * @note
 * @param   pBuffer, WriteAddr, NumByteToWrite
 * @retval  None
 */
static void L3GD20_Read(uint8_t * pBuffer, uint8_t ReadAddr,
    uint16_t NumByteToRead)
{
  /* Select read mode */
  ReadAddr |= RW_READ_WRITE_CMD;

  /* Configure the MS bit to auto increment when multiplebyte transfer */
  if(NumByteToRead > 0x01)
  {
    ReadAddr |= MULTIPLEBYTE_CMD;
  }
  else
  {
    /* Single byte */
  }

  /* Set Chip Select low */
  GPIOC->BSRRH = GPIO_BSRR_BS_1;

  /* Start RX DMA to read the data, first byte is dummy */
  Receive(pBuffer, (NumByteToRead + 1));

  /* Insert register address into TX buffer */
  TxDMABuffer[0] = ReadAddr;

  /* Start TX DMA to request data read, one more byte is sent as dummy */
  Transmit(TxDMABuffer, (NumByteToRead + 1));
}

/**
 * @brief   DMA data transmit
 * @note    SPI5_TX -> DMA2_Stream4 (Channel 2)
 * @param   pBuffer, size
 * @retval  None
 */
static void Transmit(const uint8_t * pBuffer, uint8_t size)
{
  /* Check null pointers */
  if(NULL != pBuffer)
  {
    /* Wait until DMA2_Stream4 is disabled */
    while(DMA_SxCR_EN == (DMA_SxCR_EN & DMA2_Stream4->CR))
    {
      /* Do nothing, the enable flag shall reset
       * when DMA transfer complete */
    }

    /* Set memory address */
    DMA2_Stream4->M0AR = (uint32_t)pBuffer;

    /* Set number of data items */
    DMA2_Stream4->NDTR = size;

    /* Clear all interrupt flags */
    DMA2->HIFCR = (DMA_HIFCR_CFEIF4 | DMA_HIFCR_CDMEIF4 | DMA_HIFCR_CTEIF4
        | DMA_HIFCR_CHTIF4 | DMA_HIFCR_CTCIF4);

    /* Enable DMA2_Stream4 */
    DMA2_Stream4->CR |= DMA_SxCR_EN;
  }
  else
  {
    /* Null pointers, do nothing */
  }
}

/**
 * @brief   DMA data receive
 * @note    SPI5_RX -> DMA2_Stream3 (Channel 2)
 * @param   pBuffer, size
 * @retval  None
 */
static void Receive(const uint8_t * pBuffer, uint8_t size)
{
  /* Check null pointers */
  if(NULL != pBuffer)
  {
    /* Wait until DMA2_Stream3 is disabled */
    while(DMA_SxCR_EN == (DMA_SxCR_EN & DMA2_Stream3->CR))
    {
      /* Do nothing, the enable flag shall reset
       * when DMA transfer complete */
    }

    /* Set memory address */
    DMA2_Stream3->M0AR = (uint32_t)pBuffer;

    /* Set number of data items */
    DMA2_Stream3->NDTR = size;

    /* Clear all interrupt flags */
    DMA2->LIFCR = (DMA_LIFCR_CFEIF3 | DMA_LIFCR_CDMEIF3 | DMA_LIFCR_CTEIF3
        | DMA_LIFCR_CHTIF3 | DMA_LIFCR_CTCIF3);

    /* Enable DMA2_Stream3 */
    DMA2_Stream3->CR |= DMA_SxCR_EN;
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
 * @defgroup L3GD20_Exported_Functions
 * @{
 */

/**
 * @brief   L3GD20 Init function
 * @note
 * @param   None
 * @retval  None
 */
void L3GD20_Init(void)
{
  /* Configure GPIO */
  L3GD20_GPIO_Init();

  /* Configure EXTI */
  L3GD20_EXTI_Init();

  /* Configure DMA for SPI5 TX */
  L3GD20_TX_DMA_Init();

  /* Configure DMA for SPI5 RX */
  L3GD20_RX_DMA_Init();

  /* Configure SPI5 */
  L3GD20_SPI5_Init();

  /* Wait 10ms for the sensor boot procedure */
  SysTick_Delay(10);

  /* Update Control registers */
  TxDMABuffer[1] = L3GD20_CTRL_REG1_VAL;
  TxDMABuffer[2] = L3GD20_CTRL_REG2_VAL;
  TxDMABuffer[3] = L3GD20_CTRL_REG3_VAL;
  TxDMABuffer[4] = L3GD20_CTRL_REG4_VAL;
  TxDMABuffer[5] = L3GD20_CTRL_REG5_VAL;

  /* Write Control registers */
  L3GD20_Write(TxDMABuffer, L3GD20_CTRL_REG1_ADDR, 5);
}

/**
 * @brief   L3GD20 main function
 * @note
 * @param   None
 * @retval  None
 */
void L3GD20_Main(void)
{
  /* Raw angular rate data */
  int16_t Raw_X = 0;
  int16_t Raw_Y = 0;
  int16_t Raw_Z = 0;

  /* Variable for time difference calculations */
  static uint32_t startTick = 0;
  float diffTime = 0;

  /* Variables for moving average calculation */
  int16_t averageWindow_X[AVERAGE_WINDOW_SIZE] = {0};
  int16_t averageWindow_Y[AVERAGE_WINDOW_SIZE] = {0};
  int16_t averageWindow_Z[AVERAGE_WINDOW_SIZE] = {0};

  uint32_t windowPosition = 0;
  int32_t tempSum_X = 0;
  int32_t tempSum_Y = 0;
  int32_t tempSum_Z = 0;

  /* Check current state */
  switch (currentState)
  {
    case L3GD20_READ:
      /* Wait for data ready flag to be set before */
      if(L3GD20_DATA_READY == dataReadyFlag)
      {
        /* Get current SysTick */
        startTick = SysTick_GetCurrentTick();

        /* DMA is finished, start data request */
        /* Read output registers */
        L3GD20_Read(RxDMABuffer, L3GD20_OUT_X_L_ADDR, 6);

        /* Change current state */
        currentState = L3GD20_WAIT_FOR_DATA;

        /* Clear data ready flag */
        dataReadyFlag = L3GD20_DATA_NOT_READY;
      }
      else
      {
        /* Do nothing, DMA is not finished */
      }
      break;

    case L3GD20_WAIT_FOR_DATA:
      /* Control LEDs */
      if( (Angle_Z > 20.0) || (Angle_Z < -20.0) )
      {
        /* Turn ON */
        GPIO_TurnON_LED(EVAL_ALL_LEDs);
      }
      else
      {
        /* Turn OFF */
        GPIO_TurnOFF_LED(EVAL_ALL_LEDs);
      }

      /* Check if data ready */
      if(L3GD20_DATA_READY == dataReadyFlag)
      {
        /* Data is now ready, go to process */
        currentState = L3GD20_DATA_PROCESS;
      }
      else
      {
        /* Do nothing, Data not ready */
      }
      break;

    case L3GD20_DATA_PROCESS:
      /* Read buffer data */
      Raw_X = (RxDMABuffer[2] << 8) | RxDMABuffer[1];
      Raw_Y = (RxDMABuffer[4] << 8) | RxDMABuffer[3];
      Raw_Z = (RxDMABuffer[6] << 8) | RxDMABuffer[5];

      /* Check sensor calibration */
      if ( L3GD20_CALIBRATED == currentCaliState )
      {
        /* Get current SysTick */
        uint32_t stopTick = SysTick_GetCurrentTick();

        /* Calculate time difference */
        diffTime = (float)(stopTick - startTick) / (1000.0);
        diffTime = 0.003f;

        /* Get angle rate values in degrees/s */
        AngleRate_X = (float)(Raw_X - Offset_X) * L3GD20_SENSITIVITY;
        AngleRate_Y = (float)(Raw_Y - Offset_Y) * L3GD20_SENSITIVITY;
        AngleRate_Z = (float)(Raw_Z - Offset_Z) * L3GD20_SENSITIVITY;

        /* Check angle rate X value against noise levels */
        if( (AngleRate_X > Noise_X) || (AngleRate_X < -Noise_X) )
        {
          /* Calculate angle in degrees */
          Angle_X += ((AngleRate_X + LastAngleRate_X) * diffTime) / 2.0;

          /* Store last sampled data */
          LastAngleRate_X = AngleRate_X;
        }
        else
        {
          /* Do nothing, angle rate value is noise */
        }

        /* Check angle rate Y value against noise levels */
        if( (AngleRate_Y > Noise_Y) || (AngleRate_Y < -Noise_Y) )
        {
          /* Calculate angle in degrees */
          Angle_Y += ((AngleRate_Y + LastAngleRate_Y) * diffTime) / 2.0;

          /* Store last sampled data */
          LastAngleRate_Y = AngleRate_Y;
        }
        else
        {
          /* Do nothing, angle rate value is noise */
        }

        /* Check angle rate Z value against noise levels */
        if( (AngleRate_Z > Noise_Z) || (AngleRate_Z < -Noise_Z) )
        {
          /* Calculate angle in degrees */
          Angle_Z += ((AngleRate_Z + LastAngleRate_Z) * diffTime) / 2.0;

          /* Store last sampled data */
          LastAngleRate_Z = AngleRate_Z;
        }
        else
        {
          /* Do nothing, angle rate value is noise */
        }
      }
      else
      {
        /* Start sensor calibration */
        switch (currentCaliState)
        {
          case L3GD20_COLLECT_CALIBRATION_SAMPLES:
            /* Fill calibration buffer */
            calibrationBuffer_X[caliCounter] = Raw_X;
            calibrationBuffer_Y[caliCounter] = Raw_Y;
            calibrationBuffer_Z[caliCounter] = Raw_Z;

            /* Increment calibration counter */
            caliCounter++;

            /* Check calibration counter */
            if(caliCounter >= CALIBRATION_BUFFER_LENGTH)
            {
              /* Finish sampling */
              caliCounter = 0;

              /* Change current calibration state */
              currentCaliState = L3GD20_PROCESS_CALIBRATION_SAMPLES;
            }
            else
            {
              /* Do nothing, still collecting samples */
            }
            break;

          case L3GD20_PROCESS_CALIBRATION_SAMPLES:
            /* Calculate offset using moving average */
            for(uint32_t idx = 0; idx < CALIBRATION_BUFFER_LENGTH; idx++)
            {
              /* Subtract the oldest sample from the old sum, add the new number */
              tempSum_X = tempSum_X - averageWindow_X[windowPosition] +
                  calibrationBuffer_X[idx];
              tempSum_Y = tempSum_Y - averageWindow_Y[windowPosition] +
                  calibrationBuffer_Y[idx];
              tempSum_Z = tempSum_Z - averageWindow_Z[windowPosition] +
                  calibrationBuffer_Z[idx];

              /* Store current sample in the average window */
              averageWindow_X[windowPosition] = calibrationBuffer_X[idx];
              averageWindow_Y[windowPosition] = calibrationBuffer_Y[idx];
              averageWindow_Z[windowPosition] = calibrationBuffer_Z[idx];

              /* Calculate the offset */
              Offset_X =  tempSum_X / (int32_t)AVERAGE_WINDOW_SIZE;
              Offset_Y =  tempSum_Y / (int32_t)AVERAGE_WINDOW_SIZE;
              Offset_Z =  tempSum_Z / (int32_t)AVERAGE_WINDOW_SIZE;

              /* Increment window position */
              windowPosition++;

              /* Check last window position */
              if (windowPosition >= AVERAGE_WINDOW_SIZE)
              {
                /* Reset window position */
                windowPosition = 0;
              }
              else
              {
                /* Do nothing, last position is not yet reached */
              }
            }

            /* Calculate noise levels */
            for(uint32_t idx = 0; idx < CALIBRATION_BUFFER_LENGTH; idx++)
            {
              if( ((int32_t) calibrationBuffer_X[idx] - Offset_X) > TempNoise_X)
              {
                TempNoise_X = (int32_t) calibrationBuffer_X[idx] - Offset_X;
              }
              else if( ((int32_t) calibrationBuffer_X[idx] - Offset_X) < -TempNoise_X)
              {
                TempNoise_X = -((int32_t) calibrationBuffer_X[idx] - Offset_X);
              }

              /* Calculate Y noise values */
              if( ((int32_t) calibrationBuffer_Y[idx] - Offset_Y) > TempNoise_Y)
              {
                TempNoise_Y = (int32_t) calibrationBuffer_Y[idx] - Offset_Y;
              }
              else if( ((int32_t) calibrationBuffer_Y[idx] - Offset_Y) < -TempNoise_Y)
              {
                TempNoise_Y = -((int32_t) calibrationBuffer_Y[idx] - Offset_Y);
              }

              /* Calculate Z noise values */
              if( ((int32_t) calibrationBuffer_Z[idx] - Offset_Z) > TempNoise_Z)
              {
                TempNoise_Z = (int32_t) calibrationBuffer_Z[idx] - Offset_Z;
              }
              else if( ((int32_t) calibrationBuffer_Z[idx] - Offset_Z) < -TempNoise_Z)
              {
                TempNoise_Z = -((int32_t) calibrationBuffer_Z[idx] - Offset_Z);
              }
            }

            /* Get noise values in degree/s */
            Noise_X = (float)TempNoise_X * L3GD20_SENSITIVITY;
            Noise_Y = (float)TempNoise_Y * L3GD20_SENSITIVITY;
            Noise_Z = (float)TempNoise_Z * L3GD20_SENSITIVITY;

            /* Change current calibration state */
            currentCaliState = L3GD20_CALIBRATED;
            break;

          case L3GD20_CALIBRATED:
            break;

          default:
            break;
        }
      }

      /* Change current state */
      currentState = L3GD20_READ;
      break;

    default:
      break;
  }
}

/**
 * @brief   Callback function
 * @note    Called when MEMS_INT1
 * @param   None
 * @retval  None
 */
void EXTI1_MEMS_INT1_Callback()
{
  /* Clear pending bit */
  EXTI->PR |= EXTI_PR_PR1;
}

/**
 * @brief   Callback function
 * @note    Called when MEMS_INT2
 * @param   None
 * @retval  None
 */
void EXTI2_MEMS_INT2_Callback()
{
  /* Clear pending bit */
  EXTI->PR |= EXTI_PR_PR2;
}

/**
 * @brief   IRQ callback function
 * @note    SPI5_TX -> DMA2_Stream4 (Channel 2)
 * @param   None
 * @retval  None
 */
void SPI5_TX_DMA_IRQ_Callback(void)
{
  /* Check transfer complete flag */
  if(DMA_HISR_TCIF4 == (DMA_HISR_TCIF4 & DMA2->HISR))
  {
    /* Clear all interrupt flags */
    DMA2->HIFCR = (DMA_HIFCR_CFEIF4 | DMA_HIFCR_CDMEIF4 | DMA_HIFCR_CTEIF4
        | DMA_HIFCR_CHTIF4 | DMA_HIFCR_CTCIF4);
  }
  else
  {
    /* Do nothing, this interrupt is not handled */
  }
}

/**
 * @brief   IRQ callback function
 * @note    SPI5_RX -> DMA2_Stream3 (Channel 2)
 * @param   None
 * @retval  None
 */
void SPI5_RX_DMA_IRQ_Callback(void)
{
  /* Check transfer complete flag */
  if(DMA_LISR_TCIF3 == (DMA_LISR_TCIF3 & DMA2->LISR))
  {
    /* Clear all interrupt flags */
    DMA2->LIFCR = (DMA_LIFCR_CFEIF3 | DMA_LIFCR_CDMEIF3 | DMA_LIFCR_CTEIF3
        | DMA_LIFCR_CHTIF3 | DMA_LIFCR_CTCIF3);

    /* Data is now ready */
    dataReadyFlag = L3GD20_DATA_READY;

    /* Set Chip Select high */
    GPIOC->BSRRL = GPIO_BSRR_BS_1;
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
