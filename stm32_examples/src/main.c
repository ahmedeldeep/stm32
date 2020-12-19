/*******************************************************************************
 * @file    main.c
 * @author  Ahmed Eldeep
 * @email   ahmed@almohandes.org
 * @website http://almohandes.org/stm32
 * @date    21.03.2018
 *          
 * @brief   main application called after startup
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
#include "nvic.h"
#include "SysTick.h"
#include "gpio.h"
#include "exti.h"
#include "memcpy.h"
#include "dma.h"

/**
 * @addtogroup stm32_examples
 * @{
 */

/**
 * @defgroup main
 * @brief
 * @{
 */

/**
 * @defgroup main_private_typedefs
 * @{
 */

/**
 * @}
 */

/**
 * @defgroup main_private_defines
 * @{
 */

/**
 * @brief   Buffer array size
 */
#define BUFFER_SIZE     ((uint32_t) 500)

/**
 * @}
 */

/**
 * @defgroup main_private_macros
 * @{
 */

/**
 * @}
 */

/**
 * @defgroup main_private_constants
 * @{
 */

/**
 * @}
 */

/**
 * @defgroup main_private_variables
 * @{
 */

/**
 * @brief   Source and destination buffers
 */
static uint32_t srcBuffer[BUFFER_SIZE];
static uint32_t dstBuffer[BUFFER_SIZE];

/**
 * @}
 */

/**
 * @defgroup main_private_function_prototypes
 * @{
 */

/**
 * @}
 */

/**
 * @defgroup main_private_functions
 * @{
 */

/**
 * @}
 */

/**
 * @defgroup main_exported_functions
 * @{
 */

/**
 * @brief   Main function
 * @note
 * @param   none
 * @retval  none
 */
int main(void)
{
//  ReturnStatus_Type returnResults = RETURN_STATUS_NOT_OK;

  SysTick_Init();
  GPIO_Init_LED(EVAL_ALL_LEDs);
  GPIO_Init_PB();
  EXTI_Init_PB();
  NVIC_Init();

  DMA2_Stream0_Init();
  DMA2_Stream0_Set_Addresses(srcBuffer, dstBuffer, BUFFER_SIZE);

  /* Initialize source buffer */
  for (int idx = 0; idx < BUFFER_SIZE; idx++)
  {
    srcBuffer[idx] = idx * idx;
  }

//  /* Copy data */
//  returnResults = memcpy32(srcBuffer, dstBuffer, BUFFER_SIZE);
//
//  /* Check return status */
//  if(RETURN_STATUS_OK != returnResults)
//  {
//    /* Copying is not OK, Turn on red LED */
//    GPIO_TurnON_LED(EVAL_RED_LED);
//  }
//  else
//  {
//    /* Copying is OK, Turn off green LED */
//    GPIO_TurnOFF_LED(EVAL_GREEN_LED);
//  }

  /* Clear PRIMASK, enable IRQs */
  __enable_irq();

  /* Signal start of coping data, by turning on green LED */
  GPIO_TurnON_LED(EVAL_GREEN_LED);

  /* DMA2 Stream 0 enable */
  DMA2_Stream0_Enable();

  /* Infinite loop */
  while(1)
  {

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
