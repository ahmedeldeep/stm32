/*******************************************************************************
 * @file    irq.c
 * @author  Ahmed Eldeep
 * @email   ahmed@almohandes.org
 * @website http://almohandes.org/stm32
 * @date    31.03.2018
 *          
 * @brief   Some IRQs examples using NVIC
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
#include "irq.h"

/**
 * @addtogroup stm32_examples
 * @{
 */

/**
 * @defgroup irq
 * @brief
 * @{
 */

/**
 * @defgroup irq_private_typedefs
 * @{
 */

/**
 * @}
 */

/**
 * @defgroup irq_private_defines
 * @{
 */

/**
 * @}
 */

/**
 * @defgroup irq_private_macros
 * @{
 */

/**
 * @}
 */

/**
 * @defgroup irq_private_constants
 * @{
 */

/**
 * @}
 */

/**
 * @defgroup irq_private_variables
 * @{
 */

/**
 * @}
 */

/**
 * @defgroup irq_private_function_prototypes
 * @{
 */

/**
 * @}
 */

/**
 * @defgroup irq_private_functions
 * @{
 */

/**
 * @}
 */

/**
 * @defgroup irq_exported_functions
 * @{
 */

/**
 * @brief   NVIC IRQs initialization function
 * @note
 * @param   None
 * @retval  None
 */
void IRQ_Init(void)
{
  /* Set priority group to 3
   * bits[3:0] are the sub-priority,
   * bits[7:4] are the pre-empt priority */
  NVIC_SetPriorityGrouping(3);

  /* Set priority levels */
  NVIC_SetPriority(GREEN_LED_OFF_IRQ, 1);
  NVIC_SetPriority(RED_LED_OFF_IRQ,   2);

  NVIC_SetPriority(GREEN_LED_ON_IRQ,  3);
  NVIC_SetPriority(RED_LED_ON_IRQ,    4);

  /* Enable interrupts at NVIC */
  NVIC_EnableIRQ(GREEN_LED_OFF_IRQ);
  NVIC_EnableIRQ(RED_LED_OFF_IRQ);

  NVIC_EnableIRQ(GREEN_LED_ON_IRQ);
  NVIC_EnableIRQ(RED_LED_ON_IRQ);
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
