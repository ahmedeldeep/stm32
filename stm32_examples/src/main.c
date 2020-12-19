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
#include "stm32f4xx.h"
#include "gpio.h"

#include "rtos.h"



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

static RTOS_thread_t thread1;
static RTOS_stack_t thread1stack;
static RTOS_thread_t thread2;
static RTOS_stack_t thread2stack;

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
 * @brief   thread1function
 * @note
 * @param   none
 * @retval  none
 */
void thread1function(void)
{
  while(1)
  {
    GPIO_Toggle_LED(EVAL_GREEN_LED);

    for (int var = 0; var < 1000000; ++var)
    {
    }
  }
}
/**
 * @brief   thread2function
 * @note
 * @param   none
 * @retval  none
 */
void thread2function(void)
{

  while(1)
  {
    GPIO_Toggle_LED(EVAL_RED_LED);

    for (int var = 0; var < 1000000; ++var)
    {
    }
  }
}
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
  GPIO_Init_LED(EVAL_ALL_LEDs);


  RTOS_init();

  RTOS_SVC_threadCreate(&thread1, &thread1stack, 1, thread1function);
  RTOS_SVC_threadCreate(&thread2, &thread2stack, 1, thread2function);

  RTOS_SVC_schedulerStart();

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
