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
#include "itm.h"

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
static RTOS_thread_t thread3;
static RTOS_stack_t thread3stack;
static RTOS_thread_t thread4;
static RTOS_stack_t thread4stack;

static RTOS_mutex_t mutex1;


/**
 * @}
 */

/**
 * @defgroup main_private_function_prototypes
 * @{
 */

static void thread1function(void);
static void thread2function(void);
static void thread3function(void);
static void thread4function(void);

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
static void thread1function(void)
{

  while(1)
  {
    RTOS_SVC_threadDelay(5);

    ITM_Printf("Thread 1: wants to lock mutex1 \n");

    RTOS_SVC_mutexLock(&mutex1, WAIT_INDEFINITELY);

    ITM_Printf("Thread 1: Mutex lock succeeded \n");

    RTOS_SVC_mutexRelease(&mutex1);
  }
}

/**
 * @brief   thread2function
 * @note
 * @param   none
 * @retval  none
 */
static void thread2function(void)
{

  while(1)
  {
    ITM_Printf("Thread 2: started \n");
    for (int var = 0; var < 100000; ++var)
    {

    }
    RTOS_SVC_threadDelay(50);
  }
}

/**
 * @brief   thread2function
 * @note
 * @param   none
 * @retval  none
 */
static void thread3function(void)
{

  while(1)
  {
    ITM_Printf("Thread 3: started \n");
    for (int var = 0; var < 100000; ++var)
    {

    }
    RTOS_SVC_threadDelay(50);
  }
}

/**
 * @brief   thread2function
 * @note
 * @param   none
 * @retval  none
 */
static void thread4function(void)
{

  while(1)
  {

    RTOS_SVC_threadPrioritySet(1);
    RTOS_SVC_mutexLock(&mutex1, WAIT_INDEFINITELY);
    ITM_Printf("Thread 4: started to work with mutex1 \n");

    for (int var = 0; var < 1000000; ++var)
    {

    }

    ITM_Printf("Thread 4: finished work with mutex1 \n");
    RTOS_SVC_mutexRelease(&mutex1);
    RTOS_SVC_threadPrioritySet(3);

    RTOS_SVC_threadDelay(5);

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
  RTOS_SVC_threadCreate(&thread2, &thread2stack, 2, thread2function);
  RTOS_SVC_threadCreate(&thread3, &thread3stack, 2, thread3function);
  RTOS_SVC_threadCreate(&thread4, &thread4stack, 3, thread4function);


  RTOS_SVC_mutexCreate(&mutex1, 1);


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
