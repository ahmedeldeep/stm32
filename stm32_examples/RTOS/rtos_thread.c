/*******************************************************************************
 * @file    rtos_thread.c
 * @author  Ahmed Eldeep
 * @email   ahmed@almohandes.org
 * @website http://almohandes.org/stm32
 * @date    15 Jul 2019
 *          
 * @brief   RTOS Thread
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
#include "rtos.h"

/**
 * @addtogroup rtos
 * @{
 */

/**
 * @defgroup rtos_thread
 * @brief
 * @{
 */

/**
 * @defgroup rtos_thread_private_typedefs
 * @{
 */

/**
 * @}
 */

/**
 * @defgroup rtos_thread_private_defines
 * @{
 */

/**
 * @}
 */

/**
 * @defgroup rtos_thread_private_macros
 * @{
 */

/**
 * @}
 */

/**
 * @defgroup rtos_thread_private_constants
 * @{
 */

/**
 * @}
 */

/**
 * @defgroup rtos_thread_private_variables
 * @{
 */

/**
 * @brief   Ready list for the threads waiting for the execution,
 *          arranged by priority.
 */
static RTOS_list_t readyList[THREAD_PRIORITY_LEVELS];

/**
 * @brief   Current top priority.
 */
static uint32_t currentTopPriority = (THREAD_PRIORITY_LEVELS - 1);

/**
 * @brief   Currently running thread pointer.
 */
static RTOS_thread_t * pRunningThread;

/**
 * @brief   Currently running thread ID.
 */
static uint32_t runningThreadID = 0;

/**
 * @brief   Number of threads
 */
static uint32_t numOfThreads = 0;

/**
 * @}
 */

/**
 * @defgroup rtos_thread_private_function_prototypes
 * @{
 */

/**
 * @}
 */

/**
 * @defgroup rtos_thread_private_functions
 * @{
 */

/**
 * @}
 */

/**
 * @defgroup rtos_thread_exported_functions
 * @{
 */

/**
 * @brief   Initialize thread list
 * @note
 * @param
 * @retval  None
 */
void RTOS_threadInitLists(void)
{
  /* Initialize ready list */
  for(uint32_t priority = 0; priority < THREAD_PRIORITY_LEVELS; priority++)
  {
    RTOS_listInit(&readyList[priority]);
  }
}

/**
 * @brief   Creates new thread.
 * @note
 * @param   RTOS_thread_t *, RTOS_stack_t *, uint32_t, void *, char *
 * @retval  None
 */
void RTOS_threadCreate(RTOS_thread_t * pThread, RTOS_stack_t * pStack,
    uint32_t priority, void * pFunction)
{
  /* Check input parameters */
  ASSERT(NULL != pThread);
  ASSERT(NULL != pStack);
  ASSERT(THREAD_PRIORITY_LEVELS > priority);
  ASSERT(NULL != pFunction);

  /* Create stack frame, size multiplied with 8 for the double word
   * length converted to byte length, stack frame size is 18 words */
  pThread->pStackPointer = ((uint32_t)pStack + THREAD_STACK_SIZE * 8 - 18 * 4);

  /* Write thread function into return address */
  MEM32_ADDRESS((pThread->pStackPointer + (16 << 2))) = (uint32_t) pFunction;

  /* Write initial xPSR, program status register, thumb */
  MEM32_ADDRESS((pThread->pStackPointer + (17 << 2))) = 0x01000000;

  /* Write EXC_RETURN, since the execution threads are using PSP, this will
   * allow SVC to return to the thread with PSP */
  MEM32_ADDRESS(pThread->pStackPointer) = 0xFFFFFFFDUL;

  /* Write initial CONTROL register value UNPRIVILEGED, PSP & no FPU */
  MEM32_ADDRESS((pThread->pStackPointer + (1 << 2))) = 0x3;

  /* Set thread priority */
  pThread->priority = priority;

  /* Increment number of threads and set the thread ID */
  pThread->threadID = ++numOfThreads;

  /* Thread is not yet in any list */
  pThread->item.pList = NULL;

  /* Link this thread with its list item */
  pThread->item.pThread = (void *) pThread;

  /* Set item value to the priority */
  pThread->item.itemValue = priority;

  /* Add new thread to ready list */
  RTOS_threadAddToReadyList(pThread);
}

/**
 * @brief   Get current running thread
 * @note
 * @param
 * @retval  RTOS_thread_t *
 */
RTOS_thread_t * RTOS_threadGetRunning(void)
{
  return pRunningThread;
}

/**
 * @brief   Switch the current running thread
 * @note
 * @param
 * @retval  None
 */
void RTOS_threadSwitchRunning(void)
{
  /* Find highest priority ready thread */
  while(0 == readyList[currentTopPriority].numOfItems)
  {
    /* Check current top priority not greater than the maximum */
    ASSERT(THREAD_PRIORITY_LEVELS > currentTopPriority);

    /* No threads with the current top priority,
     * increment current top priority */
    currentTopPriority++;
  }

  /* Threads are found, update list index to the next thread */
  RTOS_list_t * pReadyList = &readyList[currentTopPriority];
  pReadyList->pIndex = pReadyList->pIndex->pNext;

  /* Check if the new index pointing to the end of the list */
  if(pReadyList->pIndex == (RTOS_listItem_t *) &pReadyList->listEnd)
  {
    /* Get the next thread */
    pReadyList->pIndex = pReadyList->pIndex->pNext;
  }
  else
  {
    /* Do nothing, index is not pointing to the end */
  }

  /* Update current running thread */
  pRunningThread = (RTOS_thread_t *) pReadyList->pIndex->pThread;

  /* Update current running thread */
  runningThreadID = pRunningThread->threadID;
}

/**
 * @brief   Add thread to the ready list
 * @note
 * @param
 * @retval  None
 */
void RTOS_threadAddToReadyList(RTOS_thread_t * pThread)
{
  /* Check input parameters */
  ASSERT(NULL != pThread);

  /* Add new thread to ready list */
  RTOS_listInsertEnd(&readyList[pThread->priority], &pThread->item);

  /* Set current top priority */
  if(pThread->priority < currentTopPriority)
  {
    currentTopPriority = pThread->priority;
  }

  /* Check the need for context switch when scheduler is running
   * and this thread is the higher priority than the running thread */
  if((NULL != pRunningThread)
      && (pThread->priority < pRunningThread->priority))
  {
    /* Trigger context switch, set PendSV to pending */
    SCB->ICSR |= SCB_ICSR_PENDSVSET_Msk;
  }
  else
  {
    /* Context switch is not required */
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
