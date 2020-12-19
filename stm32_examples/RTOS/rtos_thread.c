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
 * @brief   Timer list for the threads waiting, arranged by timeout value.
 */
static RTOS_list_t timerList;

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
 * @brief   Variable to store millisecond ticks
 */
static volatile uint32_t sysTickCounter = 0;

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

  /* Initialize timer list */
  RTOS_listInit(&timerList);
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

  #if(1 == USE_PRIORITY_INHERITANCE)
  {
    /* Set thread original priority */
    pThread->originalPriority = priority;
  }
  #endif

  /* Increment number of threads and set the thread ID */
  if(0 == pThread->threadID)
  {
    /* New thread is created */
    pThread->threadID = ++numOfThreads;
  }
  else
  {
    /* Do nothing, this thread was re-created */
  }


  /* Thread is not yet in any list */
  pThread->genericListItem.pList = NULL;
  pThread->eventListItem.pList = NULL;

  /* Link this thread with its list items */
  pThread->genericListItem.pThread = (void *) pThread;
  pThread->eventListItem.pThread = (void *) pThread;

  /* Set the event item value to the priority, this will be used to order the
   * items by priority in synchronization events list, for the generic lists
   * e.g. timer list, items are ordered with the timeout value */
  pThread->eventListItem.itemValue = priority;
  pThread->genericListItem.itemValue = 0;

  /* Add new thread to ready list */
  RTOS_threadAddToReadyList(pThread);
}

/**
 * @brief   Destroys a thread
 * @note
 * @param   RTOS_thread_t *
 * @retval  None
 */
void RTOS_threadDestroy(RTOS_thread_t * pThread)
{
  /* Check input parameters */
  ASSERT(NULL != pThread);

  /* Check if the generic list item in any list */
  if(NULL != pThread->genericListItem.pList)
  {
    /* Remove the generic list item from the current list */
    RTOS_listRemove(&pThread->genericListItem);
  }
  else
  {
    /* Do nothing, generic list item is not in any list */
  }

  /* Check if the event list item in any list */
  if(NULL != pThread->eventListItem.pList)
  {
    /* Remove the event list item from the current list */
    RTOS_listRemove(&pThread->eventListItem);
  }
  else
  {
    /* Do nothing, event list item is not in any list */
  }

  /* Check if the removed thread is the current thread */
  if(pThread == pRunningThread)
  {
    /* The current thread will be removed, request context switch  */
    SCB->ICSR |= SCB_ICSR_PENDSVSET_Msk;
  }
  else
  {
    /* Do nothing */
  }
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
 * @param   RTOS_thread_t *
 * @retval  None
 */
void RTOS_threadAddToReadyList(RTOS_thread_t * pThread)
{
  /* Check input parameters */
  ASSERT(NULL != pThread);

  /* Add new thread to ready list */
  RTOS_listInsertEnd(&readyList[pThread->priority], &pThread->genericListItem);

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
 * @brief   Refresh the timer list
 * @note
 * @param   None
 * @retval  None
 */
void RTOS_threadRefreshTimerList(void)
{
  /* Pointer to the unblocked thread */
  RTOS_thread_t * pThread;

  /* Check scheduler status */
  if(1 == RTOS_isSchedulerRunning())
  {
    /* Increment SysTick counter */
    ++sysTickCounter;

    /* Check SysTick counter overflow */
    if(0 == sysTickCounter)
    {
      /* TODO: Handle counter overflow */
      ASSERT(0);
    }
    else
    {
      /* No counter overflow, do nothing */
    }

    /* Check if timer list has threads waiting */
    if(0 < timerList.numOfItems)
    {
      /* Timer list is not empty, check timeout values */
      while(sysTickCounter >= timerList.listEnd.pNext->itemValue)
      {
        /* Get first thread waiting */
        pThread = timerList.listEnd.pNext->pThread;

        /* Check returned thread */
        ASSERT(NULL != pThread);

        /* Thread timeout, remove from timer list */
        RTOS_listRemove(&pThread->genericListItem);

        /* Check if the thread waiting for synchronization event */
        if(NULL != pThread->eventListItem.pList)
        {
          /* Remove the thread from the event list */
          RTOS_listRemove(&pThread->eventListItem);
        }
        else
        {
          /* Do nothing, this thread is not in any event lists */
        }

        /* Add the returned thread into ready list */
        RTOS_threadAddToReadyList(pThread);
      }
    }
    else
    {
      /* Timer list is empty, do nothing */
    }
  }
  else
  {
    /* Scheduler is not running, do nothing */
  }
}

/**
 * @brief   Add thread to the timer list
 * @note
 * @param   None
 * @retval  None
 */
void RTOS_threadAddRunningToTimerList(uint32_t waitTime)
{
  /* Check input parameters */
  ASSERT(0 != waitTime);

  /* Temp variable for the wake up tick */
  uint32_t wakeUpTick = 0;

  /* Calculate wake up tick */
  wakeUpTick = sysTickCounter + waitTime;

  /* Check counter overflow */
  if(sysTickCounter > wakeUpTick)
  {
    /* TODO: Handle overflow */
    ASSERT(0);
  }
  else
  {
    /* No overflow, do nothing */
  }

  /* Set generic list item value */
  pRunningThread->genericListItem.itemValue = wakeUpTick;

  /* Remove from ready list */
  RTOS_listRemove(&pRunningThread->genericListItem);

  /* Add to timer list */
  RTOS_listInsert(&timerList, &pRunningThread->genericListItem);

  /* Trigger context switch, set PendSV to pending */
  SCB->ICSR |= SCB_ICSR_PENDSVSET_Msk;
}

/**
 * @brief   Priority inherit
 * @note    Gives the passed thread the priority of the current thread
 * @param   RTOS_thread_t *
 * @retval  None
 */
#if(1 == USE_PRIORITY_INHERITANCE)
void RTOS_threadPriorityInherit(RTOS_thread_t * pThread)
{
  /* Check input parameters */
  ASSERT(NULL != pThread);

  /* Check if the passed thread has higher priority that the running thread */
  if(pThread->priority > pRunningThread->priority)
  {
    /* Passed thread has priority lower than the current thread,
     * do priority inheritance */
    /* Update event list item value */
    pThread->eventListItem.itemValue = pRunningThread->priority;

    /* Check if the passed thread in the ready list,
     * it will be removed and added again */
    if((void *) &readyList[pThread->priority] == pThread->genericListItem.pList)
    {
      /* Remove from the ready list */
      RTOS_listRemove(&pThread->genericListItem);

      /* Update thread priority */
      pThread->priority = pRunningThread->priority;

      /* Added to the ready list */
      RTOS_threadAddToReadyList(pThread);
    }
    else
    {
      /* Passed thread is not in the ready state, only update the priority */
      pThread->priority = pRunningThread->priority;
    }
  }
  else
  {
    /* Do nothing, passed thread has priority higher than or equal to
     * the current thread */
  }
}
#endif

/**
 * @brief   Priority disinherit
 * @note    Restores the original priority of the current thread
 * @param   None
 * @retval  None
 */
#if(1 == USE_PRIORITY_INHERITANCE)
void RTOS_threadPriorityDisinherit(void)
{
  /* Check current priority */
  if(pRunningThread->priority != pRunningThread->originalPriority)
  {
    /* Remove from the ready list */
    RTOS_listRemove(&pRunningThread->genericListItem);

    /* Update thread priority */
    pRunningThread->priority = pRunningThread->originalPriority;

    /* Update event list item value */
    pRunningThread->eventListItem.itemValue = pRunningThread->originalPriority;

    /* Added to the ready list */
    RTOS_threadAddToReadyList(pRunningThread);
  }
  else
  {
    /* Do nothing */
  }
}
#endif

/**
 * @brief   Sets the priority of the running thread
 * @note    Used for priority ceiling
 * @param   None
 * @retval  None
 */
#if(1 == USE_PRIORITY_SET)
void RTOS_threadPrioritySet(uint32_t newPriority)
{
  /* Check input parameters */
  ASSERT(THREAD_PRIORITY_LEVELS > newPriority);

  /* Change priority flag */
  uint32_t changePriorityReq = 0;

  /* Check if priority inheritance is enabled */
  #if(1 == USE_PRIORITY_INHERITANCE)
  {
    /* Compare new priority with the current priority */
    if(newPriority != pRunningThread->originalPriority)
    {
      /* Check if the thread inherited the priority */
      if(pRunningThread->originalPriority == pRunningThread->priority)
      {
        /* Set priority change flag */
        changePriorityReq = 1;
      }
      else
      {
        /* Do nothing, the thread has inherited priority */
      }

      /* Change the original priority */
      pRunningThread->originalPriority = newPriority;
    }
    else
    {
      /* Do nothing, thread has the same priority */
    }
  }
  #else
  {
    /* Compare new priority with the current priority */
    if(newPriority != pRunningThread->priority)
    {
        /* Set priority change flag */
        changePriorityReq = 1;
    }
    else
    {
      /* Do nothing, thread has the same priority */
    }

  }
  #endif

  /* Check priority change flag */
  if(1 == changePriorityReq)
  {
    /* Check if the new priority is lower */
    if(newPriority > pRunningThread->priority)
    {
      /* Can make other thread able to run, trigger context switch,
       * set PendSV to pending */
      SCB->ICSR |= SCB_ICSR_PENDSVSET_Msk;
    }
    else
    {
      /* When increasing the priority, no need to trigger context
       * switch because the running thread is always the highest priority
       * ready thread  */
    }

    /* Change the priority */
    pRunningThread->priority = newPriority;

    /* Set event list item value */
    pRunningThread->eventListItem.itemValue = newPriority;

    /* Remove the running thread from ready list */
    RTOS_listRemove(&pRunningThread->genericListItem);

    /* Added to the ready list */
    RTOS_threadAddToReadyList(pRunningThread);
  }
  else
  {
    /* Do nothing, no priority change required */
  }
}
#endif

/**
 * @}
 */
/**
 * @}
 */
/**
 * @}
 */
