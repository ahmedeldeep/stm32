/*******************************************************************************
 * @file    lpwr.c
 * @author  Ahmed Eldeep
 * @email   ahmed@almohandes.org
 * @website http://almohandes.org/stm32
 * @date    18.10.2018
 *          
 * @brief   Low power example
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
#include "lpwr.h"
#include "gpio.h"

/**
 * @addtogroup stm32_examples
 * @{
 */

/**
 * @defgroup lpwr
 * @brief
 * @{
 */

/**
 * @defgroup lpwr_private_typedefs
 * @{
 */

/**
 * @brief   LPWR states definition
 */
typedef enum
{
  LPWR_IDLE,
  LPWR_SLEEP_MODE_WFI_REQ,
  LPWR_SLEEP_MODE_WFI_WAKEUP,
  LPWR_SLEEP_MODE_WFE_REQ,
  LPWR_SLEEP_MODE_WFE_WAKEUP,
  LPWR_STOP_MODE_REQ,
  LPWR_STOP_MODE_WAKEUP,
  LPWR_STANDBY_MODE_REQ
} LPWR_StateType;

/**
 * @brief   LPWR PB Status
 */
typedef enum
{
  LPWR_PB_PRESSED,
  LPWR_PB_NOT_PRESSED
} LPWR_PB_StatusType;

/**
 * @}
 */

/**
 * @defgroup lpwr_private_defines
 * @{
 */

/**
 * @}
 */

/**
 * @defgroup lpwr_private_macros
 * @{
 */

/**
 * @}
 */

/**
 * @defgroup lpwr_private_constants
 * @{
 */

/**
 * @}
 */

/**
 * @defgroup lpwr_private_variables
 * @{
 */

/**
 * @brief   LPWR current state
 */
static LPWR_StateType currentState = LPWR_IDLE;

/**
 * @brief   LPWR current status
 */
static LPWR_PB_StatusType PBCurrentStatus = LPWR_PB_NOT_PRESSED;

/**
 * @}
 */

/**
 * @defgroup lpwr_private_function_prototypes
 * @{
 */

/**
 * @}
 */

/**
 * @defgroup lpwr_private_functions
 * @{
 */

/**
 * @}
 */

/**
 * @defgroup lpwr_exported_functions
 * @{
 */

/**
 * @brief   LPWR_Main
 * @note
 * @param
 * @retval
 */
void LPWR_Main(void)
{
  /* Check if the PB was pressed */
  if(LPWR_PB_PRESSED == PBCurrentStatus )
  {
    /* Check current LPWR state */
    switch (currentState)
    {
      case LPWR_IDLE:
        /* Change current state */
        currentState = LPWR_SLEEP_MODE_WFI_REQ;
        break;

      case LPWR_SLEEP_MODE_WFI_REQ:
        /* LEDs OFF */
        GPIO_TurnOFF_LED(EVAL_ALL_LEDs);

        /* Green LED ON */
        GPIO_TurnON_LED(EVAL_GREEN_LED);

        /* Delay */
        for (int delay = 0; delay < 20000000; ++delay);

        /* Green LED OFF */
        GPIO_TurnOFF_LED(EVAL_GREEN_LED);

        /* Clear status flag */
        PBCurrentStatus = LPWR_PB_NOT_PRESSED;

        /* Change current state */
        currentState = LPWR_SLEEP_MODE_WFI_WAKEUP;

        /* Enter sleep mode */
        __WFI();
        break;

      case LPWR_SLEEP_MODE_WFI_WAKEUP:
        /* Clear status flag */
        PBCurrentStatus = LPWR_PB_NOT_PRESSED;

        /* Change current state */
        currentState = LPWR_SLEEP_MODE_WFE_REQ;
        break;

      case LPWR_SLEEP_MODE_WFE_REQ:
        /* LEDs OFF */
        GPIO_TurnOFF_LED(EVAL_ALL_LEDs);

        /* Red LED ON */
        GPIO_TurnON_LED(EVAL_RED_LED);

        /* Delay */
        for (int delay = 0; delay < 20000000; ++delay);

        /* Red LED OFF */
        GPIO_TurnOFF_LED(EVAL_RED_LED);

        /* Clear status flag */
        PBCurrentStatus = LPWR_PB_NOT_PRESSED;

        /* Change current state */
        currentState = LPWR_SLEEP_MODE_WFE_WAKEUP;

        /* Enter sleep mode */
        __WFE();
        __WFE();
        break;

      case LPWR_SLEEP_MODE_WFE_WAKEUP:
        /* Clear status flag */
        PBCurrentStatus = LPWR_PB_NOT_PRESSED;

        /* Change current state */
        currentState = LPWR_STOP_MODE_REQ;
        break;

      case LPWR_STOP_MODE_REQ:
        /* ALL LEDs ON */
        GPIO_TurnON_LED(EVAL_ALL_LEDs);

        /* Delay */
        for (int delay = 0; delay < 20000000; ++delay);

        /* ALL LEDs OFF */
        GPIO_TurnOFF_LED(EVAL_ALL_LEDs);

        /* Flash memory in power-down during Stop mode */
        PWR->CR |= PWR_CR_FPDS;

        /* Low-power voltage regulator ON during Stop mode */
        PWR->CR |= PWR_CR_LPDS;

        /* Low-power regulator in under-drive mode */
        PWR->CR |= PWR_CR_LPUDS;

        /* Set SLEEPDEEP bit */
        SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;

        /* Clear status flag */
        PBCurrentStatus = LPWR_PB_NOT_PRESSED;

        /* Change current state */
        currentState = LPWR_STOP_MODE_WAKEUP;

        /* Enter stop mode */
        __WFI();
        break;

      case LPWR_STOP_MODE_WAKEUP:
        /* Reset SLEEPDEEP bit of Cortex System Control Register */
        SCB->SCR &= (uint32_t)~((uint32_t)SCB_SCR_SLEEPDEEP_Msk);

        /* Enable HSE */
        RCC->CR |= ((uint32_t)RCC_CR_HSEON);

        /* Wait until HSE is ready */
        while(RCC_CR_HSERDY == (RCC_CR_HSERDY & RCC->CR))
        {
          /* Do nothing until HSE is ready */
        }

        /* Enable the main PLL */
        RCC->CR |= RCC_CR_PLLON;

        /* Wait till the main PLL is ready */
        while(RCC_CR_PLLRDY == (RCC_CR_PLLRDY & RCC->CR))
        {
          /* Do nothing until main PLL is ready */
        }

        /* Select the main PLL as system clock source */
        RCC->CFGR &= ~RCC_CFGR_SW;
        RCC->CFGR |= RCC_CFGR_SW_PLL;

        /* Wait till the main PLL is used as system clock source */
        while (RCC_CFGR_SWS_PLL != (RCC->CFGR & RCC_CFGR_SWS))
        {
          /* Do nothing until main PLL is used as system clock */
        }

        /* Clear status flag */
        PBCurrentStatus = LPWR_PB_NOT_PRESSED;

        /* Change current state */
        currentState = LPWR_STANDBY_MODE_REQ;
        break;

      case LPWR_STANDBY_MODE_REQ:
        /* ALL LEDs ON */
        GPIO_TurnON_LED(EVAL_ALL_LEDs);

        /* Delay */
        for (int delay = 0; delay < 20000000; ++delay);

        /* ALL LEDs will be OFF during standby mode */

        /* Clear Wakeup flag */
        PWR->CR |= PWR_CR_CWUF;

        /* Clear standby flag */
        PWR->CR |= PWR_CR_CSBF;

        /* Enable the wakeup pin */
        PWR->CSR |= PWR_CSR_EWUP;

        /* Select STANDBY mode */
        PWR->CR |= PWR_CR_PDDS;

        /* Set SLEEPDEEP bit */
        SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;

        /* Enter stop mode */
        __WFI();
        break;

      default:
        break;
    }
  }
  else
  {
    /* Do nothing */
  }
}

/**
 * @brief   Callback function
 * @note    Called when PB is pressed
 * @param   None
 * @retval  None
 */
void LPWR_PB_IRQ_Callback(void)
{
  /* Clear pending bit */
  EXTI->PR |= EXTI_PR_PR0;

  /* Set current status flag */
  PBCurrentStatus = LPWR_PB_PRESSED;
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
