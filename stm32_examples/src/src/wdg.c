/*******************************************************************************
 * @file    wdg.c
 * @author  Ahmed Eldeep
 * @email   ahmed@almohandes.org
 * @website http://almohandes.org/stm32
 * @date    9 Jul 2019
 *          
 * @brief   Watchdog configuration example
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
#include "SysTick.h"
#include "wdg.h"

/**
 * @addtogroup stm32_examples
 * @{
 */

/**
 * @defgroup wdg
 * @brief
 * @{
 */

/**
 * @defgroup wdg_private_typedefs
 * @{
 */

/**
 * @}
 */

/**
 * @defgroup wdg_private_defines
 * @{
 */

/**
 * @}
 */

/**
 * @defgroup wdg_private_macros
 * @{
 */

/**
 * @}
 */

/**
 * @defgroup wdg_private_constants
 * @{
 */

/**
 * @}
 */

/**
 * @defgroup wdg_private_variables
 * @{
 */

/**
 * @brief   Start Tick
 */
static uint32_t startTick = 0;

/**
 * @}
 */

/**
 * @defgroup wdg_private_function_prototypes
 * @{
 */

/**
 * @}
 */

/**
 * @defgroup wdg_private_functions
 * @{
 */

/**
 * @}
 */

/**
 * @defgroup wdg_exported_functions
 * @{
 */

/**
 * @brief   Independent watchdog initialization function
 * @note    IWDG clocked with    LSI RC 32 KHz with 1/32 prescaler
 *          so 1 millisecond count.
 * @param
 * @retval
 */
void IWDG_Init(void)
{
  /* Enable access to the IWDG_PR and IWDG_RLR registers */
  IWDG->KR = 0x5555;

  /* Wait until PVU bit is cleared */
  while(IWDG_SR_PVU == (IWDG_SR_PVU & IWDG->SR))
  {
    /* Do nothing until PVU bit is cleared */
  }

  /* Set Prescaler divider to /32 */
  IWDG->PR = (IWDG_PR_PR_0 | IWDG_PR_PR_1);

  /* Wait until RVU bit is cleared */
  while(IWDG_SR_RVU == (IWDG_SR_RVU & IWDG->SR))
  {
    /* Do nothing until RVU bit is cleared */
  }

  /* Select 10 counts at each reload */
  IWDG->RLR = 10;

  /* Stop the watchdog during debug halt */
  DBGMCU->APB1FZ |= DBGMCU_APB1_FZ_DBG_IWDG_STOP;
}

/**
 * @brief   Reloads the watchdog counter value.
 * @note
 * @param
 * @retval
 */
void IWDG_Refresh(void)
{
  /* Refresh watchdog */
  IWDG->KR = 0xAAAA;
}

/**
 * @brief   Starts the watchdog counter.
 * @note
 * @param
 * @retval
 */
void IWDG_Start(void)
{
  /* Watchdog start */
  IWDG->KR = 0xCCCC;
}

/**
 * @brief   Window watchdog initialization function
 * @note    APB1 clock = 45 MHz
 *          the timeout value from the equation;
 *          tWWDG = tPCLK1 × 4096 × 2 ^ WDGTB[1:0] × (T[5:0] + 1)
 *          WDGTB[1:0] = 3
 *          tWWDG Min = 0.728 ms at T[5:0] = 0
 *          tWWDG Max = 46.603 ms at T[5:0] = 0x3F
 *          so select 0xD and tWWDG will be = 10,1 ms ≈ 10
 *          |--------------|------|
 *          0x3F           0xD    0
 * T6 = 1   0x7F           0x4D   0x40
 *
 * @param   
 * @retval
 */
void WWDG_Init(void)
{
  /* Enable WWDG clock in RCC */
  RCC->APB1ENR |= RCC_APB1ENR_WWDGEN;

  /* Set window high limit and prescaler */
  WWDG->CFR = (0x4D | WWDG_CFR_WDGTB0 | WWDG_CFR_WDGTB1);

  /* Stop the watchdog during debug halt */
  DBGMCU->APB1FZ |= DBGMCU_APB1_FZ_DBG_WWDG_STOP;
}

/**
 * @brief   Reloads the watchdog counter value.
 * @note
 * @param
 * @retval
 */
void WWDG_Refresh(void)
{
  /* Current SysTick */
  uint32_t currentTick = 0;

  /* Get current tick */
  currentTick = SysTick_GetCurrentTick();

  /* Compare with timeout value (46.603 ms - 10,1 ms = 36,503 ms)*/
  if((currentTick - startTick) > (36))
  {
    /* Reload counter value */
    WWDG->CR = 0x7F;

    /* Update start tick */
    startTick = SysTick_GetCurrentTick();
  }
  else
  {
    /* Do nothing */
  }
}

/**
 * @brief   Starts the watchdog counter.
 * @note
 * @param
 * @retval
 */
void WWDG_Start(void)
{
  /* Write Counter value and activation bit */
  WWDG->CR = (0x7F | WWDG_CR_WDGA);

  /* Get start tick */
  startTick = SysTick_GetCurrentTick();
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
