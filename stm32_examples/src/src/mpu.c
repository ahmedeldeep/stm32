/*******************************************************************************
 * @file    mpu.c
 * @author  Ahmed Eldeep
 * @email   ahmed@almohandes.org
 * @website http://almohandes.org/stm32
 * @date    4 May 2019
 *          
 * @brief   MPU Example
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
#include "mpu.h"

/**
 * @addtogroup stm32_examples
 * @{
 */

/**
 * @defgroup mpu
 * @brief
 * @{
 */

/**
 * @defgroup mpu_private_typedefs
 * @{
 */

/**
 * @}
 */

/**
 * @defgroup mpu_private_defines
 * @{
 */

/**
 * @}
 */

/**
 * @defgroup mpu_private_macros
 * @{
 */

/**
 * @}
 */

/**
 * @defgroup mpu_private_constants
 * @{
 */

/**
 * @}
 */

/**
 * @defgroup mpu_private_variables
 * @{
 */

/**
 * @}
 */

/**
 * @defgroup mpu_private_function_prototypes
 * @{
 */

/**
 * @}
 */

/**
 * @defgroup mpu_private_functions
 * @{
 */

/**
 * @}
 */

/**
 * @defgroup mpu_exported_functions
 * @{
 */

/**
 * @brief   MPU initialization function
 * @note
 * @param   none
 * @retval  none
 */
void MPU_Init(void)
{
  /* Enable MemManage exception by setting the MEMFAULTENA bit
   * in the System Handler Control and State Register */
  SCB->SHCSR |= SCB_SHCSR_MEMFAULTENA_Msk;

  /* Check MPU exists */
  if(0 == MPU->TYPE)
  {
    /* MPU is not present */
    while(1)
    {
    }
  }
  else
  {
    /* MPU is OK */
  }

  /* Ensures memory operations order */
  __DMB();

  /* Disable MPU */
  MPU->CTRL = 0;

  /* ********************************************************* */
  /* Memory region #0 - Flash memory @ 0x08000000 - Size = 16K */
  /* Normal memory, Cacheable, Non-shareable, write-through    */
  /* Permissions: Privileged (RW)  Unprivileged (RW)           */
  /* ********************************************************* */

  /* Select region number */
  MPU->RNR = 0;

  /* Select region base address */
  MPU->RBAR = 0x08000000;

  /* Select region attribute and size */
  MPU->RASR = ((0x0D << MPU_RASR_SIZE_Pos) | MPU_RASR_C_Msk
      | (0x3 << MPU_RASR_AP_Pos) | MPU_RASR_ENABLE_Msk);

  /* ********************************************************* */
  /* Memory region #1 - SRAM @ 0x20000000 - Size = 256K        */
  /* Normal memory, Cacheable, Shareable, write-through        */
  /* Permissions: Privileged (RW)  Unprivileged (RW)           */
  /* ********************************************************* */

  /* Select region number and region base address */
  MPU->RBAR = (0x20000000 | MPU_RBAR_VALID_Msk | 1 );

  /* Select region attribute and size */
  MPU->RASR = ((0x11 << MPU_RASR_SIZE_Pos) | MPU_RASR_C_Msk | MPU_RASR_S_Msk
      | (0x3 << MPU_RASR_AP_Pos) | MPU_RASR_ENABLE_Msk);

  /* ********************************************************* */
  /* Memory region #2 - GPIOG_BASE - Size = 1K                 */
  /* Device memory, Bufferable, Shareable                      */
  /* Permissions: Privileged (RW)  Unprivileged (RW)           */
  /* ********************************************************* */

  /* Select region number and region base address */
  MPU->RBAR = (GPIOG_BASE | MPU_RBAR_VALID_Msk | 2 );

  /* Select region attribute and size */
  MPU->RASR = ((0x9 << MPU_RASR_SIZE_Pos) | MPU_RASR_B_Msk | MPU_RASR_S_Msk
      | (0x3 << MPU_RASR_AP_Pos) | MPU_RASR_ENABLE_Msk);

  /* Disabled unused regions */
  for(uint32_t idx = 3; idx < 8; idx++)
  {
    MPU->RNR = idx;
    MPU->RBAR = 0;
    MPU->RASR = 0;
  }

  /* Enable MPU */
  MPU->CTRL = MPU_CTRL_ENABLE_Msk;

  /* Ensures all explicit memory accesses before is complete */
  __DSB();

  /* Flushes the pipeline in the processor */
  __ISB();

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
