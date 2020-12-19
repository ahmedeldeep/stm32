/*******************************************************************************
 * @file    rcc.h
 * @author  Ahmed Eldeep
 * @email   ahmed@almohandes.org
 * @website http://almohandes.org/stm32
 * @date    06.07.2018
 *
 * @brief   RCC Examples
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

/* Define to prevent recursive inclusion */
#ifndef __INC_RCC_H_
#define __INC_RCC_H_

/* C++ detection */
#ifdef __cplusplus
extern "C" {
#endif

/* Includes */
#include "stm32f4xx.h"

/**
 * @addtogroup stm32_examples
 * @{
 */

/**
 * @addtogroup rcc
 * @{
 */

/**
 * @defgroup rcc_exported_typedefs
 * @{
 */

/**
 * @}
 */

/**
 * @defgroup rcc_exported_defines
 * @{
 */

/**
 * @}
 */

/**
 * @defgroup rcc_exported_macros
 * @{
 */

/**
 * @}
 */

/**
 * @defgroup rcc_exported_constants
 * @{
 */

/**
 * @}
 */

/**
 * @defgroup rcc_exported_functions
 * @{
 */
void RCC_SystemReset(void);
void RCC_DetectResetSource(void);
void RCC_ProcessReset(void);
void RCC_ClockOut_Setup(void);

/**
 * @}
 */
/**
 * @}
 */
/**
 * @}
 */

/* C++ detection */
#ifdef __cplusplus
}
#endif

#endif /*__INC_RCC_H_ */
