/*******************************************************************************
 * @file    wdg.h
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

/* Define to prevent recursive inclusion */
#ifndef __INC_WDG_H_
#define __INC_WDG_H_

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
 * @addtogroup wdg
 * @{
 */

/**
 * @defgroup wdg_exported_typedefs
 * @{
 */

/**
 * @}
 */

/**
 * @defgroup wdg_exported_defines
 * @{
 */

/**
 * @}
 */

/**
 * @defgroup wdg_exported_macros
 * @{
 */

/**
 * @}
 */

/**
 * @defgroup wdg_exported_constants
 * @{
 */

/**
 * @}
 */

/**
 * @defgroup wdg_exported_functions
 * @{
 */
void IWDG_Init(void);
void IWDG_Refresh(void);
void IWDG_Start(void);
void WWDG_Init(void);
void WWDG_Refresh(void);
void WWDG_Start(void);

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

#endif /*__INC_WDG_H_ */
