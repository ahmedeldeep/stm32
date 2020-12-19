/*******************************************************************************
 * @file    can.h
 * @author  Ahmed Eldeep
 * @email   ahmed@almohandes.org
 * @website http://almohandes.org/stm32
 * @date    30 Oct 2019
 *
 * @brief   CAN Example
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
#ifndef __INC_CAN_H_
#define __INC_CAN_H_

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
 * @addtogroup can
 * @{
 */

/**
 * @defgroup can_exported_typedefs
 * @{
 */

/**
 * @}
 */

/**
 * @defgroup can_exported_defines
 * @{
 */

/**
 * @}
 */

/**
 * @defgroup can_exported_macros
 * @{
 */

/**
 * @}
 */

/**
 * @defgroup can_exported_constants
 * @{
 */

/**
 * @}
 */

/**
 * @defgroup can_exported_functions
 * @{
 */

void CAN1_Init(void);
void CAN1_Transmit(void);
void CAN1_Receive(void);

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

#endif /*__INC_CAN_H_ */
