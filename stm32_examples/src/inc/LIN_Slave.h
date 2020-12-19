/*******************************************************************************
 * @file    LIN_Slave.h
 * @author  Ahmed Eldeep
 * @email   ahmed@almohandes.org
 * @website http://almohandes.org/stm32
 * @date    17.06.2018
 *
 * @brief   LIN slave node driver
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
#ifndef __INC_LIN_SLAVE_H_
#define __INC_LIN_SLAVE_H_

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
 * @addtogroup LIN_Slave
 * @{
 */

/**
 * @defgroup LIN_Slave_exported_typedefs
 * @{
 */

/**
 * @}
 */

/**
 * @defgroup LIN_Slave_exported_defines
 * @{
 */

/**
 * @}
 */

/**
 * @defgroup LIN_Slave_exported_macros
 * @{
 */

/**
 * @}
 */

/**
 * @defgroup LIN_Slave_exported_constants
 * @{
 */

/**
 * @}
 */

/**
 * @defgroup LIN_Slave_exported_functions
 * @{
 */

void LIN_Slave_GPIO_Init(void);
void LIN_Slave_UART5_Init(void);
void LIN_Slave_TX_DMA_Init(void);
void LIN_Slave_RX_DMA_Init(void);
void LIN_Slave_Enable(void);
void LIN_Slave_RX_DMA_IRQ_Callback(void);
void LIN_Slave_TX_DMA_IRQ_Callback(void);
void LIN_Slave_UART5_IRQ_Callback(void);
void LIN_Slave_Process(void);


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

#endif /*__INC_LIN_SLAVE_H_ */
