/*******************************************************************************
 * @file    flash.h
 * @author  Ahmed Eldeep
 * @email   ahmed@almohandes.org
 * @website http://almohandes.org/stm32
 * @date    25.03.2019
 *
 * @brief   Flash Examples
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
#ifndef __INC_FLASH_H_
#define __INC_FLASH_H_

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
 * @addtogroup flash
 * @{
 */

/**
 * @defgroup flash_exported_typedefs
 * @{
 */

/**
 * @}
 */

/**
 * @defgroup flash_exported_defines
 * @{
 */

/**
 * @}
 */

/**
 * @defgroup flash_exported_macros
 * @{
 */

/**
 * @}
 */

/**
 * @defgroup flash_exported_constants
 * @{
 */

/**
 * @}
 */

/**
 * @defgroup flash_exported_functions
 * @{
 */
void FLASH_USART1_GPIO_Config(void);
void FLASH_USART1_TX_DMA_Config(void);
void FLASH_USART1_RX_DMA_Config(void);
void FLASH_USART1_Init(void);
void FLASH_USART1_Enable(void);
void FLASH_USART1_IRQ_Callback(void);
void FLASH_USART1_TX_DMA_IRQ_Callback(void);
void FLASH_USART1_RX_DMA_IRQ_Callback(void);
void FLASH_Init(void);
void FLASH_Main(void);

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

#endif /*__INC_FLASH_H_ */
