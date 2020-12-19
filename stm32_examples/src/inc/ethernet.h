/*******************************************************************************
 * @file    ethernet.h
 * @author  Ahmed Eldeep
 * @email   ahmed@almohandes.org
 * @website http://almohandes.org/stm32
 * @date    17 Jan 2020
 *
 * @brief   Ethernet example
 * @note
 *
@verbatim
Copyright (C) Almohandes.org, 2020

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
#ifndef __INC_ETHERNET_H_
#define __INC_ETHERNET_H_

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
 * @addtogroup ethernet
 * @{
 */

/**
 * @defgroup ethernet_exported_typedefs
 * @{
 */

/**
 * @}
 */

/**
 * @defgroup ethernet_exported_defines
 * @{
 */

/**
 * @}
 */

/**
 * @defgroup ethernet_exported_macros
 * @{
 */

/**
 * @}
 */

/**
 * @defgroup ethernet_exported_constants
 * @{
 */

/**
 * @}
 */

/**
 * @defgroup ethernet_exported_functions
 * @{
 */

void ETH_GPIO_Init(void);
void ETH_MAC_Init(void);
void ETH_DMA_Init(void);
void ETH_DMA_TX_DescriptorList_Init(void);
void ETH_DMA_RX_DescriptorList_Init(void);

void ETH_Enable(void);
void ETH_Transmit(void);
void ETH_Receive(void);

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

#endif /*__INC_ETHERNET_H_ */
