/*******************************************************************************
 * @file    L3GD20.h
 * @author  Ahmed Eldeep
 * @email   ahmed@almohandes.org
 * @website http://almohandes.org/stm32
 * @date    30.10.2018
 *
 * @brief   Interfacing gyroscope sensor L3GD20 using SPI
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
#ifndef __INC_L3GD20_H_
#define __INC_L3GD20_H_

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
 * @addtogroup L3GD20
 * @{
 */

/**
 * @defgroup L3GD20_exported_typedefs
 * @{
 */

/**
 * @}
 */

/**
 * @defgroup L3GD20_exported_defines
 * @{
 */

/**
 * @}
 */

/**
 * @defgroup L3GD20_exported_macros
 * @{
 */

/**
 * @}
 */

/**
 * @defgroup L3GD20_exported_constants
 * @{
 */

/**
 * @}
 */

/**
 * @defgroup L3GD20_exported_functions
 * @{
 */

/**
 * @brief   L3GD20 Init function
 * @note
 * @param   None
 * @retval  None
 */
void L3GD20_Init(void);

/**
 * @brief   L3GD20 main function
 * @note
 * @param   None
 * @retval  None
 */
void L3GD20_Main(void);

/**
 * @brief   Callback function
 * @note    Called when MEMS_INT1
 * @param   None
 * @retval  None
 */
void EXTI1_MEMS_INT1_Callback();

/**
 * @brief   Callback function
 * @note    Called when MEMS_INT2
 * @param   None
 * @retval  None
 */
void EXTI2_MEMS_INT2_Callback();

/**
 * @brief   IRQ callback function
 * @note    SPI5_TX -> DMA2_Stream4 (Channel 2)
 * @param   None
 * @retval  None
 */
void SPI5_TX_DMA_IRQ_Callback(void);

/**
 * @brief   IRQ callback function
 * @note    SPI5_RX -> DMA2_Stream3 (Channel 2)
 * @param   None
 * @retval  None
 */
void SPI5_RX_DMA_IRQ_Callback(void);

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

#endif /*__INC_L3GD20_H_ */
