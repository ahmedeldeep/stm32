/*******************************************************************************
 * @file    audio.h
 * @author  Ahmed Eldeep
 * @email   ahmed@almohandes.org
 * @website http://almohandes.org/stm32
 * @date    22.12.2018
 *
 * @brief   Audio recording and playing example
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
#ifndef __INC_AUDIO_H_
#define __INC_AUDIO_H_

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
 * @addtogroup audio
 * @{
 */

/**
 * @defgroup audio_exported_typedefs
 * @{
 */

/**
 * @}
 */

/**
 * @defgroup audio_exported_defines
 * @{
 */

/**
 * @}
 */

/**
 * @defgroup audio_exported_macros
 * @{
 */

/**
 * @}
 */

/**
 * @defgroup audio_exported_constants
 * @{
 */

/**
 * @}
 */

/**
 * @defgroup audio_exported_functions
 * @{
 */
void AUDIO_ADC_Config(void);
void AUDIO_DAC_Config(void);
void AUDIO_Timer_Config(void);
void AUDIO_Main(void);
void AUDIO_PB_Callback(void);
void AUDIO_DMA2_Stream0_Callback(void);
void AUDIO_DMA1_Stream6_Callback(void);

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

#endif /*__INC_AUDIO_H_ */
