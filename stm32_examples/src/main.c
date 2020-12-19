/*******************************************************************************
 * @file    main.c
 * @author  Ahmed Eldeep
 * @email   ahmed@almohandes.org
 * @website http://almohandes.org/stm32
 * @date    21.03.2018
 *          
 * @brief   main application called after startup
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

/* Includes */
#include "stm32f4xx.h"
#include "nvic.h"
#include "SysTick.h"
#include "itm.h"

/**
 * @addtogroup stm32_examples
 * @{
 */

/**
 * @defgroup main
 * @brief
 * @{
 */

/**
 * @defgroup main_private_typedefs
 * @{
 */

/**
 * @}
 */

/**
 * @defgroup main_private_defines
 * @{
 */

/**
 * @}
 */

/**
 * @defgroup main_private_macros
 * @{
 */

/**
 * @}
 */

/**
 * @defgroup main_private_constants
 * @{
 */
/**
 * @brief   Sine wave
 */
const static uint16_t sineWave[256] = {
  0x8000, 0x8327, 0x864e, 0x8973, 0x8c98, 0x8fba, 0x92da, 0x95f7,
  0x9911, 0x9c27, 0x9f38, 0xa244, 0xa54c, 0xa84d, 0xab48, 0xae3c,
  0xb12a, 0xb40f, 0xb6ed, 0xb9c2, 0xbc8e, 0xbf50, 0xc209, 0xc4b7,
  0xc75b, 0xc9f4, 0xcc81, 0xcf02, 0xd177, 0xd3e0, 0xd63b, 0xd889,
  0xdac9, 0xdcfb, 0xdf1e, 0xe133, 0xe339, 0xe52f, 0xe716, 0xe8ec,
  0xeab3, 0xec68, 0xee0d, 0xefa1, 0xf123, 0xf294, 0xf3f3, 0xf540,
  0xf67a, 0xf7a3, 0xf8b8, 0xf9bb, 0xfaab, 0xfb88, 0xfc52, 0xfd08,
  0xfdab, 0xfe3b, 0xfeb7, 0xff1f, 0xff73, 0xffb4, 0xffe1, 0xfff9,
  0xfffe, 0xffef, 0xffcd, 0xff96, 0xff4b, 0xfeed, 0xfe7b, 0xfdf6,
  0xfd5c, 0xfcb0, 0xfbef, 0xfb1c, 0xfa36, 0xf93c, 0xf830, 0xf711,
  0xf5df, 0xf49b, 0xf345, 0xf1de, 0xf064, 0xeed9, 0xed3d, 0xeb8f,
  0xe9d2, 0xe803, 0xe625, 0xe436, 0xe238, 0xe02b, 0xde0f, 0xdbe4,
  0xd9ab, 0xd763, 0xd50f, 0xd2ad, 0xd03e, 0xcdc3, 0xcb3c, 0xc8a9,
  0xc60b, 0xc361, 0xc0ae, 0xbdf0, 0xbb29, 0xb858, 0xb57f, 0xb29d,
  0xafb4, 0xacc3, 0xa9cb, 0xa6cd, 0xa3c9, 0xa0bf, 0x9db0, 0x9a9c,
  0x9785, 0x9469, 0x914b, 0x8e29, 0x8b06, 0x87e1, 0x84ba, 0x8193,
  0x7e6c, 0x7b45, 0x781e, 0x74f9, 0x71d6, 0x6eb4, 0x6b96, 0x687a,
  0x6563, 0x624f, 0x5f40, 0x5c36, 0x5932, 0x5634, 0x533c, 0x504b,
  0x4d62, 0x4a80, 0x47a7, 0x44d6, 0x420f, 0x3f51, 0x3c9e, 0x39f4,
  0x3756, 0x34c3, 0x323c, 0x2fc1, 0x2d52, 0x2af0, 0x289c, 0x2654,
  0x241b, 0x21f0, 0x1fd4, 0x1dc7, 0x1bc9, 0x19da, 0x17fc, 0x162d,
  0x1470, 0x12c2, 0x1126, 0x0f9b, 0x0e21, 0x0cba, 0x0b64, 0x0a20,
  0x08ee, 0x07cf, 0x06c3, 0x05c9, 0x04e3, 0x0410, 0x034f, 0x02a3,
  0x0209, 0x0184, 0x0112, 0x00b4, 0x0069, 0x0032, 0x0010, 0x0001,
  0x0006, 0x001e, 0x004b, 0x008c, 0x00e0, 0x0148, 0x01c4, 0x0254,
  0x02f7, 0x03ad, 0x0477, 0x0554, 0x0644, 0x0747, 0x085c, 0x0985,
  0x0abf, 0x0c0c, 0x0d6b, 0x0edc, 0x105e, 0x11f2, 0x1397, 0x0154c,
  0x1713, 0x18e9, 0x1ad0, 0x1cc6, 0x1ecc, 0x20e1, 0x2304, 0x2536,
  0x2776, 0x29c4, 0x2c1f, 0x2e88, 0x30fd, 0x337e, 0x360b, 0x38a4,
  0x3b48, 0x3df6, 0x40af, 0x4371, 0x463d, 0x4912, 0x4bf0, 0x4ed5,
  0x51c3, 0x54b7, 0x57b2, 0x5ab3, 0x5dbb, 0x60c7, 0x63d8, 0x66ee,
  0x6a08, 0x6d25, 0x7045, 0x7367, 0x768c, 0x79b1, 0x7cd8, 0x8000};

/**
 * @}
 */

/**
 * @defgroup main_private_variables
 * @{
 */
/**
 * @brief   Sine wave variable
 */
static uint16_t sineWaveVar = 0;

/**
 * @}
 */

/**
 * @defgroup main_private_function_prototypes
 * @{
 */

/**
 * @}
 */

/**
 * @defgroup main_private_functions
 * @{
 */

/**
 * @}
 */

/**
 * @defgroup main_exported_functions
 * @{
 */

/**
 * @brief   Main function
 * @note
 * @param   none
 * @retval  none
 */
int main(void)
{
  char hello_world_0[] = "Hello World! to Port 0\n";
  char hello_world_1[] = "Hello World! to Port 1\n";

  SysTick_Init();
  NVIC_Init();

  /* Clear PRIMASK, enable IRQs */
  __enable_irq();


  /* Infinite loop */
  while(1)
  {
    /* Send string to ITM port 0 */
    ITM_Printf(hello_world_0, sizeof(hello_world_0));

    /* Send string to ITM port 1 */
    ITM_Printf_Port(1, hello_world_1, sizeof(hello_world_1));

    /* Send 10 to ITM port 2 */
    ITM_SendChar_Port(2, 10);

    /* Waiting time in milliseconds */
    SysTick_Delay(500);

    /* Send 20 to ITM port 2 */
    ITM_SendChar_Port(2, 20);

    /* Sine wave output */
    for(int idx = 0; idx < 256; idx++)
    {
      /* Write the sine wave variable for plotting */
      sineWaveVar = sineWave[idx];

      /* Waiting time in milliseconds */
      SysTick_Delay(100);
    }
  }
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
