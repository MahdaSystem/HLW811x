/**
 **********************************************************************************
 * @file   HLW811x_platform.h
 * @author Hossein.M (https://github.com/Hossein-M98)
 * @brief  A sample Platform dependent layer for HLW811x Driver
 **********************************************************************************
 *
 * Copyright (c) 2024 Mahda Embedded System (MIT License)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 **********************************************************************************
 */
  
/* Define to prevent recursive inclusion ----------------------------------------*/
#ifndef _HLW811X_PLATFORM_H_
#define _HLW811X_PLATFORM_H_

#ifdef __cplusplus
extern "C" {
#endif


/* Includes ---------------------------------------------------------------------*/
#include "HLW811x.h"
#include <stdint.h>


/* Functionality Options --------------------------------------------------------*/
#if (HLW811X_CONFIG_SUPPORT_SPI)
/**
 * @brief  Specify IO Pins of ESP32 connected to TM1638
 */
#define HLW811X_SPI_NUM         SPI2_HOST
#define HLW811X_SPI_RATE        1000000   // in Hz
#define HLW811X_SPI_CLK_GPIO    GPIO_NUM_14
#define HLW811X_SPI_MISO_GPIO   GPIO_NUM_36
#define HLW811X_SPI_MOSI_GPIO   GPIO_NUM_2
#define HLW811X_SPI_CSN_GPIO    GPIO_NUM_15
#endif

#if (HLW811X_CONFIG_SUPPORT_UART)
#define HLW811X_UART_NUM        UART_NUM_1
#define HLW811X_UART_BAUD       0  // 0: 9600, 1: 19200, 2: 38400, other: invalid 
#define HLW811X_UART_TX_GPIO    GPIO_NUM_2
#define HLW811X_UART_RX_GPIO    GPIO_NUM_36
#endif



/**
 ==================================================================================
                               ##### Functions #####                               
 ==================================================================================
 */

#if (HLW811X_CONFIG_SUPPORT_SPI)
/**
 * @brief  Initialize platform device to communicate HLW811x through SPI.
 * @param  Handler: Pointer to handler
 * @retval None
 */
void
HLW811x_Platform_Init_SPI(HLW811x_Handler_t *Handler);
#endif

#if (HLW811X_CONFIG_SUPPORT_UART)
/**
 * @brief  Initialize platform device to communicate HLW811x through UART.
 * @param  Handler: Pointer to handler
 * @retval None
 */
void
HLW811x_Platform_Init_UART(HLW811x_Handler_t *Handler);
#endif



#ifdef __cplusplus
}
#endif

#endif //! _HLW811X_PLATFORM_H_