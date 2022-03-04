/*
 * WNMEA - Warcomeb NMEA 0183 Parsing Library
 * Copyright (C) 2022 Marco Giammarini <http://www.warcomeb.it>
 *
 * Authors:
 *  Marco Giammarini <m.giammarini@warcomeb.it>
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

/*!
 * \file  /wnmea.h
 * \brief
 */

#ifndef __WARCOMEB_WNMEA_H
#define __WARCOMEB_WNMEA_H

#ifdef __cplusplus
extern "C"
{
#endif

/*!
 * \mainpage WNMEA - Warcomeb NMEA 0183 Parsing Library
 *
 * This library is developed in order to parse NMEA0183 string produced by GPS sensor and it use
 * libohiboard as low level driver framework.
 * This library is an evolutions of \link https://github.com/warcomeb/gpsnmea-embedded \endlink.
 *
 * \section changelog ChangeLog
 *
 * \li v1.0.0 of 2022/03/04 - First release
 *
 * \section library External Library
 *
 * The library use the following external library:
 * \li libohiboard https://github.com/ohilab/libohiboard a multiplatform C
 * framework for multi microcontroller.
 *
 * \section example Example
 *
 * \code{.c}
 * void cb (WNMEA_MessageParsed_t msg, WNMEA_MessageType_t type)
 * {
 *     if (type == WNMEA_MESSAGETYPE_RMC)
 *     {
 *         Uart_sendStringln(OB_UART2,"RMC");
 *     }
 *     else if (type == WNMEA_MESSAGETYPE_GGA)
 *     {
 *         Uart_sendStringln(OB_UART2,"GGA");
 *     }
 * }
 *
 * void main (void)
 * {
 *     Clock_Config clkConfig =
 *     {
 *         .source = CLOCK_INTERNAL_HSI | CLOCK_EXTERNAL_LSE_CRYSTAL,
 *
 *         .sysSource = CLOCK_SYSTEMSOURCE_HSI,
 *
 *         .hsiState = CLOCK_OSCILLATORSTATE_ON,
 *         .lsiState = CLOCK_OSCILLATORSTATE_OFF,
 *         .lseState = CLOCK_OSCILLATORSTATE_ON,
 *
 *         .output = CLOCK_OUTPUT_SYSCLK | CLOCK_OUTPUT_HCLK | CLOCK_OUTPUT_PCLK1 | CLOCK_OUTPUT_PCLK2,
 *         .ahbDivider = CLOCK_AHBDIVIDER_1,
 *         .apb1Divider = CLOCK_APBDIVIDER_1,
 *         .apb2Divider = CLOCK_APBDIVIDER_1,
 *     };
 *
 *     // Uart1 - GPS
 *     Uart_Config uart1Config =
 *     {
 *         .clockSource = UART_CLOCKSOURCE_SYSCLK,
 *         .mode = UART_MODE_BOTH,
 *
 *         .rxPin = UART_PINS_PA10,
 *         .txPin = UART_PINS_PA9,
 *
 *         .baudrate = 9600,
 *
 *         .dataBits = UART_DATABITS_EIGHT,
 *         .flowControl = UART_FLOWCONTROL_NONE,
 *         .parity = UART_PARITY_NONE,
 *         .stop = UART_STOPBITS_ONE,
 *     };
 *
 *     // Uart2 - Output
 *     Uart_Config uartOutConfig =
 *     {
 *         .clockSource = UART_CLOCKSOURCE_SYSCLK,
 *         .mode = UART_MODE_BOTH,
 *
 *         .rxPin = UART_PINS_PA3,
 *         .txPin = UART_PINS_PA2,
 *
 *         .baudrate = 115200,
 *
 *         .dataBits = UART_DATABITS_EIGHT,
 *         .flowControl = UART_FLOWCONTROL_NONE,
 *         .parity = UART_PARITY_NONE,
 *         .stop = UART_STOPBITS_ONE,
 *     };
 *
 *     // Initialize clock...
 *     Clock_init(&clkConfig);
 *
 *     Uart_init(OB_UART1, &uart1Config);
 *     Uart_init(OB_UART2, &uartOutConfig);
 *
 *     WNMEA_MessageCallback_t mycb =
 *     {
 *         .rmc = cb,
 *         .gga = cb,
 *     };
 *
 *     WNMEA_init(OB_UART1,mycb);
 *
 *     while (1)
 *     {
 *         WNMEA_ckeck();
 *     }
 * }
 *
 * \endcode
 *
 * \section credits Credits
 * \li Marco Giammarini (warcomeb)
 *
 * \section license License
 *
 * \code{.unparsed}
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 * \endcode
 */

/*!
 * \defgroup WNMEA WNMEA Module APIs
 * \{
 */

#include "wnmea-types.h"

#include <stdarg.h>
#include <stdio.h>

/*!
 * \defgroup WNMEA_Init WNMEA Initialization APIs
 * \{
 */

/*!
 * This function initialize all WNMEA library.
 * \note The device handle must be just configured!
 *
 * \param[in] dev: The peripheral device handle to use.
 * \param[in] cb: List of used callback
 */
void WNMEA_init (Uart_DeviceHandle dev, WNMEA_MessageCallback_t cb);

/*!
 * \}
 */

/*!
 * \defgroup WNMEA_Command WNMEA Command APIs
 * \{
 */

/*!
 * This function must be called cyclically to check and manage the message.
 */
void WNMEA_ckeck (void);

/*!
 * \}
 */

/*!
 * \}
 */

#ifdef __cplusplus
}
#endif

#endif // __WARCOMEB_WNMEA_H
