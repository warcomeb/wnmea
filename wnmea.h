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
 * This library is developed in order to ...
 *
 * \section changelog ChangeLog
 *
 * \li v1.0.0 of 2022/02/xx - First release
 *
 * \section library External Library
 *
 * The library use the following external library:
 * \li libohiboard https://github.com/ohilab/libohiboard a multiplatform C
 * framework for multi microcontroller.
 *
 * \section example Example
 *
 * \section credits Credits
 * \li Marco Giammarini (warcomeb)
 */

/*!
 * \defgroup WNMEA WNMEA Module APIs
 * \{
 */

#include "wnmea-types.h"

#include <stdarg.h>
#include <stdio.h>

/*!
 *
 * \note The device handle must be just configured!
 *
 * \param[in] dev: The peripheral device handle to use.
 */
void WNMEA_init (Uart_DeviceHandle dev);

/*!
 * \defgroup WNMEA_Command WNMEA Command APIs
 * \{
 */

/*!
 *
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
