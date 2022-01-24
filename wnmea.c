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

#include "wnmea.h"

#include <stdlib.h>

#if !defined (LIBOHIBOARD_UART)
#error "WNMEA: You must enable UART peripheral."
#endif

static Uart_DeviceHandle mDevice = {0};

void callbackRx (struct _Uart_Device* dev, void* obj)
{
    (void)obj;

    uint8_t c = 0;
    Uart_read(dev,&c,100);

//    UtilityBuffer_push(&mBufferDescriptor,c);
}

void WNMEA_init (Uart_DeviceHandle dev)
{
    if (dev == null)
    {
        ohiassert(0);
        return;
    }
    // Save device handle
    mDevice = dev;
    Uart_addRxCallback(mDevice,callbackRx);

    // Initialize buffer descriptor
    //UtilityBuffer_init(&mBufferDescriptor, mBuffer, WCDLI_BUFFER_DIMENSION+1);
}

void WNMEA_ckeck (void)
{
    char c = '\0';
}
