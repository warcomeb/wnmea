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


#define WNMEA_CONSTELLATION_GPS                  "GP"
#define WNMEA_CONSTELLATION_GLONASS              "GL"
#define WNMEA_CONSTELLATION_BOTH                 "GN"

#define WNMEA_MESSAGE_TYPE_GGA                   "GGA"
#define WNMEA_MESSAGE_TYPE_GSA                   "GSA"
#define WNMEA_MESSAGE_TYPE_GSV                   "GSV"
#define WNMEA_MESSAGE_TYPE_RMC                   "RMC"
#define WNMEA_MESSAGE_TYPE_VTG                   "VTG"
#define WNMEA_MESSAGE_TYPE_PMTK                  "PMTK"

#define WNMEA_CHAR_START                         '$'
#define WNMEA_CHAR_STOP                          '*'
#define WNMEA_CHAR_SEPARATOR                     ','
#define WNMEA_CHAR_DECIMAL                       '.'
#define WNMEA_CHAR_END                           "\r\n"
#define WNMEA_CHAR_END1                          '\r'
#define WNMEA_CHAR_END2                          '\n'


typedef enum _WNMEA_ParseState_t
{
    WNMEA_PARSESTATE_SOP,
    WNMEA_PARSESTATE_TYPE,
    WNMEA_PARSESTATE_DATA,
    WNMEA_PARSESTATE_CHECKSUM,
    WNMEA_PARSESTATE_EOP,

} WNMEA_ParseState_t;

static Uart_DeviceHandle mDevice = {0};

/*!
 * The buffer for the incoming command.
 */
static char mBuffer[WNMEA_BUFFER_DIMENSION+1] = {0};

/*!
 * The buffer descriptor useful to manage circular receive buffer.
 */
static UtilityBuffer_Descriptor mBufferDescriptor;

static WNMEA_ParseState_t mState = WNMEA_PARSESTATE_SOP;

static WNMEA_Error_t process (char c)
{
    if ((mState != WNMEA_PARSESTATE_SOP ) &&
       ((Utility_isPrintableChar(c) == false) && (Utility_isSpecialChar(c) == false)))
    {
        reset();
        return WNMEA_ERROR_WRONG_MESSAGE;
    }

    switch (mState)
    {
    case WNMEA_PARSESTATE_SOP:

        break;

    case WNMEA_PARSESTATE_TYPE:

        break;

    case WNMEA_PARSESTATE_DATA:

        break;

    case WNMEA_PARSESTATE_CHECKSUM:

        break;

    case WNMEA_PARSESTATE_EOP:

        break;
    }

    return WNMEA_ERROR_SUCCESS;
}

static void reset (void)
{
    mState = WNMEA_PARSESTATE_SOP;
}

void callbackRx (struct _Uart_Device* dev, void* obj)
{
    (void)obj;

    uint8_t c = 0;
    Uart_read(dev,&c,100);

    UtilityBuffer_push(&mBufferDescriptor,c);
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
    UtilityBuffer_init(&mBufferDescriptor,(uint8_t*)mBuffer,WNMEA_BUFFER_DIMENSION+1);
}

void WNMEA_ckeck (void)
{
    char c = '\0';
    WNMEA_Error_t err = WNMEA_ERROR_SUCCESS;

    while (!UtilityBuffer_isEmpty(&mBufferDescriptor))
    {
        UtilityBuffer_pull(&mBufferDescriptor,(uint8_t*)&c);

        err = process(c);

        if (err == WNMEA_ERROR_MESSAGE_READY)
        {

        }
    }
}
