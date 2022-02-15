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


#define WNMEA_CONSTELLATION_STRING_GPS           "GP"
#define WNMEA_CONSTELLATION_STRING_GLONASS       "GL"
#define WNMEA_CONSTELLATION_STRING_GALILEO       "GA"
#define WNMEA_CONSTELLATION_STRING_BEIDOU        "GB" // China
#define WNMEA_CONSTELLATION_STRING_NAVIC         "GI" // India
#define WNMEA_CONSTELLATION_STRING_MULTIPLE      "GN"

#define WNMEA_MESSAGE_TYPE_STRING_GGA            "GGA"
#define WNMEA_MESSAGE_TYPE_STRING_GSA            "GSA"
#define WNMEA_MESSAGE_TYPE_STRING_GSV            "GSV"
#define WNMEA_MESSAGE_TYPE_STRING_GLL            "GLL"
#define WNMEA_MESSAGE_TYPE_STRING_RMC            "RMC"
#define WNMEA_MESSAGE_TYPE_STRING_VTG            "VTG"
#define WNMEA_MESSAGE_TYPE_STRING_ZDA            "ZDA"
#define WNMEA_MESSAGE_TYPE_STRING_PMTK           "PMTK"

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

static uint16_t mChecksum = 0;

static WNMEA_Message_t mMessage = {0};
static uint8_t         mMessagePoistion = 0;

static WNMEA_MessageParsed_t mMessageParsed = {0};

/*!
 * This function is used to reset the message process state machine.
 */
static void reset (void)
{
    mState = WNMEA_PARSESTATE_SOP;
    mChecksum = 0;

    mMessagePoistion = 0;
}

/*!
 * This function process the new char extracted from the queue.
 * Check whether the new char is coherent with the previous once,
 * otherwise reset all and start again the message process.
 *
 * \return The process state.
 */
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
        if (c == WNMEA_CHAR_START)
        {
            // Reset checksum variable
            mChecksum = 0;
            // Clear message variables
            memset(&mMessage,0,sizeof(WNMEA_Message_t));
            mMessagePoistion = 0;
            // Set next state
            mState = WNMEA_PARSESTATE_TYPE;
            // TODO: Add timeout?
            // Return without errors...
            return WNMEA_ERROR_MESSAGE_PARSING;
        }
        break;

    case WNMEA_PARSESTATE_TYPE:

        // Check the first separator...
        if (c == WNMEA_CHAR_SEPARATOR)
        {
            // Jump to the next state...
            mMessagePoistion = 0;
            mState = WNMEA_PARSESTATE_DATA;

            // Compute checksum
            mChecksum ^= c;

            // Return without errors...
            return WNMEA_ERROR_MESSAGE_PARSING;
        }

        mMessage.type[mMessagePoistion++] = c;

        // The message is not good! Clear all...
        if (mMessagePoistion >= WNMEA_MESSAGE_TYPE_LENGTH)
        {
            reset();
            return WNMEA_ERROR_WRONG_MESSAGE;
        }

        // Compute checksum
        mChecksum ^= c;

        // Return without errors...
        return WNMEA_ERROR_MESSAGE_PARSING;
        break;

    case WNMEA_PARSESTATE_DATA:

        // Check the stop char...
        if (c == WNMEA_CHAR_STOP)
        {
            // Jump to the next state...
            mMessagePoistion = 0;
            mState = WNMEA_PARSESTATE_CHECKSUM;

            // Return without errors...
            return WNMEA_ERROR_MESSAGE_PARSING;
        }
        else
        {
            // The message is not good! Clear all...
            if (mMessagePoistion >= WNMEA_MESSAGE_BODY_LENGTH)
            {
                reset();
                return WNMEA_ERROR_WRONG_MESSAGE;
            }

            mMessage.body[mMessagePoistion++] = c;
            // Compute checksum
            mChecksum ^= c;

            // Return without errors...
            return WNMEA_ERROR_MESSAGE_PARSING;
        }

        break;

    case WNMEA_PARSESTATE_CHECKSUM:
        mMessage.checksum[mMessagePoistion++] = c;
        if (mMessagePoistion == WNMEA_MESSAGE_CRC_LENGTH)
        {
            if ((uint8_t) strtol(mMessage.checksum, null, 16) == mChecksum)
            {
                // Nice! The message is valid!
                return WNMEA_ERROR_MESSAGE_READY;
            }
            reset();
            return WNMEA_ERROR_WRONG_MESSAGE;
        }
        break;

    default:
        reset();
        return WNMEA_ERROR_WRONG_MESSAGE;
        break;
    }

    return WNMEA_ERROR_SUCCESS;
}


static WNMEA_Constellation_t getConstellation (void)
{
    if (strncmp(mMessage.type,WNMEA_CONSTELLATION_STRING_GPS,2) != 0)
    {
        return WNMEA_CONSTELLATION_GPS;
    }
    else if (strncmp(mMessage.type,WNMEA_CONSTELLATION_STRING_GLONASS,2) != 0)
    {
        return WNMEA_CONSTELLATION_GLONASS;
    }
    else if (strncmp(mMessage.type,WNMEA_CONSTELLATION_STRING_GALILEO,2) != 0)
    {
        return WNMEA_CONSTELLATION_GALILEO;
    }
    else if (strncmp(mMessage.type,WNMEA_CONSTELLATION_STRING_BEIDOU,2) != 0)
    {
        return WNMEA_CONSTELLATION_BEIDOU;
    }
    else if (strncmp(mMessage.type,WNMEA_CONSTELLATION_STRING_NAVIC,2) != 0)
    {
        return WNMEA_CONSTELLATION_NAVIC;
    }
    else if (strncmp(mMessage.type,WNMEA_CONSTELLATION_STRING_MULTIPLE,2) != 0)
    {
        return WNMEA_CONSTELLATION_MULTIPLE;
    }

    return WNMEA_CONSTELLATION_UNKNOW;
}

static WNMEA_MessageType_t getType (void)
{
    if (strncmp(&mMessage.type[2],WNMEA_MESSAGE_TYPE_STRING_RMC,3) != 0)
    {
        return WNMEA_MESSAGETYPE_RMC;
    }
    else if (strncmp(&mMessage.type[2],WNMEA_MESSAGE_TYPE_STRING_GGA,3) != 0)
    {
        return WNMEA_MESSAGETYPE_GGA;
    }
    else if (strncmp(&mMessage.type[2],WNMEA_MESSAGE_TYPE_STRING_GLL,3) != 0)
    {
        return WNMEA_MESSAGETYPE_GLL;
    }
    else if (strncmp(&mMessage.type[2],WNMEA_MESSAGE_TYPE_STRING_GSV,3) != 0)
    {
        return WNMEA_MESSAGETYPE_GSV;
    }
    else if (strncmp(&mMessage.type[2],WNMEA_MESSAGE_TYPE_STRING_GSA,3) != 0)
    {
        return WNMEA_MESSAGETYPE_GSA;
    }
    else if (strncmp(&mMessage.type[2],WNMEA_MESSAGE_TYPE_STRING_ZDA,3) != 0)
    {
        return WNMEA_MESSAGETYPE_ZDA;
    }

    return WNMEA_MESSAGETYPE_UNKNOW;
}

static WNMEA_Error_t parseRMC (void)
{

}

static WNMEA_Error_t parse (void)
{
    // Reset parsed message variable
    memset(&mMessageParsed,0,sizeof(WNMEA_MessageParsed_t));

    // Check the message types
    // It is a PMTK packet, useful for configuration...
    if (strncmp(WNMEA_MESSAGE_TYPE_STRING_PMTK,mMessage.type,strlen(WNMEA_MESSAGE_TYPE_STRING_PMTK) == 0))
    {
        // TODO:  manage this message!
    }
    else
    {
        // Check whether the constellation type is valid!
        if ((strncmp(mMessage.type,WNMEA_CONSTELLATION_STRING_GPS,2) != 0)     &&
            (strncmp(mMessage.type,WNMEA_CONSTELLATION_STRING_GLONASS,2) != 0) &&
            (strncmp(mMessage.type,WNMEA_CONSTELLATION_STRING_GALILEO,2) != 0) &&
            (strncmp(mMessage.type,WNMEA_CONSTELLATION_STRING_BEIDOU,2) != 0)  &&
            (strncmp(mMessage.type,WNMEA_CONSTELLATION_STRING_NAVIC,2) != 0)   &&
            (strncmp(mMessage.type,WNMEA_CONSTELLATION_STRING_MULTIPLE,2) != 0))
        {
            // The constellation is not valid
            return WNMEA_ERROR_WRONG_MESSAGE;
        }

        // Save the constellation
        mMessageParsed.constellation = getConstellation();

        // Check the message type, and parse message.
        mMessageParsed.type = getType();

        switch(mMessageParsed.type)
        {
        case WNMEA_MESSAGETYPE_RMC:
            parseRMC();
            break;
        case WNMEA_MESSAGETYPE_GGA:
            break;
        case WNMEA_MESSAGETYPE_GLL:
            break;
        case WNMEA_MESSAGETYPE_GSV:
            break;
        case WNMEA_MESSAGETYPE_GSA:
            break;
        case WNMEA_MESSAGETYPE_ZDA:
            break;
        default:
            break;
        }

    }
    return WNMEA_ERROR_SUCCESS;
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
            parse();
        }
    }
}
