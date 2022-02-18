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

#include "stdlib.h"
#include "string.h"

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

/*!
 * This function parse the coordinate string and convert the value in degrees.
 *
 * \param[in] message: The string to converte
 * \param[out] result: The converted value
 * \return
 */
static WNMEA_Error_t parseCoordinate (const char* message, WNMEA_Coordinate_t* result)
{
    char* cursor;
    float degrees = 0.0f, minutes = 0.0f;

    *result = 0;

    if ((message == null) || (*message == '\0'))
    {
        return WNMEA_ERROR_WRONG_MESSAGE;
    }

    // Check decimal point
    cursor = strchr(message,'.');
    // In case the decimal point is not present, return with error.
    if (cursor == null)
    {
        return WNMEA_ERROR_WRONG_MESSAGE;
    }

    // Move cursor 2 position before the decimal point to detect the minutes value
    cursor -= 2;
    minutes = atof(cursor);

    // Close degree string with \0 at cursor position
    cursor = '\0';

    // Compute degrees value
    degrees = (float)atoi(message);

    // Compute result
    *result = (WNMEA_Coordinate_t)(degrees + (minutes / 60.0f));

    return WNMEA_ERROR_SUCCESS;
}

/*!
 *
 */
static WNMEA_Error_t parseCardinal (char c, WNMEA_CardinalSide_t* result)
{
    switch (c)
    {
    case 'N':
        *result = WNMEA_CARDINALSIDE_NORTH;
        break;
    case 'S':
        *result = WNMEA_CARDINALSIDE_SOUTH;
        break;
    case 'E':
        *result = WNMEA_CARDINALSIDE_EAST;
        break;
    case 'W':
        *result = WNMEA_CARDINALSIDE_WEST;
        break;
    default:
        *result = WNMEA_CARDINALSIDE_UNKNOW;
        return WNMEA_ERROR_WRONG_MESSAGE;
    }
    return WNMEA_ERROR_SUCCESS;
}

/*!
 * This function parse the NMEA string RMC.
 * The parsed value will be saved into internal variable.
 * The format of RMC is the following:
 * \code{.unparsed}
 * $GPRMC,hhmmss.ss,A,llll.ll,a,yyyyy.yy,a,x.x,x.x,ddmmyy,x.x,a*hh
 * 1    = UTC of position fix
 * 2    = Data status (V=navigation receiver warning)
 * 3    = Latitude of fix
 * 4    = N or S
 * 5    = Longitude of fix
 * 6    = E or W
 * 7    = Speed over ground in knots
 * 8    = Track made good in degrees True
 * 9    = UT date
 * 10   = Magnetic variation degrees (Easterly var. subtracts from true course)
 * 11   = E or W
 * 12   = Checksum
 * \endcode
 * \see http://aprs.gids.nl/nmea/
 *
 * \return The function return WNMEA_ERROR_SUCCESS in case of success, WNMEA_ERROR_WRONG_MESSAGE
 *         otherwise.
 */
static WNMEA_Error_t parseRMC (void)
{
    char* cursor = 0;

    char tmp[WNMEA_MESSAGE_BODY_LENGTH] = {0};
    strcpy(tmp,mMessage.body);

    cursor = strtok(tmp,",");
    // 1. UTC of position fix

    cursor = strtok(null,",");
    // 2. Data status (V=navigation receiver warning)
    if (strlen(cursor) != 1)
    {
        return WNMEA_ERROR_WRONG_MESSAGE;
    }
    // Check the value
    if (cursor[0] == 'V')
    {
        mMessageParsed.message.rmc.status = WNMEA_POSITIONTYPE_INVALID;
    }
    else
    {
        mMessageParsed.message.rmc.status = WNMEA_POSITIONTYPE_VALID;
    }

    cursor = strtok(null,",");
    // 3. Latitude of fix
    if (parseCoordinate(cursor,&mMessageParsed.message.rmc.latitude) != WNMEA_ERROR_SUCCESS)
    {
        return WNMEA_ERROR_WRONG_MESSAGE;
    }

    cursor = strtok(null,",");
    // 4. N or S
    if ((strlen(cursor) != 1) || (parseCardinal(cursor[0],&mMessageParsed.message.rmc.latitudeSide) != WNMEA_ERROR_SUCCESS))
    {
        return WNMEA_ERROR_WRONG_MESSAGE;
    }

    cursor = strtok(null,",");
    // 5. Longitude of fix
    if (parseCoordinate(cursor,&mMessageParsed.message.rmc.longitude) != WNMEA_ERROR_SUCCESS)
    {
        return WNMEA_ERROR_WRONG_MESSAGE;
    }

    cursor = strtok(null,",");
    // 6. E or W
    if ((strlen(cursor) != 1) || (parseCardinal(cursor[0],&mMessageParsed.message.rmc.longitudeSide) != WNMEA_ERROR_SUCCESS))
    {
        return WNMEA_ERROR_WRONG_MESSAGE;
    }

    cursor = strtok(null,",");
    // 7. Speed over ground in knots
    mMessageParsed.message.rmc.speed = atof(cursor);

    cursor = strtok(null,",");
    // 8. Track made good in degrees True
    // FIXME: not implemented!

    cursor = strtok(null,",");
    // 9. UT date

    cursor = strtok(null,",");
    // 10. Magnetic variation degrees (Easterly var. subtracts from true course)
    // FIXME: not implemented!

    cursor = strtok(null,",");
    // 11. E or W
    // FIXME: not implemented!

    return WNMEA_ERROR_SUCCESS;
}

/*!
 * This function parse the NMEA string GGA.
 * The parsed value will be saved into internal variable.
 * The format of GGA is the following:
 * \code{.unparsed}
 * $GPGGA,hhmmss.ss,llll.ll,a,yyyyy.yy,a,x,xx,x.x,x.x,M,x.x,M,x.x,xxxx*hh
 * 1    = UTC of Position
 * 2    = Latitude
 * 3    = N or S
 * 4    = Longitude
 * 5    = E or W
 * 6    = GPS quality indicator (0=invalid; 1=GPS fix; 2=Diff. GPS fix)
 * 7    = Number of satellites in use [not those in view]
 * 8    = Horizontal dilution of position
 * 9    = Antenna altitude above/below mean sea level (geoid)
 * 10   = Meters  (Antenna height unit)
 * 11   = Geoidal separation (Diff. between WGS-84 earth ellipsoid and
 *        mean sea level.  -=geoid is below WGS-84 ellipsoid)
 * 12   = Meters  (Units of geoidal separation)
 * 13   = Age in seconds since last update from diff. reference station
 * 14   = Diff. reference station ID#
 * 15   = Checksum
 * \endcode
 * \see http://aprs.gids.nl/nmea/
 *
 * \return The function return WNMEA_ERROR_SUCCESS in case of success, WNMEA_ERROR_WRONG_MESSAGE
 *         otherwise.
 */
static WNMEA_Error_t parseGGA (void)
{
    char* cursor = 0;

    char tmp[WNMEA_MESSAGE_BODY_LENGTH] = {0};
    strcpy(tmp,mMessage.body);

    cursor = strtok(tmp,",");
    // 1. UTC of position fix

    cursor = strtok(null,",");
    // 2. Latitude of fix
    if (parseCoordinate(cursor,&mMessageParsed.message.gga.latitude) != WNMEA_ERROR_SUCCESS)
    {
        return WNMEA_ERROR_WRONG_MESSAGE;
    }

    cursor = strtok(null,",");
    // 3. N or S
    if ((strlen(cursor) != 1) || (parseCardinal(cursor[0],&mMessageParsed.message.gga.latitudeSide) != WNMEA_ERROR_SUCCESS))
    {
        return WNMEA_ERROR_WRONG_MESSAGE;
    }

    cursor = strtok(null,",");
    // 4. Longitude of fix
    if (parseCoordinate(cursor,&mMessageParsed.message.gga.longitude) != WNMEA_ERROR_SUCCESS)
    {
        return WNMEA_ERROR_WRONG_MESSAGE;
    }

    cursor = strtok(null,",");
    // 5. E or W
    if ((strlen(cursor) != 1) || (parseCardinal(cursor[0],&mMessageParsed.message.gga.longitudeSide) != WNMEA_ERROR_SUCCESS))
    {
        return WNMEA_ERROR_WRONG_MESSAGE;
    }

    cursor = strtok(null,",");
    // 6. GPS quality indicator (0=invalid; 1=GPS fix; 2=Diff. GPS fix)
    if (strlen(cursor) != 1)
    {
        return WNMEA_ERROR_WRONG_MESSAGE;
    }
    // TODO


    cursor = strtok(null,",");
    // 7. Number of satellites in use [not those in view]
    mMessageParsed.message.gga.satellites = atoi(cursor);

    cursor = strtok(null,",");
    // 8. Horizontal dilution of position
    // FIXME: not implemented!

    cursor = strtok(null,",");
    // 9. Antenna altitude above/below mean sea level (geoid)
    // FIXME: not implemented!

    cursor = strtok(null,",");
    // 10. Meters  (Antenna height unit)
    // FIXME: not implemented!

    cursor = strtok(null,",");
    // 11. Geoidal separation
    // FIXME: not implemented!

    cursor = strtok(null,",");
    // 12. Meters  (Units of geoidal separation)
    // FIXME: not implemented!

    cursor = strtok(null,",");
    // 13. Age in seconds since last update from diff. reference station
    // FIXME: not implemented!

    cursor = strtok(null,",");
    // 14. Diff. reference station ID#
    // FIXME: not implemented!

    return WNMEA_ERROR_SUCCESS;
}

static WNMEA_Error_t parse (void)
{
    WNMEA_Error_t ret = WNMEA_ERROR_SUCCESS;

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
            ret = parseRMC();
            break;
        case WNMEA_MESSAGETYPE_GGA:
            ret = parseGGA();
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

        if (ret != WNMEA_ERROR_SUCCESS)
        {
            return ret;
        }
        // TODO

    }
    return ret;
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
