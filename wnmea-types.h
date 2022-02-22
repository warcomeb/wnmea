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
 * \file  /wnmea-types.h
 * \brief
 */

#ifndef __WARCOMEB_WNMEA_TYPES_H
#define __WARCOMEB_WNMEA_TYPES_H


#define WARCOMEB_WNMEA_LIBRARY_VERSION_MAJOR     (0x1ul)
#define WARCOMEB_WNMEA_LIBRARY_VERSION_MINOR     (0x0ul)
#define WARCOMEB_WNMEA_LIBRARY_VERSION_BUG       (0x0ul)
#define WARCOMEB_WNMEA_LIBRARY_TIME              0

#ifndef __NO_PROFILES
#include "board.h"
#include "firmware.h"
#endif

/*!
 * \defgroup WNMEA_Types WCDLI Types
 * \ingroup  WNMEA
 * \{
 */

#define WNMEA_PROJECT_NAME                       "WNMEA"

static const Utility_Version_t WNMEA_FIRMWARE_VERSION =
{
    .f.major    = WARCOMEB_WNMEA_LIBRARY_VERSION_MAJOR,
    .f.minor    = WARCOMEB_WNMEA_LIBRARY_VERSION_MINOR,
    .f.subminor = WARCOMEB_WNMEA_LIBRARY_VERSION_BUG,
    .f.time     = WARCOMEB_WNMEA_LIBRARY_TIME,
};

#if !defined(WNMEA_MESSAGE_TYPE_LENGTH)
#define WNMEA_MESSAGE_TYPE_LENGTH                10
#endif

#if !defined(WNMEA_MESSAGE_BODY_LENGTH)
#define WNMEA_MESSAGE_BODY_LENGTH                100
#endif

#if !defined(WNMEA_MESSAGE_CRC_LENGTH)
#define WNMEA_MESSAGE_CRC_LENGTH                 2
#endif

/*!
 *
 */
typedef float WNMEA_Coordinate_t;

/*!
 * List of all possible errors.
 */
typedef enum _WNMEA_Errors_t
{
    WNMEA_ERROR_SUCCESS            = 0x0000,
    WNMEA_ERROR_WRONG_MESSAGE      = 0x0001,
    WNMEA_ERROR_MESSAGE_PARSING    = 0x0002,

    WNMEA_ERROR_MESSAGE_READY      = 0xFFFF,

} WNMEA_Error_t;

#if !defined (WNMEA_BUFFER_DIMENSION)
#define WNMEA_BUFFER_DIMENSION                   0x01FFu
#endif

/*!
 *
 */
typedef enum _WNMEA_Constellation_t
{
    WNMEA_CONSTELLATION_GPS,
    WNMEA_CONSTELLATION_GLONASS,
    WNMEA_CONSTELLATION_GALILEO,
    WNMEA_CONSTELLATION_BEIDOU,
    WNMEA_CONSTELLATION_NAVIC,
    WNMEA_CONSTELLATION_MULTIPLE,

    WNMEA_CONSTELLATION_UNKNOW,

} WNMEA_Constellation_t;

/*!
 *
 */
typedef enum _WNMEA_MessageType_t
{
    WNMEA_MESSAGETYPE_RMC,
    WNMEA_MESSAGETYPE_GGA,
    WNMEA_MESSAGETYPE_GLL,
    WNMEA_MESSAGETYPE_GSV,
    WNMEA_MESSAGETYPE_GSA,
    WNMEA_MESSAGETYPE_ZDA,

    WNMEA_MESSAGETYPE_UNKNOW,

} WNMEA_MessageType_t;

/*!
 *
 */
typedef enum _WNMEA_CardinalSide_t
{
    WNMEA_CARDINALSIDE_NORTH,
    WNMEA_CARDINALSIDE_SOUTH,
    WNMEA_CARDINALSIDE_EAST,
    WNMEA_CARDINALSIDE_WEST,

    WNMEA_CARDINALSIDE_UNKNOW,
} WNMEA_CardinalSide_t;

typedef enum _WNMEA_PositionType_t
{
    WNMEA_POSITIONTYPE_VALID,
    WNMEA_POSITIONTYPE_INVALID,
} WNMEA_PositionType_t;

typedef enum _WNMEA_FixQuality_t
{
    WNMEA_FIXQUALITY_INVALID          = 0,
    WNMEA_FIXQUALITY_FIX              = 1,
    WNMEA_FIXQUALITY_DIFFERENTIAL_FIX = 2,
} WNMEA_FixQuality_t;

/*!
 *
 */
typedef struct _WNMEA_Message_t
{
    char type[WNMEA_MESSAGE_TYPE_LENGTH];
    char body[WNMEA_MESSAGE_BODY_LENGTH];
    char checksum[WNMEA_MESSAGE_CRC_LENGTH];

} WNMEA_Message_t;

/*!
 * RMC Sentences - Recommended minimum specific GPS/Transit data
 */
typedef struct _WNMEA_MessageRMC_t
{
    WNMEA_PositionType_t  status;          ///< Data status (V=navigation receiver warning)
    Time_TimeType         time;            ///< UTC of position fix
    Time_DateType         date;
    WNMEA_Coordinate_t    longitude;       ///< Longitude of fix
    WNMEA_CardinalSide_t  longitudeSide;
    WNMEA_Coordinate_t    latitude;        ///< Latitude of fix
    WNMEA_CardinalSide_t  latitudeSide;
    float                 speed;           ///< Speed over ground in knots
} WNMEA_MessageRMC_t;

/*!
 * GGA Sentences - Global Positioning System Fix Data
 */
typedef struct _WNMEA_MessageGGA_t
{
    Time_TimeType         time;          ///< UTC of Position
    WNMEA_Coordinate_t    latitude;      ///< Latitude of fix
    WNMEA_CardinalSide_t  latitudeSide;
    WNMEA_Coordinate_t    longitude;     ///< Longitude of fix
    WNMEA_CardinalSide_t  longitudeSide;
    WNMEA_FixQuality_t    quality;       ///< GPS quality indicator
    uint8_t               satellites;    ///< Number of satellites in use (not in view!)
} WNMEA_MessageGGA_t;

/*!
 * ZDA Sentences - Date & Time
 */
typedef struct _WNMEA_MessageZDA_t
{
    Time_TimeType time;       ///< UTC of position fix
    Time_DateType date;       ///< UT date of position fix
    int8_t        hourDiff;   ///< Local zone description, 00 to +/- 13 hours
    int8_t        minuteDiff; ///< Local zone minutes description (same sign as hours)
} WNMEA_MessageZDA_t;

/*!
 *
 */
typedef struct _WNMEA_MessageParsed_t
{
    WNMEA_Constellation_t constellation;
    WNMEA_MessageType_t   type;

    union
    {
        WNMEA_MessageRMC_t rmc;
        WNMEA_MessageGGA_t gga;
        WNMEA_MessageZDA_t zda;
    } message;

} WNMEA_MessageParsed_t, *WNMEA_MessageParsedHandle_t;

/*!
 *
 */
typedef void (*WNMEA_pFunctionCallback) (WNMEA_MessageParsed_t msg, WNMEA_MessageType_t type);

/*!
 *
 */
typedef struct _WNMEA_MessageCallback_t
{
    WNMEA_pFunctionCallback rmc;
    WNMEA_pFunctionCallback gga;
} WNMEA_MessageCallback_t;

/*!
 * \}
 */

#endif // __WARCOMEB_WCDLI_TYPES_H
