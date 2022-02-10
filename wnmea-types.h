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

typedef struct _WNMEA_Message_t
{
    char type[WNMEA_MESSAGE_TYPE_LENGTH];
    char body[WNMEA_MESSAGE_BODY_LENGTH];
    char checksum[WNMEA_MESSAGE_CRC_LENGTH];

} WNMEA_Message_t;

typedef struct _WNMEA_MessageParsed_t
{
    WNMEA_Constellation_t constellation;

} WNMEA_MessageParsed_t, *WNMEA_MessageParsedHandle_t;

/*!
 * \}
 */

#endif // __WARCOMEB_WCDLI_TYPES_H
