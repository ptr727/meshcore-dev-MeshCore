#pragma once

#include <Arduino.h>

/**
 * GNSS control pins and their polarity, defaulted once.
 *
 * A variant may name these directly (GPS_EN), or through the PIN_GPS_* form its variant.h uses,
 * or not at all. These fallbacks resolve all three to one set of names, so that a build which
 * wires no enable or reset pin still compiles and still has something truthful to report: -1,
 * meaning there is no pin, rather than a macro that does not exist.
 *
 * Kept separate from the driver that consumes them so that code which only needs to *report* the
 * wiring does not have to pull in a GNSS parser to find out what it is.
 */

#ifndef GPS_EN
    #ifdef PIN_GPS_EN
        #define GPS_EN PIN_GPS_EN
    #else
        #define GPS_EN (-1)
    #endif
#endif

#ifndef GPS_EN_ACTIVE
    #ifdef PIN_GPS_EN_ACTIVE
        #define GPS_EN_ACTIVE PIN_GPS_EN_ACTIVE
    #else
        #define GPS_EN_ACTIVE HIGH
    #endif
#endif

#ifndef GPS_RESET
    #ifdef PIN_GPS_RESET
        #define GPS_RESET PIN_GPS_RESET
    #else
        #define GPS_RESET (-1)
    #endif
#endif

#ifndef GPS_RESET_ACTIVE
    #ifdef PIN_GPS_RESET_ACTIVE
        #define GPS_RESET_ACTIVE PIN_GPS_RESET_ACTIVE
    #else
        #define GPS_RESET_ACTIVE LOW
    #endif
#endif
