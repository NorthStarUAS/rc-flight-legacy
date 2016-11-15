/**
 * \file: gps_ublox.hxx
 *
 * u-blox 7 protocol driver
 *
 * Copyright Curt Olson curtolson@flightgear.org
 *
 */

#ifndef _AURA_GPS_UBLOX8_HXX
#define _AURA_GPS_UBLOX8_HXX


#include "python/pyprops.hxx"

#include "include/globaldefs.h"


void gps_ublox8_init( string output_path, pyPropertyNode *config );
bool gps_ublox8_update();
void gps_ublox8_close();


#endif // _AURA_GPS_UBLOX8_HXX
