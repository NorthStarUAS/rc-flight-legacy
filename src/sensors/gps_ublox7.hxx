/**
 * \file: gps_ublox.hxx
 *
 * u-blox 7 protocol driver
 *
 * Copyright Curt Olson curtolson@flightgear.org
 *
 */

#ifndef _AURA_GPS_UBLOX7_HXX
#define _AURA_GPS_UBLOX7_HXX


#include "python/pyprops.hxx"

#include "include/globaldefs.h"


void gps_ublox7_init( string output_path, pyPropertyNode *config );
bool gps_ublox7_update();
void gps_ublox7_close();


#endif // _AURA_GPS_UBLOX7_HXX
