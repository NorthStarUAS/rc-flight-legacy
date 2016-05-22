/**
 * \file: gps_ublox.hxx
 *
 * u-blox 6 protocol driver
 *
 * Copyright Curt Olson curtolson@flightgear.org
 *
 */

#ifndef _AURA_GPS_UBLOX6_HXX
#define _AURA_GPS_UBLOX6_HXX


#include "python/pyprops.hxx"

#include "include/globaldefs.h"


void gps_ublox6_init( string output_path, pyPropertyNode *config );
bool gps_ublox6_update();
void gps_ublox6_close();


#endif // _AURA_GPS_UBLOX6_HXX
