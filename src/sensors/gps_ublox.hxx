/**
 * \file: gps_ublox.hxx
 *
 * u-blox 5 protocol driver
 *
 * Copyright Curt Olson curtolson@flightgear.org
 *
 */

#ifndef _AURA_GPS_UBLOX_HXX
#define _AURA_GPS_UBLOX_HXX


#include "python/pyprops.hxx"
#include "include/globaldefs.h"


void gps_ublox_init( pyPropertyNode *base, pyPropertyNode *config );
bool gps_ublox_update();
void gps_ublox_close();


#endif // _AURA_GPS_UBLOX_HXX
