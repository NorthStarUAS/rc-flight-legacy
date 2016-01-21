/**
 * \file: gpsd.hxx
 *
 * Read gps data from the gpsd driver (provides data via a socket
 * connection on port 2947)
 *
 * Copyright Curt Olson curtolson@flightgear.org
 *
 */

#ifndef _AURA_GPS_GPSD_HXX
#define _AURA_GPS_GPSD_HXX


#include "python/pyprops.hxx"

#include "include/globaldefs.h"


void gpsd_init( pyPropertyNode *base, pyPropertyNode *config );
bool gpsd_get_gps();
void gpsd_close();


#endif // _AURA_GPS_GPSD_HXX
