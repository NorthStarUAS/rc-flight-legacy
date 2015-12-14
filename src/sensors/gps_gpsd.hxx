/**
 * \file: gpsd.hxx
 *
 * Read gps data from the gpsd driver (provides data via a socket
 * connection on port 2947)
 *
 * Copyright Curt Olson curtolson@gmail.com
 *
 */

#ifndef _AURA_GPS_GPSD_HXX
#define _AURA_GPS_GPSD_HXX


#include "include/globaldefs.h"


void gpsd_init( string rootname, SGPropertyNode *config );
bool gpsd_get_gps();
void gpsd_close();


#endif // _AURA_GPS_GPSD_HXX
