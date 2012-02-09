/**
 * \file: gps_ublox.hxx
 *
 * u-blox 5 protocol driver
 *
 * Copyright Curt Olson curtolson@gmail.com
 *
 */

#ifndef _UGEAR_GPS_UBLOX_HXX
#define _UGEAR_GPS_UBLOX_HXX


#include "include/globaldefs.h"

void gps_ublox_init( string rootname, SGPropertyNode *config );
bool gps_ublox_update();
void gps_ublox_close();


#endif // _UGEAR_GPS_UBLOX_HXX
