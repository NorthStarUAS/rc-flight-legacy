/**
 * \file: gps_ublox5.hxx
 *
 * u-blox 5 protocol driver
 *
 * Copyright Curt Olson curtolson@gmail.com
 *
 */

#ifndef _UGEAR_GPS_UBLOX5_HXX
#define _UGEAR_GPS_UBLOX5_HXX


#include "include/globaldefs.h"

void gps_ublox5_init( string rootname, SGPropertyNode *config );
bool gps_ublox5_update();
void gps_ublox5_close();


#endif // _UGEAR_GPS_UBLOX5_HXX
