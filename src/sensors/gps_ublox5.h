/**
 * \file: gps_ublox5.h
 *
 * u-blox 5 protocol driver
 *
 * Copyright Curt Olson curtolson@gmail.com
 *
 * $Id: gpsd.h,v 1.2 2009/05/01 02:04:17 curt Exp $
 */

#ifndef _UGEAR_GPS_UBLOX5_H
#define _UGEAR_GPS_UBLOX5_H


#include "globaldefs.h"

void gps_ublox5_init( string rootname, SGPropertyNode *config );
bool gps_ublox5_update();
void gps_ublox5_close();


#endif // _UGEAR_GPS_UBLOX5_H
