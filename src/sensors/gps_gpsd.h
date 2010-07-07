/**
 * \file: gpsd.h
 *
 * Read gps data from the gpsd driver (provides data via a socket
 * connection on port 2947)
 *
 * Copyright Curt Olson curtolson@gmail.com
 *
 * $Id: gpsd.h,v 1.2 2009/05/01 02:04:17 curt Exp $
 */

#ifndef _UGEAR_GPS_GPSD_H
#define _UGEAR_GPS_GPSD_H


#include "globaldefs.h"

void gpsd_init( string rootname, SGPropertyNode *config );
bool gpsd_get_gps();
void gpsd_close();


#endif // _UGEAR_GPS_GPSD_H
