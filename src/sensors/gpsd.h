/**
 * \file: gpsd.h
 *
 * Read gps data from the gpsd driver (provides data via a socket
 * connection on port 2947)
 *
 * Copyright Curt Olson curtolson@gmail.com
 *
 * $Id: gpsd.h,v 1.1 2009/01/16 19:01:33 curt Exp $
 */

#ifndef _UGEAR_GPSD_H
#define _UGEAR_GPSD_H


#include "globaldefs.h"

void gpsd_init();
bool gpsd_get_gps( struct gps *data );
void gpsd_close();


#endif // _UGEAR_GPSD_H
