/**
 * \file: gps_mediatek.hxx
 *
 * MediaTek MT3329 MTK protocol driver
 *
 * Copyright (C) 2012 - Curtis L. Olson - curtolson@flightgear.org
 *
 */

#ifndef _UGEAR_GPS_MEDIATEK_HXX
#define _UGEAR_GPS_MEDIATEX_HXX


#include "include/globaldefs.h"

void gps_mediatek3329_init( string rootname, SGPropertyNode *config );
bool gps_mediatek3329_update();
void gps_mediatek3329_close();


#endif // _UGEAR_GPS_MEDIATEK_HXX
