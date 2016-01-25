/**
 * \file: gps_mediatek.hxx
 *
 * MediaTek MT3329 MTK protocol driver
 *
 * Copyright (C) 2012 - Curtis L. Olson - curtolson@flightgear.org
 *
 */

#ifndef _AURA_GPS_MEDIATEK_HXX
#define _AURA_GPS_MEDIATEX_HXX

#include "python/pyprops.hxx"

#include "include/globaldefs.h"


void gps_mediatek3329_init( string output_path, pyPropertyNode *config );
bool gps_mediatek3329_update();
void gps_mediatek3329_close();


#endif // _AURA_GPS_MEDIATEK_HXX
