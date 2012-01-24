/**
 * \file: gps_mgr.hxx
 *
 * Front end management interface for reading GPS data.
 *
 * Copyright (C) 2009 - Curtis L. Olson curtolson@gmail.com
 *
 */


#ifndef _UGEAR_GPS_MGR_HXX
#define _UGEAR_GPS_MGR_HXX


void GPS_init();
bool GPS_update();
void GPS_close();

// return gps data age in seconds
double GPS_age();


#endif // _UGEAR_GPS_MGR_HXX
