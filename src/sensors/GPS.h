/**
 * \file: GPS.h
 *
 * Front end management interface for reading GPS data.
 *
 * Copyright (C) 2009 - Curtis L. Olson curtolson@gmail.com
 *
 * $Id: GPS.h,v 1.3 2009/04/17 18:10:03 curt Exp $
 */


#ifndef _UGEAR_GPS_H
#define _UGEAR_GPS_H


extern struct gps gpspacket;

enum gps_source_t {
    gpsNone,
    gpsGPSD,
    gpsMNAV,
    gpsUGFile
};

void GPS_init();
bool GPS_update();
void GPS_close();


#endif // _UGEAR_GPS_H
