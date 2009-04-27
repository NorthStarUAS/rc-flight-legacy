/**
 * \file: gps_mgr.h
 *
 * Front end management interface for reading GPS data.
 *
 * Copyright (C) 2009 - Curtis L. Olson curtolson@gmail.com
 *
 * $Id: gps_mgr.h,v 1.3 2009/04/27 01:29:09 curt Exp $
 */


#ifndef _UGEAR_GPS_MGR_H
#define _UGEAR_GPS_MGR_H


// extern struct gps gpspacket;

struct gps {
   double time;
   double lat,lon,alt;          /* gps position                */
   double ve,vn,vd;             /* gps velocity                */
   double date;                 /* unix seconds from gps       */
   uint64_t status;		/* data status flag            */
};

enum gps_source_t {
    gpsNone,
    gpsGPSD,
    gpsMNAV,
    gpsUGFile
};

void GPS_init();
bool GPS_update();
void GPS_close();

// return gps data age in seconds
double GPS_age();


#endif // _UGEAR_GPS_MGR_H
