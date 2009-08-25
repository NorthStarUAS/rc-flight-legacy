//
// FILE: navigation.h
// DESCRIPTION: compute the position estimate
//

#ifndef _UGEAR_NAVIGATION_H
#define _UGEAR_NAVIGATION_H


#include <stdint.h>

#include "util/matrix.h"

struct nav {
   double time;
   double lat,lon,alt;
   double ve,vn,vd;
   // float t;
   uint64_t status;		/* data status flag */
};

// global variables
//extern struct nav navpacket;
extern MATRIX nxs;


// global functions
void mnav_nav_init( string rootname );
void mnav_nav_update( struct imu *imupacket );
void mnav_nav_close();


#endif // _UGEAR_NAVIGATION_H
