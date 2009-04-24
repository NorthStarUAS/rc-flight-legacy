//
// FILE: navigation.h
// DESCRIPTION: compute the position estimate
//

#ifndef _UGEAR_NAVIGATION_H
#define _UGEAR_NAVIGATION_H


#include "util/matrix.h"


// global variables
extern struct nav navpacket;
extern short gps_init_count;
extern MATRIX nxs;


// global functions
void mnav_nav_init();
void mnav_nav_update();
void mnav_nav_close();


#endif // _UGEAR_NAVIGATION_H
