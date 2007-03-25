//
// FILE: navigation.h
// DESCRIPTION: compute the position estimate
//

#ifndef _UGEAR_NAVIGATION_H
#define _UGEAR_NAVIGATION_H


#include "util/matrix.h"


// global variables
extern MATRIX nxs;
extern short gps_init_count;

// global functions
void nav_init();
void nav_update();
void nav_close();


#endif // _UGEAR_NAVIGATION_H
