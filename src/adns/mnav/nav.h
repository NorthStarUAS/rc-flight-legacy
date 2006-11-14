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
void *navigation(void *thread_id);


#endif // _UGEAR_NAVIGATION_H
