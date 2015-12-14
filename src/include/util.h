#ifndef _AURA_UTIL_H
#define _AURA_UTIL_H


#include "include/globaldefs.h"


// simple wrap around for -180 and + 180 
inline double wraparound( double dta )
{
    if ( dta > SGD_PI ) dta -= SGD_2PI;
    if ( dta < -SGD_PI ) dta += SGD_2PI;
   
    return dta;
}	


#endif // _AURA_UTIL_H
