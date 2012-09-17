/******************************************************************************
 * FILE: misc.c
 * DESCRIPTION:
 *   
 *   
 *
 * SOURCE: 
 * LAST REVISED: 5/11/05 Jung Soon Jang
 ******************************************************************************/
#include <stdio.h>
#include <unistd.h>
#include <time.h>

#include "include/globaldefs.h"


double get_Time( bool reset )
{
    struct timespec t;
    static struct timespec tset;
    double tnow;
    static bool init = false;
   
    if ( !init || reset ) {
        init = true;
        clock_gettime(CLOCK_REALTIME,&tset);
        return 0.0;
    } 
    clock_gettime(CLOCK_REALTIME, &t);
    tnow = (t.tv_sec-tset.tv_sec) + 1.0e-9*(double)(t.tv_nsec - tset.tv_nsec);

    return tnow;
}

double get_RealTime()
{
    struct timespec t;
    double tnow;
   
    clock_gettime(CLOCK_REALTIME, &t);
    tnow = t.tv_sec + 1.0e-9*(double)(t.tv_nsec);

    return tnow;
}
