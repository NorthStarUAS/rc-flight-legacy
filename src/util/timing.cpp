/*
 * Simple time measuring and stamping routines
 */

#include <stdio.h>
#include <unistd.h>
#include <time.h>


double get_Time()
{
    struct timespec tcur;
    static struct timespec tstart;
    double elapsed;
    static bool init = false;
   
    if ( !init ) {
        init = true;
        clock_gettime(CLOCK_MONOTONIC,&tstart);
        return 0.0;
    } 
    clock_gettime(CLOCK_MONOTONIC, &tcur);
    elapsed = (tcur.tv_sec-tstart.tv_sec)
	+ 1.0e-9*(double)(tcur.tv_nsec - tstart.tv_nsec);

    return elapsed;
}

double get_RealTime()
{
    struct timespec t;
    double tnow;
   
    clock_gettime(CLOCK_REALTIME, &t);
    tnow = t.tv_sec + 1.0e-9*(double)(t.tv_nsec);

    return tnow;
}
