/*
 * Simple time measuring and stamping routines
 */

#include <stdio.h>
#include <unistd.h>
#include <time.h>


void print_Time_Resolution()
{
    struct timespec res;
    clock_getres(CLOCK_MONOTONIC, &res);
    printf("CLOCK_MONOTONIC resolution = %ld sec, %ld nanosec\n", res.tv_sec,
	   res.tv_nsec);
}

double get_Time()
{
    static struct timespec tstart;
    static bool init = false;
   
    if ( !init ) {
        init = true;
        clock_gettime(CLOCK_MONOTONIC,&tstart);
        return 0.0;
    }

    struct timespec tcur;
    clock_gettime(CLOCK_MONOTONIC, &tcur);
    
    struct timespec tdiff;
    if ( (tcur.tv_nsec - tstart.tv_nsec) < 0 ) {
	tdiff.tv_sec = tcur.tv_sec-tstart.tv_sec-1;
	tdiff.tv_nsec = 1000000000+tcur.tv_nsec-tstart.tv_nsec;
    } else {
	tdiff.tv_sec = tcur.tv_sec-tstart.tv_sec;
	tdiff.tv_nsec = tcur.tv_nsec-tstart.tv_nsec;
    }
    
    double elapsed = (double)tdiff.tv_sec + 1.0e-9*(double)tdiff.tv_nsec;
    
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
