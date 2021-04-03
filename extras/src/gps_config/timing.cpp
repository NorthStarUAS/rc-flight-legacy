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
    static double tstart;
    static bool init = false;
   
    struct timespec ts_cur;
    clock_gettime(CLOCK_MONOTONIC, &ts_cur);
    double tcur = (double)ts_cur.tv_sec + 1.0e-9*(double)ts_cur.tv_nsec;

    if ( !init ) {
        init = true;
        tstart = tcur;
    }
    
    return tcur - tstart;
}

double get_RealTime()
{
    struct timespec t;
    double tnow;
   
    clock_gettime(CLOCK_REALTIME, &t);
    tnow = t.tv_sec + 1.0e-9*(double)(t.tv_nsec);

    return tnow;
}
