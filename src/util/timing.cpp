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
#include <pthread.h>
#include "globaldefs.h"
#include "matrix.h"

//global variables
double exe_rate[3];

//
// snap time interval
//
void snap_time_interval( char *threadname, int displaytime, short id )
{
    static int 		count[5]={0,};
    struct timespec		ts;
    static struct timespec  ts_p[5];
    double 			sec,nsec,elapsed;
    static double           sum[5]={0.,};
    
    clock_gettime(CLOCK_REALTIME, &ts);
    sec     = ts.tv_sec - ts_p[id].tv_sec;
    nsec    = ts.tv_nsec- ts_p[id].tv_nsec;
    elapsed = sec + nsec*1.0e-9;
    ts_p[id]= ts;
    sum[id]+= elapsed;
	
    if (++count[id] == displaytime) {
        elapsed = sum[id]/displaytime;
        sum[id] = 0;
        exe_rate[id]=elapsed;
#ifndef NCURSE_DISPLAY_OPTION
        printf("[%s]:The cycles in %5.2f (Hz):%5.2f (ms) \n",
               threadname, 1/elapsed, elapsed*1000);
#endif		
        count[id] = 0;
    }
}

double get_time_interval(short id)
{
    struct timespec		ts;
    static struct timespec  ts_p[5];
    double 			nsec,elapsed;

    clock_gettime(CLOCK_REALTIME, &ts);
    nsec    = ts.tv_nsec- ts_p[id].tv_nsec;
    elapsed = nsec*1.0e-9;
        
    if(elapsed <=0) elapsed =0.0;        
    ts_p[id]= ts;
       
    return elapsed;	
}	

double get_Time()
{
    struct timespec t;
    static struct timespec tset;
    double tnow;
    static int init = 0;
   
    if ( init == 0 ) {
        init = 1;
        clock_gettime(CLOCK_REALTIME,&tset);
        return 0.0;
    } 
    clock_gettime(CLOCK_REALTIME, &t);
    tnow = (t.tv_sec-tset.tv_sec) + 1.0e-9*(double)(t.tv_nsec - tset.tv_nsec);
    return tnow;
}

// The clock_nanosleep() function doesn't seem to be implemented or available
// in the gumstix development environment (gcc-4.x, linux-2.6.x) and is never
// used in this code anyplace ...

#if 0

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// get microsleep
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void microsleep(int interval)
{
    struct timespec naptime;
	
    naptime.tv_sec  = 0;
    naptime.tv_nsec = NSECS_PER_SEC/interval;   // 0.1 msec

    clock_nanosleep(CLOCK_REALTIME, TIMER_ABSTIME, &naptime, NULL);
}

#endif // 0
