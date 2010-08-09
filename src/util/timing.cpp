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

#include <comms/logging.h>

#include "globaldefs.h"

//global variables
static double exe_rate[5];
static int count[5] = {0,};
static struct timespec ts_p[5];
static double sum[5] = {0.,};

//
// snap time interval
//
void snap_time_interval( const char *threadname, int displaytime, short id )
{
    struct timespec	   ts;
    double 		   sec, nsec, dt, rate;
    
    clock_gettime(CLOCK_REALTIME, &ts);
    sec     = ts.tv_sec - ts_p[id].tv_sec;
    nsec    = ts.tv_nsec- ts_p[id].tv_nsec;
    dt = sec + nsec*1.0e-9;
    ts_p[id]= ts;
    sum[id] += dt;
	
    if (++count[id] == displaytime) {
        rate = sum[id] / count[id];
        exe_rate[id] = rate;

        if ( display_on ) {
            printf("[%s]: For %.1f sec: updates ran at %.2f(hz): %.2f(ms) step\n",
                   threadname, sum[id], 1/rate, rate*1000);
        }
        sum[id] = 0;
        count[id] = 0;
    }
}

double get_time_interval( short id )
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
