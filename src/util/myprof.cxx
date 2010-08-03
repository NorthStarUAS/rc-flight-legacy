#include <stdio.h>

#include "timing.h"

#include "myprof.h"

myprofile::myprofile() {
    init_time = 0.0;
    count = 0;
    sum_time = 0.0;
}

myprofile::~myprofile() {
}

void myprofile::start() {
    if ( init_time <= 0.0001 ) {
	init_time = get_Time();
    }
    start_time = get_Time();
    count++;
}

void myprofile::stop() {
  last_interval = get_Time() - start_time;
  sum_time += last_interval;
}

void myprofile::stats( const char *header ) {
    double total_time = get_Time() - init_time;
    double avg_hz = 0.0;
    if ( total_time > 1.0 ) {
	avg_hz = (double)count / total_time;
    }
    printf( "%s: avg = %.4f count = %d total = %.4f (last = %.4f) avg hz=%.3f\n",
	    header, sum_time / (double)count,
	    count, sum_time, last_interval, avg_hz );
}


// global profiling structures
myprofile imu_prof;
myprofile gps_prof;
myprofile air_prof;
myprofile pilot_prof;
myprofile filter_prof;
myprofile control_prof;
myprofile route_mgr_prof;
myprofile health_prof;
myprofile main_prof;
