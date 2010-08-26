#include <stdio.h>

#include "comms/logging.h"

#include "timing.h"

#include "myprof.h"

myprofile::myprofile() {
    init_time = 0.0;
    count = 0;
    sum_time = 0.0;
}

myprofile::~myprofile() {
}

void myprofile::set_name( const string _name ) {
    name = _name;
}

void myprofile::start() {
    if ( init_time <= 0.0001 ) {
	init_time = get_Time();
    }
    start_time = get_Time();
    count++;
}

void myprofile::stop() {
    double stop_time = get_Time();
    last_interval = stop_time - start_time;
    sum_time += last_interval;
    
    // log situations where a module took longer that 0.10 sec to execute
    if ( event_log_on && (last_interval > 0.10) ) {
	char msg[256];
	snprintf(msg, 256, "t1 = %.3f t2 = %.3f int = %.3f",
		 start_time, stop_time, last_interval);
	event_log( name.c_str(), msg );
    }
}

void myprofile::stats() {
    double total_time = get_Time() - init_time;
    double avg_hz = 0.0;
    if ( total_time > 1.0 ) {
	avg_hz = (double)count / total_time;
    }
    printf( "%s: avg = %.4f count = %d total = %.4f (last = %.4f) avg hz=%.3f\n",
	    name.c_str(), sum_time / (double)count,
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
myprofile datalog_prof;
myprofile main_prof;
